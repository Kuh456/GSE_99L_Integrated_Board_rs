use core::sync::atomic::Ordering;

use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_can::{Frame, Id};
use esp_hal::{
    Async,
    twai::{self, ErrorKind as TwaiErrorKind, EspTwaiError, EspTwaiFrame},
};

use crate::{
    CAN_BUS_OFF, CAN_COMM_ACTIVE, CAN_HEALTH, CAN_PEER_LOST, CAN_REC, CAN_RX_ERROR_COUNT,
    CAN_STATUS_TX_INTERVAL_MS, CAN_TEC, CAN_TX_ERROR_COUNT, CAN_TX_FRAME_CREATE_FAILED,
    CAN_TX_TIMEOUT, CAN_TX_TIMEOUT_MS, COMMUNICATION_TIMEOUT_MS, CURRENT_POSITION, FAULT_FLAGS,
    INPUT_CAN_LINK_ACTIVE, INPUT_DUMP_REQUEST, INPUT_FILL_REQUEST, INPUT_FIRE_REQUEST, INPUT_FLAGS,
    INPUT_GPIO_STATUS, INPUT_O2_TEST_REQUEST, INPUT_RESET_ACK_REQUEST, INPUT_SEPARATE_REQUEST,
    INPUT_VALVE_OPEN_REQUEST, INPUT_VALVE_SET_REQUEST, OUTPUT_STATUS,
    SERVO_COMM_ACTIVE, SERVO_COMM_ERROR,
    can::{
        health::{CanHealth, classify_can_health},
        protocol::{CAN_ID_BUTTON_FROM_CTRL_PANEL, CanDecodeError, GseCanMessage},
        tx::{CanTxError, create_frame_from_message, transmit_frame_with_timeout},
    },
    clear_fault_flags_for_reset, position_to_angle_x10, replace_operator_input_flags,
    replace_operator_input_flags_and_set_can_link_active, sequence_phase, set_fault_flags,
    signal_reset_ack_event,
    tasks::espnow::latest_log_data,
    update_input_flag,
};

const CAN_HEALTH_MONITOR_INTERVAL_MS: u64 = 100;
const BUTTON_RESET_ACK_BIT: u8 = 1 << 7;
const BUTTON_COMMAND_BITS: u8 = BUTTON_RESET_ACK_BIT - 1;
const STATUS_FRAME_COUNT: u8 = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CanTxRuntimeState {
    Normal,
    SuspendedAfterTimeout,
    ProbePending,
}

#[embassy_executor::task]
pub async fn can_manager_task(mut can: twai::Twai<'static, Async>) {
    let start = Instant::now();
    let mut last_peer_rx: Option<Instant> = None;
    let mut restart_attempted = false;
    let mut tx_runtime_state = CanTxRuntimeState::Normal;
    let mut status_slot = 0u8;
    let mut reset_ack_prev = false;
    let mut tx_ticker = Ticker::every(Duration::from_millis(CAN_STATUS_TX_INTERVAL_MS));
    let mut health_ticker = Ticker::every(Duration::from_millis(CAN_HEALTH_MONITOR_INTERVAL_MS));

    update_can_health(&can);

    loop {
        if CAN_HEALTH.load(Ordering::Acquire) != CanHealth::BusOff as u8
            && tx_runtime_state == CanTxRuntimeState::Normal
        {
            restart_attempted = false;
        }

        let mut try_restart = false;
        match select3(can.receive_async(), tx_ticker.next(), health_ticker.next()).await {
            Either3::First(receive_result) => match receive_result {
                Ok(frame) => {
                    let health = update_can_health(&can);
                    if handle_received_frame(&frame, health, &mut reset_ack_prev, tx_runtime_state)
                    {
                        last_peer_rx = Some(Instant::now());
                        if tx_runtime_state == CanTxRuntimeState::SuspendedAfterTimeout {
                            tx_runtime_state = CanTxRuntimeState::ProbePending;
                        }
                    }
                }
                Err(error) => {
                    record_rx_error(error, &can);
                    if update_can_health(&can) == CanHealth::BusOff {
                        tx_runtime_state = CanTxRuntimeState::SuspendedAfterTimeout;
                        try_restart = true;
                    }
                }
            },
            Either3::Second(()) => {
                update_peer_liveness(start, last_peer_rx);
                let health = update_can_health(&can);
                if health == CanHealth::BusOff {
                    tx_runtime_state = CanTxRuntimeState::SuspendedAfterTimeout;
                    try_restart = true;
                } else {
                    match tx_runtime_state {
                        CanTxRuntimeState::Normal => {
                            if can_tx_allowed(tx_runtime_state, false) {
                                match transmit_status_frame(&mut can, status_slot).await {
                                    Ok(()) => status_slot = (status_slot + 1) % STATUS_FRAME_COUNT,
                                    Err(error) => {
                                        try_restart |=
                                            handle_normal_tx_error(error, &mut tx_runtime_state);
                                    }
                                }
                            }
                        }
                        CanTxRuntimeState::SuspendedAfterTimeout => {}
                        CanTxRuntimeState::ProbePending => {
                            if can_tx_allowed(tx_runtime_state, true) {
                                match transmit_probe(&mut can).await {
                                    Ok(()) => tx_runtime_state = CanTxRuntimeState::Normal,
                                    Err(error) => {
                                        try_restart |=
                                            handle_probe_tx_error(error, &mut tx_runtime_state);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            Either3::Third(()) => {
                update_peer_liveness(start, last_peer_rx);
                if update_can_health(&can) == CanHealth::BusOff {
                    tx_runtime_state = CanTxRuntimeState::SuspendedAfterTimeout;
                    try_restart = true;
                }
            }
        }

        if try_restart && !restart_attempted {
            // Restore receive/health servicing once; runtime TX state still controls future TX.
            can = can.stop().start();
            restart_attempted = true;
        } else if try_restart {
            Timer::after(Duration::from_millis(CAN_HEALTH_MONITOR_INTERVAL_MS)).await;
        }
    }
}

fn handle_received_frame(
    frame: &EspTwaiFrame,
    health: CanHealth,
    reset_ack_prev: &mut bool,
    tx_runtime_state: CanTxRuntimeState,
) -> bool {
    let id = match frame.id() {
        Id::Standard(id) => id.as_raw(),
        Id::Extended(_) => return false,
    };

    match GseCanMessage::decode_standard(id, frame.data()) {
        Ok(GseCanMessage::ButtonFromCtrlPanel { raw }) => {
            let reset_ack_now = raw & BUTTON_RESET_ACK_BIT != 0;
            if health == CanHealth::BusOff {
                *reset_ack_prev = reset_ack_now;
                inhibit_can_inputs();
                return false;
            }

            // A valid fresh command frame makes operator input trustworthy again. Latched CAN
            // fault flags remain for status/reset policy, and the supervisor keeps Abort.
            CAN_COMM_ACTIVE.store(true, Ordering::Release);
            replace_operator_input_flags_and_set_can_link_active(button_inputs(raw));

            if reset_ack_now && !*reset_ack_prev && raw & BUTTON_COMMAND_BITS == 0 {
                handle_reset_ack_edge(tx_runtime_state);
            }
            *reset_ack_prev = reset_ack_now;
            true
        }
        Err(CanDecodeError::InvalidDlc {
            id: CAN_ID_BUTTON_FROM_CTRL_PANEL,
            ..
        }) => {
            CAN_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            false
        }
        Ok(_) | Err(CanDecodeError::UnknownId(_)) | Err(CanDecodeError::InvalidDlc { .. }) => false,
    }
}

fn button_inputs(raw: u8) -> u32 {
    let mut inputs = 0;
    if raw & (1 << 0) != 0 {
        inputs |= INPUT_DUMP_REQUEST;
    }
    if raw & (1 << 1) != 0 {
        inputs |= INPUT_FIRE_REQUEST;
    }
    if raw & (1 << 2) != 0 {
        inputs |= INPUT_FILL_REQUEST;
    }
    if raw & (1 << 3) != 0 {
        inputs |= INPUT_SEPARATE_REQUEST;
    }
    if raw & (1 << 4) != 0 {
        inputs |= INPUT_VALVE_SET_REQUEST;
    }
    if raw & (1 << 5) != 0 {
        inputs |= INPUT_O2_TEST_REQUEST;
    }
    if raw & (1 << 6) != 0 {
        inputs |= INPUT_VALVE_OPEN_REQUEST;
    }
    if raw & BUTTON_RESET_ACK_BIT != 0 {
        inputs |= INPUT_RESET_ACK_REQUEST;
    }
    inputs
}

fn handle_reset_ack_edge(tx_runtime_state: CanTxRuntimeState) {
    let health = CAN_HEALTH.load(Ordering::Acquire);
    if health == CanHealth::BusOff as u8 {
        return;
    }

    let mut clearable_faults = CAN_PEER_LOST;
    if tx_runtime_state == CanTxRuntimeState::Normal {
        clearable_faults |= CAN_TX_TIMEOUT;
    }
    if health == CanHealth::Active as u8 {
        clearable_faults |= CAN_BUS_OFF;
    }

    let faults_to_clear = FAULT_FLAGS.load(Ordering::Acquire) & clearable_faults;
    if faults_to_clear != 0 {
        clear_fault_flags_for_reset(faults_to_clear);
    }
    signal_reset_ack_event();
}

fn update_peer_liveness(start: Instant, last_peer_rx: Option<Instant>) {
    let now = Instant::now();
    let timed_out = match last_peer_rx {
        Some(last_rx) => {
            now.duration_since(last_rx) >= Duration::from_millis(COMMUNICATION_TIMEOUT_MS)
        }
        None => now.duration_since(start) >= Duration::from_millis(COMMUNICATION_TIMEOUT_MS),
    };
    if timed_out {
        inhibit_can_inputs();
        set_fault_flags(CAN_PEER_LOST);
    }
}

fn can_tx_allowed(tx_runtime_state: CanTxRuntimeState, probe: bool) -> bool {
    if CAN_HEALTH.load(Ordering::Acquire) == CanHealth::BusOff as u8 {
        return false;
    }

    matches!(
        (tx_runtime_state, probe),
        (CanTxRuntimeState::Normal, false) | (CanTxRuntimeState::ProbePending, true)
    )
}

async fn transmit_status_frame(
    can: &mut twai::Twai<'static, Async>,
    status_slot: u8,
) -> Result<(), CanTxError> {
    let msg = match status_slot % STATUS_FRAME_COUNT {
        0 => GseCanMessage::OutputGpioStatus {
            output_bits: OUTPUT_STATUS.load(Ordering::Acquire),
        },
        1 => GseCanMessage::InputGpioStatus {
            input_bits: INPUT_GPIO_STATUS.load(Ordering::Acquire),
        },
        2 => GseCanMessage::MainValveAngleToCtrlPanel {
            angle_x10: position_to_angle_x10(CURRENT_POSITION.load(Ordering::Acquire)),
        },
        3 => internal_status_message(),
        _ => {
            let Some(log_data) = latest_log_data() else {
                return Ok(());
            };
            GseCanMessage::LoggerData {
                adc0: log_data.adc0,
                adc2: log_data.adc2,
                adc3: log_data.adc3,
                counter: log_data.counter,
            }
        }
    };

    transmit(can, msg, can_tx_allowed(CanTxRuntimeState::Normal, false)).await
}

async fn transmit_probe(can: &mut twai::Twai<'static, Async>) -> Result<(), CanTxError> {
    transmit(
        can,
        internal_status_message(),
        can_tx_allowed(CanTxRuntimeState::ProbePending, true),
    )
    .await
}

fn internal_status_message() -> GseCanMessage {
    let mut flags = FAULT_FLAGS.load(Ordering::Acquire);
    if !SERVO_COMM_ACTIVE.load(Ordering::Acquire) {
        flags |= SERVO_COMM_ERROR;
    }

    GseCanMessage::InternalStatus {
        phase: sequence_phase() as u8,
        flags,
    }
}

async fn transmit(
    can: &mut twai::Twai<'static, Async>,
    msg: GseCanMessage,
    tx_allowed: bool,
) -> Result<(), CanTxError> {
    if !tx_allowed {
        return Err(CanTxError::TxInhibited);
    }

    let frame = match create_frame_from_message(msg) {
        Ok(frame) => frame,
        Err(_) => {
            record_tx_error(CanTxError::FrameCreateFailed, can);
            return Err(CanTxError::FrameCreateFailed);
        }
    };

    let result =
        transmit_frame_with_timeout(can, &frame, Duration::from_millis(CAN_TX_TIMEOUT_MS)).await;
    if let Err(error) = result {
        record_tx_error(error, can);
    }
    result
}

fn handle_normal_tx_error(error: CanTxError, tx_runtime_state: &mut CanTxRuntimeState) -> bool {
    match error {
        CanTxError::TimedOutUnknownState | CanTxError::BusOff => {
            *tx_runtime_state = CanTxRuntimeState::SuspendedAfterTimeout;
            true
        }
        CanTxError::FrameCreateFailed | CanTxError::TransmitFailed | CanTxError::TxInhibited => {
            false
        }
    }
}

fn handle_probe_tx_error(error: CanTxError, tx_runtime_state: &mut CanTxRuntimeState) -> bool {
    *tx_runtime_state = CanTxRuntimeState::SuspendedAfterTimeout;
    inhibit_can_inputs();
    matches!(error, CanTxError::TimedOutUnknownState | CanTxError::BusOff)
}

fn update_can_health(can: &twai::Twai<'static, Async>) -> CanHealth {
    let tec = can.transmit_error_count();
    let rec = can.receive_error_count();
    CAN_TEC.store(tec, Ordering::Relaxed);
    CAN_REC.store(rec, Ordering::Relaxed);

    let health = classify_can_health(tec, rec, can.is_bus_off());
    CAN_HEALTH.store(health as u8, Ordering::Release);
    if health == CanHealth::BusOff {
        inhibit_can_inputs();
        set_fault_flags(CAN_BUS_OFF);
    }
    health
}

fn record_tx_error(error: CanTxError, can: &twai::Twai<'static, Async>) {
    match error {
        CanTxError::FrameCreateFailed => {
            CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            inhibit_can_inputs();
            set_fault_flags(CAN_TX_FRAME_CREATE_FAILED);
        }
        CanTxError::TimedOutUnknownState => {
            CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            // Timeout only proves that completion was not observed; it is not safe to retry.
            inhibit_can_inputs();
            set_fault_flags(CAN_TX_TIMEOUT);
            update_can_health(can);
        }
        CanTxError::BusOff => {
            CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            CAN_HEALTH.store(CanHealth::BusOff as u8, Ordering::Release);
            inhibit_can_inputs();
            set_fault_flags(CAN_BUS_OFF);
        }
        CanTxError::TransmitFailed => {
            CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
        }
        CanTxError::TxInhibited => {}
    }
}

fn record_rx_error(error: EspTwaiError, can: &twai::Twai<'static, Async>) {
    CAN_RX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
    handle_can_error(error, can);
}

fn handle_can_error(error: EspTwaiError, can: &twai::Twai<'static, Async>) {
    match error {
        EspTwaiError::BusOff => {
            CAN_HEALTH.store(CanHealth::BusOff as u8, Ordering::Release);
            inhibit_can_inputs();
            set_fault_flags(CAN_BUS_OFF);
        }
        EspTwaiError::EmbeddedHAL(TwaiErrorKind::Overrun) => {
            can.clear_receive_fifo();
            inhibit_can_inputs();
            set_fault_flags(CAN_PEER_LOST);
        }
        EspTwaiError::EmbeddedHAL(_) | EspTwaiError::NonCompliantDlc(_) => {}
    }
}

fn inhibit_can_inputs() {
    CAN_COMM_ACTIVE.store(false, Ordering::Release);
    if INPUT_FLAGS.load(Ordering::Acquire) & INPUT_CAN_LINK_ACTIVE != 0 {
        update_input_flag(INPUT_CAN_LINK_ACTIVE, false);
    }
    replace_operator_input_flags(0);
}
