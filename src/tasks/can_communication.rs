use core::sync::atomic::Ordering;

use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_can::{Frame, Id};
use esp_hal::{
    Async,
    twai::{self, ErrorKind as TwaiErrorKind, EspTwaiError, EspTwaiFrame},
};

use crate::{
    CAN_BUS_OFF, CAN_FAULTS, CAN_HEALTH, CAN_PEER_LOST, CAN_REC, CAN_RX_ERROR_COUNT,
    CAN_STATUS_TX_INTERVAL_MS, CAN_TEC, CAN_TX_ERROR_COUNT, CAN_TX_FRAME_CREATE_FAILED,
    CAN_TX_TIMEOUT, CAN_TX_TIMEOUT_MS, COMMUNICATION_TIMEOUT_MS, CURRENT_POSITION, FAULT_FLAGS,
    INPUT_CAN_LINK_ACTIVE, INPUT_DUMP_REQUEST, INPUT_FILL_REQUEST, INPUT_FIRE_REQUEST, INPUT_FLAGS,
    INPUT_O2_TEST_REQUEST, INPUT_SEPARATE_REQUEST, INPUT_VALVE_OPEN_REQUEST,
    INPUT_VALVE_SET_REQUEST, OUTPUT_STATUS, SERVO_FAULTS,
    can::{
        health::{CanHealth, classify_can_health},
        protocol::{CAN_ID_BUTTON_FROM_CTRL_PANEL, CanDecodeError, GseCanMessage},
        tx::{CanTxError, create_frame_from_message, transmit_frame_with_timeout},
    },
    position_to_angle_x10, replace_operator_input_flags, sequence_phase, set_fault_flags,
    update_input_flag,
};

const CAN_HEALTH_MONITOR_INTERVAL_MS: u64 = 100;
// A lost command peer is unsafe for this board, so status TX remains stopped until explicit reset.
const CAN_TX_INHIBIT_FAULTS: u32 =
    CAN_PEER_LOST | CAN_BUS_OFF | CAN_TX_TIMEOUT | CAN_TX_FRAME_CREATE_FAILED;

#[embassy_executor::task]
pub async fn can_manager_task(mut can: twai::Twai<'static, Async>) {
    let start = Instant::now();
    let mut last_peer_rx: Option<Instant> = None;
    let mut restart_attempted = false;
    let mut tx_ticker = Ticker::every(Duration::from_millis(CAN_STATUS_TX_INTERVAL_MS));
    let mut health_ticker = Ticker::every(Duration::from_millis(CAN_HEALTH_MONITOR_INTERVAL_MS));

    update_can_health(&can);

    loop {
        if FAULT_FLAGS.load(Ordering::Acquire) & CAN_TX_INHIBIT_FAULTS == 0 {
            // Explicit fault clearing begins a new CAN fault/restart cycle.
            restart_attempted = false;
        }

        let mut try_restart = false;
        match select3(can.receive_async(), tx_ticker.next(), health_ticker.next()).await {
            Either3::First(receive_result) => match receive_result {
                Ok(frame) => {
                    if handle_received_frame(&frame) {
                        last_peer_rx = Some(Instant::now());
                    }
                    update_can_health(&can);
                }
                Err(error) => {
                    record_rx_error(error, &can);
                    if update_can_health(&can) == CanHealth::BusOff {
                        try_restart = true;
                    }
                }
            },
            Either3::Second(()) => {
                update_peer_liveness(start, last_peer_rx);
                let bus_off = update_can_health(&can) == CanHealth::BusOff;
                let tx_requires_restart = !bus_off
                    && can_tx_allowed()
                    && matches!(
                        transmit_status(&mut can).await,
                        Err(CanTxError::TimedOutUnknownState | CanTxError::BusOff)
                    );
                try_restart = bus_off || tx_requires_restart;
            }
            Either3::Third(()) => {
                update_peer_liveness(start, last_peer_rx);
                if update_can_health(&can) == CanHealth::BusOff {
                    try_restart = true;
                }
            }
        }

        if try_restart && !restart_attempted {
            // Restore receive/health servicing once; latched faults still inhibit all new TX.
            can = can.stop().start();
            restart_attempted = true;
        } else if try_restart {
            Timer::after(Duration::from_millis(CAN_HEALTH_MONITOR_INTERVAL_MS)).await;
        }
    }
}

fn handle_received_frame(frame: &EspTwaiFrame) -> bool {
    let id = match frame.id() {
        Id::Standard(id) => id.as_raw(),
        Id::Extended(_) => return false,
    };

    match GseCanMessage::decode_standard(id, frame.data()) {
        Ok(GseCanMessage::ButtonFromCtrlPanel { raw }) => {
            if FAULT_FLAGS.load(Ordering::Acquire) & CAN_FAULTS == 0 {
                replace_operator_input_flags(button_inputs(raw));
                update_input_flag(INPUT_CAN_LINK_ACTIVE, true);
            } else {
                inhibit_can_inputs();
            }
            // Keep liveness observation current while faults remain latched; valid traffic does
            // not re-enable commands or clear faults without an explicit reset.
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
    inputs
}

fn update_peer_liveness(start: Instant, last_peer_rx: Option<Instant>) {
    let now = Instant::now();
    let timed_out = match last_peer_rx {
        Some(last_rx) => {
            now.duration_since(last_rx) >= Duration::from_millis(COMMUNICATION_TIMEOUT_MS)
        }
        None => now.duration_since(start) >= Duration::from_millis(COMMUNICATION_TIMEOUT_MS),
    };
    let bus_off_latched = FAULT_FLAGS.load(Ordering::Acquire) & CAN_BUS_OFF != 0;

    if timed_out || bus_off_latched {
        inhibit_can_inputs();
        if timed_out {
            set_fault_flags(CAN_PEER_LOST);
        }
    }
}

fn can_tx_allowed() -> bool {
    FAULT_FLAGS.load(Ordering::Acquire) & CAN_TX_INHIBIT_FAULTS == 0
}

async fn transmit_status(can: &mut twai::Twai<'static, Async>) -> Result<(), CanTxError> {
    let phase = sequence_phase() as u8;
    let solenoids = OUTPUT_STATUS.load(Ordering::Acquire);
    let angle_x10 = position_to_angle_x10(CURRENT_POSITION.load(Ordering::Acquire));
    let servo_error = FAULT_FLAGS.load(Ordering::Acquire) & SERVO_FAULTS != 0;

    transmit(can, GseCanMessage::SequenceState { phase }).await?;
    transmit(can, GseCanMessage::SolenoidState { bits: solenoids }).await?;
    transmit(can, GseCanMessage::MainValveAngleToCtrlPanel { angle_x10 }).await?;
    transmit(
        can,
        GseCanMessage::ServoCommunicationState { error: servo_error },
    )
    .await
}

async fn transmit(
    can: &mut twai::Twai<'static, Async>,
    msg: GseCanMessage,
) -> Result<(), CanTxError> {
    if !can_tx_allowed() {
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

fn update_can_health(can: &twai::Twai<'static, Async>) -> CanHealth {
    let tec = can.transmit_error_count();
    let rec = can.receive_error_count();
    CAN_TEC.store(tec, Ordering::Relaxed);
    CAN_REC.store(rec, Ordering::Relaxed);

    let health = classify_can_health(tec, rec, can.is_bus_off());
    CAN_HEALTH.store(health as u8, Ordering::Release);
    if health == CanHealth::BusOff {
        set_fault_flags(CAN_BUS_OFF);
        inhibit_can_inputs();
    }
    health
}

fn record_tx_error(error: CanTxError, can: &twai::Twai<'static, Async>) {
    match error {
        CanTxError::FrameCreateFailed => {
            CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            set_fault_flags(CAN_TX_FRAME_CREATE_FAILED);
            inhibit_can_inputs();
        }
        CanTxError::TimedOutUnknownState => {
            CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            // Timeout only proves that completion was not observed; it is not safe to retry.
            set_fault_flags(CAN_TX_TIMEOUT);
            inhibit_can_inputs();
            update_can_health(can);
        }
        CanTxError::BusOff => {
            CAN_TX_ERROR_COUNT.fetch_add(1, Ordering::Relaxed);
            CAN_HEALTH.store(CanHealth::BusOff as u8, Ordering::Release);
            set_fault_flags(CAN_BUS_OFF);
            inhibit_can_inputs();
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
            set_fault_flags(CAN_BUS_OFF);
            inhibit_can_inputs();
        }
        EspTwaiError::EmbeddedHAL(TwaiErrorKind::Overrun) => {
            can.clear_receive_fifo();
            inhibit_can_inputs();
        }
        EspTwaiError::EmbeddedHAL(_) | EspTwaiError::NonCompliantDlc(_) => {}
    }
}

fn inhibit_can_inputs() {
    if INPUT_FLAGS.load(Ordering::Acquire) & INPUT_CAN_LINK_ACTIVE != 0 {
        update_input_flag(INPUT_CAN_LINK_ACTIVE, false);
    }
    replace_operator_input_flags(0);
}
