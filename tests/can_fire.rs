#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_time::{Duration, Instant, Ticker};
use embedded_can::{Frame, Id};
use esp_backtrace as _;
use esp_hal::{
    Async,
    clock::CpuClock,
    gpio::{Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
    twai::{self, BaudRate, EspTwaiError, TwaiMode, filter::SingleStandardFilter},
};
use esp_println::println;
use gse_integrated_board::{
    CAN_ID_BUTTON_FROM_CTRL_PANEL, CAN_TX_TIMEOUT_MS, MAIN_VALVE_CLOSED_ANGLE_X10, OUT_IGNITER,
    can::{
        protocol::{CanDecodeError, GseCanMessage},
        tx::{CanTxError, create_frame_from_message, transmit_frame_with_timeout},
    },
};

esp_bootloader_esp_idf::esp_app_desc!();

const COMMAND_TIMEOUT_MS: u64 = 500;
const COMMAND_WATCH_INTERVAL_MS: u64 = 50;
const STATUS_TX_INTERVAL_MS: u64 = 50;
const CAN_RECOVERY_RETRY_INTERVAL_MS: u64 = 500;
const FIRE_BIT: u8 = 1 << 1;
const STATUS_FRAME_COUNT: u8 = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CanTxRuntimeState {
    Normal,
    SuspendedAfterTimeout,
    ProbePending,
}

struct TestOutputs {
    fire: Output<'static>,
    dump: Output<'static>,
    fill: Output<'static>,
    separate: Output<'static>,
    o2: Output<'static>,
    spare_solenoid: Output<'static>,
}

impl TestOutputs {
    fn set_fire(&mut self, on: bool) {
        self.fire.set_level(on.into());
    }

    fn set_all_low(&mut self) {
        self.fire.set_low();
        self.dump.set_low();
        self.fill.set_low();
        self.separate.set_low();
        self.o2.set_low();
        self.spare_solenoid.set_low();
    }
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(size: 32 * 1024);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let mut outputs = TestOutputs {
        fire: Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default()),
        dump: Output::new(peripherals.GPIO17, Level::Low, OutputConfig::default()),
        fill: Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default()),
        separate: Output::new(peripherals.GPIO33, Level::Low, OutputConfig::default()),
        o2: Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default()),
        spare_solenoid: Output::new(peripherals.GPIO32, Level::Low, OutputConfig::default()),
    };
    outputs.set_all_low();

    let mut can_config = twai::TwaiConfiguration::new(
        peripherals.TWAI0,
        peripherals.GPIO4,
        peripherals.GPIO18,
        BaudRate::B125K,
        TwaiMode::Normal,
    )
    .into_async();
    can_config.set_filter(
        const { SingleStandardFilter::new(b"0xxxxxxxxxx", b"x", [b"xxxxxxxx", b"xxxxxxxx"]) },
    );
    let mut can = can_config.start();

    print_start_banner();

    let mut fire_on = false;
    let mut last_valid_command: Option<Instant> = None;
    let mut status_slot = 0;
    let mut tx_runtime_state = CanTxRuntimeState::Normal;
    let mut next_restart_at: Option<Instant> = None;
    let mut command_watch = Ticker::every(Duration::from_millis(COMMAND_WATCH_INTERVAL_MS));
    let mut status_tx = Ticker::every(Duration::from_millis(STATUS_TX_INTERVAL_MS));

    loop {
        let mut restart_now = false;
        match select3(can.receive_async(), command_watch.next(), status_tx.next()).await {
            Either3::First(receive_result) => match receive_result {
                Ok(frame) => {
                    let Id::Standard(id) = frame.id() else {
                        continue;
                    };
                    let id = id.as_raw();

                    match GseCanMessage::decode_standard(id, frame.data()) {
                        Ok(GseCanMessage::ButtonFromCtrlPanel { raw }) => {
                            if tx_runtime_state == CanTxRuntimeState::SuspendedAfterTimeout {
                                tx_runtime_state = CanTxRuntimeState::ProbePending;
                                next_restart_at = None;
                                println!("can_tx_state=probe_pending");
                            }

                            if tx_runtime_state != CanTxRuntimeState::Normal {
                                force_fire_off(
                                    &mut outputs,
                                    &mut fire_on,
                                    &mut last_valid_command,
                                    "can_tx_not_recovered",
                                );
                                continue;
                            }

                            last_valid_command = Some(Instant::now());
                            let requested = raw & FIRE_BIT != 0;
                            outputs.set_fire(requested);
                            if requested != fire_on {
                                fire_on = requested;
                                println!(
                                    "fire={} raw=0x{:02x}",
                                    if fire_on { "ON" } else { "OFF" },
                                    raw
                                );
                            }
                        }
                        Err(CanDecodeError::InvalidDlc {
                            id: CAN_ID_BUTTON_FROM_CTRL_PANEL,
                            expected,
                            actual,
                        }) => {
                            force_fire_off(
                                &mut outputs,
                                &mut fire_on,
                                &mut last_valid_command,
                                "invalid_dlc",
                            );
                            println!(
                                "can_rx_invalid_dlc id=0x{:03x} expected={} actual={}",
                                CAN_ID_BUTTON_FROM_CTRL_PANEL, expected, actual
                            );
                        }
                        Ok(_)
                        | Err(CanDecodeError::UnknownId(_))
                        | Err(CanDecodeError::InvalidDlc { .. }) => {}
                    }
                }
                Err(error) => {
                    force_fire_off(
                        &mut outputs,
                        &mut fire_on,
                        &mut last_valid_command,
                        "can_receive_error",
                    );
                    println!("can_receive_error error={:?}", error);
                    if matches!(error, EspTwaiError::BusOff) {
                        suspend_tx(
                            &mut tx_runtime_state,
                            &mut outputs,
                            &mut fire_on,
                            &mut last_valid_command,
                        );
                        restart_now = true;
                    }
                }
            },
            Either3::Second(()) => {
                if fire_on
                    && last_valid_command.is_some_and(|last| {
                        Instant::now().duration_since(last)
                            >= Duration::from_millis(COMMAND_TIMEOUT_MS)
                    })
                {
                    force_fire_off(
                        &mut outputs,
                        &mut fire_on,
                        &mut last_valid_command,
                        "command_timeout",
                    );
                }
            }
            Either3::Third(()) => match tx_runtime_state {
                CanTxRuntimeState::Normal => {
                    let message = status_message(status_slot, fire_on);
                    match transmit_message(&mut can, message).await {
                        Ok(()) => status_slot = (status_slot + 1) % STATUS_FRAME_COUNT,
                        Err(error) => {
                            println!("can_status_tx_error slot={} error={:?}", status_slot, error);
                            if tx_error_suspends(error) {
                                suspend_tx(
                                    &mut tx_runtime_state,
                                    &mut outputs,
                                    &mut fire_on,
                                    &mut last_valid_command,
                                );
                                restart_now = true;
                            } else {
                                println!("can_status_retry slot={}", status_slot);
                            }
                        }
                    }
                }
                CanTxRuntimeState::SuspendedAfterTimeout => {
                    if next_restart_at.is_some_and(|deadline| Instant::now() >= deadline) {
                        restart_now = true;
                    }
                }
                CanTxRuntimeState::ProbePending => {
                    match transmit_message(&mut can, status_message(3, false)).await {
                        Ok(()) => {
                            tx_runtime_state = CanTxRuntimeState::Normal;
                            next_restart_at = None;
                            println!("can_probe=success can_tx_state=normal");
                        }
                        Err(error) => {
                            println!("can_probe=failed error={:?}", error);
                            if tx_error_suspends(error) {
                                suspend_tx(
                                    &mut tx_runtime_state,
                                    &mut outputs,
                                    &mut fire_on,
                                    &mut last_valid_command,
                                );
                                let retry_at = Instant::now()
                                    + Duration::from_millis(CAN_RECOVERY_RETRY_INTERVAL_MS);
                                next_restart_at = Some(retry_at);
                                println!(
                                    "can_recovery=scheduled delay_ms={}",
                                    CAN_RECOVERY_RETRY_INTERVAL_MS
                                );
                            } else {
                                println!("can_probe=retry");
                            }
                        }
                    }
                }
            },
        }

        if restart_now {
            can = can.stop().start();
            tx_runtime_state = CanTxRuntimeState::ProbePending;
            next_restart_at = None;
            println!("can_restart=attempted");
            println!("can_tx_state=probe_pending");
        }
    }
}

async fn transmit_message(
    can: &mut twai::Twai<'static, Async>,
    message: GseCanMessage,
) -> Result<(), CanTxError> {
    let frame = create_frame_from_message(message).map_err(|_| CanTxError::FrameCreateFailed)?;
    transmit_frame_with_timeout(can, &frame, Duration::from_millis(CAN_TX_TIMEOUT_MS)).await
}

fn tx_error_suspends(error: CanTxError) -> bool {
    matches!(error, CanTxError::TimedOutUnknownState | CanTxError::BusOff)
}

fn suspend_tx(
    state: &mut CanTxRuntimeState,
    outputs: &mut TestOutputs,
    fire_on: &mut bool,
    last_valid_command: &mut Option<Instant>,
) {
    *state = CanTxRuntimeState::SuspendedAfterTimeout;
    force_fire_off(outputs, fire_on, last_valid_command, "can_tx_suspended");
    println!("can_tx_state=suspended");
}

fn force_fire_off(
    outputs: &mut TestOutputs,
    fire_on: &mut bool,
    last_valid_command: &mut Option<Instant>,
    reason: &str,
) {
    outputs.set_fire(false);
    *last_valid_command = None;
    if *fire_on {
        *fire_on = false;
        println!("fire=OFF reason={}", reason);
    }
}

fn status_message(slot: u8, fire_on: bool) -> GseCanMessage {
    match slot % STATUS_FRAME_COUNT {
        0 => GseCanMessage::OutputGpioStatus {
            output_bits: if fire_on { OUT_IGNITER } else { 0 },
        },
        1 => GseCanMessage::InputGpioStatus { input_bits: 0 },
        2 => GseCanMessage::MainValveAngleToCtrlPanel {
            angle_x10: MAIN_VALVE_CLOSED_ANGLE_X10,
        },
        3 => GseCanMessage::InternalStatus { phase: 0, flags: 0 },
        _ => GseCanMessage::LoggerData {
            adc0: 0,
            adc2: 0,
            adc3: 0,
            counter: 0,
        },
    }
}

fn print_start_banner() {
    println!("========== CAN FIRE TEST START ==========");
    println!("CAN: TWAI0 baud=125k tx=GPIO18 rx=GPIO4");
    println!("OUTPUT: FIRE/IGNITER=GPIO13");
    println!(
        "BUTTON: CAN ID=0x{:03x} fire_bit=1",
        CAN_ID_BUTTON_FROM_CTRL_PANEL
    );
    println!("COMMAND_TIMEOUT_MS={}", COMMAND_TIMEOUT_MS);
    println!("STATUS_TX_INTERVAL_MS={} frames=all", STATUS_TX_INTERVAL_MS);
    println!(
        "CAN_RECOVERY_RETRY_INTERVAL_MS={}",
        CAN_RECOVERY_RETRY_INTERVAL_MS
    );
    println!("CAN_TX_RECOVERY=production_style");
}
