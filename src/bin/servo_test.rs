#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, DataBits, Parity, StopBits, Uart},
};
use esp_println::println;
use gse_integrated_board::{
    MAIN_VALVE_CLOSED_ANGLE_X10, MAIN_VALVE_OPEN_ANGLE_X10, SERVO_CENTER_POS, SERVO_MAX_POS,
    SERVO_MIN_POS, angle_x10_to_position,
    krs_servo::{IcsDevice, IcsError},
    position_to_angle_x10,
};

esp_bootloader_esp_idf::esp_app_desc!();

const SERVO_TEST_ID: u8 = 0;
const SERVO_TEST_STEP_POS: u16 = 300;
const SERVO_TEST_HOLD_MS: u64 = 1000;
const SERVO_TEST_SETTLE_MS: u64 = 500;
const SERVO_TEST_FREE_AT_END: bool = false;
const SERVO_TEST_REPEAT: bool = true;
const SERVO_TEST_OPEN_CLOSE_FULL_RANGE: bool = false;

#[derive(Debug)]
struct ServoTestSummary {
    detected_id: u8,
    initial_position: u16,
    initial_angle_x10: i16,
    final_position: u16,
    final_angle_x10: i16,
    move_count: u32,
    communication_success_count: u32,
    center_position: u16,
    min_test_position: u16,
    max_test_position: u16,
    free_at_end: bool,
}

#[derive(Debug)]
enum ServoTestError {
    GetIdFailed {
        stage: &'static str,
        source: IcsError,
    },
    GetPosFailed {
        stage: &'static str,
        source: IcsError,
    },
    SetCenterFailed {
        stage: &'static str,
        expected: u16,
        source: IcsError,
    },
    SmallMoveFailed {
        stage: &'static str,
        expected: u16,
        source: IcsError,
    },
    ReturnCenterFailed {
        stage: &'static str,
        expected: u16,
        source: IcsError,
    },
    FreeFailed {
        stage: &'static str,
        source: IcsError,
    },
    InvalidTestPosition {
        stage: &'static str,
        position: u16,
    },
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

    let _dump = Output::new(peripherals.GPIO17, Level::Low, OutputConfig::default());
    let _ignition = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());
    let _o2 = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
    let _fill = Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default());
    let _separate = Output::new(peripherals.GPIO33, Level::Low, OutputConfig::default());
    let _spare_solenoid = Output::new(peripherals.GPIO32, Level::Low, OutputConfig::default());

    let servo_tx = Output::new(peripherals.GPIO16, Level::Low, OutputConfig::default());
    let servo_rx = Input::new(peripherals.GPIO14, InputConfig::default());
    let servo_enable = Output::new(peripherals.GPIO27, Level::Low, OutputConfig::default());

    let uart_config = UartConfig::default()
        .with_baudrate(115_200)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::Even)
        .with_stop_bits(StopBits::_1);
    let uart = Uart::new(peripherals.UART1, uart_config)
        .unwrap()
        .with_rx(servo_rx)
        .with_tx(servo_tx);
    let mut servo = IcsDevice::new(uart, servo_enable);

    print_start_banner();

    match run_servo_tests(&mut servo).await {
        Ok(summary) => {
            print_pass_summary(&summary);
            pass_idle_loop().await
        }
        Err(error) => {
            print_fail_summary(&error);
            fault_backoff_loop().await
        }
    }
}

async fn run_servo_tests(krs: &mut IcsDevice<'_>) -> Result<ServoTestSummary, ServoTestError> {
    let center_position = SERVO_CENTER_POS;
    let min_test_position = checked_test_position(
        center_position.saturating_sub(SERVO_TEST_STEP_POS),
        "small_sweep_min",
    )?;
    let max_test_position = checked_test_position(
        center_position.saturating_add(SERVO_TEST_STEP_POS),
        "small_sweep_max",
    )?;

    println!("stage=get_id ...");
    let detected_id = krs.get_id().map_err(|source| ServoTestError::GetIdFailed {
        stage: "get_id",
        source,
    })?;
    println!("stage=get_id detected_id={}", detected_id);

    println!("stage=get_pos ...");
    let initial_position =
        krs.get_pos(SERVO_TEST_ID)
            .map_err(|source| ServoTestError::GetPosFailed {
                stage: "get_pos_initial",
                source,
            })?;
    println!(
        "stage=get_pos position={} angle_x10={}",
        initial_position,
        position_to_angle_x10(initial_position)
    );

    let mut summary = ServoTestSummary {
        detected_id,
        initial_position,
        initial_angle_x10: position_to_angle_x10(initial_position),
        final_position: initial_position,
        final_angle_x10: position_to_angle_x10(initial_position),
        move_count: 0,
        communication_success_count: 2,
        center_position,
        min_test_position,
        max_test_position,
        free_at_end: SERVO_TEST_FREE_AT_END,
    };

    println!("stage=set_center target={}", center_position);
    let center_return = krs
        .set_pos(SERVO_TEST_ID, center_position)
        .map_err(|source| ServoTestError::SetCenterFailed {
            stage: "set_center",
            expected: center_position,
            source,
        })?;
    summary.move_count += 1;
    summary.communication_success_count += 1;
    println!(
        "stage=set_center returned_position={} returned_angle_x10={}",
        center_return,
        position_to_angle_x10(center_return)
    );
    Timer::after(Duration::from_millis(SERVO_TEST_SETTLE_MS)).await;

    println!("stage=get_pos_after_center ...");
    let centered_position =
        krs.get_pos(SERVO_TEST_ID)
            .map_err(|source| ServoTestError::GetPosFailed {
                stage: "get_pos_after_center",
                source,
            })?;
    summary.final_position = centered_position;
    summary.final_angle_x10 = position_to_angle_x10(centered_position);
    summary.communication_success_count += 1;
    println!(
        "stage=get_pos_after_center position={} angle_x10={}",
        centered_position, summary.final_angle_x10
    );

    loop {
        if let Err(error) = run_small_sweep(krs, &mut summary).await {
            println!("stage=error_return_center target={}", center_position);
            if let Err(source) = krs.set_pos(SERVO_TEST_ID, center_position) {
                return Err(ServoTestError::ReturnCenterFailed {
                    stage: "error_return_center",
                    expected: center_position,
                    source,
                });
            }
            return Err(error);
        }

        if SERVO_TEST_OPEN_CLOSE_FULL_RANGE {
            if let Err(error) = run_full_range_sweep(krs, &mut summary).await {
                println!("stage=error_return_center target={}", center_position);
                if let Err(source) = krs.set_pos(SERVO_TEST_ID, center_position) {
                    return Err(ServoTestError::ReturnCenterFailed {
                        stage: "error_return_center",
                        expected: center_position,
                        source,
                    });
                }
                return Err(error);
            }

            println!("stage=full_range_return_center target={}", center_position);
            let returned_position =
                krs.set_pos(SERVO_TEST_ID, center_position)
                    .map_err(|source| ServoTestError::ReturnCenterFailed {
                        stage: "full_range_return_center",
                        expected: center_position,
                        source,
                    })?;
            summary.move_count += 1;
            summary.communication_success_count += 1;
            summary.final_position = returned_position;
            summary.final_angle_x10 = position_to_angle_x10(returned_position);
            println!(
                "stage=full_range_return_center returned_position={} returned_angle_x10={}",
                returned_position, summary.final_angle_x10
            );
        }

        if !SERVO_TEST_REPEAT {
            break;
        }

        print_pass_summary(&summary);
        println!("stage=repeat_wait hold_ms={}", SERVO_TEST_HOLD_MS);
        Timer::after(Duration::from_millis(SERVO_TEST_HOLD_MS)).await;
    }

    println!("stage=final_center target={}", center_position);
    let final_position = krs
        .set_pos(SERVO_TEST_ID, center_position)
        .map_err(|source| ServoTestError::ReturnCenterFailed {
            stage: "final_center",
            expected: center_position,
            source,
        })?;
    summary.move_count += 1;
    summary.communication_success_count += 1;
    summary.final_position = final_position;
    summary.final_angle_x10 = position_to_angle_x10(final_position);
    println!(
        "stage=final_center returned_position={} returned_angle_x10={}",
        final_position, summary.final_angle_x10
    );

    if SERVO_TEST_FREE_AT_END {
        println!("stage=set_free ...");
        let free_position =
            krs.set_free(SERVO_TEST_ID)
                .map_err(|source| ServoTestError::FreeFailed {
                    stage: "set_free",
                    source,
                })?;
        summary.communication_success_count += 1;
        summary.final_position = free_position;
        summary.final_angle_x10 = position_to_angle_x10(free_position);
        println!(
            "stage=set_free returned_position={} returned_angle_x10={}",
            free_position, summary.final_angle_x10
        );
    }

    Ok(summary)
}

async fn run_small_sweep(
    krs: &mut IcsDevice<'_>,
    summary: &mut ServoTestSummary,
) -> Result<(), ServoTestError> {
    println!("stage=small_sweep ...");
    let positions = [
        summary.min_test_position,
        summary.center_position,
        summary.max_test_position,
        summary.center_position,
    ];

    for target in positions {
        set_and_sample_small_move(krs, summary, "small_sweep", target).await?;
    }

    Ok(())
}

async fn run_full_range_sweep(
    krs: &mut IcsDevice<'_>,
    summary: &mut ServoTestSummary,
) -> Result<(), ServoTestError> {
    println!("stage=full_range WARNING: moving to configured main valve closed/open angles");
    let closed_position = checked_test_position(
        angle_x10_to_position(MAIN_VALVE_CLOSED_ANGLE_X10),
        "full_range_closed",
    )?;
    let open_position = checked_test_position(
        angle_x10_to_position(MAIN_VALVE_OPEN_ANGLE_X10),
        "full_range_open",
    )?;

    for target in [closed_position, open_position, closed_position] {
        set_and_sample_small_move(krs, summary, "full_range", target).await?;
    }

    Ok(())
}

async fn set_and_sample_small_move(
    krs: &mut IcsDevice<'_>,
    summary: &mut ServoTestSummary,
    stage: &'static str,
    target: u16,
) -> Result<(), ServoTestError> {
    checked_test_position(target, stage)?;
    println!(
        "stage={} set_pos target={} target_angle_x10={}",
        stage,
        target,
        position_to_angle_x10(target)
    );
    let returned_position =
        krs.set_pos(SERVO_TEST_ID, target)
            .map_err(|source| ServoTestError::SmallMoveFailed {
                stage,
                expected: target,
                source,
            })?;
    summary.move_count += 1;
    summary.communication_success_count += 1;
    println!(
        "stage={} set_pos_return returned_position={} returned_angle_x10={}",
        stage,
        returned_position,
        position_to_angle_x10(returned_position)
    );

    Timer::after(Duration::from_millis(SERVO_TEST_SETTLE_MS)).await;

    let sampled_position = krs
        .get_pos(SERVO_TEST_ID)
        .map_err(|source| ServoTestError::GetPosFailed { stage, source })?;
    summary.communication_success_count += 1;
    summary.final_position = sampled_position;
    summary.final_angle_x10 = position_to_angle_x10(sampled_position);
    println!(
        "stage={} get_pos position={} angle_x10={}",
        stage, sampled_position, summary.final_angle_x10
    );

    Timer::after(Duration::from_millis(SERVO_TEST_HOLD_MS)).await;

    Ok(())
}

fn checked_test_position(position: u16, stage: &'static str) -> Result<u16, ServoTestError> {
    if (SERVO_MIN_POS..=SERVO_MAX_POS).contains(&position) {
        Ok(position)
    } else {
        Err(ServoTestError::InvalidTestPosition { stage, position })
    }
}

fn print_start_banner() {
    println!("========== SERVO TEST START ==========");
    println!("UART: UART1 baud=115200 data=8 parity=even stop=1");
    println!("PINS: tx=GPIO16 rx=GPIO14 en=GPIO27");
    println!("SERVO_TEST_ID={}", SERVO_TEST_ID);
    println!("SERVO_TEST_STEP_POS={}", SERVO_TEST_STEP_POS);
    println!("SERVO_TEST_HOLD_MS={}", SERVO_TEST_HOLD_MS);
    println!("SERVO_TEST_SETTLE_MS={}", SERVO_TEST_SETTLE_MS);
    println!("SERVO_TEST_FREE_AT_END={}", SERVO_TEST_FREE_AT_END);
    println!("SERVO_TEST_REPEAT={}", SERVO_TEST_REPEAT);
    println!(
        "SERVO_TEST_OPEN_CLOSE_FULL_RANGE={}",
        SERVO_TEST_OPEN_CLOSE_FULL_RANGE
    );
}

fn print_pass_summary(summary: &ServoTestSummary) {
    println!("========== SERVO TEST RESULT ==========");
    println!("RESULT: PASS");
    println!("detected_id: {}", summary.detected_id);
    println!("initial_position: {}", summary.initial_position);
    println!("initial_angle_x10: {}", summary.initial_angle_x10);
    println!("final_position: {}", summary.final_position);
    println!("final_angle_x10: {}", summary.final_angle_x10);
    println!("move_count: {}", summary.move_count);
    println!(
        "communication_success_count: {}",
        summary.communication_success_count
    );
    println!("center_position: {}", summary.center_position);
    println!("min_test_position: {}", summary.min_test_position);
    println!("max_test_position: {}", summary.max_test_position);
    println!("free_at_end: {}", summary.free_at_end);
}

fn print_fail_summary(error: &ServoTestError) {
    println!("========== SERVO TEST RESULT ==========");
    println!("RESULT: FAIL");
    match error {
        ServoTestError::GetIdFailed { stage, source } => {
            println!("error: GetIdFailed stage={} source={:?}", stage, source);
        }
        ServoTestError::GetPosFailed { stage, source } => {
            println!("error: GetPosFailed stage={} source={:?}", stage, source);
        }
        ServoTestError::SetCenterFailed {
            stage,
            expected,
            source,
        } => {
            println!(
                "error: SetCenterFailed stage={} expected={} source={:?}",
                stage, expected, source
            );
        }
        ServoTestError::SmallMoveFailed {
            stage,
            expected,
            source,
        } => {
            println!(
                "error: SmallMoveFailed stage={} expected={} source={:?}",
                stage, expected, source
            );
        }
        ServoTestError::ReturnCenterFailed {
            stage,
            expected,
            source,
        } => {
            println!(
                "error: ReturnCenterFailed stage={} expected={} source={:?}",
                stage, expected, source
            );
        }
        ServoTestError::FreeFailed { stage, source } => {
            println!("error: FreeFailed stage={} source={:?}", stage, source);
        }
        ServoTestError::InvalidTestPosition { stage, position } => {
            println!(
                "error: InvalidTestPosition stage={} position={}",
                stage, position
            );
        }
    }
}

async fn pass_idle_loop() -> ! {
    loop {
        println!("SERVO TEST PASS: idle");
        Timer::after(Duration::from_secs(60)).await;
    }
}

async fn fault_backoff_loop() -> ! {
    loop {
        println!("SERVO TEST FAIL: fault backoff");
        Timer::after(Duration::from_secs(1)).await;
    }
}
