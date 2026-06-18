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
    MAIN_VALVE_CLOSED_ANGLE_X10, MAIN_VALVE_OPEN_ANGLE_X10, angle_x10_to_position,
    krs_servo::{IcsDevice, IcsError},
    position_to_angle_x10,
};

esp_bootloader_esp_idf::esp_app_desc!();

const SERVO_TEST_ID: u8 = 0;
const SERVO_TEST_WAIT_MS: u64 = 3000;

#[derive(Debug)]
struct ServoTestSummary {
    detected_id: u8,
    initial_position: u16,
    initial_angle_x10: i16,
    closed_position: u16,
    open_position: u16,
    final_position: u16,
    final_angle_x10: i16,
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
    MoveClosedFailed {
        stage: &'static str,
        target_position: u16,
        source: IcsError,
    },
    MoveOpenFailed {
        stage: &'static str,
        target_position: u16,
        source: IcsError,
    },
    ReturnClosedFailed {
        stage: &'static str,
        target_position: u16,
        source: IcsError,
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

    match run_servo_test(&mut servo).await {
        Ok(summary) => {
            print_pass_summary(&summary);
            pass_stop_loop().await
        }
        Err(error) => {
            print_fail_summary(&error);
            fail_stop_loop().await
        }
    }
}

async fn run_servo_test(krs: &mut IcsDevice<'_>) -> Result<ServoTestSummary, ServoTestError> {
    let closed_position = angle_x10_to_position(MAIN_VALVE_CLOSED_ANGLE_X10);
    let open_position = angle_x10_to_position(MAIN_VALVE_OPEN_ANGLE_X10);

    println!("stage=get_id ...");
    let detected_id = krs.get_id().await.map_err(|source| ServoTestError::GetIdFailed {
        stage: "get_id",
        source,
    })?;
    println!("detected_id={}", detected_id);

    println!("stage=get_pos ...");
    let initial_position =
        krs.get_pos(SERVO_TEST_ID)
            .await
            .map_err(|source| ServoTestError::GetPosFailed {
                stage: "get_pos_initial",
                source,
            })?;
    let initial_angle_x10 = position_to_angle_x10(initial_position);
    println!("initial_position={}", initial_position);
    println!("initial_angle_x10={}", initial_angle_x10);

    println!("stage=move_closed ...");
    println!("closed_angle_x10={}", MAIN_VALVE_CLOSED_ANGLE_X10);
    println!("closed_position={}", closed_position);
    let closed_return = krs
        .set_pos(SERVO_TEST_ID, closed_position)
        .await
        .map_err(|source| ServoTestError::MoveClosedFailed {
            stage: "move_closed",
            target_position: closed_position,
            source,
        })?;
    println!(
        "stage=move_closed returned_position={} returned_angle_x10={}",
        closed_return,
        position_to_angle_x10(closed_return)
    );
    println!("wait {} ms", SERVO_TEST_WAIT_MS);
    Timer::after(Duration::from_millis(SERVO_TEST_WAIT_MS)).await;

    println!("stage=move_open ...");
    println!("open_angle_x10={}", MAIN_VALVE_OPEN_ANGLE_X10);
    println!("open_position={}", open_position);
    match krs.set_pos(SERVO_TEST_ID, open_position).await {
        Ok(open_return) => {
            println!(
                "stage=move_open returned_position={} returned_angle_x10={}",
                open_return,
                position_to_angle_x10(open_return)
            );
        }
        Err(source) => {
            println!("stage=move_open_failed_return_closed ...");
            if let Err(return_source) = krs.set_pos(SERVO_TEST_ID, closed_position).await {
                println!(
                    "stage=move_open_failed_return_closed_failed source={:?}",
                    return_source
                );
            }
            return Err(ServoTestError::MoveOpenFailed {
                stage: "move_open",
                target_position: open_position,
                source,
            });
        }
    }
    println!("wait {} ms", SERVO_TEST_WAIT_MS);
    Timer::after(Duration::from_millis(SERVO_TEST_WAIT_MS)).await;

    println!("stage=return_closed ...");
    let return_closed = krs
        .set_pos(SERVO_TEST_ID, closed_position)
        .await
        .map_err(|source| ServoTestError::ReturnClosedFailed {
            stage: "return_closed",
            target_position: closed_position,
            source,
        })?;
    println!(
        "stage=return_closed returned_position={} returned_angle_x10={}",
        return_closed,
        position_to_angle_x10(return_closed)
    );
    Timer::after(Duration::from_millis(SERVO_TEST_WAIT_MS)).await;

    println!("stage=get_pos_final ...");
    let final_position =
        krs.get_pos(SERVO_TEST_ID)
            .await
            .map_err(|source| ServoTestError::GetPosFailed {
                stage: "get_pos_final",
                source,
            })?;
    let final_angle_x10 = position_to_angle_x10(final_position);
    println!("final_position={}", final_position);
    println!("final_angle_x10={}", final_angle_x10);

    Ok(ServoTestSummary {
        detected_id,
        initial_position,
        initial_angle_x10,
        closed_position,
        open_position,
        final_position,
        final_angle_x10,
    })
}

fn print_start_banner() {
    println!("========== SERVO TEST START ==========");
    println!("UART: UART1 baud=115200 data=8 parity=even stop=1");
    println!("PINS: tx=GPIO16 rx=GPIO14 en=GPIO27");
    println!("SERVO_TEST_ID={}", SERVO_TEST_ID);
}

fn print_pass_summary(summary: &ServoTestSummary) {
    println!("========== SERVO TEST RESULT ==========");
    println!("RESULT: PASS");
    println!("detected_id: {}", summary.detected_id);
    println!("initial_position: {}", summary.initial_position);
    println!("initial_angle_x10: {}", summary.initial_angle_x10);
    println!("closed_position: {}", summary.closed_position);
    println!("open_position: {}", summary.open_position);
    println!("final_position: {}", summary.final_position);
    println!("final_angle_x10: {}", summary.final_angle_x10);
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
        ServoTestError::MoveClosedFailed {
            stage,
            target_position,
            source,
        } => {
            println!(
                "error: MoveClosedFailed stage={} target_position={} source={:?}",
                stage, target_position, source
            );
        }
        ServoTestError::MoveOpenFailed {
            stage,
            target_position,
            source,
        } => {
            println!(
                "error: MoveOpenFailed stage={} target_position={} source={:?}",
                stage, target_position, source
            );
        }
        ServoTestError::ReturnClosedFailed {
            stage,
            target_position,
            source,
        } => {
            println!(
                "error: ReturnClosedFailed stage={} target_position={} source={:?}",
                stage, target_position, source
            );
        }
    }
}

async fn pass_stop_loop() -> ! {
    loop {
        println!("SERVO TEST PASS: stopped");
        Timer::after(Duration::from_secs(60)).await;
    }
}

async fn fail_stop_loop() -> ! {
    loop {
        println!("SERVO TEST FAIL: stopped");
        Timer::after(Duration::from_secs(1)).await;
    }
}
