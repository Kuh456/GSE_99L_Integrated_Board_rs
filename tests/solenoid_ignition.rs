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
    gpio::{Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

const ENABLE_IGNITION_OUTPUT_TEST: bool = false;
const SOLENOID_ON_MS: u64 = 1000;
const SOLENOID_OFF_MS: u64 = 1000;
const STARTUP_SAFE_WAIT_MS: u64 = 2000;
const IGNITER_PULSE_MS: u64 = 100;

struct TestOutputs {
    ignition: Output<'static>,
    dump: Output<'static>,
    fill: Output<'static>,
    separate: Output<'static>,
    o2: Output<'static>,
    spare_solenoid: Output<'static>,
}

impl TestOutputs {
    fn set_all_low(&mut self) {
        self.ignition.set_low();
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
        dump: Output::new(peripherals.GPIO17, Level::Low, OutputConfig::default()),
        ignition: Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default()),
        o2: Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default()),
        fill: Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default()),
        separate: Output::new(peripherals.GPIO33, Level::Low, OutputConfig::default()),
        spare_solenoid: Output::new(peripherals.GPIO32, Level::Low, OutputConfig::default()),
    };

    println!("=== solenoid / ignition hardware test ===");
    println!("PINS: FILL=GPIO25 DUMP=GPIO17 SEPARATE=GPIO33 O2_TEST=GPIO26");
    println!("PINS: SPARE_SOLENOID=GPIO32 IGNITER=GPIO13");
    println!("TODO: 24V relay output GPIO is not defined in existing board initialization");

    all_outputs_off(&mut outputs);
    println!("all outputs OFF");
    short_delay_ms(STARTUP_SAFE_WAIT_MS).await;

    run_solenoid_tests(&mut outputs).await;
    ignition_output_test(&mut outputs).await;

    all_outputs_off(&mut outputs);
    println!("all outputs OFF");
    println!("test completed");

    pass_stop_loop(outputs).await
}

async fn run_solenoid_tests(outputs: &mut TestOutputs) {
    all_outputs_off(outputs);
    test_solenoid("FILL", &mut outputs.fill).await;

    all_outputs_off(outputs);
    test_solenoid("DUMP", &mut outputs.dump).await;

    all_outputs_off(outputs);
    test_solenoid("SEPARATE", &mut outputs.separate).await;

    all_outputs_off(outputs);
    test_solenoid("O2_TEST", &mut outputs.o2).await;

    all_outputs_off(outputs);
    test_solenoid("SPARE_SOLENOID", &mut outputs.spare_solenoid).await;
    all_outputs_off(outputs);
    test_solenoid("FIRE", &mut outputs.ignition).await;
}

fn all_outputs_off(outputs: &mut TestOutputs) {
    outputs.set_all_low();
}

async fn test_solenoid(name: &str, pin: &mut Output<'static>) {
    println!("testing {}", name);
    println!("{} ON", name);
    pin.set_high();
    short_delay_ms(SOLENOID_ON_MS).await;

    pin.set_low();
    println!("{} OFF", name);
    short_delay_ms(SOLENOID_OFF_MS).await;
}

async fn ignition_output_test(outputs: &mut TestOutputs) {
    outputs.ignition.set_low();

    if !ENABLE_IGNITION_OUTPUT_TEST {
        println!("ignition output test skipped because ENABLE_IGNITION_OUTPUT_TEST=false");
        println!("IGNITER kept OFF");
        println!("24V relay output skipped: output GPIO is not defined");
        return;
    }

    unsafe_ignition_output_test(outputs).await;
}

async fn unsafe_ignition_output_test(outputs: &mut TestOutputs) {
    println!("WARNING: ENABLE_IGNITION_OUTPUT_TEST=true");
    println!("WARNING: issuing a short IGNITER pulse");
    println!("TODO: 24V relay output GPIO is not defined; relay ON step is skipped");

    outputs.ignition.set_low();
    short_delay_ms(100).await;

    println!("IGNITER ON");
    outputs.ignition.set_high();
    short_delay_ms(IGNITER_PULSE_MS).await;

    outputs.ignition.set_low();
    println!("IGNITER OFF");
    println!("TODO: 24V relay output GPIO is not defined; relay OFF step is skipped");
}

async fn short_delay_ms(ms: u64) {
    Timer::after(Duration::from_millis(ms)).await;
}

async fn pass_stop_loop(mut outputs: TestOutputs) -> ! {
    loop {
        all_outputs_off(&mut outputs);
        Timer::after(Duration::from_secs(60)).await;
    }
}
