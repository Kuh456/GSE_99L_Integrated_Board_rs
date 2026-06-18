#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::future::pending;

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Input, InputConfig, Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    ram,
    system::Stack,
    timer::timg::TimerGroup,
    twai::{self, BaudRate, TwaiMode, filter::SingleStandardFilter},
    uart::{Config as UartConfig, DataBits, Parity, StopBits, Uart},
};
#[cfg(feature = "can-debug-log")]
use esp_println::println;
use esp_rtos::embassy::Executor;
#[cfg(feature = "can-debug-log")]
use gse_integrated_board::tasks::can_debug::can_debug_log_task;
use gse_integrated_board::{
    krs_servo::IcsDevice,
    tasks::{
        can_communication::can_manager_task,
        espnow::{espnow_receive_task, initialize_espnow},
        servo::servo_task,
        status_led::status_led_task,
        supervisor::{InputGpioPins, supervisor_task},
    },
};
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    #[cfg(feature = "can-debug-log")]
    println!("boot can-debug-log enabled");
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    static APP_CORE_STACK: StaticCell<Stack<8192>> = StaticCell::new();
    let app_core_stack = APP_CORE_STACK.init_with(Stack::new);

    let dump = Output::new(peripherals.GPIO17, Level::Low, OutputConfig::default());
    let ignition = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());
    let o2 = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
    let fill = Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default());
    let separate = Output::new(peripherals.GPIO33, Level::Low, OutputConfig::default());
    let _spare_solenoid = Output::new(peripherals.GPIO32, Level::Low, OutputConfig::default());

    let can_tx = Output::new(peripherals.GPIO18, Level::Low, OutputConfig::default());
    let can_rx = Input::new(peripherals.GPIO4, InputConfig::default());
    let servo_tx = Output::new(peripherals.GPIO16, Level::Low, OutputConfig::default());
    let servo_rx = Input::new(peripherals.GPIO14, InputConfig::default());
    let servo_enable = Output::new(peripherals.GPIO27, Level::Low, OutputConfig::default());

    let solenoid_power_check_pin = Input::new(peripherals.GPIO23, InputConfig::default());
    let relay_12v_check_pin = Input::new(peripherals.GPIO22, InputConfig::default());
    let igniter_power_check_pin = Input::new(peripherals.GPIO34, InputConfig::default());
    let relay_24v_check_pin = Input::new(peripherals.GPIO35, InputConfig::default());

    let led_can_com_state = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());
    let led_servo_com_state = Output::new(peripherals.GPIO21, Level::Low, OutputConfig::default());

    let uart_config = UartConfig::default()
        .with_baudrate(115_200)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::Even)
        .with_stop_bits(StopBits::_1);
    let uart = Uart::new(peripherals.UART1, uart_config)
        .unwrap()
        .with_rx(servo_rx)
        .with_tx(servo_tx);
    let servo = IcsDevice::new(uart, servo_enable);
    let (wifi_controller, esp_now) = initialize_espnow(peripherals.WIFI);

    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        sw_int.software_interrupt1,
        app_core_stack,
        move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                const TWAI_BAUDRATE: twai::BaudRate = BaudRate::B125K;
                let mut can_config = twai::TwaiConfiguration::new(
                    peripherals.TWAI0,
                    can_rx,
                    can_tx,
                    TWAI_BAUDRATE,
                    TwaiMode::Normal,
                )
                .into_async();
                can_config.set_filter(
                    const {
                        SingleStandardFilter::new(
                            b"0xxxxxxxxxx",
                            b"x",
                            [b"xxxxxxxx", b"xxxxxxxx"],
                        )
                    },
                );
                let can = can_config.start();
                spawner.spawn(can_manager_task(can).unwrap());
                #[cfg(feature = "can-debug-log")]
                {
                    println!("boot spawning can_debug_log_task on second core");
                    spawner.spawn(can_debug_log_task().unwrap());
                    println!("boot spawned can_debug_log_task on second core");
                }
            });
        },
    );

    spawner.spawn(espnow_receive_task(wifi_controller, esp_now).unwrap());
    spawner.spawn(servo_task(servo).unwrap());
    spawner.spawn(status_led_task(led_servo_com_state, led_can_com_state).unwrap());
    spawner.spawn(
        supervisor_task(
            ignition,
            dump,
            fill,
            separate,
            o2,
            InputGpioPins::new(
                solenoid_power_check_pin,
                relay_12v_check_pin,
                igniter_power_check_pin,
                relay_24v_check_pin,
            ),
        )
        .unwrap(),
    );

    pending::<()>().await;
    unreachable!()
}
