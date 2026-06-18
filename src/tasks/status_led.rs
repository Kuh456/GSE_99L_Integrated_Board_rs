use core::sync::atomic::Ordering;

use embassy_time::{Duration, Ticker};
use esp_hal::gpio::{Level, Output};
#[cfg(feature = "can-debug-log")]
use esp_println::println;

use crate::{
    CAN_COMM_ACTIVE, CAN_FAULTS, CAN_HEALTH, FAULT_FLAGS, SERVO_COMM_ACTIVE, can::health::CanHealth,
};

const STATUS_LED_BLINK_INTERVAL_MS: u64 = 500;

#[embassy_executor::task]
pub async fn status_led_task(mut servo_led: Output<'static>, mut can_led: Output<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(STATUS_LED_BLINK_INTERVAL_MS));
    let mut blink_on = false;

    loop {
        ticker.next().await;
        blink_on = !blink_on;

        servo_led.set_level(active_or_blink(servo_comm_active(), blink_on));
        let can_status = CanLedStatus::load();
        let can_led_level = active_or_blink(can_status.active, blink_on);
        can_led.set_level(can_led_level);

        #[cfg(feature = "can-debug-log")]
        println!(
            "led_dbg blink_on={} can_active={} CAN_COMM_ACTIVE={} CAN_HEALTH={} FAULT_FLAGS=0x{:02X} can_led_level={}",
            blink_on,
            can_status.active,
            can_status.comm_active,
            can_status.health,
            can_status.fault_flags,
            level_name(can_led_level),
        );
    }
}

fn active_or_blink(active: bool, blink_on: bool) -> Level {
    if active || blink_on {
        Level::High
    } else {
        Level::Low
    }
}

fn servo_comm_active() -> bool {
    SERVO_COMM_ACTIVE.load(Ordering::Acquire)
}

struct CanLedStatus {
    active: bool,
    #[cfg(feature = "can-debug-log")]
    comm_active: bool,
    #[cfg(feature = "can-debug-log")]
    health: u8,
    #[cfg(feature = "can-debug-log")]
    fault_flags: u8,
}

impl CanLedStatus {
    fn load() -> Self {
        let comm_active = CAN_COMM_ACTIVE.load(Ordering::Acquire);
        let health = CAN_HEALTH.load(Ordering::Acquire);
        let fault_flags = FAULT_FLAGS.load(Ordering::Acquire);
        let active =
            comm_active && health == CanHealth::Active as u8 && fault_flags & CAN_FAULTS == 0;

        Self {
            active,
            #[cfg(feature = "can-debug-log")]
            comm_active,
            #[cfg(feature = "can-debug-log")]
            health,
            #[cfg(feature = "can-debug-log")]
            fault_flags,
        }
    }
}

#[cfg(feature = "can-debug-log")]
fn level_name(level: Level) -> &'static str {
    match level {
        Level::High => "High",
        Level::Low => "Low",
    }
}
