use core::sync::atomic::Ordering;

use embassy_time::{Duration, Ticker};
use esp_hal::gpio::{Level, Output};

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
        can_led.set_level(active_or_blink(can_comm_active(), blink_on));
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

fn can_comm_active() -> bool {
    CAN_COMM_ACTIVE.load(Ordering::Acquire)
        && CAN_HEALTH.load(Ordering::Acquire) == CanHealth::Active as u8
        && FAULT_FLAGS.load(Ordering::Acquire) & CAN_FAULTS == 0
}
