use core::sync::atomic::Ordering;

use embassy_time::{Duration, Ticker};
use esp_println::println;

use crate::{
    CAN_COMM_ACTIVE, CAN_HEALTH, CAN_REC, CAN_RX_ERROR_COUNT, CAN_TEC, CAN_TX_ERROR_COUNT,
    FAULT_FLAGS,
};

const CAN_DEBUG_LOG_INTERVAL_MS: u64 = 1000;

#[embassy_executor::task]
pub async fn can_debug_log_task() {
    println!("can_dbg task_start");
    print_can_debug_status();

    let mut ticker = Ticker::every(Duration::from_millis(CAN_DEBUG_LOG_INTERVAL_MS));

    loop {
        ticker.next().await;
        print_can_debug_status();
    }
}

fn print_can_debug_status() {
    println!(
        "can_dbg CAN_COMM_ACTIVE={} CAN_HEALTH={} CAN_TEC={} CAN_REC={} FAULT_FLAGS=0x{:02X} CAN_TX_ERROR_COUNT={} CAN_RX_ERROR_COUNT={}",
        CAN_COMM_ACTIVE.load(Ordering::Acquire),
        CAN_HEALTH.load(Ordering::Acquire),
        CAN_TEC.load(Ordering::Relaxed),
        CAN_REC.load(Ordering::Relaxed),
        FAULT_FLAGS.load(Ordering::Acquire),
        CAN_TX_ERROR_COUNT.load(Ordering::Relaxed),
        CAN_RX_ERROR_COUNT.load(Ordering::Relaxed),
    );
}
