use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, select};
#[cfg(feature = "servo-debug-log")]
use embassy_time::Instant;
use embassy_time::{Duration, Timer};
#[cfg(feature = "servo-debug-log")]
use esp_println::println;

use crate::{
    ANGLE_COMMAND_SIGNAL, CURRENT_POSITION, SERVO_COMM_ACTIVE, SERVO_COMM_ERROR,
    SERVO_COMM_ERROR_COUNT, SERVO_COMM_ERROR_LIMIT, SERVO_CONTROL_MODE, SERVO_GET_POS_INTERVAL_MS,
    SERVO_MODE_COMMAND, angle_x10_to_position, clear_fault_flags_on_recovery,
    krs_servo::{IcsDevice, IcsError},
    set_fault_flags,
};

#[derive(Clone, Copy)]
enum ServoOperation {
    SetPosition,
    GetPosition,
}

#[cfg(feature = "servo-debug-log")]
impl ServoOperation {
    const fn name(self) -> &'static str {
        match self {
            Self::SetPosition => "set_pos",
            Self::GetPosition => "get_pos",
        }
    }
}

#[embassy_executor::task]
pub async fn servo_task(mut krs: IcsDevice<'static>) {
    loop {
        let (result, operation) = match select(
            ANGLE_COMMAND_SIGNAL.wait(),
            Timer::after(Duration::from_millis(SERVO_GET_POS_INTERVAL_MS)),
        )
        .await
        {
            Either::First(angle_x10) => {
                if SERVO_CONTROL_MODE.load(Ordering::Acquire) != SERVO_MODE_COMMAND {
                    continue;
                }

                let target_position = angle_x10_to_position(angle_x10);
                (
                    krs.set_pos(0, target_position).await,
                    ServoOperation::SetPosition,
                )
            }
            Either::Second(()) => (krs.get_pos(0).await, ServoOperation::GetPosition),
        };

        match result {
            Ok(current_position) => {
                CURRENT_POSITION.store(current_position, Ordering::Release);
                let previous_error_count = SERVO_COMM_ERROR_COUNT.swap(0, Ordering::AcqRel);
                if servo_fault_active(previous_error_count) {
                    clear_fault_flags_on_recovery(SERVO_COMM_ERROR);
                }
                SERVO_COMM_ACTIVE.store(true, Ordering::Release);
                log_servo_comm_recovered(operation, previous_error_count);
            }
            Err(error) => {
                let previous_error_count = SERVO_COMM_ERROR_COUNT
                    .fetch_update(Ordering::AcqRel, Ordering::Acquire, |count| {
                        Some(count.saturating_add(1))
                    })
                    .unwrap_or_else(|count| count);
                let error_count = previous_error_count.saturating_add(1);
                if servo_fault_raised(error_count) {
                    set_fault_flags(SERVO_COMM_ERROR);
                }
                SERVO_COMM_ACTIVE.store(false, Ordering::Release);
                log_servo_comm_error(operation, &error, error_count);
            }
        }
    }
}

fn servo_fault_active(error_count: u8) -> bool {
    error_count >= SERVO_COMM_ERROR_LIMIT
}

fn servo_fault_raised(error_count: u8) -> bool {
    error_count == SERVO_COMM_ERROR_LIMIT
}

#[cfg(feature = "servo-debug-log")]
fn log_servo_comm_error(operation: ServoOperation, error: &IcsError, error_count: u8) {
    let event = if error_count == SERVO_COMM_ERROR_LIMIT {
        "fault_raised"
    } else {
        "comm_error"
    };
    println!(
        "servo_dbg event={} operation={} at_ms={} consecutive_error_count={} fault_active={} error={:?}",
        event,
        operation.name(),
        Instant::now().as_millis(),
        error_count,
        error_count >= SERVO_COMM_ERROR_LIMIT,
        error,
    );
}

#[cfg(not(feature = "servo-debug-log"))]
fn log_servo_comm_error(_operation: ServoOperation, _error: &IcsError, _error_count: u8) {}

#[cfg(feature = "servo-debug-log")]
fn log_servo_comm_recovered(operation: ServoOperation, previous_error_count: u8) {
    if previous_error_count == 0 {
        return;
    }
    println!(
        "servo_dbg event=recovered operation={} at_ms={} previous_consecutive_error_count={}",
        operation.name(),
        Instant::now().as_millis(),
        previous_error_count,
    );
}

#[cfg(not(feature = "servo-debug-log"))]
fn log_servo_comm_recovered(_operation: ServoOperation, _previous_error_count: u8) {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn servo_fault_becomes_active_at_tenth_consecutive_error() {
        assert!(!servo_fault_active(SERVO_COMM_ERROR_LIMIT - 1));
        assert!(!servo_fault_raised(SERVO_COMM_ERROR_LIMIT - 1));
        assert!(servo_fault_active(SERVO_COMM_ERROR_LIMIT));
        assert!(servo_fault_raised(SERVO_COMM_ERROR_LIMIT));
        assert!(!servo_fault_raised(SERVO_COMM_ERROR_LIMIT + 1));
    }
}
