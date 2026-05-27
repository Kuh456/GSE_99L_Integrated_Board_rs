use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};

use crate::{
    ANGLE_COMMAND_SIGNAL, CURRENT_POSITION, FAULT_FLAGS, SERVO_COMM_ERROR, SERVO_CONTROL_MODE,
    SERVO_ERROR_LIMIT, SERVO_FAULTS, SERVO_MODE_COMMAND, SERVO_POLL_INTERVAL_MS, SERVO_POS_ERROR,
    SERVO_POSITION_TOLERANCE, angle_x10_to_position, krs_servo::IcsDevice, set_fault_flags,
};

#[embassy_executor::task]
pub async fn servo_task(mut krs: IcsDevice<'static>) {
    let mut communication_error_count = 0u8;
    let mut position_error_count = 0u8;
    let mut expected_position: Option<u16> = None;

    loop {
        if SERVO_CONTROL_MODE.load(Ordering::Acquire) != SERVO_MODE_COMMAND {
            Timer::after(Duration::from_millis(SERVO_POLL_INTERVAL_MS)).await;
            continue;
        }

        let (result, requested_position, retry_angle) = match select(
            ANGLE_COMMAND_SIGNAL.wait(),
            Timer::after(Duration::from_millis(SERVO_POLL_INTERVAL_MS)),
        )
        .await
        {
            Either::First(angle_x10) => {
                if SERVO_CONTROL_MODE.load(Ordering::Acquire) != SERVO_MODE_COMMAND {
                    continue;
                }

                let target_position = angle_x10_to_position(angle_x10);
                (
                    krs.set_pos(0, target_position),
                    Some(target_position),
                    Some(angle_x10),
                )
            }
            Either::Second(()) => (krs.get_pos(0), expected_position, None),
        };

        match result {
            Ok(current_position) => {
                CURRENT_POSITION.store(current_position, Ordering::Release);
                communication_error_count = 0;

                if let Some(target_position) = requested_position {
                    expected_position = Some(target_position);
                    if current_position.abs_diff(target_position) > SERVO_POSITION_TOLERANCE {
                        position_error_count = position_error_count.saturating_add(1);
                        if position_error_count > SERVO_ERROR_LIMIT {
                            set_fault_flags(SERVO_POS_ERROR);
                        }
                    } else {
                        position_error_count = 0;
                    }
                }
            }
            Err(_) => {
                communication_error_count = communication_error_count.saturating_add(1);
                if communication_error_count > SERVO_ERROR_LIMIT {
                    set_fault_flags(SERVO_COMM_ERROR);
                } else if let Some(angle_x10) = retry_angle
                    && SERVO_CONTROL_MODE.load(Ordering::Acquire) == SERVO_MODE_COMMAND
                    && FAULT_FLAGS.load(Ordering::Acquire) & SERVO_FAULTS == 0
                {
                    ANGLE_COMMAND_SIGNAL.signal(angle_x10);
                }
            }
        }
    }
}
