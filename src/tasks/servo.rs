use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};

use crate::{
    ANGLE_COMMAND_SIGNAL, CURRENT_POSITION, FAULT_FLAGS, SERVO_COMM_ERROR, SERVO_COMM_ERROR_LIMIT,
    SERVO_CONTROL_MODE, SERVO_FAULTS, SERVO_MODE_COMMAND, SERVO_POLL_INTERVAL_MS,
    angle_x10_to_position, krs_servo::IcsDevice, set_fault_flags,
};

#[embassy_executor::task]
pub async fn servo_task(mut krs: IcsDevice<'static>) {
    let mut communication_error_count = 0u8;

    loop {
        if SERVO_CONTROL_MODE.load(Ordering::Acquire) != SERVO_MODE_COMMAND {
            Timer::after(Duration::from_millis(SERVO_POLL_INTERVAL_MS)).await;
            continue;
        }

        let (result, retry_angle) = match select(
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
                (krs.set_pos(0, target_position), Some(angle_x10))
            }
            Either::Second(()) => (krs.get_pos(0), None),
        };

        match result {
            Ok(current_position) => {
                CURRENT_POSITION.store(current_position, Ordering::Release);
                communication_error_count = 0;
            }
            Err(_) => {
                communication_error_count = communication_error_count.saturating_add(1);
                if communication_error_count > SERVO_COMM_ERROR_LIMIT {
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
