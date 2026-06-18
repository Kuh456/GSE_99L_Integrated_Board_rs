use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer};

use crate::{
    ANGLE_COMMAND_SIGNAL, CURRENT_POSITION, SERVO_COMM_ACTIVE, SERVO_CONTROL_MODE,
    SERVO_MODE_COMMAND, SERVO_POLL_INTERVAL_MS, angle_x10_to_position, krs_servo::IcsDevice,
};

#[embassy_executor::task]
pub async fn servo_task(mut krs: IcsDevice<'static>) {
    loop {
        let result = match select(
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
                krs.set_pos(0, target_position).await
            }
            Either::Second(()) => krs.get_pos(0).await,
        };

        match result {
            Ok(current_position) => {
                CURRENT_POSITION.store(current_position, Ordering::Release);
                SERVO_COMM_ACTIVE.store(true, Ordering::Release);
            }
            Err(_) => {
                SERVO_COMM_ACTIVE.store(false, Ordering::Release);
            }
        }
    }
}
