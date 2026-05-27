use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::gpio::Output;

use crate::{
    ANGLE_COMMAND_SIGNAL, CAN_FAULTS, CONTROL_UPDATE_SIGNAL, ControlDecision, ControlIntent,
    FAULT_FLAGS, FIRING_TOTAL_TIMEOUT_MS, IGNITION_WAIT_MS, INPUT_CAN_LINK_ACTIVE,
    INPUT_COMMAND_MASK, INPUT_DUMP_REQUEST, INPUT_FILL_REQUEST, INPUT_FIRE_REQUEST, INPUT_FLAGS,
    INPUT_O2_TEST_REQUEST, INPUT_SEPARATE_REQUEST, INPUT_VALVE_OPEN_REQUEST,
    INPUT_VALVE_SET_REQUEST, MAIN_VALVE_CLOSED_ANGLE_X10, MAIN_VALVE_OPEN_ANGLE_X10,
    MAIN_VALVE_OPEN_DELAY_MS, OUTPUT_DUMP_ON, OUTPUT_FILL_ON, OUTPUT_IGNITION_ON, OUTPUT_O2_ON,
    OUTPUT_SEPARATE_ON, OUTPUT_STATUS, SERVO_CONTROL_MODE, SERVO_MODE_COMMAND, SERVO_MODE_HOLD,
    SERVO_TARGET_ANGLE_X10, SUPERVISOR_INTERVAL_MS, SequencePhase, ServoAction, resolve_control,
    sequence_phase, set_sequence_phase,
};

#[embassy_executor::task]
pub async fn supervisor_task(
    mut ignition: Output<'static>,
    mut dump: Output<'static>,
    mut fill: Output<'static>,
    mut separate: Output<'static>,
    mut o2: Output<'static>,
) {
    let mut ticker = Ticker::every(Duration::from_millis(SUPERVISOR_INTERVAL_MS));
    let mut previous_inputs = 0u32;
    let mut ignition_armed = false;
    let mut firing_started_at: Option<Instant> = None;
    let mut abort_before_firing = false;
    let mut previous_phase = sequence_phase();
    let mut previous_servo_action = ServoAction::Hold;

    loop {
        match select(ticker.next(), CONTROL_UPDATE_SIGNAL.wait()).await {
            Either::First(()) | Either::Second(()) => {}
        }

        let now = Instant::now();
        let inputs = INPUT_FLAGS.load(Ordering::Acquire);
        let faults = FAULT_FLAGS.load(Ordering::Acquire);
        let fire_requested = inputs & INPUT_FIRE_REQUEST != 0;
        let fire_rising = fire_requested && previous_inputs & INPUT_FIRE_REQUEST == 0;
        let can_available = inputs & INPUT_CAN_LINK_ACTIVE != 0 && faults & CAN_FAULTS == 0;
        let sequence_inhibited = !can_available || faults != 0;

        let active_phase = sequence_phase();
        if active_phase == SequencePhase::Abort && previous_phase != SequencePhase::Abort {
            abort_before_firing = previous_phase == SequencePhase::Idle;
        }

        if sequence_inhibited && active_phase == SequencePhase::Firing {
            abort_before_firing = false;
            set_sequence_phase(SequencePhase::Abort);
            firing_started_at = None;
            ignition_armed = false;
        }

        let mut intent = ControlIntent::safe();
        let phase = sequence_phase();
        match phase {
            SequencePhase::Idle => {
                firing_started_at = None;

                let start_permission =
                    resolve_control(SequencePhase::Idle, faults, inputs, ControlIntent::safe())
                        .allow_new_ignition;
                if start_permission && !fire_requested {
                    ignition_armed = true;
                } else if !start_permission {
                    ignition_armed = false;
                }

                if start_permission && ignition_armed && fire_rising {
                    ignition_armed = false;
                    abort_before_firing = false;
                    firing_started_at = Some(now);
                    set_sequence_phase(SequencePhase::Firing);
                    intent.dump_on = inputs & INPUT_DUMP_REQUEST != 0;
                    intent.o2_on = true;
                } else {
                    intent = idle_intent(inputs);
                }
            }
            SequencePhase::Firing => {
                let start = firing_started_at.get_or_insert(now);
                let elapsed = now.duration_since(*start);
                intent.dump_on = inputs & INPUT_DUMP_REQUEST != 0;

                if elapsed >= Duration::from_millis(FIRING_TOTAL_TIMEOUT_MS) {
                    firing_started_at = None;
                    set_sequence_phase(SequencePhase::Timeout);
                    intent = idle_intent(inputs);
                } else if elapsed
                    >= Duration::from_millis(IGNITION_WAIT_MS + MAIN_VALVE_OPEN_DELAY_MS)
                {
                    intent.o2_on = false;
                    intent.servo_target_angle_x10 = Some(MAIN_VALVE_OPEN_ANGLE_X10);
                } else if elapsed >= Duration::from_millis(IGNITION_WAIT_MS) {
                    intent.o2_on = true;
                    intent.ignition_on = true;
                } else {
                    intent.o2_on = true;
                }
            }
            SequencePhase::Timeout => {
                abort_before_firing = false;
                firing_started_at = None;
                ignition_armed = false;
                intent = idle_intent(inputs);
            }
            SequencePhase::Abort => {
                firing_started_at = None;
                ignition_armed = false;

                if abort_before_firing
                    && faults == 0
                    && inputs & INPUT_COMMAND_MASK == 0
                    && can_available
                {
                    abort_before_firing = false;
                    set_sequence_phase(SequencePhase::Idle);
                }
            }
        }

        let decision = resolve_control(
            sequence_phase(),
            FAULT_FLAGS.load(Ordering::Acquire),
            INPUT_FLAGS.load(Ordering::Acquire),
            intent,
        );
        apply_decision(
            decision,
            &mut ignition,
            &mut dump,
            &mut fill,
            &mut separate,
            &mut o2,
            &mut previous_servo_action,
        );
        previous_inputs = inputs;
        previous_phase = sequence_phase();
    }
}

fn idle_intent(inputs: u32) -> ControlIntent {
    let mut intent = ControlIntent {
        ignition_on: false,
        dump_on: inputs & INPUT_DUMP_REQUEST != 0,
        fill_on: inputs & INPUT_FILL_REQUEST != 0,
        separate_on: inputs & INPUT_SEPARATE_REQUEST != 0,
        o2_on: inputs & INPUT_O2_TEST_REQUEST != 0,
        servo_target_angle_x10: None,
    };

    if inputs & INPUT_VALVE_SET_REQUEST != 0 {
        intent.servo_target_angle_x10 = Some(MAIN_VALVE_CLOSED_ANGLE_X10);
    } else if inputs & INPUT_VALVE_OPEN_REQUEST != 0 {
        intent.servo_target_angle_x10 = Some(MAIN_VALVE_OPEN_ANGLE_X10);
    }

    intent
}

fn apply_decision(
    decision: ControlDecision,
    ignition: &mut Output<'static>,
    dump: &mut Output<'static>,
    fill: &mut Output<'static>,
    separate: &mut Output<'static>,
    o2: &mut Output<'static>,
    previous_servo_action: &mut ServoAction,
) {
    ignition.set_level(decision.ignition_on.into());
    dump.set_level(decision.dump_on.into());
    fill.set_level(decision.fill_on.into());
    separate.set_level(decision.separate_on.into());
    o2.set_level(decision.o2_on.into());

    let mut output_status = 0;
    if decision.dump_on {
        output_status |= OUTPUT_DUMP_ON;
    }
    if decision.ignition_on {
        output_status |= OUTPUT_IGNITION_ON;
    }
    if decision.fill_on {
        output_status |= OUTPUT_FILL_ON;
    }
    if decision.separate_on {
        output_status |= OUTPUT_SEPARATE_ON;
    }
    if decision.o2_on {
        output_status |= OUTPUT_O2_ON;
    }
    OUTPUT_STATUS.store(output_status, Ordering::Release);

    match decision.servo_action {
        ServoAction::Hold => {
            SERVO_CONTROL_MODE.store(SERVO_MODE_HOLD, Ordering::Release);
        }
        ServoAction::MoveTo(angle_x10) => {
            SERVO_TARGET_ANGLE_X10.store(angle_x10, Ordering::Release);
            SERVO_CONTROL_MODE.store(SERVO_MODE_COMMAND, Ordering::Release);
            if *previous_servo_action != decision.servo_action {
                ANGLE_COMMAND_SIGNAL.signal(angle_x10);
            }
        }
    }
    *previous_servo_action = decision.servo_action;
}
