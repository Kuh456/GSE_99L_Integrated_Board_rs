use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::gpio::{Input, Output};

use crate::{
    ANGLE_COMMAND_SIGNAL, CAN_FAULTS, CONTROL_UPDATE_SIGNAL, ControlDecision, ControlIntent,
    FAULT_FLAGS, FIRING_TOTAL_TIMEOUT_MS, IGNITION_WAIT_MS, IN_IGNITER_POWER_PRESENT,
    IN_RELAY_12V_ON, IN_RELAY_24V_ON, IN_SOLENOID_POWER_PRESENT, INPUT_CAN_LINK_ACTIVE,
    INPUT_COMMAND_MASK, INPUT_DUMP_REQUEST, INPUT_FILL_REQUEST, INPUT_FIRE_REQUEST, INPUT_FLAGS,
    INPUT_GPIO_STATUS, INPUT_O2_TEST_REQUEST, INPUT_SEPARATE_REQUEST, INPUT_VALVE_OPEN_REQUEST,
    INPUT_VALVE_SET_REQUEST, MAIN_VALVE_CLOSED_ANGLE_X10, MAIN_VALVE_OPEN_ANGLE_X10,
    MAIN_VALVE_OPEN_DELAY_MS, OUT_DUMP, OUT_FILL, OUT_IGNITER, OUT_O2, OUT_SEPARATE, OUTPUT_STATUS,
    SERVO_CONTROL_MODE, SERVO_MODE_COMMAND, SERVO_MODE_HOLD, SERVO_TARGET_ANGLE_X10,
    SUPERVISOR_INTERVAL_MS, SequencePhase, ServoAction, resolve_control, sequence_phase,
    set_sequence_phase,
};

pub struct InputGpioPins {
    solenoid_power_check: Input<'static>,
    relay_12v_check: Input<'static>,
    igniter_power_check: Input<'static>,
    relay_24v_check: Input<'static>,
}

impl InputGpioPins {
    pub fn new(
        solenoid_power_check: Input<'static>,
        relay_12v_check: Input<'static>,
        igniter_power_check: Input<'static>,
        relay_24v_check: Input<'static>,
    ) -> Self {
        Self {
            solenoid_power_check,
            relay_12v_check,
            igniter_power_check,
            relay_24v_check,
        }
    }

    fn status(&self) -> u8 {
        let mut status = 0;
        if self.solenoid_power_check.is_high() {
            status |= IN_SOLENOID_POWER_PRESENT;
        }
        if self.relay_12v_check.is_high() {
            status |= IN_RELAY_12V_ON;
        }
        if self.igniter_power_check.is_high() {
            status |= IN_IGNITER_POWER_PRESENT;
        }
        if self.relay_24v_check.is_high() {
            status |= IN_RELAY_24V_ON;
        }
        status
    }
}

#[embassy_executor::task]
pub async fn supervisor_task(
    mut ignition: Output<'static>,
    mut dump: Output<'static>,
    mut fill: Output<'static>,
    mut separate: Output<'static>,
    mut o2: Output<'static>,
    input_gpio_pins: InputGpioPins,
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
        INPUT_GPIO_STATUS.store(input_gpio_pins.status(), Ordering::Release);
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
        output_status |= OUT_DUMP;
    }
    if decision.ignition_on {
        output_status |= OUT_IGNITER;
    }
    if decision.fill_on {
        output_status |= OUT_FILL;
    }
    if decision.separate_on {
        output_status |= OUT_SEPARATE;
    }
    if decision.o2_on {
        output_status |= OUT_O2;
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
