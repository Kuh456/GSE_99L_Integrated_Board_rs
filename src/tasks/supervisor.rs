use core::sync::atomic::Ordering;

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::gpio::{Input, Output};

use crate::{
    ANGLE_COMMAND_SIGNAL, CAN_BUS_OFF, CAN_FAULTS, CAN_HEALTH, CONTROL_UPDATE_SIGNAL, CanHealth,
    ControlDecision, ControlIntent, FAULT_FLAGS, FIRING_TOTAL_TIMEOUT_MS, IGNITION_WAIT_MS,
    IN_IGNITER_POWER_PRESENT, IN_RELAY_12V_ON, IN_RELAY_24V_ON, IN_SOLENOID_POWER_PRESENT,
    INPUT_CAN_LINK_ACTIVE, INPUT_DUMP_REQUEST, INPUT_FILL_REQUEST, INPUT_FIRE_REQUEST, INPUT_FLAGS,
    INPUT_GPIO_STATUS, INPUT_O2_TEST_REQUEST, INPUT_OPERATOR_ACTION_MASK, INPUT_RESET_ACK_REQUEST,
    INPUT_SEPARATE_REQUEST, INPUT_VALVE_OPEN_REQUEST, INPUT_VALVE_SET_REQUEST,
    MAIN_VALVE_CLOSED_ANGLE_X10, MAIN_VALVE_OPEN_ANGLE_X10, MAIN_VALVE_OPEN_DELAY_MS, OUT_DUMP,
    OUT_FILL, OUT_IGNITER, OUT_O2, OUT_SEPARATE, OUTPUT_STATUS, RECOVERABLE_CAN_FAULTS,
    SERVO_CONTROL_MODE, SERVO_MODE_COMMAND, SERVO_MODE_HOLD, SERVO_TARGET_ANGLE_X10,
    SUPERVISOR_INTERVAL_MS, SequencePhase, ServoAction, clear_fault_flags_for_reset,
    replace_operator_input_flags, resolve_control, sequence_phase, set_sequence_phase,
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
    let mut firing_started_at: Option<Instant> = None;
    let mut abort_before_firing = false;
    let mut previous_servo_action = ServoAction::Hold;

    loop {
        match select(ticker.next(), CONTROL_UPDATE_SIGNAL.wait()).await {
            Either::First(()) | Either::Second(()) => {}
        }

        let now = Instant::now();
        INPUT_GPIO_STATUS.store(input_gpio_pins.status(), Ordering::Release);
        let inputs = INPUT_FLAGS.load(Ordering::Acquire);
        let mut faults = FAULT_FLAGS.load(Ordering::Acquire);

        if has_active_can_input_fault(faults, inputs) {
            replace_operator_input_flags(0);
            enter_abort(sequence_phase(), &mut abort_before_firing);
            firing_started_at = None;
            apply_decision(
                force_safe_outputs(),
                &mut ignition,
                &mut dump,
                &mut fill,
                &mut separate,
                &mut o2,
                &mut previous_servo_action,
            );
            continue;
        }

        if handle_reset_ack(inputs) {
            faults = FAULT_FLAGS.load(Ordering::Acquire);
            return_to_idle_after_reset_ack(
                faults,
                &mut firing_started_at,
                &mut abort_before_firing,
            );
        }

        if faults != 0 && sequence_phase() == SequencePhase::Firing {
            enter_abort(SequencePhase::Firing, &mut abort_before_firing);
            firing_started_at = None;
            apply_decision(
                force_safe_outputs(),
                &mut ignition,
                &mut dump,
                &mut fill,
                &mut separate,
                &mut o2,
                &mut previous_servo_action,
            );
            continue;
        }

        let intent = match sequence_phase() {
            SequencePhase::Idle => handle_idle_phase(
                inputs,
                faults,
                now,
                &mut firing_started_at,
                &mut abort_before_firing,
            ),
            SequencePhase::Firing => handle_firing_phase(inputs, now, &mut firing_started_at),
            SequencePhase::Timeout => {
                handle_timeout_phase(inputs, &mut firing_started_at, &mut abort_before_firing)
            }
            SequencePhase::Abort => handle_abort_phase(
                inputs,
                faults,
                &mut firing_started_at,
                &mut abort_before_firing,
            ),
        };

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
    }
}

fn handle_idle_phase(
    inputs: u32,
    faults: u8,
    now: Instant,
    firing_started_at: &mut Option<Instant>,
    abort_before_firing: &mut bool,
) -> ControlIntent {
    *firing_started_at = None;

    let start_permission =
        resolve_control(SequencePhase::Idle, faults, inputs, ControlIntent::safe())
            .allow_new_ignition;

    if start_permission && inputs & INPUT_FIRE_REQUEST != 0 {
        *abort_before_firing = false;
        *firing_started_at = Some(now);
        set_sequence_phase(SequencePhase::Firing);
        return apply_firing_sequence_outputs(inputs, Duration::from_millis(0));
    }

    apply_manual_outputs(inputs, false)
}

fn handle_firing_phase(
    inputs: u32,
    now: Instant,
    firing_started_at: &mut Option<Instant>,
) -> ControlIntent {
    let start = firing_started_at.get_or_insert(now);
    let elapsed = now.duration_since(*start);

    if elapsed >= Duration::from_millis(FIRING_TOTAL_TIMEOUT_MS) {
        *firing_started_at = None;
        set_sequence_phase(SequencePhase::Timeout);
        return apply_manual_outputs(inputs, true);
    }

    apply_firing_sequence_outputs(inputs, elapsed)
}

fn handle_timeout_phase(
    inputs: u32,
    firing_started_at: &mut Option<Instant>,
    abort_before_firing: &mut bool,
) -> ControlIntent {
    *abort_before_firing = false;
    *firing_started_at = None;
    apply_manual_outputs(inputs, true)
}

fn handle_abort_phase(
    inputs: u32,
    faults: u8,
    firing_started_at: &mut Option<Instant>,
    abort_before_firing: &mut bool,
) -> ControlIntent {
    *firing_started_at = None;

    if *abort_before_firing
        && faults == 0
        && inputs & INPUT_OPERATOR_ACTION_MASK == 0
        && inputs & INPUT_CAN_LINK_ACTIVE != 0
    {
        *abort_before_firing = false;
        set_sequence_phase(SequencePhase::Idle);
        return apply_manual_outputs(inputs, false);
    }

    apply_manual_outputs(inputs, true)
}

fn apply_manual_outputs(inputs: u32, fire_means_main_valve_open: bool) -> ControlIntent {
    let mut intent = ControlIntent {
        ignition_on: false,
        dump_on: inputs & INPUT_DUMP_REQUEST != 0,
        fill_on: inputs & INPUT_FILL_REQUEST != 0,
        separate_on: inputs & INPUT_SEPARATE_REQUEST != 0,
        o2_on: inputs & INPUT_O2_TEST_REQUEST != 0,
        servo_target_angle_x10: None,
    };

    let main_valve_open_requested = inputs & INPUT_VALVE_OPEN_REQUEST != 0
        || (fire_means_main_valve_open && inputs & INPUT_FIRE_REQUEST != 0);

    if inputs & INPUT_VALVE_SET_REQUEST != 0 {
        intent.servo_target_angle_x10 = Some(MAIN_VALVE_CLOSED_ANGLE_X10);
    } else if main_valve_open_requested {
        intent.servo_target_angle_x10 = Some(MAIN_VALVE_OPEN_ANGLE_X10);
    }

    intent
}

fn apply_firing_sequence_outputs(inputs: u32, elapsed: Duration) -> ControlIntent {
    let mut intent = ControlIntent::safe();
    intent.dump_on = inputs & INPUT_DUMP_REQUEST != 0;

    if elapsed >= Duration::from_millis(IGNITION_WAIT_MS + MAIN_VALVE_OPEN_DELAY_MS) {
        intent.servo_target_angle_x10 = Some(MAIN_VALVE_OPEN_ANGLE_X10);
    } else if elapsed >= Duration::from_millis(IGNITION_WAIT_MS) {
        intent.o2_on = true;
        intent.ignition_on = true;
    } else {
        intent.o2_on = true;
    }

    intent
}

fn force_safe_outputs() -> ControlDecision {
    ControlDecision {
        ignition_on: false,
        dump_on: false,
        fill_on: false,
        separate_on: false,
        o2_on: false,
        servo_action: ServoAction::MoveTo(MAIN_VALVE_CLOSED_ANGLE_X10),
        allow_new_ignition: false,
    }
}

fn has_active_can_input_fault(faults: u8, inputs: u32) -> bool {
    faults & CAN_FAULTS != 0 && inputs & INPUT_CAN_LINK_ACTIVE == 0
}

fn handle_reset_ack(inputs: u32) -> bool {
    if inputs & INPUT_RESET_ACK_REQUEST == 0
        || inputs & INPUT_OPERATOR_ACTION_MASK != 0
        || inputs & INPUT_CAN_LINK_ACTIVE == 0
    {
        return false;
    }

    let mut clearable_faults = RECOVERABLE_CAN_FAULTS;
    if CAN_HEALTH.load(Ordering::Acquire) == CanHealth::Active as u8 {
        clearable_faults |= CAN_BUS_OFF;
    }

    let faults_to_clear = FAULT_FLAGS.load(Ordering::Acquire) & clearable_faults;
    if faults_to_clear != 0 {
        clear_fault_flags_for_reset(faults_to_clear);
    }
    true
}

fn return_to_idle_after_reset_ack(
    faults: u8,
    firing_started_at: &mut Option<Instant>,
    abort_before_firing: &mut bool,
) {
    if faults == 0 && sequence_phase() == SequencePhase::Abort {
        *abort_before_firing = false;
        *firing_started_at = None;
        set_sequence_phase(SequencePhase::Idle);
    }
}

#[cfg(test)]
fn clear_operator_inputs(inputs: u32) -> u32 {
    inputs & !crate::INPUT_COMMAND_MASK
}

fn enter_abort(current_phase: SequencePhase, abort_before_firing: &mut bool) {
    if current_phase != SequencePhase::Abort {
        *abort_before_firing = current_phase == SequencePhase::Idle;
        set_sequence_phase(SequencePhase::Abort);
    }
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

#[cfg(test)]
mod tests {
    use core::sync::atomic::Ordering;

    use embassy_time::{Duration, Instant};

    use super::*;
    use crate::{
        CAN_BUS_OFF, CAN_PEER_LOST, CAN_TX_TIMEOUT, INPUT_CAN_LINK_ACTIVE, INPUT_RESET_ACK_REQUEST,
        MAIN_VALVE_CLOSED_ANGLE_X10, MAIN_VALVE_OPEN_ANGLE_X10, SEQUENCE_PHASE, SequencePhase,
        ServoAction, sequence_phase,
    };

    fn set_phase(phase: SequencePhase) {
        SEQUENCE_PHASE.store(phase as u8, Ordering::Release);
    }

    #[test]
    fn idle_fire_starts_firing_without_manual_valve_open() {
        set_phase(SequencePhase::Idle);
        let mut started_at = None;
        let mut abort_before_firing = true;
        let now = Instant::from_millis(100);

        let intent = handle_idle_phase(
            INPUT_CAN_LINK_ACTIVE | INPUT_FIRE_REQUEST,
            0,
            now,
            &mut started_at,
            &mut abort_before_firing,
        );

        assert_eq!(sequence_phase(), SequencePhase::Firing);
        assert_eq!(started_at, Some(now));
        assert!(!abort_before_firing);
        assert_eq!(intent.servo_target_angle_x10, None);
        assert!(intent.o2_on);
    }

    #[test]
    fn idle_fire_is_blocked_while_can_fault_history_remains() {
        set_phase(SequencePhase::Idle);
        let mut started_at = None;
        let mut abort_before_firing = false;

        let intent = handle_idle_phase(
            INPUT_CAN_LINK_ACTIVE | INPUT_FIRE_REQUEST,
            CAN_PEER_LOST,
            Instant::from_millis(100),
            &mut started_at,
            &mut abort_before_firing,
        );

        assert_eq!(sequence_phase(), SequencePhase::Idle);
        assert_eq!(started_at, None);
        assert_eq!(intent.servo_target_angle_x10, None);
        assert!(!intent.o2_on);
    }

    #[test]
    fn timeout_fire_is_manual_main_valve_open_without_firing_restart() {
        set_phase(SequencePhase::Timeout);
        let mut started_at = Some(Instant::from_millis(1));
        let mut abort_before_firing = true;

        let intent = handle_timeout_phase(
            INPUT_CAN_LINK_ACTIVE | INPUT_FIRE_REQUEST,
            &mut started_at,
            &mut abort_before_firing,
        );

        assert_eq!(sequence_phase(), SequencePhase::Timeout);
        assert_eq!(started_at, None);
        assert!(!abort_before_firing);
        assert_eq!(
            intent.servo_target_angle_x10,
            Some(MAIN_VALVE_OPEN_ANGLE_X10)
        );
    }

    #[test]
    fn abort_fire_is_manual_main_valve_open_without_firing_restart() {
        set_phase(SequencePhase::Abort);
        let mut started_at = Some(Instant::from_millis(1));
        let mut abort_before_firing = false;

        let intent = handle_abort_phase(
            INPUT_CAN_LINK_ACTIVE | INPUT_FIRE_REQUEST,
            CAN_PEER_LOST,
            &mut started_at,
            &mut abort_before_firing,
        );
        let decision = resolve_control(
            SequencePhase::Abort,
            CAN_PEER_LOST,
            INPUT_CAN_LINK_ACTIVE | INPUT_FIRE_REQUEST,
            intent,
        );

        assert_eq!(sequence_phase(), SequencePhase::Abort);
        assert_eq!(started_at, None);
        assert_eq!(
            decision.servo_action,
            ServoAction::MoveTo(MAIN_VALVE_OPEN_ANGLE_X10)
        );
        assert!(!decision.ignition_on);
    }

    #[test]
    fn firing_fire_does_not_apply_manual_main_valve_open() {
        let intent = apply_firing_sequence_outputs(
            INPUT_FIRE_REQUEST,
            Duration::from_millis(IGNITION_WAIT_MS + MAIN_VALVE_OPEN_DELAY_MS - 1),
        );

        assert_eq!(intent.servo_target_angle_x10, None);
        assert!(intent.ignition_on);
    }

    #[test]
    fn firing_dump_is_the_only_manual_output_override() {
        let intent = apply_firing_sequence_outputs(
            INPUT_DUMP_REQUEST
                | INPUT_FILL_REQUEST
                | INPUT_SEPARATE_REQUEST
                | INPUT_O2_TEST_REQUEST,
            Duration::from_millis(0),
        );

        assert!(intent.dump_on);
        assert!(!intent.fill_on);
        assert!(!intent.separate_on);
        assert!(intent.o2_on);
    }

    #[test]
    fn active_can_fault_enters_abort_from_manual_phases() {
        set_phase(SequencePhase::Idle);
        let mut abort_before_firing = false;
        enter_abort(sequence_phase(), &mut abort_before_firing);
        assert_eq!(sequence_phase(), SequencePhase::Abort);
        assert!(abort_before_firing);

        set_phase(SequencePhase::Timeout);
        abort_before_firing = true;
        enter_abort(sequence_phase(), &mut abort_before_firing);
        assert_eq!(sequence_phase(), SequencePhase::Abort);
        assert!(!abort_before_firing);

        abort_before_firing = true;
        enter_abort(sequence_phase(), &mut abort_before_firing);
        assert_eq!(sequence_phase(), SequencePhase::Abort);
        assert!(abort_before_firing);
    }

    #[test]
    fn active_can_fault_clears_operator_inputs_and_forces_safe_outputs() {
        let inputs = INPUT_CAN_LINK_ACTIVE | INPUT_FIRE_REQUEST | INPUT_DUMP_REQUEST;
        let cleared = clear_operator_inputs(inputs);
        let decision = force_safe_outputs();

        assert!(has_active_can_input_fault(
            CAN_PEER_LOST,
            INPUT_FIRE_REQUEST
        ));
        assert!(!has_active_can_input_fault(
            CAN_PEER_LOST,
            INPUT_CAN_LINK_ACTIVE
        ));
        assert_eq!(cleared, INPUT_CAN_LINK_ACTIVE);
        assert!(!decision.ignition_on);
        assert!(!decision.dump_on);
        assert_eq!(
            decision.servo_action,
            ServoAction::MoveTo(MAIN_VALVE_CLOSED_ANGLE_X10)
        );
    }

    #[test]
    fn reset_ack_clears_recoverable_can_faults_and_returns_abort_to_idle() {
        set_phase(SequencePhase::Abort);
        FAULT_FLAGS.store(
            CAN_PEER_LOST | CAN_TX_TIMEOUT | CAN_BUS_OFF,
            Ordering::Release,
        );
        CAN_HEALTH.store(CanHealth::Active as u8, Ordering::Release);
        let mut started_at = Some(Instant::from_millis(1));
        let mut abort_before_firing = true;

        assert!(handle_reset_ack(
            INPUT_CAN_LINK_ACTIVE | INPUT_RESET_ACK_REQUEST
        ));
        let faults = FAULT_FLAGS.load(Ordering::Acquire);
        return_to_idle_after_reset_ack(faults, &mut started_at, &mut abort_before_firing);

        assert_eq!(faults & (CAN_PEER_LOST | CAN_TX_TIMEOUT | CAN_BUS_OFF), 0);
        assert_eq!(sequence_phase(), SequencePhase::Idle);
        assert_eq!(started_at, None);
        assert!(!abort_before_firing);
    }

    #[test]
    fn reset_ack_with_other_commands_does_not_clear_faults() {
        FAULT_FLAGS.store(CAN_PEER_LOST, Ordering::Release);
        CAN_HEALTH.store(CanHealth::Active as u8, Ordering::Release);

        assert!(!handle_reset_ack(
            INPUT_CAN_LINK_ACTIVE | INPUT_RESET_ACK_REQUEST | INPUT_FIRE_REQUEST
        ));
        assert_eq!(
            FAULT_FLAGS.load(Ordering::Acquire) & CAN_PEER_LOST,
            CAN_PEER_LOST
        );
    }

    #[test]
    fn recovered_can_button_can_drive_manual_outputs_while_abort_latched() {
        let inputs = INPUT_CAN_LINK_ACTIVE | INPUT_FIRE_REQUEST | INPUT_FILL_REQUEST;
        let intent = apply_manual_outputs(inputs, true);
        let decision = resolve_control(SequencePhase::Abort, CAN_PEER_LOST, inputs, intent);

        assert_eq!(
            decision.servo_action,
            ServoAction::MoveTo(MAIN_VALVE_OPEN_ANGLE_X10)
        );
        assert!(decision.fill_on);
        assert!(!decision.ignition_on);
    }
}
