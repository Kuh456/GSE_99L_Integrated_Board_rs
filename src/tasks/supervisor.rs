use core::sync::atomic::Ordering;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::gpio::{Input, Output};
#[cfg(feature = "can-debug-log")]
use esp_println::println;

use crate::{
    ANGLE_COMMAND_SIGNAL, CAN_FAULTS, CONTROL_UPDATE_SIGNAL, ControlDecision, ControlIntent,
    FAULT_FLAGS, FIRING_OPEN_LATCHED, FIRING_OPEN_SEND_COUNT, FIRING_OPEN_SEND_MAX,
    FIRING_TOTAL_TIMEOUT_MS, IGNITION_WAIT_MS, IN_IGNITER_POWER_PRESENT, IN_RELAY_12V_ON,
    IN_RELAY_24V_ON, IN_SOLENOID_POWER_PRESENT, INPUT_CAN_LINK_ACTIVE, INPUT_DUMP_REQUEST,
    INPUT_FILL_REQUEST, INPUT_FIRE_REQUEST, INPUT_FLAGS, INPUT_GPIO_STATUS, INPUT_O2_TEST_REQUEST,
    INPUT_OPERATOR_ACTION_MASK, INPUT_SEPARATE_REQUEST, INPUT_VALVE_OPEN_REQUEST,
    INPUT_VALVE_SET_REQUEST, MAIN_VALVE_CLOSED_ANGLE_X10, MAIN_VALVE_OPEN_ANGLE_X10,
    MAIN_VALVE_OPEN_DELAY_MS, O2_OFF_DELAY_AFTER_VALVE_OPEN_MS, OPEN_LATCH_RELEASE_DELAY_MS,
    OPEN_LATCH_RELEASE_PENDING, OUT_DUMP, OUT_FILL, OUT_IGNITER, OUT_O2, OUT_SEPARATE,
    OUTPUT_STATUS, RESET_ACK_EVENT_COUNTER, SERVO_COMM_ERROR, SERVO_CONTROL_MODE,
    SERVO_MODE_COMMAND, SERVO_MODE_HOLD, SERVO_POLL_INTERVAL_MS, SERVO_TARGET_ANGLE_X10,
    SUPERVISOR_INTERVAL_MS, SequencePhase, ServoAction, replace_operator_input_flags,
    resolve_control, sequence_phase,
    servo_control::{RequestedServoAction, ServoCommandState, ServoDispatch, ServoPhase},
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
    let mut firing_started_at: Option<Instant> = None;
    let mut abort_before_firing = false;
    let mut servo_abort_latched = false;
    let mut previous_reset_ack_event = RESET_ACK_EVENT_COUNTER.load(Ordering::Acquire);
    let mut previous_phase = sequence_phase();
    let mut servo_command_state = ServoCommandState::new();

    loop {
        match select(ticker.next(), CONTROL_UPDATE_SIGNAL.wait()).await {
            Either::First(()) | Either::Second(()) => {}
        }

        let now = Instant::now();
        let now_ms = now.as_millis();
        servo_command_state.update_latch_release(now_ms, OPEN_LATCH_RELEASE_DELAY_MS);
        INPUT_GPIO_STATUS.store(input_gpio_pins.status(), Ordering::Release);
        let inputs = INPUT_FLAGS.load(Ordering::Acquire);
        let mut faults = FAULT_FLAGS.load(Ordering::Acquire);
        let reset_ack_event = RESET_ACK_EVENT_COUNTER.load(Ordering::Acquire);
        let reset_ack_edge = reset_ack_event != previous_reset_ack_event;
        previous_reset_ack_event = reset_ack_event;

        if has_firing_fault(faults, sequence_phase()) && faults & SERVO_COMM_ERROR != 0 {
            servo_abort_latched = true;
        }

        if has_active_can_input_fault(faults, inputs) {
            replace_operator_input_flags(0);
            enter_abort(sequence_phase(), &mut abort_before_firing);
            observe_phase_transition(&mut previous_phase, &mut servo_command_state, now_ms);
            firing_started_at = None;
            apply_decision(
                force_safe_outputs(),
                &mut ignition,
                &mut dump,
                &mut fill,
                &mut separate,
                &mut o2,
                None,
            );
            publish_servo_diagnostics(&servo_command_state);
            continue;
        }

        if reset_ack_edge {
            faults = FAULT_FLAGS.load(Ordering::Acquire);
            return_to_idle_after_reset_ack(
                reset_ack_edge,
                inputs,
                faults,
                servo_abort_latched,
                &mut firing_started_at,
                &mut abort_before_firing,
            );
            observe_phase_transition(&mut previous_phase, &mut servo_command_state, now_ms);
        }

        faults = FAULT_FLAGS.load(Ordering::Acquire);
        return_to_idle_after_pre_firing_recovery(
            inputs,
            faults,
            &mut firing_started_at,
            &mut abort_before_firing,
        );
        observe_phase_transition(&mut previous_phase, &mut servo_command_state, now_ms);

        if has_firing_fault(faults, sequence_phase()) {
            enter_abort(SequencePhase::Firing, &mut abort_before_firing);
            observe_phase_transition(&mut previous_phase, &mut servo_command_state, now_ms);
            firing_started_at = None;
            apply_decision(
                force_safe_outputs(),
                &mut ignition,
                &mut dump,
                &mut fill,
                &mut separate,
                &mut o2,
                None,
            );
            publish_servo_diagnostics(&servo_command_state);
            continue;
        }

        let intent = match sequence_phase() {
            SequencePhase::Idle => handle_idle_phase(
                inputs,
                faults,
                now,
                &mut firing_started_at,
                &mut abort_before_firing,
                servo_command_state.can_start_firing(),
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
        observe_phase_transition(&mut previous_phase, &mut servo_command_state, now_ms);

        let mut decision = resolve_control(
            sequence_phase(),
            FAULT_FLAGS.load(Ordering::Acquire),
            INPUT_FLAGS.load(Ordering::Acquire),
            intent,
        );
        let firing_open_phase = sequence_phase() == SequencePhase::Firing
            && firing_started_at.is_some_and(|started| {
                now.duration_since(started)
                    >= Duration::from_millis(IGNITION_WAIT_MS + MAIN_VALVE_OPEN_DELAY_MS)
            });
        let dispatch = servo_command_state.schedule(
            sequence_phase().into(),
            firing_open_phase,
            decision.servo_action.into(),
            now_ms,
            SERVO_POLL_INTERVAL_MS,
            FIRING_OPEN_SEND_MAX,
            MAIN_VALVE_OPEN_ANGLE_X10,
            MAIN_VALVE_CLOSED_ANGLE_X10,
        );
        decision.servo_action = dispatch.action.into();
        log_servo_dispatch(dispatch, &servo_command_state, now_ms);
        apply_decision(
            decision,
            &mut ignition,
            &mut dump,
            &mut fill,
            &mut separate,
            &mut o2,
            dispatch.signal_angle_x10,
        );
        publish_servo_diagnostics(&servo_command_state);
    }
}

fn handle_idle_phase(
    inputs: u32,
    faults: u8,
    now: Instant,
    firing_started_at: &mut Option<Instant>,
    abort_before_firing: &mut bool,
    can_start_firing: bool,
) -> ControlIntent {
    *firing_started_at = None;

    let start_permission =
        resolve_control(SequencePhase::Idle, faults, inputs, ControlIntent::safe())
            .allow_new_ignition;

    if start_permission && can_start_firing && inputs & INPUT_FIRE_REQUEST != 0 {
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
    _faults: u8,
    firing_started_at: &mut Option<Instant>,
    _abort_before_firing: &mut bool,
) -> ControlIntent {
    *firing_started_at = None;
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

    let valve_open_at_ms = IGNITION_WAIT_MS + MAIN_VALVE_OPEN_DELAY_MS;
    let o2_off_at_ms = valve_open_at_ms + O2_OFF_DELAY_AFTER_VALVE_OPEN_MS;

    intent.o2_on = elapsed < Duration::from_millis(o2_off_at_ms);
    intent.ignition_on = elapsed >= Duration::from_millis(IGNITION_WAIT_MS)
        && elapsed < Duration::from_millis(valve_open_at_ms);

    if elapsed >= Duration::from_millis(valve_open_at_ms) {
        intent.servo_target_angle_x10 = Some(MAIN_VALVE_OPEN_ANGLE_X10);
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
        servo_action: ServoAction::Hold,
        allow_new_ignition: false,
    }
}

fn has_active_can_input_fault(faults: u8, inputs: u32) -> bool {
    faults & CAN_FAULTS != 0 && inputs & INPUT_CAN_LINK_ACTIVE == 0
}

fn has_firing_fault(faults: u8, phase: SequencePhase) -> bool {
    faults != 0 && phase == SequencePhase::Firing
}

fn return_to_idle_after_reset_ack(
    reset_ack_edge: bool,
    inputs: u32,
    faults: u8,
    servo_abort_latched: bool,
    firing_started_at: &mut Option<Instant>,
    abort_before_firing: &mut bool,
) {
    if reset_ack_allows_idle(
        reset_ack_edge,
        inputs,
        faults,
        servo_abort_latched,
        sequence_phase(),
    ) {
        *abort_before_firing = false;
        *firing_started_at = None;
        set_sequence_phase(SequencePhase::Idle);
    }
}

fn reset_ack_allows_idle(
    reset_ack_edge: bool,
    inputs: u32,
    faults: u8,
    servo_abort_latched: bool,
    phase: SequencePhase,
) -> bool {
    reset_ack_edge
        && faults == 0
        && !servo_abort_latched
        && phase == SequencePhase::Abort
        && inputs & INPUT_OPERATOR_ACTION_MASK == 0
        && inputs & INPUT_CAN_LINK_ACTIVE != 0
}

fn return_to_idle_after_pre_firing_recovery(
    inputs: u32,
    faults: u8,
    firing_started_at: &mut Option<Instant>,
    abort_before_firing: &mut bool,
) {
    if pre_firing_recovery_allows_idle(inputs, faults, *abort_before_firing, sequence_phase()) {
        *abort_before_firing = false;
        *firing_started_at = None;
        set_sequence_phase(SequencePhase::Idle);
    }
}

fn pre_firing_recovery_allows_idle(
    inputs: u32,
    faults: u8,
    abort_before_firing: bool,
    phase: SequencePhase,
) -> bool {
    abort_before_firing
        && faults == 0
        && phase == SequencePhase::Abort
        && inputs & INPUT_OPERATOR_ACTION_MASK == 0
        && inputs & INPUT_CAN_LINK_ACTIVE != 0
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
    angle_command: Option<i16>,
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
        }
    }
    if let Some(angle_x10) = angle_command {
        ANGLE_COMMAND_SIGNAL.signal(angle_x10);
    }
}

fn observe_phase_transition(
    previous_phase: &mut SequencePhase,
    servo_command_state: &mut ServoCommandState,
    now_ms: u64,
) {
    let current_phase = sequence_phase();
    if current_phase != *previous_phase {
        servo_command_state.on_phase_transition(current_phase.into(), now_ms);
        *previous_phase = current_phase;
    }
}

fn publish_servo_diagnostics(state: &ServoCommandState) {
    FIRING_OPEN_LATCHED.store(state.open_latched(), Ordering::Release);
    FIRING_OPEN_SEND_COUNT.store(state.firing_open_send_count(), Ordering::Release);
    OPEN_LATCH_RELEASE_PENDING.store(state.release_pending(), Ordering::Release);
}

#[cfg(feature = "can-debug-log")]
fn log_servo_dispatch(dispatch: ServoDispatch, state: &ServoCommandState, now_ms: u64) {
    if let Some(angle_x10) = dispatch.signal_angle_x10 {
        let kind = if angle_x10 == MAIN_VALVE_OPEN_ANGLE_X10 && state.open_latched() {
            "firing_open"
        } else if angle_x10 == MAIN_VALVE_CLOSED_ANGLE_X10 {
            "close"
        } else {
            "manual_open"
        };
        println!(
            "servo_cmd kind={} angle_x10={} open_count={} at_ms={}",
            kind,
            angle_x10,
            state.firing_open_send_count(),
            now_ms
        );
    }
}

#[cfg(not(feature = "can-debug-log"))]
fn log_servo_dispatch(_dispatch: ServoDispatch, _state: &ServoCommandState, _now_ms: u64) {}

impl From<SequencePhase> for ServoPhase {
    fn from(value: SequencePhase) -> Self {
        match value {
            SequencePhase::Idle => Self::Idle,
            SequencePhase::Firing => Self::Firing,
            SequencePhase::Timeout => Self::Timeout,
            SequencePhase::Abort => Self::Abort,
        }
    }
}

impl From<ServoAction> for RequestedServoAction {
    fn from(value: ServoAction) -> Self {
        match value {
            ServoAction::Hold => Self::Hold,
            ServoAction::MoveTo(angle_x10) => Self::MoveTo(angle_x10),
        }
    }
}

impl From<RequestedServoAction> for ServoAction {
    fn from(value: RequestedServoAction) -> Self {
        match value {
            RequestedServoAction::Hold => Self::Hold,
            RequestedServoAction::MoveTo(angle_x10) => Self::MoveTo(angle_x10),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn firing_outputs_at(elapsed_ms: u64) -> ControlIntent {
        apply_firing_sequence_outputs(0, Duration::from_millis(elapsed_ms))
    }

    #[test]
    fn o2_stays_on_until_two_seconds_after_valve_open_command() {
        let valve_open_at_ms = IGNITION_WAIT_MS + MAIN_VALVE_OPEN_DELAY_MS;

        assert!(firing_outputs_at(valve_open_at_ms - 1).o2_on);
        assert!(firing_outputs_at(valve_open_at_ms).o2_on);
        assert!(firing_outputs_at(valve_open_at_ms + O2_OFF_DELAY_AFTER_VALVE_OPEN_MS - 1).o2_on);
        assert!(!firing_outputs_at(valve_open_at_ms + O2_OFF_DELAY_AFTER_VALVE_OPEN_MS).o2_on);
    }

    #[test]
    fn valve_open_turns_ignition_off_without_turning_o2_off() {
        let valve_open_at_ms = IGNITION_WAIT_MS + MAIN_VALVE_OPEN_DELAY_MS;
        let before_open = firing_outputs_at(valve_open_at_ms - 1);
        let at_open = firing_outputs_at(valve_open_at_ms);

        assert!(before_open.ignition_on);
        assert_eq!(before_open.servo_target_angle_x10, None);
        assert!(!at_open.ignition_on);
        assert!(at_open.o2_on);
        assert_eq!(
            at_open.servo_target_angle_x10,
            Some(MAIN_VALVE_OPEN_ANGLE_X10)
        );
    }

    #[test]
    fn force_safe_outputs_turns_o2_off_immediately() {
        assert!(!force_safe_outputs().o2_on);
    }

    #[test]
    fn abort_forces_servo_hold_even_with_a_valve_request() {
        let intent = ControlIntent {
            servo_target_angle_x10: Some(MAIN_VALVE_OPEN_ANGLE_X10),
            ..ControlIntent::safe()
        };
        let decision = resolve_control(SequencePhase::Abort, 0, INPUT_CAN_LINK_ACTIVE, intent);

        assert_eq!(decision.servo_action, ServoAction::Hold);
    }

    #[test]
    fn servo_abort_latch_blocks_reset_ack_return_to_idle() {
        assert!(!reset_ack_allows_idle(
            true,
            INPUT_CAN_LINK_ACTIVE,
            0,
            true,
            SequencePhase::Abort,
        ));
        assert!(reset_ack_allows_idle(
            true,
            INPUT_CAN_LINK_ACTIVE,
            0,
            false,
            SequencePhase::Abort,
        ));
    }

    #[test]
    fn servo_fault_requests_abort_only_during_firing() {
        assert!(!has_firing_fault(SERVO_COMM_ERROR, SequencePhase::Idle));
        assert!(has_firing_fault(SERVO_COMM_ERROR, SequencePhase::Firing));
        assert!(!has_firing_fault(SERVO_COMM_ERROR, SequencePhase::Timeout));
        assert!(!has_firing_fault(SERVO_COMM_ERROR, SequencePhase::Abort));
    }

    #[test]
    fn pre_firing_abort_returns_to_idle_after_neutral_recovery() {
        assert!(pre_firing_recovery_allows_idle(
            INPUT_CAN_LINK_ACTIVE,
            0,
            true,
            SequencePhase::Abort,
        ));
    }

    #[test]
    fn pre_firing_abort_recovery_requires_no_faults_and_neutral_input() {
        assert!(!pre_firing_recovery_allows_idle(
            INPUT_CAN_LINK_ACTIVE,
            SERVO_COMM_ERROR,
            true,
            SequencePhase::Abort,
        ));
        assert!(!pre_firing_recovery_allows_idle(
            INPUT_CAN_LINK_ACTIVE | INPUT_FIRE_REQUEST,
            0,
            true,
            SequencePhase::Abort,
        ));
        assert!(!pre_firing_recovery_allows_idle(
            INPUT_CAN_LINK_ACTIVE,
            0,
            false,
            SequencePhase::Abort,
        ));
    }
}
