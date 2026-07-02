#![no_std]

pub mod can;
pub mod krs_servo;
mod servo_control;
pub mod tasks;

use core::sync::atomic::{AtomicBool, AtomicI16, AtomicU8, AtomicU16, AtomicU32, Ordering};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

pub use can::health::CanHealth;
pub use can::protocol::{
    CAN_ID_BUTTON_FROM_CTRL_PANEL, CAN_ID_INPUT_GPIO_STATUS, CAN_ID_INTERNAL_STATUS,
    CAN_ID_MAIN_VALVE_ANGLE_TO_CTRL_PANEL, CAN_ID_OUTPUT_GPIO_STATUS,
};

pub const COMMUNICATION_TIMEOUT_MS: u64 = 3000;
pub const CAN_STATUS_TX_INTERVAL_MS: u64 = 50;
pub const CAN_TX_TIMEOUT_MS: u64 = 10;
pub const SUPERVISOR_INTERVAL_MS: u64 = 10;

// Sequence timing.
pub const IGNITION_WAIT_MS: u64 = 10000;
pub const MAIN_VALVE_OPEN_DELAY_MS: u64 = 3200;
pub const O2_OFF_DELAY_AFTER_VALVE_OPEN_MS: u64 = 2000;
pub const IGNITION_SEQUENCE_TIMEOUT_MS: u64 = 10000;
pub const RUNNING_DURATION_MS: u64 = IGNITION_SEQUENCE_TIMEOUT_MS;
pub const FIRING_TOTAL_TIMEOUT_MS: u64 =
    IGNITION_WAIT_MS + MAIN_VALVE_OPEN_DELAY_MS + RUNNING_DURATION_MS;

// Valve and servo positions.
pub const MAIN_VALVE_OPEN_ANGLE_X10: i16 = -348;
pub const MAIN_VALVE_CLOSED_ANGLE_X10: i16 = 552;
pub const SERVO_MIN_POS: u16 = 3500;
pub const SERVO_CENTER_POS: u16 = 7500;
pub const SERVO_MAX_POS: u16 = 11500;
pub const SERVO_POS_PER_DEGREE: f32 = 29.62963;
pub const SERVO_POLL_INTERVAL_MS: u64 = 200;
pub const SERVO_GET_POS_INTERVAL_MS: u64 = 200;
pub const SERVO_COMM_ERROR_LIMIT: u8 = 10;
pub const FIRING_OPEN_SEND_MAX: u8 = 10;
pub const OPEN_LATCH_RELEASE_DELAY_MS: u64 = 10_000;

// Fault flags. CAN faults are latched until an explicit reset; the servo communication fault
// clears automatically after a successful servo transaction.
pub const CAN_PEER_LOST: u8 = 1 << 0;
pub const CAN_BUS_OFF: u8 = 1 << 1;
pub const SERVO_COMM_ERROR: u8 = 1 << 2;
// Reserved for an abnormal ignition watchdog fault; normal Firing completion is Timeout.
pub const IGNITION_TIMEOUT: u8 = 1 << 4;
pub const POWER_ERROR: u8 = 1 << 5;
pub const CAN_TX_TIMEOUT: u8 = 1 << 6;
pub const CAN_TX_FRAME_CREATE_FAILED: u8 = 1 << 7;
pub const CAN_FAULTS: u8 =
    CAN_PEER_LOST | CAN_BUS_OFF | CAN_TX_TIMEOUT | CAN_TX_FRAME_CREATE_FAILED;
pub const RECOVERABLE_CAN_FAULTS: u8 = CAN_PEER_LOST;
pub const SERVO_FAULTS: u8 = SERVO_COMM_ERROR;
pub const HARD_OUTPUT_INHIBIT_FAULTS: u8 =
    IGNITION_TIMEOUT | POWER_ERROR | CAN_TX_FRAME_CREATE_FAILED;

pub static FAULT_FLAGS: AtomicU8 = AtomicU8::new(0);

pub fn set_fault_flags(flags: u8) {
    let previous = FAULT_FLAGS.fetch_or(flags, Ordering::AcqRel);
    if previous & flags != flags {
        CONTROL_UPDATE_SIGNAL.signal(());
    }
}

/// Clears latched faults only when an explicit reset policy has approved it.
pub fn clear_fault_flags_for_reset(flags: u8) {
    FAULT_FLAGS.fetch_and(!flags, Ordering::AcqRel);
    CONTROL_UPDATE_SIGNAL.signal(());
}

/// Clears faults whose recovery policy is a successful retry.
pub(crate) fn clear_fault_flags_on_recovery(flags: u8) {
    let previous = FAULT_FLAGS.fetch_and(!flags, Ordering::AcqRel);
    if previous & flags != 0 {
        CONTROL_UPDATE_SIGNAL.signal(());
    }
}

// Input conditions and operator requests.
pub const INPUT_CAN_LINK_ACTIVE: u32 = 1 << 1;
pub const INPUT_FIRE_REQUEST: u32 = 1 << 8;
pub const INPUT_DUMP_REQUEST: u32 = 1 << 9;
pub const INPUT_FILL_REQUEST: u32 = 1 << 10;
pub const INPUT_SEPARATE_REQUEST: u32 = 1 << 11;
pub const INPUT_VALVE_SET_REQUEST: u32 = 1 << 12;
pub const INPUT_O2_TEST_REQUEST: u32 = 1 << 13;
pub const INPUT_VALVE_OPEN_REQUEST: u32 = 1 << 14;
pub const INPUT_RESET_ACK_REQUEST: u32 = 1 << 15;
pub const INPUT_OPERATOR_ACTION_MASK: u32 = INPUT_FIRE_REQUEST
    | INPUT_DUMP_REQUEST
    | INPUT_FILL_REQUEST
    | INPUT_SEPARATE_REQUEST
    | INPUT_VALVE_SET_REQUEST
    | INPUT_O2_TEST_REQUEST
    | INPUT_VALVE_OPEN_REQUEST;
pub const INPUT_COMMAND_MASK: u32 = INPUT_OPERATOR_ACTION_MASK | INPUT_RESET_ACK_REQUEST;

pub static INPUT_FLAGS: AtomicU32 = AtomicU32::new(0);

pub fn update_input_flag(flag: u32, asserted: bool) {
    let previous = if asserted {
        INPUT_FLAGS.fetch_or(flag, Ordering::AcqRel)
    } else {
        INPUT_FLAGS.fetch_and(!flag, Ordering::AcqRel)
    };
    let current = if asserted {
        previous | flag
    } else {
        previous & !flag
    };
    if previous != current {
        CONTROL_UPDATE_SIGNAL.signal(());
    }
}

pub fn replace_operator_input_flags(flags: u32) {
    let operator_flags = flags & INPUT_OPERATOR_ACTION_MASK;
    if let Ok(previous) = INPUT_FLAGS.fetch_update(Ordering::AcqRel, Ordering::Acquire, |current| {
        Some((current & !INPUT_COMMAND_MASK) | operator_flags)
    }) {
        let current = (previous & !INPUT_COMMAND_MASK) | operator_flags;
        if previous != current {
            CONTROL_UPDATE_SIGNAL.signal(());
        }
    }
}

pub fn replace_operator_input_flags_and_set_can_link_active(flags: u32) {
    let operator_flags = flags & INPUT_OPERATOR_ACTION_MASK;
    if let Ok(previous) = INPUT_FLAGS.fetch_update(Ordering::AcqRel, Ordering::Acquire, |current| {
        Some((current & !INPUT_COMMAND_MASK) | operator_flags | INPUT_CAN_LINK_ACTIVE)
    }) {
        let current = (previous & !INPUT_COMMAND_MASK) | operator_flags | INPUT_CAN_LINK_ACTIVE;
        if previous != current {
            CONTROL_UPDATE_SIGNAL.signal(());
        }
    }
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SequencePhase {
    Idle = 0,
    Firing = 1,
    Timeout = 2,
    Abort = 3,
}

impl SequencePhase {
    pub fn from_u8(raw: u8) -> Self {
        match raw {
            1 => Self::Firing,
            2 => Self::Timeout,
            3 => Self::Abort,
            _ => Self::Idle,
        }
    }
}

pub static SEQUENCE_PHASE: AtomicU8 = AtomicU8::new(SequencePhase::Idle as u8);

pub fn sequence_phase() -> SequencePhase {
    SequencePhase::from_u8(SEQUENCE_PHASE.load(Ordering::Acquire))
}

pub fn set_sequence_phase(phase: SequencePhase) {
    SEQUENCE_PHASE.store(phase as u8, Ordering::Release);
    CONTROL_UPDATE_SIGNAL.signal(());
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServoAction {
    Hold,
    MoveTo(i16),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlIntent {
    pub ignition_on: bool,
    pub dump_on: bool,
    pub fill_on: bool,
    pub separate_on: bool,
    pub o2_on: bool,
    pub servo_target_angle_x10: Option<i16>,
}

impl ControlIntent {
    pub const fn safe() -> Self {
        Self {
            ignition_on: false,
            dump_on: false,
            fill_on: false,
            separate_on: false,
            o2_on: false,
            servo_target_angle_x10: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlDecision {
    pub ignition_on: bool,
    pub dump_on: bool,
    pub fill_on: bool,
    pub separate_on: bool,
    pub o2_on: bool,
    pub servo_action: ServoAction,
    pub allow_new_ignition: bool,
}

pub fn resolve_control(
    phase: SequencePhase,
    fault_flags: u8,
    input_flags: u32,
    intent: ControlIntent,
) -> ControlDecision {
    let operator_input_available = input_flags & INPUT_CAN_LINK_ACTIVE != 0;
    let hard_output_inhibited = fault_flags & HARD_OUTPUT_INHIBIT_FAULTS != 0;
    let allow_new_ignition =
        phase == SequencePhase::Idle && operator_input_available && fault_flags == 0;

    if !operator_input_available || hard_output_inhibited {
        return ControlDecision {
            ignition_on: false,
            dump_on: false,
            fill_on: false,
            separate_on: false,
            o2_on: false,
            servo_action: ServoAction::Hold,
            allow_new_ignition: false,
        };
    }

    let manual_phase = matches!(
        phase,
        SequencePhase::Idle | SequencePhase::Timeout | SequencePhase::Abort
    );
    let firing_phase = phase == SequencePhase::Firing;
    let servo_allowed = phase != SequencePhase::Abort
        && fault_flags & SERVO_FAULTS == 0
        && !(firing_phase && fault_flags != 0);

    ControlDecision {
        ignition_on: firing_phase && intent.ignition_on && fault_flags == 0,
        dump_on: intent.dump_on,
        fill_on: manual_phase && intent.fill_on,
        separate_on: manual_phase && intent.separate_on,
        o2_on: (manual_phase || firing_phase) && intent.o2_on,
        servo_action: if servo_allowed {
            match intent.servo_target_angle_x10 {
                Some(angle) => ServoAction::MoveTo(angle),
                None => ServoAction::Hold,
            }
        } else {
            ServoAction::Hold
        },
        allow_new_ignition,
    }
}

// OutputGpioStatus payload bits.
pub const OUT_DUMP: u8 = 1 << 0;
pub const OUT_FILL: u8 = 1 << 1;
pub const OUT_SEPARATE: u8 = 1 << 2;
pub const OUT_O2: u8 = 1 << 3;
pub const OUT_IGNITER: u8 = 1 << 4;
pub const OUT_SPARE_SOLENOID: u8 = 1 << 5;
pub static OUTPUT_STATUS: AtomicU8 = AtomicU8::new(0);

// InputGpioStatus payload bits.
pub const IN_SOLENOID_POWER_PRESENT: u8 = 1 << 0;
pub const IN_RELAY_12V_ON: u8 = 1 << 1;
pub const IN_IGNITER_POWER_PRESENT: u8 = 1 << 2;
pub const IN_RELAY_24V_ON: u8 = 1 << 3;
pub static INPUT_GPIO_STATUS: AtomicU8 = AtomicU8::new(0);

pub const SERVO_MODE_HOLD: u8 = 0;
pub const SERVO_MODE_COMMAND: u8 = 1;
pub static SERVO_CONTROL_MODE: AtomicU8 = AtomicU8::new(SERVO_MODE_HOLD);
pub static SERVO_TARGET_ANGLE_X10: AtomicI16 = AtomicI16::new(MAIN_VALVE_CLOSED_ANGLE_X10);
pub static CURRENT_POSITION: AtomicU16 = AtomicU16::new(SERVO_CENTER_POS);
pub static SERVO_COMM_ACTIVE: AtomicBool = AtomicBool::new(false);
pub(crate) static SERVO_COMM_ERROR_COUNT: AtomicU8 = AtomicU8::new(0);
pub(crate) static FIRING_OPEN_LATCHED: AtomicBool = AtomicBool::new(false);
pub(crate) static FIRING_OPEN_SEND_COUNT: AtomicU8 = AtomicU8::new(0);
pub(crate) static OPEN_LATCH_RELEASE_PENDING: AtomicBool = AtomicBool::new(false);
pub static CAN_COMM_ACTIVE: AtomicBool = AtomicBool::new(false);

pub static CONTROL_UPDATE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static ANGLE_COMMAND_SIGNAL: Signal<CriticalSectionRawMutex, i16> = Signal::new();
pub static RESET_ACK_EVENT_COUNTER: AtomicU8 = AtomicU8::new(0);

pub fn signal_reset_ack_event() {
    RESET_ACK_EVENT_COUNTER.fetch_add(1, Ordering::AcqRel);
    CONTROL_UPDATE_SIGNAL.signal(());
}

pub static CAN_HEALTH: AtomicU8 = AtomicU8::new(CanHealth::Active as u8);
pub static CAN_TEC: AtomicU8 = AtomicU8::new(0);
pub static CAN_REC: AtomicU8 = AtomicU8::new(0);
pub static CAN_TX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static CAN_RX_ERROR_COUNT: AtomicU32 = AtomicU32::new(0);

pub fn angle_x10_to_position(angle_x10: i16) -> u16 {
    let position =
        ((angle_x10 as f32 / 10.0) * SERVO_POS_PER_DEGREE + SERVO_CENTER_POS as f32) as i32;

    if position < SERVO_MIN_POS as i32 {
        SERVO_MIN_POS
    } else if position > SERVO_MAX_POS as i32 {
        SERVO_MAX_POS
    } else {
        position as u16
    }
}

pub fn position_to_angle_x10(position: u16) -> i16 {
    (((position as f32 - SERVO_CENTER_POS as f32) / SERVO_POS_PER_DEGREE) * 10.0) as i16
}
