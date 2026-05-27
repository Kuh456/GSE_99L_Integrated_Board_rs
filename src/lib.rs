#![no_std]

pub mod can;
pub mod krs_servo;
pub mod tasks;

use core::sync::atomic::{AtomicI16, AtomicU8, AtomicU16, AtomicU32, Ordering};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

pub use can::health::CanHealth;
pub use can::protocol::{
    CAN_ID_BUTTON_FROM_CTRL_PANEL, CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL, CAN_ID_SEQUENCE_STATE,
    CAN_ID_SERVO_COMMUNICATION_STATE, CAN_ID_SOLENOID_STATE,
};

pub const COMMUNICATION_TIMEOUT_MS: u64 = 3000;
pub const CAN_STATUS_TX_INTERVAL_MS: u64 = 50;
pub const CAN_TX_TIMEOUT_MS: u64 = 10;
pub const SUPERVISOR_INTERVAL_MS: u64 = 10;

// Sequence timing.
pub const IGNITION_WAIT_MS: u64 = 10000;
pub const MAIN_VALVE_OPEN_DELAY_MS: u64 = 3000;
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
pub const SERVO_POSITION_TOLERANCE: u16 = 120;
pub const SERVO_ERROR_LIMIT: u8 = 10;

// Latched fault flags. Fault removal is an explicit reset operation, never a side effect of
// receiving traffic again.
pub const CAN_PEER_LOST: u32 = 1 << 0;
pub const CAN_BUS_OFF: u32 = 1 << 1;
pub const SERVO_COMM_ERROR: u32 = 1 << 2;
pub const SERVO_POS_ERROR: u32 = 1 << 3;
// Reserved for an abnormal ignition watchdog fault; normal Firing completion is Timeout.
pub const IGNITION_TIMEOUT: u32 = 1 << 4;
pub const POWER_ERROR: u32 = 1 << 5;
pub const CAN_TX_TIMEOUT: u32 = 1 << 6;
pub const CAN_TX_FRAME_CREATE_FAILED: u32 = 1 << 7;
pub const CAN_FAULTS: u32 =
    CAN_PEER_LOST | CAN_BUS_OFF | CAN_TX_TIMEOUT | CAN_TX_FRAME_CREATE_FAILED;
pub const SERVO_FAULTS: u32 = SERVO_COMM_ERROR | SERVO_POS_ERROR;

pub static FAULT_FLAGS: AtomicU32 = AtomicU32::new(0);

pub fn set_fault_flags(flags: u32) {
    let previous = FAULT_FLAGS.fetch_or(flags, Ordering::AcqRel);
    if previous & flags != flags {
        CONTROL_UPDATE_SIGNAL.signal(());
    }
}

/// Clears latched faults only when an explicit reset policy has approved it.
pub fn clear_fault_flags_for_reset(flags: u32) {
    FAULT_FLAGS.fetch_and(!flags, Ordering::AcqRel);
    CONTROL_UPDATE_SIGNAL.signal(());
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
pub const INPUT_COMMAND_MASK: u32 = INPUT_FIRE_REQUEST
    | INPUT_DUMP_REQUEST
    | INPUT_FILL_REQUEST
    | INPUT_SEPARATE_REQUEST
    | INPUT_VALVE_SET_REQUEST
    | INPUT_O2_TEST_REQUEST
    | INPUT_VALVE_OPEN_REQUEST;

pub static INPUT_FLAGS: AtomicU32 = AtomicU32::new(0);

pub fn update_input_flag(flag: u32, asserted: bool) {
    if asserted {
        INPUT_FLAGS.fetch_or(flag, Ordering::AcqRel);
    } else {
        INPUT_FLAGS.fetch_and(!flag, Ordering::AcqRel);
    }
    CONTROL_UPDATE_SIGNAL.signal(());
}

pub fn replace_operator_input_flags(flags: u32) {
    let operator_flags = flags & INPUT_COMMAND_MASK;
    let _ = INPUT_FLAGS.fetch_update(Ordering::AcqRel, Ordering::Acquire, |current| {
        Some((current & !INPUT_COMMAND_MASK) | operator_flags)
    });
    CONTROL_UPDATE_SIGNAL.signal(());
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
    fault_flags: u32,
    input_flags: u32,
    intent: ControlIntent,
) -> ControlDecision {
    let can_available = input_flags & INPUT_CAN_LINK_ACTIVE != 0 && fault_flags & CAN_FAULTS == 0;
    let output_inhibited = !can_available || fault_flags != 0 || phase == SequencePhase::Abort;
    let allow_new_ignition = phase == SequencePhase::Idle && can_available && fault_flags == 0;

    if output_inhibited {
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

    ControlDecision {
        ignition_on: phase == SequencePhase::Firing && intent.ignition_on,
        dump_on: intent.dump_on,
        fill_on: intent.fill_on,
        separate_on: intent.separate_on,
        o2_on: intent.o2_on,
        servo_action: match intent.servo_target_angle_x10 {
            Some(angle) => ServoAction::MoveTo(angle),
            None => ServoAction::Hold,
        },
        allow_new_ignition,
    }
}

pub const OUTPUT_DUMP_ON: u8 = 1 << 0;
pub const OUTPUT_IGNITION_ON: u8 = 1 << 1;
pub const OUTPUT_FILL_ON: u8 = 1 << 2;
pub const OUTPUT_SEPARATE_ON: u8 = 1 << 3;
pub const OUTPUT_O2_ON: u8 = 1 << 4;
pub static OUTPUT_STATUS: AtomicU8 = AtomicU8::new(0);

pub const SERVO_MODE_HOLD: u8 = 0;
pub const SERVO_MODE_COMMAND: u8 = 1;
pub static SERVO_CONTROL_MODE: AtomicU8 = AtomicU8::new(SERVO_MODE_HOLD);
pub static SERVO_TARGET_ANGLE_X10: AtomicI16 = AtomicI16::new(MAIN_VALVE_CLOSED_ANGLE_X10);
pub static CURRENT_POSITION: AtomicU16 = AtomicU16::new(SERVO_CENTER_POS);

pub static CONTROL_UPDATE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static ANGLE_COMMAND_SIGNAL: Signal<CriticalSectionRawMutex, i16> = Signal::new();

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
