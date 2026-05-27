pub const CAN_ID_BUTTON_FROM_CTRL_PANEL: u16 = 0x001;
pub const CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL: u16 = 0x101;
pub const CAN_ID_SOLENOID_STATE: u16 = 0x103;
pub const CAN_ID_SEQUENCE_STATE: u16 = 0x105;
pub const CAN_ID_SERVO_COMMUNICATION_STATE: u16 = 0x107;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GseCanMessage {
    ButtonFromCtrlPanel { raw: u8 },
    MainValveAngleToCtrlPanel { angle_x10: i16 },
    SolenoidState { bits: u8 },
    SequenceState { phase: u8 },
    ServoCommunicationState { error: bool },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanDecodeError {
    UnknownId(u16),
    InvalidDlc {
        id: u16,
        expected: usize,
        actual: usize,
    },
}

impl GseCanMessage {
    pub const fn id(&self) -> u16 {
        match self {
            Self::ButtonFromCtrlPanel { .. } => CAN_ID_BUTTON_FROM_CTRL_PANEL,
            Self::MainValveAngleToCtrlPanel { .. } => CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL,
            Self::SolenoidState { .. } => CAN_ID_SOLENOID_STATE,
            Self::SequenceState { .. } => CAN_ID_SEQUENCE_STATE,
            Self::ServoCommunicationState { .. } => CAN_ID_SERVO_COMMUNICATION_STATE,
        }
    }

    pub const fn dlc(&self) -> usize {
        match self {
            Self::MainValveAngleToCtrlPanel { .. } => 2,
            Self::ButtonFromCtrlPanel { .. }
            | Self::SolenoidState { .. }
            | Self::SequenceState { .. }
            | Self::ServoCommunicationState { .. } => 1,
        }
    }

    pub fn encode_payload(&self, out: &mut [u8; 8]) -> usize {
        match *self {
            Self::ButtonFromCtrlPanel { raw } => out[0] = raw,
            Self::MainValveAngleToCtrlPanel { angle_x10 } => {
                let bytes = angle_x10.to_le_bytes();
                out[0] = bytes[0];
                out[1] = bytes[1];
            }
            Self::SolenoidState { bits } => out[0] = bits,
            Self::SequenceState { phase } => out[0] = phase,
            Self::ServoCommunicationState { error } => out[0] = u8::from(error),
        }
        self.dlc()
    }

    pub fn decode_standard(id: u16, data: &[u8]) -> Result<Self, CanDecodeError> {
        let expected = match id {
            CAN_ID_BUTTON_FROM_CTRL_PANEL
            | CAN_ID_SOLENOID_STATE
            | CAN_ID_SEQUENCE_STATE
            | CAN_ID_SERVO_COMMUNICATION_STATE => 1,
            CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL => 2,
            _ => return Err(CanDecodeError::UnknownId(id)),
        };

        if data.len() != expected {
            return Err(CanDecodeError::InvalidDlc {
                id,
                expected,
                actual: data.len(),
            });
        }

        match id {
            CAN_ID_BUTTON_FROM_CTRL_PANEL => Ok(Self::ButtonFromCtrlPanel { raw: data[0] }),
            CAN_ID_SEND_MAIN_ANGLE_TO_CTRL_PANEL => Ok(Self::MainValveAngleToCtrlPanel {
                angle_x10: i16::from_le_bytes([data[0], data[1]]),
            }),
            CAN_ID_SOLENOID_STATE => Ok(Self::SolenoidState { bits: data[0] }),
            CAN_ID_SEQUENCE_STATE => Ok(Self::SequenceState { phase: data[0] }),
            CAN_ID_SERVO_COMMUNICATION_STATE => Ok(Self::ServoCommunicationState {
                error: data[0] != 0,
            }),
            _ => Err(CanDecodeError::UnknownId(id)),
        }
    }
}
