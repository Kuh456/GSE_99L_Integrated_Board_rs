pub const CAN_ID_BUTTON_FROM_CTRL_PANEL: u16 = 0x001;
pub const CAN_ID_MAIN_VALVE_ANGLE_TO_CTRL_PANEL: u16 = 0x101;
pub const CAN_ID_OUTPUT_GPIO_STATUS: u16 = 0x103;
pub const CAN_ID_INPUT_GPIO_STATUS: u16 = 0x104;
pub const CAN_ID_INTERNAL_STATUS: u16 = 0x105;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GseCanMessage {
    ButtonFromCtrlPanel { raw: u8 },
    MainValveAngleToCtrlPanel { angle_x10: i16 },
    OutputGpioStatus { output_bits: u8 },
    InputGpioStatus { input_bits: u8 },
    InternalStatus { phase: u8, flags: u8 },
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
            Self::MainValveAngleToCtrlPanel { .. } => CAN_ID_MAIN_VALVE_ANGLE_TO_CTRL_PANEL,
            Self::OutputGpioStatus { .. } => CAN_ID_OUTPUT_GPIO_STATUS,
            Self::InputGpioStatus { .. } => CAN_ID_INPUT_GPIO_STATUS,
            Self::InternalStatus { .. } => CAN_ID_INTERNAL_STATUS,
        }
    }

    pub const fn dlc(&self) -> usize {
        match self {
            Self::MainValveAngleToCtrlPanel { .. } | Self::InternalStatus { .. } => 2,
            Self::ButtonFromCtrlPanel { .. }
            | Self::OutputGpioStatus { .. }
            | Self::InputGpioStatus { .. } => 1,
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
            Self::OutputGpioStatus { output_bits } => out[0] = output_bits,
            Self::InputGpioStatus { input_bits } => out[0] = input_bits,
            Self::InternalStatus { phase, flags } => {
                out[0] = phase;
                out[1] = flags;
            }
        }
        self.dlc()
    }

    pub fn decode_standard(id: u16, data: &[u8]) -> Result<Self, CanDecodeError> {
        let expected = match id {
            CAN_ID_BUTTON_FROM_CTRL_PANEL
            | CAN_ID_OUTPUT_GPIO_STATUS
            | CAN_ID_INPUT_GPIO_STATUS => 1,
            CAN_ID_MAIN_VALVE_ANGLE_TO_CTRL_PANEL | CAN_ID_INTERNAL_STATUS => 2,
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
            CAN_ID_MAIN_VALVE_ANGLE_TO_CTRL_PANEL => Ok(Self::MainValveAngleToCtrlPanel {
                angle_x10: i16::from_le_bytes([data[0], data[1]]),
            }),
            CAN_ID_OUTPUT_GPIO_STATUS => Ok(Self::OutputGpioStatus {
                output_bits: data[0],
            }),
            CAN_ID_INPUT_GPIO_STATUS => Ok(Self::InputGpioStatus {
                input_bits: data[0],
            }),
            CAN_ID_INTERNAL_STATUS => Ok(Self::InternalStatus {
                phase: data[0],
                flags: data[1],
            }),
            _ => Err(CanDecodeError::UnknownId(id)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn messages_define_expected_ids_and_lengths() {
        assert_eq!(
            GseCanMessage::OutputGpioStatus { output_bits: 0 }.id(),
            CAN_ID_OUTPUT_GPIO_STATUS
        );
        assert_eq!(GseCanMessage::InputGpioStatus { input_bits: 0 }.dlc(), 1);
        assert_eq!(
            GseCanMessage::InternalStatus { phase: 0, flags: 0 }.dlc(),
            2
        );
    }

    #[test]
    fn valve_angle_round_trips_little_endian() {
        let message = GseCanMessage::MainValveAngleToCtrlPanel { angle_x10: -348 };
        let mut payload = [0; 8];

        assert_eq!(message.encode_payload(&mut payload), 2);
        assert_eq!(&payload[..2], &(-348i16).to_le_bytes());
        assert_eq!(
            GseCanMessage::decode_standard(message.id(), &payload[..2]),
            Ok(message)
        );
    }

    #[test]
    fn internal_status_encodes_phase_and_u8_fault_flags() {
        let message = GseCanMessage::InternalStatus {
            phase: 3,
            flags: 0xa5,
        };
        let mut payload = [0; 8];

        assert_eq!(message.encode_payload(&mut payload), 2);
        assert_eq!(&payload[..2], &[3, 0xa5]);
        assert_eq!(
            GseCanMessage::decode_standard(message.id(), &payload[..2]),
            Ok(message)
        );
    }

    #[test]
    fn invalid_dlc_and_unknown_id_are_reported() {
        assert_eq!(
            GseCanMessage::decode_standard(CAN_ID_INTERNAL_STATUS, &[1]),
            Err(CanDecodeError::InvalidDlc {
                id: CAN_ID_INTERNAL_STATUS,
                expected: 2,
                actual: 1,
            })
        );
        assert_eq!(
            GseCanMessage::decode_standard(0x7ff, &[]),
            Err(CanDecodeError::UnknownId(0x7ff))
        );
    }
}
