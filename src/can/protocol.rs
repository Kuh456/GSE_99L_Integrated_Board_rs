pub const CAN_ID_BUTTON_FROM_CTRL_PANEL: u16 = 0x001;
pub const CAN_ID_MAIN_VALVE_ANGLE_TO_CTRL_PANEL: u16 = 0x101;
pub const CAN_ID_OUTPUT_GPIO_STATUS: u16 = 0x103;
pub const CAN_ID_INPUT_GPIO_STATUS: u16 = 0x104;
pub const CAN_ID_INTERNAL_STATUS: u16 = 0x105;
pub const CAN_ID_LOGGER_DATA: u16 = 0x106;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GseCanMessage {
    ButtonFromCtrlPanel {
        raw: u8,
    },
    MainValveAngleToCtrlPanel {
        angle_x10: i16,
    },
    OutputGpioStatus {
        output_bits: u8,
    },
    InputGpioStatus {
        input_bits: u8,
    },
    InternalStatus {
        phase: u8,
        flags: u8,
    },
    LoggerData {
        adc0: u16,
        adc2: u16,
        adc3: u16,
        counter: u16,
    },
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
            Self::LoggerData { .. } => CAN_ID_LOGGER_DATA,
        }
    }

    pub const fn dlc(&self) -> usize {
        match self {
            Self::LoggerData { .. } => 8,
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
            Self::LoggerData {
                adc0,
                adc2,
                adc3,
                counter,
            } => {
                out[0..2].copy_from_slice(&adc0.to_le_bytes());
                out[2..4].copy_from_slice(&adc2.to_le_bytes());
                out[4..6].copy_from_slice(&adc3.to_le_bytes());
                out[6..8].copy_from_slice(&counter.to_le_bytes());
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
            CAN_ID_LOGGER_DATA => 8,
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
            CAN_ID_LOGGER_DATA => Ok(Self::LoggerData {
                adc0: u16::from_le_bytes([data[0], data[1]]),
                adc2: u16::from_le_bytes([data[2], data[3]]),
                adc3: u16::from_le_bytes([data[4], data[5]]),
                counter: u16::from_le_bytes([data[6], data[7]]),
            }),
            _ => Err(CanDecodeError::UnknownId(id)),
        }
    }
}
