use embassy_time::{Duration, with_timeout};
use esp_hal::{
    Async,
    twai::{self, EspTwaiError, EspTwaiFrame, StandardId},
};

use super::protocol::GseCanMessage;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanFrameError {
    InvalidId(u16),
    InvalidPayloadLength(usize),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanTxError {
    TxInhibited,
    FrameCreateFailed,
    TransmitFailed,
    TimedOutUnknownState,
    BusOff,
}

pub fn create_frame_from_message(msg: GseCanMessage) -> Result<EspTwaiFrame, CanFrameError> {
    let mut payload = [0u8; 8];
    let len = msg.encode_payload(&mut payload);
    let id = StandardId::new(msg.id()).ok_or(CanFrameError::InvalidId(msg.id()))?;
    EspTwaiFrame::new(id, &payload[..len]).ok_or(CanFrameError::InvalidPayloadLength(len))
}

pub async fn transmit_frame_with_timeout(
    can: &mut twai::Twai<'static, Async>,
    frame: &EspTwaiFrame,
    timeout: Duration,
) -> Result<(), CanTxError> {
    match with_timeout(timeout, can.transmit_async(frame)).await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(EspTwaiError::BusOff)) => Err(CanTxError::BusOff),
        Ok(Err(_)) => Err(CanTxError::TransmitFailed),
        Err(_) => Err(CanTxError::TimedOutUnknownState),
    }
}
