#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanHealth {
    Active = 0,
    Warning = 1,
    Passive = 2,
    BusOff = 3,
}

pub const CAN_ERROR_WARNING_THRESHOLD: u8 = 96;
pub const CAN_ERROR_PASSIVE_THRESHOLD: u8 = 128;

pub fn classify_can_health(tec: u8, rec: u8, bus_off: bool) -> CanHealth {
    if bus_off {
        CanHealth::BusOff
    } else {
        match tec.max(rec) {
            count if count >= CAN_ERROR_PASSIVE_THRESHOLD => CanHealth::Passive,
            count if count >= CAN_ERROR_WARNING_THRESHOLD => CanHealth::Warning,
            _ => CanHealth::Active,
        }
    }
}
