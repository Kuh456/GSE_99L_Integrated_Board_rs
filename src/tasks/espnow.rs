use core::cell::RefCell;

use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use esp_println::println;
use esp_radio::{
    esp_now::EspNow,
    wifi::{self, WifiController},
};

// Must match the Arduino ESP-NOW sender channel. The sender should ideally set
// esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE) and peerInfo.channel = 1.
pub const ESPNOW_CHANNEL: u8 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogData {
    pub start_time: u64,
    pub adc0: u16,
    pub adc2: u16,
    pub adc3: u16,
    pub counter: u16,
}

impl LogData {
    pub const LEN: usize = 16;

    pub fn parse(payload: &[u8]) -> Option<Self> {
        if payload.len() != Self::LEN {
            return None;
        }

        let start_time = u64::from_le_bytes(payload[0..8].try_into().ok()?);
        let adc0 = u16::from_le_bytes(payload[8..10].try_into().ok()?);
        let adc2 = u16::from_le_bytes(payload[10..12].try_into().ok()?);
        let adc3 = u16::from_le_bytes(payload[12..14].try_into().ok()?);

        Some(Self {
            start_time,
            adc0,
            adc2,
            adc3,
            counter: 0,
        })
    }
}

pub static LATEST_LOG_DATA: Mutex<CriticalSectionRawMutex, RefCell<Option<LogData>>> =
    Mutex::new(RefCell::new(None));

pub fn latest_log_data() -> Option<LogData> {
    LATEST_LOG_DATA.lock(|latest| *latest.borrow())
}

#[embassy_executor::task]
pub async fn espnow_receive_task(
    _controller: WifiController<'static>,
    mut esp_now: EspNow<'static>,
) {
    loop {
        let packet = esp_now.receive_async().await;
        let payload = packet.data();

        let Some(mut log_data) = LogData::parse(payload) else {
            println!(
                "[ESP-NOW] warn invalid payload length: {} bytes, expected {} bytes",
                payload.len(),
                LogData::LEN
            );
            continue;
        };

        LATEST_LOG_DATA.lock(|latest| {
            log_data.counter = latest
                .borrow()
                .map_or(1, |latest| latest.counter.wrapping_add(1));
            *latest.borrow_mut() = Some(log_data);
        });

        let _mac = packet.info.src_address;
        // println!(
        //     "[ESP-NOW] recv from {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        //     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
        // );
        // println!("[ESP-NOW] Start: {} ms", log_data.start_time / 1000);
        // println!(
        //     "[ESP-NOW] ADC(mcp3208_0 CH0/CH2/CH3): {} {} {}",
        //     log_data.adc0, log_data.adc2, log_data.adc3
        // );
    }
}

pub fn initialize_espnow(
    wifi: esp_hal::peripherals::WIFI<'static>,
) -> (WifiController<'static>, EspNow<'static>) {
    let (controller, interfaces) = wifi::new(wifi, Default::default()).unwrap();
    interfaces.esp_now.set_channel(ESPNOW_CHANNEL).unwrap();
    (controller, interfaces.esp_now)
}
