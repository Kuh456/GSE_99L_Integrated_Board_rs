use crate::{SERVO_MAX_POS, SERVO_MIN_POS};
use embassy_time::Timer;
use esp_hal::{
    Blocking,
    gpio::Output,
    rom::ets_delay_us,
    time::{Duration, Instant},
    uart::Uart,
};
use esp_println::println;
// --- 定数定義  ---
const MAX_ID: u8 = 31;
const TIMEOUT_MS: u64 = 100;

const RX_POLL_INTERVAL_US: u32 = 100;

// ESP HAL の flush() が最後のストップビット送信完了まで
// 保証しない可能性を考慮した送受信切替マージン。
// 115200bpsでは1bitが約8.7usなので、必要なら100〜200usへ増やして確認する。
const ICS_TX_TURNAROUND_DELAY_US: u32 = 20;

// transfer周辺のデバッグログ。通常時はfalseのままにする。
const DEBUG_ICS_TRANSFER: bool = false;

// エラー型を定義
#[derive(Debug)]
pub enum IcsError {
    InvalidId,
    PositionOutOfRange,
    CommunicationError,
    ParameterError,
    TimeoutError,
}

// ---  構造体定義  ---
// ハードウェアをまとめる構造体
// 'd: ライフタイム
pub struct IcsDevice<'d> {
    // UARTドライバを持たせる
    uart: Uart<'d, Blocking>,
    // ENピン（Output）を持たせる
    en_pin: Output<'d>,
}

impl<'d> IcsDevice<'d> {
    // コンストラクタ: main.rs から UART と en_pin を受け取る
    pub fn new(uart: Uart<'d, Blocking>, mut en_pin: Output<'d>) -> Self {
        en_pin.set_low();
        Self { uart, en_pin }
    }

    fn write_all_blocking(&mut self, mut data: &[u8]) -> Result<(), IcsError> {
        while !data.is_empty() {
            let written = self
                .uart
                .write(data)
                .map_err(|_| IcsError::CommunicationError)?;
            if written == 0 {
                return Err(IcsError::CommunicationError);
            }
            data = &data[written..];
        }

        Ok(())
    }

    fn drain_rx_buffer(&mut self) -> Result<(), IcsError> {
        let mut trash = [0u8; 16];
        let mut total = 0usize;

        while self.uart.read_ready() {
            let read = self
                .uart
                .read(&mut trash)
                .map_err(|_| IcsError::CommunicationError)?;
            if read == 0 {
                break;
            }

            if DEBUG_ICS_TRANSFER {
                for b in &trash[..read] {
                    println!("ICS drain rx: 0x{:02X}", *b);
                }
            }

            total += read;
        }

        if DEBUG_ICS_TRANSFER && total > 0 {
            println!("ICS drain rx total: {} bytes", total);
        }

        Ok(())
    }

    async fn read_expected_timeout(&mut self, data: &mut [u8]) -> Result<(), IcsError> {
        let expected = data.len();
        let mut read_count = 0usize;
        let start = Instant::now();
        let timeout = Duration::from_millis(TIMEOUT_MS);

        while read_count < data.len() {
            if self.uart.read_ready() {
                match self.uart.read(&mut data[read_count..]) {
                    Ok(0) => {}
                    Ok(n) => {
                        if DEBUG_ICS_TRANSFER {
                            for b in &data[read_count..read_count + n] {
                                println!("ICS rx: 0x{:02X}", *b);
                            }
                            println!("ICS read progress: {}/{} bytes", read_count + n, expected);
                        }
                        read_count += n;
                        continue;
                    }
                    Err(_) => return Err(IcsError::CommunicationError),
                }
            }

            if start.elapsed() >= timeout {
                // println!(
                //     "ICS Timeout! Expected {} bytes, but read {} bytes",
                //     expected, read_count
                // );
                if DEBUG_ICS_TRANSFER && read_count > 0 {
                    for b in &data[..read_count] {
                        println!("ICS rx before timeout: 0x{:02X}", *b);
                    }
                }
                return Err(IcsError::TimeoutError);
            }

            Timer::after_micros(RX_POLL_INTERVAL_US as u64).await;
        }

        Ok(())
    }

    // 通信部分（private)
    // 内部で ENピン操作 -> 送信 -> 受信 を行う
    async fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> Result<(), IcsError> {
        // Arduino版 synchronize() と同じく、送信前にUARTを待機状態へそろえる。
        self.uart
            .flush()
            .map_err(|_| IcsError::CommunicationError)?;

        // 前回通信の残りや起動直後のゴミを捨てる。
        self.drain_rx_buffer()?;

        if DEBUG_ICS_TRANSFER {
            for b in tx {
                println!("ICS tx: 0x{:02X}", *b);
            }
        }

        self.en_pin.set_high();

        self.write_all_blocking(tx)?;

        self.uart
            .flush()
            .map_err(|_| IcsError::CommunicationError)?;

        // flush() 直後にENを戻すと最後のビットを潰す可能性があるため、
        // 送受信切替のマージンを入れる。
        ets_delay_us(ICS_TX_TURNAROUND_DELAY_US);

        // 半二重回路で自分の送信がRX側に回り込む場合、そのエコーを捨てる。
        self.drain_rx_buffer()?;

        // 受信モードへ切り替える。
        self.en_pin.set_low();

        self.read_expected_timeout(rx).await
    }
    // ---  補助関数 (private) ---

    // IDが範囲内かチェック
    fn check_id(id: u8) -> Result<u8, IcsError> {
        if id > MAX_ID {
            Err(IcsError::InvalidId)
        } else {
            Ok(id)
        }
    }

    // 値を範囲内に収める
    fn clip_pos(pos: u16) -> Result<u16, IcsError> {
        if !(SERVO_MIN_POS..=SERVO_MAX_POS).contains(&pos) {
            return Err(IcsError::PositionOutOfRange);
        }
        Ok(pos)
    }

    // --- コマンド実装 ---
    // サーボ角度セット
    pub async fn set_pos(&mut self, id: u8, pos: u16) -> Result<u16, IcsError> {
        // IDと範囲のチェック
        let valid_id = Self::check_id(id)?;
        let valid_pos = Self::clip_pos(pos)?;

        // コマンド生成
        // CMD: 0x80 + ID
        // POS_H: (pos >> 7) & 0x7F
        // POS_L: pos & 0x7F
        let tx_cmd = [
            0x80 + valid_id,
            ((valid_pos >> 7) & 0x007F) as u8,
            (valid_pos & 0x007F) as u8,
        ];

        let mut rx_cmd = [0u8; 3]; // 受信バッファ

        // 送受信
        self.transfer(&tx_cmd, &mut rx_cmd).await?;

        // 受信データから現在位置を復元
        // ((rx[1] << 7) & 0x3F80) + (rx[2] & 0x7F)
        let re_pos = ((rx_cmd[1] as u16) << 7) & 0x3F80 | (rx_cmd[2] as u16) & 0x007F;
        Ok(re_pos)
    }

    // サーボをフリー状態にする
    pub async fn set_free(&mut self, id: u8) -> Result<u16, IcsError> {
        let valid_id = Self::check_id(id)?;

        let tx_cmd = [
            0x80 + valid_id, // CMD
            0,               // 0
            0,               // 0
        ];
        let mut rx_cmd = [0u8; 3];

        self.transfer(&tx_cmd, &mut rx_cmd).await?;

        let re_pos = ((rx_cmd[1] as u16) << 7) & 0x3F80 | (rx_cmd[2] as u16) & 0x007F;
        Ok(re_pos)
    }

    // IDを取得する
    pub async fn get_id(&mut self) -> Result<u8, IcsError> {
        let tx_cmd = [0xFF, 0x00, 0x00, 0x00];
        let mut rx_cmd = [0u8; 1];

        // 送受信
        self.transfer(&tx_cmd, &mut rx_cmd).await?;

        ets_delay_us(520_000);
        // マスク処理
        let id = 0x1F & rx_cmd[0];
        Ok(id)
    }

    // スピードを取得する
    pub async fn get_spd(&mut self, id: u8) -> Result<u8, IcsError> {
        let valid_id = Self::check_id(id)?;
        let tx_cmd = [
            0xA0 + valid_id, // CMD
            0x02,            // スピード読み取り
        ];
        let mut rx_cmd = [0u8; 3];

        // 送受信
        self.transfer(&tx_cmd, &mut rx_cmd).await?;
        let speed = rx_cmd[2];
        Ok(speed)
    }

    // 現在のサーボ角度を取得 ※ICS3.6以降で有効
    pub async fn get_pos(&mut self, id: u8) -> Result<u16, IcsError> {
        let valid_id = Self::check_id(id)?;

        let tx_cmd = [
            0xA0 + valid_id, // CMD
            0x05,            // 角度読み取り
        ];
        let mut rx_cmd = [0u8; 4];

        self.transfer(&tx_cmd, &mut rx_cmd).await?;

        let read_pos = ((rx_cmd[2] as u16) << 7) & 0x3F80 | (rx_cmd[3] as u16) & 0x007F;
        Ok(read_pos)
    }
}
