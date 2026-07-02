# GSE Integrated Board

## CAN Fire Test

`can_fire` は、コントロールパネルからの CAN コマンドで点火出力を確認する
ハードウェアテストです。ESP32 と CAN トランシーバーを接続した状態で、次の
コマンドを実行してください。

```bash
cargo test --test can_fire --features hardware-test
```

プロジェクトの runner により、テストファームウェアの書き込み後にシリアル
モニターが開きます。使用する端子と CAN 設定は次のとおりです。

- CAN TX: GPIO18
- CAN RX: GPIO4
- FIRE/IGNITER 出力: GPIO13
- CAN ビットレート: 125 kbit/s

起動すると、シリアルモニターに次のバナーが表示されます。

```text
========== CAN FIRE TEST START ==========
CAN: TWAI0 baud=125k tx=GPIO18 rx=GPIO4
OUTPUT: FIRE/IGNITER=GPIO13
```

コントロールパネルから `ButtonFromCtrlPanel` フレームを送信すると、データの
bit 1 が `1` の間は GPIO13 が High、`0` の間は Low になります。状態が変わると
`fire=ON` または `fire=OFF` が表示されます。最後の有効なコマンドから 500 ms
以内に次のコマンドを受信しなかった場合、出力は自動的に Low になります。

テスト基板からは5種類の状態フレームを50 ms間隔で巡回送信します。一時的な
送信失敗では同じフレームを次周期に再送します。送信タイムアウトまたは
Bus-Offが発生すると点火出力をLowにして周期送信を停止し、有効な
`ButtonFromCtrlPanel`を受信した後にprobe送信を行います。probeに成功すると
通常の周期送信へ戻ります。復帰後、次の有効なFIREコマンドから点火出力を
操作できます。

> 点火出力を扱うテストです。実際の点火装置を外した安全な状態で、GPIO13 の
> 出力を測定して確認してください。

## CAN Debug Logs

CAN debug logging is opt-in. Run the firmware with the `can-debug-log` feature to print
CAN health counters once per second:

```bash
cargo run --features can-debug-log
```

The project runner flashes the ESP32 and opens the serial monitor at 115200 baud:

```text
espflash flash --monitor --chip esp32 --baud 2000000 --monitor-baud 115200 --log-format serial
```

Expected monitor output starts with:

```text
boot can-debug-log enabled
boot spawning can_debug_log_task on second core
boot spawned can_debug_log_task on second core
can_dbg task_start
```

Then it includes CAN and servo command status lines like:

```text
can_dbg CAN_COMM_ACTIVE=... CAN_HEALTH=... CAN_TEC=... CAN_REC=... FAULT_FLAGS=0x... CAN_TX_ERROR_COUNT=... CAN_RX_ERROR_COUNT=... OPEN_LATCHED=... OPEN_SEND_COUNT=... OPEN_RELEASE_PENDING=... SERVO_MODE=... SERVO_TARGET_X10=...
servo_cmd kind=firing_open angle_x10=-348 open_count=... at_ms=...
```

## Servo Communication Debug Logs

Servo communication error logging is opt-in. Enable `servo-debug-log` to print the
consecutive error count for each failed transaction, the point where the fault threshold
is reached, and recovery after a successful transaction:

```bash
cargo run --features servo-debug-log
```

Example output:

```text
servo_dbg event=comm_error operation=get_pos at_ms=1000 consecutive_error_count=1 fault_active=false error=TimeoutError
servo_dbg event=fault_raised operation=get_pos at_ms=5500 consecutive_error_count=10 fault_active=true error=TimeoutError
servo_dbg event=recovered operation=get_pos at_ms=6000 previous_consecutive_error_count=10
```

`SERVO_COMM_ERROR` is included in the CAN internal status after 10 consecutive failures.
One successful servo transaction resets the count and clears the non-latched status fault.
