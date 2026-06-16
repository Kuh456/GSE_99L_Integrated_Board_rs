# GSE Integrated Board

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

Then it includes CAN status lines like:

```text
can_dbg CAN_COMM_ACTIVE=... CAN_HEALTH=... CAN_TEC=... CAN_REC=... FAULT_FLAGS=0x... CAN_TX_ERROR_COUNT=... CAN_RX_ERROR_COUNT=...
```
