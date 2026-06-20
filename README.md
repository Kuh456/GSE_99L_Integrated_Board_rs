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
