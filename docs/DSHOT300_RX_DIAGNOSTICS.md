# DShot300 RPM receiver diagnosis (bench only)

**Not flight-ready.** This branch is isolated from
`feature/dshot300-bidir-rpm` and does not alter existing DShot600
task 4, DShot300 task 17, or 9-byte RPM task 16 PDO layouts.

## Why this exists

ROS2 `valid_mask` remains true for up to **100 ms** after the latest
validated reply. It is NOT an instantaneous ESC frame-success flag.
A 20-second operator trace showed 13.65% valid ROS messages but
that cannot be converted to per-DShot packet success rate.

## New optional TaskType=18 (LAB ONLY)

- SDO parameters: `uint8 dshot_id=2` (2 bytes incl. task-type)
- Slave-to-master PDO: 10 little-endian uint32 = 40 bytes
- Existing TaskType=16 remains unchanged at 9 bytes
- The 3-task bench config uses `sdo_len=10`, `task_count=3`
- Offset for Task18 = 9, so 9 + 40 = 49 bytes within the existing
  slave-to-master 80-byte buffer.
- Compiled only for `AIM_DSHOT_RPM_LAB_RX=ON`; production/default
  binary intentionally does not implement task 18.

| PDO index | Diagnostic field | Meaning |
| --- | --- | --- |
| 0 | tx_complete | All four TIM2 command DMA completion callbacks observed |
| 1 | capture_calls | Times control loop finalized the PA0 receive window |
| 2 | no_start_edge | Armed window without a first falling edge |
| 3 | edge_overflow | Capture exceeded the fixed 24-edge buffer |
| 4 | decode_ok | Fresh 21-bit GCR/CRC reply accepted |
| 5 | decode_fail | Captured waveform failed 21-bit/GCR/CRC validation |
| 6 | total_edges | Total captured edges across windows |
| 7 | last_edge_count | Edge count in latest window |
| 8 | last_ok_age_ms | HAL milliseconds since last accepted reply; 0xFFFFFFFF if none |
| 9 | no_capture_window | Control cycle finalized before a receiver was armed |

All counters are cumulative since boot; `last_edge_count` and
`last_ok_age_ms` are gauges, not cumulative counters.
`decode_ok/(decode_ok+decode_fail+no_start_edge+edge_overflow)`
is only a preliminary proxy for receive-window success: exclude
`no_capture_window`, and correlate with `tx_complete` to ensure
there was an actual command cycle. A gap in `tx_complete` relative
to `capture_calls` suggests a missing TX completion/arming stage.

## What the diagnostics do NOT prove

- EXTI0/DWT edges can be missed under interrupt latency.
- Software turns PA0 from push-pull PWM2 into EXTI input; turnaround
  and contention have NOT been verified electrically.
- CRC accept does not guarantee absolute RPM calibration.
- Only TIM2 PA0 receives; channels 2-4 remain invalid.
- Diagnostics do not replace logic-analyzer captures of PA0 TX and reply.

## Build

GitHub CI builds both original firmware and LAB ELF, saved separately.
The LAB artifact must never be mistaken for production firmware.
The corresponding ROS2 master is:
https://github.com/ssybh2/EcatV2_Master/tree/feature/dshot300-rpm-diagnostics

## Example ROS2 commands

```bash
source /opt/ros/humble/setup.bash
source /home/hby/Rolling_ZLT/install_dshot300_diag/setup.bash

ros2 topic echo /ecat/sn4653120/app3/rpm_diagnostics \
  --qos-reliability best_effort
```

Current non-diagnostic firmware and config remain usable without change.
