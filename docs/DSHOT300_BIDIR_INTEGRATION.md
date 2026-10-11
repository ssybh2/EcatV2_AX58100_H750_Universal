# DShot300 bidirectional RPM proof of concept (H750/TIM2)

## Status: COMPILE-TESTED ONLY; NOT HARDWARE-VALIDATED

This code is additive. **Original task 4 DShot600 and every existing
AIMEtherCAT task are retained.** The default build is unchanged in
motor protocol and uses the legacy continuous DShot600 transmitter.
Do not flash this experimental opt-in binary to a flying robot.

### Why DShot300?

- DShot300 command bitrate: 300 kbit/s (3.333 us per symbol)
- DShot600 command bitrate: 600 kbit/s (1.667 us per symbol)
- Both carry a 16-bit packet with the same 11-bit command, telemetry bit,
  and 4-bit CRC; neither inherently has higher RPM resolution.
- Bidirectional eRPM reply bitrate is 1.25x the TX bitrate:
  DShot300 reply 375 kbit/s, DShot600 reply 750 kbit/s.
- Flycolor Raptor 5 35A / BLHeli_32 Proto 31.10.1 was verified by the
  user in Betaflight using DShot300 + RPM; **DShot600 RX is unverified**.

### Tasks

| Task | Behavior | Build |
| --- | --- | --- |
| 4 DSHOT | Original DShot600, normal checksum, PWM1, 4 channels | always |
| 16 DSHOT_RPM | Existing 9-byte PDO validity/period telemetry | always |
| 17 DSHOT300_BIDIR | Inverted CRC, PWM2 TX at 300 kbps, PA0 edge capture | LAB flag only |

Task 17 intentionally uses exactly the same SDO/PDO lengths and ROS
`WriteDSHOT` subscriber as task 4. Do not configure both task 4 and
task 17 on the same `dshot_id`; this would contend for one TIM/DMA block.
Only `dshot_id=2` is accepted for type 17. The legacy task 4 remains
available on TIM3 (`dshot_id=1`) independently.

### Timing

Type 17 alone: TIM2 12 MHz, ARR=39 -> 40 ticks = 3.333 us/bit.
In PWM2 mode, 0 is low for 14 ticks, 1 for 28 ticks.
The 16-bit inverted-CRC command frame lasts ~53.3 us (plus trailing
idle cells). Once all four TX DMA streams complete, release PA0-PA3
to inputs and sample PA0 EXTI0 both edges via DWT CPU cycle counter.
The expected ESC response is 375 kbit/s (2.667 us/bit).
Validate each decoded 21-bit reply's GCR and checksum before putting
data in the pre-existing RPM cache; reject missing, stale, or invalid frames.

**WARNING:** PA0 EXTI proof of concept is not guaranteed at 375 kbit/s.
DWT ISR latency, GPIO tri-state turnaround, priority interference, race
conditions and signal levels have not been hardware tested. TX DMA
completion timing must be observed on an oscilloscope. Only PA0 (CH1)
capture is implemented: CH2-CH4 deliberately publish invalid. Future
full four/eight-channel reception requires a separately validated
timer input-capture DMA design with STM32H7 cache coherency.

### Build commands

```bash
# Production-compatible default: task 4 remains DShot600 and task 17
# is unavailable. This is the only ELF uploaded by GitHub Actions.
cmake --preset Release -B build/Release -DAIM_DSHOT_RPM_LAB_RX=OFF
cmake --build build/Release --target EcatV2_AX58100_H750_Universal

# Experimental build, for detached propeller-free electrical bench only:
cmake --preset Release -B build/Lab -DAIM_DSHOT_RPM_LAB_RX=ON
cmake --build build/Lab --target EcatV2_AX58100_H750_Universal
```

New ROS2 master uses app type 17. See
https://github.com/ssybh2/EcatV2_Master/blob/feature/dshot300-bidir-rpm/docs/DSHOT300_BIDIR_RPM.md

### Acceptance gates before use

- Default full ARM build green; lab full ARM build green.
- Original EtherCAT messages/topic names and PDO offsets unchanged.
- Original task-4 DShot600 bit waveform and connection-loss action unchanged.
- 2048 commands roundtrip checked in host unit tests at both DShot speeds.
- Sample validity indicates real CRC-checked ESC response, not a simulated value.
- PA0 high/low levels, reply timing and actual RPM verified with logic analyzer.
- Multi-motor recovery, fail-safe and other EtherCAT task regressions verified
  on a dedicated hardware test bench.

A green CI does not satisfy the last two acceptance gates.
