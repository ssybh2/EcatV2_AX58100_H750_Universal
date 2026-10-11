# H750 / AX58100 DShot RPM hardware review (development branch)

> **Status (2026-10-11): protocol + PDO plumbing only. No receiver is enabled.**
> **DO NOT flash this branch expecting valid RPM or fly an unrestrained robot.**
>
> This note was checked against the user's uploaded **H750 chip schematic** (A2,
> created 2026-08-26) and **AX58100 schematics** (three PDFs, 2026-08-25),
> together with the repository's STM32CubeMX `.ioc`, `Core/Src/tim.c`,
> and `Application/Task/dshot.cpp`.

## 1. Physical timer and GPIO map

| Schematic label | Timer channel | STM32H750 GPIO | Existing DMA stream |
|---|---|---|---|
| DSHOT1_TIM2_CH1 | TIM2 CH1 | PA0 | DMA1 Stream0 |
| DSHOT1_TIM2_CH2 | TIM2 CH2 | PA1 | DMA1 Stream1 |
| DSHOT1_TIM2_CH3 | TIM2 CH3 | PA2 | DMA1 Stream2 |
| DSHOT1_TIM2_CH4 | TIM2 CH4 | PA3 | DMA1 Stream3 |
| DSHOT2_TIM3_CH1 | TIM3 CH1 | PA6 | DMA1 Stream4 |
| DSHOT2_TIM3_CH2 | TIM3 CH2 | PA7 | DMA1 Stream5 |
| DSHOT2_TIM3_CH3 | TIM3 CH3 | PB0 | DMA1 Stream6 |
| DSHOT2_TIM3_CH4 | TIM3 CH4 | PB1 | DMA1 Stream7 |

All eight signals appear on the H750-to-AX58100 inter-board connectors
and on external DSHOT headers in AX58100 schematic page 1.

**Important pre-existing mismatch:** `Application/Task/dshot.cpp` maps
`dshot_id=1` to `htim3` and `dshot_id=2` to `htim2`, **opposite** to
the printed schematic net *group* names. We deliberately have **not**
reversed the mapping in this branch: doing so without confirming existing
cabling/configuration could spin the wrong motors. The new RPM task uses
the *software* `dshot_id` selector, so this discrepancy must be resolved
with an unpowered continuity check and a propeller-removed test before
the actual RX capture layer is enabled.

## 2. Hardware feasibility and missing receiver

- Existing hardware routes DSHOT signals to MCU timer GPIOs; this makes
  reception physically plausible but is **not proof** of electrical or
  timing compatibility with a specific Flycolor Raptor 35A ESC.
- Current TIM2/TIM3 DMA transfers are **MEMORY_TO_PERIPH** into TIM CCR
  registers to generate 600 kbit/s output waveforms.
- Bidirectional DShot needs inverted **TX signal levels**, inverted TX
  checksum, then a **turnaround / high-impedance input phase** to sample
  the ESC's 750 kbit/s, 21-bit GCR telemetry reply.
- TX and RX must **not** drive the line simultaneously. Confirm MCU pad
  voltage, ESC reply voltage, idle state, pull-ups, GPIO mode changes,
  timer/DMA ownership, timing and interrupt priority on an oscilloscope
  before enabling the driver.
- A safe implementation should acquire the signal with input capture /
  appropriately timed sampling DMA, decode/check CRC, and pass the
  normalized 21-bit frame to
  `aim::ecat::task::pwm::dshot_rpm_cache::accept_captured_frame()`.
  The hook is intentionally **never called** in this phase.
- Use group/channel-specific timing and isolate TIM2 from TIM3. Existing
  DMA1 streams 0-7 are already assigned to DSHOT PWM *transmit*, so RX
  requires a resource allocation plan; do not assume eight concurrent RX
  DMAs are free.

## 3. Current standalone PDO/API status

- Legacy task **DSHOT (type 4)**: unchanged four `uint16` command words
  written by the master per task.
- New opt-in **DSHOT_RPM (type 16)**: separate **read-only** task with one
  `uint8 dshot_id` configuration arg (1 or 2).
- New read PDO per group: four `uint16` **12-bit e-period payloads** in
  little endian followed by one `uint8` `valid_mask` (**9 bytes**).
  `0xFFFF` means INVALID, `0x0FFF` means valid stopped motor, and bit i
  in valid_mask is set only after a CRC-checked frame newer than 100 ms.
- Host message `custom_msgs/msg/ReadDSHOTRpm` contains 4 unsigned
  electrical RPM **magnitudes**, an explicit validity bitmask, and a
  dshot_id. **It is not signed mechanical RPM.**
- Master task must be configured with
  `sdowrite_task_type: !uint8_t 16`,
  `sdowrite_dshot_id: !uint8_t 1`, an `pub_topic` and correct `pdoread_offset`.
  Also increment `task_count` and `sdo_len` consistently; reserve a
  9-byte RX-PDO slot per four-channel group and check the total 80-byte
  slave input limit.
- When no receiver exists or any frame goes stale, `valid_mask=0`;
  **zero eRPM alone must never be misinterpreted as verified zero speed**.

## 4. ESC firmware compatibility

BLHeli **3D mode** controls motor reversing; it does **not** enable eRPM
feedback. Single-wire bidirectional DShot requires firmware that supports
the inverted DShot transmit/return protocol. Confirm Flycolor Raptor 35A
exact model and BLHeli/Bluejay/AM32 firmware revision using its configurator,
not merely the name printed on the ESC. Separate UART telemetry may be a
lower-risk fallback if physically available.

For the Rolling-ZLT MPC, unsigned eRPM is **insufficient** to infer the
actual rotation direction through a reversal. Do not create signed
`motor_omegas_rad_s` by copying the command sign during a transition.

## 5. Tests and remaining acceptance gates

1. Host protocol unit test:
   `g++ -std=c++17 -O2 -Wall -Wextra -Werror -IApplication tests/dshot_rpm_codec_test.cpp -o /tmp/dshot_rpm_test && /tmp/dshot_rpm_test`.
2. Compile H750 with STM32 toolchain and master with ROS 2 Humble (not
   executed in the ChatGPT environment).
3. With **propellers removed**, verify the existing DSHOT command task
   still works and the new RPM PDO consistently reports invalid.
4. Confirm ESC firmware, use scope/logic analyzer to measure inverted
   TX, line turnaround and actual 750 kbit/s reply.
5. Only then implement RX capture and test CRC acceptance, sample age,
   channel order, polarity and 3D forward/reverse behavior.
6. Before real rolling: add independent arming, hardware stop, telemetry
   loss interlock and constrained thrust-to-DSHOT calibration.

References:
- https://betaflight.com/docs/development/API/Dshot
- https://betaflight.com/docs/wiki/guides/current/Dshot
