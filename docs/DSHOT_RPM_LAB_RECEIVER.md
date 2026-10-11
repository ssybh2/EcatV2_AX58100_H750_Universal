> Historical DShot600 lab design. For the user-tested DShot300 implementation,
> see [DSHOT300_BIDIR_INTEGRATION.md](./DSHOT300_BIDIR_INTEGRATION.md).
> Only the new branch's *separate bench ELF* contains experimental RX.

# H750 DShot600 RPM: opt-in one-channel hardware receive prototype

**STATUS: EXPERIMENTAL / NOT VALIDATED ON ESC HARDWARE / DO NOT FLY.**
Do not overwrite your working STM32H750 firmware with this lab build until
signal integrity, start-bit timing, watchdogs, and DMA turnaround have been
verified on a propeller-free oscilloscope bench.

## Target and backwards compatibility

- Base branch: AIMEtherCAT-compatible main + separate DSHOT_RPM PDO task type 16.
- Default compiler option: `AIM_DSHOT_RPM_LAB_RX=OFF`. Legacy DSHOT
  PWM1, checksum and uninterrupted DShot commands are unchanged.
- New portable decoder: `Application/dshot_rpm_edge_decode.hpp`.
  Can reconstruct 21-bit reply from rising/falling edge timestamps, handle
  TIM2 32-bit and TIM3 16-bit counter wrap, then check GCR/CRC.
- New TX helper: `Application/dshot_rpm_tx.hpp`.
  Generates inverted-CRC frames and timer compare values for inverted PWM2.
- Lab hardware path is compiled ONLY with `-DAIM_DSHOT_RPM_LAB_RX=ON`.
  It is not enabled in the default artifact.

## Physical experimental path (only 1 of 4 channels decoded)

The temporary hardware test path uses the **existing software dshot_id=2**
which selects TIM2, physical PA0/PA1/PA2/PA3.

1. Send four inverted DShot600 TX frames using TIM2 PWM2 + existing DMA1
   stream 0-3 to the same four outputs. There is no new command SDO layout.
2. Wait until all four TX DMA callbacks have completed, allow the tail cells
   to finish, and release all four GPIOs to input mode.
3. Listen to PA0 (TIM2 CH1 / motor channel 1) on EXTI0 both edges.
   Timestamp edges with CPU DWT CYCCNT; preempting interrupts and signal
   rise-time are **unverified**. This is intentionally a single-channel
   EXTI feasibility test, **not** a finalized timer input-capture DMA driver.
4. On the next EtherCAT command period, decode edge timestamps at 750 kbit/s
   using `SystemCoreClock`, check GCR/CRC, and update RPM cache
   (`dshot_id=2, channel=0`) if valid. Restore all four GPIOs to TIM2 AF1
   before the next PWM frame.
5. Only bit 0 of `valid_mask` can become valid; CH2..4 remain invalid.

Clock phase, interrupts, EXTI jitter, pin voltage, TIM2 PWM2 polarity,
release timing, pull-up drive, motor arming, and packet loss have **not**
been measured on the actual H750 PCB and Flycolor ESC. A successful
CI build **does not prove** usable RPM and does not authorize flight.

## Important safety boundaries

- Keep propellers REMOVED and physically isolate ESC power.
- Verify the ESC actually supports inverted bidirectional DShot600
  (the `Bidirectional 3D` motor reversal setting is unrelated).
- With power disconnected, check the DShot wire does not have one-way
  buffers or incompatible voltage translation.
- Instrument PA0 and PA1..3 with a scope; ensure the STM32 stops
  driving the ESC return line BEFORE reply edges occur.
- Before connecting ESC power, verify idle-high levels and inverted
  TX checksum at zero-throttle only.
- An external emergency power disconnect is required.
- Fail invalid/stale/no CRC: `valid_mask=0`; never substitute a command.
- DO NOT deploy this lab firmware while a rolling robot is in motion.

## Build

Default, backwards-compatible firmware:

```bash
cmake --preset Release -DAIM_DSHOT_RPM_LAB_RX=OFF
cmake --build build/Release --target EcatV2_AX58100_H750_Universal
```

Compile-only opt-in hardware experiment:

```bash
cmake --preset Release -B build/Lab -DAIM_DSHOT_RPM_LAB_RX=ON
cmake --build build/Lab --target EcatV2_AX58100_H750_Universal
```

The main GitHub Action publishes only the default firmware artifact.
The lab code is compiled as a separate CI step, but NO lab binary is uploaded
for accidental flashing.

## Remaining engineering gates

- Capture PA0 real transitions on a logic analyzer and compare software
  timestamps with a 750 kbit/s reply.
- Replace EXTI proof of concept with TIM input-capture DMA (latency and
  jitter guaranteed) and stagger or synchronize TX/RX on all 4 channels.
- Add both channel groups (up to eight; six used) with collision-free
  timer/DMA resource arbitration and D-Cache-coherent receive memory.
- Verify encoder stalls/zero-speed, sample freshness, signed direction
  ambiguity during 3D reversals, loss of EtherCAT, and safe disarm.
- Run full baseline EtherCAT and DShot feature regression on hardware.

Sources:
https://betaflight.com/docs/development/API/Dshot
https://www.st.com/resource/en/reference_manual/dm00314099-stm32h742-stm32h743753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
