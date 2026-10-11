# DShot600 ESC eRPM receiver - schematic review and implementation status

## Hardware mapping verified against supplied schematics (2026-10-11)

Files reviewed: **H750芯片原理图.pdf** and **AX58100原理图.zip / SCH_Schematic1_1-P1_2026-08-25.pdf**.

| Group | Timer | Channel | MCU pad | External net |
| --- | --- | --- | --- | --- |
| 1 | TIM2 | 1 | PA0 | DSHOT1_TIM2_CH1 |
| 1 | TIM2 | 2 | PA1 | DSHOT1_TIM2_CH2 |
| 1 | TIM2 | 3 | PA2 | DSHOT1_TIM2_CH3 |
| 1 | TIM2 | 4 | PA3 | DSHOT1_TIM2_CH4 |
| 2 | TIM3 | 1 | PA6 | DSHOT2_TIM3_CH1 |
| 2 | TIM3 | 2 | PA7 | DSHOT2_TIM3_CH2 |
| 2 | TIM3 | 3 | PB0 | DSHOT2_TIM3_CH3 |
| 2 | TIM3 | 4 | PB1 | DSHOT2_TIM3_CH4 |

The AX58100 carrier exposes the signals at headers **J18** (TIM2) and **J17** (TIM3). Schematic shows shunt ESD diodes to GND on these channels and **no obvious one-way digital buffer**. That establishes potential bidirectional wiring, NOT proven electrical signal integrity.

**Crucial:** verify net polarity, physical header numbering and 3.3 V input levels on the actual board with an oscilloscope before any motor tests. Do not assume every Flycolor Raptor 35A firmware variant supports DShot eRPM.

## Code currently implemented

- Legacy `DSHOT` application (type=4) and its PWM/DMA output left unchanged.
- Separate read-only `DSHOT_RPM` application (type=16) for a selected `dshot_id` (1 or 2).
- 21-bit NRZI/GCR + inverted checksum decoder, mapped to a 12-bit period payload.
- Four channels of feedback per task, *invalid by default*, with 100 ms freshness check.
- Return format (9 bytes): `4 x uint16 little-endian eperiod payload` followed by `uint8 valid_mask`.
  - `0xFFFF` = no valid sample.
  - `0x0FFF` + valid bit = stopped rotor (0 eRPM).
  - `valid_mask==0` MUST NOT be interpreted as stopped rotors.

## Pin-mux / DMA resources confirmed from firmware

Cross-checked `Core/Src/tim.c` and `Core/Src/dma.c`:

| Group | Capture-capable timer | Pin AF | Existing DMA |
| --- | --- | --- | --- |
| TIM2 CH1/2/3/4 | TIM2 (32-bit) | PA0/1/2/3, AF1 | DMA1 Stream0/1/2/3 |
| TIM3 CH1/2/3/4 | TIM3 (16-bit) | PA6/7 PB0/1, AF2 | DMA1 Stream4/5/6/7 |

**As shipped, all eight outputs are configured `GPIO_MODE_AF_PP`, `GPIO_NOPULL`.**
The eight DMA streams are initialized `DMA_MEMORY_TO_PERIPH` for DShot TX.
That proves there is *no active receive configuration* even though the MCU pins have TIM input capture capability.

A bidirectional implementation must explicitly provide and validate:
- non-contentious idle-high wire electrical configuration (typically open-drain output plus suitable pull-up), accounting for connector ESD capacitance and ESC input characteristics;
- inverted DShot TX waveform and inverted TX CRC (see `prepare_bidir_command()`, protocol helper only);
- TIM channel / GPIO turnaround and peripheral-to-memory DMA edge capture; potentially reuse TX DMA streams only when TX is complete, with careful exclusive ownership;
- 750 kbit/s receive bit period (1.333 us) and approximately 30 us turnaround; prove timing and edge jitter with a logic analyzer;
- D-Cache coherency and buffer memory placement for STM32H7 DMA;
- a receiver completion callback invoking the cache only after GCR+CRC validation.

The protocol host tests and firmware/task code do **not** establish that the timing-sensitive hardware receiver works.

## **Not yet implemented: hardware receiver**

The receive side needs real GPIO/timer/DMA work and **must not be enabled before oscilloscope validation**:

1. Confirm ESC firmware supports *bidirectional DShot telemetry* independently of the ESC's *3D reverse mode*.
2. Implement inverted TX levels and correct inverted DShot TX checksum; inverted DShot is NOT obtained by setting the one-bit UART telemetry-request flag.
3. Configure idle-high, non-contentious electrical signaling (open-drain / external or appropriately validated pull-up) and return the pin to input promptly after TX. Never drive the line against the ESC output.
4. Capture 21-bit, 750 kbit/s, NRZI level sequence on all active channels, with the required turnaround after the 16-bit TX frame. TIM2 and TIM3 already operate in output PWM/DMA mode and must be safely switched or supplemented with capture resources.
5. Validate alignment, timing, CRC and 100 ms validity; only then call `aim::ecat::task::pwm::dshot_rpm_cache::accept_captured_frame(...)` from a verified receive-complete path.
6. Test each channel with a logic analyzer, stationary motor, low-RPM bench, reversal transitions and telemetry disconnect.
7. Recheck this board's connector ESD capacitance, pull-up rise time and ESC signal levels on the actual PCB.

**Never flash this branch expecting actual RPM.** Until the hardware receive path is wired, enabled and tested, the read-only task always reports invalid channels.

## Bringup and safety

- Read-only task does not arm or spin the motor.
- Configure the legacy DShot task's connection-lost action to `RESET_TO_DEFAULT`, and select 0 for initial DShot command.
- Use a physical kill-switch / safe bench; commands MUST remain disarmed during initial waveform tests.
- Any missing/stale response MUST clear the valid bit; there is no fake RPM or substitution with commanded speed.
- eRPM feedback gives *magnitude*, not reliably signed rotation direction in 3D mode. Direction should be established separately using a qualified direction estimator, especially during zero crossing.
- For six motors use two 4-channel groups, leaving two physical output channels unused.

## EtherCAT task configuration encoding

The firmware's task constructor reads `dshot_id` as **one byte**, so master YAML must use `!uint8_t`, not `!uint16_t`. Each `DSHOT_RPM` task occupies 2 config bytes including task type; each original DShot TX task occupies 5 config bytes including task type. For two TX and two RX tasks, `sdo_len=1+2*5+2*2=15`.

The current slave PDO buffer is only **80 bytes per direction**. Two RPM tasks consume **18 bytes** of slave-to-master data. Check total read PDO budget before combining six IMUs or other sensors. Do not silently exceed this limit; a larger PDO branch may be required.

Protocol: https://betaflight.com/docs/development/API/Dshot
