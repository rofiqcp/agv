# ESC F411 ↔ F103 Mixed-Sensor / Bootloader Validation — 2026-09-05

## Scope
- STM32F103RCT6 dual ESC FOC firmware (`hoverboard-firmware-hack-FOC`).
- STM32F411CE BlackPill gateway for VESC transport.
- LEFT motor: ABI incremental encoder, 4096 counts/rev, steering physical -30° / 0° / +30°.
- RIGHT motor: Hall sensor.

## Changes and findings
- Detect-All is mixed-sensor: LEFT encoder detection/alignment, RIGHT Hall detection, then R/L/flux identification for both motors.
- LEFT defaults to `SENSOR_PORT_MODE_ABI` + `FOC_SENSOR_MODE_ENCODER`; RIGHT remains Hall-only.
- Encoder startup alignment verifies physical ABI motion before arming closed-loop torque.
- F103 VESC transport contract is 2,000,000 baud end-to-end.
- Bootloader now configures HSI/2 ×16 = 64 MHz SYSCLK and APB1 = 32 MHz before USART3 init; verified `USART3_BRR=0x10` for 2 Mbaud.
- Bootloader initial installation via ST-Link succeeded and verified.
- App/staging/update-meta regions were erased for recovery testing; EEPROM region at `0x0803F000` was preserved.
- F411 source gateway already uses `kBaud = 2000000`; the currently flashed F411 image is older (reported 115200) and its CDC endpoint became stuck after a diagnostic stress pattern.

## Validation completed
- `ALL_FINAL_HOST_CHECKS_PASS`.
- Encoder ABI runtime PASS: mechanical angle → electrical angle → eRPM/tachometer chain validated.
- Hall wiring permutation PASS across 12 permutations/reversal cases.
- Three mechanical revolutions forward/reverse PASS for Hall runtime tests with zero sequence/period rejects.
- EEPROM dual configuration persistence PASS.
- Builds PASS: `APP_STLINK`, `APP_USART_PC`, `APP_F411`, `BOOTLOADER_STLINK`.
- F103 bootloader runtime verified by SWD: `SystemCoreClock=64 MHz`, APB1 divider `/2`, USART3 `BRR=0x10`, CR1 TX/RX enabled.

## Hardware state / next action
- Both UART directions are electrically connected: F103 TX→F411 RX is driven, and F411 TX→F103 RX was observed directly on PB11 via SWD.
- F103 is intentionally left in bootloader/recovery state with application slot erased, so motors cannot run until APP_F411 is transferred.
- Required next physical action: reset the BlackPill F411 once so its CDC recovers; then flash the current F411 firmware (2 Mbaud gateway), upload `APP_F411` through F411, verify FW_VERSION/telemetry/faults, run Detect-All, and perform staged real-motion tests.
- Do not command real motor torque before F411 transport and F103 telemetry are confirmed healthy.
