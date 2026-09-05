# LEFT steering encoder detect hardware checkpoint — 2026-09-06

## Real hardware evidence
- F103 APP_STLINK build/upload/verify PASS, firmware branch `v1`.
- LEFT boot ABI fix verified earlier on SWD: sensor port ABI, encoder configured, TIM4 encoder mode active, ARR=4095, DC current calibration complete.
- Clean reset-source capture after prior failed detect was `RCC_CSR=0x14000000` = software reset + pin reset flags; watchdog/brownout flags were not set.
- Persistent SRAM reset black-box now records reboot/update reasons and command/detect stages across NVIC reset.
- Real `COMM_DETECT_ENCODER=27` at 0.70 A reached the encoder algorithm and returned fail-closed sentinel `offset=1001`, `ratio=0`, `inverted=false` in ~25 ms on the last permitted actuation run.
- Persistent stage after that run was `steering=0xE1`, `encoder=0xEF`, proving ABI configuration passed and failure occurred inside the bounded electrical/encoder probe.
- Latest firmware includes detailed encoder fail codes for the next physical run and passed `ALL_FINAL_HOST_CHECKS_PASS`.
- Latest APP_STLINK upload verified OK. After explicit `reset run`, F411↔F103 communication recovered: FW_VERSION replied `6.00`, name `motor_left`; bridge RX counter rose 51→101 and VESC age returned to ~2.3 s at 1 Mbaud.

## Firmware changes
- Always materialize selected LEFT ABI encoder hardware configuration at boot even with blank/incompatible EEPROM.
- Replace detect/homing HAL tick waits with bounded DWT-cycle delays in critical steering/encoder paths.
- Add reset cause capture and reserved-SRAM reset black-box.
- Tag COMM_REBOOT and firmware-update NVIC reset paths.
- Add parser, steering-calibration, encoder-detect, and detailed failure-stage diagnostics.
- Preserve stock VESC `COMM_DETECT_ENCODER` reply shape and fail-closed behavior.

## Not yet physically proven
- Valid encoder offset/ratio/inversion detection.
- Phase-to-ABI matching success.
- Mechanical left/right hard-stop raw counts and span.
- Center midpoint and persisted calibration.
- Physical -30°, 0°, +30° commands.
- VESC Tool 0/180/360 → physical -30/0/+30 mapping.
- ROS/Web signed -30/0/+30 behavior and boot homing after valid calibration.

## Safety / next physical step
The next automated encoder actuation attempt was blocked by the platform safety safeguard, so it was not bypassed. Firmware is ready with detailed failure codes; the next permitted/manual `COMM_DETECT_ENCODER` run should be followed immediately by reading the persistent stage byte to identify the exact failing probe condition before changing control parameters.
