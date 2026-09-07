# Deep Audit VESC vs STM32F103 Hoverboard FOC

Tanggal audit: 2026-09-08

## Scope
- Upstream: `/home/otomasi/ros/vesc/motor/mcpwm_foc.c`
- Upstream: `/home/otomasi/ros/vesc/motor/mc_interface.c`
- Target: `/home/otomasi/ros/hoverboard-firmware-hack-FOC/Src/motor/mcpwm_foc.c`
- Target: `/home/otomasi/ros/hoverboard-firmware-hack-FOC/Src/motor/mc_interface.c`
- Supporting target: `foc_math.c/.h`, `mcconf_default.h`, `mcconf_serial.c`, `config.h`

## Hardware constraints verified
- MCU STM32F103RCT6, Cortex-M3, 72 MHz, no FPU.
- RAM 48 KiB; current build uses 31,516 B (64.1%).
- Application linker region is only 120 KiB from 0x08002800.
- Current image is 121,576 B, leaving about 1,304 B effective app-flash margin.
- PWM/ADC frame is 16 kHz.
- Heavy regulator cadence is divided by 6 and staggered: LEFT slot 0, RIGHT slot 1, about 2.667 kHz per motor.
- LEFT feedback is ABI encoder or Hall; RIGHT feedback is Hall only.
- Current hardware is two-shunt/low-side style, so generic VESC sampling modes cannot be copied blindly.

## Executive finding
The target is no longer a minimal VESC clone. It is a board-specific, dual-motor FOC implementation that preserves important VESC 6.00 command/config semantics while replacing the upstream floating-point hot path with Cortex-M3 fixed-point/LUT logic. The core current loop, Hall/ABI feedback, VESC duty/current/speed/position commands, protection, telemetry and commissioning are substantially implemented. The largest remaining risks are not HFI or sensorless operation; they are incomplete VESC limit semantics and MC-config fields that can be accepted on the wire even though they have no runtime effect.
## Already optimized correctly for STM32F103
1. `foc_math.c` uses a 257-entry Q15 sine LUT with linear interpolation. Park, inverse Park, Clarke and centered SVPWM are integer/fixed-point.
2. The ADC ISR does not call `sinf/cosf/atan2f/sqrtf/powf/logf` or software float/divide helpers. Current ELF disassembly audit found no forbidden helper in the main ISR ranges.
3. Bus-voltage reciprocal conversion for current PI is refreshed outside the hot loop and cached as integer coefficients.
4. Current PI state is kept in physical-voltage fixed point, with D-axis priority and Q-axis voltage-circle headroom before final vector limiting.
5. Encoder phase/count, mdeg/count and RPM coefficients are precomputed outside ISR; TIM4 does hardware quadrature counting.
6. Hall GPIO is sampled once per PWM frame. Rolling majority, debounce, edge filtering, period estimation and interpolation are implemented without repeated GPIO sampling/trig.
7. Hall RPM/division work is mostly edge-driven rather than executed every 16-kHz frame.
8. Speed and position PID coefficients are precomputed and use multiply/shift in the regulator path.
9. R/L commissioning captures integer sufficient statistics in ISR and performs regression/division/float outside ISR.
10. Ortega observer is diagnostic only and runs outside ISR at about 200 Hz; it cannot disturb the torque-control deadline.
11. LEFT/RIGHT heavy PI calculations are staggered so they do not execute in the same ADC frame.
12. DWT cycle counters and overrun counters exist for real hardware timing verification.
13. VESC input-current, watt, low-battery and FET-temperature limits already feed the actual `Iq` envelope; these are not telemetry-only fields.
14. Absolute phase-current, DC-link current, VIN and FET-temperature hard protection remain at high rate.
15. Startup/driven/high-impedance current offset handling is explicitly adapted to the hoverboard low-side-shunt hardware instead of blindly using generic VESC offset fields.
16. Physical duty 1.0 is mapped to the verified safe modulation ceiling, preserving the 110-count ADC/PWM margin rather than driving mathematical rail duty.
17. Config changes that alter pole count or encoder phase mapping release the bridge before publishing the new state, preventing one-frame electrical phase jumps.
18. EEPROM layout is append-only and stores implemented fields without moving historical App Config addresses.
## High-value VESC parity gaps
### P0-A: truthful MC-config semantics
`mcconf_serial.c` correctly keeps the complete VESC 6.00 wire layout, but many deserialized fields have zero runtime references. A SET_MCCONF can therefore make an unsupported option appear enabled in GET_MCCONF until reboot. Add one normalization layer after deserialize/before publication: every wire field must be either implemented, persisted, or canonicalized to a deterministic disabled/fixed hardware value.

Examples to canonicalize until implemented: HFI family, sensorless/open-loop transition fields, MTPA, field weakening, phase-filter options, current/control sample modes, saturation compensation, temperature compensation, generic motor-temperature/DRV8301/BLDC/DC fields, and unsupported speed-source selections.

### P0-B: global ERPM current envelope
Upstream `mc_interface_update_override_limits()` uses `l_erpm_start` to reduce available motoring current as actual ERPM approaches `l_max_erpm/l_min_erpm`. Local code clamps SET_RPM but does not apply the same global torque envelope to CURRENT, DUTY, POS and other torque-producing modes. Implement precomputed signed ERPM thresholds and Q15 linear current scaling in the regulator path.

### P0-C: battery regen over-voltage derating
`l_battery_regen_cut_start/end` are wire-visible but currently unused. Add ADC-domain thresholds precomputed outside ISR and reduce regenerative input-current limit smoothly to zero before `l_max_vin`. Keep `l_max_vin` as the final hard over-voltage fault.

### P0-D: duty PI bus-voltage scaling
Upstream duty down-ramp PI multiplies gains by `1/Vbus`. Local code precomputes gains around fixed `MCCONF_DUTY_PI_BUS_NOMINAL_V=42.5 V`. Refresh duty PI coefficients whenever the slow battery-voltage scale changes, outside ISR, then keep the ISR multiply/shift-only.

### P0-E: VESC current-brake transition state machine
Upstream temporarily commands duty zero/phase short around speed or Vq sign changes and holds it for a bounded number of samples before returning to active current braking. Local CURRENT_BRAKE directly flips signed Iq from feedback direction. Add the small fixed-point/state-flag transition logic to avoid discontinuities near zero/reversal.

### P0-F: `mc_interface_init(reset_conf)` contract
Local `mc_interface_init` ignores `reset_conf`; upstream uses it to choose default/store versus load. Do not fix this function in isolation because current boot order calls `mc_interface_init(true)` before EEPROM initialization and loads EEPROM later in `Input_Init`. Align the caller and initialization ownership together, or explicitly rename/document the local split-init contract.
## Medium-priority VESC parity gaps
1. `s_pid_kd_filter` is serialized but the local speed PID differentiates raw error. Implement a Q16 derivative LPF; current default Kd=0 hides this mismatch today.
2. `s_pid_speed_source`/related speed-source selections are not runtime-selectable. Canonicalize to the one real Hall/ABI speed estimator unless a second estimator is actually added.
3. `l_duty_start` is unused. Upstream uses it to reduce current authority as actual duty approaches the voltage ceiling. Implement with precomputed permille threshold if high-speed duty saturation matters.
4. `l_temp_accel_dec` is unused. Upstream can start motoring thermal derating earlier than braking thermal derating. This is useful for preserving braking authority but not mandatory for current 60/65 C hard envelope.
5. `foc_start_curr_dec` and `foc_start_curr_dec_rpm` are unused. Add only if low-speed launch current needs upstream behavior; current Hall/steering tuning already has project-specific startup handling.
6. `l_in_current_map_start/filter` are unused. The local model-based input-current limiter already predicts `Ibus` from modulation and Iq. Add measured-current mapping only after validating the DC-shunt measurement under switching/regen.
7. `l_max_erpm_fbrake` and `l_max_erpm_fbrake_cc` are not implemented. Decide whether this AGV needs VESC full-brake-at-overspeed behavior before adding it.
8. `l_additional_faults` is not implemented. Add only supported overspeed/underspeed fault bits and canonicalize unsupported bits.
9. Public API parity is incomplete: tacho, duty-set, filtered current, sampling-frequency, Ah/Wh and fault-string wrappers are absent even when underlying state often exists. Add wrappers only when consumed by protocol/ROS/tests; do not spend flash for unused API surface.

## Intentionally different / should remain board-specific
- LEFT incremental ABI requires boot alignment because there is no absolute index. This is not a generic VESC absolute encoder and should remain explicit.
- Steering hard-stop span detection/calibration is project-specific and should not be forced into standard VESC Detect-All semantics.
- RIGHT is Hall-only and LEFT sensor port is physically multiplexed Hall/ABI; generic sensor-port combinations are invalid hardware states.
- Invalid/skipped Hall states release closed-loop torque because there is no sensorless observer fallback. This is more conservative than upstream and is appropriate for this target.
- R/L detection is an F103 step-response/capture method rather than upstream HFI-based measurement; keep `_f103` naming where algorithms genuinely differ.
- Generic current/voltage offset configuration is not authoritative because the board has hardware-specific startup, driven and high-Z offset states.
- Generic overmodulation must not override the verified 96% physical scale and 110-count PWM/ADC safety margin.
## Features deliberately not recommended on the current F103 image
- HFI V1..V5 and HFI-start modes: high CPU/flash complexity and no present requirement with Hall/ABI feedback.
- Full sensorless observer as the active rotor-angle source: current Ortega implementation should remain diagnostic until a fixed-point observer is independently timing- and stability-validated.
- MTPA: low expected benefit for this hoverboard/SPM application relative to complexity.
- Field weakening: currently disabled by hardware profile; do not enable until voltage/current model, overspeed protection and thermal envelope are complete.
- Generic phase filters/current sample mode/control sample mode: these depend on board-specific analog/PWM topology and should be fixed/canonicalized instead of user-selectable.
- FOC audio: unnecessary because the board already has a separate buzzer and flash is constrained.
- BLDC/DC motor-type switching, DRV8301 controls and analog sin/cos encoder options: incompatible with this fixed FOC hoverboard hardware profile.
- BMS/GNSS/general VESC RTOS statistics infrastructure: add only if a real project consumer exists.

## Optional P2 performance enhancement after flash is freed
A fixed-point current-controller decoupling mode can be considered after P0/P1: cross-coupling (`omega*L*I`) and BEMF (`omega*lambda`) feed-forward at the 2.667-kHz regulator cadence. Precompute motor-model scales outside ISR and use bounded 32x32->64 multiplies plus shifts. Do not implement software-float decoupling in the ADC ISR.

A lightweight Hall/encoder-observer fusion can be studied later, but it is a separate control-validation project. The present diagnostic observer at 200 Hz must not simply be promoted to control feedback.

## Implementation order
1. Add config capability/normalization table and canonicalize unsupported fields.
2. Add tests that prove unsupported SET_MCCONF fields cannot remain falsely enabled.
3. Add ERPM current derating and optional additional overspeed faults.
4. Add battery regen-cut derating before over-voltage fault.
5. Make duty down-ramp PI use cached live-Vbus normalization.
6. Add VESC-like CURRENT_BRAKE zero-duty transition state.
7. Align `mc_interface_init(reset_conf)` and boot caller ownership without changing EEPROM addresses.
8. Add speed-PID D filter and canonical speed source.
9. Add `l_duty_start`; then evaluate `l_temp_accel_dec` and startup-current reduction.
10. Add only API wrappers proven necessary by VESC protocol/ROS/diagnostics.
11. Free at least 4-8 KiB application flash before attempting decoupling or observer-fusion features.
12. Re-profile hardware DWT cycles after each control-path feature; never batch several ISR changes without timing evidence.
## Verification gates for every phase
- Build `pio run -e APP_F411` must stay warning/error free.
- Link against the real 120-KiB app region, not the misleading 256-KiB PlatformIO percentage.
- Static ISR scan must find no software float/trig/sqrt/atan2 and no `__aeabi_*div` helpers in `motor_control_step`, ADC handler and DMA ISR.
- Host numerical tests must compare new limit equations against upstream VESC equations over positive/negative ERPM, motoring, regen, VIN, duty and temperature grids.
- MC-config test: GET -> modify -> SET -> GET -> store -> reboot/load -> GET. Supported fields must persist and alter runtime coefficients; unsupported fields must return canonical disabled/fixed values.
- Fault tests must cover under/over-voltage, phase over-current, DC over-current, FET over-temp, invalid Hall sequence and timeout recovery.
- Brake tests must cover positive speed, negative speed, zero crossing and command-direction reversal without torque sign chatter.
- Encoder tests must cover unsynced boot, alignment failure, inversion, ratio/offset update and release-before-reconfigure.
- Hall tests must preserve valid 1..6 states, reject 0/7, reject non-adjacent transitions and verify interpolation at configured ERPM threshold.
- Telemetry at 50 Hz must remain independent from regulator cadence and must not add blocking work to ISR.
- DWT timing must record worst-case LEFT, RIGHT, Hall edge, encoder update, current fault and brake-transition paths.
- Physical Detect Hall/Encoder/Detect-All is excluded from automatic regression because it energizes motors and steering has mechanical stops; run only under an explicit hardware test procedure.

## Current audit verdict
- Core FOC architecture: PASS for board-specific F103 fixed-point design.
- ISR/LUT optimization: PASS, with current binary showing no heavy software math helpers in hot ISR ranges.
- VESC 6.00 wire compatibility: strong but not equivalent to full runtime feature compatibility.
- Current/input/watt/battery-low/FET limits: substantially implemented.
- ERPM-global, regen-overvoltage, duty-start and several override-limit semantics: incomplete.
- Hall/ABI feedback: robust board-specific implementation, not full VESC observer-fused sensor stack.
- `mc_interface` API: intentionally reduced; missing wrappers are not automatically missing motor behavior.
- Most important architectural fix: make every VESC Tool field truthful through implemented-or-canonicalized-disabled semantics.
- Current flash margin is too small for broad upstream feature copying; safety/parity work must be compact and measured.