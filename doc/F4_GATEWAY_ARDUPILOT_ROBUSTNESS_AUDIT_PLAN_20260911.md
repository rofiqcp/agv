# F4 Gateway ArduPilot/DroneCAN Robustness Audit Plan
Date: 2026-09-11
Scope: /home/sirobo/agv/F4gateway + AP_Periph/AP_DroneCAN/AP_Bootloader reference

## Target architecture
- STM32F411CE application at 0x08008000, resident recovery bootloader in sectors 0-1.
- NEO3 Pro -> DroneCAN classic 1 Mbit/s -> MCP2515 8 MHz -> SPI2 PB13/14/15, INT PB10, CS PB12.
- TFT + XPT2046 remain isolated on SPI1.
- USB CDC is the PC/ROS transport; ESC remains outside F4.

## Design rule
DroneCAN transport semantics shall match ArduPilot/libcanard, but the CAN hardware driver cannot be byte-for-byte identical because ArduPilot normally uses native STM32 CAN while this gateway uses MCP2515 over SPI2.

## P0 findings
1. Bootloader maintenance/update loop has no independent watchdog; a flash/USB fault can leave recovery code alive forever.
2. USB CDC TX-complete callback calls gUsb.poll() from USB IRQ context, re-entering USBD transmit logic from its completion callback.
3. CAN RX overflow is still observable during long stress; no hang occurs, but zero-loss qualification is not yet achieved.
4. ROS->F4 ESCX/PERX/NAVX telemetry has sequence/age validation but no CRC/version/length envelope.
5. libcanard is compiled with CANARD_ENABLE_DEADLINE=0; F4 emulates deadlines around MCP mailbox instead of attaching deadlines to queued Canard frames.
## P1 findings
6. CAN protocol statistics are too aggregated. ArduPilot distinguishes OOM/internal/not-wanted/wrong-address/missed-start/wrong-toggle/unexpected-TID/short-frame/bad-CRC.
7. MCP2515 has only two RX buffers; polling must be scheduled by interrupt/freshness rather than fixed loop budgets alone.
8. GetNodeInfo/Param service traffic needs one centralized transaction manager with request deadline, expected response tuple, retry/backoff and stale cancellation.
9. Dynamic Node Allocation should persist UID->node mapping for deterministic reboots instead of relying only on transient first-allocation state.
10. Application watchdog is good, but bootloader->application health handshake is weaker than ArduPilot's early-watchdog/RTC FWOK model.
11. Current boot manifest verifies image CRC and vector, but has no board-id/version/git-hash/protocol-version fields and no optional signature/authentication.
12. Bootloader DATA records have offset ordering but no per-chunk CRC/readback confirmation; only END verifies the complete image.
13. USB RX/TX queues have drop counters but no explicit high-water/backpressure state exposed to the ROS contract.
14. USB session generation exists, but host commands do not all carry/verify a host session epoch or request ID.
15. ASCII SENS v2 records have CRC16, but compatibility records remain unprotected and should never be an authority source.
16. Fault/reset reason reporting should retain watchdog/hardfault/busfault/USB/CAN recovery cause across reset for post-mortem diagnosis.

## Already good / retain
- MCP2515 fixed 8 MHz, 1 Mbit/s, single sample, SPI2 isolated from HMI.
- Bounded SPI transfer and deferred recovery.
- Async MCP TX mailbox with physical deadline, abort and exponential backoff.
- Main-loop watchdog and Cortex fault handlers reset instead of hanging.
- USB dual-priority queues, bounded parser, endpoint-stall diagnostics.
- Power-loss-safe application manifest is invalidated before application erase and committed only after full CRC verification.
- App vector validation, flash bounds, transactional ST-Link provisioning and ROM-DFU fallback.
## Stage 1 - Embedded robustness core
A. Replace USB IRQ re-entry: completion ISR only advances state/sets a service flag; starting the next CDC packet happens in main-loop service context.
B. Add USB queue watermarks, RX overflow episode counter, host-session epoch and bounded command-response IDs.
C. Enable/port libcanard deadline semantics or add an equivalent F4 queue wrapper carrying absolute deadline per transfer; never transmit an expired frame.
D. Implement ArduPilot-like CAN RX error taxonomy and publish each class independently.
E. Add interrupt-driven MCP service hint from PB10, adaptive drain-to-empty loop with microsecond execution budget, RX overflow burst metrics and no fixed starvation.
F. Centralize CAN TX/service transactions: one outstanding service tuple per service class, response deadline, TID matching, retry/backoff, stale cancellation.
G. Make recovery hierarchical: RX resync -> transfer cleanup; error-passive -> TX stop/backoff; bus-off/TEC threshold -> MCP reset; SPI failure -> SPI2 reset; repeated recovery -> degraded state.
H. Add retained reset/fault record in RTC backup registers including reset reason, PC/LR for Cortex faults where safe, last CAN/USB health and boot count.
I. Add bootloader watchdog and pat it around USB receive, sector erase, flash program, CRC verify and maintenance loop.
J. Strengthen boot protocol with per-DATA chunk CRC + flash readback, image board-id, semantic version, git/build identity and protocol version in manifest.

Stage-1 acceptance:
- 30 min PC flood with GNSS/MAG/BARO active: no reset/hang, TEC/REC/EFLG healthy, zero USB parser starvation.
- CAN peer disconnect/reconnect 100 cycles: bounded recovery, no error-passive lock, no manual reset.
- Forced SPI2 timeout, USB stall and malformed host packets: deterministic recovery.
- Brownout/power loss at every boot update phase never boots a partial image.
## Stage 2 - Protocol parity and qualification
A. Make DroneCAN subscriptions/services protocol-equivalent to AP_Periph: exact DSDL IDs/signatures, TAO, TID/toggle, multi-frame CRC, DNA, GetNodeInfo, Param.GetSet, LightsCommand and NEO3 sensor set.
B. Add deterministic UID->node allocation cache and explicit expected-board fingerprint for CUAV_GPS board-id 1001.
C. Match ArduPilot stale-transfer cleanup cadence and classify only true accepted-transfer MISSED_START as error; ignored traffic stays ignored.
D. Add CAN node/transport statistics topic mirroring ArduPilot concepts: RX frames, ignored, OOM/internal, missed-start, wrong-toggle, unexpected-TID, short-frame, bad-CRC, TX frames/errors, bus-off, overflow, recovery.
E. Replace ROS->F4 telemetry with v3 envelope: magic/domain/schema-version/sequence/source-time/age/payload-length/payload/CRC16 or CRC32. Keep old parser read-only during migration, never as safety authority.
F. Add ACK/NACK transaction IDs for configuration/maintenance commands and idempotent command handling.
G. Mark every telemetry group with provenance, source epoch, freshness and validity; no compatibility line may overwrite a validated v2/v3 source.
H. Add boot/app protocol compatibility check so an incompatible application remains in recovery rather than booting.
I. Add optional signed-firmware path later; CRC remains integrity detection, signature provides authenticity.
J. Add soak/fault-injection qualification and freeze all thresholds from measured p99 service/queue timing.

Stage-2 acceptance:
- All active NEO3 Pro messages decode against generated DSDL fixtures and live hardware.
- LED brightness/color and parameter services work while all sensor streams remain fresh.
- 1 h mixed USB+HMI+DroneCAN soak: zero bus-off, zero unrecovered USB stall, no watchdog reset, no memory growth.
- Telemetry corruption/truncation/reorder/duplicate injection is rejected without changing authoritative state.
- Bootloader rejects wrong board, wrong CRC, partial image, incompatible schema and failed readback.

## Do not copy blindly from ArduPilot
- Native STM32 CAN mailbox/driver code: replace with MCP2515-specific bounded mailbox implementation.
- ChibiOS scheduler/thread primitives: preserve their semantics using cooperative F411 state machines.
- ArduPilot parameter subsystem: only implement the subset needed for gateway configuration and NEO service access.
