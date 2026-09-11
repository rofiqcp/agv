# NEO3 Pro / MCP2515 Zero-Loss Robustness Plan
Date: 2026-09-11
Scope: /home/sirobo/agv/F4gateway + ROS stmf4, compared with ArduPilot CANIface/AP_DroneCAN and PX4 UAVCAN drivers.

## Goal
Nominal operation must show zero silent packet loss. Under overload/fault, every loss must be explicitly counted and sensor authority must fail closed; no hidden stale data.

## Reference architecture confirmed
Both ArduPilot and PX4: CAN RX ISR reads/releases HW FIFO immediately -> timestamped software RX queue -> protocol/libcanard processing later. Current F4 fast-ISR architecture now follows this model conceptually.

## P0-A: Make RX ownership deterministic
1. Keep PB10 EXTI10 as highest CAN RX event source.
2. ISR only: READ STATUS/READ RX BUFFER, timestamp, push raw FIFO, release MCP RX buffer.
3. No libcanard/DSDL/USB/printf/DNA/recovery in ISR.
4. Raw FIFO stays 128 entries initially; instrument occupancy histogram and oldest-frame age.
5. Replace shared raw_can_count critical sections with SPSC head/tail semantics + barriers so producer ISR and consumer main do not globally mask IRQs.
6. Make ISR/main shared SPI flags and queue indices explicitly volatile/atomic-safe.

## P0-B: Remove SPI2 contention against RX
1. RX interrupt always has priority over non-RX MCP operations.
2. Stop polling TXB0CTRL every main-loop pass; use MCP TXnIF completion interrupts/events.
3. Use all MCP2515 TXB0/TXB1/TXB2 mailboxes with per-mailbox deadline/state, like ArduPilot pending_tx[].
4. Cache TEC/REC/EFLG in periodic health service; publishHardware must never re-read MCP just for telemetry.
5. Coalesce MCP register reads where possible and bound every non-RX SPI transaction.
6. If PB10 asserts while non-RX SPI owns bus, finish the current tiny transaction only, then immediately drain RX before any next SPI operation.

## P0-C: Improve ISR timing fidelity
1. Replace HAL_GetTick() millisecond RX timestamp with monotonic microsecond timestamp derived from DWT/SysTick rollover-safe clock.
2. Store timestamp_us in RawCanSlot.
3. Feed exact timestamp_us to canardHandleRxFrame.
4. Measure EXTI-entry -> first CS-low latency, frame-read duration, ISR total duration, SPI-busy deferral duration/max.
5. Acceptance: p99 event-to-buffer latency < one worst-case CAN frame time; max stays safely below two-buffer overrun window.

## P0-D: Make loss accounting mathematically complete
Add counters/invariants for each boundary:
- wire/MCP: RX0OVR, RX1OVR, EFLG, TEC/REC, IRQ events.
- MCP->RAM: hw_frames_read, fast_isr_frames, fallback_frames, spi_busy_deferrals, irq_budget_exhaustions.
- raw FIFO: enqueue, dequeue, depth, HWM, overflow, oldest_age_us.
- libcanard: processed frames, MISSED_START, WRONG_TOGGLE, UNEXPECTED_TID, BAD_CRC, SHORT, OOM.
- per source+DTID: raw CAN frames, completed DroneCAN transfers, transfer-ID gaps.
- decoder: accepted transfers vs decoded valid vs rejected authority.
- USB: records produced/queued/sent/dropped/coalesced by priority/topic.
- ROS: records received/CRC fail/seq gap/stale/session reset.
No counter may be ambiguous or cumulative-only without a boot/session ID.

## P0-E: Fix USB observability/backlog
1. Add F4 boot_id + USB application session_id to diagnostics.
2. Low-priority sensor telemetry becomes latest-value/coalesced, not unbounded historical backlog.
3. Do not accumulate low-priority stream if host is not actively subscribed/heartbeating.
4. Add HELLO/SUBSCRIBE handshake from stmf4; opening a host session flushes stale low-priority backlog and sends one current snapshot.
5. High-priority health/fault messages remain separate and bounded.
6. ROS rejects records from old session_id or records whose sample age exceeds limit.
This prevents old USB backlog from masquerading as current CAN loss/peer-lost events.

## P0-F: Peer/failsafe authority exactly from heartbeat table
1. Keep node_seen/node_verified/node_healthy/last_seen per node as authority.
2. Do not derive PEER_LOST from USB publication timing or NodeInfo service timing.
3. PEER_LOST only after actual NodeStatus freshness timeout from monotonic sample timestamp.
4. Add debounce/state qualification: transition event emitted once; recovery separately counted.
5. Node unhealthy, peer lost, duplicate node and bus fault remain separate states; only real bus/controller fault may reset MCP.

## P1-A: MCP hardware filtering/prioritization
1. Audit actual bus IDs with an external sniffer.
2. If other extended traffic exists, program MCP masks/filters to admit only required DroneCAN traffic without breaking DNA/NodeStatus/GetNodeInfo/Param/sensors.
3. Preserve BUKT rollover RXB0->RXB1.
4. Consider routing high-value heartbeat/service traffic and sensor traffic across filters/buffers only if it reduces worst-case occupancy; do not over-filter blindly.

## P1-B: TX parity with ArduPilot/PX4
1. Three independent TX mailbox descriptors: frame, deadline, pending, priority.
2. CAN arbitration priority decides mailbox replacement/selection, not FIFO order alone.
3. Timeout drops only the expired mailbox/transfer, never unrelated TX queue.
4. TX completion driven by TXnIF; no high-rate status polling.
5. Error-passive/bus-off handling remains explicit and cooldown bounded.

## P1-C: libcanard resource robustness
1. Record pool peak/high-water, allocation failures, stale transfer count.
2. Verify pool size against worst-case simultaneous Fix2/MAG/services/DNA multi-frame transfers.
3. Cleanup stale transfers at deterministic cadence.
4. Transfer-ID gap diagnostics per known subject/source to distinguish upstream source loss from local frame loss.

## P1-D: physical layer qualification
1. With power OFF measure CAN-H to CAN-L resistance; target about 60 ohm for two 120-ohm terminations.
2. Scope CAN-H/CAN-L differential at NEO and MCP during traffic; check ringing, dominant/recessive levels and common-mode.
3. Scope NEO 5 V at connector during steady traffic and load changes; remain inside NEO specification with transient margin.
4. Verify common ground/return path and connector integrity.
5. Do not treat TEC=0 alone as proof that power/peer never disappeared.

## P2: end-to-end zero-loss qualification
A. Idle soak 60 min, no HMI interaction.
B. HMI stress 60 min: continuous redraw + touch while CAN active.
C. USB stress: continuous reader, slow reader, no reader 5 min, reconnect, repeated open/close.
D. Service stress: repeated GetNodeInfo/Param requests while sensor streams continue.
E. NEO power-cycle/reconnect x100; every cycle must return DISCOVERING->VERIFIED->ACTIVE without F4 reset.
F. CAN unplug/replug and no-ACK tests; fail closed and recover deterministically.
G. Inject controlled additional CAN load (20/50/80%) and measure zero-loss envelope.
H. Long combined soak 6-12 h.

## External reference measurement
For definitive proof, capture the same bus simultaneously with an independent USB-CAN/sniffer. Compare time-window frame counts, CAN IDs and DroneCAN transfer-ID continuity against F4 counters. This is the only way to distinguish packets never transmitted by NEO from packets lost inside F4/MCP.

## Acceptance criteria for nominal operation
- MCP RX0OVR delta = 0
- MCP RX1OVR delta = 0
- raw FIFO overflow delta = 0
- ISR budget exhaustion delta = 0
- libcanard BAD_CRC/WRONG_TOGGLE/UNEXPECTED_TID caused locally = 0
- transfer-ID gaps = 0 unless also present on external sniffer
- false PEER_LOST = 0
- DNA verification failures = 0
- TEC/REC/EFLG remain healthy
- high-priority USB drop = 0
- ROS CRC/session/sequence loss = 0 under supported host rate
- no watchdog reset/HardFault
- RM3100/BARO/TEMP/GNSS rates within expected envelopes with bounded jitter

## Recommended implementation order
1. USB/session observability fix first so measurements are trustworthy.
2. Microsecond timestamps + complete loss counters.
3. TX completion interrupt + 3 mailbox scheduler to remove SPI contention.
4. SPSC raw FIFO and ISR timing instrumentation.
5. Fresh 10/60 min CAN-only qualification.
6. HMI/USB stress qualification.
7. External-sniffer A/B proof.
8. Power-cycle/fault injection.
9. 6-12 h combined soak; only then freeze firmware baseline.
