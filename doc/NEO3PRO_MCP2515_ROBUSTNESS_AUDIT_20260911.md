# NEO3 Pro / MCP2515 / DroneCAN Robustness Audit — 2026-09-11

## Scope
Audit evidence from calibration logs, current F411/MCP2515 source, ArduPilot AP_Periph/AP_DroneCAN, and PX4 UAVCAN CAN drivers.

Primary evidence:
- data/navigasi/calibration/heading_8dir_raw_20260908_202227.csv
- data/navigasi/calibration/heading_8dir_raw_20260908_211808.csv
- data/navigasi/calibration/heading_8dir_raw_20260908_212631.csv
- data/navigation/yaw8/flex_20260911_164854_neo_raw.txt
- F4gateway/src/Neo3ProSensors.cpp
- referensi/navigasi/reference_repos/28_ardupilot
- referensi/navigasi/reference_repos/27_px4_autopilot

## Executive finding
The RM3100 outage is not caused only by LED commands. During the 2026-09-11 calibration run, raw CAN itself stopped for long intervals while no LED command was being sent. F4/MCP remained alive with TEC near zero and EFLG zero. This is consistent with NEO/transceiver/supply-side silence rather than an MCP bus-off.

A second independent bug extends the outage: after raw CAN returns, F4 repeatedly verifies GetNodeInfo successfully but clears `primary_node_id_` again because stale `node_` state survives identity invalidation.## Evidence from old calibration
The older calibration sessions show that stable RM3100 operation is achievable:
- 202227: neo_yaw finite 5650/5719 rows; only the initial 69-row acquisition gap was missing.
- 211808: neo_yaw finite 15179/15180 rows.
- 212631: neo_yaw finite 11069/11072 rows.

Therefore long RM3100 dropout is not inherent NEO3 Pro behavior.

## Evidence from current run
In `flex_20260911_164854_neo_raw.txt`:
- 0–60 s: raw CAN, MAG, GNSS and NodeStatus are healthy.
- approximately 80–120 s: raw CAN counter freezes completely.
- 120–200 s: raw CAN returns, but MAG/Node application output remains absent while repeated verified NodeInfo responses appear.
- approximately 200–320 s: raw CAN freezes again.
- approximately 320–460 s: raw CAN returns but sensor authority remains unstable.

At the first outage, MCP remained `can_ok=1`, TEC=0, EFLG=0 and no MCP recovery occurred. Raw traffic later resumed without MCP reset. This strongly points to temporary peer-side silence/power/transceiver interruption.## P0 software bug: authority stale loop
Current F4 invalidation clears `identity_`, params and node IDs but leaves the old `node_` timestamp intact. Before identity is verified, NodeStatus is used only to start discovery and is not decoded into `node_`.

Sequence:
1. peer NodeStatus becomes stale and authority is cleared;
2. a new NodeStatus starts GetNodeInfo;
3. GetNodeInfo succeeds and sets primary source 62;
4. next poll sees the old stale `node_.received_ms`;
5. identity is immediately cleared again.

Live proof: repeated `SENS:NODEINFO` responses reported verified=1, board=1001 and source=62 while HWPRO still reported primary node 0.

Required design: explicit peer states `NO_PEER -> DISCOVERING -> VERIFIED_WAIT_STATUS -> ACTIVE -> DEGRADED/LOST`, with discovery NodeStatus freshness tracked separately from authoritative sensor state.

## P0 source acceptance design
Current `shouldAccept()` rejects MAG/Fix2/BARO before identity verification. ArduPilot instead subscribes to sensor broadcasts by transfer source while node verification runs in parallel.

Recommended F4 behavior: always allow known sensor DTOs into libcanard transport state, decode them into a quarantine/latest-state cache, and promote them to authoritative telemetry only after the same source is verified. This preserves transport continuity during GetNodeInfo retries without trusting an unverified node.## P0 TX robustness gap
Current F4 uses a 1.5 ms MCP physical mailbox deadline and 50 ms libcanard queue deadline. On timeout it can clear the whole libcanard TX queue.

ArduPilot AP_Periph uses approximately 1 s CAN send deadline, retries later when the interface is busy, and only performs stale cleanup after repeated failures. PX4 tracks each hardware mailbox deadline separately and preserves arbitration semantics.

For MCP2515, use a longer bounded physical deadline (initial recommendation 10–20 ms for services, shorter for best-effort LED), preserve normal retransmission/arbitration, and never clear unrelated queued transfers for one mailbox timeout. Use TXB0/TXB1/TXB2 or an explicit priority-aware software scheduler.

## P0 RX architecture gap
MCP2515 has only two RX buffers. Current F4 drains hardware directly into libcanard from cooperative main/realtime paths. Any CPU service gap can overflow those buffers and break multi-frame transfer continuity.

PX4 moves frames from hardware FIFO into a software RX queue immediately in the CAN ISR path, then transport processing consumes the software queue separately. The MCP equivalent should be:
`PB10 IRQ latch -> immediate bounded MCP drain -> 64/128-frame RAM raw ring -> libcanard decode in main context`.

Track MCP RX0OVR/RX1OVR separately from raw-ring overflow. Drain hardware until INT releases, with a strict microsecond budget and immediate follow-up scheduling if INT remains low.## P0 hard-recovery protocol state
A hard MCP/SPI reset destroys frame continuity, but current `recoverCan()` reinitializes MCP/SPI while retaining libcanard RX state. Libcanard eventually times out stale state after 2 s, but deterministic recovery should purge/reinitialize RX transfer state on known hardware discontinuity while preserving outgoing transfer-ID continuity.

RX overflow alone is not a reason for a full bus reset; allow transport resynchronization on the next valid start-of-transfer. Bus-off/SPI failure can justify hard reinitialization.

## P1 dynamic node allocation
Current F4 DNA allocator has host ID 124 and generally offers ID 125, with no persistent UID-to-node database. ArduPilot DNA server persists UID mappings, detects duplicate IDs, and separately tracks seen/verified/healthy nodes.

For this AGV choose one policy explicitly: disable F4 DNA server if another allocator owns the bus, or implement deterministic persistent UID mapping. Do not infer that NEO should be 125 merely because F4 offered it; the live unit has repeatedly operated as node 62.

## P1 diagnostics
Add explicit peer state, discovery source, last raw NodeStatus timestamp, active source, GetNodeInfo success/failure, identity-generation counter, peer reboot counter, raw-frame age, MCP IRQ-low duration, RX ring HWM, pool allocator current/peak, and per-cause recovery counters.

Decode the NodeStatus prefix embedded in GetNodeInfoResponse. Its uptime lets the gateway distinguish an actual NEO reboot from a CAN/transceiver interruption without guessing.## Items already correct and worth retaining
- MCP2515 is isolated on SPI2; TFT/touch stay on SPI1.
- SPI2 is 6 MHz, below the MCP2515 SPI limit.
- Production oscillator is fixed to 8 MHz.
- CAN bit timing CNF1=0x00, CNF2=0x80, CNF3=0x80 provides 1 Mbit/s with 75% sample point and single sampling.
- OSM is disabled, preserving normal CAN retransmission/arbitration.
- hardware filter rejects standard-ID traffic and accepts extended DroneCAN traffic.
- USB formatting is decoupled from the DroneCAN RX callback.
- watchdog/fault containment should remain independent of CAN recovery.

## Hardware investigation
Because raw CAN itself repeatedly freezes without LED commands, software fixes alone cannot close this issue. Measure NEO 5 V directly at its connector and CANH/CANL during a data-only soak. Log NodeStatus/GetNodeInfo uptime after every recovery.

If raw CAN freezes while MCP TEC/REC/EFLG remain quiet and NEO uptime restarts after traffic returns, classify as NEO power/brownout reset. If uptime continues, investigate NEO CAN transceiver/cabling. If MCP REC/EFLG rise, investigate termination/noise/bit timing/wiring.

## Qualification target
Run at least 60 minutes data-only with no LED command, then repeated NEO power cycles, CAN disconnect/reconnect, TFT/touch activity, USB host flood, forced TX no-ACK, and RX overflow tests. Acceptance requires deterministic recovery, no authority-thrash, no unrecovered RX overflow, no bus-off persistence, and automatic return of MAG/GNSS/BARO without F4 reset.