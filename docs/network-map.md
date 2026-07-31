# Powersuit Network Map & Protocol Contract

**Normative.** Every node implementation codes against this document. `ARCHITECTURE.md` is the
original system specification; where engineering reality forced a deviation, it is recorded in
[§12 Deviations](#12-deviations-from-architecturemd). The C reference implementation lives in
`common/c/`, the Python mirror in `common/python/powersuit_proto/`; both are locked together by
shared test vectors. Change rule: edit this document and both implementations in the same commit.

## 1. Topology and node registry

```
                    ┌──────────────────────────────┐
                    │  NODE 9  cloud_ai_core       │
                    │  (GPU cloud, Python/AsyncIO) │
                    └──────────────┬───────────────┘
                        TLS WebSockets (5G/Wi-Fi/Sat)
                    ┌──────────────┴───────────────┐
                    │  NODE 8  node_central_orch.  │
                    │  (Pi 5, Ubuntu 24.04 RT,     │
                    │   ROS 2 Jazzy + uROS agent)  │
                    └──────────────┬───────────────┘
                          SPI 20 MHz, 512 B @ 1 kHz
                    ┌──────────────┴───────────────┐
                    │  NODE 7  node_chest_power_hub│
                    │  (ESP32-P4: BMS, gateway)    │
                    └───────┬──────────────┬───────┘
                     CAN Bus 1 (1 Mbps)  CAN Bus 2 (1 Mbps)
                    ┌───────┴──────┐   ┌───────┴──────┐
                    │ 1 arm_right  │   │ 3 leg_right  │
                    │ 2 arm_left   │   │ 4 leg_left   │
                    │ 6 helmet     │   │ 5 flight     │
                    └──────────────┘   └──────────────┘
```

| Node ID | Name                      | Hardware  | Bus   | micro-ROS client |
|--------:|---------------------------|-----------|-------|------------------|
| 0       | (broadcast destination)   | —         | —     | —                |
| 1       | `node_arm_right`          | ESP32-S3  | CAN 1 | yes              |
| 2       | `node_arm_left`           | ESP32-S3  | CAN 1 | yes              |
| 3       | `node_leg_right`          | ESP32-S3  | CAN 2 | yes              |
| 4       | `node_leg_left`           | ESP32-S3  | CAN 2 | yes              |
| 5       | `node_flight_actuation`   | ESP32-S3  | CAN 2 | yes              |
| 6       | `node_helmet_interface`   | ESP32-S3  | CAN 1 | yes              |
| 7       | `node_chest_power_hub`    | ESP32-P4  | both  | **no** (raw TELEM/MGMT only) |
| 8       | `node_central_orchestrator` | Pi 5    | via SPI through hub | agent side |
| 9       | `cloud_ai_core`           | GPU cloud | via WS through Node 8 | — |

Limb string IDs used in ROS topic names: `arm_right`, `arm_left`, `leg_right`, `leg_left`.
Joints per limb (local index): arms `0=elbow, 1=wrist`; legs `0=hip, 1=knee`.

## 2. CAN identifier layout

Classic CAN 2.0B extended frames only (TWAI has no CAN-FD). Both buses run 1 Mbps.

29-bit identifier, MSB first:

| Bits   | Field   | Width | Meaning                                        |
|--------|---------|-------|------------------------------------------------|
| 28..26 | class   | 3     | message class; lower value = higher arbitration priority |
| 25..21 | src     | 5     | source node ID (1..30)                          |
| 20..16 | dst     | 5     | destination node ID; 0 = broadcast              |
| 15..8  | type    | 8     | message type within class (§3)                  |
| 7..0   | low     | 8     | per-class: seq for TELEM/AUDIO, rolling counter for SAFETY, 0 for XRCE/CONTROL/MGMT unless stated |

| Class | Value | Purpose                                   |
|-------|-------|-------------------------------------------|
| SAFETY  | 0 | heartbeat, e-stop — raw frames, never micro-ROS |
| CONTROL | 1 | joint/flap setpoints, mode, LED patterns  |
| TELEM   | 2 | high-rate packed state (≥20 Hz data)      |
| XRCE    | 3 | micro-ROS (XRCE-DDS) byte streams         |
| AUDIO   | 4 | ADPCM voice frames                        |
| MGMT    | 5 | time sync, flow control, logs, versions   |

**Rate policy (load-bearing):** any data produced faster than 20 Hz MUST use packed TELEM
frames; the Node 8 bridge synthesizes the corresponding ROS messages. XRCE carries only
low-rate topics, parameters, and services. Rationale: one `sensor_msgs/Imu` over XRCE is
~70 CAN frames; at 100 Hz that alone exceeds a full bus. See budget in §10.

## 3. Message catalog

All payloads are little-endian, packed, ≤ 8 bytes. Reference structs: `common/c/include/powersuit_proto/wire.h`
(C) and `common/python/powersuit_proto/wire.py` (Python). Unit conventions: `crad` = 0.01 rad,
`crad_s` = 0.01 rad/s, `cNm` = 0.01 N·m, `mg` = milli-g, `ddps` = 0.1 °/s, `cC` = 0.01 °C,
`cV`/`cA` = 0.01 V / 0.01 A, `pm` = permille, `cms` = cm/s, `Q15` = value/32767.

### 3.1 SAFETY (class 0)

| Type | Name        | Payload (offset : field : type : unit)                                   |
|------|-------------|---------------------------------------------------------------------------|
| 0x01 | HEARTBEAT   | 0:seq:u16, 2:flags:u8 (b0 estop_latched, b1 degraded, b2 cloud_up), 3:src_state:u8 (§4 states), 4:uptime_ms:u32 |
| 0x02 | ESTOP       | 0:cause:u8 (§4.4), 1:origin_node:u8, 2:seq:u16, 4:uptime_ms:u32           |
| 0x03 | CLEAR_ESTOP | 0:magic:u32 = 0x52A4C13A, 4:counter:u32 (strictly greater than last accepted) |
| 0x04 | NODE_FAULT  | 0:fault_code:u8, 1:severity:u8 (0 info,1 warn,2 error,3 critical), 2:detail:u16, 4:uptime_ms:u32 |

HEARTBEAT is emitted **only by Node 8** (via the bridge, relayed by the hub to both buses).
ESTOP is emitted by any node that detects a critical fault and is repeated 3× back-to-back.
ID `low` byte = rolling counter (wraps at 256) for all SAFETY frames.

### 3.2 CONTROL (class 1)

| Type | Name       | Payload                                                                    |
|------|------------|-----------------------------------------------------------------------------|
| 0x10 | JOINT_CMD  | 0:joint:u8, 1:mode:u8 (0 passive,1 pos,2 vel,3 torque,4 impedance), 2:pos:i16:crad, 4:vel:i16:crad_s, 6:eff:i16:cNm |
| 0x11 | FLAP_CMD   | 0:flap:u8, 1:rate_lim:u8 (%/s, 0 = default), 2:pos:i16:pm, 4:flags:u16 (b0 brake_mode), 6:rsvd:u16 |
| 0x12 | MODE_SET   | 0:target_state:u8 (§4 states), 1..7 rsvd                                   |
| 0x13 | LED_PATTERN| 0:pattern:u8, 1:brightness:u8, 2:r:u8, 3:g:u8, 4:b:u8, 5:speed:u8, 6:rsvd:u16 |

### 3.3 TELEM (class 2) — `low` = per-type sequence number

| Type | Name        | Payload                                                                   |
|------|-------------|-----------------------------------------------------------------------------|
| 0x20 | JOINT_STATE | 0:joint:u8, 1:flags:u8 (b0 saturated, b1 passive), 2:pos:i16:crad, 4:vel:i16:crad_s, 6:eff:i16:cNm |
| 0x21 | IMU_QUAT    | 0:qw:i16:Q15, 2:qx:i16:Q15, 4:qy:i16:Q15, 6:qz:i16:Q15                     |
| 0x22 | IMU_ACC     | 0:ax:i16:mg, 2:ay:i16:mg, 4:az:i16:mg, 6:rsvd:u16                          |
| 0x23 | IMU_GYR     | 0:gx:i16:ddps, 2:gy:i16:ddps, 4:gz:i16:ddps, 6:rsvd:u16                    |
| 0x24 | FORCE       | 0:ch0:i16:cN, 2:ch1:i16:cN, 4:ch2:i16:cN, 6:ch3:i16:cN                     |
| 0x25 | BMS_SUMMARY | 0:pack_cV:u16, 2:current_cA:i16, 4:soc_pct:u8, 5:temp_max_C:i8, 6:fault_bits:u16 (§4.4) |
| 0x26 | BMS_CELLS   | 0:group:u8, 1:rsvd:u8, 2:mv0:u16, 4:mv1:u16, 6:mv2:u16                     |
| 0x27 | AERO_STATE  | 0:ias_cms:u16, 2:q_pa:u16, 4:aoa_cdeg:i16, 6:flags:u16                     |
| 0x28 | FLAP_STATE  | 0:flap:u8, 1:flags:u8 (b0 at_limit, b1 fault), 2:pos:i16:pm, 4:target:i16:pm, 6:rsvd:u16 |
| 0x29 | ENV         | 0:temp_cC:i16, 2:rh_pm:u16, 4:press_dhPa:u16, 6:rsvd:u16                   |
| 0x2A | NODE_STATS  | 0:cpu_pct:u8, 1:state:u8, 2:rx_fps:u16, 4:tx_fps:u16, 6:err_cnt:u16        |

Standard TELEM rates: JOINT_STATE / IMU_* / FORCE at 100 Hz per limb; AERO_STATE 50 Hz;
FLAP_STATE 20 Hz; BMS_SUMMARY 10 Hz; BMS_CELLS 1 Hz (rotating groups); ENV 1 Hz; NODE_STATS 1 Hz.

### 3.4 XRCE (class 3)

Single type `0x30 STREAM`, `low` = 0. Payload = 1..8 raw bytes of the node's
HDLC-framed XRCE-DDS stream (client configured with `framing = true`, MTU 512).
Direction up: `dst = 8`. Direction down: `dst = <node>`. Byte order within the stream is
guaranteed by CAN arbitration (fixed ID per direction ⇒ FIFO). The bridge concatenates
per-source bytes onto one PTY per node; a single `micro-ros-agent multiserial` serves all PTYs.
No fragment header, no seq: HDLC gives sync recovery and CRC; XRCE session reliability handles loss.

### 3.5 AUDIO (class 4) — `low` = frame sequence (wraps 256)

| Type | Name       | Payload                                                                    |
|------|------------|-----------------------------------------------------------------------------|
| 0x40 | FRAME_DOWN | 8 bytes IMA-ADPCM (16 samples @ 8 kHz = 2 ms); dst = 6                     |
| 0x41 | FRAME_UP   | 8 bytes IMA-ADPCM; dst = 8                                                 |
| 0x42 | SYNC       | 0:dir:u8 (0 down,1 up), 1:step_index:u8, 2:predictor:i16, 4:frame_seq:u16, 6:rsvd:u16 |
| 0x43 | CTL        | 0:dir:u8, 1:cmd:u8 (0 stop,1 start,2 rate), 2:sample_rate:u16, 4:rsvd:u32  |

Codec: IMA ADPCM, 4 bits/sample, 8 kHz mono, low nibble = first sample. SYNC is sent at
stream start and every 50 frames (100 ms): receiver adopts `predictor`/`step_index` and expects
the next FRAME with `low == frame_seq & 0xFF`. Uplink is VOX-gated (energy gate, §7 of
`docs/safety.md` has thresholds); worst-case duplex load budgeted in §10.

### 3.6 MGMT (class 5)

| Type | Name      | Payload                                                                     |
|------|-----------|------------------------------------------------------------------------------|
| 0x50 | TIME_SYNC | 0:epoch_ms_lo:u32 (lower 32 bits of Unix ms), 4:seq:u16, 6:rsvd:u16 — hub broadcasts at 1 Hz using Pi time |
| 0x51 | FLOW_CTL  | 0:plane:u8 (class value), 1:level:u8 (0 normal,1 reduce,2 pause), 2:rsvd:u16, 4:rsvd:u32 |
| 0x52 | STATS_REQ | empty (dlc 0)                                                               |
| 0x53 | LOG       | 8 ASCII chars; `low` = chunk seq                                             |
| 0x54 | VERSION   | 0:major:u8, 1:minor:u8, 2:patch:u8, 3:node_state:u8, 4:git_short:u32        |

## 4. Safety semantics (summary — full detail in `docs/safety.md`)

Node states: `0 BOOT, 1 STANDBY, 2 OPERATIONAL, 3 PASSIVE, 4 ESTOP, 5 FAULT`.

- **Heartbeat watchdog:** Node 8 broadcasts HEARTBEAT at **100 Hz**. Any actuating node that
  sees no valid HEARTBEAT for **50 ms** drops to PASSIVE (Passive Compliance Mode: FOC to
  zero-torque/active damping; flaps to neutral at rate limit). Tolerates ≥4 consecutive lost frames.
- **Re-arm:** PASSIVE → OPERATIONAL requires **250 ms of continuous heartbeat** AND a fresh
  valid command (JOINT_CMD/FLAP_CMD/MODE_SET) received after recovery.
- **Command staleness (layered):** in OPERATIONAL, no valid JOINT_CMD for 200 ms ⇒ ramp to
  zero-torque hold (soft). Heartbeat loss remains the hard trip.
- **ESTOP:** latched everywhere on reception; repeated 3×; only CLEAR_ESTOP with valid
  monotonic counter + 1 s continuous heartbeat un-latches.
- **Bus-off / TWAI error passive:** treated exactly like heartbeat loss.
- **The hub NEVER synthesizes heartbeats.** SPI link death ⇒ heartbeats stop ⇒ suit fails toward passive. 

### 4.4 ESTOP causes / BMS fault bits

Causes: `1 BMS_SHORT, 2 BMS_OVERVOLT, 3 BMS_UNDERVOLT, 4 OPERATOR, 5 THERMAL, 6 COMM_LOSS, 7 SOFTWARE`.
BMS `fault_bits`: `b0 short_latch, b1 ov, b2 uv, b3 ot, b4 ut, b5 oc_charge, b6 oc_discharge, b7 comparator_armed`.

## 5. Hub routing policy (Node 7)

| Origin → | CAN 1 | CAN 2 | SPI (Node 8) | Hub local |
|----------|-------|-------|--------------|-----------|
| SAFETY from any CAN | forward to other bus | forward to other bus | forward | observe (estop latch) |
| SAFETY from SPI     | forward | forward | — | observe |
| CONTROL from SPI    | if dst on bus 1 | if dst on bus 2 | — | dst=7: consume |
| TELEM/XRCE from CAN | never cross-bus | never cross-bus | forward | — |
| XRCE/CONTROL from SPI | by dst bus | by dst bus | — | — |
| AUDIO up (from CAN 1) | — | never | forward | — |
| AUDIO down (from SPI) | to node 6 | never | — | — |
| MGMT | broadcast both | broadcast both | forward | consume/emit |

Hub-originated data (BMS TELEM, NODE_STATS, LOG) is injected as records with `src=7` directly
into the SPI uplink; it never occupies CAN bandwidth.

## 6. SPI bridge framing (Node 7 ↔ Node 8)

Full-duplex, Pi 5 is master. Fixed **512-byte** transactions at 1 kHz, plus immediately when the
hub asserts the DATA_READY GPIO. Clock **20 MHz** (see §12). Both directions use the same frame:

```
offset size field
0      2    magic 0xA55A (LE)
2      1    ver = 1
3      1    flags   b0 estop_latched(hub) / unused(Pi), b1 overflow_dropped, b2 more_pending, b3 hub_fault
4      1    count   number of records (0..31)
5      1    seq     per-direction rolling counter
6      2    crc16   CRC16-CCITT-FALSE over bytes [2..5] + all record bytes (count*16)
8      16×N CanRecord[count], N ≤ 31; remainder of 512 B is don't-care padding
```

`CanRecord`: `0:id:u32` (29-bit value), `4:bus_dlc:u8` (high nibble bus, low nibble dlc),
`5:rsvd:u8`, `6:ts_ms:u16` (hub-local ms, unwrapped via TIME_SYNC), `8:data[8]`.
Bus nibble: `0 = CAN1, 1 = CAN2, 3 = HUB_LOCAL` (uplink: hub-originated record; downlink:
command consumed by the hub itself, e.g. LED_PATTERN, FLOW_CTL). Downlink SAFETY records are
forwarded to **both** buses regardless of bus nibble (§5).

Receiver behavior: scan for magic; validate ver/count/CRC; on failure drop bytes until next
magic (tested by corruption scenarios in `tools/suit_sim`). `seq` gaps increment a counter
surfaced in NODE_STATS / bridge diagnostics.

## 7. micro-ROS plane

- Client: `micro_ros_espidf_component` (jazzy), `RMW_UXRCE_TRANSPORT=custom`,
  `rmw_uros_set_custom_transport(framing=true, …)` writing/reading XRCE STREAM frames via `ps_can`.
- Agent (Node 8): one `micro-ros-agent multiserial` process attached to one PTY per edge node,
  created by the bridge under `/tmp/powersuit/xrce/<node_name>`.
- Session keys: node ID; creation/teardown follows bridge lifecycle. Reconnect handled by rclc
  session ping in `ps_uros` task.

## 8. ROS 2 topic map (Node 8 graph)

| Topic | Type | Producer → Consumer |
|-------|------|---------------------|
| `/suit/telemetry/<limb>` (×4) | `sensor_msgs/JointState` | bridge (from TELEM 0x20) → estimation, HUD, gateway |
| `/suit/telemetry/<limb>/imu` (×4) | `sensor_msgs/Imu` | bridge (0x21/22/23) → estimation |
| `/suit/imu/torso` | `sensor_msgs/Imu` | bridge (node 5 IMU TELEM) → EKF (`robot_localization`) |
| `/suit/force/<limb>` (×4) | `std_msgs/Float32MultiArray` | bridge (0x24) → estimation/HUD |
| `/suit/command/<limb>` (×4) | `trajectory_msgs/JointTrajectoryPoint` | IK → bridge (→ JOINT_CMD frames) |
| `/suit/aero/state` | `std_msgs/Float32MultiArray` | bridge (0x27/0x28) → flight mapper, HUD |
| `/suit/aero/target_geometry` | `geometry_msgs/PoseArray` | planner/voice → flight mapper (Node 8) |
| `/suit/voice/trigger` | `std_msgs/String` | helmet via XRCE → voice_local |
| `/suit/hud/telemetry_display` | `visualization_msgs/Marker` | HUD composer → helmet via XRCE (≤2 Hz budget) |
| `/suit/environment/ambient` | `sensor_msgs/Temperature` | helmet via XRCE (1 Hz) |
| `/suit/power/bms` | `suit_msgs/BmsStatus` | bridge (0x25/0x26) |
| `/suit/safety/state` | `suit_msgs/SafetyState` | bridge |
| `/suit/audio/uplink`, `/suit/audio/downlink` | `suit_msgs/AudioChunk` | bridge ↔ voice_local/gateway |
| `/suit/link/status` | `suit_msgs/LinkStatus` | cloud gateway |
| `/joint_states` | `sensor_msgs/JointState` | aggregator (all limbs) → robot_state_publisher |
| `/odometry/filtered` | `nav_msgs/Odometry` | `robot_localization` EKF |
| `/suit/estop` (service) | `suit_msgs/srv/SetEstop` | anyone → bridge |

`/suit/aero/target_geometry` is consumed **on Node 8** by the flight mapper, which emits packed
FLAP_CMD frames — a PoseArray over XRCE to Node 5 would violate the rate policy (§12.5).

## 9. Node 8 ↔ Node 9 link

TLS WebSockets, JSON text envelope for control, binary frames for audio. Full schema with
examples: `docs/link-protocol.md`; protobuf mirror for a future gRPC path: `proto/suit_link.proto`.
Hard rule: **downlink whitelist** — the cloud can send `tts`, `advisory`, `hud_suggestion` and
session control only. Nothing the cloud sends can reach `/suit/command/*`, FLAP_CMD, or any
actuation path.

## 10. Bus budget (worst case, 1 Mbps ≈ 6,600 extended frames/s at 150 bits/frame)

| Bus | Contributors | Frames/s | Load |
|-----|--------------|----------|------|
| CAN 1 | 2 arms × (5 TELEM + 2 CMD) @100 Hz = 1400; audio duplex worst 1000; helmet XRCE/HUD ≈ 200; heartbeat 100; XRCE background 100 | ≈ 2800 | **≈ 42 %** |
| CAN 2 | 2 legs × 700 = 1400; flight (2 CMD @100 Hz + AERO/FLAP TELEM) ≈ 400; heartbeat 100; XRCE 100 | ≈ 2000 | **≈ 30 %** |
| SPI | all of the above ≈ 4800 records/s × 16 B ≈ 77 kB/s | — | ≈ 15 % of 512 B @ 1 kHz capacity |

Typical load is roughly half of worst case (audio is VOX-gated half-duplex in practice).

## 11. Timing budget summary

| Loop | Rate | Where |
|------|------|-------|
| FOC current loop | 1 kHz | limb Core 1 |
| Kinematics/force | 250 Hz | limb Core 1 |
| TELEM emit | 100 Hz | edge Core 0 |
| Heartbeat | 100 Hz | Node 8 bridge |
| SPI transaction | 1 kHz | Pi master |
| BMS sampling | 1 kHz | hub |
| EKF | 100 Hz | Node 8 |
| IK | 100 Hz | Node 8 |
| Telemetry uplink batch | 5 Hz (coalesced) | gateway → cloud |

## 12. Deviations from ARCHITECTURE.md

1. **SPI clock 20 MHz, not "up to 80 MHz".** ESP32-P4 SPI slave is specified to ~60 MHz on
   IO_MUX pins under ideal signal conditions; through a body harness, 20 MHz is the honest
   qualification point. Throughput at 20 MHz (512 B @ 1 kHz uses ~20 % of the line) exceeds the
   worst-case need ~6×.
2. **BMS "<5 µs" trip is hardware, not firmware.** An external analog comparator latches the
   discharge gate (<1 µs). The P4's IRAM ISR is a ~10 µs-class *observer*: timestamps the trip,
   broadcasts ESTOP, manages interlocked re-arm. A software-only guarantee under 5 µs would not
   be honest engineering (see `docs/safety.md`).
3. **"micro-ROS pub/sub at 100 Hz" is realized as TELEM-plane frames at 100 Hz** with ROS
   messages synthesized by the Node 8 bridge. Full ROS messages at that rate over classic CAN
   exceed bus capacity (§2 rate policy, §10 math).
4. **Node 7 runs no on-chip micro-ROS client.** It emits raw TELEM/MGMT as `src=7` records over
   SPI; the bridge publishes its topics. Reversible (the jazzy component supports ESP32-P4).
5. **`/suit/aero/target_geometry` terminates on Node 8**, which maps poses → FLAP_CMD frames;
   Node 5 does not subscribe to the PoseArray directly (rate policy).
6. **A 6-DOF IMU is added to Node 5** (back-mounted, torso-rigid): it is the base IMU for the
   EKF and serves the aero controller. ARCHITECTURE.md listed IMUs only on limbs; the EKF needs
   a torso-rigid sensor (limb IMUs measure limb motion, not base motion).
7. **EKF phase 1 fuses the torso IMU only**; limb IMUs feed limb-state estimation, not
   `robot_localization`. A kinematics-compensated "virtual torso IMU" from limb data is a
   documented upgrade path.
8. **Audio is IMA ADPCM @ 8 kHz** (walkie-talkie quality) to fit CAN budgets; the doc's
   "raw audio buffers" from cloud TTS are transcoded by Node 8 before SPI/CAN descent.
