# Powersuit Safety Design

**Normative companion to `docs/network-map.md` §4.** This document defines the safety state
machine every actuating node implements, the trip chains, and the honest engineering behind the
two "hardware" guarantees in `ARCHITECTURE.md`. The reference state machine is pure C in
`firmware/components/ps_safety/` (`ps_safety_core.c`) and is host-tested in `firmware/tests/host/`.

## 1. Design stance

1. **Fail toward limpness.** Every failure of communication, power telemetry, or software must
   move actuators toward the state that applies the least force to the wearer: limbs free-wheel
   or actively damp; flaps go to neutral; nothing latches rigid.
2. **The heartbeat is the suit's dead-man switch.** Only Node 8 emits it. No component may
   synthesize, repeat, or extrapolate a heartbeat on behalf of Node 8 — the hub forwards it
   verbatim or not at all.
3. **Hardware first, firmware observes.** Where a reaction time is promised below what software
   can honestly guarantee, the reaction is implemented in hardware and firmware's job is to
   observe, report, and manage re-arm.

## 2. Node safety state machine

States: `BOOT(0) → STANDBY(1) → OPERATIONAL(2)`, with `PASSIVE(3)`, `ESTOP(4)`, `FAULT(5)`.

```
BOOT ──init ok──▶ STANDBY ──MODE_SET(OPERATIONAL) + heartbeat ok──▶ OPERATIONAL
 │ init fail                                                        │
 ▼                             heartbeat lost ≥ 50 ms ──────────────┤
FAULT ◀── unrecoverable ──── PASSIVE ◀──────────────────────────────┘
                               │  250 ms continuous heartbeat
                               │  + fresh valid command
                               └────────────▶ OPERATIONAL
ESTOP: entered from ANY state on ESTOP frame, BMS trip, or local critical fault.
       exit only: CLEAR_ESTOP(valid counter) + 1 s continuous heartbeat → STANDBY.
```

Transition rules (constants in `ps_safety.h`, mirrored in `powersuit_proto`):

| Constant | Value | Meaning |
|----------|-------|---------|
| `PS_HEARTBEAT_PERIOD_MS` | 10 | Node 8 emission period (100 Hz) |
| `PS_HEARTBEAT_TIMEOUT_MS` | 50 | watchdog trip → PASSIVE |
| `PS_REARM_WINDOW_MS` | 250 | continuous beats required to leave PASSIVE |
| `PS_CMD_STALE_MS` | 200 | JOINT_CMD staleness → soft zero-torque ramp |
| `PS_ESTOP_REARM_MS` | 1000 | continuous beats required after CLEAR_ESTOP |
| `PS_ESTOP_REPEAT` | 3 | back-to-back ESTOP transmissions |

**Passive Compliance Mode (PASSIVE), per node type:**
- Limb nodes: FOC drops to zero-torque; if back-EMF indicates fast motion, switch to active
  damping (short low-side FETs modulated) to dissipate energy. Joints must remain back-drivable.
- Flight node: all flaps commanded to neutral at the configured rate limit; aero engine keeps
  publishing AERO_STATE (telemetry never stops).
- Helmet: no actuation; HUD shows SAFE-state banner; audio pipeline unaffected.
- Hub: gateway and BMS unaffected (routing IS the safety path); LED pattern → amber breathing.

**Layered command staleness:** heartbeat present but JOINT_CMD stale > 200 ms ⇒ ramp effort to
zero over 100 ms and hold (soft stop). This distinguishes "orchestrator alive but planner quiet"
from "link dead" (hard PASSIVE). Both recover automatically when commands resume.

**Bus-off:** TWAI bus-off or error-passive is treated identically to heartbeat loss. `ps_can`
attempts automatic recovery with backoff; the safety state machine does not wait for it.

## 3. E-stop

- Any node may originate ESTOP (`cause` per network-map §4.4). It is sent 3× back-to-back
  (no ACK exists on the SAFETY plane; three arbitration-priority-0x02 frames on a 42 %-loaded
  bus have negligible combined loss probability).
- Every node latches ESTOP on first reception, including the originator. The hub cut-through
  forwards SAFETY frames across both buses and SPI before local processing completes.
- Node 8's bridge stops emitting heartbeats while ESTOP is latched — belt and braces: even a
  node that missed all three frames goes PASSIVE within 50 ms.
- `CLEAR_ESTOP` carries `magic 0x52A4C13A` + a monotonic `counter`. Receivers accept only
  `counter > last_accepted` (persisted in RTC RAM across soft resets). This prevents replayed or
  corrupted frames from silently un-latching an e-stop.
- After a valid clear, nodes sit in STANDBY and require `PS_ESTOP_REARM_MS` of continuous
  heartbeat plus an explicit MODE_SET before actuating again.

## 4. BMS short-circuit trip — the honest <5 µs chain

`ARCHITECTURE.md` requires "rapid short-circuit detection (hardware interrupt response under
5 microseconds)". Measured ESP32 GPIO ISR latencies are 4–6 µs *typical* with IRAM handlers and
are **unbounded** in the worst case (cache refill, competing critical sections). Promising <5 µs
from firmware would be dishonest. The design therefore is:

```
shunt/hall sense ──▶ analog comparator (fixed threshold, hysteresis)
                        │  < 1 µs, no software involved
                        ├──▶ SR latch ──▶ gate driver DISABLE (discharge FET opens)
                        └──▶ GPIO (P4)  ──▶ IRAM ISR observer
```

- **Primary trip:** comparator + SR latch open the discharge path in well under 1 µs. The latch
  holds the gate off regardless of MCU state.
- **Firmware observer (`bms_task.c` + IRAM ISR):** on the latch GPIO edge: timestamp (cycle
  counter), read fault snapshot, broadcast ESTOP(cause=BMS_SHORT) 3×, set `fault_bits.b0`,
  drive LED pattern red-strobe. Typical end-to-end observer latency ~10 µs; it does not matter,
  because the current path is already open.
- **Re-arm interlock:** clearing the SR latch requires BOTH a firmware GPIO strobe (only from
  STANDBY, after CLEAR_ESTOP) AND the comparator reading below threshold — wired as an AND gate.
  Firmware alone cannot re-close a shorted pack.
- Overvoltage/undervoltage/thermal trips are slower phenomena and are handled in `bms_task.c`
  sampling at 1 kHz with debounce, escalating STANDBY → ESTOP per thresholds in `board_hub.h`.

## 5. Flap geometric limits (Node 5)

`ARCHITECTURE.md`: "Enforces geometric mechanical limits in hardware tasks to prevent structural
collision." Implementation:

1. **Firmware clamp at the last write:** `servo_task.c` clamps every target against the
   per-flap envelope table (`board_flight.h`: min/max permille, max slew %/s, pairwise
   exclusion zones for overlapping surfaces) *immediately before* the LEDC duty write — after
   all other software (IK, mixers, trims) has had its say. No code path can bypass the clamp.
2. **Pairwise exclusion:** overlapping surface pairs define a joint envelope; if a commanded
   pair violates it, both are pulled toward the nearest legal point (documented in the table).
3. **Mechanical end-stops** remain the true "hardware" guarantee; the firmware clamp exists to
   keep the servos from ever loading the stops.
4. In PASSIVE/ESTOP the table's neutral column is authoritative.

## 6. Audio VOX gate (helmet uplink)

The mic uplink starts streaming only when the energy gate opens: RMS over 32 ms window >
`-38 dBFS` with 300 ms hang time, hard-capped at 8 s per utterance (then CTL stop + re-arm).
Prevents continuous 500 fps AUDIO load on Bus 1; budget assumes worst-case duplex anyway.

## 7. What firmware must never do

- Never actuate in BOOT/STANDBY/PASSIVE/ESTOP/FAULT (enforced in one place: the actuation
  write functions check `ps_safety_state()` — not each caller).
- Never mask, delay, or batch SAFETY-class frames (hub cut-through path is lock-free).
- Never accept CONTROL frames from the AUDIO/TELEM/XRCE planes (class check in dispatch).
- Never clear an ESTOP latch from an ISR.
- The cloud (Node 9) has **no path** to any of this: its downlink vocabulary is
  tts/advisory/HUD only, enforced by the Node 8 gateway whitelist (`docs/link-protocol.md` §6).
