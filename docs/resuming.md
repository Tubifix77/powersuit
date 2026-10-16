# Resuming Powersuit

**Parked on 2026-10-16.** Everything that can be built and proven without suit hardware
has been, and the next work needs hardware and a decision about what kind of suit this
becomes. This file is the way back in. It assumes nobody remembers anything.

## Where it stands

**Built and tested, all tiers.** Firmware for nine nodes, the Raspberry Pi orchestrator's
ROS 2 stack, and the cloud service — every tier compiles in CI, its logic is unit-tested,
and a whole-suit simulation runs ten end-to-end scenarios against the real cloud server.
See the README's [verified vs unproven](../README.md#verified-vs-unproven) table.

**Proven on real hardware** (two ESP32-S3 boards and two CAN transceivers, no motors —
the record is [`bench-log.md`](bench-log.md)):

- **The dead-man switch.** Cut the orchestrator's power and the limb drops to Passive
  Compliance about 50 ms after the last heartbeat; it re-arms only after about 250 ms of
  fresh heartbeats and a command. Shown in two power configurations, zero bus errors.
- **The one actuation gate.** The real limb firmware's gate allowed energising on every
  control tick while commanded and OPERATIONAL, refused every tick from the instant of a
  trip, and opened again only after re-arm.
- **The real limb firmware keeps time.** Its 1 kHz control loop uses about 20 % of each
  period, missed no ticks over tens of thousands, and held that through a CAN error storm.
- **SAFETY frames leave first.** A SAFETY frame queued behind twelve telemetry frames on
  the same node goes out ahead of all of them — a fix this bench prompted.
- **1 Mbit/s CAN** on the cheap SN65HVD230 modules, over 20 cm, with zero errors.

**Never run:** any actuator, the chest hub (ESP32-P4), the SPI link to the Pi, the Pi
itself, the battery-management hardware, micro-ROS against a real agent, the cloud over
a real 5G or Wi-Fi link, and anything with more than two nodes on a bus.

## The fork: what kind of suit

The electronics — nine nodes, two CAN buses, the protocol, the safety machine — are the
same for both. Almost nothing else is. Decide before buying anything mechanical
([`hardware-bom.md`](hardware-bom.md) costs both).

### Path 1 — a cosplay suit (non-load-bearing)

Lights, sound, a HUD, moving panels; it does not carry you. Roughly **€700–1,100**,
most of it shell and servos.

- **Limbs become servo nodes.** The FOC motor-control path in `node_limb` is not needed.
  `node_flight` already drives twelve servos through a clamped, rate-limited envelope
  that the safety gate controls; a servo limb reuses that pattern. One PWM pin per joint
  makes `node_limb`'s pin-budget problem (below) disappear.
- **Keep the safety architecture.** It costs nothing to keep and is correct if the
  project ever grows teeth. A servo that pinches is still worth a dead-man switch.
- **Servo power never comes from an ESP32 rail.** Separate 5–6 V BECs, sized for stall
  current.

### Path 2 — a load-bearing suit (civil, or for a defence programme)

A machine that applies force to a human. Roughly **€12,000–35,000+**, of which 80–90 %
is eight joint actuators; the electronics are rounding error.

- **Actuators: integrated joint modules, commanded over CAN.** Modules such as
  CubeMars's AK series combine motor, reducer, encoder and drive, and take commands over
  CAN at 1 Mbit/s ("MIT mode" and "servo mode"; the AK80-9 is listed, per CubeMars's AK
  Series Module Driver User Manual V1.0.15.X). That turns `node_limb` from a motor driver
  into a CAN commander, and the pin-budget problem goes away. Two things to design:
  the modules speak their own frame format, so they want either a bus of their own or a
  bridge inside the limb node; and the torques `board_limb.h` specifies (18–140 N·m)
  are well above an AK80-9's 9 N·m rated, so bigger modules.
- **The battery short-circuit trip must be built in hardware** — an analog comparator
  latching the gate — and tested with the MCU unpowered first ([`bringup.md`](bringup.md)
  §8, [`safety.md`](safety.md) §4). Firmware only observes it. Do not let anyone
  "simplify" this into software.
- **The safety design here is engineering intent, not an assessed system.** No hazard
  analysis, no independent review, no assessment against any standard has been done.
  - For a **civil** suit, ISO 13482:2014 (safety requirements for personal care robots)
    is the natural reference: its "restraint type physical assistant robot" explicitly
    includes "wearable suits or non-medical physical assistance exoskeletons" (§3.15.1).
    Generic functional safety is IEC 61508.
  - For a **military** suit, ISO 13482 does **not** apply as such: its scope excludes
    "military or public force application robots", while noting its safety principles
    can still be useful. A defence programme would run its own system-safety process,
    and defence use brings requirements of its own — export control, security
    accreditation — that belong to the organisation, not this repository.

## The known defect to resolve first

**`node_limb`'s pin map cannot run on an ESP32-S3 DevKit** (found 2026-10-16, detail in
[`bench-log.md`](bench-log.md) and at the top of `board_limb.h`). Joint 0's motor gates
use GPIO 22–25, which do not exist on the S3; joint 1's use 35–37, the PSRAM on these
boards. And the budget does not close: two six-gate FOC joints plus sensors need 29
signals against 23 safe pins. Path 1 removes the problem (servos); Path 2 removes it
(CAN modules). Either way it is decided by the path, which is why it was recorded rather
than renumbered. Until then the limb runs in bench mode, and the flash script refuses a
non-bench limb image.

## What to buy next, either path

| Item | Why | Rough cost |
|---|---|---|
| USB logic analyser, 8 channels (a "Saleae clone"; free PulseView software decodes CAN) | turns the bench's ≈ 55 ms / ≈ 259 ms timing estimates into measurements; check the listing comes with grabber leads (no soldering) | ~€10 |
| ESP32-P4 dev board | the chest hub — the only path from the limb buses to the orchestrator | ~€20 |
| Raspberry Pi 5, 8 GB, PSU, SD | the orchestrator | ~€110 |
| A third SN65HVD230 with a removable terminator, or a module without one | a three-node bus (only the two ends may be terminated) | ~€3 |

Then the order is [`bringup.md`](bringup.md) §3 onwards: hub plus one limb, the
heartbeat through the real hub, SPI to the Pi starting at 1 MHz, micro-ROS last.

## Bringing the bench back up

The bench is two DevKitC-1 boards and two SN65HVD230 modules, wired per
[`bench-wiring.html`](bench-wiring.html) — the parts list and every step are in
[`bringup.md`](bringup.md) Appendix A. Identify boards by their CH343 serial, not COM
number.

1. Start Docker Desktop yourself. Repo scripts check for it and never launch it — on this
   PC it crash-loops on stranded socket files when started carelessly.
2. `bash firmware/tools/build_bench.sh` (both `node_bench` roles), and
   `BENCH_LIMB=1 bash firmware/tools/build_bench.sh` (the real limb firmware).
3. `powershell -File firmware/tools/flash_bench.ps1 -Role <orch|limb|limbapp|selftest> -Port COMx`
4. `<idf-python> firmware/tools/read_console.py COMx --seconds 30`, then
   [`bringup.md`](bringup.md) A.5 and A.6.

## Where everything is

| Document | What it is |
|---|---|
| [`README.md`](../README.md) | Entry point: what this is, how to build and test, what is verified |
| [`ARCHITECTURE.md`](../ARCHITECTURE.md) | The original specification — historical; the contract is below |
| [`network-map.md`](network-map.md) | **The contract**: topology, CAN IDs, message catalog, routing, SPI framing, bus budget, deviations from the spec |
| [`safety.md`](safety.md) | The safety state machine, trip chains, and the honest BMS design |
| [`link-protocol.md`](link-protocol.md) | The suit ↔ cloud link |
| [`bringup.md`](bringup.md) | The order to prove hardware in; Appendix A is the two-board bench |
| [`bench-log.md`](bench-log.md) | The record: what the bench is, what was proven, what was found |
| [`bench-wiring.html`](bench-wiring.html) | The bench wiring drawn as it sits |
| [`hardware-bom.md`](hardware-bom.md) | What each path costs |
| [`CLAUDE.md`](../CLAUDE.md) | Invariants a change must not break, and why |
