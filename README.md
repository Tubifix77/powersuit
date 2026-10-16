# Powersuit

[![ci](https://github.com/Tubifix77/powersuit/actions/workflows/ci.yml/badge.svg)](https://github.com/Tubifix77/powersuit/actions/workflows/ci.yml)
[![license: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Distributed firmware and software for a nine-node powered exoskeleton: seven ESP32 edge
nodes on two isolated CAN buses, a Raspberry Pi 5 class orchestrator running ROS 2, and a
GPU cloud service reachable over 5G.

> **Status: parked on 2026-10-16, with its safety core proven on real hardware.**
> Every tier compiles in CI and its logic is tested — 253 Python tests, 12 C suites under
> `-Werror`, five firmware images, a ROS 2 workspace, and ten end-to-end scenarios against a
> simulated suit. On two ESP32-S3 boards over real CAN at 1 Mbit/s, the **dead-man switch**
> trips about 50 ms after the orchestrator's last heartbeat and re-arms only after about
> 250 ms of fresh ones; the **one actuation gate** in the real limb firmware allows and
> refuses energising exactly as specified; and the limb's **1 kHz control loop** keeps time
> with no missed ticks. No motor, hub, SPI link, Pi, battery or real cloud bearer has run.
> **To pick it up again, read [`docs/resuming.md`](docs/resuming.md).**

## Where to start

| You want to… | Read |
|---|---|
| pick the project up again, for a cosplay suit or a load-bearing one | [`docs/resuming.md`](docs/resuming.md) |
| know what has been proven on hardware, and what was found | [`docs/bench-log.md`](docs/bench-log.md) |
| understand the contract every node codes against | [`docs/network-map.md`](docs/network-map.md), [`docs/safety.md`](docs/safety.md), [`docs/link-protocol.md`](docs/link-protocol.md) |
| bring hardware up, in a safe order | [`docs/bringup.md`](docs/bringup.md) — Appendix A is the two-board bench, wired per [`docs/bench-wiring.html`](docs/bench-wiring.html) |
| know what it costs | [`docs/hardware-bom.md`](docs/hardware-bom.md) |
| change code without breaking an invariant | [`CLAUDE.md`](CLAUDE.md) |

[`ARCHITECTURE.md`](ARCHITECTURE.md) is the original system specification. The *normative*
contract is [`docs/network-map.md`](docs/network-map.md); where engineering reality forced a
departure from the specification, it is recorded in
[network-map §12](docs/network-map.md#12-deviations-from-architecturemd) rather than
quietly implemented.

## Topology

```
        Node 9  cloud_ai_core          GPU cloud, Python 3.12 / AsyncIO
             ^   TLS WebSockets over 5G / Wi-Fi / satellite
             v
        Node 8  node_central_orchestrator   Pi 5, Ubuntu 24.04 RT, ROS 2 Jazzy
             ^   SPI 20 MHz, 512 B frames at 1 kHz + DATA_READY
             v
        Node 7  node_chest_power_hub        ESP32-P4: BMS, dual-CAN gateway
          /                        \
   CAN bus 1 (1 Mbps)          CAN bus 2 (1 Mbps)
   1 arm_right                 3 leg_right
   2 arm_left                  4 leg_left
   6 helmet                    5 flight
```

The two buses are deliberately isolated; Node 7 is the only bridge between them and the
only path to the orchestrator. Nothing on a limb bus can talk to the cloud except through
two hops that are both allowed to say no.

## Design rules worth knowing before you change anything

**The heartbeat is a dead-man switch.** Only Node 8 emits it, at 100 Hz. Any actuating node
that goes 50 ms without one drops into Passive Compliance. No component may synthesise,
repeat or extrapolate a heartbeat on behalf of Node 8 — if the SPI link dies, the beats
stop and the suit goes limp, which is the correct failure direction. *Shown on hardware.*

**One actuation gate.** Every torque or servo write goes through `ps_safety_can_actuate()`,
in one place; callers never each check state. *Shown on hardware, both directions.*

**SAFETY first starts inside the node.** CAN arbitration only ranks frames already on the
bus, and the TWAI driver queues first-in, first-out — so each node sends its own frames in
class order, SAFETY first, one at a time. Without it a heartbeat could wait behind a dozen
of its own node's telemetry frames. *Found by the bench, fixed, shown on hardware.*

**Rate policy.** Anything faster than 20 Hz travels as a packed 8-byte TELEM frame, and
Node 8 synthesises the ROS message. A single `sensor_msgs/Imu` over micro-ROS is roughly 70
CAN frames, so one limb publishing it at 100 Hz would exceed a whole 1 Mbps bus. With the
TELEM plane, worst-case load is about 45 % on bus 1 and 33 % on bus 2
([§10](docs/network-map.md), corrected from 42/30 % by a bench measurement).

**The cloud cannot move the suit.** Node 8's gateway accepts exactly six message types from
Node 9 — speech, advisories, HUD hints and session control. There is deliberately no
message by which the cloud can reach an actuator, and `suit_sim` scenario 10 exists to prove
a local safety trip beats any cloud round-trip.

**Hardware does what hardware must.** The specified sub-5 µs battery short-circuit trip is an
analog comparator latching the gate in under a microsecond; firmware observes, timestamps,
broadcasts the e-stop and manages the interlocked re-arm. A software-only guarantee at that
timescale would not be honest ([safety §4](docs/safety.md)).

## Repository layout

| Path | What lives there |
|------|------------------|
| `common/c`, `common/python` | The protocol contract, implemented twice and locked together by generated test vectors: CAN ID codec, 8-byte wire structs, SPI framing, CRC16, IMA ADPCM, cloud envelope |
| `firmware/components` | Shared ESP-IDF components: `ps_can` (with its class-ordered transmit queue `ps_can_txq`), `ps_safety`, `ps_router`, `ps_spibridge`, `ps_uros`, `ps_ctl`, `ps_audio`, `ps_proto` |
| `firmware/apps` | One ESP-IDF project per node — `node_limb` (Nodes 1–4), `node_flight`, `node_helmet`, `node_chest_hub` — plus `node_bench` (the two-board bench and its CAN self-test) and `node_qemu_test` (on-target checks under emulation) |
| `firmware/tests/host` | Every pure-C module compiled and tested on the host under `-Werror` |
| `firmware/tools` | `build_bench.sh`, `flash_bench.ps1`, `read_console.py` for the bench; `qemu_test.sh`; `fetch_deps.sh` for micro-ROS |
| `orchestrator/ros2_ws` | Node 8: CAN/SPI bridge, state estimation, IK, voice, cloud gateway, black box, bringup |
| `cloud` | Node 9 service and its container |
| `tools/suit_sim` | End-to-end simulation of the whole suit, in-process |
| `docs` | The contract, the bring-up order, the bench record, the BOM, and the resuming guide |

## Building and testing

Everything below runs from the repository root. The Python tiers run natively; firmware and
ROS 2 build in containers, so Docker is the only hard prerequisite for those. The quickest
check of everything:

```bash
bash tools/verify.sh              # every tier, ~20 lines of output; full logs in .verify-logs/
bash tools/verify.sh py           # just the native Python suites, seconds, no Docker
```

### Python: protocol, cloud, and the integration simulator

```bash
python -m venv .venv && source .venv/Scripts/activate
pip install -e ./common/python -e "./cloud[dev]" -e ./tools/suit_sim
# one suite at a time: each ships its own conftest.py
pytest common/python/tests -q && pytest cloud/tests -q && pytest tools/suit_sim/tests -q
```

After any protocol change, regenerate the cross-language vectors, which rewrites both the
Python fixture and the C header (CI fails if you forget):

```bash
python common/python/tests/gen_vectors.py
```

### C host tests — every pure module, under `-Wall -Wextra -Werror`

```bash
docker run --rm -v "$PWD:/repo" -w /repo ros:jazzy bash -c "cmake -S firmware/tests/host -B /tmp/host && cmake --build /tmp/host -j && ctest --test-dir /tmp/host --output-on-failure"
```

Any Linux with gcc and CMake works too, WSL included.

### Firmware

One Docker image, `espressif/idf:v5.5.5`, builds every app. The build directory and ccache
live in named volumes because NTFS bind mounts are slow:

```bash
docker volume create ps_fw_build && docker volume create ps_ccache
```

```bash
docker run --rm -v "$PWD:/ws" -v ps_fw_build:/builds -v ps_ccache:/root/.ccache -w /ws/firmware/apps/node_chest_hub -e IDF_CCACHE_ENABLE=1 espressif/idf:v5.5.5 bash -c "git config --global --add safe.directory '*'; idf.py -B /builds/hub build"
```

Substitute `node_limb`, `node_flight` or `node_helmet` for the other targets. `node_limb`
covers Nodes 1–4; pick which one under *Limb node identity* in `idf.py menuconfig`.

micro-ROS is optional. Without it the XRCE plane compiles out and every node still runs its
safety, control, telemetry and audio functions. To enable it:

```bash
bash firmware/tools/fetch_deps.sh
```

### On-target self-test under emulation

Espressif's QEMU fork emulates the ESP32-S3, so the logic runs on Xtensa LX7 under real
FreeRTOS — not a substitute for hardware, but it reaches things host tests cannot:

```bash
bash firmware/tools/qemu_test.sh
```

### ROS 2 workspace

```bash
docker volume create ps_ros_deps && docker volume create ps_ros_out
docker run --rm -v "$PWD:/repo:ro" -v ps_ros_deps:/deps -v ps_ros_out:/out -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros:jazzy bash /repo/tools/ros_ci.sh
```

The first run builds the micro-ROS agent from source, which takes a while; the named
volumes make later runs quick.

## Running on the bench

Two ESP32-S3-DevKitC-1 boards, two SN65HVD230 CAN modules and ten female-female jumpers —
no soldering ([`docs/bringup.md`](docs/bringup.md) Appendix A,
[`docs/bench-wiring.html`](docs/bench-wiring.html)). Docker Desktop on Windows cannot reach a
COM port, so firmware is built in Docker and flashed from the host:

```bash
bash firmware/tools/build_bench.sh                 # both node_bench roles, verified
BENCH_LIMB=1 bash firmware/tools/build_bench.sh    # the real node_limb, bench mode
BENCH_SELFTEST=1 bash firmware/tools/build_bench.sh  # one-board CAN self-test
```

```powershell
powershell -File firmware/tools/flash_bench.ps1 -Role orch -Port COM4    # -DryRun opens no port
<idf-python> firmware/tools/read_console.py COM5 --seconds 30
```

## Running the suit without hardware

`tools/suit_sim` wires fake limbs and a fake hub over virtual CAN buses to a bridge speaking
the real 512-byte SPI framing, and connects it to the *actual* Node 9 server over a real
WebSocket. The ten scenarios cover watchdog trips and re-arm timing, e-stop propagation and
replay rejection, SPI corruption recovery, voice round-trip through the cloud, link failover
with session resume, and the local-beats-cloud ordering guarantee.

```bash
pytest tools/suit_sim/tests -q
```

The same simulation can be driven by the *real* Node 8 stack instead of the Python
stand-in — real ROS graph, real C++ bridge, real framing, simulated hardware:

```bash
python -m suit_sim.serve --port 9700 --cloud
```

```bash
ros2 launch suit_bringup sim.launch.py mock_port:=9700
```

## Deploying

- **Node 8**: `orchestrator/scripts/install_pi.sh` provisions Ubuntu 24.04 on a Pi 5 and
  installs the systemd unit in `orchestrator/systemd/`. Never run on a real Pi.
- **Node 9**: `docker compose -f cloud/docker-compose.yml up app`. The default engine is a
  deterministic mock. Point it at a real model with the `ollama` profile (CPU, no account
  needed) or the `gpu` profile (vLLM), both through the same OpenAI-compatible adapter.
- **Nodes 1–7**: built in Docker, flashed from the host with esptool; the bench scripts
  above do this for the bench images ([bringup A.4d](docs/bringup.md)).

## Verified vs unproven

Being explicit, because "it compiles" and "it works" are very different claims for a
machine that applies force to a human body.

| Tier | What is actually checked |
|------|--------------------------|
| Protocol contract | 85 tests; the C and Python implementations are locked together by generated vectors |
| Cloud (Node 9) | 73 tests against the real server over a real WebSocket |
| Orchestrator (Node 8) | 74 Python tests, 3 C++ gtest suites, full `colcon build` |
| Pure C firmware logic | 12 suites under `-Wall -Wextra -Werror` — FOC maths, Mahony filter, safety state machine, CAN routing, CAN transmit order, ADPCM |
| Firmware images | All four suit apps build for ESP32-S3/P4, with and without micro-ROS, plus `node_bench` |
| Whole-suit behaviour | 10 simulated scenarios: watchdog trip and re-arm timing, e-stop propagation and replay rejection, SPI corruption recovery, voice round-trip, link failover, local-beats-cloud ordering |
| On-target (QEMU) | 27 checks on emulated ESP32-S3: packed structs on Xtensa, the FPU, dual-core scheduling, stack headroom, watchdog timing against real elapsed time |
| **On silicon** | Two ESP32-S3 boards, real CAN at 1 Mbit/s: the dead-man switch (trip ≈ 53–55 ms, re-arm ≈ 258–259 ms, by frame counter), the one actuation gate both ways, the real `node_limb`'s 1 kHz loop (worst iteration ~200 µs, 0 missed ticks), SAFETY overtaking a 12-frame queue — [`docs/bench-log.md`](docs/bench-log.md) |
| Static analysis | `cppcheck` clean |

**Not proven, and only provable on more hardware:** any actuator and its peripheral
configuration (does the MCPWM emit the waveform the registers imply?), the chest hub, the SPI
link at 20 MHz, the Pi, micro-ROS against a real agent, CAN arbitration with more than two
nodes at the predicted load, the analog battery trip chain, and the cloud over a real bearer.

**Known defect, open:** `node_limb`'s pin map cannot run on an ESP32-S3 DevKit — it uses GPIO
that do not exist on the S3 and the PSRAM pins. Fixing it depends on the choice of actuator,
so it is recorded rather than renumbered ([`docs/resuming.md`](docs/resuming.md)); the limb
runs in a bench mode without motor hardware meanwhile.

## Safety

This is designed to drive motors attached to a person. Three things are worth stating
plainly:

The design fails toward limpness on purpose. Lose the heartbeat for 50 ms and every limb
goes back-drivable; kill the orchestrator and the whole suit goes slack. That is the correct
behaviour, and any change that makes the suit *hold* through a fault is a change in the
wrong direction.

The sub-microsecond battery short-circuit trip is **hardware** — an analog comparator
latching the gate. Firmware only observes it. `ARCHITECTURE.md` asks for it in software;
[`docs/safety.md`](docs/safety.md) §4 explains why that cannot honestly be delivered there.

The safety design is **engineering intent, not an assessed system**: no hazard analysis or
assessment against any standard has been done ([safety §9](docs/safety.md)). The MIT
licence's warranty disclaimer is not a formality here.

## Contributing

The contract in [`docs/`](docs) is normative: if a change alters behaviour on the wire, in
the safety state machine, or in the cloud link, the document changes in the same commit.
[`CLAUDE.md`](CLAUDE.md) lists the invariants a change must not break and the reasoning
behind each.

```bash
bash tools/dev.sh setup && bash tools/dev.sh test    # no Docker needed
bash tools/verify.sh                                  # every tier, ~20 lines out
```

## Licence

[MIT](LICENSE) © 2026 Tubifix77
