# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware and software for a nine-node powered exoskeleton: seven ESP32 edge nodes on two
isolated CAN buses, a Raspberry Pi 5 class ROS 2 orchestrator, and a cloud AI service.
`README.md` is the human-facing entry point.

**Status: parked on 2026-10-16.** Every tier builds and is tested; on a two-board bench the
dead-man switch, the actuation gate and the limb's 1 kHz loop are proven on silicon. The
next work needs hardware and a decision between a cosplay suit and a load-bearing one.
`docs/resuming.md` is the way back in; `docs/bench-log.md` is the record of what the
hardware has shown — update it when something is verified, not when it is planned.

**`ARCHITECTURE.md` is the original specification, not the contract.** The normative contract
every node codes against is `docs/network-map.md` (topology, CAN ID layout, message catalog,
routing, SPI framing, bus budget), with `docs/safety.md` (state machine, trip chains) and
`docs/link-protocol.md` (cloud link) alongside it. Where engineering reality forced a
departure from the specification it is recorded in `docs/network-map.md` §12 — read that
section before assuming the spec is authoritative on any given point.

`ARCHITECTURE.md` is written in escaped markdown (literal `\*\*`, `\##`, `&#x20;` — an export
artifact). If you edit it, match the existing escaping; don't mix styles.

## Repository layout

| Path | Contents |
|------|----------|
| `common/c`, `common/python` | The protocol contract implemented twice, locked together by generated vectors |
| `firmware/components` | `ps_proto`, `ps_can`, `ps_safety`, `ps_router`, `ps_spibridge`, `ps_uros`, `ps_ctl`, `ps_audio` |
| `firmware/apps` | `node_limb` (Nodes 1–4), `node_flight` (5), `node_helmet` (6), `node_chest_hub` (7); `node_bench` (two-board bench + CAN self-test); `node_qemu_test` |
| `firmware/tests/host` | Every pure-C module, compiled and tested on the host |
| `firmware/tools` | `build_bench.sh`, `flash_bench.ps1`, `read_console.py`, `qemu_test.sh`, `fetch_deps.sh` |
| `docs` | The contract (`network-map`, `safety`, `link-protocol`), `bringup`, `bench-log`, `bench-wiring.html`, `hardware-bom`, `resuming` |
| `orchestrator/ros2_ws` | Node 8: bridge, estimation, IK, voice, cloud gateway, blackbox, bringup |
| `cloud` | Node 9 service and container |
| `tools/suit_sim` | Whole-suit simulation with ten end-to-end scenarios |

## Commands

Verification is deliberately kept out of the conversation — build logs are enormous and only
failures matter:

```bash
bash tools/verify.sh              # every tier, ~20 lines of output
bash tools/verify.sh py           # just the native Python suites (seconds, no Docker)
bash tools/verify.sh --failures   # error lines from the last run
```

Full logs land in `.verify-logs/`. Individual tiers: `py`, `c`, `fw`, `ros`, `cloud`.

Python suites must be run **separately** — each ships its own `conftest.py`, and pytest's
import mode lets one shadow the other:

```bash
pytest common/python/tests -q && pytest cloud/tests -q && pytest tools/suit_sim/tests -q
```

After any protocol change, regenerate the cross-language vectors or CI will fail:

```bash
python common/python/tests/gen_vectors.py
```

Firmware (substitute the app; `-B` on a named volume because NTFS bind mounts are slow):

```bash
docker run --rm -v "$PWD:/ws" -v ps_fw_build:/builds -v ps_ccache:/root/.ccache -w /ws/firmware/apps/node_limb -e IDF_CCACHE_ENABLE=1 espressif/idf:v5.5.5 bash -c "git config --global --add safe.directory '*'; idf.py -B /builds/node_limb build"
```

Flashing cannot happen from that container: Docker Desktop on Windows has no
access to COM ports. Build in Docker, flash from the host (`docs/bringup.md` A.4d).
For the two-board bench:

```bash
bash firmware/tools/build_bench.sh                    # both node_bench roles, verified
BENCH_LIMB=1 bash firmware/tools/build_bench.sh       # the real node_limb, bench mode
BENCH_SELFTEST=1 bash firmware/tools/build_bench.sh   # one-board CAN self-test
powershell -File firmware/tools/flash_bench.ps1 -Role <orch|limb|limbapp|selftest> -Port COMx   # -DryRun opens no port
<idf-python> firmware/tools/read_console.py COMx --seconds 30 [--until TEXT]                    # console from boot, non-interactive
```

Identify boards by CH343 serial number (register in `docs/bench-log.md`), not by COM
number, and check it before every flash. `read_console.py` needs the ESP-IDF
environment's Python (`%USERPROFILE%\.espressif\python_env\...\python.exe`), which has
pyserial; opening a port with ordinary settings resets the board.

**`node_limb` must only go onto a DevKit in bench mode** (`CONFIG_PS_LIMB_BENCH`).
`board_limb.h`'s pin map uses GPIO 22–25, which do not exist on the ESP32-S3, and 35–37,
the octal PSRAM; `flash_bench.ps1` refuses a non-bench limb image. Replacing the map is a
hardware decision recorded in `docs/resuming.md` — do not renumber it to make it compile
somewhere.

**Check hardware facts against primary text.** Two wrong claims entered this repo from
summaries — a "250 kbit/s cap" from a model's reading of TI's datasheet, and a "1.8 V
GPIO47/48" relayed from another project. Extract the PDF or page text and search it
yourself; label what is measured, what is from a datasheet, and what is only reported.

**Never launch Docker Desktop from a script or a command.** On this PC it
crash-loops on stranded socket files when started carelessly. If `docker info`
fails, say so and stop; repo scripts only check for it.

**The bench boards are shared** with a sibling project (Potluck) that runs 24-hour
soaks on them. Before flashing, unplugging or resetting a board, read
`docs/bench-log.md`; during a soak, leave every `python.exe` alone too. Building
images touches no board and is always fine.

## Invariants — do not break these

**The heartbeat is a dead-man switch.** Node 8 alone emits it, at 100 Hz. Any actuating node
that sees 50 ms of silence enters Passive Compliance. Nothing may synthesise, repeat or
extrapolate a beat on Node 8's behalf — the hub explicitly must not. If the SPI link dies the
beats stop and the suit goes limp, which is correct.

**Anything above 20 Hz rides the raw TELEM plane**, not micro-ROS. One `sensor_msgs/Imu` over
XRCE is ~70 CAN frames; a single limb at 100 Hz would exceed a whole 1 Mbps bus. Node 8
synthesises the ROS messages. Adding a high-rate ROS publisher on an edge node breaks the bus
budget in `docs/network-map.md` §10.

**SAFETY first starts inside the node.** The TWAI driver's transmit queue is FIFO, so
arbitration alone cannot put a heartbeat or ESTOP ahead of a node's own telemetry.
`ps_can` holds pending frames in class order and gives the driver one at a time
(`ps_can_txq.h`). Do not call `twai_node_transmit()` anywhere else, and do not raise
the driver's `tx_queue_depth` — either reintroduces the inversion.

**One actuation gate.** Every torque or servo write goes through `ps_safety_can_actuate()`;
callers do not each check state. `ps_focdrv_disable()`/`brake()` are always permitted — they
are the safe directions.

**The cloud cannot move the suit.** Node 8's gateway accepts exactly six server-originated
message types (`link.DOWNLINK_WHITELIST`). There is deliberately no message that reaches an
actuator. `suit_sim` scenario 10 exists to prove a local trip beats any cloud round-trip;
if you add a downlink type, that test and `docs/link-protocol.md` §6 both have to change.

**Hardware where hardware is required.** The sub-5 µs battery short-circuit trip is an analog
comparator latching the gate; firmware only observes, timestamps and re-arms. Do not "improve"
this into a software check. Likewise `BMSF_COMP_ARMED` is a *health* bit — set means the
comparator is armed and the pack is protected.

**Pure C stays pure.** `foc_math.c`, `pid.c`, `imu_mahony.c`, `ps_safety_core.c`, `ps_can_txq.c`,
`ps_router.c`, `audio_pkt.c`, `energy_gate.c` must not include any FreeRTOS or ESP header —
they are compiled on the host under `-Wall -Wextra -Werror` by `firmware/tests/host`.

**micro-ROS is optional.** `PS_UROS_ENABLED` is 0 unless the client is vendored via
`firmware/tools/fetch_deps.sh`. All rclc-dependent code lives inside `#if PS_UROS_ENABLED`,
and `ps_uros_start()` returns `ESP_ERR_NOT_SUPPORTED` harmlessly. Nodes must stay buildable
without it.

## Conventions

- Dual-core split on ESP32-S3: core 0 comms, core 1 control. Keep control loops off core 0.
- C99, snake_case, `esp_err_t` returns, `ESP_LOGx` with the node/component tag.
- Comments explain units, contract references (`network-map §3.3`) and ISR constraints —
  not what the code already says.
- Protocol logic is never duplicated. Python uses `powersuit_proto`; C and C++ compile
  `common/c`. If you find yourself re-implementing CRC, framing or ADPCM, stop.

## Pinned versions

ESP-IDF **v5.5.5** (`espressif/idf:v5.5.5`), new `esp_driver_twai` node API on both S3 and P4.
ROS 2 **Jazzy** (`ros:jazzy`); the micro-ROS agent has no Jazzy apt package and is vendored
via `orchestrator/ros2_ws/deps.repos`. Cloud targets Python **3.12** (`python:3.12-slim`) and
is developed on 3.14. Registry components: `lvgl/lvgl ~9.5`, `esp_lvgl_port ~2.8`,
`led_strip ~3.0.3`.
