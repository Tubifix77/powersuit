# Powersuit

Distributed firmware and software for a nine-node cybernetic exoskeleton: seven ESP32
edge nodes on two isolated CAN buses, a Raspberry Pi 5 class orchestrator running ROS 2,
and a GPU cloud service reachable over 5G.

The system specification is [`ARCHITECTURE.md`](ARCHITECTURE.md). The *normative engineering
contract* — the one every node actually codes against — is [`docs/network-map.md`](docs/network-map.md),
with safety semantics in [`docs/safety.md`](docs/safety.md) and the cloud link in
[`docs/link-protocol.md`](docs/link-protocol.md). Where reality forced a departure from the
original specification, it is recorded in [network-map §12](docs/network-map.md#12-deviations-from-architecturemd)
rather than quietly implemented.

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

The two buses are deliberately isolated; Node 7 is the only bridge between them, and the
only path to the orchestrator. Nothing on a limb bus can talk to the cloud except through
two hops that are both allowed to say no.

## Repository layout

| Path | What lives there |
|------|------------------|
| `common/c`, `common/python` | The protocol contract, implemented twice and locked together by generated test vectors: CAN ID codec, 8-byte wire structs, SPI framing, CRC16, IMA ADPCM, cloud envelope |
| `firmware/components` | Shared ESP-IDF components: `ps_can`, `ps_safety`, `ps_router`, `ps_spibridge`, `ps_uros`, `ps_ctl`, `ps_audio`, `ps_proto` |
| `firmware/apps` | One ESP-IDF project per node: `node_limb` (Nodes 1–4 via Kconfig), `node_flight`, `node_helmet`, `node_chest_hub` |
| `firmware/tests/host` | Every pure-C module compiled and tested on the host under `-Werror` |
| `orchestrator/ros2_ws` | Node 8 ROS 2 Jazzy workspace |
| `cloud` | Node 9 service and its container |
| `tools/suit_sim` | End-to-end simulation of the whole suit, in-process |
| `docs` | The normative contract |

## Design rules worth knowing before you change anything

**Rate policy.** Anything faster than 20 Hz travels as a packed 8-byte TELEM frame, and
Node 8 synthesises the ROS message. This is not an optimisation: a single
`sensor_msgs/Imu` over micro-ROS is roughly 70 CAN frames, so one limb publishing at
100 Hz would exceed an entire 1 Mbps bus. With the TELEM plane, worst-case load is about
42% on bus 1 and 30% on bus 2 ([§10](docs/network-map.md)).

**The heartbeat is a dead-man switch.** Only Node 8 emits it, at 100 Hz. Any actuating node
that goes 50 ms without one drops into Passive Compliance. No component may synthesise,
repeat, or extrapolate a heartbeat on behalf of Node 8 — if the SPI link dies, the beats
stop and the suit goes limp, which is the correct failure direction.

**The cloud cannot move the suit.** Node 8's gateway accepts exactly six message types from
Node 9 — speech, advisories, HUD hints and session control. There is deliberately no
message by which the cloud can reach an actuator, and `suit_sim` scenario 10 exists to prove
a local safety trip beats any cloud round-trip.

**Hardware does what hardware must.** The specified sub-5 µs battery short-circuit trip is an
analog comparator latching the gate in under a microsecond; firmware is the observer that
timestamps it, broadcasts the e-stop and manages the interlocked re-arm. A software-only
guarantee at that timescale would not be honest ([safety §4](docs/safety.md)).

## Building and testing

Everything below runs from the repository root. The Python tiers run natively; firmware and
ROS 2 build in containers, so Docker is the only hard prerequisite for those.

### Python: protocol, cloud, and the integration simulator

```bash
python -m venv .venv && source .venv/Scripts/activate
pip install -e ./common/python -e "./cloud[dev]" -e ./tools/suit_sim
pytest common/python/tests cloud/tests tools/suit_sim/tests -q
```

Regenerating the cross-language vectors after a protocol change (this rewrites both the
Python fixture and the C header, and CI fails if you forget):

```bash
python common/python/tests/gen_vectors.py
```

### C host tests — every pure module, under `-Wall -Wextra -Werror`

```bash
docker run --rm -v "$PWD:/repo" -w /repo ros:jazzy bash -c "cmake -S firmware/tests/host -B /tmp/host && cmake --build /tmp/host -j && ctest --test-dir /tmp/host --output-on-failure"
```

### Firmware

One image serves all four apps. The build directory and ccache live in named volumes
because NTFS bind mounts are slow:

```bash
docker volume create ps_fw_build && docker volume create ps_ccache
```

```bash
docker run --rm -v "$PWD:/ws" -v ps_fw_build:/builds -v ps_ccache:/root/.ccache -w /ws/firmware/apps/node_chest_hub -e IDF_CCACHE_ENABLE=1 espressif/idf:v5.5.5 bash -c "git config --global --add safe.directory '*'; idf.py -B /builds/hub build"
```

Substitute `node_limb`, `node_flight` or `node_helmet` for the other targets. `node_limb`
covers Nodes 1–4; select which one with `CONFIG_PS_NODE_ID` via `idf.py menuconfig`.

micro-ROS is an optional dependency. Without it the XRCE plane compiles out and the node
still runs every safety, control, telemetry and audio function — which is what you want for
bench bring-up. To enable it:

```bash
bash firmware/tools/fetch_deps.sh
```

### ROS 2 workspace

```bash
docker volume create ps_ros_deps && docker volume create ps_ros_out
docker run --rm -v "$PWD:/repo:ro" -v ps_ros_deps:/deps -v ps_ros_out:/out -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros:jazzy bash /repo/tools/ros_ci.sh
```

The first run imports and builds the micro-ROS agent from source, which takes a while; the
named volumes make subsequent runs quick.

## Running the suit without hardware

`tools/suit_sim` wires fake limbs and a fake hub over virtual CAN buses to a bridge speaking
the real 512-byte SPI framing, and connects it to the *actual* Node 9 server over a real
WebSocket. The ten scenarios cover watchdog trips and re-arm timing, e-stop propagation and
replay rejection, SPI corruption recovery, voice round-trip through the cloud, link failover
with session resume, and the local-beats-cloud ordering guarantee.

```bash
pytest tools/suit_sim/tests -q
```

For the ROS 2 side against the same simulated hub, `suit_bringup`'s `sim.launch.py` starts
the real bridge with its mock transport pointed at the simulator.

## Deploying

- **Node 8**: `orchestrator/scripts/install_pi.sh` provisions Ubuntu 24.04 on a Pi 5 and
  installs the systemd unit in `orchestrator/systemd/`.
- **Node 9**: `docker compose -f cloud/docker-compose.yml up app`. The default engine is a
  deterministic mock. Point it at a real model with the `ollama` profile (CPU, no account
  needed) or the `gpu` profile (vLLM), both through the same OpenAI-compatible adapter.
- **Nodes 1–7**: `idf.py flash` per app; see the per-app board headers for the bench pinouts.
