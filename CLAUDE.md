# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository state

Greenfield project. The only content is `ARCHITECTURE.md`, the Software Architecture Document for the system below. There is no source code, build system, or test tooling yet — there are no build/lint/test commands to document. When code lands, the intended toolchains are ESP-IDF for Nodes 1–7 firmware, ROS 2 (colcon) for Node 8, and Python 3.12 / Docker for Node 9; update this file with the real commands once they exist.

Note: `ARCHITECTURE.md` is written in escaped markdown (literal `\*\*`, `\##`, `\_`, and `&#x20;` entities — an export artifact). When editing it, match the existing escaping or normalize the whole file consistently; don't mix styles.

## System architecture

Powersuit is the distributed firmware/software stack for a 9-node cybernetic exoskeleton, in three tiers:

1. **Real-time edge nodes (Nodes 1–7)** — ESP32 microcontrollers on FreeRTOS (ESP-IDF) running micro-ROS clients over TWAI (CAN bus). Nodes 1–6 are ESP32-S3 (limbs `node_arm_*`/`node_leg_*`, flight `node_flight_actuation`, helmet `node_helmet_interface`); Node 7 is ESP32-P4 (`node_chest_power_hub`) handling BMS, dual-CAN gateway routing, and the SPI bridge upstream.
2. **Central Suit Orchestrator (Node 8, `node_central_orchestrator`)** — Raspberry Pi 5 / Jetson Orin Nano, Ubuntu 24.04 with PREEMPT_RT, ROS 2 (Jazzy/Humble). Runs the micro-ROS agent, EKF sensor fusion (`robot_localization`), inverse kinematics, local offline NLP fallback, and the cloud gateway client.
3. **Cloud AI Core (Node 9, `cloud_ai_core`)** — Python 3.12 / AsyncIO / Docker on GPU instances: vLLM / TensorRT-LLM multimodal inference, RAG, streaming TTS back down to the helmet.

Per-node module breakdowns and the full node/hardware/function matrix are in `ARCHITECTURE.md`.

### Communication topology

- Nodes 1–6 sit on two isolated CAN buses — Bus 1: upper limbs + helmet; Bus 2: lower limbs + flight. Node 7 is the gateway bridging the two loops (deliberately isolated to prevent bus congestion).
- Node 7 bridges CAN traffic to Node 8 over DMA-accelerated SPI (up to 80 MHz).
- Node 8's micro-ROS agent translates the CAN/SPI transport into the ROS 2 DDS graph.
- Node 8 ↔ Node 9 uses TLS WebSockets / gRPC over HTTP/2, with dynamic 5G/Wi-Fi/satellite link selection.

### Conventions and invariants

- **ROS topic namespace:** everything lives under `/suit/…` (e.g. `/suit/telemetry/[limb_id]`, `/suit/command/[limb_id]`, `/suit/aero/state`, `/suit/voice/trigger`, `/suit/hud/telemetry_display`), using standard ROS message types (`sensor_msgs`, `trajectory_msgs`, `geometry_msgs`, `visualization_msgs`, `std_msgs`).
- **Dual-core task split on ESP32-S3 firmware:** Core 0 = comms (micro-ROS pub/sub at 100 Hz), Core 1 = control (FOC loop at 1 kHz, kinematics/force estimation at 250 Hz). Keep control loops off the comms core.
- **Safety invariants** (preserve in any control-path change):
  - Limb nodes drop into Passive Compliance Mode (free-wheeling/active dampening) if no valid CAN packet arrives within 50 ms.
  - Node 7 BMS short-circuit detection responds via hardware interrupt in under 5 µs.
  - Node 5 enforces flap geometric/mechanical limits in hardware tasks to prevent structural collision.
- **Offline-first voice:** Node 8 handles local speech commands without cloud connectivity (ONNX Runtime / TensorRT-LLM fallback); Node 9 is for long-horizon multimodal reasoning, not the critical path.
