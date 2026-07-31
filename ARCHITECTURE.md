\## Powersuit System Architecture Overview



This Software Architecture Document outlines the distributed firmware and software stack for a \*\*9-node cybernetic exoskeleton suit\*\*. The system employs a hybrid topology:



\* \*\*Real-Time Edge Nodes (Nodes 1–7):\*\* ESP32 microcontrollers executing deterministic motor, sensor, and hardware routines via \*\*FreeRTOS\*\* and \*\*micro-ROS\*\*.

\* \*\*Central Suit Orchestrator (Node 8):\*\* A Single-Board Computer (SBC) running \*\*ROS 2\*\* for high-frequency sensor fusion, state estimation, and local telemetry bridging.

\* \*\*Cloud AI Interface (Node 9):\*\* High-throughput, remote neural compute server providing multimodal reasoning and natural language processing.



```

+-----------------------------------------------------------------------+

|                         NODE 9: CLOUD AI CORE                         |

|             (Large Multimodal Models, Long-Term Strategy)             |

+-----------------------------------------------------------------------+

&#x20;                                  ^

&#x20;                      5G / Low-Latency WebSockets

&#x20;                                  v

+-----------------------------------------------------------------------+

|                   NODE 8: CENTRAL SUIT ORCHESTRATOR                   |

|         (Raspberry Pi 5 / Jetson Orin - ROS 2, Sensor Fusion)         |

+-----------------------------------------------------------------------+

&#x20;           ^                                             ^

&#x20;      High-Speed SPI                             TWAI / CAN Bus

&#x20;           v                                             v

+------------------------+             +--------------------------------+

|  NODE 7: CHEST HUB     |             | NODES 1–6: LIMB, FLAP \& OPTICS |

| (ESP32-P4 - Power/BMS) |             |  (ESP32-S3 - micro-ROS Nodes)  |

+------------------------+             +--------------------------------+



```



\---



\## 1. Limb Nodes (Nodes 1–4)



\* \*\*Nodes:\*\* `node\_arm\_right`, `node\_arm\_left`, `node\_leg\_right`, `node\_leg\_left`

\* \*\*Hardware Target:\*\* ESP32-S3 (Dual-Core LX7 @ 240 MHz, Vector Extensions)



```

&#x20;      +---------------------------------------------------------+

&#x20;      |               ESP32-S3 Firmware Stack                   |

&#x20;      |  +---------------------------------------------------+  |

&#x20;      |  | App: FOC Motor Control, IMU Filtering, Safety Cut |  |

&#x20;      |  +---------------------------------------------------+  |

&#x20;      |  | Layer: micro-ROS Client (rclc + TWAI Transport)   |  |

&#x20;      |  +---------------------------------------------------+  |

&#x20;      |  | OS Kernel: FreeRTOS (Core 0: Comms | Core 1: Control)|  |

&#x20;      +---------------------------------------------------------+



```



\### Firmware Stack



\* \*\*Base OS:\*\* Espressif ESP-IDF (FreeRTOS)

\* \*\*Middleware:\*\* \*\*micro-ROS Client\*\* configured with custom TWAI (Two-Wire Automotive Interface / CAN bus) transport.

\* \*\*Libraries:\*\* SimpleFOC / ESP-IDF Motor Control PWM (MCPWM), ESP-DSP (for fast IMU filtering).



\### Primary Software Modules



1\. \*\*Field-Oriented Control (FOC) Loop (Core 1 - 1 kHz):\*\*

\* Computes precise current/voltage vectors for BLDC motor actuators in joints.

\* Reads high-speed encoder positions over SPI/SSI.





2\. \*\*Kinematic \& Force Estimation (Core 1 - 250 Hz):\*\*

\* Reads 6-DOF IMU data over I2C/SPI; executes an inline Kalman filter to compute orientation and acceleration.

\* Monitors strain-gauge analog-to-digital converters (ADCs) to detect suit-wearer contact forces.





3\. \*\*micro-ROS Publisher/Subscriber Task (Core 0 - 100 Hz):\*\*

\* \*\*Publishes:\*\* `/suit/telemetry/\[limb\_id]` (`sensor\_msgs/msg/JointState`, `sensor\_msgs/msg/Imu`).

\* \*\*Subscribes:\*\* `/suit/command/\[limb\_id]` (`trajectory\_msgs/msg/JointTrajectoryPoint`).







\### Safety \& Fallback Behavior



> \*\*Watchdog Protocol:\*\* If no valid CAN packet is received within 50 ms, the joint controller drops into a \*\*Passive Compliance Mode\*\* (free-wheeling or active dampening) to prevent joint locking or unwanted force applied to the wearer.



\---



\## 2. Flight \& Aero Flap Node (Node 5)



\* \*\*Node:\*\* `node\_flight\_actuation`

\* \*\*Hardware Target:\*\* ESP32-S3



\### Firmware Stack



\* \*\*Base OS:\*\* ESP-IDF (FreeRTOS)

\* \*\*Middleware:\*\* micro-ROS Client over TWAI.

\* \*\*Libraries:\*\* ESP32 Servo Driver / LEDC PWM, ESP-DL (Tiny Neural Network for aerodynamics estimation).



\### Primary Software Modules



1\. \*\*Multi-Channel Servo Synchronizer:\*\*

\* Controls 8–12 micro-linear actuators for back air-brakes and control surfaces.

\* Enforces geometric mechanical limits in hardware tasks to prevent structural collision.





2\. \*\*Aero Drag Calculation Engine:\*\*

\* Reads pitot-static tube pressure sensors to calculate airspeed and dynamic pressure ($q = \\frac{1}{2}\\rho v^2$).

\* Adjusts flap resistance dynamically based on airspeed.





3\. \*\*micro-ROS Interface:\*\*

\* \*\*Publishes:\*\* `/suit/aero/state` (`std\_msgs/msg/Float32MultiArray`).

\* \*\*Subscribes:\*\* `/suit/aero/target\_geometry` (`geometry\_msgs/msg/PoseArray`).







\---



\## 3. Helmet Audio, Display \& Optics Node (Node 6)



\* \*\*Node:\*\* `node\_helmet\_interface`

\* \*\*Hardware Target:\*\* ESP32-S3



\### Firmware Stack



\* \*\*Base OS:\*\* ESP-IDF with Dual-Core task pinout.

\* \*\*Middleware:\*\* micro-ROS Client over TWAI + ESP-NOW (for short-range external diagnostics).

\* \*\*Libraries:\*\* ESP-ADF (Audio Development Framework), ESP-DL (Wake-word detection), LVGL (Light and Versatile Graphics Library for HUD driver).



\### Primary Software Modules



1\. \*\*Local Audio Pipeline (Core 0):\*\*

\* \*\*I2S Microphone Capture:\*\* Runs local wake-word recognition (e.g., detecting "Jarvis") using 8-bit quantized models via \*\*ESP-DL\*\*.

\* \*\*I2S DAC Output:\*\* Handles bone-conduction headset sound output, local sound effects, and voice playback.





2\. \*\*HUD Optics Generator (Core 1):\*\*

\* Drives twin micro-OLED/MicroLED transparent lenses using SPI or I8080 display interfaces via LVGL.

\* Renders real-time telemetry overlays (battery levels, targeting reticles, system warnings).





3\. \*\*micro-ROS Interface:\*\*

\* \*\*Publishes:\*\* `/suit/voice/trigger` (`std\_msgs/msg/String`), `/suit/environment/ambient` (`sensor\_msgs/msg/Temperature`).

\* \*\*Subscribes:\*\* `/suit/hud/telemetry\_display` (`visualization\_msgs/msg/Marker`).







\---



\## 4. Chest Arc Power \& Network Hub Node (Node 7)



\* \*\*Node:\*\* `node\_chest\_power\_hub`

\* \*\*Hardware Target:\*\* ESP32-P4 (High-Performance Dual-Core RISC-V @ 400 MHz, Hardware Encryption, Dual CAN Controllers)



\### Firmware Stack



\* \*\*Base OS:\*\* ESP-IDF (FreeRTOS)

\* \*\*Middleware:\*\* Dual TWAI Controller Driver, micro-ROS Client, High-Speed SPI Master.



\### Primary Software Modules



1\. \*\*Battery Management System (BMS) Monitor:\*\*

\* High-frequency sampling of cell voltages, current draw, and thermistor temperatures across all suit power cells.

\* Implements rapid short-circuit detection (hardware interrupt response under 5 microseconds).





2\. \*\*Dual-CAN Gateway \& Message Routing:\*\*

\* Operates as the master switch between \*\*CAN-Bus 1 (Upper Limbs/Helmet)\*\* and \*\*CAN-Bus 2 (Lower Limbs/Flight)\*\*, bridging isolated loops to prevent bus congestion.





3\. \*\*High-Speed Bus Bridge to Orchestrator:\*\*

\* Uses DMA-accelerated SPI (up to 80 MHz) to bridge low-level CAN messages to the Central Suit Orchestrator (Node 8).





4\. \*\*Arc Power Visuals Driver:\*\*

\* Drives high-density addressable LEDs (WS2812/APA102) for suit power core state feedback.







\---



\## 5. Central Suit Orchestrator (Node 8)



\* \*\*Node:\*\* `node\_central\_orchestrator`

\* \*\*Hardware Target:\*\* Raspberry Pi 5 (8GB) or NVIDIA Jetson Orin Nano (Run-time SBC)



```

&#x20;      +---------------------------------------------------------+

&#x20;      |               ROS 2 / SBC System Architecture            |

&#x20;      |  +---------------------------------------------------+  |

&#x20;      |  | Application Layer: State Machine, Speech Synthesis |  |

&#x20;      |  +---------------------------------------------------+  |

&#x20;      |  | Sensor Fusion: robot\_localization (EKF IMU+Enc)   |  |

&#x20;      |  +---------------------------------------------------+  |

&#x20;      |  | Infrastructure: micro-ROS Agent (SPI + CAN Bridge)|  |

&#x20;      |  +---------------------------------------------------+  |

&#x20;      |  | OS: Ubuntu 24.04 LTS RT-Kernel (PREEMPT\_RT)        |  |

&#x20;      +---------------------------------------------------------+



```



\### Software Stack



\* \*\*OS:\*\* Ubuntu 24.04 LTS (patched with `PREEMPT\_RT` real-time kernel).

\* \*\*Robotics Framework:\*\* \*\*ROS 2 (Jazzy Jalisco / Humble)\*\*.

\* \*\*Local AI Runtime:\*\* ONNX Runtime / TensorRT-LLM (for local offline fallback voice commands).



\### Primary Software Modules



1\. \*\*micro-ROS Agent (`micro\_ros\_agent`):\*\*

\* Acts as the translation bridge between the physical CAN/SPI transport from Nodes 1–7 and the ROS 2 Data Distribution Service (DDS) environment.





2\. \*\*Whole-Body Controller \& State Estimator:\*\*

\* Runs `robot\_localization` (Extended Kalman Filter) combining data from all 6 limb/helmet IMUs to determine spatial orientation and trajectory.

\* Executes inverse kinematics (IK) calculations to calculate joint torque setpoints.





3\. \*\*Local NLP \& Telemetry Cache:\*\*

\* Translates local speech commands ("Status report", "Deploy airbrakes") instantly without waiting for cloud connectivity.

\* Maintains a ring-buffer state memory for instantaneous logging and black-box recovery.





4\. \*\*Cloud Gateway Client:\*\*

\* Handles dynamic bandwidth selection (5G/Wi-Fi/Satellite).

\* Streamlines secure TLS WebSockets / gRPC pipe to Node 9.







\---



\## 6. Cloud AI Interface / AI Core (Node 9)



\* \*\*Node:\*\* `cloud\_ai\_core`

\* \*\*Hardware Target:\*\* Remote GPU Cloud Compute Instance (e.g., NVIDIA H100/A10G cluster)



```

&#x20; Central Orchestrator (Node 8) ──\[gRPC / Protobuf Stream]──> 

&#x20; 

&#x20; +-----------------------------------------------------------------+

&#x20; | Cloud Infrastructure (Python / AsyncIO)                         |

&#x20; |  ├── 1. API Gateway / TLS Ingress Node                          |

&#x20; |  ├── 2. Multimodal LLM Engine (vLLM / TensorRT-LLM)             |

&#x20; |  ├── 3. Retrieval-Augmented Generation (RAG / Vector Database)  |

&#x20; |  └── 4. Real-time Voice Synthesis Engine (TTS)                  |

&#x20; +-----------------------------------------------------------------+



```



\### Software Stack



\* \*\*Runtime:\*\* Python 3.12, AsyncIO, Docker Containerized.

\* \*\*Inference Engines:\*\* \*\*vLLM\*\* / \*\*TensorRT-LLM\*\* (Large Multimodal Vision-Language Models).

\* \*\*Communication:\*\* gRPC over HTTP/2, WebSockets for low-latency streaming.



\### Primary Software Modules



1\. \*\*Multimodal Telemetry Synthesizer:\*\*

\* Consumes continuous high-level suit telemetry frames (battery consumption rates, temperature matrices, wearer vitals, environmental camera streams).





2\. \*\*Long-Term Context \& Strategy Engine:\*\*

\* Performs real-time Retrieval-Augmented Generation (RAG) against technical manuals, threat analysis databases, and historical mission logs.





3\. \*\*Streaming Voice / Neural TTS Engine:\*\*

\* Converts textual AI reasoning outputs into low-latency audio streams (using models like XTTS or Piper) and pushes raw audio buffers back to Node 8 for relay to Node 6 (Helmet).







\---



\## Distributed Software Matrix



| Node ID | Physical Location | Primary Processor | Core Software Stack | Primary Function |

| --- | --- | --- | --- | --- |

| \*\*Node 1\*\* | Right Arm | ESP32-S3 | FreeRTOS + micro-ROS | Elbow/Wrist FOC, Force Sensing |

| \*\*Node 2\*\* | Left Arm | ESP32-S3 | FreeRTOS + micro-ROS | Elbow/Wrist FOC, Force Sensing |

| \*\*Node 3\*\* | Right Leg | ESP32-S3 | FreeRTOS + micro-ROS | Hip/Knee FOC, Ground Impact Detection |

| \*\*Node 4\*\* | Left Leg | ESP32-S3 | FreeRTOS + micro-ROS | Hip/Knee FOC, Ground Impact Detection |

| \*\*Node 5\*\* | Back/Wings | ESP32-S3 | FreeRTOS + micro-ROS | Air-brake Servo Sync \& Drag Calc |

| \*\*Node 6\*\* | Helmet | ESP32-S3 | FreeRTOS + ESP-ADF + LVGL | Wake-word Detection, HUD Rendering |

| \*\*Node 7\*\* | Chest | ESP32-P4 | FreeRTOS + Dual-TWAI Gateway | Arc Reactor Power / BMS \& Bus Routing |

| \*\*Node 8\*\* | Spine/Suit Core | Raspberry Pi 5 | Ubuntu PREEMPT\_RT + ROS 2 | Sensor Fusion, State Machine, IK |

| \*\*Node 9\*\* | Cloud Server | GPU Instance | Python + vLLM + gRPC | Multimodal AI Reasoning \& TTS |

