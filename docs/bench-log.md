# Bench log

The record of the physical bench: what hardware exists, what was proven on it, and what
was found along the way. [`bringup.md`](bringup.md) is the *procedure*; this file is the
*record*. Write here when something is verified, not when it is planned. To pick the
project up again, start with [`resuming.md`](resuming.md).

## Summary

**Proven on silicon**, two ESP32-S3 boards over real CAN at 1 Mbit/s, no motors:

| What | Result | Section |
|---|---|---|
| SAFETY frames leave first | queued behind 12 telemetry frames, SAFETY went out 1st, not 13th | [1](#1-the-can-self-test-one-board-no-wiring) |
| The dead-man switch | trip on heartbeat loss ≈ 50 ms after the last beat; re-arm ≈ 250 ms after beats resume; two power configurations; 0 errors | [2](#2-the-dead-man-switch) |
| The real limb firmware | 1 kHz loop, worst iteration ~200–217 µs, 0 missed ticks; ~4 KB stack free | [3](#3-the-real-limb-firmware) |
| The one actuation gate | allowed every tick while commanded and OPERATIONAL; refused every tick from the trip until re-arm | [4](#4-the-one-actuation-gate-both-ways) |

The boards were borrowed from other projects for these tests and are not kept in this
configuration. To repeat any of it, follow [`bringup.md`](bringup.md) Appendix A and wire
per [`bench-wiring.html`](bench-wiring.html).

## Boards used

The results below name boards B and C. Identify a board by its CH343 serial number in
Device Manager, never by COM number.

| Board | MAC | CH343 serial |
|---|---|---|
| A | `b8:1f:3f:da:63:00` | 5CBC414102 |
| B | `b8:1f:3f:da:73:68` | 5C93086589 |
| C | `b8:1f:3f:da:81:60` | 5C93086538 |

## The hardware, as verified

**Silicon** (sibling project, 2026-10-01): esptool reports ESP32-S3 QFN56 rev v0.2, dual
core plus LP core at 240 MHz, 16 MB flash, 8 MB PSRAM, flash eFuse mode quad at 3.3 V.
Genuine N16R8 despite an AliExpress clone listing. **Security wide open** — Secure Boot
and Flash Encryption off, every key block empty — so the boards can be reflashed
indefinitely.

**USB:** the bridge is a **WCH CH343** (VID 1A86, PID 55D3), not the CP2102 both projects'
docs first assumed; Windows 11 had its driver. The two USB-C sockets look identical and
are silkscreened **`COM`** (the CH343 on UART0, GPIO43/44) and **`USB`** (native USB);
tape-label them. DTR/RTS auto-reset is wired, so flashing needs no buttons — and opening
a console with ordinary settings **resets the board** (pyserial: set `dtr` and `rts`
false before `open()` to avoid it).

**Pins** — checked against Espressif's DevKitC-1 user guides:

- the onboard RGB LED is on **GPIO48 on v1.0** and **GPIO38 on v1.1**. Espressif documents
  the move, not the reason; which revision these units are is not established, and
  `node_bench` drives both (the LED works).
- on boards with octal PSRAM, **GPIO35, 36 and 37** "are used for the internal
  communication between ESP32-S3 and SPI flash/PSRAM memory, thus not available for
  external use" — though the header offers them as ordinary pins.
- GPIO 33 and 34 are not on the DevKitC-1's headers.

**Transceivers** (EGBO SN65HVD230 / VP230 modules): a 4-pin header, `3.3V / GND / RX / TX`
read from one end and `TX / RX / GND / 3.3V` from the other — **match by printed label**;
CANH/CANL out twice in parallel, on a 2-pin header and a screw terminal. The seller's
schematic for these exact units draws **R1 = 10 kΩ from Rs to GND** (slope-control mode,
not changeable without soldering) and **R2 = 120 Ω across CANH/CANL** (onboard
termination: two modules make a correctly terminated two-node bus; a third would
over-terminate).

**Bitrate:** slope-control mode does **not** cap the bus. A sibling-project claim of a
250 kbit/s limit was withdrawn — it came from a model's summary of the datasheet, not the
datasheet. From TI's datasheet itself (SLOS346O): the part is "designed for data rates up
to 1 Mbps"; 10 kΩ gives ~15 V/µs edges and ~100 ns loop delay (~70 ns in high-speed
mode); no data-rate limit is given for slope control, whose cost is bus length; "250
kbps" is only the caption of Figure 41's typical waveform. Two 10 kΩ nodes over 20 cm need
~200 ns of a ~750 ns sample point. **1 Mbit/s has since run clean here** (sections 2–4).

**Host tooling:** esptool 5 spells commands with hyphens (`write-flash`, `chip-id`).
pyserial, esptool 5.3.1 and esp-idf-monitor 1.9.0 live in ESP-IDF's Python environment
(`%USERPROFILE%\.espressif\python_env\idf6.0_py3.14_env`), not the system Python. Docker
Desktop on Windows cannot reach a COM port, so firmware is built in Docker and flashed
from the host ([`bringup.md`](bringup.md) A.4d).

## 1. The CAN self-test, one board, no wiring

2026-10-16, board C, `node_bench` self-test mode ([`bringup.md`](bringup.md) §2): the TWAI
controller in self-test + loopback at 1 Mbit/s, RX on the TX pin, no transceiver. The
first Powersuit firmware on silicon, and the first host flash.

| Check | Result |
|---|---|
| basic | 200/200 frames out and back; 0 bus errors, 0 drops |
| **order** | **all 13 queued before any left: SAFETY came back 1st** — in arrival order, the driver's own FIFO, it would be 13th |
| fifo | 12/12 TELEM frames back in the order sent |
| order_concurrent | 2 frames back when SAFETY was queued; SAFETY 4th — those two, the one on the wire, then SAFETY |
| latency, idle | SAFETY send-to-receive 201–218 µs |
| latency, behind 12 queued | 268–284 µs, about two frame times (the FIFO inversion would cost ~1.7 ms) |
| totals | 536 frames; 0 rx_dropped, 0 tx_timeouts, 0 bus_errors |

The first run failed its ordering check, and **the test was wrong, not the firmware**: it
assumed twelve sends finish before one frame does, but each send wakes the
higher-priority transmit task, so two or three frames genuinely left first. The test was
made decisive (the test task briefly outranks the transmit task, so all thirteen are
queued before any leaves) rather than loosened. C's LED showed steady green, the pass
colour — the first confirmation that driving both LED pins works.

## 2. The dead-man switch

[`bringup.md`](bringup.md) A.5: B `node_bench` orchestrator, C `node_bench` limb. The owner
pulled B's power and plugged it back while C's console was recorded and its LED watched;
the LED matched every step.

| | Run 1: both on the PC, no ground tie | Run 2: B on the mains charger, ground tie fitted |
|---|---|---|
| OPERATIONAL after boot | 307 ms | 187 ms |
| B pulled | **`2 -> 3 (cause 6)`** — PASSIVE, COMM_LOSS | **`2 -> 3 (cause 6)`** |
| silence before the trip | **≈ 55 ms** | **≈ 53 ms** |
| B back: beats before re-arm | **≈ 259 ms** | **≈ 258 ms** |
| errors | 0 | 0 in 43 operational samples |

Designed: 50 ms watchdog, 250 ms re-arm ([`safety.md`](safety.md) §2). The timings are
**estimates from the receive counter** — frames arrive at a steady ~101/s, which dates the
last and first beat to within about a frame period; nothing on the bench timestamps a
beat. The node_bench limb never transmits, so the trip came from missing heartbeats alone.

**Two configurations are proven:** both boards on one PC with no ground tie (they share
ground through the PC), and one on the floating mains charger with the tie. One on the
charger *without* the tie has never been tried.

## 3. The real limb firmware

[`bringup.md`](bringup.md) A.6: C runs the actual `node_limb` image (arm_right, node 1) in
bench mode — no motor driver, encoders or current sense; everything else real, including
the micro-ROS client, which is vendored in this checkout. B keeps beating.

| | Result |
|---|---|
| boot and safety | up in ~350 ms; armed (50 ms watchdog, 250 ms re-arm); OPERATIONAL ~0.7 s after boot |
| **1 kHz control loop, core 1** | ~1,000 ticks/s; **worst iteration 196–217 µs** of 1,000; **0 missed ticks** |
| control-task stack | **~4,030 B free** of 6,144 |
| TELEM out, core 0 | **~607 frames/s** with IMU streams, **~307** after the fix below (no IMU fitted) |
| CAN | ~101 frames/s in (~121 once B sent position holds), 0 bus errors |

Two things this run found, both fixed and re-verified on C:

- **38 "missed ticks" were boot latency.** The count never grew after boot: the 1 kHz
  timer runs while the absent IMU's I2C times out (~38 ms), and the first iteration
  counted the backlog. The task now discards it; re-runs read 0.
- **"IMU telemetry suppressed" was not true.** With no IMU the limb still sent all three
  IMU streams at 100 Hz — zeros, including an all-zero quaternion, which is not a
  rotation. The sensor snapshot now carries a validity flag and nothing is sent without
  a reading: 607 → 307 frames/s.

## 4. The one actuation gate, both ways

B sends node 1 a 10 Hz position hold for both joints (JOINT_CMD, addressed, never
broadcast), so the limb's joints ask to be energised and the gate has to decide. In bench
mode `joint_write()` counts the ticks on which the gate *would* have energised a joint.
The owner pulled B for ~30 s:

| | C logged |
|---|---|
| OPERATIONAL, commanded | **would-actuate +~20,000 per 10 s** — 2 joints × 1,000 ticks, every tick allowed |
| B pulled | `2 -> 3 (cause 6)`, "setpoints zeroed, joints forced PASSIVE" |
| B absent | **would-actuate frozen at 73,816** across three reports — every tick refused |
| B back | `3 -> 2`, "PIDs reset", climbing again |

The freeze is exact: 7.8 s × 2,000/s from the last report predicts 73,818. With B
unpowered, C's telemetry had nobody to acknowledge it and failed at ~1,200 errors/s — the
normal behaviour of a CAN node left alone — while the loop kept time (211 µs, 0 missed).
C never went bus-off, since acknowledgement errors alone do not drive a node there, so the
trip cause stayed heartbeat loss; the errors stopped the moment B returned.

## Defects the bench found

| Defect | Found by | Status |
|---|---|---|
| **The TWAI driver sends in arrival order**, so a SAFETY frame could wait behind up to 15 of its own node's frames before reaching arbitration | the sibling project's CAN log, confirmed in IDF 5.5.5's `esp_twai_onchip.c` | **fixed**: `ps_can_txq` sends class order, one frame at a time; shown in section 1; recorded in `network-map.md` §2 |
| **`node_limb`'s pin map cannot run on an S3 DevKit**: GPIO 22–25 do not exist on the S3, 35–37 are the PSRAM, 38 the LED; 29 signals wanted against 23 safe pins | reading why `node_limb` never booted | **open — a hardware decision** ([`resuming.md`](resuming.md)); bench mode meanwhile; `flash_bench.ps1` refuses a non-bench limb image |
| `docs/bringup.md`'s flash step (`idf.py flash`) could never work: Docker cannot reach a COM port | checking the handover | fixed: `flash_bench.ps1` |
| Five apps silently ignored a `-D SDKCONFIG_DEFAULTS` overlay (bare `set()`) | building bench mode | fixed: guarded, as `node_bench` already was |
| `PS_CAN_BITRATE` was a Kconfig option nothing read | planning a bitrate fallback | fixed: every app reads it |
| The limb sent zero-filled IMU telemetry and logged that it did not | section 3 | fixed and re-verified |
| 38 phantom missed ticks at boot | section 3 | fixed and re-verified |
| Docs said GPIO47/48 sit on a 1.8 V rail on these boards | checking Espressif's guide | corrected: that holds only for 1.8 V modules; these are 3.3 V |

## Lessons from the sibling project

Potluck ran these boards for two weeks before Powersuit did; its bench datasheet
(`D:\Projects\potluck\bench\esp32-s3-bench-datasheet.md`, every fact tagged by how it is
known) was read on 2026-10-15.

| Lesson | Powersuit's response |
|---|---|
| The TWAI driver's transmit queue is FIFO | fixed and shown — see the defects table |
| CAN runs clean at 500 kbit/s on these modules | 500 kbit/s is the first fallback ([`bringup.md`](bringup.md) A.5); 1 Mbit/s has since run clean |
| The LED works with GPIO38 and GPIO48 both driven | `node_bench` does the same by default |
| Opening a console port resets the board | A.5: never open the orchestrator's console mid-test; `esp_idf_monitor --no-reset` |
| Identify pins by printed label, not position | A.4 says so |
| The receive queue overflowed at ~3,000 frames/s (128 entries) | **open**: the hub uses the same queue and bus 1's worst case is ~3,000 frames/s; measure `rx_dropped` at bring-up §3 before changing it |
| A lone CAN node cannot transmit | the self-test uses loopback with RX on the TX pin |
| ESP-NOW wedges with static Wi-Fi TX buffers *and* AMPDU | checked: `node_helmet` uses dynamic buffers |
| `esp_random()` is truly random only while the radio runs | Powersuit never calls it |
| A small power bank was too unstable for a board | use the charger or the PC |

**Also measured there, relevant to `node_helmet`** (the only Powersuit node with a radio):
the Wi-Fi stack costs 32,264 bytes of internal DRAM with ESP-NOW active, ESP-NOW itself
152; and this house's 2.4 GHz band is saturated, so radio latency tails here are the band
before they are firmware — a hypothesis per counter, never a blanket excuse.

## Not verified

- **The timings as measurements.** ≈ 50 ms and ≈ 250 ms are counter estimates; a GPIO
  toggle on the transition and a logic analyser would make them measurements
  ([`bringup.md`](bringup.md) §4).
- **The LED pin.** Both are driven; which one carries the LED on these units is unknown.
- **The charger configuration without the ground tie.** Never tried; do not.
- **Anything with more than two nodes, a hub, SPI, a Pi, an actuator, or a battery.**

## Log

| Date | Event |
|---|---|
| 2026-09-21 | Ordered — DKK 408 landed, AliExpress ([`bringup.md`](bringup.md) A.1) |
| 2026-10-01 | Arrived. The sibling project verified silicon, security state, USB bridge and pins. The transceiver schematic settles Rs = 10 kΩ and onboard 120 Ω; the 250 kbit/s claim is withdrawn |
| 2026-10-04/05 | The sibling project runs CAN on B and C at 500 kbit/s; later removes the transceivers so the boards can be spread around a room |
| 2026-10-15 | The sibling project's bench evidence read; `ps_can` sends in class order; `node_bench` drives both LED pins and joins CI |
| 2026-10-16 | Powersuit owns the bench. CAN self-test passes on C (section 1) |
| 2026-10-16 | The dead-man switch works on B and C, run 1 (section 2) |
| 2026-10-16 | Run 2: B on the mains charger with the ground tie (section 2) |
| 2026-10-16 | The real `node_limb` firmware runs on C in bench mode (section 3); its pin-map defect found |
| 2026-10-16 | The actuation gate shown both ways (section 4) |
| 2026-10-16 | The IMU-telemetry defect fixed and re-verified (section 3). **Project parked** — [`resuming.md`](resuming.md) |
