# Bench log

The state of the physical bench: what hardware exists, what has been proven on
it, and what has not. [`bringup.md`](bringup.md) is the *procedure*; this file is
the *record*. Update it when something is verified, not when it is planned.

## Current status — 2026-10-16

| | |
|---|---|
| Hardware | **Arrived.** Verified on the real units by the sibling project (Potluck). |
| Boards right now | **In use by Potluck.** All three on the PC's USB ports; the CP2102 on board A and the PC |
| CAN | **No transceivers attached.** They were wired to B and C for Potluck's CAN tests, then removed so B and C could be placed apart in the room for radio tests. A physical tether, not an electrical problem |
| Powersuit on silicon | **Nothing yet.** No Powersuit image has been flashed to any board. |
| Images | **Both `node_bench` roles built and verified** (2026-10-01) — `bash firmware/tools/build_bench.sh` |
| Next | When Powersuit gets the bench: wire B and C per [A.4](bringup.md), flash with `firmware/tools/flash_bench.ps1` ([A.4d](bringup.md)), run [A.5](bringup.md) |

### Powersuit's bench, when it is Powersuit's turn

**B = orchestrator (COM4), C = limb (COM5).** B and C are the pair Potluck's CAN ran
clean on, and A keeps the CP2102 so Potluck's frame link is not disturbed. Each of B
and C gets one transceiver wired per [A.4](bringup.md); A is not involved and needs
no wires. The sibling project's drawing of this exact wiring, with the modules as
they sit on the bench, is `D:\Projects\potluckench\m4-can-wiring.html`: the same
ten wires as [A.4](bringup.md), plus the board-to-board ground A.4 adds. Flashing
`node_bench` replaces Potluck's firmware on B and C; Potluck's own
builds restore it afterwards, and the factory images are backed up by MAC.

### Before touching a board

The boards are shared. While a Potluck soak is running, **do not** unplug a
board, reflash one, press any button on one, or kill any `python.exe` process —
any of those ends a day of measurement. Check first:

```powershell
D:\Projects\potluck\tools\soak.ps1 -Status
```

Under Potluck firmware, **GPIO0 (the BOOT button) is a "leave the cluster"
button**: pressing it makes a node announce its departure and go quiet. That
stops mattering once `node_bench` is flashed.

## Board register

Physical identity, which survives reflashing. COM numbers are those assigned on
the development PC and may differ elsewhere; the CH343 serial number is what
identifies a board in Device Manager.

| Board | MAC | COM | CH343 serial | Factory LED colour (frozen) |
|---|---|---|---|---|
| A | `b8:1f:3f:da:63:00` | COM3 | 5CBC414102 | blue |
| B | `b8:1f:3f:da:73:68` | COM4 | 5C93086589 | — |
| C | `b8:1f:3f:da:81:60` | COM5 | 5C93086538 | green |

Factory images are backed up in `D:\esp\board-backups\`, named by MAC. Board A
is a full 16 MB dump; B and C are the first 2 MB, which covers everything in use.
Restore with:

```powershell
python -m esptool --port COMx write-flash 0 <file>
```

## Verified on these units

All verified on 2026-10-01 by the sibling project. Recorded here so it is not
re-derived.

**Silicon.** esptool reports ESP32-S3 QFN56 rev v0.2, dual core plus LP core at
240 MHz, 16 MB flash, 8 MB PSRAM, flash eFuse mode quad at 3.3 V. Genuine N16R8,
despite an AliExpress clone listing with no Espressif seal and recycled ESP32-C3
marketing copy.

**Security state is wide open**, which is what a bench wants: Secure Boot off,
Flash Encryption off, all six key blocks empty, `SPI_BOOT_CRYPT_CNT` 0. Nothing is
locked; the boards can be reflashed indefinitely.

**USB bridge: WCH CH343** (VID 1A86, PID 55D3), not the CP2102 that both
projects' docs assumed. Windows 11 already carried the driver — nothing needed
installing.

**Sockets: `COM` and `USB`**, visually identical USB-C, tape-labelled. `COM` is
the CH343 on UART0; `USB` is the chip's native USB.

**Flashing:** no buttons needed — DTR/RTS auto-reset is wired, so it scripts
cleanly. 730 KB in 11.5 s, hash verified.

**Pins:** the [A.2](bringup.md) table matches the real silkscreen. GPIO4 and 5
are present and free for TWAI. GPIO35–37 are broken out on the bottom header as if
they were ordinary GPIOs; on an R8 part they are the octal PSRAM and must never be
used.

**Transceivers** (used by the sibling project on B and C at 500 kbit/s, 2026-10-04/05;
pinout from the seller's photos of the exact units): 4-pin header `3.3V / GND / RX / TX`; no RS pin exposed; CANH/CANL out
twice in parallel, on a 2-pin header and a screw terminal.

**Transceiver schematic.** The seller's listing for these exact units includes
the module schematic. It draws **R1 = 10 kΩ from Rs to GND** and **R2 = 120 Ω
across CANH/CANL**. So:

- **Termination is onboard**, consistent with the `121` marking seen in the
  photos. Two modules give the ~60 Ω a two-node bus wants; no resistors to buy,
  and no multimeter needed to establish it.
- **Rs is fixed at 10 kΩ**, not brought out to the header, so not changeable
  without soldering. That puts the SN65HVD230 in *slope-control* mode.

Both are confirmed by drawing rather than by measuring these units, which is
strong evidence for a schematic published against the exact listing.

**Transceiver bitrate.** The sibling project first concluded that slope-control
mode limits the bus to 250 kbit/s, then withdrew the claim: it came from a model's
summary of the datasheet, not the datasheet. Checked against the text of TI's
datasheet itself (SLOS346O, rev. April 2018):

- 10 kΩ gives ~15 V/µs edges and ~100 ns loop delay, against ~70 ns in
  high-speed mode. 100 kΩ gives ~2 V/µs and ~500 ns.
- The part is "designed for data rates up to 1 Mbps". The datasheet gives no
  data-rate limit for slope-control mode; the cost it names is bus length.
- "250 kbps" appears only as the caption of Figure 41, a *typical* waveform
  measured at that rate. It is a test condition, not a ceiling.
- At 1 Mbps the bit is 1000 ns and the sample point ~750 ns. Two 10 kΩ
  transceivers plus 20 cm of wire need ~200 ns of round trip. In spec, with
  margin. At 100 kΩ it would not be.

So the bench runs at the contract's 1 Mbps, with 250 kbit/s as a one-variable
fallback (`BENCH_CAN_BITRATE=250000`, see [bringup.md](bringup.md) A.5) if the
hardware disagrees with the datasheet. That fallback was built and role-checked
before any board was touched.

**Host tooling.** esptool v5 spells commands with hyphens (`chip-id`, `flash-id`,
`read-flash`, `get-security-info`, `write-flash`); the underscore forms are
deprecated and warn. pyserial is not in the system Python — it lives in ESP-IDF's
environment (`%USERPROFILE%\.espressif\python_env\idf6.0_py3.14_env`), which also
carries esptool 5.3.1 and esp-idf-monitor 1.9.0. Anything that reads a serial port
directly should use that interpreter.

## Measured on this hardware, relevant to Powersuit nodes

**Wi-Fi stack cost: 32,264 bytes of internal DRAM** with ESP-NOW active, of which
ESP-NOW itself is only 152 bytes. The expensive part is the radio underneath, not
the protocol. This matters for `node_helmet`, the only Powersuit node that brings
up Wi-Fi (for its ESP-NOW diagnostics link). `node_bench` uses no radio.

**The RF environment is saturated** with Wi-Fi and Bluetooth. For anything radio
here, latency tails, retries and bursty loss are the expected signature of the
band, not firmware faults — but treat that as a hypothesis per counter, never a
blanket excuse. Ask whether RF can physically move the counter in question. The
CAN bench is unaffected: it is wired.

## Lessons from the sibling project, and what Powersuit did about them

Potluck has run these boards continuously since they arrived. Its bench datasheet
(`D:\Projects\potluck\bench\esp32-s3-bench-datasheet.md`, every row tagged
measured / owner / seller / datasheet) and its CAN session log are the sources. Read
2026-10-15, before Powersuit's first flash.

| Lesson | Applies to | Powersuit's response |
|---|---|---|
| **The TWAI driver's transmit queue is FIFO.** A high-priority frame queued behind the same node's traffic took 2.5–5.5 ms to leave, against ~0.3 ms of wire time; it never lost arbitration because it never reached it. Fixed there by transmitting one frame at a time in priority order: 335–683 µs under a 3,000 frames/s flood | every node; the hub most | **Confirmed in IDF 5.5.5's source** (`esp_twai_onchip.c`: `tx_mount_queue` is a FreeRTOS queue). `ps_can` now holds pending frames in class order and gives the driver one at a time (`ps_can_txq`, host-tested). Recorded in `network-map.md` §2. *Unproven on silicon.* |
| **CAN runs clean at 500 kbit/s** on these modules, straight wiring, ~20 cm, no module-to-module ground (boards on one PC) | `node_bench` | 500 kbit/s is now the first fallback, ahead of 250 ([bringup.md](bringup.md) A.5). 1 Mbps remains unrun |
| **The RGB LED lights when GPIO38 and GPIO48 are both driven** | `node_bench` | Now the default; the revision question no longer blocks anything (A.3) |
| **Opening the console port resets the board** (pyserial defaults toggle DTR/RTS; set both false *before* `open()` to avoid it) | A.5 | Documented: never open the orchestrator's console mid-test; `esp_idf_monitor --no-reset` to watch a running board |
| **Identify pins by printed label**, never position: the transceiver header reads TX/RX/GND/3.3V from one end and 3.3V/GND/RX/TX from the other | A.4 | A.4 now says so |
| **The receive ISR queue overflowed** 220 times in 881,583 frames at ~3,000 frames/s (128 entries, drained too slowly) | `node_chest_hub` | The hub uses the same 128-entry queue, and bus 1's worst case is ~2,800 frames/s (`network-map.md` §10). Not changed blind: watch `rx_dropped` on the hub at bring-up step 3, and drain faster or add an acceptance filter if it moves |
| A lone CAN node cannot transmit (every frame needs an ACK); use loopback with RX on the TX pin | bring-up step 2 | Already in `bringup.md` §2 |
| **ESP-NOW wedges for good** (`ESP_ERR_ESPNOW_NO_MEM` after 2.5–5 min) with static Wi-Fi TX buffers **and** AMPDU on; `SPIRAM_TRY_ALLOCATE_WIFI_LWIP` forces static buffers | `node_helmet` | Checked: the helmet uses dynamic TX buffers, so it is not in the trap. Do not enable Wi-Fi/lwIP buffers in PSRAM without turning AMPDU off |
| A Wi-Fi station hunting for a router it cannot find scans every channel and drops ESP-NOW peers; adjacent channels leak at desk range | `node_helmet` | The helmet does not join a router. Recorded for if it ever does |
| `esp_random()` is truly random only while the radio runs | any key or nonce | Powersuit calls `esp_random` nowhere; recorded for when something does |
| A small power bank was too unstable for a board; mains and PC ports were fine | power | Use the charger or the PC |
| The CP2102 dongle back-powers through its signal lines if left wired to a board while unplugged | Potluck only | Not used by Powersuit |
| Console at 115200; reading at the wrong rate shows nothing at all | A.4d | `node_bench` uses 115200, which esp-idf-monitor defaults to |

## Not yet verified

- **RGB LED pin** — GPIO38 (v1.1) or GPIO48 (v1.0). Potluck's firmware never
  drives the LED, so nothing was learned. Use the [A.3](bringup.md) probe mode.
- **1 Mbps on real wire.** The datasheet says it is in spec; the bench has not
  shown it yet. `bus_errors` in A.5 is the first evidence either way.
- **The [A.4d](bringup.md) flash sequence** onto a board. Checked without one: the
  script's dry run, its wrong-role guard, and esptool accepting every generated
  argument (it failed only at opening a nonexistent port).
- **Everything in [A.5](bringup.md)** — above all the dead-man switch, which is the
  reason this bench exists.

## Log

| Date | Event |
|---|---|
| 2026-09-21 | Ordered — DKK 408 landed, AliExpress ([A.1](bringup.md)) |
| 2026-10-01 | Arrived. Potluck verified silicon, security state, bridge chip and pins; backed up factory images; started a 24 h soak (~19:20) |
| 2026-10-01 | Both `node_bench` roles built in Docker and role-verified; host flash script dry-run checked. No board touched |
| 2026-10-16 | CAN transceivers removed from B and C (the wires tethered them, and Potluck's radio tests needed them apart); all three boards on the PC's USB, CP2102 on A. Powersuit's turn: B orchestrator, C limb |
| 2026-10-15 | Read the sibling project's two weeks of bench evidence. `ps_can` now transmits in class order (TWAI's FIFO), `node_bench` drives both LED pins and builds in CI, 500 kbit/s is the first fallback. No board touched |
| 2026-10-01 | Transceiver schematic found in the listing: Rs = 10 kΩ (slope control) and 120 Ω termination onboard. Sibling project's 250 kbit/s limit withdrawn; bench stays at 1 Mbps |
