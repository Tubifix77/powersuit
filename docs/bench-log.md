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
| Powersuit on silicon | **The dead-man switch works on the two-board bench** (2026-10-16). B is the orchestrator, C the limb, wires 1–10 of [`bench-wiring.html`](bench-wiring.html), no ground tie (both on the PC); A still runs Potluck |
| Images | Self-test built and on C. Both `node_bench` roles need a rebuild before flashing: the ones on disk predate `ps_can_txq` |
| Next | Powersuit has the bench (2026-10-16). First the one-board CAN self-test on C, no wiring ([bringup.md](bringup.md) §2); then wire B and C per [A.4](bringup.md), flash both roles ([A.4d](bringup.md)), run [A.5](bringup.md) |

### Powersuit's bench, when it is Powersuit's turn

**B = orchestrator (COM4), C = limb (COM5).** B and C are the pair Potluck's CAN ran
clean on, and A keeps the CP2102 so Potluck's frame link is not disturbed. Each of B
and C gets one transceiver wired per [A.4](bringup.md); A is not involved and needs
no wires. The sibling project's drawing of this exact wiring, with the modules as
they sit on the bench, is `D:\Projects\potluck\bench\m4-can-wiring.html`: the same
ten wires as [A.4](bringup.md), plus the board-to-board ground A.4 adds. Powersuit's own
version, with that eleventh wire and the roles, is [`bench-wiring.html`](bench-wiring.html). Flashing
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

## First Powersuit firmware on silicon — 2026-10-16

Board C, `node_bench` in CAN self-test mode ([bringup.md](bringup.md) §2): the TWAI
controller in self-test + loopback at 1 Mbit/s, RX on the TX pin, no transceiver.
Flashed from the host with `flash_bench.ps1` (A.4d, first run: every image hash
verified), read with `read_console.py`. Log: `.verify-logs/selftest_C.log`.

| Check | Result |
|---|---|
| basic | 200/200 frames out and back; 0 bus errors, 0 drops, 0 refused |
| **order** | **All 13 queued before any left: SAFETY came back 1st.** In arrival order — the driver's own FIFO — it would have been 13th. `ps_can_txq` confirmed on silicon |
| fifo | 12/12 TELEM frames back in the order sent |
| order_concurrent | 2 frames back when SAFETY was queued; SAFETY came back 4th — those two, the one on the wire, then SAFETY |
| latency, idle | SAFETY send-to-receive 201–218 µs over 50 frames |
| latency, behind 12 queued TELEM | 268–284 µs over 20 runs: about two frame times, where the FIFO inversion would have cost ~1.7 ms |
| totals | 536 frames sent and received; 0 rx_dropped, 0 tx_timeouts, 0 bus_errors |

**The first run failed one check, and the test was wrong, not the firmware.** It assumed
twelve `ps_can_send` calls finish before the first frame does. They do not: each wakes
the higher-priority transmit task, so queuing takes about three frame times and two or
three TELEM frames genuinely leave first. SAFETY came back 4th, having overtaken
everything still waiting — correct behaviour, judged against the wrong number. Rather
than loosen the bound, the test was made decisive (the test task briefly outranks the
transmit task on the same core, so all thirteen are queued before any leaves: SAFETY
must be first, with no tolerance), and the realistic concurrent case kept as its own
check with a bound computed from what had actually left.

**The LED works with both pins driven:** C showed steady green, the pass colour, on the
owner's look. Which pin carries it is still not established and need not be. A and B
meanwhile showed red, which in Potluck's scheme means a peer is dead (over 50 %
heartbeat loss): correctly reporting C's departure from the cell, not a fault.

Also shown by this run: host flashing works (bootloader, partition table and app, 3.4 s
for the app), Powersuit firmware boots on these boards, and `read_console.py` captures
a board from boot. Not shown: anything on a real bus — loopback has no transceiver, no
second node, and no arbitration.

## The dead-man switch, on silicon — 2026-10-16

[bringup.md](bringup.md) A.5. B runs `node_bench` as the orchestrator (COM4), C as the
limb (COM5), both on the PC's USB, wired with wires 1–10 of
[`bench-wiring.html`](bench-wiring.html) — **without wire 11, the board-to-board ground
tie**: both boards shared ground through the PC's USB, as in the sibling project's
CAN run. So the ten-wire, same-PC configuration is the one proven; the
charger-powered configuration, which needs the tie, has not been run. 1 Mbit/s. C's console recorded from boot for two minutes
(`.verify-logs/deadman_C.log`) while the owner pulled B's USB cable and plugged it back
in, watching C's LED.

| Step | Owner saw on C | C logged | |
|---|---|---|---|
| 1 boot, B already beating | white-blue pulse | `0 -> 1` at 101 ms, `1 -> 2` (OPERATIONAL) at 307 ms | ✓ |
| running ~60 s | white-blue pulse | ~102 frames/s (100 Hz heartbeat + 1 Hz MODE_SET), **0 errors** | ✓ |
| 2 **B's USB pulled** | **amber at once** | **`2 -> 3 (cause 6)`** at 60,957 ms — OPERATIONAL to PASSIVE, cause COMM_LOSS | ✓ |
| B absent ~16 s | amber | `rx` frozen at 6143, C silent, 0 errors | ✓ |
| 3 **B plugged back in** | back to white-blue after about a second | **`3 -> 2`** at 77,522 ms | ✓ |
| after | white-blue | steady, 0 errors, to the end of the capture | ✓ |

**Timing, estimated from the receive counter** (frames arrive at a steady ~101.5/s, so
the counter dates the last and first heartbeat to within about one frame period;
nothing on the bench timestamps a beat directly):

- **trip:** last heartbeat ≈ 60,902 ms, PASSIVE at 60,957 ms — **≈ 55 ms of silence**,
  against the designed 50 ms watchdog.
- **re-arm:** heartbeats resume ≈ 77,263 ms, OPERATIONAL at 77,522 ms — **≈ 259 ms**,
  against the designed 250 ms of continuous beats plus a fresh command
  ([safety.md](safety.md) §2).

Both are consistent with the design; neither is an instrumented measurement. The
GPIO-toggle-and-scope check in [bringup.md](bringup.md) §4 would make them one. C
never transmits in this role (`tx=0`), so removing B cost C no acknowledgements: the
trip came from the missing heartbeats alone, as its cause code says, not from a
bus-off.

**What this proves, and what it does not.** The safety state machine, `ps_safety`'s
watchdog, `ps_can` with its new transmit path, and the CAN wiring all behave as
specified on real silicon and real wire — the one behaviour Appendix A exists to
prove. It is two nodes at 1 Mbit/s on 20 cm, with no actuator, no hub, no SPI and no
load; everything above the dead-man switch in [bringup.md](bringup.md) is still
unproven.

### Second run: B on the mains charger, with wire 11

The same test with B powered from the 6-port charger — whose output floats — and the
board-to-board ground tie fitted (J3 `G` to J3 `G`). **On the real bench wire 11 is
green**, not black as drawn, so two wires are green: CANL between the modules on the
far left, wire 11 between the boards' J3 headers on the far right. They share no end.
C on the PC, recorded for 100 s (`.verify-logs/deadman_charger_C.log`); the owner
pulled B's lead from the charger and plugged it back, and saw the same LED sequence.

| | C logged |
|---|---|
| boot, B beating from the charger | OPERATIONAL at 187 ms |
| B's lead pulled | **`2 -> 3 (cause 6)`** at 23,197 ms |
| B plugged back | **`3 -> 2`** at 35,742 ms |
| errors | **0** in every one of 43 operational samples, and while B was gone |

Estimated from the receive counter as before (steady 101.0 frames/s): **≈ 53 ms** of
silence before the trip and **≈ 258 ms** of beats before re-arm — within a few
milliseconds of the first run's 55 and 259. So both configurations are proven: two
boards on one PC with no tie, and one board on a floating charger with the tie. One
board on the charger *without* the tie has not been tried and should not be.

## Not yet verified

- **RGB LED pin** — GPIO38 (v1.1) or GPIO48 (v1.0). Potluck's firmware never
  drives the LED, so nothing was learned. Use the [A.3](bringup.md) probe mode.
- **1 Mbps on real wire.** The datasheet says it is in spec; the bench has not
  shown it yet. `bus_errors` in A.5 is the first evidence either way.
- **The dead-man timing as a measurement.** ≈ 55 ms and ≈ 259 ms above are estimates
  from a frame counter; a GPIO toggle on the transition and a scope or logic analyser
  would make them measurements ([bringup.md](bringup.md) §4).

## Log

| Date | Event |
|---|---|
| 2026-10-16 | Dead-man test repeated with B on the mains charger and wire 11 fitted (green on the real bench): trip cause 6 at ≈ 53 ms, re-arm ≈ 258 ms, 0 errors |
| 2026-10-16 | **The dead-man switch works on silicon.** B orchestrator, C limb, 1 Mbit/s, 0 errors. Pulling B: C to PASSIVE, cause COMM_LOSS, ≈ 55 ms after the last beat; plugging B back: OPERATIONAL ≈ 259 ms after beats resumed |
| 2026-10-16 | Powersuit owns the bench. **First Powersuit firmware on silicon:** CAN self-test on C, all seven checks pass; SAFETY overtakes a 12-frame queue (1st, not 13th). C's LED steady green: both-pin drive works. A and B still on Potluck, red for C's departure |
| 2026-09-21 | Ordered — DKK 408 landed, AliExpress ([A.1](bringup.md)) |
| 2026-10-01 | Arrived. Potluck verified silicon, security state, bridge chip and pins; backed up factory images; started a 24 h soak (~19:20) |
| 2026-10-01 | Both `node_bench` roles built in Docker and role-verified; host flash script dry-run checked. No board touched |
| 2026-10-16 | CAN transceivers removed from B and C (the wires tethered them, and Potluck's radio tests needed them apart); all three boards on the PC's USB, CP2102 on A. Powersuit's turn: B orchestrator, C limb |
| 2026-10-15 | Read the sibling project's two weeks of bench evidence. `ps_can` now transmits in class order (TWAI's FIFO), `node_bench` drives both LED pins and builds in CI, 500 kbit/s is the first fallback. No board touched |
| 2026-10-01 | Transceiver schematic found in the listing: Rs = 10 kΩ (slope control) and 120 Ω termination onboard. Sibling project's 250 kbit/s limit withdrawn; bench stays at 1 Mbps |
