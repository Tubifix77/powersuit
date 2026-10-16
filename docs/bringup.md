# Hardware bring-up

Everything in this repository has been compiled and its logic tested. On silicon so
far: the two-board bench of Appendix A, where the dead-man switch, the actuation gate
and the real limb firmware's control loop all work ([bench-log.md](bench-log.md)).
What follows is the order to prove the rest in, chosen so that each step is verifiable
before anything above it can hurt you. The project is parked at the end of Appendix A;
[resuming.md](resuming.md) is the way back in.

The governing rule: **a motor is the last thing you energise, not the first.**
Every step below is safe to abandon halfway.

## 0. Before any board is powered

Confirm on the bench what the code assumes:

- Bus termination: 120 Ω at each physical end of each CAN segment, and only
  there. Measure ~60 Ω across CANH/CANL with everything unpowered.
- The pinouts in each app's `board_*.h` are **bench defaults**, not a verified
  schematic. Reconcile them with your actual wiring before the first flash.
  **`board_limb.h` is known not to fit an ESP32-S3 DevKit at all** — it uses GPIO that
  do not exist on the S3 and the PSRAM pins (note at its top, [resuming.md](resuming.md)).
  Resolve that before any limb leaves bench mode.
- Motor supply on a current-limited bench PSU, set to the lowest limit that
  still spins the joint. Not a battery.

## 1. One node, no bus, no actuators

Flash `node_chest_hub`, or one `node_limb` in bench mode (A.6), and watch the console.
Firmware builds in Docker and flashes from the host — Docker Desktop on Windows cannot
reach a COM port — so use `firmware/tools/flash_bench.ps1` and `read_console.py` as in
A.4d, or the manual esptool line there.

Success looks like the boot log reaching `waiting for OPERATIONAL` with no
panic, no watchdog reset and no `ps_can` errors beyond "no other node". A
crash here is almost certainly a peripheral init that compiled but does not
match the wiring — `ps_can_open` and `ps_focdrv_init` are the usual suspects.

The node will sit in STANDBY and never actuate: it has no heartbeat. That is
the correct behaviour, and it is the first real confirmation the safety state
machine works.

## 2. TWAI in loopback, still one node

Before wiring two boards together, prove the controller talks to itself. This
separates "my CAN driver is wrong" from "my wiring is wrong", which are otherwise
indistinguishable at 3am. `node_bench` has a self-test mode for exactly this: the
controller runs in self-test + loopback with RX on the TX pin, so it hears its own
frames and needs no ACK — **no transceiver, no second board, no wires.**

```bash
BENCH_SELFTEST=1 bash firmware/tools/build_bench.sh
```

```powershell
powershell -File firmware/tools/flash_bench.ps1 -Role selftest -Port COM5
& $py firmware/tools/read_console.py COM5 --seconds 30 --until SELFTEST_DONE
```

It checks that 200 frames go out and come back with no errors; that a SAFETY frame
queued behind twelve TELEM frames **overtakes** them (`ps_can_txq`, network-map §2 —
in arrival order it would come back last); that frames within a class keep their
order; and SAFETY's send-to-receive latency, idle and behind a queue. It ends with
`SELFTEST_DONE OK 0` and a steady green LED, or `SELFTEST_DONE FAIL <n>` and a red
strobe. `read_console.py` exits 0, 1 or 2 for pass, fail or timeout.

Loopback with RX left on its own pin reads noise and goes bus-off: RX must share the
TX pin, which the self-test does. Disconnect a transceiver if one is attached.

## 3. Two nodes on one segment

Hub plus one limb. Watch that:

- the limb receives nothing until the hub forwards something,
- `ps_can_get_stats` shows `rx_frames` climbing and `bus_errors` flat.

A steadily climbing `bus_errors` with zero `rx_frames` is nearly always
termination or a swapped CANH/CANL pair, not software.

The two-board bench covered two limb-side nodes on one segment, with `node_bench`
standing in for the heartbeat source ([bench-log.md](bench-log.md) §2–4). The hub itself
has never run.

## 4. The heartbeat and the watchdog — the important one

With Node 8 not yet involved, inject heartbeats by hand (a USB-CAN adapter, or
a second board running a stub) at 100 Hz, then stop.

**Expected:** the limb goes to OPERATIONAL after its re-arm window and a fresh
command, and returns to PASSIVE within 50 ms of the beats stopping. Verify the
transition on a scope by toggling a spare GPIO in the transition callback if you
want the timing to be more than a log line.

Do not proceed until this behaves exactly as specified. Everything downstream
assumes the dead-man switch works.

Done on the two-board bench (Appendix A.5, [bench-log.md](bench-log.md) §2): trip ≈ 53–55
ms after the last beat, re-arm ≈ 258–259 ms after beats resume — estimated from a frame
counter. The GPIO-toggle-and-scope measurement is still to do.

## 5. Motor, current-limited, one joint

Only now — and not before `board_limb.h`'s pin map is replaced, since joint 0's gate
pins do not exist on an ESP32-S3 (step 0). Which actuator the suit uses decides what
replaces it ([resuming.md](resuming.md)). Command a small position hold in
`PS_JMODE_POSITION` with the torque limit in `board_limb.h` reduced to something that
cannot hurt anyone.

Scope the three phase outputs **before** connecting the motor: confirm
centre-aligned complementary PWM with visible dead time and no overlap between
a high side and its own low side. A shoot-through here is a dead FET and
possibly a dead board.

Then check that cutting the heartbeat mid-hold produces free-wheeling (or
damping above the threshold in `board_limb.h`), not a lurch.

## 6. SPI, hub to Pi

Start at a low clock — 1 MHz, not 20 — and raise it once frames are clean.

`suit_canspi_bridge` counts `crc_errors` and `seq_gaps`; both should be zero at
rest. Non-zero CRC errors that scale with clock speed are signal integrity, and
the fix is wiring or clock, never software. Confirm the DATA_READY line
actually toggles before trusting the interrupt path; the polling fallback works
and is the right thing to use while debugging.

## 7. micro-ROS

```bash
bash firmware/tools/fetch_deps.sh   # vendors the client, enables the XRCE plane
```

Rebuild and flash, then start Node 8. The bridge creates one PTY per node under
`/tmp/powersuit/xrce/` and the agent attaches to all six. `ros2 node list`
should show the edge nodes appearing.

This is the least-proven path in the system: XRCE over 8-byte CAN frames is
custom at both ends. Expect to spend time here, and debug it with only one edge
node enabled.

## 8. BMS — hardware first, always

The sub-microsecond short-circuit trip is the analog comparator and its latch.
**Test it with the MCU unpowered**, by shorting through a suitably rated shunt
and confirming the gate opens. Only once that is proven should you power the
P4 and check that the firmware observer logs the trip, broadcasts ESTOP, and
refuses to re-arm while the comparator still reads faulted.

If you find yourself tempted to make firmware do the tripping because the
hardware is not ready yet: don't. `docs/safety.md` §4 explains why that
guarantee cannot be met in software, and a suit that half-implements it is more
dangerous than one that admits it has no protection.

## What to expect to find

The classes of defect that survive to this point are timing and configuration:
peripheral registers that compile but describe the wrong waveform, task
priorities that starve something under real load, stack sizes tuned by guesswork,
and the 1 kHz control loop overrunning once the comms core is genuinely busy.

Watch `NODE_STATS` (`cpu_pct`, `err_cnt`) and the ESP-IDF task watchdog. Raise
`CONFIG_ESP_TASK_WDT_TIMEOUT_S` only after you understand why something is late,
never to make a symptom go away.

---

# Appendix A — the two-board bench, without a soldering iron

The steps above assume a real suit. This appendix is the cheap version: two
DevKitC-1 boards, two CAN transceiver modules, jumper wires, and no soldering.
It proves the one behaviour that matters most — the dead-man switch — on real
silicon and real wire.

Flash `firmware/apps/node_bench` to both boards, one as ORCHESTRATOR and one as
LIMB: `firmware/tools/build_bench.sh` builds both roles (A.4b) and
`firmware/tools/flash_bench.ps1` flashes them from Windows (A.4d). The drawing to wire
from is [`bench-wiring.html`](bench-wiring.html). Everything in this appendix has been
done; [bench-log.md](bench-log.md) has the results.

## A.1 Shopping list

**Ordered 2026-09-21 — DKK 408 landed** (DKK 296 goods, DKK 136 Danish import
charges, DKK 23.50 duty reduction), AliExpress Choice; arrived 2026-10-01. The bench's current state — board
register, what is verified, what is not — is kept in [`bench-log.md`](bench-log.md).
An EU-supplier comparison was run and rejected: jumper wire at four times the
price, and no EU listing carried a two-USB-C-socket N16R8 board at all.

The order is shared with a sibling ESP32-S3 project. Items marked **[both]**
serve that project too; **[sibling]** is not needed for anything in this repo.

| # | Item | Qty | Paid | Notes |
|---|------|-----|------|-------|
| 1 | ESP32-S3-DevKitC-1 **N16R8**, headers pre-soldered, two **USB-C** sockets | 3 | €7.73 ea | **[both]** Two is the minimum here — one orchestrator, one limb. The third is a spare and makes the sibling project's beacon comparison meaningful. N16R8 is what `node_bench` assumes (16 MB flash, 8 MB Octal PSRAM), and the sibling's memory figures are all attributed to it. |
| 2 | SN65HVD230 CAN transceiver module | 2 | €2.17 ea | **[both]** Two is deliberate — see the termination note. |
| 3 | Dupont jumper, **female-female, 20 cm**, 40-way | 1 | €2.31 | **[both]** 11 wires for the wiring in A.4. 20 cm is set by the board-to-board ground tie, the longest run on the bench. |
| 4 | CP2102 USB-TTL adapter, **3.3 V**, USB-A dongle | 1 | €2.46 | **[sibling]** Reaches that project's UART1 frame link. Nothing here needs it — the DevKitC-1's onboard bridge carries this console. It is the only part in the order that needs a driver (Silicon Labs CP210x). |
| 5 | 6-port USB mains charger | 1 | €7.75 | Powers any board whose log you do not need to read — see A.4c. |

No resistors and no LEDs: the DevKitC-1 carries an addressable RGB LED, and
`node_bench` uses it to show safety state.

**What actually arrived** (verified 2026-10-01 by the sibling project on these
exact units):

- **Real N16R8, despite the clone listing.** No Espressif seal and recycled
  marketing copy, but esptool reports ESP32-S3 QFN56 rev v0.2, dual core at
  240 MHz, 16 MB flash, 8 MB PSRAM. Secure Boot and Flash Encryption are off and
  every key block is empty, so the boards can be reflashed indefinitely.
- **The onboard USB-UART bridge is a WCH CH343** (VID 1A86, PID 55D3), not a
  CP2102. Windows 11 already carried its driver; nothing needed installing. Each
  CH343 reports a unique serial number, so three identical boards are told apart
  in Device Manager without the unplug-and-see-what-vanishes trick.
- **The sockets are silkscreened `COM` and `USB`**, not `UART`/`USB`. `COM` is the
  CH343, wired to UART0 on GPIO43/44; `USB` is the chip's native USB.
- **DTR/RTS auto-reset is wired.** Flashing needs no buttons held and can be
  fully scripted.
- The A.2 pin table matches the real silkscreen. GPIO35-37 *are* broken out on
  the header as if they were ordinary GPIOs; on an R8 part they are the PSRAM.

**Deliberately not bought, and why:**

- **USB cables.** The one item buyable locally the same day, so not worth
  importing — and the lengths are impossible to plan before the parts are in
  hand. You need **USB-A to USB-C, data-capable**, one per board. A charge-only
  cable powers the board and it never enumerates, which presents as a dead board
  or a driver fault and costs an hour.
- **Powered USB hub.** Three direct PC ports beat a hub: each board gets its own
  500 mA budget with nothing shared. A hub only earns its place if you are short
  of ports, or want one cable rather than three crossing a room. A *bus-powered*
  hub is the wrong answer regardless — 500–900 mA shared against roughly
  1050 mA of peak demand from three boards, and it sags occasionally rather than
  failing cleanly, which fakes exactly the faults a soak exists to measure.
- **Breadboard.** Everything has male pins, so F-F jumpers join them directly.
- **120 Ω resistors.** Each module carries its own.
- **Multimeter.** Genuinely useful and only about DKK 50, but not required for
  this bench: at two nodes over 20 cm, propagation is ~1.5 ns against a 1 µs bit
  time, so termination barely matters. `node_bench`'s `rx_frames` and
  `bus_errors` counters are the better instrument for the failure you will
  actually hit, which is a swapped TX/RX pair.

**Why two transceivers and not three.** Most SN65HVD230 breakouts carry their
own 120 Ω termination. Two of them give the correct 60 Ω across the bus; three
give 40 Ω, which is out of spec. Only the two physical ends of a CAN segment
should be terminated, so a three-node bus needs the middle module's resistor
lifted — not an option without a soldering iron unless the module puts it on a
jumper. Two nodes on CAN is everything this appendix needs; a third board can
still participate over the radio.

**F-F is the jumper you will actually use.** With headers pre-soldered, both the
DevKit and the transceiver present male pins, so connecting them directly needs
female-female. That is what makes the breadboard optional rather than required.

## A.2 Which pins are safe, and why

The ESP32-S3 has fewer usable GPIOs than the pin count suggests. On an **N16R8**
module these are all unavailable or inadvisable:

| Pins | Why |
|------|-----|
| 0, 3, 45, 46 | Strapping pins — sampled at reset; driving them changes boot behaviour |
| 19, 20 | USB D− / D+ |
| 43, 44 | UART0 TX/RX — the serial console you will be reading |
| 26–32 | SPI flash and PSRAM |
| **35, 36, 37** | **Octal PSRAM only** — free on non-R8 parts, forbidden on R8 |
| 38 *or* 47/48 | Onboard RGB LED, depending on board revision (see A.3) |

That leaves `1, 2, 4–18, 21, 39–42` comfortably free on the DevKitC-1's headers, with
39–42 doubling as JTAG if you ever want hardware debugging. (GPIO 33 and 34 exist on the
chip but are not broken out on this board; 35–37 per Espressif's DevKitC-1 user guide
"are used for the internal communication between ESP32-S3 and SPI flash/PSRAM memory,
thus not available for external use" on octal-PSRAM modules.)

**`node_bench` defaults to GPIO4 for TWAI TX and GPIO5 for TWAI RX.** Both are in
the unencumbered 4–18 band: not strapping, not bonded to flash or PSRAM on any
module variant, not USB, not the console UART. They are also adjacent and
low-numbered, which makes them easy to find on the silkscreen and hard to
misjumper. Change them in menuconfig if your wiring prefers otherwise.

## A.3 The RGB LED moved between board revisions

Espressif put the onboard WS2812 on **GPIO48 on DevKitC-1 v1.0** and **GPIO38 on
v1.1** (their user guide for each revision). The guides document the move, not the
reason — an earlier version of this document said GPIO47/48 run at 1.8 V, which per
Espressif's ordering table holds only for the 1.8 V WROOM-2 boards, not these 3.3 V
N16R8 ones. Both revisions are still sold and listings rarely say which you are getting.

This matters more than it sounds: a wrong guess is a dark LED, which looks
exactly like a broken driver. So **by default `node_bench` drives both pins** with
the same colour: whichever carries the LED lights it, and the other is an ordinary
header pin with nothing on it. The sibling project's firmware does exactly this on
these boards, and the LED works (see [bench-log.md](bench-log.md)). The boot banner
says so:

```
I bench: RGB LED   : GPIO38 and GPIO48 (DevKitC-1 v1.0 or v1.1: whichever has the LED)
```

Which revision these units are is still not established, and it does not need to be.
If you want to know, or something else is wired to GPIO38 or GPIO48, pick a single pin
in menuconfig (*Powersuit bench node → DevKitC-1 revision*), or enable **LED probe at
boot**: it drives GPIO38 and GPIO48 in turn for two seconds each and announces which
is active over serial.

**A lit LED is not proof the app is driving it.** A WS2812 latches its last colour
and holds it indefinitely, so a board flashed with firmware that never touches
the LED keeps whatever the factory demo was showing when flashing interrupted it
— one of these units sat blue and another green for days. A colour that never
changes means nothing is writing to the LED, not that anything is broken.

## A.4 Wiring

The module ordered (EGBO SN65HVD230 / VP230) carries a 4-pin male header
labelled **3.3V / GND / RX / TX** (read from the other end, the same header runs
TX / RX / GND / 3.3V — **identify every pin by its printed label, never by counting
positions**, on the module and on the DevKit alike), and brings CANH and CANL out
**twice, in parallel** — on a 2-pin header and on a blue screw terminal. Every
connector on it is male pins, which is why the whole bench wires with
female-female jumpers and no breadboard. There is no RS pin on the header;
slope control is handled on the board, so there is nothing to tie to ground.

**The drawing to wire from is [`bench-wiring.html`](bench-wiring.html)**, open it in a
browser: both boards and modules as they sit on the bench, every wire colour-coded.

**Wire with both boards unplugged**, then plug them back in. Using the usual colours
for the real jumpers (red power, black ground, yellow CANH, green CANL) makes the
finished bench easy to check against the drawings below.

Per board, four jumpers to its transceiver:

```
   ESP32-S3-DevKitC-1              SN65HVD230 module
   ------------------              -----------------
   3V3  ----------------------->   3.3V    (3.3 V — NOT 5 V; the S3 is 3.3 V)
   GND  ----------------------->   GND
   GPIO4 (TWAI TX) ----------->    TX
   GPIO5 (TWAI RX) <-----------    RX
```

Then the bus itself, between the two transceiver modules:

```
   transceiver A                   transceiver B
   -------------                   -------------
   CANH ------------------------>  CANH
   CANL ------------------------>  CANL
```

And, **if either board runs from the charger**, one ground tie — **between the two
DevKits, not between the modules.** The transceiver's header carries a single GND pin
and it is already occupied by the wire to its own board. Use any spare GND on each
DevKit:

```
   DevKit A  GND (spare) ------->  DevKit B  GND (spare)
```

The screw terminals then stay free, which is useful: you can clamp meter probes
onto CANH/CANL while the bus is live without unplugging anything, and a screw
terminal takes bare wire, so the bus itself is never limited to jumper length.

Three things that account for most first-time failures:

- **TX and RX are not symmetric.** The board's TX goes to the module's TX input;
  the module's RX output goes to the board's RX. They are not swapped across the
  pair — swapping them is the most common wiring mistake and shows up as
  `bus_errors` climbing with `rx_frames` stuck at zero.
- **CANH goes to CANH.** Unlike a serial crossover, the differential pair is
  straight-through.
- **Common ground.** Two boards on the same PC share ground through it, and the tie
  is optional: the dead-man test ran that way on 2026-10-16, ten wires, zero errors.
  If one runs from the charger (A.4c) they do not, because a two-pin mains charger's
  output floats: fit the tie. With it, that configuration ran clean on 2026-10-16;
  without it, it has never been tried.

## A.4b Building the two roles (read this before scripting it)

One image, two roles. The limb is the default; the orchestrator is an overlay:

```bash
idf.py -B build_limb -D SDKCONFIG=build_limb/sdkconfig build
```

```bash
rm -rf build_orch
idf.py -B build_orch -D SDKCONFIG=build_orch/sdkconfig        -D SDKCONFIG_DEFAULTS="../../sdkconfig.defaults.common;sdkconfig.defaults;sdkconfig.defaults.orch"        build
```

**ESP-IDF treats an existing `sdkconfig` as authoritative** and consults
`sdkconfig.defaults` only for symbols the sdkconfig does not already contain.
So on the second and every later build, a changed overlay is ignored — not
partially, entirely. Reuse a build directory and you get the previous role's
image while the build reports success.

Two habits make this a non-issue, and both are cheap:

- point `-D SDKCONFIG` at a path that does not exist yet (or delete it first),
- **read the generated sdkconfig back** and confirm the override actually landed:

```bash
grep PS_BENCH_IS_ORCH build_orch/sdkconfig     # expect: CONFIG_PS_BENCH_IS_ORCH=y
```

Two boards flashed with silently identical firmware is a genuinely confusing
way to start a bench session: neither board beats, both sit amber, and it looks
exactly like a wiring fault.

**`firmware/tools/build_bench.sh` does all of this for you:** it deletes both
build directories, builds both roles in Docker from fresh sdkconfigs, and fails
unless each sdkconfig holds the role it should *and* the two app images differ.
The full log goes to `.verify-logs/bench_build.log`; only the verdict is printed.
It never starts Docker — if Docker is not running it says so and stops.

(Credit: diagnosed on a sibling ESP32-S3 project after an int override was seen
reverting between runs.)

## A.4c Powering a board whose console you do not need

Once both boards are flashed, the orchestrator needs nothing but 5 V — you are
going to cut its power on purpose, and that is the whole of step A.5.3. So it can
run from a USB mains charger, leaving a single PC port for the limb, whose log you
do need to read. Pulling the charger lead is also a cleaner trigger than yanking a
cable out of the PC.

Two limits on that:

- **A charger carries no data.** Any board whose log you need must be on a PC
  port. The sibling project's 24 h soak needs all three boards on real ports for
  exactly this reason — there, every node's stream is the measurement.
- **Never power one board from the charger and the PC at once** through its two
  USB-C sockets. Both feed the same 5 V rail, and tying two supplies together is
  not a fault you will enjoy diagnosing.

Both sockets are USB-C and visually identical; only the silkscreen distinguishes
`COM` from `USB`. Mark them at unboxing — you will plug these in hundreds of
times, and "wrong socket" looks exactly like "dead board".

In this repo both sockets work, deliberately. The S3's USB PHY interferes with
Wi-Fi, and ESP-IDF's `CONFIG_ESP_PHY_ENABLE_USB` trades one for the other. The
sibling project turns it off because its purpose is clean RF measurement, which
kills its native USB port. Powersuit leaves it at the default: `COM` carries the
primary console on UART0, and `USB` carries a secondary USB-Serial-JTAG console
plus on-chip JTAG — breakpoints and single-stepping with no probe. `node_bench`
uses no radio at all, so there is nothing to give up. Keep it that way unless a
node genuinely needs better Wi-Fi.

## A.4d Flashing from Windows when the build ran in Docker

`idf.py flash` needs the serial port, and Docker Desktop on Windows cannot pass a
COM port into a container. So build in Docker and flash from the host. The build
directories sit beside the app sources on the bind mount, so the host can see
them — a named-volume `-B` would hide them from Windows.

With the board on its **`COM`** socket:

```powershell
powershell -File firmware/tools/flash_bench.ps1 -Role limb -Port COM4 -Monitor
powershell -File firmware/tools/flash_bench.ps1 -Role orch -Port COM3 -DryRun
```

The script refuses to flash unless `build_<role>/sdkconfig` holds the role you
named, finds the newest ESP-IDF Python environment under
`%USERPROFILE%\.espressif\python_env` (override with `PS_IDF_PYTHON`), and
translates the build's `flasher_args.json` into esptool 5's syntax — ESP-IDF 5.5
writes underscore options such as `--flash_mode`, which esptool 5 spells with
hyphens. `-Monitor` follows with esp-idf-monitor, which uses the ELF to decode
panic backtraces; exit with `Ctrl+]`. `-DryRun` prints the commands and opens no
port. Device Manager shows each CH343's serial number, which identifies the board
behind a COM number.

The manual equivalent, from inside the build directory:

```powershell
& $py -m esptool --chip esp32s3 -p COM4 -b 460800 write-flash '@flash_args'
& $py -m esp_idf_monitor -p COM4 node_bench.elf
```

Quote `'@flash_args'` — a bare `@` is PowerShell's splatting operator. esptool 5
accepts `@file` argument files (checked) and warns on the file's underscore options.

Used for every flash on 2026-10-16 — bootloader, partition table and app, each hash
verified, about 4 s for an app. To capture a board's console from boot without an
interactive terminal, `firmware/tools/read_console.py COMx --seconds N [--until TEXT]`
resets it into the application (esptool's sequence) and prints what it says; run it with
the ESP-IDF environment's Python, which has pyserial.

## A.5 What you should see

Flash each board from its own build directory as in A.4d — `build_orch/` to
the orchestrator, `build_limb/` to the limb — then read
`grep PS_BENCH_IS_ORCH build_*/sdkconfig` once more before powering both. Two
boards with the same role look exactly like a wiring fault.

**Opening a board's serial console resets it.** The DTR/RTS auto-reset wiring that
makes flashing button-free also reboots the board whenever a terminal opens the port
with ordinary settings. That is why `-Monitor` shows the boot log — and why you must
**not** open the orchestrator's console in the middle of step 3: it reboots the
orchestrator, the beats stop, and the limb trips for the wrong reason, which looks
exactly like the dead-man switch working. To watch a board that is already running:

```powershell
& $py -m esp_idf_monitor --no-reset -p COM3 node_bench.elf
```

1. **Limb alone, orchestrator off.** LED amber (STANDBY). It will never arm — no
   heartbeat, no authority. That is correct, and it is the first confirmation the
   safety state machine works.
2. **Orchestrator powered.** Within a few hundred milliseconds the limb's LED
   turns white-blue and pulses: OPERATIONAL. The serial log prints the transition.
3. **Pull the orchestrator's USB lead.** The limb's LED must go amber within
   50 ms, and the log prints `>>> 2 -> 3 (cause 6)` — OPERATIONAL to PASSIVE,
   cause COMM_LOSS. This is the dead-man switch. Do not proceed past this
   appendix to anything with a motor in it until you have seen this work.
4. **Plug it back in.** The limb returns to OPERATIONAL, but only after 250 ms of
   uninterrupted heartbeats *and* a fresh command — recovery is deliberately
   harder than staying up.

If step 1 shows a dark LED rather than amber, read A.3 before suspecting the
driver. If step 2 never happens, check `rx_frames` in the limb's 2-second status
line: zero means wiring, non-zero means something else.

**If `bus_errors` climb with wiring you have checked twice**, step the rate down,
rebuild both roles and flash both boards again:

```bash
BENCH_CAN_BITRATE=500000 bash firmware/tools/build_bench.sh    # measured clean on these modules
BENCH_CAN_BITRATE=250000 bash firmware/tools/build_bench.sh    # last resort
```

1 Mbit/s ran clean on this bench on 2026-10-16 (zero errors over two minutes of the
dead-man test, [bench-log.md](bench-log.md)), so a fallback should only be needed
after a wiring change. 500 kbit/s first, because the sibling project ran these exact
transceivers and boards there with zero bit, form or stuff errors under a 3,000
frames/s flood; 250 kbit/s has not run here at all. The dead-man test needs about 100
frames a second, a few percent of the bus at any of them. Each board prints its rate in the boot banner
(`TWAI : ... @ 500000 bit/s`), and **both must match** — a rate mismatch between
the two boards looks exactly like a wiring fault. Go back to 1 Mbps afterwards;
it is the contract rate, and the bus budget in `network-map.md` §10 assumes it.

## A.6 The real limb firmware, on the same bench

`node_bench` is a stand-in. Once A.5 passes, the limb board can run the actual
`node_limb` firmware — the code an arm runs — in **bench mode** (`CONFIG_PS_LIMB_BENCH`):
the motor driver, encoder SPI and current sense are not initialised, the joints read
as at rest, and `joint_write()` counts the ticks on which the safety gate *would* have
energised a joint instead of driving gates. Everything else is real: the 1 kHz control
loop on core 1, its PIDs and FOC maths, `ps_safety`, the 100 Hz TELEM streams on core 0
(six with an IMU fitted, three without — an absent IMU sends nothing), and the
micro-ROS client if it is vendored. The orchestrator board keeps
running `node_bench` as the heartbeat source; nothing is rewired.

```bash
BENCH_LIMB=1 bash firmware/tools/build_bench.sh
```

```powershell
powershell -File firmware/tools/flash_bench.ps1 -Role limbapp -Port COM5
& $py firmware/tools/read_console.py COM5 --seconds 30
```

Every 10 s it logs a status line and a timing line:

```
node_limb: state=2 ticks=... cmd[ok=.. rej=.. mode=..] can[rx=.. tx=.. drop=.. err=..]
node_limb: timing: loop max ... us, missed ticks ..., ctrl stack free ... B, would-actuate ...
```

`loop max` is the worst single control iteration against a 1,000 µs period; `missed
ticks` counts periods that elapsed while an iteration was still running. The bench
orchestrator sends node 1 a 10 Hz position hold, so `would-actuate` climbs about 2,000
per second while OPERATIONAL. **Pull the orchestrator and it must freeze** — the trip
forces every joint PASSIVE and the gate refuses every tick — and resume after re-arm.
That is the one actuation gate (CLAUDE.md) shown working. Expect `err` to climb while
the orchestrator is gone: the limb's telemetry has no node to acknowledge it.

**Why bench mode exists at all:** `board_limb.h`'s pin map cannot run on this board —
GPIO 22–25 do not exist on the ESP32-S3, 35–37 are the PSRAM on N16R8, and the pin
budget does not close (note at the top of `board_limb.h`). `flash_bench.ps1` refuses a
`node_limb` image without bench mode for exactly that reason.
