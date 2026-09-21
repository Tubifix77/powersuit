# Hardware bring-up

Everything in this repository has been compiled and its logic tested, but no
part of it has run on silicon. What follows is the order to prove it in, chosen
so that each step is verifiable before anything above it can hurt you.

The governing rule: **a motor is the last thing you energise, not the first.**
Every step below is safe to abandon halfway.

## 0. Before any board is powered

Confirm on the bench what the code assumes:

- Bus termination: 120 Ω at each physical end of each CAN segment, and only
  there. Measure ~60 Ω across CANH/CANL with everything unpowered.
- The pinouts in each app's `board_*.h` are **bench defaults**, not a verified
  schematic. Reconcile them with your actual wiring before the first flash.
- Motor supply on a current-limited bench PSU, set to the lowest limit that
  still spins the joint. Not a battery.

## 1. One node, no bus, no actuators

Flash `node_chest_hub` or one `node_limb` and watch the console.

```bash
idf.py -p <port> flash monitor
```

Success looks like the boot log reaching `waiting for OPERATIONAL` with no
panic, no watchdog reset and no `ps_can` errors beyond "no other node". A
crash here is almost certainly a peripheral init that compiled but does not
match the wiring — `ps_can_open` and `ps_focdrv_init` are the usual suspects.

The node will sit in STANDBY and never actuate: it has no heartbeat. That is
the correct behaviour, and it is the first real confirmation the safety state
machine works.

## 2. TWAI in loopback, still one node

Before wiring two boards together, prove the controller talks to itself. Set
`enable_loopback` and `enable_self_test` in the `twai_onchip_node_config_t`
inside `ps_can_open` (temporarily), and confirm transmitted frames come back
through the class dispatch. This separates "my CAN driver is wrong" from "my
wiring is wrong", which are otherwise indistinguishable at 3am.

## 3. Two nodes on one segment

Hub plus one limb. Watch that:

- the limb receives nothing until the hub forwards something,
- `ps_can_get_stats` shows `rx_frames` climbing and `bus_errors` flat.

A steadily climbing `bus_errors` with zero `rx_frames` is nearly always
termination or a swapped CANH/CANL pair, not software.

## 4. The heartbeat and the watchdog — the important one

With Node 8 not yet involved, inject heartbeats by hand (a USB-CAN adapter, or
a second board running a stub) at 100 Hz, then stop.

**Expected:** the limb goes to OPERATIONAL after its re-arm window and a fresh
command, and returns to PASSIVE within 50 ms of the beats stopping. Verify the
transition on a scope by toggling a spare GPIO in the transition callback if you
want the timing to be more than a log line.

Do not proceed until this behaves exactly as specified. Everything downstream
assumes the dead-man switch works.

## 5. Motor, current-limited, one joint

Only now. Command a small position hold in `PS_JMODE_POSITION` with the torque
limit in `board_limb.h` reduced to something that cannot hurt anyone.

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
LIMB (`idf.py menuconfig` → *Powersuit bench node*).

## A.1 Shopping list

**Ordered 2026-09-21 — DKK 408 landed** (DKK 296 goods, DKK 136 Danish import
charges, DKK 23.50 duty reduction), AliExpress Choice, expected mid-October 2026.
An EU-supplier comparison was run and rejected: jumper wire at four times the
price, and no EU listing carried a two-USB-C-socket N16R8 board at all.

The order is shared with a sibling ESP32-S3 project. Items marked **[both]**
serve that project too; **[sibling]** is not needed for anything in this repo.

| # | Item | Qty | Paid | Notes |
|---|------|-----|------|-------|
| 1 | ESP32-S3-DevKitC-1 **N16R8**, headers pre-soldered, two **USB-C** sockets | 3 | €7.73 ea | **[both]** Two is the minimum here — one orchestrator, one limb. The third is a spare and makes the sibling project's beacon comparison meaningful. N16R8 is what `node_bench` assumes (16 MB flash, 8 MB Octal PSRAM), and the sibling's memory figures are all attributed to it. |
| 2 | SN65HVD230 CAN transceiver module | 2 | €2.17 ea | **[both]** Two is deliberate — see the termination note. |
| 3 | Dupont jumper, **female-female, 20 cm**, 40-way | 1 | €2.31 | **[both]** 11 wires for the wiring in A.4. 20 cm is set by the board-to-board ground tie, the longest run on the bench. |
| 4 | CP2102 USB-TTL adapter, **3.3 V**, USB-A dongle | 1 | €2.46 | **[sibling]** Reaches that project's UART1 frame link. Nothing here needs it — the DevKitC-1's onboard bridge carries this console. |
| 5 | 6-port USB mains charger | 1 | €7.75 | Powers any board whose log you do not need to read — see A.4c. |

No resistors and no LEDs: the DevKitC-1 carries an addressable RGB LED, and
`node_bench` uses it to show safety state.

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

That leaves `1, 2, 4–18, 21, 33, 34, 39–42` comfortably free, with 39–42 doubling
as JTAG if you ever want hardware debugging.

**`node_bench` defaults to GPIO4 for TWAI TX and GPIO5 for TWAI RX.** Both are in
the unencumbered 4–18 band: not strapping, not bonded to flash or PSRAM on any
module variant, not USB, not the console UART. They are also adjacent and
low-numbered, which makes them easy to find on the silkscreen and hard to
misjumper. Change them in menuconfig if your wiring prefers otherwise.

## A.3 The RGB LED moved between board revisions

Espressif put the onboard WS2812 on **GPIO48 on DevKitC-1 v1.0** and **GPIO38 on
v1.1** — GPIO47/48 are fed from the 1.8 V VDD_SPI rail used by PSRAM, which is
why it moved. Both revisions are still sold and listings rarely say which you
are getting.

This matters more than it sounds: a wrong guess is a dark LED, which looks
exactly like a broken driver. So the pin is a menuconfig choice, never a
literal, and the firmware announces it at boot:

```
I bench: RGB LED   : GPIO38 (DevKitC-1 v1.1)
W bench: if the LED stays dark, you have the OTHER board revision — ...
```

If you would rather have the board tell you, enable **LED probe at boot**. It
drives GPIO38 and GPIO48 in turn for two seconds each and announces which is
active over serial; whichever lights your LED is your revision. Turn it off
again afterwards.

## A.4 Wiring

The module ordered (EGBO SN65HVD230 / VP230) carries a 4-pin male header
labelled **3.3V / GND / RX / TX** top to bottom, and brings CANH and CANL out
**twice, in parallel** — on a 2-pin header and on a blue screw terminal. Every
connector on it is male pins, which is why the whole bench wires with
female-female jumpers and no breadboard. There is no RS pin on the header;
slope control is handled on the board, so there is nothing to tie to ground.

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

And one ground tie — **between the two DevKits, not between the modules.** The
transceiver's header carries a single GND pin and it is already occupied by the
wire to its own board. Use any spare GND on each DevKit:

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
- **Common ground.** Two boards on the same PC share ground through it — but if
  one runs from the charger (A.4c) they do not, because a two-pin mains charger's
  output floats. CAN needs a common reference, so run the tie regardless. It is
  one wire.

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
`UART` from `USB`. Mark the UART side of each board at unboxing — you will plug
these in hundreds of times, and "wrong socket" looks exactly like "dead board".

## A.5 What you should see

```bash
idf.py -p COM3 flash monitor     # board A, configured as ORCHESTRATOR
idf.py -p COM4 flash monitor     # board B, configured as LIMB
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
