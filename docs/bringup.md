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

| Item | Qty | Note |
|------|-----|------|
| ESP32-S3-DevKitC-1 | 2 | Buy **with headers pre-soldered**. N16R8 (16 MB flash / 8 MB Octal PSRAM) is what this config assumes. |
| SN65HVD230 CAN transceiver breakout | 2 | Most of these carry their own 120 Ω termination, so a two-node bus made from two of them is already terminated at both ends. Check your listing — modules vary, and a few ship the resistor unpopulated or on a jumper. |
| Breadboard + male-female dupont jumpers | 1 set | Push-fit only. |
| USB cables | 2 | One per board, so you can watch both serial logs at once. |

No separate resistors, no LEDs: the DevKitC-1 has an addressable RGB LED
onboard, and `node_bench` uses it to show safety state.

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

Per board, four jumpers to its transceiver:

```
   ESP32-S3-DevKitC-1              SN65HVD230 module
   ------------------              -----------------
   3V3  ----------------------->   VCC     (3.3 V — NOT 5 V; the S3 is 3.3 V)
   GND  ----------------------->   GND
   GPIO4 (TWAI TX) ----------->    CTX  / TXD
   GPIO5 (TWAI RX) <-----------    CRX  / RXD
```

Then the bus itself, between the two transceiver modules:

```
   transceiver A                   transceiver B
   -------------                   -------------
   CANH ------------------------>  CANH
   CANL ------------------------>  CANL
   GND  ------------------------>  GND     (tie the grounds together)
```

Three things that account for most first-time failures:

- **TX and RX are not symmetric.** The board's TX goes to the module's TX input;
  the module's RX output goes to the board's RX. They are not swapped across the
  pair — swapping them is the most common wiring mistake and shows up as
  `bus_errors` climbing with `rx_frames` stuck at zero.
- **CANH goes to CANH.** Unlike a serial crossover, the differential pair is
  straight-through.
- **Common ground.** Two boards on separate USB ports usually share ground
  through the PC, but tie the transceiver grounds anyway.

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
