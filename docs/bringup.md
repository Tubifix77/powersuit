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
