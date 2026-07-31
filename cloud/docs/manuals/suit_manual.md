# Powersuit Operator Manual

This manual is written for the person wearing or ground-crewing the suit. It is also the
live corpus the cloud AI core (Node 9) retrieves from when it answers a voice query — keep
it accurate and keep section headings intact, since the retrieval chunker keys off them.

## 1. The nine nodes

The suit is nine cooperating computers. If something is behaving oddly, knowing which node
owns it tells you where to look.

| Node | Name                        | What it does                                                        |
|-----:|------------------------------|----------------------------------------------------------------------|
| 1    | `node_arm_right`             | Right arm elbow/wrist actuation, joint telemetry, force sensing.     |
| 2    | `node_arm_left`               | Left arm elbow/wrist actuation, joint telemetry, force sensing.      |
| 3    | `node_leg_right`              | Right leg hip/knee actuation, joint telemetry, force sensing.        |
| 4    | `node_leg_left`               | Left leg hip/knee actuation, joint telemetry, force sensing.         |
| 5    | `node_flight_actuation`       | Aero flap surfaces (airbrakes), torso IMU, aero telemetry.           |
| 6    | `node_helmet_interface`       | HUD display, mic/speaker audio, local wake-word.                     |
| 7    | `node_chest_power_hub`        | Battery (BMS), CAN bus 1/bus 2 gateway, SPI bridge to Node 8.        |
| 8    | `node_central_orchestrator`   | Sensor fusion, kinematics, local offline voice, cloud link gateway.  |
| 9    | `cloud_ai_core` (this system)  | Long-form reasoning, RAG over this manual, advisories, voice replies.|

Nodes 1–7 are the suit itself (microcontrollers on two CAN buses). Node 8 is the on-body
computer that bridges the suit to Node 9 over a wireless link (5G, Wi-Fi, or satellite,
whichever is healthiest). Node 9 — where this text is running — never touches actuation: it
can only speak to you (advisories, voice) and never issues a movement, flap, or mode command.
That boundary is enforced by Node 8's gateway, not by convention.

## 2. Operating states and Passive Compliance Mode

The suit is always in exactly one of: BOOT, STANDBY, OPERATIONAL, PASSIVE, ESTOP, or FAULT.
OPERATIONAL is normal powered assist. The one you need to recognize in the field is PASSIVE
— "Passive Compliance Mode."

**What triggers it:** Node 8 broadcasts a heartbeat 100 times a second. Any limb or flight
node that goes 50 milliseconds without a valid heartbeat drops itself into PASSIVE
immediately, with no confirmation needed from anyone. This is a dead-man switch, not a
suggestion: it also fires on a lost wireless link between Node 7 and Node 8, on a CAN bus
going error-passive, or simply on Node 8 crashing or rebooting.

**What it feels like:** arm and leg joints go slack (zero-torque, free-wheeling); if you're
moving fast the joint will actively damp itself rather than stay limp, so it won't windmill.
Flaps drive to neutral at a controlled rate — you will not feel a sudden snap. The suit stays
back-drivable throughout: you can always move your own limbs by hand. The HUD shows a
SAFE-state banner and audio keeps working; telemetry keeps flowing the whole time.

A related, softer condition: if the suit is OPERATIONAL and heartbeats are fine but no fresh
joint command has arrived in 200 ms (planner stalled, not the link), the affected joint ramps
smoothly to zero-effort and holds — this recovers by itself the instant commands resume, no
action from you required.

**Getting back to OPERATIONAL from PASSIVE:** once heartbeats are flowing again, the suit
needs 250 ms of *continuous* heartbeat before it will even consider re-arming, and then it
still requires a fresh, explicit command (a joint command, flap command, or mode-set) after
that window — it will not resume motion just because the link came back. This is deliberate:
"the link is alive" and "someone actually wants me to move" are different facts.

## 3. E-stop: engaging and clearing

Any node can trigger an emergency stop the instant it sees a critical fault, and it announces
it three times in a row on the CAN bus so a single dropped frame can't hide it. Every other
node latches the stop the moment it hears the first copy — including, redundantly, Node 8
stopping its own heartbeat, which drives every node into PASSIVE within 50 ms even if a node
somehow missed all three ESTOP frames.

**Engaging manually:** say "emergency stop" or "e-stop" (or "kill power") to the helmet mic,
or use the physical/ground-crew e-stop control if your rig has one. Once latched, the suit
will not move — no exceptions — until it is cleared.

**Clearing it — read this carefully:** e-stop is not cleared by simply saying the fault went
away. The clear message carries a fixed magic value plus a **monotonic counter**, and every
node only accepts a clear whose counter is strictly greater than the last one it accepted.
This is what stops a stale, corrupted, or replayed clear message from silently un-latching a
stop behind your back — the counter only ever goes up, so an old or garbled message can never
count as "newer."

Practical clear procedure:
1. Diagnose and resolve the actual cause first — an e-stop clear does not fix the fault, it
   only lifts the latch.
2. Issue CLEAR_ESTOP (voice: "clear estop" / "reset estop" / "clear emergency stop"; from the
   orchestrator: the `/suit/estop` service). The system stamps it with the next counter value
   for you — you never need to pick a number yourself.
3. The suit drops to STANDBY, not straight back to OPERATIONAL.
4. Hold still for **one full second of continuous heartbeat** — this is a longer, more
   deliberate re-arm window than the 250 ms used for a plain PASSIVE recovery, because an
   e-stop was a *critical* fault.
5. Send an explicit mode-set to OPERATIONAL. Only then will the suit actuate again.

If the cause was a battery short-circuit trip, note that firmware cannot re-close the power
path by itself even after a clear — see §4, the interlock requires the hardware comparator to
agree the fault is actually gone.

## 4. Battery care

The pack is a 12S3P lithium-ion pack monitored by Node 7 at 1 kHz. Cell-group and
temperature/current readings roll up into `BMS_SUMMARY`/`BMS_CELLS` telemetry and into the
fault bits reported over the link; the table below is the actual trip/warning pairs the
hardware is configured with.

| Condition            | Warn threshold        | Trip threshold          | Notes                                   |
|-----------------------|------------------------|---------------------------|------------------------------------------|
| Cell overvoltage      | 4.15 V/cell            | 4.25 V/cell               | Stop charging at warn; trip latches ESTOP. |
| Cell undervoltage     | 2.95 V/cell            | 2.80 V/cell               | Land/dock and recharge at warn.          |
| Pack overtemperature  | 55.0 °C                | 60.0 °C                   | Same 60 °C ceiling the cloud advisory uses. |
| Pack undertemperature | −5.0 °C                | −10.0 °C                  | Cold-soaked pack; warm before hard use.  |
| Discharge overcurrent | 100 A                  | 120 A                     | Sustained heavy assist/flight load.      |
| Charge overcurrent    | 18 A                   | 20 A                      | Applies only while on charge.            |

All thresholds debounce for 50 ms of continuous out-of-range reading before they act — a
single noisy sample will not trip anything.

State-of-charge guidance from the cloud advisory rules (these are pacing advisories, not BMS
trips):
- **Below 20% SOC** — "Battery low" notice: plan to land or dock and recharge soon.
- **Below 10% SOC** — "Battery critically low" warning: reduce load immediately; flight
  operations are not advised at this level.

General care:
- Don't leave the pack sitting at full charge for extended storage; a partial charge (roughly
  40–60%) ages the cells more slowly than storing at 100%.
- Let a hot pack cool before recharging — charging into an already-warm pack compounds heat.
- A cell overvoltage/overtemperature trip escalates to ESTOP; recovery follows the e-stop
  clear procedure in §3, and — for a BMS short-circuit specifically — the discharge path is
  held open by a hardware latch that firmware cannot bypass. It only re-closes when a
  ground-crew re-arm strobe and the analog comparator both agree the fault is gone.
- `NODE_FAULT`/BMS fault-bit advisories name the specific bit that's set (short latch, OV, UV,
  OT, UT, charge/discharge overcurrent, or "comparator armed"); treat any of these as reason
  to stop and inspect before continuing.

## 5. Airbrake (aero flap) operation

The flight node drives up to twelve aero flap surfaces used as airbrakes/attitude control in
descent or high-speed flight modes.

- **Deploy:** say "deploy airbrakes," "deploy air brakes," "airbrakes out," or "full
  airbrakes." Flaps drive to full deflection at the configured rate limit.
- **Retract:** say "retract airbrakes," "retract air brakes," "airbrakes in," or "stow
  airbrakes." Flaps return to neutral.
- Every commanded flap position is clamped against a hard geometric/mechanical envelope table
  in firmware immediately before the final actuator write — after everything else (planner,
  mixer, voice mapping) has had a chance to weigh in, and with no code path around it.
  Overlapping surface pairs have a joint exclusion zone so they physically cannot collide.
- In PASSIVE or ESTOP, flaps unconditionally go to the table's neutral column and the aero
  engine keeps publishing telemetry the entire time — you always know where the surfaces are,
  even mid-fault.
- Mechanical end-stops are the ultimate backstop; the firmware clamp exists so the servos
  never have to lean on those stops in normal operation.

## 6. Voice commands

The helmet listens locally first — these are handled on-suit with no cloud round-trip and
work fully offline:

| Say                                                              | Does                                    |
|-------------------------------------------------------------------|------------------------------------------|
| "status report" / "status" / "report" / "systems check"          | Spoken state, battery %, heartbeat health. |
| "power level" / "battery level" / "battery" / "charge"            | Spoken battery % and pack voltage.        |
| "deploy airbrakes" / "deploy air brakes" / "airbrakes out" / "full airbrakes" | Deploys aero flaps.       |
| "retract airbrakes" / "retract air brakes" / "airbrakes in" / "stow airbrakes" | Retracts aero flaps.    |
| "emergency stop" / "e-stop" / "estop" / "kill power"              | Engages e-stop (see §3).                  |
| "clear estop" / "clear e stop" / "clear emergency stop" / "reset estop" | Clears e-stop (see §3 procedure). |
| "brightness" / "hud brightness [number]"                          | Sets HUD brightness.                      |

Anything that doesn't match one of the patterns above is forwarded to Node 9 (this system) as
a free-form query — that's the path that reaches this manual and produces a spoken advisory
back to the helmet. Cloud replies are always advisory speech; they are never a substitute for
the local commands above, and they can never move the suit.
