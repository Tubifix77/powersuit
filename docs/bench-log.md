# Bench log

The state of the physical bench: what hardware exists, what has been proven on
it, and what has not. [`bringup.md`](bringup.md) is the *procedure*; this file is
the *record*. Update it when something is verified, not when it is planned.

## Current status — 2026-10-01

| | |
|---|---|
| Hardware | **Arrived.** Verified on the real units by the sibling project (Potluck). |
| Boards right now | **In use by Potluck** — a 24 h unattended soak, ending ~19:20 on 2026-10-02 |
| Powersuit on silicon | **Nothing yet.** No Powersuit image has been flashed to any board. |
| Images | **Both `node_bench` roles built and verified** (2026-10-01) — `bash firmware/tools/build_bench.sh` |
| Next | After the soak: flash with `firmware/tools/flash_bench.ps1` ([A.4d](bringup.md)), then run [A.5](bringup.md) |

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

**Transceivers** (still sealed, verified from the seller's photos of the exact
units): 4-pin header `3.3V / GND / RX / TX`; no RS pin exposed; CANH/CANL out
twice in parallel, on a 2-pin header and a screw terminal.

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

## Not yet verified

- **RGB LED pin** — GPIO38 (v1.1) or GPIO48 (v1.0). Potluck's firmware never
  drives the LED, so nothing was learned. Use the [A.3](bringup.md) probe mode.
- **Transceiver termination** — an SMD part marked `121` (120 Ω) suggests onboard
  termination; unmeasured, and no multimeter was bought. At two nodes over 20 cm
  it barely matters.
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
