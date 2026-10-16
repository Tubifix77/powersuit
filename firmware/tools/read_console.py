"""Read a node_bench board's console from boot, non-interactively.

    <idf-python> firmware/tools/read_console.py COM5 --seconds 20 --until SELFTEST_DONE

Resets the board into its application first, so the capture starts at boot:
EN is pulsed low through RTS while DTR (IO0) is held high — esptool's hard-reset
sequence. Opening the port with pyserial's defaults would also reset the board,
but by toggling both lines, which is not a sequence to rely on (docs/bench-log.md).

Needs pyserial, which lives in the ESP-IDF Python environment, not the system
Python. Exit status: 0 when --until matched a line ending in "OK" or no verdict was
asked for, 1 when it matched any other line (a FAIL verdict), 2 on timeout.
"""
from __future__ import annotations

import argparse
import re
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: run this with the ESP-IDF environment's python.exe")

ANSI = re.compile(r"\x1b\[[0-9;]*m")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--until", help="stop at the first line containing this text")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-reset", action="store_true", help="attach without rebooting the board")
    args = ap.parse_args()

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = args.port, args.baud, 0.2
    s.dtr = False          # IO0 high: boot the application, not the ROM bootloader
    s.rts = False
    s.open()
    if not args.no_reset:
        s.rts = True       # EN low
        s.dtr = s.dtr      # Windows usbser quirk: re-assert DTR so RTS takes effect
        time.sleep(0.1)
        s.rts = False      # EN high: run
        s.dtr = s.dtr

    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            raw = s.readline()
            if not raw:
                continue
            line = ANSI.sub("", raw.decode("utf-8", errors="replace")).rstrip()
            print(line, flush=True)
            if args.until and args.until in line:
                return 0 if re.search(r"\bOK\b", line) else 1
    finally:
        s.close()
    return 2 if args.until else 0


if __name__ == "__main__":
    sys.exit(main())
