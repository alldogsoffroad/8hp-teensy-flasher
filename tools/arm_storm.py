#!/usr/bin/env python3
"""arm_storm.py - wake the TCU's programming session during power-up.

The TCU only opens its programming session for a few milliseconds after
reset, and only if the entry frame is ALREADY on the bus.  This tool
sends it continuously at 666.667 kbit/s on 29-bit 0x1F0C3CFE and watches
for the TCU's reply on 0x1FFFAFFE (then exits so the flasher can take
the ports).

USAGE:
  1. Start this tool.
  2. Power the TCU: OFF, hold ~30 s (rails must fully discharge), ON.
  3. It prints "TCU FRAME" when the session opens, then exits.

Mode selection (first matching frame wins):
  --mode 5   "PlantCan" entry (flash programming)   [default]
  --mode 6   "PlantKcn" entry (alternate server)

Part of 8hp-teensy-flasher.  License: GPLv3 (see LICENSE).
"""
import argparse
import subprocess
import sys
import threading
import time

import serial

PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]
WINDOW_S = 300
SLOT_S = 0.025
WORDS = {5: "506C616E7443616E", 6: "506C616E744B636E"}   # "PlantCan"/"PlantKcn"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", type=int, default=5, choices=(5, 6))
    args = ap.parse_args()
    word = WORDS[args.mode]
    frames = [("T", "1F0C3CFE", 8, word), ("T", "1FFFAFFE", 8, word)]

    ser = {}
    for p in PORTS:
        s = serial.Serial(p, 2000000, timeout=0.02)
        for cmd in (b"C\r", b"P0\r", b"S9\r", b"O\r"):
            s.write(cmd)
            time.sleep(0.05)
            s.read(64)
        s.reset_input_buffer()
        ser[p] = s

    log = open("/tmp/arm_storm.log", "w", buffering=1)
    stop = threading.Event()
    seen = threading.Event()

    def watch(port):
        s = ser[port]
        while not stop.is_set():
            buf = s.read(4096)
            for line in buf.decode("latin1").splitlines():
                line = line.strip("\r")
                if not line or line in ("z", "Z", "\x07"):
                    continue
                log.write(f"{time.time():.3f} [{port[-1]}] {line}\n")
                if line.startswith("T1FFFAFFE"):
                    print("!!! TCU FRAME: session open", flush=True)
                    seen.set()

    for p in PORTS:
        threading.Thread(target=watch, args=(p,), daemon=True).start()

    print(f"storm armed (mode {args.mode}); power the TCU: OFF 30 s, then ON",
          flush=True)
    t_end = time.time() + WINDOW_S
    next_slot = time.time()
    while time.time() < t_end and not seen.is_set():
        now = time.time()
        if now >= next_slot:
            for p in PORTS:
                for kind, id_, dlc, payload in frames:
                    ser[p].write((kind + id_ + str(dlc) + payload + "\r").encode())
            next_slot += SLOT_S
        time.sleep(0.002)

    stop.set()
    for p in PORTS:
        ser[p].write(b"C\r")
        ser[p].close()
    print("storm done" if seen.is_set() else "window expired, no catch", flush=True)
    sys.exit(0 if seen.is_set() else 1)


if __name__ == "__main__":
    main()
