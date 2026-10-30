#!/usr/bin/env python3
"""capture_engine.py - read the session engine out of YOUR OWN TCU.

The flasher downloads a small "session engine" image into TCU RAM each
time (the encrypted-session server).  It is vendor code, so this project
does not distribute it - capture it from your own unit once and pass it
to flasher.py via --engine.

The engine is RAM-resident during any programming session.  This tool
serves as a passive CAN logger: run it while the TCU is in a
programming session started by your own diagnostic equipment, and it
reassembles the 34/36/37 transfer into the image file.  It never
transmits on the bus (listen-only).

USAGE:
  python3 capture_engine.py --out engine.bin

The log format is the same slcan stream the firmware produces on either
USB port; frames are parsed from /tmp/engine_capture.log if present, or
you can adapt the reader to your own capture.  Reassembly follows the
transfer protocol: 34 (start, address+size), 36 <seq> <128B> blocks
(block N at (N-1)*0x100), 37 (end).

Part of 8hp-teensy-flasher.  License: GPLv3 (see LICENSE).
"""
import argparse
import re
import struct


def parse_log(path):
    """Yield ('tx'|'rx', bytes) UDS payloads from an slcan-format log."""
    frames = []
    for line in open(path):
        m = re.search(r"\[(B\d+)\] [tT]7E([19])([0-9A-Fa-f])", line)
        if not m:
            continue
        dlc = int(m.group(3), 16)
        rest = line[m.end():].strip().split()
        if not rest:
            continue
        h = rest[0][:dlc * 2]
        if len(h) < dlc * 2:
            continue
        frames.append(("tx" if m.group(2) == "1" else "rx", bytes.fromhex(h)))
    return frames


def reassemble(frames):
    reqs, buf, need = [], b"", None
    for _, p in frames:
        pci = p[0] >> 4
        if pci == 0:
            reqs.append(p[1:1 + (p[0] & 0x0F)])
        elif pci == 1:
            need = ((p[0] & 0x0F) << 8) | p[1]
            buf = p[2:8]
        elif pci == 2 and need is not None:
            buf += p[1:8]
            if len(buf) >= need:
                reqs.append(buf[:need])
                need = None
    return reqs


def extract_engine(reqs):
    tx = {}
    info = None
    for p in reqs:
        if not p:
            continue
        if p[0] == 0x34 and len(p) >= 11:
            info = (int.from_bytes(p[4:8], "big"), int.from_bytes(p[8:11], "big"))
            tx = {}
        elif p[0] == 0x36 and len(p) > 1:
            tx[p[1]] = p[2:]
        elif p[0] == 0x37 and tx:
            return info, b"".join(tx[k] for k in sorted(tx))
    return info, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default="/tmp/engine_capture.log")
    ap.add_argument("--out", default="engine.bin")
    args = ap.parse_args()
    frames = parse_log(args.log)
    print(f"{len(frames)} frames parsed")
    reqs = reassemble(frames)
    info, img = extract_engine(reqs)
    if img is None:
        print("no complete 34/36/37 transfer in the log")
        return 1
    print(f"transfer: address {info[0]:#x}, {info[1]} bytes")
    open(args.out, "wb").write(img)
    print(f"wrote {args.out} ({len(img)} bytes) - pass to flasher.py --engine")
    return 0


if __name__ == "__main__":
    sys.exit(main())
