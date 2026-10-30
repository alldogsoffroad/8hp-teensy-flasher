#!/usr/bin/env python3
"""flasher.py - the Teensy-only flash tool for the Ram 1500 TCU.

Drives the complete chain with no external tool: wake the TCU's
programming session at power-up, authenticate with a forged (legitimate
math, no secret) plant key, load the session engine, open the encrypted
500 kbit/s session, and erase/program/verify code flash.

One invocation = one TCU power cycle (the wake entry only opens during
boot; see hardware/README.md for the bench rig):

  1. Arm the wake storm separately (tools/arm_storm.py) while you
     power the TCU on.
  2. Run this tool; it waits for the storm to exit, then:
       plant session -> SecurityAccess -> engine download -> launch
       -> encrypted session -> priming -> your command.

Usage:
  python3 flasher.py --engine <engine.bin> status
  python3 flasher.py --engine <engine.bin> erase 0xE0000
  python3 flasher.py --engine <engine.bin> write 0xE1000 deadbeef...
  python3 flasher.py --engine <engine.bin> verify 0xE0000 0x2000

The engine image is the session engine for YOUR unit, captured from
your own TCU with tools/capture_engine.py (or any CAN logger).  It is
vendor code and is never distributed with this project.

SAFETY: erasing a block destroys its contents.  This tool refuses
addresses outside code flash (0x4000-0x27E000) and never touches the
boot region.  Write operations move real flash - use on your own unit.

Part of 8hp-teensy-flasher.  License: GPLv3 (see LICENSE).
"""
import argparse
import hashlib
import subprocess
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import canbus
import protocol as P
from priming_seq import PRIMING

ENGINE_DEFAULT = "/dev/ttyACM0"     # CAN1 = TCU bus (see hardware/)
FLASH_LO, FLASH_HI = 0x004000, 0x27E000
HAIL = bytes.fromhex("54FF3D22")
CHALLENGE = bytes.fromhex("3000000000000085")


def log(msg):
    print(msg, flush=True)


def die(msg):
    log(f"*** ABORT: {msg}")
    sys.exit(1)


class Session:
    """The encrypted 500 kbit/s session (state = header + ctr + hail + seed)."""

    def __init__(self, buses, n, engine_img):
        self.b, self.n = buses, n
        self.st = P.state_from_engine(engine_img)
        self.ctr = 0

    def _ks(self):
        s = list(self.st)
        s[12] = self.ctr & P.M32
        s[13] = (self.ctr >> 32) & P.M32
        return P.block(s)[:8]

    def send(self, pt, gap=0.004):
        assert len(pt) == 8
        ct = bytes(a ^ b for a, b in zip(pt, self._ks()))
        self.ctr += 1
        self.b.send(self.n, 0x7E1, ct)
        time.sleep(gap)          # mandatory: back-to-back frames drop

    def recv(self, timeout=2.0):
        bus = self.b.get(self.n)
        end = time.time() + timeout
        while time.time() < end:
            m = bus.recv(0.05)
            if m is None or m.arbitration_id != 0x7E9 or len(m.data) != 8:
                continue
            pt = bytes(a ^ b for a, b in zip(bytes(m.data), self._ks()))
            self.ctr += 1
            return pt
        return None

    def handshake(self):
        self.b.send(self.n, 0x7E1, HAIL)
        bus = self.b.get(self.n)
        end = time.time() + 3.0
        seed = None
        while time.time() < end:
            m = bus.recv(0.05)
            if m is None or m.arbitration_id != 0x7E9 or len(m.data) != 4:
                continue
            seed = int.from_bytes(m.data, "big")
            break
        if seed is None:
            die("no seed after hail")
        self.st[14] = int.from_bytes(HAIL, "big")
        self.st[15] = seed
        self.send(CHALLENGE)
        rep = self.recv(3.0)
        if not rep or rep[:5] != bytes.fromhex("70044D4D53"):
            die(f"handshake failed: {rep.hex() if rep else 'no reply'}")
        log(f"*** session up (seed {seed:08X}) ***")

    def cmd(self, name, frames, nreply=1, tmo=4.0):
        for f in frames:
            self.send(f)
        reps = []
        for _ in range(nreply):
            r = self.recv(tmo)
            if r is None:
                break
            reps.append(r)
        log(f"{name}: {[r.hex() for r in reps]}")
        return reps


class Flasher:
    def __init__(self, engine_path):
        self.engine = open(engine_path, "rb").read()
        self.b = None
        self.n = None
        self.sess = None

    # ------------------------------------------------------------ chain
    def wait_for_storm(self):
        log("waiting for the wake storm to finish ...")
        for _ in range(360):
            r = subprocess.run(["pgrep", "-f", "arm_storm[.]py"], capture_output=True)
            if r.returncode != 0:
                return
            time.sleep(2.0)
        die("storm still running")

    def plant_session(self):
        canbus.teensy_passive([canbus.BusArgs.bus1, canbus.BusArgs.bus2], b"P0")
        self.b = canbus.Buses(canbus.BusArgs())
        self.n = None
        for cand in (1, 2):
            try:
                u = canbus.Uds(self.b, cand)
                r = u.request(bytes.fromhex("1060"), timeout=2.0)
            except Exception:
                continue
            if r and r[:2] == b"\x50\x60":
                self.n = cand
                break
        if self.n is None:
            die("no plant session - arm the storm and power-cycle the TCU")
        u = canbus.Uds(self.b, self.n)
        u.cf_gap_s = 0.004
        u.honor_stmin = False
        self._plant = u
        log(f"plant session on bus {self.n} (666.667 kbit/s)")

    def auth(self):
        for attempt in range(16):
            r = self._plant.request(bytes.fromhex("2761"), timeout=5)
            if not r or r[:2] != b"\x67\x61":
                die("seed request failed")
            key = P.forge_plant_key(r[2:18])
            if key is None:
                continue                     # digest unusable: reseed
            time.sleep(0.2)
            r = self._plant.request(bytes([0x27, 0x62]) + key, timeout=30)
            log(f"27 62 -> {r.hex() if r else 'none'}")
            if r and r[:2] == b"\x67\x62":
                log("*** SecurityAccess passed (forged plant key) ***")
                return
        die("plant authentication failed")

    def download_engine(self):
        img = self.engine
        log(f"downloading session engine: {len(img)}B @ 0xFFF88400")
        r = self._plant.request(
            bytes([0x34, 0x00, 0x34]) + (0xFFF88400).to_bytes(4, "big")
            + len(img).to_bytes(3, "big"), timeout=15)
        if not r or r[0] != 0x74:
            die("RequestDownload refused")
        seq = 1
        for off in range(0, len(img), 0x100):
            r = self._plant.request(bytes([0x36, seq]) + img[off:off + 0x100],
                                    timeout=15)
            if not r or r[0] != 0x76:
                die(f"block {seq} rejected")
            seq += 1
        self._plant.request(b"\x37", timeout=15)
        for i in range(400):
            r = self._plant.request(bytes.fromhex("3101FF01"), timeout=5)
            if r and r[:4] == bytes([0x71, 0x01, 0xFF, 0x01]):
                log("*** engine launched ***")
                return
            time.sleep(0.01)
        die("engine launch not confirmed")

    def open_session(self):
        port = self.b.names[self.n]
        self.b.shutdown()
        for cmd in (b"C\r", b"P0\r", b"S6\r", b"O\r"):
            canbus.teensy_passive([port], cmd)
        self.b = canbus.Buses(canbus.BusArgs())
        self.sess = Session(self.b, self.n, self.engine)
        self.sess.handshake()

    def prime(self):
        log(f"priming the write context ({len(PRIMING)} commands) ...")
        for hexframe, nrx in PRIMING:
            self.sess.send(bytes.fromhex(hexframe))
            for _ in range(nrx):
                if self.sess.recv(2.5) is None:
                    die("priming stalled")
        log("context primed")

    # ------------------------------------------------------------- ops
    def verify(self, start, size, tries=4):
        for i in range(tries):
            reps = self.sess.cmd(f"verify {start:#x}/{size:#x}",
                                 P.compose_verify(start, size), nreply=2)
            if reps and reps[0][0] == 0x48:
                return reps
            log(f"  busy, retry {i + 1}")
            time.sleep(2.5)
        return reps

    def erase(self, block):
        if not (FLASH_LO <= block < FLASH_HI) or block % 0x2000:
            die(f"{block:#x} is not an aligned code-flash block")
        ff, cf = P.compose_erase(block)
        reps = self.sess.cmd(f"erase {block:#x}", [ff, cf], tmo=8.0)
        if not reps or reps[0][0] != 0x60:
            die("erase refused")
        if reps[0][1] & 0x80:
            log("content erased: waiting out the FCU busy window")
            time.sleep(2.5)

    def write(self, addr, data256):
        ff, cf = P.compose_arm(addr)
        reps = self.sess.cmd("arm", [ff, cf])
        if not reps or reps[0][0] != 0x51:
            die("arm refused")
        reps = self.sess.cmd("program", P.compose_program(data256), tmo=8.0)
        if not reps or reps[0][0] != 0x52:
            die("program refused")


def main():
    ap = argparse.ArgumentParser(
        description="Teensy-only flash tool for the Ram 1500 TCU",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--engine", required=True,
                    help="session engine image for YOUR unit (see docs)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status", help="open a session and read region checksums")
    e = sub.add_parser("erase"); e.add_argument("block", type=lambda x: int(x, 0))
    w = sub.add_parser("write")
    w.add_argument("addr", type=lambda x: int(x, 0))
    w.add_argument("data", help="<=256B hex, or @file")
    v = sub.add_parser("verify")
    v.add_argument("start", type=lambda x: int(x, 0))
    v.add_argument("size", type=lambda x: int(x, 0))
    args = ap.parse_args()

    f = Flasher(args.engine)
    f.wait_for_storm()
    f.plant_session()
    f.auth()
    f.download_engine()
    f.open_session()

    if args.cmd == "status":
        f.prime()
        f.verify(0x0E0000, 0x2000)
        f.verify(0x0C0000, 0x2000)
    elif args.cmd == "erase":
        f.prime()
        f.erase(args.block)
        f.verify(args.block, 0x2000)
        log("*** erase complete ***")
    elif args.cmd == "write":
        data = (open(args.data[1:], "rb").read() if args.data.startswith("@")
                else bytes.fromhex(args.data))
        if len(data) > 256:
            die("max 256 bytes per invocation")
        data = data + b"\xff" * (256 - len(data))
        block = args.addr & ~0x1FFF
        if not (FLASH_LO <= block < FLASH_HI):
            die("address outside code flash")
        f.prime()
        f.erase(block)
        f.write(args.addr, data)
        f.verify(block, 0x2000)
        log("*** WRITE COMPLETE ***")
    elif args.cmd == "verify":
        f.prime()
        f.verify(args.start, args.size)
    log("done.")


if __name__ == "__main__":
    main()
