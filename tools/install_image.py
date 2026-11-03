#!/usr/bin/env python3
"""install_image.py - full-image code-flash install through the proven chain.

The complete write path, bench-proven end-to-end 2026-11-03 (4465 units,
13 blocks, zero refusals, verified, booted):

  1. tools/arm_storm.py catches the entry window at power-up and exits
     on the catch; this tool takes over within seconds.
  2. plant session -> forged key -> engine DOWNLOADED early.
  3. The launch is DEFERRED to --arm-at seconds after the catch: the
     engine idles until launched, so the boot ages with no 500k session
     alive (sessions die after ~90 s idle; the ARM is served only on an
     aged boot - FWE, the write-permission pin, rises on the boot's own
     schedule around boot+2 min).
  4. Launch -> 500k handshake (the session is BORN aged) -> REDUCED
     prime (attestations + verify ladder; the identify reads are dropped
     - they only complete against the flash the table was captured from
     and one truncated frame kills the wire-order cipher).
  5. Fresh re-hail (counter reset), then the sig-first pass: one op-08
     per block decides skip/erase+write, and the FIRST UNIT'S ARM IS THE
     FWE RAIL - a 0x51 acceptance means write permission and its program
     follows immediately (the grammar is erase -> ARM -> program; never
     insert a verify between an accepted ARM and its program); a 91 02
     22 30 refusal means this boot will not write: power-cycle and
     rerun.  Sig-mismatched blocks are ALWAYS erased before their units
     (programming onto non-blank content is refused 92 02 22 33).
  6. Final verify: every block's live signature must equal the table.

Resume-safe across power cycles: the same sig comparison skips finished
blocks.  A session writes for many minutes; if it dies mid-pass, rerun
after a power cycle.

Signature convention (spec 16 §12, corrected 2026-11-03): the op-08
signature is lo = zlib_crc32(data) XOR byte_sum (NO inversion; the hi
half is the reflected CRC32 poly 0xEB31D82E - both init 0xFFFFFFFF).
Build the --sigs table with exactly this formula over your image.

Usage:
  python3 install_image.py --engine <engine.bin> --image <img.bin> \
      --sigs <sigs.json> [--arm-at 122] [--dry]

The engine image is the session engine captured from YOUR unit (vendor
code, never distributed here); <img.bin> and <sigs.json> are yours too.
SAFETY: refuses any plan block outside code flash (0x4000-0x27E000) and
never touches the boot region.

Part of 8hp-teensy-flasher.  License: GPLv3 (see LICENSE).
"""
import argparse
import json
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import protocol as P
from flasher import Flasher, log

FLASH_LO, FLASH_HI = 0x004000, 0x27E000

# op-08 signatures of an all-FF block (observed engine reads, not oracle
# outputs - keep the engine convention, see the docstring above)
BLANK = {0x10000: "de547e4e81cb0165", 0x20000: "14b603cca7a12f57"}


def die(msg):
    log(f"*** ABORT: {msg}")
    sys.exit(1)


class Installer:
    def __init__(self, engine_path, image_path, sigs_path):
        self.f = Flasher(engine_path)
        self.img = open(image_path, "rb").read()
        self.sigs = {}
        for k, v in json.load(open(sigs_path)).items():
            start, size = (int(p, 0) for p in k.split("/"))
            if not (FLASH_LO <= start < FLASH_HI) or start % size or size not in BLANK:
                die(f"bad plan block {k:#x}")
            if start + size > len(self.img):
                die(f"image too short for {k}")
            self.sigs[(start, size)] = v

    # ------------------------------------------------------- session ops
    def region_sig(self, start, size, tries=4):
        for i in range(tries):
            reps = self.f.sess.cmd(f"verify {start:#x}/{size:#x}",
                                   P.compose_verify(start, size),
                                   nreply=2, tmo=4.0)
            if reps and reps[0][0] == 0x48:
                return reps[-1].hex()
            log("  busy, retry")
            time.sleep(2.5)
        return None

    # ------------------------------------------------------------ passes
    def install(self):
        written = skipped = 0
        t0 = time.time()
        first_block = True
        for (s, sz), want in sorted(self.sigs.items()):
            live = self.region_sig(s, sz)
            if live is None and first_block:
                die("session deaf at the first verify - power-cycle and rerun")
            first_block = False
            if live == want:
                log(f"block {s:#x}: sig MATCH - done")
                skipped += 1
                continue
            tgt_blank = self.img[s:s + sz] == b"\xff" * sz
            if tgt_blank and sz in BLANK and live == BLANK[sz]:
                log(f"block {s:#x}: blank as target - done")
                skipped += 1
                continue
            units = [o for o in range(s, s + sz, 0x100)
                     if self.img[o:o + 0x100] != b"\xff" * 0x100]
            # sig-mismatched blocks are ALWAYS erased first: the grammar
            # is erase -> ARM -> program, and programming onto non-blank
            # content is refused (92 02 22 33, bench 2026-11-03)
            if tgt_blank:
                log(f"block {s:#x}: unexpected content -> erase to blank")
            else:
                log(f"block {s:#x}: sig {live} -> ERASE + {len(units)} units")
            ff, cf = P.compose_erase(s)
            reps = self.f.sess.cmd(f"erase {s:#x}", [ff, cf], tmo=8.0)
            if not (reps and reps[0][0] == 0x60):
                die(f"erase {s:#x} refused: {[r.hex() for r in reps]}")
            if reps[0][1] & 0x80:
                time.sleep(2.5)
            for off in units:
                unit = self.img[off:off + 0x100]
                ok = False
                for attempt in range(3):
                    try:
                        ff, cf = P.compose_arm(off)
                        reps = self.f.sess.cmd("ARM", [ff, cf])
                        if not (reps and reps[0][0] == 0x51):
                            raise RuntimeError(
                                f"ARM {off:#x}: {[r.hex() for r in reps]}")
                        reps = self.f.sess.cmd("program",
                                               P.compose_program(unit), tmo=8.0)
                        if not (reps and reps[0][0] == 0x52):
                            raise RuntimeError(
                                f"PGM {off:#x}: {[r.hex() for r in reps]}")
                        ok = True
                        break
                    except RuntimeError as e:
                        log(f"  {e} retry {attempt + 1}")
                        time.sleep(2.0)
                if not ok:
                    die(f"unit {off:#x} FAILED - power-cycle and rerun "
                        "(resume-safe)")
                written += 1
                if written % 200 == 0:
                    log(f"  {written} units ({time.time() - t0:.0f}s)")
        log(f"pass done: {written} units written, {skipped} blocks complete "
            f"({time.time() - t0:.0f}s)")

    def final_verify(self):
        log("--- final signature verify ---")
        bad = 0
        for (s, sz), want in sorted(self.sigs.items()):
            live = self.region_sig(s, sz)
            ok = live == want
            bad += not ok
            log(f"  {s:#x}: {live} {'OK' if ok else f'*** WANT {want} ***'}")
        log("*** INSTALL COMPLETE - power-cycle for boot ***" if bad == 0
            else f"*** {bad} BLOCKS OFF - power-cycle and rerun (resume-safe) "
                 "***")
        return bad == 0


def main():
    ap = argparse.ArgumentParser(
        description="Full-image code-flash install (sig-first, resume-safe)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--engine", required=True,
                    help="session engine image for YOUR unit")
    ap.add_argument("--image", required=True, help="full flash image to install")
    ap.add_argument("--sigs", required=True,
                    help='signature table {"<start>/<size>": "<op-08 sig>"} '
                         "(lo = crc32^bytesum, hi = reflected crc32)")
    ap.add_argument("--arm-at", type=int, default=122,
                    help="seconds after the catch to launch the engine and "
                         "open the aged session (default 122; the first unit "
                         "ARM then lands ~boot+150 where FWE is proven open)")
    ap.add_argument("--dry", action="store_true",
                    help="print the plan and exit (no bench)")
    args = ap.parse_args()

    inst = Installer(args.engine, args.image, args.sigs)
    if args.dry:
        n = 0
        for (s, sz), want in sorted(inst.sigs.items()):
            k = sum(1 for o in range(s, s + sz, 0x100)
                    if inst.img[o:o + 0x100] != b"\xff" * 0x100)
            log(f"  {s:#x}/{sz:#x}: sig {want}, {k} units")
            n += k
        log(f"plan: {len(inst.sigs)} blocks, {n} units total")
        return

    f = inst.f
    f.wait_for_storm()
    boot0 = time.time()                      # ~boot+3 at the catch
    f.plant_session()
    f.auth()
    f.download_engine()
    log(f"engine downloaded - launch deferred to catch+{args.arm_at}s "
        "(the boot ages; no session idles)")
    while time.time() - boot0 < args.arm_at:
        time.sleep(1.0)
    f.launch_engine()
    f.open_session()                          # 500k session born aged
    f.prime(reduced=True)
    f.sess.ctr = 0                            # fresh session for the write
    f.sess.handshake()                        # (handshake does not reset ctr)
    log(f"fresh session for the install (catch+{time.time() - boot0:.0f}s)")
    inst.install()                            # first unit ARM = the FWE rail
    ok = inst.final_verify()
    f.b.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
