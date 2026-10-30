#!/usr/bin/env python3
"""protocol.py - the TCU session protocol, reverse-engineered.

Covers everything the flasher needs, as validated on the bench:

  WAKE / PLANT SESSION (666.667 kbit/s, on power-up)
    - Entry frame: 29-bit 0x1F0C3CFE, 8 bytes ASCII.  "PlantCan..." arms
      mode 5 (flash programming), "PlantKcn..." mode 6.  Send it every
      ~2 ms while the TCU powers up; the TCU replies once on 29-bit
      0x1FFFAFFE (53-report) and stays in the session.
    - Plant session: UDS-style on 0x7E1/0x7E9 at 666.667 kbit/s.
      10 60 (session), then SecurityAccess 27 61 -> 16-byte seed,
      27 62 -> 128-byte key (see forge_plant_key).

  PLANT AUTH: 27 62 KEY = integer cube root (no secret exists)
    m = 00 01 FF*8 00 || "PCMFLASHMEDB" || <free bytes> || RIPEMD160(seed)
    (m is 127 bytes; the 20-byte digest is the LAST 20 bytes).  The TCU
    checks key^3 for that structure.  We pin the digest tail with a
    Hensel lift mod 2^160 and search the prefix window.  A digest whose
    2-adic valuation is not a multiple of 3 cannot be used - request a
    fresh seed and retry.

  ENGINE IMAGE (downloaded each session via 34/36/37 to 0xFFF88400)
    - 34 00 34 <addr BE32> <len BE24>; 36 <seq> <256B blocks>; 37.
    - Image layout: 0xB0-byte header, encrypted body [0xB0, 0x2A20),
      128-byte signature trailer.  The launcher cube-verifies the
      signature (m = sig^3 mod n, m[:11] = 00 01 FF*8 00,
      m[11:31] = RIPEMD160(image[:-128])) then decrypts the body with a
      byte-oriented CFB word cipher (X=0x554E5444, K=0xC34186A3, s=0)
      and jumps.  Use your own captured engine image as-is (its original
      signature already verifies).
    - 31 01 FF 01 launches it; the reply 71 01 FF 01 ... confirms.

  ENCRYPTED FLASH SESSION (500 kbit/s)
    - After launch, switch the Teensy to S6 (500 kbit/s).
    - Hail: 4-byte frame 54 FF 3D 22 on 0x7E1 -> 4-byte seed on 0x7E9.
    - Cipher: ChaCha with 8 rounds, per-word byteswap on output.
      state[0..11] = 12 header constants from the engine image
      (offsets 0x10..0x20, 0x30, 0x34..0x50, little-endian);
      state[12/13] = 64-bit block counter, +1 per 8-byte frame in wire
      order from the challenge on; state[14] = the hail word; state[15]
      = the seed.  Every 8-byte frame in both directions is one block.
    - Challenge: E(30 00*6 85) at counter 0; expect 70 04 4D 4D 53 ...

  COMMAND FRAMING (inside the encrypted session)
    - [op][PCI][...data...][ck], ck = (sum of message bytes + 0x55) & FF
      on the last frame of the message.  PCI at byte 1: 0X = single
      frame (X data bytes), 1X YY = first frame of a 0XYY-byte message
      (continuations follow as raw frames).
    - Ops: 30 ping; 01/02 attestation exchanges (constants, see
      PRIMING); 11 read-arm; 13 next-block (repeats); 08 region
      checksum; 20 code-flash erase; 21 data-flash erase; 12 program.

  WRITE GRAMMAR (all CRCs are CRC-16/CCITT-FALSE: poly 0x1021,
  init 0xFFFF, no reflect, no xorout)
    erase(block):   FF [20][10 06][00][crc16(FCU_addr,00,01)][00][ck]
                    CF [FCU_addr BE32 = 0x80800000+block][00 01][00 00]
    arm(offset):    FF [11][10 08][00][crc16(CF8)][00][ck]
                    CF [FCU_addr = 0x80800000+offset][00 01][e8][01]
    program(data):  FF [12][11 00][00 00][crc16(256B data)][00][ck]
                    + 32 raw 8-byte data frames
    verify:         FF [08][10 08][00][crc16(CF8)][00][ck]
                    CF [region start BE32][size BE32]
    Erase replies 60 XX (XX bit7 set = content was erased -> wait ~2.5 s
    for the FCU before the next command).  arm -> 51, program -> 52.

  MECHANICS
    - Pace ALL session TX with >= 4 ms gaps: back-to-back frames are
      dropped silently by the USB-CAN queue.
    - The write path requires the priming sequence (see priming_seq.py)
      before composed erase/arm/program commands are accepted.

Part of 8hp-teensy-flasher.  License: GPLv3 (see LICENSE).
"""
import hashlib
import struct

M32 = 0xFFFFFFFF
K_PLANT = "PCMFLASHMEDB"


# ---------------------------------------------------------------- cipher
def _rotl(v, n):
    return ((v << n) | (v >> (32 - n))) & M32


def _qr(x, a, b, c, d):
    x[a] = (x[a] + x[b]) & M32; x[d] ^= x[a]; x[d] = _rotl(x[d], 16)
    x[c] = (x[c] + x[d]) & M32; x[b] ^= x[c]; x[b] = _rotl(x[b], 12)
    x[a] = (x[a] + x[b]) & M32; x[d] ^= x[a]; x[d] = _rotl(x[d], 8)
    x[c] = (x[c] + x[d]) & M32; x[b] ^= x[c]; x[b] = _rotl(x[b], 7)


def block(state, rounds=8):
    """One block, output words byte-swapped, big-endian packed."""
    x = list(state)
    for _ in range(rounds // 2):
        _qr(x, 0, 4, 8, 12); _qr(x, 1, 5, 9, 13)
        _qr(x, 2, 6, 10, 14); _qr(x, 3, 7, 11, 15)
        _qr(x, 0, 5, 10, 15); _qr(x, 1, 6, 11, 12)
        _qr(x, 2, 7, 8, 13); _qr(x, 3, 4, 9, 14)
    out = [(x[i] + state[i]) & M32 for i in range(16)]
    return struct.pack(">16I", *(int.from_bytes(struct.pack(">I", w), "little")
                                 for w in out))


def state_from_engine(img):
    """The 12 header constants + room for counter/hail/seed."""
    w = lambda off: int.from_bytes(img[off:off + 4], "little")
    return [w(0x10), w(0x14), w(0x18), w(0x1C), w(0x30),
            w(0x34), w(0x38), w(0x3C), w(0x40), w(0x44), w(0x48), w(0x4C),
            0, 0, 0, 0]


# ------------------------------------------------------------- plant key
def _v2(x):
    return (x & -x).bit_length() - 1


def _icbrt(x):
    if x == 0:
        return 0
    a = 1 << ((x.bit_length() + 2) // 3)
    while True:
        b = (2 * a + x // (a * a)) // 3
        if b >= a:
            return a
        a = b


def forge_plant_key(seed16):
    """128-byte 27-62 key for a 16-byte seed, or None (reseed and retry)."""
    digest = hashlib.new("ripemd160", seed16).digest()
    d = int.from_bytes(digest, "big")
    if d == 0 or _v2(d) % 3 != 0:
        return None
    prefix = bytes([0x00, 0x01] + [0xFF] * 8 + [0x00]) + K_PLANT.encode()
    lo = int.from_bytes(prefix + b"\x00" * 105, "big")
    hi = int.from_bytes(prefix + b"\xff" * 105, "big") + 1
    s = _v2(d)
    odd = d >> s
    a0 = pow(odd % (1 << (160 - s)), pow(3, -1, 1 << (158 - s)), 1 << (160 - s)) \
        if odd % 2 else 0
    a_low = (a0 << (s // 3)) % (1 << 160)
    mod = 1 << 160
    a = a_low + ((_icbrt(lo) - a_low + mod - 1) // mod) * mod
    for _ in range(4096):
        m = a ** 3
        if lo <= m < hi and (m & (mod - 1)) == d:
            return a.to_bytes(128, "big")
        if m >= hi:
            return None
        a += mod
    return None


# ------------------------------------------------------------- checksums
def crc16(data):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def ck7(head7):
    return head7 + bytes([(sum(head7) + 0x55) & 0xFF])


# ------------------------------------------------------ write composers
FCU_BASE = 0x80800000


def compose_erase(block):
    fcu = (FCU_BASE + block).to_bytes(4, "big")
    cf = fcu + bytes([0x00, 0x01, 0x00, 0x00])
    ff = ck7(bytes([0x20, 0x10, 0x06, 0x00])
             + crc16(fcu + bytes([0x00, 0x01])).to_bytes(2, "big") + b"\x00")
    return ff, cf


def compose_arm(offset):
    fcu = (FCU_BASE + offset).to_bytes(4, "big")
    cf = fcu + bytes([0x00, 0x01, 0xE8, 0x01])
    ff = ck7(bytes([0x11, 0x10, 0x08, 0x00])
             + crc16(cf).to_bytes(2, "big") + b"\x00")
    return ff, cf


def compose_program(data256):
    ff = ck7(bytes([0x12, 0x11, 0x00, 0x00])
             + crc16(data256).to_bytes(2, "big") + b"\x00")
    return [ff] + [data256[i:i + 8] for i in range(0, 256, 8)]


def compose_verify(region_start, size):
    cf = region_start.to_bytes(4, "big") + size.to_bytes(4, "big")
    ff = ck7(bytes([0x08, 0x10, 0x08, 0x00])
             + crc16(cf).to_bytes(2, "big") + b"\x00")
    return [ff, cf]
