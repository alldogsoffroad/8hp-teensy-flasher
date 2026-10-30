# The Ram 1500 TCU (ZF 8HP / SH7254R) flash protocol

Reverse-engineered documentation of the complete programming path, as
validated on the bench. Everything here is implemented in
`tools/protocol.py` and `tools/flasher.py`.

All multi-byte fields are big-endian unless noted. CRC-16 means
**CRC-16/CCITT-FALSE**: polynomial 0x1021, init 0xFFFF, no reflection,
no xor-out.

## 1. Boot wake (666.667 kbit/s)

At power-on reset the TCU listens on 29-bit CAN ID `0x1F0C3CFE` for a
few milliseconds (timed on its free-running counter at `0xFFFFF220`).
If an entry frame is already in the receive mailbox, the boot manager
stays resident and opens a programming session; otherwise the normal
application boots and the window is gone until the next true reset.

| Entry frame payload (8 bytes ASCII) | Session |
|---|---|
| `PlantCan` | mode 5 — flash programming |
| `PlantKcn` | mode 6 — alternate server |

The first matching frame wins. The TCU replies once on 29-bit
`0x1FFFAFFE` (`53 11 A5 00 ...`, the boot RAM-test report) and keeps
the session open. Practical notes:

- The entry frame must be transmitted continuously (~every 2 ms works)
  **while** the TCU powers up; the window is sub-50 ms after reset.
- The TCU's supply rails take ~30 s to discharge after power is removed.
  Short power cuts do not reopen the window.

## 2. Plant session (666.667 kbit/s, UDS over 0x7E1/0x7E9)

ISO-TP framed UDS: `10 60` (session), `27 61` (seed request → 16-byte
seed), `27 62 <128-byte key>` (key), then the transfer services
`34/36/37` and the launch routine `31 01 FF 01`.

Pacing: keep ≥4 ms between consecutive ISO-TP consecutive frames; the
receiver drops back-to-back frames silently.

### 2.1 SecurityAccess: the integer-cube key

The 128-byte key is a **cube root** — there is no secret. The TCU
checks `key³` (as a plain integer, no modulus) for this structure:

```
m = 00 01 FF FF FF FF FF FF FF FF 00      (11 bytes)
    "PCMFLASHMEDB"                        (12 bytes, ASCII)
    <85 free bytes>                       (garbage is fine)
    RIPEMD-160(seed)                      (20 bytes — the LAST 20 bytes)
m is 127 bytes as an integer (the leading 00 is the top byte)
```

`tools/protocol.py:forge_plant_key` constructs a valid key for any seed:

1. If the digest's 2-adic valuation isn't a multiple of 3, no cube root
   with that tail exists — request a fresh seed (seeds are random per
   request).
2. Hensel-lift the cube root of the digest modulo 2¹⁶⁰ — this pins the
   low 160 bits (the tail) of the root.
3. Search upward from the cube root of the pinned prefix for a root
   whose cube lands inside the prefix window. The free middle bytes
   give the search ~2¹⁰⁴ of room; it converges in a handful of steps.

The construction is deterministic and takes milliseconds.

## 3. Session engine (RAM image, downloaded each time)

Each programming session downloads a ~11 KB image to `0xFFF88400`
(`34 00 34 <addr> <len>`, `36 <seq> <128-byte blocks>`, `37`), then
`31 01 FF 01` launches it. The launch routine:

1. Cube-verifies the image's last 128 bytes as a signature:
   `m = sig³`, `m[:11] = 00 01 FF*8 00`, `m[11:31] = RIPEMD-160(image
   minus the signature)`.
2. Decrypts the body `[0xB0, 0x2A20)` in place — a byte-oriented CFB
   word cipher (X = 0x554E5444, K = 0xC34186A3, s = 0; each word:
   `ct = ks ^ pt`, `s_next = ((ks + pt) ^ K)`).
3. Jumps to the entry in the header.

The image contains the encrypted-session server. **This project does
not distribute it** (it is vendor code); capture it once from your own
unit with `tools/capture_engine.py`.

## 4. Encrypted session (500 kbit/s)

After the launch, switch the bus to 500 kbit/s (`S6`). The session is a
raw (non-ISO-TP) 8-byte-frame protocol on 0x7E1/0x7E9.

### 4.1 Handshake

1. Send the 4-byte hail `54 FF 3D 22` (plaintext).
2. The TCU replies with a 4-byte random seed (plaintext).
3. Send `E(challenge)` where the plaintext challenge is
   `30 00 00 00 00 00 00 85` (encrypted with counter 0).
4. Expect `70 04 4D 4D 53 14 xx xx` — the session is up.

### 4.2 Cipher

A ChaCha variant: 8 rounds, standard quarter-round rotations
(16/12/8/7), column+diagonal, feed-forward add, and a **per-word
byteswap on output**. The state:

| Words | Value |
|---|---|
| 0–11 | 12 constants read from the engine image header (offsets 0x10–0x20, 0x30, 0x34–0x50, little-endian) |
| 12/13 | 64-bit block counter — **+1 per 8-byte frame, both directions, in wire order** (the challenge is counter 0) |
| 14 | the hail word `54FF3D22` |
| 15 | the wire seed |

Every 8-byte frame is one block: `ct = pt ^ block(state)[0:8]`. The key
is fully wire-derived — the tester knows both the hail it chose and the
seed it received.

### 4.3 Command framing

```
[op][PCI][data...][checksum]        checksum = (sum of message bytes + 0x55) & 0xFF
```

The PCI at byte 1 follows ISO-TP shapes: `0X` = single frame with X data
bytes; `1X YY` = first frame of an `0XYY`-byte message, the remaining
bytes follow as raw 8-byte frames (no PCI, no per-frame checksum — the
message checksum rides on the last frame).

### 4.4 Operations

| Op | Meaning | Notes |
|---|---|---|
| `30` | ping | reply `70 04 4D 4D 53 …` |
| `01`,`02` | attestation exchanges | fixed constants (see `tools/priming_seq.py`) |
| `11` | arm / read-arm | |
| `13` | next block (reads) | repeats; the read pointer persists per session |
| `08` | region checksum query | |
| `20` | code-flash erase | |
| `21` | data-flash erase | `[21][04][addr32][00][ck]` |
| `12` | program 256 bytes | |

## 5. Write grammar

All erase/arm/program messages are two-frame (first frame + raw
continuation), with a CRC-16 of the continuation bytes in the first
frame:

```
erase(block):   FF [20][10 06][00][CRC16(addr4 || 00 01)][00][ck]
                CF [FCU_addr = 0x80800000 + block][00 01][00 00]
arm(offset):    FF [11][10 08][00][CRC16(CF8)][00][ck]
                CF [0x80800000 + offset][00 01][E8][01]
program(data):  FF [12][11 00][00 00][CRC16(256-byte payload)][00][ck]
                + 32 raw frames = the 256 payload bytes
verify:         FF [08][10 08][00][CRC16(CF8)][00][ck]
                CF [region start u32][region size u32]
```

Replies: erase → `60 XX` (bit 7 of XX set = content was actually
erased), arm → `51 …`, program → `52 …`, verify → `48 10 08 …` + the
8-byte region checksum.

Mechanics that matter:

- **Priming**: the write path only accepts commands after the priming
  sequence (`tools/priming_seq.py`: the identify reads, the region
  verify ladder, and the attestation constants, in order).
- **Pacing**: ≥4 ms between all session frames.
- **Busy window**: after a content-erasing `60 80…` reply, the FCU is
  busy for ~1.75 s. Wait ~2.5 s before the next command, and retry
  verify replies that don't start with `48` .

## 6. Flash map (this TCU)

| Region | Range |
|---|---|
| Boot area (never touched) | `0x000000–0x003FFF` |
| Code flash (user MAT) | `0x004000–0x27FFFF` (2.5 MB total) |
| Data flash | `0x80100000–0x8011FFFF` (128 KB, 8 KB blocks) |
| FCU program/erase window | `0x80800000 + flash offset` |
