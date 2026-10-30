# 8hp-teensy-flasher

An open-source, Teensy 4.1-based code-flash reader/writer for the ZF 8HP
transmission control unit (Renesas SH7254R) used in the Ram 1500.

No dealer tools, no subscriptions, no black boxes: a ~$30 Teensy, two CAN
transceivers, and this software read and write your own TCU's code flash.

```
Teensy 4.1 ──CAN1──► TCU pins 5/6 (programming bus, 666.667 & 500 kbit/s)
           ──CAN2──► TCU pins 3/4 (vehicle bus, optional)
```

## What it does

- **Wake** the TCU's factory programming session during power-up
- **Authenticate** to it — the plant SecurityAccess is a RIPEMD-160 digest
  satisfied by an integer cube root (no secret key exists; see
  [docs/protocol.md](docs/protocol.md))
- **Load the session engine** into TCU RAM over the programming session
- **Open the encrypted 500 kbit/s session** (the cipher and key
  derivation are fully documented)
- **Erase, program, and verify** code-flash blocks — composed, CRC-checked
  command frames, no replays

Everything is documented: the wake window, the seed/key construction, the
session cipher, the command framing, and the flash-op grammar.

## Repository layout

| Path | Contents |
|---|---|
| `hardware/` | Bill of materials, wiring, bench rig |
| `firmware/` | The Teensy sketch (dual-channel Lawicel SLCAN) |
| `tools/` | The Python flasher suite |
| `docs/protocol.md` | Full reverse-engineered protocol documentation |

## Quick start

1. **Build the hardware** ([hardware/README.md](hardware/README.md)) —
   Teensy 4.1 + 2× SN65HVD230 transceivers, wired to the TCU connector.
2. **Flash the firmware** ([firmware/README.md](firmware/README.md)) —
   Arduino IDE with Teensyduino, USB type "Dual Serial".
3. **Capture your session engine** once
   ([tools/capture_engine.py](tools/capture_engine.py)) — the engine is
   vendor code and is not distributed here; read it from your own unit.
4. **Flash**:
   ```sh
   # terminal 1: arm the wake storm, then power the TCU (OFF 30 s, ON)
   python3 tools/arm_storm.py
   # terminal 2 (after the storm reports the session open):
   python3 tools/flasher.py --engine engine.bin status
   python3 tools/flasher.py --engine engine.bin write 0xE1000 cafe1234...
   ```

## Safety and scope

- This tool **erases and rewrites real flash**. A wrong write can brick
  your TCU. Read and back up before writing; the tool refuses addresses
  outside code flash (0x4000–0x27E000) and never touches the boot region.
- The wake window only opens during power-up — the bench rig in
  `hardware/` includes a power switch, and the 30-second off period
  exists because the TCU's rails must fully discharge.
- All frame pacing matters: the flasher enforces ≥4 ms inter-frame gaps
  (back-to-back frames are silently dropped).

## Legal

This project is reverse-engineering documentation and interoperability
tooling for the owner's own vehicle hardware. It contains no vendor code
or proprietary binaries. The session engine image is read from the user's
own unit and never redistributed.

Licensed under the **GNU GPL v3** — see [LICENSE](LICENSE).

## Acknowledgements

Protocol findings built on public SH7254R documentation and the author's
own bench captures. Inspired by the wider community of open ECU tools.

This project was produced by a human/AI pair: see
[CONTRIBUTORS.md](CONTRIBUTORS.md).
