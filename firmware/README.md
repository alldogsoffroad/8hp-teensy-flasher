# Firmware: Teensy dual-channel SLCAN

A 491-line Arduino sketch (`teensy_slcan.ino`) that turns the Teensy 4.1
into a dual-channel Lawicel-compatible SLCAN adapter:

- USB type **Dual Serial** — the first port serves CAN1 (TCU programming
  bus), the second CAN2 (vehicle bus)
- Bit rates S0–S8 plus **S9** (666.667 kbit/s — the TCU's boot rate)
- Normal (`O`) and listen-only (`L`) modes, per-channel mode commands
  (P0/P1/P2)
- Standard/extended frame send (`t`/`T`), receive streaming, remote
  frames (`r`/`R`)
- Bus-error and error-counter reporting (`E`), TX-acknowledge (`z`/`Z`)
- Power-feed monitor on the first port (`A1`): annotates the frame
  stream with supply-voltage samples — essential for correlating the
  power-up window with bus activity
- Saturation/pulsed entry-frame modes (`h`/`H`/`G`) for experimenting
  with the wake window

## Building and flashing

1. Install the Arduino IDE and Teensyduino.
2. Open `teensy_slcan.ino`.
3. Tools → USB Type → **Dual Serial**.
4. Upload.

On Linux the ports appear as `/dev/ttyACM0` and `/dev/ttyACM1` and are
used directly by the Python tools (no slcand needed; python-can's slcan
backend handles them).

## Command summary

| Command | Meaning |
|---|---|
| `S0`..`S8` | bit rate 10k..1M |
| `S9` | 666.667 kbit/s (TCU boot rate) |
| `O` / `L` | open bus / open listen-only |
| `C` | close |
| `P0` / `P1` / `P2` | normal / listen-only / listen-only+auto-baud |
| `t<id><dlc><data>` | send standard frame |
| `T<id><dlc><data>` | send extended frame |
| `A1` / `A0` | power-feed monitor on/off (port 1) |
| `z` / `Z` | TX acknowledge for standard / extended |
