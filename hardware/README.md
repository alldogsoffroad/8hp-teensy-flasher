# Hardware: the Teensy flasher bench

## Bill of materials

| Part | Notes |
|---|---|
| Teensy 4.1 | PJRC; the sketch uses CAN1 + CAN2 |
| 2× SN65HVD230 CAN transceiver | 3.3 V, the common DR2/DR module works |
| 12 V bench supply OR vehicle battery feed | powers the TCU through its connector |
| 120 Ω CAN termination resistors | one per bus end (the TCU side usually has one) |
| Wiring: twisted pairs for CAN H/L | |
| TCU connector breakout | to reach pins 3/4/5/6/9/13/14 |

## TCU connections (ZF 8HP / SH7254R, Ram 1500)

| TCU pin | Function | Connects to |
|---|---|---|
| 5, 6 | CAN H/L — programming bus | CAN1 transceiver (Teensy pins 22/23 side) |
| 3, 4 | CAN H/L — vehicle bus | CAN2 transceiver (Teensy pins 0/1 side), optional |
| 9, 13 | +12 V supply | bench supply / battery (through a switch) |
| 14 | ground | supply ground AND Teensy ground |

Ground the Teensy with the TCU (common ground is required).

## Teensy 4.1 wiring

```
Teensy pin 22 (CTX1) ──► TXD  ┌─────────┐  CANH ──► TCU pin 5
Teensy pin 23 (CRX1) ◄── RXD  │ SN65HVD │
                     3V3 ──► VCC │230    │  CANL ──► TCU pin 6
                     GND ──► GND └─────────┘
                              (RS pin to GND = high-speed mode)

Teensy pin  1 (CTX2) ──► TXD  ┌─────────┐  CANH ──► TCU pin 3
Teensy pin  0 (CRX2) ◄── RXD  │ SN65HVD │
                     3V3 ──► VCC │230    │  CANL ──► TCU pin 4
                     GND ──► GND └─────────┘
```

On Linux the two USB CDC ports enumerate as `/dev/ttyACM0` (CAN1) and
`/dev/ttyACM1` (CAN2). The tools default to those names.

## The power switch matters

The TCU's programming session only opens in the first milliseconds after
a true power-on reset, and its rails hold charge for a long time after
12 V is removed. The reliable sequence is:

1. Start `arm_storm.py` (the entry frame must already be on the bus).
2. Cut power. **Wait 30 seconds** — short outages do not fully discharge
   the TCU and the window will not open.
3. Apply power. The storm reports `TCU FRAME` within a second when the
   session opens.

A bench setup with a physical switch on the 12 V feed (or a relay driven
by a GPIO) makes this painless.
