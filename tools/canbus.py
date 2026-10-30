#!/usr/bin/env python3
"""canbus.py - minimal self-contained CAN support for the TCU flasher.

Wraps a Teensy 4.1 running firmware/teensy_slcan.ino (dual-channel Lawicel
SLCAN at 2 Mbaud over USB) into a small Python API:

  - Buses: two python-can slcan channels (TCU bus on CAN1, secondary bus
    on CAN2), with the Teensy brought to normal mode first (P0).
  - Uds: a minimal ISO-TP + UDS request client for the plant session
    (0x7E1 -> 0x7E9), with 0x78-pending handling.
  - teensy_passive(): raw SLCAN command helper (rate/mode switches).

Requires: python-can (pip install python-can), pyserial.
Part of 8hp-teensy-flasher. License: GPLv3 (see LICENSE).
"""
import threading
import time

import can
import serial

try:
    import can.interfaces.slcan as _slcan  # noqa: F401  (presence check)
except ImportError as e:
    raise SystemExit("python-can with slcan support is required: pip install python-can")


class BusArgs:
    """Channel names for the two Teensy USB serial ports."""
    iface = "slcan"
    bus1 = "/dev/ttyACM0"      # CAN1: TCU bus (pins 22/23)
    bus2 = "/dev/ttyACM1"      # CAN2: secondary bus (pins 0/1)
    teensy_mode = None
    keep_rate = True
    bitrate = 500000


def teensy_passive(ports, mode=b"P0"):
    """Send a raw SLCAN command to each Teensy port (C, P0, S6, O, ...)."""
    for port in ports:
        if not port:
            continue
        s = serial.Serial(port, 2000000, timeout=0.3)
        s.write(b"C\r" + mode + b"\r")
        time.sleep(0.05)
        s.read(16)
        s.close()


class Buses:
    """Open both Teensy CAN channels. keep_rate=True never touches the
    bit rate so a previously set S-command stays effective."""

    def __init__(self, args=None):
        args = args or BusArgs()
        self.names = {1: args.bus1, 2: args.bus2}
        self.bus = {}
        self._lock = threading.Lock()
        teensy_passive([args.bus1, args.bus2], b"P0")
        for n, chan in self.names.items():
            if not chan:
                continue
            kw = {"interface": args.iface, "channel": chan}
            if args.iface == "slcan" and not getattr(args, "keep_rate", False):
                kw["bitrate"] = 500000
            self.bus[n] = can.Bus(**kw)

    def get(self, n):
        return self.bus[n]

    def send(self, n, arb_id, data):
        data = bytes(data)
        with self._lock:
            self.get(n).send(can.Message(arbitration_id=arb_id, data=data,
                                         is_extended_id=arb_id > 0x7FF))

    def shutdown(self):
        for b in self.bus.values():
            try:
                b.shutdown()
            except Exception:
                pass
        self.bus = {}


class Uds:
    """Minimal ISO-TP / UDS client for the plant session (666.667 kbit/s).

    Single- and multi-frame requests, flow-control handling, and the
    0x78 (responsePending) retry. cf_gap_s paces consecutive frames —
    keep it >= 4 ms; the TCU-side ISO-TP drops back-to-back frames.
    """

    REQ, RSP = 0x7E1, 0x7E9

    def __init__(self, buses, n=1):
        self.b, self.n = buses, n
        self.req, self.rsp = Uds.REQ, Uds.RSP
        self.cf_gap_s = 0.004
        self.honor_stmin = False
        self.lock = threading.Lock()

    def _wait_fc(self, bus, timeout=1.0):
        end = time.time() + timeout
        while time.time() < end:
            m = bus.recv(0.05)
            if m is not None and m.arbitration_id == self.rsp and m.data[0] >> 4 == 3:
                if m.data[0] & 0x0F == 0:
                    return m.data[1], m.data[2]
                if m.data[0] & 0x0F == 2:
                    return None
                end = time.time() + timeout          # wait
        return None

    def _send(self, bus, payload):
        if len(payload) <= 7:
            self.b.send(self.n, self.req,
                        bytes([len(payload)]) + payload + b"\xAA" * (7 - len(payload)))
            return True
        self.b.send(self.n, self.req,
                    bytes([0x10 | len(payload) >> 8, len(payload) & 0xFF]) + payload[:6])
        fc = self._wait_fc(bus)
        if fc is None:
            return False
        off, sn = 6, 1
        stmin = (fc[1] / 1000.0) if (fc[1] < 0xF1 and self.honor_stmin) else self.cf_gap_s
        while off < len(payload):
            chunk = payload[off:off + 7]
            self.b.send(self.n, self.req,
                        bytes([0x20 | (sn & 0x0F)]) + chunk + b"\xAA" * (7 - len(chunk)))
            off, sn = off + 7, sn + 1
            time.sleep(max(stmin, self.cf_gap_s))
        return True

    def request(self, payload, timeout=2.0, cap=90.0):
        payload = bytes(payload)
        with self.lock:
            bus = self.b.get(self.n)
            while bus.recv(0) is not None:
                pass
            if not self._send(bus, payload):
                return None
            hard = time.time() + cap
            end = min(time.time() + timeout, hard)
            data, need = b"", None
            while time.time() < end:
                m = bus.recv(0.05)
                if m is None or m.arbitration_id != self.rsp:
                    continue
                pci = m.data[0] >> 4
                if pci == 0:
                    resp = bytes(m.data[1:1 + (m.data[0] & 0x0F)])
                    if len(resp) >= 3 and resp[0] == 0x7F and resp[2] == 0x78:
                        end = min(time.time() + 5.0, hard)
                        continue
                    return resp
                if pci == 1:
                    need = ((m.data[0] & 0x0F) << 8) | m.data[1]
                    data = bytes(m.data[2:8])
                    self.b.send(self.n, self.req, b"\x30\x00\x00" + b"\xAA" * 5)
                elif pci == 2 and need is not None:
                    data += bytes(m.data[1:8])
                    if len(data) >= need:
                        return data[:need]
            return None
