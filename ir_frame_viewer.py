#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial>=3.5"]
# ///
"""Show the details of IR frames received by the ir-receiver AVR.

Reads the new-press lines the firmware prints over serial, appends each frame
to a CSV log (recording the raw value as sent), and shows for each frame:
the raw bytes, the fields with separators, what each field means, and the
parity/inverse checks.

Run:
    ./ir_frame_viewer.py
    ./ir_frame_viewer.py --port /dev/ttyACM1 --log session1.csv
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import re
import sys
import time
from collections import deque
from dataclasses import dataclass, field as dc_field
from pathlib import Path

import serial

# Lines printed by ir-receiver.c for a new press, e.g.
#   NEC addr=0x00 cmd=0x46 raw=0x00FF629D
#   PANASONIC addr=0x00B cmd=0x16 raw=0x0220B00016A6
#   KASEIKYO vendor=0x0003 addr=0x... cmd=0x.. raw=...
#   RC5 addr=0x05 cmd=0x0C toggle=0 raw=0x314C
# Repeat lines are indented (REPEAT=1) and are skipped before this match.
FRAME_RE = re.compile(
    r"^(NEC|PANASONIC|KASEIKYO|RC5) (?:addr|vendor)=\S+ .*?raw=0x([0-9A-Fa-f]+)\s*$"
)

PROTOCOL_INFO = {
    "NEC": "32 bits, 38 kHz",
    "PANASONIC": "48 bits, ~37 kHz",
    "KASEIKYO": "48 bits, ~37 kHz",
    "RC5": "14 bits, 36 kHz",
}

SEPARATOR = "|"


@dataclass
class Field:
    label: str
    hex: str
    bits: str
    meaning: str


@dataclass
class Check:
    name: str
    ok: bool


@dataclass
class Frame:
    when: dt.datetime
    protocol: str
    raw: str                 # hex digits exactly as sent by the AVR
    line: str                # full line as sent
    fields: list[Field] = dc_field(default_factory=list)
    checks: list[Check] = dc_field(default_factory=list)
    error: str = ""


def raw_bytes(raw: str, count: int) -> list[int]:
    if len(raw) != 2 * count:
        raise ValueError(f"expected {count} bytes, got {len(raw) // 2}")
    return [int(raw[i:i + 2], 16) for i in range(0, len(raw), 2)]


def field(label: str, value: int, width: int, meaning: str) -> Field:
    """A field of `width` bits, shown in hex and binary."""
    digits = (width + 3) // 4
    return Field(label, f"{value:0{digits}X}", f"{value:0{width}b}", meaning)


def kaseikyo_vendor_parity(vendor_lo: int, vendor_hi: int) -> int:
    """4-bit parity of the vendor code, as defined by Kaseikyo."""
    p = vendor_lo ^ vendor_hi
    return (p ^ (p >> 4)) & 0x0F


def analyse(protocol: str, raw: str) -> tuple[list[Field], list[Check]]:
    if protocol == "NEC":
        b = raw_bytes(raw, 4)            # address, ~address, command, ~command
        checks = [Check("command inverse", (b[2] ^ b[3]) == 0xFF)]
        if (b[0] ^ b[1]) == 0xFF:        # 8-bit address with inverse
            checks.insert(0, Check("address inverse", True))
            fields = [
                field("address", b[0], 8, "device address"),
                field("~address", b[1], 8, "inverse of address (error check)"),
                field("command", b[2], 8, "key code"),
                field("~command", b[3], 8, "inverse of key code (error check)"),
            ]
        else:                             # extended 16-bit address
            fields = [
                field("address", b[0] | b[1] << 8, 16, "16-bit device address"),
                field("command", b[2], 8, "key code"),
                field("~command", b[3], 8, "inverse of key code (error check)"),
            ]
        return fields, checks

    if protocol in ("PANASONIC", "KASEIKYO"):
        b = raw_bytes(raw, 6)
        checks = [
            Check("vendor parity", kaseikyo_vendor_parity(b[0], b[1]) == (b[2] & 0x0F)),
            Check("data parity", b[5] == (b[2] ^ b[3] ^ b[4])),
        ]
        fields = [
            field("vendor", b[0] | b[1] << 8, 16, "manufacturer code (sent LSB first)"),
            field("vparity", b[2] & 0x0F, 4, "parity of the manufacturer code"),
            field("address", (b[2] >> 4) | b[3] << 4, 12, "device address"),
            field("command", b[4], 8, "key code"),
            field("dparity", b[5], 8, "parity of address and command"),
        ]
        return fields, checks

    if protocol == "RC5":
        v = int(raw, 16)
        if v >> 14:
            raise ValueError("RC5 frame is 14 bits")
        s1 = (v >> 13) & 1
        s2 = (v >> 12) & 1
        fields = [
            field("S1", s1, 1, "start bit (always 1)"),
            field("S2", s2, 1, "field bit: inverse of command bit 6"),
            field("toggle", (v >> 11) & 1, 1, "flips on each new key press"),
            field("address", (v >> 6) & 0x1F, 5, "device address"),
            field("command", v & 0x3F, 6, "key code (bits 0-5)"),
        ]
        return fields, [Check("start bit", s1 == 1)]

    raise ValueError(f"unknown protocol {protocol}")


def make_frame(line: str, when: dt.datetime) -> Frame | None:
    m = FRAME_RE.match(line)
    if not m:
        return None
    protocol, raw = m.group(1), m.group(2).upper()
    frame = Frame(when, protocol, raw, line)
    try:
        frame.fields, frame.checks = analyse(protocol, raw)
    except ValueError as e:
        frame.error = str(e)
    return frame


# ------------------------------------------------------------- drawing

def center_table(rows: list[tuple[str, list[str]]]) -> list[str]:
    """Rows of cells, columns separated by SEPARATOR, each cell centred."""
    ncol = len(rows[0][1])
    widths = [max(len(cells[i]) for _, cells in rows) + 2 for i in range(ncol)]
    label_w = max(len(name) for name, _ in rows)
    out = []
    for name, cells in rows:
        body = SEPARATOR.join(c.center(w) for c, w in zip(cells, widths))
        out.append(f"{name.ljust(label_w)}  {body}")
    return out


def protocol_title(frame: Frame) -> str:
    if frame.protocol == "PANASONIC":
        name = "PANASONIC (Kaseikyo, vendor 0x2002)"
    else:
        name = frame.protocol
    return f"{name}  ({PROTOCOL_INFO.get(frame.protocol, '')})"


def render_frame(frame: Frame) -> list[str]:
    out = [protocol_title(frame),
           f"Received {frame.when:%H:%M:%S}    raw value 0x{frame.raw}",
           ""]
    if frame.error:
        return out + [f"Could not split this frame: {frame.error}"]

    # Raw value split into bytes. RC-5 is 14 bits, so its two bytes
    # include two leading zero bits.
    bytes_row = [frame.raw[i:i + 2] for i in range(0, len(frame.raw), 2)]
    out += center_table([("Bytes", bytes_row)])
    out.append("")
    out += center_table([
        ("Field", [f.label for f in frame.fields]),
        ("Hex", [f.hex for f in frame.fields]),
        ("Binary", [f.bits for f in frame.fields]),
    ])

    width = max(len(f.label) for f in frame.fields)
    out += ["", "Details:"]
    for f in frame.fields:
        out.append(f"  {f.label.ljust(width)}  0x{f.hex:<5} {f.meaning}")

    if frame.checks:
        out += ["", "Checks:"]
        width = max(len(c.name) for c in frame.checks)
        for c in frame.checks:
            out.append(f"  {c.name.ljust(width)}  {'OK' if c.ok else 'FAILED'}")
    return out


def render(frame: Frame | None, history: list[Frame], status: str) -> str:
    out: list[str] = []
    if frame is None:
        out += ["Waiting for a remote...", ""]
    else:
        out += render_frame(frame)
        out.append("")

    if len(history) > 1:
        out.append("Previous:")
        for f in history[1:]:
            out.append(f"  {f.when:%H:%M:%S}  {f.protocol:<10} 0x{f.raw}")
        out.append("")
    out.append(f"[{status}]")
    return "\n".join(out)


def redraw(text: str) -> None:
    sys.stdout.write("\033[2J\033[H" + text + "\n")
    sys.stdout.flush()


# ------------------------------------------------------------ main loop

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default="/dev/ttyACM0", help="serial port of the Mega")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--log", default=None,
                    help="CSV log file (default: ir-frames-<time>.csv)")
    args = ap.parse_args()

    log_path = Path(args.log or f"ir-frames-{dt.datetime.now():%Y%m%d-%H%M%S}.csv")
    new_log = not log_path.exists() or log_path.stat().st_size == 0
    logf = log_path.open("a", newline="", encoding="utf-8")
    writer = csv.writer(logf)
    if new_log:
        writer.writerow(["timestamp", "protocol", "raw", "line"])
        logf.flush()

    frame: Frame | None = None
    history: deque[Frame] = deque(maxlen=6)
    status = f"{args.port} @ {args.baud}  |  log: {log_path}"
    ser: serial.Serial | None = None
    redraw(render(None, [], f"opening {args.port}"))

    try:
        while True:
            if ser is None:
                try:
                    ser = serial.Serial(args.port, args.baud, timeout=0.5)
                    status = f"{args.port} @ {args.baud}  |  log: {log_path}"
                except serial.SerialException as e:
                    redraw(render(frame, list(history), f"waiting for {args.port}: {e}"))
                    time.sleep(2)
                    continue

            try:
                data = ser.readline()
            except serial.SerialException:
                ser.close()
                ser = None
                continue
            if not data:                       # read timeout, keep waiting
                continue

            line = data.decode("ascii", errors="replace").rstrip("\r\n")
            # Indented lines are repeats (REPEAT=1); never log them as new presses.
            if not line or line[0].isspace():
                continue

            new = make_frame(line, dt.datetime.now())
            if new is None:
                if line.startswith("ir-receiver") or line.startswith("IR input"):
                    status = line
                    redraw(render(frame, list(history), status))
                continue

            writer.writerow([new.when.isoformat(timespec="seconds"),
                             new.protocol, new.raw, new.line])
            logf.flush()
            frame = new
            history.appendleft(new)
            redraw(render(frame, list(history), status))
    except KeyboardInterrupt:
        pass
    finally:
        if ser is not None:
            ser.close()
        logf.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
