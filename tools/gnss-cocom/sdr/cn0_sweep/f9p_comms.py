#!/usr/bin/env python3
"""A ZED-F9P capture's delivery, in slices of capture time: RXM-RAWX epochs and their measurement count, NAV-PVT,
NAV-SAT, RXM-SFRBX per second, and from UBX-MON-COMMS (if it was on) each port's transmit buffer: pending bytes,
usage and peak usage (%), skipped bytes, and the txErrors flags (bit 0 memory, bit 1 buffer full). Then the share of
RAWX epochs delivered over the pad up to the boost (T-170 .. T+2; 10 Hz expected).
    f9p_comms.py CAPTURE [SLICE_S]"""
import struct
import sys
from collections import defaultdict

PORT = {0x0000: "I2C", 0x0100: "UART1", 0x0201: "UART2", 0x0300: "USB", 0x0400: "SPI"}
IGN = 204000.0
path = sys.argv[1]
width = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
sl = defaultdict(lambda: defaultdict(int))
comms = defaultdict(dict)
raw_tow, ports_seen = [], set()
for line in open(path, errors="replace"):
    p = line.split(" ", 2)
    if len(p) < 3 or p[1] != "U":
        continue
    try:
        h, b = float(p[0]), bytes.fromhex(p[2].strip()[4:])
    except ValueError:
        continue
    k = p[2][:4]
    s = sl[int(h // width)]
    if k == "0215" and len(b) >= 16:
        s["raw"] += 1
        s["meas"] += b[11]
        raw_tow.append(struct.unpack_from("<d", b, 0)[0])
    elif k == "0107":
        s["pvt"] += 1
    elif k == "0135":
        s["sat"] += 1
    elif k == "0213":
        s["sfr"] += 1
    elif k == "0a36" and len(b) >= 8:
        s["comms"] += 1
        s["err"] |= b[2]
        for j in range(b[1]):
            g = b[8 + 40 * j: 48 + 40 * j]
            if len(g) < 40:
                break
            pid, pend = struct.unpack_from("<HH", g, 0)
            use, peak = g[8], g[9]
            skipped = struct.unpack_from("<I", g, 36)[0]
            name = PORT.get(pid, hex(pid))
            ports_seen.add(name)
            c = comms[int(h // width)].setdefault(name, [0, 0, 0, 0])
            c[0], c[1], c[2] = max(c[0], pend), max(c[1], use), max(c[2], peak)
            c[3] = max(c[3], skipped)
order = [n for n in ("USB", "UART1", "UART2", "I2C", "SPI") if n in ports_seen]
hdr = f"{'capture s':>10} {'RAWX/s':>7} {'meas':>5} {'PVT/s':>6} {'SAT/s':>6} {'SFRBX/s':>8}"
if order:
    hdr += " | " + " | ".join(f"{n}: pend use peak skip" for n in order) + " | err"
print(hdr)
for i in sorted(sl):
    s = sl[i]
    row = (f"{i * width:>5.0f}-{(i + 1) * width:<4.0f} {s['raw'] / width:>7.1f} "
           f"{(s['meas'] / s['raw'] if s['raw'] else 0):>5.1f} {s['pvt'] / width:>6.1f} {s['sat'] / width:>6.1f} "
           f"{s['sfr'] / width:>8.1f}")
    if order:
        cells = []
        for n in order:
            c = comms.get(i, {}).get(n)
            cells.append(f"{c[0]:>5} {c[1]:>3}% {c[2]:>3}% {c[3]:>6}" if c else f"{'-':>5} {'-':>4} {'-':>4} {'-':>6}")
        row += " | " + " | ".join(cells) + f" | {s['err']:#04x}"
    print(row)
pad = [t for t in raw_tow if IGN - 170.0 <= t <= IGN + 2.0]
print(f"RAWX delivered T-170 .. T+2: {len(pad)} of 1721 epochs ({100.0 * len(pad) / 1721:.0f} %)")
# Starvation, independent of how long the cold start takes to find GPS time: RAWX frames against NAV-PVT frames
# (which always arrive) over capture seconds 20..170 (the pad, before any speed window).
n_raw = sum(sl[i]["raw"] for i in sl if 20 <= i * width and (i + 1) * width <= 170)
n_pvt = sum(sl[i]["pvt"] for i in sl if 20 <= i * width and (i + 1) * width <= 170)
print(f"RAWX per NAV-PVT, capture 20..170 s: {n_raw} / {n_pvt} ({100.0 * n_raw / max(n_pvt, 1):.0f} %)")
