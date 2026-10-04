#!/usr/bin/env python3
"""How clean was a PX1105R flight of a SignalSim file? Satellite lists come from SignalSim's generation log
(the 'X with IF' tables), so any scenario works. Per constellation: satellites measured (0xE5), used in the
fix (0xE7 bit 4 in >= 10 % of epochs), never given a channel, median C/N0; channel continuity (median/min of
each satellite's longest hold, total breaks); common-mode breaks (epochs where >= 4 L1 channels lose C/N0 at
once); first fix after the first 0xDF; HackRF underruns.   sat_summary.py CAPTURE GEN_LOG [T_END_S]"""
import re
import statistics as st
import sys
from pathlib import Path

cap, gen = sys.argv[1], sys.argv[2]
t_end = float(sys.argv[3]) if len(sys.argv) > 3 else None
G = {0: "GPS", 3: "GAL", 5: "BDS"}
NAME = {"GPS L1CA": "GPS", "Galileo E1": "GAL", "BeiDou B1I": "BDS"}
FILE = {}
text = Path(gen).read_text(errors="replace")
for head, sysname in NAME.items():
    if f"{head} with IF" in text:
        block = text.split(f"{head} with IF", 1)[1].split("\n\n", 1)[0]
        FILE[sysname] = sorted({int(m) for m in re.findall(r"\|\s(\d\d)\s\|", block)})

meas, cn0, used, chan, n7, df0, fix = {}, {}, {}, set(), 0, None, None
epochs = []
t0 = None
for line in open(cap, errors="replace"):
    p = line.split(" ", 2)
    if len(p) < 3 or p[1] != "B":
        continue
    try:
        t, x = float(p[0]), bytes.fromhex(p[2].strip())
    except ValueError:
        continue
    if t_end is not None and t0 is not None and t - t0 > t_end:
        break
    if x[0] == 0xE5 and len(x) >= 14:
        for j in range(x[13]):
            r = x[14 + 31 * j: 14 + 31 * (j + 1)]
            if len(r) == 31 and (r[0] & 0x0F) in G and (r[0] >> 4) in (0, 1):
                k = (G[r[0] & 0x0F], r[1])
                meas[k] = meas.get(k, 0) + 1
                cn0.setdefault(k, []).append(r[3])
    elif x[0] == 0xE7 and len(x) >= 4:
        t0 = t if t0 is None else t0
        n7 += 1
        d = {}
        for j in range(x[3]):
            b = x[4 + 7 * j: 4 + 7 * (j + 1)]
            if len(b) == 7 and (b[1] & 0x0F) in G and (b[1] >> 4) in (0, 1):
                k = (G[b[1] & 0x0F], b[2])
                chan.add(k)
                if b[6] & 0x10:
                    used[k] = used.get(k, 0) + 1
                c = int.from_bytes(b[5:6], "big", signed=True)
                if c > 0:
                    d[k] = c
        epochs.append(d)
    elif x[0] == 0xDF and len(x) >= 3:
        df0 = t if df0 is None else df0
        if fix is None and x[2] >= 2:
            fix = t

name = Path(cap).name
print(name)
print(f"  first fix {'never' if fix is None else f'{fix - df0:.0f} s'} after the first 0xDF; {n7} 0xE7 epochs")
for sysname, svs in FILE.items():
    m = [sv for sv in svs if (sysname, sv) in meas]
    u = [sv for sv in svs if used.get((sysname, sv), 0) >= 0.1 * max(n7, 1)]
    nc = [sv for sv in svs if (sysname, sv) not in chan]
    c = [st.median(cn0[(sysname, sv)]) for sv in m]
    holds, breaks = [], 0
    for sv in svs:
        run, best, prev = 0, 0, False
        for d in epochs:
            on = (sysname, sv) in d
            run = run + 1 if on else 0
            best = max(best, run)
            breaks += 1 if (prev and not on) else 0
            prev = on
        if best:
            holds.append(best)
    hold = f"; longest hold median {st.median(holds):.0f} s, min {min(holds)} s; {breaks} breaks" if holds else ""
    print(f"  {sysname}: used {len(u)}/{len(svs)} {u}; measured {len(m)}; never a channel {nc}"
          + (f"; C/N0 {st.median(c):.0f}" if c else "") + hold)
common = sum(1 for a, b in zip(epochs, epochs[1:]) if sum(1 for k in a if k not in b) >= 4)
print(f"  common-mode breaks (>= 4 channels drop in one epoch): {common}")
hk = Path(cap + ".hackrf.txt")
if hk.exists():
    tail = [l for l in hk.read_text(errors="replace").splitlines() if "underrun" in l]
    print("  " + (tail[-1].strip() if tail else "no underrun line"))
