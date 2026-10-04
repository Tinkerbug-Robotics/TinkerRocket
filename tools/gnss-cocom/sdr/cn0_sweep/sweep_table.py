#!/usr/bin/env python3
"""One table across the C/N0 sweep runs, from each run's analysis output (post .out), accuracy npz and the boost
chart's JSON:  pad C/N0 as read, satellites listed at ignition, boost lock / raw output held to burnout, last fix
after ignition, fix back, the fix through the descent, the fewest satellites in the re-entry (T+255-300), descent
accuracy, raw-measurement gaps.

    sweep_table.py BOOST.json LABEL=POST.out=ACC.npz [...] [--json OUT.json]
"""
import json
import re
import sys

import numpy as np

argv = sys.argv[1:]
json_out = None
if "--json" in argv:
    i = argv.index("--json")
    json_out = argv[i + 1]
    del argv[i:i + 2]
boost = {r["label"]: r for r in json.load(open(argv[0]))}
AB = {"G": "GPS", "C": "BDS", "E": "GAL"}


def section(text, name):
    m = re.search(r"== " + re.escape(name) + r"\n(.*?)(?=\n== |\Z)", text, re.S)
    return m.group(1) if m else ""


rows = []
for arg in argv[1:]:
    label, post, npz = arg.split("=")
    txt = open(post, errors="replace").read()
    z = np.load(npz)
    pad = {}
    for i, c in enumerate("GEC"):
        m = (z["sys"] == i) & (z["t"] >= -60) & (z["t"] <= -5)
        if m.any():
            pad[c] = float(np.median(z["cn0"][m]))
    ts = section(txt, "traveler summary")
    ign = re.search(r"listed at ignition\s*:\s*(.*)", ts)
    last = re.search(r"last fix after ignition T\+([\d.]+): own speed (\d+) m/s, true (\d+)", ts)
    back = re.search(r"fix back at T\+([\d.]+)", ts)
    fr = section(txt, "fix runs")
    runs = [(int(a), float(b), float(c)) for a, b, c in
            re.findall(r"mode (\d): T\+\s*(-?[\d.]+) \.\. T\+\s*(-?[\d.]+)", fr)]
    desc = [r for r in runs if r[0] >= 2 and r[2] > 270]
    held = f"T+{desc[0][1]:.1f}-{desc[-1][2]:.1f}" + (f" ({len(desc)} pieces)" if len(desc) > 1 else "") if desc else "-"
    re_tab = section(txt, "re-entry and descent, second by second")
    mins = {"GPS": 99, "GAL": 99, "BDS": 99}
    for g, e, b in re.findall(r"^\s*\+(?:2[5-9]\d|300)\.\d\s+(\d+)\s+(\d+)\s+(\d+)", re_tab, re.M):
        mins["GPS"], mins["GAL"], mins["BDS"] = (min(mins["GPS"], int(g)), min(mins["GAL"], int(e)),
                                                min(mins["BDS"], int(b)))
    acc = section(txt, "accuracy")
    lines = acc.splitlines()
    prd, rrd = {}, {}
    for k, ln in enumerate(lines):
        if ln.startswith("descent, fix allowed"):
            block = [ln] + [x for x in lines[k + 1:k + 3] if x.startswith(" ")]
            dest = rrd if any("range-rate" in y for y in lines[:k]) else prd
            for b in block:
                m = re.search(r"(GPS|Galileo|BeiDou)\s+(\d+)\s+([\d.]+)", b)
                if m:
                    dest[m.group(1)] = float(m.group(3))
    gaps = section(txt, "gaps and underruns")
    ngap = len(re.findall(r"no raw measurements", gaps))
    nund = len(re.findall(r"underrun in replay second", gaps))
    b = boost.get(label)
    lock, raw = {}, {}
    if b:
        for c in "GCE":
            tot = [q for q in b["recs"] if q["sys"] == c]
            if tot:
                lock[c] = f"{sum(q['lock_end'] for q in tot)}/{len(tot)}"
                raw[c] = f"{sum(q['raw_end'] for q in tot)}/{len(tot)}"
    rows.append(dict(
        label=label,
        pad=", ".join(f"{AB[c]} {pad[c]:.0f}" for c in "GCE" if c in pad),
        ign=ign.group(1).strip() if ign else "-",
        lock=", ".join(f"{AB[c]} {v}" for c, v in lock.items()),
        raw=", ".join(f"{AB[c]} {v}" for c, v in raw.items()),
        last=f"T+{last.group(1)} ({last.group(2)}/{last.group(3)} m/s)" if last else "-",
        back=f"T+{back.group(1)}" if back else "none",
        held=held,
        mins=", ".join(f"{k} {v if v < 99 else 0}" for k, v in mins.items()),
        prd=", ".join(f"{k} {v:.1f}" for k, v in prd.items()),
        rrd=", ".join(f"{k} {v:.2f}" for k, v in rrd.items()),
        gaps=f"{ngap} gaps, {nund} underruns"))
keys = [("pad C/N0 as read, dB-Hz", "pad"), ("listed at ignition", "ign"), ("locked at burnout", "lock"),
        ("delivering raw at burnout", "raw"), ("last fix after ignition (own/true)", "last"),
        ("fix back", "back"), ("fix held (descent)", "held"), ("fewest in re-entry T+255-300", "mins"),
        ("descent pseudorange RMS, m", "prd"), ("descent Doppler RMS, m/s", "rrd"), ("raw gaps / underruns", "gaps")]
w = max(len(k) for k, _ in keys)
print(f"{'':<{w}}  " + "  |  ".join(r["label"] for r in rows))
for name, k in keys:
    print(f"{name:<{w}}  " + "  |  ".join(r[k] for r in rows))
if json_out:
    json.dump(dict(keys=keys, rows=rows), open(json_out, "w"), indent=1)
