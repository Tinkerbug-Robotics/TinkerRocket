#!/usr/bin/env python3
"""PX1105R runs side by side on one axis: seconds after ignition, whatever pad the
IQ file had. One row per capture: raw measurements per 0xE5 epoch (blue), the
receiver's own fix (green band), any mid-flight 0x01 restart (dashed), and the
injected limits (amber > 500 m/s, violet > 80 km). Prints the headline table too.

Everything a row needs is read from its capture: the IQ file's pad from the header's
C8 name (spaceshot_padNNN -> ignition at NNN s; no pad in the name -> the scenario's
prologue), the elevation mask from the header px1105r_run.py writes, the restart
from its "# host: X-START 0x01" line, and a _runN tag from the file name.

    ./px_restart_compare.py OUT.png captures/px1105r_*_el3_run1.log captures/px1105r_*_el3_hot612.log

Columns: satellites with C/N0 and with ephemeris 10..1 s before ignition (0xE7 --
lock, not measurements); "raw back" = 4+ measurements every 0xE5 epoch for 5 s
after burnout; first fix and fix epochs from 0xDF (state >= 2) after burnout.
"""

import json
import re
import statistics as stt
import struct
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                      # noqa: E402

HERE = Path(__file__).resolve().parent
TOW0 = 203400.0
INK, INK3, RULE = "#1d2129", "#6b7280", "#e3e6ea"
DOT, FIX = "#2a78d6", "#1baf7a"


def load(cap: Path, prologue: float):
    e5, df, e7, ev, offs = [], [], [], [], []
    header = ""
    for line in open(cap, errors="replace"):
        p = line.split(" ", 2)
        if len(p) < 3:
            continue
        if p[1] == "#":
            if p[2].startswith("host: tx "):
                header = p[2]
            elif "-START 0x01" in p[2]:
                ev.append((float(p[0]), p[2].split("host: ")[-1].split(" sent")[0]))
            continue
        if p[1] != "B":
            continue
        x = bytes.fromhex(p[2].strip())
        h = float(p[0])
        if x[0] == 0xE5 and len(x) >= 14:
            ft = struct.unpack(">I", x[5:9])[0] / 1000.0 - TOW0
            offs.append(h - ft)
            e5.append((ft, x[13]))
        elif x[0] == 0xDF and len(x) >= 3:
            df.append((h, x[2]))
        elif x[0] == 0xE7 and len(x) >= 4:
            e7.append((h, x))
    if not offs:
        return None                                   # still on the pad: no raw epochs yet
    m = re.search(r"_pad(\d+)", header)
    ign = float(m.group(1)) if m else prologue
    mm = re.search(r"elev mask (\d+) deg", header)
    mask = f"{mm.group(1)} deg mask" if mm else "mask not recorded"
    pw = re.search(r"power (normal|save)", header)
    power = f"power {pw.group(1)}" if pw else "factory power save"     # before --power-mode: never set, and SkyTraq ships in save
    iq = re.search(r"tx (\S+)\.C8", header)
    build = ("stock IQ" if "stock" in iq.group(1) else "smooth IQ" if "smooth" in iq.group(1) else iq.group(1)) if iq else "IQ ?"
    off = stt.median(offs)
    e5 = [(ft - ign, n) for ft, n in e5]
    fixes = sorted(h - off - ign for h, st in df if st >= 2)
    bands, s0, prev = [], None, None
    for t in fixes:
        if prev is None or t - prev > 0.5:
            if s0 is not None:
                bands.append((s0, prev))
            s0 = t
        prev = t
    if s0 is not None:
        bands.append((s0, prev))
    events = [(h - off - ign, nm) for h, nm in ev]
    kind = events[0][1].split("-START")[0].lower() + f" start at {events[0][0]:+.1f} s" if events else "no restart"
    rn = re.search(r"_run(\d+)\.log$", cap.name)
    label = f"{cap.name.split('_')[0].upper()}: {kind}" + (f", run {rn.group(1)}" if rn else "")
    return dict(e5=e5, fixes=fixes, bands=bands, events=events, label=label,
                note=f"{build}, {mask}, {power}", setup=(build, mask, power), e7=e7, off=off, ign=ign)


def summarise(d, burn, end):
    post = [(t, n) for t, n in d["e5"] if t > burn]
    held, r0, pv = None, None, None
    for t, n in post:
        if n >= 4 and (pv is None or t - pv < 0.5):
            r0 = t if r0 is None else r0
            if t - r0 >= 5.0:
                held = r0
                break
        else:
            r0 = t if n >= 4 else None
        pv = t
    seen, eph = set(), set()
    for h, x in d["e7"]:
        if not -10.0 <= h - d["off"] - d["ign"] <= -1.0:
            continue
        for i in range(x[3]):
            r = x[4 + 7 * i: 11 + 7 * i]
            if len(r) < 7 or (r[1] & 0x0F) != 0 or struct.unpack(">b", r[5:6])[0] <= 0:
                continue
            seen.add(r[2])
            if (r[6] & 8) or (r[3] & 2):
                eph.add(r[2])
    d.update(held=held, fix1=next((t for t in d["fixes"] if t > burn), None),
             nfix=sum(1 for t in d["fixes"] if burn < t <= end), seen=len(seen), eph=len(eph))


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    out, caps = sys.argv[1], [Path(c) for c in sys.argv[2:]]
    sc = json.loads((HERE / "scenarios" / "spaceshot.json").read_text())
    tr, prologue = sc["truth"], sc["prologue_s"]
    burn = next(b["t"] for a, b in zip(tr, tr[1:]) if a.get("phase") == "boost" and b.get("phase") != "boost") - prologue
    end = 280.0

    def spans(key, thr):
        cs = []
        for a, b in zip(tr, tr[1:]):
            if (a[key] - thr) * (b[key] - thr) < 0:
                cs.append((a["t"] - prologue + (thr - a[key]) / (b[key] - a[key]) * (b["t"] - a["t"]), b[key] > a[key]))
        ups, downs = [t for t, up in cs if up], [t for t, up in cs if not up]
        return [(t0, next((d for d in downs if d > t0), 1e4)) for t0 in ups]

    rows = []
    for c in caps:
        d = load(c, prologue)
        if d is None:
            print(f"  (skipped {c.name}: no raw epochs yet)")
            continue
        summarise(d, burn, end)
        rows.append(d)
    fmt = lambda v: "   -  " if v is None else f"{v:6.1f}"
    print(f"{'run':<32}{'setup':<46}{'sats':>5}{'eph':>5}{'raw back':>10}{'1st fix':>9}{'fix epochs':>12}"
          f"   (s after ignition; burnout {burn:.1f})")
    for d in rows:
        print(f"{d['label']:<32}{d['note']:<46}{d['seen']:>5}{d['eph']:>5}{fmt(d['held']):>10}"
              f"{fmt(d['fix1']):>9}{d['nfix']:>12}")

    t0, t1 = -20.0, end
    fig, axs = plt.subplots(len(rows), 1, figsize=(11, 1.95 * len(rows) + 0.7), dpi=140, sharex=True,
                            gridspec_kw=dict(hspace=0.45))
    axs = list(axs) if len(rows) > 1 else [axs]
    v500, a80 = spans("speed_mps", 500.0), spans("alt_m", 80000.0)
    ymax = max([n for d in rows for _t, n in d["e5"]] + [8]) + 2
    same = [len({d["setup"][k] for d in rows}) == 1 for k in range(3)]
    shared = ", ".join(v for v, s in zip(rows[0]["setup"], same) if s)
    for ax, d in zip(axs, rows):
        for a, b in v500:
            ax.axvspan(a, b, color="#eda100", alpha=0.10, lw=0)
        for a, b in a80:
            ax.axvspan(a, b, color="#7b61ff", alpha=0.08, lw=0)
        ax.axvline(0.0, color=INK3, lw=0.6)
        pts = [(t, n) for t, n in d["e5"] if t0 <= t <= t1]
        ax.plot([t for t, _n in pts], [n for _t, n in pts], ".", ms=1.6, color=DOT)
        for a, b in d["bands"]:
            if b >= t0 and a <= t1:
                ax.axvspan(max(a, t0), min(b + 0.05, t1), ymin=0.0, ymax=0.07, color=FIX, lw=0)
        for t, _nm in d["events"]:
            ax.axvline(t, color=INK, lw=1.0, ls=(0, (4, 3)))
        ax.set_ylim(-1.2, ymax)
        ax.tick_params(labelsize=7, colors=INK3)
        ax.grid(color=RULE, lw=0.5)
        for sp in ax.spines.values():
            sp.set_color(RULE)
        f1 = lambda v: "-" if v is None else f"{v:.1f} s"
        own = ", ".join(v for v, s in zip(d["setup"], same) if not s)
        ax.set_title(d["label"] + (f"  ({own})" if own else ""), fontsize=8.5, color=INK, loc="left", pad=3)
        ax.text(1.0, 1.02, f"ephemeris at ignition {d['eph']} of {d['seen']}   raw back {f1(d['held'])}   "
                f"first fix {f1(d['fix1'])}   fix epochs {d['nfix']}", transform=ax.transAxes, ha="right",
                va="bottom", fontsize=7.5, color=INK3)
        ax.set_ylabel("meas /\nepoch", fontsize=7.5, color=INK3)
    axs[-1].set_xlim(t0, t1)
    cross = ", ".join(f"{a:.1f}" for a, _b in v500[1:]) if len(v500) > 1 else ""
    axs[-1].set_xlabel(f"seconds after ignition (burnout {burn:.1f}; 500 m/s crossings {v500[0][1]:.1f}"
                       + (f" and {cross}" if cross else "") + f"; 80 km {a80[0][0]:.1f}-{a80[0][1]:.1f})",
                       fontsize=7.5, color=INK3)
    rxs = " vs ".join(sorted({c.name.split("_")[0].upper() for c in caps}))
    fig.suptitle(f"{rxs}, 13.5 g spaceshot: raw measurements per epoch (blue), own fix (green band), "
                 "restart (dashed); amber > 500 m/s, violet > 80 km" + (f"\nall rows: {shared}" if shared else ""),
                 fontsize=9, color=INK, x=0.07, ha="left", y=0.995)
    fig.savefig(out, bbox_inches="tight")
    print("wrote", out, "with", len(rows), "rows")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
