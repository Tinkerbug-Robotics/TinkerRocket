#!/usr/bin/env python3
"""Per-satellite C/N0 through the burn, against elevation.

The gate analysis asks when a receiver stops publishing a position. This asks a
different question: which individual satellites it stops tracking while it is
still publishing one. Doppler rate goes as `a * sin(elevation)`, so a boost
stresses the tracking loop hardest on the satellites nearest the velocity
vector and barely at all on the ones near the horizon. If that is what limits
these parts, the loss should be concentrated overhead and should scale with
acceleration.

Reads NAV-SAT (0x01 0x35) for per-satellite carrier and elevation, and NAV-PVT
(0x01 0x07) only to align the capture clock to the scenario clock. Both come
from the same raw hex capture the gate analysis uses, so this re-reads the
flights already flown rather than needing new ones.

  python3 boost_sats.py results/zed_f9p_spaceshot.log.gz \
                        results/zed_f9p_spaceshot.scenario.json
"""
import argparse, gzip, json, math, statistics, struct, sys

NAV_PVT = b"\x01\x07"
NAV_SAT = b"\x01\x35"


def read_capture(path):
    """(pvt, sat) from a `<t> U <hex>` capture. Payload follows class/id."""
    pvt, sat = [], []
    with gzip.open(path, "rt", errors="replace") as f:
        for ln in f:
            p = ln.split()
            if len(p) < 3 or p[1] != "U":
                continue
            try:
                t = float(p[0]); b = bytes.fromhex(p[2])
            except ValueError:
                continue
            if b[:2] == NAV_PVT and len(b) >= 2 + 78:
                pl = b[2:]
                pvt.append((t, pl[21] & 1, struct.unpack_from("<i", pl, 36)[0] / 1000.0))
            elif b[:2] == NAV_SAT and len(b) >= 2 + 8:
                pl = b[2:]
                n = pl[5]
                if 8 + 12 * n > len(pl):
                    continue                      # truncated line, skip it
                svs = []
                for i in range(n):
                    g, s, c, e = struct.unpack_from("<BBBb", pl, 8 + 12 * i)
                    svs.append((g, s, c, e))
                sat.append((t, svs))
    return pvt, sat


def align(pvt, truth):
    """Offset where scenario_t = capture_t + offset, from the altitude climb.

    Grid search rather than a crossing: the receiver blanks partway up the
    boost, so there is no single feature present in both series to anchor on.
    """
    tr = {round(s["t"], 1): s["alt_m"] for s in truth}
    usable = [(t, a) for t, ok, a in pvt if ok]
    if len(usable) < 20:
        return None
    best, best_err = None, None
    off = -60.0
    while off <= 60.0:
        errs = []
        for t, a in usable:
            ta = tr.get(round(t + off, 1))
            if ta is not None:
                errs.append(abs(a - ta))
        if len(errs) >= 20:
            e = statistics.median(errs)
            if best_err is None or e < best_err:
                best, best_err = off, e
        off += 0.1
    return best if best_err is not None and best_err < 500.0 else None


def phase_window(truth, name):
    """Ramp scenarios carry no phase labels at all, so .get() not [\"phase\"]."""
    ts = [s["t"] for s in truth if s.get("phase") == name]
    return (min(ts), max(ts)) if ts else None


def median_by_sv(sat, lo, hi, off):
    """{(gnss,sv): (median cno, median elev, n_epochs)} over scenario [lo,hi)."""
    acc = {}
    for t, svs in sat:
        st = t + off
        if not (lo <= st < hi):
            continue
        for g, s, c, e in svs:
            acc.setdefault((g, s), []).append((c, e))
    out = {}
    for k, v in acc.items():
        elevs = [e for _, e in v if -90 <= e <= 90]
        out[k] = (statistics.median([c for c, _ in v]),
                  statistics.median(elevs) if elevs else None,
                  len(v))
    return out


def pearson(xs, ys):
    n = len(xs)
    if n < 3:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    sy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if sx == 0 or sy == 0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (sx * sy)


GNSS = {0: "GPS", 1: "SBS", 2: "GAL", 3: "BDS", 5: "QZS", 6: "GLO"}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("scenario")
    ap.add_argument("--offset", type=float, help="scenario_t - capture_t (default: fit)")
    ap.add_argument("--baseline", type=float, default=60.0,
                    help="seconds of pad time to average before ignition (default 60)")
    ap.add_argument("--json", action="store_true", help="emit JSON")
    a = ap.parse_args()

    meta = json.load(open(a.scenario))
    truth = meta["truth"]
    pvt, sat = read_capture(a.capture)
    if not sat:
        sys.exit(f"{a.capture}: no NAV-SAT records -- this capture cannot answer "
                 f"the per-satellite question")

    boost = phase_window(truth, "boost")
    if not boost:
        sys.exit(f"{meta.get('scenario')}: no boost phase in the truth track")
    b0, b1 = boost

    off = a.offset if a.offset is not None else align(pvt, truth)
    if off is None:
        sys.exit(f"{a.capture}: could not align to {meta.get('scenario')}; pass --offset")

    dt = truth[1]["t"] - truth[0]["t"]
    bo = [s for s in truth if s.get("phase") == "boost"]
    acc = max((bo[i + 1]["speed_mps"] - bo[i]["speed_mps"]) / dt
              for i in range(len(bo) - 1)) if len(bo) > 2 else float("nan")

    # Baseline ends 5 s before ignition so a late pad epoch cannot leak in.
    base = median_by_sv(sat, b0 - a.baseline, b0 - 5.0, off)
    # Burn starts 1 s late: the first epoch straddles ignition.
    burn = median_by_sv(sat, b0 + 1.0, b1, off)

    rows = []
    for k, (c0, e0, n0) in sorted(base.items()):
        if c0 <= 0 or e0 is None or n0 < 5:
            continue                              # not tracked on the pad
        c1 = burn.get(k, (0.0, None, 0))[0]
        rows.append({"gnss": GNSS.get(k[0], str(k[0])), "sv": k[1], "elev": e0,
                     "cno_pad": c0, "cno_burn": c1, "delta": c1 - c0,
                     "lost": c1 <= 0})
    if not rows:
        sys.exit(f"{a.capture}: no satellites tracked through the pad baseline")

    r = pearson([math.sin(math.radians(x["elev"])) for x in rows],
                [x["delta"] for x in rows])

    def band(lo, hi):
        b = [x for x in rows if lo <= x["elev"] < hi]
        return (len(b), statistics.median([x["delta"] for x in b]) if b else None,
                sum(1 for x in b if x["lost"]))

    out = {"capture": a.capture, "scenario": meta.get("scenario"),
           "offset_s": round(off, 1), "peak_accel_mps2": round(acc, 1),
           "peak_accel_g": round(acc / 9.80665, 1),
           "burn_s": [b0, b1], "n_sats": len(rows),
           "r_sin_elev_vs_delta": None if r is None else round(r, 2),
           "bands": {"ge45": band(45, 91), "30to45": band(30, 45), "lt30": band(-90, 30)},
           "sats": rows}
    if a.json:
        print(json.dumps(out, indent=1)); return

    print(f"{meta.get('scenario')}  {a.capture}")
    print(f"  aligned scenario_t = capture_t + {off:.1f} s | burn {b0:.0f}-{b1:.0f} s "
          f"| peak {acc:.0f} m/s^2 ({acc/9.80665:.1f} g)")
    print(f"  {len(rows)} satellites tracked on the pad; "
          f"{sum(1 for x in rows if x['lost'])} lost through the burn")
    print(f"  r(sin elev, dC/N0) = {'n/a' if r is None else f'{r:+.2f}'}")
    print()
    print(f"  {'sat':<8} {'elev':>5} {'pad':>5} {'burn':>5} {'delta':>6}")
    for x in sorted(rows, key=lambda x: -x["elev"]):
        tag = "  LOST" if x["lost"] else ""
        print(f"  {x['gnss']+':'+str(x['sv']):<8} {x['elev']:5.0f} {x['cno_pad']:5.0f} "
              f"{x['cno_burn']:5.0f} {x['delta']:+6.0f}{tag}")
    print()
    for name, (lo, hi) in (("elev >= 45", (45, 91)), ("30-45", (30, 45)), ("< 30", (-90, 30))):
        n, med, lost = band(lo, hi)
        print(f"  {name:<10} n={n:2d}  median dC/N0 "
              f"{'--' if med is None else f'{med:+.0f} dB'}  lost={lost}")


if __name__ == "__main__":
    main()
