#!/usr/bin/env python3
"""The Doppler each satellite actually presented during the burn.

Section 06 first correlated carrier loss against sin(elevation), which is a
proxy: for a near-vertical boost the Doppler rate goes as `a * sin(elev)`, so
elevation stands in for the thing we think is the cause. This measures the
cause instead.

Doppler comes from UBX-RXM-RAWX (0x02 0x15), which reports the receiver's own
per-satellite Doppler in Hz. Only the NEO-M8T capture logged RXM, but that is
enough for all four receivers: every spaceshot run replayed a byte-identical
scenario file -- same ephemeris, same start time, same trajectory -- so the
Doppler a given satellite presents at a given moment is a property of the
injected signal, not of the receiver listening to it. `--verify` re-checks that
identity before the reference is used.

Two quantities per satellite, because they discriminate between two mechanisms:

  rate_hzs        how fast the carrier is sliding, from a least-squares fit
                  over the burn. This is what a tracking loop has to slew to
                  follow.
  peak_shift_hz   how far off nominal the carrier sits. Large but static, and
                  set mostly by the satellite's own motion, so it is close to
                  independent of elevation -- which is what makes it a usable
                  control against the rate.

A satellite whose RXM measurement drops out through the burn gets rate_hzs
null rather than a guess. That is not a gap in the data so much as the effect
itself: the instrument that would measure the stimulus is the one the stimulus
disabled.

  python3 doppler_ref.py            # writes results/doppler_ref_<scenario>.json
"""
import argparse, gzip, json, struct, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCENARIOS = ("spaceshot", "gentle_alt")
RXM_RAWX = b"\x02\x15"
PEERS = ["zed_f9p", "quescan_m10", "beitian_bn182"]
MIN_IN_BURN = 5


def rawx(path):
    """{(gnssId, svId): [(capture_t, doppler_hz)]} from RXM-RAWX."""
    out = {}
    with gzip.open(path, "rt", errors="replace") as f:
        for ln in f:
            p = ln.split()
            if len(p) < 3 or p[1] != "U":
                continue
            try:
                t = float(p[0]); b = bytes.fromhex(p[2])
            except ValueError:
                continue
            if b[:2] != RXM_RAWX:
                continue
            pl = b[2:]
            if len(pl) < 16:
                continue
            n = pl[11]
            if 16 + 32 * n > len(pl):
                continue                       # truncated line
            for i in range(n):
                o = 16 + 32 * i
                do, = struct.unpack_from("<f", pl, o + 16)
                out.setdefault((pl[o + 20], pl[o + 21]), []).append((t, do))
    return out


def slope(pts):
    n = len(pts)
    mt = sum(t for t, _ in pts) / n
    md = sum(d for _, d in pts) / n
    den = sum((t - mt) ** 2 for t, _ in pts)
    return None if den == 0 else sum((t - mt) * (d - md) for t, d in pts) / den


def verify_shared(scen):
    """Every peer must have flown the same scenario or the reference is void."""
    import hashlib
    def h(p):
        return hashlib.sha256(
            json.dumps(json.loads(Path(p).read_text()), sort_keys=True).encode()
        ).hexdigest()
    base = h(HERE / "results" / f"neo_m8t_{scen}.scenario.json")
    bad = []
    for pid in PEERS:
        p = HERE / "results" / f"{pid}_{scen}.scenario.json"
        if not p.exists() or h(p) != base:
            bad.append(pid)
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenarios", nargs="*", default=list(SCENARIOS),
                    help=f"default: {' '.join(SCENARIOS)}")
    ap.add_argument("--offset", type=float, default=None,
                    help="scenario_t - capture_t (default: fit via boost_sats)")
    ap.add_argument("--verify", action="store_true",
                    help="only check that the peer scenarios are identical")
    a = ap.parse_args()

    import subprocess
    for scen in (a.scenarios or SCENARIOS):
        bad = verify_shared(scen)
        if bad:
            sys.exit(f"{scen}: scenario mismatch against {', '.join(bad)} -- the "
                     f"Doppler reference is only shared if the injected signal "
                     f"is identical")
        if a.verify:
            print(f"  {scen}: all {len(PEERS)+1} scenarios identical")
            continue

        src = HERE / "results" / f"neo_m8t_{scen}.log.gz"
        sc = HERE / "results" / f"neo_m8t_{scen}.scenario.json"
        out = HERE / "results" / f"doppler_ref_{scen}.json"
        d = json.loads(subprocess.run(
            [sys.executable, str(HERE / "boost_sats.py"), str(src), str(sc), "--json"],
            capture_output=True, text=True).stdout)
        off = a.offset if a.offset is not None else d["offset_s"]
        b0, b1 = d["burn_s"]

        ref, dropped = {}, []
        for (g, s), v in sorted(rawx(src).items()):
            v = sorted((t + off, x) for t, x in v)
            inb = [(t, x) for t, x in v if b0 <= t <= b1]
            near = [abs(x) for t, x in v if b0 - 8 <= t <= b1 + 8]
            if not near:
                continue
            rate = slope(inb) if len(inb) >= MIN_IN_BURN else None
            if rate is None:
                dropped.append(f"{g}:{s}")
            ref[f"{g}:{s}"] = {"rate_hzs": rate, "peak_shift_hz": max(near),
                               "n_in_burn": len(inb)}

        out.write_text(json.dumps({
            "source": src.name, "scenario": d["scenario"], "offset_s": off,
            "burn_s": [b0, b1], "peak_accel_g": d["peak_accel_g"],
            "shared_with": PEERS, "min_in_burn": MIN_IN_BURN,
            "dropped_out": dropped, "sats": ref}, indent=1) + "\n")
        print(f"  {out.relative_to(HERE)}  {d['peak_accel_g']} g, {len(ref)} satellites, "
              f"{len(dropped)} with no measurable rate ({', '.join(dropped) or '--'})")


if __name__ == "__main__":
    main()
