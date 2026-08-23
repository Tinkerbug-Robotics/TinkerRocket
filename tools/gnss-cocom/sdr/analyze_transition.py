#!/usr/bin/env python3
"""Examine what the satellites were doing across a fix-loss, at any C/N0 bar.

The rig's BLOCKED test needs 4 satellites at >=30 dBHz. On a marginal RF path a
real gate closure can fall below that bar and be scored NO_LOCK -- the honest
default, but it discards the distinguishing evidence rather than showing it.
This prints the satellite population either side of the transition at several
thresholds, so "the fix went away but the satellites did not" stays visible even
when the absolute levels are poor.
"""
import sys, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from gnss_nmea_monitor import replay_source, Parser
from skytraq_binary import MSG_RCV_STATE, MSG_SV_CH_STATUS, parse_rcv_state, parse_sv_ch_status

cap, w0, tow0 = Path(sys.argv[1]), 2432, 340200
meta = json.loads(Path(sys.argv[2]).read_text())
truth = {round(s["t"],1): s for s in meta["truth"]}

rows, p, pend = [], Parser(), None
sats = []
for _t, kind, data in replay_source(str(cap)):
    if kind != "bin": continue
    if data[0] == MSG_SV_CH_STATUS:
        sats = parse_sv_ch_status(data) or sats
    if data[0] == MSG_RCV_STATE:
        r = parse_rcv_state(data)
        if not r: continue
        p.feed_binary(data)
        tt = (r["week"]-w0)*604800 + (r["tow"]-tow0)
        rows.append((tt, r["has_fix"], list(sats)))

def census(ss, bar): return sum(1 for x in ss if x["cn0"] >= bar)

print(f"{cap.name}  ({len(rows)} epochs)\n")
print(f"{'t_traj':>8} {'inj spd':>8} {'fix':>4} " +
      " ".join(f"{'>='+str(b):>6}" for b in (35,30,25,20,15)) + "   used")
print("-"*66)
prev = None
for tt, fix, ss in rows:
    if prev is not None and prev[1] and not fix:
        for label, (t2, f2, s2) in (("last fix", prev), ("first loss", (tt,fix,ss))):
            tr = truth.get(round(t2,1))
            spd = f"{tr['speed_mps']:.0f}" if tr else "--"
            print(f"{t2:>8.1f} {spd:>8} {'Y' if f2 else 'N':>4} " +
                  " ".join(f"{census(s2,b):>6}" for b in (35,30,25,20,15)) +
                  f"   {sum(1 for x in s2 if x['used_in_fix']):>4}   <- {label}")
        print()
    prev = (tt, fix, ss)
