#!/usr/bin/env python3
"""Grid B (G4, G5): the V8/V9 mosaic-only build's one navigation filter, through raw_replay_mosaic, on E0's 16
drogue flights, against its --unaided twin.

For each flight and fault pattern, sim_events.py writes the events file once (the receiver model and the fault,
common to the filter and the receiver's own fix); then each is replayed in every magnetometer arm, the flight design
and its unaided twin:
  - "0", "+10", "-10", "+40", "-40": the magnetometer fused on the pad with that heading error
    (--mag-error-deg; the seed's heading turned with it, as a long pad wait leaves it), the 10 deg launch prior;
  - "none30": no magnetometer (--mag-flags 0: every sample refused, so the 30 deg prior), the seed's heading a
    guess 30 deg off (+30 on seeds 1 and 3, -30 on 2 and 4).
The flight design is the mosaic build's defaults (--receiver mosaic: profile v0, the 16-satellite cap, act with
the whole attitude from launch, the quorum with the raw vote guard as flight_votes.h wires it -- D1b's wiring,
assumed: no firmware tree has it yet); the twin adds --unaided; "noguard" is the flight design with the guard
observed, not applied (--vote-guard 0: the quorum as RC0's main.cpp has it). Every run also flies the quorum on the
flight's truth beside it (--truth-nav: the truth's own call). Each run is scored as it finishes
(sim_tables.score_run) and its CSV removed unless --keep-csv names its pattern; the scores go to OUT/scores/, and
sim_tables.py makes the tables. The events files and each flight's truth (truth/<flight>.npz, which the scores
are made against, and truth/<flight>.nav, which the truth's quorum reads) are in --events-dir.

    PYTHONPATH=<simbuild>/tinkerrocket-sim/src:<gnss-hf-sim>/sim \\
        python3 tools/raw_replay/sim_grid.py OUT --flights <grid>/flights [--patterns none,cut05,...] \\
        [--mags 0,+10,...] [--arms design,unaided,noguard] [--jobs 8] \\
        [--binary tools/raw_replay/build/raw_replay_mosaic] [--lever-arm 0.3] [--harness-args "..."] \\
        [--pause-file FILE]
    python3 tools/raw_replay/sim_tables.py OUT

--pause-file names a file another job writes "active" into while it needs the machine quiet (a bench rig's timing
run, say): before each events file and each run the workers read it and wait, checking every 30 s, while it says so.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import glob
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

MAGS = ["0", "+10", "-10", "+40", "-40", "none30"]
ARMS = ["design", "unaided", "noguard"]
PAUSE_POLL_S = 30.0


def wait_quiet(pause_file):
    """Wait while pause_file says "active" (--pause-file): another job needs the machine quiet."""
    if not pause_file:
        return
    while True:
        try:
            with open(pause_file) as fh:
                busy = "active" in fh.read()
        except OSError:
            busy = False
        if not busy:
            return
        time.sleep(PAUSE_POLL_S)


def make_events(job):
    flight, pattern, out_dir, phases_path, lever_arm, force, pause_file = job
    import sim_events
    name = os.path.splitext(os.path.basename(flight))[0]
    path = os.path.join(out_dir, f"{name}__{pattern}.events")
    if os.path.exists(path) and not force:
        return path, None, 0.0
    wait_quiet(pause_file)
    t0 = time.time()
    phases = json.load(open(phases_path))["flights"]
    path, counts = sim_events.build(flight, pattern, out_dir, phases, lever_arm=lever_arm)
    return path, counts, time.time() - t0


def read_meta(events):
    meta = {}
    with open(events) as fh:
        for ln in fh:
            if ln.startswith("META "):
                k, _, v = ln[5:].rstrip("\n").partition(" ")
                meta[k] = v
            elif not ln.startswith("#"):
                break
    return meta


def run_one(job):
    events, mag, arm, binary, runs_dir, scores_dir, keep, extra, pause_file = job
    import sim_tables
    base = os.path.basename(events)[:-len(".events")]
    flight, pattern = base.split("__")
    rid = f"{base}__{mag}__{arm}"
    score_path = os.path.join(scores_dir, rid + ".json")
    if os.path.exists(score_path):
        return rid, "kept", 0.0
    csv = os.path.join(runs_dir, rid + ".csv")
    args = [binary, "--in", events, "--out", csv, "--receiver", "mosaic"]
    if mag == "none30":
        seed = int(flight.rsplit("_s", 1)[1])
        args += ["--mag-flags", "0", "--seed-heading-error-deg", "30" if seed % 2 else "-30"]
    else:
        args += ["--mag-error-deg", mag.lstrip("+")]
    if arm == "unaided":
        args.append("--unaided")
    elif arm == "noguard":
        args += ["--vote-guard", "0"]
    elif arm != "design":
        raise SystemExit(f"unknown arm {arm}")
    nav = os.path.join(os.path.dirname(events), "truth", flight + ".nav")
    if os.path.exists(nav):
        args += ["--truth-nav", nav]
    args += extra
    wait_quiet(pause_file)
    t0 = time.time()
    p = subprocess.run(args, capture_output=True, text=True)
    dt = time.time() - t0
    meta = read_meta(events)
    truth = sim_tables.load_truth(os.path.join(os.path.dirname(events), "truth", flight + ".npz"))
    sc = sim_tables.score_run(csv, csv + ".events.json", truth,
                              dict(flight=flight, pattern=pattern, mag=mag, arm=arm, rc=p.returncode))
    sc["stops"] = sim_tables.parse_stderr_stops(p.stderr)
    sc["sim_meta"] = {k: int(v) for k, v in meta.items() if k.startswith("sim_") and v.lstrip("-").isdigit()}
    sc["stderr_tail"] = p.stderr.strip().splitlines()[-8:]
    sc["args"] = " ".join(a for a in args[1:] if not a.startswith("/"))
    sc["seconds"] = dt
    with open(score_path + ".tmp", "w") as fh:
        json.dump(sc, fh)
    os.replace(score_path + ".tmp", score_path)
    if not (keep and (pattern in keep or "all" in keep)):
        for f in (csv, csv + ".events.json"):
            if os.path.exists(f):
                os.remove(f)
    return rid, p.returncode, dt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("--flights", required=True, help="E0's grid/flights (the pickles and truth_phases.json)")
    ap.add_argument("--patterns", default="all")
    ap.add_argument("--mags", default=",".join(MAGS))
    ap.add_argument("--arms", default=",".join(ARMS))
    ap.add_argument("--harness-args", default="", help="more raw_replay arguments for every run (an experiment: "
                                                         "use a new OUT, the scores record them)")
    ap.add_argument("--only", default="", help="flight names (comma-separated) to run, default every one")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    ap.add_argument("--binary", default=os.path.join(HERE, "build", "raw_replay_mosaic"))
    ap.add_argument("--lever-arm", type=float, default=0.3)
    ap.add_argument("--keep-csv", default="none", help="patterns whose CSVs are kept (comma-separated; 'all')")
    ap.add_argument("--events-dir", help="where the events files go (default OUT/events; ~6 MB each, made again "
                                         "in minutes: a scratch directory will do)")
    ap.add_argument("--events-only", action="store_true")
    ap.add_argument("--force-events", action="store_true")
    ap.add_argument("--pause-file", default="", help="wait before each events file and run while this file says "
                                                     "'active' (checked every 30 s)")
    a = ap.parse_args()
    import sim_events
    pats = list(sim_events.PATTERNS) if a.patterns == "all" else a.patterns.split(",")
    flights = sorted(glob.glob(os.path.join(a.flights, "*.pkl")))
    if a.only:
        flights = [f for f in flights if os.path.splitext(os.path.basename(f))[0] in a.only.split(",")]
    phases = os.path.join(a.flights, "truth_phases.json")
    ev_dir = a.events_dir or os.path.join(a.out, "events")
    runs_dir = os.path.join(a.out, "runs")
    scores_dir = os.path.join(a.out, "scores")
    for d in (ev_dir, runs_dir, scores_dir):
        os.makedirs(d, exist_ok=True)
    t0 = time.time()
    jobs = [(f, p, ev_dir, phases, a.lever_arm, a.force_events, a.pause_file) for f in flights for p in pats]
    events = []
    with cf.ProcessPoolExecutor(a.jobs) as ex:
        for path, counts, dt in ex.map(make_events, jobs):
            events.append(path)
            if counts is not None:
                print(f"events {os.path.basename(path)} {dt:.1f} s {json.dumps(counts)}", flush=True)
    print(f"{len(events)} events files in {time.time() - t0:.0f} s", flush=True)
    if a.events_only:
        return 0
    if not os.path.exists(a.binary):
        raise SystemExit(f"no harness at {a.binary}: build tools/raw_replay first")
    keep = set(a.keep_csv.split(",")) if a.keep_csv else set()
    extra = a.harness_args.split()
    rjobs = [(e, m, arm, a.binary, runs_dir, scores_dir, keep, extra, a.pause_file) for e in events
             for m in a.mags.split(",") for arm in a.arms.split(",")]
    t1 = time.time()
    n = 0
    rcs = {}
    with cf.ProcessPoolExecutor(a.jobs) as ex:
        for rid, rc, dt in ex.map(run_one, rjobs, chunksize=4):
            n += 1
            rcs[rc] = rcs.get(rc, 0) + 1
            if n % 100 == 0 or n == len(rjobs):
                print(f"runs {n}/{len(rjobs)} ({time.time() - t1:.0f} s) exit codes {rcs}", flush=True)
    print(f"{n} runs in {time.time() - t1:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
