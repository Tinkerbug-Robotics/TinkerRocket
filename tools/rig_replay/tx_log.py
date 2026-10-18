#!/usr/bin/env python3
"""The transmitter's own log of a rig run (hackrf_tx_ram -B, run_v9.sh's hackrf.txt): its underruns on the file's
timeline, whether it sent the whole file, and what that does to a V9 rig replay.

Each underrun is the HackRF's buffer running dry: it transmits nothing until bytes come again, then carries on from
where the file was, so the whole sky after it is late by the gap (2 bytes a sample at 18.48 Msps: 36.96 MB a second;
253,312 bytes is 6.85 ms). A host shortfall is the same from the other end: the reader fell behind the ring, and the
transfer went out padded with silence (at most one 262,144-byte transfer, 7.09 ms, each) -- the file then resumes
where it was, so the sky is late by the padding. Both are slips here; the padding also counts in the bytes sent, so
the file's timeline after a shortfall is read up to that much late. The replay's clock follows the 1PPS line until T-5 s and then runs on the MCU's crystal, so
  - an underrun after the freeze (file second 175) and before the landing moves the sky against the replayed IMU for
    the rest of the flight: the run is VOID;
  - one between the clock's arming and the freeze: the edges move with the sky at once, the driver's clock line
    (a lower envelope of them) only after about a minute, and the replay's clock takes the line's tags. The freeze
    is checked against the edge (the firmware moves it onto the edge, "RIG REPLAY clock: the freeze checked"), but
    while the line still lags the filter's own tags are that far off the sky: DEGRADED here, and the PPS tag check
    after the freeze decides (score_v9.py voids on any second over 200 us from the freeze to the landing);
  - one before the clock, or after the landing, does not touch the scored flight;
  - the last stats line, printed after the transmitter stopped, is the end of the file draining: not judged.
The counts are cumulative and "longest" is the longest gap so far, so a line whose count rose by n with the longest
unchanged had n gaps of at most that; the slip is given as an upper bound, and exactly where the longest rose.

    tx_log.py HACKRF_TXT [--armed-file-s X] [--freeze-file-s 175] [--landing-file-s L] [--exit N] [--stopped]
              [--json OUT]

Exit status 0 clean, 2 degraded, 3 void.
"""
import argparse
import json
import re
import sys

RATE_BPS = 2 * 18.48e6        # IQ8 at 18.48 Msps: bytes a second of the file
TRANSFER_B = 262144           # one libhackrf transfer: the most silence a host shortfall pads
STATS = re.compile(r"^([0-9.]+) MB / ([0-9.]+) sec = [-0-9.]+ MB/second, .*?(\d+) underruns, longest (\d+) bytes"
                   r".*?host shortfalls (\d+)")
SENT = re.compile(r"hackrf_tx_ram: sent ([0-9.]+) GB of ([0-9.]+) GB")


def parse(text, rate_bps=RATE_BPS):
    """Each per-second line on the file's timeline: the bytes sent so far as file seconds (the sky's own time
    from the file's start), the transmitter's wall seconds, and every rise of the underrun count."""
    lines, sent = [], None
    for ln in text.splitlines():
        ln = ln.strip()
        m = STATS.match(ln)
        if m:
            lines.append({"mb": float(m.group(1)), "sec": float(m.group(2)), "under": int(m.group(3)),
                          "longest": int(m.group(4)), "host": int(m.group(5))})
            continue
        m = SENT.search(ln)
        if m:
            sent = (float(m.group(1)), float(m.group(2)))
    # the lines print a tenth of a MB: scaled to the bytes the transmitter says it sent, when it says
    total_mb = sum(ln["mb"] for ln in lines)
    scale = sent[0] * 1e3 / total_mb if sent and total_mb > 0 and abs(sent[0] * 1e3 / total_mb - 1) < 0.01 else 1.0
    pos = wall = 0.0
    prev_u = prev_l = prev_h = 0
    events = []
    for k, ln in enumerate(lines):
        start = pos
        pos += ln["mb"] * scale * 1e6 / rate_bps
        wall += ln["sec"]
        ln["file_s"], ln["wall_s"] = pos, wall
        drain = sent is not None and k == len(lines) - 1
        if ln["under"] > prev_u:
            n = ln["under"] - prev_u
            rose = ln["longest"] > prev_l
            events.append({"line": k, "kind": "underrun", "file_s_from": round(start, 2), "file_s_to": round(pos, 2),
                           "wall_s": round(wall, 2), "added": n, "longest_bytes": ln["longest"],
                           "longest_rose": rose,
                           "slip_ms_max": round(n * ln["longest"] / rate_bps * 1e3, 3),
                           "slip_ms_longest": round(ln["longest"] / rate_bps * 1e3, 3) if rose else None,
                           "drain": drain})
        if ln["host"] > prev_h:
            n = ln["host"] - prev_h
            events.append({"line": k, "kind": "host shortfall", "file_s_from": round(start, 2),
                           "file_s_to": round(pos, 2), "wall_s": round(wall, 2), "added": n,
                           "longest_bytes": TRANSFER_B, "longest_rose": False,
                           "slip_ms_max": round(n * TRANSFER_B / rate_bps * 1e3, 3), "slip_ms_longest": None,
                           "drain": drain})
        prev_u, prev_l, prev_h = max(prev_u, ln["under"]), max(prev_l, ln["longest"]), max(prev_h, ln["host"])
    out = {"lines": len(lines), "events": events, "underruns": prev_u, "longest_bytes": prev_l,
           "host_shortfalls": prev_h if lines else None, "file_s_sent": round(pos, 2),
           "sent_gb": sent[0] if sent else None, "file_gb": sent[1] if sent else None}
    out["complete"] = bool(sent and sent[0] >= sent[1] - 0.0015)
    return out


def judge(tx, armed_file_s=None, freeze_file_s=175.0, landing_file_s=None, exit_status=None, stopped=False):
    """VOID, DEGRADED or clean, with why: each underrun's window on the file's timeline, the exit status, and
    whether the whole file went out (only a run the script did not stop has to send it all)."""
    void, degraded = [], []
    for e in tx["events"]:
        if e["drain"]:
            e["window"] = "the end of the file draining"
            continue
        if landing_file_s is not None and e["file_s_from"] >= landing_file_s:
            e["window"] = "after the landing"
        elif freeze_file_s is not None and e["file_s_to"] > freeze_file_s:
            e["window"] = "after the freeze"
            void.append(f"{e['added']} {e.get('kind', 'underrun')}(s) at file second {e['file_s_from']:.1f}-"
                        f"{e['file_s_to']:.1f}, after the replay clock froze: the sky up to {e['slip_ms_max']:.2f} ms "
                        f"late on the IMU")
        elif armed_file_s is not None and e["file_s_to"] > armed_file_s:
            e["window"] = "between the clock and the freeze"
            degraded.append(f"{e['added']} {e.get('kind', 'underrun')}(s) at file second {e['file_s_from']:.1f}-"
                            f"{e['file_s_to']:.1f} (up to {e['slip_ms_max']:.2f} ms) while the clock followed the "
                            f"1PPS line: the freeze's check against the edge and the PPS tag check after it decide")
        else:
            e["window"] = "before the clock"
    if not stopped:
        if exit_status not in (None, 0):
            void.append(f"the transmitter exited {exit_status}")
        if tx["sent_gb"] is None:
            void.append("the transmitter never said how much it sent")
        elif not tx["complete"]:
            void.append(f"the transmitter sent {tx['sent_gb']:.3f} of {tx['file_gb']:.3f} GB")
    verdict = "VOID" if void else ("DEGRADED" if degraded else "clean")
    return {**tx, "armed_file_s": armed_file_s, "freeze_file_s": freeze_file_s, "landing_file_s": landing_file_s,
            "exit_status": exit_status, "stopped_by_script": stopped, "verdict": verdict, "void_why": void,
            "degraded_why": degraded}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("hackrf_txt")
    ap.add_argument("--armed-file-s", type=float)
    ap.add_argument("--freeze-file-s", type=float, default=175.0)
    ap.add_argument("--landing-file-s", type=float)
    ap.add_argument("--exit", type=int, dest="exit_status")
    ap.add_argument("--stopped", action="store_true", help="the script stopped the transmitter (a void run)")
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    r = judge(parse(open(a.hackrf_txt, errors="replace").read()), a.armed_file_s, a.freeze_file_s,
              a.landing_file_s, a.exit_status, a.stopped)
    for e in r["events"]:
        slip = f"up to {e['slip_ms_max']:.2f} ms" + (f", one of {e['slip_ms_longest']:.2f} ms" if e["longest_rose"] else "")
        print(f"  {e.get('kind', 'underrun')} +{e['added']} at file second {e['file_s_from']:.1f}-{e['file_s_to']:.1f} "
              f"(TX second {e['wall_s']:.0f}; {slip}): {e['window']}")
    sent = "-" if r["sent_gb"] is None else f"{r['sent_gb']:.3f} of {r['file_gb']:.3f} GB"
    print(f"transmitter: {r['verdict']}; {r['underruns']} underruns, longest {r['longest_bytes']} bytes; sent {sent}; "
          f"exit {r['exit_status']}{' (stopped by the script)' if r['stopped_by_script'] else ''}")
    for w in r["void_why"] + r["degraded_why"]:
        print(f"  {w}")
    if a.json:
        with open(a.json, "w") as f:
            json.dump(r, f, indent=1)
    return 3 if r["verdict"] == "VOID" else (2 if r["verdict"] == "DEGRADED" else 0)


if __name__ == "__main__":
    sys.exit(main())
