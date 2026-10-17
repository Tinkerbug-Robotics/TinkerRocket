"""tools/raw_replay/score_static.py on synthetic runs (build unit E4, gate G3; the 10/17 review).

- the filter's stops counted in the scored window, the whole run's beside them (a stop after the window -- the
  payload carried indoors -- is not G3's);
- each satellite's refusals from the harness's epochs file (every epoch, whatever --every), and without it an upper
  bound that counts the pseudoranges in their own 1 Hz epochs;
- a corrupted epoch: how many of its corrupted satellites the filter refused, against the no-fault run's same epoch;
- the pad ZUPTs around a fault;
- the pad height against the receiver's mean, and the tilt against the accelerometer's own 1 s means.
Distances and statistics only: no position or height leaves the scorer.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools" / "raw_replay"))
import score_static as SS  # noqa: E402

HDR = ("t_mcu_s,t_s,running,in_flight,qw,qx,qy,qz,lat_deg,lon_deg,h_m,n_m,e_m,d_m,vn,ve,vd,"
       "sig_tilt_n_deg,sig_tilt_e_deg,sig_hdg_deg,sig_pn,sig_pe,sig_pd,sig_vn,sig_ve,sig_vd,"
       "ab_x,ab_y,ab_z,wb_x_dps,wb_y_dps,wb_z_dps,clk_b_m,clk_d_mps,healthy,finite,gnss_aided,baro_aided,"
       "epochs,used_pr,used_rr,rej_pr,rej_rr,last_meas,age_ms,eph_sats,tracked,cr_built,cr_in,cr_used,"
       "cr_half,cr_other,levelled,zupts,baro_fused,baro_held,mag_fused,hdg_fit_psi_deg,hdg_fit_sig_deg,"
       "hdg_handoffs,stale")
EPHDR = ("t_s,t_close_s,tow_ms,used_pr,rej_pr,used_rr,rej_rr,nis_max,rej_run_pr_s,rej_run_rr_s,corrupt_rr,"
         "corrupt_pr")


def write_run(path, secs=60.0, rate=100.0, h=100.0, vn=None, clk=None, zupt_until=None, epochs=None):
    """A still, level run: 100 rows a second, an epoch closed every 0.1 s (16 Dopplers, 16 codes each second)."""
    cols = HDR.split(",")
    n = int(secs * rate)
    rows = []
    for i in range(n):
        t = i / rate
        r = dict.fromkeys(cols, 0.0)
        r.update(t_mcu_s=t, t_s=t, running=1, qw=1.0, h_m=h, healthy=1, finite=1, gnss_aided=1,
                 epochs=int(t * 10) + 1, used_rr=16, used_pr=16 if int(t * 10) % 10 == 0 else 0,
                 sig_tilt_n_deg=1.0, sig_tilt_e_deg=1.0, sig_hdg_deg=30.0, age_ms=50.0, eph_sats=20, tracked=20)
        if vn is not None:
            r["vn"] = vn(t)
        if clk is not None:
            r["clk_b_m"] = clk(t)
        if zupt_until is not None:
            r["zupts"] = min(i, int(zupt_until * rate))
        rows.append(",".join(repr(float(r[c])) for c in cols))
    Path(path).write_text(HDR + "\n" + "\n".join(rows) + "\n")
    if epochs is not None:
        lines = [",".join(repr(float(x)) for x in e) for e in epochs]
        Path(str(path) + ".epochs.csv").write_text(EPHDR + "\n" + "\n".join(lines) + "\n")


def epochs_of(secs=60.0, rej=None):
    """One row per 0.1 s epoch: rej(t) -> (rej_pr, rej_rr, run_pr_s, run_rr_s, corrupt_rr, corrupt_pr)."""
    out = []
    for k in range(int(secs * 10)):
        t = k * 0.1
        pr = 16 if k % 10 == 0 else 0
        rp, rr, sp, sr, cr, cp = rej(t) if rej else (0, 0, 0.0, 0.0, 0, 0)
        out.append((t, t + 0.05, 1000 * k + 100, pr - rp if pr else 0, rp if pr else 0, 16 - rr, rr, 1.0, sp, sr,
                    cr, cp))
    return out


def ns(csv, **kw):
    d = dict(csv=str(csv), from_s=None, to_s=None, log=None, cache=None, rig=None, ref=None, events=None, pvt=None,
             cut_from_s=None, cut_to_s=None, corrupt_at_s=None, fuse_s=0.1)
    d.update(kw)
    return argparse.Namespace(**d)


def test_stops_count_in_the_window_and_the_run_beside(tmp_path):
    csv = tmp_path / "run.csv"
    write_run(csv)
    (tmp_path / "run.csv.events.json").write_text(json.dumps(
        {"filter": {"starts": 3, "stops": 2, "last_stop": 4, "stop_at": [[10.0, 4], [55.0, 4]]}}))
    o = SS.score(ns(csv, from_s=5.0, to_s=50.0))
    assert o["health"]["stops"] == 1
    assert o["health"]["stop_at"] == [[10.0, 4]]
    assert o["health"]["stops_run"] == 2
    o = SS.score(ns(csv, from_s=20.0, to_s=50.0))
    assert o["health"]["stops"] == 0


def test_each_satellites_refusal_from_the_epochs_file(tmp_path):
    # a whole-millisecond clock step at 30 s; one satellite's code refused at 31 and 32 s (2 s running), its
    # Doppler at 8 epochs (0.8 s)
    def rej(t):
        if 31.0 <= t < 33.0:
            return (1, 1, 2.0 if t >= 32.0 else 1.0, 0.8, 0, 0)
        return (0, 0, 0.0, 0.0, 0, 0)

    csv = tmp_path / "run.csv"
    write_run(csv, clk=lambda t: -299792.458 if t >= 30.0 else 0.0, epochs=epochs_of(rej=rej))
    o = SS.score(ns(csv, from_s=5.0))
    (j,) = o["clock_jumps"]
    assert j["refusals_from"] == "epochs file"
    assert abs(j["step_ms"] + 1.0) < 1e-6
    assert j["sat_refused_longest_s"] == {"pr": 2.0, "rr": 0.8}
    assert j["refused_pr_5s"] == 2
    assert o["epochs"]["source"] == "epochs file"
    assert o["epochs"]["sat_refused_longest_s"]["pr"] == 2.0


def test_without_the_epochs_file_the_codes_count_in_their_own_epochs(tmp_path):
    # a code refused at two 1 Hz epochs running with nine Doppler-only epochs between them: refused for 2 s,
    # which a run of epochs with any refusal (1) would hide
    cols = HDR.split(",")
    csv = tmp_path / "run.csv"
    write_run(csv, clk=lambda t: -299792.458 if t >= 30.0 else 0.0)
    lines = csv.read_text().splitlines()
    out = [lines[0]]
    for ln in lines[1:]:
        v = ln.split(",")
        t = float(v[cols.index("t_s")])
        if (31.0 <= t < 31.1) or (32.0 <= t < 32.1):
            v[cols.index("rej_pr")] = "1.0"
            v[cols.index("used_pr")] = "15.0"
        out.append(",".join(v))
    csv.write_text("\n".join(out) + "\n")
    o = SS.score(ns(csv, from_s=5.0))
    (j,) = o["clock_jumps"]
    assert j["refusals_from"] == "CSV rows"
    assert j["sat_refused_longest_s_bound"]["pr"] == 2.0


def test_a_corrupted_epoch_refused_against_the_no_fault_runs_same_epoch(tmp_path):
    ref = tmp_path / "ref.csv"
    write_run(ref, epochs=epochs_of())
    run = tmp_path / "run.csv"
    # the corrupted epoch at 20.1 s (tow 201100 ms): 2 of its 3 corrupted satellites built, both Dopplers refused;
    # a pad ZUPT every row until 21 s
    write_run(run, zupt_until=21.0,
              epochs=epochs_of(rej=lambda t: (0, 2, 0.0, 0.1, 2, 2) if abs(t - 20.1) < 1e-6 else (0, 0, 0, 0, 0, 0)))
    (tmp_path / "run.csv.events.json").write_text(json.dumps(
        {"corrupt": {"t_s": 20.1, "tow_ms": 201100, "pr_epoch_asked": 0,
                     "sats": [[0, 3, 60.0, 25.0], [1, 9, -45.0, -20.0], [2, 22, 31.0, 39.0]],
                     "built_rr": 2, "built_pr": 2}}))
    o = SS.score(ns(run, from_s=5.0, ref=str(ref), corrupt_at_s=20.0))
    r = o["recovery"]
    c = r["corrupt"]
    assert r["fault"][0] == 20.0 and abs(r["t_return"] - 20.2) < 1e-9   # the epoch the harness found
    assert c["injected"] == 3 and c["built_rr"] == 2
    assert c["refused_rr"] == 2 and c["fused_rr"] == 0
    assert c["pr_fused_epoch"] is False and c["fused_pr"] == 0
    assert c["size_rr_mps"] == [31.0, 60.0]
    assert r["refusals_from"] == "epochs file" and r["refused_rr_around"] == 2
    assert r["zupts_around"] > 0
    assert r["dv_at_2s_norm"] == 0.0 and r["dpos_at_2s_m"] == 0.0


def test_the_pad_height_and_the_tilt_against_the_accelerometer(tmp_path):
    run = tmp_path / "run.csv"
    write_run(run, h=100.0)
    pvt = tmp_path / "pvt.csv"
    pvt.write_text("t_mcu_s,t_s,h_ell_m,undulation_m,mode,v_acc_m\n" +
                   "\n".join(f"{t:.2f},{t:.2f},{101.0 + 0.2 * math.sin(t):.4f},-30.0,1,2.0"
                             for t in np.arange(0.0, 60.0, 0.05)) + "\n")
    # the accelerometer 1 deg off the level filter, about body x
    a = math.radians(1.0)
    ev = tmp_path / "run.events"
    ev.write_text("\n".join(f"I {int(t * 1e6)} 0.0 {9.80665 * math.sin(a)!r} {-9.80665 * math.cos(a)!r} 0 0 0 1"
                            for t in np.arange(0.0, 60.0, 0.01)) + "\n")
    o = SS.score(ns(run, from_s=5.0, to_s=55.0, pvt=str(pvt), events=str(ev), cache=str(tmp_path)))
    p = o["pad_height"]
    assert abs(p["raw_minus_pvt_means_m"] + 1.0) < 0.05
    assert p["raw_sd_m"] == 0.0
    assert abs(o["vs_accel"]["tilt"]["p50"] - 1.0) < 0.01
    assert o["vs_accel"]["bins"] >= 49
    # distances only: nothing the scorer wrote is near the heights it read
    assert "100.0" not in json.dumps(p) and "101" not in json.dumps(p)
