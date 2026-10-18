"""tools/raw_replay/sim_grid.py: an events file already made is reused only for the truth's antenna and spinning body
it was made with.

sim_grid names an events file by flight and pattern alone (run_one splits the name on "__"), so a grid run with
another --lever-arm, --lever-arm-b or roll would otherwise score its runs against the old file's truth. The rule is
events_match (the file's META lever_arm_m, lever_arm_b_m, sim_roll_* and sim_windup_k against the ones asked for);
make_events reuses a file only when it holds. No sim runs here: sim_events is replaced by a stand-in that records
what it was asked to build.
"""
from __future__ import annotations

import json
import sys
import types
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tools" / "raw_replay"))

import sim_grid  # noqa: E402


def events_file(path, lever_arm="0.3", lever_b=None, extra=()):
    lines = ["# sim_events.py: f_s1, pattern none; flight time = MCU s + t_base_s", "META t_base_s -40.0",
             "META flight f_s1", "META pattern none"]
    if lever_arm is not None:
        lines.append(f"META lever_arm_m {lever_arm}")
    lines.extend(extra)
    if lever_b is not None:
        lines.append(f"META lever_arm_b_m {lever_b}")
    lines.append("META sim_epochs 686")
    lines.append("I 40000000 0.0 0.0 -9.8 0.0 0.0 0.0 1")
    path.write_text("\n".join(lines) + "\n")
    return str(path)


def test_along_the_nose(tmp_path):
    p = events_file(tmp_path / "f_s1__none.events", lever_arm="0.3")
    assert sim_grid.events_match(p, 0.3, None)
    assert not sim_grid.events_match(p, 0.1, None)
    assert not sim_grid.events_match(p, 0.3, [0.3, 0.0, 0.0])      # asked for anywhere in the body: not this file


def test_anywhere_in_the_body(tmp_path):
    p = events_file(tmp_path / "f_s1__none.events", lever_arm="0.1", lever_b="0.1 0.05 -0.02")
    assert sim_grid.events_match(p, 0.3, [0.1, 0.05, -0.02])       # --lever-arm is not used with --lever-arm-b
    assert not sim_grid.events_match(p, 0.3, [0.1, -0.02, 0.05])
    assert not sim_grid.events_match(p, 0.1, None)                 # asked for along the nose: not this file


def test_a_file_that_does_not_say_is_made_again(tmp_path):
    assert not sim_grid.events_match(events_file(tmp_path / "old.events", lever_arm=None), 0.3, None)
    # the repr of a numpy scalar, as sim_events wrote lever_arm_b_m before it wrote plain floats
    p = events_file(tmp_path / "np.events", lever_arm="0.1",
                    lever_b="np.float64(0.1) np.float64(0.05) np.float64(-0.02)")
    assert not sim_grid.events_match(p, 0.3, [0.1, 0.05, -0.02])


ROLLED = ("META sim_roll_profile const:5", "META sim_roll_peak_rev_s 5.0000", "META sim_gyro_clip_dps 4587.0",
          "META sim_gyro_clipped_s 0.0000", "META sim_windup_k 0.99")
SPIN5 = dict(roll="const:5", windup_k=0.99, gyro_clip_dps=4587.0)


def test_a_rolled_file_is_reused_only_for_its_own_roll(tmp_path):
    # the roll (sim_events.py --roll, --windup-k, --gyro-clip-dps, --gyro-scale-ppm, --roll-axis-tilt-deg) is the
    # truth's too: a file made with one is not a no-roll run's, nor another profile's, rail, scale or tilt
    p = events_file(tmp_path / "f_s1__none.events", lever_arm="0.3", extra=ROLLED)
    assert sim_grid.events_match(p, 0.3, None, SPIN5)
    assert sim_grid.events_match(p, 0.3, None, dict(SPIN5, gyro_scale_ppm=0.0, roll_axis_tilt_deg=0.0))
    assert not sim_grid.events_match(p, 0.3, None)
    assert not sim_grid.events_match(p, 0.3, None, dict(SPIN5, roll="const:10"))
    assert not sim_grid.events_match(p, 0.3, None, dict(SPIN5, windup_k=0.0))
    assert not sim_grid.events_match(p, 0.3, None, dict(SPIN5, gyro_clip_dps=None))
    assert not sim_grid.events_match(p, 0.3, None, dict(SPIN5, roll_axis_tilt_deg=1.0))
    tilted = events_file(tmp_path / "tilt.events", lever_arm="0.3",
                         extra=ROLLED + ("META sim_roll_axis_tilt_deg 1.0",))
    assert sim_grid.events_match(tilted, 0.3, None, dict(SPIN5, roll_axis_tilt_deg=1.0))
    assert not sim_grid.events_match(tilted, 0.3, None, SPIN5)
    # and a file with no roll is not a rolled run's; the wind-up alone counts too
    plain = events_file(tmp_path / "plain.events", lever_arm="0.3")
    assert sim_grid.events_match(plain, 0.3, None, dict(roll=None, windup_k=0.0))
    assert not sim_grid.events_match(plain, 0.3, None, SPIN5)
    assert not sim_grid.events_match(plain, 0.3, None, dict(windup_k=0.99))


def fake_sim_events(calls):
    def build(flight, pattern, out_dir, phases, lever_arm=0.3, lever_b=None):
        calls.append((pattern, lever_arm, lever_b))
        return str(Path(out_dir) / f"{Path(flight).stem}__{pattern}.events"), {"sim_epochs": 1}
    return types.SimpleNamespace(build=build)


def test_make_events_reuses_only_a_matching_file(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setitem(sys.modules, "sim_events", fake_sim_events(calls))
    phases = tmp_path / "truth_phases.json"
    phases.write_text(json.dumps({"flights": {"f_s1": {}}}))
    ev = tmp_path / "events"
    ev.mkdir()
    path = events_file(ev / "f_s1__none.events", lever_arm="0.3")

    def job(lever_arm, lever_b=None, force=False):
        return ("flights/f_s1.pkl", "none", str(ev), str(phases), lever_arm, force, None, lever_b)

    assert sim_grid.make_events(job(0.3)) == (path, None, 0.0)     # the same antenna: kept
    assert calls == []
    out, counts, _ = sim_grid.make_events(job(0.1))                # another along the nose: made again
    assert out == path and counts == {"sim_epochs": 1}
    assert calls == [("none", 0.1, None)]
    sim_grid.make_events(job(0.3, [0.3, 0.0, 0.0]))                # anywhere in the body: made again
    assert calls[-1] == ("none", 0.3, [0.3, 0.0, 0.0])
    sim_grid.make_events(job(0.3, force=True))                     # --force-events: made again regardless
    assert len(calls) == 3


def test_make_events_builds_the_roll_it_was_asked_for(tmp_path, monkeypatch):
    calls = []

    def build(flight, pattern, out_dir, phases, lever_arm=0.3, lever_b=None, **spin):
        calls.append((pattern, lever_arm, lever_b, spin))
        return str(Path(out_dir) / f"{Path(flight).stem}__{pattern}.events"), {"sim_epochs": 1}

    monkeypatch.setitem(sys.modules, "sim_events", types.SimpleNamespace(build=build))
    phases = tmp_path / "truth_phases.json"
    phases.write_text(json.dumps({"flights": {"f_s1": {}}}))
    ev = tmp_path / "events"
    ev.mkdir()
    path = events_file(ev / "f_s1__none.events", lever_arm="0.3", extra=ROLLED)
    job = ("flights/f_s1.pkl", "none", str(ev), str(phases), 0.3, False, None, None)
    assert sim_grid.make_events(job + (SPIN5,)) == (path, None, 0.0)     # the same roll: kept
    assert calls == []
    sim_grid.make_events(job)                                             # no roll asked: made again, without one
    assert calls[-1] == ("none", 0.3, None, {})
    spin10 = dict(SPIN5, roll="const:10")
    sim_grid.make_events(job + (spin10,))                                 # another profile: made again with it
    assert calls[-1] == ("none", 0.3, None, spin10)


def test_a_score_older_than_its_events_file_is_run_again(tmp_path):
    # rerunning the same OUT with another antenna rebuilds the events file; the runs scored on the old one go with it
    import os
    ev = events_file(tmp_path / "f_s1__none.events")
    score = tmp_path / "f_s1__none__0__design.json"
    score.write_text("{}")
    os.utime(ev, (1_000, 1_000))
    os.utime(score, (2_000, 2_000))
    assert sim_grid.score_current(str(score), ev)
    os.utime(ev, (3_000, 3_000))                                   # the events file was made again
    assert not sim_grid.score_current(str(score), ev)
    assert not sim_grid.score_current(str(tmp_path / "missing.json"), ev)
