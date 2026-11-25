#!/usr/bin/env python3
"""Tonight's RF-rig runs (2026-10-15, fc/gnss-heading-fuse dc37cf07, rf1015/build_rig.sh): for each, the on-board
score, the harness's score with the same configuration and their difference (onboard_vs_harness.py), the apogee
voters on board (voters.py), and every configuration on its log (run_matrix.py, matrix_tonight/TAG).
Runs can be added one at a time: the ones given before are kept (tonight_logs.json).

    tonight.py TAG=OC_LOG [TAG=OC_LOG ...]      (TAG: f3c, c30, f4b, f4a, f2, c30b)
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
A = ["--heading-mode", "2", "--gate-full", "1", "--tilt-sigma-deg", "0.6", "--pad-sigma-deg", "30", "--handoff-deg",
     "-1", "--carrier", "1"]
C = ["--heading-mode", "2", "--gate-full", "0", "--tilt-sigma-deg", "0.6", "--pad-sigma-deg", "30", "--handoff-deg",
     "-1", "--carrier", "1"]
RUNS = {   # tag: (harness flags as built, the bench heading error, note)
    "f3c": (A, 10.0, "A: act, full gate, 0.6 deg tilt floor, 30 deg prior, no hand-off, carrier observed; "
                     "magnetometer +10"),
    "c30": (C, 10.0, "C: the same with the heading-only gate; magnetometer +10"),
    "f4b": (A, 40.0, "A with the magnetometer +40"),
    "f4a": (A, 0.0, "A with no magnetometer error"),
    "f2": (["--heading-mode", "2", "--pad-sigma-deg", "10", "--handoff-deg", "-1"], 10.0,
           "act, heading-only gate, 10 deg prior, no hand-off (r23's settings on dc37cf07); magnetometer +10"),
    "c30b": (C, 10.0, "C flown again after the chain (c30's receiver lost the sky); magnetometer +10"),
}
PY = sys.executable
if os.environ.get("TONIGHT_TEST"):   # a dry run of the pipeline on an old log under the tag "zz" (A, +0)
    RUNS["zz"] = (A, 0.0, "pipeline test on an old log")


def main():
    reg_path = os.environ.get("TONIGHT_REG", os.path.join(HERE, "tonight_logs.json"))
    reg = json.load(open(reg_path)) if os.path.exists(reg_path) else {}
    new = []
    for arg in sys.argv[1:]:
        tag, log = arg.split("=", 1)
        reg[tag] = os.path.abspath(log)
        new.append(tag)
    json.dump(reg, open(reg_path, "w"), indent=1)
    spec, extra = [], {}
    for tag, log in reg.items():
        flags, err, note = RUNS[tag]
        ev = os.path.join(HERE, "in", tag + ".txt")
        if tag in new or not os.path.exists(ev):
            subprocess.run([PY, os.path.join(HERE, "extract.py"), log, ev], check=True)
        spec.append({"run": tag, "log": log, "events": ev, "binary": os.path.join(HERE, "build", "raw_replay"),
                     "args": flags + ["--mag-error-deg", str(err), "--recorded-mag-error-deg", str(err)],
                     "note": note + " (dc37cf07, as recorded)"})
        extra[tag] = [log, "act", err, True]
    json.dump(spec, open(os.path.join(HERE, "tonight_spec.json"), "w"), indent=1)
    json.dump(extra, open(os.path.join(HERE, "tonight_extra.json"), "w"), indent=1)
    subprocess.run([PY, os.path.join(HERE, "voters.py")] + [f"{t}={reg[t]}" for t in new], check=True, cwd=HERE)
    subprocess.run([PY, os.path.join(HERE, "onboard_vs_harness.py"), os.path.join(HERE, "tonight_spec.json"),
                    os.path.join(HERE, "val", "tonight_onboard_vs_harness.md")], check=True)
    for tag in new:
        subprocess.run([PY, os.path.join(HERE, "run_matrix.py"), os.path.join(HERE, "matrix_tonight", tag),
                        "--logs", tag, "--extra", os.path.join(HERE, "tonight_extra.json")], check=True)
    subprocess.run([PY, os.path.join(HERE, "tonight_summary.py")] + list(reg), check=True)


if __name__ == "__main__":
    main()
