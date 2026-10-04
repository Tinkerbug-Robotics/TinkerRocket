#!/usr/bin/env python3
"""Rebuild the rig's IQ files (c8/*.C8, never in git) byte for byte, or check that their recipes still do.

c8_manifest.json lists every IQ file the bench work kept, with how it was made, its size and its SHA-256:

  gps-sdr-sim  a patched build from bench-backups (its README), run from c8/ on a scenario CSV with a fixed gain
  signalsim    IFdataGen (a local build, never in git: signalsim/README.md) on signalsim/configs/<config>.json, then
               signalsim/carrier_shift.py when the file carries the HackRF's carrier correction (the _cofs files)

Both generators are deterministic, so the same binary and inputs give the same bytes, and a short prefix proves a
recipe without the hours and gigabytes of a full rebuild.

    ./regen_c8.py list                     every file in the manifest: on disk or not, size, recipe
    ./regen_c8.py check [NAME ...] [-s S]  rebuild the first S seconds (default 3) in a scratch folder and compare
                                           them with the start of the file on disk; with no NAME, every file on disk
    ./regen_c8.py build NAME ...           rebuild the whole file into c8/ and verify its SHA-256 (refuses to
                                           overwrite)
    ./regen_c8.py hash [NAME ...]          hash the files on disk against the manifest

Inputs are made when missing: scenarios/*.csv (make_flights.py + pad_scenario.py, which give them byte for byte),
the GPS ephemeris in c8/ and the multi-GNSS one in signalsim/EphData/ (both gunzipped from results/). Binaries come
from bench-backups unless GPS_SDR_SIM_DIR (the folder of builds) or SIGNALSIM (the IFdataGen path) says otherwise;
C8_DIR points the tool at a c8/ folder elsewhere (another checkout's).
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
C8 = Path(os.environ.get("C8_DIR") or HERE / "c8")
SCEN = HERE / "scenarios"
SIG = HERE / "signalsim"
MANIFEST = json.loads((HERE / "c8_manifest.json").read_text())
GPS_SDR_SIM_DIR = Path(os.environ.get("GPS_SDR_SIM_DIR") or os.path.expanduser(MANIFEST["gps_sdr_sim_dir"]))
SIGNALSIM = Path(os.environ.get("SIGNALSIM") or os.path.expanduser(MANIFEST["signalsim"]))
GPS_EPH, GNSS_EPH = "BRDC_2026230.rx2.n", "BRDC_2026230_MN.rnx"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(1 << 24):
            h.update(chunk)
    return h.hexdigest()


def gunzip_to(src: Path, dst: Path):
    if dst.exists() and dst.stat().st_size:
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(src, "rb") as fi, open(dst, "wb") as fo:
        shutil.copyfileobj(fi, fo)


def ensure_scenario(name: str):
    """scenarios/NAME.csv from make_flights.py + pad_scenario.py, checked against the manifest's hash."""
    spec = MANIFEST["scenarios"][name]
    csv = SCEN / f"{name}.csv"
    if not csv.exists():
        print(f"# making scenarios/{name}.csv")
        subprocess.run([sys.executable, str(HERE / "make_flights.py"), *spec["make_flights"], "-o", str(SCEN)],
                       check=True, stdout=subprocess.DEVNULL)
        subprocess.run([sys.executable, str(HERE / "pad_scenario.py"), *spec["pad"]], check=True,
                       stdout=subprocess.DEVNULL)
    got = sha256(csv)
    if got != spec["sha256"]:
        sys.exit(f"!! scenarios/{name}.csv differs from the one the IQ files were made from ({got[:12]} vs "
                 f"{spec['sha256'][:12]}); regenerate it with: make_flights.py {' '.join(spec['make_flights'])}, "
                 f"then pad_scenario.py {' '.join(spec['pad'])}")


def recipe(name: str) -> str:
    e = MANIFEST["files"][name]
    if e["tool"] == "gps-sdr-sim":
        return (f"cd c8 && {e['binary']} -e {GPS_EPH} -x ../scenarios/{e['scenario']}.csv -b 8 -s {e['rate']} "
                f"-t 2026/08/18,08:30:00 -p {e['gain']} -d {e['seconds']} -o {name}")
    text = f"signalsim/gen.sh {e['config']}"
    if "carrier_shift_hz" in e:
        text += (f" && signalsim/carrier_shift.py c8/signalsim_{e['config']}.C8 c8/{name} {e['rate']} "
                 f"{e['carrier_shift_hz']}")
    return text


def run_gps_sdr_sim(e: dict, workdir: Path, out: str, seconds: float):
    """gps-sdr-sim from workdir, which gets the ephemeris and a link to the scenario: gps-sdr-sim copies -e, -x and
    -o into 100-byte buffers, so every path it sees is short and relative."""
    binary = GPS_SDR_SIM_DIR / e["binary"]
    if not binary.exists():
        sys.exit(f"!! {binary} not found (set GPS_SDR_SIM_DIR)")
    ensure_scenario(e["scenario"])
    gunzip_to(HERE / "results" / f"{GPS_EPH}.gz", workdir / GPS_EPH)
    motion = workdir / "_motion.csv"
    if motion.is_symlink() or motion.exists():
        motion.unlink()
    motion.symlink_to(SCEN / f"{e['scenario']}.csv")
    cmd = [str(binary), "-e", GPS_EPH, "-x", motion.name, "-b", "8", "-s", str(e["rate"]),
           "-t", "2026/08/18,08:30:00", "-p", str(e["gain"]), "-d", str(seconds), "-o", out]
    try:
        subprocess.run(cmd, cwd=workdir, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    finally:
        motion.unlink()


def run_signalsim(config: dict, workdir: Path, log: Path):
    """IFdataGen on one config, from workdir (which holds EphData/ and c8/): its paths are short and relative,
    because SignalSim truncates long JSON strings and reads the file in 255-byte lines."""
    if not SIGNALSIM.exists():
        sys.exit(f"!! {SIGNALSIM} not found (set SIGNALSIM; see signalsim/README.md)")
    gunzip_to(HERE / "results" / f"{GNSS_EPH}.gz", workdir / "EphData" / GNSS_EPH)
    (workdir / "run.json").write_text(json.dumps(config, indent="\t") + "\n")
    with open(log, "w") as fh:
        subprocess.run([str(SIGNALSIM), "-c", "run.json"], cwd=workdir, check=True, stdout=fh,
                       stderr=subprocess.STDOUT)


def carrier_shift(src: Path, dst: Path, rate: int, hz: float):
    subprocess.run([sys.executable, str(SIG / "carrier_shift.py"), str(src), str(dst), str(rate), str(hz)],
                   check=True, stdout=subprocess.DEVNULL)


def transmitter_running() -> bool:
    r = subprocess.run(["pgrep", "-f", "hackrf_tx_ram -t|hackrf_transfer"], capture_output=True)
    return r.returncode == 0


def cmd_list(_args):
    total_on, total_off = 0, 0
    for name, e in MANIFEST["files"].items():
        on = (C8 / name).is_file()
        total_on += e["bytes"] if on else 0
        total_off += 0 if on else e["bytes"]
        print(f"{'on disk' if on else 'missing':8s} {e['bytes'] / 1e9:6.1f} GB  {name}")
        print(f"{'':25s}{recipe(name)}")
    print(f"# on disk {total_on / 1e9:.1f} GB, rebuildable but not on disk {total_off / 1e9:.1f} GB")


def cmd_check(args):
    names = args.names or [n for n in MANIFEST["files"] if (C8 / n).is_file()]
    bad = 0
    for name in names:
        e = MANIFEST["files"][name]
        have = C8 / name
        if not have.is_file():
            print(f"-- {name}: not on disk, nothing to compare; recipe: {recipe(name)}")
            continue
        with tempfile.TemporaryDirectory(prefix="regen_c8_") as tmp:
            work = Path(tmp) / "c8"
            work.mkdir()
            trim = 0
            if e["tool"] == "gps-sdr-sim":
                run_gps_sdr_sim(e, work, "check.C8", int(args.seconds))
                made = work / "check.C8"
                # The smooth-carrier build sweeps each 0.1 s block toward the next motion row, so the last block of
                # a short rebuild ends unlike the same block inside the full file (2026-10-04: identical to 0.1 s
                # before the end, 3442 bytes differ after). Leave the last 0.2 s out of the comparison.
                trim = 2 * int(0.2 * e["rate"])
            else:
                cfg = json.loads((SIG / "configs" / f"{e['config']}.json").read_text())
                first = cfg["trajectory"]["trajectoryList"][0]
                if first["type"] != "Const" or first["time"] < args.seconds:
                    sys.exit(f"!! {name}: the config does not open with {args.seconds} s of Const; pick fewer seconds")
                cfg["trajectory"]["trajectoryList"] = [{"type": "Const", "time": float(args.seconds)}]
                cfg["output"]["name"] = "c8/check.C8"
                run_signalsim(cfg, Path(tmp), Path(tmp) / "gen.log")
                made = work / "check.C8"
                if "carrier_shift_hz" in e:
                    carrier_shift(made, work / "check_cofs.C8", e["rate"], e["carrier_shift_hz"])
                    made = work / "check_cofs.C8"
            n = made.stat().st_size - trim
            with open(made, "rb") as a, open(have, "rb") as b:
                same = a.read(n) == b.read(n)
        secs = n / 2 / e["rate"]
        print(f"{'ok' if same else 'MISMATCH':8s} {name}: first {n:,} bytes ({secs:.3f} s) "
              f"{'identical to' if same else 'DIFFER from'} the file on disk")
        bad += not same
    return 1 if bad else 0


def cmd_build(args):
    if transmitter_running():
        sys.exit("!! a transmitter is running -- not generating (CPU load starves it)")
    for name in args.names:
        e = MANIFEST["files"][name]
        dst = C8 / name
        if dst.exists():
            print(f"-- {name} is already on disk; not overwriting")
            continue
        C8.mkdir(exist_ok=True)
        print(f"# building {name}: {recipe(name)}")
        if e["tool"] == "gps-sdr-sim":
            run_gps_sdr_sim(e, C8, name, e["seconds"])
        else:
            cfg = json.loads((SIG / "configs" / f"{e['config']}.json").read_text())
            raw = Path(cfg["output"]["name"]).name
            if "carrier_shift_hz" in e:
                raw = f"_raw_{name}"              # the uncorrected file is only an intermediate here
            cfg["output"]["name"] = f"c8/{raw}"
            (SIG / "logs").mkdir(exist_ok=True)
            with tempfile.TemporaryDirectory(prefix="regen_c8_") as tmp:
                (Path(tmp) / "c8").symlink_to(C8.resolve())
                run_signalsim(cfg, Path(tmp), SIG / "logs" / f"gen_{e['config']}.log")
            if "carrier_shift_hz" in e:
                carrier_shift(C8 / raw, dst, e["rate"], e["carrier_shift_hz"])
                print(f"# the uncorrected intermediate {C8 / raw} can go to the Trash")
        got = sha256(dst)
        print(f"{'ok' if got == e['sha256'] else 'MISMATCH':8s} {name}: sha256 {got[:16]}... "
              f"{'matches' if got == e['sha256'] else 'differs from'} the manifest")


def cmd_hash(args):
    names = args.names or [n for n in MANIFEST["files"] if (C8 / n).is_file()]
    for name in names:
        got = sha256(C8 / name)
        ok = got == MANIFEST["files"][name]["sha256"]
        print(f"{'ok' if ok else 'MISMATCH':8s} {name}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    p = sub.add_parser("check")
    p.add_argument("names", nargs="*")
    p.add_argument("-s", "--seconds", type=float, default=3.0)
    p = sub.add_parser("build")
    p.add_argument("names", nargs="+")
    p = sub.add_parser("hash")
    p.add_argument("names", nargs="*")
    args = ap.parse_args()
    for n in getattr(args, "names", []) or []:
        if n not in MANIFEST["files"]:
            ap.error(f"{n} is not in c8_manifest.json")
    return {"list": cmd_list, "check": cmd_check, "build": cmd_build, "hash": cmd_hash}[args.cmd](args) or 0


if __name__ == "__main__":
    sys.exit(main())
