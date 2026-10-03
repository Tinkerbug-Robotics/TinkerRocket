#!/usr/bin/env python3
"""Check the front-end emulation on a rig IQ file: acquire GPS L1 C/A on the native
file and on iqtool's 2-bit 6.75 MS/s output of the same segment, and compare Doppler
and C/N0 satellite by satellite.

    emul_check.py FILE [--start S] [--dur S] [--cn0 DBHZ] [--prn 1-32] [--mode direct|adc27]
                  [--emul OUT]   (reuse an existing iqtool output instead of running it)

Needs GNSS_IQ_DIR (or a path) and a built iqtool (build/host/iqtool).
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gnssrx import acq, codes, iqio  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
IQTOOL = ROOT / "build" / "host" / "iqtool"


def prn_list(s: str) -> list[int]:
    out = []
    for part in s.split(","):
        if "-" in part:
            a, b = part.split("-")
            out += range(int(a), int(b) + 1)
        else:
            out.append(int(part))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--start", type=float, default=5.0)
    ap.add_argument("--dur", type=float, default=0.5)
    ap.add_argument("--cn0", type=float, help="emulate with noise added to this C/N0")
    ap.add_argument("--prn", default="1-32")
    ap.add_argument("--mode", default="direct")
    ap.add_argument("--emul", help="existing iqtool output covering the segment")
    ap.add_argument("--min-metric", type=float, default=6.0)
    a = ap.parse_args()
    codes.check()

    name = Path(a.file).name
    meta = iqio.manifest(name)
    path = iqio.iq_path(a.file)
    prns = prn_list(a.prn)

    # Native: the file with its fixes, L1 at 0 Hz (noise added only on the emulated side).
    z = iqio.read_c8(path, meta.fs, a.start, a.dur, meta.dc)
    shift = (meta.fc - iqio.L1_HZ) + meta.carrier_fix_hz
    if shift:
        z = z * np.exp(2j * np.pi * shift * (np.arange(z.size) / meta.fs + a.start)).astype(np.complex64)

    # Emulated.
    if a.emul:
        out = Path(a.emul)
    else:
        tmp = Path(tempfile.mkdtemp(prefix="emul_check_"))
        out = tmp / "seg.u2"
        cmd = [str(IQTOOL), "emul", str(path), "-o", str(out), "--start", str(a.start), "--dur", str(a.dur),
               "--mode", a.mode]
        if a.cn0 is not None:
            cmd += ["--cn0", str(a.cn0)]
        subprocess.run(cmd, check=True)
    e, m = iqio.read_stream(out)
    fs_e, if_e = float(m["fs"]), float(m["if_hz"])

    print(f"{name}: {a.start:.1f}+{a.dur:.1f} s; native {meta.fs/1e6:.3f} MS/s; emulated {fs_e/1e6:.3f} MS/s "
          f"IF {if_e/1e6:+.3f} MHz ({m['mode']}, density {m.get('mag_density', '-')})"
          + (f", noise to {a.cn0:.1f} dB-Hz" if a.cn0 is not None else ""))
    na = acq.acquire(z, meta.fs, 0.0, prns)
    ea = acq.acquire(e, fs_e, if_e, prns)
    print(" PRN | native: metric   dop    C/N0 (off/mom)  C (LSB^2) | emulated: metric   dop    C/N0 (off/mom) | dC/N0")
    rows = []
    for p in prns:
        if na[p].metric < a.min_metric and ea[p].metric < a.min_metric:
            continue
        nf = acq.refine(z, meta.fs, 0.0, na[p], nms=min(400, int(a.dur * 1000) - 25))
        ef = acq.refine(e, fs_e, if_e, ea[p], nms=min(400, int(a.dur * 1000) - 25))
        rows.append((nf, ef))
        print(f" {p:3d} | {na[p].metric:12.1f} {nf.dop:+8.1f} {nf.cn0_offpeak:5.1f}/{nf.cn0_moments:5.1f} "
              f"{nf.sig_power:9.4f} | {ea[p].metric:14.1f} {ef.dop:+8.1f} {ef.cn0_offpeak:5.1f}/{ef.cn0_moments:5.1f}"
              f" | {ef.cn0_offpeak - nf.cn0_offpeak:+5.2f}")
    if rows:
        d = np.array([ef.cn0_offpeak - nf.cn0_offpeak for nf, ef in rows])
        dd = np.array([ef.dop - nf.dop for nf, ef in rows])
        c = np.array([nf.sig_power for nf, _ in rows])
        n0 = np.array([nf.noise_density for nf, _ in rows])
        cn = np.array([nf.cn0_offpeak for nf, _ in rows])
        print(f"{len(rows)} satellites: native C/N0 {cn.mean():.2f} (sd {cn.std():.2f}); emulated - native "
              f"{d.mean():+.2f} dB (sd {d.std():.2f}); Doppler difference {dd.mean():+.2f} Hz (sd {dd.std():.2f})")
        print(f"native per-satellite C {c.mean():.4f} LSB^2 (sd {c.std():.4f}); N0 {n0.mean():.4e} LSB^2/Hz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
