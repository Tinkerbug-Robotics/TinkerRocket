"""Readers for the rig's IQ files, the manifest, and iqtool's output streams."""
from __future__ import annotations

import configparser
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DATA = HERE.parent.parent / "data"
L1_HZ = 1575.42e6

# u2 nibble: I sign, I magnitude, Q sign, Q magnitude (fpga/model/fe_format.h)
_W_SMALL, _W_LARGE = 1.0, 3.0


def _nibble_lut() -> np.ndarray:
    lut = np.empty(16, np.complex64)
    for c in range(16):
        i = (_W_LARGE if c & 0x4 else _W_SMALL) * (-1 if c & 0x8 else 1)
        q = (_W_LARGE if c & 0x1 else _W_SMALL) * (-1 if c & 0x2 else 1)
        lut[c] = i + 1j * q
    return lut


_NIB = _nibble_lut()


@dataclass
class Meta:
    name: str
    fs: float
    fc: float
    dc: tuple[float, float] | None  # None = auto
    carrier_fix_hz: float
    sig_power: float
    noise_density: float
    noise: bool
    truth: str
    start_gpst: str
    tropo: str = "saastamoinen"  # "none" where the generator applied no troposphere (gps-sdr-sim)
    tropo_top_m: float = 4e4  # where the generator's troposphere stops (SignalSim: 10 km, at once)
    generator: str = ""  # signalsim, gps-sdr-sim, ...: what made the file


def manifest(name: str, path: Path | None = None) -> Meta:
    cp = configparser.ConfigParser(inline_comment_prefixes=(" ;",), interpolation=None)
    cp.read(path or os.environ.get("GNSS_MANIFEST") or DATA / "iq_files.ini")
    s = cp[name]
    dc = s.get("dc", "0,0")
    dcv = None if dc == "auto" else tuple(float(v) for v in (dc.split(",") * 2)[:2])
    return Meta(name, float(s["fs"]), float(s.get("fc", L1_HZ)), dcv, float(s.get("carrier_fix_hz", 0)),
                float(s.get("sig_power", 0)), float(s.get("noise_density", 0)), s.get("noise", "no") == "yes",
                s.get("truth", ""), s.get("start_gpst", ""), s.get("tropo", "saastamoinen").lower(),
                float(s.get("tropo_top_m", 4e4)), s.get("generator", "").lower())


def traj_motion(source: str) -> str:
    """The motion a file's trajectory has in its signal (truth.Trajectory's motion): "signalsim" for SignalSim's
    files, else "central"."""
    try:
        return "signalsim" if manifest(Path(source).name).generator == "signalsim" else "central"
    except KeyError:
        return "central"


def iq_path(name: str) -> Path:
    p = Path(name)
    if p.exists():
        return p
    d = os.environ.get("GNSS_IQ_DIR")
    if d and (Path(d) / name).exists():
        return Path(d) / name
    raise FileNotFoundError(f"{name}: not found (set GNSS_IQ_DIR)")


def read_c8(path, fs: float, start_s: float, dur_s: float, dc=(0.0, 0.0)) -> np.ndarray:
    """int8 I,Q -> complex64, with DC removed (dc=None: measured on the segment)."""
    n = int(round(dur_s * fs))
    off = int(round(start_s * fs)) * 2
    x = np.fromfile(path, dtype=np.int8, count=2 * n, offset=off).astype(np.float32)
    z = x[0::2] + 1j * x[1::2]
    if dc is None:
        z -= z.mean()
    else:
        z -= dc[0] + 1j * dc[1]
    return z.astype(np.complex64)


def stream_meta(path) -> dict:
    cp = configparser.ConfigParser(interpolation=None)
    cp.read(str(path) + ".ini")
    return {k: v for k, v in cp["stream"].items()}


def read_stream(path, start: int = 0, count: int | None = None) -> tuple[np.ndarray, dict]:
    """An iqtool emul output (u2, cs8 or cf32) as complex64, with its metadata."""
    m = stream_meta(path)
    fmt = m["format"]
    if fmt == "u2":
        b0 = start // 2
        nb = -1 if count is None else (count + (start & 1) + 1) // 2
        raw = np.fromfile(path, dtype=np.uint8, count=nb, offset=b0)
        codes = np.empty(2 * raw.size, np.uint8)
        codes[0::2] = raw >> 4
        codes[1::2] = raw & 0xF
        codes = codes[start & 1:]
        if count is not None:
            codes = codes[:count]
        z = _NIB[codes]
    elif fmt == "cs8":
        nb = -1 if count is None else 2 * count
        x = np.fromfile(path, dtype=np.int8, count=nb, offset=2 * start).astype(np.float32)
        z = (x[0::2] + 1j * x[1::2]).astype(np.complex64)
    else:
        nb = -1 if count is None else 2 * count
        x = np.fromfile(path, dtype=np.float32, count=nb, offset=8 * start)
        z = (x[0::2] + 1j * x[1::2]).astype(np.complex64)
    return z, m
