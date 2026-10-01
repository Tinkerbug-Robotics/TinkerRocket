"""Reference GPS L1 C/A acquisition and C/N0 measurement (numpy), to check the C code
and to calibrate the IQ files.

Conventions: z is complex baseband at rate fs with L1 at f_if. A satellite's carrier is
at f_if + dop. tau is the time of a code epoch (chip 0) after the first sample, in
[0, 1 ms).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .codes import CHIP_RATE, CODE_LEN, ca_code

L1_HZ = 1575.42e6


@dataclass
class Acq:
    prn: int
    metric: float     # peak / median of the search grid (non-coherent)
    dop: float        # Hz
    tau: float        # s


@dataclass
class Fine:
    prn: int
    dop: float        # Hz
    tau: float        # s
    cn0_offpeak: float  # dB-Hz, noise from correlations far from the peak
    cn0_moments: float  # dB-Hz, M2/M4 moments of the prompt
    sig_power: float    # C per sample, input units^2
    noise_density: float  # N0, input units^2 per Hz


def _replica(prn: int, fs: float, n: int, tau: float, dop: float, t0: float = 0.0) -> np.ndarray:
    """+-1 code at sample times t0 + k/fs, code epoch at tau, chip rate stretched by the code Doppler."""
    t = t0 + np.arange(n) / fs
    rate = CHIP_RATE * (1.0 + dop / L1_HZ)
    chips = np.floor((t - tau) * rate).astype(np.int64) % CODE_LEN
    return ca_code(prn)[chips]


def acquire(z: np.ndarray, fs: float, f_if: float, prns, dop_max: float = 6000.0, dop_step: float = 250.0,
            ncoh: int = 10) -> dict[int, Acq]:
    """FFT search over code phase x Doppler: 1 ms coherent, ncoh ms non-coherent."""
    n = int(round(fs * 1e-3))
    if z.size < n * ncoh:
        raise ValueError("segment shorter than the integration")
    t = np.arange(n * ncoh) / fs
    out = {}
    dops = np.arange(-dop_max, dop_max + 0.5 * dop_step, dop_step)
    for prn in prns:
        cf = np.conj(np.fft.fft(_replica(prn, fs, n, 0.0, 0.0)))
        best = (0.0, 0.0, 0)
        grid = np.empty((dops.size, n))
        for i, d in enumerate(dops):
            y = (z[:n * ncoh] * np.exp(-2j * np.pi * (f_if + d) * t)).reshape(ncoh, n)
            acc = (np.abs(np.fft.ifft(np.fft.fft(y, axis=1) * cf, axis=1)) ** 2).sum(axis=0)
            grid[i] = acc
            m = int(acc.argmax())
            if acc[m] > best[0]:
                best = (float(acc[m]), float(d), m)
        out[prn] = Acq(prn, best[0] / float(np.median(grid)), best[1], best[2] / fs)
    return out


def _prompts(z, fs, f_if, prn, dop, tau, nms, code_offset_chips=0.0):
    """1 ms prompt correlations aligned to code epochs, from the first epoch at tau."""
    n = int(round(fs * 1e-3))
    first = int(np.ceil(tau * fs))
    tot = n * nms
    seg = z[first:first + tot]
    if seg.size < tot:
        raise ValueError("segment too short for the prompt sequence")
    t = (first + np.arange(tot)) / fs
    rep = _replica(prn, fs, tot, tau + code_offset_chips / CHIP_RATE, dop, t0=first / fs)
    y = seg * rep * np.exp(-2j * np.pi * (f_if + dop) * t)
    return y.reshape(nms, n).sum(axis=1), n


def refine(z: np.ndarray, fs: float, f_if: float, a: Acq, nms: int = 200) -> Fine:
    """Fine code phase (1/16 sample) and Doppler (squared-prompt spectrum), then C/N0 two ways."""
    # Code phase: non-coherent prompt power over 20 ms at sub-sample delays.
    tau = a.tau
    best = (-1.0, tau)
    for d in np.arange(-1.5, 1.5001, 1.0 / 16):
        tt = (tau + d / fs) % 1e-3
        p, _ = _prompts(z, fs, f_if, a.prn, a.dop, tt, 20)
        s = float(np.mean(np.abs(p) ** 2))
        if s > best[0]:
            best = (s, tt)
    tau = best[1]
    # Doppler: the squared prompt removes the data bits; its spectrum peaks at 2 * error.
    dop = a.dop
    for span in (40, nms):
        p, _ = _prompts(z, fs, f_if, a.prn, dop, tau, span)
        sq = p ** 2
        nfft = 16 * span
        spec = np.abs(np.fft.fft(sq, nfft))
        f = np.fft.fftfreq(nfft, d=1e-3)
        dop += 0.5 * f[int(spec.argmax())]
    # Re-centre the code phase at the final Doppler.
    best = (-1.0, tau)
    for d in np.arange(-0.5, 0.5001, 1.0 / 32):
        tt = (tau + d / fs) % 1e-3
        p, _ = _prompts(z, fs, f_if, a.prn, dop, tt, 20)
        s = float(np.mean(np.abs(p) ** 2))
        if s > best[0]:
            best = (s, tt)
    tau = best[1]
    p, n = _prompts(z, fs, f_if, a.prn, dop, tau, nms)
    off = np.concatenate([_prompts(z, fs, f_if, a.prn, dop, tau, nms, c)[0] for c in (311.0, 517.0, 733.0)])
    m2 = float(np.mean(np.abs(p) ** 2))
    m4 = float(np.mean(np.abs(p) ** 4))
    pn_off = float(np.mean(np.abs(off) ** 2))
    pd_off = m2 - pn_off
    pd_mom = np.sqrt(max(2 * m2 * m2 - m4, 0.0))
    pn_mom = m2 - pd_mom
    T = 1e-3

    def db(pd, pn):
        return 10 * np.log10(max(pd, 1e-30) / (pn * T)) if pn > 0 else float("inf")

    return Fine(a.prn, dop, tau, db(pd_off, pn_off), db(pd_mom, pn_mom), pd_off / n ** 2, pn_off / (n * fs))
