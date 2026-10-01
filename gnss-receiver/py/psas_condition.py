#!/usr/bin/env python3
"""Condition a PSAS jGPS recording (MAX2769, 2-bit, 4.092 MS/s, zero IF) into an 8-bit .C8 file
the receiver reads like the rig's files.

The recordings of PSAS Launch-12 (2015) carry four impairments that keep any receiver off them
(milestone 7):
  * DC on Q (-0.14 of a ~2.5 LSB rms signal): removed;
  * I/Q imbalance (I and Q correlated +0.17, 0.35 dB apart): balanced, so the -21 dB mirror image
    of everything in the band goes;
  * the spectrum inverted against the receiver's convention (satellites sit at minus their
    predicted Doppler): conjugated;
  * an on-board carrier near +418 kHz, 39 dB over the noise in a 250 Hz bin and drifting a few
    kHz, with the 2-bit quantizer's harmonics: excised in the frequency domain (4096-point
    sqrt-Hann frames, half-overlapped), zeroing the bins whose mean power over each 0.5 s stands
    `k` times over the median.

    psas_condition.py JGPS@-32.041913222 -o JGPS@-32.041913222_cond.C8 [--k 8] [--no-invert]

Prints what it did. The output is int8 I, Q at 4.092 MS/s with L1 at 0 Hz.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

FS = 4.092e6
N = 4096          # frame
HOP = N // 2


def decode(raw: np.ndarray) -> np.ndarray:
    """Packed nibbles (I-mag, I-sign, Q-mag, Q-sign from the top; older sample high) to complex."""
    nib = np.empty(raw.size * 2, dtype=np.uint8)
    nib[0::2] = raw >> 4
    nib[1::2] = raw & 0xF
    i = np.where(nib & 8, 3.0, 1.0) * np.where(nib & 4, -1.0, 1.0)
    q = np.where(nib & 2, 3.0, 1.0) * np.where(nib & 1, -1.0, 1.0)
    return (i + 1j * q).astype(np.complex128)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", type=Path)
    ap.add_argument("-o", type=Path, required=True)
    ap.add_argument("--k", type=float, default=8.0, help="excise bins whose mean power is k x the median")
    ap.add_argument("--no-invert", action="store_true")
    ap.add_argument("--rms", type=float, default=20.0, help="output rms per component, LSB")
    ap.add_argument("--chunk-s", type=float, default=0.5, help="spectrum (and mask) averaging span, s")
    a = ap.parse_args()

    raw = np.memmap(a.src, dtype=np.uint8, mode="r")
    nsamp = raw.size * 2
    # DC and I/Q balance from the first 5 s (they hold steady through a recording).
    z = decode(np.asarray(raw[: int(5 * FS) // 2]))
    mean = z.mean()
    z -= mean
    si, sq = z.real.std(), z.imag.std()
    rho = float(np.mean(z.real * z.imag) / (si * sq))
    print(f"{a.src.name}: {nsamp} samples ({nsamp / FS:.2f} s); DC {mean.real:+.3f} {mean.imag:+.3f} removed; "
          f"I/Q correlation {rho:+.3f} and power ratio {20 * np.log10(si / sq):+.2f} dB balanced; "
          f"{'not conjugated' if a.no_invert else 'conjugated'}")

    def condition(zc: np.ndarray) -> np.ndarray:
        zc = zc - mean
        i_n = zc.real / si
        q_n = (zc.imag / sq - rho * i_n) / np.sqrt(1.0 - rho * rho)
        out = i_n + 1j * q_n
        return out if a.no_invert else np.conj(out)

    w = np.sqrt(np.hanning(N + 1)[:N])
    inbuf = np.zeros(HOP, dtype=np.complex128)  # leading pad: output runs HOP late, trimmed below
    carry = np.zeros(HOP, dtype=np.complex128)
    chunk = int(a.chunk_s * FS) // 2 * 2
    excised, nframes_all, emitted = 0.0, 0, 0
    scale = None
    with open(a.o, "wb") as fo:
        def emit(y: np.ndarray):
            nonlocal emitted, scale
            if scale is None:
                scale = a.rms / np.sqrt(np.mean(np.abs(y) ** 2) / 2)
            iq = np.empty(2 * y.size, dtype=np.int8)
            iq[0::2] = np.clip(np.round(y.real * scale), -127, 127)
            iq[1::2] = np.clip(np.round(y.imag * scale), -127, 127)
            iq.tofile(fo)
            emitted += y.size

        def frames_out(buf: np.ndarray):
            nonlocal excised, nframes_all, carry
            nf = (buf.size - N) // HOP + 1 if buf.size >= N else 0
            if nf == 0:
                return np.zeros(0, dtype=np.complex128), buf
            idx = np.arange(nf)[:, None] * HOP + np.arange(N)[None, :]
            F = np.fft.fft(buf[idx] * w, axis=1)
            p = np.mean(np.abs(F) ** 2, axis=0)
            bad = p > a.k * np.median(p)
            F[:, bad] = 0.0
            excised += bad.sum() * nf
            nframes_all += nf
            y = np.fft.ifft(F, axis=1) * w
            out = np.zeros((nf + 1) * HOP, dtype=np.complex128)
            out[: nf * HOP] += y[:, :HOP].reshape(-1)
            out[HOP:] += y[:, HOP:].reshape(-1)
            out[:HOP] += carry
            carry = out[-HOP:].copy()
            return out[:-HOP], buf[nf * HOP:]

        skip = HOP  # the leading pad's output
        for b0 in range(0, raw.size, chunk // 2):
            zc = condition(decode(np.asarray(raw[b0: b0 + chunk // 2])))
            inbuf = np.concatenate([inbuf, zc])
            y, inbuf = frames_out(inbuf)
            if skip:
                d = min(skip, y.size)
                y, skip = y[d:], skip - d
            remain = nsamp - emitted
            emit(y[:remain])
        inbuf = np.concatenate([inbuf, np.zeros(N)])
        y, inbuf = frames_out(inbuf)
        y = np.concatenate([y, carry])
        emit(y[: nsamp - emitted])
    print(f"  excised {excised / max(nframes_all, 1):.1f} of {N} bins ({1e-3 * FS / N:.2f} kHz each) per frame on "
          f"average, k = {a.k:g}; wrote {a.o} ({emitted} samples, int8 I/Q, rms {a.rms:g} LSB per component)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
