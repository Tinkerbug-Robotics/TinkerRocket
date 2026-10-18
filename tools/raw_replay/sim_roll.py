#!/usr/bin/env python3
"""A spinning body for the simulated flights (sim_events.py --roll): the roll profile laid over a flight's truth, the
IMU that sees it (the gyro's chip at 45 deg, clipped at its rail), and the carrier wind-up its antenna puts on every
Doppler. numpy only; tests/unit/test_sim_roll.py pins the conventions.

The overlay. A flight's attitude q (body FRD -> NED, scalar first) is turned about its own nose by phi(t), the
integral of the profile's added rate: q' = q (x) qx(phi). The trajectory, the nose's direction (the C/N0 model's
dipole), the barometer are the flight's own, so the cases across roll rates are paired; the magnetometer, the IMU and
the antenna's wind-up follow q'. With R the body -> NED matrix, R' = R Rx(phi) and a body vector v reads
Rx(phi)^T v in the rolled body. The rolled body's rate is Rx(phi)^T w + phi_dot x (rad/s, body FRD). With a roll
axis a tilted off the nose (sim_events.py --roll-axis-tilt-deg: roll_axis, rot), Rx and x are Ra and a throughout.

The IMU. Its errors are the sensor's own, in the sensor's frame: the flight's specific force f and rate w less their
truth (f_true = C (a - g), w_true from the attitude) are kept as they were, and the truth's part is turned:
f' = f + (Rx^T - I) f_true, w' = w + (Rx^T - I) w_true + phi_dot x. The ISM6HG256X sits at -45 deg about the
board's +z (config.h ISM6HG256_ROT_Z_DEG; board +x forward, +y left, +z up), so its x and y axes are (x + y)/sqrt 2 and
(x - y)/sqrt 2 of body FRD and pure roll loads both at p/sqrt 2: each axis is clipped at the gyro's rail (4587 dps,
the 16-bit full scale of its 4000 dps range: 32767 x 0.14 mdps) -- pure roll reaches it at 6487 dps, 18.0 rev/s.

The wind-up (Wu et al. 1993, rate form, range-like, boresight b = body +x):
    dW/dt = -k (lambda / 2pi) w . (b + e) / (1 + b . e)        [m/s]
e the line of sight (unit, receiver to satellite) in the body. Roll alone, w = p b: -k lambda p / 2pi on every
satellite. The sign is the rig's: +0.991 range-like cycles per clockwise turn seen from above, nose up (a clockwise
turn from above is a negative turn about the boresight up). A range rate (the mosaic's -lambda * Doppler) carries it as
written. 1 + b.e is held at den_min near the null behind the antenna.
"""
from __future__ import annotations

import math

import numpy as np

TWO_PI = 2.0 * math.pi
C_LIGHT = 299792458.0
LAMBDA_L1 = C_LIGHT / 1575.42e6          # GPS L1 C/A, Galileo E1
LAMBDA_B1I = C_LIGHT / 1561.098e6        # BeiDou B1I (the firmware's signal 28)
# the shells of the receiver model: 0 GPS, 1 Galileo, 2 BeiDou (sim_events.map_prn)
SHELL_LAMBDA = (LAMBDA_L1, LAMBDA_L1, LAMBDA_B1I)
GYRO_RAIL_DPS = 4587.0                   # the ISM6HG256X's digital rail at its 4000 dps range
# the chip's axes in body FRD (rows): board +x forward, +y left, +z up; the chip -45 deg about board +z
S2 = 1.0 / math.sqrt(2.0)
CHIP_AXES = np.array([[S2, S2, 0.0], [S2, -S2, 0.0], [0.0, 0.0, -1.0]])


def windup_rate(lam, k, w, e, den_min=0.1):
    """Wu's wind-up rate (m/s, range-like) for wavelength lam (m), k cycles a turn, body rate w (rad/s) and line of
    sight e (unit, receiver to satellite), both body FRD, boresight +x. Vectorized over leading axes of w and e."""
    w = np.asarray(w, float)
    e = np.asarray(e, float)
    den = np.maximum(1.0 + e[..., 0], den_min)
    num = w[..., 0] * (1.0 + e[..., 0]) + w[..., 1] * e[..., 1] + w[..., 2] * e[..., 2]
    return -k * lam / TWO_PI * num / den


def roll_rate_windup(lam, k, p):
    """Roll alone: -k lam p / 2pi (m/s)."""
    return -k * lam * np.asarray(p, float) / TWO_PI


def parse_profile(spec):
    """'const:F' (F rev/s from the thrust's start, reached over 0.1 s, held), 'spinup:F' (0 at the thrust's start to F
    at burnout, held), 'rev:F[,T[,D]]' (F from the thrust's start, reversed to -F over D s (1) from flight time T
    (default mid-coast)). Every profile's added rate decays with a 1 s time constant from the drogue's deployment.
    None or '' or 'none': no overlay."""
    if not spec or spec == "none":
        return None
    kind, _, rest = spec.partition(":")
    vals = [float(x) for x in rest.split(",") if x]
    if kind not in ("const", "spinup", "rev") or not vals:
        raise ValueError(f"unknown roll profile {spec!r}")
    return kind, vals


def profile_rate(t, spec, phases, ramp_s=0.1, decay_tau_s=1.0):
    """The added roll rate (rad/s) at flight times t for a parsed profile (parse_profile) and the truth's phases."""
    t = np.asarray(t, float)
    kind, vals = spec
    f = vals[0] * TWO_PI
    t0 = phases["thrust_start_s"]
    tb = phases["burnout_s"]
    ta = phases["apogee_s"]
    td = phases["deploy_s"]
    if kind == "const":
        r = f * np.clip((t - t0) / ramp_s, 0.0, 1.0)
    elif kind == "spinup":
        r = f * np.clip((t - t0) / max(tb - t0, 1e-3), 0.0, 1.0)
    else:  # rev
        tr = vals[1] if len(vals) > 1 else 0.5 * (tb + ta)
        dur = vals[2] if len(vals) > 2 else 1.0
        r = f * np.clip((t - t0) / ramp_s, 0.0, 1.0)
        r = r * (1.0 - 2.0 * np.clip((t - tr) / dur, 0.0, 1.0))
    return np.where(t > td, r * np.exp(-np.clip(t - td, 0.0, None) / decay_tau_s), r)


def integrate(t, rate):
    """phi(t): the cumulative trapezoid of rate from the first row (rad)."""
    t = np.asarray(t, float)
    rate = np.asarray(rate, float)
    phi = np.zeros_like(rate)
    phi[1:] = np.cumsum(0.5 * (rate[1:] + rate[:-1]) * np.diff(t))
    return phi


def rx(phi):
    """Rx(phi): the rotation about +x by phi, one 3x3 a row (rolled body -> body)."""
    phi = np.asarray(phi, float)
    c, s = np.cos(phi), np.sin(phi)
    R = np.zeros(phi.shape + (3, 3))
    R[..., 0, 0] = 1.0
    R[..., 1, 1] = c
    R[..., 1, 2] = -s
    R[..., 2, 1] = s
    R[..., 2, 2] = c
    return R


def roll_axis(tilt_deg):
    """The roll's axis in the body (unit, FRD) for --roll-axis-tilt-deg: the nose, +x, turned tilt_deg toward +y -- a
    rocket that spins about an axis off the board's x (the holder's mounting, the IMU's own axes). The nose then cones
    about the roll axis at the roll rate, tilt_deg off it, its own rate p sin(tilt) across it. None at 0: exactly +x,
    rx's."""
    if not tilt_deg:
        return None
    d = math.radians(tilt_deg)
    return np.array([math.cos(d), math.sin(d), 0.0])


def rot(phi, axis=None):
    """The rotation about axis (unit, body; None: +x, rx itself) by phi, one 3x3 a row (rolled body -> body)."""
    if axis is None:
        return rx(phi)
    phi = np.asarray(phi, float)
    a = np.asarray(axis, float)
    c, s = np.cos(phi)[..., None, None], np.sin(phi)[..., None, None]
    K = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return c * np.eye(3) + (1.0 - c) * np.outer(a, a) + s * K


def qmul(a, b):
    """Hamilton product, scalar first, row by row."""
    w1, x1, y1, z1 = np.moveaxis(np.asarray(a, float), -1, 0)
    w2, x2, y2, z2 = np.moveaxis(np.asarray(b, float), -1, 0)
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], axis=-1)


def roll_quat(q, phi, axis=None):
    """q (x) qx(phi): the attitude turned about its own nose by phi (or about axis, unit in the body: roll_axis)."""
    phi = np.asarray(phi, float)
    if axis is None:
        qx = np.stack([np.cos(0.5 * phi), np.sin(0.5 * phi), np.zeros_like(phi), np.zeros_like(phi)], axis=-1)
    else:
        sh = np.sin(0.5 * phi)
        qx = np.stack([np.cos(0.5 * phi), sh * axis[0], sh * axis[1], sh * axis[2]], axis=-1)
    out = qmul(q, qx)
    return out / np.linalg.norm(out, axis=-1, keepdims=True)


def turn(R, v):
    """R^T v row by row: a body vector read in the rolled body (R = rx(phi), or rot(phi, axis))."""
    return np.einsum("nji,nj->ni", R, v)


def chip_clip(w, rail_rad):
    """The body rate (rad/s, rows) as the chip reads it: on its three axes, each clipped at rail_rad, back in the body;
    and which rows clipped any axis."""
    w = np.asarray(w, float)
    c = w @ CHIP_AXES.T
    clipped = np.any(np.abs(c) > rail_rad, axis=-1)
    c = np.clip(c, -rail_rad, rail_rad)
    return c @ CHIP_AXES, clipped
