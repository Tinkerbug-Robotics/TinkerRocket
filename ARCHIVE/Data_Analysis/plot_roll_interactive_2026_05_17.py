#!/usr/bin/env python3
"""
Interactive single-figure plot of roll rate, roll angle, and roll command
for the 2026-05-17 RollyPolly_54mm flight. Opens a native matplotlib
window with pan/zoom toolbar.
"""
import matplotlib
matplotlib.use("MacOSX")  # native macOS interactive backend
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CSV = "/Users/christianpedersen/Documents/Hobbies/ModelRockets/TestFlights/2026_05_17/RollyPolly_54mm/flight_20260517_133456.csv"
CMD_LIMIT = 20.0
GYRO_FS = 4000.0

df = pd.read_csv(CSV)
df.columns = df.columns.str.strip()
t = df["Time (ms)"].values / 1000.0
lf = pd.to_numeric(df["Launch Flag"], errors="coerce").fillna(0).astype(int).values
t_launch = t[np.where(lf == 1)[0][0]]
t_rel = t - t_launch

gx = pd.to_numeric(df["Gyro X (deg/s)"], errors="coerce").values
rc = pd.to_numeric(df["Roll Command (deg)"], errors="coerce").values
roll = pd.to_numeric(df["Roll (deg)"], errors="coerce").values

mg = np.isfinite(gx)
mc = np.isfinite(rc)
mr = np.isfinite(roll)

# Integrated (unwrapped) roll from gyro X
ti = t_rel[mg]
gi = gx[mg]
roll_cum = np.concatenate([[0.0], np.cumsum(0.5 * (gi[:-1] + gi[1:]) * np.diff(ti))])
# Anchor to wrapped roll at t=0
i0 = np.argmin(np.abs(t_rel[mr]))
roll_cum += roll[mr][i0]

fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True)
fig.suptitle("RollyPolly 54mm  2026-05-17  --  Roll angle / rate / command (interactive)",
             fontsize=12, fontweight="bold")

# Panel 1: roll angle (wrapped + integrated)
ax = axes[0]
ax.plot(t_rel[mr], roll[mr], "C0-", lw=0.8, label="Roll angle (wrapped, deg)")
ax.plot(ti, roll_cum, "C2-", lw=0.8, alpha=0.7, label="Roll angle (integrated, unwrapped)")
ax.axhline(  180, color="k", lw=0.3)
ax.axhline( -180, color="k", lw=0.3)
ax.axhline(    0, color="k", lw=0.3)
ax.axvline(0.5, color="red", ls="--", lw=1.0, label="Ctrl ON (0.5s)")
ax.set_ylabel("Roll angle (deg)")
ax.legend(fontsize=9, loc="upper right")
ax.grid(True, alpha=0.3)

# Panel 2: rate (left Y, green) + command (right Y, orange) -- same x, same axes
ax = axes[1]
ax_r = ax.twinx()
l1, = ax.plot(t_rel[mg], gx[mg], "g-", lw=0.8, label="Roll rate / Gyro X (dps)")
l2, = ax_r.plot(t_rel[mc], rc[mc], "C1-", lw=0.8, alpha=0.9, label="Roll command (deg)")
ax.axhline( GYRO_FS, color="g", ls=":", lw=0.5)
ax.axhline(-GYRO_FS, color="g", ls=":", lw=0.5)
ax.axhline(0, color="k", lw=0.3)
ax_r.axhline( CMD_LIMIT, color="C1", ls=":", lw=0.5)
ax_r.axhline(-CMD_LIMIT, color="C1", ls=":", lw=0.5)
ax.axvline(0.5, color="red", ls="--", lw=1.0)
ax.set_ylabel("Roll rate (dps)", color="g")
ax_r.set_ylabel("Roll command (deg)", color="C1")
ax.tick_params(axis="y", colors="g")
ax_r.tick_params(axis="y", colors="C1")
ax.set_xlabel("Time since launch (s)")
ax.grid(True, alpha=0.3)
ax.legend([l1, l2], [l1.get_label(), l2.get_label()], fontsize=9, loc="upper right")

plt.tight_layout()
plt.show()
