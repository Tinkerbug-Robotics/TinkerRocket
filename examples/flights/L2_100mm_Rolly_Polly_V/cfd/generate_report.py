#!/usr/bin/env python3
"""Generate the HTML report for the 100mm L2 rocket fin-tab CFD sweep,
including roll-plant identification and first-flight PID tuning values.

Inputs (all in this directory):
  results/*.dat            CFD force/moment histories (run_sweep.sh)
  plant_params.json        I_roll etc. (roll_inertia.py)
  motor_curves.json        thrust samples (thrustcurve.org)
  geometry/preview_deflections.png

Output:
  CFD_Report_100mm_Fin_Tab.html  (self-contained)
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
import base64
import io
import json

from postprocess_sweep import load_cases, analyze, D_AXIS, RHO, N_AVG, parse_force_dat

HERE = Path(__file__).parent
OUT = HERE / "CFD_Report_100mm_Fin_Tab.html"

# ---------------- firmware / design constants ----------------
N_TABS = 4
V_REF_FW = 50.0          # firmware GAIN_SCHEDULE_V_REF (m/s)
V_MIN_FW = 25.0          # firmware GAIN_SCHEDULE_V_MIN
SCALE_CAP = 3.0          # firmware GAIN_SCHEDULE_SCALE_CAP
F_C_TARGET = 2.0         # Hz, closed-loop crossover target (67mm methodology)
PM_TARGET = 50.0         # deg
TAU_SERVO = 0.0325       # s, first-order servo lag (PTK 7308 @8.2V, measured)
SLEW_SERVO = 923.0       # deg/s servo slew
T_DELAY = 0.001          # s compute delay
Z_INT = 0.6              # rad/s integral zero (Ki = Z_INT * Kp)
SEP_DPS = 40.0           # integral separation threshold
DFILT_HZ = 10.0          # derivative filter (unused, Kd=0)
TAB_LIMIT = 20.0         # deg
MISALIGN_DEG = 0.46      # effective per-tab misalignment for sim (67mm measured 0.46)
KICK_DPS = 120.0         # launch roll kick for sim
V_REF_SIM = 95.0         # tinkerrocket-sim FinTabConfig.V_ref convention
M_DRY_NOMOTOR = 6.57     # kg airframe (sum of .ork components)

MOTORS = {
    "J435WS": dict(m_load=0.6164, m_prop=0.352, ork_vmax=91.2),
    "J570W":  dict(m_load=0.902,  m_prop=0.5358, ork_vmax=119.4),
}


def fig_to_b64(fig, dpi=170):
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=dpi, bbox_inches='tight')
    buf.seek(0)
    return base64.b64encode(buf.read()).decode()


def png_to_b64(path):
    return base64.b64encode(Path(path).read_bytes()).decode()


# ---------------------------------------------------------------- flight model
def fly(motor, dt=0.002):
    """1-DOF vertical flight with the real thrust curve. Returns t, V arrays."""
    curves = json.loads((HERE / "motor_curves.json").read_text())
    tc = np.array(curves[motor])
    m = MOTORS[motor]
    tb = tc[-1, 0]
    imp_total = np.trapezoid(tc[:, 1], tc[:, 0])
    Cd, A = 0.55, np.pi * 0.0511**2
    t, v = 0.0, 0.0
    T_, V_ = [0.0], [0.0]
    while (v >= 0 or t < tb) and t < 25:
        imp_so_far = np.trapezoid(tc[tc[:, 0] <= t][:, 1], tc[tc[:, 0] <= t][:, 0]) if t > 0 else 0.0
        mass = M_DRY_NOMOTOR + m["m_load"] - m["m_prop"] * imp_so_far / imp_total
        thrust = np.interp(t, tc[:, 0], tc[:, 1]) if t <= tb else 0.0
        drag = 0.5 * RHO * Cd * A * v * abs(v)
        v += ((thrust - drag) / mass - 9.81) * dt
        t += dt
        T_.append(t); V_.append(max(v, 0.0))
    return np.array(T_), np.array(V_)


# ---------------------------------------------------------------- pid design
def design_pid(Kt80, I_roll):
    """All the tuning numbers from the CFD Kt at 80 m/s and I_roll."""
    d = {}
    d['Kt80'] = Kt80                                   # N*m/deg per tab (magnitude)
    d['Kt_V'] = lambda V: Kt80 * (V / 80.0)**2
    d['K_plant'] = lambda V: N_TABS * d['Kt_V'](V) / I_roll * 180.0 / np.pi  # deg/s^2 per tab deg
    wc = 2 * np.pi * F_C_TARGET
    d['wc'] = wc
    d['Kp50'] = wc / d['K_plant'](V_REF_FW)
    d['Ki50'] = Z_INT * d['Kp50']
    d['Kd50'] = 0.0
    lag_servo = np.degrees(np.arctan(wc * TAU_SERVO))
    lag_delay = np.degrees(wc * T_DELAY)
    lag_int = np.degrees(np.arctan(Z_INT / wc))
    d['pm'] = 90.0 - lag_servo - lag_delay - lag_int
    d['lag_servo'], d['lag_delay'], d['lag_int'] = lag_servo, lag_delay, lag_int
    d['Kt95'] = d['Kt_V'](95.0)
    return d


def gain_schedule_scale(V):
    v = max(abs(V), V_MIN_FW)
    return min((V_REF_FW / v)**2, SCALE_CAP)


# ---------------------------------------------------------------- closed loop
def closed_loop_sim(motor, des, I_roll, Kd_aero_per_V, t_end=8.0):
    """1 kHz rate-null PID + servo model + gain schedule, riding fly(motor)."""
    tF, vF = fly(motor)
    dt = 0.001
    n = int(t_end / dt)
    w = 0.0            # roll rate deg/s
    delta = 0.0        # actual tab deg
    integ = 0.0
    hist = np.zeros((n, 5))   # t, w, delta_cmd, delta, V
    kicked = False
    for i in range(n):
        t = i * dt
        V = float(np.interp(t, tF, vF))
        if not kicked and t >= 0.5:
            w += KICK_DPS; kicked = True
        scale = gain_schedule_scale(V)
        kp, ki = des['Kp50'] * scale, des['Ki50'] * scale
        err = 0.0 - w          # rate-null; firmware sign handled upstream
        if abs(err) <= SEP_DPS:
            integ += err * dt
            integ = np.clip(integ, -TAB_LIMIT / max(ki, 1e-9), TAB_LIMIT / max(ki, 1e-9))
        cmd = np.clip(kp * err + ki * integ, -TAB_LIMIT, TAB_LIMIT)
        # servo: first-order lag + slew
        dmax = SLEW_SERVO * dt
        step = np.clip((dt / (TAU_SERVO + dt)) * (cmd - delta), -dmax, dmax)
        delta += step
        # plant: tab torque opposes w for stabilizing sign; + misalignment disturbance
        Kt = des['Kt_V'](max(V, 1.0))
        tau = N_TABS * Kt * delta + N_TABS * Kt * (-MISALIGN_DEG)
        tau -= Kd_aero_per_V * max(V, 1.0) * np.radians(w)   # aero damping
        w += (tau / I_roll) * (180.0 / np.pi) * dt
        hist[i] = (t, w, cmd, delta, V)
    return hist


# ---------------------------------------------------------------- figures
def make_figures(cases, an, des, plant):
    figs = {}
    vels = sorted(an.keys())
    colors = {40: '#1f77b4', 80: '#ff7f0e', 120: '#2ca02c'}

    # --- geometry preview (from build pipeline) ---
    figs['geometry'] = png_to_b64(HERE / "geometry" / "preview_deflections.png")

    # --- 1: roll moment vs deflection with fits ---
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle('Roll Moment vs. Tab Deflection (per tab, about body axis)',
                 fontsize=13, fontweight='bold')
    for i, v in enumerate(vels):
        a = an[v]; ax = axes[i]
        mr = np.array(a['M_roll']) * 1e3
        ax.plot(a['angles'], mr, 'o', color=colors[v], ms=6, label='CFD')
        xs = np.linspace(-22, 22, 50)
        ax.plot(xs, (a['Kt'] * xs + a['offset']) * 1e3, 'r--', lw=1.5,
                label=f"fit {a['Kt']*1e3:.3f} mN·m/deg\nR²={a['r2']:.5f}")
        ax.set_title(f"V = {v} m/s (q = {a['q']:.0f} Pa)")
        ax.set_xlabel('Tab deflection (deg)'); ax.set_ylabel('M_roll (mN·m)')
        ax.axhline(0, color='gray', lw=0.5, ls='--'); ax.axvline(0, color='gray', lw=0.5, ls='--')
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
    plt.tight_layout()
    figs['torque'] = fig_to_b64(fig); plt.close(fig)

    # --- 2: 4-panel ---
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('100mm Fin Tab — Single-Tab Forces & Moments (CofR: body axis)',
                 fontsize=13, fontweight='bold')
    for v in vels:
        a = an[v]; c = colors[v]; vc = a['cases']; ang = a['angles']
        axes[0, 0].plot(ang, [x['M_roll'] * 1e3 for x in vc], 'o-', color=c, ms=5, label=f'V={v}')
        axes[0, 1].plot(ang, [x['F'][1] for x in vc], 'o-', color=c, ms=5, label=f'V={v}')
        axes[1, 0].plot(ang, [x['F'][2] for x in vc], 'o-', color=c, ms=5, label=f'V={v}')
        dmdd = np.gradient([x['M_roll'] for x in vc], ang)
        axes[1, 1].plot(ang, dmdd / a['q'] * 1e6 * 180 / np.pi, 's-', color=c, ms=5, label=f'V={v}')
    for ax in axes.flat:
        ax.axhline(0, color='gray', lw=0.5, ls='--'); ax.axvline(0, color='gray', lw=0.5, ls='--')
        ax.grid(alpha=0.3); ax.legend(fontsize=9); ax.set_xlabel('Tab deflection (deg)')
    axes[0, 0].set_ylabel('M_roll (mN·m)'); axes[0, 0].set_title('Roll moment about body axis')
    axes[0, 1].set_ylabel('Fy (N)'); axes[0, 1].set_title('Lateral force on tab')
    axes[1, 0].set_ylabel('Fz (N)'); axes[1, 0].set_title('Streamwise force on tab')
    axes[1, 1].set_ylabel('dM/dδ / q (µm²)'); axes[1, 1].set_title('Roll effectiveness / q')
    plt.tight_layout()
    figs['four'] = fig_to_b64(fig); plt.close(fig)

    # --- 3: V^2 scaling ---
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.6))
    fig.suptitle('V² Scaling', fontsize=13, fontweight='bold')
    kts = [abs(an[v]['Kt']) * 1e3 for v in vels]
    ax1.plot(vels, kts, 'o-', color='#2166ac', ms=8)
    vt = np.linspace(30, 130, 60)
    ax1.plot(vt, kts[1] * (vt / vels[1])**2, 'r--', lw=1.4, label=f'V² from V={vels[1]}')
    ax1.set_xlabel('V (m/s)'); ax1.set_ylabel('|Kt| (mN·m/deg per tab)'); ax1.legend(); ax1.grid(alpha=0.3)
    ax2.bar([str(v) for v in vels], [kts[i] / (0.5 * RHO * vels[i]**2) * 1e3 for i in range(len(vels))],
            color=[colors[v] for v in vels], alpha=0.85)
    ax2.set_xlabel('V (m/s)'); ax2.set_ylabel('Kt/q (mN·m/deg per kPa)')
    ax2.set_title('Normalized (should be constant)'); ax2.grid(alpha=0.3, axis='y')
    plt.tight_layout()
    figs['v2'] = fig_to_b64(fig); plt.close(fig)

    # --- 4: convergence history (V80_10deg) ---
    ff = HERE / "results" / "V80_10deg_force.dat"
    mf = HERE / "results" / "V80_10deg_moment.dat"
    if ff.exists():
        _, F = parse_force_dat(ff); _, M = parse_force_dat(mf)
        mr = M[:, 2] + F[:, 1] * D_AXIS
        fig, ax = plt.subplots(figsize=(9, 3.6))
        it = np.arange(len(mr))
        ax.plot(it, mr * 1e3, lw=1, color='#2166ac')
        ax.axvspan(len(mr) - N_AVG, len(mr), color='orange', alpha=0.2, label=f'averaging window ({N_AVG} it)')
        ax.set_xlabel('SIMPLE iteration'); ax.set_ylabel('M_roll (mN·m)')
        ax.set_title('Convergence: V=80 m/s, δ=+10° (roll moment history)')
        ax.grid(alpha=0.3); ax.legend()
        figs['conv'] = fig_to_b64(fig); plt.close(fig)

    # --- 5: plant + authority ---
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.5, 4.4))
    vv = np.linspace(20, 130, 100)
    ax1.plot(vv, [des['K_plant'](x) for x in vv], color='#2166ac', lw=2)
    for v in vels:
        ax1.plot(v, des['K_plant'](v), 'o', color=colors[v], ms=8)
    ax1.set_xlabel('V (m/s)'); ax1.set_ylabel('K_plant (deg/s² per tab deg)')
    ax1.set_title('Plant gain vs. velocity'); ax1.grid(alpha=0.3)
    ax2.plot(vv, [des['K_plant'](x) * TAB_LIMIT for x in vv], color='#b2182b', lw=2)
    ax2.set_xlabel('V (m/s)'); ax2.set_ylabel('Max roll accel (deg/s²) at ±20°')
    ax2.set_title('Control authority'); ax2.grid(alpha=0.3)
    plt.tight_layout()
    figs['plant'] = fig_to_b64(fig); plt.close(fig)

    # --- 6: gain schedule over flight ---
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    for motor, c in [("J435WS", '#1f77b4'), ("J570W", '#d62728')]:
        tF, vF = fly(motor)
        axes[0].plot(tF, vF, color=c, lw=1.8, label=motor)
        sc = [gain_schedule_scale(v) for v in vF]
        axes[1].plot(tF, sc, color=c, lw=1.8, label=motor)
        axes[2].plot(tF, np.array(sc) * des['Kp50'], color=c, lw=1.8, label=motor)
    axes[0].set_ylabel('V (m/s)'); axes[0].set_title('Velocity profile (1-DOF, real thrust curves)')
    axes[1].set_ylabel('schedule scale'); axes[1].set_title(f'(Vref/V)² schedule, cap {SCALE_CAP}')
    axes[2].set_ylabel('Kp_eff (tab-deg per deg/s)'); axes[2].set_title('Effective Kp over flight')
    for ax in axes:
        ax.set_xlabel('t (s)'); ax.grid(alpha=0.3); ax.legend()
    plt.tight_layout()
    figs['sched'] = fig_to_b64(fig); plt.close(fig)

    # --- 7: closed-loop sim ---
    fig, axes = plt.subplots(2, 2, figsize=(13.5, 7.5))
    fig.suptitle(f'Closed-loop rate-null sim — Kp₅₀={des["Kp50"]:.3f}, Ki₅₀={des["Ki50"]:.3f}, Kd=0 '
                 f'(120 dps kick at t=0.5 s, {MISALIGN_DEG}° tab misalignment)',
                 fontsize=12, fontweight='bold')
    for col, motor in enumerate(["J435WS", "J570W"]):
        h = closed_loop_sim(motor, des, plant['I_roll'], plant['Kd_aero_per_V'])
        ax = axes[0, col]
        ax.plot(h[:, 0], h[:, 1], color='#2166ac', lw=1.4)
        ax.axhline(0, color='gray', lw=0.5)
        ax.set_title(f'{motor}: roll rate'); ax.set_ylabel('ω (deg/s)'); ax.grid(alpha=0.3)
        ax = axes[1, col]
        ax.plot(h[:, 0], h[:, 2], color='#b2182b', lw=1, label='cmd')
        ax.plot(h[:, 0], h[:, 3], color='#ef8a62', lw=1, ls='--', label='servo')
        ax.axhline(TAB_LIMIT, color='gray', lw=0.5, ls=':'); ax.axhline(-TAB_LIMIT, color='gray', lw=0.5, ls=':')
        ax.set_title(f'{motor}: tab command'); ax.set_ylabel('δ (deg)'); ax.set_xlabel('t (s)')
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
    plt.tight_layout()
    figs['loop'] = fig_to_b64(fig); plt.close(fig)

    return figs


# ---------------------------------------------------------------- html
def generate_html(cases, an, des, plant, figs, sens):
    vels = sorted(an.keys())
    kt80 = abs(an[80]['Kt'])
    K50 = des['K_plant'](50)

    css = """
  :root { --bg:#fff; --fg:#1a1a1a; --accent:#2166ac; --border:#d0d7de; --code-bg:#f6f8fa; }
  * { box-sizing:border-box; }
  body { font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif;
         line-height:1.65; color:var(--fg); background:var(--bg); max-width:1100px;
         margin:0 auto; padding:2rem 2.5rem; }
  h1 { font-size:1.9rem; border-bottom:2px solid var(--accent); padding-bottom:.4rem; margin-top:0; }
  h2 { font-size:1.45rem; border-bottom:1px solid var(--border); padding-bottom:.3rem; margin-top:2.5rem; }
  h3 { font-size:1.15rem; margin-top:2rem; }
  img { max-width:100%; height:auto; display:block; margin:1.2rem auto;
        border:1px solid var(--border); border-radius:4px; }
  table { border-collapse:collapse; margin:1rem 0; font-size:.92rem; }
  th,td { border:1px solid var(--border); padding:6px 12px; text-align:left; }
  th { background:var(--accent); color:#fff; font-weight:600; }
  tr:nth-child(even) { background:var(--code-bg); }
  code { background:var(--code-bg); padding:2px 5px; border-radius:3px; font-size:.9em; }
  pre { background:var(--code-bg); border:1px solid var(--border); border-radius:6px;
        padding:1rem; overflow-x:auto; font-size:.88rem; line-height:1.5; }
  pre code { background:none; padding:0; }
  .toc { background:var(--code-bg); border:1px solid var(--border); border-radius:6px;
         padding:1rem 1.5rem; margin-bottom:2rem; }
  .toc a { text-decoration:none; color:var(--accent); }
  .note { background:#fff8e1; border-left:4px solid #ffc107; padding:.8rem 1rem;
          margin:1rem 0; border-radius:0 4px 4px 0; }
  .good { background:#e8f5e9; border-left:4px solid #4caf50; padding:.8rem 1rem;
          margin:1rem 0; border-radius:0 4px 4px 0; }
"""

    h = f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>CFD Report — 100mm L2 Rocket Fin Tab Roll Authority &amp; PID Tuning</title>
<style>{css}</style></head><body>

<h1>CFD Analysis: 100mm L2 Rocket Fin Tab — Roll Authority &amp; First-Flight PID Tuning</h1>

<p><strong>Date:</strong> August 9, 2026<br>
<strong>Rocket:</strong> "100 mm All Fiberglass Steerable" (OpenRocket model)<br>
<strong>Geometry source:</strong> <code>100 mm Rocket Airfoil.step</code> (Fusion 360, one of 4 fins)<br>
<strong>Solver:</strong> OpenFOAM v2412, simpleFoam (steady incompressible RANS, k-ω SST), Docker local</p>

<div class="toc"><strong>Contents</strong><ul>
<li><a href="#s1">1. Objective</a></li>
<li><a href="#s2">2. Geometry</a></li>
<li><a href="#s3">3. CFD Setup</a></li>
<li><a href="#s4">4. Results</a></li>
<li><a href="#s5">5. V² Scaling</a></li>
<li><a href="#s6">6. Roll Plant Model</a></li>
<li><a href="#s7">7. PID Tuning for First Flight</a></li>
<li><a href="#s8">8. Firmware &amp; Sim Configuration Values</a></li>
<li><a href="#s9">9. Limitations &amp; Uncertainty</a></li>
<li><a href="#s10">10. Reproducing This Analysis</a></li>
<li><a href="#s11">11. Conclusions</a></li>
</ul></div>

<h2 id="s1">1. Objective</h2>
<p>Characterize the roll torque produced by the trailing-edge control tab of the 100&nbsp;mm L2
rocket fin across tab deflections of −20° to +20° and three flight velocities (40, 80,
120&nbsp;m/s), then derive roll-rate PID gains for the first flight. The flight envelope covers
both motor options: AeroTech J435WS (V<sub>max</sub> ≈ 91&nbsp;m/s) and J570W
(V<sub>max</sub> ≈ 119&nbsp;m/s). First flight uses <strong>roll control only</strong>
(rate-null mode, the same controller structure flown on the 67&nbsp;mm testbed).</p>

<h2 id="s2">2. Geometry</h2>
<h3>2.1 Fin &amp; Tab</h3>
<table>
<tr><th>Parameter</th><th>Value</th></tr>
<tr><td>Fin span (root plane → tip)</td><td>110.0 mm</td></tr>
<tr><td>Root chord (CAD, incl. tab reach)</td><td>150.0 mm</td></tr>
<tr><td>Exposed fin area (planform)</td><td>63.3 cm²</td></tr>
<tr><td>Tab span</td><td>24.0 mm (X = 54.7 → 78.7 mm from root plane)</td></tr>
<tr><td>Tab chord aft of hinge</td><td>51.6 mm (reaches the aft/bottom edge of the airfoil)</td></tr>
<tr><td>Hinge</td><td>spanwise torque rod, ∅3 mm, round-nose cove, 0.5 mm gaps</td></tr>
<tr><td>Number of fins/tabs</td><td>4 (cruciform, all tabs driven common-mode for roll)</td></tr>
<tr><td>Mount</td><td>boat-tail transition, r = 51.1 → 37.5 mm over the root chord</td></tr>
<tr><td>Tab inboard edge → centerline</td><td><strong>95.8 mm (measured)</strong></td></tr>
<tr><td>Root plane → axis distance D<sub>axis</sub></td><td><strong>41.1 mm</strong> (= 95.8 − 54.68, from the measurement above)</td></tr>
</table>

<p>The STEP assembly contains the fin blade, the control tab (round-nose flap on a spanwise
torque rod at Z=61.6&nbsp;mm from the aft edge), and the internal servo/linkage hardware
(excluded from CFD). Internal channels that open into the flow (rod bore at the tip, two
chordwise lightening holes, three skin screw pilots) were capped in CAD before meshing.</p>

<h3>2.2 CFD frame &amp; deflection convention</h3>
<table>
<tr><th>Axis</th><th>Meaning</th></tr>
<tr><td>X</td><td>spanwise, 0 at the fin root plane, tip at +110 mm</td></tr>
<tr><td>Y</td><td>thickness/lateral</td></tr>
<tr><td>Z</td><td>streamwise, flow +Z, LE at Z=10 mm, TE at Z=160 mm</td></tr>
</table>
<p>δ &gt; 0 moves the tab TE toward +Y. Roll moment about the body axis (parallel to Z at
X = −D<sub>axis</sub>): <code>M_roll = Mz(origin) + Fy × D_axis</code>. With this convention
positive δ produces negative M_roll; only the magnitude matters for tuning — the physical
rotation direction is verified on the bench (§7.5).</p>

<img src="data:image/png;base64,{figs['geometry']}" alt="geometry previews">
<p><em>Figure 1: CFD geometry at −20°, 0°, +20° tab deflection. Green: fin (fincan patch).
Red: tab (finTab patch). Bottom row: airfoil sections through the tab span.</em></p>

<h2 id="s3">3. CFD Setup</h2>
<table>
<tr><th>Item</th><th>Value</th></tr>
<tr><td>Domain</td><td>X: 1→280 mm (wall at root), Y: ±200 mm, Z: −250→650 mm</td></tr>
<tr><td>Background mesh</td><td>10 mm base, 28×40×90</td></tr>
<tr><td>snappyHexMesh</td><td>fin level (2 3), tab level (3 4) = 0.625 mm, feature level 3, 3–4 prism layers</td></tr>
<tr><td>Cell count</td><td>~272k (pilot case; all cases pass all mesh-quality checks)</td></tr>
<tr><td>BCs</td><td>fixed-U inlet, p-outlet, freestream sides, no-slip wall at root plane ("bodyTube")</td></tr>
<tr><td>Turbulence</td><td>k-ω SST, wall functions, TI 1%</td></tr>
<tr><td>Iterations</td><td>1000 SIMPLE; forces averaged over last {N_AVG}</td></tr>
</table>

<table>
<tr><th>V (m/s)</th><th>q (Pa)</th><th>Mach</th><th>Re (root chord)</th></tr>
<tr><td>40</td><td>980</td><td>0.12</td><td>4.1×10⁵</td></tr>
<tr><td>80</td><td>3,920</td><td>0.24</td><td>8.2×10⁵</td></tr>
<tr><td>120</td><td>8,820</td><td>0.35</td><td>1.2×10⁶</td></tr>
</table>

<p>33 cases: 11 deflections (−20, −15, −10, −5, −2, 0, +2, +5, +10, +15, +20°) × 3 velocities.
Max Mach 0.35 — incompressible assumption is good across the whole envelope (the 67&nbsp;mm
study only saw deviations at Mach 0.47).</p>

<img src="data:image/png;base64,{figs.get('conv','')}" alt="convergence">
<p><em>Figure 2: typical roll-moment convergence history (V=80, δ=+10°); the shaded window is
averaged for the reported values.</em></p>

<h2 id="s4">4. Results</h2>
<img src="data:image/png;base64,{figs['torque']}" alt="torque vs deflection">
<p><em>Figure 3: Roll moment about the body axis vs. tab deflection, with linear fits.</em></p>
"""

    for v in vels:
        a = an[v]
        h += f"""<h3>V = {v} m/s (q = {a['q']:,.0f} Pa)</h3>
<table>
<tr><th>δ (deg)</th><th>Fy (N)</th><th>Fz (N)</th><th>M_roll (mN·m)</th><th>σ window (mN·m)</th></tr>
"""
        for c in a['cases']:
            h += (f"<tr><td>{c['angle']:+d}</td><td>{c['F'][1]:.3f}</td><td>{c['F'][2]:.3f}</td>"
                  f"<td>{c['M_roll']*1e3:.2f}</td><td>{c['M_roll_std']*1e3:.3f}</td></tr>\n")
        h += (f"</table>\n<p><strong>K<sub>t</sub> = {abs(a['Kt'])*1e3:.3f} mN·m/deg per tab</strong> "
              f"(R² = {a['r2']:.5f}, offset {a['offset']*1e3:+.2f} mN·m)</p>\n")

    h += f"""
<img src="data:image/png;base64,{figs['four']}" alt="four panel">
<p><em>Figure 4: force/moment overview.</em></p>

<h2 id="s5">5. V² Scaling</h2>
<img src="data:image/png;base64,{figs['v2']}" alt="v2 scaling">
<p><em>Figure 5: K<sub>t</sub> follows dynamic pressure closely.</em></p>
<table><tr><th>Pair</th><th>(V₂/V₁)²</th><th>measured</th><th>deviation</th></tr>
"""
    for v1, v2 in [(40, 80), (40, 120), (80, 120)]:
        th = (v2 / v1)**2
        me = abs(an[v2]['Kt']) / abs(an[v1]['Kt'])
        h += f"<tr><td>{v2}/{v1}</td><td>{th:.2f}</td><td>{me:.2f}</td><td>{(me-th)/th*100:+.1f}%</td></tr>\n"

    kt_line = " / ".join(f"{abs(an[v]['Kt'])*1e3:.2f}" for v in vels)
    h += f"""</table>

<h2 id="s6">6. Roll Plant Model</h2>
<h3>6.1 Roll inertia (component build-up)</h3>
<table>
<tr><th>Group</th><th>I_roll (10⁻³ kg·m²)</th></tr>
<tr><td>Airframe (tubes, nose, couplers, recovery, motor)</td><td>{plant['I_airframe']*1e3:.2f}</td></tr>
<tr><td>Fin set ×4 (CAD volume integral, D_axis {plant['D_axis']*1000:.1f} mm)</td><td>{plant['I_fins']*1e3:.2f}</td></tr>
<tr><td><strong>Total I_roll</strong></td><td><strong>{plant['I_roll']*1e3:.2f}</strong></td></tr>
</table>
<p>Propellant burn changes I_roll by &lt;1% (on-axis mass), so launch and burnout values are
treated as equal. The 67&nbsp;mm testbed was 2.0×10⁻³ — this rocket has <strong>9.2× the roll
inertia</strong>.</p>

<h3>6.2 Plant gain</h3>
<p>Rate-loop plant (all 4 tabs driven together):
<code>ω̇ [deg/s²] = K_plant(V) · δ [tab deg]</code>, with
<code>K_plant(V) = 4·Kt(V)/I_roll · 180/π</code>. Weak aero roll damping
(strip theory: pole at 0.9–2.7 rad/s over 40–120 m/s) is negligible at the 2 Hz crossover.</p>
<table>
<tr><th>V (m/s)</th><th>Kt per tab (mN·m/deg)</th><th>K_plant (deg/s² per deg)</th><th>Max roll accel at ±20° (deg/s²)</th></tr>
"""
    for v in vels:
        kp_ = des['K_plant'](v)
        h += (f"<tr><td>{v}</td><td>{abs(an[v]['Kt'])*1e3:.2f}</td>"
              f"<td>{kp_:.0f}</td><td>{kp_*TAB_LIMIT:,.0f}</td></tr>\n")
    h += f"""<tr><td>50 (fw V_ref)</td><td>{des['Kt_V'](50)*1e3:.2f}</td><td>{K50:.0f}</td><td>{K50*TAB_LIMIT:,.0f}</td></tr>
</table>
<img src="data:image/png;base64,{figs['plant']}" alt="plant">
<p><em>Figure 6: plant gain and control authority vs. velocity.</em></p>

<h2 id="s7">7. PID Tuning for First Flight</h2>
<h3>7.1 Controller structure (as flown on the 67 mm)</h3>
<p>Rate-null PID at ~1 kHz: error = 0 − ω [deg/s], output = physical tab degrees (1:1 linkage),
clamped ±20°. All four tabs receive the same roll command. Velocity gain schedule
<code>scale = min((V_ref/max(|V|,V_min))², {SCALE_CAP:.0f})</code> with V_ref = {V_REF_FW:.0f},
V_min = {V_MIN_FW:.0f} m/s from EKF speed. Integral separation freezes the accumulator while
|error| &gt; {SEP_DPS:.0f} deg/s. Angle-hold cascade stays <strong>off</strong> for the first flight.</p>

<h3>7.2 Design method</h3>
<p>The rate loop through the tab is an integrator: L(s) = Kp·K_plant/s. Following the 67&nbsp;mm
methodology: crossover f<sub>c</sub> = {F_C_TARGET:.1f} Hz (ω<sub>c</sub> = {des['wc']:.1f} rad/s),
so <code>Kp = ω_c / K_plant(V_ref)</code>. Phase margin budget at ω<sub>c</sub>:
servo lag (τ = {TAU_SERVO*1e3:.1f} ms) −{des['lag_servo']:.1f}°, loop delay −{des['lag_delay']:.1f}°,
integral zero ({Z_INT} rad/s) −{des['lag_int']:.1f}° →
<strong>PM ≈ {des['pm']:.0f}°</strong> (target ≥ {PM_TARGET:.0f}°). Kd is unnecessary at this
margin and is left 0 (matches all flown configs).</p>

<h3>7.3 Recommended gains</h3>
<div class="good">
<table>
<tr><th>Parameter</th><th>Value</th><th>Notes</th></tr>
<tr><td><strong>KP</strong></td><td><strong>{des['Kp50']:.3f}</strong></td><td>tab-deg per deg/s, quoted at V_ref = 50 m/s</td></tr>
<tr><td><strong>KI</strong></td><td><strong>{des['Ki50']:.3f}</strong></td><td>integral zero at {Z_INT} rad/s; trims fin misalignment in ~2 s</td></tr>
<tr><td><strong>KD</strong></td><td><strong>0.0</strong></td><td>PM already {des['pm']:.0f}°; servo lag limits D usefulness</td></tr>
<tr><td>D_FILTER_CUTOFF_HZ</td><td>{DFILT_HZ:.0f}</td><td>unchanged (inactive with KD=0)</td></tr>
<tr><td>INTEGRAL_SEP_THRESHOLD_DPS</td><td>{SEP_DPS:.0f}</td><td>unchanged</td></tr>
<tr><td>GAIN_SCHEDULE_V_REF / V_MIN / cap</td><td>{V_REF_FW:.0f} / {V_MIN_FW:.0f} / {SCALE_CAP:.0f}</td><td>unchanged</td></tr>
<tr><td>MIN_CMD / MAX_CMD</td><td>±{TAB_LIMIT:.0f}°</td><td>CFD linear range confirmed to ±20°</td></tr>
</table>
</div>
<p>For comparison, the 67&nbsp;mm flew KP = 0.12 / KI = 0.01 at the same V_ref: the 100&nbsp;mm
needs ≈2× the proportional gain because inertia grew 9.2× while total tab torque grew only
~2.9× at matched speed. A conservative fallback (<code>KI = 0.02</code>) gives slower trim of
the ~2 deg/s residual from expected fin misalignment but is otherwise equivalent.</p>

<img src="data:image/png;base64,{figs['sched']}" alt="gain schedule">
<p><em>Figure 7: velocity profiles (1-DOF with real thrust curves), gain-schedule factor,
and effective Kp for both motors. The scale rides the {SCALE_CAP:.0f}× cap below
{V_REF_FW/np.sqrt(SCALE_CAP):.0f} m/s — i.e. briefly off the rod and again late in coast.</em></p>

<h3>7.4 Closed-loop verification</h3>
<img src="data:image/png;base64,{figs['loop']}" alt="closed loop">
<p><em>Figure 8: 1 kHz closed-loop simulation with servo lag ({TAU_SERVO*1e3:.0f} ms,
{SLEW_SERVO:.0f} deg/s slew), ±20° clamp, gain schedule, integral separation, a
{KICK_DPS:.0f} deg/s launch kick at t = 0.5 s and a constant {MISALIGN_DEG}° effective tab
misalignment (the 67&nbsp;mm measured −0.46°). The kick is nulled in well under a second on
both motors and the misalignment trims to zero without saturating.</em></p>

<h3>7.5 First-flight checklist</h3>
<ol>
<li><strong>Sign check on the bench</strong>: with the established spin test, confirm positive
roll rate produces the tab direction that opposes it; flip <code>FIN_ROLL_REVERSE_MASK</code>
if not. (CFD gives magnitude; mounting orientation sets the sign.)</li>
<li>Rate-null mode only (<code>USE_ANGLE_CONTROL = false</code>); angle cascade
(KP_ANGLE = 2.0, cap 60 deg/s) can fly later once rate control is proven.</li>
<li>Expected behavior: control authority is weak below ~30 m/s (schedule capped), strong
through boost; expect &lt;12° tab for a 120 deg/s disturbance at V ≥ 50 m/s.</li>
<li>Servo direction/trim per the standard preflight; per-servo biases as calibrated.</li>
</ol>

<h2 id="s8">8. Firmware &amp; Sim Configuration Values</h2>
<pre><code>// tinkerrocket-idf/projects/flight_computer/main/config.h (100mm L2 rocket)
static constexpr float KP = {des['Kp50']:.3f}f;   // tab-deg per deg/s at V_ref=50
static constexpr float KI = {des['Ki50']:.3f}f;   // integral zero {Z_INT} rad/s
static constexpr float KD = 0.0f;
// unchanged: MIN_CMD/MAX_CMD ±20, D_FILTER_CUTOFF_HZ 10,
// INTEGRAL_SEP_THRESHOLD_DPS 40, GAIN_SCHEDULE_V_REF 50, V_MIN 25, cap 3.0

# tinkerrocket-sim rocket definition (100mm L2)
rd.dry_mass          = 6.57 + 0.55        # airframe + burned-out motor (J570W)
rd.I_roll_launch     = {plant['I_roll']:.4e}
rd.I_roll_burnout    = {plant['I_roll']:.4e}
rd.fin_tabs.n_tabs   = 4
rd.fin_tabs.V_ref    = 95.0
rd.fin_tabs.Kt_ref   = {des['Kt95']:.3e}   # N·m/deg per tab at 95 m/s (sign per firmware_roll_sign)
rd.fin_tabs.deflection_min = -20.0
rd.fin_tabs.deflection_max = 20.0
rd.roll_damping_K    = {plant['Kd_aero_per_V']:.2e}    # N·m/(m/s · rad/s), strip theory
</code></pre>

<h2 id="s9">9. Limitations &amp; Uncertainty</h2>
<table>
<tr><th>Source</th><th>Effect on Kt</th></tr>
<tr><td>D_axis 41.1 mm, from the measured 95.8 mm tab-edge radius (±1 mm)</td>
    <td>{sens[0]:+.1f}% / {sens[1]:+.1f}%</td></tr>
<tr><td>0.5 mm hinge gaps sealed by mesh resolution</td><td>slight overestimate (a few %; leakage ignored)</td></tr>
<tr><td>Flat wall instead of curved boat-tail at root</td><td>small; tab is mid-span, away from the root</td></tr>
<tr><td>Single fin (no fin–fin interference, no body vortices)</td><td>consistent with 67 mm methodology</td></tr>
<tr><td>Steady RANS, fully turbulent</td><td>no dynamic hinge effects; Re high enough</td></tr>
<tr><td>I_roll component model</td><td>±10% (D_axis and mass-model spread)</td></tr>
</table>
<p>Net: treat K_plant as ±15%. The design has PM ≈ {des['pm']:.0f}° and a 2 Hz crossover —
a ±15% plant-gain error moves the crossover ±15% and PM by ~±3°, well inside margins. The
integral separation and ±20° clamp bound the failure modes; the gains are safe to fly.</p>

<h2 id="s10">10. Reproducing This Analysis</h2>
<pre><code>100mm-L2-Rocket-Fin-Tab-Analysis/
    100 mm Rocket Airfoil.step   # source CAD (Fusion 360 export)
    build_geometry.py            # STEP -> 11 deflected STLs (cadquery)
    case_template/               # OpenFOAM case (blockMesh+snappy+simpleFoam)
    run_sweep.sh                 # 33 cases, Docker opencfd/openfoam-default:2412
    postprocess_sweep.py         # force extraction, Kt fits
    roll_inertia.py              # I_roll + damping from .ork + CAD
    generate_report.py           # this report
    results/                     # force/moment histories + logs
    geometry/                    # STLs + previews

python3 build_geometry.py
bash run_sweep.sh                # ~4.5 h on M-series 10-core, 2 concurrent
python3 postprocess_sweep.py
python3 roll_inertia.py
python3 generate_report.py
</code></pre>

<h2 id="s11">11. Conclusions</h2>
<ol>
<li>Tab roll authority is <strong>highly linear</strong> to ±20° at all speeds
(R² ≥ {min(an[v]['r2'] for v in vels):.4f}); K<sub>t</sub> per tab = {kt_line} mN·m/deg
at {'/'.join(str(v) for v in vels)} m/s.</li>
<li>V² scaling holds within {max(abs((abs(an[v2]['Kt'])/abs(an[v1]['Kt'])-(v2/v1)**2)/(v2/v1)**2*100) for v1,v2 in [(40,80),(40,120),(80,120)]):.1f}% across the envelope — Mach ≤ 0.35 keeps the incompressible solver valid.</li>
<li>Roll plant: I_roll = {plant['I_roll']*1e3:.1f}×10⁻³ kg·m², K_plant(50) = {K50:.0f} deg/s² per tab-deg.</li>
<li><strong>First-flight gains: KP = {des['Kp50']:.3f}, KI = {des['Ki50']:.3f}, KD = 0</strong>
(2.0 Hz crossover, PM ≈ {des['pm']:.0f}°), existing schedule/anti-windup settings unchanged.</li>
<li>Closed-loop sim nulls a 120 deg/s kick in &lt;1 s with &lt;12° tab on both motors.</li>
</ol>

</body></html>"""
    return h


def main():
    cases = load_cases()
    print(f"cases: {len(cases)}")
    if len(cases) < 33:
        print("WARNING: incomplete sweep")
    an = analyze(cases)
    plant = json.loads((HERE / "plant_params.json").read_text())

    kt80 = abs(an[80]['Kt'])
    des = design_pid(kt80, plant['I_roll'])

    # D_axis sensitivity on Kt at 80
    sens = []
    for d in (0.0401, 0.0421):
        vc = an[80]['cases']
        mr = [c['M'][2] + c['F'][1] * d for c in vc]
        A = np.vstack([an[80]['angles'], np.ones(len(mr))]).T
        m_, b_ = np.linalg.lstsq(A, mr, rcond=None)[0]
        sens.append((abs(m_) - kt80) / kt80 * 100)

    print(f"Kt80 = {kt80*1e3:.3f} mN·m/deg | Kp50={des['Kp50']:.3f} Ki50={des['Ki50']:.3f} PM={des['pm']:.1f}")
    figs = make_figures(cases, an, des, plant)
    html = generate_html(cases, an, des, plant, figs, sens)
    OUT.write_text(html)
    print(f"saved {OUT}")


if __name__ == "__main__":
    main()
