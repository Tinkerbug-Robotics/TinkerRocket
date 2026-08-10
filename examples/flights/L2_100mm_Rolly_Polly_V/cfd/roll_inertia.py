#!/usr/bin/env python3
"""Roll moment of inertia + roll damping for the 100mm L2 rocket.

Sources:
  - Component masses/geometry from "100 mm Fiberglass Steerable.ork"
    (mass overrides where set; simple shell/cylinder models otherwise).
  - Fin roll inertia integrated from the actual CAD solids (STEP file),
    with the fin-set mass override distributed over the CAD volume.

Assumption (documented in the report): the fin root plane sits at
D_AXIS = 44 mm from the rocket centerline (mid boat-tail radius; the
transition runs r=51.1 -> 37.5 mm over the fin root). Sensitivity to
D_AXIS in [37.5, 51.1] is reported.

Roll damping via strip theory on the exposed fin planform:
  Kd_aero(V) = 0.5 * rho * V * a_eff * N * integral( c(h) * (D+h)^2 dh )
  a_eff from Helmbold finite-AR correction of 2*pi.
Then  I_roll * phidot_dot = n_tabs*Kt(V)*delta - Kd_aero(V)*phidot.
"""
import numpy as np
import cadquery as cq
from cadquery import importers
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
import os

HERE = os.path.dirname(os.path.abspath(__file__))
STEP = os.path.join(HERE, "100 mm Rocket Airfoil.step")

D_AXIS = 0.044          # m, root plane -> rocket axis (assumption, see above)
D_RANGE = (0.0375, 0.0511)
RHO = 1.225
N_FINS = 4
FINSET_MASS = 1.076     # kg, OpenRocket override for the whole 4-fin set
                        # (printed PETG blades + tabs + servos + rods)

# ---------------------------------------------------------------- fins (CAD)
def fin_inertia(d_axis=D_AXIS):
    """Roll inertia of the 4-fin set about the rocket axis, from CAD solids.

    Axis: line parallel to STEP-Z through (x=-d_axis, y=0).
    I_axis = sum over solids rho_eff * Int[(x+d_axis)^2 + y^2] dV
    """
    solids = importers.importStep(STEP).solids().vals()
    # rotating + fixed external parts count; internal servo etc. all rotate
    # with the rocket about its axis, so ALL solids in the fin assembly count.
    vol_tot = sum(s.Volume() for s in solids) * 1e-9  # m^3
    rho_eff = (FINSET_MASS / N_FINS) / vol_tot        # per-fin mass over CAD volume
    I1 = 0.0
    for s in solids:
        p = GProp_GProps()
        BRepGProp.VolumeProperties_s(s.wrapped, p)
        V = p.Mass() * 1e-9                            # m^3 (unit density)
        com = p.CentreOfMass()
        cx, cy = com.X() * 1e-3, com.Y() * 1e-3
        M = p.MatrixOfInertia()
        # Izz about the COM axes: matrix is about the origin in mm^5 units
        # (unit density: mm^3 * mm^2). Convert: 1e-15 m^5.
        Izz_origin = M.Value(3, 3) * 1e-15             # Int(x^2+y^2) dV about origin
        # shift to the rocket axis: Int((x+d)^2 + y^2) = Int(x^2+y^2) + 2 d Int(x) + d^2 V
        Ix_int = cx * V                                # Int(x dV)
        I_axis_unit = Izz_origin + 2 * d_axis * Ix_int + d_axis**2 * V
        I1 += rho_eff * I_axis_unit
    return N_FINS * I1, rho_eff, vol_tot


# ------------------------------------------------------- planform for damping
def fin_planform():
    """Exposed chord distribution c(h) from the fin blade CAD, h from root."""
    solids = importers.importStep(STEP).solids().vals()
    fin = max(solids, key=lambda s: s.Volume())
    v, f = fin.tessellate(0.2)
    V = np.array([[p.x, p.y, p.z] for p in v])
    hs = np.linspace(1.0, 109.5, 60)                   # mm stations
    cs = []
    for h in hs:
        m = np.abs(V[:, 0] - h) < 2.5
        cs.append((V[m, 2].max() - V[m, 2].min()) if m.any() else 0.0)
    return hs * 1e-3, np.array(cs) * 1e-3              # m


def roll_damping(V, d_axis=D_AXIS):
    """Kd_aero(V): roll damping torque per unit roll rate [N*m/(rad/s)]."""
    h, c = fin_planform()
    A = np.trapezoid(c, h)
    b = h[-1]
    AR = b**2 / A
    a_eff = 2 * np.pi * AR / (2 + np.sqrt(AR**2 + 4))  # Helmbold, per rad
    integral = np.trapezoid(c * (d_axis + h) ** 2, h)
    Kd = 0.5 * RHO * V * a_eff * N_FINS * integral
    return Kd, dict(area=A, span=b, AR=AR, a_eff=a_eff, integral=integral)


# ------------------------------------------------------------ airframe table
def airframe_inertia():
    """Non-fin components: (name, mass kg, model, I_roll kg m^2)."""
    rows = []
    def shell(name, m, r):            rows.append((name, m, f"thin shell r={r*1000:.1f}", m * r**2))
    def solid_cyl(name, m, r):        rows.append((name, m, f"solid cyl r={r*1000:.1f}", 0.5 * m * r**2))
    def cone_shell(name, m, r, k=0.6):rows.append((name, m, f"ogive/cone shell r={r*1000:.1f}", k * m * r**2))

    cone_shell("Nose cone (0.8 kg override)", 0.8, 0.0511)
    solid_cyl("Nose mass component", 0.1, 0.0125)
    shell("Nose coupler (calc 0.129 kg)", 0.129, 0.0495)
    shell("Body tube 1 (0.65 override)", 0.65, 0.0511)
    solid_cyl("Main chute packed", 0.2835, 0.0491)
    solid_cyl("Shock cord 1", 0.15, 0.0375)
    shell("Body tube 2 (0.5 override)", 0.5, 0.0511)
    shell("E-bay coupler (0.5 override)", 0.5, 0.0495)
    solid_cyl("E-bay mass component", 0.35, 0.0125)
    shell("Body tube 3 fiberglass (1.423)", 1.423, 0.05105)
    solid_cyl("Shock cord 2", 0.258, 0.0425)
    solid_cyl("Drogue chute", 0.2, 0.0375)
    rows.append(("Rail buttons x2", 0.015, "point r=59mm", 0.015 * 0.059**2))
    cone_shell("Tail cone (0.1 override)", 0.1, 0.030, 0.5)
    shell("Motor tube (calc 0.072 kg)", 0.072, 0.0203)
    # motors (about their own axis, on centerline)
    rows.append(("Motor J570W (loaded 0.902 kg)", 0.902, "solid cyl r=19mm", 0.5 * 0.902 * 0.019**2))
    return rows


def main():
    print("=== 100mm L2 rocket roll inertia ===\n")
    rows = airframe_inertia()
    I_air = sum(r[3] for r in rows)
    for name, m, model, I in rows:
        print(f"  {name:38s} m={m:6.3f} kg  {model:22s} I={I*1e3:7.3f} mkg m^2")
    print(f"  {'AIRFRAME SUBTOTAL':38s} {'':33s} I={I_air*1e3:7.3f} mkg m^2")

    I_fins, rho_eff, vol = fin_inertia()
    print(f"\n  Fin set (CAD integral, D_axis={D_AXIS*1000:.0f}mm):")
    print(f"    CAD volume/fin={vol*1e6:.1f} cm^3, rho_eff={rho_eff:.0f} kg/m^3")
    print(f"    I_fins(4x) = {I_fins*1e3:.3f} mkg m^2")
    for d in D_RANGE:
        If, _, _ = fin_inertia(d)
        print(f"    sensitivity D_axis={d*1000:.1f}mm -> I_fins={If*1e3:.3f}")

    I_total = I_air + I_fins
    print(f"\n  I_ROLL TOTAL = {I_total*1e3:.2f} mkg m^2 = {I_total:.5f} kg m^2")

    print("\n=== Roll damping (strip theory) ===")
    for Vv in (40, 80, 120):
        Kd, meta = roll_damping(Vv)
        tau = I_total / Kd
        print(f"  V={Vv:3d} m/s: Kd_aero={Kd:.4f} N m s/rad   pole 1/tau={1/tau:5.2f} rad/s  (tau={tau:.3f} s)")
    print(f"  fin: area={meta['area']*1e4:.1f} cm^2  span={meta['span']*1000:.0f}mm  AR={meta['AR']:.2f}  a_eff={meta['a_eff']:.2f}/rad")

    # save machine-readable
    import json
    Kd80, meta = roll_damping(80)
    out = dict(I_roll=I_total, I_fins=I_fins, I_airframe=I_air,
               D_axis=D_AXIS, Kd_aero_V80=Kd80,
               Kd_aero_per_V=Kd80 / 80.0,
               fin_area=meta['area'], fin_span=meta['span'],
               fin_AR=meta['AR'], a_eff=meta['a_eff'], n_fins=N_FINS)
    with open(os.path.join(HERE, "plant_params.json"), "w") as f:
        json.dump(out, f, indent=2)
    print("\nsaved plant_params.json")


if __name__ == "__main__":
    main()
