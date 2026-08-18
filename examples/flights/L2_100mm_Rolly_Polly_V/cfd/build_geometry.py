#!/usr/bin/env python3
"""Build CFD geometry for the 100mm L2 rocket fin-tab sweep.

Source: "100 mm Rocket Airfoil.step" (Fusion 360 export, one of 4 fins).
Solids in the STEP (identified by inspection):
  - fin blade (largest, ~155.6 cm^3), flat root plane at X=0, span to X=110,
    chord along Z with LE at Z=160 (rounded) and TE/bottom at Z=10 (flow = -Z)
  - control tab (~13.8 cm^3): spanwise slice X=[54.68, 78.68] of the trailing
    region, round-nose flap reaching the bottom of the airfoil (Z=10),
    hinged on a spanwise torque rod at (Y=0.04, Z=61.59)
  - servo, bracket, linkage, bushing (internal mechanism -> excluded)

CFD frame (matches the 67mm pipeline): X = span (0 at fin root plane),
Y = thickness, Z = streamwise with flow in +Z.  Transform from STEP frame:
rotate 180 deg about X, translate +170mm in Z:  (x,y,z) -> (x, -y, 170-z).
  => LE at Z=10mm, TE/tab bottom at Z=160mm, hinge at (Y=-0.04, Z=108.41).

Tab deflection sign: delta > 0 moves the tab TE toward +Y (CFD frame).
Implemented as rotation about the hinge axis (+X direction) by -delta.

Outputs (mm -> exported in meters):
  geometry/fincan_<angle>deg.stl   combined ASCII STL, solids "fincan" + "finTab"
"""
import os
import numpy as np
import cadquery as cq
from cadquery import importers

HERE = os.path.dirname(os.path.abspath(__file__))
STEP = os.path.join(HERE, "100 mm Rocket Airfoil.step")
OUTDIR = os.path.join(HERE, "geometry")

# Hinge axis measured from concentric cylinders (tab hub R4.125 / rod R1.5 /
# bushing R1.75) in the STEP frame:
HINGE_Y_STEP = 0.04
HINGE_Z_STEP = 61.59
Z_FLIP = 170.0          # CFD z = Z_FLIP - z_step
HINGE_Y = -HINGE_Y_STEP  # CFD frame
HINGE_Z = Z_FLIP - HINGE_Z_STEP  # 108.41

ANGLES = [-20, -15, -10, -5, -2, 0, 2, 5, 10, 15, 20]


def load_solids():
    solids = importers.importStep(STEP).solids().vals()
    # fin = largest volume; tab = the solid whose bbox spans X ~54.7-78.7
    fin = max(solids, key=lambda s: s.Volume())
    tab = None
    for s in solids:
        bb = s.BoundingBox()
        if abs(bb.xmin - 54.68) < 0.5 and abs(bb.xmax - 78.68) < 0.5:
            tab = s
    assert tab is not None, "tab solid not found"
    print(f"fin volume {fin.Volume()/1000:.1f} cm^3, tab volume {tab.Volume()/1000:.1f} cm^3")
    return fin, tab


def plug_fin_holes(fin):
    """Seal internal channels that open into the flow so snappyHexMesh
    doesn't try to mesh them (STEP frame, mm).

    - torque-rod channel R1.5 through the outboard fin (X 79.2-107) plus
      R2.875 counterbore at the tip face (X 107-110): one rod-diameter plug
      across the pocket gap and channel, one plug for the counterbore.
    - two chordwise holes R1.75 along Z at (X=94.61, Y=+-3.75), Z 18-100.
    - three R0.9 screw pilot holes on the -Y skin.
    """
    def cyl(radius, p0, p1):
        v = np.array(p1) - np.array(p0)
        h = float(np.linalg.norm(v))
        d = cq.Vector(*(v / h))
        return cq.Solid.makeCylinder(radius, h, cq.Vector(*p0), d)

    plugs = [
        # rod across pocket gap + channel + slightly past bore
        cyl(1.65, (78.0, HINGE_Y_STEP, HINGE_Z_STEP), (110.4, HINGE_Y_STEP, HINGE_Z_STEP)),
        # tip counterbore
        cyl(3.00, (106.6, HINGE_Y_STEP, HINGE_Z_STEP), (110.4, HINGE_Y_STEP, HINGE_Z_STEP)),
        # chordwise holes (stop 1mm short of the aft breakout at Z~100)
        cyl(1.85, (94.61, 3.74, 17.0), (94.61, 3.74, 99.0)),
        cyl(1.85, (94.61, -3.76, 17.0), (94.61, -3.76, 99.0)),
    ]
    # screw pilot holes: axis along Y at (X,Z) below; plug through the -Y skin
    for (px, pz) in [(16.87, 69.97), (48.53, 52.10), (48.53, 87.69)]:
        plugs.append(cyl(1.05, (px, -12.5, pz), (px, 2.0, pz)))

    out = fin
    for p in plugs:
        out = out.fuse(p)
    return out


def to_cfd(shape):
    """STEP frame -> CFD frame: rotate 180 about X axis, translate +Z_FLIP."""
    r = shape.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 180)
    return r.translate(cq.Vector(0, 0, Z_FLIP))


def rotate_tab(tab_cfd, deg):
    """delta > 0 => TE toward +Y: rotate about +X hinge axis by -deg."""
    if deg == 0:
        return tab_cfd
    return tab_cfd.rotate(cq.Vector(0, HINGE_Y, HINGE_Z),
                          cq.Vector(1, HINGE_Y, HINGE_Z), -deg)


def tess(shape, tol=0.15):
    v, f = shape.tessellate(tol)
    V = np.array([[p.x, p.y, p.z] for p in v])
    F = np.array(f)
    return V, F


def write_stl(path, solids_named):
    """ASCII STL with named solids, mm -> m."""
    with open(path, "w") as fo:
        for name, (V, F) in solids_named:
            fo.write(f"solid {name}\n")
            for tri in F:
                p = V[tri] * 1e-3
                n = np.cross(p[1] - p[0], p[2] - p[0])
                L = np.linalg.norm(n)
                n = n / L if L > 0 else n
                fo.write(f"  facet normal {n[0]:.6e} {n[1]:.6e} {n[2]:.6e}\n    outer loop\n")
                for q in p:
                    fo.write(f"      vertex {q[0]:.6e} {q[1]:.6e} {q[2]:.6e}\n")
                fo.write("    endloop\n  endfacet\n")
            fo.write(f"endsolid {name}\n")


def main():
    os.makedirs(OUTDIR, exist_ok=True)
    fin, tab = load_solids()
    fin = plug_fin_holes(fin)
    fin_cfd = to_cfd(cq.Workplane(obj=fin)).val()
    tab_cfd0 = to_cfd(cq.Workplane(obj=tab)).val()

    bbf, bbt = fin_cfd.BoundingBox(), tab_cfd0.BoundingBox()
    print(f"CFD fin bbox  X[{bbf.xmin:.1f},{bbf.xmax:.1f}] Y[{bbf.ymin:.1f},{bbf.ymax:.1f}] Z[{bbf.zmin:.1f},{bbf.zmax:.1f}]")
    print(f"CFD tab bbox  X[{bbt.xmin:.1f},{bbt.xmax:.1f}] Y[{bbt.ymin:.1f},{bbt.ymax:.1f}] Z[{bbt.zmin:.1f},{bbt.zmax:.1f}]")

    Vf, Ff = tess(fin_cfd)
    print(f"fin tris: {len(Ff)}")

    for a in ANGLES:
        tw = cq.Workplane(obj=tab_cfd0)
        tr = rotate_tab(tw, a).val()
        # clash check: tab must not massively intersect the fin
        inter = tr.intersect(fin_cfd)
        iv = inter.Volume() if inter is not None else 0.0
        Vt, Ft = tess(tr)
        s = f"{a}deg" if a >= 0 else f"n{abs(a)}deg"
        path = os.path.join(OUTDIR, f"fincan_{s}.stl")
        write_stl(path, [("fincan", (Vf, Ff)), ("finTab", (Vt, Ft))])
        bb = tr.BoundingBox()
        print(f"  {s:>7s}: tab Y[{bb.ymin:6.2f},{bb.ymax:6.2f}] Z[{bb.zmin:6.2f},{bb.zmax:6.2f}]"
              f"  clashV={iv:7.2f} mm^3  tris={len(Ft)}  -> {os.path.basename(path)}")


if __name__ == "__main__":
    main()
