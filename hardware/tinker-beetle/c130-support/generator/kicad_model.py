"""3D model for the board's C130: the can standing upright in its TPU support, drawn in the frame
of C130's footprint (Supercap_Radial_D8.0mm_L12.0mm_P3.50mm_Horizontal), so the board's C130 uses
it with zero offset and zero rotation.

    python3 kicad_model.py ../c130-support.step ../c130-upright-with-support.step

Frames:
  board-local  (build_c130_support.py) origin at the hole midpoint on the back face, +x = KiCad +x
               (pad 1, +), +y = KiCad +y, +z away from the board
  footprint    KiCad 3D-model frame of the library footprint: pad 1 at y = +1.75, pad 2 at y = -1.75,
               +z away from the board. For C130 (back side, 90 deg) board-local -> footprint is a
               +90 deg turn about z.
The can is the same cell and palette as ../../../3dmodels/Supercap_Radial_D8.0mm_H12.0mm_P3.50mm.step,
raised CAN_Z onto the support's seat.
"""
import math, sys
import cadquery as cq

support_step, out_path = sys.argv[1], sys.argv[2]

CAN_Z = 2.0             # can bottom above the board (build_c130_support.py prints this)
D, L = 8.0, 12.0
P, LEAD_D = 3.5, 0.6
SEAL_T, TOP_R, TOP_INSET, TOP_T = 0.3, 3.2, 0.25, 0.1
LEAD_BELOW = 3.0
STRIPE_T, STRIPE_HALF_DEG = 0.03, 28
NAVY = cq.Color(0.10, 0.15, 0.45)
ALU = cq.Color(0.80, 0.80, 0.82)
STEEL = cq.Color(0.55, 0.55, 0.58)
STRIPE = cq.Color(0.86, 0.88, 0.93)
RUBBER = cq.Color(0.15, 0.15, 0.16)
TPU = cq.Color(0.95, 0.55, 0.20)


def cyl(r, z0, z1, x=0.0):
    return cq.Workplane("XY").workplane(offset=z0).center(x, 0).circle(r).extrude(z1 - z0)


# can in board-local: axis at the origin, + lead at x = +1.75, - lead (and the band) at x = -1.75
z0 = CAN_Z
sleeve = (cyl(D / 2, z0 + SEAL_T, z0 + L).faces(">Z").edges().fillet(0.4)
          .cut(cyl(TOP_R, z0 + L - TOP_INSET - TOP_T, z0 + L + 1)))
top = cyl(TOP_R - 0.02, z0 + L - TOP_INSET - TOP_T, z0 + L - TOP_INSET)
seal = cyl(D / 2 - 0.4, z0, z0 + SEAL_T)
band = cyl(D / 2 + STRIPE_T, z0 + SEAL_T + 0.5, z0 + L - 0.8).cut(cyl(D / 2 - 0.01, z0, z0 + L))
a = math.radians(STRIPE_HALF_DEG)
sector = (cq.Workplane("XY").workplane(offset=z0 - 1)
          .polyline([(0, 0), (-6 * math.cos(a), -6 * math.sin(a)), (-6 * math.cos(a), 6 * math.sin(a))])
          .close().extrude(L + 2))
stripe = band.intersect(sector)
leads = [cyl(LEAD_D / 2, -LEAD_BELOW, z0 + SEAL_T + 0.5, x=x) for x in (P / 2, -P / 2)]
support = cq.importers.importStep(support_step)

turn = lambda w: w.rotate((0, 0, 0), (0, 0, 1), 90)   # board-local -> footprint frame
assy = cq.Assembly(name="C130_upright_TPU_support")
assy.add(turn(sleeve), name="sleeve", color=NAVY)
assy.add(turn(top), name="top", color=ALU)
assy.add(turn(seal), name="seal", color=RUBBER)
assy.add(turn(stripe), name="stripe", color=STRIPE)
for i, lead in enumerate(leads):
    assy.add(turn(lead), name=f"lead{i + 1}", color=STEEL)
assy.add(turn(support), name="tpu_support", color=TPU)
assy.export(out_path)
print("wrote", out_path)
