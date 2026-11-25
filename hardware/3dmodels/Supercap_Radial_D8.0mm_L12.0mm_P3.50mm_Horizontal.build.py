import math

import cadquery as cq

# Radial EDLC supercapacitor lying on the board: CDA CXHP(2.7V) series, radial lead type,
# spec date 2022-06-10 -- 2.0 F in an 8 x 12 can (D +/-1.0, L +/-1.5), leads 0.6 mm on a
# 3.5 mm pitch. The same cell as Supercap_Radial_D8.0mm_H12.0mm_P3.50mm.step, laid down:
# the leads leave the end face at the can's mid-height, run 2.5 mm forward and bend 90 deg
# down through the pads. Nominal size, same palette.
# Frame of footprint Supercap_Radial_D8.0mm_L12.0mm_P3.50mm_Horizontal: pads at the origin
# line, the can toward +X from x = 2.5 to 14.5, resting on the board (z = 0 .. 8). KiCad
# model +Y is footprint -Y, so pad 1 (+, footprint y -1.75) is at y = +1.75 here and the
# negative band faces -Y.
D, L = 8.0, 12.0        # can diameter and length
P, LEAD_D = 3.5, 0.6    # lead pitch and diameter
GAP = 2.5               # end face to pad line
ZC = D / 2              # can axis height
SEAL_T = 0.3            # rubber seal at the lead end
TOP_R = 3.2             # exposed aluminium end, inside the sleeve's rolled-over edge
TOP_INSET, TOP_T = 0.25, 0.1
LEAD_BELOW = 3.0        # through a 1.6 mm board and ~1.4 mm out the far side
STRIPE_T, STRIPE_HALF_DEG = 0.03, 28

NAVY = cq.Color(0.10, 0.15, 0.45)
ALU = cq.Color(0.80, 0.80, 0.82)
STEEL = cq.Color(0.55, 0.55, 0.58)
STRIPE = cq.Color(0.86, 0.88, 0.93)
RUBBER = cq.Color(0.15, 0.15, 0.16)


def xcyl(r, x0, x1):
    """Cylinder on the can axis from x0 to x1 (x0 < x1, toward +X)."""
    return (cq.Workplane("YZ").workplane(offset=x0).center(0, ZC)
            .circle(r).extrude(x1 - x0))


x_face = GAP                  # lead end face (seal side)
x_end = GAP + L               # aluminium end
sleeve = (xcyl(D / 2, x_face + SEAL_T, x_end)
          .faces(">X").edges().fillet(0.4)
          .cut(xcyl(TOP_R, x_end - TOP_INSET - TOP_T, x_end + 1)))
top = xcyl(TOP_R - 0.02, x_end - TOP_INSET - TOP_T, x_end - TOP_INSET)
seal = xcyl(D / 2 - 0.4, x_face, x_face + SEAL_T)

# negative band: a thin shell sector on the sleeve, centred on -Y (pad 2's side)
band = (xcyl(D / 2 + STRIPE_T, x_face + SEAL_T + 0.2, x_end - 0.6)
        .cut(xcyl(D / 2, x_face, x_end)))
c, s = math.cos(math.radians(STRIPE_HALF_DEG)), math.sin(math.radians(STRIPE_HALF_DEG))
wedge = (cq.Workplane("YZ").workplane(offset=x_face).center(0, ZC)
         .polyline([(0, 0), (-10 * c, -10 * s), (-10, 0), (-10 * c, 10 * s)])
         .close().extrude(L + 1))
# the YZ workplane's local x is world Y, so the wedge opens toward -Y
band = band.intersect(wedge)

leads = None
for y in (P / 2, -P / 2):
    run = (cq.Workplane("XY").workplane(offset=ZC).center(0, y)
           .sphere(LEAD_D / 2)
           .union(cq.Workplane("YZ").workplane(offset=0).center(y, ZC)
                  .circle(LEAD_D / 2).extrude(x_face)))
    drop = (cq.Workplane("XY").workplane(offset=-LEAD_BELOW).center(0, y)
            .circle(LEAD_D / 2).extrude(ZC + LEAD_BELOW))
    lead = run.union(drop)
    leads = lead if leads is None else leads.union(lead)

assy = cq.Assembly(name="CXHP2R7205R_horizontal")
assy.add(sleeve, name="sleeve", color=NAVY)
assy.add(top, name="top", color=ALU)
assy.add(seal, name="seal", color=RUBBER)
assy.add(band, name="negative_band", color=STRIPE)
assy.add(leads, name="leads", color=STEEL)
assy.export("Supercap_Radial_D8.0mm_L12.0mm_P3.50mm_Horizontal.step")
