import math

import cadquery as cq

# Radial EDLC supercapacitor standing on the board: CDA CXHP(2.7V) series, radial
# lead type, spec date 2022-06-10 -- 2.0 F in an 8 x 12 can (D +/-1.0, L +/-1.5),
# leads 0.6 mm on a 3.5 mm pitch (+/-0.5). Drawn at nominal size in the footprint
# frame of Supercap_Radial_D8.0mm_H12.0mm_P3.50mm: pad 1 (+) at the origin, pad 2
# (-) at +3.5 mm on X, the can axis midway between them, seated on the board at z = 0.
# Same palette as the Beetle's Supercap_Radial_D10.0mm_L20.0mm_P5.00mm_Horizontal.
D, L = 8.0, 12.0        # can diameter and length
P, LEAD_D = 3.5, 0.6    # lead pitch and diameter
CX = P / 2              # can axis
SEAL_T = 0.3            # rubber seal visible under the can
TOP_R = 3.2             # exposed aluminium top, inside the sleeve's rolled-over edge
TOP_INSET = 0.25
LEAD_BELOW = 3.0        # through a 1.6 mm board and ~1.4 mm out the far side
STRIPE_T, STRIPE_HALF_DEG = 0.03, 28   # negative-side band, facing pad 2

NAVY = cq.Color(0.10, 0.15, 0.45)
ALU = cq.Color(0.80, 0.80, 0.82)
STEEL = cq.Color(0.55, 0.55, 0.58)
STRIPE = cq.Color(0.86, 0.88, 0.93)
RUBBER = cq.Color(0.15, 0.15, 0.16)


def cyl(r, z0, z1, x=CX):
    return cq.Workplane("XY").workplane(offset=z0).center(x, 0).circle(r).extrude(z1 - z0)


# The top disc fills the bottom of the recess, so no two faces share a plane
# (coplanar faces z-fight in the 3D viewer).
TOP_T = 0.1
sleeve = (cyl(D / 2, SEAL_T, L)
          .faces(">Z").edges().fillet(0.4)
          .cut(cyl(TOP_R, L - TOP_INSET - TOP_T, L + 1)))
top = cyl(TOP_R - 0.02, L - TOP_INSET - TOP_T, L - TOP_INSET)
seal = cyl(D / 2 - 0.4, 0.0, SEAL_T)

# A thin band on the sleeve, centred on +X (the negative lead's side), just
# proud of the sleeve so it reads in the 3D view.
band = cyl(D / 2 + STRIPE_T, SEAL_T + 0.5, L - 0.8).cut(cyl(D / 2 - 0.01, 0, L))
a = math.radians(STRIPE_HALF_DEG)
sector = (cq.Workplane("XY").workplane(offset=-1)
          .polyline([(CX, 0),
                     (CX + 6 * math.cos(a), -6 * math.sin(a)),
                     (CX + 6 * math.cos(a), 6 * math.sin(a))])
          .close().extrude(L + 2))
stripe = band.intersect(sector)

leads = [cyl(LEAD_D / 2, -LEAD_BELOW, SEAL_T + 0.5, x=x) for x in (0.0, P)]

assy = cq.Assembly(name="Supercap_Radial_D8_H12")
assy.add(sleeve, name="sleeve", color=NAVY)
assy.add(top, name="top", color=ALU)
assy.add(seal, name="seal", color=RUBBER)
assy.add(stripe, name="stripe", color=STRIPE)
for i, lead in enumerate(leads):
    assy.add(lead, name=f"lead{i}", color=STEEL)
assy.export("Supercap_Radial_D8.0mm_H12.0mm_P3.50mm.step")
