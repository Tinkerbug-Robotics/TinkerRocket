import cadquery as cq

# TI VQFN-HR-11 'VAH' package, drawing VAH0011B (TPS1HC08-Q1 datasheet SLVSI27, package outline p.67):
# body 2.2 x 3.6 mm nominal (2.1-2.3 x 3.5-3.7), 0.8-1.0 mm tall, terminals flush with the underside
# (0.00-0.05 mm standoff). Drawn in the frame of footprint VAH0011B: origin at the package centre, seated at z = 0,
# pin 1 (VOUT) at the -X/+Y corner. KiCad model +Y is footprint -Y, so the footprint's top edge (pins 1/11, y < 0)
# is +Y here. The terminals are shown only as the pin-1 dot; the land pattern carries the real pad shapes.
BODY_X, BODY_Y, BODY_Z = 2.2, 3.6, 0.9
DOT_D, DOT_T = 0.3, 0.02

BODY = cq.Color(0.15, 0.15, 0.15)
MARK = cq.Color(0.85, 0.85, 0.85)

body = cq.Workplane("XY").box(BODY_X, BODY_Y, BODY_Z, centered=(True, True, False))
dot = (cq.Workplane("XY").workplane(offset=BODY_Z)
       .center(-BODY_X / 2 + 0.35, BODY_Y / 2 - 0.35).circle(DOT_D / 2).extrude(DOT_T))

assy = cq.Assembly(name="VAH0011B")
assy.add(body, name="body", color=BODY)
assy.add(dot, name="pin1_mark", color=MARK)
assy.export("VAH0011B.step")
