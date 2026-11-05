import cadquery as cq

# Bourns FDP2009N122157H4 LTCC GNSS L1/L5 diplexer, data sheet page 2 "Product Dimensions": body
# 2.00 x 1.25 x 0.90 (+/-0.15, +/-0.15, +/-0.10), six terminals 0.30 wide on a 0.65 pitch (0.20 in
# from each end), wrapped over both long sides and 0.20 onto the top and the bottom, and a 0.3 dot
# on the top over the pin 1 / pin 6 column. Nominal values.
# Frame of footprint Footprints:FDP2009N122157H4: origin at the body centre, long side on X, seated
# at z = 0. KiCad model +Y is footprint -Y, so pin 1 (footprint (-0.65, 0.675)) is the bottom-left
# terminal here and pin 6 (footprint (-0.65, -0.675)) the top-left one.
L_X, L_Y, H = 2.00, 1.25, 0.90
PITCH, TERM_W, WRAP, TERM_T = 0.65, 0.30, 0.20, 0.02
DOT_D = 0.30

CERAMIC = cq.Color(0.88, 0.88, 0.85)
SILVER = cq.Color(0.80, 0.80, 0.82)
MARK = cq.Color(0.35, 0.35, 0.38)

body = cq.Workplane("XY").workplane(offset=TERM_T).rect(L_X, L_Y).extrude(H - 2 * TERM_T)
dot = (cq.Workplane("XY").workplane(offset=H)
       .center(-PITCH, 0).circle(DOT_D / 2).extrude(0.005))

assy = cq.Assembly(name="FDP2009N122157H4")
assy.add(body, name="body", color=CERAMIC)
assy.add(dot, name="pin1_mark", color=MARK)
# terminal n at (x, side): pins 1-3 on the -Y side left to right, 4-6 on the +Y side right to left
pins = {1: (-PITCH, -1), 2: (0, -1), 3: (PITCH, -1), 4: (PITCH, 1), 5: (0, 1), 6: (-PITCH, 1)}
for n, (x, side) in pins.items():
    y_face = side * L_Y / 2
    # the wrap over the side face, then the bands on the bottom and the top
    term = (cq.Workplane("XY").center(x, y_face + side * TERM_T / 2)
            .rect(TERM_W, TERM_T).extrude(H))
    for z0 in (0, H - TERM_T):
        band = (cq.Workplane("XY").workplane(offset=z0)
                .center(x, y_face - side * WRAP / 2).rect(TERM_W, WRAP).extrude(TERM_T))
        term = term.union(band)
    assy.add(term, name=f"terminal{n}", color=SILVER)
assy.export("FDP2009N122157H4.step")
