import cadquery as cq

# Taoglas GVLB258.A single-feed stacked L1/L5 ceramic patch, datasheet SPE-21-8-082-G,
# section 3 "Mechanical Drawing": lower ceramic 25 x 25 (+/-0.4), upper ceramic 18 x 18
# (+/-0.3) with one corner cut, 8.12 (+/-0.4) tall, feed pin 0.8 dia on the centre line
# 2.5 (+/-0.3) mm off centre, protruding 2.4 (+/-0.4) below the ceramic, and a solder
# bump of 0.5 max on the top electrode over the feed.
# Drawn in the frame of footprint ANT_GVLB258.A_TAOGLAS: origin at the patch centre,
# seated on the board at z = 0. KiCad model +Y is footprint -Y, so the feed that sits
# 2.5 mm below centre in the footprint (y = +2.5) is at y = -2.5 here, and the upper
# ceramic's cut corner is at +X/+Y, where the datasheet front view shows it.
LOWER, UPPER = 25.0, 18.0
TOTAL = 8.12
LOWER_T = 4.0            # side view: the two ceramics are about equal thickness
ELECTRODE_T = 0.02       # silver electrodes, kept off the ceramic faces' planes
UPPER_Z0 = LOWER_T + ELECTRODE_T
CORNER_R = 1.5           # rounded corners of the lower ceramic
CUT = 1.5                # corner cut of the upper ceramic
FEED_Y = -2.5
PIN_D, PIN_BELOW = 0.8, 2.4
BUMP_D, BUMP_T = 1.6, 0.3

CERAMIC = cq.Color(0.93, 0.91, 0.86)
SILVER = cq.Color(0.80, 0.80, 0.82)
TIN = cq.Color(0.72, 0.72, 0.74)


def cut_square(side, cut, z0, t):
    h = side / 2
    pts = [(-h, -h), (h, -h), (h, h - cut), (h - cut, h), (-h, h)]
    return cq.Workplane("XY").workplane(offset=z0).polyline(pts).close().extrude(t)


lower = (cq.Workplane("XY").box(LOWER, LOWER, LOWER_T, centered=(True, True, False))
         .edges("|Z").fillet(CORNER_R))
lower_patch = (cq.Workplane("XY").workplane(offset=LOWER_T)
               .rect(LOWER - 2.0, LOWER - 2.0).extrude(ELECTRODE_T)
               .edges("|Z").fillet(CORNER_R - 0.5))
upper = cut_square(UPPER, CUT, UPPER_Z0, TOTAL - UPPER_Z0)
upper_patch = cut_square(UPPER - 1.6, CUT + 0.8, TOTAL, ELECTRODE_T)
bump = (cq.Workplane("XY").workplane(offset=TOTAL + ELECTRODE_T)
        .center(0, FEED_Y).circle(BUMP_D / 2).extrude(BUMP_T))
pin = (cq.Workplane("XY").workplane(offset=-PIN_BELOW)
       .center(0, FEED_Y).circle(PIN_D / 2).extrude(PIN_BELOW))

assy = cq.Assembly(name="GVLB258.A")
assy.add(lower, name="lower_ceramic", color=CERAMIC)
assy.add(lower_patch, name="lower_patch", color=SILVER)
assy.add(upper, name="upper_ceramic", color=CERAMIC)
assy.add(upper_patch, name="upper_patch", color=SILVER)
assy.add(bump, name="feed_bump", color=TIN)
assy.add(pin, name="feed_pin", color=TIN)
assy.export("ANT_GVLB258.A_TAOGLAS.step")
