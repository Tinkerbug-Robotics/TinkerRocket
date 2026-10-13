import cadquery as cq

# Taoglas GVLB208.A single-feed stacked L1/L5 ceramic patch, datasheet SPE-25-8-318-B,
# section 3 "Mechanical Drawing": lower ceramic 20 x 20 (+/-0.2) x 6 (+/-0.1) with three
# R1 corners and a C1.7 corner cut, upper ceramic 18 x 18 (+/-0.2) x 2 (+/-0.1) with three
# R0.5 corners and a C2 cut on the same corner, 8.05 (+/-0.3) over both ceramics, a 0.6 max
# solder bump on the top electrode over the feed, and the feed pin 0.8 (+/-0.1) dia on the
# centre line 1.2 mm off centre, protruding 1.2 (+/-0.2) below the ceramic.
# Drawn in the frame of footprint ANT_GVLB208.A_TAOGLAS: origin at the patch centre,
# seated on the board at z = 0. KiCad model +Y is footprint -Y, so the feed that sits
# 1.2 mm below centre in the footprint (y = +1.2) is at y = -1.2 here, and the cut
# corners are at +X/+Y, where the datasheet top view shows them.
LOWER, UPPER = 20.0, 18.0
LOWER_T, UPPER_T = 6.0, 2.0
ELECTRODE_T = 0.02       # silver electrodes, kept off the ceramic faces' planes
UPPER_Z0 = LOWER_T + ELECTRODE_T
TOP = UPPER_Z0 + UPPER_T
LOWER_R, LOWER_CUT = 1.0, 1.7
UPPER_R, UPPER_CUT = 0.5, 2.0
TOP_ELECTRODE, TOP_ELECTRODE_CUT = 14.0, 1.5   # silver area on the top face, read off the top view
FEED_Y = -1.2
PIN_D, PIN_BELOW = 0.8, 1.2
BUMP_D, BUMP_T = 1.6, 0.6

CERAMIC = cq.Color(0.93, 0.91, 0.86)
SILVER = cq.Color(0.80, 0.80, 0.82)
TIN = cq.Color(0.72, 0.72, 0.74)


def cut_square(side, cut, radius, z0, t):
    """Square slab with one corner cut at +X/+Y and the other three corners rounded."""
    h = side / 2
    pts = [(-h, -h), (h, -h), (h, h - cut), (h - cut, h), (-h, h)]
    slab = cq.Workplane("XY").workplane(offset=z0).polyline(pts).close().extrude(t)
    if radius:
        zc = z0 + t / 2
        corners = [cq.selectors.NearestToPointSelector((x, y, zc)) for x, y in ((-h, -h), (h, -h), (-h, h))]
        slab = slab.edges(corners[0] + corners[1] + corners[2]).fillet(radius)
    return slab


lower = cut_square(LOWER, LOWER_CUT, LOWER_R, 0, LOWER_T)
lower_patch = cut_square(UPPER + 0.5, LOWER_CUT, 0, LOWER_T, ELECTRODE_T)
upper = cut_square(UPPER, UPPER_CUT, UPPER_R, UPPER_Z0, UPPER_T)
upper_patch = cut_square(TOP_ELECTRODE, TOP_ELECTRODE_CUT, 0, TOP, ELECTRODE_T)
bump = (cq.Workplane("XY").workplane(offset=TOP + ELECTRODE_T)
        .center(0, FEED_Y).circle(BUMP_D / 2).extrude(BUMP_T))
pin = (cq.Workplane("XY").workplane(offset=-PIN_BELOW)
       .center(0, FEED_Y).circle(PIN_D / 2).extrude(PIN_BELOW))

assy = cq.Assembly(name="GVLB208.A")
assy.add(lower, name="lower_ceramic", color=CERAMIC)
assy.add(lower_patch, name="lower_patch", color=SILVER)
assy.add(upper, name="upper_ceramic", color=CERAMIC)
assy.add(upper_patch, name="upper_patch", color=SILVER)
assy.add(bump, name="feed_bump", color=TIN)
assy.add(pin, name="feed_pin", color=TIN)
assy.export("ANT_GVLB208.A_TAOGLAS.step")
