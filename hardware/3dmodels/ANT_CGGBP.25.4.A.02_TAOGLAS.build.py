import cadquery as cq

# Taoglas CGGBP.25.4.A.02 single-feed L1 wideband ceramic patch, datasheet SPE-14-8-071-G, section 3
# "Mechanical Drawing": ceramic 25.1 x 25.1 (+/-0.35) x 4 (+/-0.4) mm with rounded corners, a silver
# top electrode (about 19.6 mm square, one corner cut) and a solder bump of 1 mm max over the feed,
# feed pin 0.9 (+/-0.15) dia on the centre line 2.5 (+/-0.5) mm off centre, protruding 2.4 (+/-0.4)
# below the ceramic.
# Drawn in the frame of footprint ANT_CGGBP.25.4.A.02_TAOGLAS: origin at the patch centre, seated on
# the board at z = 0. KiCad model +Y is footprint -Y, so the feed that sits 2.5 mm below centre in
# the footprint (y = +2.5) is at y = -2.5 here, and the electrode's cut corner is at +X/+Y, where the
# datasheet top view shows it.
SIDE, T = 25.1, 4.0
CORNER_R = 1.5
ELECTRODE, CUT = 19.6, 1.5
ELECTRODE_T = 0.02
FEED_Y = -2.5
PIN_D, PIN_BELOW = 0.9, 2.4
BUMP_D, BUMP_T = 1.7, 0.4

CERAMIC = cq.Color(0.93, 0.91, 0.86)
SILVER = cq.Color(0.80, 0.80, 0.82)
TIN = cq.Color(0.72, 0.72, 0.74)

ceramic = (cq.Workplane("XY").box(SIDE, SIDE, T, centered=(True, True, False))
           .edges("|Z").fillet(CORNER_R))
h = ELECTRODE / 2
pts = [(-h, -h), (h, -h), (h, h - CUT), (h - CUT, h), (-h, h)]
electrode = cq.Workplane("XY").workplane(offset=T).polyline(pts).close().extrude(ELECTRODE_T)
bump = (cq.Workplane("XY").workplane(offset=T + ELECTRODE_T)
        .center(0, FEED_Y).circle(BUMP_D / 2).extrude(BUMP_T))
pin = (cq.Workplane("XY").workplane(offset=-PIN_BELOW)
       .center(0, FEED_Y).circle(PIN_D / 2).extrude(PIN_BELOW))

assy = cq.Assembly(name="CGGBP.25.4.A.02")
assy.add(ceramic, name="ceramic", color=CERAMIC)
assy.add(electrode, name="electrode", color=SILVER)
assy.add(bump, name="feed_bump", color=TIN)
assy.add(pin, name="feed_pin", color=TIN)
assy.export("ANT_CGGBP.25.4.A.02_TAOGLAS.step")
