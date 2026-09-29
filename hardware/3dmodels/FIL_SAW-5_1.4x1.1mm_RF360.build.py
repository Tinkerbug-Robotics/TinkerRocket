import cadquery as cq

# Qualcomm (RF360) B8389 L1/L5 double-hump SAW filter, B39162B8389P810, data sheet of
# 2022-11-15, section 3 "Package": body 1.4 x 1.1 (+/-0.1), 0.45 max tall, five
# 0.25 x 0.325 terminals. Pin 1 (input) sits alone on the body's centre row, 0.5 left of
# the middle column; pins 2/5 and 3/4 form the two right-hand columns at +/-0.2875.
# Drawn in the frame of footprint FIL_SAW-5_1.4x1.1mm_RF360 (origin at the body centre,
# seated at z = 0). KiCad model +Y is footprint -Y: the land-pattern "thru view" puts pins
# 5 and 4 on the upper row, so they are at y = +0.2875 here.
L, W, H = 1.4, 1.1, 0.45
PAD_X, PAD_Y, PAD_T = 0.25, 0.325, 0.02
SUBSTRATE_T = 0.15
PADS = {1: (-0.5, 0.0), 2: (0.0, -0.2875), 3: (0.5, -0.2875),
        4: (0.5, 0.2875), 5: (0.0, 0.2875)}

LID = cq.Color(0.13, 0.13, 0.14)
SUBSTRATE = cq.Color(0.55, 0.36, 0.24)
GOLD = cq.Color(0.85, 0.70, 0.30)
MARK = cq.Color(0.75, 0.75, 0.75)

substrate = (cq.Workplane("XY").workplane(offset=PAD_T)
             .rect(L, W).extrude(SUBSTRATE_T))
lid = (cq.Workplane("XY").workplane(offset=PAD_T + SUBSTRATE_T)
       .rect(L, W).extrude(H - PAD_T - SUBSTRATE_T))
# Pin 1 marking: the top view's dot sits at the corner below pin 1.
dot = (cq.Workplane("XY").workplane(offset=H)
       .center(-0.5, -0.35).circle(0.07).extrude(0.005))

assy = cq.Assembly(name="B8389")
assy.add(substrate, name="substrate", color=SUBSTRATE)
assy.add(lid, name="lid", color=LID)
assy.add(dot, name="pin1_mark", color=MARK)
for n, (x, y) in PADS.items():
    pad = cq.Workplane("XY").center(x, y).rect(PAD_X, PAD_Y).extrude(PAD_T)
    assy.add(pad, name=f"pad{n}", color=GOLD)
assy.export("FIL_SAW-5_1.4x1.1mm_RF360.step")
