import cadquery as cq

# 1.1 x 0.9 mm five-terminal GNSS SAW filter, for footprint FIL_SAW-5_1.1x0.9mm_Murata: Murata
# SAFFB 1109 (SAFFB1G17AA0F0A, 0.5 mm max) and Tai-Saw TA2471A (DigiKey: 0.55 mm max), which
# share this land. Nominal 1.1 x 0.9 x 0.5 body, 0.20 x 0.25 terminals on the land centres
# (0.4 mm column pitch, rows at +/-0.25; port 1 alone mid-left, port 4 top right).
# Drawn in the footprint's frame: origin at the body centre, seated at z = 0. KiCad model +Y is
# footprint -Y, so footprint (x, y) is (x, -y) here.
L, W, H = 1.1, 0.9, 0.5
PAD_X, PAD_Y, PAD_T = 0.20, 0.25, 0.02
SUBSTRATE_T = 0.15
PADS_FP = {1: (-0.4, 0.0), 2: (0.0, 0.25), 3: (0.4, 0.25), 4: (0.4, -0.25), 5: (0.0, -0.25)}

LID = cq.Color(0.13, 0.13, 0.14)
SUBSTRATE = cq.Color(0.55, 0.36, 0.24)
GOLD = cq.Color(0.85, 0.70, 0.30)
MARK = cq.Color(0.75, 0.75, 0.75)

substrate = (cq.Workplane("XY").workplane(offset=PAD_T)
             .rect(L, W).extrude(SUBSTRATE_T))
lid = (cq.Workplane("XY").workplane(offset=PAD_T + SUBSTRATE_T)
       .rect(L, W).extrude(H - PAD_T - SUBSTRATE_T))
# Pin 1 marking at the corner the footprint's fab chamfer marks (top left).
dot = (cq.Workplane("XY").workplane(offset=H)
       .center(-0.38, 0.28).circle(0.06).extrude(0.005))

assy = cq.Assembly(name="SAW_1109")
assy.add(substrate, name="substrate", color=SUBSTRATE)
assy.add(lid, name="lid", color=LID)
assy.add(dot, name="pin1_mark", color=MARK)
for n, (x, y) in PADS_FP.items():
    pad = cq.Workplane("XY").center(x, -y).rect(PAD_X, PAD_Y).extrude(PAD_T)
    assy.add(pad, name=f"pad{n}", color=GOLD)
assy.export("FIL_SAW-5_1.1x0.9mm_Murata.step")
