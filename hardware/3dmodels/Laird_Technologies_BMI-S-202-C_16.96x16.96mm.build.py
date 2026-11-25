import cadquery as cq

# Laird BMI-S-202-C, the snap-on cover for the BMI-S-202-F frame. DigiKey 903-1014-ND lists
# 16.96 x 16.96 x 2.00 mm with vent holes. Drawn 0.15 mm steel: a lid with a 2.00 mm skirt,
# vented on a 5 x 5 grid of 1.0 mm holes (the pattern is illustrative, not Laird's).
# Placed in the frame footprint's coordinates as fitted: the lid rests on the frame's 3.60 mm
# top, so the cover spans z = 1.75 to 3.75. Attach it to the frame footprint as a second model.
L_X, L_Y, SKIRT = 16.96, 16.96, 2.00
T = 0.15
FRAME_H = 3.60
Z0 = FRAME_H + T - SKIRT
HOLE_D, PITCH, N = 1.0, 2.5, 5

TIN = cq.Color(0.78, 0.78, 0.80)

cover = (cq.Workplane("XY").workplane(offset=Z0).rect(L_X, L_Y).extrude(SKIRT)
         .cut(cq.Workplane("XY").workplane(offset=Z0).rect(L_X - 2 * T, L_Y - 2 * T)
              .extrude(SKIRT - T)))
pts = [((i - (N - 1) / 2) * PITCH, (j - (N - 1) / 2) * PITCH) for i in range(N) for j in range(N)]
holes = (cq.Workplane("XY").workplane(offset=FRAME_H).pushPoints(pts)
         .circle(HOLE_D / 2).extrude(T + 0.1))
cover = cover.cut(holes)

assy = cq.Assembly(name="BMI-S-202-C")
assy.add(cover, name="cover", color=TIN)
assy.export("Laird_Technologies_BMI-S-202-C_16.96x16.96mm.step")
