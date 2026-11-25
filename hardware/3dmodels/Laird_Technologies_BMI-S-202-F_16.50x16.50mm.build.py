import cadquery as cq

# Laird BMI-S-202-F, the frame of a two-piece board-level shield. DigiKey 903-1051-1-ND lists
# 16.50 x 16.50 x 3.60 mm; Laird gives no drawing of the top, so, like the BMI-S-205-F model,
# the frame is drawn as 0.20 mm walls with a 1.0 mm inward lip at the top. Its cover,
# BMI-S-202-C, is the separate model Laird_Technologies_BMI-S-202-C_16.96x16.96mm.step.
# Frame of footprint Laird_Technologies_BMI-S-202-F_16.50x16.50mm: origin at the frame centre,
# seated at z = 0; the walls sit on the footprint's pad ring.
L_X, L_Y, H = 16.50, 16.50, 3.60
T = 0.20
LIP = 1.0

TIN = cq.Color(0.78, 0.78, 0.80)

walls = (cq.Workplane("XY").rect(L_X, L_Y).extrude(H)
         .cut(cq.Workplane("XY").rect(L_X - 2 * T, L_Y - 2 * T).extrude(H)))
lip = (cq.Workplane("XY").workplane(offset=H - T).rect(L_X - 2 * T, L_Y - 2 * T).extrude(T)
       .cut(cq.Workplane("XY").workplane(offset=H - T)
            .rect(L_X - 2 * (T + LIP), L_Y - 2 * (T + LIP)).extrude(T)))

assy = cq.Assembly(name="BMI-S-202-F")
assy.add(walls.union(lip), name="frame", color=TIN)
assy.export("Laird_Technologies_BMI-S-202-F_16.50x16.50mm.step")
