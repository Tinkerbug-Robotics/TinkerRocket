import cadquery as cq

# Laird BMI-S-205-F, the frame of a two-piece board-level shield, per Laird's product page:
# 38.10 x 25.40 x 6.00 mm, 0.20 mm cold-rolled steel, matte tin. The page gives no drawing of
# the top, so the frame is drawn as 0.20 walls with a 1.0 mm inward lip at the top. Its cover,
# BMI-S-205-C, is the separate model Laird_Technologies_BMI-S-205-C_38.56x25.86mm.step.
# Frame of footprint Laird_Technologies_BMI-S-205-F_38.10x25.40mm: origin at the frame centre,
# seated at z = 0; the walls sit on the footprint's 1.0 mm pad ring.
L_X, L_Y, H = 38.10, 25.40, 6.00
T = 0.20
LIP = 1.0

TIN = cq.Color(0.78, 0.78, 0.80)

walls = (cq.Workplane("XY").rect(L_X, L_Y).extrude(H)
         .cut(cq.Workplane("XY").rect(L_X - 2 * T, L_Y - 2 * T).extrude(H)))
lip = (cq.Workplane("XY").workplane(offset=H - T).rect(L_X - 2 * T, L_Y - 2 * T).extrude(T)
       .cut(cq.Workplane("XY").workplane(offset=H - T)
            .rect(L_X - 2 * (T + LIP), L_Y - 2 * (T + LIP)).extrude(T)))

assy = cq.Assembly(name="BMI-S-205-F")
assy.add(walls.union(lip), name="frame", color=TIN)
assy.export("Laird_Technologies_BMI-S-205-F_38.10x25.40mm.step")
