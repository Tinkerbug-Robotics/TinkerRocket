import cadquery as cq

# Laird BMI-S-205-C, the snap-on cover of the BMI-S-205-F frame, per Laird's product page:
# 38.56 x 25.86 x 2.00 mm, 0.13 mm cold-rolled steel, matte tin. Drawn fitted: its top rests
# on the frame's 6.00 mm top, so the shield stands 6.13 mm and the skirt reaches down to 4.13.
# Same frame as footprint Laird_Technologies_BMI-S-205-F_38.10x25.40mm (origin at the frame
# centre, board at z = 0), so it loads as a second model on that footprint.
L_X, L_Y, H = 38.56, 25.86, 2.00
T = 0.13
FRAME_H = 6.00

TIN = cq.Color(0.78, 0.78, 0.80)

top = FRAME_H + T
cover = (cq.Workplane("XY").workplane(offset=top - H).rect(L_X, L_Y).extrude(H)
         .cut(cq.Workplane("XY").workplane(offset=top - H)
              .rect(L_X - 2 * T, L_Y - 2 * T).extrude(H - T)))

assy = cq.Assembly(name="BMI-S-205-C")
assy.add(cover, name="cover", color=TIN)
assy.export("Laird_Technologies_BMI-S-205-C_38.56x25.86mm.step")
