import cadquery as cq

# Bourns SRP4020FA shielded power inductor (metal-alloy powder core, flat wire),
# from the SRP4020FA datasheet's Product Dimensions: body 4.1 x 4.1 (+/-0.2),
# 1.9 (+/-0.2) tall; two bottom-only terminals 0.88 (+/-0.2) deep by 3.4 (+/-0.3)
# long with a 1.6 (+/-0.25) gap between them, so they sit ~0.37 in from the body
# edge. Drawn at nominal size in the footprint frame of IND_SRP4020FA-2R2M:
# origin at the body centre, pad 1 on -X, seated on the board at z = 0.
BODY, H = 4.1, 1.9          # body side and overall height
CORNER_R = 0.3              # rounded body corners in the top view
TERM_D, TERM_L = 0.88, 3.4  # terminal depth (X) and length (Y)
GAP = 1.6                   # between the terminals' inner edges
TERM_T = 0.08               # terminal plating, proud of the body bottom
SEAT = 0.04                 # body bottom; the terminals overlap it, no shared faces

GREY = cq.Color(0.20, 0.20, 0.22)
TIN = cq.Color(0.75, 0.76, 0.78)

body = (cq.Workplane("XY").workplane(offset=SEAT)
        .rect(BODY, BODY).extrude(H - SEAT)
        .edges("|Z").fillet(CORNER_R))

x0 = GAP / 2 + TERM_D / 2
terms = [cq.Workplane("XY").center(sx * x0, 0).rect(TERM_D, TERM_L).extrude(TERM_T)
         for sx in (-1, 1)]

assy = cq.Assembly(name="IND_SRP4020FA-2R2M")
assy.add(body, name="body", color=GREY)
for i, t in enumerate(terms):
    assy.add(t, name=f"terminal{i + 1}", color=TIN)
assy.export("IND_SRP4020FA-2R2M.step")
