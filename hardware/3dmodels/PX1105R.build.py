import cadquery as cq

# SkyTraq PX1105R, data sheet rev 2 (2024-12-27): "Dimension 10.1mm L x 9.7mm W x 2.9mm H",
# page 11 "Mechanical Dimensions": 9.7 x 10.1 mm body, nine castellated pads per long side on a
# 1.1 mm pitch, the end pads 0.65 mm from each end, each pad 1.0 x 0.7 mm running in from the
# edge with a half-hole at the edge. The PCB and shield thicknesses are not published; 0.8 mm
# of module PCB under a shield that makes up the rest of the 2.9 mm is an assumption.
# Drawn in the frame of footprint PX1105R, whose pad pattern is centred on (+0.06, -0.03), i.e.
# (+0.06, +0.03) here (KiCad model +Y is footprint -Y). Pad 1 is the right-hand column's
# bottom pad in the footprint, so the right-hand column's lowest pad here too.
L_X, L_Y, H = 9.7, 10.1, 2.9
CX, CY = 0.06, 0.03
PCB_T = 0.8
PAD_T = 0.03
PITCH, END = 1.1, 0.65
PAD_IN, PAD_W = 1.0, 0.7          # pad length in from the edge, pad width along the edge
NOTCH_D = 0.5
SHIELD_INSET_X, SHIELD_INSET_Y = 1.1, 0.3   # shield stays clear of the castellated pads

PCB = cq.Color(0.10, 0.33, 0.20)
GOLD = cq.Color(0.85, 0.70, 0.30)
SILVER = cq.Color(0.80, 0.80, 0.82)
DOT = cq.Color(0.20, 0.20, 0.22)

x_edges = (CX - L_X / 2, CX + L_X / 2)
ys = [CY + L_Y / 2 - END - k * PITCH for k in range(9)]

pcb = (cq.Workplane("XY").workplane(offset=PAD_T)
       .center(CX, CY).rect(L_X, L_Y).extrude(PCB_T))
for x in x_edges:
    for y in ys:
        pcb = pcb.cut(cq.Workplane("XY").center(x, y).circle(NOTCH_D / 2).extrude(PCB_T + 1))

pads = None
for side, x in zip((1, -1), x_edges):          # side: +1 = pad runs in to +X (left edge)
    for y in ys:
        xc = x + side * PAD_IN / 2
        bottom = cq.Workplane("XY").center(xc, y).rect(PAD_IN, PAD_W).extrude(PAD_T)
        top = (cq.Workplane("XY").workplane(offset=PAD_T + PCB_T)
               .center(xc, y).rect(PAD_IN, PAD_W).extrude(0.02))
        wall = (cq.Workplane("XY").center(x, y).circle(NOTCH_D / 2).extrude(PCB_T + PAD_T)
                .cut(cq.Workplane("XY").center(x, y).circle(NOTCH_D / 2 - 0.03).extrude(PCB_T + PAD_T))
                .intersect(cq.Workplane("XY").center(CX, CY).rect(L_X, L_Y).extrude(PCB_T + PAD_T)))
        for part in (bottom.cut(cq.Workplane("XY").center(x, y).circle(NOTCH_D / 2).extrude(1)),
                     top.cut(cq.Workplane("XY").center(x, y).circle(NOTCH_D / 2).extrude(5)),
                     wall):
            pads = part if pads is None else pads.union(part)

shield_z = PAD_T + PCB_T
shield = (cq.Workplane("XY").workplane(offset=shield_z)
          .center(CX, CY).rect(L_X - 2 * SHIELD_INSET_X, L_Y - 2 * SHIELD_INSET_Y)
          .extrude(H - shield_z)
          .edges(">Z").chamfer(0.15))
# pin 1 marker on the shield, toward the pad 1 corner (right column, lowest pad)
dot = (cq.Workplane("XY").workplane(offset=H)
       .center(x_edges[1] - SHIELD_INSET_X - 0.9, ys[-1] + 0.3).circle(0.3).extrude(0.01))

assy = cq.Assembly(name="PX1105R")
assy.add(pcb, name="module_pcb", color=PCB)
assy.add(pads, name="castellations", color=GOLD)
assy.add(shield, name="shield", color=SILVER)
assy.add(dot, name="pin1_mark", color=DOT)
assy.export("PX1105R.step")
