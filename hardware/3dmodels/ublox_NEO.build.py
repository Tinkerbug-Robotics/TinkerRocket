import cadquery as cq

# u-blox NEO-M8T, data sheet UBX-15025193 R10 section 5.1 (Figure 6, Table 15): 12.2 x 16.0 mm
# module (B x A), 2.4 mm tall (C), module PCB 0.82 mm (H), twelve castellated pads per long side on
# a 1.1 mm pitch (E) in groups of seven and five with a 3.0 mm gap (F), end pads 1.0 mm from each
# end (D, G), each pad 0.8 mm wide (K) and 0.9 mm in from the edge (M) around a 0.5 mm half-hole (N).
# Drawn in the frame of footprint ublox_NEO: origin at the module centre. KiCad model +Y is
# footprint -Y, so pin 1 (footprint (-6, -7)) is the left column's top pad at y = +7 here.
L_X, L_Y, H = 12.2, 16.0, 2.4
PCB_T = 0.82
PAD_T = 0.03
PAD_IN, PAD_W = 0.9, 0.8
NOTCH_D = 0.5
SHIELD_INSET_X, SHIELD_INSET_Y = 1.1, 0.3   # shield stays clear of the castellated pads

PCB = cq.Color(0.10, 0.33, 0.20)
GOLD = cq.Color(0.85, 0.70, 0.30)
SILVER = cq.Color(0.80, 0.80, 0.82)
DOT = cq.Color(0.20, 0.20, 0.22)

# pad centres along Y (model frame): pins 1-7 from +7.0 down by 1.1, then pins 8-12 from -2.6
left_ys = [7.0 - 1.1 * k for k in range(7)] + [-2.6 - 1.1 * k for k in range(5)]
x_edges = (-L_X / 2, L_X / 2)

pcb = cq.Workplane("XY").workplane(offset=PAD_T).rect(L_X, L_Y).extrude(PCB_T)
for x in x_edges:
    for y in left_ys:
        pcb = pcb.cut(cq.Workplane("XY").center(x, y).circle(NOTCH_D / 2).extrude(PCB_T + 1))

pads = None
for side, x in zip((1, -1), x_edges):          # side: +1 = pad runs in to +X (left edge)
    for y in left_ys:
        xc = x + side * PAD_IN / 2
        bottom = cq.Workplane("XY").center(xc, y).rect(PAD_IN, PAD_W).extrude(PAD_T)
        top = (cq.Workplane("XY").workplane(offset=PAD_T + PCB_T)
               .center(xc, y).rect(PAD_IN, PAD_W).extrude(0.02))
        wall = (cq.Workplane("XY").center(x, y).circle(NOTCH_D / 2).extrude(PCB_T + PAD_T)
                .cut(cq.Workplane("XY").center(x, y).circle(NOTCH_D / 2 - 0.03).extrude(PCB_T + PAD_T))
                .intersect(cq.Workplane("XY").rect(L_X, L_Y).extrude(PCB_T + PAD_T)))
        for part in (bottom.cut(cq.Workplane("XY").center(x, y).circle(NOTCH_D / 2).extrude(1)),
                     top.cut(cq.Workplane("XY").center(x, y).circle(NOTCH_D / 2).extrude(5)),
                     wall):
            pads = part if pads is None else pads.union(part)

shield_z = PAD_T + PCB_T
shield = (cq.Workplane("XY").workplane(offset=shield_z)
          .rect(L_X - 2 * SHIELD_INSET_X, L_Y - 2 * SHIELD_INSET_Y)
          .extrude(H - shield_z)
          .edges(">Z").chamfer(0.15))
# pin 1 marker on the shield, toward the pin 1 corner (left column, top pad)
dot = (cq.Workplane("XY").workplane(offset=H)
       .center(x_edges[0] + SHIELD_INSET_X + 0.9, left_ys[0] - 0.3).circle(0.3).extrude(0.01))

assy = cq.Assembly(name="ublox_NEO")
assy.add(pcb, name="module_pcb", color=PCB)
assy.add(pads, name="castellations", color=GOLD)
assy.add(shield, name="shield", color=SILVER)
assy.add(dot, name="pin1_mark", color=DOT)
assy.export("ublox_NEO.step")
