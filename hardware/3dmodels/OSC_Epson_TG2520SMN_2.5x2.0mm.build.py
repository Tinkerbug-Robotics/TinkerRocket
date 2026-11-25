import cadquery as cq

# Epson TG2520SMN TCXO, spec TG2016/2520SMN_EN ver.2.0 page 1 outline: body 2.5 x 2.0 (+/-0.2),
# 0.8 (+/-0.1) tall, ceramic package under a metal lid, notched corners, four corner terminals
# 0.6 x 0.55 with pin 1's inner corner chamfered C0.2. Nominal values; the same part as
# Oscillator_SMD_SeikoEpson_TG2520SMN-xxx-xxxxxx-4Pin_2.5x2.0mm.step, redrawn for this repo's
# footprint, which turns the package a quarter turn from KiCad's (long side on X).
# Frame of footprint OSC_Epson_TG2520SMN_2.5x2.0mm: origin at the body centre, long side on X,
# seated at z = 0, pins counter-clockwise from pin 1 at footprint bottom-left (-1.05, 0.7).
# KiCad model +Y is footprint -Y, so pin 1 is the bottom-left terminal here too (y < 0).
L_X, L_Y, H = 2.5, 2.0, 0.8
BASE_H = 0.7
LID_INSET, LID_R = 0.15, 0.15
NOTCH_R = 0.15
TERM_X, TERM_Y, TERM_T = 0.55, 0.6, 0.02
CHAMFER = 0.2

CERAMIC = cq.Color(0.90, 0.88, 0.82)
LID = cq.Color(0.78, 0.78, 0.80)
GOLD = cq.Color(0.85, 0.70, 0.30)

corners = [(sx * L_X / 2, sy * L_Y / 2) for sx in (-1, 1) for sy in (-1, 1)]
base = cq.Workplane("XY").workplane(offset=TERM_T).rect(L_X, L_Y).extrude(BASE_H - TERM_T)
for x, y in corners:
    base = base.cut(cq.Workplane("XY").center(x, y).circle(NOTCH_R).extrude(H))
lid = (cq.Workplane("XY").workplane(offset=BASE_H)
       .rect(L_X - 2 * LID_INSET, L_Y - 2 * LID_INSET).extrude(H - BASE_H)
       .edges("|Z").fillet(LID_R))

assy = cq.Assembly(name="TG2520SMN")
assy.add(base, name="package", color=CERAMIC)
assy.add(lid, name="lid", color=LID)
cx, cy = L_X / 2 - TERM_X / 2, L_Y / 2 - TERM_Y / 2
# model-frame signs: 1 bottom-left, 2 bottom-right, 3 top-right, 4 top-left
for n, (sx, sy) in {1: (-1, -1), 2: (1, -1), 3: (1, 1), 4: (-1, 1)}.items():
    x, y = sx * cx, sy * cy
    term = cq.Workplane("XY").center(x, y).rect(TERM_X, TERM_Y).extrude(TERM_T)
    for nx, ny in corners:
        term = term.cut(cq.Workplane("XY").center(nx, ny).circle(NOTCH_R).extrude(TERM_T))
    if n == 1:
        # chamfer the corner that points at the package centre
        ix, iy = x + TERM_X / 2, y + TERM_Y / 2
        tri = (cq.Workplane("XY").polyline([(ix, iy), (ix - CHAMFER, iy), (ix, iy - CHAMFER)])
               .close().extrude(TERM_T))
        term = term.cut(tri)
    assy.add(term, name=f"terminal{n}", color=GOLD)
assy.export("OSC_Epson_TG2520SMN_2.5x2.0mm.step")
