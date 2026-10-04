import cadquery as cq

# Bourns SRF3225TP series shielded common-mode choke (SRF3225TP datasheet as downloaded
# 2026-10-04 from bourns.com, "Product Dimensions"): body 3.2 x 2.5 (+/-0.2), 2.2 (+/-0.2) tall, ferrite
# sleeve over a drum core. Four bottom terminals, 0.65 x 1.0 (+/-0.1), at the corners;
# 0.3 standoff on the side view is the terminal thickness. Top marking "501".
# Drawn in the frame of footprint FIL_SRF3225TP (origin at the body centre, seated at
# z = 0). KiCad model +Y is footprint -Y; the body is symmetric, so it does not matter here.
L, W, H = 3.2, 2.5, 2.2
TERM_X, TERM_Y, TERM_Z = 0.65, 1.0, 0.3
SLEEVE = cq.Color(0.16, 0.16, 0.17)
TIN = cq.Color(0.80, 0.80, 0.82)
MARK = cq.Color(0.75, 0.75, 0.75)

body = (cq.Workplane("XY").workplane(offset=TERM_Z * 0.5)
        .rect(L, W).extrude(H - TERM_Z * 0.5)
        .edges("|Z").fillet(0.15))
mark = (cq.Workplane("XY").workplane(offset=H)
        .text("501", 0.6, 0.005, kind="regular"))
assy = cq.Assembly(name="SRF3225TP")
assy.add(body, name="body", color=SLEEVE)
assy.add(mark, name="mark", color=MARK)
for sx in (-1, 1):
    for sy in (-1, 1):
        x = sx * (L / 2 - TERM_X / 2)
        y = sy * (W / 2 - TERM_Y / 2)
        term = cq.Workplane("XY").center(x, y).rect(TERM_X, TERM_Y).extrude(TERM_Z)
        assy.add(term, name=f"term_{sx}_{sy}", color=TIN)
assy.export("FIL_SRF3225TP.step")
