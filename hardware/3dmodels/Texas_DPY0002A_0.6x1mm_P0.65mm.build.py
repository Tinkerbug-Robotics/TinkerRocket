import cadquery as cq

# TI DPY0002A (X1SON-2, 0402 size), package outline 4224561/C 07/2024 in the TPD1E0B04
# data sheet: body 1.0 x 0.6 (0.9-1.1 x 0.5-0.7), 0.30-0.45 tall, two bottom terminals
# 0.2-0.3 wide x 0.45-0.55 long on a 0.65 mm pitch. Nominal values here. KiCad ships the
# DPY0002A footprint but not this model (its model path names a file that is not in the
# KiCad 10 library), hence this one.
# Frame of footprint Texas_DPY0002A_0.6x1mm_P0.65mm: origin at the body centre, pin 1 at
# -X, seated at z = 0.
L, W, H = 1.0, 0.6, 0.375
TERM_X, TERM_Y, TERM_T = 0.25, 0.5, 0.02
PITCH = 0.65

BODY = cq.Color(0.10, 0.10, 0.11)
TIN = cq.Color(0.80, 0.80, 0.82)
MARK = cq.Color(0.30, 0.30, 0.32)

body = (cq.Workplane("XY").workplane(offset=TERM_T)
        .rect(L, W).extrude(H - TERM_T))
# Pin 1 index area on the top face, as shaded on the package outline.
index = (cq.Workplane("XY").workplane(offset=H)
         .center(-L / 4, 0).rect(L / 2 - 0.1, W - 0.1).extrude(0.005))

assy = cq.Assembly(name="DPY0002A")
assy.add(body, name="body", color=BODY)
assy.add(index, name="pin1_index", color=MARK)
for n, x in ((1, -PITCH / 2), (2, PITCH / 2)):
    term = cq.Workplane("XY").center(x, 0).rect(TERM_X, TERM_Y).extrude(TERM_T)
    assy.add(term, name=f"terminal{n}", color=TIN)
assy.export("Texas_DPY0002A_0.6x1mm_P0.65mm.step")
