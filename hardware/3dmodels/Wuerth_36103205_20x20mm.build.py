import cadquery as cq

# Würth WE-SHC shielding cabinet, 20 x 20 mm, two-piece: SMD frame 36103205 (data sheet rev 003.004
# page 1) and its snap-on cover 36003200 (rev 002.004 page 1). Frame: 21 x 21 outside, 20 x 20
# inside, 0.5 steel, 3.0 tall, a top flange with two chamfered openings either side of a bridge
# carrying the round pick-up area, and the wall's bottom edge relieved between the feet. Cover:
# 21.8 x 21.8 outside, 0.3 steel, 3.3 tall, sitting 3.5 tall on the frame. Nominal values; the
# openings, the relief and the cover's seat were scaled from the drawings. KiCad ships the
# footprint but not this model (its model path names a file that is not in the KiCad 10
# library), hence this one.
# Frame of footprint RF_Shielding:Wuerth_36103205_20x20mm: origin at the centre, seated at z = 0.
# Two files: Wuerth_36103205_20x20mm.step (the frame) and Wuerth_36003200_20x20mm_cover.step.
FRAME_OUT, FRAME_IN, FRAME_H, T = 21.0, 20.0, 3.0, 0.5
OPEN_X, OPEN_Y0, OPEN_Y1, OPEN_CH = 7.9, 0.95, 8.0, 1.8   # each opening: |x| < OPEN_X, OPEN_Y0 < |y| < OPEN_Y1
PICKUP_R = 1.65
# footprint pads: corners (1 x 1), 8.6 (2.3 long) and 3.75 (2.6 long) from the centre on each side;
# the wall is relieved RELIEF tall between them
RELIEF, RELIEF_SPANS = 0.3, [(-1.25, 1.25), (5.1, 7.4), (-7.4, -5.1)]
COVER_OUT, COVER_T, COVER_H, COVER_TOP = 21.8, 0.3, 3.3, 3.5

TIN = cq.Color(0.62, 0.63, 0.64)

frame = (cq.Workplane("XY").rect(FRAME_OUT, FRAME_OUT).extrude(FRAME_H)
         .cut(cq.Workplane("XY").rect(FRAME_IN, FRAME_IN).extrude(FRAME_H - T)))
for sy in (-1, 1):
    y0, y1 = sy * OPEN_Y0, sy * OPEN_Y1
    c = sy * OPEN_CH
    opening = (cq.Workplane("XY").workplane(offset=FRAME_H - T)
               .polyline([(-OPEN_X, y0), (OPEN_X, y0), (OPEN_X, y1 - c), (OPEN_X - OPEN_CH, y1),
                          (-OPEN_X + OPEN_CH, y1), (-OPEN_X, y1 - c)]).close().extrude(T))
    frame = frame.cut(opening)
frame = frame.union(cq.Workplane("XY").workplane(offset=FRAME_H - T).circle(PICKUP_R).extrude(T))
for a, b in RELIEF_SPANS:
    for along_x in (True, False):
        size = (b - a, FRAME_OUT) if along_x else (FRAME_OUT, b - a)
        centre = ((a + b) / 2, 0) if along_x else (0, (a + b) / 2)
        cut = (cq.Workplane("XY").center(*centre).rect(*size).extrude(RELIEF)
               .cut(cq.Workplane("XY").rect(FRAME_IN, FRAME_IN).extrude(RELIEF)))
        frame = frame.cut(cut)

cover = (cq.Workplane("XY").workplane(offset=COVER_TOP - COVER_H)
         .rect(COVER_OUT, COVER_OUT).extrude(COVER_H)
         .cut(cq.Workplane("XY").workplane(offset=COVER_TOP - COVER_H)
              .rect(COVER_OUT - 2 * COVER_T, COVER_OUT - 2 * COVER_T).extrude(COVER_H - COVER_T)))

for shape, name, path in ((frame, "frame_36103205", "Wuerth_36103205_20x20mm.step"),
                          (cover, "cover_36003200", "Wuerth_36003200_20x20mm_cover.step")):
    assy = cq.Assembly(name=f"WE-SHC_{name}")
    assy.add(shape, name=name, color=TIN)
    assy.export(path)
