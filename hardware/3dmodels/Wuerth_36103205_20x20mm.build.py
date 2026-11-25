import cadquery as cq

# Würth WE-SHC shielding cabinet, 20 x 20 mm, two-piece: SMD frame 36103205 (data sheet rev 003.004,
# 2020-11-12, page 1) and its snap-on cover 36003200 (rev 002.004, 2020-11-12, page 1). KiCad ships the
# footprint RF_Shielding:Wuerth_36103205_20x20mm but not this model (its model path names a file that
# is not in the KiCad 10 library), hence this one.
# Origin at the footprint centre, seated at z = 0 (the board surface). Two files:
# Wuerth_36103205_20x20mm.step (the frame) and Wuerth_36003200_20x20mm_cover.step (the cover; the
# footprint carries it hidden).
#
# Where a value comes from: "dim" = dimensioned on the drawing; "scaled" = measured from the drawing's
# vector geometry (frame views 1.3:1, detail B 2:1, cover views 1.5:1; calibrated on the dimensioned
# outer 21 / 21.8 and the 2.5 / 3 / 7.5 dimensions, repeatable to about 0.02 mm).

# ---- frame 36103205 ----
FRAME_OUT, FRAME_IN = 21.0, 20.0   # dim: outer 21 ±0.1, inner 20 ±0.1 (wall 0.5 ±0.05)
T = 0.5                            # dim: material thickness t, walls and top flange
FRAME_H = 3.5                      # dim: section A-A 3.5 ±0.1 to the flange top. The 3.0 ±0.1 there is
                                   # the clear height under the flange (FRAME_H - T); distributors list
                                   # that 3 mm as "height". The side view and detail B scale to 3.49.
BEND_R = 0.6                       # scaled: outer radius where the flange bends into the wall (0.59)
OPEN_X = 8.0                       # scaled: the two flange openings span |x| <= 8.0 ...
OPEN_Y0, OPEN_Y1 = 1.0, 8.0        # scaled: ... and 1.0 <= |y| <= 8.0, so the bridge is 2.0 wide
OPEN_CH = 1.5                      # scaled: 45-degree chamfer on all four corners of each opening
PICKUP_R = 2.0                     # scaled: round pick-up area on the bridge, section A-A shows Ø 4.0
# The wall's bottom edge stands on feet over the footprint's lands (2.6 long at ±3.75, the corner L to
# ±7.45) and is relieved between them. Edges measured at mid-height of each sloped transition:
# dim: detail B foot 2.5 ±0.05 (2.5..5.0), section A-A 7.5 ±0.1 between the middle-foot centres,
# side view 3 ±0.05 to the corner foot (7.5..10.5); the bottom view marks ±2.5, ±5.0, ±7.5.
RELIEF_SPANS = [(-2.5, 2.5), (5.0, 7.5), (-7.5, -5.0)]
RELIEF = 0.4                       # scaled: relief height (0.39 side view, 0.40 detail B)
SLOPE = 0.4                        # scaled: horizontal run of each transition, so the relief is
                                   # RELIEF_SPANS widened by 0.2 at the board and narrowed by 0.2 on top
HOLE_D, HOLE_Z = 0.8, 1.1          # scaled: snap holes for the cover's dimples, Ø 0.8, centre 1.1 up
HOLE_U = (-7.5, 0.0, 7.5)          # dim: 3 ±0.05 from each wall end, plus the centre; on all four walls

# ---- cover 36003200 ----
COVER_OUT, COVER_T = 21.8, 0.3     # dim: outer 21.8 ±0.1, inner 21.2 ±0.1, t 0.3 ±0.05
COVER_H = 3.3                      # dim: 3.3 ±0.1 overall, 3 ±0.1 inside
COVER_TOP = FRAME_H + COVER_T      # 3.8: the cover's top rests on the frame flange. Its skirt then ends
                                   # 0.5 above the board, and its dimples (below) line up with the frame's
                                   # holes at 1.1, which confirms the 3.5 frame height.
COVER_BEND_R = 0.4                 # scaled: outer bend radius (0.42)
DIMPLE_D, DIMPLE_Z = 0.9, 0.6      # scaled: Ø 0.9 snap dimples 0.6 above the skirt edge, at HOLE_U
DIMPLE_DEPTH = 0.15                # not on the drawing: pressed in just past the 0.1 gap to the frame

TIN = cq.Color(0.62, 0.63, 0.64)


def walls_cut(profile_xz, profile_yz):
    """A prism through both walls of each pair: profile in XZ extruded along y, and in YZ along x."""
    a = cq.Workplane("XZ").polyline(profile_xz).close().extrude(FRAME_OUT, both=True)
    b = cq.Workplane("YZ").polyline(profile_yz).close().extrude(FRAME_OUT, both=True)
    return a.union(b)


# body: a 0.5 mm shell open at the bottom, its top edges bent over with BEND_R (inside BEND_R - T)
frame = (cq.Workplane("XY").rect(FRAME_OUT, FRAME_OUT).extrude(FRAME_H)
         .edges(">Z").fillet(BEND_R)
         .cut(cq.Workplane("XY").workplane(offset=-0.01).rect(FRAME_IN, FRAME_IN)
              .extrude(FRAME_H - T + 0.01)))
frame = frame.edges(cq.selectors.BoxSelector((-FRAME_IN / 2 - 0.01, -FRAME_IN / 2 - 0.01, FRAME_H - T - 0.01),
                                             (FRAME_IN / 2 + 0.01, FRAME_IN / 2 + 0.01, FRAME_H - T + 0.01))
                    ).fillet(BEND_R - T)

# top flange: two openings with all four corners chamfered, the bridge and its round pick-up area left
openings = None
for sy in (-1, 1):
    y0, y1, c = sy * OPEN_Y0, sy * OPEN_Y1, sy * OPEN_CH
    pts = [(-OPEN_X + OPEN_CH, y0), (OPEN_X - OPEN_CH, y0), (OPEN_X, y0 + c), (OPEN_X, y1 - c),
           (OPEN_X - OPEN_CH, y1), (-OPEN_X + OPEN_CH, y1), (-OPEN_X, y1 - c), (-OPEN_X, y0 + c)]
    o = (cq.Workplane("XY").workplane(offset=FRAME_H - T - 0.1).polyline(pts).close().extrude(T + 0.2))
    openings = o if openings is None else openings.union(o)
openings = openings.cut(cq.Workplane("XY").workplane(offset=FRAME_H - T - 0.2).circle(PICKUP_R).extrude(T + 0.4))
frame = frame.cut(openings)

# bottom-edge reliefs: a trapezoid per span, RELIEF tall, sloped over SLOPE
for a, b in RELIEF_SPANS:
    lo, hi = a - SLOPE / 2, b + SLOPE / 2          # at the board
    tlo, thi = a + SLOPE / 2, b - SLOPE / 2        # at the top of the relief
    prof = [(lo, -0.01), (hi, -0.01), (thi, RELIEF), (tlo, RELIEF)]
    frame = frame.cut(walls_cut(prof, prof))

# snap holes through all four walls
for u in HOLE_U:
    hx = cq.Workplane("XZ").center(u, HOLE_Z).circle(HOLE_D / 2).extrude(FRAME_OUT, both=True)
    hy = cq.Workplane("YZ").center(u, HOLE_Z).circle(HOLE_D / 2).extrude(FRAME_OUT, both=True)
    frame = frame.cut(hx).cut(hy)

# cover: 0.3 mm shell open at the bottom, top edges bent with COVER_BEND_R, pressed dimples on each wall
c_bot = COVER_TOP - COVER_H
cover = (cq.Workplane("XY").workplane(offset=c_bot).rect(COVER_OUT, COVER_OUT).extrude(COVER_H)
         .edges(">Z").fillet(COVER_BEND_R)
         .cut(cq.Workplane("XY").workplane(offset=c_bot - 0.01)
              .rect(COVER_OUT - 2 * COVER_T, COVER_OUT - 2 * COVER_T).extrude(COVER_H - COVER_T + 0.01)))
r_in = COVER_OUT / 2 - COVER_T                              # inner face
R = ((DIMPLE_D / 2) ** 2 + DIMPLE_DEPTH ** 2) / (2 * DIMPLE_DEPTH)   # sphere through the dimple rim
for u in HOLE_U:
    for sgn in (-1, 1):
        for along_x in (True, False):
            def at(d):          # point on the wall's normal at distance d from the centre, at height z
                return (u, sgn * d, c_bot + DIMPLE_Z) if along_x else (sgn * d, u, c_bot + DIMPLE_Z)

            def slab(d0, d1):   # the wall-normal band d0..d1 (both > 0), clipped around the dimple
                size = (2 * R, d1 - d0, 2 * R) if along_x else (d1 - d0, 2 * R, 2 * R)
                return cq.Workplane("XY").box(*size).translate(at((d0 + d1) / 2))
            def ball(d):        # sphere turned so its seam (on its local +x) faces along the wall
                s = cq.Workplane("XY").sphere(R)
                return (s if along_x else s.rotate((0, 0, 0), (0, 0, 1), 90)).translate(at(d))
            bump = ball(r_in + R - DIMPLE_DEPTH).intersect(slab(r_in - DIMPLE_DEPTH, r_in + 0.001))
            dent = (ball(COVER_OUT / 2 + R - DIMPLE_DEPTH)
                    .intersect(slab(COVER_OUT / 2 - DIMPLE_DEPTH, COVER_OUT / 2 + 0.001)))
            cover = cover.union(bump).cut(dent)

for shape, name, path in ((frame, "frame_36103205", "Wuerth_36103205_20x20mm.step"),
                          (cover, "cover_36003200", "Wuerth_36003200_20x20mm_cover.step")):
    assy = cq.Assembly(name=f"WE-SHC_{name}")
    assy.add(shape, name=name, color=TIN)
    assy.export(path)
