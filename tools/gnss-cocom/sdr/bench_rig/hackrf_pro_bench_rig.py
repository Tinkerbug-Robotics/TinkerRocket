#!/usr/bin/env python3
"""Bench rig for two bare HackRF Pro boards (two-transmitter L1 + L5 setup), CadQuery, every size a parameter below.

Board facts come from Great Scott Gadgets' HackRF Pro layout (praline.kicad_pcb, CERN-OHL-P-2.0; board_geom.py):
120 x 75 x 1.585 mm, M3 holes at (4,4) (116,4) (4,71) (116,71); right edge: P1 SMA at y=12, P2 SMA at y=30, USB-C
y 45.6-56.4; left edge: antenna SMA at y=14, buttons SW1 y 45.8-55.4 / SW2 y 58.8-68.4, side LEDs y 23.9-59.3;
underside: flat test pads only. Board frame = board lower-left corner, component side up.

Layout (rig frame: origin at the plate's lower-left-bottom corner, x right, y up, z up), one plate printed flat:
  * board A (L1, clock + trigger master) above board B (L5), same orientation, on 8 mm standoffs (M3 heat-set
    inserts), each in a low-walled pocket that covers its underside; walls notched for every edge connector,
    lowered along the button/LED edge
  * right: clock A.P2 -> B.P1 and trigger A.P1 -> B.P2 jumpers run between the boards' right-edge SMAs; each USB-C
    cable exits right over a raised bed into two zip-tie points
  * left: each antenna output runs left along a V-channel that carries three 9.5 mm inline attenuators at the board
    SMA's axis height (so the stack doesn't hang off the board connector); a 2-way combiner sits on an M3 screw grid
    between the two arms; an upright tab at the far left takes an SMA bulkhead (OUT), with the DC block on a short
    V-saddle behind it at the same axis height
  * zip-tie slot pairs (with underside grooves so the tie heads don't rock the plate), rubber-foot recesses and
    countersunk screw holes for fixing it to the bench; engraved labels
Exports: rig STEP + STL, an assembly STEP with simple board stand-ins (for checking fit in Fusion 360), and
boards_standin.stl for render_rig.py.
    python3 hackrf_pro_bench_rig.py [OUTDIR]"""
import sys
from pathlib import Path

import cadquery as cq

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent

# ---------------------------------------------------------------- parameters (mm)
PLATE_W, PLATE_H, PLATE_T, PLATE_R = 290.0, 190.0, 5.0, 6.0
BOARD_W, BOARD_H, BOARD_T, BOARD_R = 120.0, 75.0, 1.585, 4.0
HOLES = [(4.0, 4.0), (116.0, 4.0), (4.0, 71.0), (116.0, 71.0)]           # M3, board frame
BX = 100.0                                   # both boards' left edge (rig x)
BY_B, BY_A = 12.0, 103.0                     # board B (lower) and board A (upper) bottom edges (rig y)
STANDOFF_H, STANDOFF_D = 8.0, 8.0            # board underside sits STANDOFF_H above the plate top
INSERT_D, INSERT_DEPTH = 4.0, 6.0            # M3 heat-set insert hole (use 2.6 for self-tapping M3)
POCKET_GAP, WALL_T, WALL_ABOVE = 0.8, 2.4, 1.0   # board-to-wall gap, wall thickness, wall top above board top
LOW_RIM_Z = 3.0                              # lowered wall height above plate top along the button/LED edge
# edge features, board frame: (y0, y1) and how deep the wall notch goes ("full" = to the plate)
LEFT_NOTCHES = [((6.0, 22.0), "full"),       # antenna SMA (body y 8.95-19.05, hangs ~4 mm below the board)
                ((22.0, 70.5), "low")]       # side LEDs y 23.9-59.3 + buttons y 45.8-68.4: wall lowered
RIGHT_NOTCHES = [((5.0, 19.0), "full"),      # P1 SMA y 6.96-17.05
                 ((23.0, 37.0), "full"),     # P2 SMA y 24.96-35.05
                 ((44.0, 58.0), "usb")]      # USB-C y 45.6-56.4 (plug overmold ~12 x 6.5)
SMA_LEFT_TIP = -11.55                        # antenna SMA tip, board frame x
SMA_RIGHT_TIP = BOARD_W + 11.55              # P1/P2 tips
SMA_Y = {"ANT": 14.0, "P1": 12.0, "P2": 30.0}
USB_Y = 51.0
# attenuator arm (V-channel) under each antenna output
PAD_D = 9.5                                  # inline attenuator / DC block body diameter (owner: 9.5 is right)
ARM_X0, ARM_X1 = 8.0, BX - 14.0              # channel extent (rig x): three ~27 mm pads; wrench room at the SMA
ARM_BLOCK_W = 18.0
ARM_TIES_X = [34.0, 56.0, 79.0]              # one tie per pad (3rd x 12-39, 2nd 39-66, 1st 66-93)
# combiner grid between the arms
COMB_X0, COMB_Y0, COMB_W, COMB_H, COMB_PITCH, COMB_HOLE_D, COMB_PAD_T = 42.0, 46.0, 44.0, 54.0, 10.0, 2.8, 2.0
# output bulkhead tab (SMA bulkhead jack, 1/4-36 thread) and the DC block's V-saddle behind it
TAB_X, TAB_T, TAB_W, TAB_H, TAB_Y, BULKHEAD_D = 6.0, 3.0, 30.0, 26.0, 72.0, 6.5
DCB_X0, DCB_X1, DCB_TIE_X = 12.0, 37.0, 24.0  # DC block (~25 mm) screwed onto the bulkhead's inner port
# USB cable tie-downs
USB_TIES_X, USB_BED_X0, USB_BED_H = [258.0, 276.0], 238.0, 1.5
# zip ties, feet, bench screws
TIE_SLOT_L, TIE_SLOT_W, TIE_GROOVE_D = 4.6, 2.2, 1.4
FOOT_D, FOOT_DEPTH, FEET = 13.0, 1.2, [(16.0, 16.0), (274.0, 16.0), (16.0, 174.0), (274.0, 174.0)]
SCREW_D, SCREW_CSK_D, SCREWS = 4.5, 9.0, [(9.0, 9.0), (281.0, 9.0), (9.0, 181.0), (281.0, 181.0)]
LABEL_DEPTH, LABEL_SIZE = 0.6, 5.0

Z0 = PLATE_T                                 # plate top
ZB = Z0 + STANDOFF_H                         # board underside
ZT = ZB + BOARD_T                            # board top
Z_AXIS = ZB + BOARD_T / 2                    # edge SMA axis
WALL_TOP = ZT + WALL_ABOVE


def box(x0, y0, z0, x1, y1, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def rounded_rect_solid(x0, y0, x1, y1, z0, z1, r):
    s = box(x0, y0, z0, x1, y1, z1)
    return s.edges("|Z").fillet(r) if r > 0 else s


# ---------------------------------------------------------------- plate
rig = rounded_rect_solid(0, 0, PLATE_W, PLATE_H, 0, PLATE_T, PLATE_R)


def pocket(by):
    """Walls around one board (open top), notched for its edge features; standoffs with insert holes."""
    g = POCKET_GAP
    ix0, iy0, ix1, iy1 = BX - g, by - g, BX + BOARD_W + g, by + BOARD_H + g
    outer = rounded_rect_solid(ix0 - WALL_T, iy0 - WALL_T, ix1 + WALL_T, iy1 + WALL_T, Z0, WALL_TOP,
                               BOARD_R + g + WALL_T)
    inner = rounded_rect_solid(ix0, iy0, ix1, iy1, Z0 - 0.1, WALL_TOP + 0.1, BOARD_R + g)
    w = outer.cut(inner)
    for (y0, y1), kind in LEFT_NOTCHES:
        zc = Z0 + LOW_RIM_Z if kind == "low" else Z0
        w = w.cut(box(ix0 - WALL_T - 1, by + y0, zc, ix0 + 1, by + y1, WALL_TOP + 1))
    for (y0, y1), kind in RIGHT_NOTCHES:
        zc = {"full": Z0, "usb": ZB - 1.5}[kind]
        w = w.cut(box(ix1 - 1, by + y0, zc, ix1 + WALL_T + 1, by + y1, WALL_TOP + 1))
    for hx, hy in HOLES:
        post = (cq.Workplane("XY").workplane(offset=Z0).center(BX + hx, by + hy)
                .circle(STANDOFF_D / 2).extrude(STANDOFF_H))
        post = post.cut(cq.Workplane("XY").workplane(offset=ZB - INSERT_DEPTH).center(BX + hx, by + hy)
                        .circle(INSERT_D / 2).extrude(INSERT_DEPTH + 0.1))
        w = w.union(post)
    return w


for by in (BY_B, BY_A):
    rig = rig.union(pocket(by))

# ---------------------------------------------------------------- zip-tie points (plate slots + underside groove)
def tie_point(x, y, half_span, along="y"):
    """Two through-slots half_span either side of (x, y), joined underneath by a shallow groove."""
    global rig
    if along == "y":
        for s in (-1, 1):
            rig = rig.cut(box(x - TIE_SLOT_L / 2, y + s * half_span - TIE_SLOT_W / 2, -1,
                              x + TIE_SLOT_L / 2, y + s * half_span + TIE_SLOT_W / 2, PLATE_T + 1))
        rig = rig.cut(box(x - TIE_SLOT_L / 2, y - half_span - TIE_SLOT_W / 2, -1,
                          x + TIE_SLOT_L / 2, y + half_span + TIE_SLOT_W / 2, TIE_GROOVE_D))
    else:
        for s in (-1, 1):
            rig = rig.cut(box(x + s * half_span - TIE_SLOT_W / 2, y - TIE_SLOT_L / 2, -1,
                              x + s * half_span + TIE_SLOT_W / 2, y + TIE_SLOT_L / 2, PLATE_T + 1))
        rig = rig.cut(box(x - half_span - TIE_SLOT_W / 2, y - TIE_SLOT_L / 2, -1,
                          x + half_span + TIE_SLOT_W / 2, y + TIE_SLOT_L / 2, TIE_GROOVE_D))


# ---------------------------------------------------------------- attenuator V-channels
r = PAD_D / 2
z_vertex = Z_AXIS - r * 2 ** 0.5             # 90-degree V: a PAD_D cylinder's axis lands on the SMA axis


def v_channel(x0, x1, yc, ties):
    """A block with a 90-degree V-groove along x that holds PAD_D parts on the SMA axis; a zip tie at each x in
    ties wraps the block and the part in it."""
    global rig
    blk = box(x0, yc - ARM_BLOCK_W / 2, Z0, x1, yc + ARM_BLOCK_W / 2, Z_AXIS)
    v = (cq.Workplane("YZ").workplane(offset=x0 - 1)
         .polyline([(yc, z_vertex), (yc + 12, z_vertex + 12), (yc - 12, z_vertex + 12)]).close()
         .extrude(x1 - x0 + 2))
    rig = rig.union(blk.cut(v))
    for tx_ in ties:
        tie_point(tx_, yc, ARM_BLOCK_W / 2 + 2.5)


for by in (BY_B, BY_A):                      # three attenuators per arm, straight off each antenna SMA
    v_channel(ARM_X0, ARM_X1, by + SMA_Y["ANT"], ARM_TIES_X)

# ---------------------------------------------------------------- combiner screw grid
pad = box(COMB_X0, COMB_Y0, Z0, COMB_X0 + COMB_W, COMB_Y0 + COMB_H, Z0 + COMB_PAD_T)
rig = rig.union(pad)
nx, ny = int(COMB_W // COMB_PITCH), int(COMB_H // COMB_PITCH)
gx0 = COMB_X0 + (COMB_W - (nx - 1) * COMB_PITCH) / 2
gy0 = COMB_Y0 + (COMB_H - (ny - 1) * COMB_PITCH) / 2
pts = [(gx0 + i * COMB_PITCH, gy0 + j * COMB_PITCH) for i in range(nx) for j in range(ny)]
rig = rig.cut(cq.Workplane("XY").pushPoints(pts).circle(COMB_HOLE_D / 2).extrude(Z0 + COMB_PAD_T + 1)
              .translate((0, 0, 0.8)))       # blind from the top: 0.8 mm floor left under each hole
for (x, y) in [(COMB_X0 - 4, COMB_Y0 + COMB_H / 2), (COMB_X0 + COMB_W + 4, COMB_Y0 + COMB_H / 2)]:
    tie_point(x, y, COMB_H / 2 - 8)

# ---------------------------------------------------------------- output bulkhead tab
tab = box(TAB_X, TAB_Y - TAB_W / 2, Z0, TAB_X + TAB_T, TAB_Y + TAB_W / 2, Z0 + TAB_H)
for s in (-1, 1):                            # gussets on the inner side
    g_ = (cq.Workplane("XZ").workplane(offset=-(TAB_Y + s * (TAB_W / 2 - 1.5)))
          .polyline([(TAB_X + TAB_T, Z0), (TAB_X + TAB_T + 12, Z0), (TAB_X + TAB_T, Z0 + TAB_H - 4)]).close()
          .extrude(-3.0 if s < 0 else 3.0))
    tab = tab.union(g_)
tab = tab.cut(cq.Workplane("YZ").workplane(offset=TAB_X - 1).center(TAB_Y, Z_AXIS)
              .circle(BULKHEAD_D / 2).extrude(TAB_T + 2))
rig = rig.union(tab)
v_channel(DCB_X0, DCB_X1, TAB_Y, [DCB_TIE_X])   # DC block on the bulkhead's inner port, same axis height

# ---------------------------------------------------------------- USB cable beds + tie-downs
for by in (BY_B, BY_A):
    yc = by + USB_Y
    rig = rig.union(box(USB_BED_X0, yc - 5, Z0, PLATE_W - 6, yc + 5, Z0 + USB_BED_H))
    for x in USB_TIES_X:
        tie_point(x, yc, 6.5)

# ---------------------------------------------------------------- feet recesses + bench screws
for fx, fy in FEET:
    rig = rig.cut(cq.Workplane("XY").center(fx, fy).circle(FOOT_D / 2).extrude(FOOT_DEPTH))
for sx, sy in SCREWS:
    rig = rig.cut(cq.Workplane("XY").center(sx, sy).circle(SCREW_D / 2).extrude(PLATE_T + 1))
    rig = rig.cut(cq.Workplane("XY").workplane(offset=PLATE_T - (SCREW_CSK_D - SCREW_D) / 2).center(sx, sy)
                  .circle(SCREW_D / 2).workplane(offset=(SCREW_CSK_D - SCREW_D) / 2 + 0.01)
                  .circle(SCREW_CSK_D / 2).loft())

# ---------------------------------------------------------------- engraved labels
labels = [("A  L1  clock + trigger master", BX + BOARD_W / 2, BY_A + BOARD_H + 6.5),
          ("B  L5", BX + BOARD_W / 2, BY_B - 6.5),
          ("CLK  A.P2 > B.P1", 258.0, 90.0), ("TRIG  A.P1 > B.P2", 258.0, 82.0),
          ("OUT", TAB_X + 12.0, TAB_Y + TAB_W / 2 + 6.0), ("COMBINER", COMB_X0 + COMB_W / 2, COMB_Y0 - 5.0)]
for text, x, y in labels:
    try:
        t = (cq.Workplane("XY").workplane(offset=Z0 - LABEL_DEPTH).center(x, y)
             .text(text, LABEL_SIZE, LABEL_DEPTH + 0.2, combine=False, halign="center", valign="center"))
        rig = rig.cut(t)
    except Exception as exc:                 # no usable font on this machine: skip the label
        print(f"label '{text}' skipped: {exc}")

# ---------------------------------------------------------------- board stand-ins for the fit check
def board_standin(by):
    b = rounded_rect_solid(BX, by, BX + BOARD_W, by + BOARD_H, ZB, ZT, BOARD_R)
    b = b.cut(box(BX + 10.0, by + 52.5, ZB - 1, BX + 98.0, by + BOARD_H + 1, ZT + 1))   # top-edge notch
    for hx, hy in HOLES:
        b = b.cut(cq.Workplane("XY").workplane(offset=ZB - 1).center(BX + hx, by + hy).circle(1.6).extrude(5))
    parts = [b]
    for name, x0, x1 in (("ANT", BX + SMA_LEFT_TIP, BX + 6.3), ("P1", BX + BOARD_W - 6.2, BX + SMA_RIGHT_TIP),
                         ("P2", BX + BOARD_W - 6.2, BX + SMA_RIGHT_TIP)):
        yc = by + SMA_Y[name]
        parts.append(cq.Workplane("YZ").workplane(offset=x0).center(yc, Z_AXIS).circle(3.175).extrude(x1 - x0))
    parts.append(box(BX + BOARD_W - 7.97, by + 45.64, ZT, BX + BOARD_W + 1.54, by + 56.36, ZT + 3.3))   # USB-C
    parts.append(box(BX + 6.9, by + 3.25, ZT, BX + 59.1, by + 42.75, ZT + 4.0))                       # shield can
    out = parts[0]
    for p in parts[1:]:
        out = out.union(p)
    return out


boards = board_standin(BY_A).union(board_standin(BY_B))

# ---------------------------------------------------------------- exports
OUT.mkdir(parents=True, exist_ok=True)
cq.exporters.export(rig, str(OUT / "hackrf_pro_bench_rig.step"))
cq.exporters.export(rig, str(OUT / "hackrf_pro_bench_rig.stl"), tolerance=0.05, angularTolerance=0.2)
asm = cq.Assembly(name="hackrf_pro_bench_rig_assembly")
asm.add(rig, name="rig", color=cq.Color(0.85, 0.55, 0.15))
asm.add(boards, name="hackrf_pro_boards_standin", color=cq.Color(0.1, 0.45, 0.2))
asm.save(str(OUT / "hackrf_pro_bench_rig_assembly.step"))
cq.exporters.export(boards, str(OUT / "boards_standin.stl"), tolerance=0.1, angularTolerance=0.3)   # renders only
bb = rig.val().BoundingBox()
print(f"rig {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, volume {rig.val().Volume() / 1000:.0f} cm3 -> {OUT}")
print(f"board underside z {ZB:.2f}, SMA axis z {Z_AXIS:.2f}, V-groove vertex z {z_vertex:.2f}, wall top z {WALL_TOP:.2f}")
