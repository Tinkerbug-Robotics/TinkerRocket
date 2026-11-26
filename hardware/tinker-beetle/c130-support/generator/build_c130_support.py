"""Stage 2 (run with a Python that has CadQuery + shapely): build the TPU support for C130 standing
upright, check it against the board's STEP export, and write the STL/STEP and fit renders.

    python3 build_c130_support.py board_map.json <board.step> <out_dir>

The board STEP comes from `kicad-cli pcb export step --subst-models --no-dnp`. Frame: see board_map.py
(origin = C130's hole midpoint on the back face, +z away from the board). The STL is in that frame,
so it prints pedestal-down with no supports.
"""
import json, math, sys, os
import cadquery as cq
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

# ---- design parameters (mm) -------------------------------------------------------------------------
CAN_D, CAN_L = 8.0, 12.0      # CDA CXHP2R7205R-TW, nominal (spec: D +/-1.0, L +/-1.5): measure yours
SLEEVE_ID = 8.4               # can + 0.4: TPU grips, RTV fills the rest
WALL = 1.2                    # three 0.4 mm perimeters
PED_H = 1.7                   # pedestal height = can standoff; tallest part under the can is 1.33 mm
SLEEVE_H = 8.0                # grip length above the pedestal (two thirds of the can)
MARGIN = 0.30                 # pedestal clearance to part courtyards and pads (TPU swells ~0.1)
TALL_CLEAR = 0.30             # ring clearance to parts taller than the pedestal
COVER_SKIRT = 0.23            # BMI-S-202-C cover overhangs the -F frame by this much per side
MIN_FEATURE = 0.8             # narrowest printable pedestal finger
ENTRY_CHAMFER = 0.6           # lead-in on the sleeve's top inner edge
NOTCH_W, NOTCH_D = 1.4, 1.0   # polarity notch in the rim, on the + lead's side (+x)
R_SEAT, LIP = 3.4, 0.8        # 45 deg seat cone inside the tube's base: bore R_SEAT at PED_H, R_IN at PED_H + LIP

R_IN, R_OUT = SLEEVE_ID / 2, SLEEVE_ID / 2 + WALL

bm_path, step_path, out_dir = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(out_dir, exist_ok=True)
bm = json.load(open(bm_path))
ox, oy = bm["origin_kicad"]

# ---- part heights from the board STEP (back side only) ---------------------------------------------
board = cq.importers.importStep(step_path)
solids = board.solids().vals()
def to_local_bb(b):   # STEP x = KiCad x, STEP y = -KiCad y, back face at z = 0, back parts at z < 0
    return (b.xmin - ox, -b.ymax - oy, b.xmax - ox, -b.ymin - oy, -b.zmax, -b.zmin)
back = []
for s in solids:
    b = s.BoundingBox()
    if b.xlen > 20 and b.ylen > 60:
        continue                      # the board itself
    x0, y0, x1, y1, z0, z1 = to_local_bb(b)
    if max(abs(x0), abs(x1), abs(y0), abs(y1)) > 20 or b.zmax > 0.05:
        continue
    if z0 > 3.0 and abs(x0 + x1) < 2 and y0 < 0:     # the horizontal C130 can in the export (floats 4 mm up)
        continue
    if b.xlen < 1.0 and b.ylen < 1.0 and min(math.hypot((x0 + x1) / 2 - px, (y0 + y1) / 2 - py) for px, py, _ in bm["c130_pads"].values()) < 1.0:
        continue                      # C130's own leads
    back.append((s, (x0, y0, x1, y1, z0, z1)))

obst_all, obst_tall, heights = [], [], {}
for o in bm["obstacles"]:
    if "circle" in o:
        x, y, r = o["circle"]
        g = Point(x, y).buffer(r, 32)
        h = 0.1
    else:
        g = unary_union([Polygon(p) for p in o["polys"] if len(p) >= 3])
        h = max([bb[5] for s, bb in back if g.buffer(0.3).contains(Point((bb[0] + bb[2]) / 2, (bb[1] + bb[3]) / 2))], default=0.0)
    heights[o["ref"]] = h
    if o["ref"] == "SH1":
        g = g.buffer(COVER_SKIRT)     # the cover is not in the export; it sits over the frame
    obst_all.append(g)
    if h > PED_H - TALL_CLEAR:
        obst_tall.append(g)

# ---- footprints ----------------------------------------------------------------------------------
# Printed rim-down (upside down), so nothing may overhang more than 45 deg in that orientation:
#   sleeve  plain tube, R_IN..R_OUT, from PED_H to the rim; trimmed clear of tall neighbours
#   seat    45 deg cone inside the tube's base: the bore narrows from R_IN at PED_H + LIP to R_SEAT at
#           PED_H, and the can's rolled bottom edge rests on it
#   feet    grow from the tube and seat toward the board at 45 deg, sideways only into free board
#           (part courtyards and pads + MARGIN never), so each 0.1 mm slice sits on the one above it
disc = Point(0, 0).buffer(R_OUT, 128)
ring_fp = disc.difference(unary_union([g.buffer(TALL_CLEAR, 16) for g in obst_tall])).difference(Point(0, 0).buffer(R_IN, 128))
free = Point(0, 0).buffer(R_OUT + PED_H + 0.5, 64).difference(unary_union([g.buffer(MARGIN, 8) for g in obst_all])).simplify(0.01, preserve_topology=True)
seat_fp = Point(0, 0).buffer(R_IN, 128).difference(Point(0, 0).buffer(R_SEAT, 128))
DZ = 0.1
slices, region = [], ring_fp.union(seat_fp).intersection(free)
for k in range(int(round(PED_H / DZ))):
    region = region.buffer(DZ, 8).intersection(free)
    region = region.buffer(-MIN_FEATURE / 2, 8).buffer(MIN_FEATURE / 2, 8).intersection(free).simplify(0.01, preserve_topology=True)
    slices.append((PED_H - (k + 1) * DZ, region))
foot0 = slices[-1][1]
print(f"feet on the board: {foot0.area:.1f} mm2 in {len(getattr(foot0, 'geoms', [foot0]))} pads")
ring_loss = disc.difference(Point(0, 0).buffer(R_IN, 128)).area - ring_fp.area
print(f"ring trimmed by {ring_loss:.2f} mm2 for tall neighbours (S1, the shield cover)")

def clean(coords, tol=0.003):
    pts = []
    for p in list(coords)[:-1]:
        if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > tol:
            pts.append(p)
    if len(pts) > 1 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) <= tol:
        pts.pop()
    return pts

def extrude(geom, z0, h):
    out = None
    geom = geom.simplify(0.002, preserve_topology=True)
    for g in getattr(geom, "geoms", [geom]):
        if g.is_empty or g.area < 0.05:
            continue
        ext = clean(g.exterior.coords)
        if len(ext) < 3:
            continue
        wp = cq.Workplane("XY").workplane(offset=z0).polyline(ext).close().extrude(h)
        for hole in g.interiors:
            hp = clean(hole.coords)
            if len(hp) >= 3 and Polygon(hp).area > 0.01:
                wp = wp.cut(cq.Workplane("XY").workplane(offset=z0 - 0.1).polyline(hp).close().extrude(h + 0.2))
        out = wp if out is None else out.union(wp)
    return out

support = extrude(ring_fp, PED_H, SLEEVE_H)
seat = (cq.Workplane("XY").workplane(offset=PED_H).circle(R_IN + 0.01).circle(R_SEAT)
        .extrude(LIP))
cone = (cq.Workplane("XZ").polyline([(R_SEAT, PED_H), (R_IN + 0.02, PED_H + LIP), (R_IN + 0.02, PED_H + LIP + 0.5), (0, PED_H + LIP + 0.5), (0, PED_H)]).close()
        .revolve(360, (0, 0, 0), (0, 1, 0)))
support = support.union(seat.cut(cone))
for z0, g in slices:
    f = extrude(g, z0, DZ + 0.001)
    if f is not None:
        support = support.union(f)
top = PED_H + SLEEVE_H
cham = (cq.Workplane("XY").workplane(offset=top - ENTRY_CHAMFER).circle(R_IN)
        .workplane(offset=ENTRY_CHAMFER).circle(R_IN + ENTRY_CHAMFER).loft())
support = support.cut(cham)
support = support.cut(cq.Workplane("XY").center(R_IN + WALL / 2, 0)
                      .polyline([(-WALL, -NOTCH_W / 2), (WALL, -NOTCH_W / 2), (WALL, NOTCH_W / 2), (-WALL, NOTCH_W / 2)]).close()
                      .extrude(1).translate((0, 0, top - NOTCH_D)))
solid = support.val()
props = GProp_GProps(); BRepGProp.VolumeProperties_s(solid.wrapped, props)
bbox = solid.BoundingBox()
print(f"support: {bbox.xlen:.2f} x {bbox.ylen:.2f} x {bbox.zlen:.2f} mm, {props.Mass() / 1000:.2f} cm3, valid={solid.isValid()}")

tmp = os.path.join(out_dir, "renders", "_inputs"); os.makedirs(tmp, exist_ok=True)
cq.exporters.export(support, os.path.join(tmp, "support.stl"), tolerance=0.01, angularTolerance=0.1)
printable = support.rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, PED_H + SLEEVE_H))
cq.exporters.export(printable, os.path.join(out_dir, "c130-support_print-rim-down.stl"), tolerance=0.01, angularTolerance=0.1)
cq.exporters.export(support, os.path.join(out_dir, "c130-support.step"))

# ---- fit check in the STEP frame ---------------------------------------------------------------
def to_step(shape):
    return shape.rotate((0, 0, 0), (1, 0, 0), 180).translate((ox, -oy, 0))   # 180 deg about x, then to the hole midpoint
sup_s = to_step(solid)
CAN_Z = PED_H + (CAN_D / 2 - 0.3 - R_SEAT)   # where the can's rounded bottom edge meets the 45 deg seat
print(f"can bottom at {CAN_Z:.2f} mm above the board, top at {CAN_Z + CAN_L:.2f} (+{1.5:.1f} at the length tolerance)")
can = cq.Workplane("XY").workplane(offset=CAN_Z).circle(CAN_D / 2).extrude(CAN_L).edges("<Z").fillet(0.4).val()
leads = cq.Workplane("XY").pushPoints([(-1.75, 0), (1.75, 0)]).circle(0.3).extrude(CAN_Z).val()
can_s, leads_s = to_step(can), to_step(leads)
rep = []
for s, bb in back:
    if max(abs(bb[0]), abs(bb[2])) > 9 or max(abs(bb[1]), abs(bb[3])) > 9:
        continue
    d1 = BRepExtrema_DistShapeShape(sup_s.wrapped, s.wrapped); d1.Perform()
    d2 = BRepExtrema_DistShapeShape(can_s.wrapped, s.wrapped); d2.Perform()
    rep.append((d1.Value(), d2.Value(), bb))
rep.sort(key=lambda t: t[0])
print("closest parts to the support (mm):  support  can   part bbox (local x0 y0 x1 y1, height)")
for d1, d2, bb in rep[:10]:
    print(f"   {d1:6.3f}  {d2:6.3f}   ({bb[0]:6.2f} {bb[1]:6.2f} {bb[2]:6.2f} {bb[3]:6.2f}) h {bb[5]:.2f}")
lead_gap = BRepExtrema_DistShapeShape(solid.wrapped, leads.wrapped); lead_gap.Perform()
print(f"lead clearance through the seat and feet: {lead_gap.Value():.2f} mm")
cover = Polygon(next(o for o in bm["obstacles"] if o["ref"] == "SH1")["polys"][0]).buffer(COVER_SKIRT, join_style=2)
print(f"ring to the shield cover outline: {ring_fp.distance(cover):.2f} mm (plan view)")
json.dump(dict(heights={k: round(v, 2) for k, v in heights.items() if v > 0.05}, params=dict(
    CAN_D=CAN_D, CAN_L=CAN_L, SLEEVE_ID=SLEEVE_ID, WALL=WALL, PED_H=PED_H, SLEEVE_H=SLEEVE_H, MARGIN=MARGIN)),
    open(os.path.join(out_dir, "fit.json"), "w"), indent=1)

# ---- render inputs (local frame) -------------------------------------------------------------------
def from_step(shape):
    return shape.translate((-ox, oy, 0)).rotate((0, 0, 0), (1, 0, 0), 180)
near = [from_step(s) for s, bb in back if max(abs(bb[0]), abs(bb[2])) < 13 and max(abs(bb[1]), abs(bb[3])) < 13]
cq.exporters.export(cq.Workplane("XY").add(near), os.path.join(tmp, "parts.stl"), tolerance=0.02)
cq.exporters.export(cq.Workplane("XY").box(30, 30, bm["board_thickness"], centered=(True, True, False)).translate((0, 0, -bm["board_thickness"])),
                    os.path.join(tmp, "board.stl"))
cq.exporters.export(cq.Workplane("XY").add([can, leads]), os.path.join(tmp, "can.stl"), tolerance=0.02)
print("wrote", out_dir)
