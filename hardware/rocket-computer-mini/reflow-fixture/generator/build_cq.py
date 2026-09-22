"""Build the open-frame carrier mold (and the cast silicone ring) with CadQuery; export STEP/STL."""
import sys, os
import cadquery as cq
from design import design, P

d = design()

def geoms(g):
    return [x for x in getattr(g, 'geoms', [g]) if not x.is_empty]

def clean(coords, tol=2e-3):
    """Round and drop consecutive (near-)duplicate vertices, including wrap-around."""
    pts = []
    for x, y in coords[:-1]:
        p = (round(x, 4), round(y, 4))
        if not pts or abs(p[0] - pts[-1][0]) > tol or abs(p[1] - pts[-1][1]) > tol:
            pts.append(p)
    while len(pts) > 2 and abs(pts[0][0] - pts[-1][0]) <= tol and abs(pts[0][1] - pts[-1][1]) <= tol:
        pts.pop()
    return pts

def prism(geom, z0, z1, taper=0.0):
    """Extrude a shapely (multi)polygon from z0 to z1. taper>0 shrinks with height."""
    result = None
    for g in geoms(geom):
        ext = clean(list(g.exterior.coords))
        s = cq.Workplane('XY').workplane(offset=z0).polyline(ext).close().extrude(z1 - z0, taper=taper)
        for ring in g.interiors:
            pts = clean(list(ring.coords))
            s = s.cut(cq.Workplane('XY').workplane(offset=z0 - 0.01).polyline(pts).close().extrude(z1 - z0 + 0.02))
        result = s if result is None else result.union(s)
    return result

wall_top = d['wall_top']
sil_w, sil_l = d['sil_w'], d['sil_l']
box_w, box_l = d['box_w'], d['box_l']

# ---------------------------------------------------------------- mold box, cavity, fill groove
mold = (cq.Workplane('XY').box(box_w, box_l, P['floor_t'] + wall_top, centered=(True, True, False))
        .translate((0, 0, -P['floor_t'])).edges('|Z').fillet(P['outer_r'] + P['mold_wall']))
cavity = (cq.Workplane('XY').box(sil_w, sil_l, wall_top + 1, centered=(True, True, False))
          .edges('|Z').fillet(P['outer_r']))
mold = mold.cut(cavity)
groove = (cq.Workplane('XY').box(sil_w + 2 * P['groove_d'], sil_l + 2 * P['groove_d'], P['groove_w'])
          .translate((0, 0, P['sil_h']))
          .cut(cq.Workplane('XY').box(sil_w, sil_l, P['groove_w'] + 1).translate((0, 0, P['sil_h']))))
mold = mold.cut(groove)

# ---------------------------------------------------------------- positives
positives = prism(d['plateau'], 0, P['board_t'])                             # board recess
core = prism(d['window'], 0, wall_top, taper=P['draft_deg'])                # open window (drafted)
catch_cut = prism(d['catch'], d['z_catch'], wall_top + 2)                    # lower the core over J2
core = core.cut(catch_cut)
positives = positives.union(core)

try:   # label, mirrored so it reads correctly on the cast ring's top face
    txt = (cq.Workplane('XY').transformed(rotate=(0, 0, 90))
           .text(P['label'], 3.0, 0.4, combine=False, halign='center', valign='center', font='DejaVu Sans'))
    txt = txt.mirror('XZ').translate((-(P['board_w'] / 2 + P['board_clr'] + P['wall'] / 2), -14.0, 0))
    positives = positives.union(txt)
    print("label added")
except Exception as e:
    print("label skipped:", e)

mold = mold.union(positives)

# ---------------------------------------------------------------- the cast silicone ring
silicone = (cq.Workplane('XY').box(sil_w, sil_l, P['sil_h'], centered=(True, True, False))
            .edges('|Z').fillet(P['outer_r']).cut(positives))

# ---------------------------------------------------------------- export
out = sys.argv[1] if len(sys.argv) > 1 else 'out'
os.makedirs(out, exist_ok=True)
cq.exporters.export(mold, f'{out}/rocket-computer-mini-reflow-mold.step')
cq.exporters.export(mold, f'{out}/rocket-computer-mini-reflow-mold.stl', tolerance=0.02, angularTolerance=0.1)
cq.exporters.export(silicone, f'{out}/rocket-computer-mini-reflow-carrier-silicone.step')
cq.exporters.export(silicone, f'{out}/_silicone.stl', tolerance=0.02, angularTolerance=0.1)
cq.exporters.export(positives, f'{out}/_positives.step')

def bb(w):
    b = w.val().BoundingBox(); return tuple(round(v, 2) for v in (b.xmin, b.ymin, b.zmin, b.xmax, b.ymax, b.zmax))
print("mold bbox", bb(mold), "volume %.1f cm3" % (mold.val().Volume() / 1000))
print("silicone bbox", bb(silicone), "volume %.1f mL -> %.0f g Mold Max 60" % (silicone.val().Volume() / 1000, silicone.val().Volume() / 1000 * 1.45))
print("mold solids:", len(mold.solids().vals()), " silicone solids:", len(silicone.solids().vals()))
