"""Stage 1 (run with KiCad's Python): read the board and write what the C130 support must avoid.

    <KiCad python> board_map.py ../../tinker-beetle.kicad_pcb board_map.json

Local frame, used by every file in this folder:
  origin  C130's hole midpoint on the board's back face
  +x      KiCad +x (toward pad 1, the + lead, V_SCAP)
  +y      KiCad +y (away from the GNSS shield SH1, toward the switch S1)
  +z      away from the back face (the way the can stands)
This is the board's STEP frame turned 180 deg about x (a rotation, not a mirror), so a part
modelled here is the right hand when printed.
"""
import json, math, sys
import pcbnew

pcb_path, out_path = sys.argv[1], sys.argv[2]
b = pcbnew.LoadBoard(pcb_path)
mm = pcbnew.ToMM
c130 = next(f for f in b.GetFootprints() if f.GetReference() == "C130")
pads = sorted(c130.Pads(), key=lambda p: p.GetNumber())
ox = (mm(pads[0].GetPosition().x) + mm(pads[1].GetPosition().x)) / 2
oy = (mm(pads[0].GetPosition().y) + mm(pads[1].GetPosition().y)) / 2


def loc(x, y):
    return (round(x - ox, 4), round(y - oy, 4))


def poly_pts(spoly):
    out = []
    for i in range(spoly.OutlineCount()):
        o = spoly.Outline(i)
        out.append([loc(mm(o.CPoint(k).x), mm(o.CPoint(k).y)) for k in range(o.PointCount())])
    return out


WIN = 12.0  # mm around the origin
obstacles = []
for f in b.GetFootprints():
    ref = f.GetReference()
    fx, fy = loc(mm(f.GetPosition().x), mm(f.GetPosition().y))
    if ref == "C130":
        for p in f.Pads():
            px, py = loc(mm(p.GetPosition().x), mm(p.GetPosition().y))
            obstacles.append(dict(ref=f"C130.{p.GetNumber()}", kind="tht_pad", circle=[px, py, mm(p.GetSize().x) / 2]))
        continue
    if f.GetLayer() != pcbnew.B_Cu:
        # front parts only reach the back face through their plated holes
        for p in f.Pads():
            if p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
                px, py = loc(mm(p.GetPosition().x), mm(p.GetPosition().y))
                if math.hypot(px, py) < WIN:
                    obstacles.append(dict(ref=f"{ref}.{p.GetNumber()}", kind="tht_pad", circle=[px, py, max(mm(p.GetSize().x), mm(p.GetSize().y)) / 2]))
        continue
    if math.hypot(fx, fy) > WIN + 15:
        continue
    cy = f.GetCourtyard(pcbnew.B_CrtYd)
    pts = poly_pts(cy)
    if not pts:
        bb = f.GetBoundingBox(False)
        x0, y0, x1, y1 = mm(bb.GetX()), mm(bb.GetY()), mm(bb.GetRight()), mm(bb.GetBottom())
        pts = [[loc(x0, y0), loc(x1, y0), loc(x1, y1), loc(x0, y1)]]
    obstacles.append(dict(ref=ref, kind="courtyard", value=f.GetValue(), polys=pts))

eb = b.GetBoardEdgesBoundingBox()
edge = [loc(mm(eb.GetX()), mm(eb.GetY())), loc(mm(eb.GetRight()), mm(eb.GetBottom()))]
json.dump(dict(origin_kicad=[ox, oy], c130_pads={p.GetNumber(): [*loc(mm(p.GetPosition().x), mm(p.GetPosition().y)), p.GetNetname()] for p in pads},
               board_thickness=mm(b.GetDesignSettings().GetBoardThickness()), board_bbox_local=edge, obstacles=obstacles),
          open(out_path, "w"), indent=1)
print(f"origin KiCad ({ox}, {oy}); {len(obstacles)} obstacles -> {out_path}")
