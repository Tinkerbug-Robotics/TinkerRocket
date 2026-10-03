#!/usr/bin/env python3
"""Mechanical facts of the HackRF Pro board for the bench rig, read from Great Scott Gadgets' layout
(praline.kicad_pcb in github.com/greatscottgadgets/hackrf-pro, CERN-OHL-P-2.0; 8 MB, not committed here -- download
it next to this script or pass its path): board outline, mounting holes, and every footprint that matters
mechanically -- edge connectors (SMA, USB-C), side buttons / LEDs, headers, the shield can, and anything on the BOTTOM
side -- with courtyard boxes in a frame whose origin is the board's lower-left corner (x right, y up, the outline
stroke removed). Run with KiCad's bundled Python (pcbnew), e.g.
    /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 \\
        board_geom.py [praline.kicad_pcb]                       -> board_geom.json"""
import json
import sys
from pathlib import Path

import pcbnew

HERE = Path(__file__).resolve().parent
b = pcbnew.LoadBoard(str(Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "praline.kicad_pcb"))
mm = pcbnew.ToMM
bb = b.GetBoardEdgesBoundingBox()                           # includes the outline's stroke width
stroke = max((mm(d.GetWidth()) for d in b.GetDrawings() if d.GetLayer() == pcbnew.Edge_Cuts), default=0.0)
x0, y1 = mm(bb.GetX()) + stroke / 2, mm(bb.GetY()) + stroke / 2     # KiCad y grows downward
W, H = mm(bb.GetWidth()) - stroke, mm(bb.GetHeight()) - stroke
x1_, y0_ = x0 + W, y1 + H


def tx(x, y):                                               # KiCad -> rig frame (origin lower-left, y up)
    return round(x - x0, 3), round(y0_ - y, 3)


edges = []
for d in b.GetDrawings():
    if d.GetLayer() == pcbnew.Edge_Cuts:
        s = d.GetShapeStr() if hasattr(d, "GetShapeStr") else str(d.GetShape())
        p0, p1 = d.GetStart(), d.GetEnd()
        e = {"shape": s, "start": tx(mm(p0.x), mm(p0.y)), "end": tx(mm(p1.x), mm(p1.y))}
        if "ARC" in s.upper() or "CIRCLE" in s.upper():
            c = d.GetCenter()
            e["center"] = tx(mm(c.x), mm(c.y))
            e["radius"] = round(mm(d.GetRadius()), 3)
        edges.append(e)

KEEP = ("SMA", "USB", "SW_", "LED", "PinSocket", "PinHeader", "MountingHole", "Laird", "BMI", "Conn", "Button",
        "Switch", "Shield", "TestPoint", "Fiducial")
fps = []
for f in b.GetFootprints():
    name = str(f.GetFPID().GetLibItemName())
    ref = f.GetReference()
    side = "bottom" if f.IsFlipped() else "top"
    cy = f.GetCourtyard(pcbnew.B_CrtYd if f.IsFlipped() else pcbnew.F_CrtYd)
    box = None
    if cy.OutlineCount():
        r = cy.BBox()
        (ax, ay), (bx, by) = tx(mm(r.GetX()), mm(r.GetY())), tx(mm(r.GetRight()), mm(r.GetBottom()))
        box = [min(ax, bx), min(ay, by), max(ax, bx), max(ay, by)]
    else:
        r = f.GetBoundingBox(False, False)
        (ax, ay), (bx, by) = tx(mm(r.GetX()), mm(r.GetY())), tx(mm(r.GetRight()), mm(r.GetBottom()))
        box = [min(ax, bx), min(ay, by), max(ax, bx), max(ay, by)]
    models = [str(m.m_Filename) for m in f.Models()]
    pos = f.GetPosition()
    rec = {"ref": ref, "fp": name, "value": f.GetValue(), "side": side, "at": tx(mm(pos.x), mm(pos.y)),
           "rot": round(f.GetOrientationDegrees(), 1), "box": [round(v, 3) for v in box], "models": models}
    if side == "bottom" or any(k.lower() in name.lower() for k in KEEP):
        fps.append(rec)
out = {"board_w": round(W, 3), "board_h": round(H, 3), "thickness": round(mm(b.GetDesignSettings().GetBoardThickness()), 4),
       "edges": edges, "footprints": sorted(fps, key=lambda r: (r["side"], r["ref"]))}
(HERE / "board_geom.json").write_text(json.dumps(out, indent=1))
print(f"board {W:.2f} x {H:.2f} mm, thickness {out['thickness']} mm, {len(edges)} edge segments, "
      f"{len(fps)} footprints kept ({sum(1 for r in fps if r['side'] == 'bottom')} on the bottom)")
for e in edges:
    print("  edge", e)
for r in out["footprints"]:
    if r["side"] == "top":
        print(f"  {r['side']:6s} {r['ref']:6s} {r['fp'][:46]:46s} at {r['at']} rot {r['rot']:6.1f} box {r['box']}")
bot = [r for r in out["footprints"] if r["side"] == "bottom"]
print(f"  bottom side: {len(bot)} footprints; kinds: {sorted(set(r['fp'][:30] for r in bot))[:40]}")
