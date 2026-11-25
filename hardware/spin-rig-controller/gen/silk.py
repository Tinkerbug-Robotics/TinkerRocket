"""Silkscreen: references go to the fab layer; connectors, buttons and lights get short function labels, each
placed beside its part where it overlaps no pad, no other label and no other part's silk.

Run with KiCad's Python after routing:  .../python3 silk.py
"""
import os
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kilib  # noqa: E402

PCB = os.path.join(kilib.PROJ, 'spin-rig-controller.kicad_pcb')
OX, OY, W, H = 100.0, 100.0, 110.0, 80.0
mm = pcbnew.FromMM

LABELS = {
    'J101': 'PACK -/+', 'J103': 'DRIVER PWR -/+', 'J102': 'BRAKE R', 'J301': 'CL57T  PUL DIR ENA ALM PEND',
    'J302': 'ENCODER  A B Z GND', 'J303': 'INDEX', 'J304': 'GNSS', 'J201': 'USB', 'J202': 'PI',
    'J203': 'QWIIC', 'J104': '5V AUX', 'SW201': 'BOOT', 'SW202': 'RESET', 'D104': 'BRAKE', 'D106': '5V',
    'D201': '3V3', 'D202': 'STAT', 'D203': 'FAULT', 'TP201': 'TX', 'TP202': 'RX', 'TP203': '3V3', 'TP204': '5V',
    'TP205': 'GND', 'TP206': 'GND', 'TP301': 'STEP', 'TP302': 'PPS', 'TP303': 'ENC A', 'TP304': 'EVA',
    'TP305': 'IDX', 'TP306': 'GND',
}

b = pcbnew.LoadBoard(PCB)

# 1. every reference on the fab layer
for fp in b.GetFootprints():
    r = fp.Reference()
    r.SetLayer(pcbnew.F_Fab)
    r.SetTextSize(pcbnew.VECTOR2I(mm(0.5), mm(0.5)))
    r.SetTextThickness(mm(0.08))
    v = fp.Value()
    v.SetLayer(pcbnew.F_Fab)

# remove labels from an earlier run
for d in list(b.GetDrawings()):
    if isinstance(d, pcbnew.PCB_TEXT) and d.GetLayer() == pcbnew.F_SilkS and d.GetText().startswith(tuple(
            set(LABELS.values()))):
        b.Remove(d)

blocks = []   # boxes labels must stay off: pads (with a margin) and placed labels
for fp in b.GetFootprints():
    for p in fp.Pads():
        bb = p.GetBoundingBox()
        bb.Inflate(mm(0.15))
        blocks.append(bb)
    for g in fp.GraphicalItems():
        if g.GetLayer() == pcbnew.F_SilkS:
            blocks.append(g.GetBoundingBox())
for d in b.GetDrawings():
    if d.GetLayer() == pcbnew.F_SilkS:
        blocks.append(d.GetBoundingBox())
edge = pcbnew.BOX2I(pcbnew.VECTOR2I(mm(OX + 0.6), mm(OY + 0.6)), pcbnew.VECTOR2I(mm(W - 1.2), mm(H - 1.2)))


def fits(t):
    bb = t.GetBoundingBox()
    if not edge.Contains(bb):
        return False
    return not any(bb.Intersects(o) for o in blocks)


placed, missed = 0, []
for ref, text in LABELS.items():
    fp = b.FindFootprintByReference(ref)
    if not fp:
        continue
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    cb = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False)
    size = 0.8
    t = pcbnew.PCB_TEXT(b)
    t.SetText(text)
    t.SetLayer(pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    t.SetTextThickness(mm(0.12))
    cx, cyy = (cb.GetLeft() + cb.GetRight()) // 2, (cb.GetTop() + cb.GetBottom()) // 2
    gap = mm(size * 0.9)
    cands = []
    for rot in (0, 90):
        for pos in ((cx, cb.GetTop() - gap), (cx, cb.GetBottom() + gap), (cb.GetLeft() - gap * 3, cyy),
                    (cb.GetRight() + gap * 3, cyy)):
            cands.append((rot, pos))
    # then slide along each side
    for rot in (0, 90):
        for k in range(1, 12):
            for sgn in (1, -1):
                cands.append((rot, (cx + sgn * mm(k * 1.0), cb.GetTop() - gap)))
                cands.append((rot, (cx + sgn * mm(k * 1.0), cb.GetBottom() + gap)))
                cands.append((rot, (cb.GetLeft() - gap * 3, cyy + sgn * mm(k * 1.0))))
                cands.append((rot, (cb.GetRight() + gap * 3, cyy + sgn * mm(k * 1.0))))
    ok = False
    for rot, (x, y) in cands:
        t.SetTextAngleDegrees(rot)
        t.SetPosition(pcbnew.VECTOR2I(int(x), int(y)))
        if fits(t):
            ok = True
            break
    if ok:
        b.Add(t)
        blocks.append(t.GetBoundingBox())
        placed += 1
    else:
        missed.append(ref)

# the title, where it fits
for d in b.GetDrawings():
    if isinstance(d, pcbnew.PCB_TEXT) and 'SPIN RIG CONTROLLER' in d.GetText():
        blocks_wo = [o for o in blocks if o != d.GetBoundingBox()]
        for x in range(30, 100, 2):
            for y in range(4, 78, 2):
                d.SetPosition(pcbnew.VECTOR2I(mm(OX + x), mm(OY + y)))
                bb = d.GetBoundingBox()
                if edge.Contains(bb) and not any(bb.Intersects(o) for o in blocks_wo):
                    break
            else:
                continue
            break
pcbnew.SaveBoard(PCB, b)
print('labels placed', placed, 'missed', missed)
