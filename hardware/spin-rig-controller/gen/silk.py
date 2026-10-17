"""Silkscreen: references go to the fab layer; connectors, buttons and lights get short function labels, each
placed beside its part where it overlaps no pad, no other label and no other part's silk. The driver's headers
also get a name at every pin, on the board side of the header, so each lead is wired by name.

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
    'J101': 'PACK', 'J103': 'DRIVER P4', 'J102': 'BRAKE R', 'J301': 'DRIVER P1', 'J305': 'DRIVER P2',
    'J302': 'ENCODER IN', 'J303': 'INDEX', 'J304': 'GNSS', 'J201': 'USB', 'J202': 'PI',
    'J203': 'QWIIC', 'J104': '5V AUX', 'SW201': 'BOOT', 'SW202': 'RESET', 'D104': 'BRAKE', 'D106': '5V',
    'D201': '3V3', 'D202': 'STAT', 'D203': 'FAULT', 'TP201': 'TX', 'TP202': 'RX', 'TP203': '3V3', 'TP204': '5V',
    'TP205': 'GND', 'TP206': 'GND', 'TP301': 'STEP', 'TP302': 'PPS', 'TP303': 'ENC A', 'TP304': 'EVA',
    'TP305': 'IDX', 'TP306': 'GND',
}

# per-pin names, in the driver's own words
PIN_LABELS = {
    'J301': {1: 'PUL+', 2: 'PUL-', 3: 'DIR+', 4: 'DIR-', 5: 'ENA+', 6: 'ENA-', 7: 'ALM', 8: 'BRK', 9: 'COM-'},
    'J305': {1: 'EB+', 2: 'EB-', 3: 'EA+', 4: 'EA-', 5: 'VCC', 6: 'EGND'},
    'J302': {6: 'EB+', 5: 'EB-', 4: 'EA+', 3: 'EA-', 2: 'VCC', 1: 'EGND'},
    'J103': {1: '+VDC', 2: 'GND'},
}
ALL_TEXT = set(LABELS.values()) | {t for d in PIN_LABELS.values() for t in d.values()}

b = pcbnew.LoadBoard(PCB)

# 1. every reference on the fab layer
for fp in b.GetFootprints():
    r = fp.Reference()
    r.SetLayer(pcbnew.F_Fab)
    r.SetTextSize(pcbnew.VECTOR2I(mm(0.5), mm(0.5)))
    r.SetTextThickness(mm(0.08))
    v = fp.Value()
    v.SetLayer(pcbnew.F_Fab)

# remove labels from an earlier run (then save and reload: after removals, KiCad's Python can hand back
# footprints as bare SWIG pointers)
old = [d for d in b.GetDrawings() if isinstance(d, pcbnew.PCB_TEXT) and d.GetLayer() == pcbnew.F_SilkS
       and d.GetText() in ALL_TEXT]
if old:
    for d in old:
        b.Remove(d)
    pcbnew.SaveBoard(PCB, b)
    b = pcbnew.LoadBoard(PCB)

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

# pin names first: a row just behind each header's pins (on the board side, clear of the header's rear silk line),
# reading along the row; a name slides further in if a pad is in the way
for ref, names in PIN_LABELS.items():
    fp = b.FindFootprintByReference(ref)
    rot = round(fp.GetOrientationDegrees()) % 360
    # inward unit vector: the header opens away from it (rot 0 opens +y on the board)
    inward = {0: (0, -1), 90: (-1, 0), 180: (0, 1), 270: (1, 0)}[rot]
    # pin 1's name may also step outward along the row, off the footprint's pin-1 mark
    pos = {p.GetNumber(): p.GetPosition() for p in fp.Pads()}
    u = (pos['1'].x - pos['2'].x, pos['1'].y - pos['2'].y)
    ul = (u[0] ** 2 + u[1] ** 2) ** 0.5
    u = (u[0] / ul, u[1] / ul)
    for p in fp.Pads():
        if not p.GetNumber() or int(p.GetNumber()) not in names:
            continue
        t = pcbnew.PCB_TEXT(b)
        t.SetText(names[int(p.GetNumber())])
        t.SetLayer(pcbnew.F_SilkS)
        t.SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
        t.SetTextThickness(mm(0.12))
        t.SetTextAngleDegrees(0 if inward[0] == 0 else 90)
        pp = p.GetPosition()
        t.SetPosition(pcbnew.VECTOR2I(int(pp.x), int(pp.y)))
        bb = t.GetBoundingBox()
        half = (bb.GetHeight() if inward[0] == 0 else bb.GetWidth()) // 2
        ok = False
        for side in ((0.0, 1.8, 2.2, 2.6) if p.GetNumber() == '1' else (0.0,)):
            for k in range(16):
                d = mm(1.31 + 0.06 + 0.15 + 0.25 * k) + half      # the rear silk line sits 1.31 mm behind the pins
                t.SetPosition(pcbnew.VECTOR2I(int(pp.x + inward[0] * d + u[0] * mm(side)),
                                              int(pp.y + inward[1] * d + u[1] * mm(side))))
                if fits(t):
                    ok = True
                    break
            if ok:
                break
        if ok:
            b.Add(t)
            blocks.append(t.GetBoundingBox())
            placed += 1
        else:
            missed.append('%s.%s' % (ref, p.GetNumber()))

# the other labels stay off the connectors' bodies too
for fp in b.GetFootprints():
    if fp.GetReference().startswith('J'):
        cy = fp.GetCourtyard(pcbnew.F_CrtYd)
        if cy.OutlineCount():
            blocks.append(cy.BBox())

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
    if ref in PIN_LABELS:
        # a driver header's name goes in a row behind its pin names, along the header, as near its middle as fits
        rot_ = round(fp.GetOrientationDegrees()) % 360
        inw = {0: (0, -1), 90: (-1, 0), 180: (0, 1), 270: (1, 0)}[rot_]
        pads = [q.GetPosition() for q in fp.Pads()]
        mx = sum(q.x for q in pads) // len(pads)
        my = sum(q.y for q in pads) // len(pads)
        along = (1, 0) if inw[0] == 0 else (0, 1)
        for d in (3.0, 3.5, 4.0, 4.5):
            for k in range(0, 25):
                for sgn in ((1,) if k == 0 else (1, -1)):
                    o = mm(sgn * k * 0.5)
                    cands.append((0 if inw[0] == 0 else 90, (mx + inw[0] * mm(d) + along[0] * o,
                                                             my + inw[1] * mm(d) + along[1] * o)))
    # beside the part, then sliding along each side, then a step further out (past a row of pin names)
    for far in (0.0, 0.7, 1.4, 2.1):
        g = gap + mm(far)
        for rot in (0, 90):
            for pos in ((cx, cb.GetTop() - g), (cx, cb.GetBottom() + g), (cb.GetLeft() - gap * 3 - mm(far), cyy),
                        (cb.GetRight() + gap * 3 + mm(far), cyy)):
                cands.append((rot, pos))
        for rot in (0, 90):
            for k in range(1, 12):
                for sgn in (1, -1):
                    cands.append((rot, (cx + sgn * mm(k * 1.0), cb.GetTop() - g)))
                    cands.append((rot, (cx + sgn * mm(k * 1.0), cb.GetBottom() + g)))
                    cands.append((rot, (cb.GetLeft() - gap * 3 - mm(far), cyy + sgn * mm(k * 1.0))))
                    cands.append((rot, (cb.GetRight() + gap * 3 + mm(far), cyy + sgn * mm(k * 1.0))))
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
        blocks.append(d.GetBoundingBox())
        title = d

# JLC's order-number marker: JLC prints the order number in place of this text when the order picks
# "Specify a location", instead of somewhere of its own choosing. Beside the title, where it fits.
for d in list(b.GetDrawings()):
    if isinstance(d, pcbnew.PCB_TEXT) and d.GetText() == 'JLCJLCJLCJLC':
        b.Remove(d)
t = pcbnew.PCB_TEXT(b)
t.SetText('JLCJLCJLCJLC')
t.SetLayer(pcbnew.F_SilkS)
t.SetTextSize(pcbnew.VECTOR2I(mm(1.0), mm(1.0)))
t.SetTextThickness(mm(0.15))
tb = title.GetBoundingBox()
cands = []
for dy in (1.4, 1.9, 2.4):
    for dx in (0, -2, 2, -4, 4, -6, 6, -8, 8):
        cands += [(tb.GetCenter().x + mm(dx), tb.GetTop() - mm(dy)), (tb.GetCenter().x + mm(dx), tb.GetBottom() + mm(dy))]
cands += [(mm(OX + x), mm(OY + y)) for x in range(10, 100, 2) for y in range(4, 78, 2)]
for x, y in cands:
    t.SetPosition(pcbnew.VECTOR2I(int(x), int(y)))
    if fits(t):
        b.Add(t)
        blocks.append(t.GetBoundingBox())
        break
else:
    missed.append('JLC order number')
pcbnew.SaveBoard(PCB, b)
print('labels placed', placed, 'missed', missed)
