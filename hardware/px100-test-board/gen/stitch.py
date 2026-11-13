"""GND stitching vias: a fence just inside and just outside the can's wall pads, both sides of the RF line, and a grid
through the pours (denser inside the can). A via goes in only where it clears every other net on every layer, keeps
off pads (no via in pad, GND ones included) and ends up in GND pour on at least two layers.

  KiCad python:  stitch.py
"""
import math
import os
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import layout  # noqa: E402

PCB = layout.PCB
OX, OY, W, H = layout.OX, layout.OY, layout.W, layout.H
mm, tomm = pcbnew.FromMM, pcbnew.ToMM
VIA_D, VIA_DR = 0.4, 0.3
CLR = 0.2           # to other nets (the RF class's gap; more than Default's 0.1)
PAD_CLR = 0.15      # to GND pads: no via in pad
PITCH_IN, PITCH_OUT = 1.3, 2.5
CU = (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu)


def main():
    b = pcbnew.LoadBoard(PCB)
    gnd = b.FindNet('GND').GetNetCode()
    items = [(p.GetNetCode(), p) for fp in b.GetFootprints() for p in fp.Pads()]
    items += [(t.GetNetCode(), t) for t in b.GetTracks()]
    vias = [t.GetPosition() for t in b.GetTracks() if t.GetClass() == 'PCB_VIA']
    holes = [fp.GetPosition() for fp in b.GetFootprints() if fp.GetReference().startswith('H')]

    def ok(x, y):
        c = pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))
        if x < 0.8 or y < 0.8 or x > W - 0.8 or y > H - 0.8:
            return False
        if any(math.hypot(tomm(v.x - c.x), tomm(v.y - c.y)) < VIA_D + 0.2 for v in vias):
            return False
        if any(math.hypot(tomm(h.x - c.x), tomm(h.y - c.y)) < 3.6 for h in holes):
            return False
        circ = pcbnew.SHAPE_CIRCLE(c, mm(VIA_D / 2))
        box = pcbnew.BOX2I(c - pcbnew.VECTOR2I(mm(1.5), mm(1.5)), pcbnew.VECTOR2I(mm(3), mm(3)))
        for net, it in items:
            if not it.GetBoundingBox().Intersects(box):
                continue
            is_pad = it.GetClass() == 'PAD'
            if net == gnd and not is_pad:
                continue
            gap = PAD_CLR if (net == gnd and is_pad) else CLR
            if is_pad and it.GetDrillSize().x > 0 and net == gnd:
                gap = 0.25
            for L in CU:
                if it.IsOnLayer(L) and it.GetEffectiveShape(L).Collide(circ, mm(gap)):
                    return False
        return True

    added = []

    def add(x, y):
        if not ok(x, y):
            return False
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pcbnew.VECTOR2I(mm(OX + x), mm(OY + y)))
        v.SetWidth(mm(VIA_D))
        v.SetDrill(mm(VIA_DR))
        v.SetNetCode(gnd)
        b.Add(v)
        vias.append(v.GetPosition())
        items.append((gnd, v))
        added.append(v)
        return True

    # the can: a ring 0.45 mm inside and outside the wall pads (they run 7.65-8.65 mm from its centre)
    ux, uy = layout.U1_AT
    cx, cy = ux + layout.SH_OFF[0], uy + layout.SH_OFF[1]
    for r in (7.2, 9.1):
        n = int(8 * r / 0.7)
        for k in range(n):
            t = 8 * r * k / n
            side, s = divmod(t, 2 * r)
            px, py = [(-r + s, -r), (r, -r + s), (r - s, r), (-r, r - s)][int(side)]
            add(cx + px, cy + py)
    # the RF line from J1 to the can: both edges, every 1 mm
    jx, jy = layout.J1_AT
    for x in [jx + 2.2 + 0.8 * k for k in range(int((cx - 8.65 - jx - 2.2) / 0.8) + 1)]:
        for dy in (-0.75, 0.75):
            add(x, jy + dy)
    # grids
    inside = lambda x, y: abs(x - cx) < 7.4 and abs(y - cy) < 7.4
    for pitch, sel in ((PITCH_IN, inside), (PITCH_OUT, lambda x, y: not inside(x, y))):
        y = pitch / 2
        while y < H:
            x = pitch / 2
            while x < W:
                if sel(x, y):
                    add(x, y)
                x += pitch
            y += pitch
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    # keep only vias that land in GND copper on two layers or more
    pours = [z for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetCode() == gnd]
    dropped = 0
    for v in added:
        n = sum(1 for z in pours for L in CU if z.IsOnLayer(L) and z.GetFilledPolysList(L).Contains(v.GetPosition()))
        if n < 2:
            b.Remove(v)
            dropped += 1
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(PCB, b, True)
    print('stitching vias: %d added, %d dropped (not in two GND pours)' % (len(added) - dropped, dropped))


if __name__ == '__main__':
    main()
