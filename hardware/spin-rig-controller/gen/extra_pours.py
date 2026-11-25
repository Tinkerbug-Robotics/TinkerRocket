"""Adds the 1.2 mm self-pours (see make_pcb.POUR_NETS) to an already-routed board, for nets it lacks them on."""
import os
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_pcb  # noqa: E402

b = pcbnew.LoadBoard(make_pcb.PCB)
nets = {}
for code in range(b.GetNetCount()):
    ni = b.FindNet(code)
    if ni:
        nets[str(ni.GetNetname())] = ni


def net(name):
    if name in nets:
        return nets[name]
    m = [n for n in nets if n.endswith('/' + name)]
    return nets[m[0]]


have = {(z.GetNetname(), z.GetLayer()) for z in b.Zones() if not z.GetIsRuleArea() and z.GetAssignedPriority() == 3}
edge = pcbnew.SHAPE_POLY_SET()
edge.NewOutline()
for x, y in ((0.5, 0.5), (make_pcb.W - 0.5, 0.5), (make_pcb.W - 0.5, make_pcb.H - 0.5), (0.5, make_pcb.H - 0.5)):
    edge.Append(pcbnew.FromMM(make_pcb.OX + x), pcbnew.FromMM(make_pcb.OY + y))
added = 0
for name in make_pcb.POUR_NETS:
    n = net(name)
    for layer in ('F.Cu', 'B.Cu'):
        lid = b.GetLayerID(layer)
        if (n.GetNetname(), lid) in have:
            continue
        ps = pcbnew.SHAPE_POLY_SET()
        for tr in b.GetTracks():
            if tr.GetNetCode() == n.GetNetCode() and tr.GetLayer() == lid and tr.GetClass() == 'PCB_TRACK':
                tr.TransformShapeToPolygon(ps, lid, pcbnew.FromMM(1.2), pcbnew.FromMM(0.01), pcbnew.ERROR_INSIDE)
        for fp in b.GetFootprints():
            for pad in fp.Pads():
                if fp.GetReference() in ('J202', 'J104'):
                    continue              # leave the header pins' neighbors room for their thermal spokes
                if pad.GetNetCode() == n.GetNetCode() and pad.IsOnLayer(lid):
                    pad.TransformShapeToPolygon(ps, lid, pcbnew.FromMM(1.2), pcbnew.FromMM(0.01), pcbnew.ERROR_INSIDE)
        if ps.OutlineCount() == 0:
            continue
        ps.Simplify()
        ps.BooleanIntersection(edge)
        z = pcbnew.ZONE(b)
        z.SetLayer(lid)
        z.SetNet(n)
        z.Outline().Append(ps)
        z.SetAssignedPriority(3)
        z.SetMinThickness(pcbnew.FromMM(0.2))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        b.Add(z)
        added += 1
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(make_pcb.PCB, b)
print('pours added', added)
