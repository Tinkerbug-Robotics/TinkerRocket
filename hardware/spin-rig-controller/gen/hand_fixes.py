"""Hand-directed fixes for this revision's routing (the 67 x 90 Pi-stacking layout, 2026-10-16): the items finalize.sh left,
each planned against the copper around it (gen/region.py lists a window's copper). They fit this routing result
only; after a fresh route_best.sh, check what is left and plan again.

Run with KiCad's Python after finalize.sh, then extra_pours.py, silk.py, cleanup.py and check.py.
"""
import os
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import patch_route as pr  # noqa: E402  (loads the board as pr.b)

b = pr.b
OX, OY = pr.OX, pr.OY
mm = pcbnew.FromMM
NETS = {str(b.FindNet(i).GetNetname()): i for i in range(b.GetNetCount()) if b.FindNet(i)}


def net(name):
    if name in NETS:
        return NETS[name]
    m = [n for n in NETS if n.endswith('/' + name)]
    assert len(m) == 1, (name, m)
    return NETS[m[0]]


def same(v, x, y):
    return abs(pcbnew.ToMM(v.x) - OX - x) < 0.006 and abs(pcbnew.ToMM(v.y) - OY - y) < 0.006


def find(name, a, c=None, layer=None):
    """The track (a -> c, either way round) or the via (at a) of a net."""
    n = net(name)
    out = []
    for t in b.GetTracks():
        if t.GetNetCode() != n:
            continue
        if c is None:
            if t.GetClass() == 'PCB_VIA' and same(t.GetPosition(), *a):
                out.append(t)
        elif t.GetClass() == 'PCB_TRACK' and (layer is None or t.GetLayerName() == layer):
            s, e = t.GetStart(), t.GetEnd()
            if (same(s, *a) and same(e, *c)) or (same(s, *c) and same(e, *a)):
                out.append(t)
    assert out, (name, a, c)
    return out


def remove(items):
    # match by UUID: the router's item list holds its own SWIG wrappers of the same objects
    gone = {t.m_Uuid.AsString() for t in items}
    pr.ITEMS[:] = [it for it in pr.ITEMS if it[0] == 'pad' or it[3].m_Uuid.AsString() not in gone]
    for t in items:
        b.Remove(t)


def track(name, pts_, w, layer):
    for a, c in zip(pts_, pts_[1:]):
        pr.add_track((OX + a[0], OY + a[1]), (OX + c[0], OY + c[1]), w, b.GetLayerID(layer), net(name))


def via(name, xy):
    pr.add_via(OX + xy[0], OY + xy[1], net(name))


def pad_ends(ref, num):
    p = [q for q in b.FindFootprintByReference(ref).Pads() if q.GetNumber() == num][0]
    x, y = pcbnew.ToMM(p.GetPosition().x), pcbnew.ToMM(p.GetPosition().y)
    return [(x, y, L) for L in pr.LAYERS if p.IsOnLayer(L)]


def pts(*xy, layer=pcbnew.F_Cu):
    return [(OX + x, OY + y, layer) for x, y in xy]


# 1. The pre-routed 0.25 mm escape from the ESP32's pin 20 (3.3 V) went unused: the router fed the pin its own
#    way, so the stub only dangles. Take it out.
remove(find('+3V3', (34.4, 62.43), (34.4, 65.87)))

# 2. The ESP32's 3.3 V pins 2 and 3 were boxed in by the CHIP_PU via beside them, the ring-corner ground via and
#    an In2 I2C line. Lift CHIP_PU's via and its escape, join pins 2/3 to the 3.3 V copper west of them, then
#    bring CHIP_PU out under that run to a smaller via.
remove(find('CHIP_PU', (31.57, 57.6), (31.0, 57.6)) + find('CHIP_PU', (31.0, 57.6), (31.0, 57.46)) +
       find('CHIP_PU', (31.0, 57.46), (30.73, 57.19)) + find('CHIP_PU', (30.73, 57.19)) +
       find('CHIP_PU', (30.11, 56.57), (30.73, 57.19)) + find('CHIP_PU', (30.73, 57.19), (30.73, 58.55)) +
       find('CHIP_PU', (30.73, 57.19), (30.73, 57.19)))
pr.CLR = 0.17
ok = pr.route(net('+3V3'), pad_ends('U201', '2') + pad_ends('U201', '3'),
              pts((28.74, 56.92), (28.74, 55.9), (28.74, 54.85)), 0.2)
print('+3V3 pins 2/3', 'routed' if ok else 'NO PATH')
# CHIP_PU: out of pin 4 under the new 3.3 V run to a 0.45 mm via, then on B.Cu to both of its old runs
track('CHIP_PU', [(31.57, 57.6), (30.85, 57.6), (30.6, 57.35)], 0.2, 'F.Cu')
v = pcbnew.PCB_VIA(b)
v.SetPosition(pcbnew.VECTOR2I(mm(OX + 30.6), mm(OY + 57.35)))
v.SetWidth(mm(0.45))
v.SetDrill(mm(0.3))
v.SetNetCode(net('CHIP_PU'))
b.Add(v)
track('CHIP_PU', [(30.6, 57.35), (30.11, 56.86), (30.11, 56.57)], 0.2, 'B.Cu')
track('CHIP_PU', [(30.6, 57.35), (30.73, 57.48), (30.73, 58.55)], 0.2, 'B.Cu')

pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(pr.PCB, b)
print('hand fixes done')
