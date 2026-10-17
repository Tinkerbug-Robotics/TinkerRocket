"""Hand-directed fixes for the 2026-10-16 revision's routing: the items finalize.sh left, each planned against the
copper around it (gen/region.py lists a window's copper). They fit this routing result only; after a fresh
route_best.sh, check what is left and plan again.

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


def P(x, y):
    return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


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


def track(name, pts, w, layer):
    for a, c in zip(pts, pts[1:]):
        pr.add_track((OX + a[0], OY + a[1]), (OX + c[0], OY + c[1]), w, b.GetLayerID(layer), net(name))


def via(name, xy):
    pr.add_via(OX + xy[0], OY + xy[1], net(name))


# 1. U101 pin 6 (+3V3): the router's 0.5 mm stub came within 0.07 mm of pin 7 and its GND stub. Neck it to 0.25.
for t in find('+3V3', (16.1, 23.0), (16.1, 23.0)) + find('+3V3', (16.1, 23.0), (15.9, 23.2)):
    t.SetWidth(mm(0.25))

# 2. BUCK_BOOT: pin 1 west along the channel between the input caps' two pad rows (as V1 ran it), down through
#    the gap between C108 and C109 on B.Cu, and up into C112.
track('BUCK_BOOT', [(17.44, 60.09), (16.72, 60.81), (13.0, 60.81), (12.6, 61.21), (12.6, 61.6)], 0.2, 'F.Cu')
via('BUCK_BOOT', (12.6, 61.6))
track('BUCK_BOOT', [(12.6, 61.6), (12.6, 65.5), (13.0, 65.9)], 0.2, 'B.Cu')
via('BUCK_BOOT', (13.0, 65.9))
track('BUCK_BOOT', [(13.0, 65.9), (13.52, 65.38), (13.52, 65.0)], 0.2, 'F.Cu')

# 3. BUCK_RT: R115 sat inside a GND track loop (R115 pin 2 to R114, which the F.Cu pour joins anyway). Take the
#    loop out; pin 4 drops into the pocket inside BUCK_EN's path, crosses on B.Cu, and comes up under R115.
for a, c in (((14.75, 66.99), (14.69, 66.99)), ((14.69, 66.99), (14.01, 67.67)), ((14.01, 67.67), (14.01, 68.36)),
             ((14.01, 68.36), (14.41, 68.76)), ((14.41, 68.76), (18.84, 68.76)), ((18.84, 68.76), (19.51, 68.09))):
    remove(find('GND', a, c, 'F.Cu'))
track('BUCK_RT', [(17.44, 63.91), (16.6, 63.91), (16.05, 64.46), (16.05, 64.95)], 0.2, 'F.Cu')
via('BUCK_RT', (16.05, 64.95))
track('BUCK_RT', [(16.05, 64.95), (14.75, 69.0)], 0.2, 'B.Cu')
via('BUCK_RT', (14.75, 69.0))
track('BUCK_RT', [(14.75, 69.0), (14.75, 68.01)], 0.2, 'F.Cu')

# 4. LED_STAT (U201 pin 17) was boxed in by pin 16's and pin 18's escapes and the 3.3 V feed. Move PI_TX's via
#    down and right (its In2 run shifts over to match), neck the 3.3 V escape from pin 20 to 0.25 mm, and give
#    LED_STAT a straight escape to its own via; the patch router takes it from there to R209.
remove(find('PI_TX', (70.6, 47.43), (70.6, 48.21), 'F.Cu') + find('PI_TX', (70.6, 48.21)) +
       find('PI_TX', (70.6, 52.93), (70.6, 48.21), 'In2.Cu'))
track('PI_TX', [(70.6, 47.43), (70.6, 48.4), (70.8, 48.6), (70.8, 48.9)], 0.2, 'F.Cu')
via('PI_TX', (70.8, 48.9))
track('PI_TX', [(70.8, 48.9), (70.95, 49.05), (70.95, 52.58), (70.6, 52.93)], 0.2, 'In2.Cu')
for t in find('+3V3', (71.4, 50.87), (71.4, 48.15), 'F.Cu'):
    t.SetWidth(mm(0.25))
track('LED_STAT', [(70.2, 47.43), (70.2, 50.1)], 0.2, 'F.Cu')
via('LED_STAT', (70.2, 50.1))
n = net('LED_STAT')
A = [(OX + 70.2, OY + 50.1, L) for L in pr.LAYERS]
B = [(OX + 60.5, OY + 71.26, pcbnew.F_Cu)]
print('LED_STAT', 'routed' if pr.route(n, A, B, 0.2) else 'NO PATH')

# 5. J303's 5 V: the feeder via and the router's stub from pin 4 never met.
track('+5V', [(102.5, 46.6), (105.02, 46.64)], 0.5, 'F.Cu')

# 6. The clamp's 5 V feeder: the router carried the In2 feeder on to a via of its own, so the pre-placed via at
#    the feeder's top only touches In2. Take it out.
remove(find('+5V', (36.5, 39.5)))

pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(pr.PCB, b)
print('hand fixes done')
