"""Hand-directed fixes for this revision's routing (the Pi-stacking layout, 2026-10-16): the items finalize.sh left,
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


# 1. U101 pin 6 (+3V3): the router's 0.5 mm stub came within 0.1 mm of pin 7's GND stub. Neck it to 0.25.
for t in find('+3V3', (15.9, 23.2), (16.1, 23.0)):
    t.SetWidth(mm(0.25))

# 2. R101's ground via sat 0.2 mm from the 1.5 mm VMAIN trunk (the Motor class wants 0.3). Put it on the pad's
#    other side, away from the trunk.
remove(find('GND', (37.71, 22.0)) + find('GND', (37.01, 22.0), (37.71, 22.0)))
track('GND', [(37.01, 22.0), (36.3, 22.9)], 0.3, 'F.Cu')
via('GND', (36.3, 22.9))

# 3. J303's 5 V: the feeder via and the router's stub from pin 4 never met.
track('+5V', [(102.5, 46.6), (105.02, 46.64)], 0.5, 'F.Cu')

# 4. C104's ground pad was walled in by INA_ALERT's loop out to its pull-up R103. Lift the loop, tie C104 to the
#    ground track just above it, and route R103 back to INA_ALERT another way (step 6).
remove(find('INA_ALERT', (11.83, 22.07), (11.9, 22.0)) + find('INA_ALERT', (8.36, 22.07), (11.83, 22.07)) +
       find('INA_ALERT', (8.36, 23.9), (8.36, 22.07)) + find('INA_ALERT', (7.75, 24.51), (8.36, 23.9)))
track('GND', [(9.5, 22.77), (9.5, 21.01)], 0.3, 'F.Cu')

# 5. The buck's small nets. BUCK_RT's route stays. R113's half of BUCK_EN (R113 to R114) hops under it on B.Cu to
#    the half at U104 pin 3 and R124. BUCK_BOOT runs west along the channel between the input caps' two pad rows,
#    drops to B.Cu, passes west of BUCK_EN's new via, and comes up under C112.
via('BUCK_EN', (17.4, 68.0))
track('BUCK_EN', [(17.4, 68.0), (12.6, 64.42)], 0.2, 'B.Cu')
via('BUCK_EN', (12.6, 64.42))
track('BUCK_BOOT', [(17.44, 60.09), (16.72, 60.81), (13.0, 60.81), (12.6, 61.21), (12.6, 61.6)], 0.2, 'F.Cu')
via('BUCK_BOOT', (12.6, 61.6))
track('BUCK_BOOT', [(12.6, 61.6), (11.9, 62.3), (11.9, 65.2), (13.0, 65.9)], 0.2, 'B.Cu')
via('BUCK_BOOT', (13.0, 65.9))
track('BUCK_BOOT', [(13.0, 65.9), (13.52, 65.38), (13.52, 65.0)], 0.2, 'F.Cu')

# 6. INA_ALERT: R103 back to the net (U101 pin 3, or its via down to B.Cu)
ok = pr.route(net('INA_ALERT'), pad_ends('R103', '1'),
              pad_ends('U101', '3') + pts((14.52, 22.7), layer=pcbnew.B_Cu) + pts((14.52, 22.7)), 0.2)
print('INA_ALERT', 'routed' if ok else 'NO PATH')

pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(pr.PCB, b)
print('hand fixes done')
