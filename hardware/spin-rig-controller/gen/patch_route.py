"""Closes the last unconnected items the autorouter left: a grid A* router on F.Cu and B.Cu (vias between),
checked against every other net's pads, tracks and vias with KiCad's own shapes. Pours are ignored (they
refill around the new copper). Reads out/drc.json, writes the board.

Run with KiCad's Python:  .../python3 patch_route.py [max_items]
"""
import heapq
import json
import math
import os
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kilib  # noqa: E402

PCB = os.path.join(kilib.PROJ, 'spin-rig-controller.kicad_pcb')
DRC = os.path.join(HERE, 'out', 'drc.json')
import make_pcb  # noqa: E402
OX, OY, W, H = make_pcb.OX, make_pcb.OY, make_pcb.W, make_pcb.H
mm = pcbnew.FromMM
G = 0.2                      # grid, mm
CLR = 0.2                    # margin over the 0.15 rule (cells are checked, the segments between them sampled)
CLR_MOTOR = 0.31             # the Motor class
MOTOR = ('/Power/VPACK', '/Power/VMAIN', '/Power/VDRV', '/Power/BRAKE_SW')
LAYERS = [pcbnew.F_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
VIA_D, VIA_DR = 0.6, 0.3
NOVIA_BAND_Y = None          # vias are fine everywhere (they don't wire on In2)

b = pcbnew.LoadBoard(PCB)
MOTOR_CODES = set()
for _i in range(b.GetNetCount()):
    _n = b.FindNet(_i)
    if _n and str(_n.GetNetname()) in MOTOR:
        MOTOR_CODES.add(_i)
IN2_KEEPOUT = [z for z in b.Zones() if z.GetIsRuleArea() and z.GetDoNotAllowTracks() and z.IsOnLayer(pcbnew.In2_Cu)]


def clr(a, b_):
    return CLR_MOTOR if (a in MOTOR_CODES or b_ in MOTOR_CODES) else CLR


def in2_ok(x, y):
    p = pcbnew.VECTOR2I(mm(x), mm(y))
    return not any(z.Outline().Contains(p) for z in IN2_KEEPOUT)


ITEMS = []
for fp in b.GetFootprints():
    for p in fp.Pads():
        ITEMS.append(('pad', p.GetNetCode(), p.GetBoundingBox(), p))
for t in b.GetTracks():
    ITEMS.append(('via' if t.GetClass() == 'PCB_VIA' else 'track', t.GetNetCode(), t.GetBoundingBox(), t))
HOLES = [(fp.GetPosition(), fp) for fp in b.GetFootprints() if fp.GetReference().startswith('H')]


def near(bb):
    return [it for it in ITEMS if it[2].Intersects(bb)]


def blocked_track(x, y, layer, net, w, cand):
    if layer == pcbnew.In2_Cu and not in2_ok(x, y):
        return True
    c = pcbnew.VECTOR2I(mm(x), mm(y))
    circ = pcbnew.SHAPE_CIRCLE(c, mm(w / 2))
    for kind, nc, bb, o in cand:
        if nc == net:
            continue
        cl = mm(clr(net, nc))
        if kind == 'pad':
            if o.IsOnLayer(layer) and o.GetEffectiveShape(layer).Collide(circ, cl):
                return True
        elif kind == 'track':
            if o.GetLayer() == layer and o.GetEffectiveShape(layer).Collide(circ, cl):
                return True
        else:
            if o.GetEffectiveShape(layer).Collide(circ, cl):
                return True
    return False


def blocked_via(x, y, net, cand):
    c = pcbnew.VECTOR2I(mm(x), mm(y))
    circ = pcbnew.SHAPE_CIRCLE(c, mm(VIA_D / 2))
    for kind, nc, bb, o in cand:
        if kind == 'pad':
            for L in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
                if o.IsOnLayer(L) and o.GetEffectiveShape(L).Collide(circ, mm(clr(net, nc) if nc != net else 0.15)):
                    return True
        elif kind == 'track':
            if nc != net and o.GetEffectiveShape(o.GetLayer()).Collide(circ, mm(clr(net, nc))):
                return True
        else:
            d = math.hypot(pcbnew.ToMM(o.GetPosition().x) - x, pcbnew.ToMM(o.GetPosition().y) - y)
            if d < VIA_D + 0.3:
                return True
    for hp, fp in HOLES:
        if math.hypot(pcbnew.ToMM(hp.x) - x, pcbnew.ToMM(hp.y) - y) < 3.5 + VIA_D / 2 + 0.3:
            return True
    return False


def endpoints(item_desc_pos, net):
    """Points of the net's copper near a DRC item: the pad center, or both ends of the track."""
    x, y = item_desc_pos
    best = []
    for kind, nc, bb, o in ITEMS:
        if nc != net:
            continue
        if kind == 'pad':
            q = o.GetPosition()
            if math.hypot(pcbnew.ToMM(q.x) - x, pcbnew.ToMM(q.y) - y) < 0.05:
                layers = [L for L in LAYERS if o.IsOnLayer(L)]
                return [(pcbnew.ToMM(q.x), pcbnew.ToMM(q.y), L) for L in layers]
        elif kind == 'track':
            for q in (o.GetStart(), o.GetEnd()):
                best.append((math.hypot(pcbnew.ToMM(q.x) - x, pcbnew.ToMM(q.y) - y), pcbnew.ToMM(q.x),
                             pcbnew.ToMM(q.y), o.GetLayer()))
    if not best:
        return []
    best.sort()
    # a DRC track item's position is its start; offer both ends of every track that close
    out = []
    for d, qx, qy, L in best:
        if d < best[0][0] + 6.0:
            out.append((qx, qy, L))
    return out[:6]


def route(net, A, B, w=0.2):
    xs = [p[0] for p in A + B]
    ys = [p[1] for p in A + B]
    m = 10.0
    x0, x1 = max(min(xs) - m, OX + 0.5), min(max(xs) + m, OX + W - 0.5)
    y0, y1 = max(min(ys) - m, OY + 0.5), min(max(ys) + m, OY + H - 0.5)
    bb = pcbnew.BOX2I(pcbnew.VECTOR2I(mm(x0 - 1), mm(y0 - 1)), pcbnew.VECTOR2I(mm(x1 - x0 + 2), mm(y1 - y0 + 2)))
    cand = near(bb)
    nx, ny = int((x1 - x0) / G) + 1, int((y1 - y0) / G) + 1
    cache = {}

    def free(i, j, li):
        k = (i, j, li)
        if k not in cache:
            cache[k] = not blocked_track(x0 + i * G, y0 + j * G, LAYERS[li], net, w, cand)
        return cache[k]

    vcache = {}

    def via_free(i, j):
        k = (i, j)
        if k not in vcache:
            vcache[k] = not blocked_via(x0 + i * G, y0 + j * G, net, cand)
        return vcache[k]

    def cell(p):
        return int(round((p[0] - x0) / G)), int(round((p[1] - y0) / G)), LAYERS.index(p[2]) if p[2] in LAYERS else 0

    starts = [cell(p) for p in A]
    goals = {cell(p)[:2] + (cell(p)[2],) for p in B}
    goalxy = [(cell(p)[0], cell(p)[1]) for p in B]
    h = lambda i, j: min(math.hypot(i - gi, j - gj) for gi, gj in goalxy)
    openq = []
    came = {}
    gsc = {}
    for s in starts:
        gsc[s] = 0
        heapq.heappush(openq, (h(s[0], s[1]), 0, s))
    steps = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    found = None
    n = 0
    while openq and n < 400000:
        f, g, cur = heapq.heappop(openq)
        n += 1
        if cur in goals:
            found = cur
            break
        i, j, li = cur
        nbrs = []
        for di, dj in steps:
            ii, jj = i + di, j + dj
            if 0 <= ii < nx and 0 <= jj < ny:
                nbrs.append(((ii, jj, li), math.hypot(di, dj)))
        for lo in range(len(LAYERS)):
            if lo != li:
                nbrs.append(((i, j, lo), 12.0))
        for nb, c in nbrs:
            ii, jj, ll = nb
            if ll != li:
                if not via_free(ii, jj):       # every via is checked, goal cells included (no via in a pad)
                    continue
            elif nb not in goals and not free(ii, jj, ll):
                continue
            ng = g + c
            if ng < gsc.get(nb, 1e18):
                gsc[nb] = ng
                came[nb] = cur
                heapq.heappush(openq, (ng + h(ii, jj), ng, nb))
    if not found:
        return False
    path = [found]
    while path[-1] in came:
        path.append(came[path[-1]])
    path.reverse()
    # string-pull: on each layer run, jump to the farthest point visible in a straight line
    def clear_line(a, c, li_):
        n_ = max(2, int(math.hypot(c[0] - a[0], c[1] - a[1]) / 0.05))
        for k in range(1, n_):
            x = a[0] + (c[0] - a[0]) * k / n_
            y = a[1] + (c[1] - a[1]) * k / n_
            if blocked_track(x, y, LAYERS[li_], net, w, cand):
                return False
        return True

    pts0 = [(x0 + i * G, y0 + j * G, li) for i, j, li in path]
    smooth = [pts0[0]]
    i = 0
    while i < len(pts0) - 1:
        j = i + 1
        k = len(pts0) - 1
        while k > i + 1:
            if all(pts0[q][2] == pts0[i][2] for q in range(i, k + 1)) and clear_line(pts0[i], pts0[k], pts0[i][2]):
                j = k
                break
            k -= 1
        smooth.append(pts0[j])
        i = j
    path = [(int(round((p_[0] - x0) / G)), int(round((p_[1] - y0) / G)), p_[2]) for p_ in smooth]
    # emit: straight runs merged, a via at each layer change
    pts = [(x0 + i * G, y0 + j * G, li) for i, j, li in path]
    # snap the ends onto the exact copper they started and finished on
    for P_, k in ((A, 0), (B, -1)):
        for p in P_:
            if cell(p) == path[k] or cell(p)[:2] == path[k][:2]:
                exact = (p[0], p[1], path[k][2])
                if k == 0:
                    pts.insert(0, exact)
                else:
                    pts.append(exact)
                break
    seg_start = pts[0]
    prev = pts[0]
    dirn = None
    for p in pts[1:]:
        if p[2] != prev[2]:
            if (prev[0], prev[1]) != (seg_start[0], seg_start[1]):
                add_track(seg_start, prev, w, LAYERS[prev[2]], net)
            add_via(prev[0], prev[1], net)
            seg_start = p
            dirn = None
        else:
            d = (round(p[0] - prev[0], 3), round(p[1] - prev[1], 3))
            if dirn is not None and d != dirn:
                add_track(seg_start, prev, w, LAYERS[prev[2]], net)
                seg_start = prev
            dirn = d
        prev = p
    if (prev[0], prev[1]) != (seg_start[0], seg_start[1]):
        add_track(seg_start, prev, w, LAYERS[prev[2]], net)
    # the first and last points snap onto the actual copper
    return True


def add_track(a, c, w, layer, net):
    t = pcbnew.PCB_TRACK(b)
    t.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
    t.SetEnd(pcbnew.VECTOR2I(mm(c[0]), mm(c[1])))
    t.SetWidth(mm(w))
    t.SetLayer(layer)
    t.SetNetCode(net)
    b.Add(t)
    ITEMS.append(('track', net, t.GetBoundingBox(), t))


def add_via(x, y, net):
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    v.SetWidth(mm(VIA_D))
    v.SetDrill(mm(VIA_DR))
    v.SetNetCode(net)
    b.Add(v)
    ITEMS.append(('via', net, v.GetBoundingBox(), v))


def main():
    lim = int(sys.argv[1]) if len(sys.argv) > 1 else 50
    nets = {str(b.FindNet(i).GetNetname()): i for i in range(b.GetNetCount()) if b.FindNet(i)}
    d = json.load(open(DRC))
    done = 0
    for x in d.get('unconnected_items', [])[:lim]:
        its = x['items']
        desc = its[0]['description']
        if '[' not in desc or desc.startswith('Zone'):
            continue
        name = desc.split('[')[1].split(']')[0]
        if name == 'GND':
            continue
        net = nets[name]
        A = endpoints((its[0]['pos']['x'], its[0]['pos']['y']), net)
        B = endpoints((its[1]['pos']['x'], its[1]['pos']['y']), net)
        if not A or not B:
            print('skip', name, 'no endpoints')
            continue
        if name in MOTOR:
            widths = (1.5, 1.0, 0.5)
        elif name in ('+5V', '+3V3', 'VUSB', '+5V_PI', 'GND') or name.startswith('/Power/'):
            widths = (0.5, 0.3, 0.2)
        else:
            widths = (0.2,)
        ok = False
        for w in widths:
            ok = route(net, A, B, w)
            if ok:
                break
        print('%-24s %s' % (name, 'routed' if ok else 'NO PATH'))
        done += ok
    # ground islands without a via: route one of their pads out to the nearest spot a via fits
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    gnd = nets['GND']
    gv = [t.GetPosition() for t in b.GetTracks() if t.GetClass() == 'PCB_VIA' and t.GetNetCode() == gnd]
    for z in [z for z in b.Zones() if z.GetNetname() == 'GND' and z.GetLayer() in (pcbnew.F_Cu, pcbnew.B_Cu)]:
        polys = z.GetFilledPolysList(z.GetLayer())
        for i in range(polys.OutlineCount()):
            one = pcbnew.SHAPE_POLY_SET(polys.Outline(i))
            if any(one.Contains(v) for v in gv):
                continue
            pads = [p for fp in b.GetFootprints() for p in fp.Pads()
                    if p.GetNetCode() == gnd and p.IsOnLayer(z.GetLayer()) and one.Contains(p.GetPosition())]
            if not pads or any(p.GetDrillSize().x > 0 for p in pads):
                continue
            r = route_to_plane(pads[0])
            print('ground island with %s: %s' % (pads[0].GetParentFootprint().GetReference(),
                                                 'via at %.1f mm' % r if r else 'NO ROOM'))
            done += bool(r)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(PCB, b)
    print('patched', done)



def route_to_plane(pad, w=0.25, max_mm=8.0):
    """From a pad, search outward on its layer for the nearest spot a via fits, route there, drop the via.
    For plane nets (GND on In1) whose pad sits in a pocket the stitcher couldn't reach."""
    net = pad.GetNetCode()
    layer = pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
    li = LAYERS.index(layer)
    q = pad.GetPosition()
    sx, sy = pcbnew.ToMM(q.x), pcbnew.ToMM(q.y)
    x0, y0 = sx - max_mm, sy - max_mm
    bb = pcbnew.BOX2I(pcbnew.VECTOR2I(mm(x0 - 1), mm(y0 - 1)), pcbnew.VECTOR2I(mm(2 * max_mm + 2), mm(2 * max_mm + 2)))
    cand = near(bb)
    n = int(2 * max_mm / G)
    si, sj = int(round((sx - x0) / G)), int(round((sy - y0) / G))
    came = {(si, sj): None}
    frontier = [(0.0, si, sj)]
    while frontier:
        dist, i, j = heapq.heappop(frontier)
        x, y = x0 + i * G, y0 + j * G
        if math.hypot(x - sx, y - sy) > 0.5 and not blocked_via(x, y, net, cand):
            path = [(i, j)]
            while came[path[-1]] is not None:
                path.append(came[path[-1]])
            path.reverse()
            pts = [(sx, sy)] + [(x0 + a * G, y0 + c * G) for a, c in path[1:]]
            for a, c in zip(pts, pts[1:]):
                add_track(a, c, w, layer, net)
            add_via(x, y, net)
            return math.hypot(x - sx, y - sy)
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            ii, jj = i + di, j + dj
            if not (0 <= ii <= n and 0 <= jj <= n) or (ii, jj) in came:
                continue
            if not blocked_track(x0 + ii * G, y0 + jj * G, layer, net, w, cand):
                came[(ii, jj)] = (i, j)
                heapq.heappush(frontier, (dist + math.hypot(di, dj), ii, jj))
    return None


if __name__ == '__main__':
    main()
