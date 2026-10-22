"""After routing: tie plane-net pads the router left hanging to their plane (a short stub and a via), and stitch
the outer ground pours to the In1 plane with vias wherever they fit. Every via and stub is checked for
clearance against all copper before it is added; nothing in a pad (no via-in-pad on 4 layers).

Run with KiCad's Python after `make_pcb.py finish` and `check.py`:  .../python3 fixup.py
"""
import json
import math
import os
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kilib  # noqa: E402
import make_pcb  # noqa: E402

PCB = os.path.join(kilib.PROJ, 'spin-rig-controller.kicad_pcb')
DRC = os.path.join(HERE, 'out', 'drc.json')
OX, OY, W, H = make_pcb.OX, make_pcb.OY, make_pcb.W, make_pcb.H
CLR = pcbnew.FromMM(0.2)
CU = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
_p5 = make_pcb.IN2_5V
PLANE = {'GND': [(0, 0, W, H)],                    # In1, everywhere
         '+5V': [(min(x for x, _ in _p5), min(y for _, y in _p5), max(x for x, _ in _p5), max(y for _, y in _p5))]}  # In2
# 3.3 V is routed as tracks: a pour on In2 only fragmented around the signals there

b = pcbnew.LoadBoard(PCB)
mm = pcbnew.FromMM


def inside(xy, rects, m=0.0):
    x, y = pcbnew.ToMM(xy.x) - OX, pcbnew.ToMM(xy.y) - OY
    return any(r[0] + m <= x <= r[2] - m and r[1] + m <= y <= r[3] - m for r in rects)


class Obstacles:
    def __init__(self):
        self.items = []   # (bbox, netcode, kind, obj)
        for fp in b.GetFootprints():
            for p in fp.Pads():
                self.items.append((p.GetBoundingBox(), p.GetNetCode(), 'pad', p))
        for t in b.GetTracks():
            self.items.append((t.GetBoundingBox(), t.GetNetCode(), 'via' if t.GetClass() == 'PCB_VIA' else 'track', t))

    def near(self, bbox):
        for it in self.items:
            if it[0].Intersects(bbox):
                yield it

    def add(self, obj, kind):
        self.items.append((obj.GetBoundingBox(), obj.GetNetCode(), kind, obj))


obs = Obstacles()


def box(c, r):
    bb = pcbnew.BOX2I(pcbnew.VECTOR2I(c.x - r, c.y - r), pcbnew.VECTOR2I(2 * r, 2 * r))
    return bb


def via_ok(c, d, drill, net):
    r = mm(d) // 2
    if not inside(c, [(0.6, 0.6, W - 0.6, H - 0.6)]):
        return False
    circ = pcbnew.SHAPE_CIRCLE(c, r)
    for bb, nc, kind, o in obs.near(box(c, r + mm(1.5))):
        if kind == 'pad':
            for L in CU:
                if not o.IsOnLayer(L):
                    continue
                gap = mm(0.15) if nc == net else CLR      # no via in a pad, even its own
                if o.GetEffectiveShape(L).Collide(circ, gap):
                    return False
            if o.GetDrillSize().x > 0:
                hd = (pcbnew.ToMM(o.GetPosition().x) - pcbnew.ToMM(c.x)) ** 2 + \
                     (pcbnew.ToMM(o.GetPosition().y) - pcbnew.ToMM(c.y)) ** 2
                if math.sqrt(hd) < pcbnew.ToMM(o.GetDrillSize().x) / 2 + drill / 2 + 0.3:
                    return False
        elif kind == 'track':
            if nc != net and o.GetEffectiveShape(o.GetLayer()).Collide(circ, CLR):
                return False
        else:  # via
            dd = math.hypot(pcbnew.ToMM(o.GetPosition().x - c.x), pcbnew.ToMM(o.GetPosition().y - c.y))
            if dd < pcbnew.ToMM(o.GetWidth(pcbnew.F_Cu)) / 2 + d / 2 + 0.25:
                return False
    return True


def stub_ok(a, c, w, layer, net, pad):
    seg = pcbnew.SHAPE_SEGMENT(a, c, mm(w))
    for bb, nc, kind, o in obs.near(box(c, mm(4))):
        if nc == net:
            continue
        if kind == 'pad':
            if o.IsOnLayer(layer) and o.GetEffectiveShape(layer).Collide(seg, CLR):
                return False
        elif kind == 'track':
            if o.GetLayer() == layer and o.GetEffectiveShape(layer).Collide(seg, CLR):
                return False
        else:
            if o.GetEffectiveShape(layer).Collide(seg, CLR):
                return False
    return True


def add_via(c, net, d=0.6, drill=0.3):
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(c)
    v.SetWidth(mm(d))
    v.SetDrill(mm(drill))
    v.SetNetCode(net)
    b.Add(v)
    obs.add(v, 'via')


def add_track(a, c, w, layer, net):
    t = pcbnew.PCB_TRACK(b)
    t.SetStart(a)
    t.SetEnd(c)
    t.SetWidth(mm(w))
    t.SetLayer(layer)
    t.SetNetCode(net)
    b.Add(t)
    obs.add(t, 'track')


def fanout(pad):
    """A stub from the pad and a via to the net's plane, as short as fits."""
    net = pad.GetNetCode()
    name = pad.GetNetname()
    layer = pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
    a = pad.GetPosition()
    w = 0.3 if name == 'GND' else 0.25
    for rad in [x * 0.1 for x in range(6, 32)]:
        for k in range(32):
            th = 2 * math.pi * k / 32
            c = pcbnew.VECTOR2I(a.x + mm(rad * math.cos(th)), a.y + mm(rad * math.sin(th)))
            if not inside(c, PLANE[name], 0.8):
                continue
            for d, dr in ((0.6, 0.3), (0.45, 0.3)):
                if via_ok(c, d, dr, net) and stub_ok(a, c, w, layer, net, pad):
                    add_track(a, c, w, layer, net)
                    add_via(c, net, d, dr)
                    return rad
    return None


def main():
    nets = {str(b.FindNet(i).GetNetname()): i for i in range(b.GetNetCount()) if b.FindNet(i)}
    d = json.load(open(DRC))
    done = set()
    for x in d.get('unconnected_items', []):
        for it in x['items']:
            desc = it['description']
            if not desc.startswith('Pad ') or '[' not in desc:
                continue
            name = desc.split('[')[1].split(']')[0]
            if name not in PLANE:
                continue
            num = desc.split()[1]
            ref = desc.split(' of ')[1].split()[0]
            key = (ref, num)
            if key in done:
                continue
            done.add(key)
            pad = [p for p in b.FindFootprintByReference(ref).Pads() if p.GetNumber() == num][0]
            r = fanout(pad)
            print('fanout %s.%s [%s]: %s' % (ref, num, name, 'via at %.1f mm' % r if r else 'NO ROOM'))
    # ground stitching on a 3 mm grid, only where the F.Cu ground pour is
    gnd = nets['GND']
    fz = [z for z in b.Zones() if z.GetNetname() == 'GND' and z.GetLayer() == pcbnew.F_Cu][0]
    fill = fz.GetFilledPolysList(pcbnew.F_Cu)
    bz = [z for z in b.Zones() if z.GetNetname() == 'GND' and z.GetLayer() == pcbnew.B_Cu][0]
    bfill = bz.GetFilledPolysList(pcbnew.B_Cu)
    n = 0
    y = 1.5
    while y < H - 1.0:
        x = 1.5
        while x < W - 1.0:
            c = pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))
            if fill.Contains(c) and bfill.Contains(c) and via_ok(c, 0.6, 0.3, gnd):
                add_via(c, gnd)
                n += 1
            x += 3.0
        y += 3.0
    # denser around the ESP32, where the decoupling caps' ground pads sit
    ux, uy = make_pcb.U201_XY
    y = uy - 13.0
    while y < uy + 13.0:
        x = ux - 13.0
        while x < ux + 14.0:
            c = pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))
            if fill.Contains(c) and via_ok(c, 0.6, 0.3, gnd):
                add_via(c, gnd)
                n += 1
            x += 1.5
        y += 1.5
    print('stitching vias:', n)
    # any F.Cu or B.Cu ground island still without a via gets one, anywhere inside it that fits
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    gvias = [t.GetPosition() for t in b.GetTracks() if t.GetClass() == 'PCB_VIA' and t.GetNetCode() == gnd]
    for z in [z for z in b.Zones() if z.GetNetname() == 'GND' and z.GetLayer() in (pcbnew.F_Cu, pcbnew.B_Cu)]:
        polys = z.GetFilledPolysList(z.GetLayer())
        for i in range(polys.OutlineCount()):
            one = pcbnew.SHAPE_POLY_SET(polys.Outline(i))
            if any(one.Contains(v) for v in gvias):
                continue
            bb = one.BBox()
            placed = False
            step = mm(0.4)
            yy = bb.GetTop() + step
            while yy < bb.GetBottom() and not placed:
                xx = bb.GetLeft() + step
                while xx < bb.GetRight():
                    c = pcbnew.VECTOR2I(xx, yy)
                    if one.Contains(c) and via_ok(c, 0.6, 0.3, gnd):
                        add_via(c, gnd)
                        gvias.append(c)
                        placed = True
                        break
                    xx += step
                yy += step
            if not placed:
                # no room inside: fan a ground pad in the island out to a via instead
                for fp in b.GetFootprints():
                    for p in fp.Pads():
                        if p.GetNetCode() == gnd and p.IsOnLayer(z.GetLayer()) and one.Contains(p.GetPosition()):
                            if fanout(p):
                                placed = True
                                break
                    if placed:
                        break
            print('island at %.1f,%.1f: %s' % (pcbnew.ToMM(bb.GetLeft()) - OX, pcbnew.ToMM(bb.GetTop()) - OY,
                                               'via added' if placed else 'NO ROOM'))
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(PCB, b)


if __name__ == '__main__':
    main()
