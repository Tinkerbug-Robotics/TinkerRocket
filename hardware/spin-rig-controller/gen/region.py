"""Lists the copper (tracks, vias, pads) in a board window, in board mm (origin top-left).

Run with KiCad's Python:  .../python3 region.py x0 y0 x1 y1 [layer]
"""
import os
import sys

import pcbnew

T = pcbnew.ToMM
b = pcbnew.LoadBoard(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'spin-rig-controller.kicad_pcb'))
x0, y0, x1, y1 = map(float, sys.argv[1:5])
lay = sys.argv[5] if len(sys.argv) > 5 else None
def inside(x, y):
    return x0 <= x <= x1 and y0 <= y <= y1
def P(v):
    return (round(T(v.x) - 100, 2), round(T(v.y) - 100, 2))
for t in b.GetTracks():
    s, e = P(t.GetStart()), P(t.GetEnd())
    if not (inside(*s) or inside(*e)):
        continue
    n = t.GetNetname().split('/')[-1]
    if t.GetClass() == 'PCB_VIA':
        print('via  %-10s %s d%.2f' % (n, s, T(t.GetWidth(pcbnew.F_Cu))))
    elif lay is None or t.GetLayerName() == lay:
        print('trk  %-10s %-5s w%.2f %s -> %s' % (n, t.GetLayerName(), T(t.GetWidth()), s, e))
for fp in b.GetFootprints():
    for p in fp.Pads():
        c = P(p.GetPosition())
        if inside(*c):
            print('pad  %-10s %s.%s %s %.2fx%.2f' % (p.GetNetname().split('/')[-1], fp.GetReference(), p.GetNumber(), c, T(p.GetBoundingBox().GetWidth()), T(p.GetBoundingBox().GetHeight())))
