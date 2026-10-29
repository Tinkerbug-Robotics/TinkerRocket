"""Autorouting around the hand-laid tracks from layout.py.

  KiCad python:  route.py prep     lock the hand routes, In1 = the GND plane, router keep-outs, export gen/out/*.dsn
                 route.py finish   import gen/out/*.ses, drop the keep-outs, pour GND on every layer, fill

Freerouting runs between the two (route.sh). The keep-outs only steer the autorouter: no new F.Cu inside the can (it is
the RF ground there) or beside the RF line, nothing on In1. They are removed again by finish. GND is left out of the
autorouting altogether: the pours and stitching vias (stitch.py) carry it.
"""
import os
import re
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import layout  # noqa: E402

PCB = layout.PCB
DSN = os.path.join(HERE, 'out', 'px100-test-board.dsn')
SES = os.path.join(HERE, 'out', 'px100-test-board.ses')
OX, OY, W, H = layout.OX, layout.OY, layout.W, layout.H
KEEP = 'router keep-out: '
mm = pcbnew.FromMM


def poly(board, pts, layer_ids, name):
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True)
    ls = pcbnew.LSET()
    for l in layer_ids:
        ls.AddLayer(l)
    z.SetLayerSet(ls)
    z.SetDoNotAllowTracks(True)
    z.SetDoNotAllowVias(False)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowFootprints(False)
    z.SetDoNotAllowZoneFills(False)
    z.SetZoneName(KEEP + name)
    o = z.Outline()
    o.NewOutline()
    for x, y in pts:
        o.Append(mm(OX + x), mm(OY + y))
    board.Add(z)


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def can_inside():
    """The can's inside, in board coordinates (to the walls' inner edges)."""
    ux, uy = layout.U1_AT
    cx, cy = ux + layout.SH_OFF[0], uy + layout.SH_OFF[1]
    return rect(cx - 7.6, cy - 7.6, cx + 7.6, cy + 7.6)


def gnd_zone(board, layer, prio=0):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(board.FindNet('GND'))
    o = z.Outline()
    o.NewOutline()
    for x, y in rect(0.3, 0.3, W - 0.3, H - 0.3):
        o.Append(mm(OX + x), mm(OY + y))
    z.SetAssignedPriority(prio)
    # the carrier's pours: SMD pads joined solid (the RF shunts' grounds), reliefs only on through-hole pads
    z.SetLocalClearance(mm(0.127))
    z.SetMinThickness(mm(0.127))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THT_THERMAL)
    z.SetThermalReliefGap(mm(0.5))
    z.SetThermalReliefSpokeWidth(mm(0.5))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetZoneName('GND ' + board.GetLayerName(layer))
    board.Add(z)
    return z


def prep():
    b = pcbnew.LoadBoard(PCB)
    for t in b.GetTracks():
        t.SetLocked(True)
    b.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_POWER)
    for z in list(b.Zones()):
        b.Remove(z)
    gnd_zone(b, pcbnew.In1_Cu)
    poly(b, can_inside(), [pcbnew.F_Cu], 'inside the can')
    jx, jy = layout.J1_AT
    poly(b, rect(jx + 1.8, jy - 0.9, 13.2, jy + 0.9), [pcbnew.F_Cu], 'RF_IN, J1 to the can')
    pcbnew.SaveBoard(PCB, b, True)
    ok = pcbnew.ExportSpecctraDSN(b, DSN)
    # GND is not autorouted: its pads join the pours (and the In1 plane through the stitching vias, stitch.py)
    d = open(DSN).read()
    d, n = re.subn(r'\(net GND\s+\(pins[^)]*\)', '(net GND\n      (pins)', d)
    assert n == 1
    open(DSN, 'w').write(d)
    print('DSN', ok, DSN)


def finish(ses=SES):
    b = pcbnew.LoadBoard(PCB)
    ok = pcbnew.ImportSpecctraSES(b, ses)
    print('SES', ok, os.path.basename(ses))
    old = [z for z in b.Zones() if not z.GetIsRuleArea() or z.GetZoneName().startswith(KEEP)]
    # new pours first: after a BOARD.Remove(ZONE) the SWIG wrapper hands back untyped SHAPE_POLY_SETs
    for layer in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
        gnd_zone(b, layer)
    for z in old:
        b.Remove(z)
    b.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_SIGNAL)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(PCB, b, True)


if __name__ == '__main__':
    {'prep': prep, 'finish': lambda: finish(*sys.argv[2:3])}[sys.argv[1]]()
