"""Builds gvlb258-patch-test.kicad_pcb: a 2-layer board as wide as the Tinker-Beetle (26.83 mm, 1.0 mm
corner radius), the GVLB258.A on the front at one end, the SMA jack on the back at the other, pointing
away from the patch, and a straight 50 ohm grounded coplanar line (RF_FEED, B.Cu) from the patch pin to
the jack's centre pin. Both copper layers are ground pours; vias 0.4/0.3 stitch them along the line and
over the board.

Run with KiCad's Python, from an empty scratch folder that holds a copy of the project file:
    KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3
    $KPY make_pcb.py <scratch dir>          # writes <scratch>/gvlb258-patch-test.kicad_pcb
then copy only the .kicad_pcb into the board folder (pcbnew's Save rewrites the .kicad_pro beside it).
"""
import math
import os
import shutil
import subprocess
import sys
import uuid

import pcbnew
from pcbnew import FromMM, VECTOR2I

NAME = 'gvlb258-patch-test'
HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
FPLIB = os.path.normpath(os.path.join(PROJ, '..', 'footprints', 'Footprints.pretty'))
KICAD_FP = '/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints'
SCH_NS = uuid.UUID('9a4e2c71-5b3d-4f8a-b6e0-1c7d3f2a9e54')     # make_sch.py's namespace
SCH_FILE = NAME + '.kicad_sch'

# ---- geometry (mm, board frame: origin at the top-left corner, y down) ----
X0, Y0 = 100.0, 100.0          # where the board frame sits on the page
W = 26.83                      # Tinker-Beetle width
R_CORNER = 1.0                 # Beetle corner radius
CX = W / 2
PATCH_C = (CX, 13.415)         # 25 mm patch: 0.915 mm margin on three sides
FEED = (CX, PATCH_C[1] + 2.5)  # footprint pin 1 sits 2.5 mm below the patch centre
SMA = (CX, 30.9)               # posts' pads end 1.0 mm past the patch ceramic (25.915)
L = 35.5                       # posts' pads end 0.75 mm before the end edge
HOLES = ((2.40, L - 2.32), (W - 2.40, L - 2.32))   # Beetle hole offsets from the side and end edges
RF_W, RF_GAP = 1.0, 0.25
VIA_D, VIA_DRILL = 0.4, 0.3


def P(x, y):
    return VECTOR2I(FromMM(X0 + x), FromMM(Y0 + y))


def sym_uuid(ref):
    return str(uuid.uuid5(SCH_NS, 'sym/' + ref))


def edge(b):
    r = R_CORNER
    segs = [((r, 0), (W - r, 0)), ((W, r), (W, L - r)), ((W - r, L), (r, L)), ((0, L - r), (0, r))]
    for (x1, y1), (x2, y2) in segs:
        s = pcbnew.PCB_SHAPE(b, pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(P(x1, y1)); s.SetEnd(P(x2, y2))
        s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(FromMM(0.05)); b.Add(s)
    k = r * (1 - math.sqrt(0.5))
    arcs = [((0, r), (k, k), (r, 0)), ((W - r, 0), (W - k, k), (W, r)),
            ((W, L - r), (W - k, L - k), (W - r, L)), ((r, L), (k, L - k), (0, L - r))]
    for a, m, e in arcs:
        s = pcbnew.PCB_SHAPE(b, pcbnew.SHAPE_T_ARC)
        s.SetArcGeometry(P(*a), P(*m), P(*e))
        s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(FromMM(0.05)); b.Add(s)


def load_fp(b, lib, name, ref, value, fields, xy, back=False):
    f = pcbnew.FootprintLoad(lib, name)
    if f is None:
        sys.exit('footprint not found: %s / %s' % (lib, name))
    b.Add(f)
    f.SetReference(ref)
    f.SetValue(value)
    for k, v in fields.items():
        f.SetField(k, v)
    f.SetPosition(P(*xy))
    if back:
        f.Flip(f.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
    f.SetPath(pcbnew.KIID_PATH('/' + sym_uuid(ref)))
    f.SetSheetname('/')
    f.SetSheetfile(SCH_FILE)
    f.SetFPID(pcbnew.LIB_ID('Footprints' if lib == FPLIB else 'MountingHole', name))
    # owner rule: no part labels on the silk or fab layers (fields added by SetField show by default)
    for fld in f.GetFields():
        fld.SetVisible(False)
    for item in list(f.GraphicalItems()):
        if isinstance(item, pcbnew.PCB_TEXT) and item.GetText() == '${REFERENCE}':
            f.Remove(item)
    return f


def via(b, net, x, y):
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(P(x, y))
    v.SetViaType(pcbnew.VIATYPE_THROUGH)
    try:
        v.SetWidth(FromMM(VIA_D))
    except TypeError:
        v.SetWidth(pcbnew.F_Cu, FromMM(VIA_D))
    v.SetDrill(FromMM(VIA_DRILL))
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    v.SetNet(net)
    b.Add(v)
    return v


def zone(b, net, layer, name):
    z = pcbnew.ZONE(b)
    z.SetLayer(layer)
    z.SetNet(net)
    z.SetZoneName(name)
    o = z.Outline()
    o.NewOutline()
    for x, y in ((0, 0), (W, 0), (W, L), (0, L)):
        o.Append(FromMM(X0 + x), FromMM(Y0 + y))
    z.SetLocalClearance(FromMM(0.25))
    z.SetMinThickness(FromMM(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(FromMM(0.4))
    z.SetThermalReliefSpokeWidth(FromMM(0.6))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    b.Add(z)
    return z


def text(b, s, x, y, layer, angle=0, size=1.0, mirror=False):
    t = pcbnew.PCB_TEXT(b)
    t.SetText(s)
    t.SetLayer(layer)
    t.SetPosition(P(x, y))
    t.SetTextSize(VECTOR2I(FromMM(size), FromMM(size)))
    t.SetTextThickness(FromMM(0.15))
    t.SetTextAngleDegrees(angle)
    t.SetMirrored(mirror)
    b.Add(t)
    return t


def stitch_spots():
    """Ground vias: two rows beside the line, then a grid over the rest of the board. Each spot keeps
    clear of the line, the patch pin's no-via circles (3.0 / 3.5 mm dia), the jack's pads, the holes and
    the edge."""
    out = []
    for dx in (-1.25, 1.25):                       # beside the line: gap 0.25 + via ring + 0.25
        y = 18.4
        while y <= 26.6:
            out.append((CX + dx, round(y, 3)))
            y += 1.5
    pitch = 3.6
    xs = [1.0 + i * pitch for i in range(int((W - 2.0) / pitch) + 1)]
    xs = [x + (W - 2.0 - (xs[-1] - 1.0)) / 2 for x in xs]
    ys = [1.0 + i * pitch for i in range(int((L - 2.0) / pitch) + 1)]
    for x in xs:
        for y in ys:
            if abs(x - CX) < 1.9:                               # line and its via rows
                continue
            if math.hypot(x - FEED[0], y - FEED[1]) < 2.6:    # patch pin keep-outs
                continue
            if abs(x - SMA[0]) < 4.4 and abs(y - SMA[1]) < 4.4:  # jack pads
                continue
            if any(math.hypot(x - hx, y - hy) < 2.6 for hx, hy in HOLES):
                continue
            if min(x, y, W - x, L - y) < 0.8:
                continue
            out.append((round(x, 3), round(y, 3)))
    return out


def main(scratch):
    path = os.path.join(scratch, NAME + '.kicad_pcb')
    b = pcbnew.LoadBoard(path)
    b.SetCopperLayerCount(2)
    tb = b.GetTitleBlock()
    tb.SetTitle('GVLB258.A patch test board')
    tb.SetDate('2026-11-27')
    tb.SetRevision('V1')
    tb.SetCompany('TinkerRocket')

    nets = {}
    for n in ('GND', '/RF_FEED'):
        ni = pcbnew.NETINFO_ITEM(b, n)
        b.Add(ni)
        nets[n] = ni
    gnd, rf = nets['GND'], nets['/RF_FEED']

    edge(b)

    ae = load_fp(b, FPLIB, 'ANT_GVLB258.A_TAOGLAS', 'AE1', 'GVLB258.A', {
        'Datasheet': 'https://www.taoglas.com/datasheets/GVLB258.A.pdf',
        'Description': 'GNSS L1/L5 single-feed stacked ceramic patch antenna (GPS L1/L5, Galileo E1/E5a, BeiDou '
                       'B1/B2a, GLONASS G1, NavIC L5), 25 x 25 x 8.12 mm, feed pin 2.5 mm off centre',
        'MPN': 'GVLB258.A', 'Mfr': 'Taoglas'}, PATCH_C)
    j1 = load_fp(b, FPLIB, 'SMA_GradConn_RFPC-SMA28-F_Vertical', 'J1', 'RFPC-SMA28-F', {
        'Datasheet': 'https://www.digikey.com/en/products/detail/gct/RFPC-SMA28-F/22162141',
        'Description': 'SMA jack (female), straight, through-hole, 50 ohm; fitted on the back, pointing away '
                       'from the patch',
        'MPN': 'RFPC-SMA28-F', 'Mfr': 'GCT (GradConn)'}, SMA, back=True)
    holes = []
    for i, xy in enumerate(HOLES, 1):
        h = load_fp(b, os.path.join(KICAD_FP, 'MountingHole.pretty'), 'MountingHole_2.2mm_M2_DIN965_Pad_TopBottom',
                    'H%d' % i, 'MountingHole_Pad', {
                        'Description': 'M2 mounting hole at the SMA end, 2.40 mm from the side edge as on the '
                                       'Tinker-Beetle; pad to GND'}, xy)
        h.SetAttributes(h.GetAttributes() | pcbnew.FP_EXCLUDE_FROM_BOM)
        holes.append(h)

    for p in ae.Pads():
        p.SetNet(rf)
    for p in j1.Pads():
        p.SetNet(rf if p.GetNumber() == '1' else gnd)
        if p.GetNumber() == '1':
            p.SetLocalClearance(FromMM(0.5))     # antipad around the centre pin on both pours
    for h in holes:
        for p in h.Pads():
            p.SetNet(gnd)

    t = pcbnew.PCB_TRACK(b)
    t.SetStart(P(*FEED)); t.SetEnd(P(*SMA))
    t.SetWidth(FromMM(RF_W)); t.SetLayer(pcbnew.B_Cu); t.SetNet(rf)
    b.Add(t)

    for x, y in stitch_spots():
        via(b, gnd, x, y)

    zone(b, gnd, pcbnew.F_Cu, 'GND top (patch ground plane)')
    zone(b, gnd, pcbnew.B_Cu, 'GND bottom (coplanar ground)')

    # board texts, back silk (the front is the patch): name + revision down the left, a use note on the right
    text(b, 'GVLB258.A PATCH TEST ${REVISION}', 3.2, 13.4, pcbnew.B_SilkS, angle=90, size=1.0, mirror=True)
    text(b, 'PX100 CARRIER: JP1 OPEN', W - 3.2, 13.4, pcbnew.B_SilkS, angle=90, size=1.0, mirror=True)

    filler = pcbnew.ZONE_FILLER(b)
    filler.Fill(b.Zones())
    b.Save(path)
    print('saved', path, 'vias', len([x for x in b.GetTracks() if isinstance(x, pcbnew.PCB_VIA)]))


if __name__ == '__main__':
    main(sys.argv[1])
