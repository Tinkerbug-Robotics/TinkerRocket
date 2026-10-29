"""Builds px100-test-board.kicad_pcb from the schematic's netlist: every footprint linked to its symbol, the
carrier's board setup on JLC's default 4-layer stackup, the outline and holes, and a rough floorplan (key parts
fixed, the small parts dropped next to what they connect to). No tracks, vias or pours: the layout is the owner's.

Run with KiCad's Python, after exporting the netlist:
  kicad-cli sch export netlist --format kicadsexpr -o gen/out/px100-test-board.net px100-test-board.kicad_sch
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 make_pcb.py

It writes a new board each time: don't rerun it over a board that has been edited by hand.
"""
import math
import os
import re
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kilib  # noqa: E402

PROJ = kilib.PROJ
NAME = 'px100-test-board'
PCB = os.path.join(PROJ, NAME + '.kicad_pcb')
NET = os.path.join(HERE, 'out', NAME + '.net')
CARRIER = os.path.join(PROJ, '..', 'gnss-px100-18mm-hv', 'gnss-px100-18mm-hv.kicad_pcb')
KFP = '/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints'

W, H = 75.0, 52.0
OX, OY = 100.0, 100.0          # board origin in KiCad coordinates (mm)
SH = (21.5, 22.0)               # shield (SH1) centre; the PX100 sits in the middle of it
SH_IN = 7.4                     # half-width of the space inside the frame's pad ring

# ref: (x, y, rotation degrees CCW)   board coordinates, y down, all on the top side
#
# Left: the edge SMA, then the bias tee, then the shield with the PX100 (turned so its RF pins, 37-40, face the
# SMA). Middle: the PX100 probe header and the 1 k links to the ESP32. Right: the ESP32, the CP2102N, the USB mux
# and its switch beside the USB-C on the right edge. Bottom edge: the buttons and the ESP32's spare-pin header.
FIXED = {
    'J1': (3.35, SH[1], 0),            # pad 3.35 mm in from the edge, as the carrier's J1
    'SH1': SH + (0,), 'U1': SH + (90,),
    'J301': (36.0, 8.6, 0),
    'U201': (49.0, 22.0, 0), 'U301': (49.0, 39.0, 0),
    'J101': (W - 6.5, 22.0, 90),       # USB-C mouth on the right edge (the spin-rig's J201 offset)
    'U101': (61.5, 17.5, 0), 'S101': (59.0, 4.6, 90),
    'U103': (62.0, 34.0, 0), 'U104': (28.0, 40.5, 0),
    'J201': (26.0, 48.6, 90),
    'SW201': (55.8, 45.6, 0), 'SW202': (64.0, 45.6, 0),
    'SW301': (11.5, 46.0, 0), 'SW302': (19.8, 46.0, 0),
    'H1': (3.5, 3.5, 0), 'H2': (W - 3.5, 3.5, 0), 'H3': (3.5, H - 3.5, 0), 'H4': (W - 3.5, H - 3.5, 0),
}

# board texts (the owner's rule hides part labels, so the controls and headers say what they are): kept free of parts
# three characters at most in the lower row: its names run down towards the can (N = a reserved "NC" pin)
J301_NAMES = {1: 'N8', 3: 'IO0', 5: 'IO6', 7: 'N14', 9: 'IO9', 11: 'RST', 13: 'RXD', 15: 'TXD', 17: 'IO11',
              19: 'GND', 2: 'IO10', 4: 'IO13', 6: 'IO14', 8: 'IO15', 10: 'N29', 12: 'BOOT', 14: 'PPS', 16: 'IO2',
              18: 'N36', 20: 'GND'}
J201_NAMES = ['GND', '3V3', 'TX0', 'RX0', '33', '34', '35', '36', '47', '48']
BUTTONS = {'SW301': 'PX RST', 'SW302': 'PX BOOT', 'SW201': 'ESP BOOT', 'SW202': 'ESP RST'}
TEXT_KEEPOUT = [(30.6, 6.8, 34.0, 33.3), (40.5, 6.8, 43.8, 33.3), (24.0, 45.4, 52.0, 47.2)]


# parts that go by a particular pad (otherwise: the nearest pad they share a signal net with)
ANCHOR = {
    'D6': ('J1', '1'), 'L1': ('J1', '1'), 'C26': ('J1', '1'), 'C20': ('J1', '1'), 'R2': ('J1', '1'),
    'R3': ('J1', '1'),
    'C101': ('J101', 'A4/B9'), 'CR101': ('J101', 'A6'), 'R101': ('J101', 'A5'), 'R102': ('J101', 'B5'),
    'R103': ('U101', '4'), 'R104': ('U101', '11'), 'R105': ('U101', '12'), 'C102': ('U101', '12'),
    'R109': ('U101', '4'),
    'C103': ('U103', '1'), 'C104': ('U103', '5'), 'C105': ('U103', '5'), 'R106': ('U103', '5'), 'D101': ('U103', '5'),
    'C106': ('U104', '7'), 'C107': ('U104', '8'), 'C108': ('U104', '6'), 'R107': ('U104', '5'),
    'C109': ('U104', '1'), 'FL101': ('U104', '1'), 'C110': ('U104', '1'), 'C111': ('U104', '1'),
    'R108': ('U104', '1'), 'D102': ('U104', '1'),
    'C204': ('U201', '2'), 'C205': ('U201', '3'), 'C206': ('U201', '20'), 'C208': ('U201', '46'),
    'C209': ('U201', '55'), 'C210': ('U201', '56'), 'C211': ('U201', '29'), 'C207': ('U201', '29'),
    'R201': ('U201', '4'), 'C203': ('U201', '4'), 'R202': ('U201', '5'), 'R203': ('U201', '52'),
    'R204': ('U201', '26'), 'R205': ('U201', '25'), 'R206': ('U201', '47'), 'D201': ('U201', '47'),
    'C301': ('U301', '6'), 'C302': ('U301', '6'), 'C303': ('U301', '5'), 'C304': ('U301', '5'),
    'R301': ('U301', '8'), 'R302': ('U301', '8'), 'R303': ('U301', '9'), 'D301': ('U301', '9'),
    'R304': ('U301', '21'), 'R305': ('U301', '20'), 'R306': ('U301', '14'), 'D302': ('U301', '14'),
    'R307': ('U301', '13'), 'D303': ('U301', '13'),
    'Q301': ('U301', '19'), 'Q302': ('U301', '23'), 'R308': ('U301', '23'), 'R309': ('U301', '19'),
    'R330': ('U1', '36'), 'C305': ('U1', '14'),
    'R331': ('U1', '9'), 'R332': ('U1', '9'), 'C306': ('U1', '9'), 'TP302': ('U1', '9'),
    'R333': ('J1', '1'), 'R334': ('J1', '1'), 'C307': ('J1', '1'), 'TP301': ('J1', '1'),
    'R1': ('U1', '31'), 'D1': ('U1', '31'), 'R12': ('U1', '17'), 'C56': ('U1', '17'), 'R13': ('U1', '30'),
    'R14': ('U1', '18'), 'R15': ('U1', '19'), 'C43': ('U1', '6'), 'C44': ('U1', '7'),
    'Y2': ('U1', '10'), 'R11': ('U1', '20'),
    'TP101': ('J101', 'A4/B9'), 'TP102': ('U103', '5'), 'TP103': ('U104', '1'), 'TP104': ('J101', 'S1'),
    'TP105': ('U104', '4'),
}
# the RF chain, the TCXO, the RTC crystal and U1's core parts go under the shield; everything else stays out
# (Y2 and R11 sat outside the carrier's shield too)
OUTSIDE_RECEIVER = {'J1', 'D6', 'L1', 'C20', 'R2', 'R3', 'C26', 'R1', 'D1', 'R12', 'R13', 'R14', 'R15', 'C56', 'SH1',
                    'Y2', 'R11'}


def mm(v):
    return pcbnew.FromMM(v)


def pt(x, y):
    return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


def load_netlist():
    root = kilib.parse(open(NET).read())[0]
    comps = {}
    for c in kilib.find(kilib.get(root, 'components'), 'comp'):
        ref = str(kilib.get(c, 'ref')[1])
        d = {'value': str(kilib.get(c, 'value')[1]), 'footprint': str(kilib.get(c, 'footprint')[1])}
        flds = kilib.get(c, 'fields')
        d['fields'] = {}
        if flds:
            for f in kilib.find(flds, 'field'):
                d['fields'][str(kilib.get(f, 'name')[1])] = str(f[-1]) if isinstance(f[-1], str) else ''
        sp = kilib.get(c, 'sheetpath')
        d['sheetnames'] = str(kilib.get(sp, 'names')[1])
        d['sheetpath'] = str(kilib.get(sp, 'tstamps')[1])
        d['tstamp'] = str(kilib.get(c, 'tstamps')[1])
        d['props'] = {str(kilib.get(p, 'name')[1]): str(kilib.get(p, 'value')[1]) if kilib.get(p, 'value') else ''
                      for p in kilib.find(c, 'property')}
        comps[ref] = d
    nets = {}
    for n in kilib.find(kilib.get(root, 'nets'), 'net'):
        nm = str(kilib.get(n, 'name')[1])
        for nd in kilib.find(n, 'node'):
            nets.setdefault(nm, []).append((str(kilib.get(nd, 'ref')[1]), str(kilib.get(nd, 'pin')[1])))
    return comps, nets


def fp_load(fpid):
    lib, name = fpid.split(':')
    path = os.path.join(PROJ, '..', 'footprints', 'Footprints.pretty') if lib == 'Footprints' else \
        os.path.join(KFP, lib + '.pretty')
    fp = pcbnew.FootprintLoad(path, name)
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def courtyard(fp):
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    b = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False)
    return (pcbnew.ToMM(b.GetLeft()) - OX, pcbnew.ToMM(b.GetTop()) - OY,
            pcbnew.ToMM(b.GetRight()) - OX, pcbnew.ToMM(b.GetBottom()) - OY)


def overlaps(a, b, m):
    return not (a[2] + m <= b[0] or b[2] + m <= a[0] or a[3] + m <= b[1] or b[3] + m <= a[1])


def place(fp, x, y, rot):
    fp.SetPosition(pt(x, y))
    fp.SetOrientationDegrees(rot)


def hide_labels(fp):
    """The owner's rule: no part labels on silk or fab. Fields are hidden; KiCad won't save a hide flag on the
    library's ${REFERENCE} fab text, so that one is deleted."""
    fp.Reference().SetVisible(False)
    fp.Value().SetVisible(False)
    for it in list(fp.GraphicalItems()):
        if isinstance(it, pcbnew.PCB_TEXT) and it.GetText() in ('${REFERENCE}', '${VALUE}'):
            fp.Remove(it)


def build():
    comps, nets = load_netlist()
    board = pcbnew.NewBoard(PCB)
    board.SetCopperLayerCount(4)
    board.GetDesignSettings().SetBoardThickness(mm(1.6))
    board.GetTitleBlock().SetTitle('PX100 Test Board')
    board.GetTitleBlock().SetRevision('V1')     # the silkscreen's ${REVISION}; tools/plot_gerbers.sh reads it here
    board.GetTitleBlock().SetCompany('TinkerRocket')

    netinfo = {}
    for nm in nets:
        ni = pcbnew.NETINFO_ITEM(board, nm)
        board.Add(ni)
        netinfo[nm] = ni
    padnet = {(ref, pin): nm for nm, nodes in nets.items() for ref, pin in nodes}

    sheetfile = {'/USB-C, USB switch and power/': 'power.kicad_sch', '/ESP32-S3/': 'esp32.kicad_sch',
                 '/PX100 USB bridge and I/O/': 'pxio.kicad_sch', '/RF chain and PX100/': 'receiver.kicad_sch'}
    fps = {}
    for ref, c in sorted(comps.items()):
        fp = fp_load(c['footprint'])
        fp.SetReference(ref)
        fp.SetValue(c['value'])
        for k in sorted(set(c['fields']) | {'MPN', 'Mfr', 'LCSC', 'Description'}):
            if k in ('Reference', 'Value', 'Footprint'):
                continue
            v = c['fields'].get(k, '')
            fp.SetField(k, v)
            f = fp.GetField(k)
            if f:
                f.SetVisible(False)
                f.SetLayer(pcbnew.F_Fab)
        fp.SetPath(pcbnew.KIID_PATH(c['sheetpath'] + c['tstamp']))
        fp.SetSheetname(c['sheetnames'])
        fp.SetSheetfile(sheetfile.get(c['sheetnames'], ''))
        if 'dnp' in c['props']:
            fp.SetDNP(True)
        if ref.startswith(('H', 'TP')):
            fp.SetExcludedFromBOM(True)
            fp.SetExcludedFromPosFiles(True)
        for pad in fp.Pads():
            nm = padnet.get((ref, pad.GetNumber()))
            if nm:
                pad.SetNet(netinfo[nm])
        hide_labels(fp)
        board.Add(fp)
        fps[ref] = fp
    missing = [r for r in fps if not any(p.GetNumber() for p in fps[r].Pads()) and not r.startswith('H')]
    assert not missing, missing

    occ = []
    for ref, (x, y, rot) in FIXED.items():
        place(fps[ref], x, y, rot)
        if ref != 'SH1':
            occ.append((ref, courtyard(fps[ref])))
    # the shield's pad ring as four blockers (its courtyard is one square that would swallow the inside)
    sx, sy = SH
    o, i = 8.95, SH_IN
    for b in ((sx - o, sy - o, sx + o, sy - i), (sx - o, sy + i, sx + o, sy + o),
              (sx - o, sy - o, sx - i, sy + o), (sx + i, sy - o, sx + o, sy + o)):
        occ.append(('SH1-ring', b))
    for b in TEXT_KEEPOUT:
        occ.append(('text', b))
    for ref, b in occ:
        if b[0] < -0.01 and ref != 'J1' or b[1] < -0.01 or b[2] > W + 0.01 and ref != 'J101' or b[3] > H + 0.01:
            print('outside board:', ref, b)
    for k, (r1, b1) in enumerate(occ):
        for r2, b2 in occ[k + 1:]:
            if overlaps(b1, b2, 0.0) and 'SH1-ring' not in (r1, r2):
                print('overlap:', r1, r2)

    def pad_pos(ref, num):
        for p in fps[ref].Pads():
            if p.GetNumber() == num:
                q = p.GetPosition()
                return pcbnew.ToMM(q.x) - OX, pcbnew.ToMM(q.y) - OY
        raise KeyError((ref, num))

    placed = set(FIXED)

    def anchor_of(ref):
        if ref in ANCHOR:
            return pad_pos(*ANCHOR[ref])
        best = None
        for p in fps[ref].Pads():
            nm = padnet.get((ref, p.GetNumber()))
            if not nm or nm in ('GND', '+3V3', '+3V3_MCU', 'VBUS') or nm.startswith('unconnected'):
                continue
            for r2, pin in nets[nm]:
                if r2 == ref or r2 not in placed:
                    continue
                big = len(list(fps[r2].Pads()))
                if best is None or big > best[0]:
                    best = (big, pad_pos(r2, pin))
        return best[1] if best else None

    rings = []
    for big, m in (('U1', 0.3), ('U201', 1.0), ('U301', 0.6), ('U101', 0.5)):
        c = courtyard(fps[big])
        rings.append((c[0] - m, c[1] - m, c[2] + m, c[3] + m))

    def receiver_inside(ref):
        return comps[ref]['sheetnames'] == '/RF chain and PX100/' and ref not in OUTSIDE_RECEIVER

    todo = sorted((r for r in fps if r not in FIXED), key=lambda r: (not receiver_inside(r), r not in ANCHOR, r))
    for _ in range(6):
        left = []
        for ref in todo:
            if ref in ANCHOR and ANCHOR[ref][0] not in placed:
                left.append(ref)
                continue
            a = anchor_of(ref)
            if a is None:
                left.append(ref)
                continue
            inside = receiver_inside(ref)
            gap = 0.15 if inside else 0.35
            fp = fps[ref]
            done = False
            for rad in [k * 0.25 for k in range(0, 160)]:
                n = max(1, int(2 * math.pi * rad / 0.25))
                for k in range(n):
                    th = 2 * math.pi * k / n
                    x, y = round((a[0] + rad * math.cos(th)) * 8) / 8, round((a[1] + rad * math.sin(th)) * 8) / 8
                    for rot in (0, 90):
                        place(fp, x, y, rot)
                        b = courtyard(fp)
                        if b[0] < 0.4 or b[1] < 0.4 or b[2] > W - 0.4 or b[3] > H - 0.4:
                            continue
                        in_sh = b[0] > sx - SH_IN and b[2] < sx + SH_IN and b[1] > sy - SH_IN and b[3] < sy + SH_IN
                        near_sh = overlaps(b, (sx - 8.95, sy - 8.95, sx + 8.95, sy + 8.95), 0.3)
                        if inside != in_sh or (not inside and near_sh):
                            continue
                        if any(overlaps(b, q, gap) for _, q in occ):
                            continue
                        if any(overlaps(b, q, 0.0) for q in rings):
                            continue
                        occ.append((ref, b))
                        placed.add(ref)
                        done = True
                        break
                    if done:
                        break
                if done:
                    break
            if not done:
                print('could not place', ref)
                place(fp, W + 10, H / 2, 0)   # parked beside the board for the owner
        todo = left
        if not todo:
            break
    if todo:
        print('no anchor:', todo)

    pts = [(0, 0), (W, 0), (W, H), (0, H)]
    for k in range(4):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pt(*pts[k]))
        s.SetEnd(pt(*pts[(k + 1) % 4]))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.05))
        board.Add(s)
    def silk(text, x, y, just=0, size=0.8, th=0.15):
        t = pcbnew.PCB_TEXT(board)
        t.SetText(text)
        t.SetPosition(pt(x, y))
        t.SetLayer(pcbnew.F_SilkS)
        t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
        t.SetTextThickness(mm(th))
        t.SetHorizJustify({-1: pcbnew.GR_TEXT_H_ALIGN_RIGHT, 0: pcbnew.GR_TEXT_H_ALIGN_CENTER,
                           1: pcbnew.GR_TEXT_H_ALIGN_LEFT}[just])
        board.Add(t)

    silk('PX100 TEST ${REVISION}', 42.0, 5.0, 0, 1.2, 0.18)
    for n, name in J301_NAMES.items():
        x, y = pad_pos('J301', str(n))
        silk(name, x - 1.4, y, -1) if n % 2 else silk(name, x + 1.4, y, 1)
    for k, name in enumerate(J201_NAMES):
        x, y = pad_pos('J201', str(k + 1))
        silk(name, x, y - 2.1, 0)
    for ref, name in BUTTONS.items():
        c = courtyard(fps[ref])
        silk(name, (c[0] + c[2]) / 2, c[3] + 0.9, 0)
    # S101: SEL0 to ground (port 2, the ESP32) at its pin-7 end, as the Beetle's "F" end; open at the pin-5 end
    e7 = [(a + b) / 2 for a, b in zip(pad_pos('S101', '3'), pad_pos('S101', '7'))]
    e5 = [(a + b) / 2 for a, b in zip(pad_pos('S101', '1'), pad_pos('S101', '5'))]
    c = [(a + b) / 2 for a, b in zip(e7, e5)]
    for e, name in ((e7, 'ESP'), (e5, 'PX100')):
        d = [a - b for a, b in zip(e, c)]
        L = math.hypot(*d)
        x, y = e[0] + d[0] / L * 2.6, e[1] + d[1] / L * 2.6
        silk(name, x, y, 1 if d[0] > 0.1 else (-1 if d[0] < -0.1 else 0))
    silk('USB', c[0], c[1] + 3.3, 0)
    pcbnew.SaveBoard(PCB, board, True)
    print('placed', len(placed), 'of', len(fps))


def carrier_setup():
    """Swap the new board's (setup ...) for the carrier's: tented, filled and capped vias (U1's pad needs them),
    plot settings; the stackup is JLC's default JLC04161H-7628, what the carrier's V1 was actually built on."""
    def block(s, head):
        i = s.index(head)
        d = 0
        for j in range(i, len(s)):
            if s[j] == '(':
                d += 1
            elif s[j] == ')':
                d -= 1
                if d == 0:
                    return i, j + 1
    car = open(CARRIER).read()
    a, b = block(car, '(setup')
    setup = car[a:b]
    setup = setup.replace('(thickness 0.0994)\n\t\t\t\t(material "FR4 3313")\n\t\t\t\t(epsilon_r 4.1)',
                          '(thickness 0.2104)\n\t\t\t\t(material "FR4 7628")\n\t\t\t\t(epsilon_r 4.4)')
    setup = setup.replace('(thickness 1.265)', '(thickness 1.065)')
    setup = setup.replace('(copper_finish "ENIG")', '(copper_finish "ENIG")')
    setup = re.sub(r'\(outputdirectory "[^"]*"\)', '(outputdirectory "gerbers")', setup)
    assert setup.count('FR4 7628') == 2 and '1.065' in setup
    s = open(PCB).read()
    a, b = block(s, '(setup')
    s = s[:a] + setup + s[b:]
    open(PCB, 'w').write(s)


if __name__ == '__main__':
    build()
    carrier_setup()
    b = pcbnew.LoadBoard(PCB)       # it must still load
    print('reloaded:', len(b.GetFootprints()), 'footprints, thickness',
          pcbnew.ToMM(b.GetDesignSettings().GetBoardThickness()))
