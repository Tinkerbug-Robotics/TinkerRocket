"""Builds spin-rig-controller.kicad_pcb from the schematic's netlist: footprints, nets, placement, outline,
holes, planes and the motor-current pours, then exports a Specctra DSN for Freerouting.

Run with KiCad's Python:
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 make_pcb.py place
  ... route with Freerouting (route.sh) ...
  .../python3 make_pcb.py finish      (imports the session, adds the outer GND pours, refills)
"""
import math
import os
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kilib  # noqa: E402

PROJ = kilib.PROJ
NAME = 'spin-rig-controller'
PCB = os.path.join(PROJ, NAME + '.kicad_pcb')
NET = os.path.join(PROJ, 'gen', 'out', NAME + '.net')
DSN = os.path.join(PROJ, 'gen', 'out', NAME + '.dsn')
SES = os.path.join(PROJ, 'gen', 'out', NAME + '.ses')
KFP = '/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints'

W, H = 110.0, 80.0
OX, OY = 100.0, 100.0          # board origin in KiCad coordinates (mm)
HOLES = [(3.5, 3.5), (W - 3.5, 3.5), (3.5, H - 3.5), (W - 3.5, H - 3.5)]


def mm(v):
    return pcbnew.FromMM(v)


def pt(x, y):
    return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


# ref: (x, y, rotation degrees CCW, side)   board coordinates, y down
FIXED = {
    # power column: the XT60 on the top edge -> Q101 (tab toward it) -> VMAIN; shunt R102 -> VDRV -> J103
    'J101': (11.95, 16.65, 0, 'F'), 'Q101': (29.5, 14.0, 180, 'F'), 'C101': (38.5, 5.6, 0, 'F'),
    'C102': (38.6, 14.3, 90, 'F'), 'C103': (42.3, 14.3, 90, 'F'), 'D102': (29.0, 22.0, 0, 'F'),
    'R102': (21.0, 27.0, 180, 'F'), 'U101': (14.0, 22.0, 0, 'F'), 'J103': (8.3, 29.0, 270, 'F'),
    # brake: resistor connector, freewheel diode and switch in a tight loop, with its own bus capacitor
    'J102': (6.0, 47.0, 90, 'F'), 'Q102': (22.5, 46.0, 180, 'F'), 'D103': (14.8, 44.6, 270, 'F'),
    'C123': (21.0, 41.0, 0, 'F'),
    'U102': (33.0, 36.0, 0, 'F'), 'U103': (33.0, 43.0, 0, 'F'),
    # 5 V buck, laid out per the TPS54560B guide: input caps by VIN, the catch diode's anode against their
    # ground, its cathode and the inductor on a short switch node at pin 8
    'F101': (9.0, 53.5, 270, 'F'), 'U104': (20.0, 62.0, 0, 'F'),
    'C108': (14.6, 61.4, 90, 'F'), 'C109': (11.0, 61.4, 90, 'F'), 'C110': (7.4, 61.4, 90, 'F'),
    'D105': (17.0, 55.6, 180, 'F'), 'L101': (31.0, 56.0, 0, 'F'),
    'C115': (42.0, 53.2, 0, 'F'), 'C116': (42.0, 56.8, 0, 'F'), 'C117': (42.0, 60.4, 0, 'F'),
    'F103': (101.0, 60.0, 90, 'F'), 'F102': (88.0, 68.5, 0, 'F'),
    # connectors on the edges. The driver's 3.5 mm headers open outward with their faces 0.3 mm inside the edge:
    # control and encoder-out along the top, encoder-in down the right (pin 1 of each at its corner end)
    'J301': (75.8, 8.3, 180, 'F'), 'J305': (99.7, 8.3, 180, 'F'), 'J302': (101.7, 31.45, 90, 'F'),
    'J304': (106.3, 41.35, 90, 'F'), 'J303': (106.3, 52.64, 90, 'F'),
    'J202': (106.0, 57.5, 0, 'F'), 'J203': (3.3, 20.0, 90, 'F'),     # Qwiic beside the INA238, the other I2C part
    'J104': (90.0, 74.5, 180, 'F'), 'J201': (70.0, 74.1, 0, 'F'),
    'SW201': (50.5, 76.0, 0, 'F'), 'SW202': (59.0, 76.0, 0, 'F'),
    # logic
    'U201': (72.0, 44.0, 0, 'F'), 'U301': (60.0, 18.0, 0, 'F'), 'U302': (85.0, 18.0, 90, 'F'),
    'Y201': (68.2, 35.8, 90, 'F'), 'L201': (68.0, 38.4, 0, 'F'), 'C202': (70.3, 36.0, 90, 'F'),
    'C201': (66.0, 34.4, 180, 'F'),
    'U303': (94.5, 31.0, 90, 'F'), 'U304': (93.5, 20.5, 90, 'F'),
    'H1': HOLES[0] + (0, 'F'), 'H2': HOLES[1] + (0, 'F'), 'H3': HOLES[2] + (0, 'F'), 'H4': HOLES[3] + (0, 'F'),
    'FID1': (3.5, 13.0, 0, 'F'), 'FID2': (100.0, 66.0, 0, 'F'), 'FID3': (3.0, 58.0, 0, 'F'),
}

# small parts that sit by a particular pad (otherwise: the nearest pad they share a signal net with)
ANCHOR = {
    'C104': ('U101', '6'), 'R103': ('U101', '3'), 'C106': ('U102', '6'), 'C107': ('U103', '1'),
    'C111': ('U104', '2'), 'C112': ('U104', '1'), 'C301': ('U301', '20'), 'C304': ('U302', '16'),
    'C306': ('U303', '5'), 'C307': ('U304', '8'),
    'C204': ('U201', '2'), 'C205': ('U201', '3'), 'C206': ('U201', '20'), 'C208': ('U201', '46'),
    'C209': ('U201', '55'), 'C210': ('U201', '56'), 'C211': ('U201', '29'), 'C207': ('U201', '29'),
    'U105': ('U201', '20'), 'D107': ('U105', '1'), 'D108': ('J201', 'A4/B9'), 'C118': ('U105', '1'),
    'C119': ('U105', '5'), 'C120': ('U105', '5'), 'C212': ('J201', 'B4/A9'), 'CR201': ('J201', 'A6'),
    'R204': ('J201', 'A5'), 'R205': ('J201', 'B5'), 'R206': ('J201', 'A6'), 'R207': ('J201', 'A7'),
    'R211': ('J202', '4'), 'R212': ('J202', '5'),
    'R213': ('J203', '3'), 'R214': ('J203', '4'),
    'R119': ('C117', '1'), 'D106': ('C117', '1'),
    'R120': ('U101', '8'), 'R121': ('U101', '8'), 'C121': ('U101', '8'),
    'R122': ('C115', '1'), 'R123': ('C115', '1'), 'C122': ('C115', '1'),
    'R101': ('Q101', '1'), 'D101': ('Q101', '1'),
    'R104': ('U102', '3'), 'R105': ('U102', '3'), 'R106': ('U102', '3'), 'C105': ('U102', '3'),
    'R107': ('U102', '1'), 'R108': ('U103', '5'), 'R109': ('Q102', '4'), 'R110': ('U103', '5'),
    'D104': ('U103', '5'), 'R111': ('U103', '5'), 'R112': ('U103', '5'),
    'R208': ('SW202', '1'), 'D201': ('SW202', '1'), 'R209': ('SW202', '1'), 'D202': ('SW202', '1'),
    'R210': ('SW202', '1'), 'D203': ('SW202', '1'),
    'TP201': ('U201', '49'), 'TP202': ('U201', '50'), 'TP203': ('U105', '5'), 'TP204': ('C116', '1'),
    'TP205': ('U201', '57'), 'TP206': ('U104', '9'), 'TP306': ('J301', '2'), 'TP301': ('J301', '1'), 'TP302': ('J304', '1'),
    'TP303': ('U302', '3'), 'TP304': ('J304', '3'), 'TP305': ('U303', '4'),
    'R317': ('J303', '3'), 'R318': ('J303', '2'), 'R319': ('U303', '2'), 'C305': ('U303', '2'),
    'R324': ('J304', '1'), 'R325': ('J304', '1'), 'R320': ('U304', '6'), 'R321': ('U304', '3'),
    'R322': ('U304', '1'), 'R323': ('U304', '7'),
}

MOTOR_POURS = {}
POUR_NETS = ('VPACK', 'VMAIN', 'VDRV', 'BRAKE_SW', 'BUCK_SW', 'VMAIN_F', '+5V', '+5V_PI', '+5V_AUX')

# In2 planes (board coordinates). 5 V: the bottom band plus branches up to the clamp, the driver buffer and the
# index connector. 3.3 V: an island under the ESP32. Everything else on In2 is free for signals.
IN2_5V = [(0.3, 50.0), (47.0, 50.0), (47.0, 56.0), (W - 0.3, 56.0), (W - 0.3, H - 0.3), (0.3, H - 0.3)]  # band
IN2_3V3 = (58.0, 27.0, 92.0, 55.0)                                          # poured after routing
# 5 V feeders: pre-routed In2 tracks from the band up to a via by each 5 V cluster outside it
FEED_5V = [(36.5, 39.5), (64.5, 12.8), (102.5, 46.6)]


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
        props = {str(kilib.get(p, 'name')[1]): str(kilib.get(p, 'value')[1]) if kilib.get(p, 'value') else ''
                 for p in kilib.find(c, 'property')}
        d['props'] = props
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
    cy = fp.GetCourtyard(pcbnew.F_CrtYd if not fp.IsFlipped() else pcbnew.B_CrtYd)
    if cy.OutlineCount():
        b = cy.BBox()
    else:
        b = fp.GetBoundingBox(False)
    return (pcbnew.ToMM(b.GetLeft()) - OX, pcbnew.ToMM(b.GetTop()) - OY,
            pcbnew.ToMM(b.GetRight()) - OX, pcbnew.ToMM(b.GetBottom()) - OY)


def overlaps(a, b, m=0.25):
    return not (a[2] + m <= b[0] or b[2] + m <= a[0] or a[3] + m <= b[1] or b[3] + m <= a[1])


def place(fp, x, y, rot, side='F'):
    fp.SetPosition(pt(x, y))
    fp.SetOrientationDegrees(rot)
    if side == 'B' and not fp.IsFlipped():
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)


def build():
    comps, nets = load_netlist()
    board = pcbnew.NewBoard(PCB)
    board.SetCopperLayerCount(4)
    ds = board.GetDesignSettings()
    ds.SetBoardThickness(mm(1.6))
    pcbnew.SaveBoard(PCB, board, True)
    board = pcbnew.LoadBoard(PCB)
    board.SetCopperLayerCount(4)
    board.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_POWER)   # the router treats it as the GND plane, no wiring

    netinfo = {}
    for nm in nets:
        ni = pcbnew.NETINFO_ITEM(board, nm)
        board.Add(ni)
        netinfo[nm] = ni
    padnet = {}
    for nm, nodes in nets.items():
        for ref, pin in nodes:
            padnet[(ref, pin)] = nm

    fps = {}
    for ref, c in comps.items():
        fp = fp_load(c['footprint'])
        fp.SetReference(ref)
        fp.SetValue(c['value'])
        for k in ('MPN', 'Mfr', 'LCSC', 'Description', 'Datasheet'):
            v = c['fields'].get(k, '')
            if True:
                fp.SetField(k, v)
                f = fp.GetField(k)
                if f:
                    f.SetVisible(False)
                    f.SetLayer(pcbnew.F_Fab)
        fp.SetPath(pcbnew.KIID_PATH(c['sheetpath'] + c['tstamp']))
        fp.SetSheetname(c['sheetnames'])
        fp.SetSheetfile({'/Power/': 'power.kicad_sch', '/MCU/': 'mcu.kicad_sch',
                         '/Driver and timing I/O/': 'io.kicad_sch'}.get(c['sheetnames'], ''))
        if 'dnp' in c['props']:
            fp.SetDNP(True)
        if ref.startswith(('H', 'FID', 'TP')):
            fp.SetExcludedFromBOM(True)
            fp.SetExcludedFromPosFiles(True)
        for pad in fp.Pads():
            nm = padnet.get((ref, pad.GetNumber()))
            if nm:
                pad.SetNet(netinfo[nm])
        board.Add(fp)
        fps[ref] = fp

    # fixed parts
    occ = []
    for ref, (x, y, rot, side) in FIXED.items():
        place(fps[ref], x, y, rot, side)
        occ.append((ref, courtyard(fps[ref])))
    for ref, b in occ:
        if b[0] < -0.01 or b[1] < -0.01 or b[2] > W + 0.7 or b[3] > H + 0.7:
            print('outside board:', ref, b)
    for i, (r1, b1) in enumerate(occ):
        for r2, b2 in occ[i + 1:]:
            if overlaps(b1, b2, 0.0):
                print('overlap:', r1, r2)

    # everything else: by its anchor pad, nearest free spot
    def pad_pos(ref, num):
        for p in fps[ref].Pads():
            if p.GetNumber() == num:
                q = p.GetPosition()
                return pcbnew.ToMM(q.x) - OX, pcbnew.ToMM(q.y) - OY
        raise KeyError((ref, num))

    def anchor_of(ref):
        if ref in ANCHOR:
            return pad_pos(*ANCHOR[ref])
        mine = [padnet.get((ref, p.GetNumber())) for p in fps[ref].Pads()]
        best = None
        for nm in mine:
            if not nm or nm in ('GND', '+3V3', '+5V') or nm.startswith('unconnected'):
                continue
            for r2, pin in nets[nm]:
                if r2 == ref or r2 not in FIXED and r2 not in placed:
                    continue
                pos = pad_pos(r2, pin)
                big = len(list(fps[r2].Pads()))
                if best is None or big > best[0]:
                    best = (big, pos)
        return best[1] if best else None

    def fullname(net):
        if net in netinfo:
            return net
        m = [n for n in netinfo if n.endswith('/' + net)]
        assert len(m) == 1, (net, m)
        return m[0]

    # Kelvin sense: SHUNT_P/N leave R102's inner sense pads straight for U101 pins 10/9 (pre-routed, locked)
    def track(net, pts_, width, layer='F.Cu'):
        for a, b in zip(pts_, pts_[1:]):
            tr = pcbnew.PCB_TRACK(board)
            tr.SetStart(pt(*a))
            tr.SetEnd(pt(*b))
            tr.SetWidth(mm(width))
            tr.SetLayer(board.GetLayerID(layer))
            tr.SetNet(netinfo[fullname(net)])
            tr.SetLocked(True)
            board.Add(tr)

    p1 = pad_pos('R102', '1')
    p4 = pad_pos('R102', '4')
    p2 = pad_pos('R102', '2')
    p3 = pad_pos('R102', '3')
    u10 = pad_pos('U101', '10')
    u9 = pad_pos('U101', '9')
    # start at the sense pads' outer ends, 0.3 mm in, clear of the force pads; leave U101 horizontally and drop
    # straight onto the sense pads (P outside, N inside); pins 8 and 9 (VBUS, IN-) bridged at their pad ends;
    # pin 7 (GND) straight out to its own via
    sgn = -1.0 if p2[1] < p1[1] else 1.0
    s2 = (p2[0], p2[1] + sgn * 0.3)
    s3 = (p3[0], p3[1] + sgn * 0.3)
    u8 = pad_pos('U101', '8')
    u7 = pad_pos('U101', '7')
    track('SHUNT_P', [u10, (s2[0], u10[1]), s2], 0.2)
    track('SHUNT_N', [u9, (s3[0], u9[1]), s3], 0.2)
    track('SHUNT_N', [u8, (u8[0] + 0.6, u8[1]), (u9[0] + 0.6, u9[1])], 0.2)
    gv = (u7[0] + 2.3, u7[1])
    track('GND', [u7, gv], 0.3)
    # force pads: short stubs straight out, away from the sense pads, for the router to land on
    out = 1.0 if p1[0] > p4[0] else -1.0
    track('VMAIN', [(p1[0] + 0.4 * out, p1[1]), (p1[0] + 2.0 * out, p1[1])], 1.0)
    track('VDRV', [(p4[0] - 0.4 * out, p4[1]), (p4[0] - 2.0 * out, p4[1])], 1.0)
    xs = [p2[0], p3[0], u10[0], u7[0], gv[0]]
    ys = [p2[1], p3[1], u10[1], u7[1], gv[1]]
    occ.append(('sense', (min(xs) - 0.4, min(ys) - 0.5, max(xs) + 0.5, max(ys) + 0.6)))
    occ.append(('stubs', (min(p1[0], p4[0]) - 2.6, p1[1] - 0.9, max(p1[0], p4[0]) + 2.6, p1[1] + 0.9)))
    j = pad_pos('J103', '1')       # VDRV
    vd0 = (p4[0] - 2.0 * out, p4[1])
    track('VDRV', [vd0, (j[0] + 1.6, vd0[1]), (j[0], vd0[1] + 1.6), j], 1.5)
    occ.append(('vdrv', (j[0] - 1.5, vd0[1] - 1.0, vd0[0] + 0.5, j[1] + 1.0)))

    # USB-C: the two D+ pads and the two D- pads interleave at 0.5 mm. D- joins on F.Cu in a loop behind
    # the row; D+ drops through two 0.45/0.25 mm vias and joins on B.Cu. The router takes it from there.
    def via(net, xy, d=0.45, drill=0.25):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pt(*xy))
        v.SetWidth(mm(d))
        v.SetDrill(mm(drill))
        v.SetNet(netinfo[fullname(net)])
        v.SetLocked(True)
        board.Add(v)

    a6, b6, a7, b7 = (pad_pos('J201', n) for n in ('A6', 'B6', 'A7', 'B7'))
    edge = a6[1] - 0.575                      # inner edge of the pad row
    yv = edge - 0.9                           # D+ vias
    yl = yv - 0.95                            # D- loop
    track('USB_DN', [a7, (a7[0], yl), (b7[0], yl), b7], 0.2)
    track('USB_DP', [a6, (a6[0], yv)], 0.2)
    track('USB_DP', [b6, (b6[0], yv)], 0.2)
    via('USB_DP', (a6[0], yv))
    via('USB_DP', (b6[0], yv))
    track('USB_DP', [(a6[0], yv), (b6[0], yv)], 0.2, 'B.Cu')
    occ.append(('usb', (b7[0] - 0.5, yl - 0.4, b6[0] + 0.5, edge)))
    via('GND', gv, 0.6, 0.3)
    for fx, fy in FEED_5V:
        via('+5V', (fx, fy), 0.6, 0.3)
        track('+5V', [(fx, fy), (fx, (50.8 if fx < 47.0 else 56.8))], 0.5, 'In2.Cu')
        occ.append(('feed', (fx - 0.8, fy - 0.8, fx + 0.8, fy + 0.8)))

    # VMAIN trunk on F.Cu: from the shunt's force stub down past J103 to the brake (J102, D103, C123)
    s_end = (p1[0] + 2.0 * out, p1[1])
    jb = pad_pos('J102', '2')
    dk = pad_pos('D103', '1')
    cb = pad_pos('C123', '1')
    ty = 38.2
    track('VMAIN', [s_end, (25.3, s_end[1]), (25.3, ty), (jb[0], ty), jb], 1.5)
    track('VMAIN', [(dk[0], ty), dk], 1.5)
    track('VMAIN', [(cb[0], ty), cb], 1.0)
    occ.append(('trunk', (24.4, s_end[1] - 0.9, 26.2, ty + 0.9)))
    occ.append(('trunk', (jb[0] - 0.9, ty - 0.9, 26.2, ty + 0.9)))

    # buck switch node: pin 8 straight up, then to the diode's cathode and the inductor
    sw = pad_pos('U104', '8')
    k5 = pad_pos('D105', '1')
    l1 = pad_pos('L101', '1')
    sn = (sw[0], sw[1] - 1.4)
    track('BUCK_SW', [sw, sn, (k5[0], k5[1] + 1.0)], 1.5)
    track('BUCK_SW', [sn, (l1[0] - 1.2, l1[1] + 1.0)], 1.5)
    # 5 V out: a via array between the inductor and the output caps, into the In2 plane
    l2 = pad_pos('L101', '2')
    cps = [pad_pos(r, '1') for r in ('C115', 'C116', 'C117')]
    vx = (l2[0] + 2.05 + cps[0][0] - 0.575) / 2
    ys = [cps[0][1], (cps[0][1] + cps[1][1]) / 2, cps[1][1], (cps[1][1] + cps[2][1]) / 2, cps[2][1]]
    track('+5V', [(vx, ys[0]), (vx, ys[-1])], 1.0)
    for y in ys:
        via('+5V', (vx, y), 0.6, 0.3)
    for c in cps:
        track('+5V', [(vx, c[1]), c], 1.0)
    track('+5V', [(vx, l2[1]), l2], 1.5)
    occ.append(('5vout', (vx - 0.6, ys[0] - 0.6, vx + 0.6, ys[-1] + 0.6)))
    # 5 V at the Pi and AUX fuses: three and two vias into the plane
    f3 = pad_pos('F103', '1')
    for dx, dy in ((-1.3, 1.1), (0.0, 1.6), (1.3, 1.1)):
        via('+5V', (f3[0] + dx, f3[1] + dy), 0.6, 0.3)
        track('+5V', [f3, (f3[0] + dx, f3[1] + dy)], 0.8)
    occ.append(('f103', (f3[0] - 2.0, f3[1], f3[0] + 2.0, f3[1] + 2.3)))
    f2 = pad_pos('F102', '1')
    for dx, dy in ((-1.3, -1.0), (-1.3, 1.0)):
        via('+5V', (f2[0] + dx, f2[1] + dy), 0.6, 0.3)
        track('+5V', [f2, (f2[0] + dx, f2[1] + dy)], 0.8)
    occ.append(('f102', (f2[0] - 2.0, f2[1] - 1.7, f2[0], f2[1] + 1.7)))
    # Pi feed: F103 to J202 pins 1 and 2, 1.5 mm
    f3b = pad_pos('F103', '2')
    j1 = pad_pos('J202', '1')
    j2 = pad_pos('J202', '2')
    track('+5V_PI', [f3b, (j1[0] - 2.0, f3b[1]), (j1[0] - 2.0, j1[1]), j1], 1.5)
    track('+5V_PI', [j1, j2], 1.5)
    occ.append(('pi', (f3b[0] - 0.8, j1[1] - 0.8, j1[0], f3b[1] + 0.8)))

    # ESP32-S3: four ground vias in the corners of the ring between its center pad and its pins
    u = fps['U201'].GetPosition()
    ux, uy = pcbnew.ToMM(u.x) - OX, pcbnew.ToMM(u.y) - OY
    for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        vxy = (ux + 2.6 * sx, uy + 2.6 * sy)
        via('GND', vxy, 0.6, 0.3)
        track('GND', [(ux + 1.9 * sx, uy + 1.9 * sy), vxy], 0.3)

    # crystal: short F.Cu runs, no vias. XTAL_N straight up and over to pad 1; XTAL_P up, left through L201
    p53 = pad_pos('U201', '53')
    p54 = pad_pos('U201', '54')
    y1p = pad_pos('Y201', '1')
    y3p = pad_pos('Y201', '3')
    l2a = pad_pos('L201', '2')
    l1a = pad_pos('L201', '1')
    track('XTAL_N', [p53, (p53[0], 37.4), (y1p[0], 37.4), y1p], 0.2)
    track('XTAL_P', [p54, (p54[0], l2a[1]), l2a], 0.2)
    track('XTAL_P_L', [l1a, (l1a[0] - 0.7, l1a[1]), (l1a[0] - 0.7, y3p[1]), y3p], 0.2)
    occ.append(('xtal', (l1a[0] - 1.1, 34.0, p53[0] + 0.5, 39.9)))
    # its load caps and ground: each ground pad gets its own via
    c2x = pad_pos('C202', '1')
    c2g = pad_pos('C202', '2')
    c1x = pad_pos('C201', '1')
    c1g = pad_pos('C201', '2')
    y2g = pad_pos('Y201', '2')
    y4g = pad_pos('Y201', '4')
    track('XTAL_N', [c2x, (c2x[0], 37.4)], 0.2)
    track('XTAL_P_L', [c1x, (l1a[0] - 0.7, c1x[1]), (l1a[0] - 0.7, y3p[1])], 0.2)
    for g, vxy in ((y4g, (y4g[0], y4g[1] + 0.95)), (y2g, (y2g[0] + 0.95, y2g[1] - 0.8)), (c2g, (c2g[0] + 0.9, c2g[1] - 0.4)),
                   (c1g, (c1g[0], c1g[1] - 0.9))):
        via('GND', vxy, 0.6, 0.3)
        track('GND', [g, vxy], 0.3)
    occ.append(('xtal2', (c1g[0] - 1.4, 33.3, c2g[0] + 1.4, 37.0)))


    rings = []
    for big, m in (('U201', 2.2), ('U301', 1.2), ('U302', 1.0), ('U104', 1.0), ('U101', 0.8), ('J201', 1.5)):
        c = courtyard(fps[big])
        rings.append((c[0] - m, c[1] - m, c[2] + m, c[3] + m))
    placed = set(FIXED)
    todo = [r for r in fps if r not in FIXED]
    # anchored first, then the rest in an order that lets chains follow their predecessors
    todo.sort(key=lambda r: (r not in ANCHOR, r))
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
            fp = fps[ref]
            done = False
            for rad in [i * 0.5 for i in range(0, 40)]:
                n = max(1, int(2 * math.pi * rad / 0.5))
                for k in range(n):
                    th = 2 * math.pi * k / n
                    x, y = a[0] + rad * math.cos(th), a[1] + rad * math.sin(th)
                    x, y = round(x * 4) / 4, round(y * 4) / 4
                    for rot in (0, 90):
                        place(fp, x, y, rot)
                        b = courtyard(fp)
                        if b[0] < 0.3 or b[1] < 0.3 or b[2] > W - 0.3 or b[3] > H - 0.3:
                            continue
                        if any(overlaps(b, o, 0.6) for _, o in occ):
                            continue
                        # leave an escape ring around the fine-pitch chips
                        if any(overlaps(b, r, 0.0) for r in rings):
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
        todo = left
        if not todo:
            break
    if todo:
        print('no anchor:', todo)

    # outline
    pts = [(0, 0), (W, 0), (W, H), (0, H)]
    for i in range(4):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pt(*pts[i]))
        s.SetEnd(pt(*pts[(i + 1) % 4]))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.05))
        board.Add(s)

    def full(net):
        if net in netinfo:
            return net
        m = [n for n in netinfo if n.endswith('/' + net)]
        assert len(m) == 1, (net, m)
        return m[0]

    def zone(net, layer, poly, prio=0, clearance=0.25, thermal=True, minw=0.2):
        z = pcbnew.ZONE(board)
        z.SetLayer(board.GetLayerID(layer))
        z.SetNet(netinfo[full(net)])
        o = z.Outline()
        o.NewOutline()
        for x, y in poly:
            o.Append(mm(OX + x), mm(OY + y))
        z.SetAssignedPriority(prio)
        z.SetMinThickness(mm(minw))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL if thermal else pcbnew.ZONE_CONNECTION_FULL)
        z.SetThermalReliefGap(mm(0.3))
        z.SetThermalReliefSpokeWidth(mm(0.5))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        board.Add(z)
        return z

    inset = [(0.3, 0.3), (W - 0.3, 0.3), (W - 0.3, H - 0.3), (0.3, H - 0.3)]
    zone('GND', 'In1.Cu', inset, 0).SetLocalClearance(mm(0.3))
    # In2: 5 V under the power column and the right-hand 5 V loads' feed, 3.3 V under the logic
    # In2: a 5 V band along the bottom (buck, Pi, AUX) and a 3.3 V island under the MCU; the rest routes
    def keepout(poly, name):
        ka = pcbnew.ZONE(board)
        ka.SetIsRuleArea(True)
        ka.SetLayer(pcbnew.In2_Cu)
        ka.SetDoNotAllowTracks(True)
        ka.SetDoNotAllowVias(False)
        ka.SetDoNotAllowPads(False)
        ka.SetDoNotAllowFootprints(False)
        ka.SetDoNotAllowZoneFills(False)
        ka.SetZoneName(name)
        o = ka.Outline()
        o.NewOutline()
        for x, y in poly:
            o.Append(mm(OX + x), mm(OY + y))
        board.Add(ka)

    zone('+5V', 'In2.Cu', IN2_5V, 1)
    # the keep-out starts 1 mm into the plane, so the feeders (which end at 56.8) touch copper without entering it
    keepout([(0.3, 51.5), (45.5, 51.5), (45.5, 57.5), (W - 0.3, 57.5), (W - 0.3, H - 0.3), (0.3, H - 0.3)],
            '5V plane: no tracks on In2')
    for net, polys in MOTOR_POURS.items():
        for layer, poly in polys:
            zone(net, layer, poly, 2, 0.3, thermal=False, minw=0.5)

    for ref in ('FID1', 'FID2', 'FID3'):
        c = fps[ref].GetPosition()
        ka = pcbnew.ZONE(board)
        ka.SetIsRuleArea(True)
        ka.SetLayerSet(pcbnew.LSET.AllCuMask())
        ka.SetDoNotAllowTracks(True)
        ka.SetDoNotAllowVias(True)
        ka.SetDoNotAllowPads(False)
        ka.SetDoNotAllowFootprints(False)
        ka.SetDoNotAllowZoneFills(True)
        ka.SetZoneName('fiducial clear')
        o = ka.Outline()
        o.NewOutline()
        for k in range(16):
            th = 2 * math.pi * k / 16
            o.Append(c.x + mm(1.6 * math.cos(th)), c.y + mm(1.6 * math.sin(th)))
        board.Add(ka)

    silk = pcbnew.PCB_TEXT(board)
    silk.SetText('SPIN RIG CONTROLLER ${REVISION}')
    silk.SetPosition(pt(80.0, 66.0))
    silk.SetLayer(pcbnew.F_SilkS)
    silk.SetTextSize(pcbnew.VECTOR2I(mm(1.5), mm(1.5)))
    silk.SetTextThickness(mm(0.25))
    board.Add(silk)

    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)
    print('placed', len(fps), 'footprints')
    ok = pcbnew.ExportSpecctraDSN(board, DSN)
    print('DSN', ok, DSN)


def finish():
    board = pcbnew.LoadBoard(PCB)
    ok = pcbnew.ImportSpecctraSES(board, SES)
    print('SES', ok)
    nets = {}
    for code in range(board.GetNetCount()):
        ni = board.FindNet(code)
        if ni:
            nets[str(ni.GetNetname())] = ni

    def net(name):
        if name in nets:
            return nets[name]
        m = [n for n in nets if n.endswith('/' + name)]
        assert len(m) == 1, (name, m)
        return nets[m[0]]

    def add_zone(netname, layer, outline, prio, clearance=None, full=False):
        z = pcbnew.ZONE(board)
        z.SetLayer(board.GetLayerID(layer))
        z.SetNet(net(netname))
        z.Outline().Append(outline)   # copy: SetOutline() would take ownership of a Python-owned object
        z.SetAssignedPriority(prio)
        if clearance:
            z.SetLocalClearance(mm(clearance))
        z.SetMinThickness(mm(0.2))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if full else pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetThermalReliefGap(mm(0.3))
        z.SetThermalReliefSpokeWidth(mm(0.6))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        board.Add(z)
        return z

    def rect(x0, y0, x1, y1):
        ps = pcbnew.SHAPE_POLY_SET()
        ps.NewOutline()
        for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            ps.Append(mm(OX + x), mm(OY + y))
        return ps

    # high-current nets (motor feed, brake, buck input/switch node/output, Pi and AUX 5 V): copper poured 1.2 mm
    # around their own tracks and pads, on the layer they run
    for netname in POUR_NETS:
        n = net(netname)
        for layer in ('F.Cu', 'B.Cu'):
            lid = board.GetLayerID(layer)
            ps = pcbnew.SHAPE_POLY_SET()
            for tr in board.GetTracks():
                if tr.GetNetCode() == n.GetNetCode() and tr.GetLayer() == lid and tr.GetClass() == 'PCB_TRACK':
                    tr.TransformShapeToPolygon(ps, lid, mm(1.2), mm(0.01), pcbnew.ERROR_INSIDE)
            for fp in board.GetFootprints():
                if fp.GetReference() in ('J202', 'J104'):
                    continue          # leave the header pins' neighbors room for their thermal spokes
                for pad in fp.Pads():
                    if pad.GetNetCode() == n.GetNetCode() and pad.IsOnLayer(lid):
                        pad.TransformShapeToPolygon(ps, lid, mm(1.2), mm(0.01), pcbnew.ERROR_INSIDE)
            if ps.OutlineCount() == 0:
                continue
            ps.Simplify()
            # stay off the board edge
            ps.BooleanIntersection(rect(0.5, 0.5, W - 0.5, H - 0.5))
            add_zone(netname, layer, ps, 3, full=True)
            print('pour', netname, layer, ps.OutlineCount())

    # outer-layer ground pours
    for layer in ('F.Cu', 'B.Cu'):
        z = add_zone('GND', layer, rect(0.3, 0.3, W - 0.3, H - 0.3), 0, clearance=0.3)
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THT_THERMAL)   # SMD pads solid, through-hole relieved

    # In2: the 5 V plane reaches a little above the router's keep-out band, to take the vias it set at the edge;
    # a 3.3 V pour under the ESP32 picks up the 3.3 V vias there


    # the buck's exposed pad: solid to the F.Cu ground pour (no vias in it)
    u104 = board.FindFootprintByReference('U104')
    for pad in u104.Pads():
        if pad.GetNumber() == '9':
            pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)

    # silk: references only on parts big enough to carry them; the rest stay on the fab layer
    keep = ('U', 'J', 'Q', 'SW', 'TP', 'L', 'F', 'D1', 'CR')
    for fp in board.GetFootprints():
        ref = fp.Reference()
        small = any(s in fp.GetFPIDAsString() for s in ('0402', '0603', 'SOD-323', 'SOT-23'))
        if ref.GetText().startswith(('H', 'FID')) or (small and not ref.GetText().startswith(('U', 'D2'))):
            ref.SetLayer(pcbnew.F_Fab)
        else:
            ref.SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
            ref.SetTextThickness(mm(0.12))

    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)
    print('finished')

if __name__ == '__main__':
    os.makedirs(os.path.dirname(NET), exist_ok=True)
    {'place': build, 'finish': finish}[sys.argv[1]]()
