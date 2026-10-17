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

W, H = 67.0, 90.0
OX, OY = 100.0, 100.0          # board origin in KiCad coordinates (mm)


def mm(v):
    return pcbnew.FromMM(v)


def pt(x, y):
    return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


# ref: (x, y, rotation degrees CCW, side)   board coordinates, y down
#
# The board sits on the Pi 4B like a HAT, 67 x 90 mm. The Pi is turned so its USB/Ethernet end is past the bottom
# edge (the HAT's 65 mm line is y = 90); its GPIO edge runs down x = 56 and its SD-card end is at y = 25, so a 25 mm
# strip overhangs at the top and an 11 mm strip at the right (wide enough for a row of pin names between the Pi
# socket's pads and the right-edge headers'). Its holes land at x 3.5/52.5, y 28.5/86.5, and pin 1
# of its header at (56 - 4.77, 25 + 8.37). Side-entry connectors use the top, left and right edges, never the
# bottom one, where the Pi's USB and Ethernet stand.
U201_XY = (35.0, 59.0)
U104_XY = (20.0, 79.0)


def _near_u201(dx, dy):
    return (U201_XY[0] + dx, U201_XY[1] + dy)


def _near_u104(dx, dy):
    return (U104_XY[0] + dx, U104_XY[1] + dy)


FIXED = {
    # pack in (top-left) -> Q101 (tab toward the XT60) -> VMAIN bulk and TVS; shunt R102 -> VDRV -> J103 (left)
    'J101': (5.2, 16.65, 0, 'F'), 'Q101': (24.0, 14.5, 180, 'F'), 'C101': (35.8, 5.6, 0, 'F'),
    'C102': (33.3, 15.0, 90, 'F'), 'C103': (36.8, 15.0, 90, 'F'), 'D102': (36.0, 21.8, 0, 'F'),
    'U101': (15.0, 24.5, 0, 'F'), 'R102': (22.0, 29.5, 180, 'F'), 'J103': (8.3, 34.85, 270, 'F'),
    'J203': (16.0, 38.5, 0, 'F'),     # Qwiic beside the INA238, the other I2C part
    # brake: resistor connector, freewheel diode and switch in a tight loop, with its own bus capacitor
    'J102': (4.55, 51.46, 90, 'F'), 'D103': (13.35, 49.06, 270, 'F'), 'Q102': (21.05, 50.46, 180, 'F'),
    'C123': (19.55, 45.46, 0, 'F'), 'U102': (31.5, 36.0, 0, 'F'), 'U103': (31.5, 42.0, 0, 'F'),
    'F101': (13.5, 56.6, 0, 'F'),
    # 5 V buck along the bottom, laid out per the TPS54560B guide: input caps by VIN, the catch diode's anode
    # against their ground, its cathode and the inductor on a short switch node at pin 8
    'U104': U104_XY + (0, 'F'),
    # its small parts at the offsets the pre-routed BOOT/EN/RT nets below were laid against
    'C112': _near_u104(-6.0, 3.0) + (0, 'F'), 'R113': _near_u104(-3.5, 4.75) + (90, 'F'),
    'R114': _near_u104(-1.0, 6.0) + (0, 'F'), 'R115': _near_u104(-5.25, 5.5) + (90, 'F'),
    'R124': _near_u104(-8.25, 3.5) + (90, 'F'), 'C111': _near_u104(-1.25, 4.25) + (0, 'F'),
    'C108': (14.6, 78.4, 90, 'F'), 'C109': (11.0, 78.4, 90, 'F'), 'C110': (7.4, 78.4, 90, 'F'),
    'D105': (17.0, 72.6, 180, 'F'), 'L101': (31.0, 73.0, 0, 'F'),
    'C115': (42.0, 70.2, 0, 'F'), 'C116': (42.0, 73.8, 0, 'F'), 'C117': (42.0, 77.4, 0, 'F'),
    'F103': (46.5, 66.0, 90, 'F'), 'F102': (38.5, 82.5, 0, 'F'), 'J104': (35.5, 86.4, 0, 'F'),
    # driver connectors: control down the right edge, encoder-out along the top, encoder-in down the right (the
    # pass-through nests round the top-right corner); 3.5 mm headers with their faces 0.3 mm inside the edge
    # (J305 stops 1.3 mm short of the right edge and J302 starts 1 mm below it, so J305's pin names fit; the
    # right-edge courtyards touch, to fit in 90 mm)
    'J305': (62.6, 8.3, 180, 'F'), 'J302': (58.7, 32.15, 90, 'F'), 'J301': (58.7, 66.25, 90, 'F'),
    'J304': (63.3, 75.85, 90, 'F'), 'J303': (63.3, 86.84, 90, 'F'),
    # the Pi: its socket on the underside (the HAT template's footprint and side, turned with the Pi) and holes
    'J202': (51.23, 33.37, 180, 'B'), 'H7': (3.5, 28.5, 0, 'F'), 'H5': (52.5, 28.5, 0, 'F'),
    'H3': (3.5, 86.5, 0, 'F'), 'H6': (52.5, 86.5, 0, 'F'),
    # USB-C on the left edge beside the ESP32; the buttons between it and the brake
    'J201': (6.5, 62.5, 270, 'F'), 'SW201': (15.2, 64.5, 0, 'F'), 'SW202': (23.0, 64.5, 0, 'F'),
    # logic
    'U201': U201_XY + (0, 'F'), 'Y201': _near_u201(-3.8, -8.2) + (90, 'F'), 'L201': _near_u201(-4.0, -5.6) + (0, 'F'),
    'C202': _near_u201(-1.7, -8.0) + (90, 'F'), 'C201': _near_u201(-6.0, -9.6) + (180, 'F'),
    'U301': (44.5, 46.0, 90, 'F'), 'U302': (48.0, 17.0, 90, 'F'),
    'U304': (45.0, 55.0, 90, 'F'), 'U303': (45.0, 60.0, 90, 'F'),
    # two test pads in the top edge's free gap (the logic's surroundings are full)
    'TP205': (20.0, 3.5, 0, 'F'), 'TP305': (24.0, 3.5, 0, 'F'),
}

# small parts that sit by a particular pad (otherwise: the nearest pad they share a signal net with)
ANCHOR = {
    'C104': ('U101', '6'), 'R103': ('U101', '3'), 'C106': ('U102', '6'), 'C107': ('U103', '1'),
    'C301': ('U301', '20'), 'C304': ('U302', '16'),
    'C306': ('U303', '5'), 'C307': ('U304', '8'),
    'C204': ('U201', '2'), 'C205': ('U201', '3'), 'C206': ('U201', '20'), 'C208': ('U201', '46'),
    'C209': ('U201', '55'), 'C210': ('U201', '56'), 'C211': ('U201', '29'), 'C207': ('U201', '29'),
    'U105': ('U201', '20'), 'D107': ('U105', '1'), 'D108': ('J201', 'A4/B9'), 'C118': ('U105', '1'),
    'C119': ('U105', '5'), 'C120': ('U105', '5'), 'C212': ('J201', 'B4/A9'), 'CR201': ('J201', 'A6'),
    'R204': ('J201', 'A5'), 'R205': ('J201', 'B5'), 'R206': ('J201', 'A6'), 'R207': ('J201', 'A7'),
    'R211': ('J202', '8'), 'R212': ('J202', '10'),
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
    'TP206': ('U104', '9'), 'TP306': ('U301', '10'), 'TP301': ('J301', '1'), 'TP302': ('J304', '1'),
    'TP303': ('U302', '3'), 'TP304': ('J304', '3'), 
    'R317': ('J303', '3'), 'R318': ('J303', '2'), 'R319': ('U303', '2'), 'C305': ('U303', '2'),
    'R324': ('J304', '1'), 'R325': ('J304', '1'), 'R320': ('U304', '6'), 'R321': ('U304', '3'),
    'R322': ('U304', '1'), 'R323': ('U304', '7'),
    # ALM and BRK: the pull-ups, series resistors and filter caps sit at the ESP32's end of the run
    'R309': ('U201', '6'), 'R310': ('U201', '6'), 'C302': ('U201', '6'),
    'R311': ('U201', '7'), 'R312': ('U201', '7'), 'C303': ('U201', '7'),
}

MOTOR_POURS = {}
POUR_NETS = ('VPACK', 'VMAIN', 'VDRV', 'BRAKE_SW', 'BUCK_SW', 'VMAIN_F', '+5V', '+5V_PI', '+5V_AUX')

# In2: a 5 V plane under the buck along the bottom; the rest of In2 carries signals. The keep-out (no In2 tracks)
# starts 1.5 mm inside the plane, so the feeders end in copper without entering it.
IN2_5V = [(0.3, 68.0), (48.6, 68.0), (48.6, H - 0.3), (0.3, H - 0.3)]
KEEPOUT_5V = [(0.3, 69.5), (47.1, 69.5), (47.1, H - 0.3), (0.3, H - 0.3)]
# 5 V feeders: (via x, y, end x, end y), pre-routed In2 tracks from a via by a 5 V load down into the plane
FEED_5V = []          # the router reaches the few 5 V loads outside the plane
# the Pi feed's waypoints from F103 to the socket's pin 2 (then pin 4): up the column's left side, over its top end
PI_FEED = [(49.0, 62.1), (49.0, 31.2), (53.77, 31.2)]


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
    board.GetTitleBlock().SetRevision('V1')     # the silkscreen's ${REVISION}; tools/plot_gerbers.sh reads it here

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
    def via(net, xy, d=0.45, drill=0.3):     # 0.45 / 0.30: JLC's standard multilayer via, no small-hole fee
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pt(*xy))
        v.SetWidth(mm(d))
        v.SetDrill(mm(drill))
        v.SetNet(netinfo[fullname(net)])
        v.SetLocked(True)
        board.Add(v)

    a6, b6, a7, b7 = (pad_pos('J201', n) for n in ('A6', 'B6', 'A7', 'B7'))
    # worked in the connector's frame: n points from the pad row into the board, whichever edge J201 is on
    th = math.radians(fps['J201'].GetOrientationDegrees())
    n_ = (-math.sin(th), -math.cos(th))         # (0, -1) turned with the footprint

    def inward(p, d):
        return (p[0] + n_[0] * d, p[1] + n_[1] * d)
    a6v, b6v = inward(a6, 0.575 + 0.9), inward(b6, 0.575 + 0.9)       # D+ vias, 0.9 mm in from the row
    a7l, b7l = inward(a7, 0.575 + 0.9 + 0.95), inward(b7, 0.575 + 0.9 + 0.95)   # D- loop behind them
    track('USB_DN', [a7, a7l, b7l, b7], 0.2)
    track('USB_DP', [a6, a6v], 0.2)
    track('USB_DP', [b6, b6v], 0.2)
    via('USB_DP', a6v)
    via('USB_DP', b6v)
    track('USB_DP', [a6v, b6v], 0.2, 'B.Cu')
    ux_ = [q[0] for q in (a6, b6, a7, b7, a7l, b7l)]
    uy_ = [q[1] for q in (a6, b6, a7, b7, a7l, b7l)]
    occ.append(('usb', (min(ux_) - 0.5, min(uy_) - 0.5, max(ux_) + 0.5, max(uy_) + 0.5)))
    via('GND', gv, 0.6, 0.3)
    for fx, fy, ex, ey in FEED_5V:
        via('+5V', (fx, fy), 0.6, 0.3)
        track('+5V', [(fx, fy), (ex, ey)], 0.5, 'In2.Cu')
        occ.append(('feed', (fx - 0.8, fy - 0.8, fx + 0.8, fy + 0.8)))

    # VMAIN trunk on F.Cu: from the shunt's force stub down past J103 to the brake (J102, D103, C123)
    s_end = (p1[0] + 2.0 * out, p1[1])
    jb = pad_pos('J102', '2')
    dk = pad_pos('D103', '1')
    cb = pad_pos('C123', '1')
    tx, ty = s_end[0] + 1.39, jb[1] - 4.84    # down 1.4 mm right of the stub, across 4.8 mm above J102's VMAIN pin
    track('VMAIN', [s_end, (tx, s_end[1]), (tx, ty), (jb[0], ty), jb], 1.5)
    track('VMAIN', [(dk[0], ty), dk], 1.5)
    track('VMAIN', [(cb[0], ty), cb], 1.0)
    occ.append(('trunk', (tx - 0.9, s_end[1] - 0.9, tx + 0.9, ty + 0.9)))
    occ.append(('trunk', (jb[0] - 0.9, ty - 0.9, tx + 0.9, ty + 0.9)))

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
    # Pi feed: F103 to the Pi socket's pins 2 and 4 (its 5 V), 2 mm on F.Cu, by the PI_FEED waypoints
    f3b = pad_pos('F103', '2')
    j4 = pad_pos('J202', '4')
    j2 = pad_pos('J202', '2')
    feed = [f3b] + PI_FEED + [j2, j4]
    track('+5V_PI', feed, 2.0)
    for a, c in zip(feed, feed[1:]):
        occ.append(('pi', (min(a[0], c[0]) - 1.3, min(a[1], c[1]) - 1.3, max(a[0], c[0]) + 1.3, max(a[1], c[1]) + 1.3)))

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
    ux0, uy0 = pcbnew.ToMM(fps['U201'].GetPosition().x) - OX, pcbnew.ToMM(fps['U201'].GetPosition().y) - OY
    xn_y = uy0 - 6.6                          # XTAL_N's run, 6.6 mm above the ESP32's centre
    track('XTAL_N', [p53, (p53[0], xn_y), (y1p[0], xn_y), y1p], 0.2)
    track('XTAL_P', [p54, (p54[0], l2a[1]), l2a], 0.2)
    track('XTAL_P_L', [l1a, (l1a[0] - 0.7, l1a[1]), (l1a[0] - 0.7, y3p[1]), y3p], 0.2)
    occ.append(('xtal', (l1a[0] - 1.1, uy0 - 10.0, p53[0] + 0.5, uy0 - 4.1)))
    # its load caps and ground: each ground pad gets its own via
    c2x = pad_pos('C202', '1')
    c2g = pad_pos('C202', '2')
    c1x = pad_pos('C201', '1')
    c1g = pad_pos('C201', '2')
    y2g = pad_pos('Y201', '2')
    y4g = pad_pos('Y201', '4')
    track('XTAL_N', [c2x, (c2x[0], xn_y)], 0.2)
    track('XTAL_P_L', [c1x, (l1a[0] - 0.7, c1x[1]), (l1a[0] - 0.7, y3p[1])], 0.2)
    for g, vxy in ((y4g, (y4g[0], y4g[1] + 0.95)), (y2g, (y2g[0] + 0.95, y2g[1] - 0.8)), (c2g, (c2g[0] + 0.9, c2g[1] - 0.4)),
                   (c1g, (c1g[0], c1g[1] - 0.9))):
        via('GND', vxy, 0.6, 0.3)
        track('GND', [g, vxy], 0.3)
    occ.append(('xtal2', (c1g[0] - 1.4, uy0 - 10.7, c2g[0] + 1.4, uy0 - 7.0)))

    # The buck's small nets, as they closed cleanly before (the router left them open twice): BUCK_RT down the
    # pocket inside BUCK_EN; BUCK_EN's two halves (pin 3 to R124, R113 to R114) joined by a B.Cu hop; BUCK_BOOT
    # west between the input caps' pad rows, under BUCK_EN's via on B.Cu, and up into C112. Laid out for U104 at
    # (20, 62); shifted with it.
    bx, by = U104_XY[0] - 20.0, U104_XY[1] - 62.0

    def bk(*p_):
        return [(x + bx, y + by) for x, y in p_]
    track('BUCK_RT', bk((17.44, 63.91), (15.45, 65.89), (15.45, 67.31), (14.75, 68.01)), 0.2)
    track('BUCK_EN', bk((17.44, 62.63), (17.25, 62.63), (15.46, 64.42), (12.32, 64.42), (11.75, 64.99)), 0.2)
    track('BUCK_EN', bk((16.5, 66.24), (15.92, 66.82), (15.92, 67.6), (16.32, 68.0), (18.49, 68.0)), 0.2)
    via('BUCK_EN', bk((17.4, 68.0))[0], 0.6, 0.3)
    track('BUCK_EN', bk((17.4, 68.0), (12.6, 64.42)), 0.2, 'B.Cu')
    via('BUCK_EN', bk((12.6, 64.42))[0], 0.6, 0.3)
    track('BUCK_BOOT', bk((17.44, 60.09), (16.72, 60.81), (13.0, 60.81), (12.6, 61.21), (12.6, 61.6)), 0.2)
    via('BUCK_BOOT', bk((12.6, 61.6))[0], 0.6, 0.3)
    track('BUCK_BOOT', bk((12.6, 61.6), (11.9, 62.3), (11.9, 65.2), (13.0, 65.9)), 0.2, 'B.Cu')
    via('BUCK_BOOT', bk((13.0, 65.9))[0], 0.6, 0.3)
    track('BUCK_BOOT', bk((13.0, 65.9), (13.52, 65.38), (13.52, 65.0)), 0.2)
    occ.append(('buck', (bx + 11.0, by + 59.8, bx + 18.8, by + 68.6)))

    # The ESP32's bottom pins 15-18 sit in a pocket closed by the 3.3 V feed from pin 20: fan them out to vias
    # (the router left LED_STAT boxed in twice) and neck pin 20's escape to 0.25 mm. Laid out for U201 at (72, 44).
    ex, ey = U201_XY[0] - 72.0, U201_XY[1] - 44.0

    def es(*p_):
        return [(x + ex, y + ey) for x, y in p_]
    track('V5_SENSE', es((69.4, 47.43), (69.4, 48.0), (68.97, 48.0), (68.44, 47.47)), 0.2)
    via('V5_SENSE', es((68.44, 47.47))[0], 0.6, 0.3)
    track('LED_FAULT', es((69.8, 47.43), (69.8, 48.93), (69.64, 49.09)), 0.2)
    via('LED_FAULT', es((69.64, 49.09))[0], 0.6, 0.3)
    track('LED_STAT', es((70.2, 47.43), (70.2, 50.1)), 0.2)
    via('LED_STAT', es((70.2, 50.1))[0], 0.6, 0.3)
    track('PI_TX', es((70.6, 47.43), (70.6, 48.4), (70.8, 48.6), (70.8, 48.9)), 0.2)
    via('PI_TX', es((70.8, 48.9))[0], 0.6, 0.3)
    track('+3V3', es((71.4, 47.43), (71.4, 50.87)), 0.25)
    occ.append(('esp-fan', (ex + 67.9, ey + 47.0, ex + 71.8, ey + 51.2)))


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
    keepout(KEEPOUT_5V, '5V plane: no tracks on In2')
    for net, polys in MOTOR_POURS.items():
        for layer, poly in polys:
            zone(net, layer, poly, 2, 0.3, thermal=False, minw=0.5)

    for ref in [r for r in fps if r.startswith('FID')]:
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
    silk.SetPosition(pt(W / 2, H / 2))      # silk.py moves it to where it fits
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
