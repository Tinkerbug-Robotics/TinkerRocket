"""Places every part and lays the hand-routed tracks (RF, the PX100's fan-out, the buck loop, USB) on the board that
make_pcb.py generated. The rest is left to the autorouter (route.sh).

The PX100's RF chain (the Wilkinson split, both SAW arms and their matching) and its left and bottom-left neighbours
(VDD_RF, TCXO, LDO_B_OUT) are copied part for part, with their tracks, from the routed PX100 carrier
(gnss-px100-18mm-hv): the same parts, nets and footprints, at the same offsets from U1. The carrier left the PX100's
GPIOs unconnected, so everything on U1's top, right and bottom-right is new.

  KiCad python:  layout.py   reads gen/out/generated.kicad_pcb, writes px100-test-board.kicad_pcb
"""
import math
import os
import shutil
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kilib  # noqa: E402

PROJ = kilib.PROJ
SRC = os.path.join(HERE, 'out', 'generated.kicad_pcb')
PCB = os.path.join(PROJ, 'px100-test-board.kicad_pcb')
CARRIER = os.path.join(PROJ, '..', 'gnss-px100-18mm-hv', 'gnss-px100-18mm-hv.kicad_pcb')
OX, OY = 100.0, 100.0
W, H = 75.0, 52.0

U1_AT = (24.6, 21.5)            # board coordinates; U1 at 90 degrees as on the carrier (RF pins 37/40 face J1)
SH_OFF = (-3.33, 0.53)          # shield centre from U1: the carrier's RF chain fills the left of the can, the GPIO
                                # escape vias the right

# copied from the carrier at their offsets from U1, with every track on these nets (carrier name -> ours)
COPY_PARTS = ['C26', 'C33', 'L10', 'L11', 'R10', 'C34', 'FL6', 'C36', 'L14', 'C35', 'L12', 'FL7', 'C37', 'L13', 'L15',
              'C53', 'C49', 'C50', 'C46', 'Y1', 'C47', 'C48']
COPY_NETS = {'LNA_OUT': 'RF_SPLIT', 'ARM_L5': 'ARM_L5', 'Net-(FL6-IN)': 'Net-(FL6-IN)',
             'Net-(FL6-OUT)': 'Net-(FL6-OUT)', 'Net-(FL7-OUT)': 'Net-(FL7-OUT)', 'LNA_IN1': 'LNA_IN1',
             'LNA_IN2': 'LNA_IN2', 'VDD_TCXO': 'VDD_TCXO', 'VDD_RF': 'VDD_RF', 'TCXO_I': 'TCXO_I',
             'Net-(Y1-OUT)': 'Net-(Y1-OUT)'}
# VDD_RF: only its F.Cu pin-to-cap tracks; the carrier's In2 link between the three cap groups is redone below
F_ONLY = {'VDD_RF'}
# carrier tracks not wanted here (U1-relative start points, mm): the VDD_RF hop that boxed in pins 31-33
SKIP_TRACKS = [(-3.59, -0.69)]

# ---------------------------------------------------------------- the receiver, in mm from U1's centre
# U1 at 90 degrees: pins 1-10 along the bottom (x -1.8 .. 1.8), 11-20 up the right side (y 1.8 .. -1.8), 21-30 along
# the top (x 1.8 .. -1.8), 31-40 down the left side (y -1.8 .. 1.8). The can's walls (pads) run 7.65-8.65 mm from its
# centre, with two 2.2 mm gaps per side; tracks leave through the gaps on F.Cu or pass under the walls on In2/B.Cu.
#   top: pins 24-30 straight up through the top gap as a 0.27 mm-pitch bundle;
#   right: pins 12-19 to two staggered rows of vias (x 3.4 / 3.9), out on the inner layers; the buck's three nets
#     (DCDC_IN, DCDC_OUT, V_CORE) through the upper gap to C38, L16, D7 and R11 just outside the can;
#   bottom-right: RTC_XIN/RTC_XOUT through the lower gap to Y2 outside, as on the carrier; NC8 and ANT_BIAS to vias;
#   left: PPS, GPIO2 and pin 36 (through R330, not fitted) to vias in the top-left, which stage 1 used to fill.
REL = {
    'C55': (-4.55, -1.72, 180),
    'C54': (-5.0, -3.4, 90), 'C51': (-6.0, -3.4, 90), 'C52': (-7.0, -3.4, 90),
    'R330': (-6.6, -0.6, 90),
    'C39': (2.6, -3.9, 0), 'C40': (1.1, -4.6, 90), 'C41': (2.6, -5.5, 90),
    'C42': (1.1, 4.3, 0), 'C44': (3.1, 4.75, 0), 'C43': (3.1, 5.85, 0),
    # outside the can
    'C38': (6.15, -0.95, -90), 'R11': (8.2, -0.95, 180), 'L16': (7.0, -3.5, 90), 'D7': (8.8, -3.9, 90),
    'Y2': (6.9, 3.6, 90),
}
TOP_BUNDLE = {24: 0.6, 25: 0.33, 26: 0.06, 27: -0.21, 28: -0.48, 29: -0.75, 30: -1.02}     # pin: x through the gap
RIGHT_VIAS = {12: (3.4, 1.4), 13: (3.9, 1.0), 14: (3.4, 0.6), 15: (3.9, 0.2), 16: (3.4, -0.2), 17: (3.9, -0.6),
              18: (3.4, -1.0), 19: (3.9, -1.4)}
SIG = 0.12          # U1 fan-out
RF_W = 0.34         # 50 ohm coplanar on JLC04161H-7628 (make_pro.py)

# ---------------------------------------------------------------- the rest, board coordinates
U1_NET = {}         # filled from the board: pin -> net
J1_AT = (3.35, 19.0)                        # RF_IN enters the can through its upper-left gap, 2.5 mm above U1
ABS = {
    'J1': J1_AT + (0,),
    'D6': (6.4, 19.35, 90), 'L1': (8.2, 19.485, 90), 'C20': (8.2, 21.45, -90),
    'R2': (9.7, 20.97, 180), 'R3': (9.7, 22.05, 180),
    # the probe header along the top edge, over the can: odd pins (U1 8-24) in the lower row, even (25-36) above
    'J301': (10.5, 9.0, 90),
    'U201': (49.0, 22.0, 0), 'U301': (56.5, 31.5, 0), 'U101': (58.2, 22.0, 180), 'U103': (62.0, 34.0, 0),
    'CR101': (62.1, 22.0, 0),
    # the mux's supply and SEL1 parts to its lower right, so CP_DN/CP_DP (pins 9/10) leave straight down
    'R104': (59.3, 25.0, 90), 'R105': (60.35, 25.0, 90), 'C102': (61.4, 25.0, 90),
    # pin 20's 100 nF just under its via (esp_extras)
    'C206': (48.4, 27.45, -90),
    # the antenna-bias monitor's 100 k to GND, GND end outward (to C307): turned the other way its GND pad was boxed in
    'R332': (8.15, 25.5, 180),
    # the S3's 3.3 V pins 2/3 (top of its left side): their caps sit above the link column
    'C205': (44.3, 19.7, 90), 'C204': (43.25, 19.65, 90),
    'J101': (W - 6.5, 22.0, 90), 'S101': (59.0, 4.6, 90), 'U104': (28.0, 40.5, 0),
    'J201': (26.0, 48.6, 90),
    'SW201': (55.8, 45.6, 0), 'SW202': (64.0, 45.6, 0), 'SW301': (11.5, 46.0, 0), 'SW302': (19.8, 46.0, 0),
    'H1': (3.5, 3.5, 0), 'H2': (W - 3.5, 3.5, 0), 'H3': (3.5, H - 3.5, 0), 'H4': (W - 3.5, H - 3.5, 0),
}
# the 1 k links, at the ESP32 end: a column off its left side (pins 6-14, top to bottom), a row under it (pins 15-27,
# left to right) and two above (pins 43/44)
LEFT_LINKS = ['R311', 'R326', 'R312', 'R314', 'R319', 'R318', 'R320', 'R321']
BOTTOM_LINKS = ['R322', 'R310', 'R313', 'R323', 'R327', 'R316', 'R317', 'R325']
for k, r in enumerate(LEFT_LINKS):
    ABS[r] = (42.3, 21.8 + 1.05 * k, 0)
for k, r in enumerate(BOTTOM_LINKS):
    ABS[r] = (44.6 + 1.05 * k, 30.8, 90)
ABS['R324'] = (51.6, 15.3, 90)
ABS['R315'] = (52.65, 15.3, 90)

# everything else goes next to a pad it connects to (spiral search for the nearest free spot), biggest first
ANCHOR = {
    'R12': ('U1', '17'), 'C56': ('U1', '17'), 'R14': ('U1', '18'), 'R15': ('U1', '19'), 'C305': ('U1', '14'),
    'R13': ('U1', '30'), 'R1': ('U1', '31'), 'D1': ('U1', '31'),
    'R331': ('R3', '1'), 'C306': ('R3', '1'), 'TP302': ('R3', '1'),
    'R333': ('C20', '1'), 'R334': ('C20', '1'), 'C307': ('C20', '1'), 'TP301': ('C20', '1'),
    'Y201': ('U201', '53'), 'L201': ('U201', '54'), 'C201': ('U201', '54'), 'C202': ('U201', '53'),
    'R201': ('U201', '4'), 'C203': ('U201', '4'),
    'R202': ('U201', '5'), 'C207': ('U201', '29'), 'C208': ('U201', '46'),
    'C209': ('U201', '55'), 'C210': ('U201', '56'), 'C211': ('U201', '55'), 'R203': ('U201', '52'),
    'R204': ('U201', '26'), 'R205': ('U201', '25'), 'R206': ('U201', '47'), 'D201': ('U201', '47'),
    'R101': ('J101', 'A5'), 'R102': ('J101', 'B5'), 'C101': ('J101', 'A4/B9'),
    'TP101': ('J101', 'A4/B9'), 'TP104': ('J101', 'S2'),
    'R103': ('U101', '4'), 'R109': ('U101', '4'),
    'C103': ('U103', '1'), 'C104': ('U103', '5'), 'C105': ('U103', '5'), 'R106': ('U103', '5'), 'D101': ('U103', '5'),
    'TP102': ('U103', '5'),
    'C106': ('U104', '7'), 'C107': ('U104', '8'), 'C108': ('U104', '6'), 'R107': ('U104', '5'),
    'FL101': ('U104', '1'), 'C109': ('U104', '2'), 'C110': ('U104', '3'), 'C111': ('U104', '1'),
    'R108': ('U104', '4'), 'D102': ('U104', '4'), 'TP103': ('U104', '1'), 'TP105': ('U104', '4'),
    'C301': ('U301', '6'), 'C302': ('U301', '6'), 'C303': ('U301', '5'), 'C304': ('U301', '7'),
    'R301': ('U301', '8'), 'R302': ('U301', '8'), 'R303': ('U301', '9'), 'D301': ('U301', '9'),
    'R304': ('U301', '21'), 'R305': ('U301', '20'), 'R306': ('U301', '14'), 'D302': ('U301', '14'),
    'R307': ('U301', '13'), 'D303': ('U301', '13'),
    'Q301': ('U301', '19'), 'R309': ('U301', '19'), 'Q302': ('U301', '23'), 'R308': ('U301', '23'),
}
# kept free: the can and its RF approach, the header and button labels, the USB-C mouth
KEEPOUT = [
    (12.0, 12.8, 30.5, 31.3),           # SH1 and its ring
    (0.0, 15.0, 12.6, 18.4), (1.6, 18.4, 12.9, 19.75),      # the RF line, J1 to the can
    (8.5, 1.2, 35.4, 5.4), (8.5, 10.1, 35.4, 12.95),        # J301 labels
    (7.0, 48.9, 24.3, 50.0), (51.0, 48.5, 69.0, 49.6),      # button labels
]


def mm(v):
    return pcbnew.FromMM(v)


def tomm(v):
    return pcbnew.ToMM(v)


class Board:
    def __init__(self):
        # the generated board loads with this project's rules and net classes (KiCad reads <name>.kicad_pro beside it)
        shutil.copy(PCB[:-len('.kicad_pcb')] + '.kicad_pro', SRC[:-len('.kicad_pcb')] + '.kicad_pro')
        self.b = pcbnew.LoadBoard(SRC)
        self.nets = {}
        for code, ni in self.b.GetNetInfo().NetsByNetcode().items():
            self.nets[ni.GetNetname().split('/')[-1]] = ni
        self.ux, self.uy = U1_AT

    def fp(self, ref):
        return self.b.FindFootprintByReference(ref)

    def net(self, name):
        return self.nets[name]

    def u(self, dx, dy):
        """U1-relative to KiCad coordinates."""
        return pcbnew.VECTOR2I(mm(OX + self.ux + dx), mm(OY + self.uy + dy))

    def a(self, x, y):
        return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))

    def place_u(self, ref, dx, dy, rot):
        f = self.fp(ref)
        f.SetOrientationDegrees(rot)
        f.SetPosition(self.u(dx, dy))

    def place_a(self, ref, x, y, rot):
        f = self.fp(ref)
        f.SetOrientationDegrees(rot)
        f.SetPosition(self.a(x, y))

    def track(self, net, layer, width, pts, rel=True):
        conv = self.u if rel else self.a
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            if math.hypot(x1 - x0, y1 - y0) < 1e-3:
                continue
            t = pcbnew.PCB_TRACK(self.b)
            t.SetStart(conv(x0, y0))
            t.SetEnd(conv(x1, y1))
            t.SetWidth(mm(width))
            t.SetLayer(self.b.GetLayerID(layer))
            t.SetNet(self.net(net))
            self.b.Add(t)

    def via(self, net, x, y, rel=True, d=0.4, drill=0.3):
        v = pcbnew.PCB_VIA(self.b)
        v.SetPosition((self.u if rel else self.a)(x, y))
        v.SetWidth(mm(d))
        v.SetDrill(mm(drill))
        v.SetNet(self.net(net))
        self.b.Add(v)
        return v


def copy_carrier(B):
    car = pcbnew.LoadBoard(CARRIER)
    cu = car.FindFootprintByReference('U1').GetPosition()
    for ref in COPY_PARTS:
        cf = car.FindFootprintByReference(ref)
        p = cf.GetPosition()
        B.place_u(ref, tomm(p.x - cu.x), tomm(p.y - cu.y), cf.GetOrientationDegrees())
    boxes = []
    for ref in COPY_PARTS:
        bb = B.fp(ref).GetBoundingBox(False)
        boxes.append((bb.GetLeft(), bb.GetTop(), bb.GetRight(), bb.GetBottom()))
    dx, dy = mm(OX + B.ux) - cu.x, mm(OY + B.uy) - cu.y
    near = mm(0.3)

    def in_boxes(pt):
        return any(b[0] - near <= pt.x + dx <= b[2] + near and b[1] - near <= pt.y + dy <= b[3] + near for b in boxes)

    n = 0
    for t in car.GetTracks():
        name = t.GetNetname().split('/')[-1]
        if name in COPY_NETS:
            ours = COPY_NETS[name]
            if name in F_ONLY and (isinstance(t, pcbnew.PCB_VIA) or t.GetLayerName() != 'F.Cu'):
                continue
        elif name == 'GND':
            ours = 'GND'
            ends = [t.GetPosition()] if isinstance(t, pcbnew.PCB_VIA) else [t.GetStart(), t.GetEnd()]
            if not all(in_boxes(e) for e in ends):
                continue
        else:
            continue
        rel = (tomm(t.GetStart().x - cu.x), tomm(t.GetStart().y - cu.y))
        if any(math.dist(rel, s) < 0.02 for s in SKIP_TRACKS):
            continue
        c = t.Duplicate()
        c.Move(pcbnew.VECTOR2I(dx, dy))
        c.SetNet(B.net(ours))
        B.b.Add(c)
        n += 1
    return n


def u1_pin(B, num):
    p = [p for p in B.fp('U1').Pads() if p.GetNumber() == str(num)][0].GetPosition()
    return tomm(p.x) - OX - B.ux, tomm(p.y) - OY - B.uy


def pad_u(B, ref, num):
    p = [p for p in B.fp(ref).Pads() if p.GetNumber() == str(num)][0].GetPosition()
    return tomm(p.x) - OX - B.ux, tomm(p.y) - OY - B.uy


def receiver(B):
    T, V = B.track, B.via
    # RF_IN: J1, ESD (D6) and the bias choke (L1) on the line, into the can's upper-left gap, down to C26
    c26 = pad_u(B, 'C26', 1)
    jx, jy = J1_AT[0] - B.ux, J1_AT[1] - B.uy
    T('RF_IN', 'F.Cu', RF_W, [(jx, jy), (-9.8, jy), (c26[0], jy + c26[0] + 9.8), c26])
    # VDD_RF: the carrier's pin-to-cap tracks are copied; the three cap groups and C51/C52/C54 join on In2
    for x, y in ((-3.35, -0.45), (-3.3, 1.2), (-2.29, 3.11), (-5.0, -2.45)):
        V('VDD_RF', x, y)
    T('VDD_RF', 'In2.Cu', 0.3, [(-5.0, -2.45), (-3.35, -0.45), (-3.85, 0.05), (-3.85, 0.75), (-3.3, 1.2), (-3.12, 1.38),
                                (-2.4, 1.38), (-2.23, 1.55), (-2.23, 3.05), (-2.29, 3.11)])
    T('VDD_RF', 'F.Cu', 0.2, [(-7.0, -2.92), (-5.0, -2.92), (-5.0, -2.45)])
    T('VDD_RF', 'F.Cu', 0.15, [(-3.31, -0.4), (-3.35, -0.45)])
    T('VDD_RF', 'F.Cu', 0.2, [(-3.79, 1.2), (-3.3, 1.2)])
    # left side: LDO_ADC_OUT to C55; PPS (31) and GPIO2 (32) to vias; pin 36 to a via, then on B.Cu to R330
    T('Net-(U1-LDO_ADC_OUT)', 'F.Cu', SIG, [u1_pin(B, 33), (-2.95, -1.0), (-3.67, -1.72), pad_u(B, 'C55', 1)])
    T('PX_GPIO2', 'F.Cu', SIG, [u1_pin(B, 32), (-2.85, -1.4), (-3.45, -2.0), (-3.45, -2.15)])
    V('PX_GPIO2', -3.45, -2.15)
    T('PPS', 'F.Cu', SIG, [u1_pin(B, 31), (-2.75, -1.8), (-2.9, -1.95), (-2.9, -2.9)])
    V('PPS', -2.9, -2.9)
    T('PX_NC36', 'F.Cu', SIG, [u1_pin(B, 36), (-2.9, 0.2), (-3.22, 0.15)])
    V('PX_NC36', -3.22, 0.15)
    T('PX_NC36', 'B.Cu', SIG, [(-3.22, 0.15), (-6.35, 0.15), (-6.6, 0.4)])
    V('PX_NC36', -6.6, 0.4)
    T('PX_NC36', 'F.Cu', SIG, [(-6.6, 0.4), pad_u(B, 'R330', 1)])
    V('PX_NC36_LINE', -6.6, -1.6)
    T('PX_NC36_LINE', 'F.Cu', SIG, [pad_u(B, 'R330', 2), (-6.6, -1.6)])
    # top: pins 24-30 straight up through the top gap, out of the can
    for pin, x in TOP_BUNDLE.items():
        px, py = u1_pin(B, pin)
        j = x - px
        T(B.u1net[pin], 'F.Cu', SIG, [(px, py), (px, -3.0), (x, -3.0 - abs(j)), (x, -8.7)])
    # VDD11 (pins 16 and 23) on In2, with C40 at pin 23 and C41 beside it
    T('VDD11', 'F.Cu', 0.15, [u1_pin(B, 23), (1.0, -3.6)])
    V('VDD11', 1.0, -3.6)
    T('VDD11', 'F.Cu', 0.15, [(1.0, -3.6), (1.1, -3.85), pad_u(B, 'C40', 1)])
    V('VDD11', 2.0, -5.0)
    T('VDD11', 'F.Cu', 0.15, [(2.0, -5.0), pad_u(B, 'C41', 1)])
    T('VDD11', 'In2.Cu', 0.2, [(3.4, -0.2), (3.0, -0.6), (2.6, -1.0), (2.6, -2.8), (1.8, -3.6), (1.0, -3.6)])
    T('VDD11', 'In2.Cu', 0.2, [(1.0, -3.6), (2.0, -4.6), (2.0, -5.0)])
    # the buck: V_CORE (22) past C39 and DCDC_OUT (21) out through the upper-right gap to L16/D7; DCDC_IN (20) to
    # C38 and R11
    T('V_CORE', 'F.Cu', 0.2, [u1_pin(B, 22), (1.4, -3.25)])
    T('V_CORE', 'F.Cu', 0.25, [(1.4, -3.25), (6.0, -3.25), (6.0, -4.287), pad_u(B, 'L16', 2)])
    T('V_CORE', 'F.Cu', 0.25, [(2.12, -3.25), pad_u(B, 'C39', 1)])
    T('DCDC_OUT', 'F.Cu', 0.25, [u1_pin(B, 21), (1.8, -2.7), pad_u(B, 'L16', 1)])
    T('DCDC_OUT', 'F.Cu', 0.3, [pad_u(B, 'L16', 1), (8.8, -2.7), pad_u(B, 'D7', 1)])
    T('DCDC_IN', 'F.Cu', 0.25, [u1_pin(B, 20), (2.855, -1.8), (3.1, -2.05), (5.7, -2.05), (6.15, -1.6),
                                pad_u(B, 'C38', 1)])
    T('DCDC_IN', 'F.Cu', 0.3, [pad_u(B, 'C38', 1), (7.0, -1.43), pad_u(B, 'R11', 2)])
    # right side: pins 12-19 to the two via rows
    for pin, (x, y) in RIGHT_VIAS.items():
        T(B.u1net[pin], 'F.Cu', SIG, [u1_pin(B, pin), (x, y)])
        V(B.u1net[pin], x, y)
    # RTC crystal outside the lower-right gap
    T('RTC_XIN', 'F.Cu', 0.15, [u1_pin(B, 11), (2.95, 1.8), (3.9, 2.75), (6.5, 2.75)])
    T('RTC_XOUT', 'F.Cu', 0.15, [u1_pin(B, 10), (1.8, 2.95), (2.25, 3.4), (6.5, 3.4), (6.9, 3.8), pad_u(B, 'Y2', 1)])
    # bottom-right: NC8 and ANT_BIAS to vias, +3V3 (6, 7) to In2, LDO_B_OUT to C42, +3V3 caps C44/C43
    T('PX_NC8', 'F.Cu', SIG, [u1_pin(B, 8), (1.0, 3.45)])
    V('PX_NC8', 1.0, 3.45)
    T('ANT_BIAS', 'F.Cu', 0.15, [u1_pin(B, 9), (1.4, 3.35), (1.9, 3.85), (2.4, 3.85)])
    V('ANT_BIAS', 2.4, 3.85)
    T('+3V3', 'F.Cu', 0.15, [(0.2, 2.75), (0.6, 2.75)])
    T('+3V3', 'F.Cu', 0.2, [(0.4, 2.75), (0.4, 3.35)])
    V('+3V3', 0.4, 3.35)
    T('+3V3', 'In2.Cu', 0.3, [(0.4, 3.35), (0.4, 4.95), (1.65, 4.95), (2.0, 5.3)])
    V('+3V3', 2.0, 5.3)
    T('+3V3', 'F.Cu', 0.25, [(2.0, 5.3), (2.62, 5.3)])
    T('+3V3', 'F.Cu', 0.25, [pad_u(B, 'C44', 1), pad_u(B, 'C43', 1)])
    T('LDO_B_OUT', 'F.Cu', 0.15, [u1_pin(B, 5), (-0.2, 4.3), pad_u(B, 'C42', 1)])


def bias_tee(B):
    """Outside the can on RF_IN: the choke's VANT node, its 1 uF, and the 0R choice of bias source."""
    B.track('VANT', 'F.Cu', 0.3, [pad_u(B, 'L1', 1), pad_u(B, 'C20', 1), pad_u(B, 'R2', 2), pad_u(B, 'R3', 2)])


def courtyard(fp):
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    b = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False)
    return (tomm(b.GetLeft()) - OX, tomm(b.GetTop()) - OY, tomm(b.GetRight()) - OX, tomm(b.GetBottom()) - OY)


def overlaps(a, b, m):
    return not (a[2] + m <= b[0] or b[2] + m <= a[0] or a[3] + m <= b[1] or b[3] + m <= a[1])


def autoplace(B, placed):
    occ = [courtyard(B.fp(r)) for r in placed if r != 'SH1'] + KEEPOUT

    def area(r):
        c = courtyard(B.fp(r))
        return -(c[2] - c[0]) * (c[3] - c[1])
    todo = sorted(ANCHOR, key=lambda r: (area(r), r))
    missing = sorted(f.GetReference() for f in B.b.GetFootprints() if f.GetReference() not in placed | set(ANCHOR))
    assert not missing, missing
    for ref in todo:
        a = [p for p in B.fp(ANCHOR[ref][0]).Pads() if p.GetNumber() == ANCHOR[ref][1]][0].GetPosition()
        ax, ay = tomm(a.x) - OX, tomm(a.y) - OY
        fp = B.fp(ref)
        done = False
        for rad in [k * 0.2 for k in range(0, 200)]:
            n = max(1, int(2 * math.pi * rad / 0.2))
            for k in range(n):
                th = 2 * math.pi * k / n
                x, y = round((ax + rad * math.cos(th)) * 20) / 20, round((ay + rad * math.sin(th)) * 20) / 20
                for rot in (0, 90, 180, 270):
                    B.place_a(ref, x, y, rot)
                    c = courtyard(fp)
                    if c[0] < 0.5 or c[1] < 0.5 or c[2] > W - 0.5 or c[3] > H - 0.5:
                        continue
                    if any(overlaps(c, q, 0.2) for q in occ):
                        continue
                    occ.append(c)
                    done = True
                    break
                if done:
                    break
            if done:
                break
        if not done:
            print('could not place', ref)


# exposed pads: their only ground, so vias in the pad (the board orders filled and capped vias for U1 anyway, which
# makes in-pad vias flat; the owner's rule allows them in QFN ground pads). Offsets from the pad centre, mm.
EP_VIAS = {
    ('U1', '41'): [(0, 0), (1.1, 0), (-1.1, 0), (0, 1.1), (0, -1.1)],                    # as the carrier
    ('U201', '57'): [(x, y) for x in (-1.2, 0, 1.2) for y in (-1.2, 0, 1.2)],
    ('U301', '25'): [(0, 0), (0.65, 0.65), (-0.65, 0.65), (0.65, -0.65), (-0.65, -0.65)],
    ('U104', '9'): [(0.62, 0.62), (-0.62, 0.62), (0.62, -0.62), (-0.62, -0.62), (0, 1.25), (0, -1.25)],
}


def pad_a(B, ref, num):
    p = [p for p in B.fp(ref).Pads() if p.GetNumber() == str(num)][0].GetPosition()
    return tomm(p.x) - OX, tomm(p.y) - OY


def esp_extras(B):
    """ESP32 pin 20 (VDD3P3_RTC) drops straight to a via: its bottom-side neighbours fan out to the links, so its
    100 nF (C206) joins it on the inner layers. J301's ground pins join the pours solid (hand-soldered header, and the
    In2 reliefs were starved). S101's unused pads keep tracks clear of their mask openings."""
    p = [p for p in B.fp('U201').Pads() if p.GetNumber() == '20'][0].GetPosition()
    x, y = tomm(p.x) - OX, tomm(p.y) - OY
    B.track('+3V3_MCU', 'F.Cu', 0.2, [(x, y), (x, y + 0.95)], rel=False)
    B.via('+3V3_MCU', x, y + 0.95, rel=False)
    B.track('+3V3_MCU', 'F.Cu', 0.2, [(x, y + 0.95), pad_a(B, 'C206', 1)], rel=False)
    # J101's ground pins (A1/B12, B1/A12) each get a via: the CC and VBUS tracks box their pour in
    for num, dy in (('A1/B12', 0.55), ('B1/A12', -0.55)):
        gx, gy = pad_a(B, 'J101', num)
        B.via('GND', gx - 1.2, gy + dy, rel=False)
        B.track('GND', 'F.Cu', 0.3, [(gx, gy), (gx - 1.2, gy + dy)], rel=False)
    for ref, nums in (('J301', ('19', '20')), ('J201', ('1',))):
        for pd in B.fp(ref).Pads():
            if pd.GetNumber() in nums:
                pd.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
    for pd in B.fp('S101').Pads():
        if pd.GetNetname().startswith('unconnected'):
            pd.SetLocalClearance(mm(0.25))


def ep_vias(B):
    for (ref, num), offs in EP_VIAS.items():
        p = [p for p in B.fp(ref).Pads() if p.GetNumber() == num][0].GetPosition()
        x, y = tomm(p.x) - OX, tomm(p.y) - OY
        for dx, dy in offs:
            B.via('GND', x + dx, y + dy, rel=False)


def silk(B):
    """Board texts (part labels stay hidden, the owner's rule): the title, J301's and J201's pin names, the buttons and
    the USB switch's two ends. make_pcb.py put them by the old floorplan, so they are redrawn from the pads here."""
    import make_pcb
    for t in list(B.b.Drawings()):
        if isinstance(t, pcbnew.PCB_TEXT) and t.GetLayer() == pcbnew.F_SilkS:
            B.b.Remove(t)

    def text(s, x, y, just=0, size=0.8, th=0.15, angle=0):
        t = pcbnew.PCB_TEXT(B.b)
        t.SetText(s)
        t.SetPosition(B.a(x, y))
        t.SetLayer(pcbnew.F_SilkS)
        t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
        t.SetTextThickness(mm(th))
        t.SetTextAngleDegrees(angle)
        t.SetHorizJustify({-1: pcbnew.GR_TEXT_H_ALIGN_RIGHT, 0: pcbnew.GR_TEXT_H_ALIGN_CENTER,
                           1: pcbnew.GR_TEXT_H_ALIGN_LEFT}[just])
        B.b.Add(t)

    def pad(ref, num):
        p = [p for p in B.fp(ref).Pads() if p.GetNumber() == str(num)][0].GetPosition()
        return tomm(p.x) - OX, tomm(p.y) - OY

    text('PX100 TEST ${REVISION}', 42.5, 4.0, 0, 1.2, 0.18)
    # J301 runs along the top edge: even pins (upper row) named above, odd pins (lower row) below, reading upwards
    for n, name in make_pcb.J301_NAMES.items():
        x, y = pad('J301', n)
        text(name, x, y - 1.6, 1, angle=90) if n % 2 == 0 else text(name, x, y + 1.6, -1, angle=90)
    for k, name in enumerate(make_pcb.J201_NAMES):
        x, y = pad('J201', k + 1)
        text(name, x, y - 2.1, 0)
    for ref, name in make_pcb.BUTTONS.items():
        c = courtyard(B.fp(ref))
        text(name, (c[0] + c[2]) / 2, c[3] + 0.9, 0)
    # S101: SEL0 to ground (port 2, the ESP32) at its pin-7 end, as the Beetle's "F" end; open at the pin-5 end
    e7 = [(a + c) / 2 for a, c in zip(pad('S101', 3), pad('S101', 7))]
    e5 = [(a + c) / 2 for a, c in zip(pad('S101', 1), pad('S101', 5))]
    mid = [(a + c) / 2 for a, c in zip(e7, e5)]
    for e, name in ((e7, 'ESP'), (e5, 'PX100')):
        d = [a - c for a, c in zip(e, mid)]
        L = math.hypot(*d)
        text(name, e[0] + d[0] / L * 2.6, e[1] + d[1] / L * 2.6, 1 if d[0] > 0.1 else (-1 if d[0] < -0.1 else 0))
    text('USB', mid[0], mid[1] + 3.3, 0)


def main():
    B = Board()
    B.place_u('U1', 0, 0, 90)
    B.place_u('SH1', SH_OFF[0], SH_OFF[1], 0)
    B.u1net = {int(p.GetNumber()): p.GetNetname().split('/')[-1] for p in B.fp('U1').Pads() if p.GetNumber()}
    n = copy_carrier(B)
    for ref, (dx, dy, rot) in REL.items():
        B.place_u(ref, dx, dy, rot)
    for ref, (x, y, rot) in ABS.items():
        B.place_a(ref, x, y, rot)
    placed = set(COPY_PARTS) | set(REL) | set(ABS) | {'U1', 'SH1'}
    autoplace(B, placed)
    receiver(B)
    bias_tee(B)
    ep_vias(B)
    esp_extras(B)
    silk(B)
    pcbnew.SaveBoard(PCB, B.b, True)     # True: leave the .kicad_pro alone
    print('copied', n, 'carrier tracks and vias; wrote', os.path.basename(PCB))


if __name__ == '__main__':
    main()
