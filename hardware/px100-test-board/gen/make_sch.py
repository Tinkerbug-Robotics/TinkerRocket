"""Draws design.py as a KiCad 10 hierarchical schematic: a root sheet and one sub-sheet per entry in
design.SHEETS. Every pin gets a short wire and a label (or a power symbol); nets that cross sheets use
global labels. UUIDs are derived from the reference designators, so a rerun keeps the PCB links.

Run:  python3 make_sch.py   (then kicad-cli sch erc px100-test-board.kicad_sch)

The hand-drawn receiver sheet (design.STATIC_SHEETS) is only linked from the root here, never rewritten.
"""
import datetime
import os
import uuid

import design
import kilib
from kilib import Q, dump, find, get

NAME = 'px100-test-board'
OUT = kilib.PROJ
NS = uuid.UUID('3b7d2a54-91c6-4f0e-8a3d-6c2e1f9b7a15')
DATE = datetime.date(2026, 10, 28).isoformat()
G = 2.54
POS_RAILS = ['VBUS', '+3V3', '+3V3_MCU']
POWER_SYM = {'GND': 'power:GND', 'VBUS': 'power:VBUS', '+3V3': 'power:+3V3'}
TWO_PIN = {'Device:R', 'Device:C', 'Device:L', 'Device:LED', 'Device:D_Schottky', 'Device:D_Zener',
           'Device:Fuse', 'Device:Polyfuse', 'Device:C_Polarized', 'Connector:TestPoint'}


def ustr(*key):
    return str(uuid.uuid5(NS, '/'.join(str(k) for k in key)))


def uid(*key):
    """A (uuid "...") node."""
    return ['uuid', Q(ustr(*key))]


ROOT_UUID = ustr('root')
SHEET_UUID = {k: ustr('sheet', k) for k, _, _ in design.SHEETS + design.STATIC_SHEETS}
# power-symbol reference prefix per sheet ('power' and 'pxio' share a first letter)
SHEET_CODE = {k: chr(ord('A') + i) for i, (k, _, _) in enumerate(design.SHEETS)}


def snap(v):
    return round(round(v / 1.27) * 1.27, 4)


def xform(px, py, rot):
    """Library offset (y up) to schematic offset (y down) for a symbol rotated rot degrees (CCW)."""
    x, y = px, -py
    for _ in range((rot // 90) % 4):
        x, y = y, -x
    return x, y


def font(size=1.27, bold=False):
    f = ['font', ['size', size, size]]
    if bold:
        f.append(['bold', 'yes'])
    return f


def eff(justify=None, size=1.27, hide=False, bold=False):
    e = ['effects', font(size, bold)]
    if justify:
        e.append(['justify'] + justify.split())
    return e


def prop(name, value, x, y, ang=0, justify=None, hide=False):
    p = ['property', Q(name), Q(value), ['at', x, y, ang]]
    if hide:
        p.append(['hide', 'yes'])
    p += [['show_name', 'no'], ['do_not_autoplace', 'no'], eff(justify)]
    return p


class Sheet:
    def __init__(self, key, title):
        self.key, self.title = key, title
        self.items = []
        self.libs = {}
        self.npwr = 0

    def lib(self, lib_id):
        if lib_id not in self.libs:
            self.libs[lib_id] = kilib.symbol(lib_id)

    def wire(self, a, b):
        self.items.append(['wire', ['pts', ['xy', a[0], a[1]], ['xy', b[0], b[1]]],
                           ['stroke', ['width', 0], ['type', 'default']], uid(self.key, 'w', len(self.items))])

    def label(self, net, at, direction, glob):
        ang = {(1, 0): 0, (-1, 0): 180, (0, -1): 90, (0, 1): 270}[direction]
        just = 'left' if ang in (0, 90) else 'right'
        if glob:
            self.items.append(['global_label', Q(net), ['shape', 'passive'], ['at', at[0], at[1], ang],
                               ['fields_autoplaced', 'yes'], eff(just), uid(self.key, 'gl', len(self.items)),
                               ['property', Q('Intersheetrefs'), Q('${INTERSHEET_REFS}'), ['at', at[0], at[1], 0],
                                ['hide', 'yes'], ['show_name', 'no'], ['do_not_autoplace', 'no'], eff(just)]])
        else:
            self.items.append(['label', Q(net), ['at', at[0], at[1], ang], eff(just + ' bottom'),
                               uid(self.key, 'l', len(self.items))])

    def power(self, net, at, direction):
        """Power symbol whose body points along direction (away from the pin)."""
        lib_id = POWER_SYM[net] if net in POWER_SYM else 'power:PWR_FLAG'
        self.lib(lib_id)
        if net == 'GND' or lib_id == 'power:PWR_FLAG':
            rot = {(0, 1): 0, (1, 0): 90, (0, -1): 180, (-1, 0): 270}[direction]
        else:
            rot = {(0, -1): 0, (-1, 0): 90, (0, 1): 180, (1, 0): 270}[direction]
        self.npwr += 1
        ref = ('#FLG%s%02d' if lib_id == 'power:PWR_FLAG' else '#PWR%s%02d') % (SHEET_CODE[self.key], self.npwr)
        val = 'PWR_FLAG' if lib_id == 'power:PWR_FLAG' else net
        k = (self.key, 'pwr', self.npwr)
        self.items.append(
            ['symbol', ['lib_id', Q(lib_id)], ['at', at[0], at[1], rot], ['unit', 1], ['body_style', 1],
             ['exclude_from_sim', 'no'], ['in_bom', 'yes'], ['on_board', 'yes'], ['in_pos_files', 'yes'], ['dnp', 'no'],
             uid(*k),
             prop('Reference', ref, at[0], at[1], hide=True),
             prop('Value', val, at[0] + (1.5 if rot in (90, 270) else 0),
                  at[1] + {0: 3.2 if net == 'GND' else -3.2, 180: -3.2 if net == 'GND' else 3.2}.get(rot, 0),
                  justify={90: 'left', 270: 'right'}.get(rot), hide=(lib_id == 'power:PWR_FLAG' and False)),
             prop('Footprint', '', at[0], at[1], hide=True), prop('Datasheet', '', at[0], at[1], hide=True),
             prop('Description', '', at[0], at[1], hide=True),
             ['pin', Q('1'), uid(*k, 'pin')],
             ['instances', ['project', Q(NAME), ['path', Q('/%s/%s' % (ROOT_UUID, SHEET_UUID[self.key])),
                                                  ['reference', Q(ref)], ['unit', 1]]]]])

    def text(self, s, at, size=2.54, bold=True):
        self.items.append(['text', Q(s), ['exclude_from_sim', 'no'], ['at', at[0], at[1], 0],
                           eff('left bottom', size, bold=bold), uid(self.key, 't', len(self.items))])


GLOBAL = set()


def net_sheets():
    m = {}
    for p in design.PARTS:
        for n in p.pins.values():
            if n:
                m.setdefault(n, set()).add(p.sheet)
    return {n for n, s in m.items() if (len(s) > 1 or n in design.RECEIVER_NETS) and n not in POWER_SYM}


def place_unit(p, unit):
    """Choose a rotation and return (rot, [(pin_num, net, dx, dy, dirx, diry)], bbox) relative to the origin."""
    pins = kilib.pins(p.lib, unit)
    rots = [0]
    if p.lib in TWO_PIN and len(pins) == 2:
        rots = [0, 90, 180, 270]
    best = None
    for rot in rots:
        placed = []
        for num, nm, typ, x, y, a, ln in pins:
            dx, dy = xform(x, y, rot)
            # stub points away from the body: opposite to the pin's direction
            import math
            vx, vy = xform(round(math.cos(math.radians(a))), round(math.sin(math.radians(a))), rot)
            placed.append((num, p.pins.get(num, 'NC?'), snap(dx), snap(dy), -vx, -vy))
        if p.lib in TWO_PIN and len(pins) == 2:
            if placed[0][2] != placed[1][2]:
                continue  # not vertical
            top, bot = sorted(placed, key=lambda q: q[3])
            score = 0
            if bot[1] == 'GND':
                score += 4
            if top[1] in POS_RAILS:
                score += 2
            if top[0] == '1' or top[0] == 'A':
                score += 1
            if top[1] == 'GND':
                score -= 4
            if best is None or score > best[0]:
                best = (score, rot, placed)
        else:
            best = (0, rot, placed)
            break
    _, rot, placed = best
    x0, y0, x1, y1 = kilib.bbox(p.lib, unit)
    c = [xform(x0, y0, rot), xform(x1, y1, rot)]
    bb = (min(c[0][0], c[1][0]), min(c[0][1], c[1][1]), max(c[0][0], c[1][0]), max(c[0][1], c[1][1]))
    return rot, placed, bb


def extent(placed, bb, two_pin):
    """Space the unit takes, including stubs, labels and the ref/value text."""
    xs = [bb[0], bb[2]]
    ys = [bb[1], bb[3]]
    for num, net, dx, dy, vx, vy in placed:
        L = 2.54
        if net is None:
            ex, ey = dx, dy
        elif net in POWER_SYM:
            ex, ey = dx + vx * (L + 4), dy + vy * (L + 4)
        else:
            tl = len(net) * 1.1 + (3.5 if net in GLOBAL else 0.8)
            ex, ey = dx + vx * (L + tl), dy + vy * (L + tl)
        xs += [dx, ex]
        ys += [dy, ey]
    if two_pin:
        xs.append(bb[2] + 12)
    else:
        ys += [bb[1] - 3, bb[3] + 3]
        xs.append(bb[0] + 20)
    return min(xs), min(ys), max(xs), max(ys)


def emit_unit(sh, p, unit, rot, placed, at):
    sh.lib(p.lib)
    X, Y = at
    two_pin = p.lib in TWO_PIN
    x0, y0, x1, y1 = kilib.bbox(p.lib, unit)
    c = [xform(x0, y0, rot), xform(x1, y1, rot)]
    bb = (min(c[0][0], c[1][0]), min(c[0][1], c[1][1]), max(c[0][0], c[1][0]), max(c[0][1], c[1][1]))
    if two_pin:
        # field justification is mirrored with the symbol; text angle turns with it
        j = 'right' if rot in (180, 90) else 'left'
        refat = (X + bb[2] + 1.0, Y - 1.27, j)
        valat = (X + bb[2] + 1.0, Y + 1.27, j)
    else:
        refat = (X + bb[0], Y + bb[1] - 1.5, 'left')
        valat = (X + bb[0], Y + bb[3] + 2.5, 'left')
    key = (p.sheet, p.ref, unit)
    sym = ['symbol', ['lib_id', Q(p.lib)], ['at', X, Y, rot], ['unit', unit], ['body_style', 1],
           ['exclude_from_sim', 'no'],
           ['in_bom', 'no' if p.ref.startswith(('H', 'FID', 'TP')) else 'yes'], ['on_board', 'yes'],
           ['in_pos_files', 'no' if p.ref.startswith(('H', 'FID', 'TP')) else 'yes'],
           ['dnp', 'yes' if p.dnp else 'no'], uid(*key),
           prop('Reference', p.ref, refat[0], refat[1], ang=(90 if rot in (90, 270) else 0), justify=refat[2]),
           prop('Value', p.value, valat[0], valat[1], ang=(90 if rot in (90, 270) else 0), justify=valat[2]),
           prop('Footprint', p.fp, X, Y, hide=True),
           prop('Datasheet', '', X, Y, hide=True),
           prop('Description', p.desc, X, Y, hide=True),
           prop('MPN', p.mpn, X, Y, hide=True),
           prop('Mfr', p.mfr, X, Y, hide=True),
           prop('LCSC', LCSC.get(p.mpn, ''), X, Y, hide=True)]
    for num, nm, typ, x, y, a, ln in kilib.pins(p.lib, unit):
        sym.append(['pin', Q(num), uid(*key, 'pin', num)])
    sym.append(['instances', ['project', Q(NAME), ['path', Q('/%s/%s' % (ROOT_UUID, SHEET_UUID[p.sheet])),
                                                    ['reference', Q(p.ref)], ['unit', unit]]]])
    sh.items.append(sym)
    # runs of neighboring pins on the same side and the same rail share one power symbol
    shared = set()
    if not two_pin:
        by_side = {}
        for q in placed:
            by_side.setdefault((q[4], q[5]), []).append(q)
        for (vx, vy), qs in by_side.items():
            along = (lambda q: q[2]) if vy != 0 else (lambda q: q[3])
            qs.sort(key=along)
            run = [qs[0]]
            for q in qs[1:] + [None]:
                if q is not None and q[1] in POWER_SYM and q[1] == run[-1][1] and abs(along(q) - along(run[-1])) < 2.6:
                    run.append(q)
                    continue
                if len(run) > 1 and run[0][1] in POWER_SYM:
                    for a, c in zip(run, run[1:]):
                        ea = (snap(X + a[2] + a[4] * G), snap(Y + a[3] + a[5] * G))
                        ec = (snap(X + c[2] + c[4] * G), snap(Y + c[3] + c[5] * G))
                        sh.wire(ea, ec)
                    for c in run[1:]:
                        shared.add(c[0])
                run = [q] if q is not None else []
    for num, net, dx, dy, vx, vy in placed:
        px, py = snap(X + dx), snap(Y + dy)
        if net is None:
            sh.items.append(['no_connect', ['at', px, py], uid(*key, 'nc', num)])
            continue
        if net == 'NC?':
            raise SystemExit('%s pin %s is not in design.py' % (p.ref, num))
        ex, ey = snap(px + vx * G), snap(py + vy * G)
        sh.wire((px, py), (ex, ey))
        if num in shared:
            continue
        if net in POWER_SYM:
            sh.power(net, (ex, ey), (vx, vy))
        else:
            sh.label(net, (ex, ey), (vx, vy), net in GLOBAL)


LCSC = {}


def build_sheet(key, title, desc):
    sh = Sheet(key, title)
    parts = [p for p in design.PARTS if p.sheet == key]
    blocks = []
    for p in parts:
        if p.block not in blocks:
            blocks.append(p.block)
    papers = [('A3', 420, 297), ('A2', 594, 420), ('A1', 841, 594), ('A0', 1189, 841)]
    for paper, W, H in papers:
        layout = []
        y = 30.0
        ok = True
        for blk in blocks:
            items = []
            for p in parts:
                if p.block != blk:
                    continue
                for u in p.units:
                    rot, placed, bb = place_unit(p, u)
                    ext = extent(placed, bb, p.lib in TWO_PIN)
                    items.append((p, u, rot, placed, ext))
            items.sort(key=lambda it: (it[0].lib in TWO_PIN, it[0].lib == 'Connector:TestPoint'))
            title_y = y
            y += 8
            x = 15.0
            rowh = 0
            row = []
            for it in items:
                ext = it[4]
                w = ext[2] - ext[0] + 4
                h = ext[3] - ext[1] + 4
                if x + w > W - 15 and row:
                    y += rowh
                    x = 15.0
                    rowh = 0
                    row = []
                ox = snap(x - ext[0])
                oy = snap(y - ext[1])
                layout.append((it, (ox, oy)))
                row.append(it)
                x += w
                rowh = max(rowh, h)
            y += rowh + 6
            layout.append((('title', blk), (15.0, title_y + 4)))
            if y > H - 40:
                ok = False
                break
        if ok:
            break
    sh.paper = paper
    for it, at in layout:
        if it[0] == 'title':
            sh.text(it[1], at)
        else:
            p, u, rot, placed, ext = it
            emit_unit(sh, p, u, rot, placed, at)
    # PWR_FLAGs, in a row at the bottom
    fx = 20.0
    fy = snap(H - 30)
    sh.text('Power flags (ERC: these nets are fed through passive parts)', (15.0, fy - 10), 1.8, False)
    for net in design.PWR_FLAG_NETS[key]:
        a = (snap(fx), fy)
        b = (snap(fx), snap(fy + G))
        sh.power('PWR_FLAG', a, (0, -1)) if False else None
        # flag at a, wire down to b, then the net there
        sh.lib('power:PWR_FLAG')
        sh.npwr += 1
        ref = '#FLG%s%02d' % (SHEET_CODE[key], sh.npwr)
        k = (key, 'flg', net)
        sh.items.append(
            ['symbol', ['lib_id', Q('power:PWR_FLAG')], ['at', a[0], a[1], 0], ['unit', 1], ['body_style', 1],
             ['exclude_from_sim', 'no'], ['in_bom', 'yes'], ['on_board', 'yes'], ['in_pos_files', 'yes'],
             ['dnp', 'no'], uid(*k), prop('Reference', ref, a[0], a[1], hide=True),
             prop('Value', 'PWR_FLAG', a[0], a[1] - 3.5), prop('Footprint', '', a[0], a[1], hide=True),
             prop('Datasheet', '', a[0], a[1], hide=True), prop('Description', '', a[0], a[1], hide=True),
             ['pin', Q('1'), uid(*k, 'pin')],
             ['instances', ['project', Q(NAME), ['path', Q('/%s/%s' % (ROOT_UUID, SHEET_UUID[key])),
                                                  ['reference', Q(ref)], ['unit', 1]]]]])
        sh.wire(a, b)
        if net in POWER_SYM:
            sh.power(net, b, (0, 1))
        else:
            sh.label(net, b, (0, 1), net in GLOBAL)
        fx += 25
    return sh


def write_sheet(sh, desc):
    root = ['kicad_sch', ['version', 20260306], ['generator', Q('eeschema')], ['generator_version', Q('10.0')],
            uid('file', sh.key), ['paper', Q(sh.paper)],
            ['title_block', ['title', Q(sh.title)], ['date', Q(DATE)], ['rev', Q('${REVISION}')],
             ['company', Q('TinkerRocket')], ['comment', 1, Q(desc)],
             ['comment', 2, Q('Generated by gen/make_sch.py from gen/design.py')]],
            ['lib_symbols'] + [sh.libs[k] for k in sorted(sh.libs)]]
    root += sh.items
    root.append(['embedded_fonts', 'no'])
    open(os.path.join(OUT, sh.key + '.kicad_sch'), 'w').write(dump(root) + '\n')


def write_root():
    items = []
    x = 25.4
    for i, (key, title, desc) in enumerate(design.SHEETS + design.STATIC_SHEETS):
        y = 50.8 + i * 40.64
        items.append(
            ['sheet', ['at', x, y], ['size', 101.6, 25.4], ['exclude_from_sim', 'no'], ['in_bom', 'yes'],
             ['on_board', 'yes'], ['dnp', 'no'], ['fields_autoplaced', 'yes'],
             ['stroke', ['width', 0.1524], ['type', 'solid']], ['fill', ['color', 0, 0, 0, 0]], ['uuid', Q(SHEET_UUID[key])],
             ['property', Q('Sheetname'), Q(title), ['at', x, y - 1.3, 0], ['show_name', 'no'],
              ['do_not_autoplace', 'no'], ['effects', ['font', ['size', 2.5, 2.5], ['thickness', 0.6], ['bold', 'yes']],
                                           ['justify', 'left', 'bottom']]],
             ['property', Q('Sheetfile'), Q(key + '.kicad_sch'), ['at', x, y + 26.0, 0], ['show_name', 'no'],
              ['do_not_autoplace', 'no'], ['effects', ['font', ['size', 1.27, 1.27]], ['justify', 'left', 'top']]],
             ['instances', ['project', Q(NAME), ['path', Q('/%s' % ROOT_UUID), ['page', Q(str(i + 2))]]]]])
        items.append(['text', Q(desc), ['exclude_from_sim', 'no'], ['at', x + 106, y + 12.7, 0],
                      eff('left bottom', 1.8), uid('root', 'desc', key)])
    notes = [
        'PX100 test board ${REVISION}: a PX100 with an active-antenna L1/L5 front end, every PX100 pin on an ESP32-S3 GPIO,',
        'and one USB-C port that S101 switches between the ESP32 and a CP2102N bridge to the PX100\'s UART.',
        'Powered from USB: an AP2112K for the ESP32 side, the carrier\'s ADP7142 for the PX100 (the ESP32 can switch it off).',
        'Reused: gnss-px100-18mm-hv RF chain, PX100 core and LDO; spin-rig ESP32-S3 circuit (an FH4R2, as on the Beetle JLC build) and USB-C; the Beetle\'s FSUSB63 mux.',
        'receiver.kicad_sch is drawn by hand; the other three sheets come from gen/design.py (gen/make_sch.py).',
    ]
    for i, n in enumerate(notes):
        items.append(['text', Q(n), ['exclude_from_sim', 'no'], ['at', 25.4, 25.4 + i * 4.5, 0],
                      eff('left bottom', 1.8), uid('root', 'note', i)])
    root = ['kicad_sch', ['version', 20260306], ['generator', Q('eeschema')], ['generator_version', Q('10.0')],
            ['uuid', Q(ROOT_UUID)], ['paper', Q('A4')],
            ['title_block', ['title', Q('PX100 Test Board')], ['date', Q(DATE)], ['rev', Q('${REVISION}')],
             ['company', Q('TinkerRocket')], ['comment', 1, Q('PX100 bring-up, GPIO discovery and flashing')],
             ['comment', 2, Q('Generated by gen/make_sch.py from gen/design.py')]],
            ['lib_symbols']] + items + [
        ['sheet_instances', ['path', Q('/'), ['page', Q('1')]]], ['embedded_fonts', 'no']]
    open(os.path.join(OUT, NAME + '.kicad_sch'), 'w').write(dump(root) + '\n')


def main():
    import json
    global GLOBAL
    GLOBAL = net_sheets()
    lf = os.path.join(kilib.HERE, 'lcsc.json')
    if os.path.exists(lf):
        LCSC.update(json.load(open(lf)))
    for key, title, desc in design.SHEETS:
        sh = build_sheet(key, title, desc)
        write_sheet(sh, desc)
        print(key, sh.paper, len(sh.items), 'items')
    write_root()


if __name__ == '__main__':
    main()
