"""Draws the GVLB258.A patch test board schematic (one sheet): the patch AE1 wired to the SMA jack J1,
two M2 mounting holes on ground. UUIDs come from the reference designators, so a rerun keeps the PCB
links.

Run:  python3 make_sch.py   (then kicad-cli sch erc ../gvlb258-patch-test.kicad_sch)
"""
import os
import uuid

import kilib
from kilib import Q, dump

NAME = 'gvlb258-patch-test'
OUT = os.path.join(kilib.PROJ, NAME + '.kicad_sch')
NS = uuid.UUID('9a4e2c71-5b3d-4f8a-b6e0-1c7d3f2a9e54')
TITLE = 'GVLB258.A patch test board'
REV = 'V1'
DATE = '2026-11-27'


def ustr(*key):
    return str(uuid.uuid5(NS, '/'.join(str(k) for k in key)))


ROOT = ustr('root')


def eff(size=1.27, justify=None, hide=False):
    e = ['effects', ['font', ['size', size, size]]]
    if justify:
        e.append(['justify'] + justify.split())
    if hide:
        e.append(['hide', 'yes'])
    return e


def prop(name, value, x, y, hide=False, angle=0, justify=None):
    p = ['property', Q(name), Q(value), ['at', x, y, angle]]
    if hide:
        p.append(['hide', 'yes'])
    p += [['show_name', 'no'], ['do_not_autoplace', 'no'], eff(justify=justify)]
    return p


def place(lib_id, ref, x, y, fields, angle=0, in_bom=True, ref_at=None, val_at=None, val_hide=False,
          ref_hide=False):
    """A symbol instance; fields = [(name, value)] after Reference/Value."""
    pins = kilib.pins(lib_id)
    rx, ry = ref_at or (x + 3.81, y - 1.27)
    vx, vy = val_at or (x + 3.81, y + 1.27)
    node = ['symbol', ['lib_id', Q(lib_id)], ['at', x, y, angle], ['unit', 1], ['body_style', 1],
            ['exclude_from_sim', 'no'], ['in_bom', 'yes' if in_bom else 'no'], ['on_board', 'yes'],
            ['in_pos_files', 'yes' if in_bom else 'no'], ['dnp', 'no'], ['uuid', Q(ustr('sym', ref))],
            prop('Reference', ref, rx, ry, hide=ref_hide, justify='left'),
            prop('Value', dict(fields).get('_value', ''), vx, vy, hide=val_hide, justify='left')]
    for k, v in fields:
        if k == '_value':
            continue
        node.append(prop(k, v, x, y, hide=True))
    for num, *_ in pins:
        node.append(['pin', Q(num), ['uuid', Q(ustr('pin', ref, num))]])
    node.append(['instances', ['project', Q(NAME), ['path', Q('/' + ROOT), ['reference', Q(ref)], ['unit', 1]]]])
    return node


def pin_xy(lib_id, x, y, num):
    for n, _, _, px, py, _, _ in kilib.pins(lib_id):
        if n == num:
            return round(x + px, 4), round(y - py, 4)
    raise KeyError(num)


def wire(*pts):
    out = []
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        out.append(['wire', ['pts', ['xy', x1, y1], ['xy', x2, y2]], ['stroke', ['width', 0], ['type', 'default']],
                    ['uuid', Q(ustr('wire', x1, y1, x2, y2))]])
    return out


_pwr = [0]


def gnd(x, y):
    _pwr[0] += 1
    ref = '#PWR%02d' % _pwr[0]
    return place('power:GND', ref, x, y, [('_value', 'GND'), ('Footprint', ''), ('Datasheet', ''),
                                          ('Description', 'Power symbol')],
                 ref_at=(x, y + 6.35), val_at=(x, y + 3.81), ref_hide=True)


def text(s, x, y, size=1.27):
    return ['text', Q(s), ['exclude_from_sim', 'no'], ['at', x, y, 0], eff(size, 'left top'),
            ['uuid', Q(ustr('text', s[:40]))]]


items = []
# AE1: feed pin at its origin, body above it
AE = (63.5, 76.2)
items.append(place('Custom:GVLB258.A', 'AE1', *AE, [
    ('_value', 'GVLB258.A'),
    ('Footprint', 'Footprints:ANT_GVLB258.A_TAOGLAS'),
    ('Datasheet', 'https://www.taoglas.com/datasheets/GVLB258.A.pdf'),
    ('Description', 'GNSS L1/L5 single-feed stacked ceramic patch antenna (GPS L1/L5, Galileo E1/E5a, BeiDou '
                    'B1/B2a, GLONASS G1, NavIC L5), 25 x 25 x 8.12 mm, feed pin 2.5 mm off centre'),
    ('MPN', 'GVLB258.A'), ('Mfr', 'Taoglas')],
    ref_at=(AE[0] + 5.08, AE[1] - 8.89), val_at=(AE[0] + 5.08, AE[1] - 6.35)))
# J1: SMA jack, centre pin on the left
J = (93.98, 86.36)
items.append(place('Connector:Conn_Coaxial', 'J1', *J, [
    ('_value', 'RFPC-SMA28-F'),
    ('Footprint', 'Footprints:SMA_GradConn_RFPC-SMA28-F_Vertical'),
    ('Datasheet', 'https://www.digikey.com/en/products/detail/gct/RFPC-SMA28-F/22162141'),
    ('Description', 'SMA jack (female), straight, through-hole, 50 ohm; fitted on the back, pointing away '
                    'from the patch'),
    ('MPN', 'RFPC-SMA28-F'), ('Mfr', 'GCT (GradConn)')],
    ref_at=(J[0] + 2.54, J[1] - 3.81), val_at=(J[0] + 2.54, J[1] - 1.27)))
feed = pin_xy('Custom:GVLB258.A', *AE, '1')
jin = pin_xy('Connector:Conn_Coaxial', *J, '1')
jext = pin_xy('Connector:Conn_Coaxial', *J, '2')
items += wire(feed, (feed[0], jin[1]), jin)
items.append(['label', Q('RF_FEED'), ['at', 71.12, jin[1], 0], eff(justify='left bottom'),
              ['uuid', Q(ustr('label', 'RF_FEED'))]])
items += wire(jext, (jext[0], jext[1] + 2.54))
items.append(gnd(jext[0], jext[1] + 2.54))
# PWR_FLAG on GND (the board has no supply; it marks the ground net as driven for ERC)
flag = (jext[0] + 7.62, jext[1] + 2.54)
items += wire((jext[0], jext[1] + 2.54), flag)
items.append(place('power:PWR_FLAG', '#FLG01', *flag, [('_value', 'PWR_FLAG'), ('Footprint', ''), ('Datasheet', ''),
                                                       ('Description', 'Special symbol for telling ERC where '
                                                                       'power comes from')],
                   ref_at=(flag[0], flag[1] - 3.81), val_at=(flag[0], flag[1] - 3.81), ref_hide=True))

# H1, H2: M2 holes, pad to ground
for i, x in enumerate((63.5, 78.74), 1):
    ref = 'H%d' % i
    y = 109.22
    items.append(place('Mechanical:MountingHole_Pad', ref, x, y, [
        ('_value', 'MountingHole_Pad'),
        ('Footprint', 'MountingHole:MountingHole_2.2mm_M2_DIN965_Pad_TopBottom'),
        ('Datasheet', ''),
        ('Description', 'M2 mounting hole at the SMA end, 2.40 mm from the side edge as on the Tinker-Beetle; '
                        'pad to GND')],
        in_bom=False, ref_at=(x + 2.54, y - 2.54), val_at=(x + 2.54, y), val_hide=True))
    p = pin_xy('Mechanical:MountingHole_Pad', x, y, '1')
    items += wire(p, (p[0], p[1] + 2.54))
    items.append(gnd(p[0], p[1] + 2.54))

items.append(text(
    'GVLB258.A patch test board: the Tinker-Beetle V2 antenna (AE1) on a board as wide as the Beetle\n'
    '(26.83 mm), with a vertical SMA jack on the back for the PX100 carrier (gnss-px100-18mm-hv) J1.\n'
    'Passive: no amplifier, no matching parts. The carrier already has the Beetle feed network\n'
    '(C81, D6, C20, C82), so with JP1 open the chain matches the Beetle from the patch pin on.\n'
    'RF_FEED is a 50 ohm grounded coplanar line on the back, straight from the feed pin to J1.',
    40.64, 134.62))

sch = ['kicad_sch', ['version', 20260306], ['generator', Q('eeschema')], ['generator_version', Q('10.0')],
       ['uuid', Q(ROOT)], ['paper', Q('A4')],
       ['title_block', ['title', Q(TITLE)], ['date', Q(DATE)], ['rev', Q(REV)], ['company', Q('TinkerRocket')]],
       ['lib_symbols'] + [kilib.symbol(l) for l in
                          ('Connector:Conn_Coaxial', 'Custom:GVLB258.A', 'Mechanical:MountingHole_Pad', 'power:GND',
                           'power:PWR_FLAG')]]
sch += items
sch += [['sheet_instances', ['path', Q('/'), ['page', Q('1')]]], ['embedded_fonts', 'no']]
open(OUT, 'w').write(dump(sch) + '\n')
print('wrote', OUT)
