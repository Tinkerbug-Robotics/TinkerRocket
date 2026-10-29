"""One-time conversion of px100-module's receiver sheet into this board's receiver.kicad_sch (run 2026-10-28).

Kept for the record; the output is edited by hand from now on, so don't rerun it over a sheet that has been edited.

- Stage 1 (the carrier's on-board LNAs FL4/U3/FL5/U4 and their parts) goes: an active antenna brings its own gain.
- Stage 2 (SkyTraq's L1/L5 split, SAWs and matches) slides left; C26 stays as the DC block in front of it.
- New input: J1 edge SMA (the carrier's), D6 ESD (kept), and a bias tee like SkyTraq's reference (L2 47 nH + C4 1 uF):
  L1 47 nH and C20 1 uF here, fed from +3V3 through R2 (fitted) or from the PX100's own ANT_BIAS pin, which the data
  sheet says has 45 mA short-circuit protection, through R3 (not fitted).
- U1 becomes PX100-5C-01A_AllPins (the reserved NC pins are passive) and every pin the PX100 might use gets a global
  label for the ESP32 sheet; no pin keeps a no-connect flag.
- The shield goes back to the carrier's Laird BMI-S-202-F frame and BMI-S-202-C cover (on hand from the V1 build).

Run:  python3 edit_receiver.py
"""
import copy
import os
import uuid

import kilib
from kilib import Q, dump, find, get
import make_sch

HW = os.path.join(kilib.PROJ, '..')
SRC = os.path.join(HW, 'px100-module', 'receiver.kicad_sch')
CARRIER = os.path.join(HW, 'gnss-px100-18mm-hv', 'receiver.kicad_sch')
OUT = os.path.join(kilib.PROJ, 'receiver.kicad_sch')

STAGE1 = {'C20', 'C21', 'L1', 'FL4', 'C22', 'L2', 'L3', 'U3', 'C23', 'L4', 'L5', 'FL5', 'L6', 'C24', 'L7', 'U4',
          'C25', 'L8'}
DX = -165.1                     # stage 2 moves left: C26 241.3 -> 76.2
PATH = '/%s/%s' % (make_sch.ROOT_UUID, make_sch.SHEET_UUID['receiver'])
n_uid = [0]


def new_uuid():
    n_uid[0] += 1
    return Q(make_sch.ustr('receiver-edit', n_uid[0]))


def r(v):
    return round(v, 4)


def props(sym):
    return {p[1]: p for p in find(sym, 'property')}


def ref_of(sym):
    p = props(sym).get('Reference')
    return p[2] if p else None


def points(item):
    """Every (x, y) an item touches: wire ends, label/junction/symbol anchors."""
    if item[0] == 'wire':
        pts = get(item, 'pts')
        return [(float(p[1]), float(p[2])) for p in find(pts, 'xy')]
    at = get(item, 'at')
    if at:
        return [(float(at[1]), float(at[2]))]
    return []


def move(node, dx):
    """Shift every at/xy in node by dx (in place)."""
    if not isinstance(node, list):
        return
    if node and node[0] in ('at', 'xy') and len(node) >= 3:
        node[1] = r(float(node[1]) + dx)
        return
    for c in node:
        move(c, dx)


def set_prop(sym, name, value):
    p = props(sym)[name]
    p[2] = Q(value)


def instance(sym, ref):
    inst = get(sym, 'instances')
    sym.remove(inst)
    sym.append(['instances', ['project', Q(make_sch.NAME), ['path', Q(PATH), ['reference', Q(ref)], ['unit', 1]]]])


def clone(src, ref, value, at, rot=0):
    s = copy.deepcopy(src)
    a = get(s, 'at')
    dx, dy = at[0] - float(a[1]), at[1] - float(a[2])
    for p in find(s, 'property'):
        pa = get(p, 'at')
        pa[1] = r(float(pa[1]) + dx)
        pa[2] = r(float(pa[2]) + dy)
    a[1], a[2] = at[0], at[1]
    if len(a) > 3:
        a[3] = rot
    get(s, 'uuid')[1] = new_uuid()
    for pin in find(s, 'pin'):
        get(pin, 'uuid')[1] = new_uuid()
    set_prop(s, 'Reference', ref)
    set_prop(s, 'Value', value)
    instance(s, ref)
    return s


def wire(a, b):
    return ['wire', ['pts', ['xy', r(a[0]), r(a[1])], ['xy', r(b[0]), r(b[1])]],
            ['stroke', ['width', 0], ['type', 'default']], ['uuid', new_uuid()]]


def junction(p):
    return ['junction', ['at', r(p[0]), r(p[1])], ['diameter', 0], ['color', 0, 0, 0, 0], ['uuid', new_uuid()]]


def glabel(net, p, ang):
    just = 'left' if ang == 0 else 'right'
    return ['global_label', Q(net), ['shape', 'bidirectional'], ['at', r(p[0]), r(p[1]), ang],
            ['fields_autoplaced', 'yes'], ['effects', ['font', ['size', 1.27, 1.27]], ['justify', just]],
            ['uuid', new_uuid()],
            ['property', Q('Intersheetrefs'), Q('${INTERSHEET_REFS}'), ['at', r(p[0]), r(p[1]), 0],
             ['hide', 'yes'], ['show_name', 'no'], ['do_not_autoplace', 'no'],
             ['effects', ['font', ['size', 1.27, 1.27]], ['justify', just]]]]


def llabel(net, p):
    return ['label', Q(net), ['at', r(p[0]), r(p[1]), 0],
            ['effects', ['font', ['size', 1.27, 1.27]], ['justify', 'left', 'bottom']], ['uuid', new_uuid()]]


def text(s, p, size=1.27):
    return ['text', Q(s), ['exclude_from_sim', 'no'], ['at', r(p[0]), r(p[1]), 0],
            ['effects', ['font', ['size', size, size]], ['justify', 'left', 'bottom']], ['uuid', new_uuid()]]


def main():
    sch = kilib.parse(open(SRC).read())[0]
    car = kilib.parse(open(CARRIER).read())[0]
    syms = {ref_of(x): x for x in find(sch, 'symbol')}
    csyms = {ref_of(x): x for x in find(car, 'symbol')}

    # ---- lib_symbols: U1's new symbol, the carrier's SMA and Laird shield
    libs = get(sch, 'lib_symbols')
    keep = [x for x in libs[1:] if x[1] not in ('Custom:PX100-5C-01A_Numbered', 'Custom:MS209-10F', 'Custom:MS209-10C',
                                                 'Custom:BGA855N6', 'Custom:B39162B8389P810', 'power:+1V8')]
    clibs = {x[1]: x for x in get(car, 'lib_symbols')[1:]}
    keep += [kilib.symbol('Custom:PX100-5C-01A_AllPins'), clibs['Custom:CON-SMA-EDGE-S'],
             clibs['Custom:BMI-S-202-F'], clibs['Custom:BMI-S-202-C']]
    libs[1:] = sorted(keep, key=lambda x: x[1])

    out = []
    for it in sch:
        if not isinstance(it, list) or it[0] in ('lib_symbols',):
            out.append(it)
            continue
        kind = it[0]
        if kind == 'symbol':
            ref = ref_of(it)
            if ref in STAGE1:
                continue
        if kind == 'text':
            t = it[1]
            if t.startswith(('Stage 1,', 'RF chain: SMA J1', 'SH1 + SH2:')):
                continue
            x, y = points(it)[0]
            if t.startswith('Stage 2,'):
                move(it, -65.1)     # beside the input note, under the split
            out.append(it)
            continue
        pts = points(it)
        if kind == 'symbol' and ref_of(it) in ('C26',):
            move(it, DX)
            out.append(it)
            continue
        if kind == 'symbol' and ref_of(it) == 'D6':
            out.append(it)          # stays where it is, on the new input line
            continue
        if pts and all(y < 112 for x, y in pts) and all(x >= 240 for x, y in pts):
            move(it, DX)           # stage 2
            out.append(it)
            continue
        if pts and all(y < 95 for x, y in pts) and all(x < 240 for x, y in pts):
            continue               # stage 1, the old RF_IN label, D6's old wiring
        out.append(it)
    sch[:] = out

    syms = {ref_of(x): x for x in find(sch, 'symbol')}
    gnd = next(x for x in find(sch, 'symbol') if get(x, 'lib_id')[1] == 'power:GND')
    p33 = next(x for x in find(sch, 'symbol') if get(x, 'lib_id')[1] == 'power:+3V3')
    add = []

    # ---- U1: the all-pins symbol, labels on every pin it may use
    u1 = syms['U1']
    get(u1, 'lib_id')[1] = Q('Custom:PX100-5C-01A_AllPins')
    left = {8: 'PX_NC8', 9: 'ANT_BIAS', 12: 'PX_GPIO0', 13: 'PX_GPIO6', 14: 'PX_NC14', 15: 'PX_GPIO9'}
    right = {24: 'PX_GPIO11', 25: 'PX_GPIO10', 26: 'PX_GPIO13', 27: 'PX_GPIO14', 28: 'PX_GPIO15', 29: 'PX_NC29',
             32: 'PX_GPIO2', 36: 'PX_NC36'}
    ux, uy = float(get(u1, 'at')[1]), float(get(u1, 'at')[2])
    pin_y = {n: r(uy - (24.13 - (n - 1) * 2.54)) if n <= 20 else r(uy - (24.13 - (n - 21) * 2.54)) for n in range(1, 41)}
    lx, rx = r(ux - 20.32), r(ux + 20.32)
    ends = set()
    # a local label's text sits above its wire, so a global label on the pin above would sit on it: those local
    # labels slide out along their wires instead (pins 10, 16, 37); pin 33's name only labelled its cap wire
    stub = {}
    for x in find(sch, 'label'):
        at = get(x, 'at')
        key = (x[1], r(float(at[1])), r(float(at[2])))
        if key == ('RTC_XOUT', 180.34, pin_y[10]):
            at[1] = 167.64
            add.append(wire((180.34, pin_y[10]), (167.64, pin_y[10])))
        elif key == ('VDD11', 180.34, pin_y[16]):
            at[1] = 170.18
        elif key == ('LNA_IN2', 226.06, pin_y[37]):
            at[1] = 238.76
            add.append(wire((226.06, pin_y[37]), (238.76, pin_y[37])))
        elif key == ('LDO_ADC_OUT', 226.06, pin_y[33]):
            sch.remove(x)
    for n, net in left.items():
        ends.add((lx, pin_y[n]))
        x = r(lx - stub.get(n, 2.54))
        add += [wire((lx, pin_y[n]), (x, pin_y[n])), glabel(net, (x, pin_y[n]), 180)]
    for n, net in right.items():
        ends.add((rx, pin_y[n]))
        x = r(rx + stub.get(n, 2.54))
        add += [wire((rx, pin_y[n]), (x, pin_y[n])), glabel(net, (x, pin_y[n]), 0)]
    sch[:] = [x for x in sch if not (isinstance(x, list) and x[0] == 'no_connect' and
                                       (r(float(get(x, 'at')[1])), r(float(get(x, 'at')[2]))) in ends)]
    left_nc = [x for x in find(sch, 'no_connect') if r(float(get(x, 'at')[1])) in (lx, rx)]
    assert not left_nc, 'U1 still has no-connect flags: %r' % left_nc

    # the +3V3 drop at the V_BACK/VDD_MAIN join hung over pins 8 and 9: move it left along pin 7's wire
    for x in find(sch, 'symbol'):
        if get(x, 'lib_id')[1] == 'power:+3V3' and (float(get(x, 'at')[1]), float(get(x, 'at')[2])) == (175.26, 143.51):
            get(x, 'at')[1] = 166.37
            v = props(x)['Value']
            get(v, 'at')[1], get(v, 'at')[2] = 166.37, 146.685
            get(props(x)['Reference'], 'at')[1] = 166.37
            add.append(junction((166.37, 143.51)))
            break
    else:
        raise SystemExit('the +3V3 symbol at (175.26, 143.51) moved')

    # ---- the RF input on y = 50.8: J1 -> D6 (ESD) -> L1 bias tee -> C26 (DC block) -> the split
    Y = 50.8
    j1 = clone(csyms['J1'], 'J1', 'CON-SMA-EDGE-S', (27.94, 53.34))
    add.append(j1)                                  # pin 1 at (30.48, 50.8), shield pins at (30.48, 53.34)
    add += [wire((30.48, 53.34), (30.48, 55.88)), clone(gnd, '#PWR0901', 'GND', (30.48, 55.88))]
    d6 = syms['D6']                                 # pin 2 (RF side) at (38.1, 50.8), pin 1 at (38.1, 58.42)
    add += [clone(gnd, '#PWR0902', 'GND', (38.1, 58.42))]
    add += [wire((30.48, Y), (73.66, Y)), junction((38.1, Y)), junction((50.8, Y)), llabel('RF_IN', (60.96, Y))]
    # bias tee: L1 from the line up to VANT, C20 to ground, R2 from +3V3 (fitted) or R3 from ANT_BIAS (not fitted)
    l_src = syms['L10']
    l1 = clone(l_src, 'L1', '47 nH', (50.8, 48.26), 0)
    set_prop(l1, 'MPN', 'LQW15AN47NG00D')
    if 'Description' in props(l1):
        set_prop(l1, 'Description', 'Antenna bias choke (SkyTraq reference L2)')
    get(props(l1)['Reference'], 'at')[1:4] = [52.705, 46.99, 0]
    get(props(l1)['Value'], 'at')[1:4] = [52.705, 48.895, 0]
    add.append(l1)                                  # pin 1 (45.72) to VANT, pin 2 (50.8) on the RF line
    c_src = syms['C47']
    c20 = clone(c_src, 'C20', '1 uF', (58.42, 40.64), 0)
    set_prop(c20, 'Description', 'Antenna bias decoupling (SkyTraq reference C4)')
    add += [c20, clone(gnd, '#PWR0903', 'GND', (58.42, 43.18))]
    add += [wire((50.8, 45.72), (50.8, 30.48)), wire((45.72, 38.1), (66.04, 38.1)), junction((50.8, 38.1)),
            junction((58.42, 38.1)), glabel('VANT', (66.04, 38.1), 0)]
    r_src = syms['C48']                             # the 0 ohm link (Device:R_Small)
    r2 = clone(r_src, 'R2', '0R', (43.18, 38.1), 90)
    set_prop(r2, 'MPN', '0402WGF0000TCE')
    r3 = clone(r_src, 'R3', '0R', (43.18, 30.48), 90)
    set_prop(r3, 'MPN', '0402WGF0000TCE')
    for p_, d_ in ((r2, 'Antenna bias from +3V3 (fitted)'),
                   (r3, 'Antenna bias from the PX100 ANT_BIAS pin, 45 mA limited (not fitted; fit instead of R2)')):
        set_prop(p_, 'Footprint', 'Resistor_SMD:R_0402_1005Metric')
        set_prop(p_, 'Description', d_)
    for k, v in (('dnp', 'yes'),):
        get(r3, k)[1] = v
    add += [r2, r3, clone(p33, '#PWR0904', '+3V3', (40.64, 38.1)), wire((45.72, 30.48), (50.8, 30.48)),
            glabel('ANT_BIAS', (40.64, 30.48), 180)]
    for p_ in (r2, r3):
        for f in ('Reference', 'Value'):
            a = get(props(p_)[f], 'at')
            a[3] = 90         # field angles count from the symbol: 90 reads level on a 90 degree part
    # R3's text above it; R2's below, clear of the +3V3 symbol on its left pin
    get(props(r3)['Reference'], 'at')[1:3] = [40.64, 27.94]
    get(props(r3)['Value'], 'at')[1:3] = [44.45, 27.94]
    get(props(r2)['Reference'], 'at')[1:3] = [40.64, 41.91]
    get(props(r2)['Value'], 'at')[1:3] = [44.45, 41.91]

    # ---- the split's input net is no longer an LNA output
    for x in find(sch, 'label'):
        if x[1] == 'LNA_OUT':
            x[1] = Q('RF_SPLIT')

    # ---- shield: the carrier's Laird frame + cover
    for ref in ('SH1', 'SH2'):
        old = syms[ref]
        new = clone(csyms[ref], ref, props(csyms[ref])['Value'][2], (float(get(old, 'at')[1]), float(get(old, 'at')[2])))
        get(new, 'uuid')[1] = get(old, 'uuid')[1]
        sch[sch.index(old)] = new

    # ---- notes
    add.append(text('RF chain: SMA J1 (active L1/L5 antenna) -> bias tee -> L1/L5 split -> PX100 '
                    '(SkyTraq PX100 DS p13)', (22.86, 17.78), 1.905))
    add.append(text('Input: J1 edge SMA (the carrier\'s), D6 ESD (TPD1E0B04, +/-3.6 V: fine at 3.3 V),\n'
                    'bias tee L1 47 nH + C20 1 uF (SkyTraq\'s L2/C4), C26 33 pF DC block.\n'
                    'Antenna power: R2 fitted = +3V3 straight, as in the reference. R3 instead\n'
                    'of R2 = the PX100\'s ANT_BIAS output (pin 9, 45 mA short-circuit limit).\n'
                    'The ESP32 reads ANT_BIAS and VANT. DC-block J1 when an SDR feeds it.', (22.86, 104.14)))
    add.append(text('SH1 + SH2: Laird BMI-S-202-F frame and BMI-S-202-C snap-on cover (16.5 x 16.5 mm),\n'
                    'the carrier\'s V1 parts. Cover U1, Y1 and the split; J1, the bias tee and the 1 k links\n'
                    'to the ESP32 sit outside.', (312.42, 132.08)))
    # keep the file's item order: symbols etc. after lib_symbols, then sheet_instances-free tail
    tail = [x for x in sch if isinstance(x, list) and x[0] == 'embedded_fonts']
    body = [x for x in sch if not (isinstance(x, list) and x[0] == 'embedded_fonts')]
    sch[:] = body + add + tail

    # ---- file identity, title block, instances
    get(sch, 'uuid')[1] = Q(make_sch.ustr('file', 'receiver'))
    tb = get(sch, 'title_block')
    get(tb, 'date')[1] = Q(make_sch.DATE)
    get(tb, 'rev')[1] = Q('${REVISION}')
    for x in find(sch, 'symbol'):
        ref = get(get(get(get(x, 'instances'), 'project'), 'path'), 'reference')[1]
        unit = get(get(get(get(x, 'instances'), 'project'), 'path'), 'unit')[1]
        x.remove(get(x, 'instances'))
        x.append(['instances', ['project', Q(make_sch.NAME), ['path', Q(PATH), ['reference', Q(ref)], ['unit', unit]]]])
    open(OUT, 'w').write(dump(sch) + '\n')
    print('wrote', OUT, len(find(sch, 'symbol')), 'symbols')


if __name__ == '__main__':
    main()
