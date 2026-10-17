"""KiCad library access for the generators: symbols (with `extends` flattened) and footprint pads.

S-expressions are kept as nested lists of strings; `dump` writes them back in KiCad's tab style.
"""
import os
import re

KICAD = '/Applications/KiCad/KiCad.app/Contents/SharedSupport'
HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)

_tok = re.compile(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+')


class Q(str):
    """A quoted string atom."""


def parse(text):
    stack = [[]]
    for m in _tok.finditer(text):
        t = m.group(0)
        if t == '(':
            stack.append([])
        elif t == ')':
            x = stack.pop()
            stack[-1].append(x)
        elif t.startswith('"'):
            stack[-1].append(Q(t[1:-1].replace('\\"', '"').replace('\\\\', '\\')))
        else:
            stack[-1].append(t)
    return stack[0]


def atom(x):
    if isinstance(x, Q):
        return '"' + x.replace('\\', '\\\\').replace('"', '\\"') + '"'
    if isinstance(x, float):
        s = ('%.4f' % x).rstrip('0').rstrip('.')
        return '0' if s in ('-0', '') else s
    return str(x)


def dump(node, ind=0):
    """KiCad style: a list whose items are all atoms goes on one line; otherwise children on new lines."""
    if not isinstance(node, list):
        return atom(node)
    if all(not isinstance(c, list) for c in node):
        return '(' + ' '.join(atom(c) for c in node) + ')'
    head = []
    i = 0
    while i < len(node) and not isinstance(node[i], list):
        head.append(atom(node[i]))
        i += 1
    out = '(' + ' '.join(head)
    for c in node[i:]:
        out += '\n' + '\t' * (ind + 1) + dump(c, ind + 1) if isinstance(c, list) else ' ' + atom(c)
    return out + '\n' + '\t' * ind + ')'


def find(node, name):
    return [x for x in node if isinstance(x, list) and x and x[0] == name]


def get(node, name):
    r = find(node, name)
    return r[0] if r else None


_libcache = {}


def _lib(path):
    if path not in _libcache:
        root = parse(open(path).read())[0]
        _libcache[path] = {s[1]: s for s in find(root, 'symbol')}
    return _libcache[path]


def lib_path(lib):
    if lib == 'Custom':      # the repo's shared library (hardware/symbols)
        return os.path.join(PROJ, '..', 'symbols', 'Custom.kicad_sym')
    return os.path.join(KICAD, 'symbols', lib + '.kicad_sym')


def symbol(lib_id):
    """Flattened symbol definition named lib_id (e.g. 'Device:R'), as used in a schematic's lib_symbols."""
    import copy
    lib, name = lib_id.split(':')
    syms = _lib(lib_path(lib))
    s = copy.deepcopy(syms[name])
    ext = get(s, 'extends')
    if ext:
        base = copy.deepcopy(syms[ext[1]])
        while get(base, 'extends'):
            base = copy.deepcopy(syms[get(base, 'extends')[1]])
        bname = base[1]
        props = {p[1]: p for p in find(s, 'property')}
        out = [x for x in base if not (isinstance(x, list) and x and x[0] == 'property')]
        bprops = {p[1]: p for p in find(base, 'property')}
        bprops.update(props)
        # keep property order of the base, then new ones
        newprops = list(bprops.values())
        out = out[:2] + [x for x in out[2:] if not (isinstance(x, list) and x[0] == 'symbol')] + newprops + \
            [x for x in out[2:] if isinstance(x, list) and x[0] == 'symbol']
        for sub in find(out, 'symbol'):
            sub[1] = Q(sub[1].replace(bname, name, 1))
        s = out
    s[1] = Q(lib_id)
    return s


def pins(lib_id, unit=1):
    """[(number, name, type, x, y, angle_deg, length)] in library coordinates (y up), for one unit
    (sub-symbols _0_* are common to all units)."""
    s = symbol(lib_id)
    out = []
    name = lib_id.split(':')[1]
    for sub in find(s, 'symbol'):
        m = re.match(re.escape(name) + r'_(\d+)_(\d+)$', sub[1])
        if not m:
            continue
        u, style = int(m.group(1)), int(m.group(2))
        if u not in (0, unit) or style not in (0, 1):
            continue
        for p in find(sub, 'pin'):
            at = get(p, 'at')
            nm = get(p, 'name')[1]
            num = get(p, 'number')[1]
            ln = float(get(p, 'length')[1]) if get(p, 'length') else 2.54
            out.append((str(num), str(nm), p[1], float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0.0, ln))
    return out


def bbox(lib_id, unit=1):
    """Body extent from rectangles/polylines/pin ends, lib coords: (xmin, ymin, xmax, ymax)."""
    xs, ys = [], []
    for num, nm, typ, x, y, a, ln in pins(lib_id, unit):
        xs.append(x); ys.append(y)
    s = symbol(lib_id)
    def walk(n):
        if isinstance(n, list):
            if n and n[0] in ('start', 'end', 'xy', 'center') and len(n) >= 3:
                try:
                    xs.append(float(n[1])); ys.append(float(n[2]))
                except ValueError:
                    pass
            for c in n:
                walk(c)
    for sub in find(s, 'symbol'):
        walk(sub)
    if not xs:
        return (-2.54, -2.54, 2.54, 2.54)
    return (min(xs), min(ys), max(xs), max(ys))


def footprint_pads(fp_id):
    lib, name = fp_id.split(':')
    if lib == 'Footprints':  # the repo's shared library (hardware/footprints)
        path = os.path.join(PROJ, '..', 'footprints', 'Footprints.pretty', name + '.kicad_mod')
    else:
        path = os.path.join(KICAD, 'footprints', lib + '.pretty', name + '.kicad_mod')
    root = parse(open(path).read())[0]
    return sorted({str(p[1]) for p in find(root, 'pad') if str(p[1])})
