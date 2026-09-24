"""Match STEP component instances (leaves.json from step_inspect.py) to KiCad footprints by position."""
import re, json, collections, sys
pcb = sys.argv[1] if len(sys.argv) > 1 else 'rocket-computer-mini.kicad_pcb'
s = open(pcb).read()
fps = []
for m in re.finditer(r'\(footprint "([^"]+)"', s):
    chunk = s[m.start():m.start() + 3000]
    lay = re.search(r'\(layer "([^"]+)"', chunk)
    at = re.search(r'\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)', chunk)
    ref = re.search(r'\(property "Reference" "([^"]+)"', chunk)
    val = re.search(r'\(property "Value" "([^"]*)"', chunk)
    fps.append(dict(ref=ref.group(1) if ref else '?', fp=m.group(1), layer=lay.group(1), x=float(at.group(1)),
                    y=float(at.group(2)), rot=float(at.group(3) or 0), value=val.group(1) if val else ''))
json.dump(fps, open('footprints.json', 'w'), indent=1)
leaves = json.load(open('leaves.json'))
groups = collections.OrderedDict()
for l in leaves:
    key = l['path'][1] if len(l['path']) > 1 else l['path'][0]
    groups.setdefault(key, []).append(l)
inst = []
for k, ls in groups.items():
    bbs = [l['bbox'] for l in ls if l['bbox']]
    bb = [min(b[0] for b in bbs), min(b[1] for b in bbs), min(b[2] for b in bbs),
          max(b[3] for b in bbs), max(b[4] for b in bbs), max(b[5] for b in bbs)]
    inst.append(dict(key=k, bbox=bb))
back = [f for f in fps if f['layer'] == 'B.Cu']
front = [f for f in fps if f['layer'] == 'F.Cu']
def dist(i, f):
    bb = i['bbox']; cx = (bb[0] + bb[3]) / 2; cy = -(bb[1] + bb[4]) / 2   # STEP y = -KiCad y
    return ((cx - f['x']) ** 2 + (cy - f['y']) ** 2) ** 0.5
matched = []
for i in inst:
    if 'PCB' in i['key']:
        continue
    bb = i['bbox']
    side = 'B' if bb[5] <= 0.05 else ('F' if bb[2] >= 0.5 else 'STRADDLE')
    cands = back if side == 'B' else (front if side == 'F' else back + front)
    best = min(cands, key=lambda f: dist(i, f))
    matched.append(dict(key=i['key'], side=side, ref=best['ref'], fp=best['fp'], value=best['value'],
                        rot=best['rot'], dist=round(dist(i, best), 3), bbox=bb))
c = collections.Counter(m['ref'] for m in matched)
print("instances:", len(matched), " duplicate refs:", [r for r, n in c.items() if n > 1])
print("far matches (>1.5 mm):", [(m['key'], m['ref'], m['dist']) for m in matched if m['dist'] > 1.5])
print("B.Cu footprints without a STEP model:", [f['ref'] for f in back if f['ref'] not in c])
json.dump(matched, open('matched.json', 'w'), indent=1)
