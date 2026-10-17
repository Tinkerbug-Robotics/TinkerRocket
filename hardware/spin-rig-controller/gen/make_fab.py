"""Review package for JLCPCB: gerbers + drill (zipped), BOM and CPL in JLC's column names, the schematic PDF,
and top/bottom renders, in ../gerbers/ (ignored by git). At order time, plot the gerbers with the repo's
tools/plot_gerbers.sh instead (it refuses a dirty tree and stamps the commit and tag); the BOM and CPL here
still apply.

Run:  python3 make_fab.py
"""
import csv
import os
import shutil
import subprocess
import zipfile

import kilib

K = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
NAME = 'spin-rig-controller'
PROJ = kilib.PROJ
FAB = os.path.join(PROJ, 'gerbers')
PCB = os.path.join(PROJ, NAME + '.kicad_pcb')
SCH = os.path.join(PROJ, NAME + '.kicad_sch')
LAYERS = 'F.Cu,In1.Cu,In2.Cu,B.Cu,F.Mask,B.Mask,F.Paste,F.Silkscreen,B.Silkscreen,Edge.Cuts'
SKIP = ('H', 'FID', 'TP')


def run(*args):
    r = subprocess.run([K, *args], capture_output=True, text=True)
    out = '\n'.join(l for l in (r.stdout + r.stderr).splitlines() if 'Fontconfig' not in l)
    if r.returncode:
        raise SystemExit(out)
    return out


def main():
    for f in os.listdir(FAB) if os.path.exists(FAB) else []:
        if f != '.gitkeep':
            q = os.path.join(FAB, f)
            shutil.rmtree(q) if os.path.isdir(q) else os.remove(q)
    os.makedirs(FAB, exist_ok=True)
    g = os.path.join(FAB, 'plot')
    os.makedirs(g)
    run('pcb', 'export', 'gerbers', '--layers', LAYERS, '--no-protel-ext', '--use-drill-file-origin', '-o', g + '/', PCB)
    run('pcb', 'export', 'drill', '--format', 'excellon', '--excellon-separate-th', '--generate-map',
        '--map-format', 'pdf', '--drill-origin', 'plot', '-o', g + '/', PCB)
    z = os.path.join(FAB, NAME + '-gerbers.zip')
    with zipfile.ZipFile(z, 'w', zipfile.ZIP_DEFLATED) as zf:
        for f in sorted(os.listdir(g)):
            if not f.endswith('.pdf'):
                zf.write(os.path.join(g, f), f)

    # CPL from KiCad's position file, renamed for JLC
    pos = os.path.join(FAB, 'pos.csv')
    run('pcb', 'export', 'pos', '--format', 'csv', '--units', 'mm', '--side', 'both', '--exclude-dnp',
        '--use-drill-file-origin', '-o', pos, PCB)
    rows = list(csv.DictReader(open(pos)))
    with open(os.path.join(FAB, NAME + '-cpl.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['Designator', 'Mid X', 'Mid Y', 'Layer', 'Rotation'])
        for r in rows:
            if r['Ref'].startswith(SKIP):
                continue
            w.writerow([r['Ref'], r['PosX'] + 'mm', r['PosY'] + 'mm', 'Top' if r['Side'] == 'top' else 'Bottom',
                        r['Rot']])
    os.remove(pos)

    # BOM from the schematic netlist (fields MPN, Mfr, LCSC), grouped by LCSC code
    net = os.path.join(FAB, 'bom.net')
    run('sch', 'export', 'netlist', '--format', 'kicadsexpr', '-o', net, SCH)
    root = kilib.parse(open(net).read())[0]
    groups = {}
    for c in kilib.find(kilib.get(root, 'components'), 'comp'):
        ref = str(kilib.get(c, 'ref')[1])
        props = {str(kilib.get(p, 'name')[1]) for p in kilib.find(c, 'property')}
        if ref.startswith(SKIP) or 'dnp' in props or 'exclude_from_bom' in props:
            continue
        fields = {}
        fl = kilib.get(c, 'fields')
        for f in kilib.find(fl, 'field') if fl else []:
            fields[str(kilib.get(f, 'name')[1])] = str(f[-1]) if isinstance(f[-1], str) else ''
        fp = str(kilib.get(c, 'footprint')[1]).split(':')[1]
        key = (fields.get('LCSC', ''), str(kilib.get(c, 'value')[1]), fp)
        groups.setdefault(key, {'refs': [], 'mpn': fields.get('MPN', ''), 'mfr': fields.get('Mfr', '')})['refs'].append(ref)
    os.remove(net)

    def sortkey(r):
        p = ''.join(ch for ch in r if ch.isalpha())
        n = ''.join(ch for ch in r if ch.isdigit())
        return (p, int(n or 0))

    with open(os.path.join(FAB, NAME + '-bom.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['Comment', 'Designator', 'Footprint', 'LCSC Part #', 'Manufacturer', 'MPN', 'Quantity'])
        for (lcsc, val, fp), d in sorted(groups.items(), key=lambda kv: sortkey(sorted(kv[1]['refs'], key=sortkey)[0])):
            refs = sorted(d['refs'], key=sortkey)
            w.writerow([val, ','.join(refs), fp, lcsc, d['mfr'], d['mpn'], len(refs)])
    print(len(groups), 'BOM lines,', sum(len(d['refs']) for d in groups.values()), 'parts')

    run('sch', 'export', 'pdf', '-o', os.path.join(FAB, NAME + '-schematic.pdf'), SCH)
    for side in ('top', 'bottom'):
        try:
            run('pcb', 'render', '--side', side, '--width', '1800', '--height', '1350', '--quality', 'high',
                '-o', os.path.join(FAB, NAME + '-%s.png' % side), PCB)
        except SystemExit as e:
            print('render failed:', str(e)[:200])
    print('wrote', FAB)


if __name__ == '__main__':
    main()
