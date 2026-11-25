"""Board checks after routing: KiCad DRC with schematic parity, and a short summary of what failed.

Run:  python3 check.py   (writes out/drc.rpt and out/drc.json)
"""
import collections
import json
import os
import subprocess

import kilib

K = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
PCB = os.path.join(kilib.PROJ, 'spin-rig-controller.kicad_pcb')
OUT = os.path.join(kilib.HERE, 'out')


def main():
    for fmt, ext in (('report', 'rpt'), ('json', 'json')):
        subprocess.run([K, 'pcb', 'drc', '--schematic-parity', '--severity-all', '--format', fmt,
                        '-o', os.path.join(OUT, 'drc.' + ext), PCB], capture_output=True, text=True)
    d = json.load(open(os.path.join(OUT, 'drc.json')))
    for key in ('violations', 'unconnected_items', 'schematic_parity'):
        items = d.get(key, [])
        c = collections.Counter((i['type'], i['severity']) for i in items)
        print('%s: %d' % (key, len(items)))
        for (t, sev), n in c.most_common():
            print('   %-28s %-8s %d' % (t, sev, n))


if __name__ == '__main__':
    main()
