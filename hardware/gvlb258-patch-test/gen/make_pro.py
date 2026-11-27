"""Writes gvlb258-patch-test.kicad_pro from the PX100 carrier's project file, with this board's rules:
vias 0.4/0.3 only (owner rule), a Default class (0.25 mm track, 0.2 mm clearance) and an RF class for
RF_FEED: 1.0 mm track with 0.25 mm clearance = a 50 ohm grounded coplanar line on 1.53 mm FR-4
(er 4.5, 1 oz), the JLC 2-layer 1.6 mm board.

Run:  python3 make_pro.py
"""
import json
import os

import kilib

NAME = 'gvlb258-patch-test'
SRC = os.path.join(kilib.PROJ, '..', 'gnss-px100-18mm-hv', 'gnss-px100-18mm-hv.kicad_pro')
OUT = os.path.join(kilib.PROJ, NAME + '.kicad_pro')
RF_W, RF_GAP = 1.0, 0.25

p = json.load(open(SRC))
p['meta']['filename'] = NAME + '.kicad_pro'
p['text_variables'] = {}
p['sheets'] = []
p['boards'] = []
ds = p['board']['design_settings']
r = ds['rules']
r.update({'min_clearance': 0.15, 'min_copper_edge_clearance': 0.3, 'min_track_width': 0.15,
          'min_via_diameter': 0.4, 'min_through_hole_diameter': 0.3, 'min_via_annular_width': 0.05,
          'min_hole_clearance': 0.25, 'min_hole_to_hole': 0.25, 'min_text_height': 0.8})
ds['via_dimensions'] = [{'diameter': 0.0, 'drill': 0.0}, {'diameter': 0.4, 'drill': 0.3}]
ds['track_widths'] = [0.0, 0.25, RF_W]
ds['diff_pair_dimensions'] = [{'gap': 0.0, 'via_gap': 0.0, 'width': 0.0}]
ns = p['net_settings']
default, rf = ns['classes'][0], ns['classes'][1]
default.update({'clearance': 0.2, 'track_width': 0.25, 'via_diameter': 0.4, 'via_drill': 0.3})
rf.update({'clearance': RF_GAP, 'track_width': RF_W, 'via_diameter': 0.4, 'via_drill': 0.3})
ns['classes'] = [default, rf]
ns['netclass_patterns'] = [{'netclass': 'RF', 'pattern': '/RF_FEED'}, {'netclass': 'RF', 'pattern': 'RF_FEED'}]
ns['netclass_assignments'] = None
p['pcbnew']['last_paths'] = {k: '' for k in p['pcbnew'].get('last_paths', {})}
p['schematic']['legacy_lib_dir'] = ''
json.dump(p, open(OUT, 'w'), indent=2)
open(OUT, 'a').write('\n')
print('wrote', OUT)
