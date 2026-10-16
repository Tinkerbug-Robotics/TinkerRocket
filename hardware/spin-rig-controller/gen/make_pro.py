"""Writes spin-rig-controller.kicad_pro from the Tinker-Beetle's project file (hardware/tinker-beetle), with this board's sheets,
net classes and JLC 4-layer design rules."""
import json
import os

import design
import kilib
import make_sch

BEETLE = os.path.join(kilib.PROJ, '..', 'tinker-beetle', 'tinker-beetle.kicad_pro')


def main():
    d = json.load(open(BEETLE))
    name = make_sch.NAME
    d['meta']['filename'] = name + '.kicad_pro'
    d['sheets'] = [[str(make_sch.ROOT_UUID), name]] + [[str(make_sch.SHEET_UUID[k]), t] for k, t, _ in design.SHEETS]
    d['schematic']['top_level_sheets'] = [{'filename': name + '.kicad_sch', 'name': name, 'uuid': str(make_sch.ROOT_UUID)}]
    d['text_variables'] = {'REVISION': 'V1'}
    d['boards'] = []
    r = d['board']['design_settings']['rules']
    r.update({'min_clearance': 0.127, 'min_track_width': 0.127, 'min_via_diameter': 0.45, 'min_via_annular_width': 0.1,
              'min_through_hole_diameter': 0.2, 'min_copper_edge_clearance': 0.3, 'min_hole_clearance': 0.2,
              'min_hole_to_hole': 0.25, 'min_text_height': 0.8, 'min_text_thickness': 0.12})
    base = dict(d['net_settings']['classes'][0])
    classes = []
    for nm, tw, cl, vd, vdr, prio in (('Default', 0.2, 0.15, 0.6, 0.3, 2147483647), ('Power', 0.5, 0.15, 0.8, 0.4, 1),
                                       ('Motor', 1.5, 0.3, 0.8, 0.4, 0)):
        c = dict(base)
        c.update({'name': nm, 'track_width': tw, 'clearance': cl, 'via_diameter': vd, 'via_drill': vdr, 'priority': prio})
        classes.append(c)
    d['net_settings']['classes'] = classes
    pats = []
    # local labels carry their sheet path (/Power/VPACK); global labels and power symbols don't
    for n in ('VPACK', 'VMAIN', 'VDRV', 'BRAKE_SW'):
        pats.append({'netclass': 'Motor', 'pattern': '*' + n})
    for n in ('+5V', '+5V_PI', '+5V_AUX', 'VMAIN_F', 'BUCK_SW', 'V3V3_IN', '+3V3', 'VUSB', 'GND'):
        pats.append({'netclass': 'Power', 'pattern': n if n == 'GND' else '*' + n})
    d['net_settings']['netclass_patterns'] = pats
    d['net_settings']['netclass_assignments'] = None
    json.dump(d, open(os.path.join(kilib.PROJ, name + '.kicad_pro'), 'w'), indent=2)
    print('wrote', name + '.kicad_pro')


if __name__ == '__main__':
    main()
