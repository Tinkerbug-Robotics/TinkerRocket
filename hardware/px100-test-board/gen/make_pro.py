"""Writes px100-test-board.kicad_pro from px100-module's project file (same PX100 footprint rules), with this board's
sheets, revision and RF net class."""
import json
import os

import design
import kilib
import make_sch

BASE = os.path.join(kilib.PROJ, '..', 'px100-module', 'px100-module.kicad_pro')
if not os.path.exists(BASE):          # px100-module is not in the repo yet: start from this board's own project file
    BASE = os.path.join(kilib.PROJ, make_sch.NAME + '.kicad_pro')

# 50 ohm coplanar waveguide over In1 on JLC04161H-7628 (F.Cu 35 um, 7628 prepreg 0.2104 mm, er 4.4): 0.34 mm with a
# 0.20 mm gap to the pour (see FABRICATION-NOTES). Every net from J1 to the PX100's two RF inputs.
RF_W, RF_GAP = 0.34, 0.20
RF_NETS = ['/RF chain and PX100/RF_IN', '/RF chain and PX100/RF_SPLIT', '/RF chain and PX100/ARM_L5',
           'Net-(FL6-IN)', 'Net-(FL6-OUT)', 'Net-(FL7-OUT)', '/RF chain and PX100/LNA_IN1',
           '/RF chain and PX100/LNA_IN2']
# supplies at 0.3 mm (the PX100's DCDC_IN feed and the antenna bias included); USB as a 0.2 mm pair. Full-speed USB
# (12 Mb/s) over 30 mm of board does not need a controlled 90 ohm pair.
EXTRA_CLASSES = {'Power': (0.3, 0.15, (0.6, 0.3)), 'USB': (0.2, 0.15, (0.4, 0.3))}
EXTRA_NETS = {'Power': ['VBUS', '+3V3', '+3V3_MCU', '*/V_LDO_GNSS', '*/DCDC_IN', '*/DCDC_OUT', '*/V_CORE', 'VANT',
                        '*/USB_MUX_VCC'],
              'USB': ['*USB_DP', '*USB_DN', '*ESP_DP', '*ESP_DN', '*CP_DP', '*CP_DN', '*USB_DP_MCU',
                      '*USB_DN_MCU']}


def main():
    d = json.load(open(BASE))
    name = make_sch.NAME
    d['meta']['filename'] = name + '.kicad_pro'
    d['sheets'] = [[str(make_sch.ROOT_UUID), name]] + \
        [[str(make_sch.SHEET_UUID[k]), t] for k, t, _ in design.SHEETS + design.STATIC_SHEETS]
    d['schematic']['top_level_sheets'] = [{'filename': name + '.kicad_sch', 'name': name, 'uuid': str(make_sch.ROOT_UUID)}]
    d['text_variables'] = {'REVISION': 'V1'}
    d['boards'] = []
    classes = d['net_settings']['classes']
    for c in classes:
        if c['name'] == 'RF':
            c.update({'track_width': RF_W, 'clearance': RF_GAP})
        if c['name'] == 'Default':
            c.update({'track_width': 0.15, 'clearance': 0.1})
    default = [c for c in classes if c['name'] == 'Default'][0]
    for cls, (w, clr, via) in EXTRA_CLASSES.items():
        c = dict(default, name=cls, track_width=w, clearance=clr, via_diameter=via[0], via_drill=via[1])
        classes[:] = [k for k in classes if k['name'] != cls] + [c]
    d['net_settings']['netclass_patterns'] = [{'netclass': 'RF', 'pattern': n} for n in RF_NETS] + \
        [{'netclass': cls, 'pattern': n} for cls, nets in EXTRA_NETS.items() for n in nets]
    d['net_settings']['netclass_assignments'] = None
    json.dump(d, open(os.path.join(kilib.PROJ, name + '.kicad_pro'), 'w'), indent=2)
    print('wrote', name + '.kicad_pro')


if __name__ == '__main__':
    main()
