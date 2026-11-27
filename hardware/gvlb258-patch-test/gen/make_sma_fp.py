"""Writes Footprints:SMA_GradConn_RFPC-SMA28-F_Vertical into the shared footprint library.

GCT (formerly GradConn) RFPC-SMA28-F: SMA jack, straight, through-hole, four square ground posts on a
5.08 mm square around the centre pin. Pad and drill sizes are the land pattern TinkerNav V25 uses for
the same part (J2): centre 2.325 mm pad / 1.55 mm drill, posts 2.625 mm pad / 1.75 mm drill.
Body (for the fab outline and courtyard): 7.0 mm square flange, 9.6 mm above the seating plane,
posts and centre pin 3.9 mm below it.

Pins are numbered as in KiCad's Connector:Conn_Coaxial: 1 = centre, 2 = the four posts (shield).
Run:  python3 make_sma_fp.py
"""
import os
import uuid

NAME = 'SMA_GradConn_RFPC-SMA28-F_Vertical'
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', '..', 'footprints', 'Footprints.pretty', NAME + '.kicad_mod')
NS = uuid.UUID('6f1d4c2e-7a0b-4b8e-9c3d-2e5f8a1b7c90')
_n = [0]


def u():
    _n[0] += 1
    return str(uuid.uuid5(NS, 'item%d' % _n[0]))


def font(size=1.0, thick=0.15):
    return '(effects\n\t\t\t(font\n\t\t\t\t(size %g %g)\n\t\t\t\t(thickness %g)\n\t\t\t)\n\t\t)' % (size, size, thick)


def prop(name, value, x, y, layer, hide=True, size=1.0):
    return ('\t(property "%s" "%s"\n\t\t(at %g %g 0)\n\t\t(layer "%s")\n%s\t\t(uuid "%s")\n\t\t%s\n\t)\n'
            % (name, value, x, y, layer, '\t\t(hide yes)\n' if hide else '', u(), font(size)))


def line(x1, y1, x2, y2, layer, w):
    return ('\t(fp_line\n\t\t(start %g %g)\n\t\t(end %g %g)\n\t\t(stroke\n\t\t\t(width %g)\n\t\t\t(type solid)\n'
            '\t\t)\n\t\t(layer "%s")\n\t\t(uuid "%s")\n\t)\n' % (x1, y1, x2, y2, w, layer, u()))


def rect(x1, y1, x2, y2, layer, w):
    return ('\t(fp_rect\n\t\t(start %g %g)\n\t\t(end %g %g)\n\t\t(stroke\n\t\t\t(width %g)\n\t\t\t(type solid)\n'
            '\t\t)\n\t\t(fill no)\n\t\t(layer "%s")\n\t\t(uuid "%s")\n\t)\n' % (x1, y1, x2, y2, w, layer, u()))


def circle(r, layer, w):
    return ('\t(fp_circle\n\t\t(center 0 0)\n\t\t(end %g 0)\n\t\t(stroke\n\t\t\t(width %g)\n\t\t\t(type solid)\n'
            '\t\t)\n\t\t(fill no)\n\t\t(layer "%s")\n\t\t(uuid "%s")\n\t)\n' % (r, w, layer, u()))


def pad(num, x, y, size, drill):
    return ('\t(pad "%s" thru_hole circle\n\t\t(at %g %g)\n\t\t(size %g %g)\n\t\t(drill %g)\n'
            '\t\t(layers "*.Cu" "*.Mask")\n\t\t(remove_unused_layers no)\n\t\t(uuid "%s")\n\t)\n'
            % (num, x, y, size, size, drill, u()))


P = 2.54            # post offset from the centre
BODY = 3.5          # half the 7.0 mm flange
CY = 4.1            # courtyard half size: posts reach 2.54 + 1.3125 = 3.85
SILK = 3.62         # silk just outside the flange, broken at the posts

s = '(footprint "%s"\n\t(version 20260206)\n\t(generator "pcbnew")\n\t(generator_version "10.0")\n\t(layer "F.Cu")\n' % NAME
s += prop('Reference', 'REF**', 0, -4.9, 'F.SilkS')
s += prop('Value', 'RFPC-SMA28-F', 0, 4.9, 'F.Fab')
s += prop('Datasheet', 'https://www.digikey.com/en/products/detail/gct/RFPC-SMA28-F/22162141', 0, 0, 'F.Fab', size=1.27)
s += prop('Description', 'GCT (GradConn) RFPC-SMA28-F SMA jack, straight, through-hole, 50 ohm, '
          '7.0 mm square flange, 9.6 mm tall', 0, 0, 'F.Fab', size=1.27)
s += ('\t(descr "GCT (formerly GradConn) RFPC-SMA28-F SMA jack (female), straight, through-hole, 50 ohm. '
      'Centre pin 1.55 mm drill / 2.325 mm pad; four ground posts on a 5.08 mm square, 1.75 mm drill / '
      '2.625 mm pad (the TinkerNav V25 land for the same part). Flange 7.0 x 7.0 mm, 9.6 mm above the '
      'seating plane, posts 3.9 mm below it. Pin 1 = centre, pin 2 = posts, as Connector:Conn_Coaxial. '
      'https://www.digikey.com/en/products/detail/gct/RFPC-SMA28-F/22162141")\n')
s += '\t(tags "SMA jack vertical through-hole coaxial RF GradConn GCT")\n'
s += '\t(attr through_hole)\n\t(duplicate_pad_numbers_are_jumpers no)\n'
for a in (-1, 1):   # silk: four short sides between the posts
    s += line(a * SILK, -1.0, a * SILK, 1.0, 'F.SilkS', 0.12)
    s += line(-1.0, a * SILK, 1.0, a * SILK, 'F.SilkS', 0.12)
s += rect(-BODY, -BODY, BODY, BODY, 'F.Fab', 0.1)
s += circle(3.175, 'F.Fab', 0.1)                   # 1/4-36 thread
s += rect(-CY, -CY, CY, CY, 'F.CrtYd', 0.05)
s += ('\t(fp_text user "${REFERENCE}"\n\t\t(at 0 0 0)\n\t\t(layer "F.Fab")\n\t\t(hide yes)\n\t\t(uuid "%s")\n\t\t%s\n\t)\n'
      % (u(), font(1.0)))
s += pad('1', 0, 0, 2.325, 1.55)
for x, y in ((-P, -P), (P, -P), (-P, P), (P, P)):
    s += pad('2', x, y, 2.625, 1.75)
s += ('\t(embedded_fonts no)\n\t(model "${KIPRJMOD}/../3dmodels/%s.step"\n\t\t(offset\n\t\t\t(xyz 0 0 0)\n\t\t)\n'
      '\t\t(scale\n\t\t\t(xyz 1 1 1)\n\t\t)\n\t\t(rotate\n\t\t\t(xyz 0 0 0)\n\t\t)\n\t)\n)\n' % NAME)
open(OUT, 'w').write(s)
print('wrote', os.path.normpath(OUT))
