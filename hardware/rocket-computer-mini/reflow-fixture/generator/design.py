"""
Reflow-support silicone carrier (open frame) for rocket-computer-mini v1.01,
and the 3D-printable mold that casts it.

Concept (v2, open frame)
------------------------
The carrier is a silicone RING.  The board drops into a 1.7 mm deep recess and
rests on narrow lips (<= 1.5 mm) that exist only where the terminal-block side
of the board is bare right at the edge.  Everything inside the lips is an open
window, so the whole back side of the board sees the oven directly and the
silicone touches nothing but those lips.  The tall back-side parts hang down
through the window; the ring is tall enough (14 mm) that the terminal block J2
clears the oven rack, and a thin silicone floor 0.3 mm under J2 catches it if
its joints go liquid and it starts to sag.  J2's body overhangs the board end,
so the end wall carries a notch; the SMA edge connector gets an open slot
through the other end wall.

The 3D-printed MOLD is an open-top box: floor = the carrier's top face; a
plateau (board outline + clearance, 1.7 mm) forms the recess; a tall drafted
CORE (the window shape) rises from the plateau to above the fill line so the
window is open; the core is lowered over J2 to leave the catch floor.

Mold coordinates (mm):
  X = KiCad x - board centre x           (board width axis, 22.5 mm)
  Y = KiCad y - board centre y           (board length axis, 69.57 mm; KiCad y-down frame)
  Z = 0 at the mold floor (== carrier top face), +Z up (== deeper into the carrier)
Board bottom copper is at Z = board_t.  A back-side part of height h occupies
Z in [board_t, board_t + h].
"""
import json, math
from shapely.geometry import box as sbox
from shapely.ops import unary_union
from shapely.geometry import MultiPolygon
from shapely.geometry.polygon import orient

def tidy(geom, min_area=0.05):
    """Drop sliver pieces and orient every polygon CCW (needed for a well-formed tapered extrude)."""
    parts = [orient(g, 1.0) for g in getattr(geom, 'geoms', [geom]) if not g.is_empty and g.area >= min_area]
    return parts[0] if len(parts) == 1 else MultiPolygon(parts)

# ---------------------------------------------------------------- parameters
P = dict(
    # board
    board_w=22.5, board_l=69.57, board_corner_r=1.0,
    board_t=1.70,          # PCB + copper as modelled in the STEP (1.53 core + 2 x 0.085)
    board_clr=0.20,        # per-side clearance of the recess around the board outline
    # silicone ring
    wall=5.0,              # ring width around the board
    sil_h=14.0,            # ring height = fill line (board_t + J2 9.85 + catch_clr + catch floor)
    outer_r=2.0,           # ring outer corner radius
    # lips (the only silicone that touches the board)
    lip_max=1.5,           # lips never reach further than this inside the board edge
    lip_min=0.6,           # lip fragments narrower than this are dropped
    lip_min_len=2.0,       # ... and shorter than this along the edge
    # clearances
    clr_xy=0.50,           # lateral clearance from every part to the window wall (at lip level)
    catch_clr=0.30,        # gap between J2's underside and the catch floor
    catch_margin=1.0,      # catch floor extends this far beyond J2's dilated outline
    draft_deg=1.5,         # draft on the core (window narrows going down; clearance compensated)
    # mold box
    mold_wall=2.5, floor_t=2.0, freeboard=3.0,
    groove_w=0.5, groove_d=0.4,
    label="RCM v1.01",
)
CX, CY = 83.74, 136.755   # board centre in KiCad coordinates (x, y)

def rounded_rect(w, l, r, cx=0.0, cy=0.0):
    b = sbox(cx - w / 2, cy - l / 2, cx + w / 2, cy + l / 2)
    return b.buffer(-r, join_style=1).buffer(r, join_style=1, quad_segs=8) if r > 1e-6 else b

def board_outline(offset=0.0):
    return rounded_rect(P['board_w'] + 2 * offset, P['board_l'] + 2 * offset, P['board_corner_r'] + offset)

def load_components():
    """Back-side components in mold coords: list of dict(ref, x0,x1,y0,y1,h)."""
    m = json.load(open('matched.json'))
    comps = []
    for c in m:
        if c['side'] == 'F' or 'Supercap' in c['key']:      # C130 is fitted after the second reflow
            continue
        bb = c['bbox']
        h = -bb[2] - 0.085
        if c['ref'] == 'U30':            # DFN model is mis-oriented in the library; real part is 0.8 mm
            h = 1.0
        comps.append(dict(ref=c['ref'], fp=c['fp'], x0=bb[0] - CX, x1=bb[3] - CX,
                          y0=-bb[4] - CY, y1=-bb[1] - CY, h=h))
    return comps

def rect(c, d=0.0):
    return sbox(c['x0'] - d, c['y0'] - d, c['x1'] + d, c['y1'] + d)

def design():
    comps = load_components()
    outline = board_outline(0.0)
    plateau = board_outline(P['board_clr'])
    tan_d = math.tan(math.radians(P['draft_deg']))

    # every part, dilated by clearance + draft compensation down to its own bottom
    dil = {c['ref']: P['clr_xy'] + tan_d * max(0.0, c['h']) for c in comps}
    comp_union = unary_union([rect(c, dil[c['ref']]) for c in comps])
    j2 = next(c for c in comps if c['ref'] == 'J2')
    j8 = next(c for c in comps if c['ref'] == 'J8')

    # --- lips: bare board edge, from the recess wall inward at most lip_max
    band = plateau.difference(outline.buffer(-P['lip_max'], join_style=2))
    lips = band.difference(comp_union)
    r = P['lip_min'] / 2 - 0.01
    lips = lips.buffer(-r, join_style=2).buffer(r, join_style=2)          # drop fragments thinner than lip_min
    keep = []
    for g in getattr(lips, 'geoms', [lips]):
        if g.is_empty:
            continue
        bx = g.bounds
        if max(bx[2] - bx[0], bx[3] - bx[1]) >= P['lip_min_len']:
            keep.append(g)
    lips = tidy(unary_union(keep).simplify(0.02))

    # --- window: everything else inside the recess, plus the J2 overhang notch
    #     and the SMA slot (which runs out through the end wall)
    sil_w = P['board_w'] + 2 * P['wall']
    sil_l = P['board_l'] + 2 * P['wall']
    sma_slot = sbox(j8['x0'] - dil['J8'], -(sil_l / 2 + P['mold_wall'] - 0.5), j8['x1'] + dil['J8'], j8['y1'] + dil['J8'])
    window = tidy(unary_union([plateau, rect(j2, dil['J2']), sma_slot]).difference(lips).simplify(0.02))

    # --- J2 catch floor: the core is lowered here so silicone fills above it
    z_catch = P['board_t'] + j2['h'] + P['catch_clr']
    catch = tidy(rect(j2, dil['J2'] + P['catch_margin']).intersection(window))

    wall_top = P['sil_h'] + P['freeboard']
    lands = lips   # what touches the board
    out = dict(P=P, comps=comps, outline=outline, plateau=plateau, lips=lips, window=window,
               catch=catch, z_catch=z_catch, wall_top=wall_top, sil_w=sil_w, sil_l=sil_l,
               box_w=sil_w + 2 * P['mold_wall'], box_l=sil_l + 2 * P['mold_wall'], dil=dil)
    return out

if __name__ == '__main__':
    d = design()
    print("silicone ring: %.2f x %.2f x %.1f mm, wall %.1f" % (d['sil_w'], d['sil_l'], P['sil_h'], P['wall']))
    print("mold box: %.2f x %.2f x %.1f mm (outer)" % (d['box_w'], d['box_l'], d['wall_top'] + P['floor_t']))
    print("lip contact area %.1f mm2 in %d segments:" % (d['lips'].area, len(getattr(d['lips'], 'geoms', [d['lips']]))))
    for g in getattr(d['lips'], 'geoms', [d['lips']]):
        print("   bounds %s  area %.1f" % ([round(v, 2) for v in g.bounds], g.area))
    print("window area %.0f mm2 (board %.0f mm2), J2 catch floor at Z %.2f, floor thickness %.2f" % (
        d['window'].area, d['outline'].area, d['z_catch'], P['sil_h'] - d['z_catch']))
    print("dilations: J2 %.2f  J3 %.2f  S1 %.2f  J8 %.2f" % (d['dil']['J2'], d['dil']['J3'], d['dil']['S1'], d['dil']['J8']))
