"""Dump the open-frame mold geometry to JSON for the Fusion 360 script."""
import json
from design import design, P

d = design()

def polys(geom):
    out = []
    for g in getattr(geom, 'geoms', [geom]):
        if g.is_empty:
            continue
        ext = [[round(x, 4), round(y, 4)] for x, y in g.exterior.coords[:-1]]
        holes = [[[round(x, 4), round(y, 4)] for x, y in r.coords[:-1]] for r in g.interiors]
        out.append(dict(ext=ext, holes=holes))
    return out

data = dict(
    version="rocket-computer-mini v1.01 open-frame reflow carrier mold (v2)",
    params=dict(P),
    box=dict(w=d['box_w'], l=d['box_l'], r=P['outer_r'] + P['mold_wall'], z0=-P['floor_t'], z1=d['wall_top']),
    cavity=dict(w=d['sil_w'], l=d['sil_l'], r=P['outer_r'], z0=0.0, z1=d['wall_top'] + 1.0),
    groove=dict(w_out=d['sil_w'] + 2 * P['groove_d'], l_out=d['sil_l'] + 2 * P['groove_d'],
                w_in=d['sil_w'], l_in=d['sil_l'], z0=P['sil_h'] - P['groove_w'] / 2, h=P['groove_w']),
    plateau=dict(w=P['board_w'] + 2 * P['board_clr'], l=P['board_l'] + 2 * P['board_clr'],
                 r=P['board_corner_r'] + P['board_clr'], z0=0.0, z1=P['board_t']),
    core=dict(z0=0.0, z1=d['wall_top'], taper_deg=P['draft_deg'], polys=polys(d['window'])),
    catch=dict(z0=d['z_catch'], z1=d['wall_top'] + 2.0, polys=polys(d['catch'])),
    label=dict(text=P['label'], height=3.0, depth=0.4, x=-(P['board_w'] / 2 + P['board_clr'] + P['wall'] / 2), y=-14.0),
    silicone=dict(w=d['sil_w'], l=d['sil_l'], r=P['outer_r'], z0=0.0, z1=P['sil_h']),
)
json.dump(data, open('out/mold_geometry.json', 'w'), indent=1)
print("core polys:", len(data['core']['polys']), "verts:", sum(len(p['ext']) for p in data['core']['polys']),
      " catch polys:", len(data['catch']['polys']))
