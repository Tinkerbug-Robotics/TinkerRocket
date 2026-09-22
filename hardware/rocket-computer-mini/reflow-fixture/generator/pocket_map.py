"""Top-view map of the open-frame carrier: lips (contact), window, J2 catch floor, components."""
import sys
from design import design, P
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPoly, Rectangle, Patch
d = design()
fig, ax = plt.subplots(figsize=(7.5, 14))
def draw(geom, **kw):
    for g in getattr(geom, 'geoms', [geom]):
        ax.add_patch(MPoly(list(g.exterior.coords), closed=True, **kw))
ax.add_patch(Rectangle((-d['box_w'] / 2, -d['box_l'] / 2), d['box_w'], d['box_l'], fill=False, lw=2, ec='black'))
ax.add_patch(Rectangle((-d['sil_w'] / 2, -d['sil_l'] / 2), d['sil_w'], d['sil_l'], facecolor='#f3c9a8', ec='sienna', lw=1.2))
draw(d['window'], facecolor='white', edgecolor='gray', lw=0.7)
draw(d['catch'], facecolor='#ffe0b0', edgecolor='darkorange', lw=0.8, hatch='//')
draw(d['lips'], facecolor='#7fc97f', edgecolor='darkgreen', lw=0.8)
draw(d['outline'], fill=False, edgecolor='green', lw=0.8, ls='--')
for c in d['comps']:
    ax.add_patch(Rectangle((c['x0'], c['y0']), c['x1'] - c['x0'], c['y1'] - c['y0'], fill=False, ec='k', lw=0.4))
    if c['h'] > 1.3 or (c['x1'] - c['x0']) > 2.5 or (c['y1'] - c['y0']) > 2.5:
        ax.text((c['x0'] + c['x1']) / 2, (c['y0'] + c['y1']) / 2, "%s\n%.1f" % (c['ref'], c['h']), ha='center', va='center', fontsize=6)
ax.legend(handles=[Patch(fc='#f3c9a8', ec='sienna', label='silicone ring, %.0f mm tall, %.0f mm wall' % (P['sil_h'], P['wall'])),
                   Patch(fc='#7fc97f', ec='darkgreen', label='lips: the only silicone touching the board (%.0f mm2)' % d['lips'].area),
                   Patch(fc='white', ec='gray', label='open window (back side sees the oven)'),
                   Patch(fc='#ffe0b0', ec='darkorange', hatch='//', label='J2 catch floor, %.1f mm below the board' % (d['z_catch'] - P['board_t'])),
                   Patch(fc='none', ec='green', ls='--', label='board outline')],
          loc='lower center', fontsize=7, bbox_to_anchor=(0.5, -0.07), ncol=1)
ax.set_xlim(-22, 22); ax.set_ylim(-46, 46); ax.set_aspect('equal'); ax.grid(True, lw=0.3)
ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)  [KiCad y - 136.755]')
ax.set_title('RCM v1.01 open-frame reflow carrier — top view (labels: ref / height mm)')
out = sys.argv[1] if len(sys.argv) > 1 else 'out'
plt.savefig(f'{out}/render_pocket_map.png', dpi=130, bbox_inches='tight')
print("wrote pocket map")
