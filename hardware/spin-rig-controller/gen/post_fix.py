"""Fixes on an already-routed board, matching what make_pcb.py now does on a fresh build: every footprint carries
the symbol's MPN/Mfr/LCSC/Description/Datasheet fields (even empty), and the drill/place origin sits at the board's
bottom-left corner."""
import os
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kilib  # noqa: E402
import make_pcb  # noqa: E402

b = pcbnew.LoadBoard(make_pcb.PCB)
comps, nets = make_pcb.load_netlist()
for fp in b.GetFootprints():
    c = comps.get(fp.GetReference())
    if not c:
        continue
    for k in ('MPN', 'Mfr', 'LCSC', 'Description', 'Datasheet'):
        fp.SetField(k, c['fields'].get(k, ''))
        f = fp.GetField(k)
        if f:
            f.SetVisible(False)
            f.SetLayer(pcbnew.F_Fab)
# drill/place origin at the board's bottom-left corner, for the gerbers, drill file and CPL
b.GetDesignSettings().SetAuxOrigin(pcbnew.VECTOR2I(pcbnew.FromMM(make_pcb.OX), pcbnew.FromMM(make_pcb.OY + make_pcb.H)))
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(make_pcb.PCB, b)
print('post_fix done')
