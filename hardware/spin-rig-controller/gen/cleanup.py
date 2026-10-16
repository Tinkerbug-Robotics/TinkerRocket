"""Removes copper the router left dangling (vias and track stubs that lead nowhere), as DRC reports them."""
import json
import os
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import kilib  # noqa: E402

PCB = os.path.join(kilib.PROJ, 'spin-rig-controller.kicad_pcb')
d = json.load(open(os.path.join(HERE, 'out', 'drc.json')))
b = pcbnew.LoadBoard(PCB)
uuids = {it['uuid'] for v in d['violations'] if v['type'] in ('via_dangling', 'track_dangling') for it in v['items']}
n = 0
for t in list(b.GetTracks()):
    if t.GetClass() == 'PCB_TRACK' and t.GetLength() < pcbnew.FromMM(0.001):
        b.Remove(t)
        n += 1
        continue
    if str(t.m_Uuid.AsString()) in uuids and not t.IsLocked():
        b.Remove(t)
        n += 1
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(PCB, b)
print('removed', n)
