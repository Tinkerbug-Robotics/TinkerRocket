"""Verify the mold against the real STEP solids.

For every back-side component (and the board), transform the STEP solid into
mold coordinates and measure the minimum distance to the cast silicone
carrier.  Also confirm the board itself does not intersect the silicone and
export a 'board in carrier' assembly for rendering.
"""
import json, math, sys, collections
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.STEPControl import STEPControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.IFSelect import IFSelect_RetDone
from OCP.gp import gp_Trsf, gp_Ax1, gp_Pnt, gp_Dir, gp_Vec
from OCP.TopoDS import TopoDS_Compound
from OCP.BRep import BRep_Builder
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from design import P, CX, CY

import os
STEP_IN = os.environ.get('BOARD_STEP', '/mnt/user-data/uploads/Downloads/rocket-computer-mini-v1_01.step')
SIL = sys.argv[1] if len(sys.argv) > 1 else 'out/rocket-computer-mini-reflow-carrier-silicone.step'

# ---- mold transform: rotate 180 deg about X (x,y,z)->(x,-y,-z), then translate
rot = gp_Trsf(); rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), math.pi)
tr = gp_Trsf(); tr.SetTranslation(gp_Vec(-CX, -CY, P['board_t'] - 0.085))
T = tr.Multiplied(rot)

def load_instances():
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    reader = STEPCAFControl_Reader(); reader.SetNameMode(True)
    assert reader.ReadFile(STEP_IN) == IFSelect_RetDone
    reader.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    def name(lbl):
        n = TDataStd_Name()
        return n.Get().ToExtString() if lbl.FindAttribute(TDataStd_Name.GetID_s(), n) else ""
    inst = collections.OrderedDict()
    def walk(lbl, trsf, path):
        nm = name(lbl)
        if st.IsReference_s(lbl):
            ref = TDF_Label(); st.GetReferredShape_s(lbl, ref)
            walk(ref, trsf.Multiplied(st.GetLocation_s(lbl).Transformation()), path + [nm]); return
        if st.IsAssembly_s(lbl):
            comps = TDF_LabelSequence(); st.GetComponents_s(lbl, comps)
            for i in range(1, comps.Length() + 1):
                walk(comps.Value(i), trsf, path + [nm])
        else:
            key = path[1] if len(path) > 1 else nm
            shp = BRepBuilderAPI_Transform(st.GetShape_s(lbl), trsf, True).Shape()
            inst.setdefault(key, []).append(shp)
    roots = TDF_LabelSequence(); st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), gp_Trsf(), [])
    out = collections.OrderedDict()
    for k, shapes in inst.items():
        c = TopoDS_Compound(); b = BRep_Builder(); b.MakeCompound(c)
        for s in shapes:
            b.Add(c, s)
        out[k] = BRepBuilderAPI_Transform(c, T, True).Shape()
    return out

def bbox(s):
    b = Bnd_Box(); BRepBndLib.Add_s(s, b, False); return b.Get()

def volume(s):
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(s, g); return g.Mass()

def read_step_single(path):
    r = STEPControl_Reader(); assert r.ReadFile(path) == IFSelect_RetDone
    r.TransferRoots(); return r.OneShape()

if __name__ == '__main__':
    inst = load_instances()
    matched = {m['key']: m for m in json.load(open('matched.json'))}
    silicone = read_step_single(SIL)
    print("silicone volume %.1f mL" % (volume(silicone) / 1000))

    rows = []
    board = None
    for key, shp in inst.items():
        if 'PCB' in key:
            board = shp; continue
        m = matched.get(key)
        if m is None or m['side'] == 'F':
            continue
        if 'Supercap' in key:
            continue
        bb = bbox(shp)
        dss = BRepExtrema_DistShapeShape(shp, silicone)
        dss.Perform()
        dist = dss.Value() if dss.IsDone() else float('nan')
        # penetration check: common volume
        com = BRepAlgoAPI_Common(shp, silicone); com.Build()
        pen = volume(com.Shape()) if com.IsDone() else float('nan')
        rows.append((m['ref'], key, dist, pen, bb))
    rows.sort(key=lambda r: r[2])
    print("%-6s %-45s %8s %10s   Z range (mold)" % ("ref", "model", "clr mm", "overlap"))
    for ref, key, dist, pen, bb in rows:
        flag = "  <-- CHECK" if (dist < 0.25 or pen > 1e-6) else ""
        print("%-6s %-45s %8.3f %10.4f   [%6.2f, %6.2f]%s" % (ref, key[:45], dist, pen, bb[2], bb[5], flag))
    # board
    bb = bbox(board)
    com = BRepAlgoAPI_Common(board, silicone); com.Build()
    print("board bbox mold coords: x[%.2f,%.2f] y[%.2f,%.2f] z[%.3f,%.3f]  overlap with silicone: %.4f mm3" % (
        bb[0], bb[3], bb[1], bb[4], bb[2], bb[5], volume(com.Shape())))
    dss = BRepExtrema_DistShapeShape(board, silicone); dss.Perform()
    print("board-to-silicone distance: %.4f (0 = resting on the lands)" % dss.Value())
    n_bad = sum(1 for r in rows if r[2] < 0.25 or r[3] > 1e-6)
    print("components checked: %d, flagged: %d, min clearance %.3f mm" % (len(rows), n_bad, min(r[2] for r in rows)))

    # export board + back components in mold coords for rendering
    from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
    w = STEPControl_Writer()
    c = TopoDS_Compound(); b = BRep_Builder(); b.MakeCompound(c)
    b.Add(c, board)
    for key, shp in inst.items():
        if 'PCB' in key or 'Supercap' in key: continue
        b.Add(c, shp)
    w.Transfer(c, STEPControl_AsIs); w.Write('out/_board_in_mold_coords.step')
