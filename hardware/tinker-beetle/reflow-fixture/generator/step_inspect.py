"""Inspect the KiCad/Fusion STEP assembly: list named components, their bounding boxes."""
import sys, json
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopoDS import TopoDS_Shape
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.IFSelect import IFSelect_RetDone
from OCP.gp import gp_Trsf

path = sys.argv[1]
doc = TDocStd_Document(TCollection_ExtendedString("doc"))
reader = STEPCAFControl_Reader()
reader.SetNameMode(True)
reader.SetColorMode(True)
status = reader.ReadFile(path)
assert status == IFSelect_RetDone, status
reader.Transfer(doc)
shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

def label_name(lbl):
    n = TDataStd_Name()
    if lbl.FindAttribute(TDataStd_Name.GetID_s(), n):
        return n.Get().ToExtString()
    return ""

def bbox(shape):
    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b, False)  # useTriangulation False -> exact-ish
    if b.IsVoid():
        return None
    xmin, ymin, zmin, xmax, ymax, zmax = b.Get()
    return [xmin, ymin, zmin, xmax, ymax, zmax]

results = []

def walk(lbl, trsf, depth, path_names):
    name = label_name(lbl)
    if shape_tool.IsReference_s(lbl):
        ref = TDF_Label()
        shape_tool.GetReferredShape_s(lbl, ref)
        loc = shape_tool.GetLocation_s(lbl)
        t2 = trsf.Multiplied(loc.Transformation())
        walk(ref, t2, depth, path_names + [name])
        return
    if shape_tool.IsAssembly_s(lbl):
        comps = TDF_LabelSequence()
        shape_tool.GetComponents_s(lbl, comps)
        for i in range(1, comps.Length() + 1):
            walk(comps.Value(i), trsf, depth + 1, path_names + [name])
    else:
        shape = shape_tool.GetShape_s(lbl)
        moved = BRepBuilderAPI_Transform(shape, trsf, True).Shape()
        bb = bbox(moved)
        results.append({"path": path_names + [name], "name": name, "bbox": bb,
                        "type": shape.ShapeType().name if hasattr(shape.ShapeType(), 'name') else str(shape.ShapeType())})

roots = TDF_LabelSequence()
shape_tool.GetFreeShapes(roots)
print("free shapes:", roots.Length())
for i in range(1, roots.Length() + 1):
    walk(roots.Value(i), gp_Trsf(), 0, [])

print(len(results), "leaf shapes")
json.dump(results, open(sys.argv[2], "w"), indent=1)
for r in results[:400]:
    bb = r["bbox"]
    if bb:
        print(f"{' / '.join(r['path']):60s} x[{bb[0]:8.3f},{bb[3]:8.3f}] y[{bb[1]:8.3f},{bb[4]:8.3f}] z[{bb[2]:7.3f},{bb[5]:7.3f}]")
    else:
        print(' / '.join(r['path']), "VOID")
