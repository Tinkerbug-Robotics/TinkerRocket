"""Offscreen VTK renders of the C130 support, alone and on the board.

    python3 render.py <out_dir>      (out_dir holds renders/_inputs/ from the build; they are deleted after)
"""
import os, sys
import vtk

out = sys.argv[1]
inp = os.path.join(out, "renders", "_inputs")


def actor(stl, color, opacity=1.0):
    r = vtk.vtkSTLReader(); r.SetFileName(stl); r.Update()
    n = vtk.vtkPolyDataNormals(); n.SetInputConnection(r.GetOutputPort()); n.SetFeatureAngle(35); n.Update()
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(n.GetOutputPort())
    a = vtk.vtkActor(); a.SetMapper(m)
    p = a.GetProperty(); p.SetColor(*color); p.SetOpacity(opacity); p.SetSpecular(0.15); p.SetSpecularPower(20)
    return a


def render(actors, fname, pos, focal=(0, 0, 3), up=(0, 0, 1), size=(1200, 1000), zoom=1.0, parallel=False):
    ren = vtk.vtkRenderer(); ren.SetBackground(1, 1, 1)
    for a in actors:
        ren.AddActor(a)
    ren.SetUseDepthPeeling(1); ren.SetMaximumNumberOfPeels(8)
    rw = vtk.vtkRenderWindow(); rw.SetOffScreenRendering(1); rw.AddRenderer(ren); rw.SetSize(*size); rw.SetAlphaBitPlanes(1)
    cam = ren.GetActiveCamera(); cam.SetPosition(*pos); cam.SetFocalPoint(*focal); cam.SetViewUp(*up)
    if parallel:
        cam.ParallelProjectionOn()
    ren.ResetCamera(); cam.Zoom(zoom)
    for lp, li in (((-60, -80, 120), 0.9), ((80, 60, 60), 0.5)):
        l = vtk.vtkLight(); l.SetPosition(*lp); l.SetIntensity(li); ren.AddLight(l)
    rw.Render()
    w2i = vtk.vtkWindowToImageFilter(); w2i.SetInput(rw); w2i.Update()
    wr = vtk.vtkPNGWriter(); wr.SetFileName(fname); wr.SetInputConnection(w2i.GetOutputPort()); wr.Write()


TPU = (0.95, 0.55, 0.20)
sup = lambda o=1.0: actor(os.path.join(inp, "support.stl"), TPU, o)
can = lambda o=1.0: actor(os.path.join(inp, "can.stl"), (0.15, 0.22, 0.55), o)
parts = lambda: actor(os.path.join(inp, "parts.stl"), (0.30, 0.30, 0.33))
board = lambda: actor(os.path.join(inp, "board.stl"), (0.10, 0.42, 0.22))
rd = os.path.join(out, "renders")
# 1. the part alone, board side down (as fitted)
render([sup()], f"{rd}/support_iso.png", pos=(40, -55, 45), zoom=1.3)
# 2. the part from below: the foot, the seat cone and the lead opening
render([sup()], f"{rd}/support_underside.png", pos=(30, -40, -60), zoom=1.3)
# 3. on the board with the can, from the S1 side (+y)
render([board(), parts(), sup(), can(0.85)], f"{rd}/fit_iso.png", pos=(45, 60, 45), focal=(0, 0, 4), zoom=1.9)
# 4. on the board, support translucent, looking down the can axis (plan view of the back face)
render([board(), parts(), sup(0.55)], f"{rd}/fit_plan.png", pos=(0, 0, 80), up=(0, -1, 0), focal=(0, 0, 0), zoom=2.0, parallel=True)
# 5. side view across the shield (-y) to S1 (+y)
render([board(), parts(), sup(), can(0.85)], f"{rd}/fit_side.png", pos=(80, 0, 5), focal=(0, 0, 5), zoom=2.0, parallel=True)
import shutil; shutil.rmtree(inp)
print("renders in", rd)
