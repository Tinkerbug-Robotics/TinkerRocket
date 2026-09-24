"""Offscreen VTK renders of the mold and of the carrier in use."""
import vtk, sys

def actor(stl, color, opacity=1.0, flip=False):
    r = vtk.vtkSTLReader(); r.SetFileName(stl); r.Update()
    n = vtk.vtkPolyDataNormals(); n.SetInputConnection(r.GetOutputPort()); n.SetFeatureAngle(40); n.Update()
    m = vtk.vtkPolyDataMapper(); m.SetInputConnection(n.GetOutputPort())
    a = vtk.vtkActor(); a.SetMapper(m)
    a.GetProperty().SetColor(*color); a.GetProperty().SetOpacity(opacity)
    a.GetProperty().SetSpecular(0.15); a.GetProperty().SetSpecularPower(20)
    if flip:
        a.RotateX(180)   # carrier in use: flipped so pockets face up
    return a

def render(actors, fname, pos, focal=(0, 0, 0), up=(0, 0, 1), size=(1500, 1100), zoom=1.0, parallel=False):
    ren = vtk.vtkRenderer(); ren.SetBackground(1, 1, 1)
    for a in actors: ren.AddActor(a)
    ren.SetUseDepthPeeling(1); ren.SetMaximumNumberOfPeels(8)
    rw = vtk.vtkRenderWindow(); rw.SetOffScreenRendering(1); rw.AddRenderer(ren); rw.SetSize(*size)
    rw.SetAlphaBitPlanes(1); rw.SetMultiSamples(0)
    cam = ren.GetActiveCamera(); cam.SetPosition(*pos); cam.SetFocalPoint(*focal); cam.SetViewUp(*up)
    if parallel: cam.ParallelProjectionOn()
    ren.ResetCamera(); cam.Zoom(zoom)
    # two lights
    l1 = vtk.vtkLight(); l1.SetPosition(-100, -150, 200); l1.SetIntensity(0.9); ren.AddLight(l1)
    l2 = vtk.vtkLight(); l2.SetPosition(150, 100, 100); l2.SetIntensity(0.5); ren.AddLight(l2)
    rw.Render()
    w2i = vtk.vtkWindowToImageFilter(); w2i.SetInput(rw); w2i.Update()
    wr = vtk.vtkPNGWriter(); wr.SetFileName(fname); wr.SetInputConnection(w2i.GetOutputPort()); wr.Write()

out = sys.argv[1] if len(sys.argv) > 1 else 'out'
mold = lambda: actor(f'{out}/rocket-computer-mini-reflow-mold.stl', (0.80, 0.82, 0.88))
# 1. mold isometric from the SMA-slot end
render([mold()], f'{out}/render_mold_iso.png', pos=(-90, -150, 150), zoom=1.5)
# 2. mold top view (parallel)
render([mold()], f'{out}/render_mold_top.png', pos=(0, 0, 300), up=(0, 1, 0), zoom=1.55, parallel=True, size=(900, 1500))
# 3. carrier in use, flipped, with board + components, silicone translucent
sil = actor(f'{out}/_silicone.stl', (0.93, 0.55, 0.35), opacity=0.55, flip=True)
brd = actor(f'{out}/_board.stl', (0.10, 0.45, 0.20), flip=True)
cmp = actor(f'{out}/_components.stl', (0.25, 0.25, 0.28), flip=True)
render([sil, brd, cmp], f'{out}/render_carrier_in_use.png', pos=(90, -150, 110), zoom=1.5)
# 4. carrier alone, flipped, pockets up
sil2 = actor(f'{out}/_silicone.stl', (0.93, 0.55, 0.35), opacity=1.0, flip=True)
render([sil2], f'{out}/render_carrier_empty.png', pos=(-80, -140, 130), zoom=1.5)
print("rendered")
