#!/usr/bin/env python3
"""Shaded previews of the bench rig (rig STL + board stand-in STL) with VTK, offscreen: an iso view from the front-left
and a straight top view. -> rig_render_iso.png, rig_render_top.png    render_rig.py"""
from pathlib import Path

import vtk

HERE = Path(__file__).resolve().parent


def actor(stl, rgb, spec=0.2):
    r = vtk.vtkSTLReader()
    r.SetFileName(str(HERE / stl))
    n = vtk.vtkPolyDataNormals()
    n.SetInputConnection(r.GetOutputPort())
    n.SetFeatureAngle(35)
    m = vtk.vtkPolyDataMapper()
    m.SetInputConnection(n.GetOutputPort())
    a = vtk.vtkActor()
    a.SetMapper(m)
    p = a.GetProperty()
    p.SetColor(*rgb)
    p.SetSpecular(spec)
    p.SetSpecularPower(20)
    return a


def render(name, pos, up, parallel=False, size=(1600, 1100)):
    ren = vtk.vtkRenderer()
    ren.SetBackground(1, 1, 1)
    ren.AddActor(actor("hackrf_pro_bench_rig.stl", (0.93, 0.60, 0.20)))
    ren.AddActor(actor("boards_standin.stl", (0.13, 0.50, 0.28), 0.4))
    cam = ren.GetActiveCamera()
    cam.SetFocalPoint(145, 95, 8)
    cam.SetPosition(*pos)
    cam.SetViewUp(*up)
    if parallel:
        cam.ParallelProjectionOn()
    ren.ResetCamera()
    cam.Zoom(1.25 if not parallel else 1.15)
    light = vtk.vtkLight()
    light.SetPosition(-200, -300, 500)
    light.SetFocalPoint(145, 95, 0)
    ren.AddLight(light)
    win = vtk.vtkRenderWindow()
    win.SetOffScreenRendering(1)
    win.AddRenderer(ren)
    win.SetSize(*size)
    win.Render()
    w2i = vtk.vtkWindowToImageFilter()
    w2i.SetInput(win)
    w2i.SetScale(1)
    w2i.Update()
    wr = vtk.vtkPNGWriter()
    wr.SetFileName(str(HERE / name))
    wr.SetInputConnection(w2i.GetOutputPort())
    wr.Write()
    print(f"{name} written")


render("rig_render_iso.png", (-60, -260, 300), (0, 0, 1))
render("rig_render_top.png", (145, 95, 600), (0, 1, 0), parallel=True)
