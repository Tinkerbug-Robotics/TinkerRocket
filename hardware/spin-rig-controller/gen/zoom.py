"""Close-up of a board window: F.Cu red, B.Cu blue, In2 green, from KiCad's per-layer SVG exports.
python3 zoom.py x0 y0 x1 y1 out.png   (board mm, origin top-left)"""
import os, re, subprocess, sys
from PIL import Image, ImageChops
K = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
PCB = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'spin-rig-controller.kicad_pcb')
TMP = os.path.join(__import__('tempfile').gettempdir(), 'spin-rig-zoom')
os.makedirs(TMP, exist_ok=True)
x0, y0, x1, y1 = map(float, sys.argv[1:5]); out = sys.argv[5]
DPI = 1000
layers = [('B.Cu', (60, 110, 230)), ('In2.Cu', (60, 170, 60)), ('F.Cu', (210, 40, 40))]
base = None
for L, col in layers:
    svg = os.path.join(TMP, L + '.svg')
    subprocess.run([K, 'pcb', 'export', 'svg', '--layers', L + ',Edge.Cuts', '--mode-single', '--exclude-drawing-sheet',
                    '--page-size-mode', '0', '-o', svg, PCB], capture_output=True)
    head = open(svg).read(2000)
    vb = [float(v) for v in re.search(r'viewBox="([^"]+)"', head).group(1).split()]
    png = svg[:-4] + '.png'
    subprocess.run(['rsvg-convert', '-d', str(DPI), '-p', str(DPI), '-b', 'white', svg, '-o', png], check=True)
    im = Image.open(png).convert('L')
    sx = im.size[0] / vb[2]; sy = im.size[1] / vb[3]
    box = (int((100 + x0 - vb[0]) * sx), int((100 + y0 - vb[1]) * sy), int((100 + x1 - vb[0]) * sx), int((100 + y1 - vb[1]) * sy))
    m = im.crop(box).point(lambda v: 255 if v < 128 else 0)
    if base is None:
        base = Image.new('RGB', m.size, (255, 255, 255))
    color = Image.new('RGB', m.size, col)
    alpha = m.point(lambda v: 140 if v else 0)
    base.paste(color, (0, 0), alpha)
base.save(out)
print(out, base.size)
