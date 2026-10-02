import cadquery as cq

# Maxim 28-pin thin QFN, outline 21-0140 code T2855-3 (MAX2769B and MAX2771): body 5.0 x 5.0,
# 0.80 max tall, 28 terminals on a 0.5 pitch, each 0.25 wide x 0.40 long (nominal), 0.20 thick
# (A3), and a 3.25 x 3.25 exposed pad. Nominal values. KiCad ships the footprint but not this
# model (its model path names a file that is not in the KiCad 10 library), hence this one.
# Frame of footprint TQFN-28-1EP_5x5mm_P0.5mm_EP3.25x3.25mm: origin at the body centre, seated
# at z = 0. KiCad model +Y is footprint -Y, so pin 1 (footprint (-2.36, -1.5)) is the left
# column's top terminal at y = +1.5 here.
D, A, A1, A3 = 5.0, 0.75, 0.02, 0.20
E_PITCH, B, L = 0.5, 0.25, 0.40
EP = 3.25
N_SIDE = 7

BODY = cq.Color(0.10, 0.10, 0.11)
TIN = cq.Color(0.80, 0.80, 0.82)
MARK = cq.Color(0.30, 0.30, 0.32)

# terminal centres, counter-clockwise from pin 1 (top of the left column, viewed from above)
offs = [E_PITCH * (k - (N_SIDE - 1) / 2) for k in range(N_SIDE)]
edge = D / 2 - L / 2
terms = ([(-edge, -o, L, B) for o in offs] +       # pins 1-7: left side, top to bottom
         [(o, -edge, B, L) for o in offs] +        # pins 8-14: bottom side, left to right
         [(edge, o, L, B) for o in offs] +         # pins 15-21: right side, bottom to top
         [(-o, edge, B, L) for o in offs])         # pins 22-28: top side, right to left

body = cq.Workplane("XY").workplane(offset=A1).rect(D, D).extrude(A - A1)
for x, y, sx, sy in terms:
    body = body.cut(cq.Workplane("XY").center(x, y).rect(sx, sy).extrude(A3))
# Pin 1 index: a dot on the top face at the pin 1 corner.
dot = (cq.Workplane("XY").workplane(offset=A)
       .center(-D / 2 + 0.6, D / 2 - 0.6).circle(0.25).extrude(0.005))

assy = cq.Assembly(name="TQFN-28_5x5")
assy.add(body, name="body", color=BODY)
assy.add(dot, name="pin1_mark", color=MARK)
assy.add(cq.Workplane("XY").rect(EP, EP).extrude(A1), name="exposed_pad", color=TIN)
for n, (x, y, sx, sy) in enumerate(terms, 1):
    term = cq.Workplane("XY").center(x, y).rect(sx, sy).extrude(A3)
    assy.add(term, name=f"terminal{n}", color=TIN)
assy.export("TQFN-28-1EP_5x5mm_P0.5mm_EP3.25x3.25mm.step")
