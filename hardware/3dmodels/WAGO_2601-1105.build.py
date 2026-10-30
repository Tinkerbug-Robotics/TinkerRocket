import cadquery as cq

# WAGO 2601-1105 PCB terminal block: lever-operated Push-in CAGE CLAMP, 1.5 mm2,
# 3.5 mm pitch, 5 poles, wire entry parallel to the board (0 deg), THT. From the
# WAGO data sheet (stan 24.07.2025): width L = (poles - 1) x 3.5 + 5 = 19, height
# above the board 12.75 (20.9 with a lever open), housing depth 14.5 and 16.58 over
# the closed lever tips; two 1 x 0.5 solder pins per pole, 3.6 long, in rows 4.15
# from the back face and 5 apart; pole 1 sits 2.44 in from its end, which carries
# a 0.75 mm side key. Drawn at nominal size in the footprint frame of
# WAGO_2601-1105: pad 1's front-row pin at the origin, poles along +X, the wire
# entry face toward -Y (the footprint's +y), seated on the board at z = 0.
N, P = 5, 3.5
X0, X1 = -2.44, (N - 1) * P + 2.56      # housing ends
Y_FRONT, Y_BACK = -5.35, 9.15           # wire-entry face, back face
H = 12.75
TIP_Y = Y_FRONT - (16.58 - 14.5)        # closed lever tips overhang the entry face
KEY_W = 0.75                            # side key on pole 1's end
KEY_Z = (9.6, 12.6)
KEY_YS = ((0.65, 1.65), (4.45, 5.55))
PIN_W, PIN_T, PIN_L = 1.0, 0.5, 3.6     # solder pin cross-section and length below
ROWS_Y = (0.0, 5.0)

LEVER_W = 2.8                           # lever width in its slot
LEVER_T = 0.8                           # lever thickness lying closed on top
LEVER_Y0 = 3.75                         # pivot end, from the open-lever side view
TIP_Z0 = 10.4                           # lever tip's lower edge on the entry face
SLOT_W = 3.0                            # housing slot the lever lies in

ENTRY_W, ENTRY_Z = 2.4, (1.6, 5.3)      # wire entry opening on the front face
ENTRY_D = 4.6                           # stops short of the front pin row
TEST_W, TEST_Z = 1.0, (5.7, 6.6)        # test-probe slot above the wire entry

GREY = cq.Color(0.62, 0.63, 0.64)
ORANGE = cq.Color(0.95, 0.42, 0.10)
TIN = cq.Color(0.75, 0.76, 0.78)


def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").workplane(offset=z0)
            .center((x0 + x1) / 2, (y0 + y1) / 2)
            .rect(x1 - x0, y1 - y0).extrude(z1 - z0))


cols = [i * P for i in range(N)]

housing = box(X0, X1, Y_FRONT, Y_BACK, 0, H)
for (ya, yb) in KEY_YS:
    housing = housing.union(box(X0 - KEY_W, X0 + 0.01, ya, yb, *KEY_Z))
for x in cols:
    # Lever slot in the top, open to the entry face.
    housing = housing.cut(box(x - SLOT_W / 2, x + SLOT_W / 2,
                              Y_FRONT - 1, LEVER_Y0 + 0.2, H - LEVER_T - 0.05, H + 1))
    housing = housing.cut(box(x - ENTRY_W / 2, x + ENTRY_W / 2,
                              Y_FRONT - 1, Y_FRONT + ENTRY_D, *ENTRY_Z))
    housing = housing.cut(box(x - TEST_W / 2, x + TEST_W / 2,
                              Y_FRONT - 1, Y_FRONT + 2.0, *TEST_Z))

# Each closed lever: a flat arm in its slot plus a tip that hangs over the entry
# face; the tip's underside is bevelled, as in the side view. Kept 0.02 clear of
# the housing so no faces are shared (coplanar faces z-fight in the 3D viewer).
levers = []
for x in cols:
    arm = box(x - LEVER_W / 2, x + LEVER_W / 2, Y_FRONT + 0.02, LEVER_Y0,
              H - LEVER_T, H - 0.02)
    tip = (cq.Workplane("YZ").workplane(offset=x - LEVER_W / 2)
           .polyline([(Y_FRONT + 0.5, H - LEVER_T), (Y_FRONT + 0.5, H - 0.02),
                      (TIP_Y, H - 0.02), (TIP_Y, H - 0.9),
                      (Y_FRONT - 0.02, TIP_Z0), (Y_FRONT - 0.02, H - LEVER_T)])
           .close().extrude(LEVER_W))
    levers.append(arm.union(tip))

pins = [box(x - PIN_W / 2, x + PIN_W / 2, y - PIN_T / 2, y + PIN_T / 2, -PIN_L, 0.3)
        for x in cols for y in ROWS_Y]

assy = cq.Assembly(name="WAGO_2601-1105")
assy.add(housing, name="housing", color=GREY)
for i, lv in enumerate(levers):
    assy.add(lv, name=f"lever{i + 1}", color=ORANGE)
for i, p in enumerate(pins):
    assy.add(p, name=f"pin{i // 2 + 1}_{i % 2 + 1}", color=TIN)
assy.export("WAGO_2601-1105.step")
