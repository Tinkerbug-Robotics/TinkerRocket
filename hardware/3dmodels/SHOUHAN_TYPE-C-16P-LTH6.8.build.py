import cadquery as cq

# SHOU HAN TYPE-C 16P LTH6.8 (LCSC C5187468): vertical USB Type-C receptacle, USB 2.0,
# 16 contacts. From the SHOU HAN drawing (VER A, 07/19/21): shell 8.94 x 3.16 outside,
# 8.34 x 2.56 opening (0.30 SUS304), 6.80 tall over the board; mating cavity 5.20 deep
# with a 6.69 x 0.65 tongue 4.45 long; four 0.50 x 0.30 shell legs bent out to the
# plated slots at x +/-2.40, y +/-2.15 and 0.95 long below the seating plane; two
# 0.40 locating posts at x +/-3.75, 0.40 long; 16 SMT tails, 0.20 wide, in two rows
# of eight. Drawn at nominal size in the footprint frame of SHOUHAN_TYPE-C-16P-LTH6.8:
# origin at the connector centre, A row toward +Y (the footprint's -y), seated on
# the board at z = 0.
W, D, T = 8.94, 3.16, 0.30          # shell outside and wall
H = 6.80
STANDOFF = 0.12                     # housing bottom over the board
CAVITY = 5.20                       # mating depth from the shell top
TONGUE_W, TONGUE_T, TONGUE_L = 6.69, 0.65, 4.45
LEG_X, LEG_Y = 2.40, 2.15
LEG_W, LEG_T, LEG_BELOW = 0.50, 0.30, 0.95
LEG_BEND_Z = 0.45                   # height of the outward bend
POST_X, POST_D, POST_BELOW = 3.75, 0.40, 0.40
TAIL_XS = (-2.75, -1.25, -0.75, -0.25, 0.25, 0.75, 1.25, 2.75)
TAIL_Y, TAIL_W, TAIL_L, TAIL_T = 0.865, 0.20, 0.75, 0.10

STEEL = cq.Color(0.78, 0.79, 0.81)
BLACK = cq.Color(0.10, 0.10, 0.11)
GOLD = cq.Color(0.85, 0.68, 0.25)


def stadium(w, d, z0, z1):
    return (cq.Workplane("XY").workplane(offset=z0)
            .slot2D(w, d).extrude(z1 - z0))


def box(x0, x1, y0, y1, z0, z1):
    return (cq.Workplane("XY").workplane(offset=z0)
            .center((x0 + x1) / 2, (y0 + y1) / 2)
            .rect(x1 - x0, y1 - y0).extrude(z1 - z0))


floor = H - CAVITY
shell = stadium(W, D, STANDOFF, H).cut(stadium(W - 2 * T, D - 2 * T, STANDOFF - 1, H + 1))
for sx in (-1, 1):
    for sy in (-1, 1):
        y_in, y_out = sy * (D / 2 - T / 2), sy * (LEG_Y + LEG_T / 2)
        bend = box(sx * LEG_X - LEG_W / 2, sx * LEG_X + LEG_W / 2,
                   min(y_in, y_out), max(y_in, y_out), LEG_BEND_Z - LEG_T, LEG_BEND_Z)
        leg = box(sx * LEG_X - LEG_W / 2, sx * LEG_X + LEG_W / 2,
                  sy * LEG_Y - LEG_T / 2, sy * LEG_Y + LEG_T / 2, -LEG_BELOW, LEG_BEND_Z)
        shell = shell.union(bend).union(leg)

# Housing: the base inside the shell up to the cavity floor, the tongue, and the
# two locating posts. The base is 0.01 inside the shell wall so no faces are shared.
housing = stadium(W - 2 * T - 0.02, D - 2 * T - 0.02, STANDOFF, floor)
housing = housing.union(box(-TONGUE_W / 2, TONGUE_W / 2, -TONGUE_T / 2, TONGUE_T / 2,
                            floor - 0.01, floor + TONGUE_L))
for sx in (-1, 1):
    housing = housing.union(cq.Workplane("XY").workplane(offset=-POST_BELOW)
                            .center(sx * POST_X, 0).circle(POST_D / 2)
                            .extrude(POST_BELOW + STANDOFF + 0.01))

tails = [box(x - TAIL_W / 2, x + TAIL_W / 2, sy * TAIL_Y - TAIL_L / 2, sy * TAIL_Y + TAIL_L / 2,
             0, TAIL_T + 0.03)
         for sy in (1, -1) for x in TAIL_XS]

assy = cq.Assembly(name="SHOUHAN_TYPE-C-16P-LTH6.8")
assy.add(shell, name="shell", color=STEEL)
assy.add(housing, name="housing", color=BLACK)
for i, t in enumerate(tails):
    assy.add(t, name=f"tail{i + 1}", color=GOLD)
assy.export("SHOUHAN_TYPE-C-16P-LTH6.8.step")
