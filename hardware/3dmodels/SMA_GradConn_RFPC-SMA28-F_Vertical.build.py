import cadquery as cq

# GCT (formerly GradConn) RFPC-SMA28-F, SMA jack, straight, through-hole. Sizes: a 7.0 mm square
# flange on four small standoffs, a 6.5 mm collar, a 1/4-36 thread to 9.6 mm above the seating
# plane, a PTFE insulator around the centre contact, and four 1.0 mm square ground posts plus the
# centre pin reaching 3.9 mm below the seating plane. Drawn in the frame of footprint
# SMA_GradConn_RFPC-SMA28-F_Vertical: origin on the centre pin, seated on the board at z = 0,
# posts at (+/-2.54, +/-2.54). Simplified: no thread form, no hex.
FLANGE, FLANGE_Z0, FLANGE_Z1 = 7.0, 0.5, 1.8
COLLAR_D, COLLAR_Z1 = 6.5, 3.5
THREAD_D, THREAD_BORE, TOP = 6.35, 4.6, 9.6
PTFE_D, PTFE_TOP = 4.1, 7.57
PIN_D, POST = 1.27, 1.0
BELOW = 3.9
P = 2.54

GOLD = cq.Color(0.85, 0.70, 0.28)
PTFE = cq.Color(0.95, 0.95, 0.93)

posts = cq.Workplane("XY")
for x, y in ((-P, -P), (P, -P), (-P, P), (P, P)):
    posts = posts.union(cq.Workplane("XY").workplane(offset=-BELOW).center(x, y)
                        .rect(POST, POST).extrude(BELOW + FLANGE_Z0))
standoffs = cq.Workplane("XY")
for x, y in ((-P, -P), (P, -P), (-P, P), (P, P)):
    standoffs = standoffs.union(cq.Workplane("XY").center(x, y).rect(1.8, 1.8).extrude(FLANGE_Z0))
flange = (cq.Workplane("XY").workplane(offset=FLANGE_Z0).rect(FLANGE, FLANGE).extrude(FLANGE_Z1 - FLANGE_Z0)
          .faces(">Z").workplane().hole(PTFE_D))
collar = (cq.Workplane("XY").workplane(offset=FLANGE_Z1).circle(COLLAR_D / 2).circle(PTFE_D / 2)
          .extrude(COLLAR_Z1 - FLANGE_Z1))
thread = (cq.Workplane("XY").workplane(offset=COLLAR_Z1).circle(THREAD_D / 2).circle(THREAD_BORE / 2)
          .extrude(TOP - COLLAR_Z1))
body = posts.union(standoffs).union(flange).union(collar).union(thread)
ptfe = (cq.Workplane("XY").workplane(offset=FLANGE_Z0).circle(PTFE_D / 2).circle(PIN_D / 2 + 0.05)
        .extrude(PTFE_TOP - FLANGE_Z0))
pin = cq.Workplane("XY").workplane(offset=-BELOW).circle(PIN_D / 2).extrude(BELOW + PTFE_TOP)

assy = cq.Assembly(name="RFPC-SMA28-F")
assy.add(body, name="body", color=GOLD)
assy.add(ptfe, name="insulator", color=PTFE)
assy.add(pin, name="centre_pin", color=GOLD)
assy.export("SMA_GradConn_RFPC-SMA28-F_Vertical.step")
