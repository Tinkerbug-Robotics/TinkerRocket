# Tinker-Beetle V2: TPU support for C130 standing upright

C130 (CDA CXHP2R7205R-TW, 2 F, Ø8 × 12 mm) stands on its own two holes instead of lying on
the GNSS shield. That leaves the BMI-S-202-C cover free to come off and snap back on (production
review item 4). The can stands 2.0 mm above the board, clear of every part under it. This
printed TPU sleeve carries it and is staked with neutral-cure RTV to the can and to the board.

The holes don't change: the footprint's 3.5 mm pair takes the straight leads of a standing can. The
footprint stays `Supercap_Radial_D8.0mm_L12.0mm_P3.50mm_Horizontal` on purpose. A standard vertical
footprint would draw the can's courtyard on the board, over the parts the can stands above. Only the
board's C130 3D model changed: it is now `${KIPRJMOD}/c130-support/c130-upright-with-support.step`
with zero offset, where it was the horizontal can at 4 mm.

## Files

| File | What it is |
|---|---|
| `c130-support_print-rim-down.stl` | **Print this.** Already rim-down on the bed; no supports needed. |
| `c130-support.step` | The same part in the board frame (origin at C130's hole midpoint on the back face, +z away from the board), for CAD fit checks. |
| `c130-upright-with-support.step` | The board's 3D model for C130: the can upright on the seat plus the support, in the footprint's own frame. The board's C130 points at it with zero offset and rotation, so KiCad's 3D view and STEP export show C130 standing. |
| `fit.json` | The parameters used and the measured height of every back-side part nearby. |
| `renders/` | The part alone, from below, and on the board with the can (iso, plan, side); `kicad_back*.png` are KiCad's own renders with the new model. |
| `generator/` | `board_map.py` (KiCad Python) reads the board; `build_c130_support.py` (CadQuery + shapely) builds and checks the part; `render.py` (VTK) draws the views; `kicad_model.py` makes the board's 3D model. |

## The part

- **Sleeve:** 8.4 mm bore and 1.2 mm wall (three 0.4 mm perimeters), gripping the lower 8 mm of the can.
  - There's a 0.6 mm lead-in chamfer at the rim.
  - A notch in the rim marks the **+ lead** (pad 1, V_SCAP, the +x hole toward U30 and U2).
  - The wall is trimmed flat where it passes the shield cover. It keeps 0.30 mm to the cover's outline (the frame plus the cover's 0.23 mm skirt).
- **Seat:** a 45° cone inside the sleeve's base, narrowing the bore to 6.8 mm. The can's rounded bottom edge rests on it, which puts the can 2.0 mm above the board.
  - The leads pass straight down through the open centre, with 0.79 mm clearance.
- **Foot:** grows from the sleeve's base toward the board at 45°, only into free board (0.3 mm off every courtyard and pad).
  - That gives one 24.9 mm² pad on the east side, between U30/C17 and U2.
  - Everywhere else the sleeve passes over low parts: C6 (1.33 mm), CR3 (1.30 mm), U3 (1.11 mm), and D9/D12, C104, U30, R2, R4, C5 and U2 at 0.4–0.95 mm.
  - No foot fits on the west: the strip between C6 and J6 is under 0.5 mm wide after clearances.
- **Size:** 12.5 × 10.6 × 9.7 mm, 0.32 cm³.

## Fit (checked against the board's STEP export, solid to solid)

| Gap | Where |
|---|---|
| 0.37 mm | Sleeve base over C6 and C17 (1.33 mm parts); also the can's bottom over C6 |
| 0.40 mm | Sleeve base over CR3 |
| 0.50 mm and up | Everything else (S1 isn't among the ten closest parts) |
| 0.89 mm | The can to U3, the magnetometer |

**Height:**
- The can's top is 14.0 mm above the back face, or 15.5 mm at the spec's +1.5 mm length tolerance. KiCad seats back-side models on the mask, about 0.08 mm below the board body, so its STEP export shows 14.09 mm. The fit check above measured from the board body, so the real gaps are about 0.08 mm larger.
- The tallest back-side part until now was J2 at 12.8 mm, so the back gets 1.2–2.7 mm taller.
- **Check the bay before flying.**

## Printing

TPU (95A works):
- Print the `_print-rim-down` STL as it lies.
- No supports, 0.1–0.15 mm layers. The foot is built from 0.1 mm steps.
- Three perimeters, 100 % infill.
- All overhangs are 45° or less.

## Fitting (after both reflow passes, as C130 always was)

1. **Measure the can.** The spec allows D 8.0 ± 1.0 mm. If it's over 8.2 mm, set `SLEEVE_ID` to D + 0.3 and rebuild.
2. **Dry-fit the support.**
   - The foot goes east, toward U30 and U2, and the notch over the + hole.
   - Both holes show through the seat.
3. **Stake the foot to the board** with neutral-cure RTV.
4. **Fit the can.**
   - Thread the leads through the seat into the holes, + lead under the notch.
   - Push the can down onto the seat, then solder and trim the leads on the front.
5. **Fill the gap** between the can and the sleeve with RTV from the rim, or coat the can before step 4.
6. **Recalibrate the magnetometer** (U3) with C130 fitted. The can is 0.9 mm from it.

## Rebuilding

```bash
cd hardware/tinker-beetle/c130-support/generator
/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 board_map.py ../../tinker-beetle.kicad_pcb board_map.json
/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli pcb export step --subst-models --no-dnp --output /tmp/beetle.step ../../tinker-beetle.kicad_pcb
python3 build_c130_support.py board_map.json /tmp/beetle.step ..
python3 render.py ..
python3 kicad_model.py ../c130-support.step ../c130-upright-with-support.step
```

The parameters (bore, wall, standoff, grip length, clearances) are at the top of `build_c130_support.py`.

Built 2026-11-25 from the main checkout's board as saved at 14:29: main 5f085ff2 plus the owner's U14 changes.
