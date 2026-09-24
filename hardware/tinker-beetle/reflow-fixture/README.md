# rocket-computer-mini v1.01 — open-frame reflow carrier mold

3D-printable mold for casting a **Mold Max 60** silicone carrier that holds the board
by its edges — terminal-block side (KiCad `B.Cu`) down, already soldered — while the
GNSS / ESP32 side (`F.Cu`) is reflowed second in the oven.

The carrier is a **ring**, not a block. The board drops into a 1.7 mm recess and rests on
narrow silicone lips that exist only where the back side is bare right at the board edge
(144 mm² in 12 places). Everything inside the lips is an open window, so the whole back
side sees the oven directly and nothing but those lips touches the board. The ring is
14 mm tall so the terminal block J2 (9.85 mm) clears the rack; a thin floor 0.3 mm under
J2 catches it if its joints go liquid. J2's body overhangs the board end, so the end wall
carries a notch; the SMA edge connector gets an open slot through the other end wall.

Put the carrier on the oven's **wire rack**, not a solid tray, so air reaches the board
from below through the window.

## Files

| File | What it is |
|---|---|
| `rocket-computer-mini-reflow-mold.stl` | Print this. 37.5 × 84.6 × 19 mm, one solid, ~40 cm³. |
| `rocket-computer-mini-reflow-mold.step` | Same geometry as a solid — the **verified reference** (every back-side part checked against the STEP solids, see below). |
| `rocket-computer-mini-reflow-carrier-silicone.step` | The cast ring (32.5 × 79.6 × 14 mm) for checking fit in CAD. |
| `fusion/ReflowMold/` | Fusion 360 script that rebuilds the mold natively (9 sketches / features, user parameters `sil_h`, `freeboard`, `floor_t`, `groove_w`, `board_t`, plus the cast ring as a hidden reference body). Run from *Utilities ▸ Add-Ins ▸ Scripts and Add-Ins* (Shift+S) ▸ **+** ▸ *Script or add-in from device* ▸ pick this folder ▸ Run. It opens a new design. |
| `renders/` | Mold, empty ring, ring with the board in it, and the top-view map. |
| `generator/` | The Python that produced all of the above from `rocket-computer-mini-v1_01.step` + the `.kicad_pcb` (CadQuery + shapely). Re-run after a board revision. |

## How the shape was derived

Every back-side footprint with a 3D model in the STEP export (104 parts; C130 the supercap
excluded because it is fitted after the second reflow) was matched to its KiCad reference by
position and bounded in mold coordinates.

* Each part is dilated by 0.5 mm clearance plus the draft compensation down to its own
  underside (the mold core has 1.5° draft, so the window narrows going down; J2 gets 0.76 mm
  at lip level, J3 0.68, S1 0.65, J8 0.61).
* **Lips** = the band from the recess wall to 1.5 mm inside the board edge, minus every
  dilated part, minus fragments narrower than 0.6 mm or shorter than 2 mm. The back side is
  populated to within 0.5 mm of both long edges in many places, so the lips are short
  segments — but there are 12 of them around the perimeter, including all four corners.
* **Window** = recess minus lips, plus J2's overhang notch (2.4 mm into the +Y end wall) and
  the SMA slot (out through the −Y end wall). Open right through the ring except under J2.
* **J2 catch floor**: the mold core is lowered to 11.85 mm over J2 (+1 mm margin), so the ring
  has a 2.15 mm floor there, 0.3 mm below the terminal block's underside.
* Board recess: outline + 0.2 mm per side, 1.7 mm deep (1.53 mm core + copper as modelled).
* Fill-line groove on the inside walls at 14 mm; the label is mirrored on the mold floor so it
  reads correctly on the ring.

Verification (`generator/verify.py`, real STEP solids transformed into the carrier): all 104
parts clear the silicone — J2 sits exactly 0.30 mm above its catch floor, S1 0.39 mm, every
edge part ≥ 0.45 mm, zero volume overlap — and the board solid sits on the lips without
intersecting the silicone.

Why not a block with pockets (the first version): silicone is an insulator (~0.2 W/m·K) and a
25 g block adds ~30 J/K of thermal mass under a ~5 g board, so the second-side profile would
have run cold and slow; and with through-pockets the terminal block would have stood proud of
an 8 mm block. The open frame fixes both.

## Casting notes (Mold Max 60, from the Smooth-On TB)

* 100A : 3B by weight, mixed viscosity 20,000 cps, pot life 40 min, cure 24 h at 23 °C,
  Shore 60A, shrinkage 0.0015 in/in (≈ 0.1 mm over the board length — ignored), heat
  resistance to 560 °F / 294 °C. Vacuum degas after mixing (29 in Hg minimum).
* Silicone needed: **≈ 23 g** (15.7 mL × SG 1.45). Mix ~35 g to have margin.
* Print the mold in PLA or PETG, 0.2 mm layers; the core is a solid block, so 15–20 % infill
  is fine. Mist with Ease Release 200 and let it dry 30 min before pouring; tin-cure silicone is
  not inhibited by common print materials, but if in doubt seal with clear acrylic lacquer as
  the TB suggests.
* Pour into the channel around the core up to the groove, tap to clear the narrow lip channels,
  cure 24 h. Demold by flexing the box walls, then stretch the ring up off the core (the 1.5°
  draft helps).
* Optional post-cure 4 h at 150 °F / 65 °C after demolding (out of the PLA mold).

## Parameters (generator/design.py)

| Parameter | Value | Note |
|---|---|---|
| `board_clr` | 0.20 mm | recess clearance per side |
| `board_t` | 1.70 mm | recess depth |
| `wall` / `sil_h` | 5.0 / 14.0 mm | ring width / height (fill line) |
| `lip_max` / `lip_min` / `lip_min_len` | 1.5 / 0.6 / 2.0 mm | lip rules |
| `clr_xy` | 0.50 mm | part-to-window clearance at lip level |
| `catch_clr` / `catch_margin` | 0.30 / 1.0 mm | J2 catch floor |
| `draft_deg` | 1.5° | mold core draft (clearance compensated) |
| `mold_wall` / `floor_t` / `freeboard` | 2.5 / 2.0 / 3.0 mm | mold box |

Regenerate: `pip install cadquery shapely` then, in `generator/`,
`python step_inspect.py <board.step> leaves.json`, `python match.py <board.kicad_pcb>`,
`python build_cq.py out`, `python verify.py`, `python render.py out`, `python pocket_map.py out`,
`python export_fusion_data.py`, `python make_fusion_script.py`.
