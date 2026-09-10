# Base station — fabrication and assembly notes

**Stencil only.** This file was created to record the one fab parameter that has
been decided for this board (#959). Every other Block A item — surface finish,
via protection, inner-layer copper weight, stackup — is still unrecorded here
and still cannot travel in the design files: it is absent from the gerbers, the
`.gbrjob`, the Excellon drill file, IPC-2581 and ODB++. Do not read this file's
brevity as "the rest is default".

## Block A — bare-board fabrication

| Item | Value | Why |
|---|---|---|
| Stencil | **100 µm (4 mil)**, flat, no step. Laser cut, electropolished and nano-coated. | Repo default set 2026-09-09 (#959). Tightest aperture on this board is F.Cu U12 MP2672AGD 0.71 x 0.20 mm at **AR 0.78** on a 100 µm foil, against the IPC-7525 floor of 0.66. Nothing here needs a foil thinner than **118 µm**, so 100 µm is the fleet default rather than a board constraint. |

## The rule

`AR = aperture area / (aperture perimeter × foil thickness)`, floor **0.66**,
computed from the **aperture** and not the pad — apertures carry KiCad's
`solder_paste_margin` and `solder_paste_margin_ratio`, and some are drawn as
polygons on the paste layer rather than as pads, so a pad-only scan misses them.

Two boards are deliberate exceptions to the 100 µm default, both recorded in
their own files: `rocket-computer-mini` at **80 µm** (its 24-ball WLCSP flash
falls to AR 0.64 at 100 µm) and `gnss-sam10m8-18mm-hv` at **120 µm** (a u-blox
requirement, UBX-22020019 R02 §4.4.1, not an area-ratio limit).

The numbers above come from the analysis on #959, which parsed pad geometry and
paste-layer membership straight from the `.kicad_pcb` for all eight boards. There
is no automated check: a pad-only scan would miss the polygon apertures, and
missing those is exactly what made two rows of that analysis 13 µm optimistic
until they were re-scanned. Recompute by hand if a footprint changes.
