# Base station mini — fabrication and assembly notes

**Stencil only.** This file was created to record the one fab parameter that has
been decided for this board (#959). Every other Block A item — surface finish,
via protection, inner-layer copper weight, stackup — is still unrecorded here
and still cannot travel in the design files: it is absent from the gerbers, the
`.gbrjob`, the Excellon drill file, IPC-2581 and ODB++. Do not read this file's
brevity as "the rest is default".

## Block A — bare-board fabrication

| Item | Value | Why |
|---|---|---|
| Stencil | **100 µm (4 mil)**, flat, no step. Laser cut, electropolished and nano-coated. | Repo default set 2026-09-09 (#959). Tightest aperture on this board is F.Cu U3 0.65 x 0.22 mm at **AR 0.82** on a 100 µm foil, against the IPC-7525 floor of 0.66. Nothing here needs a foil thinner than **125 µm**, so 100 µm is the fleet default rather than a board constraint. |

## The rule

Stencil thickness, the area-ratio floor and the paste-coverage convention are
repo-wide and live in one place: **[`hardware/SOLDER-PASTE-CONVENTION.md`](../SOLDER-PASTE-CONVENTION.md)**
(#959, #906). The short version is that this board takes the 100 µm default,
apertures must clear AR 0.66 at that thickness, and coverage stays at full pad
unless a named mechanism justifies reducing it.
