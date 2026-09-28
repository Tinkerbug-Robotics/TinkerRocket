# Tinker-Base (base station mini) — fabrication and assembly notes

**Stencil only.** This file was created to record the one fab parameter that has
been decided for this board (#959). Every other Block A item — surface finish,
via protection, inner-layer copper weight, stackup — is still unrecorded here
and still cannot travel in the design files: it is absent from the gerbers, the
`.gbrjob`, the Excellon drill file, IPC-2581 and ODB++. Do not read this file's
brevity as "the rest is default".

## Block A — bare-board fabrication

| Item | Value | Why |
|---|---|---|
| Stencil | **80 µm (3 mil)**, flat, no step. Laser cut, electropolished and nano-coated. A deliberate exception to the 100 µm repo default (#959), and the same foil as the Tinker-Beetle. | The finest aperture is now the 24-ball WLCSP boot flash (`U1`): 0.254 mm round pads at **AR 0.64** on a 100 µm foil, below the IPC-7525 floor of 0.66, and **0.79** at 80 µm. Next come the chip antenna's 0.30 × 0.30 mm pads (`U15`, 0.94 at 80 µm) and the ESP32-S3's 0.65 × 0.22 mm pins (`U3`, 1.03 at 80 µm, 0.82 at 100 µm). No through-hole pad carries paste. All of these figures are measured from the board's paste layer, where each aperture matches its pad 1:1. |

## The rule

Stencil thickness, the area-ratio floor and the paste-coverage convention are
repo-wide and live in one place: **[`hardware/SOLDER-PASTE-CONVENTION.md`](../SOLDER-PASTE-CONVENTION.md)**
(#959, #906). The short version: this board takes an 80 µm foil instead of the
100 µm default because of `U1`, apertures must clear AR 0.66 at that
thickness, and coverage stays at full pad unless a named mechanism justifies
reducing it.
