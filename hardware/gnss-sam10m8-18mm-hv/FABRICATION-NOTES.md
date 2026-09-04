# gnss-sam10m8-18mm-hv — fabrication and assembly notes

Applies from revision **V3** (LDO supply, 6-layer). Written 2026-09-02. Block A is what goes into the fab order; Block B is what the assembler needs. Numbers come from the board file and the u-blox SAM-M10Q integration manual (UBX-22020019 R02).

## Block A — PCB order (JLCPCB)

| Item | Value | Why |
|---|---|---|
| Layers | **6** — F.Cu (parts + GND pour) / In1 GND / In2 +3V3 plane / In3 GND / In4 GND / B.Cu (module + GND pour) | Decided 2026-09-02; the module's ground reference is In3/In4 directly under it. |
| Stackup | **JLC06161H-3313**, 1.6 mm: Cu 35 µm / prepreg 3313 0.0994 / Cu 15.2 µm / core 0.55 / Cu 15.2 µm / prepreg 2116 0.1088 / Cu 15.2 µm / core 0.55 / Cu 15.2 µm / prepreg 3313 0.0994 / Cu 35 µm | JLC's standard 6-layer 1.6 mm profile; entered in Board Setup, so the gerber job file carries it. |
| Finish | **ENIG** | 20-pad LGA module plus a SOIC exposed pad; HASL doming hurts coplanarity. |
| Vias | 0.4 mm diameter / 0.3 mm hole throughout; **filled and capped** (JLC's default on 6-layer). Board rules: min via 0.4 mm, min ring 0.05 mm, same-net hole-to-hole 0.25 mm. | Vias sit inside the module's pads and under paste; the filled-and-capped process is what makes that acceptable. If the order form offers a via-covering choice, it must be "epoxy filled & capped" (or copper-paste filled & capped), never tented-only or plugged-only. |
| Outline | 22.00 × 27.50 mm, 1 mm corner radius, four 2.2 mm mounting holes | Unchanged from V2. |
| Copper to edge | ≥ 0.5 mm (fills), 0.73 mm nearest track | Above JLC's 0.2 mm routed minimum. |
| Silkscreen | 0.8–0.85 mm text, 0.15 mm stroke; board ID "SAM-M10Q V3" and "TinkerRocket" on the top strip, "PATCH ANT ON BACK" along the bottom edge, "1" beside J3 pin 1 | Nothing under the module; the ID is readable assembled. |
| Stencil | 120 µm (module paste apertures per u-blox 4.4.1 assume it); see #959 for the repo-wide thickness rule | The 2 × 2 windowpane on U1 (61.9 % coverage) is drawn for 120 µm. |

Plot with `tools/plot_gerbers.sh` from the tagged commit, and confirm the `.gbrjob` says `LayerNumber: 6` and `BoardThickness: 1.5994` before uploading.

## Block B — assembly

**U1, SAM-M10Q (B.Cu, the only part on that side).**
- Reflow it **last, board upright**, on the u-blox profile: peak 245 °C, 40–60 s above 217 °C (integration manual 5.3). The module tolerates one production reflow plus one rework reflow; never reflow it upside down.
- **No hot-air rework of U1.** The manual calls a hot-air gun an uncontrolled process that can severely damage the module. If a module must come off, use a bottom preheater with a controlled nozzle or a hot plate, and fit a **new** module; never re-use a removed SAM-M10Q.
- **MSL 4**: 72 h floor life out of the bag. Bake per J-STD-033 before any reflow of a module that has been exposed longer, including any rework of an already-mounted module.
- Paste: the footprint's B.Paste apertures are drawn as polygons (2 × 2 windowpane, 0.58 × 0.72 mm per aperture); do not "add" paste to the pads, it is already there. Mask openings are pad + 0.1 mm per side.
- Nine GND pads tie solid into the pour with filled vias inside them; profile to the joint, not to the board (thermocouple at a U1 pad), and prefer a longer soak.
- No ultrasonic cleaning (the module's oscillators). Alcohol cleaning drives flux under the module; avoid.
- After assembly, record the module's unique ID (the FC prints UBX-SEC-UNIQID at boot) against the carrier serial.

**U2, ADP7142ARDZ-3.3 (SOIC-8 with exposed pad).** Fixed 3.3 V variant **only** (tube `ADP7142ARDZ-3.3`, reel `ADP7142ARDZ-3.3-R7`). SENSE (pin 3) is tied to VOUT and there is no divider: the adjustable `ADP7142ARDZ` would regulate at 1.2 V and the module would never start. Exposed pad to GND through nine filled vias.

**FL1, DLW21SN670HQ2L.** Its paste apertures are also drawn as polygons on F.Paste (four, one per pad), so it is stencilled normally; earlier notes calling it hand-solder-only were wrong.

**D1 / D3 LEDs.** Green `XL-1005UGC` only. D1's forward voltage is what holds the module's TIMEPULSE/SAFEBOOT_N node above threshold at start-up; do not substitute a low-Vf LED and do not replace D1 with a resistor.

**Polarity and identity checks before power.** D2 (SMF10A) cathode bar toward J3 pin 3 (VSYS). J3 pin 1 is marked "1" on the silk; pin 5 carries the module's 1 PPS output to the host, so the host GPIO must stay an input. Before the first power-up read the board ID on the top strip: V3 is the LDO board, V2 the buck board; both fit the same harness.

**Test points.** There is no dedicated test via (no spot inside the +3V3 pour clears the module's B.Cu pads by more than 0.12 mm). Probe +3V3 at C1/C5/C6 or FL2's output pad, VIN at C136, the UART at the outer pads of R7/R8 or at J3, PPS at R9/D2.

## Change history

- 2026-09-02 V3: TPS62913 buck replaced by ADP7142ARDZ-3.3 + BLM18PG471SN1D bead; 6-layer JLC06161H-3313 stackup; filled and capped vias; rev V3. Review: `prefab-review-2026-09-02.md`.
- 2026-08-09 V2 (tag `gnss-sam10m8-18mm-hv-v2.1.0`): TPS62913 buck, 4-layer JLC04161H-7628, ENIG. Review: `prefab-review-2026-08-04.md`.
