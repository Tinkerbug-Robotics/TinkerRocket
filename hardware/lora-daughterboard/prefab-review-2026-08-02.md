# TinkerRocket LoRa daughterboard ("LoRa V3") — Pre-Manufacturing Review #2

**Date:** 2026-08-02
**Source:** `TinkerRocket-Hardware/hardware/lora-daughterboard/` @ `cda8c03` **plus uncommitted working-tree changes** — `CR4` *and* `CR5` are both in-flight placements that exist only in the working copy. This review covers what is on disk now, because that is what would be fabbed.
**Scope:** every component and its supporting parts; system architecture; PCB layout beyond DRC (datasheet compliance and best practice); and a dedicated "anything dumb" sweep for the *lora-missing-ground* / *base-station-floating-VINA* class of defect.
**Method:** 10 domain finders working from a fresh `kicad-cli` netlist / ERC / DRC export of the current working state, each parsing the `.kicad_pcb` s-expression directly for geometry; then 22 independent adversarial verifiers, each instructed to *refute* and to re-derive every number from the files and real datasheets rather than trust the finder. 128 raw findings → 22 blocker/major adjudicated. Datasheets pulled for TPS62913, E220-900MM22S / LLCC68, ESP32-S3 (datasheet + HDG), CUS10S30, SP0503BA, SMF10A, PMPB14XN, W25Q128, and the LED parts. Gerbers, Excellon, IPC-2581 and ODB++ were re-exported and read at file level.

**Supersedes:** `prefab-review-2026-07-30.md` (status of all 19 prior findings is re-verified below).

---

## Verdict

**Do not export gerbers yet.** There is one true blocker and three fab-package defects that are unrecoverable after manufacture. None of them is a design error in the usual sense — the architecture, the buck, the RF path and the module wiring are all sound, and eleven of the prior review's nineteen findings are genuinely closed. What is left is concentrated in two places: **the two TVS placements you are working on right now**, and **the gap between what the KiCad project believes and what the fab actually receives**.

Notably, **zero of the 22 adjudicated findings were refuted** — but 16 of 22 came back `PARTIALLY_CORRECT`, meaning the defect was real while the finder's mechanism, magnitude or proposed fix was wrong. Several of the corrected fixes below are the *opposite* of the obvious one. Read the fixes, not just the titles.

---

## STATUS 2026-08-03 — the fab gate is CLEAR

Every blocker- and major-class item below that touched the board layout has been fixed and re-verified against the file. DRC: **22 → 15 violations, zero clearance errors, zero mask bridges, zero dangling vias, 0 unconnected pads.**

| # | Item | Status |
|---|---|---|
| 1 | `+3V3` via 0.0598 mm from `CR4` pad 1 | **CLOSED** — via moved to (87.78, 124.86); gap now **0.3703 mm**, 3.7× the rule |
| 2 | `+BATT` via bridging `J2`'s shield-pad mask window | **CLOSED** — via moved to (100.69, 121.10); mask web −0.002 → **+0.478 mm** |
| 3a | Filled+capped vias never reach the fab | **OPEN** — needs the `Dwgs.User` fab note; nothing in the file can carry it |
| 3b | Four lands whose drill was ≥ the land dimension | **CLOSED** — `U28.29`, `U3.4`, `U3.7`, `FL1.4` all cleared; **0 such lands remain** (was 4), via-in-pad count 70 → 66 |
| 4 | Surface finish `None` | **CLOSED** — `copper_finish` now `ENIG` |
| 5 | SMA launch capacitive discontinuity | **CLOSED** — see below |
| — | Stackup | **CLOSED** — now JLC06161H-3313, default variant, `(general (thickness))` recomputed to 1.5765 |

**As-built RF launch, measured from the file and computed on the corrected stackup.** The In1+In2 rule area is `copperpour not_allowed`, spans x 91.285–93.536 × y 103.908–111.016, and a raster of the *cached fill polygons* confirms **In1 0 %, In2 0 %, In3 100 %** under all three of the J8 pad, the ANT trace and U14's ANT pad:

| | Z₀ | | Return loss | VSWR | Round trip |
|---|---|---|---|---|---|
| J8 pad | 37.73 Ω | **902 MHz** | 41.7 dB | 1.02 | |
| ANT trace | 77.82 Ω (2.4° — immaterial) | **915 MHz** | 41.6 dB | 1.02 | **0.001 dB** |
| U14 ANT pad | 50.31 Ω | **928 MHz** | 41.4 dB | 1.02 | 0.01 % of range |

Against 10.3 dB / VSWR 1.87 / 9.2 % of range lost with no void. The void half-width ended at **1.1255 mm**, past the ~1.2 mm point where the sensitivity to slot width flattens out — the earlier 0.8825 mm measurement was the most sensitive parameter in the whole analysis, so the wider window is worth having. One GND via at (91.39, 110.83) straddles the window edge; it sits 0.465 mm from the U14 pad, far outside the 0.1275 mm coplanar gap, so it is immaterial.

**Still open before ordering:** the fab note (item 3 — it is the only remaining fab-gate item), the three fiducial/courtyard overlaps (`FID1`/`J2`, `FID4`/`U14`, `FID6`/`J6` — the three fiducials that are unusable anyway), the silkscreen items, and the non-layout items in the sections below.

---

## Fab gate — fix before exporting gerbers

### 1. BLOCKER — `+3V3` via sits 0.0598 mm from `CR4` pad 1 (GND)

`(via (at 87.47 124.81) (size 0.4) (drill 0.3) ... (net "+3V3"))` against `CR4` pad 1 (GND) at (86.7852, 124.34745). Copper-to-copper gap **0.05980 mm** — 40 % under the board's own 0.1 mm rule and 34 % under JLC-class 6-layer minimum (0.09 mm). Re-derived independently by three verifiers, all landing on the same figure to five decimals.

This is **new since 2026-07-30** and comes from the in-flight `CR4` placement — the prior review's baseline explicitly called the FID1/J2 courtyard overlap "the board's only DRC error". `CR4` is `(dnp no) (attr smd)`, so it is genuinely populated.

`+3V3` feeds `U28` pins 20/46/55/56, `U14` pin 1 (module VCC) and `U22` pin 8 (flash VCC). A bridge kills the MCU, the radio and the boot flash simultaneously. The via is mask-tented, so the reflow-bridge path is mitigated and this is an etch/plating yield risk rather than a certain short — but you cannot release gerbers with a hard clearance error 34 % under the fab floor, and it is unfixable after fab.

**Fix — move the via EAST, not north.**

```
current   (87.47, 124.81):  gap to CR4.1 = 0.0598   <-- violation
north +0.25 (124.56):       gap to CR4.1 = 0.0598   <-- NO CHANGE, and 0.003 mm from the pour edge
east  +0.25 (87.72):        gap to CR4.1 = 0.3098, CR4.2 = 0.3910, hole-to-hole 0.3042   <-- use this
```

The violation is driven entirely by x-overshoot (dx 0.2598) while dy (0.46255) is still inside the pad's 0.48895 y half-extent, so northward travel does nothing. Stay within +0.20…+0.30 mm east; beyond that `CR4` pad 2 becomes the limiting neighbour.

Two constraints on any alternative: this via is one of only **two** that tie the F.Cu `+3V3` island (the buck output pour) down to the In2 `+3V3` plane — point-in-polygon shows the via at (88.4, 125.33) is *outside* the island fill — so deleting it is not an option. And it is a legal same-net via-in-pad sitting inside `C12` pad 1, which is why it is where it is; the fix must keep it inside that pad. Equally acceptable: nudge `CR4` itself ~0.3 mm, since its placement is uncommitted anyway.

### 2. MAJOR — the unfused battery rail is one solder ball from the USB-C shield

`(via (at 100.71 121.579998) ... (net "+BATT"))` versus `J2` pad `S4` (GND shield tab). The B.Mask aperture edge is at y = 121.778 and the via copper edge at y = 121.779998: **solder-mask web = −0.002 mm**, i.e. the via annulus is inside the shield pad's own mask window with zero dam. Copper-to-copper is exactly 0.100002 mm, which is why no clearance error fires — only the `solder_mask_bridge` error does. Verified by re-exporting B_Mask and reading the aperture table directly; there is no 0.4 mm circular aperture anywhere, so tenting is not the escape.

`+BATT` has exactly two nodes — `J6` pin 2 and `CR3` pin 2 — so this is the raw host battery input, **upstream of everything**, up to 8.4 V from the rocket's 2S pack, unfused. `S4` is a 4.36 mm² tab with a 100 %-area paste aperture (~0.5 mm³ of paste) reflowing against a large brass shell that wicks solder outward.

In fairness to the odds: the nominal intrusion is ~2 µm × 56 µm, so this needs mask misregistration in the wrong direction *plus* solder wicking to actually bridge. It is a yield-and-safety risk, not a certainty. But the consequence is a LiPo short to chassis ground, and the fix is free.

**Fix:** move the via to **(100.71, 121.28)** — verified clean: 0.220 mm to the nearest GND via, 0.320 mm hole-to-hole (rule 0.200), 1.15 mm to `CR3` pad 1, and a 0.298 mm mask web to the S4 aperture. Costs 0.3 mm of extra F.Cu stub. **Do not** zero `J2`'s `solder_mask_margin` instead — it comes from the library footprint, five other parts share it, and it only buys a 0.10 mm dam.

### 3. MAJOR — "filled + capped vias" exists only inside KiCad; the fab is never told

`(capping yes) (filling yes)` in board setup drives KiCad's DRC and 3D view **and nothing else**. This was verified exhaustively by re-exporting the full deliverable set with kicad-cli 10.0.3 and reading it:

| Format | Via-protection info |
|---|---|
| Gerber + `.gbrjob` | none — GeneralSpecs has `Finish: "None"`, `ImpedanceControlled: true`, nothing about vias |
| Excellon | `TA.AperFunction,Plated,PTH,ViaDrill` with no IPC-4761 suffix |
| IPC-2581 | none |
| ODB++ | `.drill = VIA` only |

There is **no format-level escape route** — a written fab note is the only channel. Meanwhile there are **70 vias inside SMD lands**, and four where the drill is as wide as or wider than the land:

| Pad | Land | Via drill | Note |
|---|---|---|---|
| `U28.29` VDD_SPI | 0.220 × 0.650 | 0.300 | drill 0.080 mm wider than the land; mask is an open region, so the barrel is open |
| `U3.4` PGND | 1.000 × 0.200 | 0.300 | drill 0.100 mm wider |
| `U3.7` PSNS | 0.250 × 0.550 | 0.300 | drill 0.050 mm wider |
| `FL1.4` VSS | 0.900 × 0.400 | **0.400** | the board's only 0.4 mm drill; paste aperture is 0.350 × 0.900, so the drill is *wider than the paste* |

Plus **six vias inside `U28`'s thermal-pad paste windows**. If the boards come back merely tented, those joints drain at reflow: `FL1`'s VSS joint (the entire input power return) has no land left, `U3`'s PGND lifts on a 2.2 MHz buck, and the QFN-56 loses 6 of 9 thermal deposits — unreworkable at 0.4 mm pitch.

**Fix, both halves:**

**(a)** Add a fab-note text block on `Dwgs.User` (verified to be in the stored plot set, so it ships as `User_Drawings.gbr`; it is currently header-only at 434 B) and repeat it verbatim in the order notes:

```
VIAS: FILLED WITH NON-CONDUCTIVE EPOXY AND PLATED OVER (CAPPED), IPC-4761 TYPE VII.
  MANDATORY AT U28.29, U3.4, U3.7, FL1.4 AND THE U28 THERMAL PAD.
  ALL OTHER VIAS TENTED BOTH SIDES.
STACKUP: JLC06161H-3313 (6L, 1.6 mm, 1 oz OUTER / 0.5 oz INNER). BUILD TO THIS TEMPLATE.
CONTROLLED IMPEDANCE: 50 OHM SINGLE-ENDED ON THE ANT NET (F.Cu). In1.Cu AND In2.Cu ARE
  DELIBERATELY VOIDED UNDER THE ANT PATH BY A RULE AREA; THE RF REFERENCE IS In3.Cu.
  DO NOT FILL THAT VOID AND DO NOT SUBSTITUTE THE In2/In3 PREPREG THICKNESS.
SURFACE FINISH: ENIG.
SOLDER MASK: MIN DAM 0.08 mm AT U28 (0.4 mm PITCH); GANG THE OPENING IF NOT HOLDABLE.
```

> **The stackup was rewritten on 2026-08-02** from an idealised 0.1/0.535/0.1/0.535/0.1 mm all-Er-4.5 stack to JLC06161H-3313 (prepreg 3313 0.0994 mm Er 4.10 / core NP-155F 0.550 mm Er 4.41 / prepreg 2116 0.1164 mm Er 4.16, inner copper 0.0152 mm, mask 0.01524 mm Er 3.8, ENIG). Note JLC's API returns **two** templates with the identical name `JLC06161H-3313`; the default is the 0.1164 mm middle prepreg (compressionThickness 1.546), not the 0.1088 mm variant. The stackup change moves the RF launch by only ~3 % — the plane void, not the stackup, is what fixes it.

Confirm resin-fill-and-cap appears on the **quote** — it is a cost adder that gets silently dropped.

**(b)** Delete the `U28.29` via outright. `OUT_VDD_SPI` is now only `{U28.29, C96, C97}` and both caps are 6.4 mm away on the same layer, so the via buys nothing — see item 8.

### 4. MAJOR — surface finish is `None`

`(copper_finish "None")` propagates to the `.gbrjob` as `Finish: "None"`. On its own this is a NIT (most fabs default to HASL and will ask), but on a board with a 0.4 mm-pitch QFN-56, an edge-launch SMA and a USB-C connector, HASL's coplanarity is wrong. **Specify ENIG** — folded into the fab note above.

---

## RF — worth fixing now because it cannot be fixed later

### 5. MAJOR — the SMA launch is a large capacitive discontinuity

`J8` pad 1 is **3.5 × 1.5 mm on F.Cu with solid In1 GND 0.1 mm beneath it**. Verified by rasterising the In1 `filled_polygon` over the pad at 0.05 mm: **2100/2100 samples inside copper — 100 % solid**. In2 and In3 are solid there too, and there are **zero** keepout/rule areas anywhere in the PCB.

A 1.5 mm strip 0.1 mm over a plane is a ~10 Ω line — about **2.1 pF of shunt sitting directly at the antenna port**. Cascading 3.5 mm of 10.2 Ω into the 1.81 mm of 47.6 Ω GCPW back to the module gives, at 915 MHz:

| | Return loss | VSWR | Mismatch loss |
|---|---|---|---|
| As drawn | 9.6 dB | 1.99 | 0.50 dB |

Paid on **both** TX and RX, that is ~1 dB of link budget — roughly 10 % of range — thrown away at the connector. `Net-(U14-ANT)` has exactly two nodes (`J8.1`, `U14.6`): there is no pi-network, no matching part, nothing that could compensate it.

**Fix:** add a copper rule area on **In1.Cu *and* In2.Cu**, `copperpour not_allowed`, covering **the whole RF path** — the `J8` signal pad, the ANT trace, *and* `U14`'s ANT pad — approximately **x 91.50–93.28, y 103.66–110.95**. That makes In3 (0.789 mm down) the RF reference for the entire launch.

**In2 must be included and this is the part that is easy to get wrong:** In2 is the **`+3V3` plane**, not ground. Relieving only In1 would leave the launch referenced to a power plane and inject 915 MHz return current straight onto `+3V3` at the antenna port. Voiding In2 there costs nothing — that region carries no `+3V3` load and is already perforated by the GND stitching antipads.

**Void the whole path, not just the pad** (corrected 2026-08-02 after an initial recommendation to void under the pad only):

| | J8 pad | ANT trace | U14 ANT pad | Cascade RL |
|---|---|---|---|---|
| No void | 10.6 Ω | 49.4 Ω | **19.2 Ω** | 10.3 dB |
| Void under the pad only | 37.1 Ω | 49.4 Ω | **19.2 Ω** | 23.5 dB |
| **Void over the whole path** | 37.1 Ω | 76.8 Ω | **49.6 Ω** | **39.1 dB** |

`U14`'s own ANT pad is 0.75 mm wide and has exactly the same wide-pad-over-a-close-plane problem as `J8` — 19.2 Ω. A pad-only void leaves it in place, which is why it stops at 23.5 dB. Voiding the whole path also raises the 0.2 mm trace to 76.8 Ω, but that section is only **2.4° long** at 915 MHz, so it costs nothing measurable. Flat across 902–928 MHz (39.2 → 38.9 dB).

> Two modelling caveats on the numbers above. They are zero-thickness conformal-mapping CBCPW; with 35 µm copper in a 127.5 µm gap (t/S = 0.27) the zero-thickness assumption is strained, and a finite-difference field solve puts the whole-path case nearer **28.5 dB** rather than 39.1 dB. The ranking and the decision are unaffected. Second, the analytic model overestimates the 0.2 mm trace by ~11 % (49.4 Ω zero-thickness vs 43.8 Ω finite-thickness, In1-referenced). Any future launch work on this board needs a finite-thickness model.

*(The related finding that the F.Cu ground pour runs under the SMA centre pin is real and folded into the same fix. The claim that the pour comes within 0.13 mm of the ANT trace was geometrically refuted.)*

---

## Major — system and assembly

### 6. `radio_board` firmware host-UART TX/RX are still swapped — **no board change**
Traced end to end at pad level on all three boards. Both hosts drive **cable pin 4** and listen on **cable pin 3**; the straight-through cable is forced by construction (GND on pin 1 and power on pin 2 line up 1:1 at both ends, so a reversed cable is impossible). The modem must therefore be **TX = GPIO6, RX = GPIO5**; it is currently the reverse, so the modem drives the same conductor the host drives (contention limited to ~3.1 mA by `R77` — nothing is destroyed, but the level is indeterminate) while listening on a conductor nothing drives. **Zero bytes cross the link**, and this is the first thing bring-up will hit.

The fix already exists **unmerged** on `claude/lora-daughterboard-review-421478` (commit `a6ad202c`) setting `HOST_UART_TX = 6` / `HOST_UART_RX = 5`. Verify with `git merge-base --is-ancestor a6ad202c main` — it currently returns *no*. Do not touch `board_v8.h` or `board_v3.h`; both hosts are correct.

### 7. The rocket computer's low-side ground switch — **fix on the rocket computer, not here**
`Q10` (PMPB14XN, N-channel, source on GND) switches the **daughterboard's ground return**, while `+BATT` stays live on `J5.2`. With `Q10` open, `CR3` still conducts and `VSYS` charges, but the board's ground has no return — the only path is out through `U28`'s GPIO5/GPIO6, each through 1 kΩ, into the host's ESP32-S3 ESD clamps and onto its always-on `+3V3`. That path passes a few mA against the buck's 150–200 mA demand, so the TPS62913 hiccups against its 2.85–3.04 V UVLO indefinitely: the daughterboard motorboats instead of switching off.

**The LoRa daughterboard needs no change — fab it as drawn.** Preferred fix on the rocket computer: hard-ground `J5.1` and move the switch high-side on `J5.2` using a 20–30 V P-FET (DMP2035U-7 / SI2323 class) with `Q10` re-purposed as the level shifter. Note a TPS22918 is **not** usable there — 5.5 V max against an 8.4 V pack.

This is the same family as the base station's UART back-feed (see `../base-station/power-switch-review-2026-08-02.md`): on both hosts, cutting the daughterboard's power does not actually de-power it.

### 8. VDD_SPI decoupling — still 6.1 mm away, still no 1 µF
`OUT_VDD_SPI = {C96, C97, U28.29}` with `C96`/`C97` at 6.07 / 6.33 mm from pin 29. Prior finding 4's flash re-feed half is done (`U22.8` now on `+3V3`), the decoupling half is not. Lower severity than the finder claimed — the 7/31 bench run had this working on real silicon — but it is cheap to fix while the via at that pad is being deleted anyway (item 3b).

### 9. Assembly documentation is not ready for a contract assembler
- **Zero reference designators on either silkscreen** — all 76 refdes are hidden. Unchanged from prior finding 11.
- **`U14` (the module) has no pin-1 marker, no fab body outline**, and its silk outline sits **0.007–0.020 mm from all 20 of its own pads** — silk printing onto pads.
- **Three of the six fiducials are unusable**: `FID4` sits under the E220 module body, `FID1` under `J2`. All six are still 0.5 mm.
- **`J8` overhangs the board edge by 8.25 mm**, both sides are populated, and no reflow order or panelization is specified. There is nowhere on the outline for breakaway tabs, so panelization has to be agreed with the fab explicitly.

---

## Prior review — status of all 19 findings

**Closed and verified (11):**

| # | Item | Evidence |
|---|---|---|
| 1 | CHIP_PU RC capacitor | **fixed in both schematic and PCB** — the "PCB pending" caveat is now resolved |
| 2 | `FB1` Würth 782853200 | closed; MPN now in the file |
| 3 | BOM MPN/Mfr | closed; every purchased part carries MPN + Mfr |
| 3a | COUT 2×47 µF 6.3 V → 3×22 µF 16 V | closed; `C14` placed and routed |
| 6 | `Y6` load caps 18 pF → 12 pF | closed |
| 8 | 0.1 µF at VDD3P3_RTC | closed |
| 12 | VSYS TVS | closed; `SMF10A` (`CR5`) added, correct polarity and rating |
| 17 | vias in sub-0.25 mm lands | **declared only** — see fab-gate item 3 |
| 18 | stackup / netclass | closed; impedance declared, 0.09 mm class gone |
| 19a | courtyard DRC | closed — and it immediately found four real overlaps |
| 19c | mounting holes to GND | closed |

**Still open (14):** 4 (VDD_SPI decoupling), 5 (40 MHz crystal 8.2 mm away, 11.6 mm legs, routed under the buck inductor), 7 (UART polarity — item 6 above), 9 (LED resistors), 10 (fiducials), 11 (refdes), 13 (`CR4` sits on the chip side of `R77`/`R78`, so the 0402s absorb the strike), 14 (power-good goes nowhere), 15 (input caps 2.76 / 4.77 mm from VIN — the PSNS via half *is* fixed), 16 (SMA edge 0.378 mm short), 19b (silkscreen, slightly worse), 19d (no test points; U0TXD/U0RXD unrouted), 19e (firmware still declares 8 MB flash against a 16 MB `W25Q128`), 19f, 19g (ERC noise 294 → 316).

**One regression:** DRC baseline moved from "23 violations, 1 error" to "22 violations, **5 clearance errors + 1 mask bridge**" — caused by the in-flight `CR4`/`CR5` placements.

---

## Worth fixing, not gating

- **All three LEDs run at 40–150 µA.** With the MPNs now pinned, the blue (`APHHS1005QBC/D`, V_F 3.3 V typ / 4.0 V max) and green (`XL-1005UGC`) parts have V_F at or above the 3.33 V rail through 10 kΩ — the blue may not light at all, and the board's only rail indicator will read as dead in daylight. Drop `R3`/`R59`/`R60` to ~1 kΩ.
- **`U14` NRST has no pull-up/RC** — EBYTE's reference specifies 100 kΩ to VCC plus 0.1 µF.
- **No ESD/static-bleed path on the antenna port** — EBYTE's reference shunt inductor is absent. Relevant for a rocket-mounted whip.
- **`U14` GND pad 7** — the RF ground return beside the ANT pin — has no via in pad, unlike pads 2 and 16.
- **`U28` thermal pad has 8 ground vias against Espressif's minimum of 9.**
- **`VSYS` and `VBUS` carry no bulk capacitance at all**, and there is no inrush limiting: hot-plug to a live 2S pack drives ~8–10 A through `CR3`.
- **`+3V3` reaches three S3 supply pins through 0.127 mm traces**, half Espressif's recommendation.
- **`GPIO0` has no external pull-up** and runs 24.9 mm to the boot button.
- **198 of 199 vias have a 0.05 mm annular ring** (0.3 mm drill in a 0.4 mm pad) — an advanced-capability spec at most fabs; worth widening the pads to 0.45–0.5 mm.
- **`C96`'s BOM row claims 16 V but the MPN is a 6.3 V part** — the same value-vs-MPN drift class that was just found on the base station.
- **`U14`'s 3D model points at a different EBYTE module** (`E07-900MM10S`), so no 3D fit/keepout check has ever been meaningful.
- **`min_connection = 0.0`** disables the one DRC check that would catch sliver zone connections.

## Architecture

- **No reset conductor on the host link.** The vehicle's only radio has no recovery path: `J6` is 4 pins, all spoken for, so nothing reaches `U14` NRST or `U28` CHIP_PU. This is issue #412's root cause, and it is the one change that would need a connector swap — if that is ever on the table, it belongs in this revision, not the next.
- **No test points anywhere**, and `U0TXD`/`U0RXD` are unrouted, so USB-C is the sole console and sole programming path on a board that lives inside a rocket.
- **Zero rail observability** — PG is unrouted and no rail reaches an ADC, so a host cannot tell whether the modem is powered, only whether it is talking.
- **No hardware revision strap** on a board whose entire purpose is being a swappable module.
- **The LLCC68 firmware residue** (refuted as a board issue, real as firmware): `radio_board`'s `BOOT_SF = 10 / BOOT_BW_KHZ = 125` is not a legal LLCC68 pair. The *fleet* operating point is SF8/BW250 on both hosts, which is legal — so this only affects the modem's pre-`SET_CONFIG` listening state. Fix `BOOT_SF → 8`, `BOOT_BW_KHZ → 250`, and retire the SF10/BW125 struct defaults in `TR_LoRa_Comms.h`. **Do not** substitute an E22-900MM22S: it needs a DIO3-fed TCXO and would come up dead against the current `begin()` call.

## Verified correct — recorded so it is not re-reviewed

The RF trace geometry itself (≈47.6 Ω GCPW, In1 unbroken beneath the corridor, stitching vias at ~0.75 mm pitch), the TPS62913 circuit against TI's front-page application, the `E220` pin-for-pin wiring including the DIO2→TXEN split, the diode-OR topology and `CR3`'s reverse-polarity protection, USB-C CC/Rd and D+/D− series resistors, all four strapping pins, the `CHIP_PU` RC network (now correct in both schematic and PCB), `SMF10A` polarity and rating, mounting holes on GND, the `L10` 24 nH XTAL_P inductor, `S-CONF` = 7.5 kΩ, and the firmware pin map on all eight LoRa signals plus both LEDs.

## What this review did not cover

In3/In4 were never visually inspected (only sampled programmatically). No thermal simulation. No S-parameter simulation — the RF numbers above are closed-form transmission-line cascades, not EM solves. The 106 minor/nit findings were **not** put through adversarial verification, so treat them as finder-level confidence; the 22 blocker/major items were.
