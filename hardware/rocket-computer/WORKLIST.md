# Rocket computer — remaining worklist

**As of 2026-08-09, verified against the live files**, not against the review text. The 2026-08-05 review produced 170 raw findings across 13 domains; this is the deduplicated remainder after everything closed during that session. Each item was re-checked against the current schematic, netlist or board — values quoted here are what the files say now.

Board state (2026-08-09, closing record): owner-run DRC shows **three courtyard overlaps as the only errors, all reviewed and accepted** (the C15/J1 class — deliberate tight placements), **0 unconnected**, and the known-benign U9 pad-numbering as the sole parity item. The structural warnings from earlier snapshots (`hole_to_hole` on +3V3, `holes_co_located` on `Net-(U21-OV1)`) were retired by M-18; remaining warnings are silkscreen-cosmetic. Board revision **V9** (title block; the silk `${REVISION}` resolves from it). Every decision D-1–D-4 is decided, H-1–H-10 and M-1–M-25 are closed or consciously waived, firmware requirements live in issues #718/#721/#725, and the bench list below is the living remainder. One open owner choice: the USB VBUS In2 width (see the promoted Low).

**Closed during the session, not repeated below:** PVIN pad 12; the RF matching network (waived — Molex nominal L-network per base-station A1); RF feed width 0.30 → 0.18 mm (≈48.7 Ω); stackup → `JLC06161H-3313` with 0.5 oz inner declared; `copper_finish` None → ENIG; C12 retention method and `+` polarity silk; fab notes authored and mirrored to `Dwgs.User`; the In2 VBATT corridor (median 3.40 → 4.65 mm, ~10 → ~6 mΩ); the In2 corridor perforation; both dangling stubs; the U19 solder-mask bridge.

---

## Decisions — these gate other work

**D-1. Servo current budget — ✅ DECIDED 2026-08-09: 3 A aggregate design point; paths deliberately left open above it; flight IMON data gates any future increase.** The written bound is **3 A**, and the limiting element that sets it is the In2 VBATT stretch at y 118.5–122.5 (~3.0 A at 45 °C rise). Everything downstream now clears it with margin, measured this session: U28's output path 3.51 A combined, the Q11 `VBAT_Terminal` island 3.9 A, J3's two power contacts ~4 A, the GND slot-neck return 7.9 A across both planes. **Deliberately not tightened:** eFuse ILIM stays at 11.6 A and R88 puts the '811's start-up limit at 6.59 A — larger servos that momentarily exceed 3 A are carried rather than tripped, accepting that the In2 stretch runs warm during such excursions. The instrument for revisiting is now on the board: **SERVO_IMON → S3 GPIO9 (ADC1)** logs actual servo current in flight; if the data shows sustained draw above 3 A, the first hardware move is widening the In2 stretch, not changing any limit. Bench/firmware item added below. This closes the last open half of July's B-3; the return-FET half of the old text is obsolete (Q8 no longer exists — high-side conversion, D-4).

**D-2. P4 silicon revision — ✅ DECIDED 2026-08-08: this build is v1.3.** The board stays as drawn — `R74`, `R75`, `R76` and `C93` remain DNP, `FB_DCDC` runs from the P4's own pin 78 to U20.1 rather than through a divider, and `VDD_HP_1` (U17.54) stays isolated. Recorded as **`FABRICATION-NOTES.md` B8**, because no design file can express it: `ESP32-P4NRW32` is the order code for both v1.3 and v3.x, so the revision lives in the date code and package marking and a distributor ships whichever reel is current.

**The DNP set is the marker for a future revision.** R74/R75/R76/C93 being unpopulated *is* the signal that this is a v1.3 build — populate those four and the board becomes the v3.x configuration. That makes the DNP set the natural checkpoint whenever the silicon is revisited: if a future build buys v3.x, those four parts come back and B8 gets rewritten. Anyone auditing this board should read an unexplained DNP cluster on a core power rail as "this board assumes a specific silicon revision", not as leftover depopulation.

Consequence for ordering: the BOM's three DNP line items (C93, R74/R75, R76) are correct as-is and must stay unpopulated.

**D-3. Brownout behaviour — ✅ DECIDED 2026-08-08: transient sags are filtered out; anything that still trips the eFuse causes a clean reboot, not a ride-through.** H-1's C94 deglitch (τ ≈ 174 ms) owns the transient class; H-8's arithmetic shows ride-through of a real outage (~92 ms retry delay) would need ~8,800 µF and defends against nothing worth defending. Deployment energy survives on V_CAP; deployment authority is down ~1–2 s while the chain reboots, and minimising that window is a firmware requirement, not a hardware one. See H-8 for the full numbers.

**D-4. Peripheral switching topology — ✅ DECIDED: reverted to high-side.** Closed by the same work that closed H-3. All four low-side N-FETs (Q1 GNSS, Q7 camera, Q8 servo/EXP, Q10 LoRa) and their gate networks are removed from the schematic and the board, replaced by integrated high-side load switches: **U26** (camera) and **U28** (servo/EXP) on TPS22811LRPWR, **U27** (GNSS) and **U29** (LoRa) on TPS22810DRVR. Control polarity is unchanged — GPIO high still means branch on — so no firmware change was needed.

The point of the change was to make every connector's ground pin a hard ground, which it now is: J1.4, J3.15/16, J4.2 and J5.1 are all GND, netlist- and board-verified. That removes the floating-branch backfeed path at its source rather than bounding its symptoms, which is why the twelve EXP lines stay free of series impedance and the header remains usable for I2C, SPI or anything else the application wants. See `high-side-switch-design.md` for the design and `FABRICATION-NOTES.md` B6/B7 for the connector pinout changes this forced on J3 and J4.

---

## High — fix before fab

**H-1. eFuse UVLO has no deglitch — ✅ DONE, schematic and board.** **C94** (1 µF 0402) added on `Net-(U19-EN/UVLO)` = {C94.1, R44.1, R45.2, U19.6}, netlist-verified and placed. With R44 (1 M) ∥ R45 (210 k) the time constant is **τ ≈ 174 ms**, so millisecond-class pack sags — servo stall transients, vibration micro-dropouts — never reach the UVLO comparator at all. Thresholds unchanged: 6.91 V rising / 6.34 V falling. Costs ~0.4 s of added turn-on delay at power-up (the cap must charge through the 1 M), which is irrelevant on the pad. Two consequences worth knowing: recovery after a genuine deep sag is also slowed by the same RC, and the deglitch is now the *first line* of the brownout story — see H-8 and D-3, which this fix reframed.

**H-1 (original).** Verified: `Net-(U19-EN/UVLO)` = {R44.1, R45.2, U19.6}, no capacitor. Thresholds 6.91 V on / 6.34 V off. A millisecond pack sag hard-cuts VBATT to everything mid-flight. `power-eco.md` line 128 claims "deglitched, latched" — the doc asserts a circuit that does not exist. Add ~1 µF on EN/UVLO (τ ≈ 174 ms, +0.4 s turn-on) or lower UVLO to ~5.8–6.0 V and let INA230 telemetry own the warning.

**H-2. Pyro gate-drive margin — ✅ DONE 2026-08-06, schematic and board.** R22 changed 100 k → **5.11 k** (fault-case arm Vgs 2.27 V → **0.34 V**, against a 1.0 V minimum threshold), and **R78–R81 (5.11 k)** added from `PYRO1..4_FIRE` to GND (DTC123J base node under a worst-case pull-up: 0.7 V clamped/conducting → **0.29 V**). All five are the stock 5.11 k part. Netlist-verified: each new resistor lands on exactly two nets, ERC unchanged. Placed and routed: R78 (75.40,143.05) B.Cu, R79 (84.44,144.00) F.Cu, R80 (85.48,144.80) F.Cu, R81 (85.31,143.26) B.Cu — each pad 1 on GND, pad 2 on its PYROn_FIRE net, all within ~5 mm of their driver. Board verified: 0 unconnected, parity clean, no stray Net-(Qn-B) nets. Original text follows.

**H-2 (original). Pyro gate-drive margin.** Verified: R22 = 100 k, R2 = 100 k, DTC123J internal 2.2 k/47 k unchanged. A single enabled P4 internal pull-up (~45 k) can bias a fire-FET base, and the arm gate can reach ~2.3 V against the CSD16323Q3 threshold. ROM boot states are clean — the exposure is firmware-crash and glitch time. Stiffen the base pulldowns / resize the dividers so no worst-case weak pull-up can bias anything.

**H-3. LoRa and all 12 EXP lines have no series impedance — ✅ CLOSED, by removing the cause rather than adding resistors.** See D-4. The failure mode required ground-side switching: with the return open, an off branch floated toward VBATT and injected into every unprotected signal pin. Converting all four branches to high-side switching makes each connector's ground pin a hard ground, so nothing floats and there is nothing to inject. `LoRa_RX`/`LoRa_TX` and the twelve `EXP_*` nets are deliberately left resistor-free, which keeps J3 usable for I2C, SPI or any other application the header is put to. The camera UART keeps R30/R32 (1 k) and the sam10m8 GNSS board keeps its own R7–R9; those were already bounded and were not touched.

**H-3 (original).** Verified: `LoRa_RX`/`LoRa_TX` go straight to U15.15/16; all 12 `EXP_*` nets go straight to U17 with no resistor. The camera UART got R30/R32 (1 k) and the sam10m8 GNSS board carries its own R7–R9, so those two paths are bounded. These are not. With ground-side switching and live supply pins, an off branch floats to VBATT and injects through every unprotected signal.

**H-4. P4 power is gated by live S3 firmware — ✅ DONE, schematic and board.** Closed by giving the P4 a self-latch, so the rail survives an S3 failure in flight. **D9** (BAV170M, dual common-cathode low-leakage diode, DFN1006-3 / SOT-883) OR-gates two sources onto U30's EN: `P4_EN_S3` from U15.12 (S3 GPIO7) and `P4_EN_HOLD` from U17.5 (P4 GPIO5 — the only true spare, as everything else free on the P4 is MIPI DSI/CSI, USB DM/DP or analog REXT). R84's 100 k pulldown stays, so cold power-up is still default-off and the ECO's interlock survives. **C105** (10 µF 0402) on the same node rides the P4 through its own reset; without it a P4 watchdog reset while the S3 was down would cut its own power permanently.

Netlist-verified: `P4_EN_S3` = {D9.2, U15.12}, `P4_EN_HOLD` = {D9.1, U17.5}, `POWER_SWITCH` = {D9.3, C105.1, R84.1, U30.5}. Placed and routed on B.Cu — D9 at (80.49, 118.48), C105 at (78.88, 118.59) — 0 unconnected. The anodes sit swapped relative to the first draft, which is harmless: the OR is symmetric.

Numbers: EN sits at ~2.7 V when driven, against a 1.30 V worst-case rising threshold. BAV170M leakage is 80 nA max at 75 V / 150 °C, so both diodes together lift EN by only ~16 mV against a 1.08 V falling threshold — and that is a guaranteed datasheet figure rather than an extrapolation, which is why BAV170M was chosen over BAV70 (30 µA at 25 V / 150 °C). Hold-up is `t = 1.03·R·C` = 1.03 s nominal, but the 6.3 V 0402 derates under 3.3 V bias to roughly 3.5–5 µF, so plan on **360–515 ms**.

**Two deliberate consequences.** Once the P4 is up, the S3 can no longer shed the domain — only the P4 releasing GPIO5 does that. And the firmware requirements below became load-bearing rather than advisory. Pairs with D-3, which is the same question asked from the supply side.

**H-4a. Firmware to match the latch — NOT YET IMPLEMENTED, tracked as a GitHub issue.** Two hard requirements: (1) the P4 must drive **GPIO5 push-pull high within ~360 ms of reset**, as early in boot as it can be reached — miss that window and the P4 cuts its own power, recoverable only by the S3; (2) releasing GPIO5 is now the only way to power the P4 domain down, so a deliberate shutdown path has to exist. GPIO5 is not a P4 strapping pin (those are GPIO34–38), so boot will not contend for it.

**H-4 (original).** TPS22918 ON ← R71 ← `POWER_SWITCH` = S3 GPIO7, R68 100 k pulldown, default off. Implemented exactly as the ECO specified, but any S3 crash, reset or brownout recovery power-cycles the flight computer and its sensors mid-flight. Latch the rail, move deploy authority to the S3, or accept and document alongside the S3 watchdog strategy. (The part references here are stale: the switch conversion replaced the TPS22918 with U30, a TPS22810DRVR, and R71/R68 with R84.)

**H-5. Fiducials — ✅ DONE, two per side, which is this fab's requirement.** Four `Fiducial_0.5mm_Mask1mm` on the board: FID1 (78.87, 156.00) and FID2 (74.41, 127.53) on F.Cu, FID3 (74.41, 127.53) and FID4 (92.96, 134.25) on B.Cu. FID2/FID3 are deliberately co-located on opposite sides. DRC-verified clear — no fiducial appears anywhere in the report, so the FID1/J6 courtyard overlap that was the board's only error is resolved. The original finding asked for three per side on general principle; two per side is what the fab specifies, and that governs.

**H-6. Checked-in BOM is stale — ✅ REGENERATED 2026-08-08 from the live schematic.** `bom.csv` now holds **80 line items / 241 placements**, exported from the current netlist rather than hand-maintained. Columns changed: the always-empty `Supplier and ref` is replaced by **`MPN`, `Mfr` and `Fit`**, so the file is orderable as-is and DNP parts are visible rather than silently counted. MPN and Mfr are resolved across the inconsistent field names the SnapEDA imports left behind (`MPN`/`MP`/`SNAPEDA_PN`/`DigiKey_Part_Number`, and `Mfr`/`MF`/`MANUFACTURER`/`Manufacturer`). The 12 non-purchased items — four fiducials, four mounting holes and four others carrying `exclude_from_bom` — are excluded.

**Three DNP line items** are marked `DNP` in the Fit column: C93 (22 pF), R74/R75 (499 k) and R76 (0 Ω) — the P4 v1.3 configuration, which is exactly what D-2 is about. Do not populate them without settling D-2 first.

**Two entries still need an MPN before ordering:** R74/R75/R76 are DNP so it does not block, but **SW2** (`SW_Push` on `R-667995_MIT`) is fitted and has no part number assigned. Regenerate again after any schematic change — the file is now a derived artifact, not a source of truth.

**H-7. Stray layout files — ✅ DONE 2026-08-08.** The survey found more than the original finding did: three Finder-duplicate files across two projects, not one. All were untracked, all older than and different from the real file, and nothing in any `.kicad_pro`/`.kicad_prl` referenced them:

| removed | date | vs real file |
|---|---|---|
| `rocket-computer/rocket-computer 2.kicad_pcb` | 2026-08-02 | 6 days stale, 4.66 MB vs 5.07 MB |
| `base-station/base-station 2.kicad_pcb` | 2026-08-02 | 1.76 MB vs 1.78 MB |
| `base-station/esp32_p3 2.kicad_sch` | 2026-08-08 | stale duplicate sheet |
| `symbols/Custom.bak` | 2026-08-06 | stale library backup |
| `base-station/power.kicad_sch.bak` | 2026-07-25 | stale sheet backup |

Moved rather than hard-deleted — they were untracked, so git could not have recovered them. They are parked in the session scratchpad under `h7_removed/` if anything turns out to be wanted.

**The backup zips were left in place, deliberately.** There are 246 of them across five projects totalling ~229 MB (rocket-computer 97 MB, base-station 69 MB, lora 34 MB, gnss-sam10m8 24 MB, gnss-px1105r 5.6 MB). They are KiCad's own auto-backups and the only undo path for work that is not yet committed, so deleting them buys disk space at the cost of the safety net. Instead `*-backups/` was added to `.gitignore`, which was the actual risk — they were untracked but unignored, so they cluttered `git status` and could have been committed by a careless `git add -A`. Prune them by hand if disk space matters; KiCad regenerates them on every save.

`.gitignore` already carried `* 2.*` and `* 3.*`, so the Finder duplicates were never at risk of being committed — the hazard was purely that someone could open one in KiCad and generate fab outputs from a stale board.

**H-8. Hold-up arithmetic does not close — ✅ CLOSED via D-3: ride-through abandoned as a goal, brownout is a clean reboot.** Re-measured 2026-08-08 against the live files, and the finding *understated* the gap in both directions:

- **The outage is ~92 ms, not ≥20 ms.** The eFuse's fault response is pin-configured: C46 = 2.2 nF on RETRY_DLY → **91.7 ms** retry delay (TPS25982 Table 7-5, this exact value; Equation 14 with typicals gives ~103 ms), C45 = 1 µF on NRETRY → ~1024 retries before latch-off.
- **The hold-up is ~3 ms and halved since July.** C15's 330 µF moved to the servo branch output (`Net-(U28-OUT)`) in the high-side conversion, so **C56 alone** holds the logic chain (`V_MCU_2S → L5 → U18 buck → +3V3 → everything, incl. the P4 domain via U30`). At flight load (~0.5–0.8 A at 3.3 V ≈ 2–2.6 W) 330 µF from 7.4 V to buck dropout is ~3 ms. The ECO's 52 ms assumed a 50 mA load — off by 10× for flight.
- **Bridging is not buildable:** riding 92 ms at 2 W needs ~8,800 µF at the buck input. A second C12-class can.

Why acceptance is correct rather than resigned — each outage class, post-H-1:
| event | outcome |
|---|---|
| transient sag (servo stall, vibration) | absorbed by C94's τ ≈ 174 ms deglitch; eFuse never trips, no outage |
| sustained sag > ~100 ms below 6.34 V | pack is collapsing; no plausible capacitance rides it out |
| overcurrent > 11.6 A | harness short; the eFuse *should* open — ride-through is anti-protection; 92 ms × 1024 auto-retries means transient faults self-recover and persistent ones cannot brick the board |

**What survives:** V_CAP's 10,000 µF sits behind R20 with only 10 k bleeds — deployment *energy* rides through any realistic outage. **What is lost:** deployment *authority* — outage + S3 boot + S3 re-asserting `P4_EN_S3` + P4 boot ≈ **1–2 s deployment blackout** after any brownout. That is the irreducible exposure and it belongs to firmware: fast boot, checkpointed flight state, immediate re-arm (adjacent to issue #718). Bench item added below to measure the real blackout. Considered and rejected: shrinking C46 to 220 pF (9.3 ms retry) — the system reboots either way, so it trims ~80 ms from a 1–2 s blackout while hammering persistent faults 10× faster.

**H-8 (original).** C15/C56 are both 330 µF polymer tantalum. Delivered hold-up is ~3–6 ms at realistic flight load, not the ECO's 52 ms figure, against an eFuse outage of ≥20 ms. Pairs with H-1.

**H-9. Pyro fire-return path — ✅ CLOSED, adequate as built.** The original finding no longer describes the board. It is not two vias and a 1.0 mm In2 trace; the return now measures **1.5 mm B.Cu with four 0.3 mm vias**, and all four sit *inside* U9's 2.5 × 2.5 mm drain pad as via-in-pad. Full path: J2.5 (two connector contacts) → 1.5 mm B.Cu → 4 vias → U9 drain pad → CSD16323Q3 → GND.

One thing the earlier read got wrong is worth recording so nobody re-opens this: there is a 0.4 mm F.Cu segment on `PYRO_GND` that looks like a choke point at 1.67 A. **It is not in the fire path.** It runs from U9 pad 7 to R73 (1 k) and D7 — the sense/bleed tap — and carries microamps.

Ratings and pulse behaviour, with the pulse treated as adiabatic (correct at these durations):

| case | peak I | 1.5 mm B.Cu ΔT | via ΔT |
|---|---|---|---|
| normal fire, 0.8 Ω e-match | 10.4 A | 4 °C | 3 °C |
| normal fire, 1.0 Ω e-match | 8.3 A | 3 °C | 2 °C |
| fault — shorted e-match | 370 A | 169 °C | 114 °C |

Steady-state the path is good for 4.35 A (B.Cu) and 3.67 A (four vias in parallel at JLC's 18 µm plating), but steady state is not the operative case — an e-match fires in ~10–20 ms.

**The fault case is bounded and one-shot.** A shorted e-match or harness lets C12 dump ~0.35 J through roughly 23 mΩ, peaking near 370 A with a 227 µs time constant. The copper survives that comfortably. Crucially, once C12 empties, **R20's 150 Ω charge resistor limits sustained fault current to 56 mA**, so the event cannot repeat or persist. That resistor is doing more protective work than its position in the schematic suggests — do not "optimise" it away.

**H-10. MCU power trace widths below Espressif minimums.** `ESP_VDD_HP` at 0.127–0.2 mm and a 4-mil branch through R76, against the 20-mil guidance; S3 `VDD3P3` PA feed at 0.1 mm carrying 340 mA TX bursts. Cheap to widen.

---

## Medium

**M-1. Arm interlock is low-side and J2.5 is silk-labelled "GND" — ⚠️ REVIEWED 2026-08-08, accepted as drawn.** Owner reviewed the connector and judged the pin identification correct for how the harness is built. Recorded rather than changed.

**No label is wrong, and the original finding was sloppy in implying otherwise.** Verified: every silk label sits ~3 mm above its own pad and correctly identifies it — `1`→`PYRO1_EXT`, `2`→`PYRO2_EXT`, `3`→`PYRO3_EXT`, `4`→`PYRO4_EXT`, `GND`→`PYRO_GND`. Pad and net names agree (`PYRO_GND 5_1`/`5_2`).

The finding was only ever about the *word*, and the owner's rationale settles it: **to whoever is wiring the rocket, J2.5 is the ground path of the pyro circuit** — it is the common return for all four channels, and that is the mental model of the person holding the wire. Matching the label to how it is actually used is what prevents mistakes at the bench. Silk space at this connector is also tight, which rules out anything longer.

Recorded for completeness, not as an objection: electrically `PYRO_GND` = {J2.5_1, J2.5_2, R73.1, U9.5–9} returns **through** the arm FET rather than to chassis ground, so bonding it to airframe ground would short across U9. That is a harness-build constraint, not a labelling defect, and it belongs with the arming procedure rather than on the silkscreen.

The low-side-versus-high-side arming question was not re-opened; it stays low-side.

**M-2. 3V3 buck input capacitor is on the wrong side.** C43 is on B.Cu at (91.76, 127.38); U18 is on F.Cu at (88.78, 126.365) — opposite sides, 3.15 mm apart, reached through a single 0.3 mm via. TI wants the input loop tight against PVIN/PGND. Add a local 0402/0603 at U18 even if C43 stays.

**M-3. Crystal load caps — ✅ CLOSED, no change. 12 pF stays; resolve by first-article measurement.** The original finding re-litigated a settled decision without citing its history, and was wrong on two counts.

**It is not a free schematic change.** L3 (24 nH) on XTAL_P is Espressif's prescribed part — S3 HDG: "add a series component on the XTAL_P clock trace… an inductor of 24 nH to reduce the impact of high-frequency crystal harmonics." It forms a low-pass **with the load cap**, so the load cap sets the corner, and on these boards that corner matters:

| load cap | corner | @920 MHz | @2.44 GHz |
|---|---|---|---|
| **12 pF (fitted)** | 297 MHz | **−18.7 dB** | −36.5 dB |
| 15 pF | 265 MHz | −20.9 dB | −38.4 dB |

920 MHz is the 40 MHz crystal's 23rd harmonic, inside the **902–928 MHz LoRa band** the sibling board receives at −130 dBm. 15 pF would improve suppression by 2.2 dB — so harmonics argue *for* the change, not against.

**The counter-argument is startup margin, and it is why 12 pF was chosen.** The 18 → 12 pF change (e5542d08) was deliberate. `gm_crit ∝ (C0+CL)²`: 18 pF = 1.00×, **12 pF = 0.62×**, 15 pF = 0.80×. Going to 15 pF gives back ~29% of the required transconductance.

**The frequency claim does not survive contact with the stray-capacitance assumption.** At C_stray = 2.5 pF the error is +21.8 ppm; at 3.3 pF it is about +10 ppm, inside the ±10 ppm requirement. Prior analysis on these boards lands near +10 ppm. Nothing on paper distinguishes them — only a bench measurement does, which is why every prior close-out said measure and trim.

**Close-out:** leave C20/C22/C48/C49 at 12 pF. On first article, measure the 40 MHz centre frequency and trim if it is outside ±10 ppm; 13 pF is the documented single-step trim. Treat load cap and harmonic suppression as one decision, never the frequency alone.

**M-4. VDD_SPI decoupling against HDG.** C27 is still 10 uF on the VDD_SPI node; the guidance is 0.1 µF + 1 µF and explicitly warns against large capacitors there. The flash rail move (H7) is done; this is the leftover.

**M-5. TPS2121 ILIM at device maximum — ✅ DONE 2026-08-08: R64 18.7 k → 49.9 k.** The original finding had stale designators (R57's 1.5 k is in the OV2 divider; R59 pulls up the eFuse PG) but the right number: the actual ILIM resistor was **R64 = 18.7 k**, which is the first row of the TPS2121 datasheet table — the *maximum-current* setting, 4.6/5.2/5.8 A min/typ/max. Almost certainly copied from the table/EVM default.

What the branch actually is: U21 feeds the **entire avionics complex** — S3, P4 + its core buck, all three flash (NAND log + 2× NOR), the sensor suite, INA230, USB switch, buzzer, and the pyro continuity sense. Worst realistic draw at the mux is ~0.75 A sustained (recovery phase on USB 5 V: buzzer + BLE + log flush), ~1.0 A with a TX burst. A 5.2 A limit protected nothing, and a short on `V_MCU_2S` would have pulled 5 A through L5 (a 2 A-class inductor) until thermal shutdown.

**Fitted: 49.9 k** (`ILM = 65.2/R^0.861` → **2.26 A typ, ~1.8 A min**) — 2.6× worst sustained, 1.8× worst burst, and a stock part (same Yageo reel as R12). Netlist-verified: `Net-(U21-ILIM)` = {R64.2, U21.10}, R64.1 = GND. Rejected: 80 k (the textbook 1.5 A row, not in the BOM) and 100 k (stock, but min limit ~0.82 A sits *below* the recovery-phase draw — negative margin at the edge of the legal 18–100 k range). Behaviour on a persistent fault: the TPS2121 regulates, heats, TSDs at ~160 °C and auto-restarts — a thermal sawtooth, not a latch-off; same philosophy as the eFuse. Value-only change; syncs to the board on the next Update PCB from Schematic.

**M-6. FID1 courtyard overlap — ✅ CLOSED with H-5.** FID1 moved out from under the J6 shell; no fiducial appears in DRC. Note the board now has a different single error, a C15/J1 courtyard overlap — unrelated to this one, and tracked in the board-state line at the top.

**M-7. INA230 sense is half-Kelvin — ✅ CLOSED 2026-08-08: does not reproduce; the board is already Kelvin on both sides.** Point-tested against the filled copper, not the bounding box: IN+ (U23.13) runs as a **dedicated 0.127 mm In3 sense trace** — B.Cu stub → via → In3 → via landing at R72 pad 2's left edge — and no element of it touches the `VBAT_Terminal` load pour (stub, both vias all outside the fill; only pad 2 itself is in the pour). Load current enters pad 2 from the right; the sense enters from the far left. IN− (U23.12) lands *inside* pad 1 at (80.04, 129.45); the 0.4 mm track entering pad 1 from the right serves only CR2 (clamp, no normal current) and the BUS pin bias, so it does not contaminate the tap. The layout is out of the error budget — residual accuracy is the 2 mΩ shunt's own tolerance plus INA230 gain.

Two notes for the record: the two vias on the sense line are free — INA230 input bias is ~10 µA, so via resistance contributes nanovolts, and the In3 detour exists because U23 and R72 share B.Cu with the `VBAT_CON` pour flooding the direct path. **One pending tidy:** the pad-2 sense via sits centred on the pad's left *edge* (79.06, 131.28), so only half its annulus lands on pad copper — connected, but a weak joint. Slide it ~0.3 mm right to land fully on the pad (anywhere on the left half preserves the Kelvin). Owner is making that move by hand; not yet on disk at close-out.

**M-8. Power-good outputs dead-end — ⚠️ ACCEPTED 2026-08-08, deferred to a future revision as an enhancement.** Verified harmless as-is: both PG outputs are open-drain into pull-ups (`Net-(U18-PG)` = {R49.1, U18.4}, `PG_RAIL` = {R59.2, U19.13}) and drive nothing — no fault path, only lost observability. The MCUs do not need to see power faults on this revision. Written up as **issue #721** with the wiring and firmware details for when the time comes, including the third dead-end found while closing this: **U23's ALERT output (pin 3) is also unconnected** — the INA230 can generate programmable over/under-limit alerts on the I2C bus it already shares, so a future rev gets three observability signals for the price of two traces and zero new parts. Free S3 GPIOs exist today (GPIO13/14/17/18). Ties to D-3: after a brownout reboot, a latched PG flag is what would let firmware distinguish "eFuse dropped" from "software crash" in the flight log.

**M-9. No reverse-polarity protection — ✅ DONE 2026-08-09, schematic and board, verified.** **Q11** (CSD25404Q3, −20 V P-channel, 6.5 mΩ max at V_GS = −4.5 V) sits in series at the battery input on the compact `TRANS_CSD25404Q3` footprint (TI SLPS570 §7.2 pattern, 2.45 × 3.50 mm — built because the U9-family `TSON Advance_TOS` import is 4.2 mm tall and did not fit the corridor). Drain row in the `VBAT_J8` island (42.7 mm², wraps J8.2), source clip into the `VBAT_Terminal` island (narrowest cut 1.30 mm → 3.9 A at 20 °C, clears the ~3 A servo budget; the 11.6 A eFuse corner is ms-transient), gate pad self-connected in the GND pour. A reversed pack is blocked at the connector; previously it put −8.4 V simultaneously on the INA230 and eFuse IN pins (−0.3 V abs max) with only a 1 A clamp conducting the short.

**CR2 returned as a connector ESD clamp**, relocated upstream: K → `VBAT_J8` at J8.2, A → GND, placed at (78.80, 121.32) with both pads in their pours. Its old role (reverse-polarity) is Q11's now; its new role is clamping strikes on an unplugged pigtail so Q11's −20 V avalanche is not the first line. **The Kelvin sense survived the rework**: after one regression (tap at the island's source end, ≈ +20% read error) the trace was re-routed to enter R72 pad 2 from the south, outside the load path — verified against the filled copper. Board: **0 unconnected**.

Decoupling consolidation in the same area: **C76 (10 µF) deleted**, **C40 (100 nF) kept** at U23's pin. En route the BOM caught **C65 carrying a 390 pF MPN under a 100 nF value** (cloned from C44 during M-2) — fixed to the stock 100 nF part; assembly would have fitted 390 pF as the 3V3 buck's input bypass.

Ratings note, unchanged: V_GS is ±12 V — fine at 2S indefinitely, bars a 3S pack without a gate divider; the 30 V PowerPAK-class alternative is the swap if 3S ever happens. Adds ~7 mV at 3 A cruise; effective UVLO trip rises ~0.1 V during stall at the eFuse limit (conservative direction).

**M-10. Battery polarity silk is wrong — ✅ FIXED 2026-08-08, board-verified.** The `+` now sits at (82.34, 131.70) on B.Silkscreen, 2.4 mm from J8 pad 2 (`VBAT_Terminal`, the actual positive) and at the opposite end from the GND pin; `Batt` labels the connector at the pad-1 end. The mark is now on the pad it describes. Pairs with fab note B5 (confirm pigtail polarity against the schematic) and M-9.

**M-11. Magnetometer near the LoRa feed — ✅ CLOSED 2026-08-09: measured at ~1–3 µT, accepted with firmware compensation.** Re-measured after the owner's marginal supply move and the high-side conversion. Closest feed segments sit 1.5–1.9 mm from U3; the naive infinite-wire bound (15–19 µT per segment at a 140 mA TX burst) is cut by finite segment lengths (~0.3×) and — the big one — by return-current image cancellation: **all 10 close B.Cu feed segments verified to have solid In4 GND plane 0.1 mm beneath**, so the far field falls as a dipole (~0.12× at these distances). Cluster total ≈ **1–3 µT TX-correlated**, i.e. 2–4° worst-case heading wobble during transmit, against the original 5–15 µT estimate.

Board space rules out a major move, and none is needed: the perturbation is deterministic and synchronised to an event the S3 itself commands. Close-out path: (1) bench, first article — mag output idle vs continuous TX (E220 test carrier), measure the offset vector once; (2) firmware — blank or subtract during TX windows. If that copper is ever re-routed, keep the feed on B.Cu-over-In4 rather than In3 near U3; the image cancellation is what makes the numbers work. Related: the servo branch is the bigger di (~amps at ~20 mm, same ~2 µT class after plane cancellation) and its bursts coincide with deployment — the same mag-validity-vs-actuation-state tagging covers both, and deployment is precisely when the mag should be flagged rather than trusted.

**M-12. BMP585 under-body features — ✅ CLOSED 2026-08-09: fixed where it mattered, reclassified and waived where it did not.** The one genuine hazard is gone: the interior GND via that sat 0.97 mm from the body centre *in the open gap* — a capped-via-plus-mask bump of ~55–60 µm against the ~50 µm solder standoff — is now **inside pad 6 at (89.01, 99.92)**, part of the solder joint under the board's existing filled-and-capped via-in-pad spec (U19 precedent). The other interior via moved to 0.25 mm outside the body; the V_MCU_SWTCH via at (87.97, 101.52) sits 0.05 mm inside the body line at the rim — accepted, the underside clearance concern does not apply at the edge.

Two reclassifications from the original finding: the **body centre carries pad 9, the BMP585's own 1.07 mm mechanical land** — Bosch's pattern, not a violation — and the central 1.5 mm² has zero trace crossings. The remaining F.Cu runs (SENS_SCLK/SDI/SDO, CS, INT, two V_MCU_SWTCH stubs) are **short pad escapes at the ring**, which an LGA cannot be routed without; Bosch's "no traces" targets through-routing, and none remains.

**Consciously waived: solder-mask relief.** The footprint keeps mask over the inter-pad areas where Bosch prefers bare laminate. Drawing relief would expose the ENIG escape traces under the sensor instead — with the escapes confined to the pad ring where joint height sets the standoff, mask-as-is is the better of two imperfect options. Bench corroboration is free: the baro offset is visible in the first-article data the altimeter produces anyway.

**M-12 (original).** BMP585 has vias, traces and mask under its body, against Bosch's landing-pattern guidance, on the primary apogee sensor.

**M-13. NAND EPAD paste — ✅ DONE 2026-08-09, library and board.** The footprint (`8L_WSON_8x6x3p4x4p3_MAC`) had it both ways: pad 9 (the 3.45 × 4.34 mm EP) carried a full `F.Paste` aperture *and* six drawn segmentation windows — the stencil gets the union, so the windows were decorative and the EP printed 100%. Fixed by removing `F.Paste` from pad 9 in the library **and** on U11's board copy (verified: pad 9 now F.Cu+F.Mask only, six window graphics intact on both). The windows now govern: **11.43 mm² of 15.00 mm² = 76% coverage**, inside the 50–80% band WSON EP guidance targets. Less voiding, less float, and the flight-log flash sits flatter at reflow.

**M-14. Zero testpoints — ⚠️ ACCEPTED 2026-08-09.** Owner decision: the USB-C chain stays the only programming/console path for both chip-down MCUs. Recorded, not changed.

**M-15. P4 strap pins on the EXP header — ✅ CLOSED 2026-08-09: firmware power sequencing (option A), lines stay bare by design.** GPIO34/37/38 ride EXP_11/10/09; GPIO35/36 (the primary boot straps) never leave the board. The sharpened mechanism: EXP payloads are powered from U28's branch, so a **connected-but-unpowered payload clamps the straps low through its own ESD diodes** — corruption needs no hostile drive. The fix that preserves every post-boot use of the pins is ordering, not impedance: **`SERVO_ACT` before `POWER_SWITCH`** on every path that raises the P4, so payload clamps are released when the straps sample. Tracked as **issue #725** with tests; the interface rule (EXP_09/10/11 high-Z until the flight computer is up) is **fab note B9**.

Rejected: series resistors — beating the clamp divider needs ≥ ~23 k, which keeps servo PWM but kills I2C/SPI on those pins, the exact capability H-3/D-4 preserved. Future-rev upgrade if the interface rule proves unenforceable: a quad FET bus switch (CB3Q-class) on the three lines, opened during boot by default and closed by a spare S3 GPIO — hardware-guaranteed and it also mutes the boot chatter. **Open decision riding in #725:** the ROM boot log still bursts onto EXP_10 at each reset; silence permanently via the P4 `UART_PRINT_CONTROL` eFuse (one-way — verify against the TRM first) or accept and keep twitch-sensitive servos off EXP_10.

**M-16. Buzzer PWM alongside the pyro fire lines — ✅ CLOSED 2026-08-09: re-routed, re-measured, residual shown to integrate to nothing.** The owner's re-route halved the worst pairing and doubled its gap: `PIEZZO` (U17.18 → R26 → Q9 gate, ~72 mm total) now runs 32.4 mm within 0.5 mm of `PYRO2_FIRE` at a 0.20 mm closest gap on In3 (was ~80 mm at 0.1 mm), 22.5 mm near `PYRO3_FIRE` at 0.40 mm, and only 1.8 mm near `PYRO_ARM`.

The signal itself is not a special aggressor — 3.3 V CMOS edges at a ~2.7 kHz tone rate, slower repetition than the sensor SPI that shares the neighbourhood; the finding was always about geometry. Residual arithmetic: ~1–1.5 pF mutual capacitance into a victim node that **H-2's R79 (5.11 k) stiffened to ~4.6 kΩ** — worst case (P4 hi-Z/unpowered) each edge couples ~0.8 V for ~25 ns at the DTC input, and the fire-FET gate behind it (10 k pull-up, nF-class gate, τ ≈ 10 µs) integrates 25 ns blips at 5.4 k edges/s to millivolts. Actively-driven case: millivolt spikes into ~30 Ω. No credible path to an e-match. H-2 quietly retired most of this finding before the re-route improved the geometry.

Optional trims recorded, not required: a GND guard on In3 between `PIEZZO` and the PYRO2/3_FIRE runs if that corridor is ever reopened (3–10× further reduction), and a one-line firmware policy of no beeping while armed — which mostly falls out of the flight-state machine naturally (pad beeping is pre-arm, locate beeping is post-deployment).

**M-17. Exposed-pad via counts — ✅ CLOSED 2026-08-09.** Board-verified: **U17 (P4) EP now carries 10** in-pad GND vias in the 7.5 × 7.5 mm pad (was 6, Espressif minimum 9 — met). **U18 carries 4 in-pad** (was 0, fixed during the M-2 input-loop work). **U15 (S3) closes at 5** in-pad (was 2 after interim rework displaced two to just outside the pad edge), spacing 0.94 mm — short of Espressif's 9, **waived for a competing requirement**: the region under the S3 is shared with C9/U8 on B.Cu, VBUS on In2, PWR_SDA on In3, and critically the **V_CAP B.Cu pour, already necked to 1.40–1.75 mm under the EP** — the path that charges the pyro energy store. Further vias perforate it. Five vias more than serve the S3's ~0.3–0.5 W dissipation and RF ground return; the pyro path keeps its copper. The guideline yielded to the deployment store, knowingly.

**M-18. Same-net via spacing — ✅ CLOSED 2026-08-09, board-verified.** A full-board pairwise scan finds **no same-net via pair below JLC's 0.1995 mm hole-to-hole minimum anywhere** — both the 0.163 mm `+3V3` pair near (91.70, 126.28) and the co-located duplicate on `Net-(U21-OV1)` at (82.04, 135.95) are gone. The board-state line's two structural warnings are retired.

**M-19. USB VBUS capacitance — ✅ DONE 2026-08-09.** **C76** (1 µF 0402, the 16 V CL05A105KO5NNNC — deliberately not the 6.3 V stock 10 µF, since this node sees hot-plug spikes) added on `Net-(J6-VBUS)`. The failure it retires: a capacitor-less VBUS node rings toward ~2× VBUS at insertion, sailing through the OV1 divider's ~6.35 V trip and chattering the mux during the very switchover it is executing — a bench/log-download gremlin, never a flight issue (no USB in flight). With 1 µF the ring drops to ~1.3× at ~160 kHz, one clean OV crossing, and the TPS2121's fast-switchover gets the input reservoir its datasheet assumes. Placement is mid-net (~25 mm from IN1) and immaterial for this mode — at 160 kHz the trace is electrically short; the close-to-pin rule is for ns-edge decoupling, which this is not.

**Bookkeeping note:** the C76 designator is *recycled* — it previously named the 10 µF on +3V3 deleted during M-9's consolidation. Anything referencing "C76" before 2026-08-09 means the old part; the live BOM is authoritative.

**M-20. Camera port voltage — ✅ RESOLVED 2026-08-06.** *(Amended 2026-08-09: the supply moved to **J4 pin 1** in the high-side rework — pin 2 is now GND; see fab note B6 for the current pinout. The voltage-range analysis below is unchanged.)* J4 pin 2 carried switched pack voltage 6.4–8.4 V at the time of writing. Qualified cameras are the RunCam Split 4 (5–20 V) and the GoPro HERO10 Black Bones (2S–6S, 5–27 V); the port's guaranteed range is the narrower 5–20 V, and 2S sits inside it. A 5 V regulator is explicitly *not* wanted — the Bones is reported to stop recording on 5 V and GoPro recommends a higher-voltage supply. Recorded in `FABRICATION-NOTES.md` block B6, together with the Bones' ≤5 V shutter-wire constraint (J4.3/4 are 3.3 V GPIO through 1 k, so compliant).

**M-21. DRC severity ignores — ⚠️ ACCEPTED 2026-08-09.** Verified in the live `.kicad_pro`: `missing_courtyard`, `pth_inside_courtyard`, `npth_inside_courtyard`, `footprint_type_mismatch` all at `ignore`. Accepted for this revision: the known member of the hidden class is C12's courtyard-less footprint (deliberate, B1–B3), the board has had continuous manual review all session, and flipping the severities now would flood the report days before fab. For the next revision: set them to `warning` at project start, when the noise is actionable.

**M-22. FB-network MPNs — ✅ DONE 2026-08-09.** D-2 resolved as v1.3 (all four stay DNP), which makes the *consistent* state "all four carry MPNs so the future v3.x flip is turnkey" — C93 already does. Added, matching the board's Yageo RC0402 convention: **R74/R75 = 499 k → `RC0402FR-07499KL`**, **R76 = 0 Ω → `RC0402JR-070RL`**, Mfr Yageo. The BOM's Fit column already marks all four DNP, so nothing mis-orders meanwhile.

**M-23. GND necking at the slots — ✅ CLOSED 2026-08-09: measured adequate.** Live measurement: **7.8 mm of GND on In1 *and* 7.8 mm on In4** at y = 121.6 (the finding's "~7.5 mm total" undercounted — it is per plane). Return-current capacity across the neck ≈ **7.9 A at 20 °C** for the pair, against worst sustained returns of ~4–5 A (3 A servo bound + logic) and ms-class pyro pulses that are adiabatic. Signal returns crossing the slots ride the same bridges. 2× margin at the D-1 bound; no change.

**M-24. eFuse off-state draw — ⚠️ ACCEPTED 2026-08-09 with an ops rule.** The number stands (nothing in the session's rework touched the off-state loads: the R44/R45 divider draws ~7 µA and the TPS25982's own OFF-state I_Q plus the INA230 input-pin loads make up the rest). Consequence at ~250 µA: after a low-voltage cutoff the pack keeps draining at self-discharge-plus rates — weeks-to-months to deep-discharge, not days, but the ECO's "cutoff then negligible draw" claim is wrong and its EN-latch waiver rested on it. **Ops rule, added to the bench/ops list: physically disconnect the pack for storage; the LVC is flight protection, not storage protection.** Hardware fix (an EN latch) deferred unless storage practice makes it matter.

**M-25. Inter-MCU I2C along the antenna — ⚠️ ACCEPTED 2026-08-09, bench check added.** Verified on the live board: ~20 mm of `ESP_SCL`/`ESP_SDA` within 8 mm of the antenna (U14), closest ~2.4 mm from centre (≈0.3–0.9 mm from the body edge, matching the finding). Both directions of the coupling are bounded: I2C's edge spectrum is ~30 dB down by 2.4 GHz (desense contribution small next to the board's clocks), and BLE TX at +8 dBm couples millivolts into a bus with 5.11 k pulls and ~1 V hysteresis — the realistic failure is an occasional NAK during TX, which the inter-MCU protocol must tolerate anyway. Bench item added: exercise the inter-MCU bus under continuous BLE TX and count errors. Re-route not justified at this geometry.

---

## Low

Grouped, since these are mostly single-line cleanups.

**Silk and identity:** S1 legend reads `O`/`F` (looks like on/off, actually selects P4-vs-S3 USB target); the `GPS` label is buried under J4's body; C15 polarity silk is clipped at the board edge; text on exposed pads will be clipped; 0.5 mm axis texts and 0.12 mm strokes are below JLC legibility minimums; version identity lives only in the PCB title block, no schematic title blocks.

**Library hygiene:** U9 carries a TI part value in a Toshiba-named footprint with a mismatched Datasheet field; Y2/Y4 footprints differ from their library copies (residue of the added-then-reverted crystal inductor); C12's footprint still has no courtyard; the C12 3D model is Ø16 against the real Ø18.

**Decoupling and layout nits:** IMU and barometer lack the 100 nF caps their reference circuits specify; no local decoupling within 5 mm of the NAND VCC; several GND decoupling pads sit 3–4.6 mm from their nearest via; `SENS_SCLK` runs 0.10 mm alongside the P4 crystal `XTAL_N`; the S3 40 MHz crystal is 8.5 mm out with a 14 mm meander; the P4 crystal is 12.3 mm out with asymmetric legs; the 32 kHz load-cap return travels 3.2 mm of pour before finding a via.

**Connectors and mechanical:** all peripheral connectors are friction-retention only, no positive lock anywhere for flight vibration; J5's mounting tabs float while J1's are grounded; the M2 screw head at H3 clears the BMP585 body by ~0.5 mm.

**Pyro:** the cap still charges to ~2.6–2.8 V on USB-only power through the fire-FET body diodes; V_CAP is split across two B.Cu zones touching only along one accidental 1.85 mm shared fill edge *(2026-08-09: measured 1.40–1.75 mm under the S3 EP — fine for the R20-limited 56 mA charge and for ms fire pulses per the H-9 adiabatic math, but do not narrow it further; it capped the S3 via count in M-17)*; the U9 pin-9 parity warning is cosmetic but still fires; BOM lists C12's manufacturer as Nichicon when the series is Chemi-Con.

**Other:** pyro continuity LEDs run through 100 k (R9/R11/R13/R19) — tens of µA, likely invisible in daylight; the TPS62152 inductor is at the datasheet minimum with 1.89 A saturation against a 2.2 A current limit; ~~CR2 is a 1 A Schottky across an unfused 2S pack~~ *(resolved by M-9: Q11 blocks reverse at the connector; CR2 re-purposed as connector ESD clamp behind it)*; no series R/C provision on the P4 USB pair although the S3 leg has 22 Ω; GPIO0 has no external pull-up; USB ESD array sits on an ~8 mm stub.

**Promoted from the Lows, worth a decision (2026-08-09): USB VBUS runs 26.7 mm at 0.40 mm on 0.5 oz In2** (verified live, plus 0.2 mm stretches on F.Cu). Steady rating ~0.28 A at 20 °C against a full-system-on-USB draw of ~0.6–0.75 A — IPC-2221 arithmetic puts a sustained full-load USB soak at a very hot trace buried mid-board; IPC-2152 with the adjacent planes softens it to merely hot. Two outs: widen the In2 run to ≥0.8 mm where the corridor allows (layout change), or bound it operationally — programming and log download with the **P4 domain off** draws ~0.3 A and is fine indefinitely; avoid hours-long full-system soak tests powered from USB. Owner's call; the ops bound costs nothing and matches how USB is actually used.

---

## Bench, after build

Servo-stall sag measured at VBAT_CON; RunCam record-inrush on battery; RF return loss and efficiency with the antenna installed (L1's pads are the only trim point); crystal frequency across temperature; C12 bay fit and staking check.

**Servo current telemetry (D-1):** log `SERVO_IMON` (S3 GPIO9, ADC1) through every flight from the first article on. The 3 A design bound was set by the In2 corridor; this channel is the evidence that either confirms it or triggers the widen-In2 rework. Scale: 629 mV at the 6.59 A start-up limit, ~95 µA/A × 1 k below it.

**Inter-MCU I2C under TX (M-25):** run continuous BLE TX while hammering the S3↔P4 I2C bus; count NAKs/errors. Expect none to few; anything systematic reopens M-25.

**Pack storage (M-24):** physically disconnect the battery for storage. Post-cutoff draw is ~250 µA (divider + eFuse off-state + INA pins), which out-drains self-discharge; the LVC protects flights, not shelves.

**Magnetometer TX offset (M-11):** with the board static, log the mag with the radio idle, then with the E220 in continuous-carrier TX; the difference vector (expect low single-digit µT) is the compensation constant. Repeat once with a servo commanded hard-over to bound the actuation-state disturbance.

**Brownout blackout (D-3/H-8):** induce a UVLO trip (pull VBAT_CON below 6.34 V for >250 ms, release), and measure wall-clock from VBATT loss to the P4 re-armed with `P4_EN_HOLD` high. That number — expected 1–2 s — is the deployment blackout the airframe actually carries; log it in the flight documentation. While set up, confirm V_CAP droop across the outage is negligible and that the eFuse auto-retry recovers without USB attached.
