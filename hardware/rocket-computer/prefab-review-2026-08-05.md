# TinkerRocket rocket-computer — Pre-Manufacturing Review #2

**Date:** 2026-08-05 (run overnight 08-04 → 08-05, against the live working tree **including the uncommitted re-route saved 08-04 21:04**)
**Scope:** same as the 2026-07-30 review — every component + support parts, system architecture, PCB layout beyond DRC, "anything dumb" — **plus** a finding-by-finding closure audit of the July review and, new this time, an adversarial verification pass.
**Method:** 28 agents (13 domain finders + 1 closure auditor, then adversarial verifiers), 3.89 M tokens, 1,139 tool calls, ~70 min. Every finder parsed the live KiCad sources plus a fresh kicad-cli export set (netlist / ERC severity-all / DRC severity-all / BOM / per-layer PDFs+SVGs / renders) and pulled manufacturer datasheets. **170 raw findings** (3 blocker / 33 high / 57 medium / 42 low / 35 info, before dedup), **194 affirmative clean checks**, 74 checks blocked.

**Verification coverage — read first.** July's planned adversarial pass never ran (spend limit); this run got most of it done before hitting the session limit again:

- ✔ **Adversarially verified** (an independent skeptic re-derived every blocker/high/medium from the raw files): power-input, switched-power, **pyro**, esp32-p4, esp32-s3, sensors, and the closure audit — i.e. all component-level and safety-critical domains. Verdicts: most confirmed; 1 refuted, 2 downgraded (kept in Annex A for the record).
- ◐ **Main-loop spot-checked**: the headline claims of the unverified domains were re-derived by hand against the netlist after the run (EXP/GNSS/LoRa series-resistor state, the S3-gates-P4 power topology, PYRO_GND topology, L3's identity, crystal CL, the stale-file diff).
- ○ **Unverified** (finder evidence only): connectors, architecture, layout-power, layout-signal-rf, layout-grounds, layout-mfg, bom-parts verify passes were lost to the session limit. The finder work itself is evidence-cited, and several of their headline items are independently corroborated by ✔ domains. The lost passes can be re-run cheaply after the limit resets — all 14 finder results are cached in workflow run `wf_0c4ccb31-dd2` (resume instructions at the end).

**State of the board since July 30:** ~10 fix commits landed (eFuse ILIM 100R→127R, crystal caps 18→12 pF, CHIP_PU RCs, JST VH battery input, MPN/Mfr fill, filled+capped via declaration, P4 v3.x provisions, 1k UART resistors) plus tonight's **uncommitted** work: a full re-route, C23/C24/L2 removed, L3 added, 16 parts moved in the bottom-side power/pyro cluster. The July review's In3/In4 visual gap is now **closed** (both planes eyeballed and parsed: solid, single-polygon, byte-identical GND).

---

## Verdict: 1 blocker left (1 fixed in-session, 2 waived), then a short high list.

The July repair work is largely real: **8 of the 20 July blocker/high items are fixed and each fix survived independent adversarial attack** (P4 v3.x circuit B1/B4/H9, battery connector B2/H3/H4, S3 flash rail H7, CHIP_PU RCs H8). Of this review's four candidate blockers: **NB-3 (PVIN pad 12) was fixed and verified during this session**; NB-1 (RF matching network) and NB-2 (C12 placement) are deliberate owner decisions, recorded below as waivers with their residual actions; **NB-4 (power-path copper vs. the eFuse current limit) is the one blocker still open.**

### Blockers

**NB-1 — WAIVED (owner decision, 2026-08-05). The deleted S3 matching pi is the deliberate Molex-nominal network, adopted from the base-station review.** The review initially flagged the removal of C23/C24/L2 in tonight's uncommitted edits as an accidental casualty of the re-route (3 finders converged; the verifier reproduced the file evidence: `Net-(U14-Feed)` = {U15.1 LNA_IN, U14.4 feed, L1.2}, refs absent from schematic/PCB/BOM, parity 0). The owner confirms it is intentional: it implements Molex AS-479480001 Rev G §6.0's specified L-network — 4.3 nH shunt at the antenna, 0 Ω series realised as a direct trace, nothing else fitted — exactly as base-station review finding A1 resolved on 2026-08-04 (`hardware/base-station/prefab-review-2026-08-02.md`). That review also showed the old pi was not merely redundant: fitting a 1.5 pF shunt alongside the 4.3 nH **inverts the sign of the shunt element** (+j65.9 Ω → −j127.8 Ω at 2.44 GHz), so the July review's "matching topology is correct" assessment is superseded, not contradicted. The live board matches the endorsed form precisely: L1 = 4.3 nH LQW15AN4N3C00D (the same higher-Q wire-wound part A1 endorsed), and the U14 fixing pads 1–3 are left unconnected per A2. **Accepted trade, recorded:** this build has no tuning provision and no VNA break point — if the measured match is poor, L1's own pads are the single degree of freedom short of a respin. **Still actionable under the waiver:** (a) the feed trace is 0.30 mm ≈ 37–40 Ω on the declared stackup, not 50 Ω (~0.17–0.18 mm) — the same issue class the base-station closed as S7 by pinning the JLC stackup and resizing the feed; fix the width and order the stackup explicitly; (b) bench-characterise return loss/efficiency on an assembled unit and fold the result into the next revision, as the base-station record prescribes.

**NB-2 — placement WAIVED (owner decision, 2026-08-05); residual actions remain. C12 stands off above the bottom-side parts by design.** The review carried July B-2 forward as a blocker (✔ pyro verifier confirmed; switched-power's verifier independently upgraded its copy to blocker): the Ø18×25 can's body circle covers ~19 populated parts, now including the 3.1 mm-tall C56 330 µF hold-up tantalum and a SOIC-8. The owner confirms the mounting is intentional — the can sits above the parts on its leads, a board-area vs vertical-space/serviceability tradeoff — and accepts the taller tantalum beneath it. Recorded as such. **Residual actions that survive the waiver (all cheap, none move the can):** (1) **polarity silk** — still nothing but an ambiguous ring on the *negative* pad; a reversed 16 V electrolytic charged to 8.4 V on the pyro store vents. Add '+' at pad 1 and a filled bar on the negative side, placed clear of the can body (July H13, ✔ still open). (2) **Retention — SETTLED 2026-08-05:** 3D-printed TPU interposer (~95A, 4.0 mm, pocketed to clear C56/U13/the SOT-23s, feet landing on the two bare-board pockets at x 76.0–88.3 / y 142.0–150.0 and x 80.8–88.3 / y 135.5–143.7), with a **non-corrosive alkoxy-cure RTV silicone to MIL-A-46146** (flexible, gap-filling, rated for a 4 mm bondline) bonding can-to-board through the interposer openings; the TPU is captured mechanically, not bonded. Acetoxy-cure RTV and all epoxies are excluded — see `FABRICATION-NOTES.md` block B for the full spec and rationale. (3) **Honest 3D model** — replace the Ø16 STEP with the true Ø18×25 so clearance checks and renders tell the truth; re-verify bay inner diameter against board + standoff + 25 mm can. (4) DRC remains structurally blind here (no courtyard, `missing_courtyard` ignored) — now *consistent* with a deliberate overlap, but note the decision in the DRC config / footprint description so the next review doesn't re-flag it as an error.

**NB-3 — FIXED & VERIFIED 2026-08-05 (applied in-session). TPS62152 (U18) PVIN pin 12 had been floating for three consecutive reviews.** ✔ originally verified, found independently by 4 finders this run: `unconnected-(U18-PVIN-Pad12)` was in the netlist and pad 12 had no copper, so the entire 3V3-rail input current (S3 + NAND + the whole switched P4 domain via U22) squeezed through the single pin-11 bond, against TI's pin table ("11,12 PVIN — Supply voltage for power stage. Connect to same source as AVIN").

**What changed** — net assignment + zone refill only, **no new tracks**, exactly as predicted, because pads 11 and 12 are adjacent inside the existing AVIN pour:

1. `hardware/symbols/Custom.kicad_sym` **and** the cached copy in `power.kicad_sch`, symbol `TPS62152RGTR`: pin 12 moved from its orphan position (−17.78, 0) to **stack on pin 11** at (−17.78, 11.43), hidden. This is the symbol's own established convention for duplicated pads (pins 2/3 stack on pin 1; pin 16 stacks on pin 15) and is what joins pin 12 to pin 11's net **with zero schematic wiring**. A stray duplicate pin-12 entry at (−17.78, 7.62) from an in-progress edit was removed, leaving exactly one pin 12.
2. `rocket-computer.kicad_pcb`: U18 pad 12 net `unconnected-(U18-PVIN-Pad12)` → `Net-(U18-AVIN)`, then zones refilled. The F.Cu AVIN pour (priority 20, `connect_pads thru_hole_only`, so SMD pads fill solid) already enclosed pad 12 in its outline — it was only carving a clearance ring around it because of the foreign net.

**Verification:** netlist `Net-(U18-AVIN)` = {C43.1, L5.2, U18.10, U18.11, **U18.12**, U18.13}, and the `unconnected-` net no longer exists anywhere in the project; the AVIN fill polygon now geometrically contains pads 10/11/12/13; DRC (severity-all) reports **0 unconnected items and not one new violation**; ERC `pin_not_connected` 47→46 and `pin_not_driven` 1→0, with zero remaining ERC items naming U18 pin 12. Zone-area audit of the refill: AVIN **+0.5164 mm²** (the pad plus its former clearance ring), GND **−0.0151 mm²** where AVIN expanded, V_CAP re-tessellated with zero area change, every other zone byte-identical — the refill was surgical. The only other PCB-file delta is one via re-serialised in place (a known pcbnew save artifact, identical content and UUID).

**Two latent traps fixed in passing**, both introduced by the 05:41 symbol-library import rather than by the routing work: (a) the imported vendor copy of `TPS62152RGTR` had moved the **AVIN pin from y=6.35 to y=3.81** while the schematic is wired at 6.35 — a future "Update Symbols from Library" would have silently detached the AVIN wire; restored to 6.35, the value at git HEAD. (b) U18's **instance** footprint property had been changed to `TPS62152RGTR:QFN50P300X300X100-17N`, a library nickname present in neither the project nor the global `fp-lib-table`; restored to `Footprints:QFN50P300X300X100-17N`. U18 was the only instance in the sheet pointing at an unresolvable library — every other one uses `Footprints`/`Resistor_SMD`/`Capacitor_SMD`/`Connector_JST`/`LED_SMD`. ERC `footprint_link_issues` returned to 0 and `lib_symbol_mismatch` dropped 7→6. **Note:** the same import added 182 symbols to the shared `Custom.kicad_sym` (64→246, nothing removed — verified against HEAD), and left 6 symbols whose schematic copies now differ from the library (`W25Q128JVSIQ` ×2, `SC-32S-32.768kHz-12.5pF` ×2, `CSD16323Q3`, `MX35UF4G24AD-Z4I8`). Those are warnings only, but each is the same class of trap as the AVIN pin — worth a look before any library re-sync, and the library is shared with the base-station, LoRa and GNSS boards.

**NB-4 — PARTLY ADDRESSED 2026-08-05; one neck left.** *Power-path copper vs. what the eFuse permits.* Two rounds of edits landed: B.Cu was added in parallel at the eFuse exit (y 133–135), 13 vias were added, and the stackup was corrected to `JLC06161H-3313` so the inner layers are now honestly declared at **0.5 oz** rather than the 1 oz the file previously claimed. Measured on the current file, bounded to the corridor between the J3 load pads and the eFuse output, the binding constraint has **moved** from the eFuse exit to **y 123.75–126.75, where In2 runs 2.15–2.55 mm with no parallel copper on any layer** — about 1.25 A at 10 °C rise, 2.4 A at 45 °C. Median corridor width is 3.40 mm. Extending the same B.Cu parallel run down through y 120–130 closes it. Note the stackup correction halved every inner-layer ampacity figure relative to earlier numbers in this report — that is the file becoming honest, not a regression. Original finding follows.

**NB-4 (original). The power-path copper cannot carry what the eFuse now permits — the July B-3 current-budget decision was half-made.** ✔ verified on the component side (power-input verifier confirmed all five findings); the layout-power deep-measurements are ○ but agree in direction. The connector got fixed (J8 = JST VH, 10 A class — real, confirmed); ILIM came down to 11.6 A typ (R48 = 127 Ω); but between them: the eFuse **IN pad's pour seam is ~0.63 + 0.43 mm** of 1-oz copper (≈2.5 A at 10 °C rise; TI demands uniform pad soldering and 2× full-load copper), the **OUT pour necks to 0.89 mm** (vs TI's 50-mil minimum), the entire VBATT distribution rides **one 2.25–2.6 mm In2 corridor at 0.5-oz inner copper** (~1.3–2.6 A class, ~40 mm long, 4×0.3 mm vias at each end), and J8's battery+ pad reaches its pour through **4×0.5 mm thermal spokes** (~4–5 A). Meanwhile the servo branch the ECO budgets at 12–14 A stall still exits through two 2 A Milli-Grid contacts and a 0.8 mm return trace (July H5/H16, ○ this run). **Decide the budget, then make the copper and ILIM tell the same story:** either (a) bound servo current in writing (≤ ~4 A aggregate), drop ILIM to protect the real weakest link, and fix just the eFuse seams + J8 spokes (zone-outline redraws — do these regardless), or (b) engineer for stall: duplicate the corridor on B.Cu, ≥6 mm In2, 8–10 vias per transition, bigger return FET, more J3 power pins. Note the stackup declaration says 1 oz on all six layers while the JLC 6-layer class this board targets is 0.5-oz inner — the fab order must pin this, it is a 2× factor in every inner-layer number (○, Annex A.layout-mfg).

### High — fix before fab (deduplicated)

- **eFuse UVLO still has no deglitch — third appearance.** ✔ A ms-class pack sag below 6.34 V still hard-cuts VBATT mid-flight; recovery needs >6.91 V + retry + ramp, far longer than hold-up. Worse, `power-eco.md` line 128 *claims* "deglitched, latched" — the doc asserts a circuit that does not exist (C45/C46 actually configure ~1024 auto-retries). Add ~1 µF on EN/UVLO (τ ≈ 174 ms; +0.4 s turn-on) or lower UVLO to ~5.8–6.0 V and let INA230 telemetry own the warning. Pairs with: **C56 hold-up arithmetic doesn't close** (✔ medium: 3–6 ms at flight load vs the ECO's 52 ms figure; the eFuse outage it must ride is ≥20 ms). One decision covers both.
- **Pyro gate-drive is still one enabled pull-up away from firing.** ✔ Unchanged since July: the DTC123J's 47 k divider vs a worst-case ~45 k P4 internal pull-up, and the arm gate's 100 k (R22), let a single wrong WPU bias a base / lift PYRO_ARM's gate past the CSD16323Q3 threshold. Boot glitches are the exact scenario the safe-pyro-init discipline exists for — make the hardware immune: stiffen base pulldowns / resize dividers, then re-check against the P4 IO-MUX boot table. (Boot-state table itself re-verified clean this run — no pyro GPIO has IE/WPU/WPD at reset; the risk is firmware/crash-time, not ROM-time.)
- **GNSS, LoRa and all 12 EXP lines are still ground-switched with live supplies and no series impedance at the rocket end.** ✔/◐ July H6's mitigation landed on exactly one branch: camera UART got R30/R32 1 k. The sam10m8 GNSS daughterboard carries its own 1 k (R7–R9, db side — this is why one finder's GNSS claim was *refuted*; the loop is bounded for that pairing). Still bare: LoRa J5.3/4 → U15.15/16 direct, EXP_01–12 → P4 direct (netlist-confirmed by hand), and the px1105r GNSS board's RXD2 line. Off-branch floats to VBATT and injects through every unprotected signal; load-shedding dumps current into GPIO clamps (arch ○). Finish the job: 1 k in every remaining crossing line, or hard-ground J1.4/J5.1 and keep the daughterboards' local switches as the shed mechanism.
- **The P4 flight computer's power is gated by live S3 firmware.** ◐ netlist-confirmed: TPS22918 ON ← R71 ← `POWER_SWITCH` = S3 GPIO7, with R68 100 k pulldown → default OFF. The interlock is implemented exactly as the ECO documented (✔ clean check) — but the flight consequence deserves a conscious decision: any S3 crash, reset, or brownout-recovery drops V_MCU_SWTCH and power-cycles the P4 (and its sensors) mid-flight. Options: latch the rail (P4 self-hold OR-gate), move deploy authority fully to the S3, or accept + document with the S3 watchdog strategy.
- **Chip-revision build state is self-contradictory.** ✔ All four v3.x provision parts (R74/R75 499 k FB divider, C93 22 pF, R76 0 R pad-54 link) are DNP — the as-built board is the endorsed **v1.3-stock** configuration (closure B1 verdict: correct *in that form*) — but the BOM export counts C93 as populated while dropping the others, R74/R75/R76 have no MPN, and nothing in the repo pins the purchased silicon to v1.3. One purchasing slip (v3.x chip on a v1.3-built board) = floating core pin + no buck feedback = no-boot on an unreworkable QFN. Align DNP flags + BOM + a written silicon-revision pin, or just populate the v3.x network and buy v3.x.
- **Fiducials: still exactly one, still under the USB-C shell.** ○ (two finders; it is also still the board's only DRC error, so near-certain). Two chip-down 0.35/0.4 mm QFNs per side want 3 global fiducials per side, 1 mm copper / 2 mm mask, clear of courtyards.
- **The fab-package data on disk contradicts the live board.** ○ + ◐ `bom.csv`/`bom.xlsx` are 11 days stale (wrong flash, wrong S3 variant, deleted parts still listed, old values); the stray `rocket-computer 2.kicad_pcb` (Aug-2 layout, diffs against the real one — hand-verified) plus 34 backup zips sit in the project folder. Regenerate BOMs from the live schematic at order time; delete or archive the stray layout before someone gerbers the wrong file.
- **Pyro fire-return rides a deliberate 1.0 mm In2 trace and two 0.3 mm vias.** ○ unique to the lost layout-grounds verify pass; coherent with a ✔ clean check that saw the same "deliberate GND fire-return trace" on In2. Fire pulses are brief (~7–25 A for ms), so this may be survivable — but two 0.3 mm vias as the only return for the deployment pulse deserves an eyeball + a widened path or 4–6 vias before fab.
- **MCU power trace widths sit under Espressif minimums.** ✔ (P4 + S3 versions verified at medium, layout-power's harsher version ○): VDD_HP branches 0.127–0.2 mm vs 20-mil guidance with a 4-mil branch through R76; S3 VDD3P3 PA feed 0.1 mm vs 20-mil (carries 340 mA TX bursts); assorted +3V3 branches 0.127–0.2 mm. Cheap fattening pass while zones are being redrawn for NB-4.

### Decide consciously (medium highlights — full list in Annex A)

- **Arm interlock is still low-side and J2.5 is still silk-labeled "GND"** ✔ — same July complaint: one harness short from e-match return to real ground bypasses the arm FET, and the label invites it. Relabel at minimum.
- **In3 coupling:** buzzer PWM now runs 80 mm at 0.1 mm gap from PYRO2_FIRE ✔ (re-route regression); separate or ground-pour between them.
- **3V3 buck's only input cap (C43) is on the opposite board side through a single 0.3 mm via** ✔ — TI wants the input loop tight against PVIN/PGND; add a local 0402/0603 at U18 even if C43 stays.
- **TPS2121 ILIM strapped to device max (5.2 A)** ✔ — no useful protection for a ~1 A logic branch; USB VBUS also has zero capacitance before the mux ○.
- **Crystal load caps overshot the correction:** ECS-400-10-37B2 is CL = 10 pF (◐ MPN-confirmed); 2×12 pF ⇒ CL_eff ≈ 8.5–9 pF ⇒ frequency pulled high ~10–20 ppm — matters for RF center frequency. 15–16 pF lands on 10 pF; recompute once, change four values ○.
- **INA230 sense is still half-Kelvin** ✔ (IN+ taps mid-pour, better than July's connector-pad tap but still reads high); **eFuse PG / buck PG still dead-end** ○ — no MCU sees main-power faults, and eFuse auto-retry makes brownouts invisible to logs.
- **No reverse-polarity protection** ✔ — CR2's shunt clamp is the only element in the path; a reversed home-crimped VH pigtail is plausible (see the silk item below). Decide: ideal-diode/FET, or keying discipline + silk fix only.
- **Magnetometer sits over the LoRa switched-return pour** ✔ (downgraded from high: E220-22 dBm class ⇒ ~50–150 mA ⇒ ~5–15 µT perturbation, calibratable but real); **BMP585 has vias/mask/traces under its body** vs Bosch guidance ✔.
- **NAND EPAD still gets 100 % paste** ✔ (carried M20); **still zero testpoints** ✔ (carried M22) — the USB-C chain (FSUSB63 truth table now verified against the onsemi datasheet ✔, closing July's unfetchable-datasheet item) remains the only programming/console path for both chip-down MCUs.
- **Strap pins GPIO34/37/38 still run bare to J3** ✔, and the P4 ROM boot log (UART0 = GPIO37/38) transmits onto servo channel EXP_10 at every reset ○ — a payload can corrupt boot mode and servos twitch on boot chatter.
- **JLC checks:** one same-net via pair at 0.163 mm hole-to-hole vs JLC's 0.2 mm same-net minimum ○; camera J4 carries raw pack voltage — fine *for a RunCam Split 4* (5–20 V input), but the camera model is still not pinned anywhere in the repo ○.
- **Indicator LEDs are driven at 40–140 µA through 10 k** ○ — likely invisible in daylight; retune while touching the BOM.

### The "anything dumb" file

The July class-check comes up clean again — 0 unconnected items, 0 parity issues, every connector ground pin has real copper, both daughterboard links pin-for-pin correct including grounds ✔. This run's actual dumb-things list: the **battery polarity silk** ('- Batt +' still runs horizontally beside the now-vertical VH pins, with the '+' character nearest the **GND** pin ✔ — one-minute fix, fire-class consequence), the 'GPS' label buried under the camera connector body ○, the self-contradicting DNP/BOM state, FID1 hiding under the USB shell, and the stale BOM/stray-PCB files.

### Verified clean (a lot survived adversarial attack this time)

All 8 July "fixed" claims confirmed under attack: P4 v3.x provisions (pad 54 → R76 → VDD_HP_1, Fig.-5 FB network topology exact), JST VH battery input (real VH land pattern, PH-mating hazard eliminated, GND return through 5 parallel layers), S3 log flash moved to +3V3, CHIP_PU RCs on both MCUs (10 k + 1 µF per HDG). Also affirmatively clean: In1/In4 are solid single-polygon byte-identical GND planes, 163 stitch vias, worst stitch gap 6.9 mm; PYRO_GND is a proper star at the U9 source with a 1 k bleed; every eFuse/TPS2121/TPS62152/TLV62569/TPS22918 strap pin datasheet-valid (ITIMER 4.7 ms blanking, XCOMP priority math sane, OV limits block 3S packs); boot straps safe on both MCUs; crystal layouts pass every HDG rule (6.9 mm spacing, zero vias, solid plane below); the Molex 47948 on-ground antenna copper correct by design (re-affirmed); FSUSB63 truth table finally fetched and the mux wiring verified end-to-end; R72 is a real 1 W current-sense part; buzzer has its flyback; every active IC lifecycle-checked Active (S3RH2 is the designated upgrade for the EOL'd R2); sensor pinouts verified pin-for-pin with no strap conflicts; mag is ≥6.5 mm from every high-current path except the LoRa item above. Full lists per domain in Annex A.

### Suggested order of work

1. ~~**PVIN pad 12** (NB-3)~~ — **DONE 2026-08-05**, verified by netlist/ERC/DRC (see NB-3).
2. **RF feed under the NB-1 waiver** — resize the feed toward 50 Ω for the ordered stackup and declare it (RF netclass + explicit JLC stackup in the fab order), mirroring base-station S7; the pi stays deleted by decision.
3. **C12 residuals under the NB-2 waiver** — polarity silk, true Ø18×25 STEP, formed-lead + staking plan (insulate over C56), bay-height check; the placement itself stays.
4. **Current-budget decision** (NB-4) — write down the servo budget; redraw eFuse IN/OUT zone outlines and J8 spokes regardless; size In2/vias/ILIM/return-FET to the chosen budget; pin inner-layer copper weight in the fab order.
5. **Brownout decision** — UVLO deglitch cap or lower threshold; reconcile with C56 hold-up math; fix the power-eco.md doc lie.
6. **Pyro safety pass** — stiffen gate pulldowns, relabel J2.5, eyeball the In2 fire-return, decide arm topology consciously.
7. **Build-state hygiene** — pin the P4 silicon revision in writing and align DNP/BOM; regenerate bom.csv/xlsx from the live schematic; delete the stray `rocket-computer 2.kicad_pcb`; 3 fiducials/side; battery + GPS silk; 1 k on the remaining cross-board lines.
8. **Cheap opportunistic** — 15–16 pF crystal caps, local input cap at U18, MCU power-trace fattening, NAND paste segmentation, testpoints, LED resistors, EP via counts (S3 EP has 4 vs Espressif's 9 ✔).
9. **After the session limit resets** — re-run the 7 lost verify passes for free from cache: `Workflow({scriptPath: <session script>, resumeFromRunId: "wf_0c4ccb31-dd2"})` (all 14 finder results replay; only the failed verifiers run).
10. **Bench items carried:** servo-stall sag at VBAT_CON, RunCam record-inrush on battery, RF characterisation with the antenna installed (L1 is the only trim point — NB-1 waiver), crystal frequency check at temperature, C12 bay-fit + staking check after assembly.

---

# Annex A — Full findings by domain

Verdict tags: **[verified]** = adversarial verifier independently reproduced the evidence; **[refuted]**/**[downgraded]** = verifier overturned or resized it (kept for the record); **[unverified]** = finder evidence only — the verify agent for this domain was lost to the session limit (see header). Findings are per-domain as found, so the same defect can appear under several domains; the executive summary above is the deduplicated view.


## A.power-input — Battery input & primary protection

### HIGH [verified] — eFuse EN/UVLO still has no deglitch capacitor - prior H2 never implemented despite ECO claiming 'deglitched'
*eFuse / brownout architecture — confidence high*

Net-(U19-EN/UVLO) contains only R44 (1 M), R45 (210 k) and U19 pin 6 - no capacitor - so the TPS259824L UVLO comparator (us-class response, falling threshold 6.34 V = 3.17 V/cell sensed at VBAT_CON, only the 2 mOhm shunt upstream of the pack) will hard-cut VBATT on a millisecond servo-stall/connector sag, exactly the mid-flight brownout class this respin exists to kill. power-eco.md line 128 claims 'UVLO 6.4 V, deglitched, latched' but neither deglitch nor latch-off is in the live schematic (C45/C46 configure ~1024 auto-retries instead). Recovery needs VBAT_CON back above 6.91 V plus dVdt ramp (~18 ms at 0.46 V/ms) - longer than the V_MCU_2S hold-up at flight load, so both MCUs reset.

**Evidence:** netlist.net Net-(U19-EN{slash}UVLO) = {R44.1, R45.2, U19.6} only; schematic.pdf page 5 shows no cap on pin 6; TPS25982 SLVSEI3D EC table VUVLO(R/F) = 1.2/1.1 V -> 1.2x(1.21M/210k)=6.91 V on, 1.1x5.762=6.34 V off (matches the schematic's own '6.4 V cutoff' note); prefab-review-2026-07-30.md H2 quotes schematic-review.md 3.5 requirement 'RC deglitch >=100 ms so servo/radio transients cannot trip it'; power-eco.md:128 asserts 'deglitched'.

**Fix:** Add ~1 uF from EN/UVLO to GND (divider Thevenin ~174 k -> tau ~174 ms; adds ~0.4 s turn-on delay, acceptable), or lower UVLO to ~5.8-6.0 V and let INA230 telemetry own the soft warning. Bench-verify worst-case stall sag at VBAT_CON either way. EN/UVLO abs max is 7 V - 1.46 V at 8.4 V pack leaves room, the cap is safe.

**Sources:** TI TPS25982 SLVSEI3D (pages 5-9, 21) fetched 2026-08-04; prefab-review-2026-07-30.md H2; power-eco.md

**Verifier:** confirmed — Fully reproduced from the live netlist and datasheet. Net-(U19-EN{slash}UVLO) contains exactly {R44.1, R45.2, U19.6} - no capacitor. R44=1M/R45=210k gives 1.2Vx5.762=6.91V rising / 1.1Vx5.762=6.34V falling (EC table lines 415-417 of SLVSEI3D). power-eco.md line 128 asserts 'High-side eFuse, UVLO 6.4 V, deglitched, latched' - neither is implemented: no EN cap, and C46 2.2nF on RETRY_DLY + C45 1uF on NRETRY configure finite-delay auto-retry (Table 7-5: 2.2nF/0.47uF row = 1024 retries; 1uF computes above that) rather than latch-off. dVdt recovery ramp verified: C51=10nF with IdVdt 4.6uA typ = 0.46V/ms, ~18ms to 8.4V, longer than the 4-12ms V_MCU_2S hold-up computed in prior H2. EN abs max 7V confirmed (pin sits at 1.46V at 8.4V pack, so the fix cap is safe). Prior review H2 (line 124) is verbatim this finding; it remains unfixed in tonight's files.

### HIGH [verified] — Battery current into the eFuse funnels through ~1 mm of 1-oz copper: VBAT_CON zone outline stops at the IN thermal pad edge
*layout-power / eFuse IN — confidence high*

The main VBAT_CON B.Cu pour (13.8 mm2, fill ends at y=134.57) touches U19's IN thermal pad 25 (global y 134.49-137.19) only along a 0.63 mm-wide seam plus pin 16's 0.43 mm pad; the upper pour feeding IN pins 1-3 (fill starts y=137.10) touches pad 25 only along a ~1.3 mm seam, and the F.Cu pour is a dead-end stub reachable only through the 5 in-pad vias. So the ENTIRE pack current (ILIM 11.6 A typ, device rated 15 A, ECO stall case 12-14 A) crosses ~1.06 mm total of 1-oz copper: ~2.5 A at 10 C rise / ~4.8 A at 45 C by IPC-2221, current density ~313 A/mm2 at ILIM. Local overheating lands directly on the protection device's solder joints; the choke also degrades the 'uniform current distribution' TI requires for the internal current sense.

**Evidence:** rocket-computer.kicad_pcb: U19 at (76.76,135.8425) rot 90, pad 25 (IN, 2.7x1.45) center (77.385,135.8425); zone scanlines: main pour spans x76.78-77.41 at y=134.40-134.55 (0.63 mm), pin16 span 0.43 mm at y=134.0, upper pour spans x76.83-78.13 at y=137.12 overlapping pad-25 top by 0.09 mm; no other VBAT_CON copper bridges y129.5-137.1 (all 5 VBAT_CON vias sit inside pad 25). TPS25982 SLVSEI3D pin table: 'The exposed pad must be soldered to input power plane uniformly...to maintain optimal current distribution'; layout guidelines: 'High current carrying power path connections...sized to carry at least twice the full-load current'.

**Fix:** Redraw the VBAT_CON B.Cu zone outline to fully engulf pad 25, pins 1/2/3/16 and join both pours into one (the zone is already connect_pads=thru_hole_only, i.e. SMD pads fill solid - only the outline is short). Same pass on the OUT side: VBATT pour narrows to 0.89 mm (35 mil) at y=134.59 vs TI's 50 mil minimum for OUT.

**Sources:** Live rocket-computer.kicad_pcb parsed 2026-08-04; TI TPS25982 SLVSEI3D p.5, p.46

**Verifier:** confirmed — Core defect reproduced exactly and is if anything slightly worse than claimed. My scanline of the live main VBAT_CON B.Cu pour (13.8mm2, fill ends y=134.57) measures 0.627-0.631mm of copper ([76.78,77.41]) crossing pad 25's bottom edge band y=134.40-134.55; the upper pour (fill starts y=137.10) overlaps pad 25's top edge (y=137.1925) by only 0.09mm with a 1.30mm span at y=137.12; all 5 VBAT_CON vias sit inside pad 25 so the 4.8mm2 F.Cu zone is a dead-end; the VBATT OUT pour narrows to 0.889mm (35 mil) at y=134.59 vs TI's 50-mil minimum (tps25982.txt line 2601). Both TI quotes verified (pin table lines 199-201, layout lines 2592-2601). One detail correction: 'pin 16's 0.43mm pad' - pad 16 is 0.24mm wide and its copper lies inside the same 0.63mm fill interval, so the total entry seam is ~0.63mm, not ~1.06mm; the finder's IPC numbers (2.5A at 10C, 4.8A at 45C, ~313A/mm2 at ILIM) reproduce for 1.06mm and get worse at 0.63mm. High severity stands: full pack current through a sub-millimetre seam at the protection device's solder joints.

### HIGH [verified] — All VBATT distribution rides one 2.3-3 mm wide 0.5-oz internal (In2) corridor - roughly a 1.3-2.6 A path feeding claimed 12-14 A loads
*layout-power / VBATT distribution — confidence high*

Every downstream load except the mux/pyro-charge (servo J3.15/16, camera J4.2, GNSS FL1, LoRa FL2, bulk C14/C15/C18) is fed from the eFuse OUT through a single In2 pour corridor: min width 2.29 mm (y=134/136), 2.6 mm (y=122), ~3 mm typical, ~40 mm total run, entered through 4 vias (0.4/0.3 mm) at the eFuse and exited to the J3 servo pads through 4 vias. On JLCPCB-standard 0.5 oz inner copper that is ~1.3 A at 10 C / ~2.6 A at 45 C rise (IPC-2221 internal), ~13 mOhm end-to-end: nominal 4-6 A servo running dissipates 0.2-0.5 W buried in the stackup, and the ECO's 12-14 A stall case (~1.9 W, ~3 A per 0.3 mm via) is far outside any rating while the 11.6 A ILIM will not intervene.

**Evidence:** rocket-computer.kicad_pcb In2 VBATT zone (163 mm2, bbox x76-94.5 y101-139) scanline widths: y=136: 2.29 mm, y=122: 2.60 mm, y=120: 3.15 mm, y=105: 2.73 mm; VBATT vias (79.06,138.11),(79.05,137.18),(79.07,133.37),(78.85,137.64) eFuse side and (92.07,107.27),(91.69,106.85),(92.29,101.36),(91.74,101.36) J3 side, all size 0.4/drill 0.3; J3.15 (92.06,103.40), J3.16 (92.06,107.65); board stackup In2 = power layer per project declaration.

**Fix:** Duplicate the corridor on B.Cu (there is room along the right edge), widen In2 necks to >=6 mm, and double the via count at both transitions - or make the conscious decision to cap the servo budget (<=4 A aggregate) and drop ILIM to match (see ILIM finding), then document it. Confirm inner copper weight actually ordered; 1 oz inner doubles these margins but still does not reach ILIM.

**Sources:** Live .kicad_pcb zone fills; IPC-2221 internal-conductor chart; power-eco.md Change 2 servo-stall assumption

**Verifier:** confirmed — Reproduced. The In2 VBATT zone (163.4mm2, bbox x76.04-94.51 y101.15-139.02) is the only path south: my scanlines give 2.29mm at y=136, 2.60mm at y=122, 3.15mm at y=120, 2.73mm at y=105 (matching the finder to 0.01mm), with an even narrower 2.19mm at y=125 the finder missed; no VBATT copper on any other layer bridges y118.6-133.5 (B.Cu pours are local to the eFuse and J3 areas; F.Cu segments are short local jogs). Entry/exit via counts and positions reproduce exactly: 4 vias 0.4/0.3mm at the eFuse (79.06,138.11),(79.054,137.18),(79.07,133.37),(78.85,137.64) and 4 at J3 (92.067,107.27),(91.69,106.85),(92.29,101.36),(91.742,101.358); J3.15/16 at (92.06,103.40)/(92.06,107.65). IPC-2221 internal math checks (0.5oz x 2.3mm = 62 mil2 -> 1.3A at 10C, 2.6A at 45C; ~14mOhm over the ~40mm run). Caveat as the finder already hedged: the .kicad_pcb stackup declares In2 as 0.035mm (KiCad default), but JLCPCB-standard 6-layer is 0.5oz inner; even at 1oz the corridor is a ~2.6/5.2A path vs 11.6A ILIM and the claimed 12-14A stall, so severity is unchanged either way.

### HIGH [verified] — Polarity silk '- Batt +' runs perpendicular to J8's pin axis; the '+' sits nearest the GND pin
*silkscreen / battery input — confidence high*

The bottom-silk legend '- Batt +' at (88.68,124.02) is horizontal, but the new JST VH J8 is rotated -90 so its pins are stacked vertically: pin 1 = GND (square pad) at (84.73,125.59) directly below the text, pin 2 = VBAT_Terminal (round) at (84.73,129.55) further away. Neither the '-' nor the '+' is adjacent to its pin, and the '+' end of the text is physically closest to the GND pin - the legend is a leftover from the old horizontal 2 mm-pitch J7 and now invites a reversed home-crimped pigtail. Because the board has no series reverse protection (see reverse-polarity finding), acting on this silk destroys the front end.

**Evidence:** rocket-computer.kicad_pcb: gr_text '- Batt +' at (88.68,124.02) rot 0 B.SilkS; J8 pads (84.73,125.59)=GND square / (84.73,129.55)=VBAT_Terminal round; drc.json logs 5 'Silkscreen clearance' warnings of this text colliding with J8's own outline; render_bottom.png crop confirms the text reads across the connector top with '+' at upper-right above the GND pin. Old J7 pads were at (84.44,129.06)/(86.44,129.06) - horizontal - per prefab-review-2026-07-30.md B2.

**Fix:** Delete the stale text; place '-' beside pad 1 and '+' beside pad 2 (there is clear silk area left of each pad), and keep the VH outline. One-minute edit, impossible after fab.

**Sources:** Live .kicad_pcb, drc.json, render_bottom.png; prefab review B2 for old J7 geometry

**Verifier:** confirmed — Reproduced from the live file, DRC, and render. gr_text '- Batt +' at (88.68,124.02) rot 0, B.SilkS, justify left/bottom/mirror, 1mm bold - horizontal. J8 (JST B2P-VH, rot -90) has pin 1 = GND square pad at (84.73,125.593) and pin 2 = VBAT_Terminal round pad at (84.73,129.553): pins stacked vertically, text perpendicular to the pin axis and nearest pin 1. The mirrored left-justified text extends from x=88.68 toward lower x, putting the '+' character at roughly (82,123.5) - ~3.3mm from the GND pin vs ~6.5mm from the actual + pin. The bottom-side render confirms it visually: text sits across the connector top with '+' adjacent the square GND pad. drc.json contains exactly 5 silk_overlap violations of this text against J8's own silkscreen outline, confirming it is a leftover not repositioned for the new vertical VH footprint. Zero-cost fix, permanent after fab, board has no reverse protection (separate finding) - high is right.

### HIGH [verified] — ILIM lowered to 11.6 A (R48 127R) but still exceeds every series element downstream of the eFuse - and now the 10 A battery connector too
*architecture / current budget — confidence high*

R48 = 127 Ohm gives ILIM = 1460/127 + 0.11 = 11.61 A typ (~10.2-12.6 A over tolerance/temp, fast-trip 2.1x = 24.4 A). That is above: J8's 10 A VH rating (marginally), the ~1 mm eFuse IN copper seam (~2.5-5 A), the In2 corridor (~1.3-2.6 A class), the J3 servo pair (Molex Milli-Grid ~2 A/circuit, 4 A for the two VBATT pins), and the J4 camera PH contact (2 A). The prior review's core demand (B-3: 'set ILIM to protect the actual weakest link' or bound the servo budget in writing) is still unmet - the connector upsize landed but the protection threshold protects nothing except the eFuse die itself. Conversely if the 12-14 A stall case is real, ILIM+ITIMER (4.7 ms) will nuisance-trip right at stall onset while the copper still cooks during the ITIMER window and during sub-ILIM sustained overloads.

**Evidence:** netlist R48 = 127 / RC0402FR-07127RL on Net-(U19-ILIM); TPS25982 SLVSEI3D Eq.8: 'RILIM(Ohm) = 1460/(IILIM(A) - 0.11)'; EC table RILIM=100R row 13.56/14.71/15.99 A shows ~+/-9% band; JST VH datasheet: 10 A AC/DC with AWG16 standard-type header; prefab-review-2026-07-30.md B-3/H19; J3 = Molex 878321620 (~2 A/ckt per prior review citation).

**Fix:** Decide the servo budget on paper first. If servos are bounded <=4 A aggregate: R48 -> 300 Ohm (4.98 A typ) protects J3, the corridor and J8 with margin, and the copper findings shrink to comfortable. If 12-14 A is real: TPS25983 (20 A) as the ECO already notes, plus the connector-grade copper and more J3 power pins - both, not neither.

**Sources:** TI TPS25982 SLVSEI3D p.9, p.25; JST eVH datasheet + distributor listings; prefab review B-3/H19

**Verifier:** confirmed — Reproduced. R48 = 127 Ohm (RC0402FR-07127RL) on Net-(U19-ILIM); datasheet Equation 4/8 'RILIM = 1460/(IILIM - 0.11)' gives 11.61A typ, and the EC-table RILIM=100 row (13.56/14.71/15.99A over temp) confirms the ~+/-9% band; fast trip = 210% of ILIM = 24.4A (ISC row). Downstream elements verified: J8 = JST B2P-VH(LF)(SN) (VH series, 10A with AWG16); J3 = Molex 878321620 Milli-Grid (~2A/circuit, two VBATT pins); J4 = JST B4B-PH-SM4-TB (PH, 2A contacts); plus the 0.63mm eFuse IN seam and 2.2-3mm In2 corridor quantified in the other confirmed findings. So ILIM protects nothing downstream, exactly the prior B-3/H19 demand ('set ILIM to protect the actual weakest link' or bound the servo budget in writing), which the connector upsize did not close. The nuisance-trip converse (ITIMER C50=10nF ~4.7ms blanking at the 12-14A stall the ECO itself posits) also checks out. Minor note: the finder's EC-table quote merges the 25C max and full-temp max columns, immaterial to the claim.

### MEDIUM [verified] — INA230 IN+ is still not Kelvin-connected to the shunt pad - tap moved from the connector pad to mid-pour, ~2.3 mm short of R72.2
*current-sense / telemetry — confidence high*

IN+ (U23.13) now routes B.Cu->via(79.7,127.83)->In2->via(81.38,129.82) and lands in the middle of the VBAT_Terminal pour, inside its 2.0 mm-wide neck, ~2.3 mm from R72 pad 2 at (79.57,131.28). All pack current flows through that pour section, adding roughly 0.25-0.4 mOhm of copper (tempco +0.39%/K) inside the measured path: pack-current telemetry reads ~12-20% high and drifts with temperature. This is an improvement over the prior tap at the J7 connector pad (was ~+50-75%) but still violates the INA230 Kelvin requirement. Two dead F.Cu/In3 sense stubs from the old tap remain inside J8 pad 2.

**Evidence:** rocket-computer.kicad_pcb VBAT_Terminal segments: B.Cu (79.200,127.770)->(79.7,127.83), In2 (79.7,127.83)->(81.38,129.82), via at (81.38,129.82); pour neck spans x81.11-83.13 at y=129.5-130.0; R72.2 center (79.57,131.2825); orphan stubs (84.41,129.56)->(84.039,128.659) F.Cu and (85.38,130.56)->(85.24,130.70) F/In3 inside J8.2. INA230 datasheet 8.4.1 (quoted in prior review M29): 'Connect the input pins (IN+ and IN-) to the sensing resistor using a Kelvin connection'. IN- correctly lands inside R72 pad 1 at (80.04,129.45).

**Fix:** Move the IN+ via to land inside or immediately abutting R72 pad 2 and route the sense as its own 0.1-0.2 mm trace; delete the two orphan stubs. Alternatively calibrate the ~+15% out in firmware and accept the tempco - but say so in the docs.

**Sources:** Live .kicad_pcb; TI INA230 datasheet 8.4.1 via prefab-review M24/M29

**Verifier:** confirmed — Routing reproduced segment-for-segment from the live board: U23.13 (pinfunction IN+_13, verified in netlist) leaves the pad on B.Cu (79.200,127.770)->(79.640,127.770)->via (79.7,127.83)->In2 (79.7,128.57)->(80.95,129.82)->(81.38,129.82)->via landing inside the VBAT_Terminal pour's ~2.0mm neck ([81.11,83.13] at y=129.55-129.75, my scanline), 2.33mm from R72 pad 2 center (79.57,131.282). All pack current traverses that pour region, so the tap adds shared-path copper inside the measurement; my estimate of the geometric resistance (~0.3-0.6mOhm of 1oz pour, partial current share) brackets the finder's 0.25-0.4mOhm / ~12-20% high reading, with copper tempco drift. IN- is indeed Kelvin-correct: U23.12 (IN-_12) 0.1mm trace terminates at (80.04,129.45) inside R72 pad 1 (x78.87-80.27, y128.95-129.97). Both orphan stubs verified inside J8 pad 2: F.Cu (84.41,129.56)->(84.039,129.189)->(84.039,128.659) and F.Cu+In3 (85.38,130.56)->(85.24,130.70). One citation slip: the prior-review Kelvin finding is M24 (line 628), not M29 (M29 is the In3 routing ribbon) - does not affect the claim.

### MEDIUM [verified] — Reverse battery connection still destroys the front end - CR2 shunt clamp is the only element in the path (prior M30 unresolved)
*battery-input / protection — confidence high*

There is no series reverse-polarity element. A reversed pack forward-biases CR2 (CUS10S30: IO 1 A avg, IFSM 5 A) into a dead short through the 2 mOhm shunt - tens of amps until the diode fails - after which VBAT_CON/VBAT_Terminal sit at negative pack voltage, violating INA230 abs max (IN pins GND-0.3 V) and TPS25982 IN abs max (-0.3 V). The VH shell is polarized so this requires a miswired pigtail, but that is the home-built-pack failure mode, and the ambiguous '- Batt +' silk (separate finding) makes the mis-crimp more likely, not less.

**Evidence:** netlist: CR2.1(cathode)=VBAT_CON, CR2.2=GND - shunt clamp only; no fuse or series FET anywhere in {J8.2 -> R72 -> U19.IN}; TPS25982 SLVSEI3D abs max VIN min -0.3 V; prior review M30 (same finding, netlist re-verified against tonight's files).

**Fix:** Either add a series P-FET (or even a simple fuse) at J8, or consciously waive with a documented harness QC step (continuity + polarity check fixture before first connect). Do not re-open the battery-protector-IC debate - this is orthogonal, as M30 already noted.

**Sources:** Live netlist; TI SLVSEI3D p.7; Toshiba CUS10S30 ratings via prior review M30

**Verifier:** confirmed — Reproduced from the live netlist: the entire series path J8.2 -> VBAT_Terminal -> R72 (2mOhm) -> VBAT_CON -> U19 IN pins contains no fuse, FET, or series diode; VBAT_Terminal = {J8.2, R72.2, U23.13} and VBAT_CON members confirm CR2.1 (cathode) = VBAT_CON, CR2.2 = GND is a shunt clamp only. CR2 = CUS10S30_H3F (1A avg Schottky) would conduct a reversed pack through just the 2mOhm shunt - far beyond its surge rating - after which VBAT_CON/VBAT_Terminal sit at negative pack voltage: TPS25982 VIN abs max -0.3V verified in the datasheet abs-max table, and INA230 IN pins (U23.12/13 directly on these nets) share the GND-0.3V class limit. J8's VH shell is polarized, so the exposure is the home-crimped pigtail - which the confirmed '- Batt +' silk finding makes more likely. Prior M30 (line 692) is the same finding, still unresolved. Medium (consciously waivable with harness QC) is the right severity.

### MEDIUM [verified] — J8 battery+ pad connects to its pour through 4x0.5 mm thermal spokes, then a 2.0 mm pour neck - ~4-5 A class copper behind a 10 A connector
*layout-power / battery input — confidence high*

The VBAT_Terminal zone is set to thermal-relieve through-hole pads (thermal_bridge_width 0.5, gap 0.25), so the full pack current leaves J8 pad 2 over four 0.5 mm spokes (2 mm total 1-oz copper, ~4 A at 10 C rise), then crosses a 2.0 mm-wide pour neck at y=129.5-130 (~4 A at 10 C / ~7.7 A at 45 C) before reaching the shunt. The connector was upsized to 10 A but its landing copper was not; at the ECO's stall currents the spokes are the hottest points on the input side. J8 pin 1's GND return is also relieved, but across 5-6 parallel plane layers, which is adequate.

**Evidence:** rocket-computer.kicad_pcb VBAT_Terminal zone header: (connect_pads thru_hole_only (clearance 0.127)) (thermal_gap 0.25) (thermal_bridge_width 0.5); fill scanlines show spoke fragments around pad (83.82-84.08 / 85.38-85.64 at y=129.0) and neck span x81.11-83.13 at y=129.5 (2.02 mm); pour total 19.2 mm2. GND zones: thermal_bridge 0.5 on layers F/B/In1/In3/In4.

**Fix:** Set J8's pads (at least pad 2) to solid zone connection - VH pins are hand/wave soldered, thermal relief is unnecessary - and widen the pour neck toward R72 to >=4 mm. Zero-risk edits.

**Sources:** Live .kicad_pcb zone settings and fill geometry; IPC-2221 external-conductor chart

**Verifier:** confirmed — Zone settings and geometry reproduce exactly: the VBAT_Terminal zone has (connect_pads thru_hole_only (clearance 0.127)) with fill (thermal_gap 0.25) (thermal_bridge_width 0.5), so the through-hole J8 pad 2 (2.7mm pad, 1.7mm drill) is relieved to four 0.5mm diagonal spokes - my scanline at y=129.0 shows the spoke fragments [83.82,84.08]/[85.38,85.64] the finder quoted, and the neck measures 2.019-2.030mm ([81.11,83.13]) at y=129.55-129.75. GND zones on F/B/In1/In3/In4 with 0.5 bridges confirm the pin-1 return note. One correction that does not change the verdict: the 2.0mm neck is not strictly in series with the pad - south of the pad the pour widens (3.15mm at y=130.5, 4.5mm at y=130.9, 6.9mm at y=131.2) and reaches R72.2 directly, so the true series choke is the 2mm aggregate of spokes, with the pour beyond it 2-4.5mm class. That is still thermal-relieved 1oz copper behind a 10A connector at claimed 12-14A stall currents; the fixes (solid connection on pad 2, wider neck) are correct and zero-risk. Medium stands.

### LOW [unverified] — eFuse PG output is pulled up but routed to nothing - eFuse state invisible to firmware
*eFuse / telemetry — confidence high*

PG_RAIL contains only U19.13 and R59 (100 k to +3V3). No MCU pin reads it, so an eFuse trip/latch-off in flight is only inferable from INA230 current collapsing to zero (BUS voltage still reads pack-side VBAT_CON and stays healthy-looking). For a vehicle whose last respin was driven by power-event forensics, one spare-GPIO trace buys direct evidence.

**Evidence:** netlist: NET PG_RAIL = {R59.2, U19.13} only; U23.11 BUS = VBAT_CON (input side, per ECO design 'works even when the output is off'); TPS25982 SLVSEI3D p.6: PG is open-drain, requires external pull-up (satisfied electrically).

**Fix:** Route PG_RAIL to a spare ESP32-S3 GPIO (S3 already owns PWR_SCL/PWR_SDA next to U23), or document the omission as intentional.

**Sources:** Live netlist; TI SLVSEI3D p.6

### INFO [unverified] — NRETRY strap computes N=1815, above the datasheet's highest quantization band
*eFuse / fault response — confidence high*

C45 = 1 uF on NRETRY with C46 = 2.2 nF on RETRY_DLY gives N = 4 x C_NRETRY/(C_RETRY_DLY + 4 pF) = 1815, but the TPS25982 quantization table tops out at '256 < N < 1024 -> 1024'; the configured point is outside the documented range (Table 7-5 pairs 2.2 nF with 0.47 uF for 1024 retries). Likely behavior is 1024 retries at ~92-103 ms intervals then latch-off, which is a sane flight policy, but it is an undocumented operating point. Cap voltage ratings on all strap pins meet the 4 V requirement.

**Evidence:** netlist: C45 1 uF (CL05A105KO5NNNC, 16 V) on Net-(U19-NRETRY), C46 2.2 nF (CC0402KRX7R9BB222, 50 V) on Net-(U19-RETRY_DLY); TPS25982 SLVSEI3D Eq.14/15, Table 7-4/7-5; tRETRY_DLY = 128 x 2204 pF x 0.75 V / 2.05 uA = 103 ms typ.

**Fix:** If the intent is 'retry as long as possible', short NRETRY to GND (infinite retries, documented) or drop C45 to 0.47 uF (1024, on-table). If latch-off after ~1024 tries is intended, note it in power-eco.md.

**Sources:** TI SLVSEI3D p.31

### INFO [unverified] — U19 footprint library name says TPS259827 while the part is TPS259824L
*library hygiene — confidence high*

U19's footprint is 'Footprints:IC_TPS259827LNRGER' but value/MPN are TPS259824LNRGET. Both are the same RGE VQFN-24 package and pinout (variants differ in OVP threshold and breaker-vs-limiter response), so assembly is unaffected - the BOM MPN governs - but the name mismatch is a future-confusion trap between the 16.9 V-OVLO limiter actually fitted and the 27-variant the footprint advertises.

**Evidence:** netlist U19: value TPS259824LNRGET, footprint Footprints:IC_TPS259827LNRGER, MPN field TPS259824LNRGET; TPS25982 SLVSEI3D device comparison (all TPS2598xx share the RGE package).

**Fix:** Rename the library footprint or add a note field; verify at order time that the reel is the -24L (active current limiter, 16.9 V OVLO) as the ECO analysis assumed.

**Sources:** Live netlist fields; TI SLVSEI3D


**Affirmatively verified clean (power-input):**

- Battery connector upsize is real: J8 = JST B2P-VH(LF)(SN) (Conn_01x02, Connector_JST:JST_VH_B2P-VH_1x02_P3.96mm_Vertical, THT 2.7 mm pads / 1.7 mm drills at (84.73,125.59)/(84.73,129.55) - correct VH land pattern); JST VH series rated 10 A AC/DC with AWG16 on the standard top-entry header, wire range AWG22-16 - prior B2/H3/H4/H19 connector-rating findings on the 2 A JST PH are closed at the connector itself (copper behind it still flagged).
- Electrical polarity of J8 is consistent everywhere: schematic page 5, netlist (J8.1=GND, J8.2=VBAT_Terminal) and footprint (square pad = pin 1 = GND) agree; only the silk legend is wrong (reported).
- Prior M12 (2-pos battery plug part-mating into the 4-pos PH camera port J4, 15 mm away) is eliminated by the family change - a VHR-2 housing cannot seat on a PH shroud.
- R72 shunt power rating fixed: MPN now CSSH0805FT2L00 (Stackpole all-metal current-sense, 2 mOhm 1%, 1 W, ~22 A capability, 100 ppm TCR) - closes the prior 0.125 W thick-film risk; IN- sense (U23.12) is a true Kelvin: dedicated 0.1 mm trace lands at (80.04,129.45) inside R72 pad 1.
- ILIM change landed as claimed: R48 = 127 Ohm 1% (Yageo RC0402FR-07127RL, E96); computed limit 11.61 A typ from datasheet Eq.8 (was 14.71 A at 100R). R48 sits 1.5 mm from pin 8 - well under the 30 pF parasitic limit TI specifies.
- All eFuse strap pins are datasheet-valid states: LDSTRT(12)->GND ('Connect to GND if not used'), IMON(9)->GND (unused), ITIMER C50 10 nF -> 4.7 ms typ blanking (10 nF x 0.98 V / 2.1 uA), safely under the max-CITIMER bound (Eq.10: limit ~0.5 uF with CdVdt 10 nF); dVdt C51 10 nF -> SR = 4600/10000 = 0.46 V/ms.
- Inrush is controlled at both events: hot-plug at J8 sees only ~1.1 uF (C41+C42) upstream of the eFuse (no spark/connector-erosion issue), and the dVdt ramp charges the ~400 uF VBATT bank at ~0.18 A - two orders below ILIM; pyro can C12 (10 mF) charges through R20 150 Ohm (56 mA max), invisible to the eFuse.
- UVLO divider arithmetic verified: R44 1M / R45 210k -> 6.91 V rising / 6.34 V falling (matches the schematic's '6.4 V cutoff for 2s LiPo' note); EN pin sees 1.46 V at 8.4 V, under the 7 V abs max; divider current is ~57x pin leakage per TI's 20x guidance. (Missing deglitch cap reported separately.)
- eFuse input decoupling meets TI's '0.01 uF or greater between IN and GND': C41 1 uF + C42 100 nF, both 16 V, placed 3.0 mm from U19; all support components (R48/C50/C51/C45/C46/R44/R45/R59) within 4.1 mm of the device.
- Paste stencil is complete despite the SnapEDA pad style: U19 pads 1-24 carry B.Paste on the pad layers, and thermal pads 25/26 have 4 windowed fp_poly paste apertures (~78-79% coverage - correct QFN practice); U23 INA230 has paste on all 16 pins plus a 9-window EPAD - confirmed both in the .kicad_pcb and in the exported layer_B.Paste.svg apertures. (An earlier read of the pad 'layers' lines alone would have false-flagged this.)
- U19 IN thermal pad has 5 vias (0.4/0.3 mm) to an F.Cu relief pour, and the board setup declares filled+capped vias (filling yes / capping yes) - the prior via-wicking-in-pads finding class is resolved for the eFuse and the shunt region.
- Bulk/support capacitor voltage ratings all clear on the 8.4 V rail: C15/C56 = Kemet TCJE337M016R0050 330 uF 16 V polymer tantalum (53% derating, inside the polymer 80% guideline, per the recorded I58/ECO check); C14/C18/C7 = 22 uF 16 V X5R 0805 (Samsung CL21A226MOQNNNE); C41/C54 1 uF 16 V, C42 100 nF 16 V; strap caps C50/C51/C46 50 V, C45 16 V vs the 4 V pin requirement.
- CR2 negative-transient clamp orientation correct: cathode (pin 1) to VBAT_CON, anode to GND - reverse-biased at 8.4 V (VR 30 V), matching the ECO's GND->IN clamp intent for fast-trip inductive kick.
- I58 TVS-on-VBATT waiver re-checked against the live files and stands: no TVS exists on VBATT in the netlist, the 330 uF bulk the waiver relies on is present and connected (C15 pad overlaps the B.Cu pour at y109.3-110.06 plus 2 vias into In2), and nothing in tonight's re-route changes the waiver's arithmetic (SMF10A clamps at 17 V - above the 16 V caps - so bulk remains the better clamp).
- Power-path architecture matches the ECO star-tap intent: J8.2 -> VBAT_Terminal -> R72 -> VBAT_CON {U19 IN, INA230 IN-/BUS, UVLO divider, CR2, C41/C42} -> U19 OUT -> VBATT {U21.IN2, J3.15/16, J4.2, FL1, FL2, R20, C7/C14/C15/C18/C54}; USB cannot backfeed the battery side; INA230 BUS reads pack-side so voltage telemetry survives an eFuse trip.
- INA230 straps: A0=A1=GND (0x40), ALERT floating (datasheet-permitted), VS+ on +3V3 with C76 1 uF local; I2C to U15 GPIO21/33 with pull-ups R67/R69.
- FL1/FL2 (GNSS/LoRa branch feeds) = Murata BLM18PG471SN1D, 1 A rated / 0.2 Ohm DCR - adequate for those branch loads (~0.1-0.65 A).
- J8 GND return: pin 1 lands in the multi-layer GND zone system (F/B/In1/In3/In4 + In1/In4 solid planes); thermal-relieved but with 4x0.5 mm spokes repeated across 5+ layers in parallel - adequate for 10 A return.
- Camera J4.2 feed verified connected: two in-pad vias (84.26/84.88, 115.9) from the In2 corridor into the 1x5.5 mm PH pad - adequate for RunCam current (a vestigial 0.127 mm stub inside the pad noted, harmless).
- ERC battery-domain items (18) are all noise: off-grid endpoint warnings, pin-type-classification errors on the documented IMON/ST grounding, and the deliberately-unconnected INA230 ALERT.


**Checks not completed (power-input):**

- JST eVH datasheet PDF text uses a shifted embedded font (extraction garbled), so the 10 A / AWG16 / 7 A-shrouded figures were confirmed via distributor listings (Digi-Key/Farnell copies) rather than a clean first-party quote; the 2-contact simultaneous-use derating curve was not readable - worth one manual look before relying on the full 10 A.
- Battery pigtail is off-board and appears in no BOM/repo file: could not verify the harness wire gauge (VH needs AWG16 for the 10 A rating; the crimp accepts down to AWG22, which would quietly cap the harness at ~3-5 A) or that the mating housing is VHR-2N with SVH-21T-P1.1 contacts.
- Molex Milli-Grid 878321620 per-circuit rating (~2 A) taken from the prior review's citation; the Molex spec PDF was not re-fetched tonight.
- Inner-layer copper weight is assumed 0.5 oz (JLCPCB 6-layer standard); the ordered stackup file/fab notes were not in the review package. If 1 oz inner was specified, the In2 corridor margins double (finding stands either way, but severity of the corridor item depends on this).
- Pack internal resistance and true 4-servo stall current are unmeasured; UVLO sag margin and corridor/seam heating were computed from the ECO's stated 12-14 A assumption. The H2 bench item (scope VBAT_CON during worst-case stall) remains open from the prior review.
- Thermal-pad solder joint quality (79% windowed paste on U19 pad 25 over 5 filled vias) is an assembly/AOI item that cannot be verified from design files.


## A.switched-power — Power conversion & switching

### HIGH [verified] — TPS62152 (U18) PVIN pin 12 still floating — prior blocker B-4/H1 NOT fixed
*3V3 buck / TPS62152 — confidence high*

The second power-stage input pin of the main 3V3 buck remains unconnected in both schematic and copper, so the entire buck input current (S3 + NAND + whole switched P4 domain, worst ~0.8-1 A) funnels through the single 0.26 mm pad 11, out of datasheet spec, and the input hot-loop is degraded. This was blocker B-4 (also H1/H10/H15/H17) in the 2026-07-30 review, was claimed among the fix commits, and is still present in tonight's live files.

**Evidence:** netlist.net: U18.12 -> 'unconnected-(U18-PVIN-Pad12)' while U18.10/11/13 -> Net-(U18-AVIN); rocket-computer.kicad_pcb U18 pad 12 at (89.530,127.800) F.Cu net='unconnected-(U18-PVIN-Pad12)' (line ~484), pad 11 at (89.030,127.800) on Net-(U18-AVIN); erc.json errors 'pin_not_connected / pin_not_driven: Symbol U18 Hidden pin 12 [PVIN, Input]'. TPS62150/52 datasheet (SLVSAL5E) Table 6-1: '11,12 PVIN — Supply voltage for power stage. Connect to same source as AVIN.' DRC reports 0 unconnected items only because the schematic no-connects the hidden pin.

**Fix:** Unhide/connect PVIN pin 12 to Net-(U18-AVIN) in the schematic and pour pad 12 into the adjacent AVIN F.Cu zone (pad 11 is 0.5 mm away, the zone already surrounds the pad).

**Sources:** TI TPS62150/1/2/3 datasheet SLVSAL5E (pin functions table, p.3), saved from ti.com/lit/ds/symlink/tps62152.pdf

**Verifier:** confirmed — Independently reproduced end-to-end. Netlist: U18.12 is 'unconnected-(U18-PVIN-Pad12)' while pins 10/11/13 sit on Net-(U18-AVIN). Live PCB: U18 pad 12 carries the unconnected net and that net name appears exactly once in the whole .kicad_pcb (the pad itself), so no copper reaches it; coordinate math from the footprint origin (88.78,126.365,-90) reproduces pad 11 at (89.030,127.800) and pad 12 at (89.530,127.800), both 0.86x0.26 mm. erc.json contains both pin_not_connected and pin_not_driven errors for 'Symbol U18 Hidden pin 12 [PVIN, Input, Line]' — the hidden-pin no-connect is why DRC shows no unconnected items. Datasheet (tps62152.txt line 214): '11,12 PVIN I Supply voltage for power stage. Connect to same source as AVIN.' Severity high (not re-escalated to the prior review's blocker) is right: the buck will still run with one PVIN pin at ~0.5 A input current, but it is out of datasheet spec with a degraded input hot loop — fix before fab.

**Disposition (2026-08-05): FIXED & VERIFIED in-session.** U18 pad 12 now carries `Net-(U18-AVIN)` (symbol pin 12 stacked on pin 11; AVIN pour refilled over the pad — no new tracks). Netlist, ERC (`pin_not_connected` 47→46, `pin_not_driven` 1→0) and DRC (0 unconnected, no new violations) all re-verified. See NB-3.

### HIGH [verified] — Prior H6 unmitigated: GNSS, LoRa and 12-line servo/EXP ports are still ground-side switched with live VBATT and NO series resistors on crossing signals (only camera got the 1k fix)
*peripheral load switches — confidence high*

All four peripheral ports remain low-side switched (PMPB14XNX N-FETs float the connector ground while the positive feed stays on always-live VBATT). The claimed '1k series R on cross-board UARTs' landed only on the camera port. GNSS (3 UART lines to P4), LoRa (2 UART lines to S3) and the servo/EXP port (12 lines to P4) cross directly with no series impedance: with the return floated, each off peripheral floats toward VBATT (6-8.4 V) and injects through signal pins into P4/S3 GPIOs — above absolute max, into the deliberately-unpowered P4 domain, phantom-powering the switched rail and defeating the S3-gates-P4 interlock.

**Evidence:** netlist.net: Q1 drains -> Net-(J1-Pad4) (GNSS gnd), Q10 -> Net-(J5-Pin_1) (LoRa gnd), Q8 -> Net-(J3-Pad1) (servo/EXP gnd, pins 1+2), Q7 -> Net-(J4-Pad1) (camera gnd); positive feeds always-on: FL1.1=VBATT->J1.3, FL2.1=VBATT->J5.2, J3.15/16=VBATT, J4.2=VBATT. Unprotected crossings: GNSS_RX {J1.1,U17.4}, GNSS_TX {J1.2,U17.3}, GNSS_RXD2 {J1.5,U17.2}; LoRa_RX {J5.3,U15.15}, LoRa_TX {J5.4,U15.16}; EXP_01..EXP_12 each exactly {J3.x, U17.x} — two-node nets, no resistor. Mitigated only: Camera_RX via R30 1k, Camera_TX via R32 1k. Gate drives verified default-off (1k series + 10k pulldown each: R6/R7, R24/R25, R27/R29, R34/R35).

**Fix:** Before fab: either revert these branches to high-side switching, or add >=1 k series resistors on all 17 remaining crossing lines (GNSS x3, LoRa x2, EXP x12) as was done for the camera port; at minimum hard-ground J1.4 and J5.1 since those daughterboards regulate locally from VBATT.

**Sources:** Live netlist + prefab-review-2026-07-30.md H6; Nexperia PMPB14XN datasheet (pinning verified: 3=G, 4/8=S, 1/2/5/6/7=D — matches Q1/Q7/Q8/Q10 wiring)

**Verifier:** confirmed — Every element reproduced from the live netlist. Q1/Q7/Q8/Q10 drains (pins 1,2,5,6,7) land on Net-(J1-Pad4)/Net-(J4-Pad1)/Net-(J3-Pad1)/Net-(J5-Pin_1) with sources (4,8) on GND — and I extracted the PMPB14XN datasheet pin table (1,2,5,6,7=D, 3=G, 4,8=S), confirming low-side topology. Positive feeds are always-live: FL1.1=VBATT->J1.3, FL2.1=VBATT->J5.2, J3.15/16=VBATT, J4.2=VBATT. All 17 claimed crossings verified as exactly-two-node nets with no series element: GNSS_RX {J1.1,U17.4}, GNSS_TX {J1.2,U17.3}, GNSS_RXD2 {J1.5,U17.2}, LoRa_RX {J5.3,U15.15}, LoRa_TX {J5.4,U15.16}, EXP_01..EXP_12 each {J3.x,U17.x}. Only the camera port is mitigated (Camera_RX via R30, Camera_TX via R32, both 1 k per BOM). Gate default-off also verified: R6/R24/R27/R34 = 1 k series, R7/R25/R29/R35 = 10 k to GND. Matches prior H6 exactly — the claimed cross-board series-R fix landed only on the camera port. High is the right severity: off-state peripherals float toward 6-8.4 V and inject through signal pins into P4/S3 GPIOs, including the deliberately unpowered P4 domain.

### HIGH [upgraded→blocker] — C12 10 mF can still lands on top of live power parts — now including the 3.1 mm-tall C56 hold-up cap and U13 flash (prior blocker B-2 not resolved by tonight's re-route)
*layout / hold-up cap C56 vs pyro can C12 — confidence high*

The bottom-side Ø18 mm C12 can (center 84.89,144.34, r=9 mm) still overlaps populated bottom-side parts after the re-route: C56 (Kemet TCJE 7.6x4.3x3.1 mm tall, body corner only 4.62 mm from can center — up to ~4.4 mm of body inside the rim), U13 SOIC-8 flash (1.75 mm tall, d=6.91 mm), plus Q3/Q4/Q6 pyro gate drivers (d=1.0-2.5 mm), C9/C10, R16/R17, and U21 at the rim (7.55 mm). A flush-seated can is physically impossible; a stood-off can rests above live power/pyro parts — the same crush/short/vibration risk B-2 flagged, and the footprint still has no courtyard so DRC stays blind.

**Evidence:** rocket-computer.kicad_pcb: C12 at (84.890,144.340) B.Cu; C56 pads (87.728,137.900)/(93.072,137.900) B.Cu -> body x[86.6..94.2] y[135.75..140.05], nearest corner distance to can center = sqrt(1.71^2+4.29^2) = 4.62 mm < 9 mm; U13 at (91.70,145.49) B.Cu d=6.91 mm; Q4 d=1.02, Q6 d=2.16, Q3 d=2.53 mm. drc.json: no courtyard violation at C12 (footprint has none).

**Fix:** Relocate C12 or clear a true Ø19-20 mm keep-out on B.Cu (C56 and U13 must move first), add a real courtyard to the C12 footprint and re-run DRC with courtyard checks; re-verify can height vs bay.

**Sources:** Live .kicad_pcb coordinates; Kemet TCJE337M016R0050 case E dimensions (7.3x4.3x3.1 mm, content.kemet.com); prefab-review-2026-07-30.md B-2

**Verifier:** upgraded — All geometry reproduced from the live PCB and the defect is real — but this is the identical defect class the prior review rated blocker B-2, and tonight's state is worse, so high understates it. C12 at (84.89,144.34) on B.Cu with a B.Fab body circle of r=8.99 mm (Ø18 can confirmed); its footprint contains zero courtyard graphics and drc.json has zero C12 violations, so DRC remains structurally blind. Under the rim on the same side: C56 body corner at 4.62 mm from can center (my math: pads at (87.728,137.900)/(93.072,137.900), body x[86.6..94.2] y[135.75..140.05], sqrt(1.71^2+4.29^2)=4.62 — i.e. 4.38 mm of a 3.1 mm-tall polymer-tant inside the rim), U13 SOIC-8 at 6.91 mm, Q4 at 1.02 mm, Q6 at 2.16 mm, Q3 at 2.53 mm, R17 at 2.46 mm, R16 at 3.22 mm, C10 at 4.29 mm, U21 at 7.55 mm — all on B.Cu; the bottom render visually confirms the can body sitting over C56 and the pyro gate drivers. A flush seat is physically impossible (3.1 mm part under a radial can), and the only assembly alternative is a >3.1 mm stand-off resting live pyro gate-drive and hold-up parts under a 10 mF can in a high-g vehicle — that meets the rubric's 'cannot assemble / unsafe, do not fab' bar, exactly as B-2 concluded for a less severe version of the same overlap.

**Disposition (2026-08-05, owner): placement WAIVED.** The can intentionally stands off above the parts on that side of the board — a board-area vs vertical-space/serviceability tradeoff — including the taller C56 tantalum beneath it. Residual actions (polarity silk, formed leads + staking, true Ø18×25 STEP, bay-height check) remain — see NB-2 in the executive summary.

### MEDIUM [verified] — 3V3 buck's only input capacitor (C43) is on the opposite side of the board from U18, reached through a single 0.3 mm-drill via
*3V3 buck layout / TPS62152 — confidence high*

Tonight's re-route moved U18 to F.Cu but left C43 (the only capacitor on the AVIN/PVIN node) on B.Cu: the high-frequency input loop of a 1.25/2.5 MHz buck now traverses one 0.4/0.3 via plus a full 1.6 mm board crossing and returns via GND stitching, directly against TI's layout requirement. Consequence is SW-node ringing/overshoot and radiated EMI on a board with GNSS and 2.4 GHz radio, and all input HF ripple current concentrates in a single small via.

**Evidence:** rocket-computer.kicad_pcb: U18 F.Cu, PVIN pad 11 (89.030,127.800); C43 pads (90.810,127.380)/(92.710,127.380) on B.Cu; Net-(U18-AVIN) contains 0 track segments and exactly 1 via at (90.38,127.13) size 0.4/drill 0.3 landing on the C43 pad-1 edge; AVIN F.Cu zone x[88.3..91.0] y[127.0..129.7]. Netlist: C43 is the only capacitor on Net-(U18-AVIN). TPS62152 datasheet SLVSAL5E section 11.1: 'The input and output capacitance should be placed as close as possible to the IC pins' and 'Loops which conduct an alternating current should outline an area as small as possible.' Prior review noted the old same-side placement was acceptable; this is a new regression from tonight's re-route.

**Fix:** Move C43 (or add a 10 uF ceramic) on F.Cu directly across PVIN pins 11/12 and PGND 15/16; keep C43 on B as bulk if desired. Trivially combines with the pad-12 fix.

**Sources:** TI SLVSAL5E layout guidelines (p.28, quoted verbatim)

**Verifier:** confirmed — Fully reproduced. U18 is on F.Cu (88.78,126.365); C43 is an 0805 on B.Cu with pad 1 (Net-(U18-AVIN)) at (90.810,127.380) and pad 2 GND at (92.710,127.380). Parsing the live PCB: Net-(U18-AVIN) contains 0 track segments and exactly 1 via, at (90.38,127.13), size 0.4/drill 0.3, F.Cu-B.Cu, landing on the C43 pad-1 edge; the only other AVIN copper is one F.Cu zone bbox x[88.3..91.0] y[127.0..129.7]. Netlist confirms C43 is the only capacitor on the AVIN/PVIN node — and L5 (input filter inductor, V_MCU_2S->AVIN) sits in series upstream, so upstream bulk (C56/C59) cannot serve the HF loop: all pulsed input current must cross the board through that single via. Both datasheet quotes verified verbatim in tps62152.txt (lines 1785-1787: 'The input and output capacitance should be placed as close as possible to the IC', 'Loops which conduct an alternating current should outline an area as small as possible'; line 859: input cap 'between PVIN and PGND as close as possible to those pins'). Medium is right — the buck will regulate, but ringing/EMI risk on a GNSS+2.4 GHz board plus all input ripple current in one 0.3 mm via warrants a fix. Minor nit: TPS62152 switches at 2.5 MHz typ (1.25 MHz is a sibling-variant/mode detail); immaterial.

### MEDIUM [verified] — BOM export contradicts the DNP chip-rev provision: DNP'd C93 will be populated (counted with MPN) while DNP'd R74/R75/R76 ship as MPN-less lines
*P4 core buck / fab package — confidence high*

The v3.x provision (#657) marks R74/R75/C93/R76 DNP for the current v1.3 dev chips, but bom.csv includes C93 inside the populated 22 pF line (qty 5, valid Samsung MPN) — an assembler will fit it on a v1.3 build — while R74/R75 and R76 appear as orderable-looking lines with empty MPN/Mfr, guaranteeing assembler RFQs or wrong fitting. The BOM as exported matches neither the v1.3 build (C93 must be absent) nor the v3.x build (all four present with MPNs).

**Evidence:** central_processing_p4.kicad_sch: R74/R75/C93/R76 all '(dnp yes)(in_bom yes)'; PCB attrs 'smd dnp'; schematic note: 'P4 chip-rev provisions (#657): R74/R75/C93 = DCDC FB network, R76 = pin-54 VDD_HP_1 link. FIT all four for chip rev v3.x — DNP for v1.x (current dev chips = v1.3).' bom.csv line: "C19,C21,C37,C38,C93","22 pF",...,"CL05C220JB5NNNC","Samsung","","5" and "R74,R75","499 k",...,"","","2" / "R76","0",...,"","","1".

**Fix:** Re-export the assembly BOM excluding DNP parts (KiCad 'exclude DNP'), split C93 out of the 22 pF group, and keep a second v3.x BOM variant with MPNs for all four; document which chip revision each fab lot is built for.

**Sources:** Live schematic/PCB DNP attributes vs scratch bom.csv (exported tonight)

**Verifier:** confirmed — Core defect reproduces with one contextual correction. Live central_processing_p4.kicad_sch: R74/R75 (499 k), C93 (22 pF), R76 (0) all carry (dnp yes)(in_bom yes); live PCB attrs are 'smd dnp' for all four; the #657 note text matches verbatim ('FIT all four for chip rev v3.x -- DNP for v1.x (current dev chips = v1.3)'). Tonight's BOM export has no DNP column, merges C93 into the populated Samsung 22 pF line ('C19,C21,C37,C38,C93',qty 5, CL05C220JB5NNNC) and lists 'R74,R75' / 'R76' with empty MPN/Mfr — exactly as claimed, so any assembler working from a straight export fits C93 on a v1.3 build and RFQs the MPN-less lines. Correction to the framing: the quoted bom.csv is the review's fresh scratch export, while the repo's checked-in hardware/rocket-computer/bom.csv is a stale July semicolon-format file that contains none of C93/R74/R75/R76 — meaning no correct assembly BOM currently exists for either chip revision and the regeneration path has this trap, which reinforces rather than weakens the finding. Medium (process/fab-package fix) is right.

### MEDIUM [verified] — TPS2121 ILIM strap deliberately set to the device maximum (5.2 A typ) — no useful protection for a 1 A load, and above the VBUS copper's capacity
*power mux / TPS2121 — confidence high*

R64 = 18.7 kOhm is the bottom of the legal RILM range and programs 4.6-5.8 A output current limit, while the only load on V_MCU_2S is the 1 A TPS62152 branch. A downstream fault while on USB would pull up to ~5 A through the 0.4 mm-wide, ~50 mm VBUS trace (roughly 4x its ~1.2 A/10 C capability) and through a possibly non-compliant 5 V source before the mux ever limits; the schematic annotates 'Max current limit of 5.2 A' so it is a conscious choice, but it defeats the feature.

**Evidence:** Netlist: Net-(U21-ILIM) = {R64.2, U21.10}, R64 = 18.7 k (BOM). TPS2120/21 datasheet SLVSEA3F: ILM equation I=65.2/R^0.861 and EC table row 'RILM = 18.7 kOhm: 4.6/5.2/5.8 A'; RILM legal range 18-100 k. PCB: Net-(J6-VBUS) = 37 segments, 49.9 mm total, widths 0.1/0.4 mm. Load audit: V_MCU_2S feeds only C56/C59/L5->U18 (netlist).

**Fix:** Unless there is a documented reason, raise R64 to ~60.4-80 k (2.0-1.5 A limit) to actually protect the VBUS copper and the USB source; alternatively widen the VBUS path to match 5 A and waive.

**Sources:** TI SLVSEA3F pp.8,13 (ILM table + equation 2)

**Verifier:** confirmed — Fully reproduced. Netlist: Net-(U21-ILIM) = {R64.2, U21.10}, R64.1 to GND; BOM: R64 = 18.7 k (Yageo RC0402FR-0718K7L). Datasheet (tps2121.txt): EC table row 'RILM = 18.7kOhm ... 4.6 / 5.2 / 5.8 A', RILM legal range '18 100 kOhm', and Equation 9 confirms ILM = 65.2/RILM^0.861 — so 18.7 k is the bottom of the legal range, i.e. maximum current limit. Schematic annotation 'Max current limit of\n5.2 A' found at power.kicad_sch:5242 (conscious choice, as claimed). Load audit reproduced: V_MCU_2S = {U21.1/8, C56, C59, L5}, and L5 feeds only the U18 TPS62152 (a 1 A buck) — nothing else. Live PCB measurement: Net-(J6-VBUS) = 37 segments, 49.9 mm total, 35 segments at 0.4 mm plus two 0.1 mm stubs (0.4 mm total length) — matching the claim; ~0.4 mm 1 oz copper is good for roughly 1.2-1.5 A at 10 C rise, far below the 4.6-5.8 A limit band. (One small VBUS zone also exists near the connector; it does not change the 50 mm 0.4 mm bottleneck.) Medium is appropriate: fault-mode copper/source overstress, feature defeated, but no failure in normal operation.

### MEDIUM [verified] — C56 hold-up delivers ~3-6 ms at realistic flight load, not the ECO's 52 ms, and its specified 10 uF ceramic companion shrank to 1 uF
*hold-up / brownout architecture — confidence high*

The brownout story (power-eco.md Change 1) sized C56 = 330 uF assuming a 50 mA MCU-only load (0.17 W -> '~52 ms'). The as-built +3V3 rail carries S3 radio bursts plus the whole switched P4 domain (~0.4-0.9 A, 1.3-3 W input power), so the reverse-blocked hold-up carries the logic rails only ~2.9-6.4 ms from 8.4 V (2.0-4.7 ms from 7.4 V) down to the buck's ~3.5 V dropout — an order of magnitude below the ECO figure and far below the eFuse UVLO recovery gap flagged by the prior review (H2, still open: Net-(U19-EN/UVLO) has only R44/R45, no deglitch C). Additionally the ECO's '330 uF || 10 uF X7R' pairing is implemented as C59 = 1 uF.

**Evidence:** Netlist: V_MCU_2S = {U21.OUT, C56 330 uF, C59 1 uF, L5->U18}. Energy math: 0.5*330u*(8.4^2-3.5^2)=9.6 mJ; at 1.5-3.3 W buck input -> 2.9-6.4 ms. power-eco.md line 27-28: 'At the actual ~50 mA MCU load (~0.17 W)... ~52 ms hold-up... If MCU load ever climbs well above ~200 mA at low battery, add a 2nd 330 uF'. TPS2121 RCB verified always-on (SLVSEA3F 9.3.6), so the mechanism itself is sound.

**Fix:** Follow the ECO's own escape hatch: add a second 330 uF on V_MCU_2S (or accept the ~3-6 ms figure explicitly against the eFuse-deglitch decision), and bump C59 to the specified 10 uF.

**Sources:** power-eco.md Change 1; TI SLVSEA3F 9.3.6; live netlist/BOM

**Verifier:** confirmed — Math and every supporting fact reproduce. Netlist: V_MCU_2S = {U21.1, U21.8, C56, C59, L5.1}; C56 = TCJE337M016R0050 330 uF (BOM) and C59 sits in the 1 uF BOM group — the ECO's '330 uF || 10 uF X7R' pairing is indeed implemented as 1 uF, with no other cap on the node. Load reality verified from the netlist: +3V3 (U18 output) powers U15 (S3), U13, U11, U23 AND U22 = TPS22918DBVR whose output V_MCU_SWTCH carries the entire P4 domain (U17, U20/TLV62569 core buck, U2/U3/U4, sensors) — so the ECO's ~50 mA/0.17 W assumption is off by roughly an order of magnitude at realistic flight load (its own domain table even lists switched compute as 'protected by the same hold-up'). Energy math checks: 0.5*330e-6*(8.4^2-3.5^2) = 9.62 mJ -> 2.9-6.4 ms at 1.5-3.3 W buck input (2.0-4.7 ms from 7.4 V). power-eco.md quotes verified verbatim ('~52 ms hold-up', 'If MCU load ever climbs well above ~200 mA at low battery, add a 2nd 330 uF'). Prior H2 still open as claimed: Net-(U19-EN/UVLO) = {R44 1M->VBAT_CON, R45 210k->GND, U19.6} with no deglitch capacitor. TPS2121 always-on RCB confirmed (tps2121.txt 9.3.6 'Each channel has the always on reverse current blocking'), so the mechanism is sound but the margin claim in the ECO is wrong by ~10x. Medium (fix or consciously waive) is right.

### LOW [unverified] — No bypass capacitance at all on TPS2121 IN1 (USB VBUS node)
*power mux / USB input — confidence medium*

Net-(J6-VBUS) contains only the connector pins, the SP0503 TVS, the OV1/PR1 dividers and U21.7 — zero bulk or ceramic capacitance. USB hot-plug inductive ringing lands on the mux input with only the TVS to catch it, and the mux's IN1 node has no local charge reservoir during RCB/switchover events. IN2 (VBATT) by contrast has ~67 uF within millimeters.

**Evidence:** Netlist NET Net-(J6-VBUS): {CR3.2, J6.A4/B9, J6.B4/A9, R51.2, R62.2, U21.7} — no capacitor. VBATT side: C7/C14/C18 22 uF, C15 330 uF, C54 1 uF. TPS2121 ROC table footnote directs to Power Supply Recommendations for input treatment.

**Fix:** Add a 1 uF/25 V X7R (plus optional 100 nF) at U21 pin 7 / near J6 VBUS.

**Sources:** TI SLVSEA3F; live netlist

### LOW [unverified] — Duplicate stacked vias on Net-(U21-OV1) and a hole-to-hole clearance violation on +3V3 vias
*power layout hygiene — confidence high*

DRC reports two exactly co-located vias on the U21 OV1 divider net (redundant stacked drill, will be flagged/merged by the fab) and one +3V3 via pair closer than the 0.1995 mm hole-to-hole constraint (drill-breakout risk).

**Evidence:** drc.json: 'holes_co_located: Via [Net-(U21-OV1)]' x2 items; 'hole_to_hole: min 0.1995 mm' on two +3V3 vias.

**Fix:** Delete the duplicate OV1 via; nudge one +3V3 via to meet the hole-to-hole constraint.

**Sources:** drc.json exported tonight from the live board

### INFO [unverified] — Unmonitored PG outputs and stale power-sheet annotation
*power conversion misc — confidence high*

U18 PG and U19 PG are each pulled up (R49, R59 100 k to +3V3) but route to no MCU pin — dead provisions; the U22 note 'Switching on VPP enables Flight Computer, memory, radio, and GNSS' is stale (memory moved to +3V3 per ECO fix 1; GNSS is powered from VBATT via FL1, only its ground switch is P4-controlled).

**Evidence:** Netlist: Net-(U18-PG) = {U18.4, R49.1}, PG_RAIL = {U19.13, R59.2} — no other nodes. power.kicad_sch note text on sheet 5; netlist FL1.1=VBATT->J1.3; esp32s3_outputs memory on +3V3.

**Fix:** Optionally wire a PG to a GPIO/ADC or delete the pull-ups; update the schematic note.

**Sources:** Live netlist + schematic.pdf page 5


**Affirmatively verified clean (switched-power):**

- TPS2121 (U21) pin-by-pin vs SLVSEA3F RUX pinout: all 12 pins correct (1/8 OUT->V_MCU_2S, 2 IN2->VBATT, 7 IN1->VBUS, 12 GND); ST grounded exactly per datasheet 'Connect to GND if not required'
- TPS2121 strap design is TI's XCOMP automatic-priority application, and the numbers are sane: PR1=VBUS/2 (R62/R63 5.11k/5.11k), CP2=VBATT/4.01 (R60 15.4k/R61 5.11k) -> fast 5 us switchover to battery when VBUS sags below ~VBATT/2; OV1 trips at 6.35 V (R51 49.9k/R52 10k, matches '6 V' annotation, blocks 9 V PD chargers); OV2 trips at 9.54 V (R56 12k/R57 1.5k, blocks a 3S pack); RILM 18.7k is inside the legal 18-100k window; reverse-current blocking (basis of the C56 hold-up) is always-on per 9.3.6
- TPS2121 SS: C52 1 uF -> ~88 V/s slew (Table 9-1) = gentle ~95 ms first-start ramp, ~29 mA inrush into C56 330 uF; startup into the current-limited buck converges — no lockup mechanism
- TPS62152 (U18) strap set verified against SLVSAL5E: FSW tied to VOUT is explicitly endorsed ('Connect FSW to VOUT or PG in this case', selects 1.25 MHz); DEF=GND -> nominal 3.3 V; FB=GND is the documented fixed-output connection; EN tied to AVIN; VOS to +3V3; AGND/PGND/EP grounded; SS 390 pF (~125 us, current-limited into ~90 uF rail, harmless); Vin range 3-17 V covers 2S and USB
- L6 = TDK VLS3012CX-2R2M-1 2.2 uH: worst-case peak inductor current ~1.37 A (1 A + half of 0.73 A ripple at 8.4 Vin/1.25 MHz) is below the 1.89 A Isat / 2.83 A Irms rating; same part on L8 sees only ~0.9 A peak at P4 core load
- +3V3 rail budget: worst coincident load ~0.85 A (S3 BLE TX + P4 domain via U22 + flashes + sensors + buzzer) fits the 1 A converter (ILIMF min 1.4 A) — thin but workable margin, camera/GNSS/LoRa/servo loads confirmed on VBATT not 3V3
- TLV62569 (U20) symbol/pinout verified against the real SLVSDG1C DRL package drawing (1 FB, 2 GND, 3 VIN, 4 SW, 5 EN, 6 NC/PG): all six pins land correctly, and grounding pin 6 (NC on the non-P part) is explicitly allowed ('can be connected to the output or the ground')
- P4 core-buck FB network topology exactly matches Espressif ESP32-P4 HDG Fig. 5 (v3.0+): R74 499k ESP_VDD_HP->FB node, R75 499k FB->GND, C93 22 pF across R74, U17.78 FB_DCDC direct to the FB node, U17.79 EN_DCDC direct to EN; default divider output 1.2 V sits inside the specified 0.99-1.3 V VDD_HP window; TLV62569 is on Espressif's verified-DCDC list; prior blocker B-1 is structurally fixed (as a DNP provision — see BOM finding); R77 1 M CEN_D+ pulldown fitted (v1.x requirement, v3.x-allowed); R76 0R pin-54 VDD_HP_1 link provisioned
- U20 layout: hot loop compact and single-sided (C47 10 uF input cap 1.7 mm from VIN on F.Cu, L8 3.0 mm, C55/C92 output caps adjacent, ~30 uF on ESP_VDD_HP); FB net is 5.6 mm total, 0 vias, F.Cu only, routed away from SW — per Espressif 'input, output, and feedback loops as short as possible'
- TPS22918 (U22) verified pin-by-pin against SLVSD76C (1 VIN, 2 GND, 3 ON, 4 CT, 5 QOD, 6 VOUT — an earlier auto-summary claiming a different pinout was wrong, the silicon datasheet matches the design): QOD tied to VOUT is documented option 2 (internal 25 ohm discharge); ON has R68 100k pulldown so the P4 domain is deterministically OFF at boot until S3 GPIO7 asserts through R71 100R; CT 2.2 nF gives a ms-class ramp -> ~0.1 A inrush into the ~81 uF V_MCU_SWTCH bank; 3.3 V/~0.4 A load well inside 5.5 V/2 A ratings
- Peripheral ground-switch FETs Q1/Q7/Q8/Q10 (and buzzer Q9): footprint pad mapping verified against Nexperia PMPB14XN DFN2020MD-6 pinning table (1,2,5,6,7=D; 3=G; 4,8=S) — no gate/source swap; VGS(th) max 0.65 V so 3.3 V GPIO drive is solid; 8.1 A / 15-18 mOhm rating comfortably covers servo return currents; every gate has 1k series + 10k pulldown -> all default OFF with MCUs unpowered; drive mapping confirmed (GPS_ACT=P4 GPIO15->J1, CAM_ACT=P4 GPIO32->J4, SERVO_ACT=P4 GPIO8->J3, LoRa_ACT=S3 GPIO12->J5)
- Camera port (J4) crossing signals are the one implemented H6 mitigation: Camera_RX/TX each pass through 1k (R30/R32)
- USB VBUS feeds only the mux (no parallel loads), CC pulldowns/TVS present; V_MCU_2S F<->B transition uses 6 parallel 0.3 mm vias — adequate for the ~0.8 A buck input current
- C56 derating healthy: 330 uF 16 V polymer at 8.4 V = 53% of rating; C43 22 uF is the 16 V CL21A226MOQNNNE (not the 6.3 V variant) so ~8-11 uF effective at 8.4 V bias, meeting the >=10 uF class input requirement together with C56 upstream
- kicad DRC on the live board: zero unconnected items, zero clearance/short violations on the power nets (remaining hits are silk noise plus the two via items reported above)


**Checks not completed (switched-power):**

- TPS2121 'Power Supply Recommendations' section text (specific input-cap value recommendation) not extracted verbatim — the IN1 no-cap finding cites the ROC footnote and standard practice instead
- In2 plane fill arbitration between the overlapping +3V3 / VBATT / V_MCU_SWTCH zones was not re-verified by refilling; trusted the stored fills plus the clean DRC (a zone-priority audit before fab is still prudent)
- PMPB14XNX-specific datasheet page 404'd at Nexperia; verified against the PMPB14XN base datasheet (X suffix is a packing variant of the same die/package)
- Samsung capacitor DC-bias derating taken from typical X5R curves, not the exact Samsung characteristic PDFs
- No thermal verification of U18 at full load in the enclosed avionics bay (RthJA 45 C/W suggests ~14 C rise at 1 A — likely fine, not modeled)
- Could not bench-verify the TPS2121 fast-switchover dip with the real C56 ESR (ECO bench item remains open); analysis-only via SLVSEA3F equations 3-4


## A.pyro — Pyrotechnic channels

### BLOCKER [verified] — C12 10 mF can still sits on ~19 populated bottom-side parts (now incl. a 3.1 mm tantalum and a SOIC-8) with no courtyard - blocker B-2/B3/H14 NOT fixed by tonight's re-route
*pyro energy store / assembly — confidence high*

The Ø18x25 mm radial can C12 is at exactly the prior coordinates (84.89,144.34, B.Cu) and tonight's re-route moved MORE parts under it, not fewer: 19 bottom-side components now have centers inside the Ø18 body circle, including C56 (Kemet TCJE337M016R0050 E-case tantalum, 3.1 mm tall - the V_MCU_2S hold-up cap), U13 (W25Q128 SOIC-8, ~1.75 mm), the gate drivers Q3/Q4/Q6 directly under the can center, fire FETs U7/U8/U10, U21 (TPS2121), and charge resistor R20. The can cannot sit flush; it would stand off >3.3 mm on two 0.8 mm leads above live pyro gate-drive and power-path parts in a high-g vehicle. The footprint still has NO courtyard and DRC still ignores missing_courtyard, so DRC remains structurally blind. The repo STEP model is still Ø16 (2 mm undersized), so renders/3D checks understate the collision; with the true Ø18 body, a screw head or standoff in mounting hole H8 (center 11.19 mm from can center) clears the can edge by only ~0.2 mm. The power-eco.md open item 'verify can height vs bay clearance' is still unresolved anywhere in the repo.

**Evidence:** rocket-computer.kicad_pcb: C12 (at 84.89 144.339999 -90, layer B.Cu), fp graphics only on B.Cu/B.Fab/B.SilkS (no B.CrtYd), B.Fab circle r=9.00. Neighbor scan (script, this session): B.Cu-side centers inside r=9: Q4 d=1.02, Q6 2.16, R17 2.46, Q3 2.53, R16 3.22, R23 3.27, C10 4.29, R64 5.55, C9 5.63, U13 6.91, R62 7.51, U21 7.55, C52 7.94, R20 8.05, C56 8.48, R63 8.48, U7 8.39, U8 8.86, U10 8.91 (plus body-overlap: U6 9.45, C41 9.15, C54 9.45). bom.csv line 7: C56 = TCJE337M016R0050, footprint CAPMP7.6X4.3_3.1N_KEM (3.1 mm). kicad_pro: missing_courtyard=ignore. STEP parse: hardware/3dmodels/EKYC160ELL103MM25S.STEP X-extent 16.0 mm vs Chemi-Con 18 mm (prior H14 fetch). H8 at (91.97,153.0), d=11.19 from can center. power-eco.md:119 open verify note; no bay-height note in any repo doc.

**Fix:** Relocate the can (or clear the parts under it - at minimum C56, U13, Q3/Q4/Q6 and the fire FETs), add a real Ø19-20 courtyard to the footprint, re-enable missing_courtyard in DRC, replace the STEP with the true Ø18x25 model, specify nylon or low-profile hardware at H8, and write down the bay height budget (board 1.6 + standoff + 25 mm can). If a spacer-mounted can over parts is genuinely intended, document standoff height and staking (RTV) for flight vibration.

**Sources:** rocket-computer.kicad_pcb (live); bom.csv (tonight's export); hardware/3dmodels/EKYC160ELL103MM25S.STEP; prefab-review-2026-07-30.md B-2/B3/H13/H14; power-eco.md line 119

**Verifier:** confirmed — Core defect fully reproduces and blocker severity is right, with one sub-claim corrected. Independently re-parsed the live kicad_pcb: C12 at (84.89,144.339999,-90) on B.Cu, unchanged from prior B3; footprint graphics are only a B.SilkS circle (r=1.5) and B.Fab circle (r=9.00) - no B.CrtYd; kicad_pro has missing_courtyard:"ignore" (drc.json shows zero missing-courtyard items, 1 unrelated courtyards_overlap). My own neighbor scan reproduced the finder's list exactly: 19 B.Cu component centers inside r=9 (Q4 1.02, Q6 2.16, R17 2.46, Q3 2.53, R16 3.22, R23 3.27, C10 4.29, R64 5.55, C9 5.63, U13 6.91, R62 7.51, U21 7.55, C52 7.94, R20 8.05, U7 8.39, C56 8.48, R63 8.48, U8 8.86, U10 8.91) plus C41/C54/U6 at 9.15-9.45. C56 verified as TCJE337M016R0050 (bom.csv line 7, footprint CAPMP7.6X4.3_3.1N_KEM) on the V_MCU_2S net (TPS2121 U21.OUT hold-up) and U13 as W25Q128JVSIQ SOIC-8 (footprint descr in pcb). Prior B3 listed ~17 parts; now 19 - the re-route made it worse, not better. H8 at (91.97,153.0), d=11.19 from can center reproduces; a DIN965 M2 head (3.8 mm dia) leaves ~0.3 mm, a 4-4.5 mm standoff ~0.2 mm to interference - the ~0.2 mm claim is fair. power-eco.md:119 bay-clearance verify item confirmed still open, no bay-height budget anywhere in repo docs. CORRECTION: the sub-claim that the repo STEP is Ø16 is wrong - the STEP's product name is 'E-LEAD_D18_L25' and it contains CYLINDRICAL_SURFACE entries of radius 9.0 mm, i.e. a true Ø18x25 model; the '16.0 mm X-extent' (inherited from prior H14) is a parsing artifact because STEP cylinders are axis+radius and the raw CARTESIAN_POINT cloud does not span the diameter (X pts -10.34..+5.69). So renders/3D checks do NOT understate the can; drop the 'replace the STEP' fix item. This does not touch the core defect: the can cannot sit flush on 19 populated parts including a ~3.1 mm tantalum and a 1.75 mm SOIC-8, so the board is unbuildable as drawn.

**Disposition (2026-08-05, owner): placement WAIVED.** The can intentionally stands off above the parts on that side of the board — a board-area vs vertical-space/serviceability tradeoff — including the taller C56 tantalum beneath it. Residual actions (polarity silk, formed leads + staking, true Ø18×25 STEP, bay-height check) remain — see NB-2 in the executive summary.

### HIGH [verified] — A single enabled P4 internal pull-up still fires a channel: gate-drive divider unchanged (prior high finding not fixed)
*pyro gate drive / fault tolerance — confidence high*

Recomputed with the LIVE resistor values - nothing changed since the prior review. If firmware enables the ESP32-P4 internal weak pull-up (RPU 45 kOhm typ) on any fire GPIO (6/9/11/13), the DTC123J base gets I_B = (3.3-0.7)/(45k+2.2k) - 0.7/47k = 40 uA, giving I_C >= 3.2 mA (GI min 80) against the 0.84 mA needed to pull the 10 k gate pull-up (R15/R16/R17/R23) low - the TPN4R712MD turns fully ON. Simultaneously a WPU on GPIO16 puts 3.3*100k/(145.1k) = 2.27 V on the CSD16323Q3 arm gate, above its VGS(th) max 1.4 V - armed. So one blanket gpio_reset_pin/gpio_config over the pyro bank (the exact firmware class already observed on this project) fires a charged channel. V_CAP is always charged when a battery is attached (R20 has no switch) and reaches ~2.6-2.8 V even on USB. The hardware was supposed to be made immune; it was not.

**Evidence:** netlist.net (tonight): PYROx_FIRE = {U17 GPIO6/11/9/13, Qx.B} only - no external base pulldown added; Net-(Qx-C) = {Qx.C, R15/16/17/23 (10 k), Ux.GATE}; PYRO_ARM = {U17 GPIO16, R21 100}; Net-(U9-GATE) = {R21, R22 100k, U9.GATE}. ROHM DTC123J datasheet 20150311-Rev.002 p.1/p.2: R1=2.2k, R2=47k, V_I(off) max 0.5 V, GI min 80. TI SLPS224C p.3: VGS(th) 0.9-1.4 V. ESP32-P4 datasheet p.90: RPU 45 kOhm. Espressif Table 2-1 (p.14-15): GPIO6-14, GPIO16 no pulls at reset (boot itself is safe - this is a firmware-fault hazard).

**Fix:** Make the hardware immune: add ~4.7-10 k external base-emitter pulldowns on each DTC123J input (or replace DTC123J with a variant having lower R2/R1 ratio), and resize the arm divider (e.g. R22 = 10 k) so a 45-70 k WPU cannot exceed 0.5 V at the DTC input / 1.4 V at the arm gate. Re-verify: with R22=10k, WPU worst 45k gives 0.6 V at arm gate (< 0.9 V VGS(th) min).

**Sources:** netlist.net; ROHM DTC123J datasheet 20150311-Rev.002; TI CSD16323Q3 SLPS224C; ESP32-P4 datasheet (documentation.espressif.com/esp32-p4_datasheet_en.pdf) p.14-15, p.90; prefab-review-2026-07-30.md line 332

**Verifier:** confirmed — Every netlist fact and every datasheet number independently reproduces, and the math is robust. Netlist (tonight's export): PYRO1/2/3/4_FIRE = {U17 GPIO6/11/9/13, Q5/Q3/Q4/Q6 base} - exactly 2 nodes each, so no external base pulldown was added; Net-(Qx-C) = {Qx.C, R15/R16/R17/R23 (verified 10 k), U8/U6/U7/U10 GATE}; PYRO_ARM = {U17 GPIO16, R21 (100)}; Net-(U9-GATE) = {R21.1, R22.1 (100 k, pin 2 GND), U9.GATE}; V_CAP is hardwired to VBATT through R20 (150 R, no switch). Datasheets fetched fresh: ROHM DTC123J (current 20240913-Rev.006, same values as the finder's Rev.002 citation): R1 2.2 k, R2 47 k, V_I(off) max 0.5 V, GI min 80. Math reproduces exactly: I_B = (3.3-0.7)/(45k+2.2k) - 0.7/47k = 40.2 uA, I_C >= 3.2 mA vs 0.84 mA (8.4 V/10 k) needed - fire FET fully on; even worst-case internal spread (R1 2.86k, R2/R1 17) still gives ~2.2 mA. Arm: 3.3x100k/145.1k = 2.274 V vs CSD16323Q3 VGS(th) 0.9-1.4 V (verified TI SLPS224C p.3, fetched) - above max, armed. ESP32-P4 RPU 45 kOhm typ verified in local p4-ds.txt (Table 5-4) and GPIO6/9/11/13/16 confirmed with no default pulls (pin table dashes) - boot is safe, the hazard is a blanket firmware gpio_config, exactly as claimed. TPN4R712MD confirmed P-channel (Toshiba Rev.6.0.A p.1). Minor nit only: on USB-only power V_CAP sits ~1.2 V (see finding on M33), not 2.6-2.8 V, but with a battery attached V_CAP is at full pack voltage so the one-WPU-fires-a-charged-channel scenario stands. High severity (fix before fab) is right for a single-fault pyro fire path.

### MEDIUM [verified] — Arm interlock still low-side and J2.5 still silk-labeled 'GND' - the label invites the exact wiring fault that bypasses the arm FET (M9+M13 unfixed)
*pyro arm interlock / silkscreen — confidence high*

Live topology is unchanged: V_CAP -> fire P-FET -> PYROx_EXT -> match -> PYRO_GND -> low-side arm N-FET U9 -> GND (R73 1 k permanent bleed). J2 position 5 is still silk-labeled 'GND' although it is the switched arm return; any harness short from the shared match-return to real ground (airframe, commoned battery negative, a bench probe strap) removes U9 from the circuit, leaving single-FET firing. The prior review asked for a conscious high-side-vs-low-side decision and at minimum an honest silk label; neither happened.

**Evidence:** netlist.net: PYRO_GND = {J2.5_1, J2.5_2, R73.1, U9.5-9}; U9.1-3 = GND. rocket-computer.kicad_pcb B.SilkS gr_text 'GND' at (90.97,156.47) beside J2 position 5 (pads at x=89.55); silk row '1 2 3 4 GND' confirmed in render_bottom.png. Boot-glitch safety of the arm gate itself is fine (R22 100k pulldown).

**Fix:** Before fab at minimum relabel J2.5 'RET' or 'PYRO-' and add a build-doc note that match returns must never be commoned with airframe/battery ground. Preferably reconsider high-side arm (same part count: move U9 between V_CAP and the fire-FET sources) which no ground fault can bypass.

**Sources:** netlist.net; rocket-computer.kicad_pcb; prefab-review-2026-07-30.md M9 (line 452) and M13 (line 496)

**Verifier:** confirmed — All facts reproduce from the live files. Netlist: PYRO_GND = {J2.5_1, J2.5_2, R73.1 (1 k, pin 2 GND), U9 drains 5-9}; U9 (CSD16323Q3) sources 1-3 = GND - the arm FET is low-side in the match-return path exactly as claimed, and any external short from the shared match return to real ground bypasses it, leaving single-FET firing (the FIRE P-FETs are the only remaining gate). Silk: board-level gr_text 'GND' at (90.97,156.47) on B.SilkS, sitting in the row '1 2 3 4 GND' (my scan: '1' at 76.13, '2' 79.41, '3' 83.03, '4' 86.45, 'GND' 90.97) directly over J2's position-5 pad column - J2 is a CUI TBLH11-350-05 terminal block at (82.55,164.22) with pads 5_1/5_2 at x=89.55 both on PYRO_GND (verified by pad rotation math). Labeling the switched arm return 'GND' on a screw terminal is a genuine invitation to common it with battery/airframe ground. Boot-glitch note (R22 100k gate pulldown) also verified. Medium severity is right: it needs a silk fix plus a conscious topology decision, and prior M9/M13 asked for exactly that with no change made.

### MEDIUM [downgraded→low] — Pyro cap still charges to ~2.6-2.8 V on USB-only power through fire-FET body diodes - enough energy to pop an e-match on the bench (M33 unfixed)
*pyro energy store / USB power — confidence high*

The continuity pull-ups R8/R10/R12/R18 (49.9 k) still reference V_MCU_SWTCH, which is live on USB; current flows through each PYROx_EXT into the TPN4R712MD body diode (drain->source, VDSF <= 1.2 V, ~0.4-0.5 V at uA) and charges V_CAP to ~2.6-2.8 V. No bleed resistor was added to V_CAP (net has only caps, the four 10 k gate pull-ups which return to V_CAP itself, and R20). Stored energy ~35-39 mJ; if a channel driver turns on during a USB bench session (see the WPU finding), the cap dumps ~2.3 A initial (tau ~12 ms) into a 1 ohm match - above typical 0.75-1 A all-fire. 'No pyro energy on USB' remains false as drawn.

**Evidence:** netlist.net: R8/R10/R12/R18 pin 2 = V_MCU_SWTCH (60-node net incl. U17 VDD pins, U22.VOUT); pin 1 = PYROx_EXT = U6/U7/U8/U10 drains; V_CAP net (22 nodes) contains no resistor to GND. Toshiba TPN4R712MD datasheet Rev.6.0.A p.3: VDSF max 1.2 V. USB->V_MCU_SWTCH path live on USB per power sheet (TPS2121/U21 IN1 = VBUS).

**Fix:** Re-reference the four continuity pull-ups to V_CAP instead of V_MCU_SWTCH (kills the sneak path; continuity sensing still works since PYROx_CONT is clamped by D1-D4), or add a 2.2-4.7 k bleed on V_CAP, or explicitly document and accept an energized pyro store on USB.

**Sources:** netlist.net; Toshiba TPN4R712MD datasheet 2026-04-14 Rev.6.0.A; prefab-review-2026-07-30.md M33 (line 725)

**Verifier:** downgraded — The sneak path is real and fully netlist-verified, but the finder's equilibrium math ignores a load their own evidence lists, and the headline numbers collapse when it is included. Verified true: R8/R10/R12/R18 (49.9 k) pin 2 = V_MCU_SWTCH (60-node net incl. U17 VDD pins and U22 TPS22918 VOUT, live on USB - TPS2121 U21 IN1 = VBUS confirmed via Net-(J6-VBUS)), pin 1 = PYROx_EXT = TPN4R712MD drains; body diode direction drain->source into V_CAP is right for the verified P-channel part (VDSF max 1.2 V, Toshiba Rev.6.0.A p.3, fetched); V_CAP has no dedicated bleed. REFUTED detail: V_CAP does NOT reach 2.6-2.8 V, because R20 (150 R) ties V_CAP to VBATT, and VBATT carries the TPS2121 sense dividers hardwired to GND: (R56 12k + R57 1.5k) || (R60 15.4k + R61 5.11k) = 8.1 k (all four values verified in netlist; R57.1 and R61.1 confirmed on GND). Equilibrium through 49.9k/4 = 12.5 k source impedance and ~0.4 V diode drop: V_CAP ~= 2.9 x 8.26k/20.9k ~= 1.1-1.2 V - and any daughterboard load on J3.15/16 (also on VBATT) pulls it lower. That is ~6-7 mJ, not 35-39 mJ, and the fire-FET VGS would be only ~-1.1 V against Vth -0.5..-1.2 V max - marginally enhanced at best - with initial dump current <= ~1 A decaying tau~12 ms, holding above a 0.75 A all-fire for only ~3 ms. 'Enough to pop an e-match on the bench' is therefore not established; it is marginal at worst. What survives is a real but small design-intent violation ('no pyro energy on USB' is technically false; the store idles ~1.2 V) whose fix (re-reference pull-ups to V_CAP) is still worth doing opportunistically - the finder's alternative fix of a 2.2-4.7 k bleed would barely change what the existing 8.1 k path already does. Low, not medium.

### MEDIUM [verified] — In3 ribbon coupling regressed: buzzer PWM line now runs 80 mm at 0.1 mm gap from PYRO2_FIRE; FIRE/CONT still bundled with SPI and USB (M29 worse, not fixed)
*pyro layout / crosstalk — confidence high*

Tonight's re-route kept all pyro FIRE/CONT/ARM lines on the In3 ribbon and made the adjacency worse: PIEZZO (buzzer drive, kHz square wave, active in flight) runs ~80.5 mm parallel to PYRO2_FIRE at 0.10 mm edge gap and ~71.7 mm from PYRO3_FIRE at 0.3 mm; PYRO_ARM runs ~60 mm at 0.10 mm from IND_1; PYRO1_CONT||ESP_SDO 47.7 mm at 0.10 mm; PYRO4_CONT||CEN_D+ 46.8 mm at 0.10 mm; PYRO1_FIRE||ESP_SCLK 18.1 mm at 0.10 mm. Spurious fire from coupling alone is still not credible (fire gate needs ~0.6 V sustained at the DTC input; the P-FET gate node is 10 k + 4300 pF Ciss slow), but the respin's hardware no-fire margin keeps eroding and buzzer/USB/SPI edges will pollute the PYROx_CONT continuity ADC/GPIO reads during every armed pad wait and USB bench session.

**Evidence:** Parallel-run sweep on live rocket-computer.kicad_pcb (script, this session): pairs and lengths above; pyro In3 lengths: PYRO1_FIRE 42.5 mm, PYRO2_FIRE 50.4, PYRO3_FIRE 50.5, PYRO4_FIRE 29.3, CONT nets 48-69 mm, PYRO_ARM 59.1 mm. PIEZZO is the LS1 MLT-8530 buzzer drive via Q9 (external_connections sheet). Prior M29 max was 39.3 mm.

**Fix:** Reorder the In3 ribbon: give FIRE/CONT/ARM their own lane with >= 0.3 mm (3W) to any clock/USB/PWM aggressor or a stitched GND trace between groups; keep PIEZZO away from FIRE lines entirely.

**Sources:** rocket-computer.kicad_pcb (live); prefab-review-2026-07-30.md M29 (line 683); Toshiba TPN4R712MD datasheet (Ciss 4300 pF)

**Verifier:** confirmed — The core claim - the re-route kept all pyro FIRE/CONT/ARM on the In3 ribbon, added the buzzer PWM as a new 0.10 mm-gap aggressor against FIRE lines, and regressed past prior M29 - is real and severity medium is right, but the finder's parallel-run lengths are inflated roughly 2x (almost certainly double-counted overlap; several quoted lengths exceed the nets' total In3 length, which is geometrically impossible). My independent segment sweep of the live pcb (768 In3.Cu segments) reproduces the per-net In3 lengths EXACTLY as quoted (PYRO1_FIRE 42.5, PYRO2_FIRE 50.4, PYRO3_FIRE 50.5, PYRO4_FIRE 29.3, CONT 48.5-69.2, PYRO_ARM 59.1 mm) but measures the pair runs at: PIEZZO||PYRO2_FIRE 42.5 mm at 0.10 mm gap (claimed 80.5 - impossible, PIEZZO's entire In3 length is 62.4 mm), PIEZZO||PYRO3_FIRE 37.2 mm at 0.30 (claimed 71.7), PYRO_ARM||IND_1 34.7 mm at 0.10 (claimed 60), PYRO1_CONT||ESP_SDO 20.9 mm at 0.10 (claimed 47.7), PYRO4_CONT||CEN_D+ 41.2 mm at 0.10 (claimed 46.8 - close), PYRO1_FIRE||ESP_SCLK 15.3 mm total / 8.6 mm at 0.10 (claimed 18.1). Even at the corrected numbers the regression stands: prior M29's worst was 39.3 mm and two pairs now exceed it (42.5 and 41.2 mm), ESP_SDO||PYRO1_CONT grew 4.2->20.9 mm, ESP_SCLK||PYRO1_FIRE 6.1->15.3 mm, and PIEZZO (verified as the LS1 buzzer drive: PIEZZO={U17 GPIO17, R26.1} -> Q9 -> LS1 LOAD-) is a new flight-active aggressor pinned against both PYRO2_FIRE and PYRO3_FIRE. The no-spurious-fire caveat and the CONT-ADC-pollution consequence are both sound. Confirm at medium with the lengths corrected.

### MEDIUM [verified] — C12 polarity marking is still only an ambiguous silk ring on the NEGATIVE pad (H13/M35 unfixed)
*pyro energy store / assembly silkscreen — confidence high*

The only polarity indication on the hand-soldered 10 mF electrolytic is still a single 1.5 mm-radius B.SilkS circle centered on pad 2 (GND, negative) at global (84.91,148.12); there is no '+'/'-' text and no stripe bar anywhere near the part. A ring is commonly read as a pin-1/positive marker - the opposite of intended. A reversed 16 V electrolytic held at 8.4 V on the deployment energy store will degrade and vent in the airframe. The ECO explicitly required polarity marking; two review cycles later it is still absent.

**Evidence:** rocket-computer.kicad_pcb C12 footprint: fp_circle B.SilkS center (3.78,-0.02) r=1.50 (maps to pad 2); pad 1 = V_CAP at (84.90,140.60), pad 2 = GND at (84.90,148.10); board-level gr_text scan shows nearest polarity text is '- Batt +' at (88.68,124.02), 20 mm away, belonging to J7.

**Fix:** Add '+' silk text beside pad 1 and a filled negative-stripe bar beside pad 2 (plus the courtyard from the blocker fix).

**Sources:** rocket-computer.kicad_pcb (live); prefab-review-2026-07-30.md H13 (line 243), M35 (line 745); power-eco.md Change 7

**Verifier:** confirmed — Reproduced in full from the live pcb. The C12 footprint block (lines 67423-67590) contains exactly one silkscreen graphic: fp_circle on B.SilkS, center local (3.78,-0.020033), r=1.50 (stroke 0.12) - which maps by the footprint's -90 rotation to global (84.91,148.12), centered on pad 2. Pad rotation math verified: pad 1 local (-3.74,-0.01) -> global (84.90,140.60) = V_CAP (positive); pad 2 local (3.76,-0.01) -> global (84.90,148.10) = GND (negative). So the only marking is a ring on the NEGATIVE pad, with no '+', '-', or stripe anywhere: my full-board gr_text scan finds the nearest polarity text is '- Batt +' at (88.68,124.02) on B.SilkS, 20.7 mm away (J7 battery), and the only other '+' is at (94.38,100.36), 45 mm away. power-eco.md:119 explicitly requires '+ -> V_CAP, - -> GND' polarity on this footprint, and prior H13 flagged the identical omission - two cycles, no fix. The consequence chain (hand-soldered polarized 16 V electrolytic held at 8.4 V on the deployment store, ring readable as a positive/pin-1 mark) is fair. Medium severity is right for an assembly-error trap on a safety-relevant part that is trivially fixable in silk.

### LOW [unverified] — V_CAP is split across two B.Cu zones that touch only along one accidental 1.85 mm shared fill edge; C12+ connects via THT thermal spokes
*pyro fire-path copper — confidence high*

The V_CAP net uses two separate B.Cu zone objects whose fills do not overlap - they abut along a single shared boundary segment ~1.85 mm long at x=77.98, y 143.88-145.73. All fire current for channels 2/3/4 (U6/U7/U10 sources sit in the upper zone with C10/C11/C13, while C12+, R20 and U8 sit in the lower zone) crosses this knife-edge tangency. It is electrically adequate for the pulse (1.85 mm x ~35 um; Onderdonk 15 ms fusing ~150 A vs ~23 A worst 3-channel), but it is an artifact of two zones landing exactly adjacent, not designed copper - a future refill/clearance nudge can open it (DRC would then report unconnected). Separately, C12's through-hole pads get thermal-relief connection (connect_pads thru_hole_only, 0.5 mm bridges, 0.25 mm gap => at most ~4 x 0.5 mm spokes carry every fire pulse), and there are dead 0.127/0.8 mm F.Cu stub crumbs on V_CAP next to the C12 pad.

**Evidence:** Polygon analysis (script, this session): B.Cu zone A fills bbox (77.9-88.8, 143.9-151.2) contains U6/U7/U10 source pads + C10; zone B fill bbox (74.7-86.9, 138.5-146.5) contains C12.1, R20.1, U8.1-3; zero area overlap at 0.1 mm sampling; exact shared reversed edge (77.963995,145.7323)-(77.995688,143.880402) present in both fills. Zone settings: connect_pads thru_hole_only, thermal_gap 0.25, thermal_bridge_width 0.5. kicad-cli DRC tonight: 0 unconnected items (so currently continuous). F.Cu V_CAP crumbs: 0.8 mm seg (84.9,140.6)->(84.9,141.14) plus seven <0.7 mm 0.127 bits, none reaching any fill or via.

**Fix:** Merge the two B.Cu V_CAP zones into one zone outline (or add a deliberate >= 2 mm wide trace bridging them), set C12's pads to solid zone connection, and delete the dead F.Cu stubs.

**Sources:** rocket-computer.kicad_pcb (live); drc_parity_check.json (this session)

### LOW [unverified] — U9 symbol pin 9 / footprint pad 9 parity warning still fires on the arm FET (prior finding unfixed; masked by tonight's parity-less DRC export)
*pyro arm FET / library hygiene — confidence high*

The scratchpad drc.json shows zero schematic-parity items only because that export did not run the parity check; re-running kicad-cli drc --schematic-parity on the live board tonight reproduces 'No pad found for pin 9 (PYRO_GND) in schematic' on U9. The CSD16323Q3 symbol still has drain pin 9 while the shared 'TSON Advance_TOS' footprint still has two pads numbered 5 (2.5x2.5 tab + 0.381x0.66 perimeter) and no pad 9. Electrically benign (tab and pins 5-8 all PYRO_GND), but it is a permanent warning on the arm FET - the exact noise class that once hid a real missing LoRa ground. ERC also flags 'lib_symbol_mismatch' on U9.

**Evidence:** drc_parity_check.json (generated this session): 1 schematic_parity item, warning, 'No pad found for pin 9 (PYRO_GND) in schematic', Footprint U9. Live PCB U9 pad dump: pads 1-3, 4, 5 (x2), 6, 7, 8 - no 9. netlist.net: PYRO_GND includes U9.9 DRAIN_9. erc.json: lib_symbol_mismatch on U9.

**Fix:** Renumber the footprint tab pad to 9 (or delete pin 9 from the symbol and put its net on pins 5-8), then re-run parity to zero.

**Sources:** drc_parity_check.json; netlist.net; erc.json; prefab-review-2026-07-30.md M18/L (lines 570, 1067)

### LOW [unverified] — PYRO3_FIRE 0.13 mm dangling stub on In3 (leftover from re-route)
*pyro layout / cleanup — confidence high*

One dangling track remains on a pyro net after tonight's re-route: a 0.13 mm PYRO3_FIRE stub on In3.Cu at (80.27,114.56), flagged by DRC as track_dangling. Cosmetic antenna stub on a fire line; the prior review's PYRO1_CONT sliver is gone.

**Evidence:** drc.json (tonight's export): track_dangling 'Track [PYRO3_FIRE] on In3.Cu, length 0.1300 mm {x 80.27, y 114.56}'.

**Fix:** Delete the stub.

**Sources:** drc.json; prefab-review-2026-07-30.md L22/L42

### LOW [unverified] — BOM data errors on pyro parts: C12 manufacturer listed as Nichicon (it is United Chemi-Con) and U9 Datasheet field says TPN4R712MD
*pyro BOM / sourcing — confidence high*

The BOM lists C12 EKYC160ELL103MM25S with Mfr 'Nichicon', but the E*KYC* prefix/series is United (Nippon) Chemi-Con - an assembler matching manufacturer+MPN will fail to source or may substitute the wrong can (case size is the blocker-level constraint here). U9's Datasheet property is 'TPN4R712MD' (copy-paste from the fire FETs); MPN/Mfr columns are correct (CSD16323Q3/TI) so no wrong-stuffing risk, but the doc pointer misleads reviewers of the arm FET.

**Evidence:** bom.csv line 6: "C12","EKYC160ELL103MM25S",...,"EKYC160ELL103MM25S","Nichicon"; line 62: "U9","CSD16323Q3",...,"CSD16323Q3","TI","TPN4R712MD" (Datasheet column). Live PCB U9 property Datasheet = TPN4R712MD, C12 property Mfr = Nichicon. Prior review B3/H14 confirmed the part is Chemi-Con KYC via chemi-con.co.jp.

**Fix:** Set C12 Mfr to 'United Chemi-Con' and U9 Datasheet to the TI CSD16323Q3 link, in the schematic properties so exports stay correct.

**Sources:** bom.csv; rocket-computer.kicad_pcb properties; prefab-review-2026-07-30.md H14 evidence

### INFO [unverified] — Stale/overstated schematic annotations: C12 section still titled 'Super Capacitor'; U9 note claims 'Starred to battery ground' but copper is plane-returned
*pyro documentation — confidence high*

external_connections.kicad_sch still headlines the R20/C9/C12 charge circuit 'Super Capacitor' although the ECO replaced the supercap with an aluminum electrolytic - misleading for polarity/derating expectations. The 'Starred to battery ground' note at U9 is not literally implemented: U9's sources dump into the global GND planes through 8 nearby 0.3 mm vias at the SE corner, 41 mm from the battery connector. With solid In1/In4 GND planes this is electrically fine for the 12 ms pulse; the note just overpromises a star that does not exist in copper.

**Evidence:** external_connections.kicad_sch text items 'Super Capacitor' and 'Starred to battery ground' (schematic.pdf page 3); PYRO_GND copper: J2.5 -> 6 segs 1.5 mm B.Cu -> 4 vias (0.3 drill) under U9 tab; U9 sources on F.Cu GND zone with GND vias at (92.25,163.59), (91.83,162.78), (90.97,164.68), (90.68,165.18) etc.; J7 battery ~41 mm away.

**Fix:** Rename the section 'Pyro Energy Store (10 mF electrolytic)' and either delete the star note or reword to 'returns via GND planes'.

**Sources:** external_connections.kicad_sch; rocket-computer.kicad_pcb; power-eco.md Change 7

### INFO [unverified] — No bleed on V_CAP: the 10 mF store stays near 8.4 V long after battery removal
*pyro energy store / handling safety — confidence high*

V_CAP has no discharge resistor; after battery disconnect the only discharge paths are microamp-level leakages (DTC I_O(off) <= 500 nA, FET IDSS, cap leakage), so the can holds firing-capable voltage for many minutes to hours. Anyone handling e-matches or the J2 terminals right after depowering is working on a hot store. This interacts with the low-side-arm choice (fire FET fault fires a match with no battery present).

**Evidence:** netlist.net V_CAP (22 nodes): C9/C10/C11/C12/C13, R15/R16/R17/R23 (return to V_CAP itself), R20 (to VBATT - eFuse output, blocks reverse), U6-U10 sources - no path to GND. ROHM DTC123J: I_O(off) max 500 nA; Toshiba TPN4R712MD IDSS max -10 uA.

**Fix:** If the USB sneak-path fix adds a 2.2-4.7 k bleed on V_CAP, this resolves too (tau ~30-50 s); otherwise add a bench procedure note: short J2 channels through a resistor before handling.

**Sources:** netlist.net; ROHM DTC123J datasheet; Toshiba TPN4R712MD datasheet


**Affirmatively verified clean (pyro):**

- Fire FET TPN4R712MD is correct for the job: P-channel, VDSS -20 V, VGSS +/-12 V (worst in-circuit VGS = -8.3 V at 8.4 V V_CAP, 3.7 V margin), IDP -180 A at 1 ms vs ~7.6 A fire pulse, Rds(on) max 8.1 mOhm at VGS -2.5 V; the Note-5 V(BR)DSX -12 V degraded-breakdown mode requires reverse gate bias which never occurs here (off-state VGS = 0 via 10 k to source). [Toshiba datasheet Rev.6.0.A pp.1-3]
- Arm FET CSD16323Q3 at 3.3 V direct GPIO drive: VGS(th) 0.9-1.4 V, Rds(on) 5.4-7.2 mOhm at VGS 3 V, VDS 25 V, IDM 240 A - comfortably switches the 25-30 A worst-case 4-channel pulse. [TI SLPS224C p.1,3]
- DTC123J drive from a 3.3 V P4 pin: I_B ~1.16 mA -> deep saturation (VO(on) <= 300 mV), collector load only 0.84 mA vs 100 mA rating; 10 k gate pull-ups give clean FET off at all V_CAP. [ROHM 20150311-Rev.002]
- Boot/reset pin safety re-verified against the live Espressif datasheet: GPIO6/7/9/10/11/12/13/14 (VDD_LP domain) and GPIO16 all show no IE/WPU/WPD at or after reset (Table 2-1 pp.14-15); strapping pins are GPIO34-38 only - none pyro; every fire gate has the internal 47 k DTC pulldown and the arm gate has R22 100 k, so an unpowered or in-reset P4 cannot fire or arm anything.
- Channel mapping is consistent end-to-end: GPIO6->Q5->U8->J2.1 (Igniter 1), GPIO11->Q3->U6->J2.2, GPIO9->Q4->U7->J2.3, GPIO13->Q6->U10->J2.4; silk '1 2 3 4' matches pad positions; D1-D4 BAT54XV2 clamps correctly oriented (A on CONT, K on V_MCU_SWTCH); TSON pinout S1-3/G4/D5-8+tab matches both the Toshiba and TI parts.
- Fire-loop copper is pulse-adequate post-reroute: each PYROx_EXT is a ~4.6 mm-wide B.Cu pour from FET drains (solid SMD connection, connect_pads thru_hole_only) to J2; PYRO_GND runs two 1.5 mm B.Cu paths from both J2.5 tabs into 4x 0.3 mm vias in the U9 drain tab, U9 sources exit into F.Cu GND zone + 8 GND vias; ~5x below 15 ms Onderdonk fusing at the 25 A worst case; the 0.127 mm runs on EXT nets are only the R8-R19 continuity sense stubs, not the fire path (connectivity graph + KiCad 0 unconnected items).
- J2 terminal block TBLH11-350-05-BK: SMT reflow part, 10 A UL / 17.5 A IEC, 24-16 AWG, footprint dual 1.6x4.0 pads at 3.5 mm pitch with 9.30 mm row spacing exactly per Same Sky recommended layout. [sameskydevices.com tblh11-350.pdf pp.1-2]
- R20 charge resistor: Vishay CRCW1206150RFKEAHP is the pulse-proof HP series, 0.75 W; worst dissipation 0.47 W transient at battery connect (tau = 150R x 10.09 mF = 1.51 s, 95% charge in ~4.5 s) and 0.46 W in the stuck-FET continuous fault - within rating. [Vishay doc 20115/20043]
- Continuity sensing: 49.9 k pull-ups drive ~65 uA through an intact match (far below any no-fire spec); PYROx_CONT protected during fire by 100 k series (R9/R11/R13/R19) + BAT54XV2 clamp (pin sees <= ~3.6 V, 48 uA into the rail); R73 1 k bleed worst-case fault dissipation 53 mW vs 62.5 mW 0402 rating (fault-only, matches prior L41).
- C12 electrical suitability: EKYC160ELL103MM25S 10,000 uF 16 V on an 8.4 V max rail, 7.5 mm lead pitch matches footprint pad spacing (pads at +/-3.75 mm); schematic polarity correct (pad 1 = V_CAP+, pad 2 = GND); V_CAP support caps C9-C11/C13 are 16 V-rated (CL21A226MOQNNNE) - adequate at 8.4 V.
- Single-fire energetics: 8.4 V into ~1.2 ohm total loop gives ~7 A initial with tau ~12 ms - well above typical 0.75-1 A all-fire; recharge through R20 in ~4.5 s; firing one channel cannot brown out the others (10 mF droop per 100 mJ match dump is <1 V).


**Checks not completed (pyro):**

- Exact thermal-spoke count actually generated on C12's two through-hole pads: the zone fill polygons encode spokes implicitly and my parser cannot enumerate them reliably; from zone settings the ceiling is 4 spokes x 0.5 mm per pad. Open in KiCad and inspect the fill around (84.90,140.60)/(84.90,148.10) before relying on it, or just set the pads to solid connection.
- Chemi-Con KYC datasheet fetch for the can dimensions was not repeated tonight; I relied on the prior review's chemi-con.co.jp product-page fetch (Ø18 x 25 mm) plus the footprint's own 7.5 mm pitch and r=9.0 fab circle. The bay/airframe inner-diameter clearance for the 25 mm can cannot be checked at all - no bay drawing exists anywhere in the repo.
- E-match all-fire/no-fire specs are assumed typical (0.75-1 A all-fire, 40 mA no-fire); the actual match type used is not documented in the repo, so the USB-energy finding (2.3 A for ~10 ms) should be confirmed against the real match before waiving.
- PIEZZO drive amplitude/frequency in flight firmware is unknown (assumed 3.3 V logic kHz square wave); if the buzzer is driven differently the In3 coupling numbers change proportionally.


## A.esp32-p4 — ESP32-P4 & support

### HIGH [verified] — Chip-revision build config: all four v3.x-mandatory parts (R74/R75/C93/R76) are DNP while the BOM pins the EOL v1.x MPN — one sourcing slip bricks unreworkable boards
*esp32-p4 / core power / chip revision (re-check of prior B1/B4/H9) — confidence high*

The v3.x provisions landed as copper + parts, but every one of them is flagged DNP: R74 (499k ESP_VDD_HP->FB), R75 (499k FB->GND), C93 (22pF feedforward), and R76 (0R pad-54 VDD_HP_1 link). A schematic note documents this as deliberate ('FIT all four for chip rev v3.x -- DNP for v1.x (current dev chips = v1.3)'), and U17's MPN is 'ESP32-P4NRW32' — the non-X part, which is chip rev v1.x and EOL at distributors (JLC C22387510 EOL; current stock is ESP32-P4NRW32X = v3.x per datasheet Table 1-1). Espressif: for v3.0+ 'the feedback resistor and the feedback capacitor must be populated' and pin 54 is VDD_HP_1 (a core supply pin). If the assembler sources the MCU, or v1.3 dev stock runs out, a v3.x chip lands on a board whose DNPs leave a core-supply pin floating and the TLV62569 without a feedback divider — no-boot or core-rail overdrive on a chip-down QFN104 that cannot be reworked. Compounding: tonight's BOM export (bom.csv) INCLUDES the DNP'd parts (R74/R75 qty 2, R76 qty 1, C93 inside the 22pF group of 5), so the assembly outputs are internally inconsistent — it is ambiguous whether the fab would populate them or not.

**Evidence:** central_processing_p4.kicad_sch: (dnp yes) at lines 10191 (R74), 12735 (C93), 15900 (R75), 19011 (R76); note text at line 4456; U17 Value 'ESP32-P4NRW32' at line 16278. rocket-computer.kicad_pcb: footprint attrs 'smd dnp' on all four (R77 1M is fitted, correct for both revs). Scratch bom.csv: '"R74,R75","499 k",...,"2"' / '"R76","0",...,"1"' / C93 in '"C19,C21,C37,C38,C93","22 pF",..."5"' and '"U17","ESP32-P4NRW32",...,"Espressif"'. ESP32-P4 Series Datasheet v0.7 Table 1-1: 'ESP32-P4NRW32X 32 MB (OPI/HPI) ... v3.x' (only X variants listed). HDG v1.9 Overview: 'For chip revisions v3.0 and later versions, the feedback resistor and the feedback capacitor must be populated'; 'In chip revisions v3.0 and later versions, pin 54 of ESP32-P4 is defined as VDD_HP_1'; v1.0/v1.3 'not recommended for new designs'.

**Fix:** Decide and enforce one configuration before fab. (a) If this run uses the on-hand v1.3 dev chips: consign U17 (do not let the fab source it), regenerate BOM+CPL with DNP excluded and verify the four parts are absent from both, keep R77. (b) If v3.x (recommended for a new fab): clear the four DNP flags, change U17 MPN to ESP32-P4NRW32X, add a 100nF at pin 54 (see separate finding), and build firmware with the matching ESP32-P4 minimum-rev config. In both cases add the chosen rev to the fab/assembly README.

**Sources:** https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32p4/esp-hardware-design-guidelines-en-master-esp32p4.pdf (pp.4,9); https://documentation.espressif.com/esp32-p4_datasheet_en.pdf (Table 1-1); https://jlcpcb.com/partdetail/Espressif-ESP32P4NRW32/C22387510 (EOL), C54540373 (X variant)

**Verifier:** confirmed — Every element reproduces from the live files and real documentation. All four parts are DNP in both schematic and PCB (attr 'smd dnp'), R77 is fitted, the deliberate-DNP note exists, U17's Value/MPN is the non-X 'ESP32-P4NRW32', and tonight's BOM export lists the DNP parts with no DNP column at all — an assembler reading bom.csv would populate them or be confused. Espressif HDG quotes verified verbatim ('the feedback resistor and the feedback capacitor must be populated' for v3.0+; 'pin 54 of ESP32-P4 is defined as VDD_HP_1'; v1.0/v1.3 'not recommended for new designs'), and datasheet v0.7 Table 1-1 lists ONLY ESP32-P4NRW16X/NRW32X, both v3.x — the pinned non-X part is gone from the ordering table and LCSC C22387510 shows 'Not available now'. Netlist confirms the failure mechanism: R74/R75/C93 are the only FB divider for U20 (TLV62569DRLR, FB net shared with U17.78), so a v3.x chip on the DNP build gets an undefined core rail plus a floating VDD_HP_1 pin. High severity is right: not a present electrical defect, but the assembly outputs are internally inconsistent and the pinned MPN is unorderable, on a chip-down QFN104. Minor corrections: the finder's schematic line numbers are the Reference-property lines (dnp flags sit ~28 lines above each), and LCSC says 'Not available now' rather than an explicit 'EOL' label — immaterial to the claim.

### MEDIUM [verified] — VDD_HP copper below Espressif 20-mil minimum; pad-54 branch is a 4-mil trace through a 0402 0R with no local 0.1uF
*esp32-p4 / core-rail layout — confidence high*

The HDG requires the VDD_HP_0..3 main power traces be at least 20 mil (0.508 mm) with a 0.1uF at each power pin. On the board, net VDD_HP_1 (pad 54 -> R76) is 0.1 mm (4 mil) wide for its whole 2.5 mm run, and ESP_VDD_HP is distributed at 0.2 mm (8 mil) maximum (F.Cu 14.9 mm, In2 23.9 mm, In3 8.2 mm of 0.2 mm track, plus 0.1/0.127 mm stubs) with 6 vias of 0.3 mm drill — there is no ESP_VDD_HP plane. Pin 54 additionally has no decoupling capacitor on its side of R76, so when the v3.x flip happens its core current and decoupling both funnel through a 4-mil trace and a 0402 jumper. Electrically it will likely run (current shares across pins 26/76/91), but it is out of spec against the exact guideline the v3.x migration is meant to satisfy, and it adds impedance in the chip's heaviest rail (P4 minimum operating current 380 mA).

**Evidence:** Parsed rocket-computer.kicad_pcb: VDD_HP_1 segments all w=0.1 mm F.Cu, 2.5 mm total, pad54 (88.51,115.78) -> R76.2 (90.83,115.44); ESP_VDD_HP by (layer,width): F.Cu 0.2=14.9mm, In2 0.2=23.9mm, In3 0.2=8.2mm, plus 0.1/0.127 stubs; vias 6x(0.4/0.3). No capacitor on net VDD_HP_1 (nodes: U17.54, R76.2 only). HDG v1.9 PCB Layout > Power Supply: 'The trace width for the main power supply traces of VDD_HP_0, VDD_HP_1, VDD_HP_2, and VDD_HP_3 should be at least 20 mil. Place a 10 uF capacitor at the power entry point ... and a 0.1 uF capacitor for each power pin.'

**Fix:** Widen the DCDC-output-to-pin runs to >=0.5 mm (room exists along the east edge of U17), or pour a small F.Cu polygon from L8 to pins 76/91/26; widen the pad-54 branch and add a 100nF (fit-with-R76 group) between R76 and pad 54; consider a wider 0R (0603) or a solderable copper-bridge option for R76 so the core rail does not depend on a 0402 jumper.

**Sources:** https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32p4/esp-hardware-design-guidelines-en-master-esp32p4.pdf (PCB Layout Design > Power Supply, p.21)

**Verifier:** confirmed — Independent parse of the live rocket-computer.kicad_pcb reproduces the finder's numbers exactly. VDD_HP_1: three F.Cu segments, all width 0.1 mm, total 2.46 mm, running from U17 pad 54 global (88.51,115.78) to R76 pad 2 (90.83,115.44). ESP_VDD_HP: maximum width anywhere is 0.2 mm — F.Cu 14.9 mm, In2.Cu 23.9 mm, In3.Cu 8.2 mm at 0.2 mm plus 0.1/0.127 mm stubs — with exactly 6 vias of 0.4/0.3 mm and zero zones/polygons on the net (no plane). Netlist confirms VDD_HP_1 has only 2 nodes (U17.54, R76.2) — no decoupling cap on the pad-54 side. The HDG requirement fetched live and matched verbatim: 'The trace width for the main power supply traces of VDD_HP_0, VDD_HP_1, VDD_HP_2, and VDD_HP_3 should be at least 20 mil' and 'a 0.1 uF capacitor for each power pin'. 0.508 mm required vs 0.2 mm actual (2.5x under) and 0.1 mm on the pad-54 branch (5x under). Medium is the right severity: the board will likely run (current shares across pins 26/76/91 and three parallel layers), but it is a clear violation of the exact guideline the v3.x provisions target, on the chip's highest-current rail.

### MEDIUM [downgraded→low] — P4 exposed pad (the chip's only GND pin) has just 4 stitching vias in a 7.5x7.5 mm EP
*esp32-p4 / grounding + thermal layout — confidence high*

U17's sole ground connection is the exposed pad (pin 105); the 7.5x7.5 mm EP carries all supply return (>=380 mA) and essentially all heat, but only 4 GND vias (0.3 mm drill) land on it, three of them clustered in a 2.7 mm column near the center. Standard practice for a QFN of this size (and Espressif's own dev-board layouts) is a 3x3 to 5x5 grid. Fewer vias raise ground inductance and thermal resistance to the In1 plane; the P4 at 400 MHz dual-core can dissipate ~0.5-1 W. Prior finding M20 (unplugged via-in-pad in the P4's ground) is only resolved if the filled+capped via option is actually carried in the fab order — the declaration exists nowhere in the repo files.

**Evidence:** Parsed rocket-computer.kicad_pcb: U17 pad 105 size [7.5,7.5] at (83.58,111.75); GND vias inside EP: (83.64,110.24), (83.59,112.29), (83.60,112.97), (81.71,114.93), all 0.4/0.3 mm; grep for filled/capped/plugged in the .kicad_pcb finds no fab annotation. In1.Cu has zero tracks (solid GND plane), so added vias land directly on plane copper.

**Fix:** Add a 3x3 or 4x4 grid of 0.3 mm-drill GND vias on ~1.2-1.5 mm pitch inside the EP (filled+capped per the JLC 6-layer class so paste printing is unaffected), and write the filled/capped requirement into the fab notes file so it cannot be dropped from the order.

**Sources:** Espressif ESP32-P4 HDG PCB layout section (ground plane guidance); generic QFN EP via-stitching practice; prior review M20

**Verifier:** downgraded — The headline count is wrong. A robust enumeration of all 390 vias in the live .kicad_pcb finds SIX GND vias inside the 7.5x7.5 mm EP (pad 105 centered (83.585,111.755)), not four: the finder's four plus (80.17,109.74) and (85.63,115.08), all 0.4/0.3 mm. The 'three clustered in a 2.7 mm column' description fits only half the set — the other three are spread across the SW, NW-ish and SE regions of the pad, so both current-return and heat spreading are better than portrayed. Six vias is still short of a 3x3+ grid, but I verified the Espressif P4 HDG PCB-layout page contains NO explicit EP via-count requirement (the finder's own source citation is generic 'ground plane guidance'), so the only yardstick is generic practice, and six 0.3 mm vias are electrically ample for <1 A and thermally workable (~20-25 K/W array, ~10-20 C rise at 0.5-1 W) — especially once filled/capped per the JLC 6-layer class. The sub-claim that the filled+capped requirement is recorded nowhere does reproduce: grep of rocket-computer.kicad_pcb finds only zone 'filled_polygon' tokens, rocket-computer.kicad_pro has no via-fill setting, and the gerbers dir is empty — but that is a fab-order process note, not a layout defect. Real observation, wrong facts at its center, diminished impact: low (opportunistic — add a 3x3 grid and write the fab note), not medium.

### MEDIUM [verified] — Strap pins GPIO34/37/38 still run bare to expansion connector J3 — prior M1/M10/L12 not fixed; GPIO34 violates an explicit datasheet 'must'
*esp32-p4 / strapping pins (re-check of prior M1/M10/L12) — confidence high*

GPIO34 (pin 65, JTAG-source strap), GPIO37 (pin 69, UART0 TX + strap) and GPIO38 (pin 70, UART0 RX + strap) are wired directly to J3 pins 10/11/12 as EXP_11/EXP_10/EXP_09 — two-node nets with no pulls and no series resistors, unchanged since the prior review. The datasheet is unconditional about GPIO34: 'This pin does not have any internal pull resistors and the strapping value must be controlled by the external circuit that cannot be in a high impedance state.' Mitigations verified: with all JTAG eFuses at default 0, GPIO34's value is Ignored (Table 3-7), and GPIO37/38 are 'Any value' in both boot-mode rows (Table 3-3), so today's boot is not corruptible via J3 — but any future expansion board that drives these pins at power-up latches undefined strap state, and anything holding GPIO37/38 breaks UART0 console/UART-download.

**Evidence:** netlist.net: EXP_11 = {J3.10, U17.65}, EXP_10 = {J3.11, U17.69}, EXP_09 = {J3.12, U17.70} (2 nodes each, no R). GPIO35 (R53 100k PU + S3 button via R54 1k) and GPIO36 (R50 100k PU) correctly handled. ESP32-P4 Series Datasheet v0.7: Section 3.4 quote above; Table 3-1 GPIO34/36/37/38 default Floating, GPIO35 WPU; Table 3-3 boot modes; Table 3-5 (eFuse=0 -> GPIO36 Ignored for ROM print, so the 100k PU is side-effect-free).

**Fix:** Fit 100k pulls on GPIO34/37/38 between U17 and J3 (GPIO34 to a defined level per the datasheet 'must'), or reassign EXP_09/10/11 to non-strap GPIOs; at minimum add a fab/user note that J3 pins 10-12 must not be driven or loaded at power-up and that pins 11/12 double as the P4 UART0 console.

**Sources:** https://documentation.espressif.com/esp32-p4_datasheet_en.pdf (Sections 3.1, 3.4, Tables 3-1/3-3/3-5/3-7)

**Verifier:** confirmed — Fully reproduced from the live netlist and the actual datasheet text. EXP_11={J3.10, U17.65/GPIO34}, EXP_10={J3.11, U17.69/GPIO37}, EXP_09={J3.12, U17.70/GPIO38} — exactly two nodes each, no pull or series resistors anywhere on the nets. The claimed mitigations for the other two straps check out (GPIO35: R53 100k pull-up to V_MCU_SWTCH plus R54 1k to button S3; GPIO36: R50 100k pull-up), so the finder's scoping is precise. Every datasheet citation verified verbatim from the extracted PDF text (v0.7): Section 3.4 'This pin does not have any internal pull resistors and the strapping value must be controlled by the external circuit that cannot be in a high impedance state' (GPIO34); Table 3-1 GPIO34/36/37/38 Floating, GPIO35 weak pull-up; Table 3-3 GPIO37/GPIO38 'Any value' in both boot rows; Table 3-7 all-zero eFuses -> GPIO34 'Ignored'; IO MUX table pin 69 = UART0_TXD_PAD, pin 70 = UART0_RXD_PAD. The finder's own mitigation analysis is honest — today's boot is not corruptible via J3 with default eFuses — which correctly lands this at medium (explicit datasheet 'must' violated on GPIO34, plus a real trap for any expansion board or J3 load at power-up and for the UART0 console/download path), not high.

### LOW [unverified] — 40 MHz load caps (12 pF) under-load the 10 pF-CL crystal by ~1-2 pF
*esp32-p4 / clock source (re-check of prior L1/L15) — confidence medium*

The 18 pF -> 12 pF change overshot in the other direction: Y4 (ECS-400-10-37B2-CKY-TR) specifies CL = 10 pF, and C48/C49 = 12 pF give C_series = 6 pF + ~2-3 pF stray = 8-9 pF effective, pulling frequency a few ppm high. The HDG requires overall accuracy within +/-10 ppm and the crystal's own tolerance is already +/-10 ppm, so the cap-induced offset eats margin. Benign for this radio-less chip (USB-serial-JTAG tolerates far more), so low severity.

**Evidence:** bom.csv/netlist: C48/C49 = 12 pF (CL05C120JB5NNNC) on Net-(U17A-XTAL_P/N) with Y4; ECS-400-10-37B2-CKY-TR = 40 MHz, CL 10 pF, +/-10 ppm (ECX-1637B2 series). HDG: 'the accuracy of the selected crystal should be within +/-10 ppm' and CL formula CL = C4*C5/(C4+C5) + Cstray.

**Fix:** Fit 15 pF (2x15 -> 7.5 pF + stray ~ 10 pF) or verify measured frequency on the first article and retune; per HDG 'the values of C4 and C5 need to be further adjusted after an overall test'.

**Sources:** https://www.digikey.com/en/products/detail/ecs-inc/ECS-400-10-37B2-CKY-TR/14548829; HDG External Crystal Clock Source section

### LOW [unverified] — New FB-network parts have empty MPN/Mfr fields in the BOM
*esp32-p4 / BOM hygiene — confidence high*

R74/R75 (499k) and R76 (0R) — added tonight for the chip-rev provision — are the only P4-domain passives with empty MPN and Mfr columns, so the moment the v3.x flip un-DNPs them the BOM is unbuildable as specified (the prior H18 MPN-fill effort missed them).

**Evidence:** Scratch bom.csv: '"R74,R75","499 k","Resistor_SMD:R_0402_1005Metric","","","","2"' and '"R76","0",..."","","","1"' versus e.g. R77 'RC0402FR-071ML, Yageo'.

**Fix:** Fill MPN/Mfr (e.g. Yageo RC0402FR-07499KL for 499k, RC0402JR-070RL for 0R) in central_processing_p4.kicad_sch symbol fields.

**Sources:** bom.csv export from live schematic, 2026-08-04

### LOW [unverified] — No series-R/C provision on the P4 USB pair (CEN_D+/-) although the S3 leg has 22R and the HDG asks to reserve space
*esp32-p4 / USB — confidence high*

CEN_D+/- run ~64-69 mm from U17 GPIO24/25 to the FSUSB63 HSD2 port with no series resistor or shunt-C footprints anywhere near the chip, while the S3's USB leg got R38/R39 22R. The HDG layout checklist says to 'Reserve space for resistors and capacitors on the USB traces close to ESP32-P4'. USB-serial-JTAG is full-speed and will almost certainly work, but there is no tuning provision if it doesn't, and no impedance-control note exists for the 0.1 mm pair routed on In3.

**Evidence:** netlist: CEN_D+ = {U17.53, U1.8, R77.1}, CEN_D- = {U17.52, U1.7} — no series parts; parsed PCB: CEN_D+ 19.9 mm F.Cu + 48.9 mm In3 at 0.1 mm, 2 vias; S3 leg has R38/R39 22R (OUT_D+/- to Net-(U15-GPIO19/20)). HDG 1.4.3 USB quote above.

**Fix:** Add DNP-by-default 0R/22R footprints in the CEN_D+/- near U17 (and optional shunt-C pads), or consciously waive given FS-only use.

**Sources:** HDG v1.9 1.4.3 USB; 1.3.12 USB (22/33 ohm recommendation for FS OTG)

### INFO [unverified] — VDD_PSRAM_0 (pin 59) has no dedicated 0.1uF — one 100nF serves both PSRAM pins
*esp32-p4 / decoupling — confidence high*

HDG asks for 0.1uF + 1uF near each of VDD_PSRAM_0 and VDD_PSRAM_1. Pin 67 has C77 100nF at 2.0 mm and pin 59 has C80 1uF at 1.8 mm, but pin 59's nearest 100nF is C77 at ~3.9 mm. Marginal deviation only; the 1.8 V in-package PSRAM rail is short and locally bulk-decoupled (C87 1uF at VO2).

**Evidence:** Parsed PCB distances: VDDO_PSRAM caps C77 100nF (90.51,111.59) -> pin67 d=2.03 mm; C80 1uF (90.20,113.50) -> pin59 d=1.77 mm; C87 1uF -> pin72 (VO2) d=3.86 mm. HDG Flash and PSRAM IO Power Supply: '0.1 uF + 1 uF capacitors near VDD_PSRAM_0 and VDD_PSRAM_1'.

**Fix:** If space allows, add one more 100nF at pin 59; otherwise waive.

**Sources:** HDG v1.9 Flash and PSRAM IO Power Supply

### INFO [unverified] — P4 cannot power up (hence cannot be flashed) until S3 firmware raises POWER_SWITCH — unchanged from prior L13, no hardware override
*esp32-p4 / power interlock (re-check of prior L13) — confidence high*

Verified the interlock as re-routed: S3 GPIO7 (POWER_SWITCH) -> R71 100R -> TPS22918 (U22) ON, with R68 100k pull-down, VIN=+3V3, VOUT=V_MCU_SWTCH feeding all ten P4 3.3V pins and the CHIP_PU pull-up. Default state is OFF, so a blank or bricked S3 makes the P4 unreachable (the USB mux position for the P4 does not help, since the P4 has no power). This remains a deliberate architecture choice; the sequencing itself is clean (CHIP_PU RC rides the switched rail, straps read after rail-up).

**Evidence:** netlist: POWER_SWITCH = {U15.12, R71.2, R68.2}; R68.1 = GND; Net-(U22-ON) = {R71.1, U22.3}; U22.1 = +3V3, U22.5/6 = V_MCU_SWTCH; R42 (CHIP_PU 10k) pulls to V_MCU_SWTCH; C88 2.2nF on CT.

**Fix:** None required if accepted; a DNP jumper/testpoint from +3V3 to the U22 ON node would give a bench recovery path.

**Sources:** Live netlist export 2026-08-04; prior review L13

### INFO [unverified] — 19 ERC 'pin not connected' errors from unused CSI/DSI/DP/DM/GPIO5 pins lacking no-connect flags
*esp32-p4 / schematic hygiene — confidence high*

All unused MIPI CSI/DSI pins (34-48), the HS USB DP/DM (49/50) and GPIO5 raise ERC errors because no no-connect crosses were placed. Electrically fine — the HDG explicitly allows floating MIPI power/REXT pins and unused DP/DM — but the errors bury real ERC findings in noise (985 items).

**Evidence:** erc.json: 19x 'pin_not_connected' severity=error on /Central Processing - ESP32-P4/ for U17 pins 5, 34-50; HDG: 'If the MIPI interface is not used, the power and external resistor pins can be left floating'; 'If DP and DM are not required, the VDD_USBPHY power supply can be left floating'.

**Fix:** Place no-connect flags on U17 pins 5, 34-50 in central_processing_p4.kicad_sch.

**Sources:** erc.json export 2026-08-04; HDG v1.9 MIPI/USB PHY sections

### INFO [unverified] — 40 MHz crystal trace asymmetry (XTAL_N 14.8 mm vs XTAL_P 9.2 mm) unchanged by the re-route
*esp32-p4 / clock layout (re-check of prior L17) — confidence high*

The re-route left the crystal routing as before: XTAL_N total 14.8 mm vs XTAL_P 9.2 mm at 0.1 mm width. The HDG has no symmetry requirement and every hard rule is met (>=4.5 mm chip gap: 6.9 mm actual; zero vias on both nets; caps flank the crystal; solid In1 GND beneath), so this stays an observation.

**Evidence:** Parsed PCB: Net-(U17A-XTAL_N) 14.8 mm / XTAL_P 9.2 mm, 0 vias each; Y4 at (74.95,102.81), U17 XTAL pads at (80.61-80.96,106.83); C48 (76.79,102.83) and C49 (73.11,102.75) either side of Y4; In1.Cu track count = 0.

**Fix:** None required.

**Sources:** HDG v1.9 1.4.2 Crystal


**Affirmatively verified clean (esp32-p4):**

- Pad-54 provision wiring is correct when fitted: U17.54 -> VDD_HP_1 -> R76(0R) -> ESP_VDD_HP in both netlist and copper (R76 pads at (90.83,114.42/115.44)); resolves the electrical topology of prior B1/H9 subject to the DNP finding
- DCDC FB network matches Espressif Fig 5 exactly: R74 499k ESP_VDD_HP->FB, R75 499k FB->GND (R75.2=GND verified), C93 22pF across R74; FB_DCDC (U17.78) and U20 FB share the node; EN_DCDC (U17.79) -> U20 EN; FB routing 5.6 mm F.Cu, via-free
- TLV62569DRL (U20) pinout verified 1:1 against TI SLVSDG1C: 1=FB, 2=GND, 3=VIN, 4=SW, 5=EN, 6=NC ('can be connected to the output or the ground') — the ERC PG/GND warning is noise; 499k/499k on the 0.6 V reference sets 1.2 V, inside the VDD_HP 0.99-1.3 V window; L8 2.2 uH + 3x10 uF output matches the reference; DCDC sits 5-9 mm from the chip with input cap C47 at 1.7 mm ('placed close' per HDG)
- CHIP_PU RC now 10k (R42) + 1 uF (C39) exactly per HDG v1.9 (which changed 0.1->1 uF on 2026-07-21); trace 3.8 mm, zero vias; pull-up rides the switched rail so reset release always trails rail-up — prior H8(P4)/L11 fixed
- Boot straps correct: GPIO35 100k PU (R53) + download button (S3 tact via R54 1k, no cap per HDG warning); GPIO36 100k PU satisfies Joint-Download 'GPIO36=1'; with default eFuses GPIO36 does not affect ROM printing (datasheet Table 3-5: Ignored)
- 40 MHz crystal part meets HDG hard requirements: ECS-400-10-37B2-CKY-TR is 40 MHz (only frequency supported), +/-10 ppm tolerance, CL 10 pF; **L3, the 24 nH series inductor on XTAL_P, is PRESENT and must stay** — this line previously claimed it was "confirmed gone in netlist and copper", which was wrong; the revert-of-the-revert (e7b81a8a) is in effect and the live netlist reads U15.54 -> L3 -> C20/Y2.3. See the M15 note and the corrected entry later in this document. Removing it costs -18.7 dB on the crystal's 23rd harmonic at 920 MHz, inside the LoRa 902-928 MHz band
- Crystal layout passes every HDG rule: 6.9 mm chip-to-crystal (>=4.5), zero vias on clock traces, load caps flank the crystal at the trace ends, Y4 GND pads 2/4 grounded, In1 is a zero-track solid GND plane beneath
- 32k RTC crystal correct: Y3 ABS07 (12.5 pF CL per symbol name) across GPIO1/pin1 (XTAL_32K_P) and GPIO0/pin104 (XTAL_32K_N) with 22 pF loads, no vias; ESR 70k max = exactly at the HDG <=70k limit (zero margin, carried from prior L14)
- Full per-pin decoupling audit passed: all ten 3.3V pins (9,21,51,62,75,77,85,96,101,102) have a dedicated 0.1uF within 3.3 mm; VBAT has 0.1u+10u (C60/C58); VDD_LDO+VDD_DCDCC have 0.1u each plus 2x10u (C47/C82) per HDG high-current note; ESP_VDD_HP pins 26/76/91 each have 0.1uF at 1.8-3.8 mm plus 3x10uF bulk; VDD_MIPI_DPHY has the exact 10n+0.1u+1u set (C57/C61/C62); VDDO_4 1uF; VDDO_FLASH 0.1u+1u at pin 30, 1u at VO1, 100nF at flash U16 (1.67 mm)
- Flash subsystem correct: W25Q128JVSIQ (2.7-3.6 V) matches VDDO_FLASH default 3.3 V (EFUSE_0PXA_TIEH_SEL_0=0); quad-SPI mapping matches HDG Table 3 pin-for-pin (CS/CLK/D->DI/Q->DO/WP/HOLD); FLASH_CS has the required pull-up (R46 10k to VDDO_FLASH); all six flash signals route on F.Cu with zero vias; VDD_LDO headroom formula satisfied (3.3V switched rail, ~25 mA flash)
- USB architecture verified end-to-end: P4 uses default USB-serial-JTAG pins GPIO24/25 (GPIO25 = 'IE, USB_PU' after reset per datasheet Table 6); FSUSB63 truth table (onsemi datasheet p.2) mapped to board wiring — SEL1 strapped high, S1 slide switch gives SEL=10 -> HSD2 = P4 and SEL=11 -> HSD3 = S3; the NC port HSD1 (SEL=01) and sleep (00) are unreachable; mux VCC always-on via R3 10R + C1 100nF from +3V3 (2.7-4.4 V range ok); R77 1 M pulldown on CEN_D+ fitted (required v1.x, explicitly allowed v3.x); native DP/DM unused with VDD_USBPHY (pin 51) still powered at 3.3 V with 10n+0.1u+10u local caps — exceeds the HDG's optional treatment
- UART0 console is physically available: GPIO37/38 are the fixed UART0 TXD/RXD (datasheet IO MUX F0) and reach J3 pins 11/12, so UART download and console exist independent of USB (contrast prior M22 which remains true only for the S3)
- S3-gates-P4 interlock implemented as documented: S3 GPIO7 -> R71 100R -> TPS22918 ON with R68 100k pulldown (default OFF), +3V3 in, V_MCU_SWTCH out; the In2 V_MCU_SWTCH island spans the whole U17 region and 7 of 8 nearby rail vias land in its fill; QOD strapped to VOUT for discharge-on-off
- DRC clean in domain: only silk-over-mask cosmetics (U20 outline over R74/R75/C93 pads) and a benign Y4 footprint-library hash mismatch; no clearance/unconnected items touch U17, U20, R76 or the XTAL nets
- In2 plane assignments under the P4 sanity-checked by point-in-polygon probes: V_MCU_SWTCH copper under the chip and to its lower-left, VBATT east of the chip, +3V3 north of y=116.7 — no unexpected net under the crystal region


**Checks not completed (esp32-p4):**

- Espressif HDG Fig 5 (TLV62569 v3.0+ reference) component values could not be read graphically from the PDF (figures are raster); the 2x499k + 22pF values were corroborated from the HDG Overview body text and the prior review's reading of the figure instead
- The 'filled+capped vias' declaration exists in no repo file (grep of rocket-computer.kicad_pcb and project files finds nothing); whether the JLC order actually carries the via-fill option — which the EP via-in-pad and any added EP vias depend on — could not be verified from the file set
- Espressif's 'ESP32-P4 Chip Variants' page is a JS-rendered app and would not fetch; the NRW32X-suffix = v3.x mapping was instead taken from ESP32-P4 Series Datasheet v0.7 Table 1-1 (which lists only X variants, all 'v3.x') plus JLC's EOL flag on the non-X part — direct Espressif variant-page confirmation of non-X = v1.x is inferred, not quoted
- ESP32-P4 TRM chapter 'Chip Boot Control' (the authoritative source for GPIO37/38 strap sub-functions beyond 'Any value' in the boot-mode table) was not fetched; the datasheet's Table 3-3 bounds the boot-mode risk but the exact GPIO37/38 latched functions remain uncited
- USB differential impedance of the CEN_D+/- pair on In3 (0.1 mm width, reference to In4 GND) was not computed against the 90-ohm +/-10% HDG figure — stackup prepreg thicknesses for the JLC 6-layer class are not in the repo; flagged only qualitatively in the USB finding
- Thermal estimate for the 4-via EP is qualitative; no power-dissipation model of the P4 at this board's clock/load profile was available to compute junction temperature rise


## A.esp32-s3 — ESP32-S3 & support (incl. RF)

### HIGH [verified] — RF matching network (CLC pi) deleted in tonight's re-route -- LNA_IN now wired straight to the antenna feed
*esp32-s3 / RF path — confidence high*

The Espressif-recommended pi matching network between U15 LNA_IN and the antenna (documented as present and in-window in the 2026-07-30 review: C24 1.5 pF shunt, L2 2.7 nH series, C23 1.5 pF shunt) no longer exists in the live schematic or PCB; the only remaining element is L1 4.3 nH shunt at the antenna end. Without the CLC there is no impedance match, no tuning provision, and no low-pass action on TX harmonics -- telemetry range degrades and 2nd/3rd-harmonic emissions margin (FCC) is lost. This is a regression introduced by the uncommitted edits, most plausibly an accidental deletion during the re-route.

**Evidence:** Live netlist (scratchpad review/netlist.net, exported 2026-08-04 21:08): Net-(U14-Feed) = {U15.1 LNA_IN, U14.4 Feed, L1.2}; refs L2, C23, C24 absent from netlist, BOM and rocket-computer.kicad_pcb (footprint inventory 237, no such refs); DRC schematic_parity = 0 items so schematic and PCB agree on the deletion. Prior review prefab-review-2026-07-30.md line 521: 'The matching topology itself is correct: CLC pi at LNA_IN (C24 1.5 pF shunt, L2 2.7 nH series, C23 1.5 pF shunt) is inside Espressif's recommended windows'. Espressif ESP32-S3 Hardware Design Guidelines, schematic checklist: 'A CLC structure is preferred', C11 1.2-1.8 pF, L2 2.0-3.0 nH.

**Fix:** Restore the three-element pi in esp32s3_outputs.kicad_sch and the PCB between U15 pin 1 and the L1/antenna node (series L 2.7 nH, shunt caps 1.5 pF, 0402 or 0201), placed close to the chip per HDG; keep L1 4.3 nH at the antenna as the Molex-side tuner. Re-tune on the bench.

**Sources:** netlist.net; rocket-computer.kicad_pcb; prefab-review-2026-07-30.md L521; docs.espressif.com esp-hardware-design-guidelines esp32s3 schematic-checklist

**Verifier:** confirmed — Fully reproduced, and I independently proved the regression mechanism: git HEAD of both esp32s3_outputs.kicad_sch and rocket-computer.kicad_pcb still contain C24/L2/C23 (HEAD schematic lines 8887/9091/9678) with a separate Net-(U15-LNA_IN) between chip and match, while the live files have neither the refs nor that net -- the deletion is unambiguously part of tonight's uncommitted edits. Espressif's S3 schematic checklist confirms the CLC requirement and windows verbatim (C11 1.2~1.8 pF, L2 2.0~3.0 nH). High severity is right for a telemetry flight computer: no match, no harmonic low-pass, no tuning provision.

**Disposition (2026-08-05, owner): WAIVED.** Deliberate change implementing Molex AS-479480001 Rev G §6.0's nominal L-network (4.3 nH shunt only, 0 Ω series as direct trace), per base-station review A1 (resolved 2026-08-04, `hardware/base-station/prefab-review-2026-08-02.md`); the no-tune/no-VNA-break-point trade is accepted. See NB-1 in the executive summary. The 50 Ω feed-width item remains actionable.

### MEDIUM [verified] — RF feed trace is ~37-40 ohm, not 50 ohm, on the declared stackup
*layout-rf / RF feed — confidence high*

The U15 pin 1 to U14 feed trace is 0.30 mm wide microstrip over the 0.1 mm er=4.5 prepreg to In1 GND, giving roughly 37-40 ohm characteristic impedance against the antenna's specified 50 ohm input and the HDG's 50 ohm requirement, adding mismatch on top of the missing pi network.

**Evidence:** rocket-computer.kicad_pcb stackup lines 59-65: dielectric 1 prepreg 0.1 mm, epsilon_r 4.5; all 8 Net-(U14-Feed) segments are w=0.300 on F.Cu, total 7.99 mm from pad (75.53,142.47) to antenna pad (73.67,135.49). 50 ohm on this stackup needs ~0.17-0.18 mm. Molex 47948 datasheet: 'Input Impedance (Ohms): 50'; Espressif HDG layout: 'The RF trace should have a 50 Ohm characteristic impedance.'

**Fix:** Narrow the feed run to the 50 ohm width for the ordered JLC stackup (~0.17-0.18 mm with mask, verify with the fab's calculator), or re-plan as CPWG with computed gap. Keep the existing flanking GND vias.

**Sources:** rocket-computer.kicad_pcb (stackup, segments); Molex 47948 datasheet via Octopart; Espressif HDG PCB layout page

**Verifier:** confirmed — Every number reproduces. Stackup: dielectric 1 prepreg 0.1 mm, epsilon_r 4.5 (PCB lines ~59-65). All 8 Net-(U14-Feed) segments are w=0.300 F.Cu, total 7.99 mm, endpoints (75.53,142.47) and (73.67,135.49) as claimed. My independent Hammerstad calc gives Z0 = 37.2 ohm bare (eeff 3.53), a little lower with mask -- inside the finder's 37-40 window; 50 ohm needs ~0.18-0.20 mm, matching the proposed fix. Both external specs verified: Molex 47948 PDF text 'Input Impedance (Ohms): 50' and HDG 'The RF trace should have a 50 Ω characteristic impedance.' Corrective note: this too is a tonight change -- at git HEAD the feed was 0.127 mm (~55-60 ohm), so the re-route fattened it past 50 in the other direction. Medium is right: ~8 mm at 37 ohm is a real but bench-recoverable mismatch, secondary to the missing pi.

### MEDIUM [verified] — U15 exposed pad has 4 ground vias vs Espressif's minimum nine
*layout-rf / grounding-thermal — confidence high*

Only 4 GND vias sit in the S3's 4.1 x 4.1 mm EPAD, all clustered in the east half, and only 8 GND vias exist within 5 mm of the chip; Espressif requires at least nine EPAD vias, and the deficit costs RF ground return and thermal headroom during 340 mA Wi-Fi/BLE TX bursts.

**Evidence:** Via extraction from rocket-computer.kicad_pcb: GND vias inside EPAD square centered (78.955,145.07): (80.35,144.60), (80.77,146.78), (80.30,143.34), (78.41,146.93) -- 4 of 390 total board vias; 8 GND vias within 5 mm radius. HDG layout: 'The ground pad at the bottom of the chip should be connected to the ground plane through at least nine ground vias.' Board setup declares filling yes / capping yes, so via-in-pad is already fab-safe.

**Fix:** Add 5+ more 0.3 mm drill GND vias in a 3x3 grid across the EPAD (filled+capped is already ordered), plus a few stitching vias on the west side of the chip near pins 55/56 and the RF corner.

**Sources:** rocket-computer.kicad_pcb; Espressif HDG esp32s3 pcb-layout-design

**Verifier:** confirmed — Reproduced exactly. U15 at (78.955,145.07), pad 57 EPAD 4.1x4.1 mm at footprint origin; my via extraction (390 vias total, matching the finder's count) finds exactly 4 GND vias inside the EPAD at the four quoted coordinates and exactly 8 GND vias within a 5 mm radius. HDG quote verified verbatim: 'The ground pad at the bottom of the chip should be connected to the ground plane through at least nine ground vias.' Board setup (capping yes)(filling yes) confirmed at PCB lines 141-142, so via-in-pad additions are fab-safe as the fix states. One nuance: 'all clustered in the east half' slightly overstates -- (78.41,146.93) sits ~0.5 mm west of center on the south edge; three of four are east and the west/northwest quadrants are empty, so the distribution complaint stands. Medium severity appropriate for a 9-minimum guideline met at 4/9 on the radio chip.

### MEDIUM [verified] — VDD3P3 RF-supply trace is 0.1 mm vs HDG minimum 20 mil; other power branches also undersized
*layout-power / esp32-s3 — confidence high*

The trace from L4 into VDD3P3 pins 2/3 (the PA supply that carries 335-340 mA TX bursts) is routed at 0.100 mm, a quarter of the HDG's 20 mil (0.51 mm) minimum for these pins, and the +3V3 branches to pins 20/46/55/56 run 0.127-0.2 mm vs the 25 mil main-power guidance; DC drop is small (~15 mohm) but the inductance and the guideline deviation erode TX rail integrity.

**Evidence:** All 9 Net-(U15-VDD3P3) segments w=0.100 (e.g. (74.33,141.46)->(74.33,142.32), (75.10,142.87)->(75.53,142.87)); +3V3 branches near U15 measured 0.127/0.2 mm. HDG layout: 'The width of the main power traces should be no less than 25 mil. The width of VDD3P3 at pin2 and pin3 power traces should be no less than 20 mil.' Datasheet TX peaks 335/340 mA (prior review cite, Tables 5-7/5-8).

**Fix:** Fatten the L4-to-pins-2/3 run to >=0.5 mm (room exists -- it is a 0.8 mm straight shot), and widen the pin 46/20 branches to >=0.3 mm where the escape allows.

**Sources:** rocket-computer.kicad_pcb; Espressif HDG esp32s3 pcb-layout-design

**Verifier:** confirmed — Reproduced. All 9 Net-(U15-VDD3P3) segments are w=0.100 mm on F.Cu (3.08 mm total), including both quoted segments (74.33,141.46)->(74.33,142.32) and (75.10,142.87)->(75.53,142.87), running from the L4 pad to U15 pins 2/3 pads at x=75.53. +3V3 branches within 4 mm of U15 measure 0.127 and 0.200 mm as claimed. HDG quote verified verbatim: 'The width of the main power traces should be no less than 25 mil. The width of VDD3P3 at pin2 and pin3 power traces should be no less than 20 mil.' The finder's honest framing checks out: my copper calc gives ~15 mohm for the run (~5 mV at 340 mA), so this is an inductance/guideline finding, not a DC-drop one -- medium is right. Trivial detail: the 'fix' calls it a 0.8 mm straight shot; the L4-pad-to-pin-2 distance is ~1.9 mm (one 0.86 mm segment is 0.8 mm-class). Immaterial to the claim.

### MEDIUM [verified] — Prior M27 unfixed: inter-MCU I2C still runs 0.3-0.5 mm outboard of the antenna body on F.Cu
*layout-rf / antenna near-field — confidence high*

ESP_SDA (x=72.81) and ESP_SCL (x=72.61) still run as unbroken F.Cu traces along the left board edge through y=133-136, passing 0.33/0.53 mm laterally from the antenna body (x>=73.14) in the strip between antenna and board edge where no ground guard fits -- BLE TX couples into the inter-MCU I2C bus and the traces sit in the radiator near-field (detune/desense), exactly as flagged in the prior review and untouched by tonight's re-route.

**Evidence:** Live PCB segments: ESP_SDA F.Cu (72.81,120.17)->(72.81,147.09) w=0.1; ESP_SCL F.Cu (72.61,119.76)->(72.61,147.17) w=0.1; U14 body 3x3 mm centered (74.64,134.52); board edge x=72.36. Prior finding M27 in prefab-review-2026-07-30.md lines 661-668 with the same coordinates. (ESP_SCLK, by contrast, was moved to In3 under the In1 shield -- that part improved.)

**Fix:** Drop ESP_SDA/ESP_SCL to In3 for the y=128..142 stretch (In1 GND shields them from the antenna), as the prior review proposed; the In3 corridor at x~73 already carries ESP_SCLK.

**Sources:** rocket-computer.kicad_pcb; prefab-review-2026-07-30.md M27

**Verifier:** confirmed — Reproduced coordinate-for-coordinate in the live PCB: ESP_SDA F.Cu (72.81,120.17)->(72.81,147.09) 26.92 mm and ESP_SCL F.Cu (72.61,119.76)->(72.61,147.17) 27.41 mm, both w=0.1, unbroken through the antenna span y=133.0-136.0. U14 at (74.6352,134.525) with 3x3 mm body puts the body's west edge at x=73.135, giving 0.33 mm (SDA) and 0.53 mm (SCL) lateral clearance; Edge.Cuts min x = 72.36 confirmed, so only ~0.25-0.45 mm of strip exists outboard -- no guard fits. The finder's contrast claim also verifies: ESP_SCLK now runs the y=121-137 stretch on In3 at x=73.21, so the re-route improved SCLK but left SDA/SCL untouched from prior M27 (prefab-review lines 661-668, identical coordinates). HDG 'There should be no high-frequency signal traces routed close to the RF trace' fetched verbatim. Medium matches the prior review's own rating.

### MEDIUM [verified] — Prior M22 unfixed: zero testpoints, UART0 and JTAG dead-ended -- USB mux chain is still the only S3 access
*esp32-s3 / bring-up-DFT — confidence high*

U0TXD/U0RXD (pins 49/50) and all four JTAG pins (44/45/47/48) remain unconnected nets and the board still has no testpoint footprints, so the 4-part chain J6 -> CR3 -> U1 FSUSB63 -> S1 remains the single programming/console path to the chip-down S3; there is also no reset access (CHIP_PU has no button or pad), so bench reset means pulling the pack. Any assembly defect in the USB chain leaves the S3 indistinguishable from dead.

**Evidence:** netlist.net: unconnected-(U15-U0TXD{slash}GPIO43-Pad49), unconnected-(U15-U0RXD{slash}GPIO44-Pad50), unconnected MTCK/MTDO/MTDI/MTMS pads 44/45/47/48; PCB footprint inventory: zero TP* refs (237 footprints). Prior finding M22 (prefab-review-2026-07-30.md lines 608-615) requested 6 testpoints; none were added in tonight's edits. Espressif HDG: 'Usually, UART0 is used as the serial port for download and log printing.'

**Fix:** Add bare-copper testpoints for U0TXD, U0RXD, CHIP_PU and GND near the S3 (0.8-1.0 mm dots, B.Cu is fine); the ROM serial bootloader on UART0 then remains a recovery path independent of the USB mux.

**Sources:** netlist.net; rocket-computer.kicad_pcb; prefab-review-2026-07-30.md M22; Espressif HDG

**Verifier:** confirmed — All evidence reproduces from the live exports: netlist contains unconnected-(U15-U0TXD{slash}GPIO43-Pad49), unconnected-(U15-U0RXD{slash}GPIO44-Pad50), and unconnected MTCK/MTDI/MTDO/MTMS pads 44/47/45/48; the live PCB has zero TP* refs among exactly 237 footprints; the USB chain parts all exist as described (J6 USB-C, CR3 SP0503BAHTG TVS, U1 FSUSB63UMX mux, S1 JS202011JCQN switch). The reset claim also holds: Net-(U15-CHIP_PU) = {U15.4, R36 10k to +3V3, C25 1 uF} -- the H8 RC cap did land since the prior review, but there is still no button, jumper, or pad on the net, so bench reset remains pack-pull (the finder claimed only 'no button or pad', which is accurate). HDG UART0 quote fetched verbatim. Prior M22 (lines 608-615) requested the testpoints; none added. Medium is right for a DFT/bring-up gap on a chip-down board.

### MEDIUM [verified] — Prior M20 unfixed: U11 NAND exposed pad still has a 100% paste aperture
*assembly / U11 NAND — confidence high*

U11's pad 9 (3.45 x 4.34 mm GND EPAD) still lists F.Paste in its pad layers, giving a full-size paste aperture whose union with the footprint's 6 smaller windows is still 100% coverage, while every other large EPAD on the board is segmented to 40-79% -- float/tilt and open perimeter leads on the logging NAND at reflow.

**Evidence:** rocket-computer.kicad_pcb U11 footprint: pad "9" layers "F.Cu" "F.Mask" "F.Paste" with no solder_paste_margin, plus 6 F.Paste fp shapes; compare U15 pad 57 layers "F.Cu" "F.Mask" (paste via 9 windows only). Prior finding M20 (prefab-review-2026-07-30.md, U11 paste section) documented the same defect.

**Fix:** Remove F.Paste from pad 9's layer list so only the 6 segmented windows print (target 40-60% coverage), matching the board's other EPADs.

**Sources:** rocket-computer.kicad_pcb U11 footprint block; prefab-review-2026-07-30.md M20

**Verifier:** confirmed — The defect is real and unfixed, with one citation error to correct: the prior finding is M19, not M20 (M20 is the separate via-in-pad finding, which mentions U11 only for its 3 EPAD vias and is now addressed by the capping/filling declaration). Live PCB verified: U11 pad '9' (3.4544 x 4.3434 mm, GND) lists layers 'F.Cu' 'F.Mask' 'F.Paste' with no solder_paste_margin anywhere in the footprint block, so the stencil gets a full-size aperture; the 6 F.Paste segmentation shapes are still drawn but their union with the pad-layer aperture is trivially 100%. Contrast confirmed: U15 pad 57 layers are 'F.Cu' 'F.Mask' only (paste solely via windows), and prior M19 records the board's other EPADs at 40-79% coverage. Title's 'M20' aside, the claim as stated (defect, evidence, fix) is exactly right; severity medium matches M19's rating.

### MEDIUM [verified] — H7 fix incomplete: VDD_SPI pin still carries 10 uF (C27) against HDG guidance, no 100 nF added
*esp32-s3 / VDD_SPI — confidence high*

The core of prior H7 landed (U13 flash VCC moved to +3V3, OUT_VDD_SPI now serves only the in-package PSRAM), but the prescribed C27 resize (10 uF -> 1 uF) and added 100 nF at pin 29 were not done; the RH2's PSRAM node behind the internal 14-ohm R_SPI now sees a 10x-oversized cap (tau ~140 us) that the HDG explicitly warns against, slowing VDD_SPI ramp at every cold boot and light-sleep wake.

**Evidence:** netlist.net: OUT_VDD_SPI = {U15.29, C27.1} only, C27 = 10 uF (C_0402); U13.8 = +3V3 (fix confirmed). Prior H7 fix text (prefab-review-2026-07-30.md line 186): 'change C27 10 uF -> 1 uF and add a 100 nF at pin 29'; HDG/prior cite: 'add 0.1 uF and 1 uF close to VDD_SPI... do not add excessively large capacitors'.

**Fix:** Change C27 to 1 uF and add a 100 nF 0402 next to pin 29 (C27 sits 2.5 mm away at (83.71,150.1) -- room exists), or consciously waive with a note that 140 us ramp is inside the firmware's wake timing.

**Sources:** netlist.net; prefab-review-2026-07-30.md H7

**Verifier:** confirmed — Reproduced in full. Live netlist: OUT_VDD_SPI = {C27.1, U15.29 VDD_SPI} and nothing else; C27 = 10 uF 0402 Samsung CL05A106MQ5NUNC; U13.8 VCC sits on +3V3 (H7 core fix landed) and C26 100 nF migrated to +3V3 with it -- so the VDD_SPI node not only kept the oversized 10 uF, it also lost the 100 nF it used to share with the flash, leaving exactly the opposite of the HDG's prescribed 0.1 uF + 1 uF pair. Prior H7 fix text verified at line ~186: 'change C27 10 uF -> 1 uF and add a 100 nF at pin 29'; HDG quote fetched verbatim: 'It is recommended to add extra 0.1 μF and 1 μF decoupling capacitors close to VDD_SPI. Please do not add excessively large capacitors.' Tau math checks (14 ohm x 10 uF = 140 us). C27 placement (83.71,150.1), ~2.3 mm from the pin-29 corner, confirms room for the added 0402. Medium (fix-or-consciously-waive) is the right call now that only the in-package PSRAM hangs on the node.

### LOW [unverified] — GPIO0 boot strap still has no external pull-up (prior L16)
*esp32-s3 / straps — confidence high*

Net-(U15-GPIO0) still contains only the S2 button to GND; the boot strap rides on the internal ~45 k weak pull-up over an 18.9 mm trace crossing the board to S2, against the HDG's explicit recommendation of an external pull-up -- a brownout-recovery glitch can drop the S3 into download mode until power cycle.

**Evidence:** netlist.net: Net-(U15-GPIO0) = {S2.1, S2.4, U15.5}; PCB: GPIO0 routed 18.94 mm F.Cu/In3/B.Cu to S2 at (91.69,145.43). HDG: 'It is recommended to place a pull-up resistor at the GPIO0 pin.' Prior L16 unchanged.

**Fix:** Add 10 k from GPIO0 to +3V3 next to U15 or S2.

**Sources:** netlist.net; rocket-computer.kicad_pcb; Espressif HDG; prefab-review-2026-07-30.md L16

### LOW [unverified] — 40 MHz crystal still 8.5 mm from the chip with a 14 mm XTAL_N meander (prior L17)
*layout-clocks / esp32-s3 — confidence high*

Tonight's re-route left Y2 at (84.735,137.845), ~8.5 mm from U15's XTAL pads, with XTAL_N totalling 13.96 mm of 0.127 mm trace against ~8 mm on the XTAL_P/L3/C20 side -- long and asymmetric for the RF reference clock, though the mitigating factors all held up under re-measurement (no vias, no same-layer aggressors, solid In1 GND beneath).

**Evidence:** PCB: Net-(U15-XTAL_N) 26 segments 13.96 mm total from pad (77.56,141.65) to Y2.1/C22; Net-(U15-XTAL_P)+Net-(C20-Pad1) = 1.65+6.51 mm; F.Cu corridor scan y138.3-139.9 x78-87 found zero foreign nets; zero vias on all crystal nets. Prior L17.

**Fix:** Opportunistic only: pull Y2/C20/C22 toward the chip's south-east corner next time the floorplan opens; no mandatory action.

**Sources:** rocket-computer.kicad_pcb; prefab-review-2026-07-30.md L17

### LOW [unverified] — No local decoupling within 5 mm of U11 NAND VCC
*esp32-s3 / U11 NAND — confidence high*

The GD5F2GQ5UE (30 mA active, 104 MHz-class bus) has no capacitor near its VCC pin -- the nearest 100 nF on +3V3 (C16) is 5.12 mm away, serving the schematic's intent as U11's decoupler but placed too far; U13's nearest cap (C16/C26) is 2.5-3.2 mm away on the opposite board side through vias.

**Evidence:** PCB positions: U11 (91.335,137.92) F.Cu; C16 100 nF at (91.78,143.02) F.Cu = 5.12 mm from U11 center; C26 at (93.63,142.99) F.Cu vs U13 on B.Cu (91.705,145.485), path includes via (93.15,142.99). Schematic sheet 5/6 draws C16 adjacent to U11 VCC.

**Fix:** Move C16 next to U11 pin 8 (room on F.Cu at x~89.5), and optionally add a 100 nF on B.Cu at U13 pin 8.

**Sources:** pcb_fps extraction from rocket-computer.kicad_pcb; schematic.pdf page 4 (sheet 5/6)

### INFO [unverified] — Claimed '24 nH inductor revert' never landed -- live files still carry L3, which is actually the HDG-correct state
*esp32-s3 / crystal — confidence high*

The commit narrative says the 24 nH crystal-leg inductor was added then reverted in schematics, but the live schematic, netlist and PCB all still contain L3 24 nH in series on XTAL_P with C20 12 pF on the crystal side -- and this is the configuration Espressif's HDG explicitly suggests, so the correct action is to keep it and fix the bookkeeping, not to re-attempt the revert. Schematic/PCB parity is clean (no orphan pads).

**Evidence:** esp32s3_outputs.kicad_sch line 13026 property Value '24 nH' (file mtime Aug 4 16:31); netlist Net-(U15-XTAL_P) = {L3.2, U15.54}, Net-(C20-Pad1) = {C20.1, L3.1, Y2.3}; PCB L3 at (78.53,140.44) routed and connected; DRC schematic_parity 0. HDG: 'Please add a series component on the XTAL_P clock trace. Initially, it is suggested to use an inductor of 24 nH to reduce the impact of high-frequency crystal harmonics on RF performance.' Prior review L1: 'L3 24 nH on XTAL_P is NOT a bug... do not remove it.'

**Fix:** Keep L3 (LQW15AN24NH00D). Correct the changelog/commit note so nobody 'finishes' the revert later.

**Sources:** esp32s3_outputs.kicad_sch; netlist.net; rocket-computer.kicad_pcb; Espressif HDG; prefab-review-2026-07-30.md L1

### INFO [unverified] — Silkscreen 'O'/'F' at the USB-select slide switch is cryptic and no BOOT label at S2
*silkscreen — confidence medium*

S1 selects which MCU owns the USB port (FSUSB63 SEL0) but its silk reads 'O' and 'F', which is at best cryptic mnemonics (OUT-processor/Flight-processor?) and at worst reads as ON/OFF on a switch that never powers anything; S2 (S3 boot) carries no BOOT label either -- cheap bench-error insurance is missing.

**Evidence:** gr_text on F.SilkS: 'O' at (76.98,164.49), 'F' at (76.98,169.41) beside S1 (74.85,166.28); no silk near S2 (91.69,145.43). Netlist: S1.6=SEL0, S1.7=GND -- USB routing only.

**Fix:** Relabel the switch positions 'S3'/'P4' (or 'RADIO'/'MAIN') and add 'BOOT' at S2.

**Sources:** rocket-computer.kicad_pcb silk text extraction; netlist.net


**Affirmatively verified clean (esp32-s3):**

- Prior H7 core fix landed: U13 (W25Q128JVSIQ) VCC is on +3V3 and OUT_VDD_SPI = {U15.29, C27} only; correct for ESP32-S3RH2 whose datasheet Table 1-1 row confirms 2 MB Quad-SPI in-package PSRAM, VDD_SPI 3.3 V, -40~105 C (RH2 is the official upgrade of the EOL S3R2)
- Prior H8/M2 fixed: CHIP_PU now has R36 10 k to +3V3 + C25 1 uF to GND (HDG-exact values), C25 placed 1.7 mm from pin 4 at (73.91,144.31)
- Prior L1/L15 fixed: 40 MHz load caps are now 12 pF (C20/C22) against the ECS-400-10-37B2-CKY-TR's CL=10 pF; crystal spec fetched: 40 MHz, +/-10 ppm tolerance and stability (meets Espressif's +/-10 ppm), ESR 40 ohm, 2.0x1.6 mm; Y2 pins 2/4 grounded
- Prior L6 CLOSED with the datasheet nobody could fetch: FSUSB63 (Fairchild rev 1.0.4 via docs.rs-online.com) Functional Table: SEL[1:0] 00=sleep, 01=HSD1, 10=HSD2, 11=HSD3 -- board's reachable codes are 11 (S1 open -> HSD3 = S3) and 10 (S1 closed -> HSD2 = P4): both MCUs reachable, unused HSD1 unreachable, exactly right; SEL pins never float (R1/R2 100 k pullups satisfy datasheet note 4); VCC 3.3 V within 2.7-4.4 V range via R3 10R + C1 100 nF
- USB path polarity and series R verified: GPIO19(D-) -> R39 22R -> OUT_D- -> HSD3- (pin 9) and GPIO20(D+) -> R38 22R -> OUT_D+ -> HSD3+ (pin 10); 22 ohm matches HDG initial value; SP0503 ESD (CR3) sits at the connector on D+/D-/VBUS
- Molex 47948-0001 antenna use verified against datasheet: it is an on-ground monopole ('requires no ground clearance... leaving the ground layers in the PCB intact'); zone fill confirmed on all 6 copper layers under the antenna -- correct by design; pads 1-3 are 'dummy pads... for strong mechanical bonding' so their unconnected nets are per spec; feed pad 4 wired to the feed trace
- GD5F2GQ5UEYIGR (U11) wiring verified against GigaDevice DS rev 1.6: WSON8 pinout CS#1/SO2/WP#3/VSS4/SI5/SCLK6/HOLD#7/VCC8 matches the netlist exactly; WP#/HOLD# pulled 10 k to +3V3 (R33/R31) for standard SPI; EPAD (pad 9) to GND; package 8x6 mm with 3.4x4.3 mm nominal EPAD matches footprint 8L_WSON_8x6x3p4x4p3; VCC 2.7-3.6 V on +3V3; part number valid per datasheet product list (the ERC lib_symbol_mismatch is only because the symbol was cloned from the pin-compatible Macronix MX35UF4G24AD)
- GPIO35-38 NAND bus is legal on the RH2: the GPIO33-37 restriction applies only to octal flash/PSRAM parts; RH2 is quad PSRAM
- VDD3P3 filter matches the HDG CLC intent: C36 10 uF + C35 1 uF + C34 100 nF on the +3V3 side, L4 2.0 nH into C33 100 nF at pins 2/3; L4 is now a real MPN (Murata LQG15HS2N0S02D, 900 mA rated >= HDG's 500 mA minimum) -- closing the prior BOM-MPN concern for L4
- Per-pin decoupling present and placed: pin 46 VDD3P3_CPU C31 100 nF at 2.05 mm; pin 20 VDD3P3_RTC C29 100 nF at 3.2 mm; VDDA 55/56 C28 10 nF at 1.2 mm + C30 1 uF; bulk 10 uF C32/C36 within 4 mm
- Crystal nets clean where it counts: zero vias on XTAL_P/XTAL_N/C20 nets, no foreign F.Cu traces in the crystal corridor, solid In1 GND beneath; 32 kHz crystal unchanged and correct (ABS07 CL 12.5 pF with 22 pF caps, 1.9 mm from pins 21/22; prior L14 ESR note stands as opportunistic)
- Strap set verified: GPIO45 NC (internal WPD -> VDD_SPI 3.3 V mode, required for RH2), GPIO46 reinforced with external R37 10 k to GND, GPIO0 internal WPU + S2 to GND, GPIO3 on ESP_SDO to the unpowered-at-boot P4 with JTAG-select strap inert without burned eFuses; schematic sheet even annotates all four straps
- Inter-MCU and INA230 I2C pullups sane: ESP_SDA/SCL 5.11 k to V_MCU_SWTCH (correct domain isolation against back-powering the switched P4 rail), PWR_SDA/SCL 5.11 k to +3V3
- DRC: zero unconnected items and zero schematic-parity issues in the live PCB -- no orphan pads/nets anywhere in the S3 domain; via filling+capping declared in board setup (prior via-in-pad concern addressed at fab level)
- ERC noise triaged: power_pin_not_driven on U15.2/29 and U11.8 are power-flag artifacts; S1/U14/U15 pin_not_connected errors correspond to genuinely-unused pins verified above


**Checks not completed (esp32-s3):**

- Molex sales drawing SD-47948-001 / application spec AS-47948-001 could not be fetched (molex.com 403/timeout, curl exit 92): the land-pattern-level check that footprint pad 4 corresponds to the physical antenna's signal pad orientation (rot 180 as placed) could not be closed; the generic datasheet confirms the 3-dummy+1-signal pad scheme but not the geometric orientation. Verify the footprint against the drawing or a physical part before fab.
- onsemi's current FSUSB63 datasheet PDF was unfetchable (bot-blocked); the truth table was taken from the Fairchild FSUSB63 rev 1.0.4 PDF mirrored at docs.rs-online.com -- same silicon, but if onsemi issued a functional errata it would not be visible here.
- Murata LQW15AN4N3C00D (L1) and LQW15AN24NH00D (L3) parameters were confirmed only at search-result level, not from the Murata datasheet pages; tolerance grades look tuning-appropriate but were not verified line-by-line.
- S1 (JS202011JCQN) mechanical position-to-contact mapping (which slide position shorts pins 6-7, i.e. which physical position selects P4 vs S3) was not verifiable from the board files alone; the silk 'O'/'F' finding includes this ambiguity.
- RF impedance figures are analytic estimates from the declared KiCad stackup (0.1 mm, er 4.5), not the fab's real JLC stackup table; the ~38-40 ohm conclusion should be re-run in the fab's impedance calculator when ordering.


## A.sensors — Sensor suite

### HIGH [refuted] — GNSS UART lines still cross into the ground-switched branch with no series resistors (prior M36 unfixed; camera branch got them, GNSS did not)
*sensors / GNSS branch (in_sensors.kicad_sch) — confidence high*

GNSS_RX/GNSS_TX/GNSS_RXD2 run directly from J1 pins 1/2/5 to P4 GPIO4/3/2 with zero series impedance while the daughterboard's entire return (J1.4) is switched by low-side FET Q1 (GPS_ACT, default off). With Q1 off, the module's local ground floats toward VBATT (6.4-8.4 V) and current is driven through the module's and P4's UART ESD clamps; the branch never truly powers off, clamp current is unbounded, and this remains a candidate mechanism for the 'deaf GNSS UART' bench mystery. The claimed fix-commit ('1k series R on cross-board UARTs WIP') landed only on the camera branch (R30/R32 1k in Camera_RX/TX) — GNSS got nothing (LoRa_RX/TX = J5.3/4 direct to U15.15/16 is likewise bare; flagged here for the S3 agent).

**Evidence:** Live netlist (scratchpad review/netlist.net, exported tonight): NET GNSS_RX = J1.1 + U17.4 only; GNSS_TX = J1.2 + U17.3 only; GNSS_RXD2 = J1.5 + U17.2 only. Net-(J1-Pad4) = J1.4 + Q1 drains (pads 1/2/5/6/7); Q1 sources on GND; gate = R6 1k to GPS_ACT (U17.16/GPIO15) with R7 10k pulldown. Contrast: Camera_RX = R30.1 + U17.61 and Camera_TX = R32.1 + U17.60 (R30/R32 = 1k per bom.csv row 39). Daughterboard pin order re-verified against live gnss-sam10m8-18mm-hv.kicad_pcb J3 (3=VSYS, 4=VSS, 1/2/5=signals).

**Fix:** Before fab: add ~1 k series resistors in GNSS_RX, GNSS_TX and GNSS_RXD2 at the rocket side of J1 (same treatment already applied to Camera_RX/TX), or convert the branch to high-side switching and hard-ground J1.4. Apply the same to LoRa_RX/TX (J5.3/4).

**Sources:** prefab-review-2026-07-30.md M36; live netlist.net; bom.csv

**Verifier:** refuted — The core factual assertions — 'zero series impedance', 'clamp current is unbounded', 'GNSS got nothing', 'M36 unfixed' — are contradicted by the live daughterboard: all three cross-board lines have 1 kOhm series resistors on the daughterboard side of the harness (R7/R8/R9), which bounds ESD-clamp loop current at BOTH ends to single-digit mA and is exactly prior M36's prescribed fallback fix ('add ~1k series resistors ... accept that the module is only pseudo-off'). The finder greped only the rocket-side netlist and read J3 pins 1/2/5 as bare 'signals' without noticing the resistors behind them. The LoRa parenthetical is wrong the same way: the LoRa daughterboard has R77/R78 (1 k) behind J6.3/4. What IS true and I confirmed: the branch is genuinely ground-switched (J1/J3 are BM05B-SRSS-TB 5-pin JST SH; pads 6/7 are local shield tabs that do not cross the 5-conductor cable, so the only board-to-board return is switched J3.4/J1.4 via Q1), and the branch never fully powers off (a bounded few-mA sneak path through the three 1k lines keeps flowing when Q1 is off with P4 UART pins driven). That residual is the consciously accepted 'pseudo-off' of the M36 fix — at most an info-level note that firmware should tristate P4 GPIO2/3/4 when GPS_ACT is low.

### HIGH [downgraded→medium] — LoRa switched-return copper pour and feed trace still run directly under the IIS2MDC magnetometer after tonight's re-route (prior M17 unfixed) — violates the IIS2MDC high-current wiring rule
*sensors / magnetometer layout (U3) — confidence high*

The full LoRa daughterboard supply current (E220-class TX bursts, hundreds of mA) loops directly beneath the magnetometer: VBATT -> FL2 -> Net-(J5-Pin_2) 0.3 mm B.Cu trace passing 1.18 mm (lateral) from U3 center -> J5.2 -> module -> J5.1 -> Net-(J5-Pin_1) B.Cu pour whose fill runs under the mag body -> Q10 -> GND. At ~1.7-2.4 mm total distance this is tens of µT — same order as Earth's ~50 µT field — and it is current-modulated, so hard/soft-iron calibration cannot remove it. Telemetry TX and pad heading acquisition overlap operationally, so pad heading (which seeds guidance) is corrupted during every TX burst. The datasheet is explicit: 'Keep currents higher than 10 mA a few millimeters away from the sensor IC.' Tonight's re-route left this geometry byte-identical in effect.

**Evidence:** Live rocket-computer.kicad_pcb parsed tonight: U3 (IIS2MDC) F.Cu at (84.03, 99.85); Net-(J5-Pin_1) B.Cu zone (priority 12) fill edge 0.03 mm from the point directly beneath U3 center, and a 0.4 mm grid sample shows fill coverage under the mag body at y=99.45-100.65 spanning x 83.2-86.4 (mag body x 83.03-85.03, y 98.85-100.85); Net-(J5-Pin_2) B.Cu segment (84.91,100.84)-(85.71,100.84) w=0.3 at 1.18 mm from U3 center; Q10 pads (LoRa return switch) at x 85.6-87.5, y 98.0-99.5; J5 pads 1/2 at (82.64/81.64, 100.37). IIS2MDC datasheet DocID030986 section 5.2 'High-current wiring effects' quoted above.

**Fix:** Pull the Net-(J5-Pin_1) pour and the FL2/Net-(J5-Pin_2) feed out from under U3 — route the LoRa power pair along the board edge east of x~87 and add an all-layer high-current keepout under the mag (the In2 servo plane already keeps 6.6 mm). If the layout must ship as-is, add a firmware rule that blanks/discards mag samples during LoRa TX windows and document the waiver.

**Sources:** IIS2MDC DS DocID030986 Rev 1 sec. 5.2 (mirrored via datasheet.lcsc.com, saved to scratchpad review/iis2mdc.pdf); prefab-review-2026-07-30.md M17/I49

**Verifier:** downgraded — The defect is real and I reproduced the geometry independently on the live board — the LoRa switched return pour and VBATT feed do run directly under U3, unchanged by tonight's re-route, and the ST rule is quoted correctly. But the severity-high rationale rests on an overstated current class: the LoRa daughterboard uses the E220-900MM22S (22 dBm variant, TX ~110-160 mA at 3.3 V) behind a TPS62913 buck with local input caps, so connector-side current is ~50-150 mA — 'hundreds of mA' describes the 30 dBm variant this design does not use. At ~2-2.4 mm from the die that is ~5-15 uT, a 10-30% perturbation of Earth's field (up to ~10-17 deg heading skew during TX bursts), not 'same order as Earth's ~50 uT' in magnitude. A complete firmware mitigation (blank mag samples during TX windows; TX timing is locally commanded) exists and the finding itself offers it as a waiver path — which is this review's definition of medium ('should fix or consciously waive'). The prior review rated identical geometry M17 medium; no new fact justifies the upgrade. Recommend the re-route fix while the board is still uncommitted, but as medium.

### MEDIUM [verified] — Vias, escape traces and solder mask remain under the BMP585 body, against Bosch landing-pattern guidance (prior M37 unfixed)
*sensors / barometer layout (U4) — confidence high*

Bosch: 'We do not recommend vias or traces under the BMP585. Furthermore, it is recommended that there is no solder mask under the sensor... If the solder mask or other material underneath the sensor gets in contact with the sensor, there may be a negative impact on performance' (relies on >=50 µm solder standoff for mechanical decoupling). The live board still has 3 vias inside the 3.25 mm body outline (one GND via 0.9 mm from body center), five F.Cu trace runs crossing under the body, and pads-only mask openings (mask everywhere else under the body — no relief drawn in the footprint). Copper+mask+capped-via bumps under the package can contact it and induce pressure offset/stress noise on the primary apogee sensor. The declared filled+capped via finish softens the via risk (flat caps, no open barrels) but does not meet the guidance.

**Evidence:** Live rocket-computer.kicad_pcb parse: U4 body box [86.17,89.42]x[98.30,101.55]; vias (88.69,100.27) GND, (87.97,101.52) and (88.07,98.35) V_MCU_SWTCH, all 0.4 mm; F.Cu segments under body: SENS_SDO (85.76,100.21)-(86.57,101.02), SENS_SDI (86.57,99.92)-(86.11,99.46), SENS_SCLK (86.57,98.82)-(86.57,97.93), BMP585_CS (89.01,98.82)->(88.91,97.44), V_MCU_SWTCH stubs at (87.9-88.1, 98.3/101.5); footprint LGA9_BMP585_BOS has no F.Mask relief graphics (9 pad openings only).

**Fix:** Re-route U4's pad escapes as short stubs that exit the body outline before dropping to inner layers, move the 3 vias outside the 3.25 mm outline, and add a mask-free, copper-free region under the body per BST-BMP585-DS003-02 sec. 9.3.3 (20 µm horizontal mask clearance).

**Sources:** BST-BMP585-DS003-02 Rev 1.2 sec. 9.3.2/9.3.3 (downloaded from bosch-sensortec.com to scratchpad review/bmp585.pdf); prefab-review-2026-07-30.md M37

**Verifier:** confirmed — Every element reproduces on the live board, tonight's re-route did not fix it, the Bosch guidance is quoted accurately, and medium is the right severity (guidance violation with plausible pressure-offset/stress impact on the primary apogee sensor, softened by the declared filled+capped via finish and mitigable only before fab — matches 'should fix or consciously waive' and the prior review's own M37 classification). One immaterial detail: the GND via is 0.96 mm from body center, not 0.9 mm; and BMP585_INT also grazes the body corner in addition to the five listed runs.

### LOW [unverified] — IMU and barometer still lack the 100 nF decoupling caps their reference circuits specify (prior L43 unfixed)
*sensors / decoupling (U2, U4) — confidence high*

ISM6HG256X application hints: 'Power supply decoupling capacitors (C1, C2 = 100 nF ceramic) should be placed as near as possible to the supply pin.' BMP585 connection figures show 100 nF on both VDD and VDDIO. As built, U2 relies solely on C3 (10 µF 0402, 1.17 mm to VDDIO pad, 2.20 mm to VDD pad; nearest 100 nF is the mag's C2 at 2.83 mm) and U4 solely on C5 (10 µF 0402, 1.20 mm to VDDIO, 3.62 mm to VDD; nearest 100 nF C2 at 3.86 mm). A close 0402 10 µF has similar ESL to a 0402 100 nF so this is likely benign electrically, but it deviates from both reference circuits and the shared rail also carries the buzzer's ~90 mA 2.7 kHz pulse load.

**Evidence:** Pad-to-pad distances computed from live rocket-computer.kicad_pcb (script sensor_pcb2.py in scratchpad review/): C3.1 (80.96,100.92) -> U2.5 VDDIO (80.21,100.02) 1.17 mm, -> U2.8 VDD (81.27,98.74) 2.20 mm; C5.1 (87.55,102.31) -> U4.4 VDDIO 1.20 mm, -> U4.8 VDD 3.62 mm; BOM: C3-C5 = 10 µF CL05A106MQ5NUNC. The mag U3 is compliant (C2 100 nF 1.40-1.82 mm, C4 10 µF 1.12-1.62 mm, C6 220 nF at 1.03 mm from pin 5).

**Fix:** Add one 100 nF 0402 tight to U2 pin 8 (VDD) and one tight to U4 pin 8 (VDD); optional seconds at the VDDIO pins. Local placement only, no reroute.

**Sources:** ISM6HG256X DS15034 sec. 7 application hints; BST-BMP585-DS003-02 connection figures (100 nF + 100 nF); prefab-review-2026-07-30.md L43

### LOW [unverified] — SENS_SCLK still runs 0.10 mm edge-to-edge alongside the P4 crystal XTAL_N trace after the re-route (prior L30 unfixed)
*sensors / SPI bus routing vs P4 crystal — confidence high*

The continuously-clocked sensor SPI SCLK and the P4 main-crystal XTAL_N leg still share an F.Cu corridor at 0.10 mm edge gap (0.1 mm widths) with several mm of close-parallel run near (80.2-81.1, 102.6-103.0) — against Espressif crystal-isolation guidance. P4 has no radio so the consequence is bounded to clock-jitter margin and minor SCLK integrity, but the re-route was the opportunity to separate them and did not.

**Evidence:** Live PCB parse: min edge gap between Net-(U17A-XTAL_N) segment (80.64,102.79)-(80.21,102.79) and SENS_SCLK segment (81.12,102.99)-(80.72,102.59), both F.Cu w=0.1, = 0.10 mm; multiple additional pairs under 0.3 mm gap. In1 GND is solid beneath the region (vertical shielding intact).

**Fix:** Drop SENS_SCLK to In3 for the parallel stretch (In1 GND shields it from the crystal), or move it >=3W away on F.Cu.

**Sources:** prefab-review-2026-07-30.md L30; Espressif hardware design guidelines (crystal isolation)

### LOW [unverified] — M2 screw head at H3 clears the BMP585 body by only ~0.5 mm — washers/wide standoffs will encroach
*sensors / barometer mechanical environment — confidence high*

H3 (M2 mounting hole) center is 4.25 mm from U4 center; a DIN965-class head (~3.8 mm dia) leaves ~0.5 mm to the 3.25 mm sensor body, and the top render shows the U4 silk frame nearly touching H3's pad ring. Any washer, oversized head or wide standoff contacts the strain-sensitive metal-lid baro (Bosch: no material may touch the sensor; 50 µm standoff is its mechanical decoupling), and screw-torque board strain couples straight into the pressure reading.

**Evidence:** Live PCB: H3 at (91.97,99.14), U4 at (87.79,99.92) -> 4.25 mm center distance; render_top.png crop shows the adjacency. BMP585 DS sec. 9.3.2 (contact/material warnings, 50 µm solder height).

**Fix:** Ops/assembly note: plain small-head M2 only at H3, no washer; or nudge U4 ~1 mm west at next spin. Consider torque-controlled assembly for the sled.

**Sources:** BST-BMP585-DS003-02 sec. 9.3.2/9.3.3; prefab-review-2026-07-30.md I47

### INFO [unverified] — Firmware init-order requirement on the shared SPI bus: BMP585 powers up in I2C/I3C mode (prior review's 'starts in SPI4 after POR' is wrong), and the IMU keeps I2C enabled until disabled
*sensors / interfaces (firmware bring-up) — confidence high*

BMP585: 'After soft reset or power-up, the primary interface of the device is in I²C/I3C mode... The HIF switches over to SPI mode if there are at least 16 full serial clock (SCK) edges during the CSB low phase, and CSB has risen again.' The IMU (CS idle-high via R5) likewise keeps its I2C/I3C interface live until IF_CFG.I2C_I3C_disable=1 is written. Until both are locked to SPI, each device sees the other's SPI traffic on the shared SCLK/SDI lines as potential I2C start/stop patterns — risk of spurious register writes before init. Required order: (1) first transaction after V_MCU_SWTCH rises must be a valid >=16-clock SPI access to the BMP585 with CSB low (locks it to SPI); (2) first IMU access sets IF_CFG.I2C_I3C_disable=1; (3) run the shared bus at <=10 MHz (IMU limit) even though the baro allows 12 MHz. This corrects prior finding I48's claim that the baro 'starts in SPI4 after POR'.

**Evidence:** BST-BMP585-DS003-02 interface-selection section (page ~46, quoted); ISM6HG256X DS15034 Table 25 (CS default input with pull-up; I2C disabled via IF_CFG 03h) and Table 6 (fc(SPC)=10 MHz); netlist: SENS_SCLK/SDI/SDO shared by U2+U4+U17 only, R4/R5 10k pulls to V_MCU_SWTCH.

**Fix:** Encode the three-step order in the sensor bring-up code (tinkerrocket-idf components TR_BMP585 / TR_ISM6HG256) and cap the shared bus at 10 MHz.

**Sources:** BST-BMP585-DS003-02; ISM6HG256X DS15034 Tables 6/25

### INFO [unverified] — ISM6HG256_INT2 remains a single-pin net — second interrupt unavailable, datasheet-safe floating
*sensors / IMU (U2) — confidence high*

Net ISM6HG256_INT2 contains only U2.9. DS15034 Table 25: INT2 default is 'output forced to ground', so floating is explicitly safe, but the high-g channel cannot raise an independent interrupt — everything must share INT1 (GPIO50). If intentional, delete the label or mark no-connect to silence the ERC isolated-label warning; if dual-interrupt operation was wanted for the high-g wake path, this is the last chance to route it.

**Evidence:** netlist.net: NET ISM6HG256_INT2 = U2.9 only; erc.json 'Label connected to only one pin: Global Label ISM6HG256_INT2'; DS15034 Table 25 pin 9 row (quoted).

**Fix:** Either route INT2 to a free P4 GPIO or replace the global label with a no-connect flag.

**Sources:** ISM6HG256X DS15034 Table 25; prefab-review-2026-07-30.md L5/I45

### INFO [unverified] — P4 core buck (U20/L8) sits 2-4 mm from the barometer — local heating gradient at the primary apogee sensor
*sensors / barometer thermal environment — confidence medium*

U20 (TLV62569, P4 core supply) at (89.14,104.04) and its shielded inductor L8 at (92.03,103.07) are ~2-4.5 mm from U4 (87.79,99.92). Dissipation is small (high-efficiency buck) but load-dependent; BMP585 temperature-induced offset is ±0.5 Pa/K typ, and fast local gradients (P4 load steps during flight) are what the internal compensation handles least well. Not a blocker — recorded so baro noise during P4 load transients is not chased as a sensor fault, and as a nudge to keep the baro away from regulators next spin.

**Evidence:** Live PCB coordinates above; BMP585 DS Table (pressure temperature-induced offset ±0.5 Pa/K); render_top.png shows L8 adjacent to U4/H3.

**Fix:** None required for this spin; optionally log board temperature alongside baro during bring-up to bound the effect.

**Sources:** BST-BMP585-DS003-02 key features table

### INFO [unverified] — Sensor-frame bookkeeping: IMU rotated 45 deg, mag 0 deg, and the silk axis marks are 13 mm away at 0.75 mm height
*sensors / axes & silk — confidence high*

U2 is placed at rot=45 (79.92,99.03), U3 at rot=0 — the IMU frame is +45 deg about board Z relative to mag/board axes, so firmware orientation constants must carry an exact 45 deg IMU-to-body rotation and a distinct mag frame. The 'X'/'Y' axis gr_texts are unchanged at (92.86,126.60)/(91.86,127.77), 0.75 mm tall (below the board's own 0.8 mm silk minimum, 2 DRC warnings) and ~13 mm from the IMU — easy to misread as IMU axes.

**Evidence:** Live PCB: U2 footprint at rot=45; gr_text 'X' at (92.86,126.6) F.SilkS (line 68304); drc.json min-text-height warnings.

**Fix:** Bump the marks to 0.8 mm, label them explicitly as BOARD axes, and pin the 45 deg IMU mapping in the firmware orientation constants (#390).

**Sources:** prefab-review-2026-07-30.md L25/I46; live PCB

### INFO [unverified] — J5 (LoRa) connector mates directly beneath the sensor cluster — insertion force flexes the board under the strain-sensitive baro and IMU
*sensors / mechanical (bottom-side placement) — confidence medium*

J5 sits on B.Cu at (81.14,99.04) with pads spanning x 78.3-84.0 — directly opposite U2 (79.92,99.03) and within 3 mm of U3/U4. SH-family mating/unmating force is applied straight into the board region whose top side carries the baro that Bosch mechanically decouples via a 50 µm solder standoff. Repeated field mating with the board sled-mounted (supported at H1/H3, 4-5 mm away) is probably fine, but avoid mating J5 while probing baro offsets, and support the board edge during harness work.

**Evidence:** Live PCB coordinates (J5 pads and U2/U3/U4 positions); BMP585 DS sec. 9.3.2 (mechanical decoupling).

**Fix:** Handling note in the assembly/ops doc; next spin consider moving daughterboard connectors off the sensor keep-out zone.

**Sources:** BST-BMP585-DS003-02 sec. 9.3.2


**Affirmatively verified clean (sensors):**

- U2 ISM6HG256XTR pinout verified pin-for-pin against DS15034 Table 2/Table 25 (LGA-14 2.5x3.0x0.83, footprint matches): SDO/SCL/SDA on SENS_SDO/SCLK/SDI, INT1->GPIO50, CS->GPIO49 with R5 10k pull to V_MCU_SWTCH (SPI mode; internal 30-50k PU in parallel), SDx/SCx tied GND ('Connect to VDDIO or GND'), OCS_Aux/SDO_Aux unconnected ('Connect to VDDIO or leave unconnected'), INT2 float safe (default output forced to ground)
- U2 supplies in range: VDD=VDDIO=V_MCU_SWTCH 3.3 V vs DS15034 VDD 1.71-3.6 V / VDDIO 1.08-3.6 V; SPI modes 0/3, 10 MHz max noted for firmware
- U3 IIS2MDCTR fully compliant with DocID030986: pinout matches Table 2 exactly (pins 2/11/12 NC allowed floating), CS pin 3 hard-tied to VDD_IO as required for I2C mode, C1 cap C6 = 220 nF X7R at 1.03 mm from pin 5 (spec: 220 nF, close to pins 5/6, ESR<=200 mOhm), decoupling C2 100 nF (1.82 mm) + C4 10 uF (1.62 mm) at pin 9 per 'as near as possible to pin 9'
- U3 I2C bus is dedicated (GPIO47/48 to U17 only, no other members) with R40/R43 5.11k pulls to the same switched rail — no address conflicts, pulls die with the domain
- U4 BMP585 pinout verified against BST-BMP585-DS003-02 Table 26: SCX/SDX/SDO on shared SPI, CSB->GPIO41 with R4 10k pull to VDDIO, INT->GPIO42, pad 9 L/M unconnected ('No external connection possible'), VDD/VDDIO 3.3 V in range (1.71-3.6/1.08-3.6), VDD/VDDIO 'can be energized in any order'
- BMP585 damage rule 'interface pin high while VDDIO off' is structurally satisfied — every sensor bus net terminates only at U17/V_MCU_SWTCH/GND (same switched domain), verified by netlist membership of SENS_*, *_CS, *_INT nets
- Shared-MISO contention safe: ISM SDO default 'input without pull-up' when CS high, BMP585 SDO is address-LSB input in I2C state — no drive conflict on SENS_SDO
- No P4 boot-strap conflicts on sensor pins: ESP32-P4 strapping pins are GPIO34-38 (Espressif docs, fetched); sensor GPIOs 41/42/46/47/48/49/50/51/52/53 are unconstrained, and both ST sensors' INT pins default output-low
- Magnetometer distance from other high-current paths (after tonight's re-route): In2 VBATT plane fill edge 6.58 mm, servo return Net-(J3-Pad1) 6.55 mm, B.Cu VBATT zone 7.64 mm, V_CAP/PYRO_GND/battery paths >=23 mm, buzzer LS1 57 mm — only the LoRa branch violates the standoff (reported as finding)
- IMU placement vs vibrating mass: U2 is 45 mm from the C12 10 mF can (84.89,144.34), and the sensor row (y~97.5-102.3) is flanked by mounting holes H1/H3 at y=99.14 — stiff, well-supported region
- In1 GND is solid under the sensor cluster except a ~0.8 mm via-clearance void at the mag center — SPI/I2C return paths intact (grid-sampled)
- Baro top side unobstructed: render_top.png shows open area over U4, no overhanging parts, no bottom-entry issue (J5/J3 on B side); no F.Cu zone fill under the U4 body center
- GNSS branch power path: FL1 BLM18PG471SN1D (1 A) feeds J1.3 from VBATT with C7 22 uF bulk 3.3 mm away; branch current (SAM-M10 + TPS62913 daughterboard) well under ferrite rating; Q1 gate R6 1k/R7 10k pulldown = default OFF at boot; J1 pin order matches live daughterboard J3 (3=VSYS, 4=VSS, 1/2/5 signals); SH retention tabs J1.6/7 grounded rocket-side with no electrical path through the 5-wire cable (no arm-switch bypass)
- ERC/DRC contain no genuine sensor-domain electrical defects: all sensor-area errors are symbol pin-type noise (doubled GND power-out pins, unspecified pin classes); DRC sensor items are silkscreen cosmetics only
- No VBAT ADC divider exists in the sensor sheet by design: battery telemetry is INA230 (U23) reading VBAT_CON/VBAT_Terminal over the inter-MCU I2C — nothing feeds a P4 ADC from VBATT (netlist-verified); divider-vs-ADC-range check is N/A
- BOM/footprint/MPN consistency for the suite: U2 ISM6HG256XTR = LGA-14L 2.5x3.0x0.83 footprint, U3 IIS2MDCTR = LGA-12 2x2x0.7, U4 BMP585 = LGA9_BMP585_BOS, pull/cap values (10k CS pulls, 5.11k I2C pulls, 100n/220n/10u/22u) all match schematic and datasheet requirements


**Checks not completed (sensors):**

- st.com unreachable from this session (all fetches timed out); ISM6HG256X reviewed against the full DS15034 Rev 1 (212 pp) mirrored by LCSC — a Rev 2 (Oct 2025) exists per search snippets and its deltas could not be checked
- IIS2MDC reviewed against DocID030986 Rev 1 (Sep 2017) via LCSC mirror; later ST revisions (if any) unverified
- IIS2MDC Table 14 row for pin 7 (INT/DRDY internal default state) was lost in PDF text extraction across a page break; assumed ST-standard output-low — either way it only drives a P4 input, no conflict possible
- LoRa TX burst current magnitude (hundreds of mA class for the E220 at 22 dBm) taken from the prior review/module docs, not re-measured; the field estimate under U3 scales linearly with it
- Mask-under-BMP585 conclusion is inferred from the footprint containing only per-pad mask openings (no relief polygon) — not confirmed against a generated F.Mask gerber render
- Conformal-coat/no-clean process rules for the gel-lid BMP585 are a fab-note item (prior I47); no fabrication-notes file for rocket-computer was checked for it in this pass


## A.connectors — Connectors & cross-board links

*(verify pass for this domain did not run — session limit; tags below are [unverified] except where the main loop spot-checked)*

### HIGH [unverified] — 1k series-resistor fix is incomplete: 12 EXP/servo lines and the px1105r RXD2 line still have no series impedance anywhere in the link
*connectors/cross-board-signals — confidence high*

The WIP claim '1k series R on cross-board UARTs' is only partially real in the live files. Protected: GNSS pins 1/2 (1k on daughterboard: sam10m8 R7/R8, px1105r R9/R8), GNSS pin 5 with sam10m8 only (db R9 1k), LoRa pins 3/4 (db R77/R78 1k), camera pins 3/4 (mainboard R30/R32 1k). NOT protected: all twelve EXP lines J3.3-14 (EXP_01..EXP_12, two-node nets straight to P4 GPIOs 45/44/43/54/39/40/29/28/38/37/34/33; servo-adapter is passive, no R), and GNSS_RXD2 (J1.5 <-> U17.2 GPIO2) when the px1105r board is attached (px1105r J4.5 wires directly to module RXD2, U1.15, no resistor on either board). Because all four branches are still ground-switched (Q1/Q7/Q8/Q10 PMPB14XNX low-side, unchanged from prior review H-finding), an off branch floats its peripheral ground toward VBATT (6.4-8.4 V) and every unprotected line is an uncontrolled injection/backfeed path into the P4; on 'shed under load' the servo return current re-routes through the 12 unprotected EXP wires into GPIO clamps.

**Evidence:** Live netlist scratchpad review/netlist.net: GNSS_RX={J1.1,U17.4}, GNSS_TX={J1.2,U17.3}, GNSS_RXD2={J1.5,U17.2}, LoRa_RX={J5.3,U15.15}, LoRa_TX={J5.4,U15.16}, EXP_01..12 all exactly {J3.x,U17.y} two-node nets; Camera_RX=R30.1/U17.61 with R30.2=J4.3 (1k), Camera_TX likewise R32. Daughterboards (netlists exported tonight from live sch): gnss-sam10m8 R7/R8/R9=1k on J3.1/2/5; gnss-px1105r R9/R8=1k on J4.1/2 but net RXD2={J4.5,U1.15} direct; lora R77/R78=1k on J6.4/3. Q1/Q7/Q8/Q10 drains on Net-(J1-Pad4)/(J4-Pad1)/(J3-Pad1)/(J5-Pin_1), sources on GND.

**Fix:** Add 1k series resistors on EXP_01..EXP_12 between U17 and J3 (or bus switches), and a 1k on GNSS_RXD2 on the rocket-computer side (protects regardless of which GNSS board is attached; also fix px1105r J4.5 with a 1k when that board next spins). Alternatively revert the branches to high-side switching, which removes the whole class.

**Sources:** prefab-review-2026-07-30.md lines 32, 171-175, 354-356 (prior state); live netlists as cited

### HIGH [unverified] — Servo power chain still undersized end-to-end after tonight's re-route (In2 corridor min-cut 2.25 mm, 2x2A Milli-Grid contacts, Q8 8.1 A with 3.0 V gate drive, ILIM now ~11.6 A)
*connectors/servo-power (J3, J8) — confidence high*

The prior review's H5/H16/M6 servo-current findings are unfixed and effectively unchanged by the re-route: servo power still leaves on exactly two Milli-Grid contacts (J3.15/16, ~2 A/contact) and returns on two contacts (J3.1/2) through one PMPB14XNX (8.1 A continuous, gate at only ~3.0 V via the unchanged 1k/10k divider), the In2 VBATT plane corridor between the eFuse exit and J3 necks to 2.25-2.6 mm of 0.5 oz inner copper, the plane is entered/exited through 4-5 vias of 0.3 mm drill, and the eFuse ILIM was lowered only to ~11.6 A (R48 now 127R) — still above the 10 A battery-connector rating and roughly 3x the J3 power-pair rating, so the connectors and Q8 remain the fuses under a multi-servo stall.

**Evidence:** Tonight's live-file scan (pcbnew fill query): In2 VBATT zone filled-width scan = 2.60 mm @y122, 2.30 mm @y134, 2.25 mm @y136, 3.25-4.05 mm through y102-112 (script conn_analysis.py in scratchpad/review/). VBATT vias near J3: (92.07,107.27),(91.69,106.85),(91.74,101.36),(92.29,101.36); eFuse-exit vias (79.06,138.11),(79.05,137.18),(78.85,137.64),(79.07,133.37) — 0.4/0.3 mm. netlist: J3.15/16=VBATT only power pins, J3.1/2=Net-(J3-Pad1)=Q8 drains, R27=1k/R29=10k gate divider unchanged, R48=127R (RC0402FR-07127RL). PMPB14XNX = 40 V, 8.1 A DFN2020MD-8 (Farnell/Nexperia listing; prior review datasheet fetch: Ptot 1.9 W, RDSon 15/18 mOhm at Vgs 4.5 V).

**Fix:** Decide and write down the real servo budget. If >4 A aggregate is credible: widen the In2 corridor to >=6 mm or duplicate on B.Cu, 8-10 vias per transition, parallel more J3 contacts (or a dedicated XT30/VH servo-power connector), larger return FET driven rail-to-rail (drop the 1k/10k divider), and set ILIM below the weakest contact rating. If micro-servos are the spec, document <=4 A and lower ILIM to match.

**Sources:** prefab-review-2026-07-30.md H5/H16/M6 (lines 157-166, 276-283, 419-428); Nexperia PMPB14XNX listing (uk.farnell.com/nexperia/pmpb14xnx: 'N-CH 40V 8.1A DFN2020MD'); Same Sky TBLH11-350 datasheet (fetched)

### MEDIUM [unverified] — J1 pin 5 has different semantics on the two GNSS daughterboards (px1105r: module RXD2 input; sam10m8: TIMEPULSE/PPS output) with no interlock
*connectors/GNSS (J1) — confidence high*

J1 is pin-compatible with BOTH GNSS boards for pins 1-4 (1=module RXD via db 1k, 2=module TXD via db 1k, 3=VSYS, 4=VSS switched return — verified against both live daughterboard sources), but pin 5 is wired for the px1105r meaning (GNSS_RXD2, P4 GPIO2 driving module RXD2): on the sam10m8 board the same pin is the module's TIMEPULSE (PPS) OUTPUT through R9 1k. If firmware configures GPIO2 as UART2 TX while a sam10m8 is plugged in, two push-pull outputs fight through 1k (~3 mA, not damaging but PPS unusable and a silent config trap); conversely with px1105r the firmware must drive it or the module RXD2 floats. Nothing in the repo records which pin-5 personality each daughterboard has.

**Evidence:** rocket-computer netlist: GNSS_RXD2={J1.5, U17.2[GPIO2]}. gnss-px1105r live netlist: RXD2={J4.5, U1.15(RXD2)} (no series R). gnss-sam10m8 live netlist: J3.5 -> R9 (1k) -> PPS net = {R1.1, U1.7(TIMEPULSE)}. All three connectors are BM05B-SRSS-TB, 1:1 cable.

**Fix:** Add a 1k series resistor on GNSS_RXD2 on the rocket-computer (also mitigates finding 1), and document in the repo (README or a harness doc) that J1.5 = RXD2 for px1105r / PPS for sam10m8, with the firmware pin-mux rule per model.

### MEDIUM [unverified] — 'GPS' silk label is buried under the camera connector J4 body (and over its shield pad); the GNSS connector J1 has no label at all
*connectors/silkscreen — confidence high*

On B.Silkscreen the text 'GPS' sits at (80.16,111.82), inside the J4 (camera, B4B-PH-SM4-TB at 83.54,112.13) body outline and clipped by J4's SH2 shield pad — after assembly it is invisible, and what little shows is next to the CAMERA port, not the GNSS port. J1 (GNSS, at 74.78,115.35) has no adjacent label. 'CAM' at (86.36,119.03) is legible below J4 (correct port) but also overlaps R30/R32 pads. Mis-mating is physically prevented (SH 1.0 mm vs PH 2.0 mm families), but the misleading/hidden label defeats its purpose and the silk-over-pad will print broken.

**Evidence:** drc.json (tonight): silk_over_copper "PCB text 'GPS' on B.Silkscreen | Pad SH2 [GND] of J4" at (80.16,111.82); silk_over_copper 'CAM' over R30/R32/C7 pads at (86.36,119.03); render_bottom.png shows 'CAM' visible, 'GPS' hidden under the white PH shroud, no text near J1. Exact pad coords from pcbnew: J4.SH2=(78.14,112.13), J1 pads x=73.35 y=113.35-117.35.

**Fix:** Move 'GPS' next to J1 (clear of copper and of the SH body), keep 'CAM' at J4 but shift it off the R30/R32 pads. One-minute silk edit; do before fab.

### MEDIUM [unverified] — Battery polarity silk '- Batt +' runs horizontally beside vertical J8 pins and marks neither pad; text also overlaps the connector outline
*connectors/battery (J8) — confidence high*

J8's pins are vertical in board coordinates (pin1 GND at 84.73,125.59; pin2 VBAT+ at 84.73,129.55) but the only polarity marking is the horizontal string '- Batt +' above/right of the connector — the '-' and '+' ends do not line up with either pin, so the mark cannot tell a harness builder which pin is which; DRC also flags the text overlapping J8's silk outline 5 times. The VH housing is keyed, so a correctly built harness cannot reverse, but the harness itself is built against this silk, and reverse pack on pin2/pin1 would put -8.4 V on the INA230/eFuse side (CR2 TVS notwithstanding).

**Evidence:** pcbnew: J8 (JST_VH_B2P-VH vertical, B.Cu, rot -90) pad coords as above; silk text '- Batt +' at (88.68,124.02) rot 0 on B.Silkscreen; drc.json silk_overlap 'PCB text - Batt + | Segment of J8' x5 at (82.62-88.68, 123.5-124.0). render_bottom.png confirms the text sits beside, not on, the pin axis.

**Fix:** Place a '+' immediately at pad 2 and '-' at pad 1 (or rotate the existing text 90 degrees so the '+' end is at pin 2), clear of the J8 outline.

### MEDIUM [unverified] — FID1, the board's only fiducial, is still inside the USB-C (J6) courtyard under the shell — carry-over from prior review, unfixed
*connectors/USB-C (J6) / assembly — confidence high*

The single DRC error tonight is still 'Courtyards overlap: Footprint J6 | Footprint FID1'. FID1 (84.32,168.07) sits fully inside J6's F.CrtYd (77.88-90.83 x, 162.86-171.34 y) and under the metal shell in the top render, making it useless for pick-and-place vision on a board with chip-down 0.35 mm-pitch QFN104 (U17) — exactly as reported in the 2026-07-30 review (M-item), with no change in the live file.

**Evidence:** drc.json: courtyards_overlap error at (84.36,164.79) J6|FID1; pcbnew: FID1 at (84.32,168.07) F.Cu; J6 courtyard extents computed from live footprint graphics; render_top.png shows the spot covered by the GCT shell.

**Fix:** Move FID1 clear of J6 and add 2-3 fiducials (>=1 mm dot, both sides) per assembler guidance.

**Sources:** prefab-review-2026-07-30.md lines 237-239

### MEDIUM [unverified] — USB VBUS node has zero capacitance between the connector and the TPS2121 IN1 pin
*connectors/USB-C (J6) — confidence high*

Net-(J6-VBUS) contains only the J6 VBUS pads, CR3.2 (TVS), the R51/R62 divider taps and U21.7 (TPS2121 IN1) — no capacitor at all. USB hot-plug into cable inductance with no local bulk produces ringing/overshoot at IN1, and TI's TPS2121 datasheet application/layout guidance calls for a >=0.1 uF (typically 1 uF) capacitor close to each IN pin. The SP0503 TVS clamps hard ESD but not the plug-in LC ring shape.

**Evidence:** Live netlist: Net-(J6-VBUS) node list = {J6.A4/B9, J6.B4/A9, CR3.2, R51.2, R62.2, U21.7}; no Cxx member. Board search confirms no capacitor footprint on that net.

**Fix:** Add 1 uF (>=16 V) X7R from VBUS to GND adjacent to U21 IN1, plus optionally 0.1 uF at the connector.

**Sources:** TI TPS2121 datasheet input-capacitor recommendation (application section; not re-fetched tonight — flag if you want the exact sentence pulled)

### MEDIUM [unverified] — Camera port J4 pin 2 carries raw eFuse-switched pack voltage (6.4-8.4 V); within RunCam Split 4's 5-20 V spec but against RunCam's own 'no direct battery' guidance, and the input range is still not recorded in the repo
*connectors/camera (J4) — confidence high*

Answering the assignment question: J4.2 = VBATT (post-eFuse, otherwise raw 2S pack), J4.1 = Q7-switched ground return, J4.3/4 = UART through 1k R30/R32 to P4 GPIO31/30. The intended camera is now recorded (README: 'RunCam Split 4 and GoPro Hero 10 Black'), and Split 4 is specified DC 5-20 V / ~450 mA, so 2S is in range — but RunCam retail guidance explicitly warns that direct battery power surges can destroy the camera, and this rail is shared with the servo branch (the board's designated transient aggressor per power-eco.md). The 5-20 V figure appears nowhere in the repo, and a 5 V-only RunCam (e.g. Thumb) plugged into the same PH connector would be destroyed.

**Evidence:** Netlist: J4.2=VBATT (with J3.15/16 servo power on the same 22-node net); README.md lines 10, 29, 134 name Split 4/GoPro; RunCam Split 4 retail spec 'DC 5-20V' and 'requires non-direct power supply from battery' (Amazon/RunCam listings, Oscar Liang review). power-eco.md: servos are the main aggressor on this node.

**Fix:** Record 'J4 supplies 6.4-8.4 V raw pack; camera must be a wide-input model (Split 4 OK, 5 V-only models will be destroyed)' in the repo/silk; consider an LC or ferrite+bulk on the J4.2 feed to divorce camera Vin from servo transients.

**Sources:** https://www.amazon.com/RunCam-Split-Camera-Latency-Recording/dp/B08BWWXK49 (DC 5-20V); https://oscarliang.com/runcam-split-4/ (BEC recommendation); prefab-review-2026-07-30.md M11

### MEDIUM [unverified] — ESP32-P4 strapping pins GPIO34/37/38 still routed bare to EXP connector J3 with no pulls — prior-review M10 unfixed
*connectors/EXP (J3) boot straps — confidence high*

EXP_11=J3.10=GPIO34 (JTAG source strap, floating default), EXP_10=J3.11=GPIO37 and EXP_09=J3.12=GPIO38 (boot-mode straps, floating default) are still two-node nets with no pull resistors, so anything attached to J3 that loads these wires while the S3 releases the P4 into reset can corrupt boot/JTAG configuration. Unchanged from the 2026-07-30 review despite the P4 v3.x circuit work.

**Evidence:** Live netlist: EXP_09={J3.12,U17.70[GPIO38]}, EXP_10={J3.11,U17.69[GPIO37]}, EXP_11={J3.10,U17.65[GPIO34]} — no other nodes. Espressif esptool boot-mode doc + P4 hardware design guidelines: strapping pins GPIO34-38; GPIO35 WPU, 34/36/37/38 floating (fetched tonight).

**Fix:** Add 100k pulls to the documented default level on GPIO34/37/38 between U17 and J3, or move EXP_09/10/11 to non-strap GPIOs; at minimum document that J3 positions 10-12 must not be driven/loaded at P4 power-up (and keep servo channels on EXP_01..06 in firmware).

**Sources:** https://docs.espressif.com/projects/esptool/en/latest/esp32p4/advanced-topics/boot-mode-selection.html ; prefab-review-2026-07-30.md M1/M10

### LOW [unverified] — USB ESD array CR3 hangs on an ~8 mm stub on the far side of the D+/D- run from the connector
*connectors/USB-C (J6) layout — confidence high*

D+/D- leave J6 (x~84.4) and route LEFT to the FSUSB63 (U1 at 78.04,159.67), while CR3 (SP0503, at 92.11,159.83) is reached by a separate ~8 mm branch running RIGHT — the TVS sits on a stub rather than in the connector-to-mux flow-through path, reducing ESD clamping effectiveness (added trace inductance) and adding a reflection stub. Both attached MCU USB ports are Full-Speed (12 Mbps: P4 USB-Serial-JTAG GPIO24/25, S3 GPIO19/20), so it will work, but it is against SP0503/USB layout practice. CC1/CC2 have no ESD device (only the 5.11k pulldowns).

**Evidence:** pcbnew track dump: D- path F.Cu (82.28->79.41,161.74)->(78.24,160.57) to U1 vs branch (86.13->90.79,161.74)->(93.01,158.87) to CR3; component positions J6=(84.36,164.79), U1=(78.04,159.67), CR3=(92.11,159.83). SP0503BAHTG on VBUS/D+/D- only.

**Fix:** Move CR3 adjacent to J6 so the pair passes it before heading to U1 (or accept for FS-only use and note it); optionally add ESD on CC1/CC2.

### LOW [unverified] — S1 (USB routing switch) silk says only 'O'/'F' — reads as ON/OFF but it actually selects P4-vs-S3 USB target
*connectors/USB mux control — confidence high*

S1 pin 6 (pole common) = SEL0 with R2 100k pull-up to 3V3, pin 7 throw = GND: closed = SEL0 low = FSUSB63 routes the USB-C to the P4 (HSD2), open = SEL0 high = S3 (HSD3). The panel marks are 'O' and 'F', which any user will read as a power on/off switch; nothing indicates which position programs which MCU. Electrically the switch wiring is correct (common on the center pad, verified from pad coordinates).

**Evidence:** pcbnew: S1 pads col x=76.05 at y 163.78/166.28/168.78 with pad6 (center)=SEL0, pad7=GND; R2=100k to +3V3; F.Silk texts 'O' (76.98,164.49) and 'F' (76.98,169.41); FSUSB63 functional table (datasheet fetched tonight): SEL1/SEL0 10=Port2, 11=Port3.

**Fix:** Relabel the two positions 'P4' and 'S3' (or 'MCU'/'RADIO').

### LOW [unverified] — All peripheral connectors are friction-retention only; no positive lock anywhere for flight vibration
*connectors/mechanical retention — confidence medium*

Retention inventory: J1/J5 JST SH (1.0 mm, lightest friction fit, most vibration-sensitive), J4 JST PH (friction), J8 JST VH (high-force friction, no latch in B2P-VH standard header), J3 Molex Milli-Grid 87832 shrouded header (mating 51110 receptacle friction/polarized), J2 TBLH push-in spring (good wire retention), J6 USB-C (bench only). For a high-g airframe the SH links (GNSS/LoRa) and the PH camera link are the credible in-flight disconnect risks; there is no note in the repo about staking/gluing harnesses.

**Evidence:** BOM/netlist MPNs: BM05B-SRSS-TB, BM04B-SRSS-TB, B4B-PH-SM4-TB, B2P-VH(LF)(SN), 878321620, TBLH11-350-05-BK; JST SH/PH/VH families have no positive-lock variants in these headers.

**Fix:** Add a fabrication/integration note (like lora-daughterboard/FABRICATION-NOTES.md) requiring silicone staking or tape-down of SH/PH/VH harnesses for flight; consider locking-family connectors (GH instead of SH) at the next respin.

### LOW [unverified] — INA230 IN+ Kelvin tap connects into the VBAT_Terminal zone ~1.9 mm from the shunt pad, adding zone copper drop to the 2 mOhm measurement
*connectors/battery current sense — confidence medium*

The dedicated 0.1 mm sense trace from U23.13 (IN+) runs via In2 and drops into the B.Cu VBAT_Terminal zone at (81.38,129.82) — 1.9 mm from the R72 shunt pad 2 at (79.57,131.28), inside the zone that carries the full battery current from J8.2 (84.73,129.55) to the shunt. The measured 'shunt' therefore includes a fraction of zone spreading resistance (order 0.1-0.5 mOhm depending on current path), a 5-25% gain error on pack-current telemetry that the app uses ('monitor current usage'). IN- (VBAT_CON) taps within ~0.5 mm of R72 pad 1, which is fine.

**Evidence:** pcbnew: 0.1 mm VBAT_Terminal tracks B.Cu (79.20,127.77)->(79.70,127.83) at U23.13, In2 (79.70,127.83)->(80.95,129.82)->(81.38,129.82), via at (81.38,129.82) into the 19.2 mm2 B.Cu VBAT_Terminal zone; R72 pad2 at (79.57,131.28); VBAT_CON 0.1 mm trace ends (80.04,129.45) ~0.47 mm from R72 pad1 (79.57,129.46).

**Fix:** Land the IN+ sense via directly on/at the R72 pad-2 pour neck (true Kelvin), not mid-zone.

### INFO [unverified] — GNSS and LoRa links require same-polarity (1:1) SH jumpers; a reversed cable puts the switched-ground pin on daughterboard signal pins
*connectors/harness — confidence high*

Both ends of the GNSS link (rocket J1 and db J3/J4) and of the LoRa link (rocket J5, db J6) use identical top-entry BM0xB-SRSS receptacles, so the pin mapping is only correct with a same-side (pin1-to-pin1) SH cable; reversed-polarity SH jumpers are commercially common. Reversed on GNSS: rocket pin4 (Q1 switched GND) lands on db TXD (via 1k) and db VSS lands on rocket GNSS_TX — non-functional but current-limited; the symmetric VSYS pin 3 still connects. Nothing in the repo specifies the cable polarity.

**Evidence:** Connector MPNs and pinouts from live netlists both sides (rocket J1/J5; sam10m8 J3; px1105r J4; lora J6).

**Fix:** Document '1:1 (same-side) 5-pin and 4-pin SH cables required' in the repo; ideally photograph the correct harness.

### INFO [unverified] — eFuse ILIM landed at 127R (~11.6 A) — improvement, but still above the battery connector's 10 A rating and ~3x the J3 servo pair rating
*connectors/battery (J8) protection coordination — confidence high*

The claimed ILIM commit is real (R48 = 127R, was 100R), lowering the TPS259824 limit from ~14.7 A to roughly 11.5-12 A. That is still above the JST VH 10 A/contact rating and far above the 2x2 A Milli-Grid servo path, so overcurrent protection still does not protect the weakest links (ties into the servo-budget finding).

**Evidence:** Live netlist/BOM: R48 = '127' RC0402FR-07127RL on Net-(U19-ILIM); JST VH rated 10 A; prior review documented the TPS25982 RILIM=100R row = 14.71 A typ.

**Fix:** Fold into the servo-budget decision: either document a <=4-6 A system budget and drop ILIM accordingly, or upsize the connector chain.

**Sources:** prefab-review-2026-07-30.md lines 138-151; JST VH series catalog rating


**Affirmatively verified clean (connectors):**

- FSUSB63 SEL truth table — the datasheet nobody could fetch in the prior review — is now fetched and the mux wiring VERIFIED CORRECT: functional table is 00=Sleep(open), 01=HSD1, 10=HSD2, 11=HSD3; board has SEL1 hard-pulled high (R1 100k to +3V3) and SEL0 toggled by S1 (100k up / switch to GND), so only Port 2 (P4, CEN_D+/- on U17.52/53) and Port 3 (S3, via 22R R38/R39 to U15 GPIO19/20) are reachable — the unconnected HSD1 port and Sleep are unreachable; SEL pins can never float (datasheet note 4 satisfied); VCC=3.3 V within 2.7-4.4 V range. Source: FSUSB63 datasheet (docs.rs-online.com/6704/0900766b813c090e.pdf, saved to scratchpad review/fsusb63.pdf)
- USB-C J6 basics: independent 5.11k CC1/CC2 pulldowns to GND (R41, R47 — correct UFP/sink), D+/D- correctly paralleled A6/B6 and A7/B7, SBU unconnected, all four shield pads to GND, connector courtyard flush to board edge for cable insertion, GCT USB4110-GF-A all-SMT with two 0.65 mm NPTH locating pegs matching the footprint
- P4-side USB pins are the correct programming path: CEN_D+/- land on U17 GPIO25/24 = ESP32-P4 USB-Serial-JTAG (the FS OTG default is GPIO26/27, not used here); S3-side on GPIO19/20 (native USB) with the Espressif-recommended 22R series resistors
- J2 pyro terminal block verified against the Same Sky TBLH11-350 datasheet: it IS a reflow-solderable SMT part (footprint pad 1.6x4.0, 3.5 mm pitch, 13.3 mm row extent matches the datasheet's recommended layout exactly, L=18.20 mm for 5 poles), rated 10 A UL / 17.5 A IEC per contact, 300 V, push-button spring clamp 24-16 AWG — comfortably above worst-case cap-dump fire current (~7 A pulse from V_CAP through ~1.2 ohm loop)
- J2 pyro silk labels '1 2 3 4 GND' are present, correctly ordered and aligned with terminals 1_x..5_x (board x 76.13/79.41/83.03/86.45/90.97 vs pads 75.55/79.05/82.55/86.05/89.55); wire entry faces off the y=171.32 board edge
- Pyro fire paths: each channel runs from its high-side driver (U8/U6/U7/U10) to J2 through a dedicated 24-39 mm2 B.Cu pour plus 1.5 mm tracks (connectivity holds with all <0.6 mm copper removed — the 0.127 mm spurs are only the continuity-sense taps to R8-R19); PYRO_GND return is 1.5 mm B.Cu into U9 CSD16323Q3 (25 V) with R73 1k bleed to GND; the terminal block's through-row is SMD so there is no pin-protrusion conflict with the USB-C mounted back-to-back on the F side
- J8 battery input: prior blocker B-3 connector fix CONFIRMED — JST PH replaced by B2P-VH(LF)(SN) (VH 10 A class, keyed/polarized housing, 1.7 mm PTH drills correct for VH posts); pin1=GND, pin2=VBAT_Terminal -> R72 2 mOhm shunt -> INA230-monitored -> TPS259824 eFuse; the J8.2-to-shunt current path is a 19.2 mm2 B.Cu zone (no thin-trace bottleneck; the 0.1 mm segments on these nets are Kelvin sense traces, see finding)
- GNSS link pin-for-pin against BOTH live daughterboard sources: J1.1->db RXD (input) via db 1k, J1.2<-db TXD via db 1k, J1.3=VSYS (FL1 ferrite from VBATT; db side has SMF10A TVS + DLW21SN670 common-mode choke), J1.4=VSS switched return (Q1) — pins 1-4 identical on gnss-sam10m8-18mm-hv (J3) and gnss-px1105r (J4); UART directions correct; SH mounting tabs on both boards are local-GND solder anchors only and cannot create a cross-board ground (SH plugs carry 5 circuits), so the ground-switch cannot be defeated by a standard cable
- LoRa link pin-for-pin against hardware/lora-daughterboard live source: J5.1=switched return (Q10) <-> db J6.1 VSS (via CMC), J5.2=FL2-filtered VBATT <-> db J6.2 +BATT (db has CUS10S30 series schottky), J5.3 LoRa_RX (U15.15) <-> db TX (U28 GPIO6 via 1k R78), J5.4 LoRa_TX (U15.16) <-> db RX (U28 GPIO5 via 1k R77) — correctly crossed, 1k series both signals
- Camera port J4 pinout matches a RunCam Split-4-style 4-wire harness (VIN / switched-GND / RX / TX), UART has 1k series both directions (R30/R32), PH 2 A contacts adequate for the camera's ~450 mA; RunCam Split 4 model support now recorded in README.md (prior review gap partially closed)
- J3 EXP connector mechanics: Molex 87832-1620 Milli-Grid footprint pad numbering matches the dual-row odd/even convention, two 1.04 mm NPTH fitting-nail holes per Molex drawing, polarized shroud; silk '+' marks the VBATT end (pads 15/16) and 'GND' the switched-return end (pads 1/2) correctly
- Mounting: 8x M2 (2.2 mm drill, top+bottom annular pads, electrically isolated) mounting holes at x 75.22/91.97, 2.7-2.9 mm from the board edges, clear of all connector courtyards
- DRC tonight: 0 unconnected items, 0 schematic parity issues — every connector net is electrically complete; BOM MPN/manufacturer/footprint agree with the netlist for J1-J8, U1 (FSUSB63UMX), and the connector footprints match their parts (SH 1.0 mm, PH 2.0 mm SM4, VH 3.96 THT, Milli-Grid 2 mm, TBLH 3.5 mm, GCT USB4110)
- eFuse ILIM commit verified real: R48 = 127R (was 100R per prior review), and the crystal-leg 24 nH inductor reversion is consistent with the live netlist (no such L in the connector domain nets)


**Checks not completed (connectors):**

- Servo harness signal map: J3 (16-pin Milli-Grid) to servo-adapter J1 (6-pin TE 826576-6) is cable-defined and no harness document exists in the repo — cannot verify which EXP pins drive Servo1-4, nor that the harness avoids the strap pins EXP_09/10/11 (J3.10-12). Recommend adding a harness doc before first flight.
- PX1105R module behavior with RXD2 floating or contended (J1.5) — module datasheet not fetched; severity of the pin-5 dual-personality finding assessed from netlist topology only.
- JST SH/PH/VH per-contact ratings (1 A / 2 A / 10 A) and Molex Milli-Grid 87832 2 A rating taken from vendor catalog values and the prior review's fetched citations, not re-fetched tonight (jst.fr/product PDFs 403'd in prior review too).
- 'Naked' GoPro (Hero 10 Black) input voltage tolerance on J4 — configuration not recorded in the repo, cannot verify 2S compatibility for that camera option.
- TPS2121 exact input-capacitor wording — TI datasheet not re-fetched tonight (onsemi/TI PDF endpoints 403/timeout); the no-VBUS-cap finding rests on the netlist fact plus standard TI power-mux application guidance.
- Vibration qualification of friction-lock connectors: no spec or test data in the repo to check against; reported as judgment-level risk only.


## A.architecture — System architecture

*(verify pass for this domain did not run — session limit; tags below are [unverified] except where the main loop spot-checked)*

### HIGH [unverified] — Flight-critical P4 power is chained through live S3 firmware: any S3 reset power-cycles the flight computer mid-flight
*architecture / power sequencing — confidence high*

V_MCU_SWTCH (P4 + all sensors + pyro continuity + buzzer) is gated by U22 TPS22918 whose ON pin is driven only by S3 GPIO7 (POWER_SWITCH) against a 100 k pulldown, with QOD active discharge enabled. Any S3 reset — watchdog, brownout, BLE/Wi-Fi stack panic, OTA restart — tristates GPIO7, R68 pulls ON low, QOD discharges VPP in ~tens of ms, and the P4 hard-resets for the 0.3–1 s it takes S3 to reboot and re-assert the pin. A radio-MCU software fault therefore erases the flight computer's state during flight (missed apogee/main deploy window).

**Evidence:** netlist.net: POWER_SWITCH :: R68.2 | R71.2 | U15.12(GPIO7); Net-(U22-ON) :: R71.1 | U22.3; R68.1 -> GND (100 k); U22.5(QOD) tied to U22.6(VOUT) = V_MCU_SWTCH; no latch, no P4-side feedback, no default-on option. S3 GPIO7 is not a strapping pin and has no default pull-up, so during S3 reset the pin floats and R68 wins. Prior review only flagged the programming aspect (L13); the in-flight coupling is unaddressed.

**Fix:** Add a hardware hold: e.g. P4 GPIO OR-ed into U22 ON via diode/resistor so a booted P4 keeps its own rail up (S3 retains kill authority via a stronger pulldown or dedicated kill FET), or make U22 default-ON (pull-up to +3V3) with S3 able to force it off, keeping the pad-safety interlock via a different arming step.

**Sources:** TPS22918 datasheet behavior (ON threshold, QOD); live netlist

### HIGH [unverified] — Brownout/hold-up story does not close in flight configuration: 3–5 ms of hold-up vs >=20 ms (UVLO) / ~112 ms (fault) eFuse outages, and the UVLO sense has no deglitch
*power architecture / brownout coherence — confidence high*

The single 330 uF hold-up (C56) carries the logic domain for only ~3.4–4.7 ms at the real flight load (~0.5 A on +3V3 with P4+BLE up; 47 ms only in S3-only pad idle, which is what the ECO sized for). Any eFuse OFF event exceeds that: a UVLO dip recovery costs turn-on delay + dVdt ramp (C51 10 nF -> ~0.46 V/ms -> ~18–22 ms to 8.4 V), and an overcurrent fault costs ~92 ms retry delay (C46 2.2 nF) plus the ramp. Meanwhile the EN/UVLO divider (R44 1 M / R45 210 k, thresholds 6.91 V on / 6.34 V off) has NO filter capacitor and all bulk capacitance sits downstream of the eFuse (VBAT_CON carries only C41 1 uF + C42 100 nF), so a millisecond pack-lead sag below 6.34 V — plausible near end of discharge under servo load — turns the whole board off and guarantees a full dual-MCU reset. This is prior finding H2, still unfixed in the live files, now quantified with the live cap values.

**Evidence:** netlist.net: Net-(U19-EN{slash}UVLO) :: R44.1 | R45.2 | U19.6 (no C); VBAT_CON members: C41 1 uF, C42 100 nF, CR2, R44, R72, U19 IN pins, U23; hold-up: V_MCU_2S :: C56 TCJE337M016R0050 330 uF + C59 1 uF. TPS25982 datasheet: VUVLO(R)=1.2 V, VUVLO(F)=1.1 V -> 6.91/6.34 V with 1 M/210 k; dVdt table 'CdVdt=6800pF -> 0.68 V/ms'; retry Table 7-5 '2.2nF -> 91.7ms'; ITIMER 10 nF -> 0.98 V x 10 nF / 2.1 uA = 4.7 ms blanking. Energy math: 0.5*330u*(8.4^2-3.5^2)*0.9 = 8.7 mJ; at ~1.83 W buck input power -> 4.7 ms (3.4 ms from 7.4 V). NRETRY C45 1 uF computes to ~1815 retries, beyond the datasheet's 1024 quantization bucket (undefined region, prior L38).

**Fix:** Add an RC deglitch on EN/UVLO (>=100 ms as schematic-review.md 3.5 specified), add a second 330 uF on V_MCU_2S (the ECO's own escape clause for loads >200 mA), and consciously re-derive ITIMER/dVdt/RETRY/NRETRY values (record in ECO). Optionally add ~100 uF upstream of the eFuse on VBAT_CON to filter lead inductance sags.

**Sources:** TI TPS25982 datasheet (SLVSEI3D) sections 6, 7.3.1, 7.4, Table 7-5; live netlist; power-eco.md Change 1/2

### HIGH [unverified] — Worst-case 4-servo stall exceeds the eFuse current limit and the servo return path rating; the ILIM change 100R->127R reduced margin in the wrong direction for this load
*power tree closure / current capacity — confidence medium*

Sum of downstream worst case (4 servo stall 8–12 A + logic ~0.5 A + camera ~0.3 A + LoRa TX ~0.6 A) reaches or exceeds ILIM with R48=127 R: ~11.6 A typical, ~10.3–10.7 A min-corner. A simultaneous stall current-limits the eFuse, collapses VBATT for the 4.7 ms ITIMER blanking, then faults -> 92 ms off + ~20 ms ramp -> full board reset (see hold-up finding). Independently, the servo return path is undersized: all stall current returns through one PMPB14XNX (Q8, 8.1 A continuous rating), 2x Molex Milli-Grid contacts (~2 A each), a 0.8 mm B.Cu trace, and a single ground pin on the adapter's J1 header.

**Evidence:** netlist.net: Net-(U19-ILIM) :: R48 127 R; TPS25982 datasheet ILIM at RILIM=100 R = 13.56/14.71/15.66 A at 25 C (K~1471 A*ohm -> 127 R = 11.6 A typ). Servo path: J3.15/16 = VBATT feed, J3.1/2 -> Q8 (PMPB14XNX 40 V / 8.1 A / 15 mOhm, DFN2020MD) -> GND; live PCB: Net-(J3-Pad1) tracks 0.8 mm B.Cu, 9.2 mm total; db_servo-adapter.net: single GND pin J1.1 for all four servo returns. power-eco.md Change 2 already warned: 'if sustained 4-servo stall is real, step to the TPS25983 (20 A)'. Prior H5/H16/M6/M26/M38 all still open in live files.

**Fix:** Decide the real worst-case servo load. Either (a) bound it in firmware/mechanics and document that simultaneous stall is precluded, or (b) move to TPS25983 (20 A), parallel a second return FET, use 4 power + 4 ground contacts on J3, and widen the return copper. Do not leave ILIM below a load the system is expected to survive.

**Sources:** TI TPS25982 datasheet; Nexperia PMPB14XNX (Newark/element14: 40 V, 8.1 A, 15 mOhm, DFN2020MD); live PCB tracks; prior review H5/M6

### HIGH [unverified] — TPS62152 (U18) PVIN pad 12 is still unconnected — the sole 3.3 V regulator has half its power-input pins floating
*power tree / regulator wiring — confidence high*

Prior review flagged this five separate ways (H1/H10/H15/H17/H22); the fix did not land. The live netlist still shows U18 pad 12 in the unconnected list and ERC still errors on it. The entire logic-domain input current flows through the single remaining PVIN pin 11, against the datasheet requirement to tie both PVIN pins to the input; this degrades the power stage impedance and pin current margin on the one regulator every MCU depends on.

**Evidence:** netlist.net line 27078: (name "unconnected-(U18-PVIN-Pad12)"); erc.json: 'pin_not_driven | Symbol U18 Hidden pin 12 [PVIN, Input]'; Net-(U18-AVIN) contains only U18.10 (AVIN), U18.11 (PVIN), U18.13 (EN), C43, L5.

**Fix:** Tie U18 pad 12 to Net-(U18-AVIN) in central power schematic and re-pour the PCB connection; verify both PVIN pins share the input copper per TPS62152 datasheet layout guidance.

**Sources:** erc.json; netlist.net; TI TPS62152 datasheet pinout (PVIN pins 11,12); prefab-review-2026-07-30.md H10/H15

**Disposition (2026-08-05): FIXED & VERIFIED in-session.** U18 pad 12 now carries `Net-(U18-AVIN)` (symbol pin 12 stacked on pin 11; AVIN pour refilled over the pad — no new tracks). Netlist, ERC (`pin_not_connected` 47→46, `pin_not_driven` 1→0) and DRC (0 unconnected, no new violations) all re-verified. See NB-3.

### HIGH [unverified] — Load-shedding creates backfeed: 12 EXP servo lines run directly from P4 GPIOs to the ground-switched J3 branch with no series resistors
*load shedding / GPIO protection — confidence high*

All four peripheral branches are low-side switched (returns through Q1/Q7/Q8/Q10 to GND) while their VBATT feeds stay hot. For the servo/expansion branch this is unprotected: EXP_01..EXP_12 connect P4 GPIOs straight to J3 pins with zero series impedance. If firmware sheds the servo branch (SERVO_ACT low) while servos are drawing current, the adapter's floating ground diverts return current through the servo PWM signal pins into the P4's I/O clamps — amp-scale, destructive. Even statically off, the adapter ground floats toward VBATT through servo internals and injects into 12 P4 pins (abs-max pad voltage 3.6 V exceeded), also back-powering the off V_MCU_SWTCH domain. GNSS/LoRa/camera have the same topology but are current-limited by 1 k series resistors (mainboard R30/R32 for camera; daughterboard-side R7/R8/R9 and R77/R78) so they are a lesser instance.

**Evidence:** netlist.net: EXP_01 :: J3.3 | U17.87(GPIO45) ... EXP_12 :: J3.9 | U17.64(GPIO33) — two-node nets, no resistors; Net-(J3-Pad1) :: J3.1/2 | Q8 drains; Q8.4/8 -> GND; J3.15/16 = VBATT (always hot); db_servo-adapter.net: Servo1..4 :: J1.5..J1.2 direct to servo signals, LiPoPos hot. Prior H6/H20/H23/M5/M14 open; the '1k series R on cross-board UARTs' commit covered only the three UART ports, not EXP.

**Fix:** Minimum: 1 k series resistors on all 12 EXP lines (or a bus switch powered from the switched domain). Better: switch the servo branch high-side, or interlock SERVO_ACT shedding in firmware to only occur with PWM outputs tristated and servos unloaded — and document that J3 payloads must tolerate floating ground.

**Sources:** live netlist; db_servo-adapter.net; Espressif ESP32-P4 abs max ratings

### MEDIUM [unverified] — Off peripherals still back-power the 'cold' P4 domain through their 1 k UART paths — ECO Change 6 goal not met with peripherals attached, VPP floats at ~1 V
*load shedding / sneak power — confidence high*

With battery in and VPP off (pad, S3-only), a connected camera (and GNSS/LoRa on the S3 side) has hot VBATT and floating local ground; up to ~5 mA per 1 k signal line flows through the peripheral's UART pins into MCU clamps. For the camera this injects ~10 mA into the OFF V_MCU_SWTCH rail; the TPS22918 QOD (~225 ohm) turns that into a standing ~0.8–1.5 V float on VPP, holding sensors and P4 in an undefined partially-powered state for hours on the pad and adding an unbudgeted pack drain. The inter-MCU ESP_* SPI also still has no series protection (firmware tristate note from ECO Change 6 remains the only mitigation, prior M3).

**Evidence:** netlist.net: Camera_RX/TX :: R30/R32 1 k -> J4.3/4; J4.2 = VBATT unswitched; J4.1 = Q7 switched return; U22.5 QOD tied to VOUT; ESP_CS/SCLK/SDI/SDO two-node nets U15<->U17, no resistors, no pulls. power-eco.md Change 6: 'the P4 domain is genuinely cold at first S3 power-up' — not true with a camera plugged in.

**Fix:** Accept-and-document or fix: a small bleed keeps VPP low (QOD already helps; the injected current is the problem), better is high-side switching for the camera branch or open-drain level isolation. Keep the ECO Change 6 firmware rule (hold ESP_* SPI hi-Z until VPP good).

**Sources:** live netlist; TPS22918 datasheet QOD resistance; power-eco.md Change 6

### MEDIUM [unverified] — Power-fault telemetry dead-ends: eFuse PG and buck PG reach no MCU, so auto-retry brownout events are invisible to firmware
*observability / ECO verification — confidence high*

PG_RAIL (U19 pin 13) terminates at pull-up R59 and U18's PG at R49 — neither reaches an MCU pin, so firmware cannot log or react to eFuse current-limit/retry events or buck faults, despite the ECO pin map explicitly promising 'PG_RAIL -> MCU: main-power-good / fault telemetry'. With the auto-retry eFuse, a mid-flight trip would be indistinguishable from a firmware crash in post-flight analysis. INA230 telemetry exists but only on the S3 I2C and won't capture a 92 ms outage (it loses power too).

**Evidence:** netlist.net: PG_RAIL :: R59.2 | U19.13 (two nodes only), R59.1 -> +3V3; Net-(U18-PG) :: R49.1 | U18.4 only. power-eco.md Change 2 pin table row PG. Prior M7/M32/L29/L45 all still open in live files.

**Fix:** Route PG_RAIL to a spare S3 GPIO (always-on domain member is the right observer) before fab; one wire + existing pull-up. Same for U18 PG if a pin remains.

**Sources:** live netlist; power-eco.md Change 2

### MEDIUM [unverified] — eFuse off-state quiescent current is 204-300 uA, not the <1 uA the ECO's 'no EN latch needed' rationale assumed — pack keeps draining after low-voltage cutoff
*battery protection / ECO verification — confidence high*

power-eco.md Change 2 waived the EN latch partly because 'off-state draw is only the divider (~5 uA) + eFuse Iq (<1 uA)'. The TPS25982 datasheet specifies IQ = 204 uA typ / 300 uA max in exactly the UVLO-off state (VSD < VEN < VUVLO), plus ~7 uA divider. After a 6.34 V cutoff a small 2S pack (~650 mAh, ~20% SoC remaining) continues draining at ~0.2-0.3 mA and reaches over-discharge damage in roughly 2-4 weeks if left connected. The protection buys weeks, not indefinite storage.

**Evidence:** tps25982.txt line 362-363: 'IQ IN Quiescent Current VEN >= VUVLO(R) 800/1200 uA; VSD < VEN < VUVLO 204/300 uA'. Divider R44+R45 = 1.21 M -> 7 uA at 8.4 V. power-eco.md Change 2 'EN latch — decided OPTIONAL' paragraph contains the incorrect <1 uA figure.

**Fix:** Correct the ECO text and add an operating rule (physically disconnect pack for storage), or revisit the EN latch decision. Also note enabled-state Iq is ~0.8-1.2 mA — include it in the idle power budget.

**Sources:** TI TPS25982 datasheet electrical characteristics; power-eco.md

### MEDIUM [unverified] — P4 ROM boot log (UART0 = GPIO37/38) transmits onto servo channel EXP_10 at every reset; strapping pins GPIO34/37/38 are exposed on J3
*boot-state safety / interconnect — confidence high*

ESP32-P4 UART0 defaults to TXD=GPIO37, RXD=GPIO38; in this design GPIO37=EXP_10 (J3.11) and GPIO38=EXP_09 (J3.12). Every P4 power-up, reset, and brownout-recovery, the ROM prints its boot log as 115200-baud serial onto a servo/expansion signal, and download-mode listens on another. Additionally 3 of the 5 strapping pins (GPIO34/37/38) route bare to the external connector, so an attached payload that drives them low with GPIO35 low can force Joint Download mode (boot-mode corruption). GPIO35 itself is protected (R53 100 k pull-up to V_MCU_SWTCH, button via R54 1 k), so the default SPI-boot path is safe when nothing external interferes.

**Evidence:** p4-hdg.txt: 'The UART0_TXD pin is GPIO37, and the UART0_RXD pin is GPIO38'; 'GPIO34, GPIO35, GPIO36, GPIO37, and GPIO38 are strapping pins'; boot-mode table: SPI Boot = GPIO35=1, others any value. netlist.net: EXP_10 :: J3.11 | U17.69(GPIO37); EXP_09 :: J3.12 | U17.70(GPIO38); EXP_11 :: J3.10 | U17.65(GPIO34). Prior M1/M10/L12 still open.

**Fix:** Reassign the four servo PWM channels to non-strapping, non-UART0 GPIOs (P4 has spares among EXP set), or burn the UART_PRINT_CONTROL eFuse and document that J3 payloads must not load GPIO34/37/38 at boot. Servos tolerate serial garbage poorly at best (jitter/twitch).

**Sources:** Espressif ESP32-P4 Hardware Design Guidelines (local p4-hdg.txt); live netlist

### MEDIUM [unverified] — A single wrong internal pull-up enable arms the pyro chain: PYRO_ARM gate rises to ~2.3 V (> CSD16323Q3 Vth) and fire-channel DTC123J margin is thin
*boot-state safety / pyro — confidence high*

Hardware defaults are safe (verified: P4 pyro GPIOs 6/9/11/13/16 have no reset-default weak pull-up; all gates have discrete pulldowns/pull-ups to the safe state; DTC123J base networks ignore a floating pin). But the design's software margin is one bit: enabling the ~45 k internal WPU on GPIO16 puts (45k vs R22 100k pulldown behind R21 100R) about 2.27 V on the arm-FET gate — above CSD16323Q3's max threshold — turning the low-side arm ON; a WPU on a fire GPIO drives the DTC123J (R1 2.2k/R2 47k) into marginal conduction. Firing still needs both a fire and the arm path, but two GPIO-config bugs (or one misused gpio_config with a pin mask) defeat the interlock while the pad-power interlock (U22 off) is the only remaining barrier — and it is released in flight.

**Evidence:** netlist.net: PYRO_ARM :: R21.2 | U17.17(GPIO16); Net-(U9-GATE) :: R21.1 | R22.1(100k->GND) | U9.4; PYROx_FIRE two-node nets to DTC123J bases Q3/Q4/Q5/Q6; fire-FET gates pulled to V_CAP via 10 k R15/R16/R17/R23. p4-hdg.txt pin table shows WPU default only on GPIO35. Divider math: 3.3 x 100k/(45k+0.1k+100k) = 2.27 V; CSD16323Q3 Vgs(th) max ~1.1 V. Prior H21 re-verified against live files.

**Fix:** Strengthen R22 to 10 k (WPU then yields ~0.6 V, below threshold) and consider 10 k (not 47 k-class) external base-emitter shunts on the fire lines; add a firmware GPIO-init assertion test. Cheap insurance for a deployment channel.

**Sources:** live netlist; TI CSD16323Q3 datasheet threshold; Rohm DTC123J internal resistor values; ESP32-P4 HDG pin defaults

### MEDIUM [unverified] — Pyro energy store has zero observability and a single-point charge path; USB bench power covertly charges it to ~3 V
*flight-critical SPOF / pyro energy — confidence high*

V_CAP (C12 10 mF) has no voltage divider to any ADC — firmware cannot verify the deploy energy store is charged before launch, and an R20 failure (single 1206 resistor, the only charge path) would only be discovered as a failed deploy. Conversely on USB-only power (VBATT dead), V_MCU_SWTCH leaks through the four 49.9 k continuity pull-ups and the fire-FET body diodes into V_CAP, charging 10 mF to ~2.5-3 V (~30-45 mJ) over minutes — enough that a 'safe, battery-out' bench test that toggles ARM+FIRE can pop a sensitive e-match (prior M33 re-verified).

**Evidence:** netlist.net: V_CAP members are only caps, R20, fire-FET sources, and the four 10 k gate pull-ups — no sense divider; R8/R10/R12/R18 (49.9 k) from V_MCU_SWTCH to PYROx_EXT, fire FETs U6/U7/U8/U10 drain->source body diodes into V_CAP; charge path VBATT -> R20 150R (CRCW1206...HP) alone.

**Fix:** Add a high-value divider (e.g. 1 M/240 k) from V_CAP to a spare P4 ADC for pre-arm verification, and a bleed or interlock note for USB bench work. Both are trivial additions before fab.

**Sources:** live netlist; prior M33; e-match all-fire energy ranges

### LOW [unverified] — USB input (TPS2121 IN1) has no local capacitance at all
*power tree / input filtering — confidence high*

Net-(J6-VBUS) contains only the connector, TVS CR3, two divider resistors and U21 IN1 — zero capacitance. Hot-plug and cable-inductance transients hit the mux input unfiltered, and the 5 us fast switchover has no source-side reservoir; TI's TPS2121 reference circuits include input caps on both inputs. Prior L20/L37 still open in the live files.

**Evidence:** netlist.net: Net-(J6-VBUS) :: CR3.2 | J6.A4/B9 | J6.B4/A9 | R51.2 | R62.2 | U21.7 — no C. TPS2121 datasheet application schematics show CIN on IN1/IN2.

**Fix:** Add 1-10 uF X7R (>=16 V for surge margin) at U21 IN1.

**Sources:** live netlist; TI TPS2121 datasheet application section

### LOW [unverified] — ECO value drift left undocumented: hold-up companion ceramic 1 uF (ECO: 10 uF), eFuse ITIMER/dVdt/NRETRY caps changed, ILIM changed — with no recorded recalculation
*ECO verification / documentation — confidence high*

Live values differ from the 'IMPLEMENTED & VERIFIED' record in power-eco.md: C59 = 1 uF vs the specified 10 uF companion; ITIMER C50 = 10 nF (ECO 4.3 nF -> blanking 2 ms became 4.7 ms); dVdt C51 = 10 nF (ECO 8.2 nF); NRETRY C45 = 1 uF (ECO 560 nF, now computing past the 1024-retry quantization table); ILIM R48 = 127 R (ECO 100 R). Each change is individually defensible but none is recorded, and two (ITIMER, ILIM) interact with the servo-stall/brownout findings above. The ECO also names mux input assignments backwards (text says battery on IN1) versus the live IN1=USB/IN2=battery.

**Evidence:** compdb.json values C50/C51/C45/C46/R48/C59 vs power-eco.md Change 1/2 pin table; TPS25982 Table 7-4/7-5 quantization; netlist U21.7=IN1=VBUS, U21.2=IN2=VBATT.

**Fix:** One documentation pass over power-eco.md recording final values and the reasoning; bump C59 to 10 uF or note why 1 uF suffices.

**Sources:** power-eco.md; compdb.json; TI TPS25982 datasheet

### LOW [unverified] — INA230 IN+ sense is improved by tonight's re-route but still not Kelvin at the shunt pad
*telemetry accuracy / layout — confidence medium*

The IN+ sense trace now taps the VBAT_Terminal copper zone at ~(81.4, 129.8), roughly 2.3 mm of current-carrying zone away from R72's pad at (79.57, 131.28) and between it and J8's pad at (84.73, 129.55). The added zone resistance in the sense loop is on the order of 0.1-0.3 mOhm against a 2 mOhm shunt: pack-current telemetry reads ~5-15% high at load. Better than the prior state (M24/M28/M31 measured ~1-1.5 mOhm from tapping the connector pad) but not a true Kelvin connection.

**Evidence:** rocket-computer.kicad_pcb: R72 at (79.57,130.37) rot -90, pad2 (VBAT_Terminal) at (79.57,131.28); U23.13 at ~(79.1,127.8); VBAT_Terminal 0.1 mm sense route B.Cu (79.20,127.77)->(79.64,127.77) then In2 (79.7,128.57)->(80.95,129.82)->(81.38,129.82) terminating mid-zone; B.Cu zone bbox 79.3-86.8 x 127.5-132.4 carries the battery current.

**Fix:** Land the 0.1 mm sense trace directly on R72 pad 2 (and verify IN- similarly lands on pad 1).

**Sources:** live rocket-computer.kicad_pcb coordinates; prior M24/M28/M31

### INFO [unverified] — USB-only operation matrix: works as intended for bench, but a virgin board cannot program the P4 until working S3 firmware exists, and worst-case draw exceeds the USB default budget
*USB operation / bring-up — confidence high*

On USB only (eFuse off, VBATT dead): powered = S3, its flash/NAND, INA230 (reads 0 V pack), and — once S3 raises GPIO7 — the full P4 domain, sensors, buzzer, and pyro continuity sensing (65 uA through e-matches, safe). Dead = GNSS, LoRa, camera, servo branch, pyro firing energy (by design, except the sneak-charge finding). Consequences: (1) factory bring-up order is forced — S3 must be flashed first (S1 slide selects S3 via FSUSB63 SEL0) and must run code asserting POWER_SWITCH before the P4 can even enumerate (prior L13, no hardware override); (2) with P4+radio active the 5 V draw (~0.6-0.9 A) exceeds the 500 mA default USB budget with only 5.11 k CC pulldowns (no BC1.2/PD negotiation); (3) with both sources present the mux boundary sits at VBUS = VBATT/2.007 (~4.19 V at full pack) — a weak USB source can sit at the PR1/CP2 comparator boundary and toggle sources; benign due to seamless switchover but it will confuse INA230 telemetry.

**Evidence:** netlist: U21 IN1=VBUS, IN2=VBATT; PR1 divider R62/R63 5.11k/5.11k, CP2 divider R60/R61 15.4k/5.11k; TPS2121 datasheet: 'If the voltage on PR1 is higher than CP2, then IN1 is powering the output', VREF 1.06 V; CC pulldowns R41/R47 5.11k; continuity source R8 49.9k from V_MCU_SWTCH with return via R73 1k.

**Fix:** Accept, but add a bring-up note to the repo; consider a hardware pull-up option (DNP resistor) on POWER_SWITCH for bench override of the S3 interlock.

**Sources:** TI TPS2121 datasheet section 10.2.3; live netlist

### INFO [unverified] — Single-point-of-failure inventory for the deploy chain (residual, mostly inherent)
*flight-critical SPOF — confidence high*

Deploy chain SPOFs in the live design: J8 battery connector (a mid-flight disconnect kills logic in <5 ms even though V_CAP still holds ~350 mJ — the pyro store cannot sustain the logic by design); U19 eFuse (auto-retry mitigates transient faults; NRETRY exhaustion after ~94 s of continuous retrying latches the board off); U21 mux, U18 buck, U22 VPP switch, R20 charge resistor (each single, unmonitored); U9 arm FET is a single common-return part — one shorted FET removes the arm layer, one open FET removes all four channels; the common e-match return J2.5 means one harness-side short to airframe ground bypasses the low-side arm interlock entirely (prior M9/M13 still open — silk still implies GND). None of these is newly introduced; listed for the record with the two actionable ones (V_CAP monitor, PG telemetry) filed above.

**Evidence:** netlist: PYRO_GND :: J2.5 | R73 | U9 drains; single U9 in return; V_CAP single charge path R20; hold-up energy math in the brownout finding.

**Fix:** For drogue/main the two-FET (fire+arm) plus S3 power interlock is reasonable; implement the V_CAP monitor and PG telemetry findings, correct the J2.5 silk ('PYRO RET', not GND), and document the airframe-short failure mode. For any airstart use, prior review C-4's physical interlock requirement stands.

**Sources:** live netlist; prefab-review-2026-07-30.md M9/M13/C-4


**Affirmatively verified clean (architecture):**

- Power tree closes at every stage except the two flagged (servo-stall vs ILIM; hold-up vs eFuse outage): pack->J8 JST VH B2P-VH (10 A, replaces the 2 A JST PH — prior B2/H3/H4/H19 FIXED); R72 2 mOhm now has MPN CSSH0805FT2L00 (0.5 W, 0.27 W at ILIM — prior M25/L3 resolved); eFuse ILIM 11.6 A > any non-stall load; TPS2121 ILIM (R64 18.7k -> 4.6/5.2/5.8 A per datasheet) >> logic demand; TPS62152 1 A vs ~0.6-0.8 A worst coincident +3V3 peak (S3 BLE TX + P4 heavy + buzzer + NAND); TPS22918 2 A vs ~0.35 A VPP; TLV62569 2 A vs ~0.5 A P4 core; L5/L6 (VLS3012CX-2R2M-1, ~1.3-1.5 A) within saturation margin; R20 150R 1206-HP 0.75 W vs 0.47 W peak charge
- eFuse configuration verified against TPS25982 datasheet: UVLO divider R44 1M/R45 210k -> 6.91 V on / 6.34 V off (VUVLO(R)=1.2 V, VUVLO(F)=1.1 V), sensing pack side (VBAT_CON) upstream of the FET as the ECO intended; OVLO variant '24' = 16.9 V appropriate for 2S; retry 2.2 nF = 91.7 ms per datasheet Table 7-5; CR2 negative-clamp orientation consistent (pin1=cathode on VBAT_CON, matching series usage of the same part on the LoRa DB); inrush during dVdt ramp ~0.2 A << ILIM
- TPS2121 mux configuration verified: IN1=USB/IN2=VBATT with XCOMP priority — PR1(VBUS/2) vs CP2(VBATT/4.01) gives USB priority whenever VBUS > ~VBATT/2; OV1 rejects USB >6.35 V, OV2 rejects packs >9.54 V (blocks accidental 3S); 5 us reverse-current blocking is the hold-up mechanism and is valid per datasheet; SS 1 uF soft-start makes C56 inrush trivial (~30 mA); USB CC pulldowns 5.11k = correct UFP sink
- ECO Change 1 (VCC hold-up): IMPLEMENTED — C56 TCJE337M016R0050 330 uF on V_MCU_2S (mux OUT), 16 V part at 8.4 V = 53% derating; deviation: companion ceramic 1 uF not 10 uF (filed); bonus C15 330 uF on VBATT not in ECO; hold-up math verified correct for the ECO's stated 50 mA case (47 ms) — the flight-load gap is filed as a finding
- ECO Change 2 (eFuse): IMPLEMENTED with drifted values (filed); all pins verified against the live netlist pin-by-pin including ITIMER/RETRY_DLY/NRETRY/dVdt/LDSTRT-to-GND/IMON-to-GND (IMON/ST tied to GND is datasheet-sanctioned, ERC pin_to_pin noise); -BATT eliminated, battery negative = continuous GND, pyro e-match return now via PYRO_GND->U9/R73->GND as specified
- ECO Change 3 (shedding): all four branches independently firmware-switchable (GNSS=P4 GPIO15/Q1, camera=P4 GPIO32/Q7, servo=P4 GPIO8/Q8, LoRa=S3 GPIO12/Q10) and LoRa is now a true power switch, resolving the ECO's open question; branch bulk lives on the daughterboards (servo adapter 4x100 uF, GNSS DB has TPS62913 regulator with own input caps, LoRa DB has bulk bank) — topology changed to low-side switching, consequences filed as findings
- ECO Change 5 (housekeeping): IMPLEMENTED — pyro return on continuous ground; INA230 across R72 with IN+ pack-side/IN- and BUS on VBAT_CON (correct true-pack measurement, 41 A full scale at 2 mOhm); post-disconnect rail (VBATT) feeds mux, all four peripheral feeds, and pyro charge; U22 ON pulldown R68 100k present so VPP defaults OFF (C-4 interlock condition (a) satisfied)
- ECO Change 6 (sneak power): core item IMPLEMENTED — U11 NAND VCC and R31/R33 WP/HOLD pull-ups verified on +3V3; MRAM U12 removed from the design entirely; all P4-domain pull-ups (R40/R43 mag I2C, R55/R58 inter-MCU I2C, R4/R5 sensor CS, R42 CHIP_PU, R46 flash CS, R50/R53 straps) verified referenced to V_MCU_SWTCH so they die with the domain; U22 QOD tied to VOUT (discharge enabled); residual paths filed
- ECO Change 7 (pyro store): IMPLEMENTED — C12 EKYC160ELL103MM25S 10 mF/16 V, single R20 150R (upgraded to 1206-HP, better than ECO's 0603), R70/R71/R72 refs recycled to unrelated parts (no dangling remnants); V_CAP ceramic bank now pinned to 16 V MPN CL21A226MOQNNNE (prior M34 resolved)
- Boot-state safety table verified for every actuator-relevant GPIO across power-up / reset / brownout-recovery / watchdog: all six switch gates (Q1/Q7/Q8/Q9/Q10/U9) have discrete pulldowns (R7/R25/R29/R28/R35/R22), all four fire-FET gates pull up to V_CAP via 10k, DTC123J stages are immune to floating pins, P4 CHIP_PU has RC (R42 10k + C39 now 1 uF — prior H8/L11 fixed) on the switched domain so P4 stays in reset until VPP is good, S3 CHIP_PU R36+C25 1 uF on always-on rail (prior M2 fixed); no default weak pull-ups on pyro pins per P4 HDG pin table; firmware-crash-with-watchdog returns all actuators to safe; the software-margin caveat is filed separately
- P4<->S3 interconnect verified: SPI ESP_CS/SCLK/SDO/SDI = S3 GPIO1/2/3/4 <-> P4 GPIO18/21/19/20 and I2C ESP_SCL/SDA = S3 GPIO6/5 <-> P4 GPIO22/23 with 5.11k pull-ups on V_MCU_SWTCH; none of the eight pins is a strapping pin on either MCU; no UART crosses between the MCUs; USB demux (FSUSB63, SEL1 tied high, SEL0 via slide switch S1 to GND/pull-up) routes J6 to either P4 native USB (CEN_D+/- GPIO25/24, R77 1M detach pulldown) or S3 native USB (OUT_D+/- via 22R R38/R39 to GPIO20/19); S3 GPIO46 has 10k pulldown (R37), GPIO0 has boot button S2
- Daughterboard interfaces cross-checked against tonight's DB netlist exports: GNSS J1<->DB J3 pin-for-pin (RX/TX/VBATT/switched-GND/PPS) with 1k series on the DB side (R7/R8/R9) and local TPS62913 regulation of raw VBATT; LoRa J5<->DB J6 pin-for-pin with DB-side series resistors (R77/R78) and CM-choke power entry; servo adapter power/GND/4-signal mapping consistent with J3 EXP subset
- In1 and In4 solid GND planes confirmed present in the live .kicad_pcb (multi-layer GND zone spanning F/In1/In3/In4/B), In2 carries the +3V3 / V_MCU_SWTCH / VBATT power splits as declared
- Off-state (post-cutoff) drain paths enumerated: only the 1.21 M UVLO divider (~7 uA), eFuse UVLO-state Iq (filed — larger than ECO claimed), CR2 leakage, and unpowered INA230 input bias remain on the pack; all mux dividers (R51/R52, R56/R57, R60/R61, R62/R63) are downstream of the eFuse and die with it
- Pyro continuity sensing chain verified: V_MCU_SWTCH -> 49.9k (R8/R10/R12/R18) -> PYROx_EXT -> e-match -> PYRO_GND -> R73 1k -> GND gives ~65 mV (match present) vs 3.3 V (open) at the P4 pins through 100k with BAT54 clamps D1-D4 to the VPP rail; during firing the clamp current is a safe ~48 uA; sensing works on USB power and with the arm FET off (R73 bypass), and is dead when VPP is off (P4 also off — consistent)


**Checks not completed (architecture):**

- TPS2121 XCOMP comparator hysteresis at the PR1~CP2 boundary is not specified numerically in the datasheet sections retrieved — the weak-USB-plus-full-pack source-toggling behavior (filed as info) could not be bounded precisely; bench check recommended
- PMPB14XNX pulse SOA could not be fetched (distributor summary only: 40 V / 8.1 A / 15 mOhm); the servo-stall margin on Q8 is computed against the continuous rating — if Nexperia's pulse SOA covers seconds-long 12 A events the severity of that sub-item drops
- Servo worst-case stall current (8-12 A) is taken from the prior review's spec, not from a named servo datasheet — no servo model is recorded anywhere in the repo; the eFuse ILIM finding's probability hinges on this number
- RunCam camera model and its input-voltage range are not recorded in the schematic (prior M11) — could not verify 6.4-8.4 V compatibility of the J4 branch
- ESP32-P4 full datasheet 'Boot Configurations' section (exact GPIO36/37/38 download-mode combinations and per-pin reset pull states beyond the HDG pin table) was not re-fetched; boot-safety conclusions rest on the local p4-hdg.txt pin table
- In2 VBATT plane neck widths (prior M23 camera-branch 0.7 mm neck) were not re-measured polygon-by-polygon after tonight's re-route — only zone extents were verified; a targeted DRC-style cross-section pass on In2 is still owed
- S3 GPIO7 reset-state (pull-up vs floating) taken from general ESP32-S3 behavior (non-strapping pins reset to input, no pull); if GPIO7 had a default WPU the S3-reset->P4-power-loss finding would partially mitigate — Espressif S3 datasheet pin-state table not re-fetched to confirm


## A.layout-power — Layout — power delivery

*(verify pass for this domain did not run — session limit; tags below are [unverified] except where the main loop spot-checked)*

### BLOCKER [unverified] — Battery feed into eFuse necks to 0.45 mm at U19 pin 16 — entire pack current through a ~1.3 A bridge
*pack input -> shunt -> eFuse copper — confidence high*

All battery current (logic + 4 servos + camera + LoRa + GNSS + pyro charge) crosses a single 0.45-0.53 mm wide B.Cu bridge where the VBAT_CON zone feeds U19's IN pins; at IPC-2152-class 1 oz numbers that is ~1.34 A for 10 C rise, so normal flight load (2.5-4 A) runs it at 35-90 C rise and a servo-stall event (6-10 A, still below the 11.6 A eFuse limit) chars the battery input feed before any protection acts.

**Evidence:** Min-cut y-scan of the VBAT_CON connected component (live rocket-computer.kicad_pcb, parsed tonight): y=133.50 B=0.68 mm, y=133.87-134.00 B=0.45 mm (single run x76.83-77.26, aligned with U19 pad 16 at (77.01,133.93)), y=134.37 B=0.65 mm; south zone is 2.8 mm at y=132.5 and the pad-25/EP region north of y=134.5 is 1.48 mm B + 1.53 mm F, so 0.45 mm is the sole crossing. F.Cu VBAT_CON island (x76.6-78.1, y134.5-137.7) is reachable only through the 5 vias on the EP, not from the south, so it cannot bypass the neck. Path context: J8.2 (B2P-VH) -> B zone (min-cut 1.48 mm at x=80.31) -> R72 2 mΩ -> this neck -> U19 IN. Prior review measured 'R72.1->U19 IN corridor avg 2.3 mm' with worst battery-path neck 1.0-1.1 mm at R72 — tonight's re-route made this point WORSE.

**Fix:** Rebuild the VBAT_CON pour so the south zone connects to pads 1/2/3/16/25 over the full pad-row width (>=2.5-3 mm continuous), and add a parallel F.Cu path with 4-6 extra vias south of the EP so both sides carry current. Re-run the min-cut scan; target >=3 mm-equivalent 1 oz on the battery input at every cross-section.

**Sources:** IPC-2221/2152 ampacity approximation I=0.048*dT^0.44*A^0.725 (external); TI TPS25982 datasheet SLVSEI3D (RILIM equation p. 'RILIM = 1460/(IILIM-0.11)')

### HIGH [unverified] — In2 VBATT corridor still 2.25 mm at 0.5 oz — sole feed for servos+camera+LoRa, and slightly narrower than at the July 30 review
*VBATT distribution copper — confidence high*

The only path from the eFuse to every north-side load (J3 servo pins 15/16, J4.2 camera, FL2 LoRa, FL1 GNSS) is the In2 inner-layer corridor; its min-cut is 2.25 mm at two places, worth ~1.3 A at 10 C rise on JLC-standard 0.5 oz inner copper (~2.2 A if 1 oz inner is actually bought), against ~2-3.5 A normal concurrent load and 6-10 A servo-stall worst case that the 11.6 A eFuse setting will not interrupt; inner-layer overheating is invisible and unrepairable.

**Evidence:** Min-cut y-scans of the VBATT component: y=105.27 total 2.25 mm (all In2, right-edge column x91.64-94.48, squeezed by the mounting-hole antipad); y=120.99 2.70 mm; y=124.99 2.25 mm; y=126.99 2.85 mm; corridor is In2-only from y~119 to y~134 (B.Cu VBATT zones exist only at the eFuse end y133.5-138.9 and at J3/C15 ends y101.1-110.1). Prior review (prefab-review-2026-07-30.md line 279-281) measured 2.50 mm at y=105.5 and 3.2-3.45 mm at y120-132 — the re-route shaved both (2.50->2.25 and 3.2->2.25 at y125). The ESP_VDD_HP 0.2 mm In2 track (80.46,113.77)-(86.61,113.77) and the VBAT_Terminal 0.1 mm In2 Kelvin trace additionally slice the corridor region.

**Fix:** Widen the In2 corridor to >=6-8 mm along its whole run, and/or add a parallel B.Cu strap from the eFuse-out zone to the J3-end zone (the B side has room along x88-94); raise the corridor's entry/exit via clusters (see separate finding). Re-route the ESP_VDD_HP In2 diagonal off the corridor waist.

**Sources:** IPC-2221/2152 internal-conductor approximation I=0.024*dT^0.44*A^0.725; JLCPCB standard 6-layer stackup (0.5 oz inner copper); prior review prefab-review-2026-07-30.md finding at line 279

### HIGH [unverified] — eFuse ILIM = 127 Ω -> ~11.6 A typ still cannot protect any downstream element
*eFuse protection coordination — confidence high*

R48=127 Ω sets the TPS259824 current limit to ~11.6 A typical (1460/127+0.11), while the weakest protected elements are the 0.45 mm eFuse-entry bridge (~1.3 A), the 2.25 mm In2 corridor (~1.3 A at 0.5 oz), J3 Milli-Grid contacts (2 A each) and Q8 (8.1 A) — a sustained 5-11 A fault (multi-servo stall, chafed servo harness) passes indefinitely and the copper/connectors become the fuse; the July 30 review's blocker-class coordination gap was only nudged (14.7 A -> 11.6 A), not closed.

**Evidence:** Netlist: Net-(U19-ILIM) = R48.2 + U19.8, R48.1 = GND; bom.csv R48 = 127 Ω RC0402FR-07127RL. TPS25982 datasheet SLVSEI3D: 'RILIM(Ω) = 1460/(IILIM(A) − 0.11)' and EC table (100 Ω -> 14.71 A typ, 182 Ω -> 8.13 A typ); interpolated 127 Ω -> 11.6 A typ. Copper numbers from tonight's min-cut scans (this review); J3 = Molex 878321620 Milli-Grid ~2 A/contact (prior review's cited rating); Q8 = PMPB14XNX 8.1 A.

**Fix:** Pick the real aggregate servo budget (project ECO says micro-servos: ~4-6 A worst) and set R48 accordingly, e.g. 249 Ω -> ~6.0 A typ (4.7 A min over temp still above nominal flight load), then size the copper for >=1.5x that limit. ILIM alone is not sufficient — pair with the two copper fixes above.

**Sources:** TI TPS25982 datasheet SLVSEI3D (ILIM equation and EC table, quoted above)

### HIGH [unverified] — TPS62152 PVIN pin 12 is floating — hidden pin in the schematic symbol never connected
*3V3 buck input — confidence high*

One of the two power-stage input pins of the 3V3 buck (U18 pad 12) is unconnected on the live board, so the full 1 A power-stage input current funnels through pin 11 alone, violating the datasheet pin table and worsening input-loop resistance/inductance; the root cause is a symbol defect (pin 12 drawn with 'hide yes' and type 'input'), so ERC shows it only as an unconnected-pad net.

**Evidence:** netlist.net contains net 'unconnected-(U18-PVIN-Pad12)' with single node U18.12 (PVIN_12); pcb pad U18.12 at (89.53,127.8) carries that net while pads 10/11/13 carry Net-(U18-AVIN); power.kicad_sch symbol pin block: '(pin input line (at -17.78 0 0) (length 5.08) (hide yes) (name "PVIN") (number "12")'. TPS62152 datasheet pin table: '11,12 PVIN I Supply voltage for power stage. Connect to same source as AVIN.'

**Fix:** Unhide pin 12 in the U18 symbol (type power_in), wire it to Net-(U18-AVIN), update the PCB so the AVIN F.Cu zone floods pad 12 (it is adjacent to pad 11 — a zone rule tweak or short 0.3 mm jog suffices).

**Sources:** TI TPS6215x datasheet (TPS62150/51/52/53), pin functions table, quoted above

**Disposition (2026-08-05): FIXED & VERIFIED in-session.** U18 pad 12 now carries `Net-(U18-AVIN)` (symbol pin 12 stacked on pin 11; AVIN pour refilled over the pad — no new tracks). Netlist, ERC (`pin_not_connected` 47→46, `pin_not_driven` 1→0) and DRC (0 unconnected, no new violations) all re-verified. See NB-3.

### HIGH [unverified] — 3V3 buck input capacitor C43 is on the opposite board side through one 0.3 mm via
*3V3 buck input loop — confidence high*

The only input capacitor of the TPS62152 (C43, 22 uF) sits on B.Cu while the buck is on F.Cu; its positive terminal reaches PVIN through a single 0.3 mm-drill via and its ground return needs another 3.7 mm to the nearest GND via, so the high-di/dt input loop spans the 1.6 mm board twice plus ~4 mm laterally — directly against the datasheet's input-cap placement requirement — and, because the 2.2 uH series filter L5 blocks the upstream bulk (C56) at switching frequency, all ~0.5-0.7 A RMS pulsed input current crosses that one via (EMI, input ripple, via stress).

**Evidence:** Pads: U18 PVIN (88.53/89.03,127.8) F.Cu; C43.1 Net-(U18-AVIN) (90.81,127.38) B.Cu; C43.2 GND (92.71,127.38) B.Cu. Net-(U18-AVIN) has exactly 1 via, at (90.4,127.1) (in-pad), and 0 routed segments — connectivity is zone-only. Nearest GND via to C43.2 is (92.87,123.67), 3.71 mm away. Netlist: Net-(U18-AVIN) = {C43.1, L5.2, U18.10/11/13} — C43 is the only cap on the node. TPS62152 datasheet 9.2.2.2.2.2: input capacitor 'should be placed between PVIN and PGND as close as possible to those pins'; layout section: 'Paths conducting the switched load current should be as short and wide as possible.'

**Fix:** Place a 10-22 uF X5R/X7R on F.Cu directly across PVIN pins 11/12 and PGND 15/16 (room exists north of the IC); keep C43 as bulk if desired. Also add the datasheet-recommended 0.1 uF AVIN-AGND.

**Sources:** TI TPS6215x datasheet, sections 9.2.2.2.2.2 (Input Capacitor) and 11.1 (Layout Guidelines), quoted above

### HIGH [unverified] — P4 core rail (ESP_VDD_HP) routed at 0.127-0.2 mm — half of Espressif's minimum 20 mil requirement, mostly on 0.5 oz inner copper
*P4 core rail copper — confidence high*

The VDD_HP tree from L8 to the P4's three supply pins runs 0.2 mm (7.9 mil) on F/In2/In3 with a 0.127 mm In2 branch to VDD_HP_1, versus the ESP32-P4 hardware design guideline's explicit '>= 20 mil' (0.508 mm); at 0.5 oz inner copper a 0.2 mm trace is ~0.22 A at 10 C rise and the ~17 mm run to pin 26 adds ~60-100 mΩ that the FB divider (sensing at the tree root, R74 at (86.87,104.29)) does not compensate — tens of mV static droop plus transient sag on a 0.99-1.3 V rail.

**Evidence:** Segments (live PCB): In2 0.2 mm runs (87.10,109.32)->(83.76,105.98) and (86.61,109.99)->(86.61,113.77)->(80.46,113.77)->(79.17,115.06)->(77.59,115.89) feeding U17.26 at (78.66,116.13); In3 0.2 mm (90.57,106.37)->(87.64,109.32) feeding U17.76; 0.127 mm In2 branch (91.16,113.95)->(87.32,114.48) feeding R76 (0 Ω) -> VDD_HP_1 pin 54; 6 vias total. Espressif ESP32-P4 Hardware Design Guidelines: 'The trace width for the main power supply traces of VDD_HP_0..3 should be at least 20 mil' and 'the external DCDC should be placed close to the chip to ensure that the input, output, and feedback loops are as short as possible.' Per-pin 100 nF + 10 uF caps are present (C70/C73 at pin 26, C64 at 91, C67/C92/C55 near L8) — decoupling itself is per guide.

**Fix:** Rework ESP_VDD_HP as >=0.5 mm traces (prefer F.Cu pours/short hops; the F side under the chip-down P4 fanout has room along y104-116) in a star from L8; widen the pin-54 branch too. If length cannot shrink, move the FB tap toward the pin-26/91 cluster.

**Sources:** Espressif ESP32-P4 Hardware Design Guidelines, PCB layout design section (trace-width rule quoted); ESP32-P4 Series Datasheet v0.7 (VDD_HP operating 0.99-1.3 V)

### MEDIUM [unverified] — Servo return path: single 0.8 mm B.Cu track (~2 A) plus Q8 gated at only 3.0 V
*servo return path and switch FET — confidence high*

The combined return of all four servos (J3.1+J3.2) crosses a single 0.8 mm 1 oz track (~2.0 A at 10 C rise; 6 A stall ~ +90 C) before reaching Q8, and Q8's gate sees only 3.0 V through the R27 1k / R29 10k divider, raising RDS(on) above the 15-18 mΩ spec point (4.5 V); this is essentially unchanged from the July 30 findings (0.85 mm then, 0.80 mm now).

**Evidence:** Min-cut x-scan of Net-(J3-Pad1): worst 0.80 mm B.Cu at x=76.46 (J3.1/J3.2 pads at x78.06 y103.4/107.65 -> Q8 drain cluster at (73.0-75.1,108.7-109.9)); segments: 7x 0.8 mm B.Cu totalling 9.2 mm plus a 3 mm² zone only at the Q8 end. Netlist: Net-(Q8-Pad3) = {Q8.3, R27.1, R29.1}; SERVO_ACT = R27.2 + U17.8 -> Vgs = 3.3*10/11 = 3.0 V. PMPB14XNX: 40 V, 8.1 A, RDS(on) 15-18 mΩ spec'd at Vgs=4.5 V (distributor datasheet listings). Return current then exits via Q8.4/Q8.8 into the 622 mm² B.Cu GND zone with 3 local GND vias plus plane spreading — GND side is adequate.

**Fix:** Duplicate the return as a pour or 2x 1.5 mm tracks (one per J3 pad) into a widened Q8 drain zone; consider dropping R29 to 100 k or driving the gate from a rail so Vgs is at least 4.5 V during ON. The architectural alternative (high-side switching, prior review H) still stands.

**Sources:** Nexperia PMPB14XNX ratings via https://www.nexperia.com/product/PMPB14XN and https://uk.farnell.com/nexperia/pmpb14xnx/mosfet-n-ch-40v-8-1a-dfn2020md/dp/3438541 ; IPC-2221 external-trace approximation

### MEDIUM [unverified] — VBATT layer transitions ride on 4-via clusters (0.3 mm drill) at both corridor ends
*VBATT via transitions — confidence high*

The full downstream load enters In2 through 4 vias at the eFuse and the servo current exits through 4 vias at J3 (2 per pin); at the ~1 A/via continuous guideline for 0.3 mm-drill vias these clusters are at rating for normal load and 1.5-2.5 A/via during servo stall — same structure the prior review flagged, unchanged tonight.

**Evidence:** Via extraction (live PCB): VBATT entry cluster (79.06,133.4/137.2/137.6/138.1); J3.15 exit (91.7,101.4),(92.3,101.4); J3.16 exit (92.1,107.3),(91.7,106.8); camera J4.2 has 2 vias (84.3/84.9,115.9) — acceptable for ~1 A camera; FL1 GNSS 1 via (79.8,118.6) — fine at 0.15 A. All 13 VBATT vias are 0.4/0.3 mm. Local min-cut at the exits is 4.5-5 A (zones fine); the vias are the constraint.

**Fix:** Grow both servo-path clusters to 8-10 vias (room exists inside the B.Cu zones and pads); cheap insurance even if the corridor is also widened.

**Sources:** Common 0.3 mm via ampacity guideline (~1 A at 10 C rise, cross-section ~0.026 mm² at 25 um plating), consistent with prior review's 1-2 A/via criterion

### MEDIUM [unverified] — Declared stackup (1 oz on all six layers) does not match the JLCPCB 6-layer class the board is built for
*stackup / fab data — confidence medium*

rocket-computer.kicad_pcb declares 0.035 mm copper on In1-In4, but JLCPCB's standard 6-layer (filled+capped via) offering ships 0.5 oz (17.5 um) inner copper unless a premium option is explicitly ordered — every inner-layer ampacity and impedance assumption silently halves if the order form and the file disagree, and all critical VBATT distribution lives on In2.

**Evidence:** PCB stackup section: In1.Cu/In2.Cu/In3.Cu/In4.Cu all '(thickness 0.035)' with 0.535 mm cores; project context declares 'filled+capped vias (JLCPCB 6-layer class)'. This review's In2 numbers were computed at 17.5 um (worst case): corridor 2.25 mm -> 1.3 A; at the declared 35 um they would be ~2.2 A.

**Fix:** Decide: either order JLC's 1 oz-inner option and keep the file, or set the file's inner layers to 0.0175 and size In2 copper for 0.5 oz (the corridor fix above should assume 0.5 oz regardless).

**Sources:** JLCPCB multilayer capability page (standard inner copper 0.5 oz); live rocket-computer.kicad_pcb stackup block

### MEDIUM [unverified] — Low-side-switch injection mitigation landed only on the camera UART — GNSS, LoRa and all 12 EXP lines still direct
*switched-return power branches (re-check of prior blocker-class finding) — confidence high*

The claimed '1k series R on cross-board UARTs' fix is incomplete: only Camera_RX/TX got series resistors; GNSS UART, LoRa UART and the 12 servo/EXP PWM lines still connect P4/S3 GPIOs directly to connectors whose local grounds float toward VBATT (6-8.4 V) whenever their return FET is off, which is the default state — the prior review's injection/back-feed mechanism is still live for three of four branches.

**Evidence:** Live netlist: Camera_RX = {R30.1, U17.61} with Net-(J4-Pad3) = {J4.3, R30.2} (1 k in series, fix landed); but GNSS_RX = {J1.1, U17.4}, GNSS_TX = {J1.2, U17.3}, GNSS_RXD2 = {J1.5, U17.2}, LoRa_RX = {J5.3, U15.15}, LoRa_TX = {J5.4, U15.16}, EXP_01..12 = J3.x direct to U17 GPIOs — no series elements. Branch supplies remain hard-wired VBATT (J1.3 via FL1, J5.2 via FL2, J3.15/16, J4.2) with returns switched by Q1/Q10/Q8/Q7 (all gates pulled down, default off).

**Fix:** Finish the mitigation: 1 k series in GNSS_RX/TX/RXD2 and LoRa_RX/TX, 330R-1k in EXP_01..12 (or accept and document the prior review's full high-side rework recommendation for V10).

**Sources:** prefab-review-2026-07-30.md lines 354-358 (mechanism and fix list); live netlist.net exported tonight

### MEDIUM [unverified] — eFuse UVLO still has no RC deglitch — unchanged from prior high finding, and it compounds with the new 0.45 mm input neck
*eFuse UVLO (re-check) — confidence high*

Net-(U19-EN/UVLO) still contains only R44 (1 M) and R45 (210 k) with no capacitor, so the ~6.1-6.3 V falling UVLO threshold sampled at VBAT_CON responds in microseconds to servo-transient sag — the exact in-flight brown-out reset mechanism the respin was meant to fix — and the added IR drop of the 0.45 mm eFuse-entry neck now sits directly inside the sensed path, making nuisance UVLO trips more likely.

**Evidence:** Live netlist: Net-(U19-EN{slash}UVLO) = {R44.1, R45.2, U19.6}; bom.csv R44 = 1 M, R45 = 210 k, no cap on the node. Prior finding prefab-review-2026-07-30.md line 127 (H-class, threshold 6.34 V, hold-up 4-12 ms vs recovery 20-40 ms+). VBAT_CON neck measured 0.45 mm tonight (this review) adds ~15-25 mΩ localized resistance in series with the 2 mΩ shunt ahead of the divider tap.

**Fix:** Add the review-specified RC (e.g., 100 nF-1 uF from EN/UVLO to GND for >=10-100 ms deglitch, checked against TPS25982 EN input behavior) when fixing the input copper.

**Sources:** prefab-review-2026-07-30.md line 127; TI TPS25982 datasheet EN/UVLO section

### LOW [unverified] — USB VBUS routed 26.7 mm at 0.4 mm on 0.5 oz In2
*USB power copper — confidence high*

Net-(J6-VBUS) runs 0.4 mm wide for 26.7 mm on In2 (plus 16.9 mm on F, 5.9 mm on B), ~0.37 A at 10 C rise for the inner portion and ~90 mΩ total — at the realistic ~0.6 A USB-powered draw that is ~20-25 C local rise and ~60 mV drop; adequate but with no margin toward USB-C's 1.5 A expectation, and the track slices the +3V3 In2 plane.

**Evidence:** Segment inventory (live PCB): Net-(J6-VBUS) = 37 segs, In2 0.4 mm 26.7 mm total (visible as the yellow slit through the +3V3 In2 region, x79-84 y139-158 in tonight's In2 render), F 0.4 mm 16.9 mm, B 0.4 mm 5.9 mm, 3 vias 0.4/0.3. Load: U21 IN1 -> V_MCU_2S -> TPS62152 (max ~0.7 A at 5 V input equivalent).

**Fix:** Widen to 0.8-1.0 mm and/or keep VBUS on outer layers; opportunistic while re-pouring In2.

**Sources:** IPC-2221 internal approximation at 17.5 um; measured segment lengths from live file

### LOW [unverified] — Buck/eFuse thermal-pad layout below datasheet guidance (no vias under U18 EP; U19 EP heatsink island only 5 mm²); U18 VOS tied into the power pour
*buck/eFuse thermal and sense layout — confidence high*

U18's exposed pad has no thermal vias (nearest GND vias 2.8 mm away) contrary to the TPS6215x layout example, U19's IN thermal pad has 5 vias into only a ~5 mm² F.Cu island versus the datasheet's 'as much copper area as possible using an array of thermal vias', and U18 VOS is connected into the +3V3 output pour rather than a separated sense trace — all workable at this board's dissipation (~0.3-0.5 W each) but each is a named datasheet deviation.

**Evidence:** Via scan: zero vias within 1.2 mm of U18 EP center (88.78,126.365), 2 GND vias within 3 mm; F.Cu GND fill fraction around U18 = 0.48. U19 EP (VBAT_CON, 2.7x1.45 mm) has 5 vias to F.Cu zone x76.6-78.1 y134.5-137.7 (5 mm²). TPS25982 layout guidelines: 'Connect to as much copper area as possible using an array of thermal vias.' TPS6215x layout: VOS 'in the shortest way to VOUT at the output capacitor... separated from the VOUT power line and plane'; VOS pad (90.215,126.615) merges into the +3V3 F pour x89.8-92.0 y126.1-127.1.

**Fix:** Add 4 GND vias in/next to the U18 EP; enlarge the U19 F-side heatsink island while fixing the input neck (same pour); optionally slit the VOS connection into a thin spur from C53.

**Sources:** TI TPS25982 SLVSEI3D section 8.5.1 (quoted); TI TPS6215x section 11.1 (quoted)

### LOW [unverified] — J8 battery polarity silk is a horizontal '- Batt +' beside a vertical pin pair
*silkscreen — confidence high*

The only polarity marking for the battery connector is gr_text '- Batt +' running horizontally at (88.68,124.02) on B.SilkS while J8's pins are stacked vertically (pin 1 GND at y125.59, pin 2 V+ at y129.55), so neither text end aligns with a specific pin — ambiguous for harness building, though the VH housing itself is keyed.

**Evidence:** gr_text '- Batt +' (at 88.68 124.02 0) (layer B.SilkS), justify left bottom mirror; J8.1 GND (84.73,125.5925), J8.2 VBAT_Terminal (84.73,129.5525), footprint JST_VH_B2P-VH_1x02_P3.96mm_Vertical on B side.

**Fix:** Replace with '+' next to pin 2 and '-' next to pin 1 (individual 1 mm glyphs beside each hole).

**Sources:** Live rocket-computer.kicad_pcb silk and pad coordinates

### INFO [unverified] — TPS2121 current limit strapped to its maximum (5.2 A) versus ~0.7 A actual branch load
*power mux — confidence high*

R64 = 18.7 k sets the TPS2121 ILM to 5.2 A typ (4.6-5.8 A), the top of its range, while V_MCU_2S only feeds the 1 A 3V3 buck through L5 — the mux therefore provides no meaningful fault limiting for the logic branch and a V_MCU_2S fault pulls up to ~5 A through the branch's 6 vias.

**Evidence:** Netlist: Net-(U21-ILIM) = {R64.2, U21.10}, bom.csv R64 = 18.7 k. TPS2121 datasheet SLVSEA3F EC table: 'RILM = 18.7 kΩ ... 4.6 / 5.2 / 5.8 A'. Load: V_MCU_2S = {C56, C59, L5.1, U21 OUT} -> TPS62152 only.

**Fix:** Optional: raise R64 (e.g., 44.2 k -> 2.5 A typ) for genuine logic-branch protection; verify against buck inrush (SS/TR soft-start already present).

**Sources:** TI TPS2120/TPS2121 datasheet SLVSEA3F EC table (quoted)

### INFO [unverified] — Orphan copper slivers from tonight's re-route
*copper hygiene — confidence high*

VBAT_Terminal has dead-end 0.1 mm stubs on F.Cu and In3.Cu near the J8.2 barrel, and DRC reports a dangling 0.13 mm PYRO3_FIRE stub on In3 — no electrical effect (the 0.1 mm In2 VBAT_Terminal trace pair is legitimate: it is the INA230 IN+ Kelvin route), just re-route debris worth sweeping before fab.

**Evidence:** Segments: VBAT_Terminal F.Cu (84.41,129.56)->(84.04,129.19)->(84.04,128.66) and (85.38,130.56)->(85.24,130.70/130.71); In3 (85.38,130.56)->(85.24,130.70) — none reach R72 or any second node. drc.json: track_dangling 'Track [PYRO3_FIRE] on In3.Cu, length 0.1300 mm'. Kelvin route confirmed: U23.13 -> via (79.7,127.8) -> In2 0.1 mm -> via (81.4,129.8) -> R72.2 region.

**Fix:** Delete the stubs (Edit > Cleanup tracks & vias).

**Sources:** Live PCB parse; drc.json exported tonight


**Affirmatively verified clean (layout-power):**

- Battery connector fix landed: J8 is now JST B2P-VH (B2P-VH(LF)(SN), 10 A class, thru-hole), replacing the 2 A JST PH the July 30 review blocked on (B-3/H4); pad connections are solid — ring scans show J8.1 (GND) lands 4.18 mm F + 3.61 mm In1 + 3.61 mm In3 + 3.61 mm In4 + 3.08 mm B of spoke copper (~15 A capability) and J8.2 nearly solid 9.07 mm B.Cu into the VBAT_Terminal zone.
- J8.2 -> R72 shunt corridor: min-cut 1.48 mm B.Cu 1 oz (~3.3 A at 10 C rise) — improved from the prior review's 1.0-1.1 mm; adequate for realistic continuous load (heads-up only if stall budget stays >6 A).
- Shunt R72 now has a real part: Stackpole CSSH0805FT2L00 2 mΩ metal-element (prior empty-MPN 0.125 W thick-film risk closed); at the 11.6 A eFuse limit P=0.27 W within its rating; INA230 Kelvin sensing is properly routed (IN- via 0.1 mm B.Cu trace to R72.1; IN+ via dedicated 0.1 mm In2 trace + 2 vias to R72.2; BUS tap R at U23.11).
- Pyro energy-store and fire loop copper: V_CAP min-cut 2.18 mm B.Cu (y=146.75) with 75+ mm² of B.Cu pour — fine for ~8 A initial e-match pulses (tau ~ 10 ms with C12 10 mF) and 4.2 A continuous; all four FET source clusters (U8/U6/U7/U10, B.Cu) sit inside the pours; charge path VBATT->R20 (150 Ω, CRCW1206-HP 0.75 W) on 0.5 mm track carries only ~56 mA.
- PYRO_GND return: 1.5 mm B.Cu from both J2.5 pads, 4 vias (0.4/0.3) up to U9 (CSD16323Q3 arm FET, F.Cu), 5 GND stitching vias at its sources, and C12.2 ties into 5 copper layers — pulse-rated with margin; R73 1 k bleed/sense present.
- Ground system: In1 and In4 are effectively solid GND (1415 mm² each on a 1676 mm² board) plus F 832 mm², In3 1063 mm², B 622 mm² fills and 163 stitching vias; worst whole-board GND min-cut is ~21 A-equivalent (y=121.6) — an order of magnitude above any return current, including servo stall and pyro pulses.
- +3V3 rail: TPS62152 correctly configured for the fixed-3.3 V variant (FB pin 5 to GND per datasheet, VOS senses output, DEF to GND, FSW strapped high, EN tied to VIN); output cap C53 10 uF is 1.5 mm from L6; L6/L5/L8 are VLS3012CX-2R2M (3.1 A sat) — adequate; output injects to In2 through 4 vias for a 1 A max converter; In2 +3V3 plane min neck 3.78 mm (~2.6 A at 0.5 oz) versus <=1 A worst load — 2x+ margin everywhere.
- V_MCU_SWTCH switched logic rail: In2 top-zone min-cut ~4.6 A vs ~1 A worst load; U22 TPS22918 (2 A rating) feeds it through 2 vias — adequate; the 0.3 mm In2 run south to the pyro continuity pull-ups carries only uA-mA.
- TLV62569 (U20, P4 core buck) layout follows its datasheet: input cap C47 10 uF at 1.0 mm same-side, output caps C55/C92 10 uF at L8, FB divider R74/R75/C93 (499k/499k/22 pF per Espressif P4 v3.x reference, including the R76 0 Ω pin-54 provision) placed ~4 mm from the SW node; only the downstream distribution width is deficient (separate finding).
- V_MCU_2S mux output: U21 -> C56 330 uF -> L5 path has pours on both sides plus 6 vias for <=1 A — adequate; C56/C15 (Kemet TCJE337M016R0050, 16 V polymer tantalum) polarity correct in netlist and derated to 53% on an 8.4 V max rail; C12 (Nichicon 16 V) polarity correct.
- Camera branch: J4.2 tap into the wide In2 region measures ~3.9 A local min-cut with 2 vias, and the switched return (Net-(J4-Pad1)) min-cut is 1.25 mm (~2.8 A) — the prior review's M23 0.70 mm camera neck was fixed by tonight's re-route; camera UART series resistors R30/R32 (1 k) are in place.
- LoRa and GNSS branches: FL2 feed 0.3 mm B.Cu (~1.0 A at 10 C) and return zone min 0.62 mm (~1.7 A) versus ~0.5 A TX bursts; FL1/GNSS 0.4 mm + 1 via versus ~0.15 A — both adequate (note FL1/FL2 beads BLM18PG471SN1D are 500 mA-rated parts, sized to the same ~0.5 A ceiling).
- Return-switch FETs Q1/Q7/Q8/Q9/Q10 are PMPB14XNX 40 V / 8.1 A / 15-18 mΩ — correct rating class for the branch currents (Q8 gate-drive voltage flagged separately).
- eFuse support parts match the TPS25982 datasheet application circuit: R48 single ILIM resistor to GND, ITIMER C50 10 nF, dVdT C51 10 nF, NRETRY C45, RETRY_DLY C46, PG pull-up R59, IN bypass C41/C42 at the IN pins — all present in the live netlist with sane values.


**Checks not completed (layout-power):**

- Actual fabbed inner-copper weight unknown: the file declares 35 um on In1-In4 but JLCPCB's standard 6-layer class ships 17.5 um inner; all In2 ampacity figures are given at 0.5 oz (worst case) with 1 oz equivalents noted — needs confirmation against the actual JLC order options (reported as a medium finding).
- ESP32-P4 datasheet v0.7 does not publish a VDD_HP maximum current, so the core-rail requirement could not be bounded numerically; the finding is anchored on Espressif's explicit >=20 mil trace-width rule instead.
- Prior review's widest-single-path metrics (e.g., 0.70 mm U19->J3.15) could not be reproduced exactly for one-to-one comparison — the Dijkstra widest-path run at 25 um resolution exceeded the time budget; min-cut totals (equal or more conservative) were used throughout.
- Nexperia PMPB14XNX PDF was not retrievable directly (assets URL returned HTML); ratings taken from Nexperia product page and distributor datasheet listings (Farnell/TME agree: 40 V, 8.1 A, DFN2020MD-6).
- Molex 878321620 (J3) per-contact rating not re-fetched tonight; the 2 A/contact figure is carried from the prior review's citation (Milli-Grid PS-87831-027).
- Thermal analysis is IPC-2152-class steady-state approximation only; no transient FEA for the servo-stall case — the 10 C-rise ratings quoted are conservative for pulses under ~1 s but the stall scenario (10 s+) is effectively steady-state and the numbers hold.


## A.layout-signal-rf — Layout — signals & RF

*(verify pass for this domain did not run — session limit; tags below are [unverified] except where the main loop spot-checked)*

### HIGH [unverified] — S3 2.4 GHz chip-side CLC matching network (C23/L2/C24) deleted in tonight's uncommitted edits — LNA_IN now runs bare to the antenna with an orphaned 4.3 nH shunt
*S3 RF / antenna match — confidence high*

The pi matching network between U15 LNA_IN and the Molex 47948 antenna was removed from the live schematic and PCB tonight; the only remaining element is L1 (4.3 nH shunt at the antenna), which was tuned as part of the old 5-element chain and now, standing alone, mistunes the port by itself (50 ohm antenna paralleled by +j66 ohm at 2.44 GHz looks like 31.8+j24 ohm, VSWR ~2.1, ~0.55 dB mismatch loss even with a perfect antenna), and there is no longer any placeholder to bench-tune TX power/harmonics against Espressif's guidance.

**Evidence:** Live netlist (scratch review/netlist.net, exported tonight): Net-(U14-Feed) = {U15.1 LNA_IN, L1.2, U14.4} only; zero occurrences of refs L2/C23/C24 in netlist, live esp32s3_outputs.kicad_sch (mtime Aug 4 16:31), bom.csv, or live rocket-computer.kicad_pcb footprints. git HEAD of the same schematic still contains all three refs (6 hits) — deletion is tonight's work. Prior review (prefab-review-2026-07-30.md M15) documented the CLC as present and inside Espressif's windows. Molex 47948 datasheet: input impedance 50 ohm.

**Fix:** Restore the CLC pi footprints next to U15 pin 1 (C 1.2-1.8 pF shunt, L 2.0-3.0 nH series, C shunt, 0402 acceptable) plus keep L1 at the antenna; fit the previously documented 1.5 pF / 2.7 nH / 1.5 pF starting values and plan a conducted/radiated tune. If the deletion was intentional, it must be justified per the HDG's own exception (antenna impedance guaranteed 50 ohm at the chip port by simulation) — which the 39-ohm feed and orphan shunt do not meet.

**Sources:** Espressif ESP32-S3 Hardware Design Guidelines (docs.espressif.com, fetched 2026-08-04): 'A CLC structure is preferred', values 'C11: 1.2 ~ 1.8 pF, L2: 2.0 ~ 3.0 nH', matching 'must be placed close to the chip', exception only if 'the antenna impedance point can be guaranteed to be 50 ohm by simulation'. Molex 47948 datasheet p.1 (Input Impedance (Ohms): 50).

**Disposition (2026-08-05, owner): WAIVED.** Deliberate change implementing Molex AS-479480001 Rev G §6.0's nominal L-network (4.3 nH shunt only, 0 Ω series as direct trace), per base-station review A1 (resolved 2026-08-04, `hardware/base-station/prefab-review-2026-08-02.md`); the no-tune/no-VNA-break-point trade is accepted. See NB-1 in the executive summary. The 50 Ω feed-width item remains actionable.

### MEDIUM [unverified] — RF feed trace is 0.3 mm wide = ~37-39 ohm on the declared stackup, not 50 ohm, over an 8.0 mm (42 electrical degrees) run
*S3 RF / feed line — confidence high*

The LNA_IN-to-antenna feed is routed 0.3 mm wide over the 0.1 mm prepreg to In1 GND, giving ~37-39 ohm (microstrip; the measured 0.13-0.40 mm pour gap makes it CPWG and slightly lower still); at 8.0 mm (~42 deg at 2.44 GHz) this is a real impedance step (VSWR ~1.4 contribution on its own), which compounds with the missing matching network and cannot be tuned out afterward.

**Evidence:** Live PCB Net-(U14-Feed) segments: 8 segments, all F.Cu, all width 0.3 mm, total 7.99 mm, U15.1 (75.53,142.47) -> L1.2 (74.335,136.90) -> U14.4 (73.67,135.49). Declared stackup in rocket-computer.kicad_pcb: F.Cu 0.035 / prepreg 0.1 mm er 4.5 / In1. Computed (Hammerstad): w=0.3 -> 38.8 ohm; w=0.2 -> 50.3 ohm. Measured GND pour edge gap along feed: 0.128-0.40 mm (min centerline-to-pour-edge 0.278 mm minus 0.15 half-width). Prior review recorded 'feed ~50 ohm' before tonight's re-route.

**Fix:** Re-route the feed at ~0.19-0.21 mm width with the F.Cu GND pour pulled back to a uniform >=0.3 mm gap (or keep 0.3 mm width only if the JLC impedance-control stackup is ordered and recomputed). Keep the run short and stitched as it is now (9 GND vias within 2 mm are already there).

**Sources:** JLCPCB JLC06161H-3313 stackup (3313 prepreg 0.0994 mm, er ~4.05 — in-file declaration says 0.1 mm / er 4.5; both give 36-39 ohm at w=0.3); Hammerstad-Jensen microstrip and CPWG elliptic-integral calcs in scratch rf4.py.

### MEDIUM [unverified] — PRIOR M27 NOT FIXED: ESP_SDA/ESP_SCL still run 37-39 mm along the board edge passing 0.33/0.53 mm from the antenna radiator, outboard of it
*S3 RF / antenna keepout — confidence high*

The inter-MCU I2C pair still runs the full left board edge on F.Cu, squeezing between the antenna body and the board edge where no ground guard fits; BLE TX couples common-mode RF into a bus feeding both MCUs and the traces sit in the antenna near field (detune/desense), exactly as flagged in the 2026-07-30 review (M27) — tonight's re-route did not address it.

**Evidence:** Live PCB: ESP_SDA min distance to antenna body bbox (73.14-76.14, 133.02-136.02) = 0.33 mm at (72.81,133.63) F.Cu; ESP_SCL = 0.53 mm at (72.61,133.47); F.Cu runs 37.3 mm / 39.0 mm spanning y 113-148 at x 72.6-72.8 (board edge x ~72.4). Visually confirmed on F.Cu crop: two parallel traces hugging the edge past the antenna pads. Prior finding M27 with Espressif quote 'There should be no high-frequency signal traces routed close to the RF trace'.

**Fix:** Drop the ESP_SDA/SCL pair to In3 for the y~128-142 stretch (In1 GND then shields them from the antenna), or re-route inboard east of the antenna with a stitched GND guard. Cheap, no BOM change (fix already specified in prior review).

**Sources:** prefab-review-2026-07-30.md M27; Espressif ESP32-S3 HDG PCB layout chapter (RF trace isolation guidance).

### MEDIUM [unverified] — 40 MHz load-cap 'fix' overshot: 12 pF gives CL_eff ~8.5-9 pF against the crystal's 10 pF spec — frequency error flipped sign, magnitude unchanged (~15-20 ppm high)
*crystals / S3+P4 40 MHz — confidence medium*

Commit e5542d08 changed C20/C22/C48/C49 from 18 pF to 12 pF, but the correct value for a 10 pF-CL crystal with 2-3 pF chip-down stray is ~15-16 pF (as the prior review's L15 fix said); 12 pF under-loads the crystal by about the same 1.5 pF the old 18 pF over-loaded it, so the S3 radio's reference now runs an estimated 15-20 ppm HIGH instead of low — Wi-Fi's +/-25 ppm budget is consumed by this alone once the crystal's own +/-10 ppm tolerance and stability are added; BLE (+/-50 ppm) retains margin.

**Evidence:** Live BOM: 'C20,C22,C48,C49','12 pF',CL05C120JB5NNNC. ECS-400-10-37B2-CKY-TR: CL = 10 pF, +/-10 ppm tol. CL_eff = 12/2 + Cstray(2-3) = 8-9 pF vs 10 pF. Pull-sensitivity estimate ~12 ppm/pF (Cm~3 fF, C0~1 pF assumed, stated as estimate). HDG formula CL = C1xC2/(C1+C2) + Cstray. Prior review L1/L15 recommended 15-16 pF, not 12.

**Fix:** Fit 15 pF (C0G 0402) on all four positions (or 16 pF if measured stray is low); verify center frequency on a spectrum analyzer during RF conducted test as prior L15 already prescribed. P4 side (Y4/C48/C49) is not RF-critical — acceptable to leave if BLE-only operation is accepted on S3, but the parts are shared BOM lines.

**Sources:** ECS-400-10-37B2 datasheet (CL 10 pF); Espressif S3 HDG (crystal accuracy +/-10 ppm, CL formula); commit e5542d08.

### MEDIUM [unverified] — USB ESD array CR3 sits 9.7 mm from the USB-C connector on a ~7-9 mm stub off the D+/D- path; D+ additionally takes a 4-via three-layer detour while D- stays topside
*USB / ESD and pair routing — confidence high*

The SP05 TVS clamps through ~7-9 mm of 0.127 mm stub (~5 nH), so a connector ESD strike drives the U1 mux branch before the clamp's L*di/dt-degraded clamping catches up — weak ESD hardening for a field-programmed board; separately D+ is routed J6 -> B.Cu jog under the connector -> In3 (17.5 mm, referencing the +3V3 In2 plane) -> F.Cu with 4 vias while D- runs 21.3 mm entirely on F.Cu over solid In1 GND (0 vias), an asymmetric pair with 2.5 mm skew — electrically tolerable at USB Full-Speed but poor practice on the only programming/recovery path for both chip-down MCUs.

**Evidence:** Live PCB: CR3 (TVS-SP05) at (92.11,159.83); J6 at (84.36,164.79); D+ branch to CR3: In3 (85.09,162.15)->(91.95,162.15) 6.86 mm + jog, via (92.50,160.12) -> CR3.3; D- branch (86.13,161.74)->(90.79,161.74) 4.66 mm + 2 mm to CR3.4. D+ totals: F 4.04 / In3 17.49 / B 2.28 mm, 4 vias; D- 21.34 mm F.Cu only, 0 vias. In2 under the D+ In3 run is the +3V3 zone; crossing count 3 with 5.8 mm over the fill gap. All D+/CEN/OUT vias do have GND return vias within 0.53-1.7 mm.

**Fix:** Move CR3 (or a second small array) directly in line at the J6 pin field before the line branches west, and re-route D+ to stay with D- on F.Cu from J6 to U1 (there is room along the y~161-162 corridor D- already uses). Keep the existing return vias.

**Sources:** Littelfuse SP05 series app guidance (TVS at connector, in the signal path); USB 2.0 FS electrical budget (skew negligible — severity driven by ESD placement, not SI).

### MEDIUM [unverified] — In3 coupling corridor (M28 class, partially improved, partially worse): buzzer PIEZZO rides PYRO2_FIRE for ~40-80 mm at 0.195 mm gap; USB and SPI still pair with pyro FIRE/CONT lines for 30-50 mm
*In3 routing / pyro no-fire margin — confidence high*

Tonight's re-route widened the worst gaps from 0.10 to 0.195-0.20 mm (good) but lengthened the co-runs: the continuously-toggling buzzer drive now parallels the drogue/main fire line PYRO2_FIRE for tens of mm, PYRO1_CONT pairs with ESP_SDO ~50 mm, CEN_D+ with PYRO4_CONT ~48 mm, PYRO1_FIRE with ESP_SCLK ~34 mm, and M_MISO with PIEZZO ~20 mm; coupled glitches (est. 100-200 mV) remain far below the DTC123J ~0.6 V sustained threshold and the two-switch arm chain still gates energy, so spurious fire stays non-credible — but the respin's hardware no-fire margin keeps eroding, and USB/buzzer activity will pollute the PYROx_CONT continuity ADC readings during bench sessions.

**Evidence:** Pair sweep over all 2499 live segments (scratch rf3.py), In3 gap<0.35 mm coupled-length estimates: PIEZZO||PYRO2_FIRE 83 mm @0.195 (estimate double-counts sub-segments; >=40 mm real), PYRO1_CONT||ESP_SDO 49.8 @0.200, CEN_D+||PYRO4_CONT 47.7 @0.200, PYRO1_FIRE||ESP_SCLK 33.6 @0.200, M_MISO||PIEZZO 20.4 @0.195. All 0.1 mm traces; In2 0.1 mm above, In4 GND 0.535 mm below. Prior review M28 quantified the same class at 0.10 mm gaps.

**Fix:** In the In3 ribbon, group pyro FIRE/CONT away from PIEZZO/USB/SPI with a poured GND spacer trace (pour already exists between some ribbons), or move PIEZZO to B.Cu where it has no long neighbors. At minimum re-accept M28's residual-risk rationale consciously for the new, longer geometry.

**Sources:** prefab-review-2026-07-30.md M28 (thresholds: DTC123J V_I(on) 1.1 V min / ~0.6 V sustained, arm-chain analysis).

### LOW [unverified] — In3 signals ride the In2 plane-split slot for up to ~19 mm (PIEZZO 19, PYRO3_FIRE 18, PYRO2_FIRE 17.3, PYRO_ARM 11.4 mm) with only the 0.535 mm-distant In4 GND as continuous reference
*In3 / return path & EMC — confidence high*

Because In2 is split three ways (+3V3 prio13 upper / V_MCU_SWTCH prio14 lower / VBATT prio22 island x76-94.5 y101-139), every In3 net crosses reference splits 1-6 times and several run lengthwise ALONG the unfilled split slot, so their tight 0.1 mm reference disappears for centimeters; the solid In4 GND at 0.535 mm keeps this functional (loop stays small in absolute terms), but it concentrates return currents and raises crosstalk/emissions for exactly the pyro and buzzer nets already flagged above.

**Evidence:** Point-in-polygon sampling of all In3 segments against the four In2 filled_polygons (scratch rf4.py + in2_fills.json from the live PCB): over-gap lengths PIEZZO 19.0 mm, PYRO3_FIRE 18.0, PYRO2_FIRE 17.3, PYRO_ARM 11.4, CAM_ACT 7.8, IND_2 7.0, D+ 5.8; crossing counts per net printed in full (e.g. ESP_CS 4 transitions incl. VBATT->+3V3). In2 zone nets/priorities parsed from live file.

**Fix:** No re-route required for function; opportunistically shift the worst lengthwise runs (PIEZZO, PYRO2/3_FIRE) a few tenths sideways onto solid In2 copper, and keep the existing GND stitching. Note for EMC test expectations.

**Sources:** Live rocket-computer.kicad_pcb zone fills; stackup declaration (In3-In4 core 0.535 mm).

### LOW [unverified] — S3 32 kHz crystal (Y1) load caps' ground return travels 3.2-3.3 mm of pour before reaching a GND via; USB lines skirt the crystal area
*crystals / S3 32k — confidence high*

C19/C21 (22 pF, XTAL_32K) have their GND pads 3.33/3.20 mm from the nearest GND via, the longest cap-ground return of any crystal on the board, on the high-impedance (ESR up to 70 k) oscillator that the S3 light-sleep plan depends on; meanwhile OUT_D+/OUT_D- pass beneath the crystal on In3 (shielded by solid In1, acceptable) and Net-(U15-GPIO19) (USB) passes ~1.3 mm away on F.Cu — startup margin at temperature is the known 32k failure mode, so the free via is worth taking.

**Evidence:** Live PCB: C19 GND pad (77.95,152.81) nearest GND via 3.33 mm; C21 (79.06,152.84) 3.20 mm. Contrast: P4 32k caps C37/C38 have vias at ~1.0 mm, Y2 40M C22 at 0.44 mm, C48 via-in-pad. OUT_D+ In3 (82.35,150.63)->(77.10,154.45) under Y1 (pads x77.64-79.94, y150.32); Net-(U15-GPIO19) F.Cu (80.36,148.92)->(81.05,149.61).

**Fix:** Add one GND via adjacent to the C19/C21 shared ground pour neck (space exists at ~78.5,152.9). No other change needed.

**Sources:** prefab-review-2026-07-30.md L14 (SC-32S ESR at Espressif's 70 k limit, startup-margin rationale).

### LOW [unverified] — P4 40 MHz crystal Y4 unchanged from prior L17: 12.3 mm from U17 with 14.8/9.2 mm asymmetric 0.1 mm traces; C49 ground via 1.9 mm out
*crystals / P4 40 MHz — confidence high*

Tonight's re-route left the P4 crystal exactly where the prior review flagged it: Y4 sits 12.3 mm from the chip, XTAL_N meanders 14.76 mm against XTAL_P's 9.17 mm, and the pair runs parallel to itself at 0.2 mm for most of that length; workable (P4 is not the radio) but outside Espressif's keep-it-close guidance, and C49's ground return runs 1.91 mm to its via while C48 has via-in-pad.

**Evidence:** Live PCB: Y4 (74.95,102.81), U17 (83.58,111.75); Net-(U17A-XTAL_N) 14.76 mm F.Cu, XTAL_P 9.17 mm, width 0.1 mm; crystal-pair mutual spacing 0.200 mm measured; Y4 pad2 GND via 1.82 mm, pad4 0.71 mm, C49 1.91 mm. Identical numbers to prior L17 (14.8 vs 9.2 mm).

**Fix:** Accept (prior review's disposition) or, if the area is touched again, slide Y4 toward U17 and equalize the pair. Add a via at C49's GND pad opportunistically.

**Sources:** prefab-review-2026-07-30.md L17; Espressif P4 HDG crystal section (crystal close to chip, CL formula).

### INFO [unverified] — Tonight's re-route left a 0.13 mm dangling PYRO3_FIRE stub on In3 and both 40 MHz crystal footprints out of sync with the library
*In3 / housekeeping — confidence high*

DRC on the live board shows one dangling track end — PYRO3_FIRE, In3 at (80.27,114.56), 0.13 mm — a harmless re-route leftover worth deleting before gerbers, and 'lib_footprint_mismatch' on Y2/Y4 (XTAL_ECS-400-10-37B2-CKY-TR modified in-board, likely the GND via-in-pad edits), which should be pushed back to the library so the next board inherits it.

**Evidence:** scratch drc.json (severity-all, live board): track_dangling Track [PYRO3_FIRE] In3.Cu length 0.13 mm (80.27,114.56); lib_footprint_mismatch Footprint Y4, Footprint Y2. Also noted: ERC endpoint_off_grid on U15 pin 1 (LNA_IN), L1, L3 — cosmetic fallout of tonight's RF-area schematic edit, connectivity intact in the netlist.

**Fix:** Delete the stub, resync the crystal footprint to the library (or document the board-local variant), re-snap the off-grid RF wire ends.

**Sources:** drc.json / erc.json exports from the live files (tonight).


**Affirmatively verified clean (layout-signal-rf):**

- In1.Cu and In4.Cu carry ZERO routed segments across all 2499 tracks in the live PCB — both are unbroken solid GND planes; every F.Cu signal (all four crystal networks, RF feed) references solid GND at 0.1 mm
- Molex 47948 on-ground antenna config correct per datasheet: GND fills verified present directly beneath the antenna on In1/In3/In4/B.Cu (point-in-polygon test at 4 points under the body); datasheet explicitly requires NO ground cutout ('No removal of ground layers from beneath the antenna is needed', 'Ground Clearance: None needed'); the three unconnected pads 1-3 are 'dummy pads... for strong mechanical bonding' per datasheet p.2 — correct as drawn
- RF feed ground stitching: 9 GND vias within 2 mm of the feed centerline; L1's GND pad reaches a via in 1.73 mm of continuous pour
- L3 24 nH series inductor in the S3 XTAL_P leg is PRESENT in the live netlist (U15.54 -> L3 -> C20/Y2.3) — the revert-of-the-revert (commit e7b81a8a) is in effect, matching Espressif's S3 HDG suggestion of 24 nH for crystal-harmonic suppression; L1/L3/L4 now carry Murata MPNs (LQW15AN4N3C00D / LQW15AN24NH00D / LQG15HS2N0S02D), closing the prior M15 MPN gap for the parts that remain
- Y2 (S3 40 MHz) local environment clean: no foreign-net copper inside the crystal region on any relevant layer; GND pad 4 has via-in-pad, C22 GND via at 0.44 mm; no traces beneath any crystal on the adjacent layer (In1 is solid); the only under-body foreign copper anywhere is SERVO_ACT/Q8-gate on B.Cu beneath Y3/Y4 — opposite side of the board, shielded by two solid planes
- Crystal-to-switching-node distances: buck inductor L8 is 14-17 mm from Y4/Y3; U20 (P4 core buck) 12+ mm; U18 (3V3 buck) 12 mm from Y2; nearest same-layer foreign copper to every crystal net is >=0.2 mm and is either its partner crystal line or a static strap (GPIO46), never a switch node
- P4 32.768 kHz crystal Y3 correctly on U17 pins 104/1 (GPIO0/GPIO1 = XTAL_32K per P4 datasheet pin list); 22 pF caps vs ABS07 CL 12.5 pF lands CL_eff ~13 pF — in spec; C37/C38 have GND vias at ~1.0 mm
- USB pair skews all negligible for Full-Speed: D 2.5 mm, CEN 4.8 mm, OUT 1.3 mm; every USB signal-layer transition has a GND return via within 0.53-1.7 mm (worst non-USB case ESP_SDO 2.04 mm); series elements correct class: R38/R39 = 22R (RC0402FR-0722RL) in OUT_D+/-, R77 1M CEN_D+ keeper to GND; CR3 SP05 covers D+, D- and VBUS
- FSUSB63 mux port usage consistent: common D+/D- to J6/CR3, HSD2 -> P4 (CEN_D+/-), HSD3 -> S3 via 22R (OUT_D+/- -> GPIO19/20), HSD1 unused with no copper stubs (pads only); SEL0 driven by slide switch S1 with GND on adjacent pad
- Daughterboard RF separation: J1 (GNSS, B.Cu) is 19 mm and J5 (LoRa, B.Cu) 36 mm from the 2.4 GHz antenna, both on the opposite side and far end; FL1/FL2 are supply-side ferrites on B.Cu with two solid GND planes between them and the antenna; LoRa/GNSS UARTs run buried on In3 between In2 and solid In4
- SDIO/QSPI-class buses healthy: P4 flash bus FLASH_* 8.6-18.4 mm all F.Cu beside U16; S3 flash bus OUT_SPI_* ~10 mm; S3 NAND bus M_* 9-18 mm; none pass within 1.5 mm of the antenna or feed; length spreads irrelevant at SPI clocks used
- In3 full visual pass (board-fit render at 86 px/mm) matches the numeric inventory: pyro/SPI/USB/UART ribbons with GND pour flooded between and around them, stitched with vias; no unterminated long runs (only the 0.13 mm stub flagged), no acid-trap acute pours, slots and mounting holes properly voided; In2 visual confirms the three-plane split geometry parsed from the file
- Prior-review M28's vertical-shielding claim still holds: all In3 runs have In2 copper 0.1 mm above (where filled) and solid In4 GND 0.535 mm below per the declared stackup (F/0.1 prepreg/In1/0.535 core/In2/0.1/In3/0.535/In4/0.1/B, filled+capped vias declared '(filling yes) (capping yes)')


**Checks not completed (layout-signal-rf):**

- Antenna/port tuning cannot be paper-verified: with the matching network deleted there is no simulation or measurement on file showing the LNA_IN port sees 50 ohm (the HDG's stated condition for omitting the match); needs a VNA/conducted-power bench pass after the CLC is restored
- HEAD-PCB feed-width comparison (was the old feed really ~50 ohm?) could not be completed — the feed net was renamed by the component deletion so the HEAD netname lookup failed; immaterial to the live findings
- Exact ppm error from the 12 pF load caps depends on the ECS crystal's unpublished motional parameters (Cm/C0) and true board stray; my 15-20 ppm figure is a formula estimate (12 ppm/pF pull sensitivity assumed) — bench spectrum measurement required to pin it
- Coupled-length figures from the In3 parallel sweep are segment-accumulation estimates (can double-count sub-segments); treat as relative ranking, not exact overlap lengths
- Radiated performance of the mid-edge antenna placement (components/S3 within 2-4 mm on three sides vs Molex's 100x40 mm edge-mounted reference platform) is not assessable without a range or sim; datasheet gives no minimum lateral clearance to other components


## A.layout-grounds — Layout — grounds & stackup

*(verify pass for this domain did not run — session limit; tags below are [unverified] except where the main loop spot-checked)*

### HIGH [unverified] — Pyro fire-return current is confined to two 0.3 mm vias and one 1.0 mm 0.5 oz In2 trace — plane connection at the U9 star point is blocked by an all-layer keepout
*grounds/pyro star point — confidence high*

The single PYRO_GND-to-GND star tie at U9 (CSD16323Q3 arming FET) cannot dump fire current into the GND planes: an all-layer copper-pour keepout covering (91.4,162.4)-(94.0,164.0) contains U9's three source pads, so the entire deployment-pulse return runs U9 sources -> 0.4/0.5 mm F.Cu jumper tracks -> exactly two 0.3 mm-drill vias at (92.25,163.59) and (91.83,162.78) -> one 1.0 mm-wide, ~18 mm-long GND trace on In2 (0.5 oz inner copper, ~18 mOhm) -> C12.2 barrel. Neither via overlaps any plane fill (verified: zero fill contact at r=0.15 and r=0.36 sampling on In1/In3/In4/B for the (92.25,163.59) via; no annular overlap at (91.83,162.78) either). At the prior review's own worst case (~25 A, 4-channel simultaneous) the trace dissipates ~11 W for the ~10-15 ms pulse (~0.17 J into ~1 mg of copper, adiabatic rise of several hundred K — fusing-risk territory; Onderdonk fusing time for 25 A through 0.0175 mm2 is ~66 ms). Even a normal 7.6 A single-channel fire bounces U9's source ~0.14 V above system GND, directly subtracting from U9's Vgs mid-fire. Two-match redundant deployment (~15 A) is marginal.

**Evidence:** Live rocket-computer.kicad_pcb: keepout zone bbox (91.4,162.4)-(94.0,164.0), layers F/B/In1/In2/In3/In4, copperpour=not_allowed, tracks/vias allowed; U9 source pads 1/2/3 at (93.61/92.97/92.31,163.50) all return pip()=False against F.Cu GND fill; F.Cu GND segs w=0.5 (92.25,163.59)->(93.53,163.59) and w=0.4 chain to (91.83,162.80); In2 GND segs w=1.0 (91.83,162.78)->(92.31,163.26)->(91.86,163.36)->(84.26,155.76)->(84.26,148.74)->(84.90,148.10)=C12.2 barrel; only GND vias in region: (92.25,163.59) and (91.83,162.78) (plus (94.31,162.17) in the pour but with no F.Cu track link to the sources). Netlist: U9.1/2/3=GND, U9.5-9=PYRO_GND, R73(1k) bleed. Same geometry exists in pcb_HEAD.kicad_pcb, so this is not a tonight-only artifact — but it contradicts the prior review's 'pyro fire-path copper checks out (~25 A 4-channel worst case)' claim for the RETURN side.

**Fix:** Either shrink/split the keepout so the F.Cu GND pour reaches U9's source pads (then add 4-6 GND vias immediately south/east of the source row), or keep the guided-return concept but widen the In2 return to a >=3 mm corridor (or parallel it on In3), and add at least 2 more return vias. Re-run fill + DRC afterwards.

**Sources:** Live .kicad_pcb parse; TI CSD16323Q3 (SON3x3, pad=drain); Onderdonk/Preece pulse-fusing estimates

### MEDIUM [unverified] — Two 8x3 mm board slots choke both GND planes to ~7.5 mm total width at y~121.6, with mounting holes H5/H7 sitting dead-center in the necks
*grounds/stackup — plane continuity — confidence high*

Two internal Edge.Cuts cutouts (79.75-87.77, 119.54-122.55 and 79.73-87.73, 132.34-135.34 — presumably battery-strap slots) sever ALL copper layers across 8 mm of the 22.35 mm board width, and the Ø2.2 mm mounting holes H5 (75.22,121.64) and H7 (91.97,121.64) sit exactly in the middle of the two remaining necks of the northern slot. Measured In1/In4 filled copper across the full board at y=121.5 is only 7.5 mm total (4.0 mm left neck + 3.5 mm right neck); the southern slot band bottoms out at ~11.75 mm. Every north-south current — battery return from J8 (which sits between the slots) to both MCUs, the P4<->S3 SPI/I2C return, USB return — squeezes through these necks. Capacity: 2 planes x 7.5 mm of 0.5 oz plus outer pours is adequate for the realistic 4-8 A but thin against the 14.7 A eFuse ceiling, and the necks raise return-path impedance for the inter-MCU bus that crosses both slots.

**Evidence:** Edge.Cuts geometry in live rocket-computer.kicad_pcb (gr_line/gr_arc sets forming two rounded rects, r=0.5); raster of the In1/In4 GND filled_polygon: row y=121.52 left fill 4.00 mm, right 3.50 mm; row y=134.02 total 11.75 mm; H5/H7 pads 2.6 mm/drill 2.2 at (75.22,121.64)/(91.97,121.64); J8 battery at (84.73,125.59-129.55) between the slots; layer_In1.Cu.pdf visually confirms both white rectangles.

**Fix:** If the slots are load-bearing (battery strap), accept consciously and document; consider relocating H5/H7 out of the neck lines (or shrinking to Ø1.6 hardware) to recover ~3 mm of plane per side, and keep the inter-MCU bus + its GND stitching vias routed through one neck together.

**Sources:** Live .kicad_pcb parse + layer PDFs

### MEDIUM [unverified] — ESP32-P4 exposed pad has only 6 GND vias in a 7.5x7.5 mm pad; ESP32-S3 EP has 4 in 4.1x4.1 mm
*grounds/EP via arrays — confidence medium*

U17 (ESP32-P4NRW32, the board's main heat source at up to ~400 MHz dual-core) grounds/cools its 7.5 mm exposed pad through only 6 vias, and U15 (ESP32-S3RH2 radio, whose EP is also its RF ground return) has only 4 — well short of the dense arrays (9+ for S3-class, ~5x5 for P4-class) Espressif uses on its own reference boards. Thermal resistance of the via path and RF ground inductance are both higher than reference practice; with the P4 chip-down and no other heatsinking, this costs real thermal margin in a sealed airframe.

**Evidence:** Parse of live PCB: U17.105 EP 7.5x7.5 at (83.58,111.75), 6 GND vias inside pad outline; U15.57 EP 4.1x4.1 at (78.95,145.07), 4 GND vias inside. Espressif ESP32-P4 HDG (local extract) mandates 'complete GND plane for the crystal and chip' but the local extract contains no numeric EP-via count — count judged against Espressif reference designs and QFN practice.

**Fix:** Fill both EPs with a grid of 0.3 mm vias (P4: 4x4 minimum, prefer 5x5; S3: 3x3) — filled+capped vias are already declared for this fab class so in-pad vias are safe.

**Sources:** Espressif ESP32-P4 hardware design guidelines (scratch p4-hdg.txt); via count from live PCB parse

### MEDIUM [unverified] — U18 (TPS62152 buck) GND thermal pad has zero vias in-pad; nearest GND via 2.3-2.8 mm away
*grounds/regulator layout — confidence medium*

The 3x3 QFN buck U18's exposed pad (1.68 mm, net GND) contains no vias and the nearest GND vias to any of its GND pins are 2.27-2.78 mm away, so the converter's high-di/dt input-loop return closes only through the F.Cu pour before reaching the planes — a deviation from TI's QFN PowerPAD layout practice (thermal/analog ground vias at the pad) that raises ground bounce and thermal resistance for a 1 A-class converter.

**Evidence:** Parse: U18.17 EP 1.68x1.68 at (88.78,126.36) net=GND, 0 vias inside; U18 GND pads 5/6/8/15/16/17, nearest GND via distances 2.27-2.78 mm; U18 is in the sparsest via region of the board (worst any-point gap 6.9 mm at (94.1,130.5) nearby).

**Fix:** Add 1-2 GND vias in or immediately beside the U18 EP and one via cluster at its input-cap ground.

**Sources:** TI TPS62152 layout practice (datasheet section not re-fetched tonight — see blocked); live PCB parse

### LOW [unverified] — LoRa-link connector J5 mounting tabs left floating (no net) while the identical GNSS link J1 grounds its tabs
*grounds/connector grounds — confidence high*

J5's two mechanical pads (J5.MP at (78.34,97.84) and (83.94,97.84)) have no net and sit in copper-free islands, whereas the same-family JST SH connector J1 grounds its tabs (pins 6/7 -> GND). Floating tabs give up a free chassis/EMI tie and the retention pads' solder joints float at undefined potential next to the switched-ground daughterboard wiring.

**Evidence:** Pad dump: J5.MP x2 net='' ; netlist: J1.6/J1.7 = GND with vias at 0.00/0.75 mm. Both are BM0xB-SRSS-TB family (BOM lines J1/J5).

**Fix:** Assign the J5 shield/mount pads to GND like J1's (footprint edit), or record the asymmetry as intentional.

**Sources:** Live netlist + PCB parse

### LOW [unverified] — Dangling 0.13 mm PYRO3_FIRE stub on In3
*grounds/route hygiene — confidence high*

A 0.13 mm orphan track segment of PYRO3_FIRE remains on In3 at (80.27,114.56) (DRC 'Track has unconnected end') — harmless electrically but a leftover from tonight's re-route worth deleting before gerber generation.

**Evidence:** drc.json: track_dangling warning, 'Track [PYRO3_FIRE] on In3.Cu, length 0.1300 mm' at (80.27,114.56).

**Fix:** Delete the stub and re-run DRC.

**Sources:** drc.json (severity-all export, tonight)

### LOW [unverified] — Several GND decoupling pads sit 3-4.6 mm from their nearest GND via
*grounds/stitching — confidence high*

The worst GND SMD pads reach the planes only after 3+ mm of surface pour: C53.2 (94.13,128.04) 4.55 mm, C43.2 3.71 mm, C44.2 3.64 mm, U11.4 (SPI-NAND GND) 3.54 mm, U16.4 (W25Q flash GND) 3.43 mm, C19/C21 (S3 area) 3.2-3.3 mm — longer-than-ideal return loops for decoupling, all in the U18/J8 and flash neighborhoods that are also the board's sparsest via regions (6.5-6.9 mm gaps near (92.6,131) and (94.1,130.5)).

**Evidence:** Nearest-GND-via computation over all 163 GND vias vs every GND SMD pad (list in analysis); board-wide worst any-point-to-GND-via distance 6.9 mm at (94.1,130.5).

**Fix:** Opportunistically drop ~6 GND stitching vias: 2 near U18/C53, 1 at U11.4, 1 at U16.4, 2 in the (92,130) area.

**Sources:** Live PCB parse

### INFO [unverified] — Battery return pin J8.1 connects to all five GND layers through thermal reliefs only (~3.4 mm of copper per layer)
*grounds/battery return — confidence high*

J8.1 (JST VH battery GND, 1.7 mm drill) is zone-connected with thermal reliefs (zone connect_pads=thru_hole_only, gap/bridge 0.5 mm) on F/In1/In3/In4/B — measured arc copper at r=1.9 mm is 2.6-3.4 mm per layer (~11 mm 1 oz-equivalent total). Adequate for realistic 4-8 A and short 14.7 A faults, but a solid connection on this one pad would remove the derating for free.

**Evidence:** GND zone options in live PCB: (connect_pads thru_hole_only (clearance 0.127)), thermal_gap/bridge 0.5; arc-coverage sampling at (84.73,125.59): F 29%, B 22%, In1/In3/In4 29% of circumference.

**Fix:** Set J8.1 pad to solid zone connection (pad override), or accept: soldering iron access is not an issue on a 1.7 mm VH pin.

**Sources:** Live PCB parse

### INFO [unverified] — In2 fire-return GND trace diagonally slots the +3V3 inner power pour
*stackup/power plane — confidence high*

The deliberate 1.0 mm In2 GND trace from (91.9,163.4) to C12.2 (84.9,148.1) cuts an ~18 mm diagonal slot (trace + clearance) through the +3V3 In2 zone's bottom-right region; return paths are unaffected (In4 solid beneath In3) and +3V3 current has ample width elsewhere, but be aware the power pour is locally constricted.

**Evidence:** In2 zones: +3V3 (72.5,116.7)-(94.7,171.3) prio 13; In2 GND segments listed at those coordinates in the live PCB.

**Fix:** None required; if the fire-return corridor is widened per the high finding, keep it along the same diagonal to avoid new +3V3 constrictions.

**Sources:** Live PCB parse

### INFO [unverified] — Two +3V3 vias violate the board's own hole-to-hole spacing (0.163 vs 0.1995 mm) and a duplicated via is stacked at (82.04,135.95)
*grounds/fab hygiene — confidence high*

DRC warnings that survive tonight's re-route: same-net via wall spacing 0.163 mm between +3V3 vias at (91.7,126.28)/(91.81,126.73) (below the 0.1995 mm board constraint — JLC may flag or silently accept same-net), and two identical Net-(U21-OV1) vias exactly co-located at (82.04,135.951805) (duplicate object).

**Evidence:** drc.json: hole_to_hole warning and holes_co_located warning with those coordinates.

**Fix:** Nudge one +3V3 via 0.1 mm; delete the duplicate via.

**Sources:** drc.json

### INFO [unverified] — Mounting holes H1-H8 are plated but electrically isolated from GND
*grounds/chassis — confidence high*

All eight Ø2.2 mm mounting holes carry no net (isolated 2.6 mm pads), so the avionics sled/standoffs are galvanically isolated from board ground — defensible for a rocket (avoids uncontrolled chassis loops) but worth confirming it is a decision, not an omission; grounding one hole is a common ESD drain choice.

**Evidence:** Pad dump: H1-H8 thru_hole net='' drill 2.2, pad 2.6; plane fills clear around them (the 6.4-6.9 mm2 voids at each H location).

**Fix:** Confirm intent; optionally tie one standoff to GND through a 1 nF||1 M network if chassis bonding is ever wanted.

**Sources:** Live PCB parse

### INFO [unverified] — Load-switch source stitching at Q7/Q10 is thinner than at Q1/Q8/Q9
*grounds/load-switch FETs — confidence high*

Camera switch Q7's source pads (B.Cu) reach their nearest GND vias at 1.80/2.14 mm with only ~36% surrounding pour ring; LoRa switch Q10 at 0.96/1.79 mm with ~23% ring — connected and adequate for the ~0.5-1 A loads, but the thinnest of the five PMPB14XN grounds (Q8 servo: 0.06 mm; Q9: 0.00 mm — vias effectively in-pad).

**Evidence:** Parse: Q7.4/8 at (89.79/89.92,119.2-120.1) inside B.Cu GND fill (pip=True), arc coverage r=1.0 mm = 32/90; Q10 21/90; nearest-via distances as listed.

**Fix:** Optionally add one GND via within 0.5 mm of each of Q7.8 and Q10.8.

**Sources:** Live PCB parse; Nexperia PMPB14XN datasheet pinning (Table 2)


**Affirmatively verified clean (layout-grounds):**

- In1.Cu and In4.Cu are genuinely solid GND: zero track segments/arcs on either layer, no other zone claims them, and the GND zone fill is a SINGLE connected polygon per layer (12157 vertices, 1415 mm2 = ~87% of the 22.35x75 mm board) with island_removal_mode 0 (no orphan islands possible) — confirmed by parse and visually against layer_In1.Cu.pdf / layer_In4.Cu.pdf
- Every void in the In1/In4 fills is accounted for: the two 8.0x3.0 mm routed slots, eight Ø2.2 mounting-hole antipads, J8/C12 THT antipads and thermal gaps, ordinary via antipads, one deliberate 2.6x1.6 mm all-layer keepout at (91.4,162.4)-(94.0,164.0), and four tiny B.Cu-only footprint keepouts under the PYROn_CONT diodes — nothing unexplained
- Exactly two ground-family nets exist (GND, 188 pins; PYRO_GND, 8 pins) and both are fully accounted for: PYRO_GND = J2.5_1/5_2 + U9 drain (pad+pins 5-8) + R73.1, tied to GND at a single star point (U9 source) plus a 1 k bleed (R73, Yageo RC0402 1k per BOM); J2-to-U9 fire feed verified as 1.5 mm B.Cu track + 4x 0.3 mm vias into the 2.5 mm drain pad
- DRC (severity-all, run tonight on the live file) reports 0 unconnected items — no ground pin anywhere has a net label without copper (the LoRa-board bug class is absent)
- Copper-to-edge clearance: minimum fill distance is 0.200-0.201 mm on all five GND-zone layers to the outer outline AND to both internal slots (rounded-corner geometry accounted for), meeting the board's min_copper_edge_clearance=0.2 rule (severity error, DRC clean)
- All connector grounds carry real copper: J6 USB-C 6 GND pads (A1/B12, B1/A12, S1-S4) in pour with vias 0.07-2.5 mm; J8.1 battery GND THT into all five copper layers; J1.6/7 and J4.SH1/2 shield tabs in B.Cu pour with vias <=1.4 mm; J2 pyro returns as above
- Daughterboard link pin-mapping re-verified against the daughterboard netlists exported tonight: rocket J1.3 (power via FL1) / J1.4 (switched return) match GNSS-db J3.3=VSYS / J3.4=VSS; rocket J5.1 (switched return) / J5.2 (power) match LoRa-db J6.1=VSS / J6.2=+BATT; shield tabs do not defeat the ground switching because SH-family tabs are board-local (cable carries only pins 1-5). The low-side-switching architecture itself remains the prior review's open H-class finding — unchanged tonight
- All five load-switch FETs (Q1, Q7-Q10, PMPB14XNX) match the Nexperia datasheet pinning exactly (Table 2: 1,2,5,6,7=D / 3=G / 4,8=S, DFN2020MD-6); every source pad lies inside GND pour on its own layer; part is rated 40 V / 11.5 A / 15-18 mOhm — no swapped source/drain anywhere
- Molex 47948-0001 antenna (U14): datasheet states 'requires no ground clearance' / 'No removal of ground layers from beneath the antenna is needed' (on-ground MID antenna) — the solid In1/In4 under U14 is correct by design; pads 1-3 unconnected matches the 'dummy pads' in the datasheet; RF feed (U15.1 -> L1 -> U14.4) has 8 GND vias within 1.5 mm of the trace
- GND via stitching: 163 through-vias (F.Cu-B.Cu, filled+capped class) tie In1/In4 together; worst any-point-to-via gap on the whole board is 6.9 mm; RF/crystal neighborhoods are properly stitched (U14: 7 vias <=3 mm; Y2: 4; Y4: 4; U15: 4; U17: 3 within 3 mm); 34 vias within 2 mm of the board edge (~5.7 mm average spacing)
- Crystals: Y2/Y4 (ECS-400 40 MHz) GND pads land on solid plane with vias 1-3 mm away and sit over unbroken In1 (both clear of the slots); Y1/Y3 are the 32 kHz crystals with no ground pins by design
- Stackup integrity: In2 carries only power nets (+3V3 / V_MCU_SWTCH / VBATT zones, distinct priorities 13/14/22), the deliberate GND fire-return trace, and one stray signal (EXP_12, 5.6 mm) — every In3 signal references solid In4; every F.Cu signal references solid In1; zone priorities are unique per layer across all 33 top-level zones and every non-keepout zone is filled (no orphan/unfilled zones)
- Battery input is now J8 = JST VH B2P-VH THT (prior blocker B-3's connector portion fixed): 2.7 mm pad / 1.7 mm drill = 0.5 mm annular ring; C12 (10 mF can) 2.54/1.0 = 0.77 mm ring; C12.2's fire-side ground has 94% plane arc contact on In1/In4 plus a dedicated solid In2 trace into its barrel
- The In1-vs-In4 comparison shows byte-identical fill geometry (same vertex count, bbox, area), i.e. no layer-specific defect on either plane; prior review's 'In3/In4 never visually checked' gap is now closed (both PDFs eyeballed)


**Checks not completed (layout-grounds):**

- TPS62152 (U18) datasheet layout section could not be fetched tonight (TI PDF not in scratch, web fetch not attempted after earlier 403s) — the '0 vias in EP' finding is graded on general TI QFN PowerPAD practice, not an exact quote
- Espressif's numeric EP-via-count recommendation for ESP32-P4/S3 chip-down designs is not present in the local p4-hdg.txt / p4-ds.txt extracts and was not re-fetched — the EP via findings cite reference-design practice rather than a normative sentence
- Intent of the two 8x3 mm board slots and of the U9-area all-layer copper keepout is undocumented (no mention in power-eco.md, schematic-review.md, or the prior review) — analyzed as-built; if the keepout is a deliberate guided-return isolation, the high finding's fix should widen the guided path rather than restore pour
- JLCPCB's current official capability page was not re-fetched; edge-clearance verdict is against the board's own 0.2 mm rule (JLC's published minimum), not tonight's live capability listing
- In3 GND-pour island-by-island single-via connectivity audit not performed (36 fill fragments; island removal confirmed on, DRC connectivity clean, visual PDF check only)


## A.layout-mfg — Layout — manufacturability

*(verify pass for this domain did not run — session limit; tags below are [unverified] except where the main loop spot-checked)*

### BLOCKER [unverified] — C12 10 mF can: still no courtyard, still placed over ~15 populated bottom-side parts, still no polarity silk (prior B-2/H11 NOT fixed by tonight's re-route)
*courtyards/assembly-collision — confidence high*

The Ø18x25 mm Chemi-Con KYC radial can (C12) cannot seat: its body circle still covers live pyro/power parts on the same (bottom) side, the footprint still has zero courtyard geometry so DRC remains structurally blind to the collision, and the only polarity mark is a single ambiguous silk circle — a 10,000 uF 16 V electrolytic installed reversed on the pyro energy store can vent violently.

**Evidence:** Live rocket-computer.kicad_pcb: C12 (Footprints:EKYC160ELL103MM25S) at (84.89,144.34) B.Cu, crtyd_shapes=0, silk = 1 fp_circle only, ref hidden (mfg_audit.py/mfg_audit2.py in scratchpad/review). Same-side neighbors inside the r=9.0 body: Q4 d=1.02 mm, Q6 2.16, R17 2.46, Q3 2.53, R16 3.22, R23 3.27, C10 4.29, R64 5.55, C9 5.63, U13 SOIC-8 flash 6.91 (NEW under the can vs the 07-30 list), U21 TPS2121 7.55, R20 8.05, U7 8.39, U8 8.86. render_bottom.png shows the can body over the FET row. Tonight's re-route moved U18/L6 out but moved U13 in. drc.json: missing_courtyard in ignored_checks; no courtyard violation reported for C12.

**Fix:** Before fab: relocate the can or clear all parts from under its Ø18 body + ~1 mm margin; add a real Ø19-20 mm courtyard to the EKYC160ELL103MM25S footprint; add unmistakable +/- silk at both leads; set missing_courtyard to error and re-run DRC. If a stood-off/staked mounting is intended, document standoff height and add RTV/strap retention for flight vibration.

**Sources:** prefab-review-2026-07-30.md B-2/H11/H13; live .kicad_pcb parse; render_bottom.png

**Disposition (2026-08-05, owner): placement WAIVED.** The can intentionally stands off above the parts on that side of the board — a board-area vs vertical-space/serviceability tradeoff — including the taller C56 tantalum beneath it. Residual actions (polarity silk, formed leads + staking, true Ø18×25 STEP, bay-height check) remain — see NB-2 in the executive summary.

### HIGH [unverified] — Fiducials: still exactly one, still buried under the USB-C shell (the board's only DRC error), none on the bottom side (prior H12/M16/L9 NOT fixed)
*fiducials — confidence high*

FID1 is the board's only fiducial and it sits fully inside the USB-C (J6) courtyard where the connector shell covers it, so a double-sided assembly with a 0.35 mm-pitch chip-down QFN-104 (U17 top) and 0.4 mm-pitch X2QFN (U21 bottom) has zero usable vision-alignment marks on either side; the 0.5 mm copper dot is also below the 1 mm many assemblers require.

**Evidence:** Live file: FID1 (Fiducial:Fiducial_0.5mm_Mask1mm, pad 0.5 mm Cu / 1.0 mm mask) at (84.32,168.07) — coordinates unchanged since the 07-30 review; J6 (GCT_USB4110GFA) F.CrtYd bbox x 77.91..90.81 / y 162.89..171.32 computed from live geometry contains it. drc.json (tonight): the single error-severity violation is courtyards_overlap 'Footprint J6' (84.36,164.79) vs 'Footprint FID1' (84.32,168.07). Footprint scan: no other Fiducial:* on 237 footprints; 82 footprints on B.Cu incl. U21/U19/U23 QFNs get no bottom fiducials. render_top.png shows no visible fiducial.

**Fix:** Delete or relocate FID1 and add three global fiducials per assembly side (1.0 mm copper dot, 2.0 mm mask opening), placed asymmetrically near corners, >=5 mm from the edge and clear of all courtyards. This also clears the only DRC error.

**Sources:** drc.json; live .kicad_pcb; prefab-review-2026-07-30.md H12

### HIGH [unverified] — Checked-in bom.csv/bom.xlsx are stale and contradict the live board (wrong capacitor sizes, e.g. 47 uF 1206 vs live 22 uF 0805) — kitting from them places wrong parts
*file-hygiene/BOM — confidence high*

The BOM files sitting in the fab folder (dated Jul 24) no longer describe the board being fabbed: capacitor case sizes and values differ from the live layout, so any quote/kit/consigned-assembly built from them delivers parts that do not fit the lands.

**Evidence:** hardware/rocket-computer/bom.csv line 5: 'C9, C10, C11, C13, C65 ; C_1206_3216Metric ; 5 ; 47 uF' and line 4: 'C7, C18 ; C_0603_1608Metric ; 22 uF' — but the live .kicad_pcb has C7/C9/C10/C11/C13/C18 all as 22 uF in C_0805_2012Metric and C65 as 47 uF 0805 (fp_full.json dump, tonight). bom.xlsx/bom-priced.xlsx carry the same Jul-24 date. The fresh netlist-derived export in scratchpad/review/bom.csv shows the correct current parts (incl. MPN+Mfr columns now filled).

**Fix:** Regenerate bom.csv/bom.xlsx from the live schematic as part of the fab-package export, and commit them with the layout; delete or clearly mark superseded pricing sheets.

**Sources:** repo bom.csv vs live .kicad_pcb footprints

### MEDIUM [unverified] — DRC config still ignores missing_courtyard (plus pth/npth_inside_courtyard, footprint_type_mismatch) — the exact blindness that hides C12, and now also the new U20 buck
*DRC-config — confidence high*

The project's rule_severities still set missing_courtyard (and pth/npth_inside_courtyard, footprint_type_mismatch, footprint_filters_mismatch) to ignore, and min_silk_clearance is 0, so the two courtyard-less footprints (C12, U20) and silk-on-pad collisions pass DRC silently; prior finding I26(5) was not acted on.

**Evidence:** rocket-computer.kicad_pro rule_severities: 'missing_courtyard': 'ignore', 'pth_inside_courtyard': 'ignore', 'npth_inside_courtyard': 'ignore', 'footprint_type_mismatch': 'ignore', 'footprint_filters_mismatch': 'ignore'; rules.min_silk_clearance = 0.0. Footprint scan: exactly two of 237 footprints lack any CrtYd geometry — C12 and U20 (Footprints:DRL0006A, TLV62569 buck at (89.14,104.04) F.Cu, whose silk outline crosses the FB_DCDC divider pads: drc.json silk_over_copper 'Segment of U20' vs pads of R74/R75/C93 at (88.19,104.94); nearest neighbor C47 center is only 1.68 mm away).

**Fix:** Add courtyards to C12 and U20 (DRL0006A), set missing_courtyard to error (at least warning), and re-run DRC before generating the fab package.

**Sources:** .kicad_pro; drc.json; fp_full.json

### MEDIUM [unverified] — U11 NAND EPAD still gets a 100% paste aperture — pad-level F.Paste defeats its six segmentation windows (prior M19 NOT fixed)
*paste/stencil — confidence high*

U11's 3.45x4.34 mm ground EPAD still lists F.Paste in the pad layer stack, producing a full 15 mm2 stencil aperture on top of the six drawn segmentation windows, unlike every other EPAD on the board (40-79% coverage) — float/tilt and perimeter-lead opens risk on the logging flash.

**Evidence:** Live .kicad_pcb: U11 (GD5F NAND, 8L_WSON footprint) pad 9, size 3.4544x4.3434, layers ['F.Cu','F.Mask','F.Paste'] plus 6 drawn F.Paste polys totalling 11.43 mm2 (mfg_audit2.py). Contrast U17 EPAD: layers ['F.Cu','F.Mask'] with 9 windows / 22.47 mm2 = 40% of 7.5x7.5; U15 54%; U18 40%; U19 79%.

**Fix:** Remove F.Paste from U11 pad 9's layer list so the six drawn windows define the stencil (~60-76% coverage), matching the other EPADs.

**Sources:** live .kicad_pcb parse; prefab-review-2026-07-30.md M19

### MEDIUM [unverified] — Same-net +3V3 via pair with 0.163 mm hole-to-hole web — below JLCPCB's 0.2 mm same-net via minimum (new since re-route)
*DFM-drill — confidence high*

Two +3V3 vias are drilled 0.163 mm edge-to-edge, under JLCPCB's published 0.2 mm via-to-via (same net) minimum — drill breakout/barrel damage risk and a likely DFM hold at order time.

**Evidence:** drc.json hole_to_hole warning: 'board setup constraints min 0.1995 mm; actual 0.1632 mm' between Via [+3V3] (91.7,126.28) and Via [+3V3] (91.81,126.73), both 0.4/0.3 mm. JLCPCB capabilities page: 'Via-to-Via (same net): 0.2 mm'.

**Fix:** Nudge one via so hole edges are >=0.2 mm apart (>=0.5 mm center-center), or delete one — they are the same net.

**Sources:** drc.json; jlcpcb.com/capabilities/pcb-capabilities (fetched 2026-08-04)

### MEDIUM [unverified] — Still zero testpoints on the board (prior M22 NOT fixed); USB-C chain remains the only programming/console path
*testpoints — confidence high*

No testpoint footprints or TP references exist anywhere, and U15's U0TXD/U0RXD are still unconnected, so bring-up of two chip-down MCUs depends entirely on the USB-C-to-FSUSB63 mux chain with no bench fallback and no probing aids.

**Evidence:** Footprint scan of live .kicad_pcb: 0 TestPoint footprints, 0 TP* refs (mfg_audit.py). netlist.net still contains 'unconnected-(U15-U0TXD/GPIO43-Pad49)' and 'unconnected-(U15-U0RXD/GPIO44-Pad50)'.

**Fix:** Add ~6 small pads/testpoints (0.8-1.0 mm) for S3 UART0, P4 UART0, both CHIP_PU nets, and a labeled GND before fab; on a 22x75 mm 6-layer board this is cheap insurance.

**Sources:** netlist.net; footprint scan; prefab-review-2026-07-30.md M22

### MEDIUM [unverified] — Stray 'rocket-computer 2.kicad_pcb' (stale Aug-2 layout) plus 34 backup zips inside the project folder — wrong-file-fab hazard
*file-hygiene — confidence high*

A 4.66 MB stale copy of the layout with a near-identical name sits next to the live board file, and 34 dated backup zips live in the project tree; plotting gerbers from the Aug-2 copy (which predates tonight's re-route and the Aug-4 power work) would silently fab an obsolete board.

**Evidence:** ls hardware/rocket-computer/: 'rocket-computer 2.kicad_pcb' dated Aug 2 04:06 (mode 600) beside rocket-computer.kicad_pcb dated Aug 4 21:04; rocket-computer-backups/ holds 34 zips (16:35 through 21:04 tonight). The review brief itself had to warn every reviewer off the stale file. gerbers/ is empty, so the stray .kicad_pcb is the one live wrong-source risk.

**Fix:** Delete 'rocket-computer 2.kicad_pcb' or move it outside the project (it is fully superseded; git history and the backups dir already preserve state), and generate the fab package only from rocket-computer.kicad_pcb.

**Sources:** directory listing

### LOW [unverified] — Two vias drilled at the identical coordinate on Net-(U21-OV1) — duplicate drill hit
*DFM-drill — confidence high*

Exactly co-located stacked vias produce a double drill hit at one coordinate, which fab DFM will flag and which serves no purpose.

**Evidence:** drc.json holes_co_located: Via [Net-(U21-OV1)] at (82.04,135.951805) listed twice at the identical position.

**Fix:** Delete one of the two vias.

**Sources:** drc.json

### LOW [unverified] — 0.13 mm dangling PYRO3_FIRE stub on In3.Cu survived the re-route
*layout-hygiene — confidence high*

The dangling 0.13 mm track stub on a pyro fire net, already flagged 07-30, is still present after tonight's re-route — dead copper on a firing net.

**Evidence:** drc.json track_dangling: 'Track [PYRO3_FIRE] on In3.Cu, length 0.1300 mm' at (80.27,114.56); identical item appears in prefab-review-2026-07-30.md.

**Fix:** Delete the stub (Edit > Cleanup tracks & vias).

**Sources:** drc.json

### LOW [unverified] — Function labels and the buzzer polarity mark print on exposed pads (will be clipped), plus an orphan '+' next to unconnected mounting hole H3
*silkscreen — confidence high*

Several bottom-silk function labels sit on pad openings, so the fab will clip them into partial illegibility — including LS1's '+' polarity mark which sits on its own pad and will vanish — and a stray '+' floats next to a plain mounting hole where it reads as a false wiring hint.

**Evidence:** drc.json silk_over_copper (23 warnings): 'LoRa' over J5 pad 4 (79.69,101.78); 'GPS' over J4 SH2 (80.16,111.82); 'CAM' over R30/R32/C7 pads (86.36,119.03); 'GND' over R27 pad 2 (74.02,106.99); '- Batt +' segments over J8 (silk_overlap x5); 'Footprint text of LS1 (+)' over Pad LOAD+ of LS1 (81.18,151.48). Orphan '+' B.SilkS at (94.38,100.36): nearest copper is mounting hole H3 (no net) 2.70 mm away, no polarized part within 5 mm (pad-proximity scan).

**Fix:** Nudge each label off the pad openings, move LS1's '+' beside the pad, and delete or re-anchor the orphan '+'.

**Sources:** drc.json; pad-proximity scan

### LOW [unverified] — C15 tantalum polarity silk still clipped at the board edge; C15/C14/S3 courtyards extend past the outline (prior M21 NOT fixed)
*silkscreen/courtyards — confidence high*

C15's polarity-band silk still crosses the board edge so the printed pattern will be truncated on the '+' end, and three footprints' courtyards extend 0.03-0.08 mm past the milled outline.

**Evidence:** drc.json silk_edge_clearance: 'Segment of C15 on B.Silkscreen' at (93.972,109.665), (94.667,109.665), (94.667,117.519) vs edge x=94.71 — same three hits as 07-30. Courtyard-vs-outline scan: C15 B.CrtYd to x=94.79, C14 F.CrtYd (94.74,115.77), S3 F.CrtYd (94.76,115.89/119.50) vs outline x max 94.71. S1/J2 silk circles also clipped (cosmetic).

**Fix:** Shift C15 inboard ~0.2 mm or trim its silk; accept S3 overhang only if the switch actuator deliberately overhangs the edge.

**Sources:** drc.json; mfg_audit2.py courtyard scan

### LOW [unverified] — Silk legibility below JLCPCB minimums: 0.5 mm 'X'/'Y' axis texts and dominant 0.12 mm stroke width
*silkscreen — confidence high*

The IMU axis-label texts are 0.5 mm tall (below the project's own 0.8 mm minimum and JLC's 1.0 mm) and most silk is drawn at 0.12 mm stroke with a few 0.10/0.05 mm items, below JLC's stated >=0.15 mm line width — expect blurred or dropped features.

**Evidence:** drc.json text_height warnings: PCB text 'Y' (91.86,127.77) and 'X' (92.86,126.6). Stroke census of live file: 0.12 mm x377, 0.127 x74, 0.1 x10, 0.05 x2 silk widths. JLCPCB capabilities: silkscreen min text height 1.0 mm, min line width >=0.15 mm.

**Fix:** Enlarge X/Y to >=1.0 mm/0.15 mm stroke (they also currently sit over L6's pads) or delete them; optionally bulk-edit silk widths to 0.15 mm.

**Sources:** drc.json; stroke census; JLCPCB capabilities page

### LOW [unverified] — U9 part-identity mix: TI CSD16323Q3 value in a Toshiba 'TSON Advance' footprint whose Datasheet field says TPN4R712MD; symbol has 9 pins vs 5 pads
*footprint-vs-part — confidence medium*

U9 alone among the five fire-FET slots carries a TI part number in the Toshiba footprint (the other four are TPN4R712MD), the component's Datasheet field still points at the Toshiba part, and its 9-pin symbol maps to a 5-pad footprint — electrically benign today (drain pad 5 carries PYRO_GND; pins 6-9 are redundant drain pins), but the mixed identity invites ordering the wrong part or trusting the wrong land pattern.

**Evidence:** netlist.net comp U9: value 'CSD16323Q3', footprint 'Footprints:TSON Advance_TOS', datasheet 'TPN4R712MD', MPN field CSD16323Q3/TI. drc_parity_check.json net_conflict: 'No pad found for pin 9 (PYRO_GND) in schematic' (Footprint U9) — the only parity item on the board. Net trace: PYRO_GND contains U9 pins 5-9 (DRAIN_5..DRAIN_9); footprint pad '5' (2.5x2.5) carries the net, so continuity is intact.

**Fix:** Confirm CSD16323Q3's recommended land pattern against the TSON-Advance footprint (both are 3.3x3.3 source/gate-leads-plus-drain-tab styles, but pad geometry differs), fix the Datasheet field, and use a matching 5-pin symbol or a CSD16323-specific footprint.

**Sources:** netlist.net; drc_parity_check.json

### LOW [unverified] — Y2/Y4 crystal footprints differ from their library copy (in-place edits, likely residue of the added-then-reverted 24 nH crystal-leg change)
*library-hygiene — confidence high*

Two board copies of XTAL_ECS-400-10-37B2-CKY-TR no longer match the Footprints library, so any future 'update footprints from library' silently rewrites them — the classic path for the reverted crystal-inductor geometry to resurface or disappear unnoticed.

**Evidence:** drc.json lib_footprint_mismatch: Y4 (74.955,102.805) and Y2 (84.735,137.845), both 'XTAL_ECS-400-10-37B2-CKY-TR does not match copy in library Footprints'. (07-30 the same class of finding named Y1/Y3 — the drift moved, not resolved.)

**Fix:** Decide the canonical geometry, push it to the library, and re-sync all instances.

**Sources:** drc.json

### LOW [unverified] — Version identity lives only in the PCB title block (rev V9); all six schematic sheets have no title block at all
*documentation — confidence high*

The schematics print with no title, revision, or date on any sheet, so a fab/assembly package or archived PDF cannot be matched to board rev V9 except by file dates.

**Evidence:** grep title_block across rocket-computer.kicad_sch, power.kicad_sch, central_processing_p4.kicad_sch, esp32s3_outputs.kicad_sch, in_sensors.kicad_sch, external_connections.kicad_sch: no matches. rocket-computer.kicad_pcb title_block contains only (rev "V9"). The silk '${REVISION}' does resolve — render_top.png shows 'Tinker Rocket V9'.

**Fix:** Add title/rev/date title blocks to the root and child sheets (KiCad propagates via page settings) before exporting the fab-package schematic PDF.

**Sources:** schematic files; render_top.png

### INFO [unverified] — No visible reference designators anywhere on silk (all 237 hidden) — deliberate, but bring-up and rework depend entirely on the assembly PDFs
*silkscreen — confidence high*

Every footprint's silk reference is hidden; only functional labels (PWR, CAM, GPS, LoRa, '- Batt +', pyro 1-4/GND) are printed. Acceptable for a dense 22x75 mm board and fine for machine assembly (F.Fab refs exist for assembly drawings), but hand-rework and field debugging will need the assembly_F/B.pdf at hand.

**Evidence:** fp_full.json: 0 of 237 footprints have a visible silk-layer Reference; board-level gr_text census shows only function labels; F.Fab reference texts present (0.04-0.15 mm thickness census).

**Fix:** None required; consider printing refs for the connectors and pyro FETs only, where field debugging is most likely.

**Sources:** fp_full.json

### INFO [unverified] — Filled+capped is a board-file declaration — the JLCPCB order must still select 'Epoxy Filled & Capped', and several parameters sit at JLC's absolute minimums with zero margin
*DFM-order — confidence high*

The KiCad setup now declares (filling yes)(capping yes), which resolves the prior via-in-pad wicking finding M20 on paper, but the option is actually applied by the JLC order form (it is their default for 6-layer, so this is a checklist item, not a defect). Separately, 63 track segments at 0.09 mm and every via at 0.05 mm annular are exactly at JLC's published 3.5 mil / min-annular limits with a +/-20% etch tolerance — legal but zero-margin.

**Evidence:** .kicad_pcb setup: (tenting front yes back yes)(capping yes)(filling yes); all 390 vias 0.4/0.3 mm — drill inside JLC's 0.15-0.55 mm epoxy-fill window. Track census: 0.09 mm x63 (PIEZZO 40 segs, GPS_ACT 9, U17 GPIO0/1 — all low-current signals), rest 0.1 mm and up. JLC capabilities: 0.09/0.09 mm min trace/space +/-20%, filled+capped default for 6-layer.

**Fix:** Add 'Via Covering: Epoxy Filled & Capped' to the order notes/fab README (mirroring the lora fabrication-notes commit), and don't tighten anything further in this area.

**Sources:** .kicad_pcb setup; via/track census; JLCPCB capabilities page

### INFO [unverified] — All eight M2 mounting-hole pads are electrically floating (not tied to GND)
*mechanical — confidence high*

H1-H8 have 2.6 mm pads plus 3.8 mm top/bottom rings with no net, so the airframe standoffs are galvanically isolated from board ground — defensible for a rocket (avoids ground loops through the airframe) but should be a documented choice, and the floating rings sit adjacent to the orphan '+' silk (separate finding).

**Evidence:** fp_full.json: H1-H8 MountingHole_2.2mm_M2_DIN965_Pad_TopBottom, all pads net=None, drill 2.2 mm, 0.2 mm annular, >=2.7 mm from board edges.

**Fix:** None if isolation is intended; otherwise assign GND to one or more holes.

**Sources:** fp_full.json


**Affirmatively verified clean (layout-mfg):**

- Filled+capped via declaration is now consistent with the via-in-pad usage (prior M20 resolved): setup has (filling yes)(capping yes); U17 EPAD carries 6 vias-in-pad, U15 4, U19 6+3, U9 1 — all 0.3 mm drills inside JLCPCB's 0.15-0.55 mm epoxy-fill window
- Via geometry is uniform board-wide: all 390 vias are 0.4/0.3 mm (0.05 mm annular = JLC minimum), no microvias/buried vias, min_through_hole 0.3 mm — single via class simplifies the filled+capped order
- EPAD paste segmentation correct on U17 (9 windows, 22.47 mm2 = 40% of 7.5x7.5), U15 (54%), U18 (40%), U19 (79%), U23 (vendor-style drawn mask+paste poly windows; pad itself B.Cu-only is intentional) — only U11 wrong (reported)
- No through-hole pad anywhere carries a paste aperture (full-board scan: zero THT pads with F/B.Paste in layers)
- P4 0.35 mm-pitch mask dams are buildable: 0.18 mm pads leave 0.17 mm dams at pad_to_mask_clearance=0, above JLC's 0.10 mm minimum; DRC solder_mask_bridge (error severity): zero hits; copper_sliver: zero hits
- Board outline is closed and sane: 24 Edge.Cuts shapes forming a 22.35 x 75.00 mm rounded rectangle plus two 8.0 x 3.0 mm internal rounded slots (far above routing minimums); copper_edge_clearance enforced at 0.2 mm as error with zero violations
- Mounting provision: 8x M2 holes (2.2 mm drill, 2.6 mm pad, 3.8 mm rings both sides), all >=2.7 mm from board edges and clear of courtyards; all electrical THT pads have >=0.3 mm annular rings
- gerbers/ directory contains only .gitkeep — no stale plotted fab outputs to grab by mistake
- Silk ${REVISION} resolves correctly: render_top.png shows 'Tinker Rocket V9' matching the PCB title-block rev
- Schematic-to-PCB parity is clean apart from the U9 pin-9 symbol-hygiene warning (net continuity verified intact through footprint pad 5 on PYRO_GND); no missing or extra footprints
- Polarity silk present and correct on CR1/CR2 Schottkys (cathode band), D1-D4 SOD-523 (cathode bar), D6-D8 LEDs (cathode dot), C56 tantalum (band, unclipped), and battery input labeled '- Batt +' with pyro channels labeled 1-4/GND on bottom silk
- Fresh BOM export (scratchpad review/bom.csv, from tonight's schematic) has MPN + Manufacturer filled for essentially all lines including U20 TLV62569DRLR — the 07-30 empty-MPN finding is fixed at the schematic level
- Drill-to-drill spacing is clean everywhere except the single reported +3V3 pair (DRC hole_to_hole scan, min 0.1995 mm constraint)
- Track widths: bulk of routing at 0.1-0.127 mm with power at 0.2-1.5 mm; only 63 segments at the 0.09 mm minimum, all on low-current signal nets (PIEZZO, GPS_ACT, P4 GPIO0/1, J4 camera pads)


**Checks not completed (layout-mfg):**

- JLCPCB's official fiducial-requirement help article could not be fetched (page returned navigation only); the fiducial finding is grounded in the DRC courtyard error, the live geometry, and their general capabilities page instead
- TI CSD16323Q3 vs Toshiba TSON-Advance land-pattern dimensional equivalence not verified against the TI datasheet (not fetched in session time) — carried as a low 'verify before ordering' finding on U9
- Did not re-run DRC with missing_courtyard promoted to error, since that requires editing the user's .kicad_pro; courtyard absences were instead established by parsing all 237 footprints directly
- C12 can height (25 mm) vs airframe bay clearance remains unverifiable from the repo files — the ECO's open mechanical-fit item from 07-30 is still open
- Stencil thickness / assembly-drawing package for the 0.35 mm-pitch P4 (prior I27 recommended <=0.10 mm stencil) cannot be checked from the design files — it is an ordering parameter; carry it into the fab notes


## A.bom-parts — BOM & part selection

*(verify pass for this domain did not run — session limit; tags below are [unverified] except where the main loop spot-checked)*

### HIGH [unverified] — TPS62152 PVIN pin 12 still floating in schematic and copper (prior H1/H10/H15/H17/H22 unfixed)
*BOM/part application - 3V3 buck — confidence high*

One of the two paralleled power-stage input pins of U18 (TPS62152RGTR) is unconnected, contrary to the TI datasheet requirement that both PVIN pins tie to the input source; input current concentrates in a single 0.26 mm QFN pin and the floating pad sits adjacent to SW. This was flagged five separate times in the 2026-07-30 review and survives tonight's re-route untouched.

**Evidence:** erc.json: pin_not_connected item 'Symbol U18 Hidden pin 12 [PVIN, Input, Line]' (the pin is hidden in the symbol, which is why it keeps being missed). Live rocket-computer.kicad_pcb U18 (Footprints:QFN50P300X300X100-17N at 88.78,126.365): pad 12 net = 'unconnected-(U18-PVIN-Pad12)' while pads 10/11/13 = Net-(U18-AVIN). TI SLVSAL5E Table 6-1: pins '11,12 PVIN — Supply voltage for power stage. Connect to same source as AVIN.'

**Fix:** Unhide/connect pin 12 in the U18 symbol to the AVIN/PVIN input net and update the PCB so pad 12 joins the input copper at pin 11 (it is physically adjacent). Re-run ERC to confirm the hidden-pin warning clears.

**Sources:** TI TPS62150/1/2 datasheet SLVSAL5E (fetched tonight); erc.json; live .kicad_pcb

**Disposition (2026-08-05): FIXED & VERIFIED in-session.** U18 pad 12 now carries `Net-(U18-AVIN)` (symbol pin 12 stacked on pin 11; AVIN pour refilled over the pad — no new tracks). Netlist, ERC (`pin_not_connected` 47→46, `pin_not_driven` 1→0) and DRC (0 unconnected, no new violations) all re-verified. See NB-3.

### HIGH [unverified] — BOM does not pin ESP32-P4 chip revision v3.0+ although the circuit is v3.x-only
*BOM lifecycle - U17 — confidence high*

The live schematic implements exactly the Espressif v3.0+ reference (pin 54 = VDD_HP_1 via R76, 499k/499k FB divider R74/R75 + 22 pF C93, 1 MOhm R77 on DP), which the HDG says 'must be populated' for v3.0+ and must NOT be populated for v1.0/v1.3; the BOM line is bare 'ESP32-P4NRW32' with no revision constraint, and v1.x stock is still in distribution (v3.0 only reached market ~March 2026). A v1.x chip soldered to this board drives its FB_DCDC pin against a divider its own HDG says should be absent, and pin 54 semantics differ.

**Evidence:** P4 HDG (p4-hdg.txt lines 116-121): 'The main differences between chip revisions v1.0/v1.3 (not recommended for new designs) and v3.0 and later versions include the definition of pin 54, the 1 MOhm resistor on the DP pin, the two 499 kOhm resistors and one 22 pF capacitor in the DCDC circuit... In chip revisions v3.0 and later versions, pin 54 of ESP32-P4 is defined as VDD_HP_1; in chip revisions v1.0 and v1.3, this pin is defined as NC.' Netlist: R76 0R ESP_VDD_HP->VDD_HP_1 (U17.54), R74/R75 499k with C93 22pF on FB_DCDC (U17.78), R77 1M on CEN_D+. bom.csv line U17: 'ESP32-P4NRW32','Espressif' with no revision note. LCSC C22387510 listing does not distinguish revisions.

**Fix:** Add an explicit 'chip revision v3.0/v3.1 ONLY' constraint to the U17 BOM line and purchasing docs; verify incoming-chip marking per the ESP32-P4 Chip Revision v3.x User Guide / SoC Errata chip-identification section before assembly, and confirm the firmware ESP-IDF version supports v3.x.

**Sources:** Espressif ESP32-P4 HDG + datasheet v0.7 (scratchpad p4-hdg.txt/p4-ds.txt); Espressif ESP32_P4_v3.x_Upgrade news; cnx-software 2026-03-23; LCSC/JLCPCB C22387510

### HIGH [unverified] — Pyro gate-drive parts unchanged after prior H21: DTC123J (2.2k/47k) + R22 100k lets a single enabled P4 pull-up fire or arm a channel
*BOM part selection - pyro safety — confidence high*

The prior review's must-fix cluster showed a P4 internal weak pull-up (45 kOhm typ) on a fire GPIO biases DTC123J (R1 2.2k / R2 47k) above its guaranteed-ON input level and pulls a TPN4R712MD gate low, fully enabling a fire FET while V_CAP is always charged; the arm gate has the same weakness through R21 100R vs R22 100k on the CSD16323Q3. Tonight's BOM still selects DTC123JEBTL for Q3-Q6 and R22 is still 100k — the recommended part-swap hardening did not land.

**Evidence:** bom.csv (tonight): 'Q3-Q6','DTC123J',...,'DTC123JEBTL','Rohm'; netlist: Net-(U9-GATE) = R21 100R from PYRO_ARM + R22 100k to GND; R15/R16/R17/R23 10k pullups from fire-FET gates to V_CAP. prefab-review-2026-07-30.md H21 (line 329-336) with the datasheet math (DTC123J V_I(on) min 1.1 V vs ~1.65 V from a 45 k WPU; CSD16323Q3 Vgs(th) max 1.4 V vs 2.27 V at the arm gate).

**Fix:** Change Q3-Q6 to DTC123EEBTL (2.2k/2.2k, same EMT3 footprint) and R22 from 100k to 10k as the prior review computed; re-verify against P4 IO-MUX boot states.

**Sources:** prefab-review-2026-07-30.md H21; Rohm DTC123J/DTC123E bias-resistor values; live bom.csv/netlist.net

### HIGH [unverified] — S3 RF matching pi network deleted in tonight's edit: LNA_IN now wired straight to the antenna with only a 4.3 nH shunt, no tuning pads
*BOM/part selection - RF — confidence high*

The Jul-24 BOM and the Jul-30 review both carried a CLC pi at LNA_IN (C24 1.5 pF shunt, L2 2.7 nH series, C23 1.5 pF shunt) that the prior review verified as inside Espressif's recommended windows; in the live files L2/C23/C24 no longer exist and U15 pin 1 (LNA_IN) connects directly to the Molex 47948-0001 feed with only L1 4.3 nH to GND. With no series element and no spare pads, the RF match cannot be bench-tuned after fab, risking degraded BLE/Wi-Fi range and emissions margin.

**Evidence:** netlist.net: Net-(U14-Feed) = {L1.2, U14.4, U15.1} only; tonight's bom.csv has no 1.5 pF or 2.7 nH lines (old hardware/rocket-computer/bom.csv rows 11-12 and L2 line show they existed on Jul 24). prefab-review-2026-07-30.md M15: 'CLC pi at LNA_IN ... is inside Espressif's recommended windows (C 1.2-1.8 pF, L 2.0-3.0 nH)'. Espressif HDG asks designs to reserve a CLC matching network; Molex 47948-0001 is an on-ground antenna whose application spec expects board-specific matching.

**Fix:** Restore the pi footprints (series + two shunts) between LNA_IN and the antenna feed even if shunt caps ship DNP; populate per bench tune. If the single-shunt topology is a deliberate re-design, document the S-parameter basis before fab.

**Sources:** netlist.net; both bom.csv generations; prefab-review-2026-07-30.md M15; Molex AS-47948-001

**Disposition (2026-08-05, owner): WAIVED.** Deliberate change implementing Molex AS-479480001 Rev G §6.0's nominal L-network (4.3 nH shunt only, 0 Ω series as direct trace), per base-station review A1 (resolved 2026-08-04, `hardware/base-station/prefab-review-2026-08-02.md`); the no-tune/no-VNA-break-point trade is accepted. See NB-1 in the executive summary. The 50 Ω feed-width item remains actionable.

### MEDIUM [unverified] — R74/R75 (499k FB divider) and R76 (0R VDD_HP_1 link) have no MPN/Mfr — the only unpinned lines in the BOM, and they are P4-core-critical
*BOM completeness — confidence high*

The three components added for the P4 v3.x core-supply loop are the only BOM lines without manufacturer part numbers; the 499k pair sets the VDD_HP base voltage (1.2 V from the TLV62569 0.6 V reference) and Espressif specifies exactly two 499 kOhm resistors, so a substitution or E24 'nearest value' at assembly moves the core rail.

**Evidence:** Tonight's bom.csv rows: 'R74,R75','499 k',...,'','',''; 'R76','0',...,'','',''. Live central_processing_p4.kicad_sch symbols R74/R75/R76 carry no MPN/Mfr properties (verified by per-symbol parse). Every other line in the 74-line BOM has MPN+Mfr.

**Fix:** Add MPNs: R74/R75 = Yageo RC0402FR-07499KL (1%), R76 = RC0402JR-070RL (or equivalents already used board-wide), plus Mfr fields, and regenerate the BOM.

**Sources:** live schematic fields; tonight's bom.csv

### MEDIUM [unverified] — Repo BOM files are 11 days stale and describe a different board (wrong flash, wrong S3 variant, deleted parts still present)
*BOM drift — confidence high*

hardware/rocket-computer/bom.csv, bom.xlsx and bom-priced.xlsx (all Jul 24) disagree with the live schematic on at least a dozen lines — W25Q64JVXGIQ vs live W25Q128JVSIQ, plain ESP32-S3 vs S3RH2, MR25H10 MRAM and TPS3840 supervisor that no longer exist, F35SQB004G vs GD5F2GQ5UEYIGR NAND, SMT-0540 vs MLT-8530 buzzer, J7 JST-PH vs J8 JST-VH battery, 47 uF/1206 bulk group vs 22 uF/0805/16V, R48 100R vs 127R. Anyone ordering or quoting from the repo files builds the wrong, previously-rejected board.

**Evidence:** File dates: bom.csv/bom.xlsx/bom-priced.xlsx Jul 24 10:53 vs schematics Aug 4. bom.xlsx sharedStrings contains 'W25Q64JVXGIQ TR', 'ESP32-S3', 'MR25H10CDFR', 'TPS3840PL30DBVR' (parse tonight); live netlist/BOM export contains none of these.

**Fix:** Regenerate bom.csv/bom.xlsx (and re-price) from the live schematic as part of the fab package, and archive or delete the Jul-24 files so they cannot be picked up by mistake.

**Sources:** file mtimes; xlsx sharedStrings parse; tonight's KiCad BOM export

### MEDIUM [unverified] — Indicator LEDs are driven at 40-140 uA through 10k resistors — blue/green likely invisible in daylight
*BOM values - indicators — confidence medium*

All three 0402 LEDs use 10 kOhm series resistors from 3.3 V sources: green D6 (XL-1005UGC, InGaN Vf ~2.8-3.0 V) gets ~30-50 uA, blue D7 (APHHS1005QBC/D, Vf ~2.7-3.0 V) ~30-60 uA, red D8 (NCD0402R1, Vf ~1.9 V) ~140 uA. At these currents the blue and green indicators will be marginal indoors and effectively invisible outdoors at a launch site, defeating status indication.

**Evidence:** netlist.net: R65 10k +3V3->D6 anode, R66 10k IND_1(U17 GPIO27)->D7, R70 10k IND_2(U17 GPIO26)->D8; bom.csv: R group 'R4,R5,R7,R15-R17,R23,R25,R29,R31,R33,R35-R37,R42,R46,R52,R65,R66,R70','10 k'. Current = (3.3-Vf)/10k.

**Fix:** If visible indication is wanted, drop the three series resistors to 1-2.2 k (0.6-1.3 mA, well within GPIO drive); if the microamp drive is a deliberate battery-saving choice, record it as such.

**Sources:** netlist.net; typical Vf from Kingbright APHHS1005QBC/D and generic 0402 InGaN/AlGaInP data (exact XL-1005UGC/NCD0402R1 curves not fetched)

### MEDIUM [unverified] — eFuse limit at R48=127R (~11.5 A typ) still exceeds the weakest protected elements (10 A battery connector, 2-3 A/pin servo header, 8.1 A return FET)
*BOM ratings - protection coordination — confidence high*

The ILIM ECO (100R -> 127R) lowered the TPS259824L limit from ~14.7 A to ~11.5 A typ (interpolated from TI's table: 182R -> 8.13 A, 100R -> 14.71 A), but the limit is still above the J8 JST-VH 10 A/contact rating, far above J3 Milli-Grid per-pin ratings carrying servo return, and above Q8's 8.1 A continuous rating — a sustained 8-11 A overload cooks those parts without ever tripping the eFuse. Prior M26/B-3 is only partially resolved.

**Evidence:** power.kicad_sch R48 = 127R (MPN RC0402FR-07127RL, verified in live fields); TI TPS25982 EC table (fetched): ILIM 8.13 A @182R / 14.71 A @100R, approx K~1479 A*Ohm -> ~11.6 A @127R; JST VH rating 10 A/contact; Nexperia PMPB14XN ID 8.1 A @25C (5.1 A @100C); netlist: J3.1/2 -> Q8 drain return.

**Fix:** Either raise RILIM toward 182R (~8 A) to protect the VH connector and servo chain, or document the servo duty envelope that justifies 11.5 A and verify J3/Q8 thermal margins for it. Note also RILIM >= 182R is required for the device's UL 2367 recognition if that matters.

**Sources:** TI TPS25982 datasheet (fetched tonight); Nexperia PMPB14XN datasheet (fetched); JST VH catalog rating; prior review M26

### LOW [unverified] — TPS62152 inductor is the datasheet minimum (2.2 uH at 1.25 MHz) with Isat 1.89 A vs 2.2 A max current limit
*BOM ratings - inductor saturation — confidence high*

With FSW tied high (1.25 MHz) TI recommends 3.3 uH ('for applications running with the low-frequency setting (FSW = high) ... 3.3 uH is recommended'); the fitted VLS3012CX-2R2M-1 (2.2 uH, Isat 1.89 A, 2.83 A temp, 74 mOhm) is fine in normal operation (calculated peak 1.37-1.46 A) but saturates below the worst-case high-side current limit (ILIMF max 2.2 A) during short-circuit/current-limit events.

**Evidence:** TI SLVSAL5E: ILIMF 1.4/1.7/2.2 A; inductor-selection text quoted above; TDK VLS3012CX-2R2M-1 ratings 1.89 A sat / 2.83 A / 74 mOhm (distributor data; TDK product page 403s). Netlist: L6 between U18 SW and +3V3; L5 and L8 same MPN with peak currents ~1.05 A and ~0.6 A respectively — comfortable.

**Fix:** Accept (typ limit 1.7 A is below Isat and DCS-Control bounds runaway) or move L6 to the 3.3 uH VLS3012CX variant for both the fsw recommendation and fault margin.

**Sources:** TI TPS62152 datasheet; TDK/DigiKey listing; TLV62569 datasheet (local) for L8 check

### LOW [unverified] — CR2 reverse-battery clamp is a 1 A Schottky across an unfused 2S pack input
*BOM ratings - input protection — confidence medium*

CUS10S30 (30 V, 1 A average) from VBAT_CON to GND is the only reverse-polarity element and sits upstream of the eFuse's protection; a reversed pack (polarized JST-VH makes this a pigtail-build error case, not a plug-in error) drives tens of amps through the diode until it fails, after which -8.4 V lands on U19/U21/U23 pins. The part's rating does not match the fault it is meant to absorb.

**Evidence:** netlist.net: CR2.1 = VBAT_CON, CR2.2 = GND; no fuse or reverse-blocking element between J8 and VBAT_CON (J8.2 -> R72 2 mOhm -> VBAT_CON); Toshiba CUS10S30 = 1 A average rectified, USC package.

**Fix:** Accept as a documented sacrificial clamp (VH polarization is the real guard), or upgrade to a higher-surge diode / P-FET reverse-blocking stage if reversed-pigtail assembly is credible in field builds.

**Sources:** Toshiba CUS10S30 datasheet (BOM-linked URL); netlist.net

### INFO [unverified] — BOM metadata errors: wrong datasheet references on U9, U11, C15/C56; footprint named for the -27L eFuse variant
*BOM hygiene — confidence high*

U9 (CSD16323Q3, TI) carries Datasheet field 'TPN4R712MD' (a different vendor's FET); U11 (GD5F2GQ5UEYIGR, GigaDevice) points at Macronix 'MX35UF4G24AD-Z4I8' and its footprint is named ..._MAC; C15/C56 (TCJE polymer tantalum) link Kemet's T491 MnO2 datasheet; U19's footprint is named IC_TPS259827LNRGER while the fitted part is TPS259824LNRGET (same RGE package; 24L adds a 16.9 V OVLO that is harmless on 2S — confirm the 24L choice is intentional). None of these affect the netlist, but they will mislead the next reviewer or assembler.

**Evidence:** Tonight's bom.csv rows U9, U11, C15/C56, U19; TPS25982 device-comparison table: 'TPS259824LNRGE 16.9 [V OVLO] Active Current Limiter / TPS259827LNRGE No OVLO'; GD5F2GQ5UEYIG confirmed 2.7-3.6 V WSON8 8x6 (gd5f.pdf product list), so the Macronix-derived footprint is dimensionally compatible but mislabeled.

**Fix:** Correct the Datasheet fields and consider renaming the U19 footprint to the fitted MPN during the next library pass.

**Sources:** bom.csv; gd5f.pdf; TI TPS25982 datasheet

### INFO [unverified] — TSON Advance footprint numbers its exposed pad '5' (duplicate) instead of symbol pin 9 — works, but only by accident of net sharing
*BOM/footprint hygiene - U6-U10 — confidence high*

The 'TSON Advance_TOS' footprint used by U6/U7/U8/U10 (TPN4R712MD) and U9 (CSD16323Q3) has two pads numbered 5 (the 2.5x2.5 mm EP and lead 5) and no pad 9; the schematic symbols pin 9 (drain/EP) therefore never maps to a pad, and the EP only gets the right net because pad '5' shares the drain net. Prior M18 re-checked against the live PCB: copper is electrically correct on all five instances, but any future part with a non-drain EP on this footprint would be silently miswired.

**Evidence:** Live .kicad_pcb U9: pad 5 (2.5x2.5) = PYRO_GND and pad 5 (0.381x0.66) = PYRO_GND, pads 1-3 = GND, 4 = gate; U6 same pattern on V_CAP/PYRO2_EXT. Netlist U9 lists pins 1-9 with 9 = PYRO_GND.

**Fix:** Renumber the EP pad to match the symbol (or add pad 9 aliasing) in the library at the next opportunity; no action required for this fab.

**Sources:** live .kicad_pcb parse; prefab-review-2026-07-30.md M18

### INFO [unverified] — Commit history claims the 24 nH crystal-leg inductor was reverted, but L3 is (correctly) still in the live files
*BOM change tracking — confidence high*

L3 (LQW15AN24NH00D, 24 nH) remains in series with U15 XTAL_P in the live schematic and BOM despite a commit message describing it as reverted. Per the S3 HDG and the prior review ('L3 24 nH on XTAL_P is literally the HDG's suggested value — do not remove it') the part SHOULD stay; the risk is that someone 'finishes' the revert based on the commit message. MPN verified real and in stock (DigiKey: 24 nH, +/-3%, 280 mA, 0.52 Ohm).

**Evidence:** esp32s3_outputs.kicad_sch symbol L3 with MPN LQW15AN24NH00D; netlist: Net-(U15-XTAL_P) = {L3.2, U15.54}, Net-(C20-Pad1) = {C20, L3.1, Y2.3}; prefab-review-2026-07-30.md line 901.

**Fix:** Add a schematic note on L3 ('per Espressif HDG - keep') and reconcile the commit narrative in the fab-package notes.

**Sources:** live schematic; DigiKey LQW15AN24NH00D listing; prior review

### INFO [unverified] — FL2 ferrite bead has ~30-40% current margin over LoRa TX peaks; buzzer switching load rides the logic rail
*BOM ratings - margins — confidence medium*

FL2 (BLM18PG471SN1D, 1 A rated, 0.2 Ohm) feeds the LoRa daughterboard whose 1 W-class TX draws ~0.6-0.7 A at pack voltage: inside rating with modest margin, ~0.13 V drop at peak. LS1 (MLT-8530, rated 3.6 V, 2.5-4.5 V range, max 95 mA) is correctly driven from the 3.3 V switched rail with CR1 flyback, but its ~100 mA audio-rate square-wave load shares V_MCU_SWTCH with the P4 IO domain and sensors — a firmware-duty consideration, not a part misfit.

**Evidence:** Murata BLM18PG471SN1D: 470 Ohm @100 MHz, 1 A, 0.2 Ohm DCR (distributor data). MLT-8530 datasheet: rated 3.6 V, operating 2.5-4.5 V, max 95 mA @2700 Hz 5 Vpp, coil 16 Ohm. Netlist: FL2 VBATT->J5.2; LS1 LOAD+ = V_MCU_SWTCH, LOAD- -> Q9, CR1 across.

**Fix:** None required; if LoRa TX current is ever specified above ~0.8 A, move FL2 to the 2 A BLM18PG221SN1 class.

**Sources:** Murata/element14 listings; LCSC MLT-8530 datasheet summary; netlist.net


**Affirmatively verified clean (bom-parts):**

- Capacitor voltage-rating pass CLEAN, checked cap-by-cap against netlist rails: every 8.4 V-rail cap is 16 V-rated — C7/C14/C18/C43 and C9-C11/C13 22 uF = CL21A226MOQNNNE (16 V X5R 0805, MPN existence confirmed at DigiKey/TME); C41/C54/C59 1 uF and C42 etc. 100 nF use Samsung 'O' (16 V) codes; C12 = Nichicon KY 16 V 10,000 uF; C15/C56 = TCJE337M016R0050 330 uF 16 V polymer at 52% derating (inside Kemet's 80% guidance, eFuse dV/dt soft-start bounds surge). All 6.3 V parts (CL05A106MQ5NUNC 10 uF, CL21A476MQYNNNE 47 uF) sit on 3.3 V-or-lower rails only.
- Prior blockers B1/B4/H9 (P4 v1.x-only circuit) are FIXED in the live files: R76 0R to pin 54 VDD_HP_1, R74/R75 499k + C93 22 pF on FB_DCDC, R77 1 M on DP — matches the Espressif v3.0+ reference exactly; TLV62569 is on Espressif's verified-DCDC list; base output 1.2 V sits inside the VDD_HP 0.99-1.3 V window; U20 VIN shares the VDDPST_DCDC rail as required; EN_DCDC (U17.79) -> U20 EN and FB_DCDC (U17.78) -> FB node per HDG.
- Prior B2/H3/H4/H19 battery-connector cluster FIXED: J8 = JST B2P-VH(LF)(SN) 10 A THT vertical (netlist J8.1 GND / J8.2 VBAT_Terminal; footprint JST_VH_B2P-VH_1x02_P3.96mm_Vertical).
- Pyro fire FETs U6-U8/U10 TPN4R712MD verified from the official Toshiba datasheet (Rev 6.0.A 2026-04): P-channel, VDSS -20 V, VGSS +/-12 V (8.4 V gate swing OK), ID -36 A DC / -180 A pulsed, EAS 320 mJ; pinning 1-3=S / 4=G / 5-8=D matches the netlist high-side topology (source at V_CAP, 10k gate-to-source pullups default-OFF, DTC123J pulls to fire). Arm FET CSD16323Q3 nets (1-3 GND source, 4 gate, 5-8+EP PYRO_GND drain) consistent with the same standard SON3.3 arrangement.
- Q1/Q7-Q10 PMPB14XNX verified from Nexperia datasheet: 40 V, 8.1 A @25C, IDM 33 A, VGS +/-8 V (3.3 V logic drive OK); DFN2020MD-6 pinning table (1,2,5,6,7=D; 3=G; 4,8=S) matches the custom footprint pad-for-pad in the live PCB.
- TPS62152 application verified against SLVSAL5E: TPS62152 = fixed 3.3 V variant; FB->GND (recommended for fixed versions), FSW->VOUT (footnote-sanctioned, 1.25 MHz), DEF->GND, EN->VIN, VOS->+3V3, SS 390 pF — all correct except the PVIN pin-12 finding.
- TLV62569 DRL pin-mapping verified pin-for-pin from the datasheet (1=FB,2=GND,3=VIN,4=SW,5=EN,6=NC-on-non-P harmlessly grounded); L8 peak ~0.6 A vs 1.89 A Isat.
- eFuse ILIM ECO landed: R48 = 127R (RC0402FR-07127RL confirmed in live schematic fields) -> ~11.5 A typ from TI's measured table; UVLO divider R44 1M/R45 210k unchanged from the previously-verified 6.9/6.3 V thresholds.
- R72 = CSSH0805FT2L00 is a genuine all-metal 1 W 0805 2 mOhm 1% current-sense part; 0.26 W at the 11.5 A eFuse limit; INA230 wiring clean: A0/A1 grounded (0x40), BUS pin on VBAT_CON, IN+/IN- across the shunt, VS+ = 3V3, ALERT float (permitted).
- Crystals: Y2/Y4 ECS-400-10-37B2-CKY-TR = 40 MHz +/-10 ppm tol/stab, CL 10 pF — meets Espressif's +/-10 ppm RF requirement; 12 pF load caps give CL_eff ~9-10 pF with stray (18->12 pF ECO landed, C20/C22 and C48/C49 confirmed in netlist). Y1/Y3 ABS07-32.768KHZ-T = 12.5 pF CL, +/-20 ppm with 22 pF loads (~13-14 pF eff incl. stray) — RTC-grade OK. C93 22 pF is the DCDC feedforward cap, NOT a crystal load (netlist-verified).
- CHIP_PU RC fix landed on both MCUs: C39 1 uF + R42 10k (P4), C25 1 uF + R36 10k (S3) — prior H8/M2 closed.
- L4 (S3 VDD3P3 feed) = LQG15HS2N0S02D, 900 mA rated — satisfies the HDG's >=500 mA requirement (prior M15 fixed for L4); L1/L3 have real orderable Murata MPNs (L3 in stock at DigiKey).
- Memory parts: GD5F2GQ5UEYIG(R) confirmed 3.3 V (2.7-3.6 V) WSON8 8x6 from the GigaDevice datasheet — matches footprint and +3V3 rail; W25Q128JVSIQ (x2) common/active, SOIC-8 208 mil; U16 correctly powered from the P4's VDDO_FLASH LDO output with 100n/1u decoupling.
- Lifecycle: ESP32-S3RH2 is a current part (datasheet Table 1-1; it is the designated upgrade for the EOL'd ESP32-S3R2 per Espressif PCN — good pick); FSUSB63UMX listed Active at onsemi; TPS2121/TPS22918/TPS62152/TLV62569/INA230/TPS259824L all current TI catalog parts with no NRND indications found; Molex 47948-0001 antenna still active at Molex/distributors; ISM6HG256X/IIS2MDC/BMP585 current ST/Bosch parts.
- R20 pyro-cap charge resistor = CRCW1206150RFKEAHP, 0.75 W pulse-proof AEC-Q200 HP series vs 0.47 W decaying peak (tau ~1.5 s) and 0.35 J per charge — comfortable; 22 uF group on V_CAP retains ~9-12 uF each under 8.4 V DC bias, fine alongside the 10 mF can.
- Buzzer MLT-8530 correctly applied: 3.3 V switched-rail drive inside its 2.5-4.5 V range, coil current below the 95 mA rating at 3.3 V, CR1 (CUS10S30, pin1=cathode) flyback fitted, Q9 low-side switch massively overrated for the load.
- U22 TPS22918 hookup re-verified: VIN 3.3 V (5.5 V part), QOD strapped to VOUT, CT 2.2 nF, ON held low by R68 100k with R71 100R from POWER_SWITCH — P4 domain defaults OFF.
- BOM MPN/Mfr coverage: 71 of 74 lines fully populated with plausible package-matched MPNs (prior H18 substantially closed); footprint-vs-MPN spot-checks passed for U1 (UQFN-12 1.8x1.8), U15/U17 (common S3 land pattern per datasheet note; QFN104), J-series JST/GCT/Molex parts, 0402/0805/1206 passives, XTAL footprints, and the C12 radial footprint (7.5 mm lead spacing, 1.0 mm drills = Nichicon 16 mm can, matching the corrected 18 mm-dia debate is a layout-domain item).


**Checks not completed (bom-parts):**

- CSD16323Q3 (U9) pin-1 orientation was not verified against the TI datasheet (fetch not run); netlist arrangement matches the industry-standard SON3.3x3.3 pinout and the Toshiba-footprint numbering, but the TI land-pattern vs Toshiba TSON-Advance pad-geometry comparison is left to the layout reviewer.
- TDK product-center page for VLS3012CX-2R2M-1 returns HTTP 403 (same as prior review); Isat/Irated/DCR (1.89 A / 2.83 A / 74 mOhm) taken from DigiKey/element14 listings, not the primary PDF.
- Exact Vf curves for XL-1005UGC (Xinglight) and NCD0402R1 (NationStar) not fetched (LCSC datasheets); LED-current finding uses typical 0402 InGaN/AlGaInP Vf ranges — the 10k conclusion is robust to +/-0.3 V but exact currents are estimates.
- LCSC/JLC stock-depth check for assembly (FSUSB63UMX, TCJE337M016R0050, GCT USB4110-GF-A, Nichicon EKY can, C&K KMR221GLFS, Samsung 16 V 22 uF) was not run — lifecycle was checked, assembly-house availability was not.
- ESP32-P4NRW32 revision mix at LCSC (C22387510) could not be determined remotely — the revision-pinning finding stands regardless; incoming inspection per the Espressif v3.x chip-identification guide is the mitigation.
- bom-priced.xlsx not audited line-by-line (established stale wholesale via bom.xlsx; pricing audit pointless until regeneration).
- ECS-400-10-37B2-CKY-TR temperature-stability letter decode (-40..+85 C assumed from the CKY suffix and DigiKey listing) not confirmed against the primary ECS datasheet PDF.


---

# Annex B — July-30 finding closure audit

Statuses determined against the live files; the 8 “fixed” calls on blockers/highs were each re-attacked by an adversarial verifier and all 8 survived.

| Prior | Title | Status | Evidence (condensed) |
|---|---|---|---|
| B1 | P4 circuit was v1.x-only: pad 54 floating, no DCDC FB network | **fixed ✔** | Live netlist: FB_DCDC = {C93.2, R74.2, R75.1, U17.78, U20.1}; R74/R75 = 499k 0402, C93 = 22 pF across R74 (ESP_VDD_HP->FB), R75 FB->GND — exactly Espressif Fig. 5 topology. U17 pad 54 is now net VDD_HP_1 = {R76.2, U17.54… |
| B2 | Battery connector was 2 A JST PH vs 12-16 A load | **fixed ✔** | J7 (JST PH) is deleted from the live netlist and PCB. New battery input J8 = JST B2P-VH(LF)(SN) (VH series, 10 A/contact, 3.96 mm THT), placed at (84.73,125.59) B.Cu; VBAT_Terminal = {J8.2, R72.2, U23.13}, J8.1 = GND. eF… |
| B3 | C12 10 mF can sits on top of populated bottom-side parts; no courtyard | **not fixed** | Live PCB: C12 unchanged at (84.89,144.34) B.Cu, B.Fab body circle r=9.0, courtyard items = 0, and live drc.json ignored_checks still lists missing_courtyard (only courtyard error remains J6/FID1). Components with centers… |
| B4 | FB_DCDC 499k/499k + 22 pF network missing for P4 v3.0+ | **fixed ✔** | Same evidence as B1: the full Fig.-5 network now exists in schematic and PCB (R74 499k ESP_VDD_HP->FB, R75 499k FB->GND, C93 22 pF feed-forward; FB_DCDC = {C93.2, R74.2, R75.1, U17.78, U20.1}), fitted-for-v3.x / DNP-for-… |
| H1 | TPS62152 U18 PVIN pad 12 unconnected (= summary blocker B-4; dup H10/H15/H17) | **not fixed** | Live netlist: net unconnected-(U18-PVIN-Pad12) = {U18.12} still exists. Live PCB: U18 was moved in tonight's re-route (pad 12 now at (89.53,127.80) B.Cu) and pad 12 STILL carries net 'unconnected-(U18-PVIN-Pad12)' while … |
| H2 | eFuse UVLO has no deglitch; ms pack sag hard-resets board mid-flight | **not fixed** | Live netlist: Net-(U19-EN{slash}UVLO) = {R44.1, R45.2, U19.6} — still no capacitor; R44 = 1M, R45 = 210k unchanged, so falling threshold is still ~6.34 V sensed at VBAT_CON. No waiver found: power-eco.md untouched since … |
| H3 | Battery input 2 A JST PH vs 14.7 A eFuse ILIM | **fixed ✔** | Duplicate of B2: J8 = JST B2P-VH (10 A class) replaces J7; R48 = 127R giving ILIM ~11.6 A typ (commit #658, 'min limit >= 10 A' sized to the VH budget). VBAT_Terminal = {J8.2, R72.2, U23.13} in the live netlist. |
| H4 | Battery J7 2 A/contact vs 14.7 A allowance and servo stall | **fixed ✔** | Duplicate of B2/H3 — same live-file evidence (J8 B2P-VH at (84.73,125.59) B.Cu; R48 127R). |
| H5 | Servo power via J3 Milli-Grid 2 A pins and 8.1 A return FET | **not fixed** | Live netlist/PCB: J3 is still 878321620 (Molex Milli-Grid 16-pos); VBATT still enters the servo harness only on J3.15/16 (VBATT net includes J3.15, J3.16); return is still Net-(J3-Pad1) = {J3.1, J3.2, Q8 drains x5}; Q8 i… |
| H6 | All four peripheral ports ground-side switched, supplies always hot | **partial** | Architecture unchanged in live netlist: Q1/Q7/Q8/Q10 (PMPB14XNX) still switch the connector grounds (Net-(J1-Pad4), Net-(J4-Pad1), Net-(J3-Pad1), Net-(J5-Pin_1)) with sources on GND and gates on 1k/10k unchanged; positiv… |
| H7 | S3 log flash on VDD_SPI violates 14-ohm R_SPI budget | **fixed ✔** | Live netlist: U13.8 (VCC) is now on +3V3 (net includes C26.1, C28.1, C29.1, U13.8, U15.20/46/55/56); OUT_VDD_SPI is now only {C27.1, U15.29} — the finding's exact recommended config (1): VDD_SPI serves only the in-packag… |
| H8 | S3 CHIP_PU missing RC delay cap (P4 undersized at 100 nF) | **fixed ✔** | Live netlist: Net-(U15-CHIP_PU) = {C25.1, R36.2, U15.4} with C25 = 1 uF 0402 and R36 = 10k — the exact HDG R=10k/C=1uF network. P4 side likewise: Net-(U17A-CHIP_PU) = {C39.1, R42.2, U17.103} with C39 = 1 uF (was 100 nF) … |
| H9 | P4 pad 54 NC but VDD_HP_1 on v3.0+ silicon | **fixed ✔** | U17 pad 54 is no longer NC: live PCB pad 54 at (88.51,115.78) carries net VDD_HP_1, routed to R76 (0R, at (90.83,114.93), ~2.5 mm away) whose other pad is ESP_VDD_HP. R76 is DNP for the current v1.3 build (pin unbonded o… |
| H10 | U18 PVIN pad 12 unconnected (dup of H1) | **not fixed** | Same as H1: live PCB U18 pad 12 at (89.53,127.80) B.Cu on net unconnected-(U18-PVIN-Pad12); pads 10/11/13 on Net-(U18-AVIN); erc.json still flags hidden pin 12. |
| H11 | C12 body over parts + footprint has no courtyard so DRC blind | **not fixed** | Same as B3: C12 footprint courtyard items = 0; drc.json ignored_checks still contains missing_courtyard; only courtyards_overlap violation is J6/FID1; Q3/Q4/Q6/R16/R17/R23/C9/C10/U13/U21 centers inside the r=9.0 fab circ… |
| H12 | Only one fiducial, under the USB-C body | **not fixed** | Live PCB: FID1 (Fiducial_0.5mm_Mask1mm) is still the only fiducial footprint, still at (84.32,168.07) F.Cu; live drc.json still reports exactly one courtyards_overlap error: 'Footprint J6' vs 'Footprint FID1'. No fiducia… |
| H13 | C12 has no polarity marking on silkscreen | **not fixed** | Live C12 footprint contains exactly one silkscreen graphic: a circle r=1.5 at local (3.78,-0.02) = global (84.91,148.12), which is the GND (negative) pad 2 — the same single ambiguous ring; no '+'/'-' text in the footpri… |
| H14 | C12 real size Ø18x25 vs Ø16 STEP; body over ~10 parts | **not fixed** | hardware/3dmodels/EKYC160ELL103MM25S.STEP unchanged (mtime Jul 25; measured X extent 16.03 mm dia vs the Chemi-Con-published 18 mm), so renders/clearance checks still understate the can by 2 mm. No courtyard added; parts… |
| H15 | U18 PVIN pin 12 unconnected (dup of H1) | **not fixed** | Same as H1/H10: unconnected-(U18-PVIN-Pad12) present in live netlist, PCB and ERC. |
| H16 | Servo feed chain copper/connector/FET sized ~2-4 A vs 12-14 A stall | **not fixed** | Live PCB scanline of the In2 VBATT zone: min-cut ~2.45 mm at y=105.5 (spans 91.7-93.1 + 93.5-94.5) and ~2.19 mm at y=125 — essentially the prior 2.50 mm 1-oz inner corridor (~3.8 A at 10 C with inner derate). J3 transiti… |
| decide-usb-camera-power | USB-only operation no longer powers camera/GNSS/LoRa/servos/pyro-charge | **not fixed** | Architecture unchanged in live netlist: J4.2, FL1.1, FL2.1, J3.15/16, R20.2 all hang on VBATT (eFuse output), and USB VBUS still enters only at U21 TPS2121 IN1 (Net-(J6-VBUS) = {CR3.2, J6 VBUS pads, R51.2, R62.2, U21.7})… |
| decide-pyro-cap-on-usb | Pyro cap charges to ~2.5-2.7 V on USB via fire-FET body diodes | **not fixed** | Pyro network unchanged in the live netlist: V_CAP = {C9-C13, R15.2, R16.2, R17.2, R20.1, R23.2, U6/U7/U8/U10 sources x3 each}; fire FETs still TPN4R712MD with drains on PYRO*_EXT; no blocking diode or bleed added on V_CA… |
| decide-camera-raw-pack-voltage | Camera port J4 feeds raw 2S pack voltage; RunCam input range unverified | **not fixed** | Live netlist: J4.2 still on VBATT (6.4-8.4 V raw pack) and J4 is still B4B-PH-SM4-TB. grep -ri 'runcam' over all repo *.md finds only the prior review itself and an unrelated architecture map — no camera model or input-r… |
| decide-nand-geometry | NAND changed to GD5F2GQ5UE (2 Gbit) — firmware geometry constants must follow | **not verifiable** | Hardware side confirmed stable in live files: U11 = GD5F2GQ5UEYIGR in netlist and fresh BOM (WSON-8). The action item lives in firmware (geometry constants), which is outside the hardware repo files under review; no evid… |
| decide-strap-pins-exp | P4 strap pins GPIO34/37/38 bare to EXP connector J3 | **not fixed** | Live netlist: EXP_09 = {J3.12, U17.70 (GPIO38)}, EXP_10 = {J3.11, U17.69 (GPIO37)}, EXP_11 = {J3.10, U17.65 (GPIO34)} — all still exact 2-node nets: no pulls, no series resistors, no buffer. A payload on J3 can still cor… |
| decide-testpoints-uart0-mux | Zero testpoints, no UART0 fallback, FSUSB63 SEL truth table unverified | **not fixed** | Live netlist/PCB have zero TP* references; U15.49 (U0TXD) and U15.50 (U0RXD) are both on unconnected-() nets, so USB remains the only console/programming path for both chip-down MCUs; no document or note anywhere in the … |
| decide-ina230-kelvin-pg | INA230 half-Kelvin sense; PG_RAIL and buck PG dead-end at pull-ups | **partial** | Kelvin half: IN+ (U23.13 at (79.115,127.775)) now leaves on a dedicated 0.1 mm sense trace (B.Cu -> via (79.7,127.83) -> In2 -> via (81.38,129.82)) instead of tapping the battery-connector pad — but that via lands mid-wa… |
| decide-paste-via-in-pad | Unplugged via-in-pad in P4 GND/eFuse pads; NAND EPAD 100% paste | **partial** | Via-in-pad half fixed: the live .kicad_pcb setup now declares (filling yes) and (capping yes) (plus tenting front/back) — filled+capped vias per commit c9918f05 — so the 6 vias inside U17 pad 105 (F.Cu/F.Mask pad, paste … |


**Adversarial verdicts on the “fixed” claims:**

- **B1 — confirmed.** Fix is real and complete in the endorsed v1.3-stock form. The full Fig.-5 FB network exists (R74/R75 499k, C93 22 pF, correct topology: R74.1+C93.1 on ESP_VDD_HP, R75.2 on GND, midpoint = FB_DCDC with U17.78 and U20.1), pad 54 is no longer NC (net VDD_HP_1 via R76 0R to ESP_VDD_HP), and the v1.x-required 1M CEN_D+ pulldown R77 is fitted. All four v3.x parts are DNP in both netlist and PCB (exactly 4 dnp attrs), and the in-schematic '#657' note (central_processing_p4.kicad_sch line 4456) documents the variant rule verbatim. Corrective notes beyond the claim's own residuals: the fresh bom.csv export has NO DNP column — C93 is grouped with four fitted 22 pF caps (qty 5) and R74/R75/R76 appear as ordinary fitted lines, so a fab package built from bom.csv as-is would populate the v3.x parts on v1.3 silicon; and the VDD_HP_1 provision is routed as 2.5 mm of 0.1 mm-wide F.Cu, electrically thin for a core-supply pin if the v3.x variant is ever built.
- **B2 — confirmed.** The 2 A JST PH is genuinely gone and replaced by a 10 A-class JST VH with the eFuse limit re-sized. J7 has zero occurrences in the live netlist and PCB; J8 (B2P-VH(LF)(SN), 3.96 mm THT, 1.7 mm drills) carries VBAT_Terminal/GND and its pad 2 is served by a 19.2 mm2 B.Cu zone reaching the R72 shunt (the 0.1 mm segments on the net are only the Kelvin sense route to U23.13). R48 = 127R gives ILIM ~11.6 A typ, reproduced independently from the TPS25982 datasheet table (100R=14.71 A, 182R=8.13 A, K~1471-1480 => 127R~11.6 A), and commit edef97e7 (#658) exists and documents the conscious 'guarantee 10 A delivery' sizing using eq. 4 RILIM=1460/(ILIM-0.11). JST eVH datasheet (fetched) confirms VH = 10 A/contact with AWG#16. The residual (ILIM typ/max ~11.6/12.6 A still ~1.2x the connector rating, servo branch not upsized) is accurately disclosed as a documented decision, not silent — vs the original 7-8x overload this is a completed fix.
- **B4 — confirmed.** Same evidence base as B1, independently reproduced: net FB_DCDC grew from the original 2 nodes {U17.78, U20.1} to 5 nodes with the exact Espressif Fig.-5 divider (R74 499k ESP_VDD_HP->FB, R75 499k FB->GND, C93 22 pF feed-forward across R74), placed on F.Cu next to U20 with copper routed (7 segments on FB_DCDC). For the currently populated v1.3 build the parts are DNP and the chip-driven no-divider FB is the correct v1.x Espressif circuit, with the variant rule documented in-schematic (#657 note). The claim's stated residual is accurate and I add one sharpening of it: the risk of building v3.x silicon without fitting the four parts is not hypothetical bookkeeping — the exported bom.csv has no DNP column and lists all four as ordinary fitted quantities (C93 inside a qty-5 22 pF group), so the assembly-variant control currently lives only in the PCB/netlist dnp attributes and the schematic text. Pinning the variant in the fab package, as the claim's note demands, is genuinely required.
- **H3 — confirmed.** Duplicate of B2 and verified by the same independent reproduction: the JST PH J7 is fully deleted from the live netlist and PCB, J8 = JST VH B2P-VH (10 A/contact per the JST eVH datasheet) is placed and routed on B.Cu with a 19.2 mm2 pour to the shunt, and R48 was changed 100R->127R giving ILIM ~11.6 A typ (reproduced from the TPS25982 EC table by interpolation and matching commit edef97e7's eq.-4 math). The connector is now rated for the realistic 10 A board budget instead of being 7x under it. The remaining ~1.16x gap between ILIM typ and the VH rating is exactly the documented #658 sizing philosophy the claim discloses; the finding's alternative option (limit protects the weakest link) was consciously not taken, which is an acceptable waiver given it is recorded in the commit and the downstream branch issue is tracked separately (H5/H16).
- **H4 — confirmed.** Duplicate of B2/H3, same reproduced evidence: J7 (JST PH, 2 A) is gone; J8 (JST VH B2P-VH, 10 A/contact, 3.96 mm THT at (84.73,125.59) B.Cu) is the battery input with proper copper (B.Cu zone to R72, GND pad into the GND pour), and the eFuse ILIM is re-sized to 127R (~11.6 A typ). The claim correctly identifies the pigtail wire gauge as a harness item outside the board files — nothing in the repo contradicts it, and the #658 commit message specifies the VH/AWG context. Fix complete at board level.
- **H7 — confirmed.** The core defect — external log flash sharing the 14-ohm R_SPI budget with the in-package PSRAM — is genuinely fixed in the finding's own recommended config (1): U13.8 (VCC) is now on +3V3 and OUT_VDD_SPI has shrunk to exactly {C27.1, U15.29}, so VDD_SPI serves only the in-package PSRAM (~0.42 V budget, within the 2.7 V min) and the flash is no longer power-cycled in light sleep. GPIO45 (U15.51) remains unconnected = 3.3 V VDD_SPI mode, and the flash's CS/CLK/data wiring to the S3 is intact (U13.1=OUT_SPI_CS0 to U15.32 etc.). The claim's disclosed residual reproduces: C27 is still a single 10 uF (CL05A106MQ5NUNC) — the HDG's '0.1 uF + 1 uF, do not add excessively large capacitors' sub-item was not executed. Additional minor note from the live PCB: C26, the 100 nF that used to decouple U13 on B.Cu, now sits on F.Cu at (93.63,142.99) ~3.2 mm from the B.Cu-mounted U13 at (91.705,145.485), so the flash's local decoupling goes through vias — worth an opportunistic tidy, not a closure failure.
- **H8 — confirmed.** Both RC networks now exist exactly per the Espressif HDG (R=10k, C=1uF). S3: Net-(U15-CHIP_PU)={C25.1, R36.2, U15.4} with C25=1 uF (CL05A105KO5NNNC) at (73.91,144.31) directly beside the pin, C25.2 to GND, R36=10k to +3V3. P4: Net-(U17A-CHIP_PU)={C39.1, R42.2, U17.103} with C39=1 uF, C39.2 to GND, R42=10k — and I verified R42.1 pulls up to V_MCU_SWTCH, the P4's own switched rail, so the delay correctly runs from P4-domain power-up rather than the always-on rail. Two side observations: the repo-root bom.csv (semicolon format) is stale — it still lists C39 in the 100 nF group, contradicting the live 1 uF value (the fresh export is correct, but the stale file in the repo is a fab-package trap); and the suggested bench-reset test point was indeed not added, as the claim discloses.
- **H9 — confirmed.** Pad 54 is no longer NC: the live PCB pad carries net VDD_HP_1 (pinfunction 'VDD_HP_1_54') routed 2.5 mm on F.Cu to R76 (0R 0402 at (90.83,114.93), measured 2.47 mm center-to-pad), whose other pad is ESP_VDD_HP; R76 is DNP for the current v1.3 build (pin unbonded on v1.x, floating is per-spec) and fitted for v3.x per the #657 schematic note — the B1-endorsed variant strategy. The claim's residual reproduces exactly: net VDD_HP_1 = {R76.2, U17.54} only, no local 100 nF, so in the v3.x variant pin 54 is served through R76 by the six ESP_VDD_HP caps — the finding's 'add a 100 nF adjacent' sub-item was consciously skipped. One corrective addition: the provision is routed entirely in 0.1 mm-wide copper (3 segments, 2.5 mm total) plus a 0402 0R — roughly 10 mOhm of trace plus the jumper in series with one of the four VDD_HP core pins if v3.x is ever fitted; acceptable for a DNP provision but should be fattened whenever the v3.x variant becomes the build standard.