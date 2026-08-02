# TinkerRocket rocket-computer (Full V9) — Pre-Manufacturing Review

**Date:** 2026-07-30
**Scope:** every component + its support parts, system architecture, PCB layout beyond DRC (datasheet compliance, best practices), and "anything dumb" (the lora-missing-ground class).
**Method:** 12 parallel review agents (power input, switched domains, pyro, ESP32-P4, ESP32-S3, sensors, connectors/cross-board, architecture, and 4 layout dimensions), each parsing the KiCad sources + a fresh kicad-cli netlist/ERC/DRC/BOM export and pulling manufacturer datasheets. 168 findings, 934 tool calls.

**Verification caveat — read first.** The planned adversarial verification pass (16 agents) failed to run: the Claude monthly spend limit was hit right after the find phase. Mitigations applied instead: (a) the highest-impact findings were independently converged on by 3–5 finders working from separate contexts; (b) the file-level facts behind every blocker were re-checked directly against `netlist.net` after the run (marked ✅ below); (c) datasheet-dependent claims carry the finder's citation and should be eyeballed against the linked doc before editing. The full verify pass can be re-run cheaply after the limit resets (workflow resume re-uses all 12 cached finder results).

**Two meta-corrections to prior docs:**
1. **The board was fully re-annotated since the July 12 review docs.** Current refs: P4 = **U17** (was U18), S3 = **U15** (ESP32-S3RH2), 3V3 buck TPS62152 = **U18**, eFuse = **U19**, P4 buck TLV62569 = **U20**, TPS2121 mux = **U21**, TPS22918 = **U22**, INA230 = **U23**, battery = **J7**, pyro = **J2**, servo/EXP = **J3**, camera = **J4**, GNSS = **J1**, LoRa = **J5**, USB-C = **J6**, pyro cap = **C12**, sensors = U2 (IMU) / U3 (mag) / U4 (baro). `schematic-review.md` / `power-eco.md` use the old refs.
2. **The board is 6-layer** (F / In1 / In2 / In3 / In4 / B — In1+In4 solid GND, In2 split power, In3 routing), not 4 as the July docs implied. The review's visual pass covered F/In1/In2/B only; **In3/In4 were checked by file-parsing, never visually** — do one In3/In4 export + eyeball before fab.

---

## Verdict: NOT ready to send out. 4 blocker clusters, then a short must-fix list.

### Blockers

**B-1. The P4 circuit is the obsolete chip-rev v1.x reference — current v3.x silicon won't run on it.** ✅ netlist-verified: `FB_DCDC` net = {U17.78, U20.1} only, no divider, no 499k anywhere in the project; P4 pad 54 = NC. Espressif's v3.0+ reference requires pad 54 tied to `ESP_VDD_HP` (it became core-supply pin VDD_HP_1) and a 499k/499k + 22 pF feedback network on the TLV62569 FB node ("must be populated"). v1.x silicon is NRND; 2026 distributor stock is v3.x. As drawn: a v3.x chip has a floating core-supply pin and the external buck has no valid feedback — no-boot or core-rail overdrive on a chip-down QFN you can't rework. Fix: pad-54 copper to ESP_VDD_HP + the 3-part FB network per Espressif Fig. 5; pin the chip revision at purchase; set `CONFIG_ESP32P4_REV_MIN`. (If you deliberately buy v1.3 stock instead: add the v1.x-required 1 MΩ pulldown on CEN_D+ and DNP-reserve the FB pads.)

**B-2. C12 (pyro 10 mF, Ø18×25 mm radial can) is placed on top of ~15 populated bottom-side parts.** Its body circle covers the pyro gate driver Q4, fire FETs U7/U8, TPS2121 (U21), TPS62152 (U18), L6 and a dozen passives — it cannot sit flush, and standing it off >1.3 mm on two 0.8 mm leads above live pyro gate-drive parts is a crush/short/vibration-fatigue risk in a high-g vehicle. The footprint has **no courtyard** and the DRC config **ignores missing_courtyard**, so DRC is structurally blind to it. Also: no +/− polarity silk (a single ambiguous ring on the *negative* pad). Fix: relocate can or parts, add a real Ø19–20 courtyard + polarity silk, re-run DRC with courtyard checks on, and check the 25 mm can height against the bay.

**B-3. Battery/servo current path is under-rated end-to-end.** ✅ netlist-verified: J7 = JST PH 2-pos (2 A/contact) is the only battery input, and the external servo adapter takes its power through J3.15/16 (Molex Milli-Grid, ~2 A/pin) — while the eFuse is strapped for 14.7 A and the ECO's own worst case is 12–14 A servo stall. Every amp crosses one PH contact each way (7–8× rating = melt/fire path on a 2S pack). Fix: either bound the servo budget in writing (≤~4 A aggregate: then only J7 still needs upsizing — nominal loads already exceed 2 A) or upsize the whole chain (XT30-class battery connector, more J3 power pins or a dedicated servo power connector, bigger return FET Q8) and set ILIM to protect the actual weakest link.

**B-4. TPS62152 (U18) PVIN pin 12 is floating.** ✅ netlist-verified: `unconnected-(U18-PVIN-Pad12)`; found independently by 5 agents. Datasheet requires both power-stage input pins tied to VIN — as drawn, the full 3V3-rail current squeezes through one PVIN pin/bond. One-wire schematic + copper fix.

### Must-fix before fab (high, condensed — full text in the body)

- **Pyro gate-drive margin (safety):** a single enabled internal P4 pull-up (~45 k) on a fire GPIO can lift a DTC123J base enough to turn that fire FET on (R2 = 47 k divides too weakly); the arm gate has the same weakness via R22 = 100 k. Boot glitches are the exact scenario the safe-pyro-init memory exists for — make the hardware itself immune: stiffen the base pulldowns / resize the divider so a worst-case WPU cannot bias any base, then re-check with the P4 IO-MUX boot states.
- **Arm interlock moved to the LOW side post-ECO** (J2.5 → PYRO_GND → U9 → GND) and J2.5 is silk-labeled "GND": one harness short from the e-match return to real ground (airframe, sensor wire) bypasses the arm FET entirely, and the label invites exactly that wiring. Re-decide high-side vs low-side consciously; at minimum relabel the silk and document.
- **eFuse UVLO has no deglitch:** a millisecond pack sag below 6.34 V hard-resets the *whole board* mid-flight — the July ECO's own §3.5 required the LVC to be slow/deglitched; the eFuse implementation lost that property. Decide: lower UVLO, add RC on the EN/UVLO divider, or accept (hold-up cap only protects the logic rail *behind* the mux, not against the eFuse opening the main disconnect).
- **All four peripheral branches are ground-switched** (camera/GNSS/LoRa/servo return-side FETs): off-state branch grounds float to VBATT and back-feed the MCUs through every signal line (5 finders converged; also breaks the clean "shed load under sag" story — shedding under load dumps current into GPIO clamps). This is an architecture regression vs the ECO's high-side plan — revisit or explicitly waive with the backfeed paths enumerated.
- **S3 VDD_SPI:** powering log flash U13 from the VDD_SPI pin violates the ~14 Ω R_SPI budget once the RH2's in-package PSRAM shares the rail (the open item from the MRAM→RH2 swap; no series R is fitted). Settle per the S3 datasheet inequality — likely: feed U13 from +3V3 directly.
- **S3 CHIP_PU** has a 10 k pull-up but no RC cap → violates the 50 µs t_STBL requirement; P4 CHIP_PU uses 100 nF vs recommended 1 µF. Two caps.
- **Fiducials:** exactly one on the board, and it sits inside the USB-C connector courtyard (that's the single DRC courtyard error). A chip-down 0.35 mm-pitch QFN-104 assembly needs 3 clear global fiducials.
- **BOM has no MPN column:** the fab/assembler can't order it as-is, and voltage-critical passives are unpinned — headline risk: the four "22 µF" 0805s on the 8.4 V V_CAP rail (commonly 6.3–10 V parts!), the 2 mΩ shunt R72 (needs a real current-sense part ≥0.5 W), and the RF/VDD3P3 feed inductors.

### Decide-consciously (medium highlights)

- **USB-only operation no longer powers the camera** (nor GNSS/LoRa/servos/pyro-charge — all hang off eFuse-gated VBATT which is dead on USB). The battery-side RunCam brownout fix looks credible (330 µF + mux hold-up + 4.7 ms blanking — analysis-only, bench item still open), but the *current* "record on USB" workaround physically disappears. Confirm that's intended.
- **Pyro cap charges to ~2.5–2.7 V even on USB** through fire-FET body diodes — "no pyro energy on USB" is false as drawn.
- **Camera port J4 feeds raw 2S pack voltage (6.4–8.4 V)** — the RunCam model + its input range aren't recorded anywhere in the repo; verify before first plug-in.
- **NAND changed again**: now GD5F2GQ5UE (2 Gbit — half the documented F35SQB004G) — firmware geometry constants must follow (same class as the OC 512 MB lesson, #492).
- **P4 strap pins GPIO34/37/38 go bare to the EXP connector J3** — a payload can corrupt boot mode.
- **Zero testpoints** and no UART0 fallback: the USB-C chain (incl. FSUSB63 mux with an **unverified SEL truth table** — the one datasheet nobody could fetch) is the only programming/console path for both chip-down MCUs. Worth 6 testpoints and a pre-fab confirmation of the mux mapping.
- **INA230 sense is half-Kelvin** (IN+ taps the J7 pad, not the shunt pad → reads ~15–20 % high), and PG_RAIL ✅ + buck PG dead-end at pull-ups — no MCU sees main-power faults.
- **Paste layer:** unplugged via-in-pad in the P4's only ground pad + eFuse pad (solder wicking risk); NAND EPAD gets a 100 % aperture defeating its segmentation.

### Verified clean (the review also *confirms* a lot)

MRAM deletion + S3RH2 swap fully implemented; TPS22918 default-off interlock (old C-4a) settles CLOSED (pulldown + QOD verified); ECO Changes 1/2/5/6/7 netlist-verified as implemented; the 2.4 GHz Molex 47948 is an **on-ground** antenna — the full copper beneath it is correct by design (keepout worry affirmatively refuted), feed ≈50 Ω, match per Espressif; USB mux/slide-switch logic verified end-to-end; **both daughterboard links are pin-for-pin correct** against the LoRa and both GNSS board sources (the lora-missing-ground class comes up clean); pyro fire-path copper, FET and connector ratings all check out (~7.6 A/channel, 25 A 4-channel worst case); rail budgets fit (+3V3 ~0.7 A vs 1 A buck); boot-state truth table is hardware-safe at every power stage; all four crystals sit over solid unbroken GND; the U9 "pad 9 PYRO_GND" DRC parity hit is **benign** (drain tab is pad "5" in copper; symbol/footprint tidy-up only).

### Suggested order of work

1. B-1…B-4 schematic/layout edits (P4 v3.x circuit, C12 relocation + courtyard, battery/servo path decision, PVIN-12).
2. The pyro gate-drive + arm-side decision pass (safety cluster) and eFuse UVLO deglitch decision.
3. S3 VDD_SPI + CHIP_PU caps, fiducials, BOM MPN fill (pin the 16 V-rated 22 µF parts, real shunt, inductors).
4. Export In3/In4 SVGs + visual pass (only unreviewed copper), re-run DRC with courtyard checks enabled, then the deferred ERC symbol-hygiene cleanup.
5. Re-run the adversarial verify pass on this report once the spend limit resets, then the ECO bench checklist (hold-up sag test, servo-stall test, RunCam record-inrush on battery).

Full findings below — 4 blocker / 23 high / 39 medium / 45 low / 57 info, then the 70 checks the reviewers explicitly could not complete (mostly unfetchable datasheets and the In3/In4 visual gap).

---

## BLOCKER — do not fab until resolved

### B1. Schematic implements the v1.x-only ESP32-P4 circuit: pin 54 floating and no DCDC feedback network — incompatible with current v3.x silicon
*esp32-p4 / ESP32-P4 core power / chip revision — confidence high*

The P4 circuit is a copy of Espressif's chip-revision v1.0/v1.3 reference: U17 pad 54 is NC/unconnected and the external buck's FB pin (U20 TLV62569) connects ONLY to the P4's FB_DCDC pin — no feedback divider exists, not even DNP provisions (no 499k anywhere in the project). Espressif's v3.x user guide states pin 54 becomes VDD_HP_1 (a core supply pin) on v3.0+, and the HW Design Guidelines state that for v3.0+ 'the feedback resistor and the feedback capacitor must be populated' (2x 499 kOhm + 22 pF in the TLV62569 FB path). v1.0/v1.3 are marked 'not recommended for new designs' and v3.x is what distributors increasingly ship in mid-2026. Soldering a v3.x chip on this board leaves a VDD_HP core pin floating and the DCDC without its required feedback network — boards that won't boot reliably and can't be reworked (LGA pad under the chip). Note the board also doesn't fully match the v1.x checklist: v1.x required reserving FB-divider pads and a 1 MOhm pull-down on the DP pin (GPIO25/CEN_D+, absent — net has only U17.53 and U1.8). ERC independently flags 'U17 pin 78 FB_DCDC input not driven'.

**Evidence:** netlist.net: U17 pin 54 = unconnected-(U17A-NC-Pad54); net FB_DCDC = {U17.78, U20.1} only (2 nodes); net CEN_D+ = {U17.53, U1.8} (no 1M pulldown); grep '499' in central_processing_p4.kicad_sch = 0 resistors; PCB pad 54 exists at (88.51,115.78) with ESP_VDD_HP pad 76 at (88.51,108.08) on the same edge. Sources: Espressif 'ESP32-P4 Chip Revision v3.x User Guide' (pin 54 NC->VDD_HP_1; 2x499k+22pF added; v1.x/v3.x firmware incompatible) and 'ESP Hardware Design Guidelines — ESP32-P4 Schematic Checklist' (FB components 'must be populated' for v3.0+; 1M DP pulldown for v1.0/v1.3).

**Fix:** Update to the v3.0+ reference before fab: (1) connect pad 54 to ESP_VDD_HP (copper-only change; pad exists and the rail is on the same package edge); (2) add the 2x 499 kOhm divider + 22 pF at U20 FB exactly per the v3.0+ reference figure, keeping FB_DCDC connected to the divider midpoint; (3) recompile firmware with CONFIG_ESP32P4_REV_MIN for v3.x; (4) pin the chip revision at purchase. If instead v1.3 stock is deliberately bought: keep circuit but add the v1.x-required 1 MOhm pulldown on CEN_D+ and reserve DNP pads for the FB network so the next spin can take v3.x.

**Source:** https://documentation.espressif.com/esp32-p4-chip-revision-v3.x_user_guide_en.html ; https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32p4/schematic-checklist-esp32p4.html

### B2. Battery connector J7 is a 2 A JST PH; full board load incl. 12-14 A servo stall crosses one contact pair
*layout-power / battery-input — confidence high*

J7 (B2B-PH-SM4-TB, 2-pos SMT JST PH) is the only battery input: J7.2->VBAT_Terminal->R72 shunt->U19 eFuse->VBATT feeds logic, camera (J4.2), LoRa (FL2->J5), GNSS (FL1->J1) AND the servo adapter (J3.15/16 -> external 4x servo board, verified from net_servo-adapter.net: adapter has no own battery input, takes LiPoPos/GND through the J3 cable). JST PH contacts are rated 2 A. Worst case (task spec): 12-14 A servo stall + 1-2 A camera + ~1 A logic ~= 15-16 A through ONE PH contact each way (7-8x rating); even nominal 4-6 A servo running exceeds the rating. On a 2S LiPo this is a melt/fire path and it cannot be bodged after fab (SMT footprint change). The eFuse ILIM (~14.7 A) will not protect it. Same JST-PH family part on the camera port (J4) is fine at 2 A.

**Evidence:** netlist.net: VBAT_Terminal = {J7.2, R72.2, U23.13}; servo-adapter netlist J1.6=LiPoPos/J1.1=GND only power entry; J7 pads at (84.44,129.06)/(86.44,129.06) B.Cu in rocket-computer.kicad_pcb; JST PH series rating 2 A/contact (JST ePH datasheet via jst-mfg.com/product/pdf/eng/ePH.pdf, TME/DigiKey listings)

**Fix:** Replace J7 with a connector rated for the real pack current (XT30 = 15 A class, or split servo power onto its own high-current battery lead). If servos are guaranteed never to route through this board, document that and de-rate J3 instead.

**Source:** JST PH series datasheet (2 A, AWG24)

### B3. C12 (10 mF, Ø18×25 mm radial can) is placed on top of ~15 bottom-side components — cannot be assembled as drawn
*pyro / pyro energy store / layout — confidence high*

C12 (EKYC160ELL103MM25S, confirmed KYC series Ø18 mm × 25 mm, 7.5 mm lead pitch) is a through-hole radial can placed on B.Cu at (84.89,144.34) with its Ø18 body (B.Fab circle r=9.0) directly over other bottom-side parts. Components whose centers fall INSIDE the can circle include: Q4 DTC123J gate driver (d=1.02 mm from can center), R17 (1.94), Q3 (2.53), R16 (3.22), R49 (3.40), C10 22µF (4.29), C11 (4.36), L6 VLS3012 1.2 mm-tall inductor (5.02), R64 (5.55), C9 (5.63), C13 (6.06), C44 (7.25), U21 TPS2121 power mux (7.55), U18 TPS62152 buck (7.75), R20 charge resistor (8.05), U7 fire FET (8.39), U8 fire FET (8.86). The can cannot sit flush; assembly would require standing it off >1.3 mm on its two 0.8 mm leads above live pyro gate-drive and power-path parts — a crush/short/vibration-fatigue risk on the deployment energy store in a high-g vehicle. The footprint has NO courtyard (only B.SilkS + B.Fab circles), and the DRC config ignores missing_courtyard, so DRC is structurally blind to this. The only courtyard violation DRC does report is unrelated (J6/FID1).

**Evidence:** rocket-computer.kicad_pcb: C12 at (84.89,144.34,-90) layer B.Cu, fp_circle B.Fab center(0.01,-0.01) r=9.00; neighbor distances computed from footprint 'at' fields (script pyro_extract.py in scratchpad/review). drc.json: ignored_checks includes missing_courtyard; no courtyard overlap reported for C12. render_bottom.png shows the can body overlapping the fire-FET row.

**Fix:** Before fab: relocate C12 (or the parts under it) so the full Ø18 body + ~1 mm margin is clear on the B side, add a real courtyard (Ø19-20) to the EKYC160ELL103MM25S footprint, and re-run DRC with courtyard checks enabled. If a deliberate stood-off/laid-down mounting is intended, document it, add adhesive/strap retention, and still keep the area under the can free of tall or exposed-metal parts.

**Source:** https://www.chemi-con.co.jp/en/products/detail-condenser.php?part_number=EKYC160ELL103MM25S (KYC, 10000 µF, 16 V, Ø18×L25)

### B4. FB_DCDC feedback network (499k/499k + 22 pF) required for ESP32-P4 rev v3.0+ is missing
*switched-domains / P4 core buck (U20 TLV62569) — confidence high*

The TLV62569 (U20) FB pin connects ONLY to the P4's FB_DCDC pin (net FB_DCDC has exactly 2 nodes: U17.78 and U20.1). Espressif's ESP32-P4 Hardware Design Guidelines Fig. 5 ('TLV62569 Schematic, Chip Revisions v3.0 and Later') requires R1 499k from ESP_VDD_HP to the FB node, R2 499k from FB to GND, and C2 22 pF feed-forward across R1, and states: 'For chip revisions v3.0 and later versions, the feedback resistor and the feedback capacitor must be populated' (the no-divider circuit is the v1.0/v1.3 variant, which is 'not recommended for new designs'). Any P4 bought for this build in 2026 will be v3.0+ silicon. Without the divider, if the chip-internal FB drive is absent/insufficient on v3.0 silicon, the TLV62569 sees FB below its 0.6 V reference and drives ESP_VDD_HP toward VIN = 3.3 V - onto the 0.99-1.3 V VDD_HP core domain (won't boot, possible chip kill on a chip-down QFN that cannot be reworked cheaply).

**Evidence:** netlist.net: net FB_DCDC = {U17.78 FB_DCDC [input], U20.1 FB [input]} - no other nodes; net ESP_VDD_HP = {L8.2, C55, C64, C67, C70, C73, C92, U17.26/76/91} - no resistor to FB. Espressif esp-hardware-design-guidelines (esp32p4), 'Internal Voltage Regulators and External DCDC', p.9-10 of PDF export: Fig. 5 shows R1=499K, R2=499K, C2=22pF, C3=22uF, L1=2.2uH; text quoted above.

**Fix:** Add per Espressif Fig. 5: 499 kOhm from ESP_VDD_HP to the FB_DCDC/U20-FB node, 499 kOhm from that node to GND, 22 pF in parallel with the upper 499 k. Existing L8 2.2 uH and 30 uF output capacitance already match the reference. Edit in central_processing_p4.kicad_sch (net FB_DCDC), place next to U20 at (89.1,104.0) where free area exists.

**Source:** https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32p4/schematic-checklist-esp32p4.html (PDF export, 'Internal Voltage Regulators and External DCDC', Fig. 5/6)

## HIGH — fix before fab

### H1. TPS62152 PVIN pin 12 is unconnected in schematic and copper
*architecture / power/3V3-buck — confidence high*

The +3V3 buck U18 has two power-stage input pins (11, 12); the datasheet pin table says 'PVIN: Supply voltage for power stage. Connect to same source as AVIN' for both. Pin 11 is on Net-(U18-AVIN); pin 12 is floating in the schematic (ERC: 'Hidden pin 12 PVIN not connected' + 'not driven') and the PCB pad 12 has no net. The entire logic-rail input current flows through a single 0.26 mm QFN pin, out of datasheet spec, adding resistance/inductance in the highest-priority power path. Trivial copper fix; must be done before fab.

**Evidence:** netlist.net: unconnected-(U18-PVIN-Pad12); erc.json /Power/: 'Symbol U18 Hidden pin 12 [PVIN] not connected/not driven'; rocket-computer.kicad_pcb U18 pad 12 net=unconnected-(U18-PVIN-Pad12) while pads 10/11/13 = Net-(U18-AVIN). TPS62150/52 datasheet Table 6-1 pins 11,12 PVIN.

**Fix:** Tie pad 12 to the AVIN/PVIN net (short trace to pad 11). Also unhide the pin in the symbol so ERC sees it.

**Source:** TI TPS62150/1/2/3 SLVSAL5E (fetched)

### H2. eFuse UVLO has no deglitch: a millisecond pack sag below 6.34 V hard-resets the whole board mid-flight
*architecture / power/brownout — confidence high*

The ECO replaced the TPS3840 LVC with the TPS259824 eFuse UVLO, but the schematic-review's explicit requirement (section 3.5: 'RC deglitch >=100 ms so servo/radio transients cannot trip it') was never implemented. Net-(U19-EN/UVLO) contains only R44 (1 M), R45 (210 k) and pin 6 — no capacitor — and the TPS25982 EN/UVLO comparator responds in microseconds. Falling threshold computes to 6.34 V at VBAT_CON, sensed directly at the pack (only the 2 mohm shunt upstream). A 4-servo stall or connector micro-drop on a mid-discharge 2S pack (7.2 V minus ~1 V ESR+connector sag) crosses 6.34 V and cuts VBATT to everything: servos, camera, pyro charge AND the logic branch feed. The V_MCU_2S hold-up (C56 330 uF) then carries the MCUs for only ~4-12 ms at realistic flight load (3.7 ms at 1.65 W on +3V3, 12 ms at 0.5 W; the ECO's 52 ms figure assumed a 50 mA S3-only load that no longer reflects the P4-on flight state). Recovery needs VBAT_CON back above 6.91 V plus retry delay plus the ~20-40 ms dVdt ramp — far longer than the hold-up — so both MCUs reset in flight and deployment is lost (prior project data: recovery gap 15-20 s). The old low-side LVC was equally total but this respin was supposed to fix exactly this two-timescale problem.

**Evidence:** netlist.net: Net-(U19-EN{slash}UVLO) [3] = R44.1, R45.2, U19.6 (no C). TPS25982 datasheet (SLVSEI3D): V_UVLO(R/F) = 1.2/1.1 V, no deglitch spec on EN. Thresholds: 1.2x(1.21M/210k)=6.914 V, 1.1x5.762=6.338 V. Hold-up: 0.5x330uF x (8.0^2-3.5^2) x 0.85 = 8.5 mJ; at 1.94 W input -> 3.7 ms. power-eco.md CHANGE 2 marked 'IMPLEMENTED & VERIFIED' does not include the deglitch that schematic-review.md section 3.5 required.

**Fix:** Add a capacitor on the EN/UVLO pin: divider Thevenin impedance is ~174 k, so 1 uF gives tau ~174 ms of sag filtering (also delays turn-on ~0.4 s — acceptable). Alternatively lower the UVLO to ~5.8-6.0 V (3.0 V/cell) to buy transient headroom and let firmware (INA230 telemetry) own the soft low-battery warning. Bench-verify worst-case servo stall sag at VBAT_CON before fab either way.

**Source:** TI TPS25982 SLVSEI3D (fetched); power-eco.md; schematic-review.md 3.5

### H3. Battery input is a 2 A-rated JST PH 2-pin, feeding an eFuse configured for 14.7 A
*architecture / power/connectors — confidence high*

J7 (battery) is JST B2B-PH-SM4-TB — PH contacts are rated 2 A. Every load current (servo branch via J3, camera, pyro recharge, logic) passes through one PH contact each way, and the eFuse current limit is set to 14.71 A (R48 = 100 ohm), i.e. the protection will happily pass 7x the connector rating. Even the moderate case (4 micro-servos moving ~3-4 A + 1 A rest-of-board) is 2.5x the contact rating: heating plus contact-resistance sag lands directly on VBAT_CON where the un-deglitched UVLO (previous finding) samples it — the two findings compound into the exact brownout class this respin exists to kill.

**Evidence:** netlist/PCB: J7 fp=Footprints:JST_B2B-PH-SM4-TB, J7.2=VBAT_Terminal -> R72 (2 mohm) -> VBAT_CON -> U19 IN; U19 ILIM: Net-(U19-ILIM)=R48 100 ohm -> 14.71 A typ per TPS25982 ILIM table (13.56-15.66 A at 25C). JST PH series rating 2 A/contact.

**Fix:** Move the pack input to a connector rated for the real pack current (XT30 pigtail, or at minimum PH->multiple parallel positions), and/or reduce RILIM to ~182 ohm (8.1 A typ) so the limit matches what the wiring can actually carry. Re-rate after deciding the true servo class.

**Source:** TI TPS25982 ILIM table (fetched); JST PH rating (vendor standard, not fetched)

### H4. Battery input J7 (JST PH 2-pos) rated 2 A/contact vs 14.7 A eFuse allowance and 8-12 A servo stall load
*connectors / connectors/battery — confidence high*

J7 = JST B2B-PH-SM4-TB, one contact per polarity (pin1=GND, pin2=VBAT_Terminal -> R72 2 mOhm -> VBAT_CON -> U19 TPS259824 eFuse -> VBATT). JST PH is rated 2 A with AWG24 wire. The eFuse current limit is set by R48=100R (ILIM pin 8) to ~14.7 A, and the board legitimately feeds 4 servos at 2-3 A stall each via J3 (VBATT, 22-node net), plus camera (J4.2), logic, and pyro charge. Sustained multi-servo activity of 4-8 A is 2-4x the connector rating on a single contact pair; worst-case stall approaches 6-7x. Overheated/failed battery contact mid-flight kills everything downstream including pyro charge.

**Evidence:** netlist.net: J7.1=GND, J7.2=VBAT_Terminal (3 nodes: J7.2, R72.2, U23.13); Net-(U19-ILIM)= R48.2 (val 100) + U19.8; VBATT feeds J3.15/16, J4.2, FL1.1, FL2.1. BOM: J7 'JST_B2B-PH-SM4-TB'. power-eco.md CHANGE 2 documents 14.7 A typ ILIM at 100R.

**Fix:** Move the battery input to a connector rated for the real load (XT30 8-15 A, JST VH 10 A, or at minimum doubled-up PH contacts per polarity), or drop RILIM so the eFuse limit protects the connector and accept reduced servo capability. Match pigtail wire gauge (AWG20 or heavier).

**Source:** JST ePH datasheet (jst-mfg.com/product/pdf/eng/ePH.pdf, 2 A AWG24); TI TPS25982x (ILIM equation per power-eco.md)

### H5. Servo power path through J3 Milli-Grid: 2 A/contact pins, 26-30 AWG crimps and an 8.1 A return FET vs 8-12 A worst-case servo stall
*connectors / connectors/servo (J3 EXP) — confidence high*

The servo adapter (hardware/servo-adapter: J1 = TE 826576-6 MTA-156, GND/Servo4..1/LiPoPos, fanning to two 6130xx21021 headers with 4x100 uF bulk) has no Milli-Grid mate, so its harness must originate at rocket J3 (Molex 87832-1620): VBATT on pins 15/16 only (2 contacts x 2 A), return on pins 1/2 only (2 contacts, through Q8 PMPB14XN rated 8.1 A continuous @25 C, 5.1 A @100 C, ~1.4 W at 8 A). Milli-Grid crimp terminals take 26-30 AWG; the adapter's single MTA-156 power contact is 7 A. Four servos at 2-3 A stall = 8-12 A shared across two 2 A contacts each way. Additionally SERVO_ACT turn-on slams Q8 into charging the adapter's 400 uF bulk (IDM 33 A ok, but stress), and the servo PWM reference is the switched return (see low-side finding).

**Evidence:** netlist.net: VBATT nodes include J3.15/16 only for power; Net-(J3-Pad1)={J3.1,J3.2,Q8}; net_servo-adapter.net: J1.1=GND,J1.6=LiPoPos, J2/J3 headers 3A-class; Molex 87832 rated 2 A/contact (Molex PS-87831/distributor spec); PMPB14XN ID=8.1 A (datasheet table 4); TE MTA-156 7 A/contact.

**Fix:** Give the servo branch dedicated high-current pins: parallel more J3 contacts for VBATT and return (e.g. 4+4), or add a separate 2-pos high-current servo power connector (XT30/VH) and keep J3 signal-only; replace Q8 with a larger FET or move it high-side; specify 22 AWG-capable terminals for the harness (Milli-Grid cannot -- so power must leave J3).

**Source:** Molex 87832 series (molex.com PS-87831-027 / RS listing: 2 A); Nexperia PMPB14XN; TE MTA-156 (te.com MTA-100/156 catalog: 7 A)

### H6. All four peripheral ports are ground-side switched while their supply pins stay live: off-state peripherals float to VBATT and inject into MCU GPIOs through signal lines
*connectors / power-switching/all peripheral connectors — confidence high*

GNSS J1, camera J4, EXP/servo J3 and LoRa J5 are switched by N-FET low-side switches Q1/Q7/Q8/Q10 (PMPB14XN: 1,2,5,6,7=D to connector 'ground' pin; 4,8=S to GND; 3=G via 1k from GPS_ACT/CAM_ACT/SERVO_ACT/LoRa_ACT with 10k pulldowns, so all default OFF at power-up). Meanwhile each port's positive supply is UNswitched pack voltage: J1.3 (FL1->VBATT), J4.2 (VBATT), J3.15/16 (VBATT), J5.2 (FL2->VBATT). With the return open, the whole peripheral floats toward VBATT (6.4-8.4 V), so every signal pin (GNSS_RX/TX/RXD2 -> P4 GPIO4/3/2, Camera_RX/TX -> P4 GPIO31/30, EXP_01..12 -> 12 P4 GPIOs, LoRa_RX/TX -> S3 GPIO10/11 -- all direct, no series resistors) becomes an above-abs-max (3.6 V) injection path into P4/S3 pins, including into the UNPOWERED P4 domain that the S3-gates-P4 interlock (schematic-review.md C-4) and power-eco.md CHANGE 6 tried to keep cold. It also means 'shed' peripherals are phantom-powered through their signal pins, and this is a credible cumulative-stress mechanism for the deaf-UART GNSS daughterboard (its RXD input gets pulled low relative to its floating ground at every off/on cycle). Injection magnitude is topology-dependent (uA..mA) but uncontrolled by design. V8 used high-side switching; this is a V9 regression class.

**Evidence:** netlist.net: Net-(J1-Pad4)={J1.4,Q1 drains}; Net-(J4-Pad1)={J4.1,Q7 drains}; Net-(J3-Pad1)={J3.1,J3.2,Q8 drains}; Net-(J5-Pin_1)={J5.1,Q10 drains}; Q*.4/8=GND; gates via R6/R24/R27/R34 (1k) + R7/R25/R29/R35 (10k->GND); GNSS_RX 2 nodes {J1.1,U17.4} (no series R); Nexperia PMPB14XN DS: pin map 1,2,5,6,7=D / 3=G / 4,8=S confirms low-side topology.

**Fix:** Before fab, either (a) revert these branches to high-side P-FET switching (as V8) keeping grounds hard-wired to the connectors, or (b) keep low-side switching but add series resistors (>=1 k) or bus switches on every cross-connector signal AND switch the positive feed too. At minimum, hard-ground LoRa J5 pin 1 and GNSS J1 pin 4 (their daughterboards already have local switching value only for load-shedding).

**Source:** Nexperia PMPB14XN (assets.nexperia.com/documents/data-sheet/PMPB14XN.pdf): 40 V, Vgs +/-8 V, ID 8.1 A, pinning table 2

### H7. S3 log flash U13 powered from VDD_SPI pin violates the 14-ohm R_SPI budget once the RH2's in-package PSRAM shares it
*esp32-s3 / S3 VDD_SPI / external flash — confidence high*

This settles the recorded open item (VDD_SPI ~14-ohm budget + C27). BOM/netlist: U15 = ESP32-S3RH2 (confirmed real ordering code, datasheet v2.2 Table 1-1: 2 MB Quad-SPI in-package PSRAM, -40~105C, VDD_SPI = 3.3 V, chip rev v0.2; R2 is EOL and upgraded to RH2). In 3.3 V mode VDD_SPI is fed internally from VDD3P3_RTC through R_SPI = 14 ohm typ (Table 5-3), and the datasheet requires VDD3P3_RTC > VDD_flash_min + I_flash_max x R_SPI. On the RH2 the in-package PSRAM hangs on this same node (PSRAM min 2.7 V, Table 5-12), and the board ALSO powers the external W25Q128JVSIQ (min 2.7 V, read <=20 mA @104 MHz, program/erase <=25 mA) from net OUT_VDD_SPI (U15.29 -> U13.8). Budget: at rail min 3.0 V even flash alone fails (3.0 - 0.025x14 = 2.65 V < 2.7 V); at 3.3 V nominal, concurrent flash program (25 mA) + PSRAM traffic (~25-30 mA typical for 2 MB quad PSRAM; Espressif publishes no figure) gives 3.3 - 0.055x14 = 2.53 V — both memories out of spec exactly during heavy logging. Shared-bus topology itself is correct (Table 2-14: external flash on SPICS0 pin 32, in-package PSRAM on SPICS1 pin 28 NC, shared SPICLK/D/Q/WP/HD — as placed). Also, in light-sleep VDD_SPI is powered down (Table 5-10), so the log flash would be power-cycled every sleep. Present parts on OUT_VDD_SPI: C26 100 nF (B.Cu at U13) + C27 10 uF (F.Cu 2.7 mm from pin 29) — C27=10 uF exceeds the HDG guidance ('add 0.1 uF and 1 uF close to VDD_SPI... do not add excessively large capacitors'). Note: 'C27' in the July-12 docs maps to today's C27 only by coincidence of renumbering; current C27 is the 10 uF on OUT_VDD_SPI.

**Evidence:** netlist.net: net OUT_VDD_SPI = {U15.29 VDD_SPI, U13.8 VCC, C26 100nF, C27 10uF} and nothing else. U15 pin 51 (GPIO45) unconnected -> internal WPD -> 3.3 V VDD_SPI mode (Table 3-1). ESP32-S3 datasheet v2.2 (scratchpad/review/esp32-s3-datasheet.pdf): p13 Table 1-1 (RH2), p29 Table 2-11, p31 Table 2-14, p64 Table 5-2 note 2, p65 Table 5-3 (R_SPI 14 ohm), p68 Tables 5-11/5-12 (flash/PSRAM min 2.7 V) and Table 5-10 (light-sleep powers down VDD_SPI). W25Q128JV: read 20 mA max @104 MHz, program/erase 25 mA max (Winbond W25Q128JV rev datasheet via Mouser/Winbond).

**Fix:** Exact final config (Espressif-standard chip-down for R-variants): (1) In esp32s3_outputs.kicad_sch move U13 pin 8 VCC off OUT_VDD_SPI to +3V3; keep C26 100 nF at U13 (now decoupling +3V3) and optionally add 1 uF there. (2) Leave VDD_SPI (U15.29) on its own node with 1 uF + 0.1 uF only: change C27 10 uF -> 1 uF and add a 100 nF at pin 29. (3) Keep GPIO45 floating (3.3 V mode). Resulting budget: VDD_SPI serves only the in-package PSRAM: ~30 mA x 14 = 0.42 V -> ~2.88 V at 3.3 V nominal, >= 2.7 V min — the same envelope Espressif ships. No external series resistor is needed; the 14-ohm budget is reserved entirely for the PSRAM. Documented alternative (also acceptable, single edit): tie OUT_VDD_SPI to +3V3 — datasheet Table 5-2 lists VDD_SPI as an input at 3.3 V ('in-package memory backup power line'); this removes the R_SPI drop for both memories and keeps the flash powered in light-sleep (+40 uA per Table 5-10 note 1).

**Source:** ESP32-S3 Series Datasheet v2.2 (documentation.espressif.com/esp32-s3_datasheet_en.pdf); ESP32-S3 Hardware Design Guidelines schematic checklist (docs.espressif.com); Winbond W25Q128JV

### H8. CHIP_PU (EN) has the 10 k pull-up but no RC delay capacitor — violates Espressif's 50 us t_STBL power-up requirement
*esp32-s3 / S3 reset/EN — confidence high*

Net Net-(U15-CHIP_PU) contains exactly U15.4 and R36 (10 k to +3V3) — no capacitor, no reset button, no test point. Espressif requires >=50 us between the 3.3 V rails being stable and CHIP_PU going high, and the HDG prescribes an RC of R = 10 k / C = 1 uF on EN. With only a 10 k to the same rail, CHIP_PU tracks +3V3 with essentially zero delay, so every cold boot and every brownout-recovery boots outside spec — intermittent failure-to-boot with no field remedy except pulling the pack (the S3 has no reset button; S2 is BOOT/GPIO0 only, and U0TXD/U0RXD are unconnected so USB is the only console). On a flight computer with this project's brownout history that is a must-fix. Note the P4 side has the same construction (Net-(U17A-CHIP_PU)) — same fix applies there, flagged for the P4 reviewer.

**Evidence:** netlist.net: Net-(U15-CHIP_PU) = {U15.4, R36.1}; R36.2 = +3V3; no C on the net. PCB: R36 at (73.92,143.35), U15 pin 4 pad at (75.53,143.67) — room exists for a 0402 cap. HDG schematic checklist: 'Time reserved for the power rails to stabilize before the CHIP_PU pin is pulled high: 50 us'; 'usually R = 10 kOhm and C = 1 uF'.

**Fix:** Add a 1 uF 0402 from CHIP_PU to GND placed at U15 pin 4 (next to R36). Opportunistic: add a test point or pad on CHIP_PU so the chip can be hard-reset on the bench.

**Source:** ESP32-S3 Hardware Design Guidelines — schematic checklist (docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/schematic-checklist.html)

### H9. P4 pad 54 left as NC, but on ESP32-P4 chip revision v3.0+ pin 54 is core-supply pin VDD_HP_1
*layout-decoupling / cpu/esp32-p4 — confidence high*

The board symbol/footprint treat U17 pin 54 as 'NC_54' (no-connect). Espressif: 'In chip revisions v3.0 and later versions, pin 54 of ESP32-P4 is defined as VDD_HP_1; in chip revisions v1.0 and v1.3, this pin is defined as NC', and 'For new designs, please refer to the schematic of chip revisions v3.0 and later versions.' Chips purchased in 2026 will increasingly be v3.0+; on those parts one of the four VDD_HP core-supply pins floats, concentrating core current (P4 core is the heaviest rail on the board) into the remaining three pins and departing from the validated reference design. Tying pad 54 to ESP_VDD_HP is safe on v1.x silicon (pin unbonded).

**Evidence:** netlist.net: U17.54 = 'NC_54' on net 'unconnected-(U17A-NC-Pad54)'. Scratchpad p4_datasheet.txt (v0.7) pin table line '54 VDD_HP_1 Power'. Espressif HDG schematic-checklist-esp32p4: revision note quoted above; VDD_HP decoupling rec = 10uF at entry + 0.1uF per pin.

**Fix:** Route U17 pad 54 to ESP_VDD_HP and add a 100nF adjacent to it (matching pins 26/76/91 treatment).

**Source:** Espressif ESP32-P4 datasheet + ESP Hardware Design Guidelines (docs.espressif.com/.../esp32p4/schematic-checklist-esp32p4.html)

### H10. TPS62152 (U18) power-input pin PVIN pad 12 is unconnected in schematic and layout
*layout-decoupling / power/buck-3V3 — confidence high*

The main 3.3V buck has two power-stage input pins (11,12). Pin 11 is tied to Net-(U18-AVIN) with C43 22uF; pin 12 is on net 'unconnected-(U18-PVIN-Pad12)' and has no copper. The TI datasheet pin table requires both: pins '11,12 PVIN - Supply voltage for power stage. Connect to same source as AVIN.' All buck input current (S3 + NAND + the entire switched P4 domain via U22, worst case ~0.6-1 A at low battery) funnels through the single pin-11 bond/pad, and the recommended input-cap loop ('place capacitors between PVIN and PGND as close as possible to those pins') is degraded. One-trace fix: pour pad 12 into the adjacent AVIN copper.

**Evidence:** netlist.net: net 'unconnected-(U18-PVIN-Pad12)' contains only U18.12 (PVIN_12); PCB pad U18.12 at (91.15,137.45) B.Cu with no track/zone. TPS62152 DS (SLVSAG7) pin table line 'AVIN 10 ... Connect to same source as PVIN' / '11,12 PVIN ... Connect to same source as AVIN'; layout section 'between PVIN and PGND as close as possible to those pins'. C43 pad1 (92.42,135.70) B.Cu, 2.49mm from pin 11 (OK once pin 12 joined).

**Fix:** Connect U18 pad 12 to Net-(U18-AVIN) (adjacent pad 11 / C43 pour) in schematic and layout before fab.

**Source:** TI TPS62152 datasheet (tps62152.pdf, layout+pin-function sections)

### H11. C12 10mF radial can body (fab outline diameter 18mm) sits on top of populated bottom-side parts, and the footprint has no courtyard so DRC cannot see it
*layout-decoupling / pyro/mechanical — confidence high*

C12 (EKYC160ELL103MM25S) is a vertical THT radial at (84.89,144.34) on B.Cu, 7.5mm lead pitch, body circle r=9.0mm drawn only on B.Fab. Components inside that radius on the same side: Q4 DTC123 at 1.02mm from can center, Q3 at 2.51mm, C10 22uF 0805 at 4.30mm, L6 buck inductor (3x3x1.2mm tall) at 5.02mm, C13 22uF at 6.07mm, and at the rim U21 TPS2121 (7.55mm) and U18 buck (7.76mm). Even if the can is 16mm dia, Q3/Q4/C10/C13/L6 remain under it; a flush-seated can rests on parts up to ~1.2mm tall - tilted body, stressed leads, vibration risk in flight. The footprint has zero F/B.CrtYd geometry, which is why DRC reports no overlap here (the only courtyard error on the board is J6/FID1). Polarity: pad1=V_CAP(+)/pad2=GND matches the ECO, but the only silk polarity mark is a 1.5mm-radius circle at the negative pad. The ECO's own open item ('verify the exact can diameter <=18mm and height for this case-code against the Chemi-Con KY datasheet and your bay clearance') is still unresolved; web sources for this P/N are ambiguous between 16x25 and 18x25.

**Evidence:** rocket-computer.kicad_pcb footprint C12: pads at local (-3.74,-0.01)/(3.76,-0.01) drill 1.0; fp_circle B.Fab center(0.01,-0.01) r=9.00; no CrtYd items. Neighbor distances computed from footprint coordinates (all B.Cu). drc.json: only courtyards_overlap = J6+FID1. render_bottom.png shows the can body overlapping the L6/U18/U21 cluster.

**Fix:** Confirm the real case size from the Chemi-Con KY datasheet, add a proper courtyard (body + lead margin) to the footprint, then either relocate the can or clear Q3/Q4/C10/C13/L6 (and U21/U18 if 18mm) from under it; if a spacer-mounted can over parts is intended, document standoff height and add strain relief/staking for flight vibration.

**Source:** Chemi-Con KY series (exact MM25S case dims not conclusively pinned - see checks_blocked)

### H12. Only one fiducial on the whole board, and it sits under the USB-C connector body
*layout-misc / assembly/fiducials — confidence high*

FID1 (Fiducial:Fiducial_0.5mm_Mask1mm, F.Cu, 84.32,168.07) is the board's ONLY fiducial. It lies fully inside J6's courtyard (USB4110-GF-A, courtyard X 77.91..90.81 / Y 162.89..171.32) — this is the one DRC courtyards_overlap ERROR — and the top render confirms the connector shell covers that spot, so it is useless for post-placement inspection and physically conflicts with the shell. There are no other fiducials: none elsewhere on top, none on bottom. Both sides carry fine-pitch chip-down parts that want vision alignment: top has the ESP32-P4 QFN-104 at 0.35 mm pitch (U17) and ESP32-S3 QFN-56 0.4 mm (U15); bottom has the TPS2121RUXR X2QFN at 0.4 mm pitch (U21), two 0.5 mm QFNs (U18 buck, U19 eFuse) and the INA230 QFN16 (U23). A 0.5 mm copper dot is also below the 1 mm minimum some assemblers require.

**Evidence:** drc.json courtyards_overlap error: 'Footprint J6' (84.36,164.79) vs 'Footprint FID1' (84.32,168.07). mfg_audit.py fid_tp: exactly one Fiducial footprint on the board. J6 courtyard global bbox computed from fp graphics; FID1 mask opening extent 83.82..84.82 / 167.57..168.57 is fully inside it. render_top.png shows no visible fiducial (hidden by J6 shell).

**Fix:** Delete or relocate FID1 and add three global fiducials per assembly side (1.0 mm copper dot, 2.0 mm mask opening), placed asymmetrically near three corners, >=5 mm from the board edge and clear of all courtyards. This also clears the only DRC error.

### H13. C12 (10 mF pyro electrolytic) has no polarity marking on silkscreen
*layout-misc / assembly/polarity — confidence high*

C12 (EKYC160ELL103MM25S, V_CAP/GND, bottom side at 84.89,144.34) is a polarized radial aluminum electrolytic that will be hand-soldered (through-hole part on an otherwise reflow board). Its footprint contains exactly ONE silkscreen graphic (an outline) and no +/- or stripe marking; no board-level polarity text exists near it (nearest gr_texts are the J2 'GND 4 3 2 1' labels). Pad 1 = V_CAP (+), pad 2 = GND. A reversed 16 V electrolytic charged to 8.4 V via R20 will heat and vent - on the pyro energy store, in the airframe. The ECO (power-eco.md Change 7) explicitly says 'set polarity: + -> V_CAP, - -> GND' but the footprint never got the marking.

**Evidence:** fp_detail.py C12: 'silk graphic items: 1'; pads 1=V_CAP, 2=GND, drill 1.0 mm, 7.5 mm pitch. mfg_audit.py texts: no polarity text near (84.89,144.34) on B.SilkS. render_bottom.png shows no +/- near the can. Contrast: J7 battery has '- Batt +' silk, buzzer LS1 has '+'.

**Fix:** Add a '+' silk label next to C12 pad 1 (V_CAP) and a filled polarity bar on the negative side, placed clear of the Ø18 can body so it stays visible after soldering.

**Source:** power-eco.md Change 7; Chemi-Con KYC product page (chemi-con.co.jp)

### H14. C12 can is really Ø18x25 mm (repo STEP is Ø16) and its body sits on top of ~10 components up to 1.45 mm tall
*layout-misc / mechanical/C12 — confidence high*

Chemi-Con's product page for EKYC160ELL103MM25S gives KYC series, Ø18 x 25 mm, 10,000 uF/16 V, 30 mOhm — the repo 3D model (hardware/3dmodels/EKYC160ELL103MM25S.STEP) measures only Ø16, so the renders and any model-based clearance checks understate the can by 2 mm diameter (the ECO's own 'verify the exact can diameter' warning is still open, now resolved: 18 mm). Within the real Ø18 body circle centered (84.89,144.34) on the bottom side sit C9/C10/C11/C13 (0805 22 uF, ~1.4-1.5 mm tall incl. solder), L6 (VLS3012, 1.2 mm), U18 (QFN buck, 1.0 mm), U21 (X2QFN), Q3/Q4 (SOT-23, ~1.1 mm), R23 near the rim. The can base cannot sit flush; a ~10 g mass ends up standing off on two 1 mm leads (plus resting on part bodies) under flight vibration and shock. C12's footprint also has no courtyard, and the DRC config ignores missing_courtyard, so nothing checks this collision. Board fit itself is OK: Ø18 spans x 75.9..93.9 vs edges 72.36/94.71.

**Evidence:** WebFetch chemi-con.co.jp print-condenser.php?part_number=EKYC160ELL103MM25S: 'Case Diameter: 18mm, Length: 25mm'. STEP bbox parse: X extent 16.0 mm. Distances from (84.89,144.34): C11 4.4, C10 4.3, C13 6.1, C9 5.6, L6 5.0, U18 7.8, Q4 1.0, Q3 2.5 mm — all < 9.0 mm radius. drc.json ignored_checks includes missing_courtyard.

**Fix:** Replace the STEP with the true Ø18x25 model and add an Ø18+margin courtyard to the C12 footprint; either clear the tall parts (the 0805s and L6) out of the can's footprint or plan formed leads + adhesive/RTV staking of the can to the board; re-verify bay inner diameter against board 1.6 mm + 25 mm can.

**Source:** https://www.chemi-con.co.jp/en/products/print-condenser.php?part_number=EKYC160ELL103MM25S

### H15. TPS62152 (U18) PVIN pin 12 is unconnected - datasheet requires both PVIN pins tied to the input
*layout-power / 3v3-buck — confidence high*

U18 pad 12 carries net 'unconnected-(U18-PVIN-Pad12)' in both the netlist and the PCB (no copper); ERC flags 'Symbol U18 Hidden pin 12 [PVIN, Input]' not connected. TI's datasheet pin table: '11,12 PVIN - Supply voltage for power stage. Connect to same source as AVIN.' Half the power-stage input connection is missing on the always-on 3.3 V rail that feeds the S3, logging, and (via U22) the whole P4 domain. It may work in the lab at reduced capability, but it is out of spec on the most flight-critical converter. Pads 10 (AVIN), 11 (PVIN), 13 (EN) are correctly on Net-(U18-AVIN); the fix is a trivial copper stub since pad 12 is adjacent to pad 11.

**Evidence:** netlist.net U18: pin 11 PVIN -> Net-(U18-AVIN), pin 12 PVIN -> unconnected-(U18-PVIN-Pad12); erc.json pin_not_connected on U18 hidden pin 12; U18 pads 11/12 at (90.65,137.45)/(91.15,137.45) B.Cu; TPS62152 datasheet (SLVSAB2) pin functions lines 190-191 of extracted text

**Fix:** Connect pad 12 to the Net-(U18-AVIN) copper (extend the B.Cu AVIN zone over pads 10-13); fix the schematic symbol so PVIN-12 is a visible, connected pin.

**Source:** TI TPS62150/51/52/53 datasheet, pin functions and layout section

### H16. Servo feed chain (VBATT->J3->Q8 return) sized for ~2-4 A, spec'd worst case is 12-14 A
*layout-power / servo-distribution — confidence high*

Every element of the servo branch is undersized for the stated 12-14 A stall: (1) VBATT distribution to J3 runs on the In2 inner-layer zone (166 mm2 total) whose aggregate min-cut is only 2.5 mm of 1-oz inner copper at y=105.5 (~3.8 A at 10 C rise with 0.5x inner derate; single widest-path neck 0.70 mm at (90.79,102.23)); (2) the In2 zone is entered from the eFuse through a 4-via cluster (79.0,133.4-138.1) and exits to J3 pads through 4 vias (91.7-92.3, y 101.4-107.3) - 0.3 mm drill vias at 3-3.5 A each vs the 1-2 A guideline; (3) J3 (Molex 87832-1620 Milli-Grid) supply = 2 contacts, return = 2 contacts at 2.0 A/contact (4 A per direction); (4) the switched return Net-(J3-Pad1) necks to 0.85 mm B.Cu; (5) return switch Q8 (PMPB14XN, DFN2020-6 2x2 mm) is abs-max rated 11.5 A for t<=5 s on a 6 cm2 pad, RDS(on) 15-18 mOhm at Vgs=4.5 V (driven here at 3.3 V, so higher) - 12 A stall = ~3 W in a 2x2 mm package -> thermal runaway; its GND source current exits through 2 tiny pads and ~5 nearby GND vias (2.4-2.8 A/via at stall).

**Evidence:** Min-cut scan of VBATT copper: y=105.5 total 2.50 mm (all In2); y=120-132: 3.2-3.45 mm In2-only. widest_path: U19.20->J3.15 bottleneck 0.70 mm on In2.Cu; J3.1->Q8.1 bottleneck 0.85 mm B.Cu. Via positions from pcb parse. Molex Milli-Grid PS-87831-027 (2.0 A/contact); Nexperia PMPB14XN datasheet Table 1 (ID 11.5 A t<=5s, RDSon 15/18 mOhm typ/max at 4.5 V)

**Fix:** Widen the In2 VBATT corridor to >=6-8 mm (or duplicate on B.Cu), raise via clusters to 8-10 per transition, give the servo return a direct wide path, and replace Q8 with a FET sized for stall (e.g. LFPAK33/56 class, >=30 A, with real source copper). Reconsider Milli-Grid for servo power (2 A/contact) - move servo power to dedicated high-current pins/connector. Alternatively bound the worst case: firmware/mechanical guarantee that 4-servo stall cannot persist, and size for the bounded current - but the copper should still cover >=2x nominal.

**Source:** Nexperia PMPB14XN; Molex PS-87831-027; IPC-2152 approx (1 mm ~ 3 A at 10 C, 1 oz outer; inner x0.5; 0.3 mm via ~1.5-2 A)

### H17. PVIN pin 12 unconnected in schematic and isolated on PCB
*power-input / 3V3 buck (U18 TPS62152) — confidence high*

The TPS62152 has two power-stage input pins (11,12 PVIN). Pin 12 is a hidden pin in the custom symbol and never got wired; the netlist assigns it 'unconnected-(U18-PVIN-Pad12)' and the PCB pad at (91.150,137.450) is electrically isolated copper while pads 10/11/13 (AVIN/PVIN/EN) are on Net-(U18-AVIN). All power-stage input current funnels through the single 0.26mm-wide pad 11, and the input hot-loop from C43 to the pin-12 side is broken. Datasheet pin table: pins 11,12 PVIN 'Supply voltage for power stage. Connect to same source as AVIN.' The part will probably still run at this load (<1A) but it is out-of-datasheet and degrades the input loop; this is a one-trace fix before fab.

**Evidence:** netlist.net: U18.12 -> unconnected-(U18-PVIN-Pad12); erc.json: pin_not_connected error 'Symbol U18 Hidden pin 12 [PVIN, Input, Line]'; rocket-computer.kicad_pcb: U18 pad 12 net='unconnected-(U18-PVIN-Pad12)' adjacent to pad 11 net='Net-(U18-AVIN)'; TPS62150/52 datasheet pin functions table (pins 11,12).

**Fix:** Unhide/connect PVIN pin 12 to the AVIN/PVIN net in the U18 symbol instance, re-annotate the net, and pour/trace PCB pad 12 into the existing Net-(U18-AVIN) copper (pad 11 is 0.5 mm away, C43 is on the same net).

**Source:** TI TPS62150/TPS62152 (via ti.com/lit/ds/symlink/tps62152.pdf)

### H18. BOM carries no MPNs; voltage/power-critical passives on 8.4 V rails are unpinned
*power-input / BOM — confidence high*

bom-fresh.csv has an empty MPN column for every line. Specifically at risk: R72 shunt is just '2 m' 0805 (needs a metal-strip current-sense part with known tolerance/TCR/power — 0.43 W flows at the 14.7 A eFuse limit); the 22 uF 0805 caps sitting on 8.4 V nets (C7,C9,C10,C11,C13,C14,C18 on VBATT/V_CAP, C43 on the buck input) need >=16 V X5R/X7R parts pinned — many stock 0805 22 uF are 6.3-10 V and would lose most capacitance at 8.4 V bias or be over-voltaged; the ECO requires CIN C41/C42 to be >=25 V; C65 47 uF on 3V3 is uncritical. The eFuse dVdt/ITIMER/NRETRY caps should also be C0G/X7R with pinned values since timing depends on them.

**Evidence:** bom-fresh.csv: MPN column empty on all rows; e.g. '"C7,C9-C11,C13,C14,C18,C43","22 uF","Capacitor_SMD:C_0805_2012Metric","",...' and '"R72","2 m","Resistor_SMD:R_0805_2012Metric","",...'; power-eco.md Change 2 pin map requires CIN >=25 V.

**Fix:** Populate MPNs before ordering: 16-25 V rated 22 uF 0805 (or accept measured derating), 25 V CIN parts, a named 2 mOhm 0805/1206 current-sense resistor (>=0.5 W, <=1%, low TCR), C0G for the eFuse timing caps. This repeats closed finding C-1's lesson: re-export alone is not enough if MPNs never lived in the schematic.

### H19. Battery feed is a 2 A-rated JST PH while the eFuse limit and servo loads are 12.9-16 A
*power-input / Battery input / current budget — confidence high*

Battery enters on J7 (JST B2B-PH-SM4-TB, 2-pin PH, rated 2 A/contact at AWG24) — one power contact, one ground contact. Downstream, the eFuse current limit is set to 14.71 A typ (12.85-15.99 A over temp) explicitly to ride a ~14 A four-servo stall (per power-eco.md Change 2 note), and the servo/expansion branch J3 (Molex Milli-Grid 878321620, ~2 A/circuit) carries servo power on two VBATT contacts with the entire branch return through one DFN2020 FET Q8 (PMPB14XN: Ptot 1.9 W at 6 cm2 pad; 15/18 mOhm typ/max at VGS=4.5V — ~8-14 A stall would dissipate 1-3.5 W). Realistic concurrent load (servo transients + camera record + logic + LoRa TX) is ~3-6 A, already 1.5-3x the PH contact rating; a fault only trips the eFuse at ~13-16 A, far above every connector's ampacity, so the connectors and Q8 are the fuses. Note the ECO's own V8 diagnosis blamed 'connector micro-disconnects' — a hot, undersized contact makes that worse.

**Evidence:** netlist.net: J7.2 -> VBAT_Terminal (sole battery+ contact); J3.15/16 -> VBATT, J3.1/2 -> Net-(J3-Pad1) -> Q8 drains, Q8.4/8 -> GND; R48=100R on U19 ILIM -> 14.71 A typ per TPS25982 EC table (RILIM=100R row: 13.56/14.71/15.66 A, cold-min 12.85 A); JST PH catalog rating 2 A/100 V per contact; PMPB14XN datasheet Ptot 1.9 W, RDSon 15/18 mOhm.

**Fix:** Either (a) fit a battery connector rated for the design current (XT30 class, 15-30 A) and give the servo branch more contacts, or (b) consciously shrink the eFuse ILIM to what the distribution can carry (e.g. RILIM ~300R -> ~5 A) and cap the servo duty; document whichever is chosen. Also check Q8 dissipation against the real servo stall spec.

**Source:** TI TPS25982 SLVSEI3D; Nexperia PMPB14XN; JST PH series catalog (jst.com ePH.pdf)

### H20. All four peripheral branches are ground-switched; off-state branch grounds float to VBATT with live signal lines into both MCUs
*power-input / Peripheral power switching — confidence high*

Q1 (GNSS, GPS_ACT=P4 GPIO15), Q7 (camera, CAM_ACT=P4 GPIO32), Q8 (servo/expansion, SERVO_ACT=P4 GPIO8), Q10 (LoRa, LoRa_ACT=S3 GPIO12) are PMPB14XN N-FETs in the branch GROUND return; branch power pins stay hard-wired to VBATT (6.4-8.4 V). All gates have 10 k pulldowns, so every branch boots OFF. With a daughterboard attached and its ground open, the device's local ground floats up toward VBATT through its own load, and its UART/GPIO pins (referenced to that floating ground) present near-VBATT potentials to the still-grounded MCU pins: GNSS_RX/TX/RXD2 -> P4 GPIO4/3/2, Camera_RX/TX -> P4 GPIO31/30, EXP_01..12 -> 12 P4 GPIOs, LoRa_RX/TX -> S3 GPIO10/11. ESP32 GPIO abs max is ~3.6 V; injection current is limited only by the peripheral's internals. This also back-powers the branch (defeating shedding) and, when ON, puts servo di/dt ground bounce (Rds ~15-18 mOhm) under every branch signal reference. This contradicts the power-eco.md record, which describes high-side P-FET switches (old Q9/Q3) for these branches — the netlist is authoritative and shows low-side.

**Evidence:** netlist.net: Q7 pins 1,2,5,6,7 (drain) -> Net-(J4-Pad1) = J4.1 (camera GND pin), pins 4,8 (source) -> GND; identically Q1->J1.4, Q8->J3.1/2, Q10->J5.1; J4.2/J1.3(via FL1)/J3.15,16/J5.2(via FL2) all remain on VBATT; PMPB14XN datasheet pinout 1,2,5,6,7=D 3=G 4,8=S confirms low-side wiring; ESP32 GPIO abs max 3.6 V.

**Fix:** Switch these branches high-side (P-FET + NPN driver, as the ECO originally specified) or, at minimum, add series resistors (~1 k) or bus isolation on every signal crossing into a switchable branch and verify injected current is within ESP32 clamp limits. If the branches will in practice never be off while a daughterboard is connected, document that and remove the pretense of shedding.

**Source:** Nexperia PMPB14XN (assets.nexperia.com); Espressif ESP32-P4/S3 abs max (from prior project verification)

### H21. A single enabled internal pull-up on a P4 fire GPIO fully turns on that fire FET (DTC123J R2=47k is too weak); arm gate has the same weakness via R22=100k
*pyro / pyro gate drive / boot safety — confidence high*

Hardware requirement: no pyro fire from boot-time pin states regardless of firmware. Verified GOOD at reset: ESP32-P4 datasheet v0.7 Table 2-1 shows GPIO6/9/11/13 (PYRO1/3/2/4_FIRE via Q5/Q4/Q3/Q6) and GPIO16 (PYRO_ARM) all high-Z with no IE/WPU at or after reset, and their IO MUX F0 default is plain GPIO (no boot-ROM-driven function). NOT protected against the already-observed firmware class (memory: 'gpio_reset_pin fires squib rail'): P4 internal WPU = 45 kΩ typ. Fire path: 3.3 V via 45 k into DTC123J (R1=2.2k, R2=47k) gives ~1.65 V at the input — above the guaranteed-ON V_I(on) min of 1.1 V — with ~42 µA base current × GI min 80 = 3.4 mA sink vs the 0.84 mA needed to pull the TPN4R712MD gate low through R15-R17/R23 (10k to V_CAP) → fire FET fully ON. Arm path: WPU on GPIO16 through R21 (100R) against R22 (100k) puts 3.3×100/145.1 = 2.27 V on the CSD16323Q3 gate, above its Vgs(th) max 1.4 V → arm FET ON. WPUs enabled on both one fire pin and the arm pin (e.g., a blanket gpio_reset_pin/gpio_config over the pyro bank) fires a channel, since V_CAP is always charged when a battery is connected (R20 has no switch). Robust across the WPU spread: even at 70 k the DTC input is 1.30 V (>1.1) and arm gate 1.94 V (>1.4).

**Evidence:** netlist.net: PYROx_FIRE = U17 GPIO6/9/11/13 → Q5/Q4/Q3/Q6 pin B; Net-(Qx-C) = TPN gate + 10k to V_CAP; PYRO_ARM = U17 GPIO16 → R21 100R → Net-(U9-GATE), R22 100k → GND. ESP32-P4 datasheet v0.7 pp.14-17 (reset pin settings all '–' for these pins), p.90 (RPU 45 kΩ). Rohm DTC123J datasheet Rev.006: R1 2.2k/R2 47k, V_I(on) min 1.1 V, V_I(off) max 0.5 V, GI min 80. TI SLPS224C: Vgs(th) 0.9-1.4 V.

**Fix:** Zero-layout-change hardening: (1) change Q3-Q6 to the 2.2k/2.2k ratio part (DTC123E series, same EMT3F package) — a 45 k pull-up then yields only ~0.15 V at the input, below the 0.5 V guaranteed-OFF level; (2) change R22 from 100k to 10k — WPU then gives 3.3×10/55.1 ≈ 0.6 V on the arm gate, below Vgs(th) min 0.9 V (normal 3.3 V drive still fully enhances: 3.27 V). Optionally also gate V_CAP charging from a switched rail so fire energy is absent until deliberately enabled.

**Source:** https://fscdn.rohm.com/en/products/databook/datasheet/discrete/transistor/digital/dtc123je3-e.pdf ; https://documentation.espressif.com/esp32-p4_datasheet_en.pdf ; https://www.ti.com/lit/ds/symlink/csd16323q3.pdf

### H22. PVIN pin 12 floating - only one of the two power-stage input pins is connected
*switched-domains / Main 3V3 buck (U18 TPS62152) — confidence high*

TPS62152 RGT: pins 11 AND 12 are PVIN, 'Supply voltage for power stage. Connect to same source as AVIN.' The schematic connects pin 11 (+10 AVIN, 13 EN) to Net-(U18-AVIN) but leaves pin 12 on an ERC-flagged hidden-pin no-connect, and on the board pad 12 at (91.15,137.45) is netless copper-isolated (the B.Cu AVIN zone spans (90.0-93.2, 135.2-138.3) but keeps clearance from the orphan pad). The entire buck input current for BOTH MCU domains crowds through a single 0.25 mm QFN pad - out of spec, higher resistance/EMI in the hot loop, long-term reliability risk on the rail everything depends on.

**Evidence:** ERC: '[error] pin_not_driven @ /Power/: Symbol U18 Hidden pin 12 [PVIN, Input]'. netlist.net: unconnected-(U18-PVIN-Pad12). PCB: U18 pad 11 (90.65,137.45)=Net-(U18-AVIN), pad 12 (91.15,137.45)=unconnected, 0.5 mm apart. TI SLVSAL5E Table 6-1: '11,12 PVIN ... Connect to same source as AVIN.'

**Fix:** Connect U18 pin 12 to the AVIN net in power.kicad_sch (the symbol hides the pin - unhide or stack it), re-run parity, and let the existing B.Cu AVIN zone/short track pick up pad 12 (0.5 mm extension).

**Source:** https://www.ti.com/lit/ds/symlink/tps62152.pdf (SLVSAL5E, Table 6-1 Pin Functions)

### H23. Low-side (ground-return) switching leaves V+ hot and creates back-feed paths through every signal line into P4/S3 GPIOs
*switched-domains / Peripheral power switching (Q1/Q7/Q8/Q10) — confidence high*

All four peripheral branches switched the GROUND return (PMPB14XN N-FET low-side), not the supply: camera J4.2, servo J3.15/16, GNSS J1.3 (via FL1), LoRa J5.2 (via FL2) sit at raw VBATT (6.4-8.4 V) whenever the pack is connected. With a branch OFF, the peripheral's local GND floats up toward VBATT and every signal line crossing the boundary becomes a sneak path: camera UART (J4.3/4 direct to P4 GPIO31/30, no series R), GNSS UART (GPIO2/3/4), servo PWM (EXP_01..12), LoRa UART (S3 GPIO10/11). Current is injected through the peripheral's and the MCU's ESD clamp diodes - into the OFF V_MCU_SWTCH domain (re-opening exactly the ECO Change 6 sneak-power scenario via connectors, with the QOD's 25 ohm as the return) or into the live 3V3 rail for the always-on S3. Reverse direction too: an idle-high UART TX (P4 or S3) phantom-powers the floating peripheral through its RX pin. The OFF state with peripherals cabled is the NORMAL pad state (all ACT gates default low). This contradicts the high-side topology recorded in power-eco.md Change 3/Change 0 table and is unbounded by any series impedance today.

**Evidence:** netlist.net: J4.1->Q7 drains, J4.2->VBATT; J3.1/2->Q8, J3.15/16->VBATT; J1.4->Q1, J1.3->FL1->VBATT; J5.1->Q10, J5.2->FL2->VBATT; Camera_RX={J4.3,U17.61} and Camera_TX={J4.4,U17.60} with no series element; PMPB14XN sources (pads 4,8) all on GND. power-eco.md Change 0 table describes Q9/Q3/Q8 as high-side VBATT switches - as-built differs.

**Fix:** Cheapest fab-ready mitigation: add series resistors in every switched-boundary signal line (1 k for the UARTs: Camera_RX/TX, GNSS_RX/TX/RXD2, LoRa_RX/TX; 330R-1k for EXP_01..12 servo PWM) to bound clamp-injection to ~1 mA, and codify the firmware rule 'branch FET ON before UART/PWM init, deinit+drive-low before OFF'. Full fix (V10): return to high-side P-FET switching per the ECO. Either way document the topology change in power-eco.md.

**Source:** https://assets.nexperia.com/documents/data-sheet/PMPB14XN.pdf (pinning Table 2); Espressif P4 guidelines (no injection spec published - industry <1 mA assumed)

## MEDIUM — should fix / consciously waive

### M1. P4 strapping pins GPIO34/37/38 routed bare to the expansion connector with no pulls
*architecture / esp32-p4/straps — confidence medium*

GPIO34 (JTAG signal source, floating default), GPIO37 and GPIO38 (strap pins, floating default) are wired directly to J3 as EXP_11/EXP_10/EXP_09. Anything the attached daughterboard or harness does to these lines during V_MCU_SWTCH power-up is latched as strap state. Today's servo adapter only listens (servo signal inputs, high-Z) so boot is likely unaffected, but any future expansion board that drives or loads these pins at power-on can corrupt JTAG/boot configuration. GPIO35 (R53 100 k pull-up + S3 button via R54 1 k) and GPIO36 (R50 100 k pull-up) are correctly handled, and the pull-up on 36 satisfies the 'GPIO36 must be high for download mode' requirement.

**Evidence:** netlist: EXP_11 [2] = J3.10 + U17.65 (GPIO34); EXP_10 = J3.11 + U17.69 (GPIO37); EXP_09 = J3.12 + U17.70 (GPIO38); no resistors on these nets. ESP32-P4 datasheet strap table (GPIO34 JTAG source, 35/36 boot, 37/38 strap, defaults floating except 35 WPU); esptool P4 boot-mode doc.

**Fix:** Add 100 k pulls (to the documented default level) on GPIO34/37/38 between the P4 and J3, or reassign EXP_09/10/11 to non-strap GPIOs before fab; at minimum document that expansion boards must not drive pins 10-12 of J3 at power-up.

**Source:** Espressif ESP32-P4 datasheet + esptool boot-mode doc (fetched; GPIO37/38 exact roles summarized with medium confidence)

### M2. S3 CHIP_PU missing the RC delay capacitor required by Espressif's design guideline
*architecture / esp32-s3 — confidence high*

Net-(U15-CHIP_PU) = R36 (10 k to +3V3) + pin only. The ESP32-S3 hardware design guideline specifies R=10 k with C=1 uF on CHIP_PU so EN rises after the 3.3 V rail is stable; without it EN tracks the rail and marginal/brown-out ramps (e.g. the eFuse auto-retry re-ramp, USB plug chatter) can violate the EN-after-VDD requirement and hang the boot. The P4 has the proper RC (R42 10 k + C39 100 nF), making the omission on the S3 look like an oversight rather than a choice.

**Evidence:** netlist: Net-(U15-CHIP_PU) [2] = R36.1, U15.4; Net-(U17A-CHIP_PU) [3] = C39.1, R42.2, U17.103. ESP32-S3 hardware design guidelines schematic checklist: 'R = 10 kohm and C = 1 uF' on CHIP_PU.

**Fix:** Add 1 uF (0402) from CHIP_PU to GND next to R36.

**Source:** Espressif ESP32-S3 hardware design guidelines (fetched)

### M3. ESP_* inter-MCU SPI remains an unprotected always-on -> off-domain path (ECO Change 6 residual), with no series resistors fitted
*architecture / inter-mcu — confidence high*

The mechanical sweep confirms ESP_CS/SCLK/SDO/SDI (S3 GPIO1/2/7/8/9 <-> P4 GPIO18-21) are the only remaining always-on push-pull lines into the switched P4 domain: 2-pin nets, no pulls, no series resistors (ECO fix 3 not taken). Mitigation is still the firmware rule (hold low/hi-Z until VPP settles). This is as the ECO decided, but V9 was the chance to add the 1 k series resistors and it didn't. If the S3 firmware ever initializes that SPI host before asserting POWER_SWITCH, it back-powers the P4 through its clamps (QOD contains the rail, but the clamp current is unlimited by design). ESP_SCL/SDA are safe (pull-ups R55/R58 to V_MCU_SWTCH, open-drain).

**Evidence:** arch_domains.py output: cross-domain nets = ESP_CS/SCLK/SDO/SDI (+ CEN_D± via mux, + I2C w/ SW-domain pulls). power-eco.md CHANGE 6 fixes 2/3.

**Fix:** Fit 4x 1 k series resistors in the ESP_* SPI lines (cheap insurance, negligible at the used SPI rate), or record the firmware ordering constraint as a hard invariant next to the C-4 interlock note.

**Source:** power-eco.md CHANGE 6

### M4. Log NAND changed again to GD5F2GQ5UE (2 Gbit) — half the capacity the July review recorded, firmware geometry constants must follow
*architecture / logging/NAND — confidence high*

The July review closed section 5-6 with F35SQB004G (4 Gbit) as the replacement for MX35UF4G24AD (4 Gbit). The V9 BOM/netlist now places GD5F2GQ5UEYIGR = 2 Gbit, 3.3 V, WSON-8 — a further, unrecorded part change that HALVES the log store. Project history makes this dangerous specifically because flash geometry constants have bitten before (the OC used only 128 of 512 MB due to hardcoded 1 Gbit geometry, fixed in #492): the S3 logging driver, log-ring sizing, and auto-evict floor (#315/#499's 10% free) all need the new geometry. Pinout on the WSON-8 footprint matches the standard SPI-NAND map in the netlist (CS 1, SO 2, WP# 3, GND 4, SI 5, SCLK 6, HOLD# 7, VCC 8, pad 9 GND) and WP#/HOLD# pull-ups R31/R33 correctly reference +3V3 (always-on domain, ECO Change 6 intact).

**Evidence:** bom-fresh.csv line 62: U11 GD5F2GQ5UEYIGR. netlist U11 pin/net map; R31/R33 -> +3V3. schematic-review.md section 5-6 ('NAND F35SQB004G ... confirm footprint/pinout on the placed part'). Project memory #492 (geometry constants).

**Fix:** Confirm the 2 Gbit capacity is intentional (availability?), update firmware geometry/driver ID tables for GD5F2GQ5, and re-run the log-capacity budget (dynamic logging rate #623 changed the write profile). If 4 Gbit was intended, GD5F4GQ6UE is the same footprint family.

**Source:** GigaDevice GD5F2GQ5 (naming/pinout from vendor convention; datasheet not fetched — see checks_blocked)

### M5. All peripheral switching is low-side (return-path): OFF peripherals float to VBATT with live signal lines into the MCUs, and shedding under load diverts current into GPIO clamps
*architecture / power/architecture — confidence high*

Q1 (GNSS), Q7 (camera), Q8 (servo/EXP), Q10 (LoRa) are N-FETs in each branch's GROUND return; the positive feeds (VBATT) are hard-wired. Consequences: (1) an OFF peripheral's local ground floats toward VBATT while its UART/signal pins remain wired to P4/S3 GPIOs — leakage flows VBATT -> peripheral internals -> signal pin -> MCU ESD clamp. When the P4 domain is also off this is contained by the TPS22918 QOD (25 ohm to GND holds V_MCU_SWTCH near 0), but the injected current is set by the peripheral's internals, not by the board. (2) ECO Change 3 'shedding' (opening Qx under load) interrupts the return while inductive/capacitive load current is flowing — that current re-routes through the signal wires into the GPIO clamps (worst for servos: amps-scale). (3) The scheme only works because each daughterboard's ONLY ground is the switched pin — verified true for LoRa (J5 has no hard GND pin; db grounds via CMC to pin 1) and GNSS (db J4.4 VSS only), but rocket J1 pins 6/7 are hard GND shield pads: any cable/shield that lands them on the daughterboard ground silently defeats the GNSS switch. The July review section 3.5 warned against exactly this class for the main disconnect; the eFuse fixed it there while this respin re-introduced it per-branch.

**Evidence:** netlist: Net-(J4-Pad1)/Net-(J1-Pad4)/Net-(J3-Pad1)/Net-(J5-Pin_1) are Qx drain nets; J4.2/J1.3(FL1)/J3.15-16/J5.2(FL2) all = VBATT unswitched; Camera_RX/TX = P4 GPIO31/30; GNSS UART = P4 GPIO2/3/4; EXP_01-12 = P4 GPIOs; J1.6/7 = GND. arch_domains.py RAW-vs-SW net list. lora-db netlist: VSS only via FL1/J6.1; gnss-px netlist: J4.4=VSS via DLW21SN CMC.

**Fix:** Minimum for V9: series resistors (=1 k) in Camera_RX/TX, GNSS_RX/TX/RXD2 and any EXP line that can be driven while the branch is off, plus a firmware rule to idle UART pins low before opening a branch and never to open the servo return under load. V10: move the branch switches high-side (P-FET) as section 3.3 originally drew.

**Source:** schematic-review.md 3.5; daughterboard netlists (exported this session)

### M6. Servo branch current path (J3 Milli-Grid + Q8 DFN2020) is undersized for the ECO's own stall numbers
*architecture / power/servo-branch — confidence high*

Servo power leaves on J3 pins 15/16 (Molex Milli-Grid 87832, 2 A/contact = 4 A for the pair) and returns on pins 1/2 through Q8 (PMPB14XN, DFN2020MD-6: 8.1 A continuous, Ptot 1.9 W on a 6 cm2 pad). The ECO sized the eFuse around '4 servos x 2-3 A worst-case stall (~14 A)': that current would be 3.5x the connector pair rating and, at Q8's Vgs of only ~3.0 V (CAM/SERVO/GPS gates are driven through a 1 k series / 10 k pulldown divider off 3.3 V GPIO, so the FETs see 3.0 V, partially enhanced), dissipation exceeds Q8's 1.9 W. With realistic micro-servos (~1 A stall each) everything holds (~4 A pk, <0.5 W) — so the severity hinges on the servo class, but the design should not silently assume the smaller one while its protection is set for 14.7 A. The servo-adapter harness (single LiPoPos / single GND pin on its J1 1x06) has the same issue.

**Evidence:** netlist: J3.15/16=VBATT, J3.1/2=Net-(J3-Pad1)=Q8 drains; Q8 gate divider R27 1k / R29 10k -> Vgs=3.3x10/11=3.0 V. PMPB14XN datasheet: ID 8.1 A (Tamb 25C, 6 cm2), Ptot 1.9 W, RDSon 15-18 mohm at Vgs 4.5 V (higher at 3 V). Molex Milli-Grid 87832 = 2.0 A/contact. servo-adapter.kicad_sch J1 = Conn_01x06 (1 LiPoPos + 1 GND pin). power-eco.md CHANGE 2 current-limit note ('~14 A worst-case').

**Fix:** Decide the real servo budget. If >4 A stall is credible: more J3 contacts for VBATT/return, a larger low-side FET (or parallel pair), and drop the 1k/10k gate dividers to direct 100 ohm drive so Vgs = 3.3 V. If micro-servos are the spec: document it and reduce eFuse ILIM accordingly (previous finding).

**Source:** Nexperia PMPB14XN (fetched); Molex Milli-Grid PS-87831 (rating via distributor listings)

### M7. PG_RAIL (eFuse power-good) and U18 buck PG dead-end at their pull-ups — no MCU can see main-power faults
*architecture / power/telemetry — confidence high*

The ECO pin map says 'PG (13): PG_RAIL -> MCU, main-power-good / fault telemetry'. As built, PG_RAIL contains only U19.13 and R59 (100 k pull-up to +3V3); it reaches no GPIO. Same for the 3V3 buck's PG (Net-(U18-PG): U18.4 + R49 only). Firmware therefore cannot distinguish an eFuse UVLO/retry event from any other reset, cannot log brownout causes (exactly the diagnostic this respin's history begs for), and cannot warn before cutoff. S3 pins GPIO8/9/13/14/17/18 sit unconnected on the same sheet region.

**Evidence:** netlist.net: PG_RAIL [2] = R59.2, U19.13; Net-(U18-PG) [2] = R49.1, U18.4. power-eco.md CHANGE 2 pin table row PG. Unused S3 GPIOs: unconnected-(U15-GPIO8/9/13/14/17/18).

**Fix:** Route PG_RAIL to a spare always-on S3 GPIO (and optionally U18 PG to another); one trace each, no BOM change.

**Source:** TI TPS25982 PG description (fetched)

### M8. Camera (and GNSS/LoRa/servos/pyro-charge) are unpowered on USB — the RunCam-records-on-USB workflow no longer exists
*architecture / power/usb-camera — confidence high*

USB VBUS enters only at TPS2121 IN1 and feeds V_MCU_2S -> +3V3; nothing back-feeds VBATT (eFuse output; TPS2121 blocks OUT->IN2). So on USB-only power the entire VBATT rail is dead: camera J4.2, GNSS FL1, LoRa FL2/J5.2, servo J3.15/16, pyro charge R20. The maintainer's current bench workflow — RunCam records on USB because battery recording browned out — is impossible on V9: recording now REQUIRES the battery. The architecture change is defensible (camera inrush now lands on the pack behind 330 uF C15 + eFuse, isolated from logic by mux reverse-blocking + hold-up, which is the actual brownout fix), but it must be a conscious decision and the battery-record path is bench-unproven. Flag for explicit sign-off and a bench item: record-start inrush on battery with V_MCU_2S scoped.

**Evidence:** netlist: Net-(J6-VBUS) members = J6 VBUS pins, CR3.2, R51/R62 dividers, U21.7 (IN1) only. VBATT [22] sources solely from U19.OUT. J4.2 = VBATT (camera feed). USB-only: logic+sensors+BLE+NAND logging all live (V_MCU_SWTCH derives from +3V3 via U22).

**Fix:** If USB recording matters, add a diode-OR (or second eFuse path) from VBUS to the camera branch, or accept battery-only recording and bench-verify record-start on battery before the first flight. Document the change for the bench workflow.

**Source:** TI TPS2121 (fetched, leakage/RCB specs)

### M9. Pyro arm FET moved to the low-side return: one harness short to ground bypasses the arm interlock
*architecture / pyro — confidence high*

V9 fire topology: V_CAP -> high-side fire P-FETs (U6/U7/U8/U10) -> PYROx_EXT -> e-match -> PYRO_GND -> low-side arm N-FET U9 -> GND (R73 1 k permanently ties PYRO_GND near GND for continuity sensing). The V8/review-era design armed on the HIGH side (V_CAP -> arm FET -> fire FETs), which no wiring fault to ground could bypass. Now, if the shared match-return wire (J2 position 5) or any match negative shorts to airframe/board ground — a classic ejection-charge wiring fault — the arm FET is out of the circuit and a single fire-FET turn-on fires the channel. Boot-glitch safety is still solid (all gates have hardware pulldowns, P4 pins are input-disabled at reset, and the P4 domain is held off by U22 until the S3 acts), and with U9 open a stuck fire FET only leaks ~8 mA through R73 (below all-fire). But the two-independent-switches property against WIRING faults has been lost. C-4's physical-interlock caveat for airstart becomes stronger: for drogue/main this is acceptable-with-eyes-open, and worth reconsidering before fab since the high-side arm variant costs the same parts.

**Evidence:** netlist: U6/U7/U8/U10 SOURCE=V_CAP, DRAIN=PYROx_EXT; PYRO_GND [8] = J2.5_1/5_2, R73.1, U9 drains 5-9; U9 SOURCE=GND, gate = R21 100 ohm from PYRO_ARM (P4 GPIO16) + R22 100 k to GND. Continuity: V_MCU_SWTCH -> R8/R10/R12/R18 49.9 k -> PYROx_EXT, sense ~65 uA through match via R73. schematic-review.md C-4 and section 1 describe the previous high-side arm ('U9 arm -> PYRO_POS').

**Fix:** Either restore the arm switch to the high side of V_CAP (P-FET between V_CAP and the fire-FET sources, keeping R73-based continuity), or explicitly accept the reduced fault coverage in the flight-readiness docs and keep the ban on airstart channels without a physical interlock.

**Source:** TI CSD16323Q3 (selection per project memory); topology from netlist

### M10. ESP32-P4 strapping pins GPIO34/37/38 are routed to the EXP connector J3; attached payloads can corrupt boot mode
*connectors / connectors/EXP (J3) boot straps — confidence high*

EXP_11=GPIO34 (J3.10), EXP_10=GPIO37 (J3.11), EXP_09=GPIO38 (J3.12). Espressif lists GPIO34-38 as P4 strapping pins (GPIO34 selects JTAG signal source at early boot; 34/36/37/38 float by default). The P4 is released from reset (S3 enables V_MCU_SWTCH) with the payload already attached, so any pull/load inside a servo or expansion device on those wires is sampled as a strap. GPIO35 (boot) is properly held by R53 100k to V_MCU_SWTCH with button S3 via R54 1k, and GPIO36 has R50 100k pull-up -- both correct.

**Evidence:** netlist.net: EXP_09/10/11 -> U17.70/69/65 (GPIO38/37/34); Net-(U17A-GPIO35)={R53.1,R54.2,U17.66}, R53.2=V_MCU_SWTCH; Net-(U17A-GPIO36)={R50.1,U17.68}, R50.2=V_MCU_SWTCH. Espressif esptool boot-mode docs for ESP32-P4.

**Fix:** Prefer non-strap GPIOs for the four servo PWM channels in firmware (EXP_01..06 side: GPIO45/44/43/54/39/40); if the connector netting can still change, swap GPIO34/37/38 off J3 for non-strap pins; otherwise document which EXP positions are strap-sensitive.

**Source:** docs.espressif.com esptool ESP32-P4 boot-mode-selection (strapping pins GPIO34-38, GPIO35 weak pull-up, GPIO36 must be high for reliable download)

### M11. Camera port J4 supplies raw 2S pack voltage (6.4-8.4 V); RunCam model input range not verified
*connectors / connectors/camera (J4) — confidence medium*

J4 pin 2 is unswitched VBATT and pin 1 is the Q7-switched return, so the camera sees full pack voltage. This respin's stated goal is fixing the record-start brownout, and architecture-wise the eFuse + ~400 uF VBATT bulk (C7/C14/C18 22u + C15 330u) + logic isolation via TPS2121/C56 hold-up addresses it -- but only if the camera itself accepts 6.4-8.4 V. Wide-input RunCam Split boards take 5-20 V; USB-powered variants (e.g. Thumb) are 5 V-only and would be destroyed. Control pins match bench-confirmed firmware (Camera_RX=GPIO31, Camera_TX=GPIO30, CAM_ACT=GPIO32), and J4's 2 A PH contacts are adequate for ~1-2 A record inrush.

**Evidence:** netlist.net: J4.2=VBATT, J4.1=Net-(J4-Pad1) (Q7 drains), Camera_RX={J4.3,U17.61 GPIO31}, Camera_TX={J4.4,U17.60 GPIO30}, CAM_ACT={R24.2,U17.63 GPIO32}; VBATT bulk C7/C14/C18/C15/C54.

**Fix:** Confirm the deployed RunCam variant's input range covers 8.4 V before fab; if it is a 5 V camera, add a local 5 V buck on the camera branch. Label the port voltage on silk.

**Source:** RunCam model datasheet NOT retrieved (model not identified in repo) -- see checks_blocked

### M12. A JST PH 2-pos battery plug can partially mate into the PH 4-pos camera port J4, bypassing the eFuse or putting pack voltage on P4 UART pins
*connectors / connectors/mis-mate — confidence medium*

J7 (battery) and J4 (camera) are the same JST PH family 15 mm apart on the bottom side; a PHR-2 housing physically seats on any adjacent pin pair of a B4B-PH shroud. On J4 positions 1-2 the pack lands across Q7-drain/VBATT: battery+ backfeeds the VBATT rail directly (bypassing the TPS259824 eFuse via current through board loads returning through Q7's body diode); on positions 3-4 it puts 6.4-8.4 V across Camera_RX/Camera_TX = P4 GPIO31/30 (abs max 3.6 V).

**Evidence:** netlist.net: J4.1=Q7 drain net, J4.2=VBATT, J4.3/4=P4 GPIO31/30; J7=PH-2 battery; pcb: J4 at (83.5,112.1), J7 at (85.4,126.8), both B.Cu.

**Fix:** Use a different family for one of the two (battery to XT30/VH also fixes the current-rating finding), or at least distinct housing colors + silk warning.

**Source:** JST ePH catalog (housing/header interface drawings)

### M13. Pyro return terminal (J2.5) silk-labeled 'GND' although it is the U9-armed switched return; external grounding would bypass the low-side arm interlock
*connectors / pyro/J2 silkscreen — confidence high*

J2 (TBLH11-350-05BK) position 5 carries PYRO_GND, which reaches board GND only through arm FET U9 (CSD16323Q3, drains 5-9 = PYRO_GND, sources 1-3 = GND) with R73 1k bleed. The bottom silk labels the position 'GND'. A user treating it as a general ground (e.g., commoning e-match returns with airframe/battery negative, or probing/strapping it to GND during bench work) shorts out U9 and removes the low-side half of the two-FET firing chain: any single high-side FET fault or boot glitch then fires the channel. Same label class: vertical 'GND' text sits next to J3 pins 1/2, which are the Q8-switched servo return, not ground.

**Evidence:** netlist.net: PYRO_GND = {J2.5_1, J2.5_2, R73.1, U9.5-9}; U9.1-3 = GND; rocket-computer.kicad_pcb B.SilkS texts 'GND 4 3 2 1' row at y~156.5 (GND at x=90.97 maps to position 5) and 'GND' at (74.02,106.99) beside J3.1/2; render_bottom.png confirms.

**Fix:** Relabel J2.5 'RET' or 'PYRO-' and the J3.1/2 area 'SW-GND' or 'RTN'; add a doc note that pyro returns must not be commoned to airframe or battery negative.

**Source:** TI CSD16323Q3 (ti.com/lit/ds/symlink/csd16323q3.pdf): S=1-3, G=4, D=5-8+pad

### M14. Camera, GNSS and servo branches are low-side switched with always-hot VBATT: off-state loads float to battery potential on P4 logic pins
*esp32-p4 / Peripheral power switching / P4 I/O protection — confidence high*

This spin switched the peripheral branches from high-side (V8/ECO description: 'Pack -> Q9 -> servos', 'Pack -> Q3 -> camera') to low-side N-FETs (PMPB14XNX) in each load's GROUND return, while the load's V+ stays permanently on raw VBATT (6.4-8.4 V): camera J4.1, GNSS J1.4, servo/expansion J3.1/2, LoRa J5.1 are switched returns. When a branch is OFF its load's local ground floats up toward VBATT through the load's internals, so every logic line crossing into that domain — Camera_TX/RX (GPIO30/31), GNSS UART (GPIO2/3/4), 12 servo/expansion PWM lines (J3) — can see well above 3.6 V (P4 GPIO abs-max VDD_IO+0.3) and provides a phantom-power/injection path through the P4's (or the load's) ESD clamps. This partially defeats the ECO Change-3 'shed camera/GNSS in flight' intent (an off camera can stay back-fed through its UART) and stresses P4 pins with currents limited only by the off-load's undefined internal impedance. Secondary note: the entire 4-servo return current flows through one 2x2 mm PMPB14XNX (8.1 A ID at Ta, ~14-20 mOhm) — fine for normal loads, marginal against the ECO's own 14 A worst-case stall scenario.

**Evidence:** netlist.net: Q7 drain(1,2,5,6,7)=Net-(J4-Pad1)=J4.1, source(4,8)=GND, gate<-R24<-CAM_ACT/GPIO32, J4.2=VBATT(22-node always-hot eFuse output); identical pattern Q1/J1.4 (FL1<-VBATT on J1.3), Q8/J3.1-2 (J3.15/16=VBATT), Q10/J5.1 (FL2<-VBATT); Camera_RX=U17.61<->J4.3, Camera_TX=U17.60<->J4.4; PMPB14XN datasheet: N-channel, pinning 1,2,5,6,7=D/3=G/4,8=S, ID 8.1 A (Ta).

**Fix:** Prefer reverting camera and GNSS branches to high-side switching (DTC123J + P-FET pattern already used for pyro fire FETs), keeping grounds continuous. If low-side is kept for V9: add >=1 k series resistors in Camera_TX/Camera_RX and the GNSS UART lines, document that servo signals into J3 are only driven when SERVO_ACT is on, and enforce firmware tristate of all cross-branch lines whenever a branch is off (including through panics/reboots).

**Source:** https://assets.nexperia.com/documents/data-sheet/PMPB14XN.pdf

### M15. RF chain and VDD3P3 feed inductors have no MPNs — L4 (2 nH VDD3P3 feed) must be rated >=500 mA per Espressif; generic 0402 'L' entries are unbuildable as specified
*esp32-s3 / S3 RF / VDD3P3 supply — confidence high*

BOM rows L1 (4.3 nH shunt at antenna), L2 (2.7 nH series match), L3 (24 nH XTAL_P series), L4 (2 nH VDD3P3 feed) carry only values and the generic Inductor_SMD:L_0402_1005Metric footprint — no manufacturer or MPN. L4 carries the full radio TX current (BLE TX peak 335 mA @21 dBm, Wi-Fi 340 mA, datasheet Tables 5-7/5-8) and the HDG explicitly asks for the VDD3P3 LC-filter inductor to be rated 500 mA or above; many 0402 RF inductors are 300 mA class and would sag/heat at TX bursts, degrading the exact camera-brownout class of problem this respin targets (different rail, same lesson). L1/L2 and C23/C24 (1.5 pF) are tuning-critical: tolerance class (e.g. 0.1 pF NP0, +/-0.1 nH wirewound) determines whether the bench tune is reproducible. The matching topology itself is correct: CLC pi at LNA_IN (C24 1.5 pF shunt, L2 2.7 nH series, C23 1.5 pF shunt) is inside Espressif's recommended windows (C 1.2-1.8 pF, L 2.0-3.0 nH), and L3 24 nH on XTAL_P is literally the HDG's suggested value — do not remove it.

> **2026-08-01 — removed, then restored; recording why so it is not removed again.**
> The inductor was briefly deleted from all three S3 boards on the reasoning that
> it does almost nothing at 40 MHz: 6.0 Ω against the 331 Ω of the 12 pF load cap,
> worth −1.1 ppm, forming a ~297 MHz low-pass with that cap. Those numbers are
> correct, and they are also the whole point — that low-pass is the function
> Espressif is asking for, and on **these** boards it lands where it matters:
>
> | harmonic | frequency | falls in | attenuation |
> |---|---|---|---|
> | 23rd | **920 MHz** | **the 902–928 MHz LoRa band** | **−18.7 dB** |
> | 61st | 2.44 GHz | the BLE band | −36.5 dB |
>
> A LoRa receiver working near −130 dBm does not need a large on-board spur to
> lose sensitivity, and the 23rd harmonic of the S3's own crystal landing in its
> own receive band is exactly the case the guideline is aimed at. The 1 ppm and
> the 0402 saved by deleting it are not worth that. Restored in `e7b81a8`
> (schematics; PCBs re-placed by hand, since the crystal areas had moved).
> Side effect in the right direction: the +0.055 pF of effective load takes the
> base-station's network from +10.9 to **+9.9 ppm**, inside ±10 (issue #704).


**Evidence:** bom-fresh.csv rows L1-L4: MPN and Manufacturer columns empty. netlist.net: Net-(U15-VDD3P3) = {U15.2, U15.3, C33 100nF, L4.1}; L4.2 = +3V3. HDG: 'The inductor's rated current is preferably 500 mA and above'; RF match 'CLC structure preferred', C 1.2~1.8 pF / L 2.0~3.0 nH; XTAL_P 'inductor of 24 nH' suggestion. Datasheet p66-67: BLE TX 335 mA / Wi-Fi TX 340 mA peak.

**Fix:** Pin MPNs: L4 = multilayer/wirewound 2.0 nH 0402 rated >=500 mA (e.g. Murata LQW15A/LQG15HS class with current check); L1/L2 = +/-0.1 nH RF wirewound (LQW15A); C23/C24 = 1.5 pF +/-0.1 pF C0G; L3 = 24 nH RF 0402. Espressif also recommends 0201 for RF matching — 0402 is acceptable on this board but note the larger parasitics when bench-tuning.

**Source:** ESP32-S3 Hardware Design Guidelines; ESP32-S3 Series Datasheet v2.2 Tables 5-7/5-8

### M16. The board's only fiducial (FID1) sits inside the USB-C connector courtyard - this is the single DRC courtyards_overlap error - leaving zero usable fiducials for a chip-down QFN104/LGA assembly
*layout-decoupling / assembly/fiducials — confidence high*

FID1 (0.5mm fiducial, courtyard r=0.75) at (84.32,168.07) lies ~4mm inside J6's courtyard x[77.91,90.81] y[162.89,171.32]; the USB4110GFA shell covers it, so vision systems lose it as soon as J6 is considered, and the DRC error will recur on every run. There is no other fiducial on either side (233 footprints scanned), on a board carrying a 0.5mm-pitch QFN104 (P4), QFN56 (S3), and three LGA sensors. Physically the overlap is buildable (a fiducial is flush copper; the connector can sit over it) - the harm is assembly-vision quality and permanent DRC noise.

**Evidence:** drc.json courtyards_overlap items: 'Footprint J6' (84.36,164.79) + 'Footprint FID1' (84.32,168.07). J6 F.CrtYd extents computed from fp_line geometry. Footprint inventory: FID1 is the only Fiducial:* footprint.

**Fix:** Move FID1 clear of J6 and add two more global fiducials (diagonal corners, both sides ideally) before fab.

### M17. LoRa switched-power pour runs directly beneath the IIS2MDC magnetometer; dynamic (uncalibratable) field source, though static hard-iron environment looks no worse than the old board
*layout-decoupling / sensors/magnetometer — confidence medium*

U3 (mag) is at (84.03,99.85) F.Cu. The B.Cu pour of Net-(J5-Pin_1) - the Q10-switched VBATT feed to the LoRa daughterboard on J5 - has its filled-copper boundary passing 0.03mm from the point directly beneath the sensor center (~1.6mm below through the board), so E220 TX burst current (hundreds of mA class) flows essentially under the package. FL2 (LoRa VBATT ferrite) is 2.6mm and Q10 2.7mm from the mag; a 0.3mm VBATT trace passes at 3.3mm. A few hundred mA at ~2mm is tens of uT - same order as Earth's 50uT field - and it is current-dependent, so hard-iron calibration cannot remove it; LoRa TX and pad-phase heading acquisition overlap operationally (telemetry runs on the pad, and pad heading seeds guidance per project memory). Mitigating factors: servo current stays in the In2 VBATT plane whose fill keeps >=6mm from the mag; buck inductor L8 is 8.6mm and shielded; the magnet-bearing buzzer LS1 is 57mm away; nearest ferromagnetic hardware is connector fittings at 3-6mm and crystal cans at 11mm. Versus the previous layout's 1.7mT hard-iron: no comparably large magnetized aggressor is adjacent, so the static situation is likely better; the under-sensor LoRa feed is a new dynamic regression risk.

**Evidence:** Point-in-polygon on zone Net-(J5-Pin_1) B.Cu filled_polygon: nearest fill vertex 0.03mm from (84.03,99.85); zone bbox x[82.5,87.9] y[98.6,101.0]. FL2 (86.45,100.86) d=2.6mm; Q10 (86.53,98.80) d=2.71mm; VBATT B.Cu seg d=3.30mm w=0.3mm; VBATT In2 fill vertices all >6mm; L8 (92.03,103.07) d=8.62mm; LS1 d=57mm. netlist: J5.1=Net-(J5-Pin_1) via Q10, J5.2=FL2->VBATT, J5.3/4=LoRa_RX/TX.

**Fix:** Pull the Net-(J5-Pin_1) pour and the FL2/VBATT feed out from under U3 (route the LoRa feed along the board edge east of x87) and add a small all-layer high-current keepout under the mag; keep the >=6mm standoff the In2 servo plane already has.

**Source:** ST IIS2MDC datasheet (iis2mdc.txt) - decoupling verified separately; field estimate is first-principles (u0*I/2*pi*d)

### M18. U9 schematic pin 9 (PYRO_GND) has no footprint pad; 'TSON Advance_TOS' footprint has two pads numbered 5
*layout-misc / library/parity — confidence high*

The DRC schematic-parity warning is real: the CSD16323Q3 symbol on U9 has drain pins 5,6,7,8,9 all on PYRO_GND, but the shared 'TSON Advance_TOS' footprint numbers the 2.5x2.5 mm thermal/drain pad '5' (duplicating the perimeter pad 5) and has no pad '9'. Electrically benign HERE because pin 9's net is also on pads 5-8, but it permanently produces a parity warning on the arm-FET, which is exactly the noise that can mask a genuinely unconnected pin next revision (the LoRa-board missing-ground class). U6/U7/U8/U10 use the same footprint with the Toshiba TPN4R712MD symbol and do not warn, so only the U9 symbol/footprint pairing is misaligned.

**Evidence:** drc.json schematic_parity: 'No pad found for pin 9 (PYRO_GND) in schematic' on U9 (92.64,165.28). fp_detail.py U9: pad '5' smd 2.500x2.500 net=PYRO_GND AND pad '5' smd 0.381x0.660 net=PYRO_GND both present; pads 1-3=GND (source), 4=GATE. netlist.net: PYRO_GND nodes = J2.5_1 J2.5_2 R73.1 U9.5 U9.6 U9.7 U9.8 U9.9.

**Fix:** In Footprints.pretty/'TSON Advance_TOS.kicad_mod' (or a U9-specific copy), renumber the thermal pad to '9' (keeping perimeter 5-8), matching both the TI and Toshiba 9-pin symbols; re-run DRC to confirm parity is clean.

**Source:** csd16323q3.pdf (local scratchpad copy)

### M19. U11 NAND (WSON-8) EPAD gets 100% paste — pad-level aperture defeats its 6 segmentation windows
*layout-misc / paste/stencil — confidence high*

U11 (GD5F2GQ5UEYIGR, 8x6 mm WSON, top side) pad 9 (3.45x4.34 mm GND EPAD) includes F.Paste in its pad layer list, giving a full-size 15 mm2 paste aperture, while the footprint also draws 6 smaller F.Paste windows intended as segmentation — the union is still 100%. Every other big EPAD on the board is properly segmented (U17 P4: 9 windows/40%; U15 S3: 9 windows/54%; U18 buck: 40%; U23 INA230: 40%; U19 eFuse: 79%). 100% paste plus 3 unplugged vias in this pad risks part float/tilt and opens on the perimeter leads of the logging flash.

**Evidence:** poly_pad_align.py: 'U11 pad 9 3.45x4.34: paste = full pad (pad layer)' plus mask_paste_gfx.py showing 6 F.Paste fp_polys on U11; viainpad: 'U11 pad 9 net=GND: 3 vias in pad'. Contrast U17 pad 105: layers [F.Cu,F.Mask] only, 9 poly windows totalling 22.47 mm2 = 40%.

**Fix:** Remove F.Paste from U11 pad 9's layer list so the 6 drawn windows define the stencil (~40-60% coverage), matching the other EPADs.

### M20. Unplugged via-in-pad in paste-printed pads — worst in the P4's only ground connection and the eFuse current path
*layout-misc / paste/vias — confidence high*

Board setup: tenting front/back=yes, plugging/capping/filling=no. Vias inside mask-opened, paste-printed pads are open barrels that wick solder during reflow: U17 EPAD 6 vias (the ESP32-P4's ONLY ground connection — no perimeter GND pins exist on the QFN-104, so EPAD joint quality is boot-critical), U15 EPAD 4, U19 eFuse IN/OUT thermal pads 5+3 (the 15 A battery path), U9 drain 4, U11 3, U23 1, plus assorted connector/inductor pads (L5:4, L6:3, J3/J4/J7 power pads). The 40-54% EPAD paste coverage already compensates partially, but wicking subtracts from it unpredictably.

**Evidence:** rocket-computer.kicad_pcb setup: (tenting (front yes)(back yes)) (plugging (front no)(back no)) (capping no) (filling no). mfg_audit.py viainpad full list, e.g. 'U17 pad 105 net=GND: 6 vias in pad', 'U19 pad 25 net=VBAT_CON: 5 vias in pad'.

**Fix:** Order the fab option 'epoxy-plugged and capped' (IPC-4761 Type VII) for vias in the U17/U15/U19 pads at minimum, or accept standard voiding knowingly; alternatively move the U19 vias just outside the paste windows.

### M21. C15 tantalum polarity band silk is clipped at the board edge; its courtyard overhangs the outline
*layout-misc / silk/polarity — confidence high*

C15 (TCJE337M016R0050 330 uF polymer tant, VBATT hold-up, bottom side at 92.39,113.59 rot -90) generates 3 of the 5 silk_edge_clearance warnings: its silk extends to x=94.667 vs board edge 94.71, so the fab will clip the outline/polarity-band segments on the edge side; the '+' pin-1 end is at y~110.9 and the clipped segments at y=109.665 are on that end. Its courtyard reaches x=94.79, 0.08 mm past the outline. The pattern's polarity band will only partially print. C56 (same part, top, V_MCU_2S) is fine.

**Evidence:** drc.json silk_edge_clearance items at (93.972,109.665), (94.667,109.665), (94.667,117.519) all 'Segment of C15 on B.Silkscreen'. fp_detail.py C15: courtyard rel +-2.40 across => global X 89.99..94.79 vs edge X 94.71; pad 1 (VBATT,+) at (92.39,110.92).

**Fix:** Nudge C15 ~0.3 mm inboard (-x) or redraw its polarity band inside the outline so the '+' end marking prints completely.

### M22. Zero testpoints; no UART0 fallback — USB chain is the single programming/console path for both MCUs
*layout-misc / test/bring-up — confidence high*

There are no TP footprints anywhere (footprint inventory complete). Power rails are probe-able on large passives (VBATT: C15 tant/J3.15-16/J4.2; V_MCU_2S: C56 tant; +3V3: C65; V_MCU_SWTCH: LS1.LOAD+ 2.4 mm pad; V_CAP: C12 through-hole leads; VBAT_CON: R72 0805 shunt; PYRO_GND/PYROx_EXT: J2 screw terminals), so bring-up voltage checks are workable. The real gap is recovery/console: S3 UART0 pads 49/50 (U0TXD/U0RXD) are UNCONNECTED nets, and both MCUs are programmed solely via J6 USB-C -> CR3 ESD -> U1 FSUSB63 mux -> S1 (SEL0) -> CEN_D+/- (P4) or GPIO19/20 (S3). On a chip-down board where assembly defects are first-tier hypotheses, any defect in that 4-part chain leaves no ROM-loader or console access to either MCU — an unrecoverable board with no way to distinguish 'MCU dead' from 'USB path dead'.

**Evidence:** netlist.net: 'unconnected-(U15-U0TXD/GPIO43-Pad49)', 'unconnected-(U15-U0RXD/GPIO44-Pad50)'; SEL0 = R2.1/S1.6/U1.4; CEN_D+ = U1.8/U17.53; OUT_D+ = R38.1/U1.10. fid_tp scan: no TP* refs, no TestPoint footprints.

**Fix:** Add small testpoints (even 0.8 mm bare-copper dots) for S3 U0TXD/U0RXD, P4 UART0, both CHIP_PU nets, and a labeled GND — six pads on a 22x75 mm board with a 6-layer stack is cheap insurance for chip-down bring-up.

### M23. Camera VBATT branch necks to 0.7 mm on inner layer In2 with 2 vias - at capacity for sustained record current
*layout-power / camera-feed — confidence high*

The J4.2 camera feed leaves the In2 VBATT zone through a 0.70 mm single-path neck at (84.44,114.28) and drops to the B.Cu pad through 2x 0.3 mm vias at (84.88,115.91)/(84.26,115.90). At the 1-2 A record-start inrush this is survivable (brief), and sustained ~1 A record current sits right at the ~1 A/10 C-rise capability of 0.7 mm inner copper. Given the RunCam record-start brownout is an explicit goal of this respin, the branch should not be the marginal element. The architectural fix IS in place and verified: camera is on raw VBATT (J4.2), isolated from logic by the U21 mux + C56 330 uF hold-up, with C15 330 uF + C7/C14/C18 22 uF bulk on VBATT - the old shared-rail brownout coupling is gone.

**Evidence:** widest_path U19.20->J4.2: 0.70 mm on In2.Cu at (84.44,114.28); 2 VBATT vias near J4.2; C15 pads (92.39,110.92) on VBATT 8.6 mm from J4.2; ECO power-eco.md Change 1/4 cross-checked against netlist (C56 on V_MCU_2S at U21 OUT confirmed)

**Fix:** Widen the In2 corridor at the J4 tap to >=1.5 mm and add 2 more vias at the pad; optionally move a 22 uF (or restore the ECO's 100 uF) directly at J4.2.

**Source:** IPC-2152 approximation as stated

### M24. INA230 IN+ is not Kelvin-connected: it taps the J7 battery pad, adding ~1-1.5 mOhm of zone copper in series with the 2 mOhm shunt
*layout-power / current-sense — confidence high*

IN- (U23.12) correctly taps R72 pad 1 dead-center via a dedicated 0.1 mm trace ending at (80.04,129.45). But the IN+ (U23.13) sense trace runs on F.Cu to a via at (84.41,129.56) that lands inside J7 pad 2 (battery + terminal), not at R72 pad 2 (80.04,131.28). The full battery current flows through the ~5.5 mm long, ~2 mm wide B.Cu VBAT_Terminal zone between those two points (~1-1.5 mOhm), so the measured 'shunt' is ~3-3.5 mOhm instead of 2 mOhm: pack-current telemetry reads ~+50-75% high, with copper tempco (+0.4%/C) on the error term. Voltage (BUS) readings are unaffected.

**Evidence:** VBAT_Terminal segments: F.Cu 0.1 mm (84.41,129.56)->(84.04,129.19)->...->(79.70,127.83) -> B.Cu -> U23.13 at (79.2,127.77); via at (84.41,129.56) inside J7.2 pad (84.44,129.06, 1.0x5.5 mm); VBAT_Terminal B.Cu zone bbox (79.3-85.0, 129.2-132.4), 11.6 mm2

**Fix:** Move the IN+ tap so the 0.1 mm sense trace originates at R72 pad 2 (the pad end facing away from the current entry), mirroring the IN- connection.

**Source:** TI INA230 (Kelvin connection guidance in layout section)

### M25. R72 shunt (2 mOhm, 0805) has no MPN in the BOM - a generic 0.125 W 0805 would overheat at servo stall
*layout-power / current-sense — confidence high*

At the spec'd 12-14 A worst case, P = I2R = 0.29-0.39 W (plus tolerance), and ~0.45 W with camera load added. Standard 0805 thick-film resistors are 0.125 W; only metal-element current-sense parts (e.g. Vishay WSLP0805, Susumu KRL1220E: 0.5-1 W) survive. bom-fresh.csv row 'R72, 2 m, R_0805' has empty MPN/Manufacturer, so assembly could legitimately fit a 0.125 W part. Also note the 1.0-1.1 mm copper necks at both R72 pads are the narrowest points of the whole battery path (~3 A at 10 C rise each) - widen when the connector chain is fixed.

**Evidence:** bom-fresh.csv line 53: "R72","2 m","Resistor_SMD:R_0805_2012Metric","","","1",""; widest_path J7.2->R72.2 bottleneck 1.10 mm at (80.25,131.22) B.Cu; R72.1->U19 IN corridor avg 2.3 mm (13.5 mm2 over 5.8 mm)

**Fix:** Pin a metal-strip shunt MPN rated >=0.5 W (e.g. WSLP0805R0020FEA or KRL1220E-M-R002) in the BOM; widen the pad-entry copper to the full pad width on both sides.

**Source:** Vendor unverifiable - MPN absent from BOM (flagged for that reason)

### M26. eFuse ILIM (~14.7 A) is above what the downstream connectors and copper can carry - the weakest links are unprotected
*layout-power / protection-coordination — confidence medium*

U19 (TPS259824LN) ILIM is set by R48 = 100 Ohm -> ~14.7 A typ / ~12.9 A cold-min (matches the ECO). But the protected domain now contains 2 A-rated PH contacts (J7 is upstream so unprotected anyway), 2 A-rated Milli-Grid contacts (J3), a 2.5 mm inner-copper corridor and an 11.5 A-max return FET (Q8). A sustained 8-12 A overload (e.g. 3-servo stall, partial short on the servo cable) sits below the trip point indefinitely while cooking the corridor, J3 and Q8. ITIMER C50 = 10 nF (ECO recorded 4.3 nF -> ~2 ms; 10 nF gives roughly twice that blanking) so even limit-touching events ride longer than recorded.

**Evidence:** netlist: U19.8 ILIM -> R48 (BOM: 100 Ohm); C50 10 nF on ITIMER (bom-fresh.csv line 12 group); TPS25982 datasheet ILIM equation/section; copper capacities measured above

**Fix:** After upsizing the servo chain, keep 14.7 A; if the copper/connector chain stays as-is, drop ILIM to ~6-8 A (R48 up) so the eFuse actually protects the real weakest links, and re-verify servo inrush rides ITIMER blanking.

**Source:** TI TPS25982 datasheet (local text extract)

### M27. Inter-MCU I2C (ESP_SDA/ESP_SCL) runs 27 mm along the board edge passing 0.3-0.5 mm from the antenna body, outboard of it, with no ground between
*layout-rf / RF / antenna (U14 Molex 47948-0001) — confidence high*

ESP_SDA (x=72.81) and ESP_SCL (x=72.61) run as unbroken 26.9/27.4 mm F.Cu traces from y~120 to y~147 along the left board edge (edge at x=72.36). The antenna body occupies x 73.14-76.14, y 133.02-136.02 on the same layer, so both I2C lines pass laterally 0.28-0.53 mm from the radiator base, BETWEEN the antenna and the board edge where no ground pour fits. Espressif S3 layout guide: 'There should be no high-frequency signal traces routed close to the RF trace' and UART/USB lines 'must be as far away from the antenna as possible'. BLE TX (up to ~+20 dBm at 2.44 GHz) will couple common-mode RF into the inter-MCU I2C bus feeding both MCUs, and the traces sit inside the antenna near-field (potential detune + TX desense of nearby runs). I2C edges themselves are slow, so the dominant risk is RF immunity/desense rather than emissions.

**Evidence:** rocket-computer.kicad_pcb segments: ESP_SDA (72.81,120.17)->(72.81,147.09) L=26.92 mm, ESP_SCL (72.61,119.76)->(72.61,147.17) L=27.41 mm, both w=0.1 F.Cu; U14 at (74.64,134.52), body 3.0x3.0 mm (Molex product literature, local molex-47948.pdf).

**Fix:** Reroute ESP_SDA/SCL inboard of the antenna (e.g., drop to In3 for the y=128-142 stretch, where In1 GND shields them from the antenna), or at minimum move them east of the antenna with a stitched GND guard. Cheap re-route; no BOM change.

**Source:** Espressif ESP Hardware Design Guidelines (ESP32-S3) PCB Layout Design page; Molex 47948 product literature

### M28. INA230 IN+ is not Kelvin-connected to the 2 mOhm shunt — pack-current telemetry reads ~15-20% high
*layout-rf / current-sense (INA230) — confidence high*

U23 (INA230) IN- (pin 12, net VBAT_CON) correctly Kelvin-taps shunt pad R72.1 via a dedicated 0.1 mm trace (79.93,127.05)->(80.05,129.44), ~2.5 mm. IN+ (pin 13, net VBAT_Terminal) instead taps at the battery connector pad J7.2 (84.44,129.06) and reaches U23.13 by its own 0.1 mm sense trace — but shunt pad R72.2 (80.04,131.28) is joined to J7.2 ONLY through the B.Cu VBAT_Terminal zone pour (12 mm2). All pack current therefore flows through ~2.3 mm of 35 um pour between the IN+ tap point (J7.2) and R72.2, putting an estimated ~0.3-0.4 mOhm of copper (temperature-dependent, +0.39%/K) inside the measured path in series with the 2 mOhm shunt. Current telemetry will read high by roughly 15-20% and drift with temperature. TI INA230 datasheet section 8.4.1 requires: 'Connect the input pins (IN+ and IN-) to the sensing resistor using a Kelvin connection or a 4-wire connection.'

**Evidence:** netlist.net: VBAT_Terminal = J7.2 + R72.2 + U23.13; rocket-computer.kicad_pcb netpath: VBAT_Terminal segments total 5.74 mm F.Cu + 0.52 mm B.Cu form only the J7.2->U23.13 path (no segment touches R72.2 at (80.04,131.28)); B.Cu zone net=VBAT_Terminal 12 mm2 is the sole J7.2->R72.2 connection. IN- trace: (79.92,127.05)->(80.05,127.17)->(80.05,129.44)->R72.1 (80.04,129.45).

**Fix:** Route a dedicated 0.1 mm IN+ sense trace from U23.13 directly to the R72.2 pad center (paired alongside the existing IN- trace), and keep the J7.2->R72.2 load current in the pour only. 2-minute edit, no BOM change.

**Source:** TI INA230 datasheet section 8.4.1 Layout Guidelines (local extract ina230.txt line 1827)

### M29. In3 routing ribbon bundles pyro FIRE/CONT lines 0.10 mm edge-gap from flight-active SPI clock and USB pair for up to 39 mm
*layout-rf / pyro / inter-MCU routing (In3) — confidence high*

The left-side In3.Cu ribbon carries pyro gate-drive and continuity-sense lines directly adjacent to continuously-switching buses. Measured same-layer parallel runs (edge gap, overlap): ESP_SCLK || PYRO1_FIRE 0.10 mm for ~6.1 mm (x=73.41/73.46, y 125.5-136.8 — passing under the antenna region); ESP_SDO || PYRO1_CONT 0.10 mm for ~4.2 mm; PYRO4_CONT || CEN_D+ 0.10 mm for ~39.3 mm; PYRO3_CONT || CEN_D+ 0.30 mm for ~39.3 mm; M_MISO || PYRO2_FIRE 0.29 mm for ~8.5 mm; PYRO2_EXT || CEN_D+ 0.10 mm for ~8.0 mm on F.Cu. PYRO1_FIRE runs 42.5 mm on In3 from U17.6 (78.66,109.13) to its gate transistor Q5 (74.32,143.83). Spurious firing from this coupling alone is NOT credible — coupled ns-scale glitches (~100-200 mV est.) are far below the DTC123J ~0.6 V sustained threshold and the two-stage arm (PYRO_ARM + U9 PYRO_GND low-side) still gates the energy path — but it erodes the hardware no-fire margin the respin is supposed to guarantee, and USB Full-Speed traffic will inject noise into the PYRO*_CONT ADC continuity readings during every bench session with USB attached. All traces are 0.1 mm wide with In2 plane 0.1 mm above and In4 GND 0.535 mm below (vertical shielding good; lateral coupling is the issue).

**Evidence:** rf_geom.py parallel sweep on rocket-computer.kicad_pcb (worktree copy, matches TinkerRocket-Hardware clone): full pair list with min_edge_gap/runlen printed above; PYRO1_FIRE netpath totals {F.Cu 0.63, B.Cu 2.12, In3.Cu 42.52} mm.

**Fix:** On In3, reorder the ribbon so PYRO*_FIRE and PYRO*_CONT never neighbor ESP_SCLK/ESP_SDO/CEN_D+/- ; enforce >=0.3 mm (3W) spacing to any clock/USB line, or move FIRE/CONT to their own lane with a GND trace between groups. Also separate PYRO2_EXT from CEN_D+ on F.Cu near the pyro connector.

### M30. Reverse battery connection destroys CR2, INA230 and likely the eFuse; only the connector keying prevents it
*power-input / Battery input — confidence high*

There is no series reverse protection. A reversed pack forward-biases CR2 (CUS10S30: IO 1.0 A avg, IFSM 5 A/10 ms) into a dead short across the pack through R72 (2 mOhm) — tens of amps until the diode fails, with wiring-heating risk. During/after that, VBAT_CON and VBAT_Terminal sit far below ground: INA230 IN+/IN-/BUS abs max is GND-0.3 V (with 5 mA/pin limit) and the TPS25982 IN abs max is -0.3 V — both violated. Downstream stays protected (eFuse body diode blocks with negative input). The JST PH shell is polarized, so this requires a miswired pack lead — but that is exactly the home-built-pack failure mode. Deliberate scope note: this does NOT re-open the settled battery-protector-IC question; a simple series P-FET or fuse at J7 is orthogonal to that decision.

**Evidence:** netlist.net: J7.2 -> R72 -> VBAT_CON -> U19 IN + U23 IN-/BUS, U23.13 -> VBAT_Terminal; CR2 pin1(cathode)=VBAT_CON pin2(anode)=GND per Toshiba CUS10S30 datasheet (1:Cathode 2:Anode; VR 20 V, IO 1.0 A, IFSM 5 A/10 ms); TI INA230 abs max common-mode GND-0.3 V; TPS25982 abs max VIN -0.3 V.

**Fix:** Accept-with-eyes-open (keyed connector) and note it in the ECO, or add a reverse-blocking P-FET/ideal-diode or fuse+diode at J7. At minimum, add a fat polarity silk legend at J7.

**Source:** Toshiba CUS10S30 Rev2.0.A; TI INA230 SBOS601B; TI TPS25982 SLVSEI3D

### M31. Half-Kelvin shunt sense: IN+ taps the battery connector pad, not the shunt pad
*power-input / INA230 current sense — confidence high*

U23 IN- (pin 12) Kelvins correctly to R72 pad 1 with a dedicated 0.1 mm trace (80.05,129.44 -> 79.925,127.045). But the IN+ (pin 13) sense trace starts at a via at (84.41,129.56) inside J7 pad 2 (84.44,129.06) and runs on F.Cu to the chip — placing ~4.5 mm of the shared high-current B.Cu pour between J7.2 and R72.2 (80.04,131.275) in series with the 2 mOhm shunt measurement. At ~0.3-1 mOhm of pour/spreading resistance that is a +15..50% gain error on every current reading, load-pattern dependent, uncalibratable.

**Evidence:** rocket-computer.kicad_pcb: VBAT_Terminal segments (84.41,129.56)->(84.039,129.189)->(83.21,127.83)->(79.7,127.83)->U23.13 (79.200,127.770); R72 pad 2 at (80.040,131.275) has no sense trace; R72 pad 1 sense trace lands directly on the pad.

**Fix:** Re-route the IN+ sense as a thin trace tapping R72 pad 2 itself (mirror of the IN- tap), ideally as a symmetric Kelvin pair under the shunt.

**Source:** TI INA230 SBOS601B (layout guidance: kelvin connection at shunt)

### M32. PG_RAIL goes nowhere — the ECO's claimed main-power-good/fault telemetry is not implemented
*power-input / eFuse (U19) telemetry — confidence high*

U19 PG (pin 13, open-drain, asserted when the FET is enhanced) is pulled up by R59 100k to +3V3, but the PG_RAIL net has exactly two nodes (U19.13, R59.2) — no MCU pin. power-eco.md Change 2 pin map says 'PG (13): R27 100 k -> +3V3; PG_RAIL -> MCU | main-power-good / fault telemetry'; that second half never happened, so firmware cannot see eFuse retry/fault events (e.g. distinguishing an eFuse auto-retry brownout from a pack sag in flight logs). Similarly U18's PG dead-ends in R49 100k to its own output (+3V3), which is definitionally useless as a fault flag. Both are IMPLEMENTED-claim mismatches, not electrical faults.

**Evidence:** netlist.net: NET 'PG_RAIL' (2 nodes): R59.2, U19.13; R59.1 -> +3V3. NET 'Net-(U18-PG)' (2 nodes): R49.1, U18.4; R49.2 -> +3V3. power-eco.md line: 'PG (13) | R27 100 k -> +3V3; PG_RAIL -> MCU | main-power-good / fault telemetry'.

**Fix:** Route PG_RAIL to a spare S3 GPIO (the S3 is the always-on supervisor and has the power I2C already: GPIO21/33) so eFuse events are loggable, or amend power-eco.md to record that PG is pull-up-only.

**Source:** TI TPS25982 SLVSEI3D (PG pin description)

### M33. Pyro cap charges to ~2.5-2.7 V on USB-only power via fire-FET body diodes — 'no pyro energy on USB' assumption is false
*pyro / pyro energy store — confidence high*

Intended charge path is battery-only: R20 (150R) hangs on VBATT = eFuse U19 OUT, fed only from the pack (J7 → R72 shunt → VBAT_CON → U19); USB VBUS goes to TPS2121 IN1 (U21 pin 7) and cannot reach VBATT. BUT with V_CAP at 0 V and the P4 domain powered (which USB does power), each continuity pull-up R8/R10/R12/R18 (49.9k to V_MCU_SWTCH 3.3 V) forward-biases its TPN4R712MD body diode (drain→source when drain > source on a P-FET) into the V_CAP bank: 4 × ~52 µA charges the 10 mF + 4×22 µF bank with τ ≈ 12.5k×10 mF ≈ 125 s toward ≈ 3.3 − V_F ≈ 2.5-2.7 V. That stores ~30-35 mJ deliverable at ~2.4 A initial into a 1 Ω match — above typical e-match all-fire current. So on a USB bench session with matches connected, the energy side is NOT inert; only the control-side faults (arm+fire, see the WPU finding) separate this from ignition.

**Evidence:** netlist.net: R20.2=VBATT (22 nodes, all eFuse OUT loads); U21 IN1=Net-(J6-VBUS), IN2=VBATT; R8/R10/R12/R18 pin2 = V_MCU_SWTCH, pin1 = PYROx_EXT = TPN drains. Toshiba TPN4R712MD datasheet: body diode V_DSF ≤1.2 V at -36 A (≈0.4-0.6 V at µA).

**Fix:** Either add a bleed on V_CAP sized to dominate the ~210 µA sneak current (e.g., 2.2-4.7k keeps V_CAP < ~0.6 V on USB at the cost of ~2-4 mA from VBATT when on battery), or re-reference the continuity pull-ups R8/R10/R12/R18 to V_CAP instead of V_MCU_SWTCH (kills the cross-charge path entirely; continuity math still works since PYROx_CONT is already clamped to V_MCU_SWTCH by D1-D4), or explicitly document/accept that USB does energize the pyro store to ~2.6 V.

**Source:** https://toshiba.semicon-storage.com/info/TPN4R712MD_datasheet_en_20250613.pdf

### M34. V_CAP ceramic bank (C9/C10/C11/C13, '22 uF' 0805) has no MPN/voltage rating anywhere — 0805 22 µF parts are commonly 6.3-10 V, below the 8.4 V rail
*pyro / pyro energy store — confidence high*

The four 22 µF 0805 ceramics paralleling C12 on V_CAP carry only the value '22 uF' in the schematic and an empty MPN column in bom-fresh.csv. If a 6.3 V or 10 V part is bought, it is overstressed (or DC-bias-derated to near-zero capacitance) at V_CAP = 8.4 V. Even a 16 V X5R 0805 22 µF will deliver only roughly 30-50% of nominal C at 8.4 V bias; that mostly matters for ESR/peak-current sharing at fire time, which the 10 mF electrolytic dominates anyway (KYC ESR ≤30 mΩ), so the electrical role survives — the voltage rating is the real risk. The same unspecified '22 uF' value is reused on other VBATT-domain nodes (C7, C14, C18, C43).

**Evidence:** bom-fresh.csv rows for C9-C11/C13 (blank MPN fields); external_connections.kicad_sch symbol properties Value='22 uF', no voltage/MPN field; netlist.net V_CAP node list.

**Fix:** Pin the MPN to a ≥16 V (prefer 25 V) X5R/X7R 0805 22 µF in the schematic Value or an MPN field and the BOM before ordering.

### M35. C12 polarity marking is a single ambiguous silk ring on the NEGATIVE pad — no + or − text; reversal puts a 10 mF electrolytic backwards across the pyro rail
*pyro / pyro energy store / assembly — confidence high*

The only polarity indication on the C12 footprint is a 1.5 mm-radius B.SilkS circle centered on pad 2 (GND/negative, global ≈ (84.91,148.12)); pad 1 = V_CAP (positive). There is no '+'/'−' text, no negative-stripe bar, and no board-level marking near the part (nearest gr_text '+' is at (94.38,100.36), 45 mm away, belonging to another connector). A ring is read by many assemblers as a pin-1 (i.e., positive) marker — the opposite of what is intended here. A reversed 16 V electrolytic held at 8.4 V will degrade/vent; this cap is the pyro deployment energy store. Correct polarity for reference: netlist C12 pin 1 = V_CAP (+), pin 2 = GND (−); with the -90° footprint rotation, pad 1 (+) is the pad at global (84.90,140.60) (toward the battery connector) and pad 2 (−) at (84.90,148.10) (toward the J2 pyro terminal).

**Evidence:** rocket-computer.kicad_pcb C12 footprint: single fp_circle B.SilkS center (3.78,-0.02) r=1.50 = pad-2 location; no other silk graphics or text. netlist.net: C12 pin1=V_CAP, pin2=GND.

**Fix:** Add explicit '+' silk adjacent to pad 1 and a filled negative-stripe bar at pad 2, placed OUTSIDE the Ø18 can outline so they remain visible after mounting; 16 V rating itself is fine (8.4 V = 53% derating).

**Source:** https://www.chemi-con.co.jp/en/products/detail-condenser.php?part_number=EKYC160ELL103MM25S

### M36. GNSS daughterboard is ground-switched (low-side Q1) while its UART pins cross to the always-driven P4 domain
*sensors / sensors/GNSS branch (in_sensors.kicad_sch) — confidence high*

J1 (BM05B-SRSS-TB, GNSS) gets permanent VBATT on pin 3 via FL1; its return (pin 4) is switched to GND by N-FET Q1 (PMPB14XNX), gate=GPS_ACT (P4 GPIO15) with R6 1k series and R7 10k pulldown (default OFF - good). But GNSS_RX/GNSS_TX/GNSS_RXD2 (J1.1/2/5) run directly to P4 GPIO4/3/2 with no series resistance. With Q1 off, the module's local ground floats toward VBATT (6.4-8.4 V) through its own load while VSYS stays hard-connected, so current is driven from the floating module through its UART ESD structures into the P4 pins (and vice versa when P4 TX idles high). Consequences: GNSS branch never truly powers off (defeats GPS_ACT shedding per ECO Change 3), sustained clamp current through P4 and module I/O, and possible module misbehavior/latch-up. This may also be a candidate mechanism for the open 'new GNSS board deaf UART' bench mystery. The daughterboard connector pinout itself matches (gnss-sam10m8 J3: 1=GNSS_RX, 2=GNSS_TX, 3=VSYS, 4=VSS, 5=RXD2). NOTE for orchestrator: the same low-side pattern is used on J5/Q10 (LoRa), J4/Q7 and J3/Q8 (other sheets) - systemic, should be cross-checked by the agents owning those sheets.

**Evidence:** netlist.net: NET VBATT includes FL1.1; Net-(FL1-Pad2)=FL1.2+J1.3; Net-(J1-Pad4)=J1.4+Q1.1/2/5/6/7; GND includes Q1.4/Q1.8; Net-(Q1-Pad3)=Q1.3+R6.1+R7.1; GPS_ACT=R6.2+U17.16[GPIO15]; GNSS_RX=J1.1+U17.4[GPIO4], GNSS_TX=J1.2+U17.3[GPIO3], GNSS_RXD2=J1.5+U17.2[GPIO2]. Daughterboard: gnss-sam10m8-18mm-hv.kicad_pcb J3 pads 1-5 = GNSS_RX/GNSS_TX/VSYS/VSS/RXD2.

**Fix:** Switch the GNSS branch high-side (P-FET on the FL1/VSYS feed, gate driven via level-shift NPN, keep default-off), tying daughterboard VSS permanently to GND. If the low-side switch must stay for this spin, add ~1k series resistors in GNSS_RX/TX/RXD2 to bound cross-domain clamp current, and accept that the module is only pseudo-off.

### M37. Vias and top-layer traces routed under the BMP585 body, against Bosch landing-pattern guidance
*sensors / sensors/baro U4 (BMP585) — confidence high*

Bosch: 'We do not recommend vias or traces under the BMP585. Furthermore, it is recommended that there is no solder mask under the sensor... If the solder mask or other material underneath the sensor gets in contact with the sensor, there may be a negative impact on performance.' The BMP585 relies on >=50 um solder standoff for mechanical decoupling; copper+mask bumps under the body can contact the package and induce pressure offset/stress noise. U4 body (3.25x3.25 mm centered at 87.79,99.92 F.Cu) has 3 vias inside/at its outline: (88.69,100.27) GND near body center, (87.97,101.52) and (88.07,98.35) V_MCU_SWTCH at the body edge; F.Cu segments of SENS_SDO (85.76,100.21)-(86.57,101.02), SENS_SDI (86.57,99.92)-(86.11,99.46), SENS_SCLK (86.57,98.82)-(86.57,97.93), BMP585_INT (88.08,101.95)-(89.01,101.02) and BMP585_CS (~89.0-89.1, 98.7-98.8) cross under the body. The baro is the primary apogee sensor, so offset/noise here is flight-relevant.

**Evidence:** rocket-computer.kicad_pcb via/segment elements listed above (extracted by paren-matching parser); BMP585 datasheet BST-BMP585-DS003-02 sec. 9.3.2/9.3.3 (p.67): no vias/traces under sensor, no solder mask under sensor, min 50 um solder height.

**Fix:** Reroute the escape traces of U4's own pads on B.Cu/inner layers (short stubs out of the pads first, then via down outside the body), pull the GND via and the two V_MCU_SWTCH vias outside the 3.25 mm body outline, and keep the area under the body copper- and mask-free.

**Source:** BST-BMP585-DS003-02 (bosch-sensortec.com, downloaded, sec. 9.3.3)

### M38. Servo return path sized ~4 A-class (0.8 mm trace, 2 connector pins) vs multi-servo stall worst case
*switched-domains / Servo branch (J3/Q8) — confidence medium*

The whole servo/expansion branch current returns through J3 pins 1/2 (Molex 87832 Milli-Grid, ~2 A/pin class), a 0.8 mm B.Cu track (9.2 mm) plus a 2.4x1.2 mm zone patch into Q8, and out its source to the GND plane. Supply side is 2 pins into the wide In2 VBATT pour (fine). Q8 itself is fine (8.1 A cont, 17-22 mOhm). Four fin servos at simultaneous hard stall at 8.4 V can reach ~8 A aggregate: >2 A/pin on the connector and ~mm-scale 0.8 mm 1-oz copper heating (tens of degC transiently, worse sustained). At <=4 A aggregate everything is comfortable. eFuse ILIM (~14.6 A) will not protect this branch.

**Evidence:** PCB: Net-(J3-Pad1) = 0.8 mm B.Cu, 7 segs, 9.16 mm total + zone (72.7-75.1, 108.7-109.9); J3 pads 1/2 at (78.06,103.40/107.65), Q8 at (73.97,109.68). Netlist: J3.15/16 = VBATT (In2 pour 18 mm wide).

**Fix:** Widen the J3.1/2 -> Q8 return to >=1.5 mm or duplicate on another layer, and bound the worst case: document max simultaneous servo stall for the servo-adapter (its 400 uF bulk helps ms-scale peaks); if >4 A sustained is credible, use more J3 pins for supply/return or a dedicated servo power connector.

**Source:** PMPB14XN datasheet (Nexperia) for Q8 rating; Molex 87832 rating not retrieved (see checks_blocked)

### M39. LED series resistors of 10 k give 0.05-0.15 mA - indicators will be invisible outdoors
*switched-domains / Status LEDs (D6/D7/D8) — confidence high*

D6 green (always-on power, R65 10k from +3V3): ~(3.3-2.0)/10k = 0.13 mA. D7 blue (P4 GPIO27, R66 10k): ~(3.3-2.9)/10k = 0.04 mA. D8 red (P4 GPIO26, R70 10k): ~0.15 mA. Typical 0402 LEDs need ~1-5 mA for daylight/sunlight visibility; at these currents the board's only visual status (arm/pad indications on a rocket recovered in a field) is effectively dark outdoors. GPIO drive at 1-5 mA is well within P4 limits.

**Evidence:** netlist.net: Net-(D6-A)={D6.2,R65.2}, R65.1=+3V3; Net-(D7-A)={D7.2,R66.2}, IND_1={R66.1,U17.56}; Net-(D8-A)={D8.2,R70.2}, IND_2={R70.1,U17.55}; all cathodes to GND; all resistors 10 k (bom-fresh.csv).

**Fix:** Change R65/R66/R70 to 470R-1k (2.6-1.2 mA red/green, ~0.5-1 mA blue) in external_connections.kicad_sch; keep 10k only if the LEDs are deliberately night-dim.

**Source:** generic LED Vf values; no specific LED MPN in BOM (value fields are 'Green/Blue/Red')

## LOW — opportunistic

### L1. 40 MHz crystal load caps (18 pF) overshoot the ECS-400-10-37B2's 10 pF CL spec
*architecture / clocks — confidence medium*

Y2 (S3) and Y4 (P4) are ECS-400-10-37B2 (40 MHz, CL = 10 pF). Fitted load caps are 18 pF per side (C20/C22, C48/C49) giving effective CL ~9 pF + 2-3 pF stray ~11-12 pF: the oscillator will run a few tens of ppm low of nominal. Espressif's guideline asks for ±10 ppm net for RF (matters only on the S3/BLE side). Likely inherited from a 12 pF-CL crystal era and probably fine (V8 flew BLE with this class of setup), but it eats crystal tolerance budget for free. Note the 24 nH L3 in the S3 XTAL_P leg is NOT a bug — it is Espressif's documented harmonic-suppression inductor.

**Fix:** Change the four load caps to ~15-16 pF (or bench-measure offset and trim); no layout change.

### L2. eFuse timing caps drifted from the ECO's recorded values without a recorded recalculation
*architecture / power/eFuse-config — confidence high*

As built: ITIMER C50 = 10 nF (ECO: 4.3 nF ~2 ms blanking — 10 nF gives roughly 2.3x longer overcurrent blanking, ~5-12 ms, during which the FET current-limits at 14.7 A), dVdt C51 = 10 nF (ECO: 8.2 nF/21 ms — slightly slower ramp, still gentle: ~0.1 A into the ~400 uF VBATT bulk), NRETRY C45 = 1 uF (ECO: 560 nF — more retries), RETRY_DLY C46 = 2.2 nF (matches). Auto-retry topology is preserved (correct for flight). None of this is dangerous, but the ECO says these values were chosen deliberately; the silent drift means either an undocumented rework or transcription error — worth one pass with the TPS25982 equations before fab.

**Fix:** Recompute blanking/ramp/retry from the TPS25982 datasheet equations and update either the caps or the ECO record.

### L3. R72 2 mohm 0805 shunt runs at ~0.43 W if the eFuse limit is ever reached
*architecture / power/shunt — confidence high*

At the configured 14.7 A limit the pack shunt dissipates I2R = 0.43 W — at or above the rating of a generic 0805 (0.125-0.5 W; current-sense-grade 0805s reach 0.5-1 W). Brief servo stalls are fine; a sustained current-limit event cooks a standard part. The BOM row gives no MPN ('2 m', 0805), so the build could land a 0.125 W generic. Also confirm the INA230 Kelvin routing at layout (out of this pass's scope). If ILIM is reduced per the connector finding, this resolves itself.

**Fix:** Pin a current-sense-grade MPN (>=0.5 W, e.g. 0508 wide-terminal) in the BOM, or lower ILIM.

### L4. U9 symbol/footprint pad-9 parity warning is benign — drain tab is pad '5' in copper — but should be tidied
*architecture / pyro/parity — confidence high*

The advertised parity failure ('U9 pad 9 PYRO_GND has no matching pad') is a numbering mismatch, not a missing ground: the TSON Advance footprint numbers the 2.5 mm thermal/drain tab as a SECOND pad '5' (netted PYRO_GND with pins 5-8), while the CSD16323Q3 symbol was given an extra 9th drain pin. Electrically verified sound in the PCB (tab and pins 5-8 all PYRO_GND; sources 1-3 GND; gate 4). The same dual-pad-5 pattern exists on U6/U7/U8/U10 (tab = drain = PYROx_EXT) so the fire FETs' tabs are also correctly netted. Fix the symbol (or renumber the tab pad to 9) so schematic-parity runs clean — this is the check that once caught a real missing LoRa ground, and leaving a permanent warning trains people to ignore it.

**Fix:** Renumber the footprint tab pad to 9 (or drop the symbol's 9th pin) in Footprints.pretty/TSON Advance_TOS + the Custom symbol; re-run parity.

### L5. IMU INT2 (ISM6HG256X pin 9) is a labeled single-pin net going nowhere
*architecture / sensors — confidence high*

ISM6HG256_INT2 is a named net with exactly one pin — the classic missed-connection signature the single-pin sweep exists to catch. INT1 is wired to P4 GPIO50. If dual-interrupt operation (e.g. separate high-g wake vs data-ready routing) is ever wanted, there is no trace. If intentional, the label should be removed or marked no-connect to silence the isolated-label ERC warning.

**Fix:** Route to a spare P4 GPIO or delete the label / add a no-connect flag.

### L6. USB port-select rests on an unverified FSUSB63 SEL truth table; wrong mapping would strand one MCU's USB
*architecture / usb — confidence medium*

U1 FSUSB63 routes J6 D± to either P4 (HSD2, CEN_D±) or S3 (HSD3, OUT_D±); HSD1 is unconnected. SEL1 is pulled high fixed (R1 100 k), SEL0 pulled high (R2) and grounded by slide switch S1 — so the two reachable codes are 11 and 10. If the truth table is 00=sleep/01=HSD1/10=HSD2/11=HSD3 (search-confirmed only for 00=sleep), both used ports are reachable and the design is correct; if 10 selects HSD1, one switch position is dead USB. Datasheet fetch failed repeatedly (403/timeout) so this could not be closed. Also note: with USB attached, S1 toward the P4 and VPP off, the mux couples host D± into the cold P4 — bench-only, contained by QOD.

**Fix:** Pull the FSUSB63 datasheet and confirm 10->HSD2 / 11->HSD3 before fab; it is a one-resistor fix (move R1 to GND) if the mapping is offset.

### L7. J5 has no MPN (value 'Conn_01x04'); U19 footprint is named for the TPS259827 variant
*connectors / BOM — confidence high*

J5's BOM value is the generic 'Conn_01x04' with the JST_SH_BM04B-SRSS-TB footprint -- the buyable part (BM04B-SRSS-TB) is only implied, risking a wrong-part order. Cosmetic: U19 (TPS259824LNRGET) uses footprint 'IC_TPS259827LNRGER' -- same RGE QFN-24 land, but the name will confuse future BOM/footprint audits. Also noted on the sibling base-station (out of scope here): J4 value S3B-XH-A with a JST EH footprint.

**Fix:** Set J5 value/MPN to BM05B-SRSS-TB's 4-pin sibling BM04B-SRSS-TB(LF)(SN); rename the U19 footprint or add a note.

### L8. 4-pin SH LoRa link has no protection against a reversed jumper: pin-order reversal puts pack voltage on the daughterboard's UART pin
*connectors / connectors/LoRa harness — confidence high*

Rocket J5 (1=switched GND, 2=FL2->VBATT, 3=LoRa_RX/S3 GPIO10, 4=LoRa_TX/GPIO11) pairs 1:1 with daughterboard J6 (1=VSS, 2=+BATT, 3=LoRa_TX/GPIO6, 4=LoRa_RX/GPIO5) -- pin-for-pin verified correct including TX/RX crossover, and identical to the base-station's J6 (1=GND,2=V_LORA,3=LoRa_RX,4=LoRa_TX), so one straight cable convention serves the whole fleet. But SH jumpers assembled with contacts on the same ribbon side reverse pin order (1<->4, 2<->3): that puts 6.4-8.4 V on the daughterboard S3's GPIO5 and the switched ground on its +BATT. The 5-pin GNSS link keeps power on the center pin under reversal (less catastrophic).

**Fix:** Standardize and label the harness ('straight, 1:1'); optionally add a small series resistor or TVS on the daughterboard UART pins in its next spin.

### L9. USB-C connector J6 courtyard overlaps fiducial FID1 (DRC error)
*connectors / connectors/USB — confidence high*

The only DRC error on the board: J6's courtyard overlaps FID1. A fiducial under/next to the USB shell may be unusable for assembly vision alignment.

**Fix:** Relocate FID1 outside the USB4110 courtyard (3 fiducials should remain usable).

### L10. 'GPS' port label is clipped under camera connector J4's shield pad and sits ambiguously between J1 and J4
*connectors / silkscreen — confidence high*

The bottom-silk 'GPS' text at (80.16,111.82) is overlapped by J4's SH2 pad (DRC silk_over_copper) and by the J4 connector body; in the render only a partial character is visible, and it sits closer to J4 (camera) than to J1 (the actual GNSS port). Port labels LoRa (J5) and CAM (J4) are correct.

**Fix:** Move 'GPS' next to J1's pad row (left of x=76) where it is visible and unambiguous.

### L11. CHIP_PU RC uses 100 nF vs Espressif-recommended 1 uF
*esp32-p4 / ESP32-P4 reset — confidence high*

CHIP_PU has R42 10k (to V_MCU_SWTCH) + C39 100 nF, i.e. ~1 ms delay. The guidelines recommend R=10k, C=1 uF (~10 ms). The 50 us t_STBL requirement is met with margin given the TPS22918's CT-controlled ~1-2 ms rail ramp (C88 2.2 nF), so this is functional; the larger cap mainly buys noise/brownout-glitch immunity on a vibration-heavy vehicle where the P4 rail is hot-switched by U22.

**Fix:** Change C39 to 1 uF 0402 (no layout impact).

### L12. GPIO34 (JTAG-source strap) floats and is exposed on expansion connector J3.10
*esp32-p4 / ESP32-P4 straps / JTAG — confidence medium*

The P4 datasheet lists GPIO34 as the JTAG signal-source-selection strap, default floating, and says it 'requires an external pull-up/down'. On this board GPIO34 = EXP_11 routed to J3 pin 10 with no pull either way, so the strap level at boot is undefined (and can be biased by whatever daughterboard is on J3). Worst plausible effect is intermittent loss of USB-Serial-JTAG debugging (boot modes are controlled by GPIO35/36 which are correctly pulled); it may additionally be eFuse-gated (unconfirmed). Boot-mode straps GPIO37/38 are also on J3 (11/12) but are 'any value' for both boot modes and double as the UART0 factory-flash pins — external adapters must simply not drive J3.10-12 at power-up.

**Fix:** Add a 10 k pull on GPIO34 set to the level that selects USB-Serial-JTAG (copy the level from the P4 datasheet strap table), placed board-side of J3. Document in the servo-adapter spec that J3 pins 10-12 must not be driven during power-up.

### L13. P4 cannot be powered (hence flashed) until working S3 firmware raises POWER_SWITCH — no hardware override provided
*esp32-p4 / Factory programming / bring-up — confidence high*

V_MCU_SWTCH (the whole P4 domain) comes only from U22 (TPS22918) whose ON pin is pulled down by R68 100k and driven solely by S3 GPIO7 (POWER_SWITCH). That is the intended C-4 safety interlock, but it serializes factory bring-up: a blank or bricked S3 makes the P4 unreachable by USB (mux position P4) AND by UART0 on J3, because the chip is simply unpowered. Any S3-side fault during board bring-up blocks all P4 verification.

**Fix:** Add a DNP 0-ohm/solder-jumper from +3V3 to the U22 ON side of R71 (or at least a labeled test pad on POWER_SWITCH) so production/bench can force the P4 domain on without S3 firmware. Keep it unpopulated for flight builds to preserve the interlock.

### L14. SC-32S ESR spec (70 kOhm max) sits exactly at Espressif's <=70 kOhm limit — compliant with zero margin; light-sleep work depends on this oscillator starting
*esp32-s3 / S3 32.768 kHz crystal — confidence high*

Y1 = SC-32S 32.768 kHz, CL 12.5 pF, ESR max 70 kOhm, on the dedicated XTAL_32K_P/N pins 21/22 (GPIO15/16) with C19/C21 = 22 pF (CL_eff = 11 pF + stray ~ 12-13 pF vs 12.5 pF spec — correct). Espressif's requirement is ESR <= 70 kOhm, so a worst-case crystal is at the limit; oscillator startup margin at temperature extremes is the classic 32 k failure mode, and the S3 light-sleep plan (SC-32S noted in project context) relies on it. Placement is good (Y1 1.9 mm from pins 21/22). This is the same construction Espressif sanctions, so low severity: an opportunistic swap to a lower-ESR part (e.g. Epson FC3215AN, 50 kOhm max, same 3215 body) buys margin for free.

**Fix:** Optional: substitute a 50 kOhm-max 32.768 kHz 3215 crystal (FC3215AN class, CL 12.5 pF) at next BOM touch; otherwise verify 32 k oscillator startup at cold during light-sleep bring-up.

### L15. 40 MHz load caps (18 pF) overshoot the crystal's 10 pF CL by ~1-2 pF and the crystal sits ~8 mm from the chip
*esp32-s3 / S3 40 MHz crystal — confidence high*

Y2 = ECS-400-10-37B2-CKY-TR: 40.000 MHz, CL = 10 pF, +/-10 ppm tolerance, +/-10 ppm stability, ESR 40 ohm — tolerance meets Espressif's +/-10 ppm requirement. Load caps C20/C22 = 18 pF each give CL_eff = 18/2 + C_stray = 9 + (2-3) = 11-12 pF vs the 10 pF spec (HDG formula CL = C1xC2/(C1+C2) + Cstray), pulling frequency a few ppm low; combined worst case stays inside BLE (+/-50 ppm) but eats into Wi-Fi margin (+/-25 ppm). Layout: Y2 at (84.73,137.84) is ~8 mm from U15 XTAL pads (77.2-77.6,141.6) with 0.127 mm traces; XTAL_N meanders ~11 mm. Espressif asks for the crystal close to the chip; 8 mm is workable but not tight. The series L3 24 nH on XTAL_P is per HDG — correct as placed.

**Fix:** Change C20/C22 from 18 pF to 15-16 pF (targets CL_eff ~ 10 pF with 2-3 pF chip-down stray). Keep footprints; verify center frequency on the bench with a spectrum analyzer during RF conducted test. No layout change required.

### L16. GPIO0 relies on internal weak pull-up only; Espressif recommends an external pull-up
*esp32-s3 / S3 straps/boot — confidence high*

Net-(U15-GPIO0) contains only the BOOT button S2 (to GND) — no external resistor. The internal 45 k WPU (datasheet Table 3-1/5-4) makes this work on DevKits, but the HDG states 'it is recommended to place a pull-up resistor at the GPIO0 pin'; in a high-vibration, ESD-prone rocket bay a 45 k source impedance on the boot strap is soft. A boot-mode glitch at a brownout-recovery instant would drop the S3 into download mode until power cycle. All other straps check out: GPIO46 has an external 10 k to GND (R37) reinforcing its WPD; GPIO45 is NC (WPD -> VDD_SPI 3.3 V mode — correct for RH2); GPIO3 is on ESP_SDO to the P4 (GPIO19) which is unpowered/hi-Z at S3 boot — its default is 'floating' and the JTAG-source strap is inert with unburned eFuses.

**Fix:** Add 10 k from GPIO0 to +3V3 beside S2. No change needed on GPIO3/45/46.

### L17. Both 40MHz crystals sit 9-12mm from their MCUs with asymmetric traces (Y2: 14.0 vs 5.4mm; Y4: 14.8 vs 9.2mm)
*layout-decoupling / clocks/crystals — confidence high*

Y2 (S3) at (84.73,137.84) is ~9mm from U15's XTAL pins with total net copper XTAL_N=13.95mm vs XTAL_P-side=5.36mm; it also sits directly opposite the B-side U21 mux / U18 buck / L5-L6 cluster (In1 GND plane shields it). Y4 (P4) traces are 14.76/9.17mm. Positives verified: zero vias on all four crystal nets, load caps 1.1-1.8mm from crystal pads, L3 24nH series on S3 XTAL_P matches the Espressif reference, and both 32k crystals (Y1/Y3) are tight and symmetric. Espressif asks for crystals close to the chip (>=2mm gap, no vias, no digital traces under) - long runs cost EMI/level margin, not frequency accuracy.

**Fix:** Opportunistic: shorten/balance the Y2 run toward U15 if the S3-side floorplan is ever touched; no action mandatory.

### L18. S3 RF supply pins (VDD3P3 pins 2/3) have only one local 100nF; Espressif recommends 10uF per RF pin
*layout-decoupling / cpu/esp32-s3 RF — confidence medium*

Net-(U15-VDD3P3) is fed from +3V3 through L1 4.3nH and carries only C33 100nF (1.21/1.42mm from pins 2/3). Espressif S3 layout guide: 'RF-related pins (pin2 and pin3): place a 10uF capacitor for each pin. You can also add a 0.1uF or 1uF in parallel.' The nearest 10uF (C36) sits on the far side of L1, 8.2mm away. BLE TX (~100mA bursts) will ride mostly on 100nF + the inductor-isolated plane; this affects TX droop/phase noise margin, not functionality.

**Fix:** Grow C33's neighbor into a 10uF (or add one) on the VDD3P3 side of L1.

### L19. U11 NAND (GD5F2GQ5, +3V3) has a single 100nF at 3.67mm and no bulk within 7.5mm
*layout-decoupling / memory/NAND — confidence high*

NAND program/erase draws ~25-30mA bursts; the only nearby decoupler is C16 100nF at 3.67mm from VCC pin 8, with the nearest 10uF (C76) 7.5mm away. The In2 +3V3 plane sits directly under it, so impedance is acceptable, but this is the thinnest-decoupled digital IC on the board and it is the flight logger's storage.

**Fix:** Add 1-10uF near U11 pin 8 or shift C16 closer.

### L20. TPS2121 IN1 (USB VBUS from J6) has no capacitor anywhere on the net
*layout-decoupling / power/USB-mux — confidence high*

Net-(J6-VBUS) = J6 VBUS pins + CR3 TVS + R51/R62 + U21.7 only. The TPS2121 DS advises 'To help nullify the inductance of the cables and prevent ringing... a large capacitance can be used near the input of the device' (qualitative, not mandatory). Bench use is USB-powered (RunCam recording on USB per project history), so switchover/ringing robustness on this input has real exposure.

**Fix:** Add 1-10uF from VBUS to GND near U21 IN1 (respect USB inrush limits).

### L21. TPS22918 (U22) VIN has no local bypass - nearest +3V3 cap is 6.0mm away (C65 47uF), nearest 100nF is 19.6mm
*layout-decoupling / power/load-switch — confidence high*

The switch that energizes the whole P4/sensor domain (V_MCU_SWTCH) has its VIN bypass requirement in the DS: 'typical recommended bypass capacitance is 1uF ceramic... This capacitor must be placed as close to the device pins as possible.' U22 sits at the very edge of the In2 +3V3 plane (0.65mm inside its boundary), which plus the CT-configured slow ramp (C88 2.2nF) softens the practical impact, but a +3V3 dip at the S3's rail during VPP turn-on is avoidable for one 0402.

**Fix:** Add 1uF X7R at U22 VIN.

### L22. Two dangling In3.Cu track slivers on pyro nets and one duplicated stacked via
*layout-misc / layout/cleanup — confidence high*

DRC warnings, all real but harmless: a 5.7e-6 mm sliver of PYRO1_CONT at (76.98,119.13) and a 0.13 mm stub of PYRO3_FIRE at (80.27,114.56), both on In3.Cu; and two identical vias stacked at exactly (82.04,135.951805) on Net-(U21-OV1) (the holes_co_located warning — same drill twice, fab will drill the same hole twice or flag it).

**Fix:** Delete the two slivers and one of the stacked vias; re-run DRC.

### L23. Silk triage: buzzer '+' partially unprintable; J2 pin-1 circle drawn off-board; the rest is cosmetic
*layout-misc / silk — confidence high*

Of the remaining silk warnings: (1) LS1 (MLT-8530 buzzer, polarized) has its only '+' mark sitting over the LOAD+ pad's mask opening and overlapping Y1's outline — the part over bare copper won't print, weakening the polarity mark (buzzer is reflowed from centroid data, so low). (2) J2's pin-1 circle is at (75.55,171.42), OUTSIDE the 171.32 board edge — it will never print; orientation is still recoverable from the 'GND 4 3 2 1' channel labels above the terminal block. (3) S1's pin-1 circle clipped at the left edge. (4) D7/D8 cathode circles overlap the '2'/'1' channel-LED labels. (5) C11/C13 refs collide with C13/R23 silk and pads — those refs are under the C12 can anyway. (6) U19/U21/J6/U13 pin-1 dots partially over neighboring pads — all these QFNs are machine-placed, cosmetic. (7) 'LoRa'/'GPS'/'CAM'/'GND' labels partially over pad openings — still legible in renders. No polarity mark other than LS1/C15/C12 (reported separately) is compromised.

**Fix:** Move LS1's '+' off the pad, move J2's circle inside the outline next to pad 5_1; batch-tidy the rest opportunistically.

### L24. S1 slide switch labeled 'O'/'F' (reads ON/OFF) but it actually selects the USB target MCU
*layout-misc / silk/labels — confidence high*

S1 (JS202011 DPDT slide, top side next to USB) drives only SEL0 (with R2 100k to +3V3), the select input of U1 FSUSB63 which routes J6's D+/- to either the P4 (CEN_D+/-) or the S3 (via 22 R R38/R39 to GPIO19/20). The 1.0 mm silk letters 'O' (76.98,164.49) and 'F' (76.98,169.41) beside it read as ON/OFF, which misdescribes a USB-routing switch and invites 'why won't it turn off' confusion at the bench/pad.

**Fix:** Relabel to 'P4'/'S3' (or 'FC'/'OC') at the two switch positions.

### L25. IMU axis marks 'X'/'Y' at 0.75 mm violate the board's own 0.8 mm min text height, and sit 13 mm from the IMU
*layout-misc / silk/text — confidence high*

The two 0.75 mm gr_texts 'X' (92.86,126.60) and 'Y' (91.86,127.77) with the axis-arrow glyph are below the board min_text_height 0.8 constraint (2 DRC warnings). 0.75 mm/0.15 mm stroke usually still prints, but marginally. They are placed next to D6/PWR LED area, ~13 mm from U2 (ISM6HG256X IMU at 79.92,99.03, mounted at 45 deg) — worth double-checking they mark the intended frame (board axes vs IMU axes) since the IMU is rotated 45 degrees.

**Fix:** Bump to 0.8 mm and confirm the arrows describe the intended (board) axes given the 45-degree IMU mounting.

### L26. TPS62152 input HF loop is loose: 22 uF cap 2.5 mm from PVIN with a 5.6 mm ground return, and no local 100 nF
*layout-power / 3v3-buck — confidence high*

C43 (22 uF, the only input cap on Net-(U18-AVIN)) sits with its + pad 2.5 mm from PVIN pad 11 and its GND pad 5.6 mm from PGND pads 15/16, giving an input loop perimeter of ~8-10 mm through the B.Cu zone and fragmented B.Cu GND fill. There is no small HF cap at the buck input (C41 1 uF/C42 100 nF are at the eFuse input, 13 mm away). At the selected 1.25 MHz (FSW tied to +3V3 - note this is the datasheet-endorsed method, footnote: 'Connect FSW to VOUT', so NOT a bug) this is workable but noisier than TI's layout. Output side is fine: SW node is a compact 3.2 mm2 zone, L6 = recommended 2.2 uH with Isat 1.89 A vs ~1.35 A peak, VOS senses at +3V3 after L6, nearest output cap C53 10 uF at 6.1 mm plus ~35 distributed +3V3 caps and the 860 mm2 In2 plane (DCS-Control tolerates).

**Fix:** Add a 100 nF 0402 directly across PVIN-11/12 and PGND-15, and pull C43 to the pin row; connect its GND pad to the U18 PGND pads with direct copper rather than through the fill.

### L27. ESP_VDD_HP distribution necks to 0.14-0.2 mm; TLV62569 layout itself is excellent
*layout-power / p4-core-buck — confidence high*

U20 (TLV62569) placement is textbook: input C47 10 uF 1.0 mm from VIN, SW node 1.24 mm x 0.2 mm total, L8 2.9 mm away, output C55/C92 10 uF within 2.5 mm of L8.2, FB (0.1 mm) exits the opposite side from SW and never parallels it. FB/EN go to P4 pins 78/79 - the Espressif P4-controlled external-DCDC scheme (P4 provides feedback/DVS), not a missing divider. But the ESP_VDD_HP rail then fans to U17 pads 26/76/91 through 0.127-0.2 mm traces (bottlenecks 0.14-0.20 mm), 16 mm on F.Cu + 20 mm In2 + 8 mm In3 with only 4 vias, no zone. P4 VDD_HP transients (~0.5-0.8 A) put these at/above their ~0.6-0.9 A/10 C rating; IR drop is compensated by the P4's own feedback, so this is a heating/robustness nit, not regulation.

**Fix:** Widen ESP_VDD_HP arteries to 0.4-0.5 mm (space exists on In2/In3) and add 2-3 vias at the L8/C55 node and at the U17 entry.

### L28. eFuse strap values and hold-up ceramic drifted from the ECO's recorded 'IMPLEMENTED' values
*layout-power / power-eco-drift — confidence high*

Re-verification of power-eco.md Change 2 against the current netlist: matches - ILIM 100 Ohm (R48), UVLO 1 M/210 k (R44/R45), CIN 1 uF + 100 nF (C41/C42), COUT 1 uF (C54), RETRY_DLY 2.2 nF (C46), input clamp (CR2 CUS10S30), IMON/LDSTRT to GND. Deviations: ITIMER now 10 nF (recorded 4.3 nF -> overcurrent blanking roughly doubles), NRETRY now 1 uF (recorded 560 nF -> different retry count), dVdt now 10 nF (recorded 8.2 nF, minor). Change 1 deviation: hold-up is C56 330 uF || C59 1 uF (recorded 330 uF || 10 uF X7R). Change 7 upgraded: R20 is a 150 Ohm CRCW1206-HP (0.75 W) instead of the recorded 0603 - better. None are electrically wrong, but the ECO record and board disagree; NRETRY/ITIMER changes alter protection timing and should be deliberate.

**Fix:** Either restore the recorded values or update power-eco.md with the rationale for 10 nF/1 uF (and re-derive blanking time vs servo inrush).

### L29. eFuse PG (PG_RAIL) and TPS62152 PG dead-end at their pullups - no MCU can read them
*layout-power / power-telemetry — confidence high*

PG_RAIL is exactly 2 nodes (U19.13 + R59 pullup to +3V3); Net-(U18-PG) is 2 nodes (U18.4 + R49). The ECO (Change 2 pin map) specifies 'PG 13 ... PG_RAIL -> MCU; main-power-good / fault telemetry'. As built the fault/PG telemetry the ECO promised does not exist; an eFuse retry/fault in flight is invisible to firmware.

**Fix:** Route PG_RAIL to a spare S3 or P4 GPIO (spares exist: several unconnected-(U15-GPIOxx) pins); same for U18 PG if desired.

### L30. P4 XTAL_N escape runs 0.10-0.16 mm edge-gap from SENS_SCLK (sensor SPI clock) for ~5-6 mm
*layout-rf / P4 40 MHz crystal (Y4) — confidence high*

The XTAL_N trace leaving U17.99 runs south at x=80.92 (y 106.79->103.07) while SENS_SCLK runs parallel at x=81.12 (0.20 mm centers, 0.10 mm edge gap, ~3.7 mm), then both turn west and continue at 0.10-0.16 mm gap for another ~2.8 mm (y~102.2-102.8). SENS_SCLK clocks the IMU/baro SPI continuously in flight, coupling into the crystal node (jitter risk). Violates Espressif crystal-isolation guidance ('best not to route any signal trace' near crystal traces), though the P4 has no radio so the functional consequence is limited to clock jitter margin. Vertical shielding is intact (In1 GND 100% under the region).

**Fix:** Nudge SENS_SCLK one grid step east/south (>=0.3 mm gap) along the shared stretch, or slip a thin GND guard between them. QFN escape congestion only forces the first ~1 mm, not 6 mm.

### L31. H2 mounting hole is a floating plated ring 0.6 mm from the antenna body — grounded-or-nylon decision needed
*layout-rf / RF / antenna mechanical — confidence high*

H2 at (75.22,130.50): plated 2.2 mm drill with 2.6 mm pad plus 3.8 mm 'connect' rings on F.Cu and B.Cu, assigned to NO net (zones do not connect to it). Its ring edge sits ~0.6 mm from the antenna body outline (body y starts 133.02) and ~1.5 mm from dummy pad U14-2. A steel screw/standoff there is floating metal immediately beside the radiator: minor detune/pattern skew, plus an undefined-potential ESD gap next to the RF section. The on-ground Molex antenna is tolerant of nearby ground by design, so grounding the ring is safe.

**Fix:** Tie the H2 ring to GND (preferred, matches the on-ground antenna concept and gives the standoff a defined potential) or specify nylon hardware for this hole in the build notes.

### L32. RF feed is ~52-60 Ohm (est.) instead of 50 Ohm: 0.127 mm CPWG over the 0.1 mm prepreg
*layout-rf / RF feed line — confidence medium*

The LNA_IN->match->antenna feed (total ~8.8 mm: LNA_IN net 3.53 mm + U14-Feed net 5.27 mm, all F.Cu) is 0.127 mm wide with a measured 0.127 mm edge gap to the F.Cu GND pour, over solid In1 GND 0.1 mm below (er 4.5 per board stackup). Analytic CPWG estimate lands ~52-60 Ohm (plain-microstrip term alone would be ~62 Ohm; the tight coplanar gap and soldermask pull it down). At 0.13 lambda the added mismatch is small and the CLC match (values mid-range of Espressif's recommended windows) can absorb it, so this is a tuning-margin item, not a functional break. Espressif requires a 50 Ohm RF trace.

**Fix:** Before fab, run the geometry through a field solver; if confirmed high, widen the feed to ~0.16-0.20 mm keeping the same coplanar gap. Otherwise accept and tune the match at bench.

### L33. S3 40 MHz crystal loop is long and asymmetric (XTAL_N 14.0 mm vs XTAL_P 8.1 mm) with 18 pF caps on a CL=10 pF crystal — BLE frequency-offset margin eaten, bench verify
*layout-rf / S3 40 MHz crystal (Y2) — confidence medium*

Y2 (ECS-400-10-37B2-CKY: 40 MHz, CL=10 pF, +/-10 ppm tol, +/-10 ppm stability — meets Espressif's +/-10 ppm requirement) sits 7-9 mm from the S3 XTAL pins. Trace totals: XTAL_P side 2.75 mm (U15.54->L3) + 5.36 mm (L3->Y2.3/C20) = 8.11 mm; XTAL_N = 13.95 mm. L3 = 24 nH series on XTAL_P matches Espressif's recommended harmonic-suppression inductor (good). Per Espressif CL = C1*C4/(C1+C4) + Cstray: with C20=C22=18 pF the series term is 9 pF; the long traces add Cstray ~1.5-3 pF giving CL_eff ~10.5-12 pF vs the 10 pF spec — predicted pull roughly -5 to -25 ppm. Stacked with crystal tolerance/stability this approaches +/-35-45 ppm worst-case against the BLE +/-50 ppm budget. Same 18 pF (C48/C49) on the P4's Y4 (XTAL_P 9.2 mm / XTAL_N 14.8 mm) is uncritical — no radio on P4.

**Fix:** No layout change mandatory; add a bench RF frequency-offset measurement (esp-phy RF test / BLE tester) to the bring-up checklist and be ready to drop C20/C22 to 15-16 pF. If respinning the area anyway, shorten/balance XTAL_N.

### L34. USB D+/D- asymmetry and ESD array on ~10 mm stubs — benign at Full-Speed, tidy opportunistically
*layout-rf / USB — confidence high*

(a) From J6 to mux U1, D+ dives through 4 vias and runs 17.5 mm on In3 while D- stays entirely on F.Cu (21.3 mm); connector-to-mux path mismatch ~2 mm with different reference layers. (b) ESD array CR3 (SP0503BAHTG) sits at (92.11,159.83), ~9 mm EAST of J6 while the data path exits WEST to U1 — the clamps hang on ~10.4 mm (D+) / ~10.7 mm (D-) stubs, adding inductance in the clamp path; mitigating: the FSUSB63 datasheet rates the mux's USB pins for IEC61000-4-2 system-level ESD. (c) R38/R39 = 22 Ohm series into S3 GPIO19/20 — acceptable at FS. The whole USB tree is Full-Speed only (P4 side is USB-Serial-JTAG on GPIO24/25; P4's HS OTG DP/DM pins 49/50 deliberately NC; S3 native USB is FS), so intra-pair skew/impedance effects are electrically negligible: no functional risk, robustness cleanup only. CEN_D+/- pair itself is clean: 0.2 mm pitch pair on In3, 64.7 vs 64.0 mm (0.76 mm mismatch), 4 In2 pour-boundary crossings but continuous In4 GND 0.535 mm below; no stubs on unused mux port HSD1.

**Fix:** If touched anyway: place CR3 directly at J6 between connector and mux, and re-route D+ alongside D- on F.Cu. No action strictly required for function.

### L35. 390 pF soft-start is ~150 us into ~87 uF of output capacitance — startup runs in current limit
*power-input / 3V3 buck (U18) — confidence medium*

SS/TR sources 2.5 uA; C44 390 pF ramps the reference in roughly 130-200 us. Output capacitance on +3V3 is C65 47 uF + 4x10 uF + smalls (~87 uF nominal, maybe ~55 uF effective at bias). Charging that in 150 us needs ~1.9 A plus load — beyond the intent of soft-start, so startup is governed by the converter's current limit instead. It will start (DCS-Control tolerates it) but the inrush hits the mux/hold-up node every power-up. Datasheet typical application uses 3.3 nF.

**Fix:** Increase C44 to 3.3-10 nF (1-3 ms soft start).

### L36. Hold-up companion ceramic is 1 uF (ECO: 10 uF); ECO Change-3 branch bulk for GNSS/LoRa never added
*power-input / Hold-up / branch bulk (ECO deviations) — confidence high*

The Change-1 hold-up itself is correctly implemented: C56 (TCJE337M016R0050, 330 uF 16 V polymer-tant) sits on V_MCU_2S (mux OUT) with correct polarity (CAPMP pad 1 = anode at the marked end, netlist pin 1 -> rail), and mux SS (C52 1 uF, ~88-92 V/s) keeps its charge inrush to ~30 mA as calculated. But the specified parallel 10 uF X7R is only C59 1 uF, and the Change-3 recommendation to add 10-47 uF at the GNSS and LoRa supply pins is absent — FL1.2->J1.3 and FL2.2->J5.2 are two-node nets with no capacitor, so branch bulk exists only if the daughterboards carry it.

**Fix:** Swap C59 to 10 uF/16 V, and either add 10-22 uF at J1.3 and J5.2 or verify the GNSS/LoRa daughterboards carry their own input bulk and note it in the ECO.

### L37. No input capacitor on the USB (IN1) input
*power-input / Power mux (U21) — confidence high*

Net-(J6-VBUS) contains only the USB-C VBUS pads, the CR3 TVS, the OV1/PR1 divider tops and U21 IN1 — no local ceramic. TI recommends local input capacitance on each mux input for hot-plug and switchover transients; IN2 has the whole VBATT bank but IN1 has nothing, so USB hot-plug rings into the OV1 divider and the mux switchover comparators.

**Fix:** Add ~1 uF (25 V) X7R from U21 pin 7 to GND near the pin.

### L38. NRETRY 1 uF computes to ~1815 retries — beyond the largest defined quantization bucket
*power-input / eFuse (U19) retry config — confidence high*

Auto-retry itself is confirmed as the ECO claims: RETRY_DLY C46 2.2 nF gives ~92-103 ms between retries (Eq.14: (128*2200+4)pF*0.75V/2.05uA, datasheet table lists 2.2 nF -> 91.7 ms). But NRETRY C45 is 1 uF, and Eq.15 gives N = 4*2.05uA*1e6pF/(2.05uA*2204pF) ~ 1815, above the last defined bucket (256<N<1024 -> 1024); the datasheet's own combination table for 2.2 nF RETRY_DLY tops out at 0.47 uF for 1024 retries. Behavior at N>1024 is formally unspecified. The ECO recorded 560 nF (~1016 -> 1024 bucket). Also worth a design thought: after the retry count exhausts, the eFuse LATCHES OFF until power-cycle — for a flight computer, indefinite retry (NRETRY short to GND) is arguably the safer terminal behavior.

**Fix:** Change C45 to 470 nF (defined 1024 retries) or replace with a short to GND for indefinite retry; record the choice in power-eco.md.

### L39. DRC parity mystery solved: U9 schematic pin 9 (thermal drain) has no pad in the shared 'TSON Advance_TOS' footprint — electrically harmless, net verified correct on the PCB
*pyro / pyro arm FET / library parity — confidence high*

The fresh DRC reports (reverse of the task note): 'No pad found for pin 9 (PYRO_GND) in schematic' on footprint U9. Root cause: U9 uses the TI CSD16323Q3 symbol, which models the SON 3.3×3.3 center/thermal drain pad as pin 9 (DRAIN_9), but the placed footprint 'Footprints:TSON Advance_TOS' (shared with the four Toshiba fire FETs) implements the center drain as a 2.5×2.5 mm pad numbered '5' (duplicate of perimeter pad 5) and has no pad '9'. Physically the TI Q3 package and Toshiba TSON Advance are the same industry 3.3×3.3 8-lead + center-drain-pad outline with identical S(1-3)/G(4)/D(5-8+pad) assignment, so the part fits and the center pad IS soldered and IS on the right net: PCB U9 pads read 1-3=GND(source), 4=Net-(U9-GATE), 5(big)+5-8=PYRO_GND(drain). Netlist pin 9 and pads 5-8 are the same PYRO_GND net, so no connectivity is lost. The other three fire FETs (U6/U7/U8/U10, Toshiba symbols with drains 5-8 only) have no mismatch — DRC is otherwise parity-clean.

**Fix:** Restore parity on the symbol side: renumber the CSD16323Q3 symbol's pin 9 to a stacked duplicate of pin 5 (or delete it), rather than adding a pad 9 to the shared footprint (which would break the four TPN parts that have no pin 9). Cosmetic; no electrical change.

### L40. R20 value field '150 - CRCW1206-HP' is not an orderable MPN; rating itself is adequate (0.5 W pulse-proof vs 0.47 W worst-case)
*pyro / pyro charge path — confidence medium*

The charge resistor moved from the ECO's RCP0603 to a 1206 Vishay CRCW-HP ('pulse proof, high power' series, 0.5 W, 200 V limiting element voltage per the CRCW-HP e3 datasheet), which resolves the original 0603 pulse-rating question robustly: worst-case charge peak is 8.4²/150 = 0.47 W decaying with τ = RC = 150Ω×10 mF = 1.5 s (average over the first τ ≈ 0.24 W); a permanent V_CAP short (failed cap) gives 0.47 W continuous = 94% of rating — survivable but at-limit. However the schematic Value string is informal and the BOM MPN column is empty, so purchasing could substitute a standard 0.25 W CRCW1206, which WOULD be overstressed at 0.47 W.

**Fix:** Set the exact MPN (e.g., CRCW1206150RFKEAHP) in the schematic/BOM.

### L41. R73 (1k 0402 PYRO_GND bleed) sees ~70 mW if a fire FET is on while disarmed — above typical 0402 rating if sustained
*pyro / pyro continuity — confidence high*

With a channel fire FET on and the arm FET off, V_CAP (8.4 V) appears across match (1Ω) + R73 (1k): 8.3 mA flows through the e-match (safe: 3.6× below the 30 mA no-fire line, but non-zero and worth knowing) and R73 dissipates ≈69 mW. A 1/16 W (62.5 mW) 0402 is marginally over rating if this state persists (it is fine for the intended 200 ms fire pulse). This state occurs during any fire attempt in which arming failed, or a stuck-on driver.

**Fix:** Use an 0402 rated 100 mW (or move to 0603), or raise R73 to 2-4.7k (continuity low-level rises from ~65 mV to ~0.13-0.3 V, still far below the P4 VIL of 0.825 V).

### L42. Two dangling pyro track slivers on In3.Cu (PYRO1_CONT, PYRO3_FIRE) — cosmetic; also note the board is 6-layer, not 4
*pyro / pyro layout hygiene — confidence high*

DRC flags 'track has unconnected end' for a 5.7e-6 mm PYRO1_CONT sliver at (76.98,119.13) and a 0.13 mm PYRO3_FIRE stub at (80.27,114.56), both on In3.Cu. Connectivity is complete (unconnected_items is empty); these are edit leftovers. Also: the stackup is 6 copper layers (F, In1-In4, B) — the task brief said 4-layer, and the pre-generated SVGs only cover F/In1/In2/B, so In3/In4 (where all pyro FIRE/CONT/ARM control routing lives, 30-70 mm runs at 0.1 mm) were not visually reviewed by artifact.

**Fix:** Delete the two slivers. Re-export In3/In4 SVGs if a visual pass of the control-line routing is wanted.

### L43. IMU (U2) and baro (U4) have no 100 nF decoupling caps; each relies on a single 10 uF 0402
*sensors / sensors/decoupling — confidence high*

ST DS15034 application hints: 'Power supply decoupling capacitors (C1, C2 = 100 nF ceramic) should be placed as near as possible to the supply pin' (one on VDD, one on VDDIO). Bosch BMP585 Fig. 23/24 show 100 nF on VDD and 100 nF on VDDIO. As built: U2 has only C3 (10 uF 0402) at 1.17 mm (pad-pad to VDDIO pin 5) / 2.20 mm (VDD pin 8); nearest 100 nF (C2) is 4.3 mm away and serves the mag. U4 has only C5 (10 uF 0402) at 1.20 mm (VDDIO pin 4) / 3.62 mm (VDD pin 8); nearest 100 nF (C90) is 4.4 mm away. The mag U3 is compliant (C2 100 nF at 1.6-1.8 mm + C4 10 uF + C6 220 nF on pin C1 at 1.03 mm, matching IIS2MDC '100 nF ceramic, 10 uF' guidance). Electrically a 0402 10 uF at ~2 mm is likely adequate (similar ESL to a 0402 100 nF), so this is a datasheet-conformance deviation rather than a probable failure; the shared rail also carries the buzzer's pulsed load, which argues for keeping local decoupling strong.

**Fix:** Add one 100 nF 0402 tight to U2 VDD (pin 8) and one tight to U4 VDD (pin 8); optionally one each at the VDDIO pins. Cheap insurance before fab; no reroute needed beyond local placement.

### L44. eFuse timing caps drifted from the ECO record (ITIMER/dVdt/NRETRY) - electrically benign, docs stale
*switched-domains / eFuse config (U19) — confidence high*

power-eco.md Change 2 records ITIMER=4.3 nF (~2 ms), dVdt=8.2 nF (~21 ms), NRETRY=560 nF; the board has C50 ITIMER=10 nF (~4.7 ms overcurrent blanking), C51 dVdt=10 nF (~26 ms ramp), C45 NRETRY=1 uF with C46 RETRY_DLY=2.2 nF (per TI Table 7-5, 2.2 nF = 91.7 ms retry delay; 1 uF NRETRY clamps to the max retry count). All drifts are in the 'gentler/more retries' direction and consistent with the flight intent (never latch off mid-flight); ILIM (R48 100R ~14.6 A) and UVLO (R44 1M/R45 210k = 6.34/6.91 V) match the record exactly. Re-verified rest of the 26-pin map matches the IMPLEMENTED claim.

**Fix:** Update power-eco.md Change 2 table to the as-built values (or revert caps if the ECO values were deliberate); no electrical change needed.

### L45. eFuse PG output is a dead-end stub - power-good/fault telemetry never reaches an MCU
*switched-domains / eFuse telemetry (U19) — confidence high*

PG_RAIL consists solely of U19.13 (PG) and its 100k pullup R59 to +3V3. The ECO Change 2 pin map documented 'PG ... PG_RAIL -> MCU: main-power-good / fault telemetry'. As built, firmware cannot see eFuse retry/fault events (e.g., the auto-retry cycling the ECO relies on being observable), and R59 just burns a stub. Bench debug of over-current events will need a scope instead of a log line.

**Fix:** Route PG_RAIL to a spare always-on S3 GPIO (several free: GPIO8/9/13/14/17/18/34) in power.kicad_sch, or drop R59 and the net to stop implying telemetry that does not exist; update power-eco.md either way.

## INFO — observations & verified-good records

### I1. Boot-state truth table: pyro and peripheral defaults are hardware-safe at every stage; V_CAP is hot whenever a battery is inserted
*architecture / boot-safety — confidence high*

Battery in -> eFuse dVdt ramps VBATT (~20-40 ms) -> TPS2121 selects IN2 -> +3V3 -> S3 boots. At that instant: U22 ON held low by R68 100 k (P4/sensor domain cold — C-4 interlock (a) verified in hardware), LoRa/GPS/CAM/SERVO branch gates held low by R35/R7/R25/R29 10 k, pyro fire-FET gates pulled to V_CAP by R15/R16/R17/R23 10 k, DTC123J drivers off (internal pulldowns), arm FET U9 off (R22 100 k). When the S3 later asserts POWER_SWITCH, V_MCU_SWTCH ramps in ~ms; P4 GPIO6-17 are input-disabled at reset (P4 datasheet) so FIRE/ARM stay externally defined until safePyroOutputInit; firing requires FIRE and ARM both driven high on a powered P4 — three independent layers. Two facts to keep in the ops notes: (1) V_CAP (10 mF) charges through R20 150 ohm unswitched — the pyro store is at pack voltage ~5 s after battery insertion regardless of arm state (unchanged philosophy from V8, but now with the low-side-arm caveat in the pyro finding); (2) continuity sensing (65 uA through the match via R73 1 k) works un-armed by design. S3 straps: GPIO46 R37 pulldown, GPIO0 button S2, GPIO45/GPIO3 on internal defaults - clean SPI boot; S3 UART0 is unconnected so console/flash is USB-only (ROM USB-Serial-JTAG covers recovery).

### I2. Daughterboard interfaces verified compatible: LoRa J5<->J6 and GNSS J1<->J4 pin-correct, both regulate raw VBATT locally
*architecture / daughterboards — confidence high*

LoRa: rocket J5 (1 = Q10-switched ground, 2 = VBATT via FL2, 3 = LoRa_RX, 4 = LoRa_TX) mates 1:1 with daughterboard J6 (1 = VSS via common-mode choke, 2 = +BATT via series CUS10S30, 3 = its TX, 4 = its RX) — power straight-through, UART crossed correctly by netnames on each side; daughterboard is a self-contained S3+E220-900MM22S (SX1262 SPI) radio with TPS62913 wide-VIN buck, so 6.4-8.4 V feed is by design. GNSS: rocket J1 (SRSS 5-pin + shield) maps 1:1 to the PX1105R board's J4 (GNSS_RX/GNSS_TX/VSYS/VSS/RXD2), also TPS62913-regulated; its only ground is the switched VSS pin, so the low-side switch genuinely powers it down. Caveat carried in the low-side-switching finding: rocket J1 shield pads 6/7 are hard GND — keep daughterboard shield pads and cable shields off the daughterboard ground net. Servo adapter connects via harness from J3 (signals EXP_01..12 = 3.3 V P4 GPIO, LiPoPos = raw 2S — HV-servo assumption carried over from the flying V8 fleet).

### I3. ECO implementation audit: Changes 1, 2, 5, 6, 7 verified in the netlist; open checklist items 1 and 3 remain unbenched/undone
*architecture / eco-verification — confidence high*

Verified as-built (new designators): CHANGE 1 hold-up = C56 330 uF TCJE337M016R0050 + C59 1 uF on V_MCU_2S (mux OUT), 16 V part at 8.4 V = 53% derating, L5 2.2 uH + C43 22 uF filter into the buck. CHANGE 2 eFuse = U19 TPS259824L, divider/ILIM/PG per ECO (timing caps drifted — separate finding), CR2 clamp orientation correct (pin 1 = cathode per Toshiba datasheet), INA230 across R72 2 mohm at the pack terminal, -BATT eliminated (J2.5 e-match return -> PYRO_GND -> GND). CHANGE 5: U22 ON defaults off (R68). CHANGE 6: U11 NAND on +3V3 with R31/R33 pulls to +3V3; U12 MRAM deleted outright (S3RH2 embedded-PSRAM variant confirmed as the active Espressif P/N replacing EOL S3R2); QOD tied to VOUT; inter-MCU SPI residual stands (separate finding). CHANGE 7: C12 EKYC160ELL103MM25S 10 mF + single R20 150 ohm 1206 (CRCW-HP, 0.47 W peak vs 0.75 W rating). VDD_SPI open item from memory: no series R fitted, C26 100 nF + C27 10 uF on OUT_VDD_SPI; W25Q128 worst-case droop through the S3's internal ~14 ohm path is ~0.3 V -> ~3.0 V at the flash, above its 2.7 V min (the 10 uF slightly exceeds the guideline's 'avoid excessively large'). Still open from the review checklist: item 1 (TPS2121 hold-up bench test) and item 3 (LVC deglitch — now the top finding).

### I4. ERC/DRC triage: 2 real items (already reported); the rest is symbol hygiene noise plus a 6-layer surprise
*architecture / erc-drc-hygiene — confidence high*

Of the ERC errors: U18 hidden-PVIN (real, reported) and the ISM6HG256_INT2 isolated label (reported) are the only genuine electrical items. The other 45 pin_not_connected are deliberate NCs without X-flags (S1 spare poles, U1 HSD1, U14 antenna dummy pads — correct per Molex 47948-0001 which has 1 feed + 3 mechanical pads, S3 JTAG/UART0/unused GPIO, P4 MIPI/DSI/CSI/DM/DP, sensor NC pins); the 13 power_pin_not_driven and 11+2 pin_to_pin/pin_not_driven all stem from regulator symbols lacking power-output pin types and sensor symbols with doubled ground pins. Y1/Y3 have a lib_symbol_mismatch vs the Custom library (edited in place — resync). DRC: courtyard overlap J6/FID1, one 0.13 mm dangling PYRO3_FIRE stub on In3.Cu, silk cosmetics. Note for the review pipeline: the board is SIX copper layers (In1-In4 in the stack); the task brief said 4-layer and only F/In1/In2/B SVGs were generated, so In3/In4 have not been visually reviewed by anyone working from those artifacts.

### I5. All three status LEDs are driven at ~120 uA (10 k series at 3.3 V) — near-invisible in daylight
*architecture / indicators — confidence high*

D6 (green, always-on from +3V3 via R65 10 k), D7/D8 (blue/red from P4 GPIO27/26 via R66/R70 10 k) each get (3.3-Vf)/10 k = 0.1-0.13 mA. 0402 LEDs are visible indoors at this current but effectively invisible in sunlight at a launch field. If this is a deliberate light-sleep power choice (matches the #519 ~1 mA S3 budget), keep D6 as-is but consider 1-2 k for the P4-controlled pair since they only burn power when firmware lights them. Note the nets are named IND_1/IND_2 — do not confuse with the P4's USB1P1 alternate function on those pins (GPIO26/27, verified plain-GPIO use is fine).

### I6. Rail budgets check out: +3V3 ~0.7 A worst-case vs 1 A buck; P4 domain <1 A vs 2 A switch; TLV62569 matches Espressif's P4 external-DCDC reference
*architecture / rails/budget — confidence medium*

+3V3 (TPS62152, 1 A, ILIMF 1.4-2.2 A): S3 RF burst ~0.3-0.35 A + NAND program ~25 mA + P4 domain via U22 (~0.25-0.4 A incl. TLV62569 input for VDD_HP, P4 IO/LDOs, U16 flash, sensors) + buzzer 90 mA worst-case-coincident ~0.7 A — inside rating with ~30% margin; camera no longer loads this rail (the V8 brownout mechanism is structurally gone). V_MCU_SWTCH through TPS22918 (2 A, 53 mohm): ~0.6 A max, CT 2.2 nF slews ~ms so switch-on inrush into ~91 uF is ~0.1 A. P4 core: EN_DCDC (P4 output pin 79) -> U20 EN, FB_DCDC (pin 78) -> U20 FB with no external divider — verified as the documented Espressif external-DCDC scheme (P4 closes the loop internally); TLV62569 = 2 A, L8 2.2 uH per reference. TPS2121: ILIM R64 18.7 k = 5.2 A typ (effectively no limit for a <=1 A branch, inrush of C56 330 uF is soft-started by SS C52 1 uF); OV1 trips USB at 6.35 V, OV2 trips battery at 9.54 V, PR1 gives USB priority above 2.12 V VBUS, CP2 arms fast 5 us switchover for any battery >4.25 V — all sane. Startup: TPS62152 SS 390 pF (~0.1-0.4 ms) will briefly current-limit charging the 91 uF +3V3 bank; harmless.

### I7. USB-C and programming path verified: both MCUs reachable via FSUSB63 mux and slide switch; no reset buttons; no dedicated debug header
*connectors / USB/debug — confidence high*

J6 USB4110-GF-A (3 A class): CC1/CC2 have independent 5.11k pulldowns (R41/R47, correct UFP), VBUS -> TPS2121 IN1 (U21.7) with PR1 (R62 5.11k) and OV1 (R51 49.9k) dividers, SP0503BAHTG ESD array across VBUS/D+/D- (CR3), SBU1/2 NC, shell + A1/B12/B1/A12 grounded, A/B row D+/D- correctly paralleled. FSUSB63 truth table (SEL1,SEL0): 01=HSD1, 10=HSD2, 11=HSD3; board straps SEL1=1 (R1 100k to 3V3) and S1 toggles SEL0 (R2 100k pull-up / short to GND), so the two reachable states are exactly HSD3 = S3 GPIO19/20 via 22R R39/R38 (silk 'O' = OC) and HSD2 = P4 GPIO24/25 (silk 'F' = FC); unused HSD1 is unreachable -- no dead switch position. Boot buttons: S2 -> S3 GPIO0, S3 -> P4 GPIO35 via R54 1k (P4's documented boot strap) with R53 100k pull-up; GPIO36 pulled up (R50) as esptool requires. No EN/reset buttons: entering download mode requires a power cycle with the button held. P4 UART0 console (GPIO37/38) is incidentally available on J3 pins 11/12.

### I8. Battery polarity and fleet consistency verified: J7 pin1=GND, pin2=+, silk '- Batt +' matches pads; no other PH-2 battery port exists in the fleet
*connectors / connectors/battery polarity — confidence high*

Computed pad positions (pad1 GND at x=86.44, pad2 VBAT at x=84.44, back view mirrors x) match the back-silk '- Batt +' orientation in render_bottom.png. The base-station uses 18650 holders (BT2) plus a 3-pin balance connector (J4: GND/BatteryMid/BatteryPos), so no fleet cable can cross-polarize between boards. Caveats: commercial PH-2 pack pigtails vary in polarity, and there is no reverse-input protection -- a reversed pigtail forward-biases CR2 (CUS10S30, 1 A) across the pack and puts negative voltage on the TPS259824 IN pins; the polarized housing is the only guard.

### I9. GNSS link pin-for-pin verified against both daughterboards; board-side has no series resistors or level shifting; deaf-UART hardware hypothesis ranked
*connectors / cross-board/GNSS — confidence high*

Rocket J1 (BM05B-SRSS-TB): 1=GNSS_RX->P4 GPIO4 (P4's TX), 2=GNSS_TX->GPIO3 (P4's RX), 3=FL1(BLM18PG471SN1D, 1 A)->VBATT, 4=switched return (Q1), 5=GNSS_RXD2->GPIO2, tabs 6/7=GND. Both daughterboards match 1:1: sam10m8-hv J3 and px1105r J4 both have 1=GNSS_RX->module RXD, 2=GNSS_TX->module TXD, 3=VSYS (TPS62913, 17 V max -- handles 8.4 V), 4=VSS, 5=RXD2 (wired to module RXD2 on the PX board, NC on the SAM board). No off-by-one, no crossover error, power-on-power. Deaf-UART contribution: with everything powered, module ground offset across Q1 is ~2 mV (100 mA x ~20 mOhm) -- not a live fault cause; but the off-state float (see the low-side switching finding) reverse-stresses the module's RXD input against its floating ground at every GPS_ACT cycle and at every board boot, a credible cumulative damage mechanism for a module whose RXD has gone deaf while TXD still works. Note J1.3 carries raw pack voltage: any legacy 3.3 V-input GNSS pod on this connector family would be destroyed -- label the port '8V4'.

### I10. Mounting holes: 8x M2 (2.2 mm drill, 3.8 mm top/bottom pads), PCB-only footprints, electrically isolated
*connectors / mechanical — confidence high*

H1-H8 exist only in the PCB (not in any schematic sheet) and their pads carry no net -- isolated from GND/pyro/battery, safe for metal standoffs; no chassis-ground provision (fine for an airframe). 2.2 mm is the close-fit M2 clearance; DIN965 countersunk head clears the 3.8 mm annular pads.

### I11. Pyro terminal block J2 verified: TBLH11-350-05BK rated 10 A UL / 17.5 A IEC, 24-16 AWG -- adequate for 8 A fire pulses
*connectors / pyro/J2 — confidence high*

Each of the 5 positions has two 1.6x4 mm SMT tabs; channels PYRO1-4_EXT each connect dual tabs plus the 49.9k/100k continuity divider and the TPN4R712MD high-side FET drains; position 5 is the common return (see the 'GND' label finding). Bottom-silk channel numbering GND/4/3/2/1 matches net assignment in back view. Being SMT, pyro leads should be strain-relieved -- wire tug works against solder tabs only.

### I12. DRC parity warning on U9 pad 9 is benign: TI CSD16323Q3 pinout matches the Toshiba TSON-Advance footprint; drain lands on pads 5-8 plus the EP numbered '5'
*connectors / pyro/U9 footprint parity — confidence high*

The schematic symbol for U9 has drain pins 5-9 (TI numbers the thermal pad 9); the placed Toshiba 'TSON Advance_TOS' footprint numbers its exposed pad '5', so symbol pin 9 finds no pad -- the sole schematic_parity item in drc.json. Electrically PYRO_GND connects via pads 5-8 and the EP. Pin functions verified from both datasheets: CSD16323Q3 = S1,S2,S3,G4,D5-8+pad; TPN4R712MD TSON Advance = same arrangement -- no gate/source swap risk. This is NOT a repeat of the lora missing-ground bug, but the numbering mismatch should be tidied so real parity errors stay visible.

### I13. Board is 6-layer (In1-In4 present), not 4-layer as briefed, and the pre-generated layer SVGs omit In3/In4
*esp32-p4 / Documentation / review artifacts — confidence high*

The kicad_pcb enables six copper layers (F, In1, In2, In3, In4, B) with 784 objects referencing In3.Cu; the review brief said 4-layer and the artifact set only exported F/In1/In2/B SVGs, so any purely-visual inner-layer review missed In3/In4. Also two um-scale dangling track stubs (PYRO1_CONT, PYRO3_FIRE) sit on In3.Cu — cosmetic, but they show In3 carries real routing.

### I14. ESP_SDO lands on S3 strap GPIO3; cross-domain SPI tristate discipline from power-eco.md Change 6 still required
*esp32-p4 / Inter-MCU bus — confidence high*

The inter-MCU SPI is push-pull with no pull-ups and bridges the always-on S3 to the switched P4 domain: the ECO's firmware rule (hold ESP_CS/SCLK/SDO/SDI low/hi-Z until V_MCU_SWTCH is up) still applies unchanged. Additionally P4's ESP_SDO drives S3 GPIO3, which is the S3's JTAG-source strap — benign in the default eFuse configuration and the P4 is off during S3 boot anyway, but worth remembering if JTAG-related eFuses are ever burned on the S3.

### I15. No series 0-ohm provisions on the QSPI flash lines (guideline suggestion) and flash sits ~16 mm from the P4
*esp32-p4 / P4 external flash — confidence high*

Espressif's checklist suggests reserving series zero-ohm footprints on the SPI lines for signal-integrity tuning. U16 is ~16 mm from U17 (routed, DRC-clean); wiring, CS pull-up (R46 10k to VDDO_FLASH — correct power domain) and supply (VO1) are all per reference. Purely an opportunistic tuning provision for the next spin.

### I16. Molex 47948-0001 is an on-ground antenna — full copper beneath it is by design and verified present; feed is ~50 ohm; plan a bench match-tune since the ground plane is smaller than Molex's reference
*esp32-s3 / S3 RF / antenna — confidence high*

Initial suspicion of a missing antenna keepout is REFUTED by the Molex datasheet: 47948-0001 is the LDS MID 'on-ground' monopole — 'Ground Clearance: None needed', 'No removal of ground layers from beneath the antenna is needed'; 3 of its 4 pads are dummy mechanical pads (soldered, electrically NC — matching the netlist's unconnected pads 1-3) and pad 4 is the feed. Measured board state matches: GND fill under the antenna body on F/In1/In3/In4/B (In1 ~solid 220/225 sample points) plus the In2 +3V3 plane. Feed: pad 4 -> 0.127 mm F.Cu trace over the 0.1 mm prepreg to In1 GND = ~50 ohm microstrip (with coplanar pour slightly lower) — matches the 50 ohm input spec; matching network L1 4.3 nH shunt + C23 1.5 pF shunt at the antenna, L2 2.7 nH series + C24 1.5 pF shunt at LNA_IN, within Espressif's CLC windows and placed 1.4-2.6 mm from their nodes. Caveats: the datasheet reference platform is 100x40 mm with full rear ground; this board is 22.4x75 mm, so resonance/efficiency will deviate — the discrete match pads exist precisely so it can be tuned; keep the VNA tune on the bring-up checklist. Nothing tall within 2.5 mm on the top side; board edge 2.3 mm away; antenna temp range -35~+85 C.

### I17. NAND changed from the documented Fudan F35SQB004G (4 Gb) to GigaDevice GD5F2GQ5UEYIGR (2 Gb) — pin-verified OK against the placed footprint, but capacity halved and 85 C grade
*esp32-s3 / S3 log memory — confidence high*

The schematic-review/ECO open item named F35SQB004G; the current design has U11 = GD5F2GQ5UEYIGR (decode per GigaDevice: U = 3.3 V 2.7-3.6 V, Y = WSON8 8x6 mm, I = industrial -40~+85 C, G = green, R = reel). Pin-by-pin vs the GigaDevice datasheet: 1 CS# -> M_FLASH_CS (U15.41/GPIO36), 2 SO -> M_MISO (GPIO35), 3 WP# -> R33 10k -> +3V3, 4 VSS -> GND, 5 SI -> M_MOSI (GPIO38), 6 SCLK -> M_SCK (GPIO37), 7 HOLD# -> R31 10k -> +3V3, 8 VCC -> +3V3, pad 9 -> GND — all correct; WP#/HOLD# pulls and VCC are on the same always-on +3V3 rail (ECO Change 6 intent preserved under new refs R31/R33; the old R43-R46 designators now belong to unrelated parts). Footprint check: pads 0.86x0.43 at 1.27 mm pitch, centers +/-3.924 mm, EP 3.45x4.34 vs package L 0.45-0.55/b 0.35-0.45/EP 3.4x4.3 nom — fits. GPIO35-38 are legal on the RH2 (the GPIO35-37 restriction applies only to octal-PSRAM variants). Two factual notes for the maintainer: capacity is 2 Gb (=256 MB) vs the documented 4 Gb part, and the I-grade is 85 C while the S3 was deliberately upped to the 105 C RH2. Internal ECC (<=4 bit) is on by default.

### I18. MRAM deletion fully implemented: no U12/MRAM in schematic, BOM, or PCB
*esp32-s3 / S3 memories (MRAM) — confidence high*

The recorded decision (delete MRAM U12, move to embedded-PSRAM S3 variant) is complete in all three artifacts: the netlist has no MRAM part and no MRAM_CS net; the fresh BOM jumps U11 -> U13; the PCB has no U12 footprint (allrefs sweep). Its replacement — the RH2's 2 MB in-package PSRAM — is correctly provisioned (see the VDD_SPI finding for the one remaining electrical action). The former M_ bus now serves only the NAND.

### I19. USB mux logic verified end-to-end: slide switch S1 selects S3 (default) or P4 on the connector; ESD at connector; all states legal
*esp32-s3 / USB — confidence high*

U1 = FSUSB63UMX (3:1 HS USB mux, VCC 2.7-4.4 V — fed +3V3 via R3 10 ohm + C1 100 nF). Truth table (datasheet): 00 = all open, 01 = HSD1, 10 = HSD2, 11 = HSD3. Board: SEL1 = R1 100k to +3V3 (fixed 1), SEL0 = R2 100k to +3V3 with slide switch S1 (common pad 6) to GND (pad 7). Switch open -> SEL=11 -> HSD3 = OUT_D+/- -> 22 ohm R38/R39 -> S3 GPIO19/20 (native FS USB); closed -> SEL=10 -> HSD2 = CEN_D+/- -> P4 GPIO24/25. HSD1 is unconnected and unreachable (SEL1 never 0); SEL inputs never float (satisfies the datasheet's 'control input must not float'); 126 us break-before-make forces re-enumeration on switch — all sound. The silk 'O'/'F' by S1 matches OC(S3)/FC(P4) selection. ESD: CR3 SP0503BAHTG sits on D+, D-, VBUS at the connector side of the mux — right place topologically, though it hangs on ~8 mm stubs off the D+/D- runs (J6 at x=84.4, CR3 at x=92.1, mux to the left at x=78); acceptable because both downstream PHYs are Full-Speed only (the P4's HS DM/DP pins 49/50 are unconnected). Note for the maintainer: when the board is off, +3V3 rises from USB via the TPS2121, so the mux is always alive when a cable is in.

### I20. Board file is 6-layer (F/In1-In4/B), not 4-layer; review artifacts only exported In1/In2 — In3 carries real signal routing (USB) that other reviewers' SVGs missed
*esp32-s3 / board/stackup (cross-scope) — confidence high*

The kicad_pcb layer table and physical stackup define six copper layers: F.Cu / In1 (GND) / In2 (+3V3 + V_MCU_SWTCH + VBATT planes) / In3 (signals: D+, CEN_D+/-, OUT_D+/- verified routed there) / In4 (GND) / B.Cu, total ~1.58 mm. The task brief said 4-layer, and the pre-generated artifacts include only layer_In1.Cu.svg and layer_In2.Cu.svg — any reviewer reasoning from those SVGs has a blind spot on In3/In4. Flagging for the orchestrator; also confirm the fab order is quoted as 6-layer.

### I21. Designators renumbered since the July-12 review docs — mapping table for reconciling schematic-review.md / power-eco.md against the current design
*esp32-s3 / documentation — confidence high*

Every reference in the two July-12 docs is stale. Verified current mapping (old -> new): S3 U16 -> U15; P4 U18 -> U17; NAND U11 -> U11 (unchanged ref, changed part); MRAM U12 -> deleted; S3 flash U13 -> U13 (still S3 flash); P4 flash U17 -> U16; USB mux U3 -> U1; antenna AT1 -> U14; TPS22918 U4 -> U22; TPS2121 U2 -> U21; TPS62152 U10 -> U18; eFuse U1 -> U19; INA230 U8 -> U23; NAND WP/HOLD pulls R43-R46 -> R31/R33; inter-MCU I2C pulls R57/R58 -> R55/R58 (on V_MCU_SWTCH, the renamed VPP — still dies with the P4 domain per ECO Change 6); pyro cap C13 -> C12. The VPP rail is now named V_MCU_SWTCH. Inter-MCU map for the record: ESP_CS/SCLK/SDO/SDI/SDA/SCL = S3 GPIO1/2/3/4/5/6 <-> P4 GPIO18/21/19/20/23/22, each net exactly the two MCUs (+5.11 k pulls on I2C only) — SPI has no pulls, matching the ECO's firmware-tristate note.

### I22. Board is 6-layer (not 4 as briefed); In3 is the main inner routing layer and is missing from the review artifact SVGs; minor cosmetic DRC leftovers
*layout-decoupling / board/meta — confidence high*

Stackup: F/In1(GND)/In2(split power: +3V3, V_MCU_SWTCH, VBATT)/In3(747 segments - more than B.Cu)/In4(GND)/B, total ~1.58mm. The pre-generated layer SVGs cover only F/In1/In2/B, so In3/In4 routing was reviewed via file parsing, not visually. Cosmetic: two dangling In3 stubs (PYRO1_CONT 5.7e-6mm, PYRO3_FIRE 0.13mm) and two identical stacked vias on Net-(U21-OV1) at (82.04,135.95) - delete one; 11 silk-over-copper and 5 silk-edge clips are cleanup-grade. Placement/access verdicts requested in scope: all connectors are pluggable with the can and antenna in place (J2 pyro terminal 14mm from the can, J4/J5/J7/J1/J3 clear, USB-C on the opposite face); silk labeling is good (pyro 'GND 4 3 2 1', 'Batt + -', LoRa/CAM/GPS, O/F power switch); sensor pin-1 dots visible on U2/U3/U4; U2 IMU is mounted at 45 deg (firmware axis mapping must reflect this); the Molex 47948-0001 antenna is an on-ground LDS part ('Ground Clearance: None needed') so the copper under it is correct by design - initial concern retracted after datasheet check.

### I23. Chip-down P4/S3 per-pin decoupling is generally well placed; full audit table produced (script left in scratchpad)
*layout-decoupling / cpu/decoupling-survey — confidence high*

Worst-case items that did NOT rise to findings: P4 VDD_HP_2 (pin 76) 100nF at 3.33mm (others 1.3-1.7mm; 10uF C70 at 1.34mm + 2x10uF at the U20 buck); VDDO_4 has only its 1uF LDO-output cap at 3.63mm (VO4 rail is otherwise unloaded); P4 VDDA/VBAT share C63 100nF at ~2.5mm; S3 VDD3P3_RTC 100nF at 2.83mm; S3 VDD_SPI: C27 10uF 2.36mm from pin 29 + C26 100nF 1.32mm from the U13 flash it feeds (opposite side, via-coupled). Sensors vs datasheets: IIS2MDC per ST ('100nF ceramic, 10uF aluminum... as near as possible to pin 9'): 10uF 1.62mm + 100nF 1.82mm, C1 220nF at 1.03mm - compliant; BMP585 VDDIO 10uF at 1.20mm but VDD's nearest cap is 2.94mm (shared with the mag's C4); ISM6HG256X 100nF at 2.83mm with 10uF at 2.2mm ('as near as possible' - acceptable). GND return quality is excellent: solid GND on F/In1/In3/In4/B under both MCUs. S3-R2 migration state (factual): U15 = ESP32-S3RH2, no MRAM footprint on board, OUT_VDD_SPI has C26 100nF + C27 10uF and NO series resistor (the ECO's 14-ohm budget item remains unaddressed as a schematic choice, not a layout defect).

### I24. ECO hold-up implemented and electrically sound: C56 330uF on V_MCU_2S, 9.5mm from U21 OUT via wide pours, adjacent to the buck feed; ceramic partner is 1uF not the ECO's 10uF
*layout-decoupling / power/ECO-verification — confidence high*

CHANGE 1 verification: C56 (TCJE337M016R0050, +pad on V_MCU_2S, -pad GND - polarity nets correct, KEMET silk bar present) sits at (91.36,137.03) F.Cu; U21 OUT pads are (81.9,137.4) B.Cu, ~9.5mm away, but the net is carried by dedicated F.Cu and B.Cu pours plus 7 vias (no thin traces - zero track segments on the net), and C56 is only 2.9mm from L5, the series 2.2uH feeding the buck input (C43 22uF). For ms-scale hold-up the energy path C56->L5->C43 is short and wide; 'directly across VCC-GND at U2 OUT' is met electrically if not literally. Deviation: the ECO's '|| 10uF X7R' became C59 1uF. eFuse CIN/COUT (CHANGE 2) placement verified tight: C41 1uF 1.0-1.5mm from IN pins, C42 100nF 1.04mm, C54 1uF 1.01mm from OUT; ITIMER/RETRY/NRETRY/ILIM support parts 0.7-1.9mm per TI layout note (DVDT cap is opposite-side through vias - harmless on that pin). U22 QOD is tied to VOUT (internal RPD) = ECO Change 6 item 4 done. Pyro V_CAP: fire-FET sources see 22uF ceramics (C9/C10/C11/C13) at 2.2-3.8mm - fine for ms fire pulses.

### I25. ECO/BOM state observations: MRAM deletion implemented (S3RH2), NAND is now a third part, eFuse value/footprint name mismatch, BOM has no MPN column data
*layout-misc / bom/eco-state — confidence high*

Factual state for the orchestrator: (1) MRAM U12 no longer exists and U15's value is ESP32-S3RH2 — the MRAM->R2-PSRAM decision IS implemented; C27 10 uF sits near OUT_VDD_SPI; no ~14 ohm VDD_SPI series resistor exists in the BOM (values scanned) — confirm that open ECO item was resolved as 'no resistor'. (2) The placed NAND U11 is GD5F2GQ5UEYIGR — neither the original MX35UF4G24AD nor the F35SQB004G the review recorded; schematic-review item 5 ('confirm placed NAND footprint/pinout') applies to yet another part now. (3) U19's value is TPS259824LNRGET inside a footprint named IC_TPS259827LNRGER — same RGE-24 package so buildable, but the name mismatch invites a wrong-variant order; power-eco.md documents the '24' (16.9 V OVLO) suffix as deliberate. (4) bom-fresh.csv has an MPN column that is empty for essentially every line and a DNP column that is empty everywhere (no DNP parts) — quoting/kitting will need MPNs from bom-priced.xlsx or the schematic fields. (5) DRC config ignores missing_courtyard, pth_inside_courtyard, footprint_type_mismatch etc. — C12's missing courtyard (finding above) slips through because of this.

### I26. Outline, holes, edge clearance and access all check out; bottom stack dominated by C12 can and 11.2 mm terminal block
*layout-misc / outline/mechanical — confidence high*

Outline: 22.35 x 75.00 mm rectangle, 1.0 mm corner radii, plus two internal 8.0 x 3.0 mm slots (0.5 mm corner radii) at mid-board (tie-wrap/harness pass-throughs) — all routable geometry, no acute slivers. 8x M2 mounting holes (2.2 mm drill, 2.6 mm pad, 3.8 mm top+bottom annular pads) in two columns at x=75.22/91.97. Copper-to-edge: board rule min_copper_edge_clearance=0.2 mm and DRC passes; several tracks sit right at 0.2 mm (zero margin over a typical fab minimum, fine for JLC-class fabs). USB-C (J6) is on the bottom short edge, accessible end-on when sled-mounted; J2 pyro screw-terminal wire entries overhang that same edge (intended). Tallest top parts: U14 antenna 4.0 mm, USB-C ~3.2, buzzer ~3; bottom: C12 can 25 mm (dominant), J2 TBLH11 11.2 mm tall, J3 Milli-Grid header, JST PH/SH shells. Lib tables are ${KIPRJMOD}-relative and resolve inside the repo (footprints/Footprints.pretty, symbols/Custom.kicad_sym, 3dmodels/ all present); no absolute paths.

### I27. Vendor-style poly mask/paste verified correct on all chip-down parts; P4 is 0.35 mm pitch — specify <=0.10 mm stencil and expect gang mask openings
*layout-misc / paste/mask — confidence high*

U17/U15/U21/U23/Q1/Q7-Q10 draw mask and paste as fp_poly graphics instead of pad layers; every mask-less pad was verified covered by an aligned mask poly (104/104 on U17, 56/56 on U15, all pads on the five PMPB14XN FETs, 12/12 on U21, 22 polys on U23), so the initial 'pads have no mask aperture' alarm is refuted. EPAD paste segmentation is present and sane: U17 40% (9x 1.58 mm windows), U15 54%, U18 40%, U23 40%, U19 79% (current-carrying, intentionally high). Perimeter pad style is NSMD (0.28 mm openings over 0.18 mm pads). Two process notes: (a) the P4 is 0.35 mm pitch (brief said 0.4), so mask dams between pin openings are only ~0.07 mm — below typical 0.08-0.10 mm minimums; the fab will gang-open them, which is normal for this package but should be expected, not queried; (b) paste aperture area ratio for the 0.18x0.65 mm pins is 0.70 with a 100 um foil but drops to ~0.54 at 130 um — specify 100 um (4 mil) or thinner laser-cut stencil.

### I28. Copper under the 2.4 GHz antenna is CORRECT — Molex 47948 is an on-ground antenna (concern checked and refuted)
*layout-misc / rf/antenna — confidence high*

U14 (Molex 479480001, top side at 74.64,134.52, 3x3x4 mm) sits over GND fill on F/B/In1/In3/In4 and the +3V3 In2 plane (53-96% coverage under the body, measured by point-in-polygon sampling). The Molex 47948 datasheet explicitly states 'Ground Clearance: None needed' / 'No removal of ground layers from beneath the antenna is needed' — the intact planes are per design. Feed is pad 4 into the L1/L2/L4/C23/C24 matching network from the S3 LNA_IN. No finding; recorded so nobody 'fixes' it later. Note the 4.0 mm body height is the tallest top-side part.

### I29. Board is SIX copper layers, not four — and the review artifact set omits In3/In4 SVGs
*layout-misc / stackup — confidence high*

The stackup is F.Cu / In1 / In2 / In3 / In4 / B.Cu, 1.6 mm total (0.535 mm cores, 0.1 mm prepregs). In1 and In4 are solid GND, In2 carries +3V3 and VBATT plane islands, In3 is a real signal layer with 747 track segments (1.49 m of routing, including pyro FIRE/CONT nets). The task brief said '4-layer board' and the pre-generated SVG exports cover only F/In1/In2/B — so no visual pass was possible on In3/In4 from the artifacts (geometric checks were done from the .kicad_pcb instead). Matters for fab quoting (6-layer price) and for any future visual diff workflow.

### I30. Stackup is 6-layer (not 4): In1+In4 solid GND, In2 split power plane, In3 routing - structure is sound; review artifacts miss In3/In4
*layout-power / gnd-planes — confidence high*

rocket-computer.kicad_pcb declares 6 copper layers, all 0.035 mm (1 oz), total 1.6 mm. In1 and In4 are unbroken full-board GND (1437 mm2 each, single polygon); In2 carries the power planes (+3V3 860 mm2, V_MCU_SWTCH 287 mm2, VBATT 166 mm2); In3 has 747 routed segments + fragmented GND fill; 170 GND stitching vias, all through-hole 0.4/0.3. Every signal layer is adjacent to at least one solid GND plane, so return paths for servo and pyro loops are continuous - the servo return (Q8 -> planes -> J7.1) and pyro return never cross a split. The high-current concern on GND is only the via counts at the Q8 (5) and J7.1 (7) transitions, covered in the servo finding. Note for the orchestrator: the task brief said '4-layer' and the pre-generated SVGs cover only In1/In2 - In3/In4 content is not in the artifact set (analysis here used the parsed board directly).

### I31. Protected logic chain verified: mux -> 330 uF hold-up -> filter -> buck all correctly sized and placed
*layout-power / logic-power-chain — confidence high*

U21 (TPS2121: IN2=VBATT, IN1=USB VBUS with PR1/OV dividers) -> V_MCU_2S (19.5+7.9 mm2 zones, 7 vias, necks 0.6 mm - fine for ~1 A) with C56 330 uF hold-up at the mux output exactly as ECO Change 1 specifies -> L5 (VLS3012CX 2.2 uH, 74 mOhm, carries ~0.6 A vs 2.83 A rating; the LC input filter it forms with C43 has Z0 ~0.32 Ohm << buck negative input impedance ~15 Ohm, stable) -> U18 -> +3V3 In2 plane (860 mm2, 18 vias). U22 (TPS22918) ON has the R68 100 k pulldown to GND so the P4 domain defaults OFF at power-up - C-4 interlock confirmed in this netlist. V_MCU_SWTCH necks 0.81-0.85 mm to U17/U20 - adequate for the P4 domain's ~1 A. eFuse thermal: 5 VBAT_CON vias in the IN pad + 3 GND vias at the GND pad, ~20 mm2 local spread; ~0.6 W at the 14.7 A limit is fine, though smaller than TI's recommended array. S3 3V3 pin stubs are 0.1-0.127 mm off the plane - normal for pin escapes with local decoupling. FL1/FL2 (BLM18PG471SN1D, 1 A, 0.2 Ohm) suit GNSS ~0.1 A and LoRa TX ~0.6-0.7 A (0.36 mm neck at FL2 is adequate). Peripheral switching is LOW-side (Q1/Q7/Q8/Q10 in each return) - works for power gating, but an off peripheral's local ground floats toward VBATT, so P4/S3 UART lines into an off camera/GNSS/LoRa will back-drive through I/O clamps (same class as ECO Change 6): one for the schematic owner, noted here for the record.

### I32. Pyro copper and return path verified adequate - energy is i2t-bounded by the 10 mF cap
*layout-power / pyro — confidence high*

Fire loop: C12 (10 mF EKYC, THT) -> V_CAP B.Cu zones (45+35 mm2, neck 0.64 mm to FET sources) -> U6/U7/U8/U10 (TPN4R712MD) -> PYROx_EXT 1.5 mm traces + 24-39 mm2 zones (necks 1.8-2.7 mm) -> J2 (TBLH11-350-05, 3.5 mm screw terminal, ~10 A class, dual pads per circuit) -> match -> J2.5 -> PYRO_GND 1.5 mm trace (neck 0.95 mm) -> U9 (CSD16323Q3 low-side arm, 4.4 mOhm at 4.5 V, 20 A silicon limit) -> GND planes -> back to C12- . Total available charge is capped at ~84 mC (10 mF x 8.4 V), so worst-case adiabatic heating at the 0.64 mm neck is under ~10 C per fire - all necks pass. The loop closes cap-to-FET within the pyro end of the board (y=140-168); the ECO's 'star at battery negative' is superseded by this cap-local return, which is electrically better. Sensors (U2/U3/U4 at y=99-100) are ~50 mm from the firing loop; the S3 (y=145) borders it but sits on solid In1/In4 planes with 7 GND vias at U9 and 16 within 6 mm of C12-, so plane gradients are mV-for-ms scale. Arm topology note for the record: arming is now a low-side master switch (U9) in the shared e-match return, with the 4 high-side FETs per channel; DTC123J pre-drivers and R22 gate pulldown keep it off at boot.

### I33. Antenna implementation verified GOOD: on-ground type needs NO keepout, dummy pads correctly unconnected, In1 solid, match per Espressif
*layout-rf / RF / antenna (U14) — confidence high*

The review brief's assumed 'required ground keepout' does not apply: Molex 47948-0001 is the ON-GROUND MID antenna — official literature states 'Ground Clearance: None needed' and 'No removal of ground layers from beneath the antenna is needed', with bottom-side component placement under it explicitly part of the concept (U19 eFuse on B.Cu 2.5 mm away is acceptable). Pads: 'four SMD pads... Three dummy pads are to be attached to the PCB for strong mechanical bonding while the remaining signal SMD pad connects to the radio' — so unconnected-(U14-Pad1..3) + pad 4 = feed is correct as built. Verified: In1 GND 100% solid under body (0/1600 sample holes) and under the entire feed path; F.Cu match chain LNA_IN -> C24 1.5 pF shunt (on a 1.4 mm stub at the L2 junction, 3.4 mm from pin 1) -> L2 2.7 nH series -> C23 1.5 pF shunt + L1 4.3 nH DC-ground shunt at the feed — topology and every value inside Espressif's recommended CLC windows (C 1.2-1.8 pF, L 2.0-3.0 nH); parts are 0402 (Espressif suggests 0201; fine at 2.4 GHz). Antenna is 0.78 mm from the left board edge at mid-edge (reference board uses a corner; mid-edge acceptable for this type). 3D neighbors: C12 electrolytic can is bottom-side with body >=8.4 mm away (courtyard x76.79-84.90, y144.35-148.25); buzzer LS1 22 mm; battery connector J7 ~10 mm east — keep pack wiring dressed away from the left edge top face in the bay. No second antenna exists (V8 issue resolved). No LC harmonic filter after the pi network (Espressif optional recommendation) — acceptable for BLE-only, note for EMC. Board ref is U14, not 'AT1' as in the brief/schematic docs.

### I34. Crystal under-layer audit clean: nothing routes on In1 anywhere; all four crystals sit over 99-100% solid GND
*layout-rf / crystals — under-layer audit — confidence high*

Direct answer to the brief's In1-under-crystal question: In1 carries zero routed traces board-wide, and its GND fill is 100%/99.2%/98.8%/100% solid under Y4/Y2/Y3/Y1 regions respectively. F.Cu under each crystal body carries only that crystal's own nets. Foreign copper near crystals exists only >=2 solid planes away: SERVO_ACT (slow enable) on B.Cu under Y3/Y4 regions, OUT_D+/- (USB FS) on In3 under the Y1 region, PYRO1_EXT B.Cu pour opposite Y1 — all negligible through In1+In2(+In4). Crystal-to-chip gaps 5.5-9 mm satisfy the >=2 mm rule. 32 kHz pairs verified: S3 Y1 traces 3.95/4.23 mm with 22 pF on the 12.5 pF-CL SC-32S (correct); P4 Y3 lands on GPIO0/GPIO1 which the P4 datasheet pin table confirms are XTAL_32K_N/XTAL_32K_P, with C37/C38 22 pF present (correct).

### I35. Board is 6-layer, not 4 — pre-generated layer SVGs omit In3/In4, hiding the busiest inner routing layer
*layout-rf / review process / stackup — confidence high*

The kicad_pcb stackup is F.Cu / In1 / In2 / In3 / In4 / B.Cu (0.1 mm prepreg outer, 0.535 mm cores, er 4.5, ~1.58 mm total). Layer usage: In1 = pure GND plane (zero routed segments board-wide), In2 = power plane (+3V3 860 mm2, V_MCU_SWTCH 287 mm2, VBATT 166 mm2), In3 = GND pour + 747 routed segments (the pyro/SPI/LoRa/USB ribbon), In4 = pure GND plane. The scratchpad review artifacts include only layer_F/In1/In2/B SVGs — any reviewer working from those images cannot see In3 routing (where findings 2 and several USB runs live) or In4. All 383 vias are through-hole F-to-B.

### I36. Board is 6-layer, not 4-layer as briefed; In3/In4 were not in the review artifacts
*power-input / Board / stackup — confidence high*

rocket-computer.kicad_pcb declares F.Cu, In1-In4, B.Cu. The pre-generated SVGs cover only F/In1/In2/B, so In3/In4 copper was not visually reviewed; DRC shows two dangling pyro track stubs on In3 (PYRO1_CONT 6e-6 mm, PYRO3_FIRE 0.13 mm — cosmetic). Power distribution verified electrically instead: VBATT/VBAT_CON/V_MCU_2S/+3V3/V_MCU_SWTCH are zone-poured (B.Cu + In2), zones use 'reliefs for PTH' so SMD pads connect solid, and DRC reports 0 unconnected items — the only schematic-parity issue is the known U9 pad 9 PYRO_GND.

### I37. ECO document has drifted from the as-built board: full designator remap plus three changed eFuse cap values and the switch topology
*power-input / Documentation (power-eco.md) — confidence high*

The board was re-annotated after the ECO was written. Mapping (ECO -> as-built): eFuse U1->U19, mux U2->U21, INA230 U8->U23, shunt R24->R72 (now 2 mOhm), buck U10->U18, filter L9->L5, hold-up cap ->C56 (+C15 duplicate part on VBATT), pyro cap C13->C12, charge R R69->R20 (now CRCW1206-HP 150R, 0.47 W peak vs 0.66 W rating — good), UVLO R25/R26->R44/R45, ILIM R18->R48, CIN C93/C77->C41/C42, clamp CR3->CR2, P4 switch U4->U22, P4 buck U19->U20. Value drift vs the ECO's IMPLEMENTED record: ITIMER 4.3 nF -> 10 nF (blanking ~2 ms -> 4.7 ms typ, 2.5-9.3 ms range; fine, current-limited during blanking), dVdt 8.2 nF -> 10 nF (SR 0.46 V/ms, ~18 ms ramp, ~185 mA inrush into ~400 uF — fine), NRETRY 560 nF -> 1 uF (see separate finding). Peripheral switching is low-side, not the documented high-side Q9/Q3. Anyone editing from the ECO's designators will touch the wrong parts (today's R24/R25/R27 are FET gate resistors, R69 is an I2C pull-up).

### I38. Power-path element ratings vs computed worst-case currents — everything else checks out
*power-input / Rail budgets (verified numbers) — confidence high*

Verified good: UVLO divider R44 1M/R45 210k -> 6.92 V on / 6.34 V falling (VUVLO 1.2/1.1 V, matches ECO exactly; EN pin sees 1.46 V at 8.4 V, under the 7 V abs max). ILIM R48 100R -> 14.71 A typ. TPS259824L = 16.9 V OVLO + active current limiter per device table; RGE pads: Pad1(25)=IN, Pad2(26)=GND correct; LDSTRT and IMON grounding are the documented unused-state hookups. TPS2121: IN1=USB+TVS, IN2=VBATT; PR1=VBUS/2 (IN1 priority when VBUS>2.12 V), CP2=0.249xVBATT (>VREF 1.06 V always on 2S -> 5 us fast comparator switchover; USB 2.5 V > battery 1.6-2.09 V so USB wins, battery-only falls back to IN2); OV1 rejects USB>6.35 V, OV2 rejects pack>9.54 V; ILIM R64 18.7k -> 5.24 A (65.2/RILM^0.861, RILM within the valid 18-100k window); ST->GND is datasheet-sanctioned; reverse-current blocking (IRCB 0.2-2 A, 10 us) makes the hold-up scheme valid, C56 bleed on RCB trip ~30 mV. TPS62152 confirmed fixed-3.3 V; FB->GND recommended for fixed variants; FSW->VOUT is footnote-sanctioned (1.25 MHz); DEF->GND=nominal; L6 2.2 uH is the datasheet minimum at 1.25 MHz, peak inductor current 1.37-1.46 A < Isat 1.89 A (VLS3012CX-2R2M-1: 2.83 A temp / 1.89 A sat / 74 mOhm). L5 (ex-L9) carries <=~1.05 A worst-case buck input current at VCC=3.5 V dropout — inside both ratings. +3V3 worst-coincident load ~0.85 A of the 1 A converter (S3 TX + P4 domain through U22 + buzzer) — thin but workable; bench-measure. INA230: A1=A0=GND -> 0x40, ALERT float explicitly permitted, BUS on VBAT_CON reads the pack side (works with eFuse off) per ECO; 2 mOhm shunt -> 40 A range, 29 mV at eFuse limit. U22 TPS22918: ON has R68 100k pulldown (P4 defaults OFF — closes review C-4a), QOD strapped to VOUT (VPP discharges when off — ECO Change 6 item 4), CT 2.2 nF -> ~165 mA VPP inrush. Star taps: mux IN2, camera J4.2, servo J3.15/16, pyro R20, GNSS FL1, LoRa FL2 all on VBATT = eFuse OUT (Change 4/5 electrically implemented). CR1 buzzer flyback and CR2 input clamp both correctly oriented (CUS10S30 pin1=cathode); CR2 sees 8.4 V < 20 V continuous VR. Change 6 fix 1 re-verified: NAND U11 VCC on +3V3, MRAM deleted outright (S3 is now ESP32-S3RH2; C27 10 uF + C26 100 nF present on OUT_VDD_SPI feeding U13). eFuse ILIM vs 4-servo stall margin note from the ECO still stands (TPS25983 upgrade path).

### I39. Architecture changed after the July ECO: e-match return is now switched by a low-side arm FET (J2.5 → PYRO_GND → U9 → GND), not tied straight to GND; full designator re-annotation
*pyro / pyro architecture — confidence high*

Current topology (netlist-verified): VBATT → R20 150R → V_CAP (C12 10 mF + C9-C11/C13 22 µF) → four high-side P-ch fire FETs U8/U6/U7/U10 (TPN4R712MD; ch1/2/3/4) → PYROx_EXT → J2 terminals 1-4 → e-match → J2 terminal 5 → PYRO_GND → N-ch arm FET U9 (CSD16323Q3) drain-source → GND, with R73 1k PYRO_GND bleed (keeps continuity sensing functional while disarmed and bleeds the floating return). This supersedes both the July review's 'arm P-FET → PYRO_POS' description and the ECO Change-2 note 'e-match return (J4.5) now goes to GND': the return goes to GND only THROUGH the arm FET. Fire requires fire-FET AND arm-FET on — a genuine series interlock in the energy path, electrically sound (both FET body diodes also clamp match-lead L·di/dt at burn-open: supply-side into V_CAP, return-side below GND at U9). Note the wholesale re-annotation vs all July docs and memory notes: P4=U17 (was U18), S3=U15 ESP32-S3RH2 (MRAM deletion + R2-variant decision IS implemented; no MRAM in BOM), pyro cap C12 (was C1/C13), charge R R20 (was R69), pyro connector J2 5-pos terminal block (was J4), battery J7, fire FETs U6-U8/U10 (was U7/U24-U26). Stale doc references will mislead future ECOs.

### I40. J2 (TBLH11-350-05) is adequately rated but SMT-only anchored; e-match field wiring pulls on solder joints
*pyro / pyro connector — confidence high*

Same Sky/CUI TBLH11-350-05-BK: 5-pole 3.5 mm push-button clamp terminal block, 10 A UL (17.5 A IEC), 300 V, 24-16 AWG, SMT L-leg mount; the placed footprint's 1.6×4.0 mm pads on 13.3 mm row spread match the datasheet's recommended layout, and each pole lands on two pads (both rows) which also matches the part's dual solder tabs. Ratings comfortably exceed the ≈7.6 A decaying fire pulse. Channel labeling '1 2 3 4 GND' is present on B.SilkS next to the block. The only caution is mechanical: SMT terminal blocks with field wiring (matches tugged during prep/recovery) rely entirely on solder-joint peel strength; the DRC also notes the J2 silk clipped by the board edge (cosmetic).

### I41. Continuity sense is digital-only: GPIO7/10/12/14 have no ADC on the P4 (touch-channel pins); sense current ≈65 µA, well below no-fire
*pyro / pyro continuity — confidence high*

Continuity network per channel: V_MCU_SWTCH (3.3 V switched P4 rail) → 49.9k (R8/R10/R12/R18) → PYROx_EXT → match (≈1 Ω) → PYRO_GND → R73 1k (disarmed) or U9 (armed) → GND. Match present: ≈65 µA through the match (≈460× below the 30 mA no-fire line), node reads ≈65 mV = solid logic LOW (P4 VIL 0.25×VDD = 0.825 V); match absent: 3.3 V = HIGH, via R9/R11/R13/R19 100k into U17 GPIO7/10/12/14. Per ESP32-P4 datasheet Table 2-7, those pins carry only TOUCH_CHANNEL analog functions — ADC1 is GPIO16-23, ADC2 is GPIO49-54 — so this is a go/no-go digital read, not a resistance measurement (note PYRO_ARM sits on GPIO16 = ADC1_CH0, an ADC pin spent on a digital output). When a fire FET is on, D1-D4 (BAT54XV2, 30 V/200 mA Schottky) clamp PYROx_CONT to V_MCU_SWTCH+VF with only ≈47 µA through 100k — GPIO-safe; back-injection into the 3.3 V rail via the 49.9k pull-ups is ≈102 µA/channel — negligible. Pull-ups are referenced to the switched P4 rail, so they die with the domain (no sneak arming path when the P4 is held off by the S3/TPS22918 interlock).

### I42. ECO Change-7 case-size warning resolved: EKYC160ELL103MM25S is KYC series Ø18×25 mm, 7.5 mm pitch — placed footprint geometry matches; 25 mm height needs a bay-clearance check
*pyro / pyro energy store — confidence high*

Chemi-Con part page confirms: KYC series, 10000 µF, 16 Vdc, Ø18 mm × L25 mm, ESR 30 mΩ max (20°C/100 kHz), ripple 3350 mArms, 5000 h @105°C. Placed footprint: two Ø2.54 mm THT pads, 1.0 mm drills, 7.5 mm spacing (pads at ±3.75 mm), B.Fab body circle Ø18 — pitch and body match the part (1.0 mm drill suits the standard 0.8 mm lead of an Ø18 radial; lead diameter not stated on the summary page). 16 V ≥ 8.4 V = 53% voltage derating — healthy. The ECO's feared Ø18×40 case does not apply. Remaining mechanical unknown: the can stands 25 mm proud of the BOTTOM face of a 22.5 mm-wide board — bay/airframe clearance must be checked (and see the separate blocker on the parts under the can). Note the ECO calls it 'KY' series; the orderable part is KYC.

### I43. Fire-path currents verified adequate: ≈7.6 A peak per channel (τ≈11 ms), ≈25 A worst-case 4-channel through U9 and its 4× 0.3 mm-drill via return — all within ratings
*pyro / pyro fire path — confidence high*

Single channel: 8.4 V into [match 1 Ω + C12 ESR ≤30 mΩ + TPN4R712MD ≤8.1 mΩ + CSD16323Q3 ≤7.2 mΩ + board copper ≈20-40 mΩ + J2 contacts ≤2×20 mΩ] ≈ 7.6 A peak decaying with τ ≈ 11 ms; energy ½CV² ≈ 0.35 J vs a few mJ all-fire. Ratings: TPN4R712MD IDP −180 A (1 ms) / ID −36 A; CSD16323Q3 IDM 240 A; J2 rated 10 A UL / 17.5 A IEC. All-four simultaneous: ≈25 A total through the common return (J2.5 → 1.5 mm B.Cu trace 12.8 mm → four 0.4/0.3 mm vias → 0.4 mm F.Cu 1.1 mm → U9 on F side): ≈6 A/via for ≈3 ms — adiabatic ΔT < 5 K per via barrel (0.024 mm² Cu), total return resistance ≈ a few mΩ — acceptable, though more/larger vias would be free margin. Fire feeds are 1.5 mm traces into per-channel B.Cu zones reaching both J2 pad rows; the thin 0.127/0.1 mm PYROx_EXT runs (up to 13.3 mm) were traced geometrically and are continuity-sense stubs to the R8-R19/D1-D4 cluster at x≈93 mm, NOT in the fire loop. 200 ms fire pulse: if a match fails to burn open, the cap simply drains to the 56 mA R20 trickle — no component overstress. Sequential-fire note: recharge is τ=1.5 s (≈95% in 4.5 s); two channels fired <~4 s apart start the second from a partially charged cap if the first match stayed conductive — the 22 µF bank + remaining charge still exceed all-fire needs for typical matches, matching the ECO's 3-5-fire budget claim. Vgs re-check (closed C-2 still holds with current values): fire-FET gate is pulled to DTC Vo(on) ≤0.3 V, so Vgs ≈ −8.1 to −8.35 V vs VGSS ±12 V (Toshiba Rev 6.0.A) — ≈3.7 V margin, 2S-only as before; arm gate 3.3 V vs +10/−8 V — fine; gate pull-ups R15-R17/R23 (10k→V_CAP) hold Vgs=0 when off, and the τ=1.5 s V_CAP ramp cannot dV/dt-trigger a 43 µs gate RC.

### I44. V9 re-annotation: sensors are U2/U3/U4 (not U5/U6/U23); S3 is ESP32-S3RH2 and the MRAM is gone
*sensors / documentation/refs — confidence high*

The July-12 schematic-review.md and power-eco.md refer to sensors as U5/U6/U23 and to U12 MRAM, U16 S3, U18 P4, U2 TPS2121, etc. The V9 board re-annotated: U2=ISM6HG256XTR, U3=IIS2MDCTR, U4=BMP585, U23=INA230, U15=ESP32-S3RH2, U17=ESP32-P4NRW32, U19=TPS259824 eFuse, U21=TPS2121, U22=TPS22918. Factual state per the current BOM/netlist: no MRAM part exists and the S3 is the R2 embedded-PSRAM variant - consistent with the recorded MRAM-deletion decision (the VDD_SPI 14-ohm/C27 open item is the P4/S3 agent's scope, not checked here). Risk is only documentation skew during bring-up/debug; the mismatch also means the review docs' designator-keyed instructions must not be applied literally to V9.

### I45. ISM6HG256X INT2 is a labeled single-node net (floating) - datasheet-compliant, but the second interrupt is unavailable
*sensors / sensors/IMU U2 — confidence high*

Net ISM6HG256_INT2 contains only U2.9; ERC flags it as isolated_pin_label (the one sensor-related ERC hit). Datasheet Table 25: INT2 default is 'output forced to ground', so floating is explicitly safe - no tie required. Cost: the high-g channel (or DEN) cannot raise an independent interrupt to the P4; everything must share INT1 (GPIO50). All unused-pin rules on the sensor suite otherwise check out: U2 SDx/SCx tied GND ('Connect to VDDIO or GND'), OCS_Aux/SDO_Aux unconnected ('leave unconnected, soldered to PCB'), U3 NC pins 2/11/12 unconnected ('internally not connected'), U4 pad 9 L/M unconnected ('no external connection possible').

### I46. IMU mounted at 45 degrees, magnetometer at 0 degrees - sensor axes not aligned with each other or the board
*sensors / sensors/axes — confidence high*

U2 (ISM6HG256X) is placed at rot=45.0 on F.Cu at (79.92,99.03); U3 (IIS2MDC) at rot=0.0 at (84.03,99.85); U4 baro has no axes. Both sensors are top-side (no Z flip between them), so the IMU frame is rotated +45 deg about board Z relative to the mag and board axes. Not an electrical fault - but firmware/EKF must carry an exact 45 deg IMU-to-body rotation, and the mag-vs-IMU frames differ, which interacts with the existing mag hard-iron/soft-iron cal pipeline. Flagging purely so the firmware orientation constants (IMU orient flags per #390) get set deliberately.

### I47. Baro top port is unobstructed; add no-coat/no-adhesive fab note and watch the M2 screw head at H3
*sensors / sensors/baro environment + fab notes — confidence high*

Render check: nothing covers U4 on the top side - it sits in open board area near the top edge; the nearby metal-lid part is L8 (buck inductor), not over the baro; daughterboard connectors J5/J3 are on the bottom side. The BMP585 is the gel-filled metal-lid media-robust variant sensing through the top of the lid; datasheet requires avoiding contact with liquids/small particles and any material touching the sensor. It is NOT explicitly a no-clean-only note in DS003, but conformal coating or potting over the lid would kill it. H3 (M2 mounting hole) center is 4.25 mm from U4 center: a DIN965 head (~3.8 mm dia) clears the 3.25 mm body by only ~0.5 mm - a washer or wide standoff would encroach. Also note BMP585 solder guidance: max 3 reflows, >=50 um solder height, and a temporary pressure-offset shift for ~24 h after soldering (do not calibrate immediately after assembly).

### I48. Interface wiring verified correct; one firmware note - disable the IMU's I2C/I3C on the shared SPI bus
*sensors / sensors/interfaces (firmware note) — confidence high*

Verified against datasheets: U2 ISM6HG256X in Mode-1 SPI 4-wire (SCL/SDA/SDO pins 13/14/1 on SENS_SCLK/SDI/SDO, CS pin 12 pulled to V_MCU_SWTCH via R5 10k - idle high correct; max SPI 10 MHz). U4 BMP585 SPI 4-wire (SCX/SDX/SDO pins 1/2/3, CSB pulled up via R4 10k; starts in SPI4 after POR; max SPI 12 MHz - bus can run 12 MHz to the baro but must drop to <=10 MHz for IMU transactions). U3 IIS2MDC in I2C mode: CS pin 3 tied to VDD_IO rail as required, SCL/SDA pulled up 5.11k (R40/R43) to V_MCU_SWTCH - supports fast-mode+ (1 MHz) easily. INT map: BMP585_INT->GPIO42, IIS2MDC INT->GPIO46, ISM INT1->GPIO50; CS: GPIO41 (baro), GPIO49 (IMU); SPI: GPIO51/52/53 = SDO/SDI/SCLK; no net-level conflicts, and none of these are P4 strap pins to my knowledge (not re-verified against the Espressif datasheet - see checks_blocked). All pulls die with the switched rail (ECO Change 6 intent upheld: every sensor net terminates at U17, V_MCU_SWTCH, GND, or NC - no cross-domain sensor line). Firmware note: with CS idle-high the IMU keeps I2C/I3C enabled and pins 13/14 see the baro's SPI traffic, which can occasionally look like I2C starts; set IF_CFG.I2C_I3C_disable=1 at IMU init (ST's standard shared-bus advice). BMP585's warning that interface pins must not be held high while its VDDIO is off is satisfied structurally (host and sensor share the same switched rail).

### I49. LoRa daughterboard power path and the P4 core buck sit directly under/beside the sensor cluster
*sensors / sensors/mag noise environment (layout cross-ref) — confidence high*

Layout distance-to-copper is another agent's scope, but flagging what the schematic+placement shows: on B.Cu directly beneath the sensor cluster sit FL2 (LoRa VBATT feed, 2.6 mm from mag U3 center), Q10 (LoRa low-side switch, 1.7 mm from baro U4), C18 22 uF VBATT (1.16 mm from U4), and J5 (LoRa connector, 1.2 mm from IMU U2); LoRa TX bursts (hundreds of mA) will loop within ~2-3 mm of the magnetometer through the board. On F.Cu, the U20 TLV62569 buck and its shielded inductor L8 are 4.3 mm / ~5 mm from U4 and ~8.6 mm from U3. Given the standing ~1.7 mT hard-iron/cal issue on the current PCB, the mag's magnetic environment on V9 deserves the layout agent's explicit attention. Schematic-side the mag rail itself is fine (V_MCU_SWTCH, same as the other sensors).

### I50. Buzzer (MLT-8530 + Q9 low-side + CR1 flyback) shares the switched sensor rail V_MCU_SWTCH
*sensors / sensors/supply domain — confidence high*

LS1 LOAD+ hangs on V_MCU_SWTCH - the same TPS22918-switched 3.3 V rail feeding all three sensors, the P4 I/O domains and the U20 core buck. A magnetic transducer draws ~60-90 mA square-wave pulses at ~2.7 kHz when sounding, injected into the sensor supply. This matches the ECO's as-built domain table (buzzer listed in the VPP domain), so it is by design, and the buzzer is 57 mm from the mag (its internal magnet is negligible at that range, and is not the observed ~1.7 mT hard-iron source). Worst realistic effect is supply ripple during pad beeps, not in flight. Recorded so nobody chases sensor noise during beeping later.

### I51. MLT-8530 drive circuit is correct for a magnetic transducer; one firmware guard needed against DC-stall
*switched-domains / Buzzer (LS1/Q9) — confidence high*

Topology: V_MCU_SWTCH (3.3 V) -> coil -> Q9 (PMPB14XN, absurdly derated - fine) -> GND; flyback CR1 CUS10S30 with cathode (pin 1) to the rail and anode to the switched node - correct clamp for the inductive coil; gate from P4 GPIO17 via 100R with 100k pulldown (off at boot). MLT-8530: 2.5-4.5 V operating, 16+/-3 ohm coil, passive (needs the PWM the firmware announcer already generates), ~80 dB spec is at 5 Vp-p - at 3.3 V it will be a few dB quieter. Guard: if firmware ever leaves GPIO17 stuck high, DC through the 16 ohm coil is ~200 mA / 0.7 W - cooks the transducer and loads the sensor rail; keep the LEDC/PWM watchdogged.

### I52. Camera brownout fix is credible: raw-VBATT feed behind the eFuse with 400 uF bulk and ~40 mOhm total series R
*switched-domains / Camera branch (J4/Q7) - the headline — confidence medium*

Estimated record-start inrush for a RunCam Split-class camera: steady max 450 mA @5 V (~2.7 W); record-start transient assumed 1.5-2 A for <=ms (not published - see checks_blocked). As built the camera rides raw 2S (6.4-8.4 V, Split 4 accepts DC 5-20 V) from the eFuse output pour with C15 330 uF polymer (50 mOhm ESR) 8 mm away plus 3x22 uF ceramic; series elements: R72 shunt 2 mOhm + eFuse 2.7 mOhm + Q7 ~20 mOhm + ~10-20 mOhm copper. A 2 A step sags the camera terminals ~80-120 mV - the camera sees >=6.2 V even on an empty pack, 24% above its 5 V minimum, vs the old board where the camera shared a sagging switched rail. eFuse ILIM 14.6 A with ~4.7 ms ITIMER blanking will not false-trip on it. MCU rail is isolated behind the TPS2121 + C56 330 uF hold-up. Bench proof: 2S pack on J7 (repeat near-empty), scope J4.2-J4.1 differentially + V_MCU_2S, current-probe the camera lead, assert CAM_ACT then issue RCDEVICE START_RECORDING (#251 - explicit start needed); PASS = camera terminal >=5.5 V throughout, V_MCU_2S unperturbed, no eFuse retry (probe PG at R59), 20 repeats + once during a 4-servo sweep.

### I53. JS202011 USB-select and both KMR221 buttons are wired correctly (verified against C&K datasheets)
*switched-domains / Controls (S1/S2/S3) — confidence high*

S1 JS202011 DPDT slide uses one pole's center pad (6, the common) to SEL0 (R2 100k pullup to +3V3) and adjacent throw (7) to GND: one position grounds SEL0, the other floats it high - selecting S3 vs P4 on the FSUSB63 USB-C mux; contacts rated 0.3 A/6 V, fine for logic. KMR2 datasheet's electrical diagram shows terminals 4-1 and 3-2 as the two internal pairs; both buttons wire 1,4=signal and 2,3=GND with the ground tab (5) grounded - no phantom short. S2 grounds S3-MCU GPIO0 (boot strap, internal pullup); S3 button grounds P4 GPIO35 through R54 1k with R53 100k pullup to V_MCU_SWTCH.

### I54. C-4a and Change 6 item 4 both settle CLOSED: ON pulldown present, QOD enabled, CT ramp benign
*switched-domains / P4 domain switch (U22 TPS22918) — confidence high*

ON pin: R71 100R series from POWER_SWITCH (S3 GPIO7) with R68 100k pulldown to GND - P4/V_MCU_SWTCH domain defaults OFF until the S3 deliberately asserts it (the two-MCU pyro interlock's hardware leg). CT=2.2 nF gives t_r ~3.7 ms at 3.3 V (datasheet: 1680 us at 1 nF, linear in CT), so inrush into the ~91 uF domain is ~90 mA, 4% of the 2 A limit. QOD strapped to VOUT enables the internal ~25 ohm discharge (tau ~2.3 ms) so the off domain sits at 0 V instead of floating on injected charge. Ron 53/80 mOhm typ/max at 3.3 V; worst-case domain draw (P4 core buck input + IO + sensors + buzzer ~0.8 A peak) drops <65 mV.

### I55. LoRa_ACT/GPS_ACT are real power gating; per-branch bulk lives on the daughterboards; old C10/C80 designators recycled
*switched-domains / Peripheral branch bulk (ECO Change 3 settle) — confidence high*

ECO Change 3 asked to confirm the ACT lines gate branch POWER and to add 10-47 uF local bulk on GNSS/LoRa. As built: every ACT line opens the branch's ground return - genuine power gating (with the back-feed caveat filed separately). The BOM designators from the ECO no longer mean what the docs say: 'C10' is now a 22 uF on V_CAP and 'C80' a 1 uF on VDDO_PSRAM; there is no rocket-side per-branch bulk. Instead the LoRa daughterboard carries 2x47 uF + 4x22 uF + its own TPS62913 buck, and both GNSS boards carry their own TPS62913 + local caps, so branch inrush/TX bursts are buffered at the load as intended; the servo/camera branches lean on the shared VBATT bulk (C15 330 uF + 3x22 uF) which sits within 10 mm of both connectors. FL1/FL2 (BLM18PG471SN1D, 1 A / 0.2 ohm) pass ~100-130 mA branch currents with ~25 mV drop.

### I56. MRAM deletion + S3RH2 swap is fully reflected; C27 present; no external VDD_SPI series R fitted
*switched-domains / S3 memory (post-MRAM state) — confidence high*

Current state, reported factually per the review brief: U15 is ESP32-S3RH2 (embedded PSRAM), the MRAM U12 is gone from the netlist/BOM, the S3's NAND (now GD5F2GQ5UEYIGR 2Gb, U11) sits on always-on +3V3 with R31/R33 10k WP#/HOLD# pullups (ECO Change 6 item 1 intact through the part swap - note the NAND part itself changed from the F35SQB004G recorded in schematic-review.md section 6, so the footprint/pinout confirmation item transfers to the GD5F). S3 flash U13 (W25Q128) is powered from the S3's VDD_SPI pin (OUT_VDD_SPI) decoupled by C27 10 uF + C26 100 nF; there is no external series resistor, so the '~14 ohm VDD_SPI budget' open item from the memory notes is resolved as 'direct connection, rely on internal path' - flash+in-package-PSRAM current share on VDD_SPI remains a bring-up verification (eFuse VDD_SPI config), not a netlist defect.

### I57. Narrow VBATT stubs (0.10/0.127 mm) exist but are sense taps, not power paths
*switched-domains / VBATT distribution — confidence medium*

The VBATT net shows one 0.127 mm B.Cu segment (1.03 mm) and two 0.1 mm F.Cu segments (0.95 mm) - these serve the R56/R60 TPS2121 OV/CP sense dividers and similar taps; bulk current flows in the 18 mm-wide In2 VBATT pour plus 0.3-0.5 mm locals and 13 vias. No action needed; noted so a later reviewer does not re-derive it.

## Checks the reviewers could NOT complete (open items)

- TDK VLS3012CX catalog PDF returned 403; Isat/Irated/DCR taken from the DigiKey product listing instead (1.89 A / 2.83 A / 74 mOhm)
- Murata BLM18PG471SN1D current rating not pulled this session; treated as a ~1 A-class 0603 power bead — branch loads are <=~0.15 A so margin is large either way
- Molex 878321620 (Milli-Grid) per-circuit rating cited as ~2 A from series knowledge; product spec PS-87832 not fetched
- Actual servo stall/inrush current of the servo-adapter stack is unknown, so the 8-14 A worst-case (and the J3/Q8/J7 stress numbers) inherit the ECO's own 2-3 A/servo assumption
- RunCam record-start inrush profile unknown; the camera-brownout fix (330 uF + ceramics on VBATT, logic isolated behind mux hold-up, 4.7 ms eFuse blanking) is verified by analysis only — bench item 1-2 of the ECO checklist remains open
- TCJE337M016R0050 KEMET datasheet not re-fetched; 330 uF/16 V/~50 mOhm taken from the ECO's previously verified record
- In3.Cu and In4.Cu copper not reviewed (no SVGs provided; board is 6-layer, brief said 4)
- VBATT 'star' branch geometry (Change 4 layout intent: separate branches, not daisy-chained) not audited segment-by-segment — only the electrical net membership was verified across the three B.Cu + one In2 VBATT zones
- EKYC160ELL103MM25S can dimensions vs bay clearance (ECO Change 7 warning) not verified — supplier drawing not fetched; footprint pitch 7.5 mm matches the ECO's expectation
- RunCam record-start inrush current is not published by the manufacturer; only steady-state max (450 mA @ 5 V, Split 4 V2) was found on retailer/review pages. Camera-branch sag margin was computed against an assumed 1.5-2 A / ms-scale inrush; the proposed bench test is required to close this.
- Number of servos and their stall current on the J3 expansion branch is not documented in the repo; the servo conductor-sizing finding is parametric (fine at <=4 A aggregate, undersized at 8 A+).
- Molex 87832-1620 (J3) per-pin current rating not retrieved (datasheet fetch not attempted after repeated 403s on other vendors); ~2 A/pin Milli-Grid class assumed from memory - verify before relying on 2 pins for the whole servo branch.
- Pin-by-pin verification of the custom 105-pin ESP32-P4NRW32 symbol (U17) against the Espressif datasheet pin table was not performed; only the power-architecture pins (VDD_HP/VDDPST/VFBx/EN_DCDC/FB_DCDC/CHIP_PU) were validated against the ESP Hardware Design Guidelines. DRC schematic-parity is clean for U17, so symbol==footprint at least.
- CUS10S30 pinning (pin 1 = cathode, USC package) taken from distributor datasheet summaries (LCSC/datasheet4u search result), not the Toshiba PDF itself; both CR1 and CR2 orientations are consistent with that pinning and with the bench-proven lora-daughterboard that reuses the same part+footprint.
- TPS2121 (U21) detailed pin config (PR1/OV1/OV2/CP2/ILIM divider values) was only sanity-checked, not re-derived against the TI datasheet - the July 12 review already retracted its TPS2121 findings as correct-by-design and it is outside this pass's core scope.
- Voltage rating/MPN of the '22 uF' 0805 ceramics on V_CAP (C9/C10/C11/C13) and other 8.4 V nodes — no MPN exists in the schematic or any BOM in the repo, so 16 V-suitability and DC-bias derating could not be verified (raised as a finding)
- Bay/airframe mechanical clearance for the 25 mm-tall C12 can standing off the bottom face — no mechanical/enclosure data in the repo
- Vishay CRCW-HP single-pulse energy curve not extracted from the PDF (rating check passed on the series' 0.5 W continuous figure and pulse-proof classification from Vishay doc 20043 search results); exact orderable R20 MPN also unspecified in the design
- KYC lead diameter not stated on the Chemi-Con summary page (0.8 mm assumed standard for an Ø18 radial; the placed 1.0 mm drill fits that assumption)
- ESP32-P4 datasheet v0.7 gives only a typical (45 kΩ) internal pull-up value; min/max spread unpublished — the WPU-turn-on conclusion was checked robust across 30-70 kΩ
- E-match all-fire/no-fire thresholds taken as the task's generic figures (30 mA no-fire, ~0.5 A all-fire); no specific match datasheet was named for the project
- In3.Cu/In4.Cu copper artwork was not visually reviewed (pre-generated SVGs cover only F/In1/In2/B; the board is actually 6-layer) — pyro control routing on In3 was verified by netlist/DRC connectivity only
- Exact node-by-node topology of the v3.x DCDC feedback network (2x499k + 22pF) is shown only as a schematic figure in Espressif's HW Design Guidelines; text extraction gave the requirement and values but not the figure — copy the v3.0+ reference figure directly when editing.
- ESP32-P4NRW32 ordering-code-to-chip-revision mapping (which silicon revision ships under this MPN in July 2026) could not be confirmed online — must be pinned with the distributor / Espressif Product Selector at purchase time.
- RunCam model datasheet not identifiable from the BOM (J4 is a bare JST PH-4); could not verify the mating cable pin order (J4.3=P4 RX must receive RunCam TX; J4.4=P4 TX to RunCam RX) or the camera's supply range vs raw VBATT 6.4-8.4 V — bench-check before first record test.
- SC-32S 32.768 kHz crystal ESR not re-verified from a fetched Seiko datasheet (known spec: 70 kOhm max, exactly at Espressif's <=70 kOhm limit); same part already proven on the S3/V8 boards, so treated as acceptable.
- Whether the floating GPIO34 JTAG-source strap is gated by an eFuse (as on ESP32-S3) could not be confirmed from the P4 datasheet extract; the datasheet states it 'requires external pull-up/down'.
- Molex application spec AS-47948-001-001 (feed-line/land-pattern detail beyond the product datasheet) - both WebFetch and curl timed out; layout rules were verified from the PS-47948 product datasheet instead (which states ground clearance 'none needed' and pad functions), so only fine-grained mechanical/soldering guidance is unverified
- In-package PSRAM active supply current for the RH2 is not published by Espressif (datasheet Table 5-12 gives only voltage/clock); the VDD_SPI budget uses a bounded estimate of 25-30 mA typical for 2MB quad PSRAM - the finding's conclusion holds even at half that estimate because flash-alone already violates the inequality at rail-min
- FSUSB63 power-off (VCC=0) switch-path behavior section was not extracted from the datasheet (truth table, VCC range, and float prohibition were); moot in practice because +3V3 rises from USB VBUS via the TPS2121 whenever a cable is present
- Espressif's numeric ESR limit for the 40 MHz crystal was not present in the HDG extract; the ECS part's 40 ohm ESR is far below any limit Espressif has published for this family, so not material
- ESP32-S3-WROOM-1 module internal schematic (as precedent for external-flash powering on R-variants) was not retrieved; unnecessary - the datasheet inequality settles the VDD_SPI question directly
- st.com and mouser.com direct downloads are blocked from this network (curl HTTP/2 error 92; WebFetch 60 s timeout) - ISM6HG256X and IIS2MDC datasheets were instead obtained as full-document mirrors from datasheet.lcsc.com; content matched ST document numbers (DS15034 Rev 1, DocID030986 Rev 1). IIS2MDC mirror is Rev 1 (production data) - if ST has issued a later revision, pin/appl-hint deltas were not checked
- ESP32-P4 datasheet not retrieved (same network block): the claim that GPIO2/3/4/15/41-53 are not strap pins is from memory, not re-verified; the P4 agent should confirm strap overlap for GNSS UART (GPIO2/3) in particular
- MLT-8530 buzzer drive-current figure (~60-90 mA) is from memory, not a fresh datasheet pull - affects only the info-level rail-sharing note
- Solder-mask aperture under U4 (whether the footprint opens the mask beneath the body) not verified - would require parsing the footprint's mask layers; the vias/traces finding stands regardless
- Axis-marker silkscreen meaning (X/Y arrows seen near the S3 area in the render) not correlated to sensor frames; firmware constants review is out of scope here
- RunCam camera datasheet: the specific camera model is not identified anywhere in the hardware repo, so the 6.4-8.4 V input-range compatibility of J4's raw-VBATT feed could not be verified against a datasheet -- flagged as a medium finding instead
- Harness/cable-level verification (straight vs reversed SH jumpers, Milli-Grid-to-MTA servo harness wire gauge): cables are hand-built and not in the repo; only the board-edge pinouts could be verified
- onsemi.com and mouser.com blocked direct PDF fetches (403); FSUSB63 was instead obtained from the RS-hosted official PDF -- content verified, no gap
- V8 (previous PCB) files are not in the repo, so the claim that V8 used high-side peripheral switching rests on power-eco.md section 0 wording, not on V8 sources
- PMPB14XN Rds(on) at Vgs=3.3 V is not tabulated (spec point is 4.5 V); ~20-25 mOhm estimated from datasheet curves -- does not change any conclusion
- FSUSB63 SEL truth table: onsemi/Mouser/RS PDF fetches all failed (403/timeout). Search snippet confirms SEL[1:0]=00 = sleep/open; memory says 01/10/11 -> HSD1/HSD2/HSD3 which would make the fitted S1 positions select HSD3(S3)/HSD2(P4) correctly, but the exact 10/11 mapping is unverified — confirm against the datasheet before fab (wrong mapping would make one switch position select the unconnected HSD1 = dead USB).
- TPS25982 exact dVdt/ITIMER/RETRY_DLY/NRETRY design equations (later datasheet pages not extracted). Fitted caps drifted from the ECO's stated values (ITIMER 10 nF vs 4.3 nF, dVdt 10 nF vs 8.2 nF, NRETRY 1 uF vs 560 nF); pin functions and auto-retry topology verified, but the resulting blanking time / retry count were not recomputed from the official equations.
- TPS2121 behavior with NO valid input (battery below its 2.55 V UVLO and no USB) and the precise reverse-blocking/hold-up sequence — RCB specs retrieved (1 A typ detect, 10 us response) but the ECO bench checklist item 1 (hold-up with 330 uF while IN2 sags) remains bench-only.
- MLT-8530 buzzer and GD5F2GQ5UEYIGR NAND datasheets not fetched this session (drive topology and 3.3 V/WSON-8 pin map assessed from netlist + vendor naming conventions, high confidence).
- JST PH (2 A) and JST SH/SRSS (1 A) contact ratings cited from standard JST specifications, not fetched this session.
- In3.Cu / In4.Cu copper not reviewed — the board is 6-layer but only F/In1/In2/B SVGs were pre-generated (task context said 4-layer).
- ESP32-P4 GPIO37/GPIO38 exact strap roles: datasheet summarizer labeled them 'boot mode functionality' with floating defaults — direction of risk is clear (they are strapping pins routed bare to J3) but the precise sampled function is low-confidence.
- S3 radio burst / P4 core current figures used in the +3V3 budget are datasheet-typical values from training (S3 TX ~285-350 mA, P4 domain ~250-400 mA at 3.3 V), not re-fetched; the conclusion (~0.7 A worst case vs 1 A) has margin but was not confirmed against the current ESP32-S3/P4 datasheet consumption tables.
- Chemi-Con EKYC160ELL103MM25S exact case dimensions not conclusively pinned from a vendor datasheet (web sources ambiguous 16 vs 18mm dia; board fab circle says 18mm) - the C12 collision finding holds for either size, but the ECO's bay-clearance verification remains open
- In3.Cu/In4.Cu layer SVGs absent from the pre-generated artifact set - In3 is the largest inner routing layer (747 segments); reviewed by file parsing only, no visual pass
- Previous-generation (V8) board layout is not in the repo, so the magnetometer better/worse comparison vs the 1.7mT hard-iron history is geometric/qualitative, not a measured before/after
- TPS2121 datasheet gives only qualitative input-capacitance guidance (no mandatory CIN value), so the IN1 no-cap finding is graded low rather than against a hard spec
- ESP32-P4 pin-54 revision behavior confirmed via Espressif docs search-result text and the HDG checklist page fetch; the full current-revision P4 datasheet PDF itself was not downloaded (scratchpad copy is pre-release v0.7)
- TDK product-center page returned HTTP 403; VLS3012CX-2R2M-1 ratings (2.2 uH, 2.83 A rated, 1.89 A Isat, 74 mOhm) taken from DigiKey/Farnell listings instead of the primary PDF
- TBLH11-350 numeric current rating not present in the local text extraction (table lost in conversion); family rating ~10 A/300 V taken from Same Sky series data - pyro pulse duty is far below either value
- R72 shunt power rating unverifiable: BOM has no MPN/manufacturer (raised as a finding rather than assumed)
- JST PH (2 A) and Molex Milli-Grid (2 A/contact) figures come from distributor-hosted datasheet summaries and the Molex product-spec reference, not page-verified PDFs
- Actual worst-case servo stall current not measurable from files; the 12-14 A figure is the task-provided assumption - if the servo set is bounded lower, the servo-chain severities scale down accordingly
- No thermal simulation performed; all temperature-rise statements are IPC-2152-approximation estimates with stated assumptions
- Molex AS-47948-001 application spec and SD-47948-001 sales drawing could not be retrieved (molex.com HTTP/2 stream errors + WebFetch 60 s timeouts; distributor mirrors host only the product literature). Exact recommended land-pattern dimensions, feed-line entry geometry, and any specified minimum component clearance to the antenna body are therefore unverified; on-ground/no-keepout and dummy-pad conclusions rest on the official 4-page product literature (available locally) which states them explicitly
- Seiko SC-32S datasheet not fetched — Espressif's 32 kHz ESR <= 70 kOhm requirement not independently verified for Y1/Y3 (mitigation: the same SC-32S part is already in service on current boards per project memory)
- RF feed Z0 given as an analytic CPWG estimate (52-60 Ohm band); a field-solver pass is needed for a firm number before deciding whether to widen the trace
- ESP32-P4 hardware design guidelines were not separately fetched (P4 has no RF; USB assessed against the S3 guide's USB rules + P4 datasheet pin tables). If the orchestrator wants P4-specific layout-rule citations (e.g., for the Serial-JTAG pair), that pull remains to be done
- In3.Cu and In4.Cu visual review: the pre-generated artifact set has no SVGs for these layers (board is 6-layer, only 4 layers exported); geometric checks were done from the .kicad_pcb instead, but no human-style visual sweep of In3/In4 routing was possible
- GCT USB4110-GF-A datasheet not retrieved: J6 shell-overhang extent over FID1 judged from the courtyard geometry and 3D render, not the mechanical drawing (conclusion robust - the fiducial is inside the courtyard either way)
- MLT-8530 buzzer datasheet not retrieved: polarity criticality of LS1's '+' mark asserted from the footprint's own polarity marking, not the manufacturer sheet
- PMPB14XN (Q1/Q7-Q10) and TSON-Advance land-pattern conformance vs manufacturer recommended patterns not re-derived dimension-by-dimension (local PDFs exist in the scratchpad from a prior pass; my check was limited to mask/paste layer correctness and alignment)
- Stencil/paste judgment for the 0.35 mm P4 assumes a 100-120 um foil range; the actual assembler's stencil spec was not available to check against
