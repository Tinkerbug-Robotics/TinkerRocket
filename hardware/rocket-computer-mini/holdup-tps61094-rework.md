# Hold-up rework: TPS61094 supercap buck/boost replaces the 4-IC chain

**Status: DRAWN AND VERIFIED ON BOTH BOARDS 2026-08-29 (branch
`claude/tps61094-holdup`, uncommitted). Mini PCB deletions done (the corner was
placed but unrouted — zero tracks touched); V10 hold-up was never placed, so its
PCB needed nothing.** Supersedes the 2026-08-26 LM66100×2 + TPS22810 + TPS3840 +
33 R chain on both boards.

Verification: netlist diffs exactly to spec on both boards (mini 215→214 incl. the
arm parts, V10 272→270; V_HOLD/V_UVLO_EN gone, SCAP_SW/OSEL_SET/ICHG_SET/VCHG_SET
new, VBUCK_OK replaces CAP_ACTIVE); ERC strictly better on both (mini 803→756,
V10 1029→1012 — includes clearing two pre-existing lib_symbol issues on deleted
parts); both bom.csv reconciled 0/0/0 — **including a repair of a duplicate R22/
R133 membership on the mini's precision-100k row that #981 had shipped**; zones
refilled, untouched. MERGE CAUTION: the owner's main-checkout WIP has the five arm
parts placed on the mini PCB — replay this branch's 10 deletions onto that board
semantically, as before, not textually.

## What changed

One TPS61094 (U47) now does the whole job: in normal operation it bypasses
V_BUCK → +3V3 (150 mΩ FET) while buck-charging the supercap on V_SCAP to a
programmed termination; when V_BUCK collapses it auto-switches and **boosts from
the cap to a flat 3.3 V** instead of letting the rail sag to a 2.9 V cutoff.

Deleted per board: U40, U42, U44, U45, R120 (V10 also R128 — its second 33 R),
R127 (ST pullup), C134, C136 (CT caps), C137, C138 (V_HOLD decoupling); nets
V_HOLD and V_UVLO_EN are gone. Added: U47 TPS61094DSSR (WSON-12), L7 2.2 µH
(XGL4020-222ME class, Isat ≥ 4 A), R134 4.75 k (OSEL → VOUT 3.3 V), R135 9.53 k
(VCHG → 2.6 V), R136 22.1 k (ICHG → 100 mA) — all three 1%, values straight from
the datasheet §8.2.4 reference design — C141 10 µF at VIN, C142 10 µF ceramic at
V_SCAP (the prismatic cap's ~190 mΩ ESR needs a local ceramic for the 1 MHz boost
pulses), and the VBUCK_OK divider R137 100 k / R138 1 M.

**CAP_ACTIVE is renamed VBUCK_OK and INVERTED**: the TPS61094 has no status pin,
so the MCU pin now watches a V_BUCK divider — HIGH = buck present, LOW = riding
the cap. (Old ST semantics were HIGH = on cap.) Firmware must flip the sense and
lose the first-boot-blip special case.

## Corner pass (datasheet SLVSFH6C)

- **Termination is capped by VIN, not by the cap**: clean constant-current charge
  needs VCHG ≤ VIN − 800 mV; with V_BUCK = 3.465 V that is ~2.66 V ⇒ **VCHG =
  2.6 V**. (Up to VIN − 500 mV works with end-of-charge taper; 2.9 V was rejected
  — it buys ~25% energy but tapers into the accuracy band and precludes ever
  fitting 2.7 V-rated caps.) This corrects the earlier claim that the cap could
  terminate at ~3.2 V.
- **Boost floor vs load** (Eq. 4, ILIM min 1.7 A, η 0.85, L 2.2 µH):
  IOUT(max) ≈ VSUP·(ILIM − ΔIL/2)·η / 3.3. Worst-silicon floors: 0.19 A (mini
  cruise) ≈ 0.45 V — the 0.7 V SUP UVLO governs; 0.3 A ≈ 0.75 V; **0.7 A (V10)
  ≈ 1.8–1.9 V worst / ~1.5 V typ**.
- **Hold-up** (E = ½C(Vchg² − Vfloor²)·0.9, flat 3.3 V out):
  - mini, existing 2 F TWQ: 2.6 → 0.9 V ⇒ ~5.4 J ⇒ **~8.5 s @190 mA / ~5.4 s
    @300 mA** (today: 5.0 s / 2.8 s on a sagging rail).
  - V10, both 2 F TWQ kept (4 F): 2.6 → 1.5 V ⇒ ~8.1 J ⇒ **~3.5 s @0.7 A**
    (today: ~2.1 s). A single TWQ would be 1.2–1.7 s — **short of today**, which
    is why the second cap stays in this drawing (see cap options below).
- **Inductor**: peak = IL(DC) + ΔIL/2 ≈ 2.0/3.3·0.7/0.85·… worst ≈ 2.6 A at the
  V10 floor. XGL4020-222: Isat 2.7 A (10% drop) / 4.4 A (20%), soft-sat, DCR
  21.5 mΩ max — fine. The stocked VLS3012 line was REJECTED (Isat ~1.5 A).
  Footprint drawn to Coilcraft doc 1529: pads 0.98×3.4 mm, 2.37 mm gap
  (`Footprints:L_XGL4020_4.0x4.0mm`).
- **Charge current**: ICHG 100 mA against the TPS62152's 1 A budget (worst
  concurrent system load ~0.45 A) — full charge in ~52 s/cap-bank vs ~200 s with
  the old 33 R. ICHG_PRE ≤ 250 mA below VSUP 0.85 V, bounded.
- **Rail in normal op**: bypass FET 150 mΩ ⇒ ~0.1 V drop at 0.7 A — comparable
  to the two deleted series FETs. VOUT target 3.3 V < V_BUCK − 100 mV ⇒ bypass
  engages correctly; boost only below ~3.4 V input.
- **What disappears with the sag**: the E220/LC86G 2.83 V minimums audit, the
  supervisor-threshold overlap class (trap #5 ESR-vs-hysteresis, and the K33-type
  landmine), and the TPS3840/TPS22810 EN precision issue (trap #2). The FC BOD
  (2.44 V) never sees a legitimate brownout — the rail is 3.3 V until the cap is
  spent, then gone.
- **EMI**: the converter switches only while charging (~1 min after power-on)
  and during backup events; 60 nA quiescent otherwise. Keep the L7/SW loop tight
  (layout note on sheet); mag cal unaffected in cruise.

## Cap decisions (updated 2026-08-29 — DECIDED and drawn)

**Eaton HV1020-2R7505-R chosen for BOTH boards (owner call); VCHG = 2.5 V
(6.65 kΩ).** Note the correction: Table 7-2 has **no 2.4 V code** — the options
are 2.2 V (4.75 k) or 2.5 V (6.65 k), and 2.2 V would leave the V10 at ~1.2 s
worst-case with zero margin, so 2.5 V (93% of the 2.7 V rating; brief pad/flight
duty makes the derating acceptable). Drawn on both boards: C130 revalued
(HV1020-2R7505-R / Eaton), R135 9.53 k → 6.65 k, notes and BOMs updated, mini's
prismatic footprint deleted from the PCB (re-place as the new radial). Mini
footprint = `Footprints:Supercap_Radial_D10.0mm_L20.0mm_P5.00mm_Horizontal`
(drawn to the house prismatic-horizontal conventions: pad 1 positive, 2.5 mm
bend allowance, no courtyard over the glued body); V10 = stock
`Capacitor_THT:CP_Radial_D10.0mm_P5.00mm` standing 20 mm (swap to the
horizontal variant at layout if preferred). As-drawn hold: **mini ~19 s
@190 mA / ~12 s @300 mA; V10 ~2.6–3.9 s @0.7 A** (2.5 → floor, η 0.9,
flat 3.3 V). Verified: only C130/R135 differ in the netlists, zero net changes,
ERC identical, BOMs reconciled, parity lists refreshed (V10 17, mini 15
unplaced).

**V10 reduced to ONE cap (owner call): C139 deleted.** 1× 2 F TWQ at VCHG 2.6 V
gives ~1.2–1.7 s @0.7 A — enough to ride any firing event, which is the hold-up's
job on this board. Sheet note and bom updated; netlist/ERC verified.

**2.7 V single-cell selection (open; both boards ~22 mm wide — mini GND zone
21.9×65.4, V10 outline 22.4×75):**

- **LCSC is a dead end for this class**: its entire EDLC category is 5 rows, and
  the stocked ones are 3.8 V Li-ion capacitors (LIC) — rejected outright: ~2.2 V
  discharge floor wastes the boost's reach, and −20 °C minimum fails a cold pad.
- **Primary candidate: Eaton HV1020-2R7505-R** — 5 F, 2.7 V, Ø10×20 mm, ~40 mΩ,
  1000 h @65 °C (family rated −40..+85 with derating). LIVE 2026-08-29 at Verical
  (channel already used for JS202011): ~16.6k pcs across listings, $0.86–1.12.
  **One shared line serves BOTH boards** at VCHG 2.5 V:
  - mini: ~12.6 J ⇒ **~20 s @190 mA** (4× today), lying on the bottom side =
    10 mm tall over a Ø10×~23 mm footprint (today's prismatic bay: 28.5×13,
    ~7–8 mm tall — fits with ~2 mm height growth).
  - V10: ~5.9–9 J ⇒ **~2.6–3.9 s @0.7 A** (over the accepted 1.2–1.7 s target);
    standing = 20 mm tall (vs 28.5 today) or lying = 10 mm.
  - Cost: ~−$2.4/mini, ~−$5.6/V10 vs the TWQ arrangement, at 40 mΩ (80 mV dip
    at the 2 A boost draw — comfortably inside the floor margins).
- Smaller/lower fallback (mini only, if the 2–3 mm height growth matters): the
  HV family's 1 F Ø8×10.5 class — but 1 F ⇒ ~4.0 s @190 mA, slightly UNDER
  today's 5.0 s. Verify exact MPN on a live page at buy time (guessed HV/TV
  sibling MPNs did not resolve on aggregators — only the 5 F is live-verified).
- VCHG code: 2.5 V = 93% of a 2.7 V cell; the 2.4 V code trades ~8% energy for
  lifetime margin at temperature. Swap = footprint + one VCHG resistor change;
  ask before drawing (footprint/bay placement is a layout call).

## Sourcing (live 2026-08-28 unless noted)

- TPS61094DSSR: LCSC C3034939, 1114 stock, $2.55@100 (~$2.42@500).
- XGL4020-222ME: Coilcraft — check DK/Mouser at buy time; LCSC-friendly 4020-size
  2.2 µH ≥4 A substitutes exist (sourcing pass, land is compatible-generic).
- R134/R135/R136: Yageo RC0402FR 1% E96 values — three new (cheap) BOM lines.
- Deleted line value ≈ $1.24/board silicon+passives; net chain delta ≈ +$2.0/board
  before any cap change; the single-cell cap swaps flip the total to ≈ −$1 (mini)
  and ≈ −$4 (V10).

## Firmware handoff

- `VBUCK_OK` replaces `CAP_ACTIVE`, **inverted** (HIGH = buck present). Mini: FC
  GPIO3 + OC GPIO34; V10: S3 GPIO34 (P4 via link). Drop the first-boot-blip
  handling.
- Backup detection = VBUCK_OK low; cap health = existing V_SCAP_ADC (unchanged,
  R125/R126). Shed choreography simplifies: the rail holds 3.3 V flat; budget
  seconds from the tables above, then the rail simply ends.
- Remove the 2.83 V-floor logic tied to the old sag behavior.

## Bench / first-article

1. Charge profile: V_SCAP 0 → 2.6 V at 100 mA, termination accuracy, buck off at
   VCHG (no chatter with the ~190 mΩ prismatic ESR — C142 in place).
2. Yank V_BUCK at 0.19/0.3 A (mini) and 0.5/0.7 A (V10): switchover transient on
   +3V3 (expect < 100 mV excursion per Fig 8-20), flat 3.3 V, floor voltage at
   collapse; compare hold times to the table.
3. Bypass drop at max load; thermal on U47 (WSON EP stitched).
4. WSON-12 EP paste/mask on first article (stock KiCad land used:
   WSON-12-1EP_3x2mm — verify against TI DSS0012A before fab).
5. Inductor substitute (if not XGL4020): verify Isat ≥ 4 A class and land fit.
