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
the cap to a flat 3.0 V** instead of letting the rail sag to a 2.9 V cutoff.

Deleted per board: U40, U42, U44, U45, R120 (V10 also R128 — its second 33 R),
R127 (ST pullup), C134, C136 (CT caps), C137, C138 (V_HOLD decoupling); nets
V_HOLD and V_UVLO_EN are gone. Added: U47 TPS61094DSSR (WSON-12), L7 2.2 µH
(XGL4020-222ME class, Isat ≥ 4 A), R134 3.09 k (OSEL → VOUT 3.0 V), R135 6.65 k
(VCHG → 2.5 V), R136 22.1 k (ICHG → 100 mA) — all three 1%, values straight from
the datasheet §8.2.4 reference design — C141 10 µF at VIN, C142 10 µF ceramic at
V_SCAP (the prismatic cap's ~190 mΩ ESR needs a local ceramic for the 1 MHz boost
pulses), and the VBUCK_OK divider R137 100 k / R138 1 M (R138 became 360 k under #1000,
and C152 100 nF was added at the FC pin under #1169 — see the firmware handoff).

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
- **Hold-up** (E = ½C(Vchg² − Vfloor²)·0.9) — *these rows are superseded twice:
  the cap became a 5 F HV1020 on 2026-08-29 and the rail became 3.0 V on
  2026-09-03; see both sections below for the current figures*:
  - mini, existing 2 F TWQ: 2.6 → 0.9 V ⇒ ~5.4 J ⇒ **~8.5 s @190 mA / ~5.4 s
    @300 mA** (today: 5.0 s / 2.8 s on a sagging rail).
  - V10, both 2 F TWQ kept (4 F): 2.6 → 1.5 V ⇒ ~8.1 J ⇒ **~3.5 s @0.7 A**
    (today: ~2.1 s). A single TWQ would be 1.2–1.7 s — **short of today**, which
    is why the second cap stays in this drawing (see cap options below).
- **Inductor**: peak = IL(DC) + ΔIL/2 ≈ 2.0/3.3·0.7/0.85·… worst ≈ 2.6 A at the
  V10 floor. XGL4020-222: Isat 2.7 A (10% drop) / 4.4 A (20%), soft-sat, DCR
  21.5 mΩ max — fine. The stocked VLS3012CX line was REJECTED: its 30%-drop saturation is 1.7 A max-spec / 1.89 A typ (TDK) against a converter current limit of up to 2.6 A + ripple, exercised by design during backup at load.
  Footprint drawn to Coilcraft doc 1529: pads 0.98×3.4 mm, 2.37 mm gap
  (`Footprints:L_XGL4020_4.0x4.0mm`).
- **Charge current**: ICHG 100 mA against the TPS62152's 1 A budget (worst
  concurrent system load ~0.45 A) — full charge in ~52 s/cap-bank vs ~200 s with
  the old 33 R. ICHG_PRE ≤ 250 mA below VSUP 0.85 V, bounded.
- **Rail in normal op**: bypass FET 150 mΩ ⇒ ~0.1 V drop at 0.7 A — comparable
  to the two deleted series FETs. VOUT target 3.0 V sits 465 mV under V_BUCK,
  which clears the **150 mV worst-case** VBYPASS rather than the 100 mV typical —
  see the 2026-09-03 section; boost engages only below ~3.0 V input.
- **What disappears with the sag**: the E220/LC86G 2.83 V minimums audit, the
  supervisor-threshold overlap class (trap #5 ESR-vs-hysteresis, and the K33-type
  landmine), and the TPS3840/TPS22810 EN precision issue (trap #2). The FC BOD
  (2.44 V) never sees a legitimate brownout — the rail is 3.0 V until the cap is
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
flat 3.3 V — the mini's rail became 3.0 V on 2026-09-03, same energy; see
below). Verified: only C130/R135 differ in the netlists, zero net changes,
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
  GPIO3 only as of 2026-09-03 — the OC was taken off this net, see below; V10:
  S3 GPIO34 (P4 via link). Drop the first-boot-blip handling. The mini's pin has
  a 100 nF reservoir (C152, 2026-09-04) so the ADC can also read it: 78 kΩ source
  into the S3's sampling capacitor otherwise, the same defect #1022 fixed on
  V_SCAP_ADC.
- Backup detection = VBUCK_OK low; cap health = existing V_SCAP_ADC (unchanged,
  R125/R126). Shed choreography simplifies: the rail holds 3.0 V flat (mini;
  3.3 V on the V10 until it takes the same change); budget
  seconds from the tables above, then the rail simply ends.
- Remove the 2.83 V-floor logic tied to the old sag behavior.

## Bench / first-article

1. Charge profile: V_SCAP 0 → 2.5 V at 100 mA, termination accuracy, buck off at
   VCHG (no chatter with the ~190 mΩ prismatic ESR — C142 in place).
2. Yank V_BUCK at 0.19/0.3 A (mini) and 0.5/0.7 A (V10): switchover transient on
   +3V3 (expect < 100 mV excursion per Fig 8-20), flat 3.0 V, floor voltage at
   collapse; compare hold times to the table.
3. Bypass drop at max load; thermal on U47 (WSON EP stitched).
4. WSON-12 EP paste/mask on first article (stock KiCad land used:
   WSON-12-1EP_3x2mm — verify against TI DSS0012A before fab).
5. Inductor substitute (if not XGL4020): verify Isat ≥ 4 A class and land fit.

## OSEL lowered to 3.0 V (2026-09-03 — mini drawn; V10 NOT yet)

Review finding #999. **The cap was never guaranteed to charge.** The TPS61094
buck-charges only while `V_BUCK ≥ OSEL target + VBYPASS`, and VBYPASS is specified
50 / 100 / **150** mV — the corner that matters is the maximum, not the typical.
With OSEL at 3.3 V the requirement is 3.450 V against a nominal `V_BUCK` of
3.465 V, i.e. 15 mV of margin; the TPS62152's **Power-Save-Mode** accuracy
(−2.3 %, specified at COUT = 22 µF, which is exactly `C135`, and PSM is the mode
this light load sits in) puts the worst case at 3.385 V — **65 mV short**. Such a
unit runs normally through the bypass FET as an LDO with the supercap never
charged and nothing reporting it.

**Change: `R134` 4.75 kΩ → 3.09 kΩ, OSEL 3.3 V → 3.0 V.** The requirement drops to
3.150 V, leaving 235 mV at the same worst case. Table 7-1 does have a 3.0 V code
between 2.7 V and 3.3 V; an earlier note claiming otherwise was wrong.

| | OSEL 3.3 V | OSEL 3.0 V |
|---|---|---|
| `V_BUCK` needed to charge | ≥ 3.450 V | ≥ 3.150 V |
| nominal `V_BUCK` 3.465 V | +15 mV | +315 mV |
| worst-case `V_BUCK` 3.385 V | **−65 mV** | **+235 mV** |

**Normal operation is unchanged**: OSEL sets only the boost regulation point, so
with the pack present the part is still in bypass and `+3V3` is still `V_BUCK` at
3.465 V. The single consequence is a 3.0 V hold-up bridge instead of 3.3 V.

**Energy is unchanged** at ~12.2 J (5 F, 2.5 → 0.9 V, η 0.9). Hold time is
unchanged for constant-power loads and roughly 10 % longer for constant-current
ones: ~21 s @190 mA / ~13.6 s @300 mA against the 19 s / 12 s quoted above.

**3.0 V is acceptable for the bridge.** `+3V3` carries `U15`, the boot flash, the
pack monitor, the load switch and `U47` itself; only the ESP32-S3 is at its edge
(3.0 V is its recommended minimum, and boost PFM accuracy could put it at 2.94 V),
while its brownout detector sits at the ESP-IDF default near 2.5 V. A hold-up that
works at 3.0 V beats one that is statistically absent.

**Do not try to fix this by raising `V_BUCK`.** Worst-high is already
3.465 × 1.028 = **3.562 V**, and in bypass that *is* `+3V3` — about 38 mV under the
3.6 V absolute maximum shared by everything on the rail. `DEF` must also stay high:
at 3.3 V nominal the worst-low 3.224 V leaves only 74 mV even with OSEL at 3.0.

**The V10 rocket-computer has the identical circuit and has NOT been changed** —
same `U47`, `R134` 4.75 k, `R135` 6.65 k, `R136` 22.1 k, and `DEF` tied high. It
carries the same defect and wants the same one-resistor fix.

Open on #999: log `V_SCAP` from cold start on the first article to confirm the cap
actually charges, and add a preflight advisory if it is still low a few minutes
after power-on, so any future silent failure is visible.

## VBUCK_OK made usable (2026-09-03 — mini drawn; V10 NOT reviewed)

Review finding #1000. The one divider node fed **both** processors, and that broke
it in both directions.

**It read "on the cap" through all of pad standby.** With `V_MCU_SWTCH` down,
`U30`'s quick-output-discharge actively holds that rail at ground through
250–400 Ω, and the FC's GPIO3 pad clamps the divider node about a diode drop
above it. Only 28 µA flows through `R137`, so the QOD wins and the node sits near
0.65 V against the out computer's V_IL of 0.87 V — the OC read LOW exactly in the
window where it is the only processor awake.

**And after a real pack loss it was far too slow.** `V_BUCK` carries **42 µF**
(`C135` 22 + `C141` 10 + `C53` 10). Once the bypass opens the only discharge is
the divider's 1.1 MΩ plus the buck's 1.5 µA shutdown current, about 4.3 µA, so
0.10 V/s. The pin left the guaranteed-HIGH band ~5.6 s after the bypass opened
and reached a guaranteed LOW only after ~22 s, against ~21 s of hold-up: the flag
arrived as the cap died.

**Changes drawn:**

1. **The out computer is off this net.** Its `VBUCK_OK` label and the stub wire to
   GPIO34 are deleted; GPIO34 is now a spare pad. The OC does not need the signal —
   the pack monitor sits on the always-on I2C and reads `VBAT_CON` bus voltage
   directly, which is the earliest and least ambiguous pack-loss indicator on the
   board and, unlike `VBUCK_OK`, works in pad standby. This removes the clamp by
   construction: the FC pad only loads the node when the FC rail is up.
2. **`R138` 1 MΩ → 360 kΩ** so the node lands in ADC range. GPIO3 is `ADC1_CH2`.
   At 1 MΩ the node was 0.909 × `V_BUCK` = 3.15 V, at or above the top of ADC1's
   usable span even at maximum attenuation, so it could only be read as a logic
   level. At 360 kΩ it is 0.783 ×: **2.71 V** with the buck up, **2.39 V** once the
   boost takes over — a 325 mV step the FC sees immediately instead of waiting
   seconds for a threshold crossing.

Verified: `VBUCK_OK` is now `R137`/`R138`/`U32` GPIO3 only, GPIO34 falls out as
unconnected, ERC total unchanged at 808 (+1 spare-pin notice, −1 off-grid from the
deleted stub).

**Firmware consequences:** the OC stops reading GPIO34 and uses the pack monitor's
bus voltage instead; the FC reads GPIO3 as `ADC1_CH2` rather than a digital input,
comparing against ~2.71 V / ~2.39 V. Both board headers need the change (#1011 — done 2026-09-03).

**Not done:** no bleed resistor was added — with the ADC read it is unnecessary. If
a valid *logic-level* read is ever wanted as well, ~10 kΩ across `V_BUCK` collapses
it in ~0.4 s, at ~350 µA off the pack whenever the buck is up. **Two numbers worth
confirming on the first article:** ADC1's actual full-scale at maximum attenuation
(the extracted datasheet text does not carry the attenuation table), and the
pad-clamp voltage, which is inferred from the ESD structure rather than specified.
Also note GPIO3 is a strapping pin, but only when `EFUSE_STRAP_JTAG_SEL` is burned
to 1 — the default is 0, where it is ignored; the divider must stay non-floating,
which it is.

## Output capacitance raised (2026-09-03 — mini drawn; V10 NOT laid out yet)

Closes the schematic half of #1012. `U47`'s VOUT (pins 9/10) is `+3V3`, and the
datasheet's recommended operating conditions ask for **20 µF minimum / 30 µF
nominal effective at the pin**. Nameplate on the net looked comfortable — 44.4 µF
across ten parts — but neither derating nor distance was in that number:

| | nameplate | package | est. @3.3 V | distance to VOUT |
|---|---|---|---|---|
| `C17` | 22 µF | 0805 16 V X5R | ~16.5 µF | **2.45 mm, same side** |
| `C32`, `C36` | 10 µF ×2 | 0402 **6.3 V** X5R | ~3.5 µF each | 8.0 / 8.1 mm, top |
| `C30`, `C35` | 1 µF ×2 | 0402 16 V X5R | ~0.9 µF each | 7.4 / 8.1 mm, top |
| 4×100 nF + 10 nF | 0.41 µF | 0402 | ~0.39 µF | 4.3–9.4 mm, top |

That is **~26 µF effective on the whole net, but only ~16.5 µF local to the pin** —
under the minimum by itself, and short of nominal even if every remote part is
credited in full. The two 10 µF parts are the trap: 10 µF in an 0402 at a 6.3 V
rating is the most aggressively DC-biased combination on this board and contributes
roughly a third of its label at 3.3 V. Loop margin and the switch-over transient
were resting on that, worst exactly when the board is running off the cap.

**Drawn:** `C143`, a second 22 µF 0805 16 V X5R (same part as `C17`), on `+3V3`
with its own `+3V3`/`GND` symbols on the power sheet. Takes the pin to ~33 µF
effective and the net to ~42 µF. ERC unchanged at 808; netlist delta is exactly one
component, `C143` pin 1 → `+3V3`, pin 2 → `GND`, no other net touched.

**Placement is outstanding** and is the whole point of the change — `C143` must go
on the **bottom** side beside `C17`, next to pins 9/10. Parked anywhere else it
adds nameplate and fixes nothing. The area is tight: twelve bottom-side parts sit
within 6 mm of `C17` today.

**Correction to the review text in #1012,** which offered "a second 22 µF 0805 **or**
a 47 µF 6.3 V X5R" as equivalent options. They are not. A 47 µF 6.3 V 0805 derates
to roughly 19 µF at 3.3 V — barely past what `C17` alone already provides. The
16 V-rated 22 µF is the better part despite the smaller label.

The derating percentages above are typical-part estimates, not vendor curves; the
6.3 V 0402 figure carries the most uncertainty. It does not change the conclusion —
`C17` alone is under 20 µF on any reasonable reading, since even at a generous
−20 % with its ±10 % tolerance on the low side it lands near 15.8 µF.

**V10 shares the topology** — same `U47`, same `C17` as the only 0805 bulk, plus
three 10 µF 0402s (54.6 µF nameplate). Its hold-up parts are not placed yet, so the
distance half of this finding does not exist there yet. Apply the same rule when
that board is laid out, and add the second 0805 at the same time.

## EN/MODE are hard-wired, and that is correct (2026-09-03 — documents #1023)

`U47`'s EN and MODE both tie to `V_BUCK` (= VIN), putting the part permanently in
**Auto buck/boost**. The four modes are selected by these two pins:

| EN | MODE | mode |
|---|---|---|
| 1 | 1 | Auto buck/boost — **this board** |
| 1 | 0 | Forced buck |
| 0 | 0 | Forced bypass (VOUT = VIN through the bypass FET, 4 nA IQ) |
| 0 | 1 | True shutdown (load disconnected from VIN *and* SUP) |

So **true shutdown is unreachable by construction**, and that is a deliberate
consequence of a constraint, not an oversight:

**EN cannot be referenced to VOUT.** Its threshold is VOUT-referenced (V_EN_H =
0.58 V above VOUT 1.8 V), which invites tying it to `+3V3` — but §7.3.2 requires
EN to be high *before* VOUT exists: "When the voltage at the VIN pin is above the
UVLO rising threshold and the EN pin is pulled to logic high voltage, the
TPS61094 is enabled and starts ramping up the output voltage." Tie EN to VOUT and
the part deadlocks at power-on. EN has to come from VIN.

**VIN collapsing does not stop the boost.** §7.3.1: "After the TPS61094 starts up
and the output voltage is above 1.7 V typically, the TPS61094 can work with SUP
pin voltage as low as 0.6 V and **input voltage down to 0 V**."

### Consequences to know about on the bench

**The board runs on after the pack is unplugged.** 12.2 J at the out computer's
~45 mA standby (0.135 W at 3.0 V) is ~90 s on paper; at pad-idle ~190 mA it is
~21 s. **The ~90 s figure is probably optimistic.** EN decays with `V_BUCK`, which
is 42 uF (`C135`+`C141`+`C53`) into ~460 k (`R137`+`R138`), tau ~19 s — so EN
crosses 0.58 V about **34 s** after pack loss and truncates the hold there. At
flight load the cap is spent in ~21 s so this never binds in the intended case,
but neither number is measured. **Measure it on the first article.**

**The hold terminates in forced bypass, not shutdown.** EN and MODE fall together
(both on `V_BUCK`), so the end state is (0,0) — the bypass FET closes and connects
`+3V3` to the dead `V_BUCK`. Voltages are equal by then, so it is benign, but that
is the actual termination behaviour rather than a clean cut-off.

**A code-resistor change needs a full EN toggle.** §7.1.1: "The TPS61094 does not
detect the VCHG, ICHG, and OSEL pins during operation, so changing the resistor
during operation does not change the VCHG, ICHG, and OSEL settings. Toggling the
EN pin during operation is one way to refresh the VCHG, ICHG, and OSEL settings."
Since EN follows `V_BUCK`, refreshing OSEL/VCHG/ICHG means letting `V_BUCK` decay
below 0.58 V — a quick unplug-replug does **not** do it. Wait ~1 minute, or
discharge the cap.

### Discharging the supercap

**Never short it.** 5 F at 2.5 V through the cell's ~40 mOhm ESR is roughly 62 A.

Use **4.7 Ohm or more** across `V_SCAP`: 532 mA initial, 1.3 W initial (so a 2 W
part), tau ~24 s, essentially flat after two minutes. 10 Ohm is gentler at 250 mA
/ 0.6 W and tau ~50 s.

### If a bench affordance is ever wanted

100 k in series from `V_BUCK` into EN, with the junction brought to a **test
pad**. Grounding the pad gives an immediate rail-off and a code refresh.

**Do not put this on an out-computer GPIO.** EN low with MODE still high is *true
shutdown* — it drops `+3V3` and takes both processors down. That is not a power
firmware should hold in flight. A pad a human has to touch is the right shape.

