# rocket-computer (V10) — pack-direct pyro + computer hold-up: design options

**Status: Option A DRAWN 2026-08-26** (off-page blocks on `power.kicad_sch` /
`external_connections.kicad_sch`; R20/C12 deleted, V_CAP relabeled VBAT_CON,
C94 = 4.7 µF, DEF tied to V_BUCK, P4 pad 17 = ARM_CLK, pad 18 = V_SCAP_ADC,
PIEZZO moved to S3 GPIO13 (pad 18), CAP_ACTIVE on S3 GPIO34 (pad 39); refdes
match the mini one-for-one). **One manual step remains: the V_BUCK rail split
at U18** — see the MANUAL SPLICE note on the power sheet. Options B–D below are
kept for the record. This ports the
architecture just built into rocket-computer-mini (pack-direct firing, +3V3
supercap hold-up, 2.9 V hardware cutoff, charge-pump ARM with a latching
one-shot ceiling) onto the full V10 board, which is a different animal in three
ways: the logic load is 3–4× the mini's, GNSS/LoRa/servos/camera are
battery-rail loads behind the eFuse, and the P4 has no spare pins.

Constraint from the requirements discussion: **GNSS must survive a firing event**
(re-acquisition is unacceptable); LoRa and servos may brown out; camera was
already accepted as sag-exposed.

## What V10 has today (netlist-verified from the working tree)

```
J8 (JST-VH) → Q11 (AONR21321) → R72 2 mΩ → VBAT_CON → U19 eFuse → VBATT
   TPS259824L: ILM ≈ 11.6–14.7 A, UVLO 6.91/6.34 V, C94 1 µF deglitch (τ ≈ 174 ms)
VBATT ─┬─ R20 150 Ω → V_CAP (C12 10 mF EKYC) → U6/7/8/10 → channels → U9 ARM
       ├─ U26 → J4.1  CAMERA        (CAM_ACT,  P4 GPIO)
       ├─ U27 → FL1 → J1.3  GNSS    (GPS_ACT,  P4 GPIO16 — pad 16)
       ├─ U28 → J3.1/2  SERVO rail  (SERVO_ACT, P4)
       ├─ U29 → FL2 → J5.2  LoRa    (LoRa_ACT,  S3)
       └─ U21 mux (vs USB) → V_MCU_2S → U18 TPS62152 → +3V3
+3V3 ─ S3 (U15), NOR/NAND (U11/U13/U16), INA230 → U30 → V_MCU_SWTCH (P4 U17,
       IMU, mag, BMP585, U20 buck, buzzer)
```

Flight load on the +3V3 chain: **0.5–0.8 A** (S3 + P4 domain; power-eco H-8
correction). The mini was 0.19–0.3 A — every hold-up number scales accordingly.

## The two decisive findings

1. **Both GNSS daughterboards (sam10m8-hv, px1105r-highpower) regulate through a
   TPS62913 — a 3–17 V buck.** They tolerate VBATT sagging to roughly 3.8 V.
   A firing sag (6.6 V pack − 1–2 V) never gets near that. **The only event that
   kills GNSS is the eFuse OPENING VBATT** when a >174 ms sag crosses the 6.34 V
   UVLO. Protect against the cut, not the sag.
2. **The P4 has zero unconnected GPIOs — but the ARM rework frees one.**
   `PYRO_ARM` (P4 pad 17) ceases to exist in the reworked arm architecture, and
   becomes `ARM_CLK` — the same pin-recycling that happened on the mini (GPIO8).

---

## Option A — recommended: mini-port ×2 supercap + deglitch stretch

**Pyro (identical in shape to the mini):**
- Delete R20 + C12 (V_CAP is gone; the Ø18 stood-off can and its adhesive
  constraints in FABRICATION-NOTES disappear — real board area back).
- Feed U6/7/8/10 sources from **VBAT_CON** (upstream of the eFuse), so firing
  current never meets the eFuse ILM (with camera + logic it would: 13 + 2–5 +
  0.8 A > 11.6 A).
- Port the arm protections verbatim: charge pump (doubler anchored to
  V_MCU_SWTCH, ~6 V gate), one-shot rework 2 (1 M / 10 µF-0805 timing
  `ARM_GATE`, D+22k reset, latch via the buffer's own output, release =
  V_MCU_SWTCH power-cycle — which the existing `POWER_SWITCH` / P4_EN_HOLD
  machinery already provides). `ARM_CLK` = the freed P4 pad 17.

**Hold-up (scaled for 0.5–0.8 A):**
- Same chain: V_BUCK split at U18 → LM66100 → V_HOLD → TPS22810 → +3V3, supercap
  behind 33 Ω charge / LM66100 discharge, TPS3840x29 supervisor, DEF high
  (3.465 V), CT cap.
- **Two CHP5R5L205R-TWQ in parallel (4 F, ~190 mΩ)**, one 33 Ω 2010 charge
  resistor **each** (keeps per-resistor dissipation at 73 % and τ at 66 s
  rather than doubling it):

| Store | 0.5 A | 0.7 A | 0.8 A | after P4-shed (0.25 A) | ESR bump @0.7 A |
|---|---|---|---|---|---|
| 1× 2 F | 1.3 s | 0.67 s | 0.47 s | 3.6 s | 330 mV |
| **2× = 4 F** | **3.4 s** | **2.1 s** | **1.7 s** | **7.9 s** | 197 mV |

- The 197 mV load-release bump still exceeds the supervisor's 75–125 mV
  hysteresis, so the mini's answer carries over: **CAP_ACTIVE → park logs →
  S3 drops `POWER_SWITCH` (P4 domain off)** → residual ~0.25 A rebounds inside
  the hysteresis → one clean cut. The CT cap defuses any corner chatter.

**GNSS protection: stretch the eFuse deglitch.**
- C94: 1 µF → **4.7 µF (τ ≈ 0.8 s)** or 6.8 µF (τ ≈ 1.2 s). A 500 ms fire sag
  rides through without the eFuse opening; GNSS, camera, LoRa all simply see a
  sag their regulators tolerate. The 6.34 V DC threshold — the pack protection —
  is untouched; a genuinely dead pack cuts ~1 s later than today, which is
  noise. This is the *designed role* of the deglitch (worklist H-1), just sized
  for the new firing transient.
- Side effect: battery plug-in gains ~1–2 s of eFuse turn-on delay. Harmless,
  but worth knowing on the bench.

**Delta:** ~2 supercaps + ~14 smalls + 1 cap value change; frees the C12 can
area; one new P4 net on a freed pin. Closest possible sibling to the mini —
same firmware contract (`ARM_CLK`, `CAP_ACTIVE`, `V_SCAP_ADC`, BOD, shed
order), which keeps the two firmwares congruent.

## Option B — budget variant: single supercap + fast P4 shed

Same as A but one 2 F cap. At 0.7 A the trip comes **0.67 s** after buck loss —
the P4 must park its state and be shed inside ~0.5 s, making firmware speed
load-bearing rather than belt-and-braces. Saves $4 and 13×7 mm. Take only if
the P4's park-and-shed path is measured fast on the bench first. (The Eaton
PM-5R0V155-R's 100 mΩ ESR makes a 1.5 F behave like ~2.5 F here — 0.92 s at
0.7 A — but at ~$17 it costs more than two CDAs.)

## Option C — GNSS local ride-through (add-on or substitute)

Repurpose the deleted **C12 (10 mF, 16 V, already on the BOM)** behind a plain
Schottky (CUS10S30 class — note the LM66100 is a 5.5 V part and CANNOT be used
on the battery rail) on the GNSS branch after U27:

```
U27 out ──►|── GNSS_HOLD ── FL1 ── J1.3      C12 10 mF on GNSS_HOLD
```

- 40 mA GNSS input draw × 0.5 s = 2 V droop: 7.4 → 5.4 V, far above the
  daughterboard buck's ~3.8 V floor. Covers a full eFuse cut of ~0.5 s, not
  just a sag.
- Limits: an 80 mA draw (px1105r acquiring, hot) for 1 s runs the cap dry.
  This is a *transient* bridge, not a hold-up.
- As an **add-on to A** it makes GNSS immune even to an actual eFuse trip
  (deglitch exhausted, tired pack + hot igniter + camera). As a **substitute**
  for the C94 stretch it protects only GNSS — camera and LoRa still drop on any
  trip. Recommendation: A now; add C if flight logs ever show an eFuse event.

## Option D — variant on the arm path: two-processor consent

Instead of `ARM_CLK` on the freed P4 pad, put it on one of the **S3's eight
spare GPIOs**: the S3 (command path from the ground) clocks the dead-man pump
while the P4 (flight state machine) pulses the per-channel FIRE pins — firing
then requires two processors to independently agree, a real interlock the mini
could not offer. Cost: an arm-consent message on the existing inter-MCU link,
and the arm latency of that hop; the mini/full firmware paths diverge.
Worth considering; not the default because it splits the pyro authority that
today lives entirely in the P4, and the LoRa stand-back test path (#803) would
need the consent hop audited.

---

## Complications register (all verified against the V10 netlist unless noted)

1. **LM66100 is a 5.5 V part** — fine everywhere on the 3V3 chain, unusable on
   VBATT. Any battery-rail ideal-diode (option C) is a plain Schottky or an
   LM74700-class controller.
2. **U18 (TPS62152) is a 1 A buck** now asked for 0.8 A load + 0.21 A supercap
   charge (2× caps) — peak 1.0 A. It current-limits gracefully, but charge time
   stretches when flight load is high. Acceptable; note it.
3. **Buzzer** hangs off V_MCU_SWTCH: post-landing siren on supercap power would
   eat the hold-up. Shed order already turns the P4 domain off, which silences
   it — but firmware should not re-enable the buzzer on cap power.
4. **P4_EN_HOLD interplay** (#848/#859): the in-flight rail-hold latch keeps
   U30 enabled through OC resets. The supervisor cuts *upstream* of U30, so the
   latch neither helps nor fights the cutoff — but the P4-shed step in the
   CAP_ACTIVE handler must release EN_HOLD or it will re-power the P4 the
   moment the rail bounces. Firmware detail, easy to miss.
5. **Camera during fire**: stays powered through the sag under option A; a deep
   sag may still reboot the camera (its own regulator's floor — unverified).
   Already accepted, but the deglitch stretch makes it *more* likely the camera
   sees the sag rather than a clean cut. Recording glitch, not a safety issue.
6. **J8 (JST-VH, ~10 A)** now carries 10–13 A firing pulses — better than the
   mini's JST-PH by 5×, still at rating. XT30 is the upgrade if the harness is
   ever revised; V8→V9 cable-compat rules apply.
7. **Q11 (AONR21321) and R72 (2 mΩ 0805)** sit in the firing path. Q11 is far
   inside its ratings; R72's 0.34 W/500 ms pulse needs the same pulse-rating
   check as on the mini. Bonus: the INA230 senses across R72 *upstream* of the
   eFuse, so firing current appears in pack telemetry for free (26 mV of its
   82 mV range at 13 A).
8. **USB masks everything** (TPS2121 mux): every ride-through and firing test
   must be run with USB unplugged, same as the mini.
9. **Pyro continuity dividers (R8–R19)** reference V_MCU_SWTCH, not V_CAP —
   they survive the V_CAP deletion untouched. The continuity-needs-battery
   behavior changes flavor: with the store gone, CONT sense is live whenever
   V_MCU_SWTCH is up, but a real fire still needs the pack.
10. **Fire-duration knob goes live** here too (`PYRO_FIRE_DURATION_MS` in the
    FC config, currently 200 ms) — same consequence as the mini.
11. **eFuse boot delay** from the bigger C94 (~1–2 s at battery connect) — the
    whole VBATT domain, including the buck, starts late. Everything downstream
    already tolerates slow ramps; flag for bench expectations.
12. **Board headers / parity sweep**: pad 17 changes meaning (PYRO_ARM →
    ARM_CLK), plus CAP_ACTIVE and V_SCAP_ADC pins on both MCUs. board_v10.h
    must follow, and the kicad-cli GPIO→net parity sweep is the only check.

## Recommendation

**Option A**, with **C as a flight-data-driven add-on**. It solves GNSS with a
single capacitor value change on a mechanism designed for exactly this job,
keeps the two boards' firmware contracts identical, and reuses every design
decision the mini review already hardened. Option D is the one genuinely new
idea worth a deliberate yes/no: it buys a two-processor arming interlock for
the cost of diverging firmware.
