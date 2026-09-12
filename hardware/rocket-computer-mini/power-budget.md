# rocket-computer-mini — 3V3 budget

**Purpose.** Decide whether the inherited 3V3 supply survives the reduction, once
the board carries a GNSS module and the 900 MHz telemetry radio on that rail.
The short answer is yes, with room to spare — but the reasoning matters more
than the number, because the rail's load *fell* during the reduction and the
obvious-looking comparison board is solving a different problem.

**Status.** Estimate, not a measurement. The topology below is verified against
the netlist; every current figure is part-class reasoning and none of it is
datasheet-confirmed. See *Before this is a closed budget* at the end.
**Rewritten 2026-09-12** for the hold-up converter and the fabbed board; the
2026-08-22 text is in git history.

---

## The rail as built

Verified from the exported netlist on 2026-09-12 (main `80e1860d`, the board as
fabbed):

```
2S pack ─> mux U21 ─> V_MCU_2S ─> L5 ─> [buck U18] ─> V_BUCK ─> [hold-up U47] ─> +3V3
                                                          │                        │
                                                    supercap C130 5 F        ┌─────┴──────────┐
                                                    on V_SCAP, via L11       │                │
                                                                        load switch U30   direct loads:
                                                                             │            out computer U15,
                                                                             v            boot flash U13,
                                                                        V_MCU_SWTCH       pack monitor U23,
                                                                        flight computer   USB mux U1
                                                                        U32 + flash U33,
                                                                        IMU, baro, mag, GNSS,
                                                                        LoRa radio, NAND U11
```

Two stages stand between the pack and the rail where the 2026-08-22 version of
this document had one:

- **`U18`** is the inherited fixed-output synchronous buck, 1 A class, feedback
  tied to ground for its internal divider and `DEF` tied high, so it regulates
  **3.465 V** — the +5 % option — on `V_BUCK`, not 3.3 V. Its input is the muxed
  pack rail through `L5`, never below roughly 6.4 V (the eFuse's own cutoff), so
  it is a buck by a factor of two at all times.
- **`U47`** is the hold-up converter added on 2026-08-29
  ([`holdup-tps61094-rework.md`](holdup-tps61094-rework.md)). With the pack
  present it is a **150 mΩ bypass switch** from `V_BUCK` to `+3V3` — the whole
  rail current flows through it, about 0.1 V of drop at 0.7 A — while it
  buck-charges the 5 F supercap `C130` on `V_SCAP` at **100 mA to a 2.5 V
  termination**, about two minutes from empty. When `V_BUCK` collapses it boosts
  from the cap to a **flat 3.0 V** until the cap is spent: about 21 s at 190 mA,
  13.6 s at 300 mA. It draws 60 nA once the cap is full.

So `+3V3` is `V_BUCK` less the bypass drop while the pack is present — about
3.4 V; the hold-up note walks the tolerance stack, with the ESP32-S3's 3.6 V
ceiling the closest — and 3.0 V on stored energy. **Both processors are on this
rail.** `U30` gates the flight computer as well as the peripherals, so the
switched branch is the larger of the two, but it is still the same buck, and the
budget below is a budget for `U18` with `U47` as a series element and one more
load.

Sensors do not sit on `+3V3` directly. They hang off `V_MCU_SWTCH`, downstream
of load switch `U30`, and so still land on this budget.

## What the rail carries

Direct `+3V3` loads are the out computer (`U15`), its boot flash (`U13`), the
current monitor (`U23`), the USB mux (`U1`) and the load switch `U30`. Behind
`U30` sit the flight computer (`U32`) and its boot flash (`U33`), all four
flight sensors — the magnetometer included, on the flight computer's own
`MAG_SCL`/`MAG_SDA` bus with its pull-ups on the same rail since 2026-08-22 —
the GNSS receiver, the LoRa radio and the NAND (`U11`). Upstream of the rail,
on `V_BUCK`, sits the one load that is new: the supercap charger inside `U47`.

With the GNSS module, the radio and the charger added the inventory becomes:

| Load | Condition | Estimate |
|---|---|---|
| Out computer | CPU active, radios idle | 40–60 mA |
| Out computer | BLE transmit | ~130 mA |
| Out computer | WiFi transmit, peak | ~350 mA |
| Flight computer | CPU active, **no antenna fitted** | 40–60 mA |
| Flight computer | held in reset by `U30` | 0 mA |
| Telemetry radio | transmit, 22 dBm class | 110–130 mA |
| Telemetry radio | receive | ~15 mA |
| Telemetry radio | sleep | µA |
| GNSS module | acquisition | ~45 mA |
| GNSS module | tracking | ~30 mA |
| NAND + NOR flash | writing | ~40 mA |
| Sensors + current monitor | active | ~4 mA |
| Hold-up converter `U47` | charging the supercap — the first ~2 min after a cold power-up, or after a hold-up event | **100 mA**, drawn on `V_BUCK` |
| Hold-up converter `U47` | cap charged, bypass | ~0 (60 nA) |

The telemetry radio is the same 900 MHz module already carried by
[`../lora-daughterboard/`](../lora-daughterboard/) and
[`../base-station-mini/`](../base-station-mini/); identity lives in those BOMs.

## Scenarios

| Scenario | Composition | Total |
|---|---|---|
| Flight computer off | OC idle on `+3V3`, `U30` open — pack monitor still readable, everything behind the switch dark | **~45 mA** |
| Pad idle | both MCUs idle, radio RX, GNSS tracking, no logging | **~160 mA** |
| Realistic flight | OC active + BLE, FC active, radio TX, GNSS tracking, logging | **~300 mA** |
| Worst credible | OC WiFi TX, FC active, radio TX, GNSS acquisition, NAND write | **~605 mA** |
| Flight computer off, cap charging | as the first row plus `U47` charging `C130` — the two minutes after a pad power-up | **~145 mA** |
| Pad idle, cap charging | as pad idle plus the charge current | **~260 mA** |
| Worst credible, cap charging | a launch inside two minutes of power-up — what the hold-up advisory (`hu`, #1166) exists to flag | **~705 mA** |

The flight computer's contribution is a flat ~50 mA in every powered scenario:
it has no antenna fitted, so it never reaches the WiFi or BLE transmit figures
that dominate the out computer's column. The first row is new and is the reason
the diode-OR exists — it is the pad-standby case, and the whole switched branch
is off in it.

Worst credible is a deliberately pessimistic alignment: it assumes the MCU's own
radio transmits at full tilt while the telemetry radio is mid-burst and the GNSS
has not yet locked. Whether that combination is reachable in flight is the single
biggest open question in this document.

## Headroom

Against a 1 A-class regulator, worst credible sits near **60 %** — **70 %**
while the supercap is still charging — and realistic flight near **30 %**.
`U47`'s bypass switch carries the same current in series and costs only its
drop, about 0.1 V at 0.7 A; what it can deliver *from the cap* in a hold-up
event is a different budget, argued in the hold-up note (at the 0.19 A cruise
load the boost could run the cap down to 0.45 V, and the converter's 0.7 V
input lockout stops it first).

> **Incremented, not rebuilt.** The 2026-09-12 rewrite re-read the rail from
> the fabbed netlist and added the hold-up stage and its charge current; the
> load figures are still the 2026-08-22 part-class estimates with the flight
> computer's ~50 mA carried through. The headroom conclusion survives both
> changes comfortably, but if this becomes a closed budget the scenarios should
> be rebuilt from datasheet figures rather than incremented again (item 4
> below).

The comparison that matters is historical rather than absolute. Before the
reduction this same rail fed the second processor's supply through `U30`, and
the project's own 2026-08-08 correction in
[`../rocket-computer/power-eco.md`](../rocket-computer/power-eco.md) puts that
combined load at a realistic flight figure of **0.5–0.8 A**. Adding a GNSS
module and a telemetry radio does not get back to where the board already was.

Two consequences follow:

- **The supply does not need changing.** It was sized for a heavier rail than
  the one it will now feed.
- **Input hold-up improves.** The ~3 ms ride-through in that correction was
  computed at 0.5–0.8 A. Hold-up scales inversely with load, so the same input
  capacitance buys proportionally longer at the reduced draw. This is the one
  number in the reduction that got better on its own.

## Why a buck, and not the ground station's buck-boost

The obvious move is to copy the supply from
[`../base-station-mini/`](../base-station-mini/), which was recently reworked.
That would be a mistake, and the reason is the battery, not the load:

| | base-station-mini | rocket-computer-mini |
|---|---|---|
| Source | single Li-ion cell, ~3.0–4.2 V | 2S LiPo, 6.4 V cutoff → 8.4 V |
| Relative to 3.3 V out | **straddles it** | always roughly 2× above |
| Required topology | buck-boost | buck |

A single cell falls *below* the 3.3 V output near end of discharge, where no
buck can hold regulation — hence a buck-boost on that board. A 2S pack never
approaches that condition. Porting the buck-boost here would buy a larger
inductor, more input ripple, more board area and cost, and lower efficiency, in
exchange for a boost stage that can never engage.

**The rule to carry forward:** topology is set by the source, headroom by the
load. The two boards differ in the first, which is why they should not share a
supply even though they share a 3.3 V rail.

## What the reduction changed

- **One switching regulator instead of two — for a week.** The second
  processor's dedicated buck left with it; the hold-up converter added on
  2026-08-29 is a second switcher again, but one that switches only while it
  charges the cap (about two minutes after a power-up) or rides it, and is a
  static bypass FET the rest of the time. The hold-up note carries the EMI
  reasoning.
- **The switched rail got its purpose back.** `V_MCU_SWTCH` was left gating
  only the sensors when the P4 went; with the second S3 it again carries a
  processor — the flight computer, its flash, the sensors, the GNSS receiver,
  the LoRa radio and the NAND — and its gating is the pad-standby and
  in-flight-hold mechanism described in [`README.md`](README.md). The
  2026-08-22 question of whether the MCU should gate its own sensors at boot is
  answered by that design.

## Before this is a closed budget

Four things, in the order they affect the answer:

1. **Does the MCU's own radio ever transmit while the telemetry radio does?**
   This single assumption is roughly 350 mA of the 605 mA worst case. If the two
   are mutually exclusive in firmware, worst credible falls to about 300 mA and
   the rail is barely working.
2. **GNSS antenna supply.** If the module needs antenna bias or an external LNA
   feed, that current is not in the table above — it was excluded because the
   configuration is not yet decided, not because it is negligible.
3. **Branch switching — decided 2026-08-22.** The GNSS receiver and the radio
   sit behind `U30` with the flight computer, on the one switch, and do not get
   switches of their own (the parent board's per-branch switches are in
   [`../rocket-computer/high-side-switch-design.md`](../rocket-computer/high-side-switch-design.md)).
   What that costs — no power-cycling a hung receiver or radio in flight without
   taking the flight computer down with it — is accepted; see *Open items* in
   [`README.md`](README.md).
4. **Confirm the figures.** Every current in this document is part-class
   reasoning. Before anything is committed to layout, replace them with
   datasheet values at 3.3 V and the actual operating modes, and re-run the
   three scenarios.

## Method

Topology, rail membership and the load-switch chain were read from a
`kicad-cli` netlist export of the schematic as it stood after the processor
removal, not from the schematic drawing or from prior documents. The 0.5–0.8 A
historical figure is quoted from the parent board's power ECO, which derived it
independently and against a board that no longer exists here.

Re-read on 2026-09-12 from a fresh netlist export of the fabbed board (main
`80e1860d`): the two-stage rail, the hold-up stage and the charge current above
come from that export and from
[`holdup-tps61094-rework.md`](holdup-tps61094-rework.md); the load figures were
not re-measured.


---

# C56 — replacing the bulk polymer with ceramics already on the board

**Done — drawn and fabbed.** `C56` is gone from the netlist; `V_MCU_2S` carries
`C47`/`C48`/`C49` (22 µF) and `C59` (1 µF), and `L5` is the same 2.2 µH part as
`L6` on the buck's output. The argument is kept as the record of why.

**Decision.** Delete `C56` (330 µF polymer tantalum, 16 V, ~50 mΩ ESR, 7.3 × 4.3 mm)
and fit **three 22 µF 16 V X5R 0805** in its place — the part already used at `C8`,
`C43` and seven other positions, so no new BOM line and no new footprint.

**Why this is not just a value change.** `C56` is the only polymer tantalum on the
board and the only part in its case size. Removing it deletes a line item, a large
footprint on a 22 mm-wide board, and the most expensive passive in the power path.

## The original justification no longer applies

`C56` was sized to ride out a specific event: *"when servos/camera sag the raw VBATT
node, the mux reverse-blocks IN1 and this cap holds VCC up"*
([`../rocket-computer/power-eco.md`](../rocket-computer/power-eco.md)). **Both
aggressors are gone from this board** — the servo branch and the camera branch were
removed during the reduction, along with their activation and current-monitor
signals. The transient it was sized against cannot occur here.

That document had already retracted its own 52 ms figure on 2026-08-08, moving the
job upstream to the EN/UVLO deglitch. It also records that the part was chosen partly
for *"BOM consolidation — same part as the servo cap"*. With the servo branch gone,
that rationale inverts: the shared part became a singleton.

## What actually constrains the value

**The mux imposes no minimum.** Verified three ways in the mux datasheet (TI
SLVSEA3F rev F): Recommended Operating Conditions has no capacitance row, Electrical
Characteristics has no C_OUT or ESR row, and the design procedure has three steps,
none of which is output-capacitor selection. Output capacitance appears only as a
user-supplied application input. The same document recommends *"low ESR ceramic
capacitors with X5R or X7R dielectric"* — ceramic is the preferred construction, not
a compromise.

**Inrush improves.** Soft-start slew is set by the SS capacitor alone, so inrush
scales with output capacitance: roughly 29 mA today, under 1 mA after. The existing
soft-start network gets gentler, not more stressed.

**The polymer was never the damping element.** `L5`'s DC resistance is 74 mΩ,
*larger* than `C56`'s 50 mΩ ESR, so the inductor already dominated damping of the
`L5`/`C43` input filter. Removing the polymer leaves filter Q essentially unchanged.
Checking the Middlebrook criterion: converter input impedance is 77 Ω at a full pack
and **20 Ω at the 6.4 V cutoff — the binding case** — against a filter peak output
impedance of 1–3.5 Ω. That is 16–27 dB of margin, and it is no worse after the change.

## Ceramic capacitance is not nameplate capacitance

**At 8.4 V these parts retain about 18% of their rating** — roughly 4 µF each, not
22 µF. Three give **~12 µF effective**, not 66 µF. Size against the effective figure;
any calculation starting from nameplate will be wrong by more than 5×.

*(This retention figure is single-sourced from the manufacturer's simulator. The
magnitude is consistent with published guidance for this class of part, but it was
not reproduced from a second document — treat it as good but not independently
confirmed.)*

## The margin, computed at the corner that binds

The event to survive is a USB removal while the pack is connected. **Switchover does
not begin at 5 V.** It triggers when the priority divider crosses the comparison
divider, at `V_trip = 0.498 × V_pack` — 4.19 V at a full pack, 3.19 V at cutoff. The
rail coasts down to that point *before* the 5 µs fast switchover starts, so the
budget is measured from the trip point, not from 5 V.

| Corner | Headroom to buck dropout | Dip at ~12 µF | Margin |
|---|---|---|---|
| Full pack, 8.4 V | 0.75 V | ~100 mV | ~7× |
| **Cutoff, 6.4 V** | **0.39 V** | ~100 mV | **~4×** |

Fast switchover is confirmed armed by the board's own dividers — the comparison
node sits at 2.09 V at a full pack and 1.59 V at cutoff, both above the 1.06 V
reference — so the 5 µs figure applies, not the 100 µs standard path. That is a
property of the fitted resistors, not an assumption.

Two caps would give ~8 µF and ~3× margin at the binding corner. **Three is specified
because the third costs nothing** — same line item, same footprint — and buys back
the margin lost to DC-bias derating.

## A pre-existing finding, not caused by this change

At the 6.4 V pack cutoff the switchover trip point (3.19 V) is **already below the
buck's dropout floor** (~3.44 V at worst-case load). Unplugging USB on a nearly-flat
pack therefore drops the buck into 100%-duty dropout **regardless of what is fitted
at `C56`** — the 330 µF does not prevent it either. Below the trip point the
remaining budget is to the converter's undervoltage lockout, about 0.39 V.

This is a property of the priority-divider ratio, and it is worth revisiting on its
own terms. It is recorded here because it was found while sizing `C56`, not because
the change causes it.

## What was not verified

- The retention figure above is single-sourced (see note).
- Worst-case switchover time is not published — the datasheet gives typical values
  with the min/max columns blank. All margins above use the typical figure.
- The output inductor's DC resistance was taken from distributor listings rather
  than the manufacturer datasheet.
