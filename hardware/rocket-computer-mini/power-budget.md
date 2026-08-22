# rocket-computer-mini — 3V3 budget

**Purpose.** Decide whether the inherited 3V3 supply survives the reduction, once
the board carries a GNSS module and the 900 MHz telemetry radio on that rail.
The short answer is yes, with room to spare — but the reasoning matters more
than the number, because the rail's load *fell* during the reduction and the
obvious-looking comparison board is solving a different problem.

**Status.** Estimate, not a measurement. The topology below is verified against
the netlist; every current figure is part-class reasoning and none of it is
datasheet-confirmed. See *Before this is a closed budget* at the end.

---

## The rail as built

Verified from the exported netlist at the time of writing:

```
2S pack ──> mux ──> V_MCU_2S ──> L5 ──> [3V3 buck U18] ──> +3V3
                                                             │
                                        ┌────────────────────┤
                                        │                    │
                                  load switch U30       direct loads
                                        │              (out computer U15,
                                        │               its boot flash, monitor)
                                        v
                                  V_MCU_SWTCH ──> flight computer U32 + its flash,
                                                  the three sensors, GNSS,
                                                  LoRa radio, NAND
```

**Both processors are on this rail.** `U30` gates the flight computer as well as
the peripherals, so the switched branch is now the larger of the two — but it is
still the same buck, and the budget below is a budget for `U18`.

`U18` is a fixed-output synchronous **buck**, 1 A class, feedback tied to ground
for the internal divider. Its input is the muxed 2S pack rail through `L5`, so
the input is never below roughly 6.4 V — the pack's own cutoff — against a 3.3 V
output.

Sensors do not sit on `+3V3` directly. They hang off `V_MCU_SWTCH`, downstream
of load switch `U30`, and so still land on this budget.

## What the rail carries

Direct `+3V3` loads today are the out computer (`U15`), its boot flash (`U13`),
the current monitor (`U23`), and the load switch `U30`. Behind `U30` sit the
flight computer (`U32`) and its boot flash (`U33`), the three sensors, the GNSS
receiver, the LoRa radio and the NAND (`U11`).

With the two additions the inventory becomes:

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

The telemetry radio is the same 900 MHz module already carried by
[`../lora-daughterboard/`](../lora-daughterboard/) and
[`../base-station-mini/`](../base-station-mini/); identity lives in those BOMs.

## Scenarios

| Scenario | Composition | Total |
|---|---|---|
| Flight computer off | OC idle on `+3V3`, `U30` open — everything else dark | **~45 mA** |
| Pad idle | both MCUs idle, radio RX, GNSS tracking, no logging | **~160 mA** |
| Realistic flight | OC active + BLE, FC active, radio TX, GNSS tracking, logging | **~300 mA** |
| Worst credible | OC WiFi TX, FC active, radio TX, GNSS acquisition, NAND write | **~605 mA** |

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

Against a 1 A-class regulator, worst credible sits near **60 %**, realistic
flight near **30 %**.

> **These totals have not been re-argued from first principles since the second
> processor landed** — the flight computer was added to the inventory and its
> ~50 mA carried through the arithmetic, nothing more. The headroom conclusion
> survives the change comfortably, but if this becomes a closed budget the
> scenarios themselves should be rebuilt rather than incremented.

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

- **One switching regulator instead of two.** The second processor's dedicated
  buck left with it. One less inductor, one less feedback network, one less
  switching node near the sensors.
- **The switched rail lost its purpose.** `V_MCU_SWTCH` is gated by the MCU
  through `U30`, and its original job was powering the removed processor's
  supply. It now feeds only the sensors and some clamps. Whether the MCU should
  still gate its own sensors at boot is a live design question, and is not a
  power-budget question — see the note on it in [`README.md`](README.md).

## Before this is a closed budget

Four things, in the order they affect the answer:

1. **Does the MCU's own radio ever transmit while the telemetry radio does?**
   This single assumption is roughly 350 mA of the 555 mA worst case. If the two
   are mutually exclusive in firmware, worst credible falls to about 300 mA and
   the rail is barely working.
2. **GNSS antenna supply.** If the module needs antenna bias or an external LNA
   feed, that current is not in the table above — it was excluded because the
   configuration is not yet decided, not because it is negligible.
3. **Branch switching.** Both new loads are described here as sitting on the
   3V3 rail. Today the equivalent branches are switched, per
   [`../rocket-computer/high-side-switch-design.md`](../rocket-computer/high-side-switch-design.md).
   Hard-wiring them costs the ability to power-cycle a hung receiver or radio in
   flight, and their off-state isolation on the pad. That is a capability
   decision, not a supply one, and it should be made deliberately rather than
   fallen into.
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


---

# C56 — replacing the bulk polymer with ceramics already on the board

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
