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
                                        │              (MCU, flash, monitor)
                                        v
                                  V_MCU_SWTCH ──> the three sensors
```

`U18` is a fixed-output synchronous **buck**, 1 A class, feedback tied to ground
for the internal divider. Its input is the muxed 2S pack rail through `L5`, so
the input is never below roughly 6.4 V — the pack's own cutoff — against a 3.3 V
output.

Sensors do not sit on `+3V3` directly. They hang off `V_MCU_SWTCH`, downstream
of load switch `U30`, and so still land on this budget.

## What the rail carries

Direct `+3V3` loads today are the MCU (`U15`), NAND and NOR flash (`U11`, `U13`),
the current monitor (`U23`), and the load switch `U30` feeding the sensors.

With the two additions the inventory becomes:

| Load | Condition | Estimate |
|---|---|---|
| MCU | CPU active, radios idle | 40–60 mA |
| MCU | BLE transmit | ~130 mA |
| MCU | WiFi transmit, peak | ~350 mA |
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
| Pad idle | MCU idle, radio RX, GNSS tracking, no logging | **~110 mA** |
| Realistic flight | MCU active + BLE, radio TX, GNSS tracking, logging | **~250 mA** |
| Worst credible | MCU WiFi TX, radio TX, GNSS acquisition, NAND write | **~555 mA** |

Worst credible is a deliberately pessimistic alignment: it assumes the MCU's own
radio transmits at full tilt while the telemetry radio is mid-burst and the GNSS
has not yet locked. Whether that combination is reachable in flight is the single
biggest open question in this document.

## Headroom

Against a 1 A-class regulator, worst credible sits near **55 %**, realistic
flight near **25 %**.

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
