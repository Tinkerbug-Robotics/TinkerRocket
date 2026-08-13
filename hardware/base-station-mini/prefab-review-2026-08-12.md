# TinkerRocket Base Station Mini — Design Review

**Date:** 2026-08-12
**Board:** `hardware/base-station-mini/`, rev **V1**, never fabricated.
**Scope:** the whole board, schematic and PCB — but weighted toward what is *new*
or *changed* relative to `base-station`, since the unchanged majority is covered
by that board's [2026-08-02 pre-manufacturing review](../base-station/prefab-review-2026-08-02.md)
and by its fabbed V5 silicon.
**Method:** `kicad-cli` XML netlist as connectivity ground truth; `kicad-cli sch
erc`; `kicad-cli pcb drc --schematic-parity`; `pcbnew` Python for geometry,
stackup, zone fills and point-in-polygon copper probes; every claim about the
fabbed `base-station` and `lora-daughterboard` checked against their own netlists
rather than from memory.

**This is not a full datasheet-compliance sweep.** base-station's review pulled
primary datasheets for every fitted part; this one did not repeat that for the
inherited circuitry. Treat it as a review of the delta plus a structural pass,
not a replacement for a pre-fab review of the whole BOM.

## Headline

**The board is structurally sound and internally consistent.** Schematic and PCB
are fully in sync — 0 parity issues, 0 unconnected items — and all 25 remaining
DRC violations are silkscreen cosmetics. The three substantive changes from
base-station (on-board LoRa, fixed-output regulator, external charger removed)
are each wired correctly.

One finding is genuinely new to this board. The rest are inherited from
base-station, already documented there, and unfixed in both.

| # | Finding | Severity | Status |
|---|---|---|---|
| **A1** | LoRa antenna feed too narrow for the real stackup | low | **FIXED** 2026-08-12 — 0.20 → 0.36 mm, matching the `RF` class |
| **A2** | LoRa feed was not in the `RF` netclass | medium | **FIXED** 2026-08-12 — pattern added, zones refilled |
| **B1** | Zero thermal vias in `U9` PowerPAD and `U3` EPAD | medium | open; inherited (base-station **B3**) |
| **B2** | ~~base-station's review documents a wrong stackup~~ | **withdrawn** | see below — the review was right |
| **C1** | `external_charger.kicad_sch` orphaned on disk | tidiness | **FIXED** 2026-08-12 — deleted |
| **C2** | Three keepout zones exist on base-station, none on mini | unknown | open; needs eyes |
| **C3** | 25 silkscreen DRC violations, 2 now on the back layer | cosmetic | open; mostly inherited |

---

## A1 — The LoRa antenna feed is not 50 Ω

`J8` (SMA edge launch) is **new copper — base-station has no `J8` at all**, so
this trace has never been reviewed or fabbed.

`Net-(U16-ANT)` runs U16 pin 6 → J8 pin 1: 3 segments, **6.04 mm total, 0.20 mm
wide, all on F.Cu, no vias.** Reference plane is In1.Cu, which probes as solid
GND across the whole board, so the return path is continuous and unbroken —
that part is right.

The width was not. The 0.20 mm it was drawn at would be right over a **0.1 mm**
prepreg; over the board's real **0.2104 mm** prepreg it is far too narrow.
base-station hit exactly this and reached the same conclusion independently — its
finding **S7** widened that board's RF feed from 0.20 to **0.36 mm** on this same
stackup, and set the `RF` netclass to 0.36 mm / 0.15 mm.

**Correction to the first issue of this review.** The original text put the
as-drawn trace at 73.5 Ω from a *plain-microstrip* formula. That is too high: the
feed has a **0.1275 mm coplanar gap** to the F.Cu GND pour, making it grounded
coplanar waveguide, and the side coupling pulls Z0 well below the microstrip
value. The direction of the finding was right and base-station's S7 corroborates
it, but the specific figure overstated the mismatch. In either reading the trace
is 0.033 λ at 915 MHz, so the practical cost was small.

**Fixed 2026-08-12**, in two steps. First widened 0.20 → 0.40 mm, taken from the
plain-microstrip figure above. Then corrected to **0.36 mm** once the coplanar gap
was accounted for: as GCPW the side coupling pulls Z0 down, so 0.40 mm sat below
50 Ω, and 0.36 mm is both nearer the target and exactly what the board's own `RF`
netclass specifies — the value base-station's S7 derived for this same stackup.

Final geometry: **2 segments, 5.99 mm, 0.36 mm wide, 0.15 mm coplanar gap**, all
F.Cu, no vias, over a solid unbroken In1.Cu GND reference. Zones refilled after
the change; no ANT-related DRC items remain.

## B1 — Zero thermal vias in the PowerPAD and EPAD

This is base-station's finding **B3**, carried over unchanged:

| pad | size | net | vias in pad |
|---|---|---|---|
| `U9` pin 15, TPS63021 PowerPAD | 1.59 × 4.41 mm | GND | **0** |
| `U3` pin 57, ESP32-S3 EPAD | 4.1 × 4.1 mm | GND | **0** |

Confirmed identical on the fabbed base-station, so this is not a regression from
the fork — but it is also still not fixed, and both sibling boards
(`rocket-computer` 4 vias, `lora-daughterboard` 7) do it correctly.

Thermally the TPS63021 is fine here: at the ~520 mA worst-case load it dissipates
roughly 0.18 W, so even a via-less θJA gives under 10 °C rise. The stronger
argument is the one base-station's review already makes — for `U3` the EPAD is
the **primary ground return**, and for a switching converter the PowerPAD is
PGND, so this is about ground inductance and switching noise, not heat.

base-station's prescribed fix applies verbatim: a 3×3 array of 0.45/0.3 vias in
`U3`'s EPAD and a 2×4 column in `U9`'s PowerPAD, both the board's existing drill,
tented or plugged so paste does not wick.

## A2 — The LoRa feed was not in the `RF` netclass

Found while resolving B2. base-station's **S7** created an `RF` netclass (track
0.36 mm, clearance 0.15 mm) and assigned the antenna nets to it — that netclass is
how the fab is told the net is controlled impedance. The mini inherited the class
and its two patterns, `Net-(U15-Feed)` and `Net-(U3-LNA_IN)`, both of which are
the **ESP32 WiFi** feed. `Net-(U16-ANT)` — the new LoRa feed, the only net on this
board that did not exist on base-station — sat on `Default`.

So the one RF net unique to this board was the one net not declared as RF. S7
repeating itself on new copper.

**Fixed 2026-08-12** — `Net-(U16-ANT)` added to the `RF` pattern list and zones
refilled. The refill was not optional: the GND pour had been filled at the
`Default` 0.1 mm clearance, leaving a 0.1275 mm coplanar gap that violated the RF
class's 0.15 mm. Refilling pulled the pour back and cleared all four resulting
DRC violations. Fill area moved 9468.10 → 9467.62 mm² (−0.48), which is the gap
opening around the feed and nothing else.

## B2 — *Withdrawn.* The stackup is correct and so was base-station's review

**This finding was wrong and is retracted.** The first issue of this review
claimed base-station's review documented a stackup its board file contradicts.
It does not.

base-station's review header describes the stackup **as found at the start of
that review** — 0.1 / 1.24 / 0.1 mm, which is KiCad's untouched 4-layer default,
not an ordered stack. Its own finding **S7** then records the fix: *"Impedance not
declared — **FIXED** — JLC04161H-7628 stackup, RF netclass, feed widened
0.20 → 0.36 mm."* The header is the before, S7 is the after. Reading the header
as current truth was my error.

The board files were right all along. Both boards carry:

| layer | thickness |
|---|---|
| F.Cu | 0.035 mm |
| prepreg | **0.2104 mm** |
| In1.Cu | 0.0152 mm |
| core | **1.065 mm** |
| In2.Cu | 0.0152 mm |
| prepreg | **0.2104 mm** |
| B.Cu | 0.035 mm |

— which is **JLCPCB `JLC04161H-7628`** verbatim: the standard 1.6 mm 4-layer
stack, 1 oz outer / 0.5 oz inner, ENIG. That is the intended fab, so nothing here
needs changing. base-station's review header has been annotated to say so, since
quoting it out of context is what produced this false finding in the first place.

**What is genuinely inconsistent is elsewhere.** Two other 4-layer boards still
carry the KiCad default that base-station moved off:

| board | prepreg / core / prepreg | copper finish |
|---|---|---|
| base-station | 0.2104 / 1.065 / 0.2104 | ENIG |
| base-station-mini | 0.2104 / 1.065 / 0.2104 | ENIG |
| gnss-sam10m8-18mm-hv | 0.2104 / 1.065 / 0.2104 | ENIG |
| **servo-adapter** | **0.1 / 1.24 / 0.1** | **None** |
| **gnss-px1105r-…-ext-ant** | **0.1 / 1.24 / 0.1** | **None** |

(`rocket-computer` and `lora-daughterboard` are 6-layer on `JLC06161H-3313`, a
separate and correctly-declared stack.)

servo-adapter is passive, so its stackup is cosmetic. **`gnss-px1105r` is not** —
it has a GNSS antenna feed whose width was chosen against *some* stackup, and it
is tagged `v1.1.0`. Correcting its declaration without re-deriving that feed would
repeat A1 on a third board. Flagged, not touched.

## C1 — `external_charger.kicad_sch` was orphaned

The root sheet references `battery`, `esp32_p3` and `power`. `external_charger.kicad_sch`
sat on disk referenced by nothing, holding the only remaining copies of
`SDA_SENS`, `SCL_SENS`, `PosADC`, `MidADC`, `BatteryMid`, `BatteryPos` and
`CHG_VCC`, plus 25 symbols already absent from the netlist and BOM.

Precisely the *"source folders cloned from other boards kept the leftovers"*
pattern `hardware/README.md` calls out as a known hazard.

**Fixed 2026-08-12** — deleted. Verified after: 78 components, 90 nets, none of
those seven labels present, 0 parity issues, 0 unconnected.

## C2 — Keepout zones present on base-station, absent here

base-station carries three F.Cu zones with keepout attributes; base-station-mini
has none. I could not determine their purpose from the file headlessly, and they
may well have belonged to circuitry this board deleted. **Worth opening both
boards side by side to confirm nothing load-bearing was lost.**

## C3 — Silkscreen

25 DRC violations, all cosmetic: 17 overlaps, 6 edge-clearance, 2 non-mirrored
text on the back layer. base-station's numbers are 26/6/2, so this is the same
inherited class rather than anything new — with one exception worth a look: `BT2`
now sits on B.Cu on this board, so **check the two back-layer text items are not
the battery holder's own markings** before plotting.

---

## Verified correct

Recorded so the next reviewer does not re-derive it:

- **TPS63021 configuration is right for the fixed-output part.** `FB` (pin 3) ties
  directly to `+3V3`/VOUT with no divider — the adjustable part's 1 M/180 k
  network is gone. `VINA`/`VIN`/`EN` on `V_SWITCH`, `PS/SYNC` to GND (power-save
  enabled), `PG` unconnected, L1 (8,9) and L2 (6,7) across `L9` 2.2 µH. Input
  22 µF + 2×100 nF, output 4×22 µF plus distributed decoupling.
- **The +3V3 rail is driven.** It had no source at all between the boost being
  deleted and the TPS63021 landing; it does now.
- **All eight ESP32↔E220 signals match the fabbed `lora-daughterboard` exactly**
  — SCK/17, CS/18, MOSI/21, MISO/33, BUSY/34, RXEN/35, RST/38, DIO1/2. The
  `radio_board` firmware pin map applies unchanged.
- **`L_DI02` is a module-local DIO2→TXEN loopback**, so the SX1262 drives its own
  RF switch and spends no GPIO. `DIO3` unconnected. Both match the daughterboard.
- **No pull-up on `L_RST` is correct** — the fabbed daughterboard has the same
  bare two-node net.
- **`U15` needs no antenna keepout.** It is a Molex 47948-0001 *on-ground* LDS
  antenna; its datasheet says "Ground Clearance: None needed". Probes confirm
  solid copper on all four layers beneath it, which is the intended arrangement,
  not a defect. (Checked precisely because it looks like one.)
- **Planes:** In1.Cu solid GND, In2.Cu a +3V3 plane (2399 mm²), F.Cu/B.Cu GND
  pours — matching base-station's documented arrangement.
- **`BT2` sits at base-station's exact coordinates**, verified pad-for-pad:
  N at (58.84, 112.60), P at (58.84, 184.90), B.Cu, 90°.
- **Sync and routing are complete:** 0 schematic-parity issues, 0 unconnected,
  513 track segments, 251 vias, 12 zones filled.

## Before fab

Closed 2026-08-12: **A1** (feed widened), **A2** (RF netclass, zones refilled),
**C1** (orphan sheet deleted). **B2** withdrawn — it was not a real finding.

Still open:

1. Stitch the two EPADs (**B1**) — or record a decision to keep shipping without,
   as base-station has now done twice.
2. Confirm nothing was lost with the keepout zones (**C2**).
3. Clear or waive the silkscreen violations (**C3**), checking the two back-layer
   text items now that `BT2` is on B.Cu.
4. Then: `kicad-cli pcb drc --severity-error --schematic-parity` must be clean,
   bump nothing (V1 is correct — never fabbed), tag `base-station-mini-v1.0.0`,
   and plot with `tools/plot_gerbers.sh`.

Out of scope here but worth a ticket: `gnss-px1105r-18mm-highpower-ext-ant` still
declares KiCad's default stackup while carrying a GNSS antenna feed, and is
tagged `v1.1.0`.
