# Three rockets on one base station — bench procedure

Branch `multi/integration`: base-station-run transmit slots (uplink and BS BLE 82),
the duplicate-rocket-ID flag (`dup`), gap-gated uplinks, and the iOS changes (next free
rocket ID, duplicate warning + pyro block, per-rocket map pins, focus kept on a rocket
that has flown, board/firmware per rocket, per-rocket profiles, slot switch, one CSV per
rocket from the shared base-station log).

Hardware for 2026-10-24/25: Mantis V5, Mantis V9 (UART radio daughterboard) and a
Beetle (radio on the NAND SPI bus), one V2 base station ("BaseStation V4", network 240).
Out-computer images and the base-station image come from this branch; flight-computer
images stay whatever each board flies.

**No e-matches or charges on any pyro channel during this bench.**

Analysis for every step: download the base station's log in the app (or over USB) and run

    python3 Data_Analysis/analyze_bs_multi_rocket.py <log.bin> --plot <step>.png

It prints, per rocket, received/expected frames, loss, the longest gap, how misses
cluster, and the transmit phase in the 500 ms cycle; per pair, the phase separation
(under 102 ms = FAST frames overlap on air); and the base station's slot adjusts.

## 0. Setup (both runs)
- Every rocket on the base station's network (ID 240), rocket IDs **1, 2, 3**, distinct
  names. Slot = (ID − 1) % 3, so IDs 1/2/3 get three different slots.
- Fixed frequency (the default). Base station logging on.

## 1. Baseline — today's images
- All three powered, READY, 10 minutes. Analyze.
- Expect: loss in the tens of percent per rocket, in streaks; pair separations that
  wander and sit under 102 ms for whole windows.

## 2. Flash the bench images
Out computers (V5 `build_v5`, V9 `build_v9`, Beetle `build_m1`) and the base station
(`build_v2`). Check each USB serial number (= MAC) before writing; the Beetle's FC and
OC share a port name.

## 3. Slots on (the default)
- Power all three. Within ~10 s the base station console's [STATS] line shows each
  rocket locked to a slot; the rockets log `[SLOT] adjust ... LOCKED`.
- 10 minutes. Expect: three phase clusters ~167 ms apart, near-zero loss, few adjusts.

## 4. Slots off, then on again
- App: base-station settings → transmit slots off. The base station broadcasts a
  release; the rockets log `[SLOT] release`. 10 minutes: loss returns toward the
  baseline. Turn slots back on; they re-lock within ~10 s.

## 5. Commands per rocket
- Camera and logging toggled from each rocket's own section change only that rocket.
- "Follow" on a non-focused rocket moves the base station's focus to it.
- LoRa pyro continuity test (no e-matches) on the focused rocket.

## 6. Duplicate rocket ID
- Set two rockets to the same ID. Within a few beacons the app shows the duplicate
  warning on that rocket and refuses the LoRa pyro test. Restore the IDs; the warning
  clears after three agreeing beacons.

## 7. Sim flight on one rocket
- Start a sim on one rocket while the others sit READY. Slots hold through INFLIGHT
  (the base station only adjusts a rocket more than 50 ms off while anything is in
  flight). After it lands and goes quiet, the focus stays on it and its pin stays on
  the map. The downloaded log splits into three per-rocket CSVs.
- Run it once on the Beetle: its radio shares the NAND bus, so check its phase spread
  while it logs stays well inside the slot guard (~65 ms).

## 8. Reboot mid-run
- Power-cycle one rocket. It comes back free-running and is re-slotted within ~10 s.

## 9. Soak — the go/no-go for slots in the flight build
- All three powered, slots on, ≥ 1 hour. Pass: no rocket loses its slot, loss stays
  near zero, adjusts are rare (drift corrections only). If it fails, the flight build
  ships with slots switched off at the base station.
