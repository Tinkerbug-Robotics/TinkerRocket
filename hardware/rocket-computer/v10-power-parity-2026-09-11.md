# rocket-computer (V10) — power-tree parity with the mini, and the hold-up checked against the full computer

**Date:** 2026-09-11. **Scope:** the V10 schematic only (`power.kicad_sch`,
`bom.csv`). The board is untouched and is now 72 schematic-parity items
behind the schematic; the layout pass is the owner's and is listed at the
end. Everything below was taken from `kicad-cli` netlist exports of both
boards, never from the drawings.

**Requirement this note is written against:** through a brief power
interruption, losing the LoRa radio is acceptable, losing the camera and
servos is undesirable but tolerable, and losing GNSS is not — a GNSS restart
costs a flight sensor for tens of seconds. **Decision recorded the same day
(owner): the GNSS branch stays on the battery rail.** The carriers are built
for the pack rail and that reason stands, so GNSS protection has to happen on
the `VBATT` branch (section 1.3), not by moving the receiver onto the 3 V
supercap rail. A first-pass edit that did move it was reverted; nothing of it
remains in the files.

## 1. What was found

### 1.1 The buck output was never spliced onto the hold-up converter's input

`pack-fire-holdup-options.md` has said since 2026-08-26 that "one manual step
remains: the V_BUCK rail split at U18". It was never done. At HEAD before this
pass the V10 netlist read:

| net | members |
|---|---|
| `+3V3` | `L6.2` (buck output), `C53`, `R49`, `U18` VOS/FSW, `U47` **VOUT** 9/10, every 3.3 V load … |
| `V_BUCK` | `C141`, `R137`, `U18` DEF, `U47` MODE/EN/**VIN** — nothing else |

So the TPS62152 fed the rail directly, and the TPS61094's input hung on its
own capacitor and the `VBUCK_OK` divider. The converter could never charge the
supercap from anything but its own output through the bypass FET, and in a
pack-loss event the boost would have been fighting the dead buck on the same
node. The mini has had the split drawn since 2026-08-29 (`V_BUCK` = buck
output; `+3V3` = converter output), and its `holdup-tps61094-rework.md`
numbers all assume it.

**Fixed:** the `+3V3` power symbol on the buck-output junction at (136.525,
93.98) on the power sheet is now a `V_BUCK` global label, exactly the mini's
drawing. `V_BUCK` gained `L6.2`, `C53`, `R49`, `U18` VOS and FSW, and the
power LED `R65`/`D6`; `+3V3` is now purely the converter's output. `V_BUCK`
carries `C141` 22 µF + `C53` 10 µF, which keeps the TPS62152 inside the
Power-Save-Mode accuracy condition (COUT = 22 µF) that #999 needs. The power
LED now shows the **buck**, so it goes dark while the board rides the cap —
useful on the bench, and 1 mA less on the cap. If the LED is wanted on the
converter output instead, it is one wire cut and one label.

### 1.2 `VBUCK_OK` timing (#1165)

On the V10 the signal lands on the out computer's GPIO34, which has no ADC,
so the mini's re-ratio-and-read-as-ADC fix does not transfer. Option 1 of
#1165 is drawn: **`R140` 10 kΩ across `V_BUCK`**. With the buck output now on
that net the capacitance is 32 µF (`C141` + `C53`), τ = 0.32 s:

| after the bypass opens (V_BUCK ≈ 3.05 V) | time |
|---|---|
| pin leaves the guaranteed-HIGH band (V_BUCK < 2.475 V) | ~70 ms |
| pin at a guaranteed LOW (node < 0.75 V ⇒ V_BUCK < 0.825 V) | **~0.42 s** |

against 2–4 s of hold-up (section 3). Cost: ~350 µA off the buck while the
pack is present. No reservoir capacitor is needed (the mini's `C152`) because
this pin is read as a logic level. `R138` stays 1 MΩ.

### 1.3 GNSS stays on the battery rail — what that means for the hold-up

`U27` (TPS22810, enabled by `GPS_ACT` from the P4) switches **`VBATT`** — the
eFuse output — onto J1.3, and the V3 carrier regulates that with an
ADP7142-3.3 LDO. The carriers are designed and built for the pack rail; that
stays (owner, 2026-09-11). Two facts then bound what the hold-up can do for
the receiver:

- The carrier leaves the SAM-M10Q's `V_BCKP` **open** by design (carrier
  pre-fab review, item C6: "the host's power gating gives a cold start each
  cycle"). Any interruption of the branch, however short, restarts the
  receiver from a **23 s cold start** (data sheet TTFF) plus the multi-minute
  satellite ramp the field review measured. `C7` 22 µF holds the branch for a
  few milliseconds at the module's ~25 mA.
- The branch is upstream of nothing the cap feeds: the supercap boosts `+3V3`
  only, and no 3 V store can hold a 6–8 V branch.

So the GNSS's protection is the eFuse's UVLO deglitch (`C94`, section 3.3),
and it covers exactly the event the original requirement named — a **firing
sag never trips the eFuse**, so `VBATT` and the receiver never see a cut. What
it does not cover is a `VBATT` interruption of any length: a pack-connector
bounce, an eFuse overcurrent retry, or a severance. In those the processors,
sensors and log ride the cap (section 3) and the receiver cold-starts when the
branch returns. That is the accepted state of the design today.

**Options on the `VBATT` branch, for discussion — none drawn:**

- *Option C of the options doc:* a local store behind a Schottky on the GNSS
  branch, downstream of `U27`. At the module's ~25 mA a 4.7 mF/16 V can
  (Ø10 × 20) bridges ~0.7 s of a full cut (7.4 → 3.5 V at the LDO's
  headroom), a 10 mF can ~1.6 s. It covers bounces and the 92 ms eFuse retry
  cycles; it does not cover a severance. The diode drop costs nothing at
  these headroom levels.
- *A backup domain on the carrier:* a diode from VCC and a capacitor on
  `V_BCKP`. The receiver's backup current is ~3 µA, so a 1 mF/6.3 V part
  keeps `V_BCKP` above 1.65 V for minutes after a cut, turning the 23 s cold
  start into a ~1 s hot start with the ephemeris kept. The carrier review
  declined this (C6) because the host power-gates the branch; it is a
  carrier-revision item, not a V10 one, and the fabbed V3 carriers do not
  have it.

### 1.4 Second 22 µF at the converter output (mini #1012 port)

`C144` 22 µF 0805 16 V (`CL21A226MOQNNNE`, same part as `C17`) on `+3V3` with
its own label and ground — the TPS61094 wants 20 µF minimum / 30 µF nominal
**effective at the pin**, and one derated 0805 delivers ~16 µF. It must be
placed beside `C17` at pins 9/10 when the hold-up block is placed; anywhere else
it adds nameplate and fixes nothing.

### 1.5 The BOM was two edits behind the schematic

Reconciling `bom.csv` against the netlist designator-for-designator found the
2026-09-03 V10 edits had never reached it: `C141` was still on the 10 µF 0402
line (schematic: 22 µF 0805 `CL21A226KOQNNNE`) and `R134` still read 4.75 k
(schematic: 3.09 k `RC0402FR-073K09L`, the #999 fix). Both repaired, `C144` and
`R140` added: **83 rows, 260 designators, zero mismatches** against the netlist
(fiducials and mounting holes excluded, as before). An assembler ordering from
the old file would have fitted a 10 µF 0402 into `C141`'s 0805 land and left the
supercap never charging through `R134`.

## 2. Mini → V10 parity table

Every circuit change on the mini since the two boards last matched (the
TPS61094 rework, 2026-08-29), and where the V10 stands:

| mini change | date | V10 |
|---|---|---|
| #999 OSEL 3.3 → 3.0 V (`R134` 3.09 k); V10 also `C141` 10 → 22 µF | 09-03 | done 09-03 (BOM caught up today) |
| #1000 out computer off `VBUCK_OK`, `R138` 1 M → 360 k, ADC read | 09-03 | **does not port** — `R140` bleed instead (§1.2) |
| #1012 second 22 µF 0805 at `U47` VOUT (`C143` on the mini) | 09-03 | **done today** as `C144` |
| #1022 100 nF at the `V_SCAP_ADC` pin | 09-03 | done 09-05 (`C143` on the V10) |
| #1019 `R73` 1 k → 2.2 k | 09-03 | done 09-05 |
| rework-4 arm: Q12/Q13/Q14, R132/R139; U46/R133/D16/C140 deleted | 09-02/05 | done 09-05 |
| DTC123J → DTC123JETL on all six | 09-05 | done 09-05 |
| #1169 `C152` 100 nF on `VBUCK_OK` | 09-04 | not needed (logic read, §1.2) |
| #1014 100 k CS pull-ups on the shared memory/radio bus | 09-03 | mini-specific: the V10's NAND is alone on its bus and on the S3's own rail; `M_FLASH_CS` still has no pull-up, which only matters for the milliseconds the S3 is in reset — low, not drawn |
| #1015 `L_RXEN` off U0RXD + pull-down | 09-03 | mini-specific (on-board E220) |
| #1270 S3 brownout level (`sdkconfig.defaults.m1`) | 09-09 | mini-specific: the V10 FC is a P4 at SEL_5 = 2.42 V, which is the intended level |
| #1029 tired-pack case: eFuse re-enables only above 6.96 V, `FLT` unread | open | **applies**: same 6.34 V off / 6.91 V on divider on `U19`; `PG_RAIL` unread (#721). Mitigation is firmware — refuse arm below ~7.2 V by INA230 — plus the #721 wiring below |
| #1021 buck runs 2.2 µH at the 1.25 MHz FSW setting | open | **applies identically** (`L6` 2.2 µH, FSW high on both). Ripple current ~0.7 A p-p; peaks stay under `L6`'s 1.89 A saturation at 0.85 A load + 0.1 A charge. Note only |
| #1023 EN/MODE hard-wired to VIN, no true shutdown | documented | identical, same reasoning applies |
| GNSS on the switched 3.3 V rail (`V_MCU_SWTCH`, LC86G on-board) | 08-22 | **not ported by decision** — the V10's GNSS is a pack-rail carrier (§1.3) |

## 3. Does the hold-up work for the full computer?

### 3.1 What is on the cap, and what is not

| rail | loads | on the cap? |
|---|---|---|
| `+3V3` (converter output) | S3, NAND, both boot NORs, INA230, `U30` | yes |
| `V_MCU_SWTCH` (via `U30`) | P4 + its core buck, IMU, baro, mag, buzzer, pyro continuity sense | yes, while `POWER_SWITCH` is held |
| `V_BUCK` | nothing but the converter's input, the divider and the power LED | drains in ~0.4 s |
| `VBATT` (eFuse output) | **GNSS (`U27`, by decision)**, LoRa (`U29`), camera (`U26`), servos/EXP (`U28`), pyro gate pull-ups | no |
| `VBAT_CON` (pack, ahead of the eFuse) | pyro firing FETs | no (fires from the pack by design) |

### 3.2 Events, with the eFuse deglitch as drawn (`C94` 10 µF, τ ≈ 1.74 s)

| event | eFuse | `VBATT` loads incl. GNSS | `+3V3` chain |
|---|---|---|---|
| firing sag, healthy pack (7.8 → ~7.2 V, 0.2 s) | never trips (needs a sag below 6.34 V lasting > ~1.4–2.5 s) | ride it — GNSS LDO has >3 V of headroom, camera 5–20 V, LoRa buck, servos | untouched; the buck never leaves regulation |
| firing sag on a tired pack (7.0 → ~6.0 V, 0.2–0.5 s) | never trips | ride it | untouched |
| pack bounce < 0.27 s (connector, harness) | never trips | dark for the bounce; **GNSS cold-starts** | ~3 ms on `C56`, then the cap; back on the buck when the pack returns |
| pack bounce 0.3–1.0 s | trips at 0.27 s; re-enables **1.5–3.3 s after the pack returns** (at 7.4 V; faster on a full pack, slower on a tired one) | dark for bounce + re-enable = **1.8–4.3 s**; GNSS cold-starts | on the cap for the whole dark period — see 3.3 |
| eFuse overcurrent (harness short) | 92 ms retries, up to ~1024 | dark/blinking; GNSS cold-starts | on the cap; VBATT returns when the fault clears |
| pack exhausted (< 6.34 V, sustained) | trips; stays off until 6.91 V | dark | on the cap until it is spent, then off. Nothing to do but land |
| Rolly Polly V-class severance (V8, 2026-08-29: healthy pack, 20–30 g shock, gone at T+0.45 s) | — | dark | on the cap for the seconds below, then off. The V8 had no cap and went dark instantly |

### 3.3 How long the cap lasts

HV1020-2R7505-R, 5 F, terminated at 2.5 V, boost to 3.0 V at η ≈ 0.85, floor
set by the TPS61094 current limit (1.7 A worst / ~2.2 A typ, minus half the
ripple) or its 0.7 V SUP UVLO, 40 mΩ ESR included. The V9-class logic load is
0.5–0.8 A at 3.3 V (`power-eco.md` correction of 2026-08-08).

| `+3V3` load | worst (4 F, ILIM 1.7 A) | typical (5 F, ILIM 2.2 A) |
|---|---|---|
| 0.25 A — P4 domain shed, S3 + NAND only | floor 0.75 V, **13 s** | 16 s |
| 0.55 A — light flight load | floor 1.35 V, **4.6 s** | 6.6 s |
| 0.75 A — full computer | floor 1.82 V, **2.2 s** | 4.0 s |
| 0.85 A — full load, TX burst | floor 2.06 V, **1.3 s** | 3.1 s |

Two conclusions the owner should read directly from this table:

1. **At full load the worst corner is ~1.3–2.2 s**, which covers a firing
   event with margin to spare (it never touches the cap — the deglitch does
   that job), covers every bounce short enough not to trip the eFuse, and
   covers a bounce that does trip it only *typically*. The thing that makes a
   0.5–1 s bounce survivable at the worst corner is the **firmware shed**: on
   `VBUCK_OK` LOW, park the log, then drop `POWER_SWITCH`. Shed, the S3 lives
   13 s+ and logs the whole event; not shed, the whole board dies just as the
   eFuse would have re-enabled. This is the same choreography the mini's
   `firmware-notes.md` §4 describes and nothing implements yet on either board.
2. The eFuse's **re-enable delay after a trip is of the same order as the
   hold-up**, because the deglitch capacitor that stops a firing sag from
   tripping it also has to recharge through 1 MΩ afterwards. `C94` is a
   trade, not a fixed value:

| `C94` | τ | trips after a full disconnect of | trips on a 0.5 s sag to 6.0 / 5.5 V? | re-enable after a 0.3 / 0.5 / 1.0 s bounce |
|---|---|---|---|---|
| 1 µF (H-1 original, stocked) | 0.17 s | 27 ms | no / **yes** | 0.44 / 0.46 / 0.47 s |
| 4.7 µF (options-doc value, not stocked) | 0.82 s | 126 ms | no / no (0.67 s) | 1.3 / 1.6 / 1.9 s |
| **10 µF (as drawn)** | 1.74 s | 269 ms | no / no | 1.5 / 2.3 / 3.3 s |

Left at 10 µF: with the GNSS on `VBATT` the deglitch is the receiver's only
protection against a firing sag, and the sag it guards against on a tired
pack would end in a trip that does **not** self-clear (#1029: the eFuse waits
for 6.91 V). A slow re-enable is the lesser evil, and firmware can ride it
out. Revisit with the pack-yank bench data in #1211.

### 3.4 Every 3.3 V load at the 3.0 V hold-up level

| part | minimum | at 3.0 V (via `U30` for the P4 domain: −~50 mV) |
|---|---|---|
| ESP32-S3 `U15` | 3.0 V recommended, BOD 2.44 V | at the edge, above BOD — same acceptance as the mini (#999 section) |
| ESP32-P4 `U17` + `U20` core buck | 3.0 V recommended (3.3 V mode), BOD 2.42 V (SEL_5) | ~2.95 V, above BOD; same as the mini's switched S3 |
| NAND GD5F2GQ5UE, NOR GD25Q128E / W25Q128 | 2.7 V | fine |
| BMP585, ISM6HG256, IIS2MDC | 1.71 V | fine |
| INA230 | 2.7 V | fine |
| TPS22810 `U30` | 2.7 V (UVLO 2.62 V max) | fine |

Nothing on the rail is below its minimum; the two processors sit at their
recommended minimum with their brownout detectors 0.5 V lower, which is the
same posture the mini accepted. The GNSS branch is not on this rail (§1.3).

### 3.5 What the hold-up deliberately does not cover

Servos and the camera need 5–8 V at amps; no 3 V supercap can carry them and
none is attempted — `C15` 330 µF on the servo branch is milliseconds. LoRa
draws 110–130 mA in TX and every out-computer reset already power-cycles it;
it stays on `VBATT` as accepted. GNSS stays on `VBATT` by decision (§1.3).

## 4. Pick-up lists

### 4.1 Layout (owner)

- `R140` 0402 across `V_BUCK` — beside `R137`/`R138`.
- `C144` 0805 — beside `C17` at `U47` pins 9/10.
- The hold-up block still sits off the A4 page at x ≈ 300–440 mm; the
  new parts are drawn there too (`R140` at 311, 56; `C144` at 397.5, 38). The
  DEF pin's `V_BUCK` label at (106.68, 118.745) sits under the "USB power input
  for testing" note — pre-existing, worth moving at the same tidy.
- DRC with schematic parity now reports 72 items (was 64): 20 missing
  footprints (the 18 from #1180 plus `R140`, `C144`), 39 pad-net mismatches
  (the 32 from the earlier reworks plus the six nodes moved onto `V_BUCK`
  today), and the same 3 unconnected. `tools/check_board_parity.py` prints
  the 20-symbol list; its exemption text was refreshed.

### 4.2 Firmware (nothing here is implemented on either board yet)

- **`board_v10.h` is needed.** The V10 pin map no longer matches V9:
  `VBUCK_OK` S3 GPIO34, `V_SCAP_ADC` P4 GPIO17 (ADC1), `PIEZZO` S3 GPIO13
  (was P4 pad 18), `OC_ARM_EN` S3 GPIO14, `FC_ARM` P4 GPIO33 (was pad 17
  `PYRO_ARM`). The netlist parity sweep (`board-header-netlist-parity`) is the
  only check.
- `VBUCK_OK` on the out computer: **logic read, HIGH = buck present**; LOW is
  guaranteed ~0.4 s after the pack goes. On LOW: park the NAND, tell the P4,
  then shed the P4 domain (`POWER_SWITCH`) after a bounded delay; the P4 must
  release `P4_EN_HOLD` in that path or the rail bounces back (complication 4 in
  the options doc).
- The V_SCAP monitor (#1166, `holdup_policy.h`) reads the cap on the **out
  computer's** ADC on the mini. On the V10 the sense pin is on the **P4**, so
  the flight computer has to sample GPIO17 and carry the volts over the link,
  or the policy moves. `SCAP_ADC_PIN` is `-1` in `board_v9.h` today.
- Arm refusal below ~7.2 V pack (INA230), for #1029's permanent-trip case.
- After a `VBATT` cut the GNSS driver sees a cold-started module: the
  bootstrap sweep and the 18 Hz configuration have to be re-run, and the
  degraded-flight verdict should say so.

### 4.3 For discussion, not drawn

- **#721 `PG_RAIL` → a spare S3 GPIO.** S3 GPIO17 (pad 23) and GPIO18
  (pad 24) are the only free pins left on `U15`. One wire, `R59` already pulls
  it up, and it is the signal that tells the log *why* the board was on the
  cap (eFuse trip vs pack loss vs buck failure). Also the fix for the "`FLT`
  unread" half of #1029 on this board.
- GNSS branch ride-through per §1.3 (Option C on this board, or a backup
  domain on a future carrier).
- `C94` per §3.3.

## 5. Verification record

- Netlist (kicadxml, refdes-keyed diff, pre vs post): +`C144`, +`R140`; node
  moves exactly `L6.2`, `C53.1`, `R49.2`, `R65.1`, `U18.14`, `U18.7` (`+3V3` →
  `V_BUCK`). 272 components, 244 nets. `in_sensors.kicad_sch` is byte-identical
  to HEAD.
- ERC `--severity-all`: 1055 → 1054. One `endpoint_off_grid` gone (the deleted
  power symbol); the `power_pin_not_driven` note on `V_BUCK` moved from `U18`
  DEF to `U18` FSW (same net, same pre-existing class: a power input fed through
  `L6`); the GND/PGND `multiple_net_names` note re-anchored to a different
  symbol. No new category.
- `bom.csv`: 83 rows / 260 designators, 0 mismatches against the netlist.
- `tools/check_board_parity.py`: rocket-computer still exempt, 20 unplaced,
  text updated; every gated board unchanged.
- The on-page buck-output edit was rendered and inspected; the off-page
  additions were placed by coordinate against a scan of everything within
  20 mm.

## 6. Close-outs from the V9 review list (same day, second pass)

### 6.1 #664 — the S3's VDD_SPI decoupling is now 1 µF + 100 nF

`OUT_VDD_SPI` carried only `C27` 1 µF; the design guide asks for 1 µF + 0.1 µF
at the pin and there was no pad for the second part. `C145` 100 nF 0402 is
drawn beside `C27` on the S3 sheet, on the same wire, and goes at U15 pin 29 on
the layout pass. Everything else in #664 was already true on the fabbed V9:
the boot flash and the NAND run from `+3V3`, so the pin serves only the
in-package PSRAM, and `C27` has been 1 µF since the pre-fab close-out.

### 6.2 #665 — CHIP_PU through a rail cycle

Both reset RCs are in copper on V9 (`C25`/`R36` on the S3, `C39`/`R42` on the
P4, 10 ms each). The residual was the rail off-then-on case on the P4 domain:
when `U30` turns off, its quick-output-discharge pulls `V_MCU_SWTCH` to
ground through 250–400 Ω in about a millisecond, while `C39` (1 µF, referenced
to ground) can only bleed through `R42` into the dead rail at τ = 10 ms. On
paper `CHIP_PU` stays above the P4's V_IL for ~14 ms, so a rail brought back
inside that window would ramp with `CHIP_PU` already high — outside the 50 µs
t_STBL rule.

What bounds it:

- **The window is a coincidence, not a path.** The rail only drops when an
  out-computer reset outlasts `C105`'s 0.45–0.94 s hold on `POWER_SWITCH`; the
  re-assert then lands whenever the OC's boot reaches it. For the bad case that
  instant has to fall inside the 14 ms after the drop — a few percent of the
  spread even if every miss were uniformly timed — and a re-assert any later
  finds `CHIP_PU` low and the normal 10 ms delay in force.
- **The P4's own pad likely collapses the node with the rail.** An input pad
  clamped to its supply pulls `C39` down through the QOD path in ~0.3 ms, which
  would make the window moot; the datasheet gives the pin's absolute maximum as
  a fixed 3.6 V rather than VDD + 0.3, so the clamp is inferred, not specified
  (the same caveat as #1000 on the mini).
- **A boot that misses t_STBL is not a hang.** A P4 that fails to come up never
  asserts `P4_EN_HOLD`, and the out computer's restore policy re-cycles a rail
  whose flight computer stays silent (#825/#859, and #1129's budget on exactly
  that loop), so the next cycle starts from a fully discharged `CHIP_PU`. The
  cost is one extra rail cycle, ~1–2 s, in a case that already involved an OC
  fault reset.

What settles it: the scope check now on #1211 — `V_MCU_SWTCH` against U17
pin 103 through (a) an OC panic reset that exercises the restore path and (b)
a cmd-8 power-off followed by an immediate power-on, confirming `CHIP_PU` is
below V_IL before the rail returns. No part changes; the 100 nF this issue
replaced would have shortened the window ten-fold at the cost of the delay the
design guide asks for, and is not worth reverting.

### 6.4 #678 — the low-severity sweep, worked through

Every item mapped against the live files; ten were already done by the pre-fab
close-out or the later reworks (USB VBUS cap `C76`, GNSS/LoRa branch bulk
`C7`/`C18`, `R73` 2.2 k, both dangling slivers, the co-located via, the 12 pF
load caps, `U30`'s input cap 1.6 mm away, `C65` at the buck input). This pass
drew the rest of the schematic-side items and recorded the owner's decisions:

| item | decision / change |
|---|---|
| NRETRY `C45` 1 µF (~1800 retries, past the datasheet's last bucket) | **NRETRY tied to GND, `C45` deleted — indefinite auto-retry** with the 92 ms `C46` delay. A finite count ends in latch-off, which is the one terminal state a flight computer must not have; the datasheet's fault-response table gives "auto-retry indefinitely with finite delay" for this pin pair |
| status LEDs at 10 k (0.05–0.15 mA, invisible outdoors) | **`R65`, `R66`, `R70` → 1 k** (stocked line): ~1.5 mA green on the power LED, ~1.3 mA red, ~0.4 mA blue |
| `H2`'s plated ring 0.6 mm from the antenna body, on no net | **`H2` → `MountingHole_Pad`, pin 1 to GND.** The other seven holes stay unconnected; H2 alone sits in the antenna's near field. The pad's zone connection on the board is the layout pass's |
| `C44` soft-start 390 pF (~150 µs, startup in current limit) | `C44` → 10 nF (stocked): ~5 ms soft start, datasheet-class |
| S3 GPIO0 on the internal pull-up only | `R141` 10 k to `+3V3`, on the wire to the boot button |
| IMU and baro with no 100 nF (nearest was the magnetometer's at 3–4 mm) | `C146` at U2 VDD, `C147` at U4 VDD, both 100 nF on `V_MCU_SWTCH`; place at pin 8 of each |
| S3 RF supply with only `C33` 100 nF (guide: 10 µF per RF pin) | `C148` 10 µF on `Net-(U15-VDD3P3)`, converter side of `L4`; place at pins 2/3 |
| `ISM6HG256_INT2` single-pin net | no-connect flag; the label is gone. P4 GPIO16 is free if a second interrupt is ever wanted |
| `U9` symbol pin 9 with no pad in the shared TSON footprint | pin 9 renumbered **5** in `Custom.kicad_sym` and in both boards' cached copies — the footprint numbers the drain tab as a second pad 5, so the symbol now matches it and the permanent parity warning is gone on both boards. Netlists unchanged |
| eFuse timing caps drifted from the ECO record | `power-eco.md` Change 2 table rewritten to the as-built refdes and values |
| `C59` 1 µF where the ECO wanted 10 µF beside `C56` | **not changed**: a 16 V 10 µF is not a stocked 0402 and the mini's analysis showed the mux imposes no minimum. Open only if the bench shows a switchover dip |
| SC-32S ESR at the 70 kΩ limit | not changed; the fitted ABS07 is the same class. Bench: 32 k start-up at cold |
| NAND bulk, crystal placement, USB D+/D− and `CR3` stub, `C43` side (#868), `ESP_VDD_HP` (#867), In3 slivers | layout, on the pass list in §4.1 |

ERC on the V10 after this pass is 1063, all of it the pre-existing classes: the
new off-grid warnings are the half-grid y of the existing wires the new parts
hang from, `U9`'s cached-symbol mismatch is gone, and `NRETRY` grounded now
raises the same `pin_to_pin` note the grounded `IMON` pin already did, because
the symbol types both as outputs (the #680 hygiene list). The mini's ERC
drops by one (the same `U9` mismatch) and its netlist is unchanged.

### 6.3 #677 — mis-connection protection

Reverse battery is closed by `Q11` (AONR21321, ±25 V gate) with `CR2` deleted;
the PH-2-into-PH-4 mismate cannot happen since the battery moved to a JST-VH;
the reversed-jumper case is closed as a harness rule rather than a circuit —
the straight "A"-suffix part numbers, the catalogue-naming trap and the
pad-1-to-pad-1 check are in [`../cables.md`](../cables.md) and fab note B10.
A reversal-tolerant LoRa pinout is recorded there as a future-spin option, not
planned.
