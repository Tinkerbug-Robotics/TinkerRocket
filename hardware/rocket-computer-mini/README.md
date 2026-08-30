# rocket-computer-mini

A reduced-capability flight computer. The design starts from a copy of
[`rocket-computer/`](../rocket-computer/) as it stood at the fork point, and
diverges from there by **removing** capability rather than adding it.

## This is a fork, not a variant

There is **no ongoing relationship** between the two boards. `rocket-computer-mini`
is not a derived revision, a build option, or a stripped configuration of
`rocket-computer` — nothing is shared, nothing tracks upstream, and a change to
one is never expected to propagate to the other. The starting geometry and
schematic were simply a convenient place to begin.

Treat anything inherited as a first draft to be justified on its own terms, not
as a decision already made.

## Two processors again — an Out Computer and a Flight Computer

The board arrived from the fork carrying `rocket-computer`'s ESP32-P4 and
ESP32-S3. The P4 was removed entirely, leaving the S3 alone for a while; a
**second ESP32-S3RH2 has since been added back** as the flight computer, and the
two-processor split is once more the one `rocket-computer` uses — with an S3 on
both ends instead of an S3 and a P4.

| | at fork | S3 alone | now |
|---|---|---|---|
| Parts | 255 | 156 | **196** |
| Nets | 234 | 139 | **200** |
| Sheets | 5 | 4 | **5** |

*Parts and nets are netlist component and net counts. The "S3 alone" column
supersedes an earlier 190/191 in this table, which was measured before the
expansion header, camera, servo and piezo came out.*

### Who owns what

| | Out Computer `U15` | Flight Computer `U32` |
|---|---|---|
| Sheet | [`esp32s3_mcu.kicad_sch`](esp32s3_mcu.kicad_sch) | [`fc_esp32s3.kicad_sch`](fc_esp32s3.kicad_sch) |
| Rail | `+3V3` — always on | `V_MCU_SWTCH` — **starts off** |
| Radio | 2.4 GHz chip antenna `U31`, 900 MHz LoRa `U16` | none |
| Memory | NAND `U11` + boot flash `U13` | boot flash `U33` only |
| Sensors | pack monitor only, on `SEN_SC*` (`+3V3`) | IMU, baro, GNSS **and magnetometer** |
| Pyro | none | all four channels and `PYRO_ARM` |
| USB | `OC_D±` | `FC_D±` |

The flight computer is a copy of the out-computer sheet with the chip antenna,
its matching network and the NAND removed; everything else — the MCU, its own
boot flash, both crystals, the boot button, the strapping network and the full
decoupling set — is duplicated. Its parts occupy a deliberate `C110`/`R110`
block so the sheet is identifiable at a glance in the BOM.

### The flight computer starts off, and holds itself on

This is `rocket-computer`'s arrangement, reproduced part for part:

```
FC_EN_OC   (OC GPIO7) ──>|─┐
                          ├── POWER_SWITCH ──> U30 EN/UVLO ──> V_MCU_SWTCH
FC_EN_HOLD (FC GPIO17) ─>|─┘        │  │
                        D9          │  └── C105 10 uF   (rides out a drop-out)
                     BAV170M        └───── R84 100 k    (holds the rail off)
```

At power-on `R84` holds `U30` off, so `V_MCU_SWTCH` — and with it the flight
computer, the sensors, the GNSS receiver, the radio and the NAND — is dead. The
out computer boots on the always-on `+3V3` rail and raises `FC_EN_OC` to start
the flight computer, which then asserts `FC_EN_HOLD` to hold its own rail up.
**An out-computer reboot in flight can no longer power the flight computer
down**, which is the whole point of the diode-OR.

### The hold has to survive the flight computer's own crash

`rocket-computer` closed #825/#848 in firmware (PR #859): the flight computer
drives its hold pin HIGH **and calls `gpio_hold_en`**, so the pad stays latched
through the FC's own panic/WDT/SW resets. That is what makes the latch a real
in-flight guarantee rather than a race against the `R84`/`C105` decay — without
it, an FC crash mid-flight drops the rail and takes all four pyro channels with
it, ballistic.

**That mechanism ports to this board, and the pin was chosen for it.** On the
ESP32-S3 `gpio_hold_en()` only reaches the RTC-domain latch for RTC-capable
pads; ESP-IDF dispatches to `rtc_gpio_hold_en()` when `rtc_gpio_is_valid_gpio()`
passes, and falls back to the digital hold — which is **deep-sleep only** —
otherwise. The S3 has `SOC_RTCIO_PIN_COUNT = 22`, covering **GPIO0–GPIO21**, and
`RTCIO_GPIO17_CHANNEL` exists. So `FC_EN_HOLD` on GPIO17 latches exactly the way
`P4_EN_HOLD` does on the P4's GPIO5.

> **Constraint for whoever writes the flight-computer board header: the hold pin
> must stay inside GPIO0–GPIO21.** Move it to GPIO26 or above and
> `gpio_hold_en()` silently degrades to a deep-sleep-only hold — the call still
> returns `ESP_OK`, the latch still looks asserted, and it evaporates on the
> first panic reset. That reintroduces #825 with no error to notice it by.

The decay window is unchanged from `rocket-computer`: `R84` 100 k and `C105`
10 µF give roughly 1.4 s from 3.3 V to `U30`'s enable threshold. Note the decay
is on the *enable* node, not on `V_MCU_SWTCH` — `U30` is a load switch, so its
output stays at full rail until the enable crosses the threshold and then
collapses. The flight computer is fully powered for the whole window, with no
brownout race on the way down.

One difference from `rocket-computer`, and it favours this board: there the
radio has its own switch, so a flight-computer hold does not keep it alive.
Here the radio and the GNSS receiver both ride `V_MCU_SWTCH`, so a held rail
keeps telemetry *and* position up — which is what the downed-rocket tracker mode
behind that firmware actually wants.

`FC_EN_HOLD` sits on **GPIO17, not GPIO3**. GPIO3 is where `POWER_SWITCH` lived
on the single-MCU board and is where a copy of that sheet would have left it —
but GPIO3 is a strapping pin, and inserting `D9` puts a diode between it and
`R84`, so it would have booted floating. GPIO17 is unconditionally free. The
move also decouples the latch from boot configuration: `gpio_hold_en` on GPIO3
would have held a *strapping* pin HIGH through every in-flight reset. The same
problem in reverse is why `R34` (100 k) pulls `ESP_I2S_SD` down on the out
computer: that net is GPIO3 *there*, driven by a flight computer that is
unpowered while the out computer boots.

**The flight computer's GPIO3 did not stay bare, and this file said it had
until 2026-08-30.** The supercap hold-up rework landed `VBUCK_OK` on it, through
the `R137` 100 k / `R138` 1 M divider off `V_BUCK`. That keeps the property the
move was made for — the pad has a defined level at reset, set by a passive
divider rather than by a driven pin — so the strapping argument still holds;
it is simply no longer true that nothing is connected there. See the correction
section in [`pin-budget.md`](pin-budget.md).

Note the consequence of the mini's rail split, which differs from
`rocket-computer`: here the LoRa radio and the NAND sit on `V_MCU_SWTCH` rather
than on a switch of their own, so "flight computer off" also means "no telemetry
and no logging". `rocket-computer` gives the radio its own switch (`U29`) and
can therefore transmit on the pad with the flight computer asleep. That is a
deliberate choice for this board, not an oversight — see *Open items*.

### The link between them

Six wires, the same net names and the same protocols as `rocket-computer`:

| Net | OC pin | FC pin | Role |
|---|---|---|---|
| `ESP_I2S_BCLK` | GPIO2 | GPIO21 | I2S bit clock |
| `ESP_I2S_WS` | GPIO1 | GPIO18 | I2S word select |
| `ESP_I2S_SD` | GPIO3 | GPIO13 | I2S data, FC → OC |
| `ESP_I2S_FSYNC` | GPIO4 | GPIO14 | frame sync — a plain GPIO, not an I2S signal |
| `ESP_SCL` | GPIO6 | GPIO33 | I2C clock |
| `ESP_SDA` | GPIO5 | GPIO35 | I2C data |

**The four I2S nets were called `ESP_SCLK` / `ESP_CS` / `ESP_SDO` / `ESP_SDI`
until 2026-08-30**, on this board and on `rocket-computer`. Those names read as
SPI and the link has never been SPI; boards fabbed before that date carry the
old labels, and the GPIO numbers are the same either way. `ESP_I2S_FSYNC` is the
odd one of the four — the peripheral does not drive it, the master pulses it
around each `writeFrame()` and the slave reads it to resync.

`R115`/`R116` (5.11 k to `V_MCU_SWTCH`) pull the I2C pair up, matching
`rocket-computer`'s `R55`/`R58` — tied to the switched rail rather than `+3V3`
on purpose, so they are not two more paths feeding a dead rail. `R34` (100 k)
pulls `ESP_I2S_SD` down; see above.

**The magnetometer belongs to the flight computer, and getting there took two
tries.** It is a flight sensor, so it should always have sat with the IMU, the
barometer and the GNSS — which is exactly where `rocket-computer` puts its
`IIS2MDCTR`, on the flight computer's own I2C with the pull-ups on the same
rail as the part. The mini instead shared it onto the out computer's
power-monitor bus, a single-MCU decision (`pin-budget.md`: *"shares the
power-monitor I²C bus — no new pins"*) that nothing revisited when the board
grew a second processor.

That left it stranded in two ways at once. Electrically, `R67`/`R69` pulled up
to `+3V3` while `U3`'s supply sat behind `U30`, whose QOD actively discharges
the rail — so in pad standby those pull-ups drove its I2C pads against a
grounded supply, above abs-max, clamping the bus below VIL and making the
INA230 unreadable. That was the whole of the board's battery telemetry, since
nothing else reaches `VBAT`. And in firmware, every magnetometer driver in the
tree is built into the flight computer, which could not reach a part on the
other processor's bus — so the board had no heading source at all.

Both are gone now: `U3` moved to `MAG_SCL`/`MAG_SDA` on the flight computer
(GPIO37/GPIO36), with `R117`/`R118` (5.11 k) pulled up to `V_MCU_SWTCH` — the
same rail as the part and its master. `SEN_SCL`/`SEN_SDA` is a pure pack-monitor
bus on `+3V3`, the same shape as `rocket-computer`'s `PWR_SCL`/`PWR_SDA`. No
passive pull-up crosses the rail boundary any more; every remaining crossing is
an actively driven signal.

The part exposes no DRDY on this land — every pad but SCL/SDA/VDD/GND is NC —
so the magnetometer is poll-only. That is the part, not an omission.

**Firmware constraint.** All six of these cross the `+3V3` / `V_MCU_SWTCH`
boundary, and they are the only signals live during pad standby with the flight
computer off. The out computer must park every one of them Hi-Z whenever
`V_MCU_SWTCH` is down, or it injects through the flight computer's ESD diodes
into the dead rail. This exact failure already bit `rocket-computer` — the
FC-relay OTA contention, where `i2s_del_channel()` left BCLK/WS/DOUT driven.

**The out computer's six GPIO numbers are identical to `rocket-computer`'s**, so
`projects/out_computer` ports across with a board header and nothing else. The
flight computer matches on I2S BCLK (GPIO21) and WS (GPIO18) but **not** on the
data pair: `rocket-computer`'s P4 uses GPIO19/20, which on an S3 are the USB
D−/D+ pads and are spent on `FC_D±` here. `I2S_DOUT_PIN` and `I2S_FSYNC_PIN`
become 13 and 14 in the flight-computer board header.

### Either processor can be flashed over the one USB port

`U1`, an FSUSB63UMX 2:1 high-speed mux on the root sheet, sits between the USB-C
connector and the two processors; `S1` (JS202011JCQN) selects which one the host
sees. Both parts, and the `R1`/`R2`/`R3`/`C1` around them, are copied from
`rocket-computer`'s `U1`/`S1`. Without it the flight computer would be
reachable only through an OTA relay, which cannot recover a bricked board.

### What the P4 took with it, and where it landed

Removing the P4 orphaned **34 named signals**. All of them are now either
re-homed on the flight computer or deliberately gone:

| Was orphaned | Now |
|---|---|
| IMU / baro SPI + interrupts, GNSS UART | flight computer, same GPIO as the single-MCU map |
| all four `PYRO*_FIRE` / `PYRO*_CONT`, `PYRO_ARM` | flight computer, same GPIO as the single-MCU map |
| the six `ESP_*` link lines | live again, both ends populated |
| `P4_EN_HOLD` | `FC_EN_HOLD`, into `D9` |
| all twelve `EXP_01`–`EXP_12` | removed |
| camera UART, `PIEZZO`, `SERVO_ACT` | removed |

Because the moved signals kept their GPIO numbers, the sensor, GNSS and pyro
halves of `pin-budget.md`'s assignment carry over unchanged — they simply belong
to `U32` now rather than `U15`. The out computer keeps the radio, the memory
bus, the power-monitor I2C and USB, and gains twelve spare pads.

## Design docs

- [`power-budget.md`](power-budget.md) — the 3V3 rail after the reduction, with
  a GNSS module and the telemetry radio added. Concludes the inherited buck
  stays, and explains why the ground station's buck-boost must not be copied
  here.
- [`pin-budget.md`](pin-budget.md) — whether the single processor can carry the
  board alone. 23 signals into 27 usable pads, with a proposed assignment that
  keeps the serial console and spends JTAG. **Written against the single-MCU
  board**; the pin *assignments* still hold, but they are now split across two
  processors — see *The split* at the end of that document.

Both budgets predate the second processor. `pin-budget.md` has a section
covering the split; `power-budget.md` has the flight computer added to its load
table, but its scenario totals have not been re-argued from scratch.

## What was and wasn't carried over

Copied and rewritten for the new project name: the root schematic, the five
sub-sheets (`in_sensors`, `power`, `central_processing_p4`, `esp32s3_outputs`,
`external_connections`), the PCB, the project file, the library tables, and
`bom.csv`. Of those, `central_processing_p4` has since been deleted,
`esp32s3_outputs` renamed to `esp32s3_mcu`, and `external_connections` renamed to
`pyro` — the sheet now carries only the four pyro channels, the expansion header,
camera, servo and piezo having been removed. A fifth sheet,
[`fc_esp32s3.kicad_sch`](fc_esp32s3.kicad_sch), was added later: it is a copy of
`esp32s3_mcu` carrying the second ESP32-S3RH2, so the sheet count is back to
five — but the sheet it replaces is not the one that was deleted.

Deliberately **not** copied — `rocket-computer`'s seven design documents. All of
them are records of *that* board, and each would be actively misleading here:

- **The reviews** — `prefab-review-2026-07-30.md`, `prefab-review-2026-08-05.md`
  and `schematic-review.md`. Carrying them would assert this board has been
  reviewed when it has not.
- **`WORKLIST.md`** — the closing record of the V9 pre-fab review, written
  against V9's live files. Its board state, its closed items, and its bench list
  describe a board this one will not be.
- **`FABRICATION-NOTES.md`** — fab and assembly instructions keyed to V9's exact
  geometry (22.35 × 75.00 mm, 6 layer, 1.546 mm stack) and its specific parts.
  A reduced board will not share those numbers, and stale fab notes are the kind
  of error that reaches a fab house.
- **`power-eco.md`** and **`high-side-switch-design.md`** — design rationale for
  the V9 power architecture. These remain the best reading on *why* the
  inherited power tree looks the way it does; read them in
  [`../rocket-computer/`](../rocket-computer/) rather than in a copy here that
  would drift.

## Inherited state — read before editing

The fork is faithful: at the fork commit the exported netlist, ERC report and
DRC report are identical to `rocket-computer`'s apart from the project name and
timestamps (255 components, 234 nets). That also means the **inherited
violations came along with it** — 1012 ERC at `--severity-all` and 26 DRC, plus
11 schematic-parity items, 0 unconnected. None were introduced by the copy, and
none have been fixed here.

### Where the numbers stand now

| | at fork | after `rev1` | after P4 removal | after the second S3 |
|---|---|---|---|---|
| ERC (`--severity-all`) | 1012 | 1012 | 823 → 610 | **739** |
| DRC (`--severity-all`) | 26 | 28 | **88** | not re-run |
| Schematic parity | 11 | 11 | **8** | not re-run |
| Unconnected | 0 | 0 | **0** | not re-run |

*The 823 in the third column was measured before the expansion header, camera,
servo and piezo were removed; the same board measures 610 today, and that 610 is
the baseline the 735 is a delta against. DRC and parity are unmeasured because
the second processor is schematic-only so far — **the PCB has not been
touched**, so every board-side number above is stale by construction.*

The +129 ERC items are all in categories the board already had, and none of them
is a new wiring defect:

| Added | Why |
|---|---|
| 98 `endpoint_off_grid` | inherited with the copied sheet geometry (515 already present) |
| 19 `pin_not_connected` | spare pads — twelve freed on `U15`, seven unused on `U32`. Deliberately left bare rather than flagged, because they are spares, not decisions |
| 6 `pin_to_pin` (warning) | the flash symbol types its bus pins *Unspecified*; `U13` already does this against `U15` |
| 3 `power_pin_not_driven` | power inputs fed through a passive — `L10` into `U32`, `R3` into `U1` — same shape as the five already present |
| 1 `pin_to_pin` (error) | the flash symbol types GND as a *power output*; identical to `U13`'s existing pair |
| 2 `lib_symbol_mismatch` | `U33`/`Y3` inherit the cached-symbol drift `U13`/`Y1` already report |

Every pre-existing DRC category went *down* with the parts count. The rise to 88
is one thing: **66 `track_dangling` + 2 `via_dangling`** — stubs that used to run
to a P4 pad on nets that still have pads elsewhere, so they survived the
dead-net sweep.

**Left in place deliberately.** That rewire has now happened — the orphaned
signals belong to the flight computer — but it happened *in the schematic only*.
The board still carries the P4-era stubs and has no footprint for `U32`, `U33`,
`U1`, `S1` or `D9` at all, so the layout pass those stubs were waiting for is
still outstanding and still the right time to clear them.

Three stale items are worth knowing about, none fixed:

- **`C12`'s footprint disagrees between schematic and board.** The pyro energy
  store was changed from the Nichicon `EKYC160ELL103MM25S` to a Rubycon 2200 µF
  16 V (`16ZLH2200MEFC12.5X20`) in a horizontal lay-down can with a hold-down
  strap — which is what the inherited fab notes' "can floating 4 mm above the
  board" warning was about. **The schematic carries the change; the PCB still
  has the old vertical footprint.** Swapping it on the board means re-placing
  C12 and re-routing it, which is layout work for the next pass.
- **`bom.csv` is still V9's full bill of materials** — 255 parts, including the
  65 that no longer exist, and now also missing the 40 parts the second
  processor added. The C12 row is the one line that is current.
- **The on-board fabrication-note text still cites `QFN-104 (U17)`** as the
  reason ENIG is required. `U17` was the P4 and is gone. ENIG is still justified
  by `U15` (0.4 mm QFN-56) and `U21` (0.4 mm X2QFN), but the citation is dead and
  the note also still calls the board "ROCKET COMPUTER" at V9's dimensions.

### The revision, and what changing it cost

The title block arrived reading `(rev "V9")` — the revision of a *different*
design's fab release — and the front silkscreen renders
`Tinker\nRocket\n${REVISION}` from it. It now reads **`rev1`**, this board's own
first revision.

That is the only edit made since the fork, and it is not free. `rev1` is two
characters wider than `V9`, so the silkscreen block grew and now touches R1:

| | at fork | after `rev1` |
|---|---|---|
| DRC (`--severity-all`) | 26 | **28** |

The two new items are `silk_overlap` (the text against R1's silkscreen segment)
and `silk_over_copper` (the text against R1's pad 1), both at
`@(72.64 mm, 159.51 mm)`, both severity *warning*. They are cosmetic and in the
same class as the 26 already inherited — but they are **new, not inherited**,
and they are a layout problem, not a naming one.

Deliberately not fixed by nudging the text: this board exists to have most of
its content removed, and that silkscreen will have to be repositioned anyway
once the layout is cut down. Fixing it now would be laying out a board that is
about to stop existing. Whoever does the reduction should clear it then — or
shorten the string, since a two-character revision (`V1`, matching the `V9` /
`V5` convention on the other boards) reoccupies the original footprint exactly
and drops both warnings.

## Status

**Schematic-complete for the two-processor split; the PCB has not been laid
out.** The flight computer, its power switch, the diode-OR enable and the USB
mux exist in the schematic and export a clean netlist — **214 components, 205
nets** (2026-08-30, after the arm and TPS61094 hold-up reworks; this paragraph
said 196/200 before those landed), no duplicate references and no single-node
nets.

Three things about the board file, all verified 2026-08-30:

- **The footprints now exist but are not placed.** `U32`, `U1`, `S1`, `D9`,
  `C105`, `R34` and the rest have been imported — this file used to say they had
  no footprint at all. Twenty-two of the newest parts (`C114`–`C125`, `L9`,
  `L10`, `R110`–`R118`) sit in an import scatter block at x > 130 mm, roughly
  45 mm clear of where the board is. DRC reports 330 unconnected pads.
- **There is no board outline.** `Edge.Cuts` carries no geometry — only the
  layer declaration. The 22.0 × 54.8 mm outline existed at `0e0f2d5` and went
  missing in `7ad7508` (the pack-direct pyro/supercap WIP commit) along with
  ~82 track segments and 10 vias. Recover it from that commit before placing
  anything against a boundary that is not there.
- **The stored net names are one rework behind the schematic.** The PCB still
  says `CAP_ACTIVE` and `ARM_CLK`; the schematic says `VBUCK_OK`, `WDT_PET`,
  `FC_ARM`, `OC_ARM_EN` and five more from the TPS61094 work. Nine schematic
  nets have no PCB counterpart and three PCB nets no longer exist. Run *Update
  PCB from Schematic* before trusting anything read out of the board file — a
  pin-map audit taken from it will be wrong in exactly those places.

Not reviewed, not fabbed, no tag. Firmware exists —
[`tinkerrocket-idf/projects/rocket_computer_mini`](../../tinkerrocket-idf/projects/rocket_computer_mini/)
carries the **single-MCU** merge, with a board map netlist-verified against the
tree as it was before this change (note its README's warning that
`pin-budget.md`'s assignment table has drifted from the schematic). That merge
is now the wrong shape for this board: the split needs `projects/out_computer`
and `projects/flight_computer` board headers instead, which is the reason the
GPIO numbers above were chosen to match `rocket-computer` wherever an S3 pad
allowed it.

Before it goes to fab it needs its own pre-manufacturing review — see *Sending a
board to fab* in [`../README.md`](../README.md).

### Open items

Each of these is a decision left open rather than an oversight:

- ~~**The radio and the NAND are behind the flight computer's rail.**~~
  **Decided 2026-08-22: this is intended.** `rocket-computer` puts the LoRa
  radio on its own switch off `VBATT` (`U29`, enabled by `LoRa_ACT`) so the out
  computer can hold the pad awake and transmitting while the flight computer
  sleeps. This board deliberately does not: one switch carries the flight
  computer, the sensors, the GNSS receiver, the radio and the NAND together, and
  a dark radio while the flight computer is off is acceptable here. Do not add a
  second `TPS22810` to "fix" it.
- **The 3V3 budget has a second MCU on it that was never in the original
  reasoning.** `power-budget.md` now lists the flight computer, but the
  worst-credible total is a sum that predates it. The buck is a 1 A class part
  and the added load is real.
- **No 1 M bleeder on `FC_D+`.** `rocket-computer` carries `R77` for this on
  `CEN_D+`; it was not copied, on the grounds that the out computer has never
  had one either. Symmetry, not a defect — but it is an asymmetry with the
  reference design.
- **The out computer keeps twelve spare pads and the flight computer six.**
  (ERC reports seven on the flight computer; the seventh is `GPIO26`, which is
  not usable on this part.) They are left unconnected rather than
  no-connect-flagged, so ERC reports each one. Flagging them would assert they
  are permanently unused, which is not the intent.
- **Both processors' spare `GPIO26`** is the quad-PSRAM chip select on this part
  and remains unusable on each — see `pin-budget.md`.
- **The six link lines are the only signals live during pad standby.** Firmware
  must park them Hi-Z while `V_MCU_SWTCH` is down; the hardware does not enforce
  it. See *The link between them*.
