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

| | at fork | S3 alone | second S3 (2026-08-22) | V1 as fabbed / main (2026-09-12) |
|---|---|---|---|---|
| Parts | 255 | 156 | 196 | **229** |
| Nets | 234 | 139 | 200 | **238** |
| Sheets | 5 | 4 | 5 | **5** |

*Parts and nets are netlist component and net counts; the 229 includes the four
fiducials and four mounting holes, so 221 parts are fitted. The "S3 alone" column
supersedes an earlier 190/191 in this table, which was measured before the
expansion header, camera, servo and piezo came out; the last column is the board
after the arm and hold-up reworks, the 2026-09-03 pin swap and the V1 fab gate.*

### Who owns what

| | Out Computer `U15` | Flight Computer `U32` |
|---|---|---|
| Sheet | [`esp32s3_mcu.kicad_sch`](esp32s3_mcu.kicad_sch) | [`fc_esp32s3.kicad_sch`](fc_esp32s3.kicad_sch) |
| Rail | `+3V3` — always on | `V_MCU_SWTCH` — **starts off** |
| Radio | 2.4 GHz chip antenna `U14`, 900 MHz LoRa `U16` | none |
| Memory | NAND `U11` + boot flash `U13` | boot flash `U33` only |
| Sensors | pack monitor only, on `SEN_SC*` (`+3V3`) | IMU, baro, GNSS **and magnetometer** |
| Pyro | arm consent (`OC_ARM_EN`) | all four channels and the arm request (`FC_ARM`) |
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
10 µF give roughly **0.9 s**, not the 1.4 s this section used to state. With
`tau` = 100 k x 10 µF = 1.0 s, a node starting at 3.3 V less a `BAV170M` drop
(~0.4 V at the 28 µA `R84` pulls) and `U30`'s `V_ENF` of 1.13 V, the window is
`1.0 s x ln(2.9 / 1.13)` = 0.94 s. The old figure reproduces only as
`ln(3.3 / 0.8)` — dropping the diode drop *and* using a 0.8 V threshold. `C105`
is an 0402 6.3 V X5R biased at ~2.9 V, where a ~50% DC-bias loss is typical, so
the window in practice is nearer **0.45 s**.

Note the decay is on the *enable* node, not on `V_MCU_SWTCH` — `U30` is a load
switch, so its output stays at full rail until the enable crosses the threshold
and then collapses. The flight computer is fully powered for the whole window,
with no brownout race on the way down.

**The window is not load-bearing in flight.** Once `pwrHoldAssert()` has run,
`GPIO17` drives the enable through `D9` itself and nothing is decaying; an out-
computer reset is covered with zero gap, which is the whole point of #825. The
window only applies on the ground, before launch detect, where the out computer
alone drives the enable — and there a dropped rail is the *designed* behaviour,
because a flight computer that cannot be powered off on the pad is an operator
trap. #1270 weighed growing `C105` and closed without a part change for exactly
this reason.

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
the `R137` 100 k / `R138` 360 k divider off `V_BUCK` (`R138` was 1 M until #1000
lowered it to put the node inside ADC range; GPIO3 is `ADC1_CH2` and `C152`
100 nF sits across the tap). That keeps the property the
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

### Arming needs both processors

The pyro return switch `U9` is closed from pack voltage by `Q13`, through
`R139` and `R21` onto the `ARM_GATE` node, with `R22` bleeding the gate in about
0.2 ms whenever the drive stops. `Q13`'s base is pulled down only through two
digital transistors in series: `Q12`, driven by the flight computer's `FC_ARM`
(GPIO44) through `R132`, and `Q14`, driven by the out computer's `OC_ARM_EN`
(GPIO11). **Both pins have to be driven high to arm.** A processor that is off,
in reset, or leaving its pin high-impedance holds its transistor off through the
transistor's built-in base-emitter resistor, so neither MCU can arm the board
alone and a dead MCU disarms it.

The out computer's term is the fail-safe, and it is there for a reason: GPIO44
is the flight computer's U0RXD and carries a weak pull-up at and after reset, so
the flight-computer term on its own is *not* a safe idle. The firmware contract
that goes with the circuit: the out computer drives `OC_ARM_EN` low at boot,
raises it only on an explicit arm, and drops it when flight-computer heartbeats
stop. The flight computer keeps its own task watchdog for everything else.

**2026-09-02: the window watchdog is gone.** The 2026-08-28 rework had a
supervisor on the flight computer's `CHIP_PU` that reset it whenever GPIO8
stopped toggling. It was removed from the schematic and the board (`U46`,
`R133`, `C139` and their copper) as complexity the consent stage makes
unnecessary — liveness is a firmware check on the out computer now, and a
hardware reset line on `CHIP_PU` also blocked flashing the flight computer over
USB. `FC_CHIP_PU` is back to its reset RC only, and GPIO8 is a spare pad. The
same change replaced the pack-node veto diode `D16` with `Q14`, which is what
took the pack voltage off the out computer's GPIO11. `Q14` is placed beside `Q12`/`Q13` on the
front face and routed; the V1 fab carries the consent stage.

### The link between them

Six wires, the same net names and the same protocols as `rocket-computer`:

| Net | OC pin | FC pin | Role |
|---|---|---|---|
| `ESP_I2S_BCLK` | GPIO2 | GPIO21 | I2S bit clock |
| `ESP_I2S_WS` | GPIO1 | GPIO18 | I2S word select |
| `ESP_I2S_SD` | GPIO3 | GPIO13 | I2S data, FC → OC |
| `ESP_I2S_FSYNC` | GPIO4 | GPIO14 | frame sync — a plain GPIO, not an I2S signal |
| `ESP_SCL` | GPIO6 | GPIO37 | I2C clock |
| `ESP_SDA` | GPIO5 | GPIO36 | I2C data |

**The flight computer's I2C pins moved on 2026-09-03.** This table said FC
`GPIO33` / `GPIO35` until 2026-09-04; the pin swap that freed the sensor SPI
bus put `ESP_SCL` on FC `GPIO37` (pad 42) and `ESP_SDA` on FC `GPIO36`
(pad 41). **`GPIO33` and `GPIO35` are `PYRO4_FIRE` and `PYRO2_FIRE` now**, so a
header written from the old table would have run the inter-processor I2C
clock straight into two pyro gates. The out-computer column never changed.

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

Both are gone now: `U3` moved to `MAG_SCL`/`MAG_SDA` on the flight computer —
GPIO2/GPIO1 since the 2026-09-03 pin swap; they sat on GPIO37/GPIO36 until then,
which is where the `ESP_SCL`/`ESP_SDA` link pair lives now — with `R117`/`R118`
(5.11 k) pulled up to `V_MCU_SWTCH`, the same rail as the part and its master. `SEN_SCL`/`SEN_SDA` is a pure pack-monitor
bus on `+3V3`, the same shape as `rocket-computer`'s `PWR_SCL`/`PWR_SDA`. No
passive pull-up crosses the rail boundary any more; every remaining crossing is
an actively driven signal.

The part exposes no DRDY on this land — every pad but SCL/SDA/VDD/GND and the `C1`
reservoir pin is NC — so the magnetometer is poll-only. That is the part, not an omission.

**Firmware constraint.** All six of these cross the `+3V3` / `V_MCU_SWTCH`
boundary — and they are not the only out-computer signals that do. The radio
control lines (`L_CS`, `L_RST`, `L_RXEN`, `L_BUSY`, `L_DI01`) and the memory bus
the NAND shares with the radio (`M_SCK`, `M_MOSI`, `M_MISO`, `M_FLASH_CS`) land
on parts powered from `V_MCU_SWTCH` too: **fifteen crossings** in all (netlist,
2026-09-12; this paragraph said six until then). The out computer must park
every one of them Hi-Z whenever `V_MCU_SWTCH` is down, or it injects through
the dead parts' ESD diodes into the rail. This exact failure already bit
`rocket-computer` — the FC-relay OTA contention, where `i2s_del_channel()` left
BCLK/WS/DOUT driven.

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
| all four `PYRO*_FIRE` / `PYRO*_CONT` | flight computer, same GPIO as the single-MCU map |
| `PYRO_ARM` | flight computer as `FC_ARM` (GPIO44), and it no longer arms alone — see *Arming needs both processors* |
| the six `ESP_*` link lines | live again, both ends populated |
| `P4_EN_HOLD` | `FC_EN_HOLD`, into `D9` |
| all twelve `EXP_01`–`EXP_12` | removed |
| camera UART, `PIEZZO`, `SERVO_ACT` | removed |

Because the moved signals kept their GPIO numbers, the sensor, GNSS and pyro
halves of `pin-budget.md`'s assignment carry over unchanged — they simply belong
to `U32` now rather than `U15`. The out computer keeps the radio, the memory
bus, the power-monitor I2C and USB, and gains twelve spare pads.

## Design docs

- [`power-budget.md`](power-budget.md) — the 3V3 rail: the inherited buck, the
  hold-up converter that now sits between it and the rail, the loads on both
  sides of the switch, and what charging the supercap adds. Concludes the
  inherited buck stays, and explains why the ground station's buck-boost must
  not be copied here.
- [`arm-watchdog-rework.md`](arm-watchdog-rework.md) — the 2026-08-28
  supervised-arm design **as proposed**. The window watchdog it specifies was
  removed on 2026-09-02 and the veto diode became `Q14`; read it for the
  reasoning that retired the charge pump, not for the current circuit, which is
  described under *Arming needs both processors* above.
- [`pin-budget.md`](pin-budget.md) — whether the single processor can carry the
  board alone. 23 signals into 27 usable pads, with a proposed assignment that
  keeps the serial console and spends JTAG. **Written against the single-MCU
  board**; the pin *assignments* still hold, but they are now split across two
  processors — see *The split* at the end of that document.

Both budgets were written for the single-processor board and carry dated
sections that bring them to the two-processor one: `pin-budget.md` in *The
split* (re-verified against the netlist 2026-09-12), `power-budget.md` in its
2026-09-12 rewrite of the rail, the loads and the scenarios.

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
  and `schematic-review.md`. Carrying them would have asserted this board had
  been reviewed before it was. It has its own review now — tracker
  [#993](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/993)
  (2026-09-01, findings #994–#1032) and the 2026-09-04 fab gate — and the
  parent's reviews are still the parent's.
- **`WORKLIST.md`** — the closing record of the V9 pre-fab review, written
  against V9's live files. Its board state, its closed items, and its bench list
  describe a board this one will not be.
- **`FABRICATION-NOTES.md`** — **rewritten for this board (2026-09-03).** It now
  carries the mini's own geometry (22.55 × 69.62 mm, 8 layer, 1.630 mm on
  JLCPCB `JLC08161H-2116`) and its own parts. It began as a copy keyed to V9's
  22.35 × 75.00 mm, 6 layer, 1.546 mm stack; the on-board `User.Drawings` text
  was still the V9 block verbatim until the same date. Stale fab notes are the
  kind of error that reaches a fab house — re-read this file against the board
  file after any stackup or part change.
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

Re-measured on main `80e1860d` (2026-09-12) with `kicad-cli` at
`--severity-all`, DRC with `--schematic-parity`. The board file on main is the
fabbed board: copper and placement identical to the
`rocket-computer-mini-v1.0.1` tag; only 3D-model references and `U5` pad 12's
paste apertures (window-paned 2026-09-12, #960) touched since.

| | at fork | after the rev change | after P4 removal | after the second S3 | **V1.0.1 / main** |
|---|---|---|---|---|---|
| ERC (`--severity-all`) | 1012 | 1012 | 823 → 610 | 739 | **737** (10 errors) |
| DRC (`--severity-all`) | 26 | 28 | 88 | not re-run | **23** (2 errors) |
| Schematic parity | 11 | 11 | 8 | not re-run | **0** |
| Unconnected | 0 | 0 | 0 | not re-run | **0** |

*The columns before the last are the record of the fork and of the
schematic-only period, kept as history; every board-side number in them
predates the layout. The per-category breakdown of the 2026-08-22 ERC delta
that used to follow this table is in this file's git history.* What the live
numbers are:

- **ERC 737.** The 10 errors are the out computer's ten bare spare pads (see
  *Status* below). The warnings are 704 `endpoint_off_grid` — the power sheet's
  half-grid offset — 22 `pin_to_pin` and 1 `lib_symbol_mismatch`, down from 56
  and 25 before the 2026-09-12 hygiene pass
  ([#1030](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/1030)): the
  pyro P-FETs and the NAND now carry symbols named for the fitted parts with
  typed pins, eighteen cached copies that differed from the shared library only
  by a footprint nickname or a datasheet line were refreshed, and every sheet
  carries a title block — all with the netlist unchanged. The one mismatch left
  is `U30`: its cached copy is a differently drawn variant of the load-switch
  symbol (`TPS22810DRVR_1`, other pin positions), so replacing it means
  re-wiring the symbol in the editor.
- **DRC 23.** The 2 errors are courtyard overlaps between the fiducials `FID2` /
  `FID3` and the connectors `J6` / `J2` they sit beside — accepted; a fiducial
  inside a connector's courtyard costs nothing. The 21 warnings are silk over
  copper or too near the edge (`C130`'s outline over four pads, `J8` and `J2` at
  the board edge), three silk overlaps, six `lib_footprint_mismatch` against the
  shared library (`U5`'s is pad 12's window-paned paste, a board-copy-only
  change from 2026-09-12) and one `lib_footprint_issues` on the logo. No `track_dangling`
  or `via_dangling` remain: the P4-era stubs this section used to list were
  cleared in the layout pass, and `U32`, `U33`, `U1`, `S1` and `D9` — once
  schematic-only — are placed and routed.

Three stale items were listed here; **all three are now resolved** (2026-09-03):

- **`C12`'s footprint disagreement is gone** — with it the whole part. The
  2200 µF pyro energy store no longer exists anywhere in the design; the
  refdes `C12` has been reused for the BLE antenna's DNP series-match position.
- **`bom.csv` is current** — 70 rows, 221 designators, reconciling against the
  netlist exactly (the eight unmatched refs are `FID1-4`/`H1-4`, excluded by
  design). It is no longer V9's 255-part bill.
- **The on-board fabrication note has been rewritten** against this board. It
  names the board `TINKERROCKET ROCKET COMPUTER MINI` at its own
  22.55 × 69.62 mm, and justifies ENIG by the parts that are actually fitted —
  the 0.5 mm-pitch WLCSPs (`U13`, `U33`), the 0.4 mm QFN-56s (`U15`, `U32`),
  the 0.4 mm X2QFN (`U21`) and the 0.4 mm 12-lead (`U1`). The dead `QFN-104
  (U17)` citation is gone.

### The revision

**This board is `V1`.** The title block carries `(rev "V1")` and the back
silkscreen renders `TR-Mini\n${REVISION}` from it, so the revision reaches the
board from one place and cannot drift from the documentation.

`V1` is this board's own first revision. It is deliberately two characters, to
match the `V10` / `V6` / `V4` convention on the other boards in this repo — a
`rev1`-style string was tried during the fork and was wider, which grew the
silkscreen block until it collided with `R1`. That is history now: the text sits
on `B.Silkscreen` at (81.19, 119.37) and `R1` is 21 mm away at (76.81, 140.71),
so neither the `silk_overlap` nor the `silk_over_copper` warning that the longer
string caused still exists.

The title block arrived from the fork reading `(rev "V9")` — the revision of a
*different* design's fab release. `V1` has since gone out under its own tags:
`rocket-computer-mini-v1.0.0` (2026-09-05, merged to main by PR #1172) and
`rocket-computer-mini-v1.0.1` the same day, after JLCPCB's engineering query
moved the vias out of the WLCSP ball pads. The GitHub releases carry the gerber
and assembly zips, plotted from the tagged commit by `tools/plot_gerbers.sh`,
which refuses a dirty tree and checks the title-block revision against the tag.
`gerbers/` is gitignored except its `.gitkeep`: whatever it holds locally is a
working plot, never the release — the stale 2026-08-31 archive that used to sit
there is gone.

## Status

**Fabricated.** `V1` went out as `rocket-computer-mini-v1.0.1` on 2026-09-05,
after its own review
([#993](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/993),
2026-09-01) and the 2026-09-04 fab gate recorded in
[`FABRICATION-NOTES.md`](FABRICATION-NOTES.md). The board file on main is that
board: copper and placement identical to the tag; only 3D-model references and
`U5` pad 12's paste apertures (#960) touched since (checked 2026-09-12).

Board file, verified 2026-09-12 on main `80e1860d`:

- **22.55 × 69.62 mm, 8 layers, 1.630 mm stack**; the stackup and the fab
  parameters are in `FABRICATION-NOTES.md`.
- **230 footprints** — the 229 netlist components (221 fitted; `FID1–4` and
  `H1–4` are not) plus the logo — **2,296 track segments, 598 vias, 26 zones**;
  0 unconnected, 0 schematic-parity items. The netlist exports **229
  components, 238 nets**, no duplicate references and no single-node nets. (The
  214/205 this paragraph gave on 2026-08-30 was the pre-layout schematic.)
- **DRC 23 at `--severity-all`**: 2 courtyard-overlap errors (`FID2` inside
  `J6`'s courtyard, `FID3` inside `J2`'s — accepted) and 21 warnings, itemised
  under *Where the numbers stand now*.
- **The net names match the schematic** (`VBUCK_OK`, `FC_ARM`, `OC_ARM_EN` and
  the TPS61094 nets are all on the board). Until the 2026-08-30 re-sync the
  board file lagged the schematic by two reworks — 9 schematic nets had no PCB
  counterpart and 3 PCB nets no longer existed — and `kicad-cli pcb drc
  --schematic-parity` did not flag it, because it reconciles footprints rather
  than stale net strings. Take pin maps from a `kicad-cli sch export netlist`,
  never from the board file; an audit run against the stale copy produced three
  wrong findings before the netlist corrected them.

**ERC (2026-09-11, `--severity-all`): 10 errors, and every one of them is a
spare pad on the out computer** (GPIO9/12/34/39–45), left unflagged on purpose
— see *Open items* below. The other 23 of the floor this sheet used to carry
were noise and are gone, with the netlist byte-identical before and after: the
QMC5883P and BMP581 grounds are power inputs (they were typed as power outputs,
which is what made every ground pin argue with every other), `TPS259631DDAR`
and `AONR21321` are in the shared library instead of living only in this
sheet's cache, seven PWR_FLAGs mark the nets fed through an inductor, a resistor
or a connector (`GND`, `VBAT_CON`, `V_BUCK`, VBUS, `U1` VCC, both VDD3P3 nets),
and the pins both reviews cleared as unused carry no-connect flags (the
octal-SPI pads on both processors, the GNSS module's reset, external-antenna
and antenna-detect pads, the IMU's second interrupt, the radio's DIO3, the
INA230's ALERT, the USB-C sideband pair). The pin-type conventions are the
ones recorded in
[`../rocket-computer/v10-power-parity-2026-09-11.md`](../rocket-computer/v10-power-parity-2026-09-11.md)
§6.6; this board shares those symbols. Left as drawn: 704 off-grid endpoint
warnings — most of the power sheet sits on a half-grid offset. The #1030 items
went on 2026-09-12 with the netlist unchanged: the borrowed P-FET and NAND
symbols replaced, the stale cached copies refreshed, a title block on every
sheet. `ARM_GATE` is a sheet-local label by design, so its net is
`/Pyro/ARM_GATE` — the name the docs use is the label, the path is KiCad's.

**Firmware is the two-processor pair, and CI builds it.** The fabbed board runs
[`projects/out_computer`](../../tinkerrocket-idf/projects/out_computer/) and
[`projects/flight_computer`](../../tinkerrocket-idf/projects/flight_computer/)
with `-DTR_BOARD_M1=1` — a `board_m1.h` in each, netlist-verified on 2026-09-04
(73 constants, none pointing at an unconnected pad) — and `firmware-build.yml`
builds both as board `M1`.
[`projects/rocket_computer_mini`](../../tinkerrocket-idf/projects/rocket_computer_mini/),
the single-MCU merge, is still built so it does not rot, but it targets a shape
that was never fabbed (#1188): do not flash it to this board.

The next spin's checklist is the *Still open* list in `FABRICATION-NOTES.md`;
the bench items the review collected are on #993 and
[#1211](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/1211).

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
- **The out computer keeps twelve spare pads and the flight computer two.**
  The flight computer's are GPIO47/GPIO48; the 2026-09-03 pin swap spent the
  rest (`pin-budget.md`, *The split*). Ten of the out computer's (GPIO9, 12, 34,
  39–45) are left bare rather than no-connect-flagged, so ERC reports each one —
  flagging them would assert they are permanently unused, which is not the
  intent. GPIO47/GPIO48 on both processors carry no-connect flags since the
  2026-09-11 ERC pass, as octal-SPI pads that will never be general-purpose here.
- **Both processors' spare `GPIO26`** is the quad-PSRAM chip select on this part
  and remains unusable on each — see `pin-budget.md`.
- **Fifteen out-computer signals cross into the switched rail** — the six link
  lines, the five radio control lines and the four memory-bus lines. Firmware
  must park them Hi-Z while `V_MCU_SWTCH` is down; the hardware does not enforce
  it. See *The link between them*.
