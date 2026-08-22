# rocket-computer-mini — MCU pin budget

> **Superseded in premise, not in content.** This document asks whether *one*
> ESP32-S3 can carry the board. The board now has two — an out computer and a
> flight computer — so the question no longer binds. Every pin *classification*
> below still applies to both processors, and every assignment it argues for is
> still the assignment in the schematic; the signals simply live on two parts
> now. See **The split**, at the end, for which pad ended up on which processor.

**Purpose.** Establish whether the single MCU can carry the reduced board by
itself, now that the telemetry radio is on-board rather than on a daughterboard.

**Answer: yes, with two pads to spare.** It did not fit as first drawn — 33 pins
against 27 usable pads — but sharing the memory bus with the radio and dropping
signals firmware never used brings the demand to **25**, all of it now wired.
Two pads remain free, and the serial console survives.

**Status.** Counted from a netlist export of the working tree, including the
on-board radio. Pin *classification* (which pads are safe to use) is from part
family behaviour and should be checked against the datasheet before anything is
committed to layout.

---

## Supply — what the MCU actually has

57 pads: 30 already committed, 27 unconnected. Of the 30, fourteen are supply,
crystal, chip-enable and the exposed pad; the remaining sixteen are signals
already spoken for — boot button, USB pair, the power-monitor I²C, two
strapping pulls, the six-wire boot flash bus, and a second four-wire flash bus.

The 27 unconnected pads are **not** 27 usable GPIO:

| Class | Count | Pads | Cost of using them |
|---|---|---|---|
| Unconditionally free | **16** | GPIO1, 2, 4–14, 17, 18, 34 | none |
| Strapping | 2 | GPIO3, GPIO45 | boot-time level constrained; GPIO45 sets flash rail voltage |
| Console UART | 2 | GPIO43, GPIO44 | lose the serial console |
| JTAG | 4 | GPIO39–42 | lose hardware debug |
| Flash/PSRAM adjacent | 3 | GPIO26, 47, 48 | free **only** if the package has no in-package PSRAM |
| **Total** | **27** | | |

**Resolved against the datasheet (v2.2, Table 1-1).** The part is
**ESP32-S3RH2: 2 MB Quad SPI PSRAM, no in-package flash, VDD_SPI 3.3 V**, and it
is the replacement for the end-of-life ESP32-S3R2. That settles the group
individually rather than collectively:

- **GPIO26 (SPICS1) is the in-package PSRAM chip select and is NOT available.**
  The earlier "no PSRAM" reasoning only ruled out *octal*; this part has *quad*,
  which uses exactly this pin. It is now unused.
- **GPIO47 and GPIO48 (SPICLK_N/P) are available.** They serve octal PSRAM only.
  Datasheet note 4 confirms they are GPIO47/GPIO48 and that their rail follows
  VDD_SPI — 3.3 V on this part, so they behave like ordinary GPIO here.
- **GPIO33–37 are available** for the same reason: SPIIO4–7 and SPIDQS are octal
  functions.
- **Pins 30–35 are shared deliberately.** The in-package PSRAM and the external
  boot flash sit on one bus: SPICS0 selects the flash, SPICS1 the PSRAM. Using
  those six for the external flash is the intended arrangement, not a conflict.

**The realistic ceiling is 16.** A flight computer that cannot be debugged over
JTAG or watched over a serial console during bring-up is a poor trade for four
pins. Treat the 27 as a theoretical maximum, not a plan.

## Demand — what the board asks for

| Group | Signals | Pins |
|---|---|---|
| Inertial + baro | shared SPI (3), two chip selects, two interrupts | 7 |
| Magnetometer | **shares the power-monitor I²C bus** — no new pins | 0 |
| GNSS | **UART pair only** — no second RX, no branch enable | 2 |
| Telemetry radio | chip select, BUSY, RST, DIO1 — **bus shared with memory** | 4 |
| Pyro | four fire, four continuity, arm | 9 |
| Housekeeping | rail power-good, power switch sense | 2 |
| Peripheral power | single enable for receiver and radio | 1 |
| **Core total** | | **25** |
| Legacy radio connector | UART pair + branch enable, now redundant | 3 |
| **Still drawn, pending removal** | | **28** |

**The GNSS branch enable is gone with that decision.** Two pins buys the data
interface and nothing else, which means the receiver is permanently powered
whenever the rail is up and cannot be power-cycled to clear a wedged module.
That is a defensible trade — a receiver that needs a power-cycle mid-flight is
already having a bad day — but it is a capability decision riding along with a
pin count, and it should be a deliberate one. If only the *data* lines were
meant, the enable comes back and the core total returns to 28.

### The radio costs seven, and the firmware already knows how

The radio is SPI everywhere in this project — the daughterboard and the ground
station carry the same module, and neither uses a UART version of it. The
daughterboard's UART is its *host* interface, produced by that board's own
processor converting from the radio's SPI. **The bare module on this board is
the intent**, so the daughterboard is not an alternative to weigh; it is the
thing being designed out.

The cost is **seven pins, not nine.** Two of the nine drawn signals do not need
a processor pin:

- **DIO2 / transmit-enable** is a single net joining two pins of the module to
  each other. The radio drives its own transmit-enable from DIO2 once configured
  to do so — no processor involvement, and correctly drawn today.
- **Receive-enable** is connected to nothing but the module. It is floating, and
  no firmware pin exists for it.

Seven is not an estimate. The out-computer firmware still carries the direct-SPI
radio driver from the V7 topology, and it defines exactly seven pins: the four
SPI wires, BUSY, reset, and DIO1. The V8 map sets all seven to −1 with the note
that the radio moved to the UART daughterboard. **Putting the bare module back
on-board is a return to the V7 topology, and the driver for it was never
deleted** — this is a board change, not a firmware rewrite.

**Receive-enable is the one open item.** It is floating in the schematic and
absent from firmware. Either it is tied to a fixed level, or the module expects
it driven and something must drive it. Resolve it deliberately — a floating
receive-enable is the kind of thing that works on the bench and fails in the
field.

## Closing the gap

In rough order of cost to the design:

Three savings are already reflected in the 27 above and are decided: the
magnetometer moving onto the power-monitor I²C bus, the radio's two non-pin
signals, and the GNSS reducing to its UART pair. What remains:

### Sharing an SPI bus beats adding an expander

The board runs three general-purpose SPI buses where it needs two. Putting the
radio on the memory bus frees three pins for **no new parts** — SPI is a bus, and
the two devices keep their own chip selects.

That is what was done. It also makes room for continuity to go straight onto
processor pins, so the I²C expander this document previously recommended is not
needed at all.

**Which pair to combine is a latency question, not a bandwidth one.** There is
enormous bandwidth margin either way; what matters is head-of-line blocking:

| Device | Traffic shape | Bus demand |
|---|---|---|
| Memory | page-sized bursts at 40 MHz — a single page blocks the bus for hundreds of µs | long transactions |
| Sensors | read per-sample on data-ready, not batched through a FIFO | high rate, latency-critical |
| Radio | one small packet twice a second | negligible |

So **memory + radio share, sensors stay alone.** Pairing the sensors with memory
would put the most latency-sensitive device behind the longest transactions, and
that delay lands directly as jitter on the sample timebase — which matters,
because those samples get integrated. The radio is three orders of magnitude
below either and does not care if a packet waits half a millisecond.

This does mean sensors and memory no longer share a host, which is how the V7
firmware had them. That costs nothing: this board has no firmware map yet, so
the host assignment is being written from scratch regardless.

Continuity is a digital read — firmware reads it as a GPIO level, and nothing on
this board needs an ADC channel, so the ADC2-versus-radio restriction does not
constrain the design at all.

| Configuration | Pins | Spare |
|---|---|---|
| As drawn, legacy connector removed | 27 | 0 |
| **Radio on the memory bus, continuity direct** | **25** | **2** |

**Caution on the pyro lines.** All eight pyro signals ended up on direct
processor pins, which is the right outcome. Had pin pressure forced an expander,
it should have carried *sensing* only — putting a shift register or expander
between the processor and a pyro gate adds a failure mode that can assert an
output without the processor commanding it, on the one path where latency and
determinism matter most.

**Why continuity avoids GPIO45.** The fourth channel sits on GPIO44, spending the
console's receive line, rather than on the spare GPIO45. GPIO45 is sampled at
reset to set the flash rail voltage, and a continuity input's level at boot
depends on whether an igniter happens to be connected — which is not something
the design controls. An armed channel could hold that pad high and stop the board
booting. GPIO43 is kept free so the boot ROM can still print, which preserves
bring-up diagnostics even without console input.

## Special-function pins — verdict per group

The processor's pins carry secondary functions in three groups, and their safety
is not uniform. The distinction that matters is **octal** versus **quad**
flash/PSRAM, because the design's proof only covers one of them.

| Group | Pins | Secondary function | Verdict |
|---|---|---|---|
| Quad flash bus | GPIO27–32 | flash clock, data, hold, write-protect, CS0 | **In use as intended** — they carry the external boot flash. Not repurposed, no risk. |
| Octal PSRAM data | GPIO33–37 | SPIIO4–7, SPIDQS | **Proven safe.** The V7 board used GPIO34–38 for its memory bus on hardware that was built and flown — a part with octal PSRAM could not have done that. |
| Octal PSRAM clock | GPIO47, GPIO48 | SPICLK_N / SPICLK_P | **Safe by the same proof** — these serve octal PSRAM only, and this part has none. Not directly exercised on V7, so slightly weaker evidence. |
| **Quad PSRAM select** | **GPIO26** | **SPICS1** | **Confirmed unavailable** — datasheet Table 1-1 gives this part 2 MB quad PSRAM, whose chip select is this pin. **Left unused.** |
| JTAG | GPIO39–42 | debug | Safe as GPIO; cost is losing hardware debug. |
| Console | GPIO43, GPIO44 | UART0 | Safe as GPIO; cost is the serial console. **Free on the flight computer; on the out computer GPIO44 carries `L_RXEN`, so that processor has console TX but no RX.** The "both left free" this row used to claim was never true of the shipped schematic. |
| Strapping | GPIO0, 3, 45, 46 | boot mode, JTAG select, flash rail voltage, ROM log | **GPIO3: see the correction below.** **GPIO45 left free** on both processors: it sets the flash rail voltage, and anything whose boot level the design does not control can stop the board booting. |

### Correction — GPIO3 no longer has the pulldown this document claimed

This document used to justify GPIO3 with one sentence: *"GPIO3 is safe here — it
drives a switch enable and its pulldown holds it low at reset."* Adding the
second processor invalidated it on both parts, and the reasoning is worth
recording because the failure was silent:

- On the **flight computer**, GPIO3 was still the switch enable — but `D9` now
  stands between the pin and `R84`, and a diode does not pass a pulldown. The
  very change that created the enable OR is what removed the pull.
- On the **out computer**, GPIO3 stopped being the enable altogether (that moved
  to GPIO7) and became `ESP_SDO`, an *input* driven by the flight computer —
  which is unpowered while the out computer boots, so the net floated.

Both are now closed. `FC_EN_HOLD` moved to **GPIO17**, which is unconditionally
free, so the flight computer's strapping pin is a bare pad; and `R34` (100 k)
pulls `ESP_SDO` down so the out computer's GPIO3 has a defined level at reset.

The residual risk was low either way — GPIO3 selects the JTAG signal source and
is only sampled when `EFUSE_JTAG_SEL_ENABLE` is burned, so an unburned part
ignores it, and `rocket-computer` flies `GPIO3 = ESP_SDO` with no pull at all.
It is fixed because a floating input on a strapping pad is not something to
carry into a layout, not because it was going to stop a boot.

**Two signals were moved to reduce risk**, using pads freed by dropping the
power-good and peripheral-enable signals:

| Signal | From | To | Why |
|---|---|---|---|
| `BMP585_INT` | GPIO26 (SPICS1) | GPIO41 (JTAG) | removes the only quad-PSRAM exposure |
| `PYRO4_CONT` | GPIO44 (U0RXD) | GPIO42 (JTAG) | returns the full serial console |

After the move, **four pads are free — GPIO26, GPIO43, GPIO44, GPIO45** — and the
two genuinely hazardous ones are among them. The cost is JTAG, which is now fully
spent. That is the right trade: a console works from first power-on with no
adapter, and the two risky pads are removed from the design rather than managed.

## Assignment — as built

**This is wired in the schematic.** All 25 signals below are connected to the
processor by global label at the pin, and the netlist confirms every one reaches
it. Two pads remain free: **GPIO43** and **GPIO45**.

The radio's clock, data-in and data-out are absent from the table because they
are no longer dedicated — the radio shares the memory bus and keeps only its own
chip select. Those three freed pins went to continuity sensing.

Wired here, but **not yet wired at the far end**: `PERIPH_EN` reaches the
processor and nothing else. The peripheral switch it should drive does not exist
in a usable form yet — see *The switched rail problem* below.

The allocation is driven by three rules, in order:

1. **Pyro firing gets the safest pins available** — unrestricted GPIO only,
   never a strapping pin, never a pin the boot ROM drives. The console transmit
   pin is active during every boot; a pyro gate must never share that behaviour.
2. **High-rate buses stay on unrestricted pins** — the two SPI buses, so that no
   boot-time strapping or debug function can interfere with them.
3. **Passive inputs absorb the compromised pins** — interrupts, BUSY,
   power-good and switch sense tolerate the debug and flash-adjacent pads,
   because nothing bad happens if they float briefly during reset.

| Signal | Pin | Tier | Why here |
|---|---|---|---|
| PERIPH_EN | GPIO3 | strapping | enable idles low, which is what this pad wants at reset |
| PYRO1_FIRE | GPIO4 | free | safety-critical output |
| PYRO2_FIRE | GPIO5 | free | safety-critical output |
| PYRO3_FIRE | GPIO6 | free | safety-critical output |
| PYRO4_FIRE | GPIO7 | free | safety-critical output |
| PYRO_ARM | GPIO8 | free | safety-critical output |
| PYRO1_CONT | GPIO10 | free | continuity sense |
| PYRO2_CONT | GPIO11 | free | continuity sense |
| PYRO3_CONT | GPIO12 | free | continuity sense |
| L_CS | GPIO13 | free | radio chip select (bus shared with memory) |
| L_BUSY | GPIO14 | free | radio input |
| L_DIO1 | GPIO17 | free | radio interrupt |
| L_RST | GPIO18 | free | radio output |
| SENS_SCLK | GPIO34 | free | sensor bus |
| SENS_SDI | GPIO1 | free | sensor bus |
| SENS_SDO | GPIO2 | free | sensor bus |
| ISM6HG256_CS | GPIO9 | free | sensor select |
| BMP585_CS | GPIO47 | flash-adjacent | sensor select |
| ISM6HG256_INT1 | GPIO48 | flash-adjacent | passive input |
| BMP585_INT | GPIO26 | flash-adjacent | passive input |
| GNSS_TX | GPIO39 | JTAG | UART, tolerant |
| GNSS_RX | GPIO40 | JTAG | UART, tolerant |
| PG_RAIL | GPIO41 | JTAG | passive input |
| POWER_SWITCH | GPIO42 | JTAG | passive input |
| PYRO4_CONT | GPIO44 | console RX | continuity sense — see the note below |

### The switched rail problem

`PERIPH_EN` is meant to power the receiver and the radio from one pin. **No
switch on this board can currently do that.** Both existing load switches take
their input from the battery rail, not 3.3 V — they were switching power to
daughterboards that carried their own regulators:

| Switch | Input | Output goes to |
|---|---|---|
| `U27` | VBATT (6.4–8.4 V) | `FL1` → `J1`, the external GNSS connector |
| `U29` | VBATT (6.4–8.4 V) | `FL2` → `J5`, the external radio connector |

The bare modules are 3.3 V parts — the receiver is rated 2.55–3.6 V. **Connecting
either module to either switch output as currently wired would destroy it.**

The decided fix, not yet implemented: repoint one switch's input to +3V3 and
feed both modules from it, with `PERIPH_EN` as its enable; the other switch and
both legacy connectors come out. Until that is done the radio remains on
unswitched +3V3 and the receiver's supply pin is unconnected.

### Still unwired

Seven named nets still have a single endpoint, each for a known reason:

| Net | Why |
|---|---|
| `PERIPH_EN` | reaches the processor; awaits the switched rail above |
| `IIS2MDCTR_SCL` / `_SDA` | magnetometer moves onto the power-monitor I²C bus |
| `GNSS_RXD2` | dropped by decision — the receiver keeps its UART pair only |
| ~~`L_RXEN`~~ | **resolved** — now on GPIO44, matching the other two radio boards |
| `LoRa_RX` / `LoRa_TX` | legacy connector `J5`, redundant with the on-board radio |

The receiver's own pins — supply, ground, UART, reset — are also not yet wired.

**What this costs and keeps:**

- **JTAG is spent** — all four pads used.
- **The serial console is kept** — GPIO43/44 stay free. Of the two debug
  channels this is the one worth keeping: it works from first power-on with no
  adapter, and it is where the boot ROM and the bootloader talk.
- **Two strapping pads spare** — GPIO3 and GPIO45. GPIO45 sets the flash rail
  voltage and must read low at reset, so it suits an output that idles low, not
  an input with an unknown source.

**The risk to check first.** Three signals sit on flash-adjacent pads, which are
free only if the part carries no in-package PSRAM. If that assumption is wrong,
those three must move, and the only remaining homes are the two strapping pads
and the console pair — meaning the console gets spent after all. **Confirm the
package variant before this assignment is committed to layout**; it is the one
input that can invalidate the result.

## Firmware note

Two interrupts were carried in hardware that firmware never used — the
magnetometer's and the inertial sensor's second one. Both are already gone from
the working tree, which is why this budget shows two sensor interrupts rather
than four.

Related: the firmware's board map does not yet describe this board. It carries
V7 and V8 maps only, and defaults to V7. Any pin assignment taken from firmware
today describes hardware two revisions behind, and the reduction will invalidate
it again.

## Method

Pad-by-pad membership, net names and existing connections were read from a
`kicad-cli` netlist export of the working tree at the time of writing, with all
57 pads accounted for — none inferred, none omitted. Demand was counted from
nets that require a processor pin and currently have none, excluding supply
rails and connector-side nets.


## Power-up glitches — the safety check that matters

Datasheet Table 2-2 lists pins that glitch during power-up, before firmware
runs. For a board with four pyro channels this is the single most important
property of the assignment, and it holds:

| Pins | Glitch at power-up | What sits there |
|---|---|---|
| GPIO1–14, GPIO17 | **low-level**, ~60 µs | all four pyro fire lines, arm, three continuity, radio bus |
| GPIO18 | low **and high-level**, ~60 µs | radio reset (active-low) |
| GPIO19, GPIO20 | high-level / pull-down | USB pair, pre-existing |

**Every pyro output is on a low-glitch pin.** `PYRO1_FIRE`–`PYRO4_FIRE` (GPIO4–7)
and `PYRO_ARM` (GPIO8) sit in the GPIO1–14 group, which glitches *low* only —
they cannot be driven high by the chip before firmware takes control. That is
the property the design needs and it is satisfied by construction.

**GPIO18 is the one pin that glitches high**, and it carries the radio's
active-low reset. A high glitch releases reset early rather than asserting
anything, so it is harmless — but nothing that must not assert should ever be
moved onto GPIO18.

**GPIO45 defaults safe.** Table 2-1 gives it a weak pull-**down** at reset, so
VDD_SPI selects 3.3 V by default. That is precisely why it must stay free: an
external pull-up would select 1.8 V and the board would not boot.


## The radio's RF switch — matching the other boards

The E220 module's transmit/receive switch is split the way the vendor prescribes,
and both existing radio boards implement it identically:

| | `lora-daughterboard` | `base-station-mini` | here |
|---|---|---|---|
| DIO2 → TXEN | shorted on-board | shorted on-board | shorted on-board |
| RXEN | GPIO35 | GPIO35 | **GPIO44** |
| DIO3 | floating | — | floating |

Only the **RX half** reaches a processor pin. The transmit half is driven by the
radio itself once DIO2 is configured as RF-switch control, which is why TXEN
needs no pin. An inverter is *not* the house pattern and would not match either
existing board.

DIO3 is left floating deliberately: the module carries its own passive 32 MHz
crystal, so the radio must never be configured for a DIO3-powered TCXO. Ours is
floating, matching the daughterboard.

**Firmware already supports this** — `TR_LoRa_Comms::Config` carries `rxen_pin`,
and both `radio_board` and `base_station` set it. **But the out-computer firmware
defines no `LORA_RXEN_PIN`**, so the pin will do nothing until the board map for
this board declares it. That map has to be written from scratch anyway.

Worth knowing: the base-station firmware records this exact defect having bitten
before — *"was defined but never driven — RXEN floated in RX"*. A floating
receive-enable is a known failure on this hardware, not a theoretical one.


---

## The split

A second ESP32-S3RH2 was added as the flight computer. The signals this document
placed did not move pads — they moved *processors*. Everything the flight
computer owns kept the exact GPIO number assigned above, so the reasoning for
each choice (why `PYRO4_CONT` is on GPIO42 and not GPIO45, why `BMP585_INT`
left GPIO26) carries over verbatim.

### Flight computer `U32` — on `V_MCU_SWTCH`, starts off

| Signal | GPIO | Note |
|---|---|---|
| `SENS_SDI` / `SENS_SDO` / `SENS_SCLK` | 1, 2, 34 | unchanged |
| `ISM6HG256_CS` / `ISM6HG256_INT1` | 9, 48 | unchanged |
| `BMP585_CS` / `BMP585_INT` | 47, 41 | unchanged |
| `GNSS_TX` / `GNSS_RX` | 39, 40 | unchanged |
| `PYRO1..4_FIRE` | 4, 5, 6, 7 | unchanged |
| `PYRO1..4_CONT` | 10, 11, 12, 42 | unchanged |
| `PYRO_ARM` | 8 | unchanged |
| `ESP_SDO` / `ESP_SDI` | 13, 14 | new — I2S data and frame sync to the OC |
| `ESP_CS` / `ESP_SCLK` | 18, 21 | new — I2S word select and bit clock |
| `ESP_SCL` / `ESP_SDA` | 33, 35 | new — I2C to the OC, 5.11 k to `V_MCU_SWTCH` |
| `FC_EN_HOLD` | 17 | new — self-hold into `D9`. **Deliberately not GPIO3** — see the correction above. **Must stay in GPIO0–21**: the in-flight latch needs `gpio_hold_en`, which only reaches the reset-surviving RTC hold on an RTC pad (`SOC_RTCIO_PIN_COUNT = 22` on this part). Above GPIO21 the call still returns `ESP_OK` but degrades to a deep-sleep-only hold |
| `FC_D−` / `FC_D+` | 19, 20 | USB, through the `U1` mux |

**Spare on the flight computer: GPIO3, 36, 37, 38, 43, 44** — six pads, with
the serial console (43/44) among them and therefore intact. GPIO45 and GPIO46
carry the strapping network as before; **GPIO26 stays unused** for the
quad-PSRAM reason argued above, and **GPIO3 is now a bare pad** with no trace on
it, which is the safest state for a strapping pin the design has no use for.

### Out computer `U15` — on `+3V3`, always on

Keeps the radio (`L_CS`, `L_BUSY`, `L_DI01`, `L_RST`, `L_RXEN`), the shared
memory bus (`M_SCK`, `M_MOSI`, `M_MISO`, `M_FLASH_CS`), the power-monitor and
magnetometer I2C (`SEN_SCL`, `SEN_SDA`), its boot flash and USB. It gains the
six `ESP_*` link pins on GPIO1–6 and `FC_EN_OC` on GPIO7.

**Spare on the out computer: GPIO8, 9, 10, 11, 12, 34, 39, 40, 41, 42, 47, 48**
— twelve pads freed by the sensors, GNSS and pyro moving away.

**GPIO39–42 are among them, so the out computer has regained hardware JTAG.**
The single-MCU board spent all four on the GNSS pair, `BMP585_INT` and
`PYRO4_CONT`; those signals now belong to the flight computer, which spends its
own four. Nothing was designed to recover the out computer's JTAG — it is a side
effect of the split, and it is worth knowing before someone spends those pads
again.

### Why these numbers

GPIO1–7 on the out computer are `rocket-computer`'s out-computer numbers exactly
(`ESP_CS`=1, `ESP_SCLK`=2, `ESP_SDO`=3, `ESP_SDI`=4, `ESP_SDA`=5, `ESP_SCL`=6,
enable=7), so that firmware ports with a board header alone.

The flight computer matches on the two I2S clock lines but not the data pair.
`rocket-computer`'s P4 puts `ESP_SDO`/`ESP_SDI` on GPIO19/20; on an S3 those
pads *are* USB D−/D+, and this board spends them on the USB mux. GPIO13/14 take
their place, which is a two-constant change in the flight-computer board header.

**The pressure this document was written about is gone.** Two processors offer
roughly 50 usable pads against a demand of 25 plus six link wires and two enable
lines. The interesting constraint is no longer count — it is which pads on an S3
can host USB and I2S at the same time, and the answer is that they cannot.
