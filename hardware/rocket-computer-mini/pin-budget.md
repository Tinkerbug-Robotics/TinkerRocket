# rocket-computer-mini — MCU pin budget

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
| Console | GPIO43, GPIO44 | UART0 | Safe as GPIO; cost is the serial console. **Both left free.** |
| Strapping | GPIO0, 3, 45, 46 | boot mode, JTAG select, flash rail voltage, ROM log | GPIO3 is safe here — it drives a switch enable and its pulldown holds it low at reset. **GPIO45 left free**: it sets the flash rail voltage, and anything whose boot level the design does not control can stop the board booting. |

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
| `L_RXEN` | radio receive-enable, floating and absent from firmware |
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
