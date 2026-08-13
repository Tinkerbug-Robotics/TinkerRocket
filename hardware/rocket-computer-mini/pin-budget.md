# rocket-computer-mini — MCU pin budget

**Purpose.** Establish whether the single MCU can carry the reduced board by
itself, now that the telemetry radio is on-board rather than on a daughterboard.

**Answer: not as currently drawn.** The demand is 33 pins against 16 that are
unconditionally free, or 27 if every strapping, debug and console pin is spent.
The board is short by **at least 6 pins** even in the most aggressive case.

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

**On the PSRAM three.** The design already uses four of the octal-PSRAM pin
group as ordinary GPIO, which implies the part is not an octal-PSRAM variant and
that these three are genuinely free. That inference is consistent but it is an
inference — confirm it against the ordered part before counting on them.

**The realistic ceiling is 16.** A flight computer that cannot be debugged over
JTAG or watched over a serial console during bring-up is a poor trade for four
pins. Treat the 27 as a theoretical maximum, not a plan.

## Demand — what the board asks for

| Group | Signals | Pins |
|---|---|---|
| Inertial + baro | shared SPI (3), two chip selects, two interrupts | 7 |
| Magnetometer | **shares the power-monitor I²C bus** — no new pins | 0 |
| GNSS | **UART pair only** — no second RX, no branch enable | 2 |
| Telemetry radio | SPI (4), BUSY, RST, DIO1 | 7 |
| Pyro | four fire, four continuity, arm | 9 |
| Housekeeping | rail power-good, power switch sense | 2 |
| **Core total** | | **27** |
| Legacy radio connector | UART pair + branch enable, now redundant | 3 |
| **As currently drawn** | | **30** |

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

| Change | Saves | Note |
|---|---|---|
| Remove the legacy radio connector and its branch enable | 3 | Pure redundancy — the on-board module replaces it |
| Pyro continuity onto an I²C expander | **4** | Zero GPIO — see below |

**Continuity belongs on the I²C bus, not a multiplexer.** A four-to-one
multiplexer needs two select lines plus a data line — three pins to replace
four, saving one. An expander on the bus the magnetometer is already joining
costs **no processor pins at all** and saves all four.

Continuity is a digital read, not an analogue one — firmware reads it as a GPIO
level, and nothing on this board needs an ADC channel. That matters, because it
means the ADC2-versus-radio restriction does not constrain this design at all.
Bus latency is irrelevant here: continuity is a status check, not a control
path.

| Configuration | Pins | Fits 16 free? | Fits 27 ceiling? |
|---|---|---|---|
| As drawn, legacy connector removed | **27** | no | exactly, zero margin |
| Also: continuity on the I²C expander | **23** | no | yes — 4 spare |

**27 fits only on paper.** It consumes every strapping pin, the serial console,
JTAG and the three flash-adjacent pads at once, leaving nothing for a pin that
turns out to be unusable or a bodge during bring-up. **23 is the number that
makes the board buildable**, and it is reached with one small part.

**Caution on the pyro lines.** Continuity sensing is a fine candidate for the
expander; the four *fire* lines are not. Putting a shift register or expander
between the processor and a pyro gate adds a failure mode that can assert an
output without the processor commanding it, and adds latency to the one path
where latency is least acceptable. Sensing goes on the bus; firing stays direct
on dedicated pins.

## Assignment — as built

**This is wired in the schematic.** All 24 signals below are connected to the
processor by global label at the pin, and the netlist confirms every one reaches
it. The count is 24 rather than 23 because a single peripheral power enable was
added, `PERIPH_EN` on GPIO3 — one pin powering both the receiver and the radio.

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
| L_SCK | GPIO10 | free | radio bus |
| L_MOSI | GPIO11 | free | radio bus |
| L_MISO | GPIO12 | free | radio bus |
| L_CS | GPIO13 | free | radio bus |
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
