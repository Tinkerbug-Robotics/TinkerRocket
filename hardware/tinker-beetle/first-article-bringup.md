# Tinker-Beetle (rocket-computer-mini) — first-article bring-up and fault isolation

Written 2026-09-21 for the first article, which has never enumerated on USB. The design
values quoted here come from the netlist and the parts' datasheets; nothing here has been
measured on hardware. Results belong on
[#1316](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/1316).

## What the bench already knows, and what it does not

The host's power-delivery controllers reported no sink termination on any port, in either
switch position, across several re-plugs. Three things follow.

- **The fault is upstream of anything powered.** The termination is two resistors to
  ground on the connector's configuration pads. No regulator, no multiplexer, no strapping
  pin and no firmware can hide it, so a dead rail or a wrong switch position is not the
  explanation.
- **It is not a design error.** The whole path is identical to the fabricated, working
  full computer: same netlist, same connector footprint with the same pad geometry, same
  resistor values.
- **It is therefore physical.** Something is open between the plug's contacts and ground:
  the connector's signal row, the pull-down resistors, the plug not seating, or the cable.

## Order of work

Do these in order. Each step tells you whether to continue or to stop and fix.

### Step 1: unpowered, nothing plugged in

Measure to board ground with the board unpowered. Use the connector's own contacts where
the table says so, which needs a breakout or a probe on the receptacle tongue.

| # | Measure | Where | Expect | If wrong |
|---|---|---|---|---|
| 1.1 | Resistance | Receptacle contact CC1 (A5) to ground | 5.11 k | Go to 1.2 |
| 1.2 | Resistance | Receptacle contact CC2 (B5) to ground | 5.11 k | Go to 1.3 |
| 1.3 | Resistance | The connector-side pad of R41, and of R47, to ground | 5.11 k each | Resistor missing, tombstoned, or its ground pad is open. Inspect both under magnification. |
| 1.4 | Resistance | R41 connector-side pad to J6 pad A5; same for R47 to B5 | under 1 ohm | The connector's signal row is not wetted. This is the prime suspect. |
| 1.5 | Resistance | Each of the four shell pads, and the A1/B12 and B1/A12 pads, to ground | under 1 ohm | The connector's ground return is open, which removes both terminations at once |
| 1.6 | Resistance | VBUS pad to ground | not a short | Stop. Find the short before applying power. |
| 1.7 | Resistance | +3V3 to ground | not a short | Stop. |
| 1.8 | Continuity | J6 D+ to U1 pin 1, J6 D- to U1 pin 2 | under 1 ohm | Same signal-row fault as 1.4 |
| 1.9 | Continuity | U1 pins 9 and 10 through R38/R39 to the out computer's USB pins | under 1 ohm | Open at the multiplexer or the series resistors |

**If 1.1 and 1.2 read 5.11 k**, the board presents a correct termination and the fault is
the cable, the port, or the plug not seating. Go to step 2 and also do step 5.

**If 1.1 and 1.2 are open but 1.3 is correct**, the connector's signal row is open. Reflow
it and repeat. This is the mechanism the review ranks first: the signal pins are 0.30 mm
wide on 0.5 mm pitch and get very little paste from the thin foil chosen for the flash
balls, while the shell pads print at full thickness and hold the part down anyway.

**If 1.3 is open too**, a pull-down resistor is missing or lifted. Each of them, and 68
other two-terminal parts on this board, has one pad tied solid into a copper pour and the
other on a thin track, which is tombstone geometry. Measured: 70 of the board's 180
two-terminal surface-mount parts are in that condition.

### Step 2: power from a USB-A to USB-C cable

That cable puts the termination in the cable and supplies power unconditionally, so it
proves the rest of the board without depending on step 1. Put the switch in the
out-computer position. No battery, no supercapacitor.

| # | Measure | Where | Expect |
|---|---|---|---|
| 2.1 | Voltage | J6 VBUS | about 5 V |
| 2.2 | Voltage | U21 pin 1 or 8, the mux output | about 5 V less a small drop |
| 2.3 | Voltage | V_BUCK at L6 output | 3.465 V nominal |
| 2.4 | Voltage | +3V3 at the hold-up converter's output pins 9 and 10 | same as V_BUCK, the converter passes through in bypass |
| 2.5 | Voltage | U1 pin 12, the multiplexer supply | 3.3 V less the drop across R3 |
| 2.6 | Voltage | U1 pin 4 (select) and pin 11 | pin 11 high; pin 4 follows the switch |
| 2.7 | Voltage | Out computer reset pin | 3.3 V after its rise time |
| 2.8 | Voltage | Out computer GPIO0 | high, unless the boot button is held |
| 2.9 | Oscillation | Out computer 40 MHz crystal | running |
| 2.10 | Host | Serial device appears | a USB serial or JTAG device |

The hold-up converter passes power through without a supercapacitor fitted, which is the
intended bring-up configuration. If 2.3 is right and 2.4 is not, suspect that converter.

### Step 3: the flight computer will not appear yet

This is designed behaviour and not a fault. The flight computer's rail is held off until
the out computer raises it. So:

1. Flash and run the out computer first, with the switch in its position.
2. Command the rail on from the app.
3. Only then move the switch to the flight-computer position and look for a second device.

Neither processor has a reset button. The flight computer's download mode needs its boot
button held while the out computer cycles the rail.

### Step 4: battery power

Only after steps 2 and 3. With a pack fitted, check the electronic fuse passes, the pack
monitor reads a real current, and the rails hold. Note that with USB only the monitor can
report a phantom pack voltage, because the channel pull-ups back-feed through the
high-side switches' body diodes.

### Step 5: the plug and the mechanical fit

The receptacle face sits 0.47 mm behind the board edge, where the working full computer
has 0.275 mm. Contacts for the configuration channel mate second, so a plug that bottoms
on the board edge before full insertion shows the host nothing.

- Does the plug click home, or does it stop early?
- Does the plug's overmold touch the board edge?
- Caliper the receptacle face against the board edge.
- Try a different cable and a different plug body, ideally a slim one.

## Paste-defect inspection, in the order the review ranks it

Inspect under magnification, at 30 to 40 degrees rather than straight down, because the
features that matter are side fillets.

1. **The connector's twelve signal pins**, and its four shell pads. The suspected
   mechanism is a part seated on well-fed shell pads with starved signal pins.
2. **Both configuration pull-down resistors**, for a lifted end.
3. **The two processors' pin rows.** The solder-mask webs between their pins measure
   0.070 mm, below the fabricator's 0.10 mm minimum, so the fabricator may have gang-opened
   them and there is nothing constraining bridges.
4. **The pack monitor**, whose mask openings touch in places, and the buck converter,
   whose corner pins have a 0.035 mm web.
5. **The two boot flash parts.** Their ball pads have a mask opening exactly equal to the
   copper, so each ball is randomly mask-defined on one side and copper-defined on the
   other. A bad joint here stops the processor booting its application but does not stop
   the built-in serial device appearing.
6. **The arm switch's source and gate joints.** That part is 3.0 mm on a land drawn for
   3.3 mm parts and its terminals reach the pads by about 0.05 mm.

## A correction to the assembly notes, before anyone acts on them

Note B1 tells the assembler that the buck converter, the electronic fuse and the hold-up
converter get no paste on their exposed pads, and forbids substituting a stencil that has
them. **All three are pasted**, on every version of this board including the one that
shipped, at 40, 70 and 84 percent coverage. Two more bottom-side parts are pasted and are
not mentioned at all. Do not mask those apertures, and do not treat a stencil that has
them as wrong.

## What to record

For each step: the measured value, not just pass or fail. Then:

- Which of the four mechanisms the evidence supports, and which it rules out.
- Photographs of the connector's fillets at an angle, before any reflow.
- Whether a second board behaves the same way, if one exists.
- The foil thickness the assembler actually used, which is still an open question in the
  fabrication notes.
