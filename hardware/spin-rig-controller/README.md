# Spin rig controller, V1

A 4-layer, 110 x 80 mm board that runs the StepperOnline closed-loop NEMA 23 kit on the rig's 6S pack.

- **Kit:** 1-CL57T-S20-V41, a 23HS30-5004D-E1000 motor (2 N·m) with a CL57T-V41 driver.
- **What it does:**
  - an ESP32-S3 controller;
  - the 5 V supply for the Pi;
  - a regen clamp;
  - driver current and voltage monitoring;
  - the CL57T step/direction/enable interface;
  - an encoder tap;
  - the index sensor;
  - the mosaic-go's PPS and EventA.
- **What it replaces in the V2 design (section 9):**
  - the Pico carrier board;
  - the Pololu 5 V buck;
  - the Pololu shunt regulator;
  - the 74ACT244 and RS-422 adapters;
  - the proto board.

**Status (2026-10-16):** the schematic and board are generated and checked.
- **ERC:** 0 errors. Four warnings remain, all from the shared library's SP0503BAHTG symbol (its pins are typed "unspecified"; the Beetle uses the same symbol).
- **The repo's parity gate** (`tools/check_board_parity.py`): 170 board symbols, all in the layout and linked by path.
- **DRC:** 0 errors, 0 unconnected items, schematic parity clean. Two warnings remain: the USB-C connector's own silkscreen runs past the board edge, as the connector does.
- **Routing:** an autorouter (Freerouting 2.4.1) did most of it. I pre-routed the parts that matter (Kelvin sense, USB-C pairing, 5 V feeders, driver feed), closed the router's leftovers with a small router of my own, and checked everything against KiCad's DRC. It is not hand-tuned (see "Before ordering").
- **Nothing has been ordered.**

Files:
- `spin-rig-controller.kicad_pro`: the KiCad 10 project.
- `.kicad_sch` files: a root sheet plus `power`, `mcu` and `io` sheets.
- `.kicad_pcb`: the board.
- `FABRICATION-NOTES.md`: the order notes.
- **Libraries:** the repo's shared ones (`../symbols/Custom.kicad_sym`, `../footprints/Footprints.pretty`, `../3dmodels`). This board added three symbols (TLV3011, UCC27517, AO4264E) and one footprint (R_0805_Kelvin) to them.
- `gerbers/` (ignored by git), written by `gen/make_fab.py`:
  - a review copy of the gerbers;
  - the JLC BOM and CPL with LCSC part numbers;
  - the schematic as a PDF;
  - renders.
- `gen/`: the scripts that build all of the above from one parts list, `gen/design.py` (see "Regenerating"). Their scratch output, `gen/out/`, is ignored too.

## What I reused from the rocket boards

- **The ESP32-S3:** the Tinker-Beetle's footprint (`Footprints:IC_ESP32-S3`).
  - Its 40 MHz crystal circuit: ECS-400-10-37B2, the 24 nH series inductor, and 12 pF loads.
  - Its CHIP_PU RC and GPIO46 pull-down.
  - The part is the ESP32-S3FN8, because the boards' ESP32-S3RH2 has no JLC stock. The FN8 has 8 MB of flash in the package, so there is no external flash chip.
- **USB-C:** the Beetle's J6 circuit: GCT USB4110GFA, SP0503BAHTG ESD array, 5.1 kΩ CC pull-downs, 22 Ω in D+/D-. The S3's own USB serves both flashing and the data link to the Pi.
- **The 2 mΩ PA0805 shunt** (the Beetle's R72), here Kelvin-sensed.
- **Connector families:**
  - JST VH for power, as the Mantis battery connector;
  - JST PH SM4 for signals and SH for I2C, as the Beetle and Mantis use them;
  - the rocket boards' 2-pin power convention: pin 1 GND, pin 2 positive.
- **The Pi header** is the SKR Pico's P5, pin for pin (5 V, 5 V, GND, RX, TX to Pi pins 2, 4, 6, 8, 10), so the V1 harness plugs straight on.

The power stage is new: the rocket boards run from 1S/2S and none of their regulators take 25 V.

## Power

**Input and the driver feed (sheet `power`):**
- J101 takes the pack, after the anti-spark switch and the inline 10 A fuse.
- Q101 (NCE60P50K, 60 V P-FET, gate held at -12 V by D101) blocks a reversed plug. Once on, it conducts both ways, so regen still flows back into the pack.
- D102 (SMCJ30A) and C101 (100 µF, 50 V) take the plug-in spike.
- The driver's current goes through R102 (2 mΩ) to J103.
- U101 (INA238, 85 V, I2C 0x40) reads the driver's current (both directions, so regen shows) and the bus voltage at the shunt.

**Regen clamp:**
- **What it does:** it replaces the Pololu shunt regulator, for the case the V2 doc flags: the pack's path opening during a stop.
- **How it switches:** U102 (TLV3011, 1.242 V reference) compares the fused bus with its reference, with hysteresis. It turns Q102 (AO4264E) on at 28.8 V and off at 27.7 V (typical), through a UCC27517 gate driver.
  - At the worst-case tolerances (reference at its 1.208 V minimum, offset, 1 % resistors) it still turns off above about 26.3 V. That keeps it clear of a full 25.2 V pack, so a regen spike can't latch it on against the pack.
  - The highest it can turn on, about 30.6 V, is still below the SMCJ30A's 33.3 V breakdown.
- **The resistor:** Q102 switches an external brake resistor on J102.
  - Use 6.8 Ω, at least 25 W, aluminum-housed, screwed to the hub plate as its heatsink.
  - At 28.8 V it draws 4.2 A (122 W while on). The rotor's 20-32 J is a fraction of a second of that.
  - C123 (4.7 µF) sits at the switch as a local bus capacitor. A dedicated 1.5 mm trunk on F.Cu carries the bus from the shunt down to the brake.
- **What it costs in normal use:** with the pack connected the bus never gets near 28.8 V (a full 6S pack is 25.2 V), so the clamp does nothing.
- **Indication:** the red BRAKE light (D104) and a GPIO show when it fires.

**5 V (U104, TPS54560B):**
- **The design:** TI's own 5 V / 5 A reference design, moved to JLC basic-part values.
  - 490 kHz (R115 = 200 k);
  - compensation 15 k / 4.7 nF / 47 pF;
  - 6.8 µH Sunlord MWSA1004S (12 A saturation);
  - B560C catch diode;
  - 3 x 47 µF output.
- **Output:** 5.13 V, a little high for the Pi on purpose.
- **Input:** after F101 (2 A).
- **Undervoltage lockout:** it starts at 18.8 V and stops at 17.1 V (2.85 V a cell). A rig left switched on can't flatten the pack through the Pi.
- **Layout, per TI's guide:**
  - The input caps (C108-C110) are beside VIN.
  - The catch diode's anode lands against their ground pad.
  - Its cathode and the inductor share a short 1.5 mm switch node off pin 8, poured wider.
  - The thermal pad has no vias in it (the 4-layer rule). It connects through the top ground pour at its open end, so expect the IC to run warm at the full 5 V load.
- **The 5 V output** reaches the In2 plane through five 0.6/0.3 mm vias between the inductor and the output caps. The Pi and AUX fuses draw from it through three and two vias. The Pi feed itself is a 1.5 mm track.
- **Outputs:**
  - the Pi header (J202, moved down into the 5 V band), through F103 (4 A);
  - J104 AUX (PH 2-pin, 1.5 A resettable fuse), for the V0 SDR board or the Beetle;
  - the driver interface, the clamp and the index sensor.

**3.3 V (U105, AP2112K):**
- Fed from 5 V or from USB through B5819WS diodes. USB alone runs the logic, so the board can be flashed from a laptop with the pack off; the Pi stays unpowered then.

**Budget:** about 8.4 W at 5 V (V2 doc, section 11), about 0.4 A from the pack. The buck has 2x headroom.

## The CL57T interface (sheet `io`)

- **J301** (PH 10-pin): PUL+ PUL- DIR+ DIR- ENA+ ENA- ALM+ ALM- PEND+ PEND-, wired to the driver's control terminal in that order.
- **Drive:** a 74ACT244 at 5 V drives PUL+, DIR+ and ENA+ (±24 mA, through 33 Ω). The minus terminals are ground.
- **Set the driver's S3 switch to 5 V** (it ships at 24 V).
- **STEP and DIR** stay off through reset: R301 holds the buffer's output enable high until the firmware enables it.
  - DRV_OE_N is on GPIO21, which doesn't glitch at power-up (GPIO1-14 can pulse low for 60 µs).
  - R302 (4.7 k) holds STEP low against GPIO39's weak pull-up during reset.
- **ENA through reset:**
  - ENA has its own always-on buffer. Its level through reset is set by R304 (pull-up, fitted) or R305 (pull-down, not fitted).
  - The CL57T manual says an unconnected ENA means enabled. So a driven ENA (opto current) should mean disabled, and R304 then holds the driver off through reset.
  - **Check this on the bench** before trusting it. If it is the other way round, move the resistor to R305.
- **ALM and PEND** are the driver's optocoupler outputs, pulled up to 3.3 V here.
- **Firmware:** STEP must be at least 2.5 µs high and low, with DIR set 5 µs before the edge (the V2 doc's numbers).

## Encoder tap, index, GNSS timing (sheet `io`)

- **J302** (PH 8-pin: EA+ EA- EB+ EB- EZ+ EZ- EGND):
  - wired in parallel with the driver's encoder input, label to label;
  - read by an AM26LV32E (3.3 V RS-422 receiver, fail-safe on open inputs);
  - the Z channel is there in case this encoder has one;
  - the 120 Ω terminations (R313-R315) are not fitted: measure whether the driver terminates the pairs first;
  - R316 (0 Ω) ties the encoder ground to the board ground. Remove it if the driver's encoder ground turns out not to be its supply ground.
- **J303 index** (PH 4-pin: GND, signal, LED anode, 5 V):
  - takes a slotted optical switch (its LED fed through the 330 Ω on the pin) or a Hall switch (5 V supply, open-drain out);
  - the signal goes through a 74LVC1G17 Schmitt buffer, which tolerates 5 V.
- **J304 GNSS** (PH 3-pin: PPS, GND, EventA) mates the mosaic-go's pins 8, 9, 10. Keep the 1 kΩ resistors at the receiver end, as on V1.
- **EventA has a hardware path** from either source, through a 74LVC2G125 with separate enables:
  - the index (a few ns, no firmware in the path);
  - an MCU pin (the commanded turn).
  - Both buffers are off through reset, and R326 (100 k) holds EventA low then. The firmware should enable one buffer, and only while PPS is alive, as V1 does.

## ESP32-S3 pins

The S3's GPIO matrix lets any of these signals sit on any pin. I chose pins on the side of the chip that faces the
connector each signal goes to; that cut the routing problems the first pin map had.

| Side (faces) | Pin / GPIO | Signal |
|---|---|---|
| Left (power column) | 6 / 1 | ALM |
| | 7 / 2 | PEND |
| | 8 / 3 | INA_ALERT |
| | 9 / 4, 10 / 5 | I2C SDA, SCL (INA238 and Qwiic J203) |
| | 11 / 6 | BRAKE_SENSE |
| | 12 / 7 | DIR |
| | 14 / 9 | VBUS_SENSE (ADC1, 100 k / 10 k from the fused bus) |
| Bottom (USB, lights, Pi) | 15 / 10 | V5_SENSE (ADC1, 10 k / 10 k) |
| | 16 / 11, 17 / 12 | LED_FAULT (red), LED_STAT (white) |
| | 18 / 13, 22 / 16 | PI_TX, PI_RX (UART1) |
| | 25 / 19, 26 / 20 | USB D-, D+ |
| | 27 / 21 | DRV_OE_N (no power-up glitch on this pin) |
| Right (encoder, index, GNSS) | 36 / 48 | EVT_IDX_OE_N |
| | 37 / 47 | EVT_MCU (EventA from the MCU) |
| | 38 / 33 | PPS |
| | 39 / 34 | INDEX |
| | 40 / 35, 41 / 36, 42 / 37 | ENC_Z, ENC_B, ENC_A |
| Top (driver and EventA buffers) | 43 / 38 | EVT_MCU_OE_N |
| | 44 / 39 | STEP |
| | 48 / 42 | ENA |
| | 49 / 43, 50 / 44 | UART0 debug (test pads TP201, TP202) |
| Fixed | 5 / 0 | BOOT button |
| | 4 | CHIP_PU (RC, RESET button) |
| | 52 / 46 | 10 k pull-down (strap) |

Free: GPIO 8, 14, 15, 17, 18, 40 and 41. GPIO 14 (pin 19), 40 (pin 45) and 41 (pin 47) sit next to 3.3 V
pins and are left unconnected on purpose, so those pins have room for their vias.
- GPIO 33-37 and 47-48 are used. That rules out an octal-PSRAM S3 (R8), which needs them; the FN8 (quad flash in the package) leaves them free.
- GPIO 3 is a strapping pin only when the JTAG eFuse is burned. Here it carries INA_ALERT, which idles high.

## Timing on the S3

The V1 firmware's PIO programs don't port: the S3 has no PIO. The S3 does have the peripherals for the same job:

- **MCPWM capture:** two units, three channels each, with a 12.5 ns timer. Both capture timers can be synced to PPS through the MCPWM sync input. The six channels cover PPS, STEP (read back on its own pin), ENC_A, ENC_B, ENC_Z and INDEX. Every edge is timed on the same clock as PPS.
- **STEP generation:** RMT with DMA, or an MCPWM generator synced to PPS. With every STEP edge measured, the truth no longer depends on the step train starting a fixed delay after PPS.
- **PCNT:** counts A/B quadrature as a cross-check.
- **Load:** about 80,000 captures a second at 10 rev/s (encoder plus STEP), roughly a tenth of one core in an IRAM interrupt. Core 1 can carry it.

All of these take any GPIO through the GPIO matrix, so the pin map above doesn't constrain the firmware. The scheduler and the host tools carry over. The PIO stepper, the PPS capture and the TMC2209 code are replaced. I'd budget one to two weeks for that port.

## Board

- **Stackup:** JLC 4-layer, 1.6 mm (JLC04161H-7628), 1 oz outer copper.
  - F.Cu: parts and signals; motor-current and ground pours.
  - In1: solid ground, no tracks.
  - In2: a 5 V plane along the bottom (no tracks there), stepping up to 50 mm under the buck. Three 0.5 mm feeders run up from it to the clamp, the 74ACT244 and the index connector. The rest of In2 carries signals.
  - 3.3 V is routed as tracks. A pour on In2 broke into 32 one-via fragments around the signals there, so I took it out.
  - B.Cu: signals and a ground pour.
- **Ground stitching:** about 450 vias tie the outer ground pours to In1, on a 3 mm grid wherever one fits, and on a 1.5 mm grid around the ESP32.
- **Parts:** all on the top side.
  - The three VH connectors and the 1x5 header are through-hole: hand-solder them, or pay for JLC's through-hole assembly.
- **Vias:** tented, none in pads (the rule for the 4-layer rocket boards).
  - The ESP32's center pad (its only ground) has four ground vias in the corners of the ring between it and the pins, the closest a via can sit without going in the pad.
  - The decoupling caps around it are stitched on a 1.5 mm grid.
- **Crystal:** short F.Cu runs to the 40 MHz crystal and its 24 nH inductor, with no vias.
- **Net classes:** Default 0.2 / 0.15 mm, Power 0.5 / 0.15 mm, Motor 1.5 / 0.3 mm. The motor nets (pack, bus, driver feed, brake) are also poured 1.2 mm around their tracks on both outer layers.
- **Kelvin sensing:** R102's sense lines are pre-routed (locked) from its inner sense pads straight to U101, so the router can't tap the current path.
- **Mechanical:** four M3 holes (not plated, so the board ground doesn't touch the frame).
  - The board spins with the rotor. At 10 rev/s and 50 mm out it sees about 20 g.
  - The heavy parts (C101, L101, the VH connectors) are surface-mount or held by their pins. A dab of adhesive on C101 and L101 costs nothing.

## Cost

| Item | Estimate |
|---|---|
| Parts, per board, JLC prices for 5 boards | about $25 (the INA238 is $8 of it) |
| JLC extended-part fees | about $108 per order (36 part types at $3). Every passive and LED is a basic part |
| PCB, 4-layer, 5 pieces, plus assembly setup | about $40-60 |
| **5 assembled boards, before shipping** | **about $280** |
| The motor kit (1-CL57T-S20-V41) | $58.66 (it replaces the NEMA 24 kit's $93.99) |

The board saves these V2 BOM lines: the Pico ($4), the Pololu buck ($39.95), the shunt regulator ($23.33), the 74ACT244 and RS-422 parts, adapters and proto board (about $21). That is about $88 of the V2 BOM, roughly what one assembled board costs. The external brake resistor (about $5-10) is new.

## Before ordering

1. **Look over the routing in KiCad.** It is mostly autorouted. I'd look at:
   - the USB pair (short, but not length-matched);
   - the crystal;
   - the buck. I placed it and pre-routed its switch node; the rest of its copper is the router's.
     - The bootstrap cap (C112) sits about 7 mm from the switch node, joined by a B.Cu trace. There's no room for it at the package's top end without moving the catch diode off the input caps. Its loop carries only gate-charge pulses.
     - The thermal pad has no vias (the 4-layer rule), so the IC will run warm at full load.
2. **The ENA default (R304/R305) and the CL57T's S3 switch:** the bench checks above.
3. **The three power connectors are the same JST VH part**, side by side: PACK IN, DRIVER PWR and BRAKE R.
   - A pack plugged into DRIVER bypasses the reverse-polarity switch; reversed, it shorts through D102.
   - Crimp the three harnesses so they can't swap (different lengths, colored sleeves), and check the silkscreen before plugging in.
4. **The 23HS30's shaft is 8 mm** (the NEMA 24 was 10 mm), so the V2 coupling becomes an SCPW28-8-x.
5. **In JLC's CPL preview**, check the rotation of every IC and connector. KiCad's and JLC's zero orientations differ for some packages.

## Independent review

On 2026-10-16 a second reviewer checked every pinout against its datasheet and the footprint pads, all of the circuit math, and the routed copper. It found no blockers.

**Majors, all fixed in this version:**
- **ESP32 ground:** the ESP32's center pad reached the ground plane through one via. It now has four.
- **5 V output:** it ran through two small vias. There are now via arrays at both plane transitions.
- **Buck layout:** the hot loop was spread out on thin tracks. It is now laid out per TI's guide.

**Minors fixed:**
- the clamp's worst-case off threshold (R126 is now 20 k);
- the bus capacitor at the brake (C123);
- DRV_OE_N moved off a pin that glitches at power-up, and R302 is now 4.7 k;
- an EventA pull-down (R326);
- the crystal routing.

**Left for the bench:** the CL57T manual wasn't reachable, so its terminal order, input current and ENA polarity are still to check.

## Bring-up

1. **Bench supply, current-limited to 0.3 A, at 24 V, nothing connected.**
   - The 5 V rail is 5.10-5.16 V and 3.3 V is present.
   - Draw is under 30 mA.
   - Sweep the supply down: the buck stops near 17.1 V and restarts near 18.8 V.
2. **USB only.** The S3 enumerates. Flash a blink that toggles STATUS and FAULT.
3. **Read the INA238:** bus voltage, and about zero current.
4. **The clamp:** a 28-30 V bench supply with the brake resistor fitted. BRAKE lights at about 28.8 V and goes out at about 27.7 V.
5. **The driver:** S3 at 5 V, motor unloaded. Check the ENA polarity (driver off with the S3 in reset), then STEP/DIR at a slow rate.
6. **The encoder:** spin the motor shaft by hand and watch the counts.
7. **Then the pack**, through the anti-spark switch and the 10 A fuse.

## Regenerating

From `gen/`:
- `python3 make_sch.py`: the schematic, from `design.py`. It reads the shared libraries in `hardware/symbols` and `hardware/footprints`.
- `python3 make_pro.py`: the project file and net classes.
- `python3 lcsc_lookup.py`: the LCSC codes, into `lcsc.json`.
- `./route_best.sh`: the board.
  - It places the parts, pre-routes and adds the planes (`make_pcb.py place`).
  - It routes three ways in parallel with Freerouting.
    - Set `FREEROUTING_JAR` to a Freerouting 2.x jar ([github.com/freerouting/freerouting](https://github.com/freerouting/freerouting); I used 2.4.1).
    - It needs Java 25 or newer: set `JAVA`, or let it default to Homebrew's `openjdk`.
  - It finishes each result (`make_pcb.py finish`: the pours) and fixes it (`fixup.py`: plane fan-out and stitching), then keeps the best.
- `./finalize.sh`: field and keep-out fixes (`post_fix.py`), the leftovers routed (`patch_route.py`), re-stitching, and DRC.
- With KiCad's Python (`/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3`):
  - `silk.py`: references to the fab layer, function labels on the silkscreen;
  - `cleanup.py`: dangling vias and stubs out.
- `python3 check.py`: DRC with schematic parity.
- `python3 make_fab.py`: the review package and the JLC BOM/CPL, into `gerbers/`. At order time, plot the gerbers with `tools/plot_gerbers.sh spin-rig-controller` and tag the commit; see `FABRICATION-NOTES.md`.

Freerouting's results vary from run to run. If a rerun leaves more than a few unconnected items, run `route_best.sh` again.

This version took two hand-directed fixes after `finalize.sh`, both done with `patch_route.py`'s functions:
- The input cap's VMAIN_F pad was joined to C111.
- Two BUCK_EN segments that came within 0.14 mm of VMAIN_F were ripped up and rerouted.

`extra_pours.py`, `silk.py` and `cleanup.py` ran after that.

Rerunning `make_pcb.py place` or `route_best.sh` discards hand edits to the board. Once the layout is being edited in KiCad, change the schematic in KiCad too, and use Update PCB from Schematic.
