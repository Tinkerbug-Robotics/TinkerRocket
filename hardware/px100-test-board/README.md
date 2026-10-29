# px100-test-board — PX100 test board

A bench board for bringing up the SkyTraq PX100 and finding out what its undocumented pins do. It has:
- a PX100 with an RF front end for an **active L1/L5 antenna** on an edge SMA;
- an **ESP32-S3** with **every PX100 signal pin on its own GPIO**, through 1 kΩ, on ADC-capable pins;
- one **USB-C** port that a slide switch gives either to the ESP32 or to a **CP2102N USB-UART bridge** wired to the
  PX100's UART, with DTR/RTS reset and boot control, so a Mac can talk to and reflash the PX100 directly;
- power from USB only: the carrier's ADP7142 for the PX100 and the spin-rig controller's AP2112K for the ESP32 side.

**State (2026-10-29):** schematic complete; PCB placed and routed, not yet reviewed by the owner.
- **ERC:** 0 errors. Its 5 warnings: 4 pin-to-pin notes from the shared SP0503BAHTG symbol (its pins are typed
  "unspecified", as on the Beetle and the spin-rig controller), and 1 off-grid wire end inherited from the
  px100-module sheet (the PPS LED's wire).
- **PCB:** 164 footprints, 1452 track segments, 623 vias (457 of them GND stitching and exposed-pad vias), GND poured
  on all four layers. DRC with schematic parity: 0 errors, 0 unconnected items, parity 0; the only warnings are J1's
  silk over the board edge, as on the carrier. `tools/check_board_parity.py` passes.
- **Part labels are hidden** on silk and fab. Board texts name the buttons, the switch ends and the header pins.
- **Nothing is committed.**

## Sheets

| Sheet | Holds |
|---|---|
| root (`px100-test-board.kicad_sch`) | the four sheets |
| `receiver.kicad_sch` ("RF chain and PX100") | J1, the bias tee, SkyTraq's L1/L5 split with the carrier's SAWs and matches, the PX100 with its TCXO, RTC crystal, core parts and decoupling, reset/boot/UART pull-ups, the PPS LED, the Laird shield |
| `power.kicad_sch` ("USB-C, USB switch and power") | USB-C with ESD, the FSUSB63 mux and its slide switch, both 3.3 V regulators, power LEDs, test points, holes |
| `esp32.kicad_sch` ("ESP32-S3") | ESP32-S3FN8 with crystal, decoupling, BOOT/RESET, status LED, spare-pin header |
| `pxio.kicad_sch` ("PX100 USB bridge and I/O") | CP2102N, the DTR/RTS reset and boot transistors, PX100 RESET and BOOT buttons, the 1 kΩ links, the PX100 probe header, the antenna-bias monitors |

The receiver sheet is drawn by hand. It started as px100-module's receiver sheet; `gen/edit_receiver.py` made the
one-time changes listed below. The other three sheets are generated from `gen/design.py` (see "Regenerating").

## RF front end: an active antenna

An active L1/L5 antenna brings its own LNA, so the carrier's on-board LNA stage (FL4/U3/FL5/U4 and their parts) is
gone. What is left is SkyTraq's own reference front end (PX100 data sheet p13), which assumes an active antenna:

```
J1 SMA ─┬─ D6 ESD ─┬─ L1 47 nH ─ VANT ─┬─ R2 0R ── +3V3            (fitted)
        │          │                   ├─ R3 0R ── PX100 ANT_BIAS  (not fitted)
        │          │                   └─ C20 1 uF
        │          └─ C26 33 pF ─ split (C33, L10/L11, C34/C35, R10) ─┬─ FL6 B4327 (L1) ─ C36 + L14 ─ LNA_IN1 (pin 40)
                                                                     └─ FL7 SAFFB1G17 (L5) ─ L12, C37 + L15 ─ LNA_IN2 (pin 37)
```

- **The split, SAWs and matches** are the carrier's V2 parts unchanged: C37 is 1.3 pF and L13 stays unfitted, so
  neither PX100 RF input has a DC path to ground.
- **Bias tee:** L1 (Murata LQW15AN47NG00D) and C20 are SkyTraq's L2 and C4. C26 blocks the bias from the split.
- **Antenna power, R2 fitted:** +3V3 goes straight to the antenna, as in SkyTraq's reference schematic.
- **Antenna power, R3 instead:** the PX100's ANT_BIAS output (pin 9), which the data sheet says limits a short to
  45 mA. The reference doesn't use this pin, and the stock firmware may not switch it on, so it is the option and not
  the default.
- **The ESP32 reads both** ANT_BIAS (GPIO15) and VANT (GPIO16), each through a 100 k / 100 k divider. So the first
  power-up shows whether ANT_BIAS is live, and an antenna short shows up as VANT dropping.
- **D6** (TPD1E0B04, ±3.6 V working) is fine with 3.3 V of bias.
- **When J1 is fed from an SDR or the HackRF rig,** put a DC block in the lead, or remove R2.
- **Shield:** SH1 + SH2 are the carrier's Laird BMI-S-202-F frame and BMI-S-202-C cover, the V1 parts already on
  hand. They cover U1, Y1 and the split. J1, the bias tee, Y2 and R11 sit outside, as on the carrier.

## Every PX100 pin on an ESP32 GPIO

Every PX100 pin that is or might be a signal reaches the ESP32 through a 1 kΩ resistor. The resistor keeps either
side from damaging the other if both drive the line. The ESP32 pins were picked to be as flexible as inputs can be:
- **No strapping pins, no USB, no flash pins.** GPIO3 (JTAG strap) stays free.
- **Every unknown pin can also be read as a voltage:** the nine PX100 GPIOs sit on ADC1 (GPIO1–10), the four
  reserved pins on ADC2 (GPIO11–14).
- **Touch and RTC capable:** GPIO1–14 are also touch pads and RTC GPIOs, so the ESP32 can wake from deep sleep on a
  PX100 edge.
- **RSTN and BOOT_SEL sit on GPIO38/39,** outside GPIO1–14, which can glitch low while the S3 powers up.

The **probe header J301** (2 × 10, 2.54 mm) sits on the PX100 side of the resistors. It is for a logic analyzer or a
scope. It runs along the top edge above the shield: odd pins in the lower row, even pins in the upper row, each named
on the silk beside it (N8/N14/N29/N36 are the reserved "NC" pins).

| PX100 pin | Name | Net | ESP32 | ADC | J301 |
|---|---|---|---|---|---|
| 8 | NC (reserved) | PX_NC8 | GPIO11 | ADC2_CH0 | 1 |
| 9 | ANT_BIAS | ANT_BIAS | GPIO15 (÷2) | ADC2_CH4 | — |
| 12 | GPIO0 | PX_GPIO0 | GPIO1 | ADC1_CH0 | 3 |
| 13 | GPIO6 | PX_GPIO6 | GPIO4 | ADC1_CH3 | 5 |
| 14 | NC (reserved) | PX_NC14 | GPIO12 | ADC2_CH1 | 7 |
| 15 | GPIO9 | PX_GPIO9 | GPIO5 | ADC1_CH4 | 9 |
| 17 | RSTN | PX100_RSTN | GPIO38 | — | 11 |
| 18 | RXD1 | GNSS_RX | GPIO17 (UART TX) | ADC2_CH6 | 13 |
| 19 | TXD1 | GNSS_TX | GPIO18 (UART RX) | ADC2_CH7 | 15 |
| 24 | GPIO11 | PX_GPIO11 | GPIO7 | ADC1_CH6 | 17 |
| 25 | GPIO10 | PX_GPIO10 | GPIO6 | ADC1_CH5 | 2 |
| 26 | GPIO13 | PX_GPIO13 | GPIO8 | ADC1_CH7 | 4 |
| 27 | GPIO14 | PX_GPIO14 | GPIO9 | ADC1_CH8 | 6 |
| 28 | GPIO15 | PX_GPIO15 | GPIO10 | ADC1_CH9 | 8 |
| 29 | NC (reserved) | PX_NC29 | GPIO13 | ADC2_CH2 | 10 |
| 30 | BOOT_SEL | PX100_BOOT_SEL | GPIO39 | — | 12 |
| 31 | 1PPS | PPS | GPIO21 | — | 14 |
| 32 | GPIO2 | PX_GPIO2 | GPIO2 | ADC1_CH1 | 16 |
| 36 | NC (reserved) | PX_NC36 | GPIO14 | ADC2_CH3 | 18, through R330 |
| — | VANT (antenna feed) | VANT | GPIO16 (÷2) | ADC2_CH5 | — |

J301 pins 19 and 20 are GND.
- **Reading the pins:** the ESP32's internal pull-ups and pull-downs (about 45 kΩ) still work through 1 kΩ. Read
  each pin floating, pulled up and pulled down. A pin that follows the pull is an input or open; a pin that holds
  its level is driven. The ADC reads its voltage too.
- **The four "NC" pins** are wired on purpose: one of them may be the Phoenix's AHB-UART debug link.
  - **Pin 14:** SkyTraq's reference puts 2.2 µF on it (their C21), so this board does too (C305, at the pin).
    The ESP32 reads its DC level.
  - **Pin 36** sits between VDD_ANA2 and LNA_IN2, so R330 (0 Ω, not fitted) keeps the pin open until you fit it.
  - **Symbol:** U1 uses the new `Custom:PX100-5C-01A_AllPins`, the `_Numbered` symbol with those four pins typed
    passive so they can be wired.
- **GPIO6:** one SkyTraq image's name, "…GPIO6LNA…", suggests the firmware can use PX100 GPIO6 as an external-LNA
  enable.

GNSS_EN (GPIO40) switches the PX100's regulator, so the ESP32 can power-cycle the receiver. GPIO37 reads the USB
switch. GPIO41 drives the status LED.

**J201** carries UART0 (the ESP32 console), 3V3, GND and six spare GPIOs: 33–36, 47 and 48.

## One USB-C port, switched

S101 (JS202011JCQN) drives the FSUSB63's SEL0. SEL1 is tied high, as on the Beetle:

| S101 end (silk) | SEL0 | USB goes to | CP2102N |
|---|---|---|---|
| ESP (pin 7 end) | low | port 2: the ESP32's own USB (flashing, USB serial/JTAG) | held in reset through D301: its pins are weak pull-ups only |
| PX100 (pin 5 end) | high (R103) | port 3: the CP2102N, i.e. the PX100's UART | running |

- **The switch-end labels follow the Beetle's S1 ("F" at the pin-7 end).** Check the first board with a meter.
- **The CP2102N** is wired as Silicon Labs' self-powered 3.3 V circuit: the internal regulator is unused, a
  22.1 k / 47.5 k divider senses VBUS, and RSTb has a 1 k pull-up. macOS has its driver built in (AppleUSBSLCOM), so
  it shows up as `/dev/cu.usbserial-*`.
- **TX/RX LEDs (D302/D303)** hang on GPIO.0/TXT and GPIO.1/RXT. If they stay dark, enable TXT/RXT in Silicon Labs'
  Xpress Configurator.
- **Both the CP2102N and the ESP32 drive PX100 RXD1,** each through 1 kΩ. The CP2102N is in reset whenever the ESP32
  has the USB port. When the CP2102N has it, the ESP32 firmware must release GPIO17 (make it an input); GPIO37
  tells it which side has the port. Without firmware the ESP32's pins are inputs, so a blank ESP32 never gets in the
  way.
- **DTR/RTS (Q301/Q302)** use the ESP32 dev-board circuit:

  | DTR | RTS | PX100 |
  |---|---|---|
  | high | low | RSTN pulled low (reset) |
  | low | high | BOOT_SEL pulled low |
  | equal | equal | untouched |

  - **ROM boot loader:** "DTR low/RTS high, then pulse reset" gives the PX100's ROM boot loader, BOOT_SEL low at
    reset. So does holding PX BOOT (SW302) and pressing PX RST (SW301).
  - **Terminals:** a terminal that opens or closes the port changes both lines together, so it leaves the PX100
    running.
  - **To disable it:** remove R308/R309.
  - **Caveat:** the PX1105R's cold ROM turned out to be echo-only, and nobody has yet shown a PX100 ROM reflash.
    Until that round trip is proven, the RAM-run route stays the safe one.

## Power

USB is the only input.

| Rail | Regulator | Loads | Current |
|---|---|---|---|
| +3V3 | U104 ADP7142ARDZ-3.3 (200 mA) → FL101 BLM18PG471 bead: the carrier's U2/FL2 circuit | PX100 (120 mA), active antenna (up to about 45 mA), PPS LED | about 170 mA |
| +3V3_MCU | U103 AP2112K-3.3 (600 mA): the spin-rig controller's | ESP32-S3 (radio off), CP2102N, FSUSB63, LEDs | about 90 mA |

- **Two regulators:** one ADP7142 alone would carry about 260 mA, over its 200 mA. The split also keeps the ESP32's
  supply noise off the receiver and lets the ESP32 switch the PX100 off (GNSS_EN; R107 holds it on by default).
- **Heat:** the ADP7142 dissipates about 0.3 W from 5 V, much less than on the carrier from VSYS.
- **USB input capacitance:** VBUS carries 6.8 µF, under USB's 10 µF limit.
- **The ESP32 has no antenna** (LNA_IN open). Leave Wi-Fi and BLE off.

## Parts

**From the gnss-px100-18mm-hv V1 build, on hand:**
- the PX100 and everything on the receiver sheet (SAWs, Murata matches, Epson TCXO, SC-32S, LQM18PN2R2, NSR0320,
  TPD1E0B04);
- J1 CON-SMA-EDGE-S;
- SH1/SH2 BMI-S-202-F/-C;
- U104 ADP7142ARDZ-3.3 (LCSC has none: DigiKey 505-ADP7142ARDZ-3.3-ND);
- FL101 BLM18PG471SN1D;
- the 0402 caps and 1 k / 10 k resistors they share.

Y1 pin 1 is open, so the TG2016SMN stand-in fits (px100-module's change).

**From the spin-rig controller and the Beetle (JLC stock):** ESP32-S3FN8 (C2913196), ECS-400-10-37B2 40 MHz
(C5357833), USB4110-GF-A (C5143397), SP0503BAHTG (C3040626), AP2112K-3.3TRG1, FSUSB63UMX (C895430, 175 in stock),
JS202011JCQN (C221665), TS-1187A buttons.

**New here:** CP2102N-A02-GQFN24 (C969151), MMBT3904LT1G (C81464), LQW15AN47NG00D (C22334), 22.1 k / 47.5 k 0402
(C163478 / C137974). The two pin headers are hand-fit through-hole parts.

LCSC codes for everything are in `gen/lcsc.json` (looked up 2026-10-28) and in each symbol's LCSC field.

## Layout

Placed and routed by script (see Regenerating); 4 layers: F.Cu parts, RF and short signals; In1.Cu solid GND; In2.Cu
signals and supplies; B.Cu signals. GND is poured on all four, stitched with vias.

- **Floorplan:** J1 on the left edge, the bias tee on the RF line, then the shield (SH1) with the PX100. J301 runs
  along the top edge over the shield. The PX100's pull-ups, its buck parts and the RTC crystal sit just right of the
  shield; the 1 kΩ links sit at the ESP32's left side and under it. The CP2102N, the mux and the regulators are by the
  USB-C on the right edge; the buttons and J201 along the bottom.
- **The receiver is the carrier's:** the Wilkinson split, both SAW arms with their matches, and U1's VDD_RF, TCXO and
  LDO parts are copied part for part, with their tracks, from the routed gnss-px100-18mm-hv, at the same offsets from
  U1. The carrier left U1's GPIOs unconnected, so U1's top, right and bottom-right are new:
  - pins 24–30 run straight up through the shield's top gap as a 0.27 mm-pitch bundle to J301;
  - pins 12–19 go to two staggered rows of vias and leave on In2/B.Cu under the shield's wall;
  - PPS, GPIO2, NC8, ANT_BIAS and pin 36 (through R330, not fitted) drop to vias;
  - RTC_XIN/RTC_XOUT leave through the lower-right gap to Y2, as on the carrier.
- **The PX100's buck parts moved outside the can.** The new escapes leave no room inside for C38, L16 and D7. They
  sit just outside the right wall, with DCDC_IN, DCDC_OUT and V_CORE through the upper-right gap (3–6 mm of track).
  C39 (V_CORE) and C40/C41 (VDD11) stay at the pins. The switching node is now outside the shield, away from the RF.
- **RF line:** RF_IN is 0.34 mm with a 0.20 mm gap from J1, past D6 and L1, into the shield's upper-left gap, then
  down to C26: 50 Ω coplanar over In1 on JLC's JLC04161H-7628 (7628 prepreg 0.21 mm, εr 4.4). Confirm the width in
  JLC's impedance calculator. The copied split keeps the carrier's 0.16 mm lines, a few mm long. GND vias fence both
  sides of the line and both sides of the shield's walls.
- **Exposed pads:** U1 (5 vias, as the carrier), the ESP32 (9), the CP2102N (5) and the ADP7142 (6) have GND vias in
  their exposed pads; every via is filled and copper capped (fab note 3), so they are flat.
- **USB D+/D−** run as 0.2 mm pairs, not impedance controlled: full-speed USB, 12 Mb/s, over a few cm.
- **Pours:** SMD pads join the GND pours solid (the carrier's setting); through-hole pads get reliefs, except J301's
  and J201's GND pins, which are solid.
- **Hand fixes after `finalize.sh`** (on this routing; a fresh `route.sh` run routes differently): USB_SEL rerouted
  through the middle of S101's pad gap (it grazed an unused pad's mask opening); R332 turned so its GND pad faces C307
  (now in `layout.py`); one zero-length track and one one-layer via removed; the four long lower-row J301 names
  shortened (RST, RXD, TXD, N14: N marks the reserved "NC" pins).
- **Not checked yet:** the RF width in JLC's calculator; the copied receiver against the carrier's V2 changes once
  they are final; the board in KiCad by eye.

## Regenerating

`gen/design.py` is the parts list for the three generated sheets. The scripts:
- `gen/make_sch.py` draws them and the root, and never touches `receiver.kicad_sch`;
- `gen/make_pro.py` writes the project (net classes: Default 0.15/0.10, RF 0.34/0.20, Power 0.30/0.15, USB
  0.20/0.15);
- `gen/make_pcb.py` builds the unplaced board from the exported netlist (run it with KiCad's Python); copy it to
  `gen/out/generated.kicad_pcb`;
- `gen/route.sh` places it (`gen/layout.py`: the copied receiver, every hand-placed part, the hand-laid tracks and
  vias, the silk texts; the small parts go next to a pad they connect to), then autoroutes it with Freerouting
  (`gen/route.py prep` / `finish`: router keep-outs, In1 as the GND plane, GND left to the pours). It needs
  `FREEROUTING_JAR` pointing at a Freerouting 2.x jar and Java (Homebrew's openjdk);
- `gen/finalize.sh` stitches GND (`gen/stitch.py`), closes what the autorouter left with a grid router
  (`gen/patch_route.py`), then runs DRC with schematic parity (`gen/check.py`) and the repo's parity gate;
- `gen/lcsc_lookup.py` fills `gen/lcsc.json`.

KiCad's Python is `/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3`.
`gen/edit_receiver.py` is the one-time conversion of px100-module's receiver sheet. **Don't rerun** it,
`make_pcb.py` or `route.sh` once the sheet or board has been edited by hand: they overwrite.
