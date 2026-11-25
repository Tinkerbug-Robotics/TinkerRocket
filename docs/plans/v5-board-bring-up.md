# TinkerRocket Full V5 — bring-up on current firmware

The checklist for running today's firmware on the archived V5 board. The firmware is
`-DTR_BOARD_V5=1` for both processors (`flight_computer/main/board/legacy/board_v5.h`,
`out_computer/main/board/legacy/board_v5.h`). The board files live only in the archive
(`Circuit Board Designs ARCHIVE 2026-07-24/TinkerRocket Full V5/`); the pin maps come from a
netlist export of that schematic, whose net names match the fabbed Gerbers (2026-04-17).
Ignore the `.txt` BOM in that folder: it is older than the board and lists two pyro
channels and an MMC5983MA. Record results on the PR that added the target, and correct
this file and the board headers wherever the board disagrees with them.

**No squibs, e-matches or motors on the board until steps 3–5 are signed off.** Use LEDs
with a series resistor, or 1–2 Ω power resistors, as dummy loads on the pyro outputs.

## What is different on V5, and why it matters here

V7 is a respin of this layout: every part sits in the same place, and every MCU pad but
the ones below carries the same net.

- **An arm per channel, driven as one.** `PYRO1_ARM`…`PYRO4_ARM` are P4 GPIO14/22/37/49,
  each a DTC123J (Q2/Q1/Q5/Q6) that turns on a TPN4R712MD P-FET (U7/U24/U25/U26) feeding
  that channel's VN5E160 driver (U9/U10/U21/U22). The firmware raises all four together
  wherever V7–V10 raise their one shared arm (the owner's choice, 2026-10-18). A fire
  therefore powers all four drivers. Only the chosen channel's FIRE goes high.
- **FIRE and CONT swap places on channels 1–3** compared with V7: FIRE 15/18/38/50,
  CONT 16/19/34/51. A V7 image on a V5 fires nothing.
- **Continuity needs the arm up.** CONT is the VN5E160's STATUS output. It reads HIGH when
  a load is present and LOW when the output is open. That holds only while the driver is
  powered: unarmed, STATUS floats to its pull-up and every channel reads "connected". So
  on V5 the pad continuity reads (the PRELAUNCH edge and the app's continuity test) raise
  the arm for 10 ms with every FIRE low, read, and drop it. The charge sees about 0.2 mA
  through R35/R36/R67/R75 (50 k). A read the firmware cannot take reports continuity
  unknown. That happens during a sim, after landing, or with a channel mid-fire.
- **Channel 3 through boot (expected, not yet measured).** GPIO37 and GPIO38 are the P4's
  UART0 TXD and RXD out of reset. `PYRO3_ARM` therefore sits on a line that idles HIGH, and
  `PYRO3_FIRE` on one with a weak pull-up. The firmware takes both pads first thing in
  `setup_fc()`, but the boot ROM, the bootloader and IDF start-up all run before that.
  V7 moved these lines.
- **Camera:** `CAM_ACT` is GPIO30 and `Camera_TX` (J6.4, the GoPro shutter) is GPIO32,
  the reverse of V7.
- **No GPS rail, no servo rail switch.** The GNSS connector (J3) takes VPP directly. V7's
  out-computer GPS_ACT pad (GPIO33) is an unconnected label here.
- **LoRa:** `L_RXEN` (S3 GPIO10) has no resistor, and the firmware does not drive it. V7
  has the same net and the same firmware.

## 1. Before power

- [ ] All four channels are populated: U9/U10/U21/U22 (VN5E160), U7/U24/U25/U26
      (TPN4R712MD), Q2/Q1/Q5/Q6 (DTC123J).
- [ ] The magnetometer is U23, an IIS2MDC (not an MMC5983MA).
- [ ] Ohm out each channel's FIRE pull-down (R29/R32/R65/R73, 10 k to GND) and STATUS
      pull-up (R33/R34/R66/R74, 10 k to 3V3).
- [ ] Read both boot NOR parts (`esptool flash-id`). Expect W25Q64 (8 MB) on both. The
      images declare 8 MB.

## 2. Flash and boot log

Flash over USB: `idf.py -B build_v5 -DTR_BOARD_V5=1 flash` in each project. The USB switch
(S2) picks which processor is on the connector. CI builds both images; no release ships
them.

- [ ] Flight computer:
  - `[BOARD] pin map: V5`
  - `[BOARD] pyro: ARM=14 FIRE=15/18/38/50 CONT=16/19/34/51`
  - `[BOARD] pyro: further ARM lines 22/37/49, driven with ARM; CONT reads with the arm up, HIGH = load`
  - `[PYRO] Pins initialized (safe, 4 channels, 4 ARM lines driven as one)`
  - `IIS2MDC found and initialized`, plus the BMP585 and the ISM6HG256.
- [ ] Out computer: `[BOARD] pin map: V5`, the INA230 answers, NAND and MRAM mount.
- [ ] Provision the board as V5 (the out computer stores it):
      `python3 tools/bench_ble_cmd.py --name <unit> --cmd 74 --payload 005635`

## 3. Scope check: every arm and fire line across reset and boot

With no loads, probe through power-on, a reset (EN), a USB flash and a normal boot, up to
the `[PYRO] Pins initialized` line:

| Signal | Where |
|---|---|
| `PYRO1_ARM` … `PYRO4_ARM` | Q2/Q1/Q5/Q6 base (P4 GPIO14/22/37/49) |
| `PYRO1_FIRE` … `PYRO4_FIRE` | U9/U10/U21/U22 pin 2 (INPUT) |
| Channel 3 driver supply | U21 pin 5/8 (VCC, U25 drain) |
| Channel 3 output | U21 pin 6/7 |

- [ ] Channels 1, 2 and 4: ARM and FIRE stay low throughout.
- [ ] **Channel 3:** record what `PYRO3_ARM` (GPIO37) and `PYRO3_FIRE` (GPIO38) do before
      the pins are taken. If ARM3 goes high during boot, the channel 3 driver is powered
      through every boot. If FIRE3 then sits anywhere near the VN5E160's 0.9 V input-low
      limit, do not put a charge on channel 3. Fly channels 1, 2 and 4, unless the board
      is reworked.

## 4. Continuity, with dummy loads

Put a load on two channels and leave two open.

- [ ] App continuity test on each channel: loaded channels read connected, open ones read
      open. On the scope, all four ARM lines rise for about 10 ms and every FIRE stays low.
- [ ] READY → PRELAUNCH: the boot log's `[PYRO] Prelaunch CONT:` line shows `raw=1` for the
      loaded channels and `raw=0` for the open ones.
- [ ] Optional: with the arm down, every CONT pin reads HIGH whatever is connected. This is
      why the reads raise the arm.

## 5. Fire test, with dummy loads

- [ ] App fire test, one channel at a time: all four ARM lines rise, then only that
      channel's FIRE pulses for 200 ms, and only its load conducts.
- [ ] After landing (post-flight lockout), a continuity test logs "not read" and the
      channel shows continuity unknown. The arm stays down.

## 6. Everything else

- [ ] Magnetometer frame: `MAG_ROT_Z_DEG_IIS2MDC` = 90 is inferred from the placement (U23
      at 0°, U5 at 45°, as on V9), not measured. Check that heading follows the board
      after a calibration.
- [ ] Camera: CAM_ACT powers it. A RunCam answers on Camera_RX/Camera_TX. A GoPro toggles
      on J6.4.
- [ ] GNSS on J3 gets a fix (the driver detects an RX/TX swap).
- [ ] LoRa: telemetry arrives, **and** an uplink command (for example, mute) is answered.
      With RXEN undriven, receive is the half that could be weak.
