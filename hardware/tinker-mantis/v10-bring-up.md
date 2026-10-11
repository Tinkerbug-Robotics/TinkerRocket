# Tinker-Mantis V10 — first-article bring-up

The checklist for the first V10 board (tag `tinker-mantis-v10.0.0`, ordered), written before
one arrives (#1593). The firmware is
`-DTR_BOARD_V10=1` for both processors (`flight_computer/main/board/board_v10.h`,
`out_computer/main/board/board_v10.h`). Record results in #1593, and correct this file and
the board headers wherever the board disagrees with them.

**No squibs, e-matches or motors on the board until step 5 is signed off.** Use LEDs with a
series resistor, or 1–2 Ω power resistors, as dummy loads on J2.

## What is new on V10, and why it matters here

- **The arm needs both processors.** `FC_ARM` (P4 GPIO33) → R132 → Q12, and `OC_ARM_EN`
  (S3 GPIO14) → Q14, in series. Together they pull Q13, which drives U9's gate from
  `VBAT_CON`. With U9 open the squib return reaches ground only through R73 (2.2 k), so an
  LED on a channel lights without the channel firing. That is the design working, not a
  dead igniter.
- **A V9 image cannot arm a V10.** It drives GPIO16 (unconnected) and has no consent pin.
  Check the boot log before anything else.
- **New out-computer inputs:** `PG_RAIL` (GPIO17), `INA_ALERT` (GPIO18), `VBUCK_OK` (GPIO34).
- **New parts behind unchanged pins:** BMP581 (not BMP585), QMC5883P magnetometer (not
  IIS2MDC), and WSD20L50DN33 P-FETs as the pyro switches. The TPS1HC08-Q1 switches of
  #1554 are V11's.

## 1. Before power

- [ ] U6/U7/U8/U10 are WSD20L50DN33 P-FETs, as ordered. The #1553 trip stays at 30 A for
      them (#1595).
- [ ] Ohm out the arm chain against the schematic: R132 100 Ω, R139 1 k, R21 100 Ω, R22
      100 k (U9 gate to GND), R73 2.2 k (`PYRO_GND` to GND), R78–R81 5.11 k (each `PYROn_FIRE`
      to GND).
- [ ] `PYRO_GND` (J2.5) to GND reads about 2.2 k (R73), not a short.

## 2. Rails, both processors held in reset or unflashed

- [ ] `VBAT_CON`, `VBATT` (after U19), `V_BUCK` about 3.47 V (TPS62152 with DEF high), `+3V3`.
- [ ] `V_MCU_SWTCH` is OFF until the out computer raises `P4_EN_S3`.
- [ ] `V_SCAP` charges to its termination (the V10 has no firmware monitor for it yet,
      #1409, so measure it).
- [ ] `PG_RAIL` high (R59), `INA_ALERT` high (R142), `VBUCK_OK` about 0.91 × `V_BUCK` ≈ 3.15 V.
- [ ] Read both boot NOR parts (`esptool flash-id`) and fill in the V10 row of
      `docs/board-versioning.md` (it says "unmeasured").

## 3. Flash and boot log

Flash over USB: `idf.py -B build_v10 -DTR_BOARD_V10=1 flash` in each project. Releases ship
no V10 image until this checklist passes.

- [ ] Flight computer:
  - `[BOARD] pin map: V10`
  - `[BOARD] pyro: ARM=33 FIRE=6/11/9/13 CONT=7/10/12/14`
  - `QMC5883P found and initialized`, and `QMC5883P CTRL1=… CTRL2=… (RNG=2)`. RNG must read 2
    (±8 G); #1590 saw a scale that looked like ±30 G on the Beetle.
  - The BMP581 and ISM6HG256 come up (`found and initialized`).
- [ ] Out computer:
  - `[BOARD] pin map: V10`
  - `arm consent (OC_ARM_EN, GPIO14) driven LOW at boot`
  - `[SUPPLY] PG_RAIL (GPIO17) HIGH at boot` and `[SUPPLY] VBUCK_OK (GPIO34) HIGH at boot`
  - The INA230 answers, with no `INA_ALERT (GPIO18) ISR not installed` line.
- [ ] Provision the board as V10 (the out computer stores it):
      `python3 tools/bench_ble_cmd.py --name <unit> --cmd 74 --payload 00563130`

## 4. Scope check: the bring-up requirement

With no loads on J2, probe all of these at once if the channels allow, or in groups that
always include `FC_ARM`, `OC_ARM_EN` and U9's gate:

| Signal | Where |
|---|---|
| `PYRO1_FIRE` … `PYRO4_FIRE` | P4 GPIO6/11/9/13, or across R78/R79/R80/R81 |
| `PYRO1_EXT` … `PYRO4_EXT` | J2.1–J2.4 |
| `FC_ARM` | either end of R132 |
| `OC_ARM_EN` | Q14 base (S3 GPIO14) |
| U9 gate | the R21/R22 node |
| `PYRO_GND` | J2.5 |

Designators are not in channel order: channel 1 is Q5/U8, 2 is Q3/U6, 3 is Q4/U7, 4 is Q6/U10.
Probe by net, not by counting designators.

Trigger on a rising edge of each, and confirm every one stays low through:

- [ ] cold power-on (pack connected), then the out computer raising `P4_EN_S3`;
- [ ] an out-computer reset (button or `esptool`);
- [ ] a flight-computer reset and its whole boot, through `initPyroPins()`;
- [ ] a flight-computer reboot after an OTA;
- [ ] a power-off from the app, and the rail dropping.

Datasheet basis to confirm, not assume: P4 GPIO33 is input-enabled with no pull at reset and
is not a strap; S3 GPIO14 has no pull at reset; Q12 and Q14 each have a built-in 47 k
base-emitter resistor; R78–R81 hold the FIRE nets down. FC_ARM was never part of an earlier
scope check: on V9 that pad was EXP_12.

## 5. Arm chain, with dummy loads

- [ ] Continuity: each channel reads continuous with a load on its J2 pin and open without.
      Swap the load around all four.
- [ ] Fire test from the app on each channel in turn:
  - `OC_ARM_EN` rises first and stays up about 2 s;
  - `FC_ARM` pulses about 210 ms (10 ms settle plus the 200 ms pulse);
  - the right channel's FIRE pin pulses 200 ms, inside the arm window;
  - U9's gate rises to about `VBAT_CON`, and the load on THAT channel's J2 pin conducts.
- [ ] Between tests, `PYRO_GND` to GND is back to R73 (U9 open).
- [ ] Each fire test logs `fault-current trip ARMED: … (… INA_ALERT edge + 1 ms shunt poll)`
      and then `stood down`, with no `FAULT-CURRENT TRIP` line for a normal dummy-load fire.
      V10 is the first board to run the INA_ALERT interrupt path (#1553): the Beetle leaves
      ALERT unconnected. Check that `INA_ALERT` stays high through a normal pulse on the
      scope.
- [ ] Simulated flight: ARM and FIRE stay low throughout (the flight computer dry-fires a
      sim), and `OC_ARM_EN` stays low.
- [ ] Before the first live charge, check the #1553 trip with a controlled short on one
      channel. The #1554/#1588 short tests are for V11's switches.

## 6. Supply status inputs

- [ ] Drop the bench supply below the eFuse UVLO (6.34 V falling): `[SUPPLY] … PG_RAIL fell`
      on the console. It rises again on recovery.
- [ ] Remove the pack with the board running: `VBUCK_OK fell — buck gone: running on the
      hold-up cap`. Note how long the board then stays up.
- [ ] The `[SUPPLY] PG_RAIL=… VBUCK_OK=… falls=…` stats line matches what happened.

## 7. Sensors and frames

- [ ] Magnetometer: lying flat at rest, the field dips about 65° DOWN in the board frame
      (#1590). The 180° rotation is inferred, not measured.
- [ ] |B| near the local field (about 50 µT), not about 13 µT (the suspected scale error).
- [ ] IMU at 7,680 Hz: the `[IMU FIFO]` and `[GAP DIAG]` counters show no drops (#1485).
- [ ] Barometer: plausible pressure, and `BMP585_INT` firing.

## 8. OTA

- [ ] A `-v10` image OTAs to both processors.
- [ ] A `-v9` image is refused with `image_identity_mismatch` and nothing is flashed.

## 9. Close out

- [ ] Results on #1593; board headers and this file corrected where the board disagreed.
- [ ] The V10 row of `docs/board-versioning.md` measured.
- [ ] Only then: add the V10 pair to `firmware-release.yml` (the matrix, the image count and
      the notes table).
