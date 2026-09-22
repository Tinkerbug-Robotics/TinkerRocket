# rocket-computer V10 — schematic review and parity with the mini, 2026-09-21

Review of the `rocket-computer` V10 schematic **as it stands in the owner's working copy**,
against the `rocket-computer-mini` and against the fabricated, working V9. The question
this review was asked first was whether the V10 has pulled in everything the mini learned.

## Read this first: the files reviewed are not the files in git

The owner's working copy carries uncommitted edits on every schematic sheet and on the
board. This review is of those live files. The netlist differs from git HEAD by:

| change | what |
|---|---|
| removed | C56 330 uF bulk, CR1 Schottky, L1 4.3 nH, LS1 buzzer, Q9, R26, R28, U14 Molex antenna |
| added | U22 chip antenna with L2 2.2 nH and C23 5.1 pF, C45 and C88 22 uF, C12 do-not-populate |
| re-pinned | expansion header pins 9 to 14; the processor pin that was the magnetometer interrupt now leaves the board |
| orphaned | the out computer's GPIO13, which used to drive the buzzer |

Component count goes 277 to 275, nets 279 to 274. The board has been updated from the
schematic, so it reports no parity items, but **routing is in progress** with 124
unconnected items. Routing was deliberately not reviewed. Every layout observation belongs
on #1365.

## What this review is, and what it is not

Six reviewers were assigned to this board; three finished before the run reached an
account spend limit. The power tree, the electrical-rule and bill-of-materials sweep, and
the placement pass **did not run**. The adversarial verification pass did not run either,
so the findings below are single-source except the five I re-derived myself, which are
marked *Verified here*.

**What is missing from this review, and should be run before the board is committed to
fab:** the V10-only power blocks pin by pin against their datasheets (the electronic fuse,
the processor core converter, the camera and servo rails, the satellite carrier switch),
the documentation sweep against the live files (the fabrication notes still describe a
six-layer board; the live board is eight), and the firmware pin-map delta.

## Parity with the mini

The answer to the question asked: **the V10 has pulled in essentially all of the mini's
circuit work.** Every item from the hold-up converter rework, the pack-direct firing
change, arm rework 4, the part-parity swaps and the September power passes is present in
the live netlist, verified block by block. Four things did not come across, all small, and
they are the findings in the next section.

The full block-by-block table the reviewer built, with the evidence for each row:


PARITY TABLE (block | mini | V10 | status | evidence)
1. USB-C J6 + CC + ESD | J6 USB4110GFA; CC1(A5)→R41 5.11k→GND, CC2(B5)→R47 5.11k→GND; VBUS A4/B9+B4/A9 → C76 1µF, CR3 SP0503 pin 2, R51, R62, U21 IN1; D+/D− both rows → CR3 pins 3/4 + U1 pins 1/2; SBU1/2 NC-flagged; S1–S4 shields + A1/B12 + B1/A12 GND | identical part, pins, values, nets | IDENTICAL (also identical to V9) | nets Net-(J6-CC1), Net-(J6-CC2), Net-(J6-VBUS), D+, D−, GND in both netlists.
2. TPS2121 U21 | IN1 VBUS, IN2 VBATT, OUT 1/8 V_MCU_2S; OV1 R51 49.9k/R52 10k (VREF 1.06 V → VBUS OV 6.35 V); OV2 R56 12k/R57 1.5k off VBATT (→ 9.54 V); PR1 R62/R63 5.11k/5.11k (IN1 priority once VBUS > 2.12 V); CP2 R60 15.4k/R61 5.11k off VBATT; ILIM R64 49.9k; SS C52 1 µF; ST 9 GND; GND 12 | identical | IDENTICAL thresholds and pins (V9 identical) | tps2121.txt line 363 VREF 1.01–1.10 V; both netlists' Net-(U21-*). OUTPUT BULK DIFFERS: mini C47/C48/C49 3×22 µF + C59 1 µF on V_MCU_2S; live V10 C59 1 µF only (C56 330 µF deleted live, C45/C88 added on the buck side of L5) → finding v10-parity-3.
3. L5 + TPS62152 U18 | L5 VLS3012 2.2 µH → Net-(U18-AVIN): AVIN 10, PVIN 11/12, EN 13 tied; C43 22 µF + C65 100 nF (V10 also C45, C88 22 µF); SW 1/2/3 → L6 VLS3012 2.2 µH → V_BUCK; VOS 14 V_BUCK; FB 5 GND (fixed 3.3 V part); AGND 6, PGND 15/16, EP 17 GND; FSW 7 = V_BUCK (=VOUT: starts low as DS §8.3.5 mandates, then 1.25 MHz); DEF 8 = V_BUCK (DS §8.3.4: +5 % → 3.465 V); PG 4 → R49 100k → V_BUCK, unread; SS/TR 9 → C44 | identical except C44 (mini 390 pF, V10 10 nF) and V_BUCK bank (mini C135 22 + C141 10 + C53 10 = 42 µF; V10 C141 22 + C53 10 = 32 µF, DS recommends 22 µF, both fine) | PORTED WITH DOCUMENTED DIFFERENCES (C44 per §6.4; V_BUCK bank per §1.1/§1.2; V10 R140 10k bleed and R65 LED on V_BUCK per decisions) | tps62150.txt lines 318–320, 603–618, 901; nets V_BUCK, Net-(U18-*) both boards. C44 → mini finding v10-parity-8.
4. TPS61094 U47 hold-up | OSEL 1 → R134 3.09k→GND; MODE 2, EN 3, VIN 4 = V_BUCK; SW 5 → L11 XGL4020 2.2 µH → V_SCAP; SUP 6 = V_SCAP (C130 HV1020 5 F + C142 10 µF); PGND 7, AGND 8, EP 13 GND; VOUT 9/10 = +3V3 with C17 22 µF + second 22 µF (mini C143 / V10 C144); ICHG 11 → R136 22.1k→GND; VCHG 12 → R135 6.65k→GND; VIN cap C141 (mini 10 µF, V10 22 µF; DS CIN ≥ 2.2 µF eff, COUT 20–30 µF eff, CSUP ≥ 2.2 µF eff, tps61094.txt lines 221–223) | identical | IDENTICAL (part, every pin, every value) | nets OSEL_SET, VCHG_SET, ICHG_SET, SCAP_SW, V_SCAP, +3V3.
4a. VBUCK_OK divider | R137 100k / R138 360k + C152 100 nF → FC U32 GPIO3 (ADC1_CH2); OC off the net (#1000) | R137 100k / R138 1M → OC U15 GPIO34 logic read; R140 10k bleed across V_BUCK; no FC reader | PORTED WITH DELIBERATE DIFFERENCE (decisions.md 'VBUCK_OK'; §1.2; #1409 owner comment) | nets VBUCK_OK, V_BUCK.
4b. V_SCAP sense | R125/R126 100k/100k + C144 100 nF → OC U15 GPIO8 | R125/R126 100k/100k + C143 100 nF → P4 U17 GPIO17 | PORTED WITH DELIBERATE DIFFERENCE (decisions.md; #1365 row 'supercap sense') | net V_SCAP_ADC.
5. +3V3 decoupling census | converter out C17+C143 22 µF; S3 U15 set: C29/C31/C34 100 nF, C30/C35 1 µF, C32/C36 10 µF, C28 10 nF, C26 100 nF (U13 NOR), C33 100 nF (RF, via L4), C25 1 µF (CHIP_PU), C27 1 µF (VDD_SPI); C149 100 nF (INA230, board 5.0 mm from U23); C145–C148 4×100 nF drawn with no IC context (power sheet 250–273,94) and placed on the board as distributed +3V3-plane caps (nearest U33/L3, U14, U4/U2, U9 — mini layout-only, no counterpart needed) | converter out C17+C144 22 µF; same S3 set + C145 100 nF (VDD_SPI) + C148 10 µF (RF); C16 100 nF (NAND U11, on +3V3 here), C40 100 nF (P4 NOR U16) | S3 set IDENTICAL; V10 additions are #664/#678 items → mini findings v10-parity-6/-7; INA230 bypass NOT PORTED → finding v10-parity-2 | +3V3 net members both boards; symprox.py positions; board positions from pcbnew.
5a. Pull-ups on +3V3 | R1/R2 100k (U1 SEL1/SEL0), R3 22Ω+C1 100 nF (U1 VCC), R36 10k (CHIP_PU), R59 100k (eFuse FLT), R67/R69 5.11k (INA I2C), R68 10k (D8 LED) | same R1/R2/R3/R36/R59/R67/R69 + R141 10k (GPIO0), R142 100k (INA_ALERT), R31/R33 10k (NAND WP#/HOLD#, mini's are on V_MCU_SWTCH), R46 10k (P4 NOR CS) | consistent with the rail each slave sits on | +3V3 net members.
6. eFuse U19 | TPS259631DDAR (auto-retry variant, tps2596.txt line 138): IN 4 VBAT_CON, OUT 5 VBATT, GND 1/PAD 9; EN/UVLO 3 = C94 10 µF + string VBAT_CON–R44 100k–[EN]–R45 7.15k–[OVLO 8]–R85 13.7k–GND (all RT0402 0.1 %) → EN = 0.1725·VBAT: on 6.96 V / off 6.38 V (VUVLO 1.2/1.1 V, lines 371–373); OVLO = 0.1134·VBAT → 10.6 V (VOVLO 1.2 V); ILM 7 → R48 1k → ILIM ≈ 0.9 A (eq. 5 RILM = 903/ILIM^1.0112, table 909 Ω → 1.005 A); dVdt 2 → C51 100 nF → SR = 42000/100000 = 0.42 V/ms (eq. 4); FLT 6 → R59 100k → +3V3, unread; CIN C41 1 µF + C42 100 nF; COUT C54 1 µF + C8 22 µF | TPS259824LNRGET: IN 1/2/3/16/25 VBAT_CON, OUT 17–24 VBATT, GND 4/5/14/26; EN/UVLO 6 = C94 10 µF + R44 1M / R45 210k (1 %) → 0.1736·VBAT: on 6.91 V / off 6.34 V (VUVLO 1.2/1.1 V, tps25982.txt 415–417); OVP fixed 16.7 V variant (line 965); ILIM 8 → R48 127 Ω → ~11.6 A (table 100 Ω → 14.71 A, 182 Ω → 8.13 A); DVDT 15 → C51 10 nF → SR = 4600/10000 = 0.46 V/ms (eq. 3) — same ramp class as the mini; ITIMER 7 → C50 10 nF → 4.7 ms blanking (0.47 ms/nF, eq. 30); RETRY_DLY 10 → C46 2.2 nF (~92–100 ms, eq. 31); NRETRY 11 GND = retry indefinitely (pin table line 230); LDSTRT 12 GND (line 237 'connect to GND if not used'); IMON 9 GND; PG 13 → R59 100k → +3V3 + S3 GPIO17 (PG_RAIL); same CIN/COUT | PORTED WITH DELIBERATE DIFFERENCE (decisions.md: TPS259824 stays; NRETRY/PG_RAIL wiring). UVLO thresholds match within 50 mV; deglitch identical; ramp rates equivalent; only tolerance grade differs (mini 0.1 % vs V10 1 %, ±0.1 V on 6.91 V — display-only thresholds by decision, note only) | nets Net-(U19-*) both boards.
7. INA230 U23 | A0/A1 GND (0x40); VS+ 9 +3V3; BUS 11 + IN− 12 VBAT_CON; IN+ 13 VBAT_Terminal (R72 2 mΩ PA0805 between); SCL/SDA → OC GPIO33/21 (SEN_SCL/SDA) with R67/R69 5.11k → +3V3; ALERT NC-flagged; NC pins flagged | identical incl. R72; nets PWR_SCL/PWR_SDA; ALERT → S3 GPIO18 + R142 100k (INA_ALERT) | IDENTICAL except ALERT (deliberate, #721; owner in #1409: 'The mini has neither #721 wire … and does not need them') | Kelvin entry is layout (#1365 line 52), not checked here. Bypass → finding v10-parity-2.
8. FC rail switch U30 TPS22810 + D9 | VIN 6 +3V3; VOUT 1 + QOD 2 = V_MCU_SWTCH; CT 3 → C104 10 nF; GND 4/7; EN/UVLO 5 = POWER_SWITCH = C105 10 µF + R84 100k→GND + D9 K; D9 BAV170 A1 ← FC_EN_HOLD (U32 GPIO17), A2 ← FC_EN_OC (U15 GPIO7) | identical; A1 ← P4_EN_HOLD (U17 GPIO5), A2 ← P4_EN_S3 (U15 GPIO7) | IDENTICAL (V9 identical) | nets POWER_SWITCH, Net-(U30-CT), V_MCU_SWTCH.
9. FSUSB63 U1 + S1 + R1/R2/R3/C1 | D+/D− 1/2 from J6; VCC 12 via R3 22 Ω from +3V3 + C1 100 nF; SEL1 11 → R1 100k → +3V3; SEL0 4 → R2 100k → +3V3 + S1.6, S1.7 GND, S1.1/2/3/5 NC-flagged; HSD1 5/6 NC-flagged; HSD3 9/10 → R39/R38 22 Ω → OC U15 GPIO19/20; HSD2 7/8 → R113/R112 22 Ω → FC U32 GPIO19/20 | identical through HSD3 (OUT_D±); HSD2 → P4 GPIO24/25 direct, R77 1M pull-down on CEN_D+ | IDENTICAL on the OC side; FC side differs by MCU; R77 asymmetry accepted (decisions.md 'No 1 M bleeder on FC_D+') | nets D+, D−, Net-(U1-VCC), Net-(U1-SEL1), SEL0, OC_D±/OUT_D±, FC_D±/CEN_D±.
10. S3 OC U15 block | CHIP_PU C25 1 µF + R36 10k↑; GPIO0 SW3 only; GPIO46 R37 10k↓; GPIO45 unflagged spare; GPIO3 = ESP_I2S_SD with R34 100k↓; Y2 ECS-400 40 MHz: XTAL_N C22 12 pF, XTAL_P via L3 24 nH with C20 12 pF; Y1 ABS07 32.768k with C19/C21 22 pF; RF: LNA_IN → L1 2.2 nH shunt, C7 5.1 pF series, C12 DNP shunt, U14 AANI-CH-0070 FEED 1/4, GND 2/3; VDD3P3 2/3 via L4 2 nH + C33 100 nF; VDD3P3_RTC/CPU/VDDA1/2 on +3V3; VDD_SPI 29 C27 1 µF; USB GPIO19/20 R39/R38; UART0 43/44 unflagged spare; JTAG 44–48 unflagged spare; SPICS1/SPICLK_N/P NC-flagged; NOR U13 W25Q128JVYIQ WLCSP on OC_SPI_* + VCC +3V3, no CS pull-up (SPICS0 WPU at reset, esp32-s3 DS Table 2-4 line 822); GPIO13 L_CS, GPIO8 V_SCAP_ADC, GPIO10 L_RXEN, GPIO11 OC_ARM_EN, GPIO14 L_BUSY, GPIO17 L_DI01, GPIO18 L_RST, GPIO35–38 M_* bus, GPIO21/33 INA I2C, GPIO1–4 I2S, GPIO5/6 ESP_SDA/SCL, GPIO7 FC_EN_OC, GPIO9/12/34/39–42 spare | CHIP_PU identical; GPIO0 + R141 10k↑; GPIO46 R37; GPIO45 NC-flagged; GPIO3 no pull; Y2/L3/C20/C22 identical; Y1/C19/C21 identical; RF identical topology & values (L2 2.2 nH, C23 5.1 pF, C12 DNP, U22 AANI-CH-0070 — the live V10 ported the mini's match, replacing HEAD's Molex U14 + L1 4.3 nH); VDD3P3 via L4 + C33 + C148 10 µF; VDD_SPI C27 + C145 100 nF; USB identical; UART0/JTAG NC-flagged; NOR U13 identical nets/balls/no pull-up; GPIO8 CAM_IMON, GPIO9 SERVO_IMON, GPIO10/11 LoRa UART, GPIO12 LoRa_ACT, GPIO13 open (one ERC error), GPIO14 OC_ARM_EN, GPIO17 PG_RAIL, GPIO18 INA_ALERT, GPIO34 VBUCK_OK, GPIO35–38 NAND, GPIO39–42 unused (no pads on this symbol variant), I2S/I2C/EN identical pins | IDENTICAL for CHIP_PU, straps 46/45, crystals, RF match, USB, NOR, inter-MCU pins. R34 NOT PORTED → finding v10-parity-1. R141/C148/C145 are V10-only → mini findings 5/6/7. GPIO13 open = decisions.md (#1438 question). | nets Net-(U15-*), Net-(C12-Pad1), Net-(C20-Pad1), OC_SPI_*/OUT_SPI_*.
10a. LEDs | D6 green R65 10k on V_MCU_SWTCH; D8 green R68 10k on +3V3; D7 blue R66 on U5 1PPS; D10 red R70 IND_1 (FC U32 GPIO43); D11 blue R71 IND_2 (FC GPIO45) | D6 green R65 10k on V_BUCK; D7 red R66 IND_1 (P4 GPIO27); D8 blue R70 IND_2 (P4 GPIO26); no +3V3-side or switched-rail indicator | PORTED WITH DOCUMENTED DIFFERENCE (§1.1 'power LED now shows the buck … one wire cut and one label' if wanted on +3V3); IND colours match (IND_1 red, IND_2 blue); all 10 k by decision | Net-(D*-A) both boards.
10b. Buttons | SW3 → OC GPIO0; SW2 → FC U32 GPIO0 | SW3 → OC GPIO0 (+R141); SW2 → P4 GPIO35 via R54 1k, R53 100k↑ V_MCU_SWTCH | equivalent per MCU | nets Net-(U15-GPIO0), Net-(R54-Pad1).
11. Sensors | U2 ISM6HG256: VDD/VDDIO V_MCU_SWTCH, CS → R5 10k↑V_MCU_SWTCH + FC GPIO4, INT1 → FC GPIO8, SDO/TA0, SCL, SDA on SENS_* bus, SDX/SCX GND, INT2/OCS_AUX/SDO_AUX NC-flagged; decoupling C3 10 µF + C150 100 nF (board 2.5 mm). U4 BMP581: CSB → R4 10k↑ + FC GPIO9, INT → FC MTDI, SCK/SDI/SDO shared, VDD/VDDIO V_MCU_SWTCH, VSS 3/8/9 GND; C5 10 µF + C151 100 nF (board 2.3 mm). U3 QMC5883P: SCL/SDA → MAG_SCL/MAG_SDA with R117/R118 5.11k↑V_MCU_SWTCH → FC GPIO2/1, VDD V_MCU_SWTCH, C1 → C6 10 µF, GND 9/11, NC 3–8/12–15 (no_connect-typed); C2 100 nF + C4 10 µF. Inter-MCU ESP_SCL/SDA R115/R116 5.11k↑V_MCU_SWTCH | U2 identical wiring on P4 GPIO49/50/51–53, C3 + C146 100 nF; U4 identical on P4 GPIO41/42, C5 + C147 100 nF; U3 identical with R40/R43 5.11k, P4 GPIO48/47, C6 10 µF, C2 + C4; ESP_SCL/SDA R55/R58 5.11k↑V_MCU_SWTCH | IDENTICAL (parts, pull-up values, rails, INT/CS, C6, decoupling incl. the 100 nF pair) | nets ISM6HG256_*, BMP585_*, MAG_*, SENS_*, Net-(U3-C1), ESP_SCL/SDA; pcbnew positions for C150/C151 vs C146/C147.
12. Pyro channels ×4 | FIRE: MCU → Qn DTC123JETL base with R78–R81 5.11k↓ on the FIRE net; Qn E GND; Qn C → R15/R16/R17/R23 10k↑VBAT_CON + Un WSD20L50DN33 gate; Un S 1/2/3 VBAT_CON, D 5–8 PYROn_EXT → J2.n; continuity R8/R10/R12/R18 49.9k V_MCU_SWTCH→EXT, R9/R11/R13/R19 100k EXT→CONT, D1–D4 BAT54 A=CONT K=V_MCU_SWTCH, CONT → FC GPIO10/11/12/MTMS; PYRO_GND = J2.5 + R73 2.2k↓ + U9 AON7534 D 5–8; U9 gate → R21 100 from ARM_GATE + R22 100k↓; bulk C9/C10/C11/C13 22 µF on VBAT_CON; J2 JL212R-SMT-35005BP1 | identical refs/values/nets; FIRE on P4 GPIO6/11/9/13, CONT on P4 GPIO7/10/12/14 | IDENTICAL | nets PYROn_FIRE/EXT/CONT, PYRO_GND, Net-(Qn-C), Net-(U9-GATE).
13. Arm chain | FC_ARM (U32 GPIO44) → R132 100 → Q12 B; Q12 E GND, C → Q14 E; Q14 B ← OC_ARM_EN (U15 GPIO11); Q14 C → Q13 B; Q13 DTA123J E = VBAT_CON, C → R139 1k → ARM_GATE → R21 | identical (FC_ARM = P4 GPIO33, OC_ARM_EN = U15 GPIO14; R139 pins swapped live, no electrical change) | IDENTICAL (rework 4 per decisions.md) | nets Net-(Q12-B/C), Net-(Q13-B/C), ARM_GATE.
14. NAND U11 | GD5F1GQ5UEYIGR on V_MCU_SWTCH; CS# → R140 100k↑V_MCU_SWTCH + OC GPIO36; WP#/HOLD# → R33/R31 10k↑V_MCU_SWTCH; SO/SI/SCLK on M_* shared with U16 LoRa; GND 4 + pad 9 | GD5F2GQ5UEYIGR on +3V3; CS# → OC GPIO36 only; WP#/HOLD# → R33/R31 10k↑+3V3; bus alone; C16 100 nF | PORTED WITH DELIBERATE DIFFERENCES (1G vs 2G, rail: decisions.md; CS pull-up: §2 'low, not drawn' → note finding v10-parity-4) | nets M_*, Net-(U11-*).
15. Boot NORs | OC U13 W25Q128JVYIQ: CLK/CS/DI/DO/WP/HOLD on S3 SPI pads, VCC +3V3, 16 NC balls no_connect-typed, no CS pull-up; FC U33 same on U32's SPI pads, VCC V_MCU_SWTCH | OC U13 identical; P4 U16 W25Q128JVYIQ on P4 FLASH_* pads, VCC +3V3 (V9 heritage, always-on while the P4 is switched), R46 10k CS↑+3V3, C40 100 nF | OC IDENTICAL; FC/P4 NOR differs by MCU and rail (V9-fabbed arrangement, unchanged) | nets OC_SPI_*/OUT_SPI_*, FC_SPI_*, FLASH_*.
16. 32 kHz crystals | Y1 ABS07 + C19/C21 22 pF (OC); Y3 ABS07 + C110/C111 22 pF (FC U32) | Y1 identical; Y3 ABS07 + C37/C38 22 pF on P4 GPIO0/1 | IDENTICAL parts and load caps (decisions.md: crystals stay) | nets Net-(U15-XTAL_32K_*), Net-(U32-XTAL_32K_*), Net-(U17A-GPIO0/1).
16a. 40 MHz FC crystal | Y4 ECS-400 + L9 24 nH + C112/C113 12 pF (S3 U32) | Y4 ECS-400 + C48/C49 12 pF on P4 XTAL_P/N (no inductor) | MCU-specific | nets.
17. PWR_FLAGs | 7: #FLG07 GND (fc sheet, on the GND symbol), #FLG05 VBAT_CON (power sheet wire from C42/R44 into U19 IN), #FLG06 V_BUCK, #FLG04 J6 VBUS, #FLG01 Net-(U1-VCC), #FLG02 Net-(U15-VDD3P3), #FLG03 Net-(U32-VDD3P3) | 6: #FLG05 GND (on #PWR0705), #FLG04 V_BUCK, #FLG03 J6 VBUS, #FLG01 Net-(U1-VCC), #FLG02 Net-(U15-VDD3P3), #FLG06 ESP_VDD_HP | EQUIVALENT: every inductor/resistor-fed power net is flagged on both; ERC shows zero power_pin_not_driven on either board (live-mini: only 10 pin_not_connected errors; live-v10: 1) | schctx.py flag positions; erc.json census.
18. No-connect flags | flagged: J6 SBU ×2, S1 ×4, U1 HSD1 ×2, U15 SPICS1/SPICLK_N/P, U32 LNA_IN/SPICS1/SPICLK_N/P, U2 INT2/OCS_AUX/SDO_AUX, U5 ×5 (AADET_N, EX_ANT, RESERVED ×2, RESET_N), U16 DIO3/NC ×4, U23 ALERT + NC ×6, U3 NC ×10 and NOR NC balls (no_connect pin type); deliberately unflagged: U15 GPIO9/12/34/45, MTCK/MTDO/MTDI/MTMS, U0RXD/U0TXD (10 ERC errors, decisions.md) | flagged: same J6/S1/U1/U15 SPICS1/CLK pins, plus U15 JTAG ×4, UART0 ×2, GPIO45; U17 MIPI CSI/DSI ×16, DP/DM, GPIO16; U2 ×3; U23 NC ×6; U26/U28 PG + DNC; U3/NOR NC balls typed; unflagged: U15 GPIO13 (the one ERC error, decisions.md #1438) | SAME PINS FLAGGED wherever the pin exists on both boards; each board's unflagged set is an owner decision | single-node-net census with pintype from both netlist.xml files.
19. V10-only blocks (listed, not reviewed) | — | U17 ESP32-P4NRW32 (CHIP_PU C39 1 µF + R42 10k↑V_MCU_SWTCH; straps GPIO35 R53/R54/SW2, GPIO36 R50 100k↑; VDDPST/VDDA/VBAT on V_MCU_SWTCH; VDDO_FLASH/PSRAM/3/4 internal LDO outputs with C57/C61/C62/C81/C78/C83/C85/C89/C77/C80/C87); U20 TLV62569 core buck (EN_DCDC P4.79, FB_DCDC P4.78 with R74/R75 499k + C93 22 pF, L8 → ESP_VDD_HP + C55/C92/C58/C64/C67/C70/C73, R76 0 Ω → VDD_HP_1, PG GND, C47 10 µF input); J3 878321620 expansion EXP_01–12 on P4 GPIO45/44/43/54/39/40/29/28/38/37/34/46 (renumbered live), J3.1/2 servo rail from U28 TPS22811 (SERVO_ACT P4 GPIO8 + R87, IMON → S3 GPIO9 via R88 1k, C15 330 µF tant, C100 1 µF, C101 DVDT 10 nF); J4 camera from U26 TPS22811 (CAM_ACT P4 GPIO32 + R86, IMON → S3 GPIO8 via R85 2k, C14 22 µF, C98, C97 DVDT, Camera_TX/RX R32/R30 1k); J1 GNSS via U27 TPS22810 on VBATT (GPS_ACT P4 GPIO15 + R82, C95 CT, C99, C7 22 µF, FL1 bead; GNSS_RX/TX/RXD2 P4 GPIO4/3/2); J5 LoRa via U29 TPS22810 on VBATT (LoRa_ACT S3 GPIO12 + R83, C96 CT, C102, C18 22 µF, FL2; LoRa_RX/TX S3 GPIO10/11); J8 JST-VH battery; H2 MountingHole_Pad to GND; PG_RAIL/INA_ALERT | V10-ONLY | live-v10 netlist.
20. Mini-only by design | U32 second S3 with Y3/Y4/L9/L10/C110–C125/SW2, U5 LC86G on V_MCU_SWTCH (C37 22 µF/C38 100 nF/C39 10 nF, 1PPS → R66/D7), U16 E220-900MM22S on V_MCU_SWTCH (C23 22 µF/C24 100 nF, L_* control, R141 100k L_CS↑V_MCU_SWTCH, R142 100k L_RXEN↓, DIO2/TXEN tied), J8 SMA, J3 JST-PH battery, D6/D8 rail LEDs, C145–C148 plane caps | — | MINI-ONLY (README 'What the P4 took with it'; decisions.md) | live-mini netlist. Note refdes collisions across boards: mini R140/R141/R142/C143/C144 are CS pull-ups/RXEN/22 µF/100 nF while V10 R140/R141/R142/C143/C144 are bleed/GPIO0/ALERT/100 nF/22 µF — matched here by function, never by refdes.

CLEARED AGAINST V9 (fabbed, working): USB-C/CC/ESD, TPS2121 network, TPS62152 pin configuration (FSW/DEF to VOUT, PG pull-up), TPS22810/D9 enable chain, S3 crystals incl. the L3 24 nH on XTAL_P, USB series resistors, boot-NOR wiring, INA230 wiring, pyro channel network, eFuse divider values (V10 = V9) are all identical to V9 on the V10, and on the mini except where the mini's docs record a change (R34, C149, C47–C49, C150/C151, C152, R140/R141 pull-ups, TPS2596 eFuse, TPS61094 block which V9 lacked).

FIRST-ARTICLE (no-USB) RELEVANCE: nothing in this parity pass touches the mini's USB attach path — J6, CC pull-downs, VBUS, CR3, U21, U1/S1 are pin-for-pin and value-for-value identical to the V10 and to the fabbed, enumerating V9. None of the eight findings can explain the missing Rd on CC.


## The V9 to V10 change ledger

Every difference between the fabricated V9 and the live V10, with the rework or decision it
belongs to and whether it is complete:


CHANGE LEDGER (change | belongs to | verdict | evidence)
1. U14 Molex 479480001 chip antenna + L1 4.3 nH shunt removed; U22 AANI-CH-0070 + C12 DNP 0402 (shunt at antenna) + C23 5.1 pF (series) + L2 2.2 nH (shunt at LNA_IN) added | live 2026-09-21 edit, mini part parity (mini U14/C12/C7/L1) | CORRECT, exact match to the mini | live: Net-(C12-Pad1): C12.1 C23.1 U22.1[FEED] U22.4[FEED]; Net-(U15-LNA_IN): C23.2 L2.1 U15.1; C12.2/L2.2/U22.2/U22.3 = GND; C23 GJM1555C1H5R1WB01, L2 LQW15AN2N2C10. Mini: Net-(C12-Pad1): C12.1 C7.1 U14.1 U14.4; Net-(U15-LNA_IN): C7.2 L1.1 U15.1; same MPNs, C12 DNP. Abracon AANI-CH-0070 datasheet (fetched, work/v10-vs-v9/aani-ch-0070.txt): EVB match X1 = not mounted, X2 = 5.1 pF GJM1555C1H5R1WB01, X3 = 2.2 nH LQW15AN2N2C10 — the fitted values and the DNP position. V9 removal leaves nothing dangling: Net-(U14-Feed) and unconnected-(U14-Pad1..3) gone, no L1/U14 on the live PCB. Placement: U22 at (73.19,136.93) on the board's west edge with a 3.2×5.6 mm all-net rule area under it (73.1–76.3 × 132.5–138.1), match parts at (74.8–75.8, 139.1–140.5), U15 pad 1 at (75.53,142.47), H2 (grounded per decision) at (75.22,130.5).
2. LS1 MLT-8530, Q9 PMPB14XNX, R26 100, R28 100 k, CR1 CUS10S30 removed; PIEZZO net gone; U15 GPIO13 unconnected | live edit; #1438 context | COMPLETE except the missing no-connect flag (finding 2); no buzzer on the V10 at all | V9 nets quoted in finding 2; CR1 was only the LS1 flyback. Firmware: no OC code references GPIO13; FC PIEZO_PIN = 17 in board_v9.h now lands on V_SCAP_ADC on V10 (already on #1409).
3. C56 330 µF tant removed from V_MCU_2S; C45 22 µF 0805 and C88 22 µF 0805 added on Net-(U18-AVIN) | port of the mini's C56 decision (mini power-budget.md) | electrically sound, ported to a different node and not re-derived at the V10 load (finding 3) | nets in finding 3. New C45 is NOT on NRETRY: U19.11 NRETRY → GND (decision), Net-(U19-NRETRY) no longer exists; the old C45 1 µF 0402 (V9: C45.1 Net-(U19-NRETRY)) is a re-used designator, now CL21A226MOQNNNE.
4. C12 re-used: V9 C12 = EKYC160ELL103MM25S 10 mF pyro store on V_CAP (with R20 150 Ω charge resistor, both removed by the pack-fire rework, #1365 §A) → live C12 = DNP 0402 on the antenna feed shunt, identical to the mini's C12 DNP | pack-fire rework + antenna edit | CORRECT | comps/nets above.
5. J3 pins 9–14 reversed, EXP_12 = P4 GPIO46 | live edit | contract change undocumented (finding 1); no harness uses 9–14 (servo adapter netlist: J1 pin1 GND, 2–5 Servo4..1, 6 LiPoPos; rocket-side J3 1/2 servo rail, 3–6 EXP_01–04, 15/16 GND); pins 1–8, 15, 16 unchanged from V9 (Net-(U28-OUT), EXP_01..06, GND) | netlists in finding 1.
6. FC_ARM P4 GPIO33 (pad 64, was EXP_12); U17 pad 17 GPIO16 (was PYRO_ARM) no-connect | arm rework 4 | CORRECT and identical to the mini's chain: FC_ARM → R132 100 → Q12 DTC123JETL base; Q12 E GND; Q12 C → Q14 E; Q14 B = OC_ARM_EN (S3 GPIO14 pad 19 on V10, GPIO11 on the mini); Q14 C → Q13 DTA123JETL base; Q13 E = VBAT_CON; Q13 C → R139 1 k → ARM_GATE → R21 100 → U9 GATE; R22 100 k (was 5.11 k) U9 gate → GND; U9 AON7534 S1–3 GND, D5–8 PYRO_GND (pin 9 gone per the #678 symbol fix); R73 2.2 k PYRO_GND bleed. P4 GPIO33: IE at reset, no pull (datasheet pin 64); S3 GPIO14 not a strap. The R139 pin swap between HEAD and live is non-electrical.
7. TPS61094 hold-up block (U47, L11 XGL4020 2.2 µH, C130 HV1020 5 F, C141 22 µF, C142 10 µF, R134 3.09 k OSEL, R135 6.65 k VCHG, R136 22.1 k ICHG, R125/R126 100 k + C143 100 nF V_SCAP_ADC → P4 GPIO17 = ADC1_CH1, R137 100 k/R138 1 M VBUCK_OK → S3 GPIO34, R140 10 k bleed) | holdup-tps61094-rework.md, #999, #1165, #1022 | CORRECT, all 13 U47 pins identical to the mini's U47 (OSEL/MODE/EN/VIN/SW/SUP/PGND/AGND/VOUT×2/ICHG/VCHG/EP); VIN 3.465 V is inside the 0.7–5.5 V range; C17 22 µF + C144 22 µF on +3V3 mirror the mini's C17 + C143 | out/live-v10 vs out/live-mini pins U47; TPS61094 datasheet pins 1–12.
8. V_BUCK splice at U18 | v10-power-parity §1.1 | CORRECT | V_BUCK: C141 C53 L6.2 R137 R140 R49 R65 U18 VOS/FSW/DEF U47 MODE/EN/VIN; +3V3: U47 VOUT 9/10, C144, C17 and every 3.3 V load; nothing else on +3V3 from the buck. U18 DEF is high (V_BUCK → 3.465 V) as on the mini and unlike the fabbed V9 (DEF = GND, 3.3 V) — a recorded decision (decisions.md 'DEF high'; mini holdup doc: worst-high 3.562 V < 3.6 V abs max; TPS62152 §8.3.4 DEF = VOUT + 5 %, recommended connection to VOUT).
9. PG_RAIL / INA_ALERT | #721 / v10-power-parity §6.5 | CORRECT | PG_RAIL: R59.2 U15.23[GPIO17] U19.13[PG], R59.1 +3V3; INA_ALERT: R142.2 U15.24[GPIO18] U23.3[ALERT], R142.1 +3V3.
10. R141 10 k GPIO0 pull-up | #678 | CORRECT | Net-(U15-GPIO0): R141.1 SW3.2 U15.5; R141.2 +3V3.
11. C145 100 nF | #664 | CORRECT | OUT_VDD_SPI: C145.1 C27.1 U15.29. C146/C147 100 nF (#678, IMU/baro) both on V_MCU_SWTCH/GND — placement at U2/U4 pin 8 is #1365's. C148 10 µF (#678, RF supply) on Net-(U15-VDD3P3) with C33, L4.1, U15.2/3.
12. Pack-fire rework: V_CAP → VBAT_CON | pack-fire-holdup-options.md | CORRECT | VBAT_CON carries the four P-FET sources (U6/U7/U8/U10 pins 1–3), C9/C10/C11/C13 22 µF, R15/R16/R17/R23 10 k gate pull-ups, Q13 E, U19 IN, U23 BUS/IN−, R72.1; store R20/C12 gone; continuity dividers R8/R10/R12/R18 and D1–D4 BAT54 cathodes stay on V_MCU_SWTCH (complication 9). Complication 12's 'pad 17 → ARM_CLK' is the pre-rework-4 plan and is stale (pad 17 is now NC).
13. Part parity (PR #1452): U13/U16 W25Q128JVYIQ WLCSP — B2 VCC(+3V3) B3 /CS C2 /HOLD(HD) C3 DO(Q) D2 CLK D3 /WP E2 DI(D) E3 GND for both, matching the W25Q128JV §3.9 WLCSP24 ball table; all other balls NC-flagged. U4 BMP581 pins 1 VDDIO 2 SCK 3 VSS 4 SDI 5 SDO 6 CSB 7 INT 8 VSS 9 VSS 10 VDD — matches the BMP581 pin table (work/v10-vs-v9/bmp581.txt lines 2213–2223) and the mini pin-for-pin. U3 QMC5883P: 1 SCL(MAG_SCL), 2 VDD(V_MCU_SWTCH), 9 GND, 10 C1 → C6 10 µF, 11 GND, 16 SDA(MAG_SDA), 3–8/12–15 NC — identical to the mini's U3 (no datasheet in the repo or obtainable, see gaps); R40/R43 pull-ups on V_MCU_SWTCH. U6/U7/U8/U10 WSD20L50DN33 and U9 AON7534 keep the TSON land with S1–3/G4/D5–8 exactly as the mini. 22 µF K-grade swap reverted: every 22 µF is CL21A226MOQNNNE (M), per decision.
14. Earlier V9→HEAD items re-verified present and correct on live: H2 MountingHole_Pad pin 1 → GND; C44 10 nF SS; C94 10 µF EN/UVLO deglitch; C6 10 µF; Q3–Q6 DTC123JETL; D7 Red NCD0402R1 / D8 Blue (bench-confirmed swap); J2 JL212R-SMT-35005BP1 on the CUI TBLH11 land (10 SMD pads) — same part/land as the fabbed mini's J2; R72 PA0805FRF870R002L 2 mΩ — same as the mini; Q11 AONR21321 with S1–3 = VBAT_Terminal, G4 = GND, D5–9 = VBAT_J8 (correct reverse-battery orientation, identical to the mini; CR2 deleted per #677); U2 INT2 no-connect (unused by firmware); I2S link rename ESP_CS/SCLK/SDO/SDI → ESP_I2S_WS/BCLK/SD/FSYNC with both ends (U15.6–9 ↔ U17.19/23/20/22) unchanged; CHIP_PU net renamed FC_CHIP_PU (C39/R42/U17.103 unchanged); NRETRY → GND.
15. Live board: schematic parity 0, the eight removed footprints absent, the six new ones placed; routing not reviewed (out of scope). ERC: 1 error (finding 2), everything else the pre-existing off-grid/lib-mismatch/pin-type warning classes.

Docs drift noticed but not filed separately (owner's text): v10-power-parity §3.1/§3.2 and pack-fire-holdup-options still list the buzzer and C56; FABRICATION-NOTES B4/B7/B9 still describe the V9 (C12 can, J3 order); board_v9.h EXP comment.


## Findings


### Not carried over from the mini

#### 1. [Minor] R34 (100 k pull-down on ESP_I2S_SD = S3 GPIO3 JTAG strap) was not ported to the V10

- **Refs:** V10: U15 pin 8 (GPIO3), U17 pin 20 (P4 GPIO19). Mini counterpart: R34 100 k (RC0402FR-07100KL)
- **Nets:** ESP_I2S_SD
- **Reviewer:** `v10-parity-1`, confidence 0.9
- **Verified here:** CONFIRMED by netlist: on the mini the net carries R34 to ground; on the live V10 it has only the two processor pins.

**Claim.** On the mini ESP_I2S_SD carries R34 100 k to GND so the out computer's GPIO3 strapping pad has a defined level while the flight computer (its driver) is unpowered. The V10 drives the same S3 pad from P4 GPIO19 on the switched rail and has no pull at all.

**Evidence.** live-mini netlist: ESP_I2S_SD = R34.1, U15.8[GPIO3], U32.18; R34.2 = GND. live-v10 netlist: ESP_I2S_SD = U15.8[GPIO3_8], U17.20[GPIO19_20] only; U17 is on V_MCU_SWTCH (off at S3 boot). ESP32-S3 datasheet §3.4 (esp32-s3_datasheet_en.txt line 1761): GPIO3 'does not have any internal pull resistors and the strapping value must be controlled by the external circuit that cannot be in a high impedance state'; Table 3-1 lists GPIO3 default 'Floating'; Table 3-5: GPIO3 is 'Ignored' while EFUSE_STRAP_JTAG_SEL = 0 (default). The mini's own pin-budget.md line 235 records that 'rocket-computer flies the same net on GPIO3 with no pull'. V9 (fabbed) ESP_SDO on U15.8 has the same no-pull condition and boots.

**Consequence.** Inert with unburned eFuses (the strap is ignored), so no boot failure today; the pad floats for the whole P4-off period (extra input leakage, susceptibility) and the board would be one eFuse burn away from an undefined JTAG-source strap. The mini decided this was worth one 0402; the V10 did not get it.

**Fix.** Add a 100 k 0402 from ESP_I2S_SD to GND beside U15 pin 8 (the mini's R34, same MPN); no other change.

#### 2. [Minor] INA230 supply bypass: the mini's C149 (100 nF at U23 VS+) has no V10 counterpart, and the live placement moved the V9 cap that used to serve U23

- **Refs:** V10: U23 pin 9 (VS+), C40 (now at U16), C50/C46/C94 (nearest caps, eFuse timing). Mini counterpart: C149 100 nF
- **Nets:** +3V3
- **Reviewer:** `v10-parity-2`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** The mini gives the INA230 its own 100 nF on +3V3 (C149, placed 5.0 mm from U23 on B.Cu; FABRICATION-NOTES groups it with U23). The V10 schematic has never assigned a bypass to U23; on the fabbed V9 the nearest +3V3 100 nF (C40) sat 2.0 mm from U23 and did the job by placement, but in the live V10 board C40 has been moved to the P4 boot NOR U16 (2.5 mm) and nothing on +3V3 is within 5.5 mm of U23.

**Evidence.** live-mini netlist: C149.1 on +3V3, C149.2 GND; mini board C149 at (82.18,110.02) B, nearest U23@5.0 mm. V10 netlist +3V3 members contain no cap drawn at U23 (in_sensors sheet caps: C2–C7, C95, C99, C146, C147 — none on +3V3). V9 board: C40 (100 nF, +3V3) at (76.90,124.90), U23@2.0 mm. Live V10 board: C40 at (73.58,118.59) nearest U16@2.5 mm; U23's nearest caps C50 (10 nF ITIMER)@1.1, C46 (2.2 nF)@2.3, C94 (10 µF)@2.4, C17 (22 µF)@5.5 — none on +3V3 except C17. INA230 datasheet (ina230.txt lines 29–31 and 911–913): CBYPASS 0.1 µF on the supply in both application figures.

**Consequence.** The pack-current/voltage monitor — the always-on I2C device that #1000 and #1409 make the OC's pack-loss indicator — has no local supply bypass on the V10; V9 had one by accident of placement. Works, but a datasheet-conformance gap that the mini closed and the V10 placement is currently opening.

**Fix.** Draw a 100 nF 0402 on +3V3 at U23 pin 9 (the mini's C149) so the bypass survives placement; or return C40 to U23 and give U16 its own. Placement-level, for #1365.

#### 3. [Minor] Live V10 deleted C56 (330 µF) from the TPS2121 output and put the replacement bulk on the far side of L5; the mini keeps 3×22 µF on V_MCU_2S

- **Refs:** V10: C59 1 µF (only cap left on V_MCU_2S), C45 + C88 22 µF (new, on Net-(U18-AVIN)), C43 22 µF, L5 2.2 µH, U21. Mini counterpart: C47/C48/C49 3×22 µF on V_MCU_2S, C43 22 µF beyond L5
- **Nets:** V_MCU_2S, Net-(U18-AVIN)
- **Reviewer:** `v10-parity-3`, confidence 0.6
- **Verified here:** CONFIRMED by my own netlist comparison. The mux output net carries C47/C48/C49 22 uF plus C59 1 uF on the mini, C56 330 uF plus C59 1 uF on the V9 and on the committed V10, and C59 1 uF alone on the live V10. The two new 22 uF parts sit on the far side of L5 at the buck input.

**Claim.** The owner's uncommitted V10 work removes the V9's 330 µF tantalum from the mux output and adds two 22 µF ceramics at the buck input, i.e. behind L5. That leaves the TPS2121 OUT node with 1 µF. The mini — the design the V10 is meant to match — puts its three 22 µF on the mux output (V_MCU_2S) and one 22 µF after L5.

**Evidence.** dump.py diff head-v10 → live-v10: '- removed C56 TCJE337M016R0050', '+ added C45 22 uF 0805', '+ added C88 22 uF 0805'. live-v10 netlist: V_MCU_2S = C59.1, L5.1, U21.1, U21.8; Net-(U18-AVIN) = C43, C45, C65, C88, L5.2, U18 AVIN/PVIN/EN. live-mini: V_MCU_2S = C47, C48, C49 (22 µF), C59 (1 µF), L5.1, U21 OUT; Net-(U18-AVIN) = C43 22 µF + C65 100 nF. v9 netlist: V_MCU_2S = C56 (330 µF) + C59 + L5.1 + U21. TPS2121 datasheet (tps2121.txt lines 314, 34, 816): tSW switchover time 100 µs typ (EC table, 0.5 A / 100 µF), fast switchover 5 µs; VOUT,MIN = VSW − tSW × IOUT/COUT; §11: 'To avoid output voltage drop, the capacitance on OUT can be increased'. v10-power-parity-2026-09-11.md §3.2 still says a pack bounce is '~3 ms on C56, then the cap'.

**Consequence.** During any USB↔pack switchover the mux OUT node collapses (0.5 A × 5 µs / 1 µF = 2.5 V; at 100 µs it goes to zero) and rings against L5 (f0 ≈ 107 kHz with 1 µF), while the buck itself rides the 66 µF behind the inductor. Functionally the buck input is arguably better held than on the mini, but the mux sees a different load than it was qualified with on V9, and the §3.2 hold-up narrative and the pack-bounce timing in the parity doc are now wrong by ~15×. Undocumented; may be mid-edit.

**Fix.** Either mirror the mini (put ≥2 of the 22 µF ceramics on V_MCU_2S at U21 OUT, keep C43 at U18 PVIN) or record the intent — and update v10-power-parity §3.2 whichever way. Question for the owner since the edit is uncommitted.

#### 4. [Note] M_FLASH_CS (NAND CS#) has no pull-up on the V10; the mini's R140 100 k was classed 'low, not drawn' but the reasoning that made it mini-specific does not apply

- **Refs:** V10: U11 pin 1 (CS#), U15 pin 41 (GPIO36). Mini counterpart: R140 100 k to V_MCU_SWTCH
- **Nets:** M_FLASH_CS
- **Reviewer:** `v10-parity-4`, confidence 0.7
- **Verification:** not run (single source).

**Claim.** The V10's NAND is powered (+3V3, same rail as its master) while the S3 is in reset/ROM/being flashed, and GPIO36 has no reset pull, so CS# floats with SCK/MOSI. v10-power-parity §2 records this as 'low, not drawn' because the mini's #1014 rationale (pull to the slave's rail to avoid back-feeding a dead V_MCU_SWTCH) is mini-specific — but on the V10 master and slave share +3V3, so a plain 100 k to +3V3 has no back-feed concern and the floating-CS exposure is the same as the one #1014 fixed.

**Evidence.** live-v10 netlist: M_FLASH_CS = U11.1[CS#_1], U15.41[GPIO36_41] (2 nodes); U11.8 VCC on +3V3. live-mini: M_FLASH_CS = R140.2, U11.1, U15.41; R140.1 = V_MCU_SWTCH. #1014 (closed): 'Whenever the out computer is in reset, in the ROM or being flashed while the flight computer holds the rail up, both chip selects float together with SCK and MOSI'. v10-power-parity-2026-09-11.md §2 row '#1014': 'M_FLASH_CS still has no pull-up, which only matters for the milliseconds the S3 is in reset — low, not drawn'.

**Consequence.** Exposure is bounded (a write needs WREN first) but the flight logger's storage sees a floating select during every OC reset and OTA flash. One 0402.

**Fix.** If the owner wants it: 100 k from M_FLASH_CS to +3V3 beside U11 pin 1. Otherwise leave as documented.


### The uncommitted changes on the live schematic

#### 5. [Minor] S3 GPIO13 left dangling without a no-connect flag after the buzzer deletion (the one live ERC error)

- **Refs:** U15 pin 18 (GPIO13); deleted LS1/Q9/R26/R28
- **Nets:** unconnected-(U15-GPIO13-Pad18) (HEAD: PIEZZO)
- **Reviewer:** `v10-mcu-headers-1`, confidence 0.95
- **Verified here:** CONFIRMED, same measurement as v10-vs-v9-2.

**Claim.** In the live schematic U15.18 (GPIO13) is neither wired nor flagged NC. HEAD had it as PIEZZO -> R26 -> Q9 -> LS1; the live work deletes all four parts and leaves the pad bare, which is the single ERC error on the board. Whether the buzzer is meant to go entirely (V9 issue #1438 moved it here precisely because the V9 piezo was silent) is a question for the owner, not something this review can settle.

**Evidence.** $S/out/live-v10/erc.json: sheet '/OUT - ESP32-S3 Outputs' error pin_not_connected 'Symbol U15 Pin 18 [GPIO13, Bidirectional, Line]'. $S/out/head-v10/netlist.xml: PIEZZO: R26.1 U15.18[GPIO13_18]. $S/out/live-v10/netlist.xml: U15.18 GPIO13_18 bidirectional unconnected-(U15-GPIO13-Pad18) with no no_connect pintype (contrast U15.51 GPIO45 'bidirectional+no_connect'). dump.py diff v9->live: '- removed LS1 MLT-8530, Q9 PMPB14XNX, R26 100, R28 100 k'. S3 datasheet Table 2-1 row 18: GPIO13 IO, VDD3P3_RTC, after reset IE only, not a strapping pin - so the bare pad is electrically harmless.

**Consequence.** ERC is no longer clean (the decisions say the V10's ERC floor should be zero on this sheet; the ten deliberately unflagged spares are a mini rule). A bare S3 input pad floats, which is harmless on GPIO13. If the buzzer is meant to return, GPIO13 is the pin it must come back on (the S3 has no other spare).

**Fix.** If the deletion is intended: place a no-connect flag on U15.18 and note in esp32s3_outputs.kicad_sch that GPIO13 is the last S3 spare. If not: restore the HEAD circuit (PIEZZO -> R26 100 -> Q9 base, R28 100 k pulldown, LS1).

#### 6. [Minor] J3 pins 9-14 renumbered (EXP_07..12 now ascending): consistent in the netlist/PCB, but FABRICATION-NOTES B9 and board_v9.h now name the wrong J3 pins for the P4 boot straps and a V9 payload harness on those pins no longer matches

- **Refs:** J3 878321620 pins 9-14; U17 GPIO29/28/38/37/34/46; FABRICATION-NOTES.md B9; tinkerrocket-idf/projects/flight_computer/main/board/board_v9.h
- **Nets:** EXP_07 EXP_08 EXP_09 EXP_10 EXP_11 EXP_12
- **Reviewer:** `v10-mcu-headers-2`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** Live V10: J3.9=EXP_07 (GPIO29), .10=EXP_08 (GPIO28), .11=EXP_09 (GPIO38), .12=EXP_10 (GPIO37), .13=EXP_11 (GPIO34), .14=EXP_12 (now P4 GPIO46, freed by the QMC5883P swap). V9 (fabbed): J3.9=EXP_12 (GPIO33) ... J3.14=EXP_07 (GPIO29). Every one of the six connector pins changes GPIO. The strap lines GPIO38/37/34 therefore move from J3.12/11/10 to J3.11/12/13. The schematic, netlist and PCB agree (0 parity items), but two documents that harness and payload builders read still carry the V9 order: FABRICATION-NOTES B9 ('EXP LINES 09/10/11 (J3 PINS 12/11/10) CARRY P4 BOOT STRAPS') and the firmware header comment ('pins 9..14 carry EXP_12..07 DESCENDING - pin 9 is EXP_12'). The four fin servos (EXP_01-04, J3.3-6) and the power/ground pins are untouched, so the servo adapter and its cable are unaffected. No V10 doc records why the order was changed.

**Evidence.** $S/out/live-v10/netlist.xml J3.9..14 -> EXP_07..EXP_12; $S/out/v9/netlist.xml J3.9..14 -> EXP_12..EXP_07; dump.py diff v9->live: '~ J3.9: EXP_12 -> EXP_07 ... ~ J3.14: EXP_07 -> EXP_12', '~ U17.88: IIS2MDCTR_INT -> EXP_12', '~ U17.64: EXP_12 -> FC_ARM'. Live EXP_11: J3.13 U17.65[GPIO34]; EXP_10: J3.12 U17.69[GPIO37]; EXP_09: J3.11 U17.70[GPIO38]. $S/live/hardware/rocket-computer/FABRICATION-NOTES.md line 136: 'B9. EXP LINES 09/10/11 (J3 PINS 12/11/10) CARRY P4 BOOT STRAPS (GPIO38/37/34)'. board_v9.h (FC) fin-servo block: 'pins 9..14 carry EXP_12..07 DESCENDING - pin 9 is EXP_12, not EXP_07' and 'EXP_12 33 64' (GPIO33 is FC_ARM on the live V10). Servo adapter netlist ($S/work/v10-mcu-headers/servo-adapter.xml): J1.1 GND, J1.2-5 Servo4..1, J1.6 LiPoPos - only EXP_01-04 and power are used. P4 datasheet v0.7 Table 3-1: GPIO34/37/38 default Floating; sec 3.4: GPIO34 is the JTAG-source strap 'must be controlled by the external circuit that cannot be in a high impedance state' (only sampled with EFUSE_JTAG_SEL_ENABLE=1, Table 3-7); Table 2-3: GPIO37=UART0_TXD (ROM boot log burst), GPIO38=UART0_RXD. WORKLIST.md M-15 / issue #725: the B9 interface rule is the chosen mitigation, series resistors were rejected.

**Consequence.** A payload built to B9 keeps J3.10/11/12 high-Z at power-on and may drive J3.13 - which on the V10 is GPIO34, the JTAG-source strap - and J3.11 (GPIO38, UART0_RXD). Today that is benign (the eFuse default ignores GPIO34; SPI boot ignores GPIO37/38), so the exposure is the same one #725 already tracks, but pointed at the wrong pins. The ROM boot-log burst moves from J3.11 to J3.12. Any V9 expansion harness that used pins 9-14 is silently re-mapped on a V10. The V10 firmware header (#1409) must not inherit the V9 comment.

**Fix.** Confirm the renumbering is intended (it reads as a tidy-up enabled by GPIO46 -> EXP_12). Then: rewrite B9 to 'EXP_09/10/11 (J3 PINS 11/12/13)' and the boot-log line to J3.12; when board_v10.h is written (#1409) give it the live table (J3.9..14 = EXP_07..12 ascending, EXP_12 = GPIO46, FC_ARM = GPIO33) and a one-line 'V9 boards are pins 9..14 descending' warning; add a line to cables.md / B7 that a V9-built pins-9-14 harness does not carry over.

#### 7. [Minor] J3 pins 9–14 reversed vs the fabbed V9: the header contract changed and nothing that documents it (board_v9.h, FABRICATION-NOTES B9, cables.md) knows

- **Refs:** J3 (Molex 878321620), U17 pads 58/57/70/69/65/88
- **Nets:** EXP_07…EXP_12
- **Reviewer:** `v10-vs-v9-1`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** The live V10 puts EXP_07…EXP_12 on J3.9…J3.14 ascending (EXP_12 = P4 GPIO46). The fabbed V9 has the same connector with the opposite order: J3.9 = EXP_12 (GPIO33) … J3.14 = EXP_07 (GPIO29). HEAD had J3.9 unconnected (GPIO33 taken by FC_ARM) with 10–14 still descending; the live edit re-filled pin 9 and flipped all six. No document or header follows: board_v9.h (which is also the V10's header, no board_v10.h exists) says 'pins 9..14 carry EXP_12..07 DESCENDING — pin 9 is EXP_12, not EXP_07'; FABRICATION-NOTES B9 says 'EXP LINES 09/10/11 (J3 PINS 12/11/10) CARRY P4 BOOT STRAPS (GPIO38/37/34)' and 'the P4 ROM boot log transmits on EXP_10' — on the live V10 those strap lines are J3 pins 11/12/13 and the boot-log pin moved from J3.11 to J3.12; cables.md has no J3 map at all.

**Evidence.** out/v9/netlist.xml: EXP_07: J3.14 U17.58[GPIO29]; EXP_12: J3.9 U17.64[GPIO33]. out/head-v10/netlist.xml: J3.9 unconnected, J3.10..14 = EXP_11..07. out/live-v10/netlist.xml: EXP_07: J3.9 U17.58; EXP_08: J3.10 U17.57; EXP_09: J3.11 U17.70[GPIO38]; EXP_10: J3.12 U17.69[GPIO37]; EXP_11: J3.13 U17.65[GPIO34]; EXP_12: J3.14 U17.88[GPIO46]; live PCB J3 pads 9–14 carry the same (pcbnew). tinkerrocket-idf/projects/flight_computer/main/board/board_v9.h lines 178–189 (table + 'DESCENDING' note; only SERVO_PIN_1..4 = EXP_01..04 on pins 3–6 are used in code). live/hardware/rocket-computer/FABRICATION-NOTES.md B9. Servo adapter (live/hardware/servo-adapter netlist exported to work/v10-vs-v9/servo-adapter-netlist.xml): uses only Servo1–4, LiPoPos, GND — i.e. J3 pins 1/2, 3–6, 15/16; pins 9–14 are used by no harness in the repo. ESP32-P4 datasheet: GPIO34/35/36/37/38 are the strapping pins; GPIO46 (pad 88, VDD_IO_5) has no strap or reset pull.

**Consequence.** A payload cable built to the V9/B9 contract plugged into a V10 gets EXP_07↔12, 08↔11, 09↔10 swapped. Worse, B9's interface rule ('these three lines must be inputs or high-Z until the flight computer is up') now points at J3 pins 12/11/10, of which only pin 12 (EXP_10/GPIO37) is still a strap; pin 13 (EXP_11 = GPIO34, the JTAG-source strap) is no longer on the protected list and a payload driving it at power-on can corrupt P4 boot mode — the exact failure #672/#725 closed. Today nothing uses pins 9–14, so it is a contract/doc defect, not a live fault.

**Fix.** Owner's call, two clean options: (a) restore the V9 order on J3.9–14 (nothing electrical forced the flip; EXP_12 = GPIO46 on J3.9 keeps the fabbed pinout) or (b) keep the new order and in the same commit update FABRICATION-NOTES B9 (straps on J3 pins 11/12/13, boot log on pin 12), the board_v9.h comment (or the future board_v10.h), and add a J3 row to cables.md with the revision the harness is built for. Either way add the V9→V10 J3 difference to the cable-compatibility notes.

#### 8. [Minor] U15 GPIO13 left dangling after the buzzer removal — the one live ERC error; needs a no-connect flag

- **Refs:** U15 pin 18 (GPIO13)
- **Nets:** unconnected-(U15-GPIO13-Pad18) (was PIEZZO)
- **Reviewer:** `v10-vs-v9-2`, confidence 0.95
- **Verified here:** CONFIRMED: the live electrical-rule export carries exactly one error, U15 pin 18 GPIO13 not connected.

**Claim.** Removing LS1/Q9/R26/R28/CR1 deleted the PIEZZO net and left the S3's GPIO13 pin with no wire and no no-connect flag. ERC on the live V10 reports exactly one error, this pin. The rest of the removal is clean: no residual PIEZZO/piezo/buzzer label or text survives in any live .kicad_sch, no orphan pull-down, and CR1 was purely the LS1 flyback (V9: CR1.1 = V_MCU_SWTCH, CR1.2 = Net-(CR1-Pad2) shared only with LS1 LOAD− and Q9 drain), so nothing else lost protection.

**Evidence.** out/live-v10/erc.json: '/OUT - ESP32-S3 Outputs/ error pin_not_connected Symbol U15 Pin 18 [GPIO13]' — the only error-severity item (the #680 floor was 0). out/v9/netlist.xml: PIEZZO: R26.1 U17.18[GPIO17]; Net-(CR1-Pad2): CR1.2 LS1.LOAD- Q9.1/2/5/6/7; Net-(Q9-Pad3): Q9.3 R26.2 R28.1; GND: R28.2, Q9.4/8, CR1 absent. grep -i 'PIEZZO|piezo|buzzer' over live/hardware/rocket-computer/*.kicad_sch: no text hits. Live PCB: LS1/U14/C56/CR1/Q9/R26/R28/L1/R20 all absent (pcbnew footprint scan), parity 0.

**Consequence.** ERC no longer at its 0-error floor, so the next real error can hide behind it. Functionally: the V10 has no sound output at all — #1438 §4 and #1409's PIEZZO line both assume the buzzer moved to S3 GPIO13 and plan an OC piezo driver; that plan is now moot (decisions.md already records the intent question as the owner's).

**Fix.** Add a no-connect flag on U15 pin 18. When the removal is committed, close the loop on #1438 §4 / #1409 (no OC piezo driver needed; FC PIEZO_PIN must be -1 on V10 so nothing drives V_SCAP_ADC) and drop 'buzzer' from the V_MCU_SWTCH load lists in v10-power-parity-2026-09-11.md §3.1 and pack-fire-holdup-options.md (tree line 39, complication 3).

#### 9. [Minor] C56 330 µF removal is the mini's ceramic-only decision, but ported to the other side of L5 and never re-run at the V10's 2–3× load

- **Refs:** C56 (removed), C45, C88 (new 22 µF 0805), C43, C59, L5, U21, U18
- **Nets:** V_MCU_2S, Net-(U18-AVIN)
- **Reviewer:** `v10-vs-v9-3`, confidence 0.6
- **Verified here:** CONFIRMED, same measurement as v10-parity-3.

**Claim.** On the V9, C56 was the only bulk on V_MCU_2S (TPS2121 output, ahead of L5 2.2 µH); power-eco.md sized it to ride out servo/camera sags on the raw pack node, then retracted the figure on 2026-08-08 (3 ms at the real 0.5–0.8 A load; the job moved upstream to the eFuse deglitch C94). The mini deleted its C56 for the same reason and fitted three 22 µF 0805 in its place on V_MCU_2S (C47/C48/C49 + C59 1 µF, before L5), and its power-budget.md computed the TPS2121 USB-removal switchover dip at ~12 µF effective (18 % retention at 8.4 V) as ~100 mV against 0.39 V headroom at the 6.4 V cutoff — 4× margin at the mini's ~0.3 A. The V10 port puts the two new 22 µF (C45, C88) on Net-(U18-AVIN) beside C43, i.e. after L5, and leaves V_MCU_2S with C59 1 µF alone. The V10's flight load on this chain is 0.5–0.8 A (power-eco correction; v10-power-parity §3.3), so the same 5 µs switchover draws 2–3× the charge from whatever holds the node; scaled from the mini's own arithmetic the buck-input dip is ~0.25–0.35 V against the same 0.39 V headroom at cutoff — ~1.1–1.5× margin, not 4×. Nothing in the V10 docs records the port or re-runs the number; v10-power-parity §3.2 still says 'pack bounce … ~3 ms on C56, then the cap'.

**Evidence.** out/v9/netlist.xml: V_MCU_2S: C56.1 C59.1 L5.1 U21.1/8; C56 = TCJE337M016R0050 330 µF. out/live-v10/netlist.xml: V_MCU_2S: C59.1 L5.1 U21.1/8 (C59 1 µF); Net-(U18-AVIN): C43.1 C45.1 C65.1 C88.1 L5.2 U18.10/11/12/13 (C43/C45/C88 = CL21A226MOQNNNE 22 µF 16 V). out/live-mini/netlist.xml: V_MCU_2S: C47.1 C48.1 C49.1 C59.1 L5.1 U21.1/8 (22 µF ×3 + 1 µF); Net-(U18-AVIN): C43.1 C65.1 L5.2. live/hardware/rocket-computer-mini/power-budget.md 'C56 — replacing the bulk polymer with ceramics' (mux imposes no minimum, Middlebrook margin 16–27 dB, 18 % retention, dip table: full pack 0.75 V headroom/~100 mV, cutoff 0.39 V/~100 mV, fast switchover armed by the fitted dividers). live/hardware/rocket-computer/power-eco.md lines 25–29 (C56 rationale and its 2026-08-08 retraction). v10-power-parity-2026-09-11.md §3.2 row 'pack bounce < 0.27 s … ~3 ms on C56'. Live placement (pcbnew): U18 (88.81,138.25) F.Cu; C43 (91.99,127.35) B.Cu; C65 (91.16,127.81); C45 (84.2,141.58); C88 (82.18,138.25); C59 (84.12,135.86); L5 (84.91,138.11) — placement in progress.

**Consequence.** Not a flight event (USB is never attached in flight) and the supercap now covers every pack-side interruption the polymer was once sized for, so the removal itself is sound. The exposure is a bench one: unplugging USB with the pack connected at a heavy load and a low pack may dip the buck input into dropout and reboot the board, and the reasoning that says it will not was done for a different node at a third of the load. The Middlebrook side is if anything better on the V10 (more C after L5 lowers the filter's characteristic impedance).

**Fix.** Discussion item, not a redraw: either move C45/C88 onto V_MCU_2S beside U21 to mirror the mini exactly (then the mini's numbers transfer directly, scaled by load), or keep them at the buck input and record the V10 switchover-dip arithmetic at 0.8 A in v10-power-parity (and fix the '~3 ms on C56' row). Add 'USB unplug with pack attached, full load, 6.4 V pack — scope V_MCU_2S and Net-(U18-AVIN)' to the #1211 bench list; the mini's own note says its retention figure is single-sourced. Placement (for #1365): whichever node they end on, the buck-input 22 µFs currently sit 5–11 mm from U18's PVIN pads.

#### 10. [Note] The V10 replaces the V9's bench-proven Molex antenna and 4.3 nH match with the mini's AANI-CH-0070 chip antenna and L2/C23/C12 network - a match that has not yet been verified on any board

- **Refs:** U22 AANI-CH-0070, L2 2.2 nH, C23 5.1 pF, C12 DNP; removed U14 479480001, L1 4.3 nH
- **Nets:** Net-(U15-LNA_IN), Net-(C12-Pad1)
- **Reviewer:** `v10-mcu-headers-5`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** Live V10 RF: U15 LNA_IN -> L2 2.2 nH shunt to GND and C23 5.1 pF series -> U22 feed (pads 1 and 4), with C12 (DNP) as a shunt tuning slot at the feed. This is pad-for-pad the mini's network (L1 2.2 nH shunt, C7 5.1 pF series, C12 DNP shunt, U14 AANI-CH-0070). The old Molex feed net (Net-(U14-Feed) with L1 4.3 nH shunt) is fully gone - no U14, no L1 4.3 nH, no orphan net. The point of record: the V9's BLE link is bench-proven with the Molex part; the mini's chip-antenna match has never been measured because the first mini never enumerated. The V10 therefore inherits an unverified RF front end, and a chip antenna's match is also layout-dependent (ground clearance and keep-out), which is the owner's in-progress placement.

**Evidence.** $S/out/live-v10/netlist.xml: Net-(U15-LNA_IN): C23.2 L2.1 U15.1; L2.2 -> GND; Net-(C12-Pad1): C12.1 C23.1 U22.1[FEED_1] U22.4[FEED_4]; C12.2 -> GND; C12 value 'DNP' with the dnp property set. $S/out/live-mini/netlist.xml: Net-(U15-LNA_IN): C7.2 L1.1 U15.1; Net-(C12-Pad1): C12.1 C7.1 U14.1 U14.4; L1 2.2 nH LQW15AN2N2C10, C7 5.1 pF GJM1555C1H5R1WB01. $S/out/v9/netlist.xml: Net-(U14-Feed): L1.2 U14.4[Feed_4] U15.1[LNA_IN_1], L1 4.3 nH to GND. dump.py diff v9->live: '- removed L1 4.3 nH', '- removed U14 479480001', '+ added C23 5.1 pF', '+ added L2 2.2 nH', '+ added U22 AANI-CH-0070'; grep of nets-live.txt for 'U14' returns nothing. BRIEF: the mini 'never enumerated on USB' on 2026-09-20.

**Consequence.** If the mini's match turns out to be off, both boards are off together and the V10 has no fallback footprint for the proven Molex part. Not a defect - the mini parity was decided - but the V10's RF sign-off should wait for a mini BLE range measurement, or the C12 slot should be exercised on the mini first.

**Fix.** No schematic change. Add to the V10's bring-up plan: verify BLE RSSI on the mini (or on the V10 itself) before any range-critical flight; keep the C12 DNP slot and the mini's L1/C7 values as the tuning baseline; make sure the U22 keep-out from the mini's layout is reproduced at placement (#1365).

#### 11. [Note] P4 GPIO46 now leaves the board as EXP_12 while the V10's firmware header still declares it the magnetometer interrupt

- **Refs:** U17 pad 88 (GPIO46), J3.14, U3
- **Nets:** EXP_12
- **Reviewer:** `v10-vs-v9-4`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** mini-part-parity-2026-09-12.md freed P4 GPIO46 when the IIS2MDC (INT on GPIO46) became the QMC5883P (no INT pad) and left it as a no-connect; the live edit re-uses it as EXP_12 on J3.14. board_v9.h — the header a V10 builds with until board_v10.h exists — still has IIS2MDC_INT = 46 and passes it into the sensor collector, so a V9-image on V10 hardware would configure a header pin as the magnetometer's interrupt input and select the ST driver for a QST part.

**Evidence.** out/head-v10/netlist.xml: unconnected-(U17A-GPIO46-Pad88). out/live-v10/netlist.xml: EXP_12: J3.14 U17.88[GPIO46]. tinkerrocket-idf/projects/flight_computer/main/board/board_v9.h line 114 'IIS2MDC_INT = 46'; main.cpp ~line 114 passes config::IIS2MDC_INT to the collector. mini-part-parity §3 'board_v9.h still declares IIS2MDC_INT = 46 and selects the ST driver'. #1409 already lists the QMC driver/no-INT pick-up but not that the pin is now an external line. ESP32-P4 datasheet pin 88: IO, VDD_IO_5, no reset IE/pull, not a strapping pin — electrically fine for a header line.

**Consequence.** Harmless as an input, but until board_v10.h lands a payload toggling EXP_12 would raise spurious magnetometer-driver interrupts on a V9-image V10. Also the fourth pin on the same VDD_IO_5 domain as EXP_01/02 (GPIO45/44, pads 87/86) — consistent.

**Fix.** Add one line to #1409's board_v10.h item: IIS2MDC_INT → -1 on V10, GPIO46 is EXP_12 (header), and the EXP table in the header comment follows finding 1's resolution.

#### 12. [Note] Committed bom.csv is fourteen designators behind the live schematic

- **Refs:** C12, C23, C45, C88, L2, U22 (new/changed); C56, CR1, L1, LS1, Q9, R26, R28, U14 (removed)
- **Nets:** —
- **Reviewer:** `v10-vs-v9-5`, confidence 0.95
- **Verification:** not run (single source).

**Claim.** The hand-maintained hardware/rocket-computer/bom.csv still lists C56 (with C15 on the 330 µF line), CR1, L1 4.3 nH, LS1, Q9, R26, R28 and U14 479480001 as Fit, has no rows for C23, L2, U22 or the new C45/C88 22 µF positions, and has no DNP entry for C12. Expected for uncommitted work — recorded so the commit that lands the antenna/buzzer/C56 edits also reconciles the BOM, as the two parity passes did (v10-power-parity §1.5 caught exactly this class: a 10 µF 0402 ordered into a 22 µF 0805 land).

**Evidence.** grep of live/hardware/rocket-computer/bom.csv vs out/live-v10/bom.csv (fresh export): repo rows 4 ('C15, C56' TCJE337M016R0050), 12 (CR1), 27 (L1 4.3 nH), 32 (LS1), 35 (Q9), 41 (R26 in the 100 Ω line), 37 (R28 in the 100 k line), 68 (U14); fresh export has C12 DNP, C23 5.1 pF, L2 2.2 nH, U22 AANI-CH-0070, and 14 designators on the 22 µF line including C45/C88.

**Consequence.** An assembler ordering from the committed file builds a V9 buzzer and a Molex antenna onto a board with no lands for them and misses the chip antenna's match.

**Fix.** Regenerate/reconcile bom.csv against a fresh kicad-cli export in the same commit as the schematic edits.


### Processors, headers and external interfaces

#### 13. [Minor] ESP_I2S_SD lands on the S3's GPIO3 JTAG-source strap with no pull, while the mini added R34 100 k for exactly this net - the V10 is a fresh layout and the mini's own rule says not to carry a floating strap into one

- **Refs:** U15 pin 8 (GPIO3), U17 pin 20 (GPIO19); mini R34
- **Nets:** ESP_I2S_SD
- **Reviewer:** `v10-mcu-headers-3`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** On the live V10, ESP_I2S_SD is a two-node net (S3 GPIO3 <- P4 GPIO19). The P4 is on V_MCU_SWTCH, which is OFF while the S3 boots (the S3 raises P4_EN_S3 afterwards), so at the S3's reset the net is driven by an unpowered pad and floats. GPIO3 is the S3's JTAG-signal-source strapping pin, which the datasheet says has no internal pull and 'must be controlled by the external circuit that cannot be in a high impedance state'. The V9 flies the same net the same way and works because EFUSE_STRAP_JTAG_SEL is unburnt (the strap is ignored). The mini reached the same conclusion and still fitted R34 (100 k to GND) on ESP_I2S_SD, with the documented reason that a floating strap is not something to carry into a new layout; the V10 is a new layout and did not carry the resistor.

**Evidence.** $S/out/live-v10/netlist.xml: ESP_I2S_SD: U15.8[GPIO3_8] U17.20[GPIO19_20] (no other member). $S/out/live-mini/netlist.xml: ESP_I2S_SD: R34.1 U15.8[GPIO3_8] U32.18[GPIO13_18]; R34 = 100 k, R34.2 = GND. $S/out/v9/netlist.xml: ESP_SDO: U15.8[GPIO3_8] U17.20[GPIO19_20]. S3 datasheet Table 3-1: 'GPIO3 Floating'; sec 3.4: 'This pin does not have any internal pull resistors and the strapping value must be controlled by the external circuit that cannot be in a high impedance state'; Table 2-1 row 8: GPIO3 at reset IE only. P4 datasheet Table 2-1: GPIO19 in VDD_IO_0 (VDDPST_1 pin 9 = V_MCU_SWTCH per live netlist U17.9). $S/live/hardware/rocket-computer-mini/README.md lines 133-135 and pin-budget.md lines 213-217, 233-238: R34's rationale, 'rocket-computer flies the same net on GPIO3 with no pull at all ... It is fixed because a floating input on a strapping pad is not something to carry into a layout'.

**Consequence.** Harmless today: with EFUSE_STRAP_JTAG_SEL = 0 the ROM ignores GPIO3, and a floating input on an RTC-domain pad costs microamps. The risk is a latent one (any future eFuse work that enables the strap makes the OC's JTAG/USB-JTAG source depend on noise), plus the parity asymmetry with the mini, which the V10 is otherwise being aligned to part-for-part.

**Fix.** Add a 100 k pull-down from ESP_I2S_SD to GND on the V10 (the mini's R34), placed at the S3 end. It does not load the I2S data line (the P4 drives it push-pull) and gives GPIO3 a clean 0 at every S3 reset. Record in the same breath that EFUSE_STRAP_JTAG_SEL must stay unburnt on the OC (and EFUSE_JTAG_SEL_ENABLE on the P4, see -8).

#### 14. [Note] Question: the P4 boot NOR U16 is powered from the always-on +3V3 while its IO domain (VDDO_FLASH = VDD_LDO = V_MCU_SWTCH via ~3 ohm) is switched; the mini deliberately put the FC's NOR on the switched rail

- **Refs:** U16 W25Q128JVYIQ (VCC B2), R46 10 k, U17 pins 27-33/30/71/75; mini U33
- **Nets:** +3V3, VDDO_FLASH, FLASH_CS, V_MCU_SWTCH
- **Reviewer:** `v10-mcu-headers-4`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** U16's VCC is on +3V3 (always on), while the P4's flash pads (FLASH_CS/CK/D/Q/WP/HD, VDDPST_3 pin 30) are powered from VDDO_FLASH = the P4's flash LDO output, which in the default 3.3 V configuration is VDD_LDO (pin 75 = V_MCU_SWTCH) through ~3 ohm. Levels are compatible (both 3.3 V) and the V9 is wired identically and works, so this is not a defect. But whenever the OC has the FC rail off, U16 stays powered with CLK/DI/WP/HOLD floating (only /CS is held high by R46 10 k to +3V3) and R46 back-feeds ~0.3 mA into the unpowered FLASH_CS pad and thence into V_MCU_SWTCH's quick-output-discharge. The mini's FC NOR U33 sits on V_MCU_SWTCH instead, so nothing is powered or back-fed while the FC is off. The brief asks for a reason for every difference from a working design; here the V10 keeps the V9 arrangement and differs from the mini - which is the intended one?

**Evidence.** $S/out/live-v10/netlist.xml: U16.B2 VCC_B2 -> +3V3; FLASH_CS: R46.1 U16.B3 U17.27; R46.2 -> +3V3; VDDO_FLASH: C83 C85 C89 U17.30[VDDPST_3] U17.71[VFB1/VO1]; U17.75 VDDPST_LDO -> V_MCU_SWTCH; U30 (TPS22810) VOUT/QOD both on V_MCU_SWTCH. $S/out/v9/netlist.xml: U16.8 VCC -> +3V3, R46 identical, VDDO_FLASH members identical. $S/out/live-mini/netlist.xml V_MCU_SWTCH members include U33.B2[VCC_B2] (FC NOR) and U16.1[VCC] (NAND); the mini's OC NOR U13.B2 is on +3V3. P4 datasheet v0.7 sec 3.2 'ESP32-P4 supplies power to flash via VDDO_FLASH, which outputs 3.3 V by default'; Table 5-3 'VDDO_FLASH powered by VDD_LDO via R_VFB 3 ohm for 3.3 V flash'; Table 2-1 row 27 FLASH_CS 'Dedicated Output, VDD_FLASHIO'.

**Consequence.** Functionally fine (V9 proves it). Costs: a few tens of uA of NOR standby plus whatever the floating inputs draw from +3V3 while the FC is off (the rail the supercap must carry), a small sneak current into the off V_MCU_SWTCH domain, and a parity difference with the mini that firmware/bring-up notes will otherwise trip over.

**Fix.** No change if the owner wants V9 identity. If parity with the mini is preferred, move U16 VCC (and R46's pull-up) to V_MCU_SWTCH - a two-wire change that the mini has already drawn - and keep the decoupling at the WLCSP. Either way, record the choice in central_processing_p4.kicad_sch next to U16.

#### 15. [Note] No series resistors on the four-wire I2S link on the V9, the V10 or the mini; the OTA-flip contention (both ends driving BCLK/WS/SD) has no hardware current limit - design discussion, not a defect

- **Refs:** U17 GPIO18/19/20/21 <-> U15 GPIO1/3/4/2; mini U32 GPIO18/13/14/21 <-> U15 GPIO1/3/4/2
- **Nets:** ESP_I2S_WS ESP_I2S_SD ESP_I2S_FSYNC ESP_I2S_BCLK
- **Reviewer:** `v10-mcu-headers-6`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** All four link nets are exactly two nodes (P4 pad, S3 pad) on the V10, as on the V9 (old names ESP_CS/SDO/SDI/SCLK) and the mini (which adds only the R34 pull-down on SD). Directions: BCLK, WS, SD and FSYNC are all P4 -> S3 in normal operation (S3 is the I2S slave RX). The known firmware item (i2s_del_channel() leaving BCLK/WS/DOUT driven across the OTA role flip) means both pads can be outputs for a while; with no series element the contention current is limited only by the pads' own drive (tens of mA per pin). Same on every board, so no parity difference; the V9 has survived its OTA sessions. A hardware limit (e.g. 33-100 ohm in each line) would be a design change and the decisions say those are discussed, not drawn.

**Evidence.** $S/out/live-v10/netlist.xml: ESP_I2S_BCLK: U15.7 U17.23; ESP_I2S_FSYNC: U15.9 U17.22; ESP_I2S_SD: U15.8 U17.20; ESP_I2S_WS: U15.6 U17.19. $S/out/v9/netlist.xml: ESP_CS/ESP_SCLK/ESP_SDI/ESP_SDO with the same two members each. $S/out/live-mini/netlist.xml: ESP_I2S_BCLK: U15.7 U32.27; ESP_I2S_SD: R34.1 U15.8 U32.18; ESP_I2S_WS: U15.6 U32.24; ESP_I2S_FSYNC: U15.9 U32.19. board_v9.h (FC and OC) I2S blocks: pin numbers 21/18/19/20 and 2/1/3/4 match the netlist per net. Memory: 'I2S flip GPIO contention (FC OTA): i2s_del_channel() leaves BCLK/WS/DOUT driven'.

**Consequence.** None today beyond the firmware item already tracked. Recorded so the synthesis knows the link has no hardware protection and that the V10 matches both references.

**Fix.** Nothing to draw. If the firmware fix for the flip contention proves fragile, propose (for discussion) 33 ohm series resistors at the P4 end of BCLK/WS/SD on both boards; at 4 MHz-class bit clocks they cost nothing.

#### 16. [Note] ESD protection exists only on the USB port (CR3); J1 GNSS, J3 expansion, J4 camera, J5 LoRa, J8 battery and J2 pyro have none - identical to the fabbed V9

- **Refs:** CR3 SP0503BAHTG; J1 J2 J3 J4 J5 J8; R30/R32 1 k; daughterboard CR4/R77/R78; carriers R7/R8/R9
- **Nets:** D+ D- Net-(J6-VBUS); GNSS_RX GNSS_TX GNSS_RXD2; EXP_01..12; Net-(J4-Pad3/4); LoRa_RX LoRa_TX
- **Reviewer:** `v10-mcu-headers-7`, confidence 0.9
- **Verification:** not run (single source).

**Claim.** The only TVS on the V10 is CR3 (SP0503, pins 2/3/4 on VBUS/D+/D-). GNSS UART lines run bare from J1 to the P4; the twelve EXP lines run bare from J3 to the P4 (WORKLIST H-3/D-4: deliberately resistor-free after the high-side conversion so J3 stays usable for I2C/SPI); the camera lines have R30/R32 1 k series only; LoRa_RX/TX run bare to the S3 but the daughterboard carries CR4 (SP0503) and R77/R78 1 k at its end; both GNSS carriers carry 1 k series on pins 1/2 (and the sam10m8 on pin 5). J8 and J2 have no clamp (J2's exposed screw terminals 'have never had a clamp', WORKLIST M-9). The V9 netlist is identical on every one of these ports.

**Evidence.** $S/out/live-v10/netlist.xml: comps with TVS footprint = CR3 only; D+: CR3.3 J6.A6 J6.B6 U1.1; D-: CR3.4 ...; Net-(J6-VBUS): ... CR3.2; GNSS_RX: J1.1 U17.4 (two nodes); EXP_01: J3.3 U17.87 ... EXP_12: J3.14 U17.88 (two nodes each); Net-(J4-Pad3): J4.3 R30.2; Net-(J4-Pad4): J4.4 R32.2; LoRa_RX: J5.3 U15.15; LoRa_TX: J5.4 U15.16. $S/work/v10-mcu-headers/lora-daughterboard.xml: LoRa_RX: CR4.3 R77.1 U28.10; LoRa_TX: CR4.4 R78.1 U28.11; R77/R78 = 1 k. gnss-px1105r: R9 (J4.1<->GNSS_RX), R8 (J4.2<->GNSS_TX); gnss-sam10m8: R7/R8/R9. $S/out/v9/netlist.xml: same members for every port. WORKLIST.md H-3, M-9, D-4.

**Consequence.** No change from the working V9, so a clearance for parity; recorded because the brief asks for the ESD posture of every external interface and because the EXP header is the one port whose bare lines reach P4 strapping pads (see -2 and -8).

**Fix.** None required for V9 identity. If the owner wants to raise the posture for discussion, the cheapest step with no functional cost is a 1 k on GNSS_RXD2 on the rocket side (it is the only UART line unprotected on either end when the px1105r carrier is fitted, per prefab-review-2026-08-05).

#### 17. [Note] Keep-true constraints on the P4's eFuses: GPIO34 (JTAG-source strap, floating on EXP_11/J3.13) and GPIO5 (P4_EN_HOLD = MTDO) are only safe while EFUSE_JTAG_SEL_ENABLE stays 0; GPIO36's on-board pull-up (R50) fixes which UART_PRINT_CONTROL value silences the boot log

- **Refs:** U17 GPIO34 (pin 65), GPIO5 (pin 5), GPIO36 (pin 68), R50 100 k, R53/R54/SW2
- **Nets:** EXP_11, P4_EN_HOLD, Net-(U17A-GPIO36)
- **Reviewer:** `v10-mcu-headers-8`, confidence 0.85
- **Verification:** not run (single source).

**Claim.** (a) GPIO34 is the P4's JTAG-signal-source strap. The datasheet says it has no internal pull and must not be high-Z at reset, but per Table 3-7 it is ignored unless EFUSE_JTAG_SEL_ENABLE=1; on the V10 (and V9) it is EXP_11, bare to J3.13. (b) The P4's JTAG pad functions are GPIO2/3/4/5 = MTCK/MTDI/MTMS/MTDO; on the V10 those pins are GNSS_RXD2, GNSS_TX, GNSS_RX and P4_EN_HOLD. Default JTAG source is the USB Serial/JTAG controller, so the pads are plain GPIO - but if the JTAG pins were ever selected (EFUSE_JTAG_SEL_ENABLE=1 with GPIO34 low, or EFUSE_DIS_USB_JTAG=1), MTDO would drive P4_EN_HOLD (D9 anode -> POWER_SWITCH) and could hold the FC rail on. (c) GPIO36 is the ROM-print strap and the second boot-mode bit; R50 100 k to V_MCU_SWTCH holds it 1, so per Table 3-5 burning EFUSE_UART_PRINT_CONTROL to 1 or 3 disables UART0 ROM printing (the #725 'silence EXP_10' option), whereas 2 would leave it enabled. None of this is a board defect; it is the same on the V9. It is the eFuse rule set that #725 / board_v10.h should carry.

**Evidence.** P4 datasheet v0.7 sec 3 (Boot Configurations): JTAG signal source strap GPIO34 with EFUSE_DIS_PAD_JTAG/EFUSE_DIS_USB_JTAG/EFUSE_JTAG_SEL_ENABLE; Table 3-1 GPIO34 Floating, GPIO35 Weak pull-up, GPIO36/37/38 Floating; sec 3.4 'This pin does not have any internal pull resistors and the strapping value must be controlled by the external circuit that cannot be in a high impedance state'; Table 3-7 default row (0,0,0,Ignored) = USB Serial/JTAG controller; Table 2-3 rows 2-5: GPIO2 MTCK, GPIO3 MTDI, GPIO4 MTMS, GPIO5 MTDO (O/T); sec 2.3.5 'GPIO2, GPIO3, GPIO4, GPIO5 : JTAG interface'; Table 3-5 UART0 ROM printing vs EFUSE_UART_PRINT_CONTROL and GPIO36. $S/out/live-v10/netlist.xml: EXP_11: J3.13 U17.65; Net-(U17A-GPIO36): R50.1 U17.68, R50.2 -> V_MCU_SWTCH; Net-(U17A-GPIO35): R53.1 R54.2 U17.66 (R53 100 k to V_MCU_SWTCH, R54 1 k to SW2 -> GND); P4_EN_HOLD: D9.1 U17.5. FABRICATION-NOTES B9 and WORKLIST M-15 / #725 for the interface rule.

**Consequence.** Nothing today. A future eFuse burn done for one reason (e.g. disabling USB-JTAG for security) would make the JTAG-source strap live on a bare header pin and could route MTDO onto the FC power latch.

**Fix.** Record in board_v10.h / #725: never burn EFUSE_JTAG_SEL_ENABLE or EFUSE_DIS_USB_JTAG on the P4 (and EFUSE_STRAP_JTAG_SEL on the S3); if the UART0 boot log is to be silenced, EFUSE_UART_PRINT_CONTROL = 3 (unconditional) is the value that does not depend on a strap.


### Reverse parity: what the full computer has and the mini does not

#### 18. [Note] Mini: neither S3 has the external GPIO0 pull-up the V10 gained on 2026-09-11 (R141 10 k)

- **Refs:** Mini: U15 pin 5 (GPIO0) + SW3; U32 pin 5 (GPIO0) + SW2. V10 counterpart: R141 10 k to +3V3
- **Nets:** Net-(U15-GPIO0), Net-(U32-GPIO0)
- **Reviewer:** `v10-parity-5`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** The V10's #678 sweep added R141 10 k GPIO0→+3V3 on the S3 ('a 45 k source impedance on the boot strap is soft' in a vibration/ESD environment). The mini was fabbed six days earlier with both S3 boot straps on the internal weak pull-up only.

**Evidence.** live-mini netlist: Net-(U15-GPIO0) = SW3.2, U15.5; Net-(U32-GPIO0) = SW2.2, U32.5 — no resistor. live-v10: Net-(U15-GPIO0) = R141.1, SW3.2, U15.5; R141.2 = +3V3. ESP32-S3 datasheet Table 3-1: GPIO0 default 'Weak pull-up'. v10-power-parity §6.4 row 'S3 GPIO0 on the internal pull-up only → R141 10 k'. V9 (fabbed) also has no external pull-up and boots.

**Consequence.** A boot-mode glitch at a brownout-recovery instant could drop an S3 into download mode until power-cycled. Works today (V9 and every DevKit ship this way); a hygiene item the fleet's other board now has.

**Fix.** Next mini spin: 10 k from GPIO0 to the processor's rail beside SW3 (OC, +3V3) and SW2 (FC, V_MCU_SWTCH).

#### 19. [Note] Mini: the S3 RF supply pins (VDD3P3 2/3) have only 100 nF behind the 2 nH bead; the V10 added 10 µF (C148)

- **Refs:** Mini: C33 100 nF + L4 2 nH (OC U15 pins 2/3); C122 100 nF + L10 2 nH (FC U32 pins 2/3). V10 counterpart: C148 10 µF on Net-(U15-VDD3P3)
- **Nets:** Net-(U15-VDD3P3), Net-(U32-VDD3P3)
- **Reviewer:** `v10-parity-6`, confidence 0.8
- **Verification:** not run (single source).

**Claim.** The V10's #678 pass drew C148 10 µF on the converter side of L4 ('guide: 10 µF per RF pin'). Both of the mini's S3s carry only the 100 nF.

**Evidence.** live-mini netlist: Net-(U15-VDD3P3) = C33.1, L4.1, U15.2, U15.3; Net-(U32-VDD3P3) = C122.1, L10.1, U32.2, U32.3. live-v10: Net-(U15-VDD3P3) = C148.1 (10 µF), C33.1, L4.1, U15.2, U15.3. v10-power-parity §6.4 row 'S3 RF supply with only C33 100 nF (guide: 10 µF per RF pin) → C148 10 µF'. V9 (fabbed, working BLE) has only C33.

**Consequence.** Weaker RF supply decoupling than Espressif's guide asks for; V9 works with the same. The OC's BLE link is the mini's only radio to the app.

**Fix.** Next mini spin: 10 µF 0402 at U15 pins 2/3 and U32 pins 2/3 on the bead's converter side.

#### 20. [Note] Mini: VDD_SPI has 1 µF only on both S3s; the V10 added the guide's 100 nF (C145)

- **Refs:** Mini: C27 1 µF (OC U15 pin 29), C116 1 µF (FC U32 pin 29). V10 counterpart: C145 100 nF beside C27
- **Nets:** OC_VDD_SPI, FC_VDD_SPI
- **Reviewer:** `v10-parity-7`, confidence 0.75
- **Verification:** not run (single source).

**Claim.** #664 closed on the V10 with C145 100 nF beside C27 because the design guide asks for 1 µF + 0.1 µF at VDD_SPI (the in-package PSRAM rail on the RH2). Both mini nets are a single 1 µF.

**Evidence.** live-mini netlist: OC_VDD_SPI = C27.1, U15.29; FC_VDD_SPI = C116.1, U32.29. live-v10: OUT_VDD_SPI = C145.1, C27.1, U15.29. v10-power-parity §6.1: 'the design guide asks for 1 µF + 0.1 µF at the pin and there was no pad for the second part'. #664 comment: 'the flight-log ring lives in the in-package PSRAM, so VDD_SPI is no longer a rail nothing uses'.

**Consequence.** HF decoupling of the PSRAM rail is the 1 µF's ESL alone; V9 shipped the same and its PSRAM memtest passes. Hygiene, not a fault.

**Fix.** Next mini spin: 100 nF 0402 at pin 29 of each S3 beside the 1 µF.

#### 21. [Note] Mini: TPS62152 soft-start C44 is 390 pF where the V10 moved to 10 nF

- **Refs:** Mini: C44 390 pF (CL05C391JB5NNNC) on U18 pin 9 SS/TR. V10: C44 10 nF
- **Nets:** Net-(U18-SS-TR)
- **Reviewer:** `v10-parity-8`, confidence 0.7
- **Verification:** not run (single source).

**Claim.** The V10's #678 pass changed C44 390 pF → 10 nF ('~150 µs, startup in current limit' → ~5 ms, datasheet-class). The mini carries the V9-era 390 pF into a heavier V_BUCK (42 µF) than the V9 had.

**Evidence.** live-mini comps: C44 390 pF; live-v10 comps: C44 10 nF; v9 comps: C44 390 pF. v10-power-parity §6.4 row 'C44 soft-start 390 pF (~150 µs, startup in current limit) → 10 nF (stocked)'. TPS62150 datasheet §8.3.2/9.2.2.2.2.3: SS/TR capacitor sets the start-up slope; ISS/TR 2.5 µA. Mini V_BUCK = C135 22 + C141 10 + C53 10 = 42 µF (holdup-tps61094-rework.md line 247); 42 µF × 3.465 V / 150 µs ≈ 0.97 A — at the 1 A limit, so the buck starts in current limit.

**Consequence.** Start-up in current limit is a supported mode (the limit regulates, no hiccup), and V9 fabbed with 390 pF works; the mini's bigger output bank just makes it more certain. Cosmetic/inrush only.

**Fix.** Next mini spin: 10 nF (stocked CL05B103KB5NNNC) at C44 to match the V10.

## Open questions for the owner

These are decisions, not defects. Several are consequences of the live edits.

1. **The V10 now has no sounder at all.** The buzzer and its driver were removed in the
   live edits. Was that the intent, or a step in moving it to the other processor? The
   out computer's pin is left dangling either way and needs a no-connect flag. Related
   open work is on #1438 and #1409.
2. **The expansion header's pins 9 to 14 changed meaning.** Nothing downstream knows: not
   the firmware board header, not the fabrication notes, not the cable document. If the
   re-pin is intended, those three need to follow it in the same change.
3. **The proven antenna was replaced by the mini's chip antenna.** The V9's part is
   bench-proven; the replacement match is the mini's, and no mini has yet demonstrated it
   works. Worth recording the reason in the schematic.
4. **Should the processor's boot flash move to the switched rail?** The V10 keeps the V9
   arrangement, which back-feeds a pull-up into the switched domain while that rail is
   off; the mini deliberately put its flight computer's flash on the switched rail. Either
   is defensible. The choice should be written next to the part.
5. **The mux output lost its bulk capacitance.** On the V9 and on git HEAD there is 330 uF
   at the multiplexer output; the live board has 1 uF there, with the replacement bulk
   moved to the far side of the filter inductor. The mini keeps three ceramics at the
   output. This board draws two to three times the mini's current.

## Not reviewed

Stated plainly so the gaps are not mistaken for clearances.

- The V10-only power blocks pin by pin against datasheets: the electronic fuse, the core
  converter, the camera and servo rails, and the satellite carrier switch.
- The electrical-rule warning classification, the bill of materials against the live
  schematic, and the documentation sweep. The committed bill of materials is known to be
  fourteen designators behind.
- Placement. The board is mid-placement and every layout item belongs on #1365.
- Routing, by instruction.
