"""Spin rig controller: every part, its value, footprint, order code and pin-to-net map.

The schematic generator (make_sch.py) draws this; nothing else defines connectivity.
Conventions taken from the rocket boards: 2-pin power connectors are pin 1 GND, pin 2 positive
(Tinker-Mantis J8, Tinker-Beetle J3); passives are Yageo RC0402 / Samsung CL05 as on the Beetle.
The connectors that face the CL57T take its own pluggable 3.5 mm plugs and follow its own pin order, so every
lead to the driver is wired position for position.

HAND lists the through-hole parts, soldered by hand: they stay out of the JLC BOM and CPL.
"""
import re

SHEETS = [
    ('power', 'Power', 'Pack input, reverse-polarity switch, driver feed and current monitor, regen clamp, 5 V and 3.3 V'),
    ('mcu', 'MCU', 'ESP32-S3, USB-C, Raspberry Pi link, I2C expansion, status lights'),
    ('io', 'Driver and timing I/O', 'CL57T step/dir/enable and alarm, encoder tap, index sensor, PPS and EventA'),
]

PARTS = []


class P:
    def __init__(self, sheet, ref, lib, value, fp, pins, mpn='', mfr='', dnp=False, desc='', block=None, units=None):
        self.sheet, self.ref, self.lib, self.value, self.fp = sheet, ref, lib, value, fp
        self.pins = {str(k): v for k, v in pins.items()}
        self.mpn, self.mfr, self.dnp, self.desc = mpn, mfr, dnp, desc
        self.block = block
        self.units = units or [1]
        PARTS.append(self)


# ---------------------------------------------------------------- passives
R_SIZES = {'0402': ('Resistor_SMD:R_0402_1005Metric', 'RC0402FR-07{}L'),
           '0805': ('Resistor_SMD:R_0805_2012Metric', None)}


def _yageo(v):
    """'10k' -> '10KL', '4.7k' -> '4K7L', '100' -> '100RL', '1M' -> '1ML', '0' -> RC0402JR-070RL."""
    if v.endswith('k'):
        n = v[:-1]
        code = n.replace('.', 'K') + ('' if '.' in n else 'K')
    elif v.endswith('M'):
        n = v[:-1]
        code = n.replace('.', 'M') + ('' if '.' in n else 'M')
    else:
        code = v.replace('.', 'R') + ('' if '.' in v else 'R')
    return 'RC0402FR-07%sL' % code


def _uniroyal(v):
    """'10k' -> 0402WGF1002TCE, '100' -> 0402WGF1000TCE, '22' -> 0402WGF220JTCE, '1M' -> 0402WGF1004TCE."""
    mult = {'k': 1e3, 'M': 1e6}.get(v[-1], 1)
    ohms = float(v.rstrip('kM')) * mult
    if ohms == 0:
        return '0402WGF0000TCE'
    if ohms < 100:
        return '0402WGF%03dJTCE' % round(ohms * 10)
    e = 0
    while ohms >= 1000:
        ohms /= 10
        e += 1
    return '0402WGF%03d%dTCE' % (round(ohms), e)


def R(sheet, ref, value, a, b, block, dnp=False, size='0402', mpn=None, desc=''):
    fp = R_SIZES[size][0]
    if mpn is None:
        mpn = _uniroyal(value)
    v = value.replace('k', ' k').replace('M', ' M')
    return P(sheet, ref, 'Device:R', v, fp, {1: a, 2: b}, mpn, 'UNI-ROYAL' if mpn.startswith('0402WGF') else 'Yageo', dnp, desc, block)


CAPS = {  # value: (footprint, MPN, maker); JLC basic parts where one exists
    '12pF': ('Capacitor_SMD:C_0402_1005Metric', '0402CG120J500NT', 'FH'),
    '47pF': ('Capacitor_SMD:C_0402_1005Metric', '0402CG470J500NT', 'FH'),
    '100pF': ('Capacitor_SMD:C_0402_1005Metric', '0402CG101J500NT', 'FH'),
    '1nF': ('Capacitor_SMD:C_0402_1005Metric', '0402B102K500NT', 'FH'),
    '4.7nF': ('Capacitor_SMD:C_0402_1005Metric', '0402B472K500NT', 'FH'),
    '10nF': ('Capacitor_SMD:C_0402_1005Metric', 'CL05B103KB5NNNC', 'Samsung'),
    '100nF': ('Capacitor_SMD:C_0402_1005Metric', 'CL05B104KO5NNNC', 'Samsung'),
    '100nF/50V': ('Capacitor_SMD:C_0402_1005Metric', 'CL05B104KB54PNC', 'Samsung'),
    '1uF': ('Capacitor_SMD:C_0402_1005Metric', 'CL05A105KA5NQNC', 'Samsung'),
    '10uF': ('Capacitor_SMD:C_0402_1005Metric', 'CL05A106MQ5NUNC', 'Samsung'),
    '4.7uF/50V': ('Capacitor_SMD:C_1210_3225Metric', 'GRM32ER71H475KA88L', 'Murata'),
    '47uF/16V': ('Capacitor_SMD:C_1210_3225Metric', 'GRM32ER61C476KE15L', 'Murata'),
}


def C(sheet, ref, value, a, b, block, dnp=False, desc=''):
    fp, mpn, mfr = CAPS[value]
    v = re.sub(r'([0-9.]+)([pnu]F)(?:/(\d+)V)?', lambda m: m.group(1) + ' ' + m.group(2) + (' ' + m.group(3) + ' V' if m.group(3) else ''), value)
    return P(sheet, ref, 'Device:C', v, fp, {1: a, 2: b}, mpn, mfr, dnp, desc, block)


LED_PARTS = {'red': ('KT-0603R', 'KENTO'), 'white': ('KT-0603W', 'KENTO')}  # the JLC basic LEDs


def LED(sheet, ref, color, anode, cathode, block, desc=''):
    mpn, mfr = LED_PARTS[color]
    return P(sheet, ref, 'Device:LED', color.capitalize(), 'LED_SMD:LED_0603_1608Metric',
             {1: cathode, 2: anode}, mpn, mfr, desc=desc, block=block)


def TP(sheet, ref, net, block):
    return P(sheet, ref, 'Connector:TestPoint', net, 'TestPoint:TestPoint_Pad_D1.0mm', {1: net}, block=block)


GND = 'GND'
V5 = '+5V'
V33 = '+3V3'

# ================================================================ POWER
s = 'power'
b = 'Pack input and driver feed'
P(s, 'J101', 'Connector_Generic:Conn_01x02', 'PACK IN', 'Connector_AMASS:AMASS_XT60PW-M_1x02_P7.20mm_Horizontal',
  {1: GND, 2: 'VPACK'}, 'XT60PW-M', 'Amass',
  desc='XT60, right angle: the 6S pack, after the anti-spark switch and the 10 A fuse (pin 1 is the footprint\'s "-")',
  block=b)
P(s, 'Q101', 'Transistor_FET:Q_PMOS_GDS', 'NCE60P50K', 'Package_TO_SOT_SMD:TO-252-2',
  {1: 'PFET_G', 2: 'VPACK', 3: 'VMAIN'}, 'NCE60P50K', 'Wuxi NCE',
  desc='Reverse-polarity switch; on, it passes regen back to the pack', block=b)
R(s, 'R101', '47k', 'PFET_G', GND, b)
P(s, 'D101', 'Device:D_Zener', 'BZT52C12', 'Diode_SMD:D_SOD-123', {1: 'VMAIN', 2: 'PFET_G'}, 'BZT52C12',
  'Diodes Inc', desc='Holds Vgs at -12 V', block=b)
P(s, 'D102', 'Device:D_Zener', 'SMCJ30A', 'Diode_SMD:D_SMC', {1: 'VMAIN', 2: GND}, 'SMCJ30A', 'Littelfuse',
  desc='TVS, 30 V standoff', block=b)
P(s, 'C101', 'Device:C_Polarized', '100 uF 50 V', 'Capacitor_SMD:CP_Elec_8x10.5', {1: 'VMAIN', 2: GND},
  'RVT1H101M0810', 'Honor Elec', desc='Bulk: damps the pack lead on plug-in', block=b)
C(s, 'C102', '4.7uF/50V', 'VMAIN', GND, b)
C(s, 'C103', '4.7uF/50V', 'VMAIN', GND, b)
P(s, 'R102', 'Device:R_Shunt', '2 mΩ', 'Footprints:R_0805_Kelvin',
  {1: 'VMAIN', 2: 'SHUNT_P', 3: 'SHUNT_N', 4: 'VDRV'}, 'PA0805FRF870R002L', 'Yageo',
  desc='Driver current shunt, Kelvin sensed (the Beetle\'s R72)', block=b)
P(s, 'J103', 'Connector_Generic:Conn_01x02', 'DRIVER PWR',
  'Connector_Phoenix_MC:PhoenixContact_MC_1,5_2-G-3.5_1x02_P3.50mm_Horizontal',
  {1: 'VDRV', 2: GND}, '1844210', 'Phoenix Contact',
  desc='To the CL57T\'s P4 in its order (+Vdc, GND); 3.5 mm pluggable, 8 A', block=b)
b = 'Driver current and voltage monitor'
P(s, 'U101', 'Sensor_Energy:INA238', 'INA238AIDGSR', 'Package_SO:MSOP-10_3x3mm_P0.5mm',
  {1: GND, 2: GND, 3: 'INA_ALERT', 4: 'I2C_SDA', 5: 'I2C_SCL', 6: V33, 7: GND, 8: 'SHUNT_N', 9: 'SHUNT_N',
   10: 'SHUNT_P'}, 'INA238AIDGSR', 'TI', desc='85 V, I2C address 0x40; bus voltage read at the shunt (driver side)',
  block=b)
C(s, 'C104', '100nF', V33, GND, b)
R(s, 'R103', '10k', 'INA_ALERT', V33, b)

b = 'Regen clamp (on at 28.8 V, off at 27.7 V; worst case off stays above a full pack)'
R(s, 'R104', '200k', 'VMAIN_F', 'CLAMP_TOP', b, desc='Senses the bus after F101')
R(s, 'R126', '20k', 'CLAMP_TOP', 'CLAMP_SNS', b)
R(s, 'R105', '10k', 'CLAMP_SNS', GND, b)
R(s, 'R106', '1M', 'CLAMP_CMD', 'CLAMP_SNS', b, desc='Hysteresis')
C(s, 'C105', '1nF', 'CLAMP_SNS', GND, b)
P(s, 'U102', 'Custom:TLV3011', 'TLV3011AIDBVR', 'Package_TO_SOT_SMD:SOT-23-6',
  {1: 'CLAMP_CMD', 2: GND, 3: 'CLAMP_SNS', 4: 'CLAMP_REF', 5: 'CLAMP_REF', 6: V5}, 'TLV3011AIDBVR', 'TI', block=b)
C(s, 'C106', '100nF', V5, GND, b)
R(s, 'R107', '4.7k', 'CLAMP_CMD', V5, b)
P(s, 'U103', 'Custom:UCC27517', 'UCC27517DBVR', 'Package_TO_SOT_SMD:SOT-23-5',
  {1: V5, 2: GND, 3: 'CLAMP_CMD', 4: GND, 5: 'BRAKE_DRV'}, 'UCC27517DBVR', 'TI', block=b)
C(s, 'C107', '1uF', V5, GND, b)
R(s, 'R108', '10', 'BRAKE_DRV', 'BRAKE_G', b)
R(s, 'R109', '100k', 'BRAKE_G', GND, b)
P(s, 'Q102', 'Custom:AO4264E', 'AO4264E', 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm',
  {1: GND, 2: GND, 3: GND, 4: 'BRAKE_G', 5: 'BRAKE_SW', 6: 'BRAKE_SW', 7: 'BRAKE_SW', 8: 'BRAKE_SW'},
  'AO4264E', 'Alpha & Omega', block=b)
P(s, 'J102', 'Connector_Generic:Conn_01x02', 'BRAKE RES', 'Connector_JST:JST_VH_B2P-VH_1x02_P3.96mm_Vertical',
  {1: 'BRAKE_SW', 2: 'VMAIN'}, 'B2P-VH(LF)(SN)', 'JST', desc='6.8 Ω, 25 W or more, on the hub plate', block=b)
P(s, 'D103', 'Device:D_Schottky', 'B560C', 'Diode_SMD:D_SMC', {1: 'VMAIN', 2: 'BRAKE_SW'}, 'B560C-13-F',
  'Diodes Inc', desc='Freewheel for the resistor\'s inductance', block=b)
C(s, 'C123', '4.7uF/50V', 'VMAIN', GND, b, desc='Bus capacitor at the brake switch')
R(s, 'R110', '1k', 'BRAKE_DRV', 'BRAKE_LED', b)
LED(s, 'D104', 'red', 'BRAKE_LED', GND, b, desc='BRAKE')
R(s, 'R111', '10k', 'BRAKE_DRV', 'BRAKE_SENSE', b)
R(s, 'R112', '20k', 'BRAKE_SENSE', GND, b)

b = '5 V buck (TI TPS54560B design example at 490 kHz; starts at 18.8 V, stops at 17.1 V)'
P(s, 'F101', 'Device:Fuse', '2 A', 'Fuse:Fuse_1206_3216Metric', {1: 'VMAIN', 2: 'VMAIN_F'}, '0466002.NRHF',
  'Littelfuse', block=b)
P(s, 'U104', 'Regulator_Switching:TPS54560BDDA', 'TPS54560BDDAR',
  'Package_SO:Texas_R-PDSO-G8_EP2.95x4.9mm_Mask2.4x3.1mm',
  {1: 'BUCK_BOOT', 2: 'VMAIN_F', 3: 'BUCK_EN', 4: 'BUCK_RT', 5: 'BUCK_FB', 6: 'BUCK_COMP', 7: GND, 8: 'BUCK_SW',
   9: GND}, 'TPS54560BDDAR', 'TI', block=b)
C(s, 'C108', '4.7uF/50V', 'VMAIN_F', GND, b)
C(s, 'C109', '4.7uF/50V', 'VMAIN_F', GND, b)
C(s, 'C110', '4.7uF/50V', 'VMAIN_F', GND, b)
C(s, 'C111', '100nF/50V', 'VMAIN_F', GND, b)
R(s, 'R113', '1M', 'VMAIN_F', 'BUCK_EN', b, desc='R113 || R124 = 500 k')
R(s, 'R124', '1M', 'VMAIN_F', 'BUCK_EN', b)
R(s, 'R114', '33k', 'BUCK_EN', GND, b)
R(s, 'R115', '200k', 'BUCK_RT', GND, b, desc='490 kHz')
C(s, 'C112', '100nF', 'BUCK_BOOT', 'BUCK_SW', b)
P(s, 'L101', 'Device:L', '6.8 uH', 'Inductor_SMD:L_Sunlord_MWSA1004S', {1: 'BUCK_SW', 2: V5}, 'MWSA1004S-6R8MT',
  'Sunlord', desc='8.5 A, 12 A saturation', block=b)
P(s, 'D105', 'Device:D_Schottky', 'B560C', 'Diode_SMD:D_SMC', {1: 'BUCK_SW', 2: GND}, 'B560C-13-F', 'Diodes Inc',
  block=b)
R(s, 'R116', '15k', 'BUCK_COMP', 'BUCK_CZ', b)
C(s, 'C113', '4.7nF', 'BUCK_CZ', GND, b)
C(s, 'C114', '47pF', 'BUCK_COMP', GND, b)
R(s, 'R117', '33k', V5, 'BUCK_FB', b, desc='5.13 V with R118 + R125')
R(s, 'R118', '5.1k', 'BUCK_FB', 'BUCK_FB2', b)
R(s, 'R125', '1k', 'BUCK_FB2', GND, b)
C(s, 'C115', '47uF/16V', V5, GND, b)
C(s, 'C116', '47uF/16V', V5, GND, b)
C(s, 'C117', '47uF/16V', V5, GND, b)
R(s, 'R119', '2.2k', V5, 'LED5_A', b)
LED(s, 'D106', 'red', 'LED5_A', GND, b, desc='5V')

b = '5 V outputs, 3.3 V, and the ADC dividers'
P(s, 'F102', 'Device:Polyfuse', '1.5 A', 'Fuse:Fuse_1206_3216Metric', {1: V5, 2: '+5V_AUX'}, 'SMD1206P150TF',
  'PTTC', block=b)
P(s, 'J104', 'Connector_Generic:Conn_01x02', '5V AUX', 'Connector_JST:JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical',
  {1: GND, 2: '+5V_AUX'}, 'B2B-PH-K-S(LF)(SN)', 'JST', desc='For the V0 SDR board or the Beetle', block=b)
P(s, 'F103', 'Device:Fuse', '4 A', 'Fuse:Fuse_1206_3216Metric', {1: V5, 2: '+5V_PI'}, '0466004.NRHF',
  'Littelfuse', desc='Pi feed', block=b)
P(s, 'D107', 'Device:D_Schottky', 'B5819WS', 'Diode_SMD:D_SOD-323', {1: 'V3V3_IN', 2: V5}, 'B5819WS', 'Diodes Inc',
  block=b)
P(s, 'D108', 'Device:D_Schottky', 'B5819WS', 'Diode_SMD:D_SOD-323', {1: 'V3V3_IN', 2: 'VUSB'}, 'B5819WS',
  'Diodes Inc', desc='USB alone runs the logic for flashing', block=b)
P(s, 'U105', 'Regulator_Linear:AP2112K-3.3', 'AP2112K-3.3TRG1', 'Package_TO_SOT_SMD:SOT-23-5',
  {1: 'V3V3_IN', 2: GND, 3: 'V3V3_IN', 4: None, 5: V33}, 'AP2112K-3.3TRG1', 'Diodes Inc', block=b)
C(s, 'C118', '1uF', 'V3V3_IN', GND, b)
C(s, 'C119', '1uF', V33, GND, b)
C(s, 'C120', '10uF', V33, GND, b)
R(s, 'R120', '100k', 'VMAIN_F', 'VBUS_SENSE', b, desc='28 V reads 2.55 V')
R(s, 'R121', '10k', 'VBUS_SENSE', GND, b)
C(s, 'C121', '100nF', 'VBUS_SENSE', GND, b)
R(s, 'R122', '10k', V5, 'V5_SENSE', b)
R(s, 'R123', '10k', 'V5_SENSE', GND, b)
C(s, 'C122', '100nF', 'V5_SENSE', GND, b)

# ================================================================ MCU
s = 'mcu'
b = 'ESP32-S3'
# pins chosen by side, facing what they talk to; neighbors of the 3.3 V pins left free for their vias (U201 unrotated: 1-14 left, 15-28 bottom, 29-42 right, 43-56 top)
ESP = {1: None, 2: V33, 3: V33, 4: 'CHIP_PU', 5: 'BOOT0',
       # left, toward the power column
       6: 'ALM', 7: 'DRV_BRK', 8: 'INA_ALERT', 9: 'I2C_SDA', 10: 'I2C_SCL', 11: 'BRAKE_SENSE', 12: 'DIR',
       13: None, 14: 'VBUS_SENSE',
       # bottom, toward USB, the lights and the Pi
       15: 'V5_SENSE', 16: 'LED_FAULT', 17: 'LED_STAT', 18: 'PI_TX', 19: None, 20: V33, 21: None, 22: 'PI_RX',
       23: None, 24: None, 25: 'USB_DN_MCU', 26: 'USB_DP_MCU', 27: 'DRV_OE_N', 28: None,
       # right, toward the encoder, index and GNSS connectors (30-35 are the in-package flash)
       29: 'VDD_SPI', 30: None, 31: None, 32: None, 33: None, 34: None, 35: None, 36: 'EVT_IDX_OE_N', 37: 'EVT_MCU',
       38: 'PPS', 39: 'INDEX', 40: None, 41: 'ENC_B', 42: 'ENC_A',
       # top, toward the driver buffer and the EventA buffer
       # 45 and 47 stay free so pin 46 (3.3 V) has room for its via
       43: 'EVT_MCU_OE_N', 44: 'STEP', 45: None, 46: V33, 47: None, 48: 'ENA', 49: 'DBG_TX', 50: 'DBG_RX',
       51: None, 52: 'GPIO46', 53: 'XTAL_N', 54: 'XTAL_P', 55: V33, 56: V33, 57: GND}
P(s, 'U201', 'MCU_Espressif:ESP32-S3', 'ESP32-S3FN8', 'Footprints:IC_ESP32-S3', ESP, 'ESP32-S3FN8', 'Espressif',
  desc='8 MB flash in the package; the rocket boards\' footprint', block=b)
P(s, 'Y201', 'Device:Crystal_GND24', '40 MHz', 'Footprints:XTAL_ECS-400-10-37B2-CKY-TR',
  {1: 'XTAL_N', 2: GND, 3: 'XTAL_P_L', 4: GND}, 'ECS-400-10-37B2-CKY-TR', 'ECS', block=b)
P(s, 'L201', 'Device:L', '24 nH', 'Inductor_SMD:L_0402_1005Metric', {1: 'XTAL_P_L', 2: 'XTAL_P'}, 'LQW15AN24NH00D',
  'Murata', block=b)
C(s, 'C201', '12pF', 'XTAL_P_L', GND, b)
C(s, 'C202', '12pF', 'XTAL_N', GND, b)
R(s, 'R201', '10k', V33, 'CHIP_PU', b)
C(s, 'C203', '1uF', 'CHIP_PU', GND, b)
R(s, 'R202', '10k', V33, 'BOOT0', b)
R(s, 'R203', '10k', 'GPIO46', GND, b)
C(s, 'C204', '10uF', V33, GND, b)
C(s, 'C205', '100nF', V33, GND, b)
C(s, 'C206', '100nF', V33, GND, b)
C(s, 'C207', '1uF', 'VDD_SPI', GND, b)
C(s, 'C208', '100nF', V33, GND, b)
C(s, 'C209', '100nF', V33, GND, b)
C(s, 'C210', '10nF', V33, GND, b)
C(s, 'C211', '10uF', V33, GND, b)
P(s, 'SW201', 'Switch:SW_Push', 'BOOT', 'Button_Switch_SMD:SW_Push_1P1T_XKB_TS-1187A', {1: 'BOOT0', 2: GND},
  'TS-1187A-B-A-B', 'XKB', block=b)
P(s, 'SW202', 'Switch:SW_Push', 'RESET', 'Button_Switch_SMD:SW_Push_1P1T_XKB_TS-1187A', {1: 'CHIP_PU', 2: GND},
  'TS-1187A-B-A-B', 'XKB', block=b)

b = 'USB-C (the Beetle\'s J6 circuit)'
P(s, 'J201', 'Custom:USB4110GFA', 'USB4110GFA', 'Footprints:GCT_USB4110GFA',
  {'A1/B12': GND, 'B1/A12': GND, 'S1': GND, 'S2': GND, 'S3': GND, 'S4': GND, 'A4/B9': 'VUSB', 'B4/A9': 'VUSB',
   'A5': 'USB_CC1', 'B5': 'USB_CC2', 'A6': 'USB_DP', 'B6': 'USB_DP', 'A7': 'USB_DN', 'B7': 'USB_DN', 'A8': None,
   'B8': None}, 'USB4110-GF-A', 'GCT', block=b)
R(s, 'R204', '5.1k', 'USB_CC1', GND, b)
R(s, 'R205', '5.1k', 'USB_CC2', GND, b)
P(s, 'CR201', 'Custom:SP0503BAHTG', 'SP0503BAHTG', 'Footprints:TVS-SP05_LTF',
  {1: GND, 2: 'VUSB', 3: 'USB_DP', 4: 'USB_DN'}, 'SP0503BAHTG', 'Littelfuse', block=b)
R(s, 'R206', '22', 'USB_DP', 'USB_DP_MCU', b)
R(s, 'R207', '22', 'USB_DN', 'USB_DN_MCU', b)
C(s, 'C212', '1uF', 'VUSB', GND, b)

b = 'Pi link, expansion, lights, test points'
# The board stacks on the Pi 4B like a HAT (the Pi's port end sticks out past the bottom edge): 5 V in on pins 2
# and 4, its UART0 on 8 (TXD) and 10 (RXD), and every ground pin. The 3.3 V pins and the GPIOs stay unconnected.
PI_HDR = {n: None for n in range(1, 41)}
PI_HDR.update({2: '+5V_PI', 4: '+5V_PI', 8: 'PI_RX_H', 10: 'PI_TX_H'})
PI_HDR.update({n: GND for n in (6, 9, 14, 20, 25, 30, 34, 39)})
P(s, 'J202', 'Connector_Generic:Conn_02x20_Odd_Even', 'PI 4B', 'Connector_PinSocket_2.54mm:PinSocket_2x20_P2.54mm_Vertical',
  PI_HDR, 'HC-PM254-8.5H-2x20PZ', 'Hong Cheng',
  desc='2x20 socket, 8.5 mm, on the underside: stacks on the Pi 4B\'s header (11 mm M2.5 standoffs)', block=b)
R(s, 'R211', '100', 'PI_RX_H', 'PI_RX', b)
R(s, 'R212', '100', 'PI_TX', 'PI_TX_H', b)
P(s, 'J203', 'Connector_Generic:Conn_01x04', 'QWIIC', 'Connector_JST:JST_SH_BM04B-SRSS-TB_1x04-1MP_P1.00mm_Vertical',
  {1: GND, 2: V33, 3: 'I2C_SDA', 4: 'I2C_SCL'}, 'BM04B-SRSS-TB(LF)(SN)', 'JST', block=b)
R(s, 'R213', '4.7k', 'I2C_SDA', V33, b)
R(s, 'R214', '4.7k', 'I2C_SCL', V33, b)
R(s, 'R208', '1k', V33, 'LED_PWR_A', b)
LED(s, 'D201', 'red', 'LED_PWR_A', GND, b, desc='3V3')
R(s, 'R209', '100', 'LED_STAT', 'LED_STAT_A', b)
LED(s, 'D202', 'white', 'LED_STAT_A', GND, b, desc='STATUS')
R(s, 'R210', '1k', 'LED_FAULT', 'LED_FAULT_A', b)
LED(s, 'D203', 'red', 'LED_FAULT_A', GND, b, desc='FAULT')
TP(s, 'TP201', 'DBG_TX', b)
TP(s, 'TP202', 'DBG_RX', b)
TP(s, 'TP203', V33, b)
TP(s, 'TP204', V5, b)
TP(s, 'TP205', GND, b)
TP(s, 'TP206', GND, b)
for r in ('H3', 'H5', 'H6', 'H7'):          # the Pi 4B's 58 x 49 mm pattern: the board mounts on the Pi
    P(s, r, 'Mechanical:MountingHole', 'M2.5', 'MountingHole:MountingHole_2.7mm_M2.5', {}, block='Mechanical')

# ================================================================ IO
s = 'io'
b = 'CL57T step, direction, enable (5 V, through the optocouplers)'
P(s, 'U301', '74xx:74HCT244', 'SN74ACT244PWR', 'Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm',
  {1: 'DRV_OE_N', 2: 'STEP', 4: 'DIR', 6: GND, 8: GND, 18: 'PUL_O', 16: 'DIR_O', 14: None, 12: None,
   19: GND, 17: 'ENA', 15: GND, 13: GND, 11: GND, 3: 'ENA_O', 5: None, 7: None, 9: None, 10: GND, 20: V5},
  'SN74ACT244PWR', 'TI', desc='±24 mA at 5 V; TTL inputs take 3.3 V', block=b)
C(s, 'C301', '100nF', V5, GND, b)
R(s, 'R301', '10k', 'DRV_OE_N', V33, b, desc='STEP/DIR off through reset')
R(s, 'R302', '4.7k', 'STEP', GND, b, desc='Holds STEP low against GPIO39\'s reset pull-up')
R(s, 'R303', '10k', 'DIR', GND, b)
R(s, 'R304', '10k', 'ENA', V33, b, desc='Fit the one that disables the driver in reset (bench check)')
R(s, 'R305', '10k', 'ENA', GND, b, dnp=True)
R(s, 'R306', '33', 'PUL_O', 'PUL_P', b)
R(s, 'R307', '33', 'DIR_O', 'DIR_P', b)
R(s, 'R308', '33', 'ENA_O', 'ENA_P', b)
P(s, 'J301', 'Connector_Generic:Conn_01x09', 'CL57T CONTROL',
  'Connector_Phoenix_MC:PhoenixContact_MC_1,5_9-G-3.5_1x09_P3.50mm_Horizontal',
  {1: 'PUL_P', 2: GND, 3: 'DIR_P', 4: GND, 5: 'ENA_P', 6: GND, 7: 'ALM_P', 8: 'BRK_P', 9: GND}, '1844281',
  'Phoenix Contact', desc='The CL57T\'s P1 in its order: PUL+ PUL- DIR+ DIR- ENA+ ENA- ALM BRK COM-', block=b)
# ALM and BRK are open-collector optocoupler outputs to the driver's COM-, which is this board's ground
R(s, 'R309', '4.7k', 'ALM_P', V33, b)
R(s, 'R310', '1k', 'ALM_P', 'ALM', b)
C(s, 'C302', '1nF', 'ALM', GND, b)
R(s, 'R311', '4.7k', 'BRK_P', V33, b)
R(s, 'R312', '1k', 'BRK_P', 'DRV_BRK', b, desc='BRK: the driver is holding the motor (the CL57T has no in-position output)')
C(s, 'C303', '1nF', 'DRV_BRK', GND, b)

b = 'Encoder pass-through and tap (the receiver listens in parallel with the driver)'
P(s, 'U302', 'Interface:AM26LV32xD', 'AM26LV32EIDR', 'Package_SO:SOIC-16_3.9x9.9mm_P1.27mm',
  {2: 'ENC_A_P', 1: 'ENC_A_N', 3: 'ENC_A', 6: 'ENC_B_P', 7: 'ENC_B_N', 5: 'ENC_B', 10: None, 9: None,
   11: None, 14: None, 15: None, 13: None, 4: V33, 12: GND, 8: GND, 16: V33}, 'AM26LV32EIDR', 'TI',
  desc='Fail-safe on open inputs, no termination of its own; the motor\'s encoder has no index', block=b)
C(s, 'C304', '100nF', V33, GND, b)
R(s, 'R313', '120', 'ENC_A_P', 'ENC_A_N', b, dnp=True)
R(s, 'R314', '120', 'ENC_B_P', 'ENC_B_N', b, dnp=True)
# J302 takes the kit's encoder extension lead; J305 carries the same six wires on to the driver's P2. J302 is
# numbered from the bottom so that, read from the top, both say EB+ EB- EA+ EA- VCC EGND, as the driver does,
# and the six traces between them nest around the corner without crossing.
P(s, 'J302', 'Connector_Generic:Conn_01x06', 'ENCODER IN',
  'Connector_Phoenix_MC:PhoenixContact_MC_1,5_6-G-3.5_1x06_P3.50mm_Horizontal',
  {6: 'ENC_B_P', 5: 'ENC_B_N', 4: 'ENC_A_P', 3: 'ENC_A_N', 2: 'ENC_VCC', 1: 'ENC_GND'}, '1844252',
  'Phoenix Contact', desc='From the motor\'s encoder extension lead (top to bottom: EB+ EB- EA+ EA- VCC EGND)', block=b)
P(s, 'J305', 'Connector_Generic:Conn_01x06', 'ENCODER OUT',
  'Connector_Phoenix_MC:PhoenixContact_MC_1,5_6-G-3.5_1x06_P3.50mm_Horizontal',
  {1: 'ENC_B_P', 2: 'ENC_B_N', 3: 'ENC_A_P', 4: 'ENC_A_N', 5: 'ENC_VCC', 6: 'ENC_GND'}, '1844252',
  'Phoenix Contact', desc='To the CL57T\'s P2 in its order: EB+ EB- EA+ EA- VCC (the driver\'s 5 V) EGND', block=b)
R(s, 'R316', '0', 'ENC_GND', GND, b, desc='Open it if the encoder ground is not the pack ground')

b = 'Index sensor (slotted optical or Hall)'
P(s, 'J303', 'Connector_Generic:Conn_01x04', 'INDEX', 'Connector_JST:JST_PH_B4B-PH-K_1x04_P2.00mm_Vertical',
  {1: GND, 2: 'IDX_SIG', 3: 'IDX_LED', 4: V5}, 'B4B-PH-K-S(LF)(SN)', 'JST',
  desc='GND, signal (open collector), LED anode (through 330 Ω), 5 V', block=b)
R(s, 'R317', '330', V5, 'IDX_LED', b)
R(s, 'R318', '4.7k', 'IDX_SIG', V33, b)
R(s, 'R319', '1k', 'IDX_SIG', 'IDX_IN', b)
C(s, 'C305', '100pF', 'IDX_IN', GND, b)
P(s, 'U303', '74xGxx:74LVC1G17', 'SN74LVC1G17DBVR', 'Package_TO_SOT_SMD:SOT-23-5',
  {1: None, 2: 'IDX_IN', 3: GND, 4: 'INDEX', 5: V33}, 'SN74LVC1G17DBVR', 'TI', desc='5 V tolerant input',
  block=b)
C(s, 'C306', '100nF', V33, GND, b)

b = 'GNSS timing: PPS in, EventA out (the index, or the MCU)'
P(s, 'U304', '74xGxx:74LVC2G125', 'SN74LVC2G125DCTR', 'Package_SO:SSOP-8_2.95x2.8mm_P0.65mm',
  {1: 'EVT_IDX_OE_N', 2: 'INDEX', 6: 'EVT_IDX_Y', 7: 'EVT_MCU_OE_N', 5: 'EVT_MCU', 3: 'EVT_MCU_Y', 4: GND, 8: V33},
  'SN74LVC2G125DCTR', 'TI', desc='Both buffers off through reset; firmware enables one, only while PPS runs',
  block=b, units=[1, 2, 3])
R(s, 'R320', '100', 'EVT_IDX_Y', 'EVENTA_OUT', b)
R(s, 'R321', '100', 'EVT_MCU_Y', 'EVENTA_OUT', b)
R(s, 'R322', '10k', 'EVT_IDX_OE_N', V33, b)
R(s, 'R323', '10k', 'EVT_MCU_OE_N', V33, b)
C(s, 'C307', '100nF', V33, GND, b)
P(s, 'J304', 'Connector_Generic:Conn_01x03', 'GNSS', 'Connector_JST:JST_PH_B3B-PH-K_1x03_P2.00mm_Vertical',
  {1: 'PPS_IN', 2: GND, 3: 'EVENTA_OUT'}, 'B3B-PH-K-S(LF)(SN)', 'JST',
  desc='mosaic-go pins 8, 9, 10 (the 1 kΩ resistors stay at the receiver end)', block=b)
R(s, 'R324', '100', 'PPS_IN', 'PPS', b)
R(s, 'R325', '100k', 'PPS_IN', GND, b)
R(s, 'R326', '100k', 'EVENTA_OUT', GND, b, desc='EventA low when both buffers are off')
TP(s, 'TP301', 'PUL_P', b)
TP(s, 'TP302', 'PPS', b)
TP(s, 'TP303', 'ENC_A', b)
TP(s, 'TP304', 'EVENTA_OUT', b)
TP(s, 'TP305', 'INDEX', b)
TP(s, 'TP306', GND, b)

POWER_NETS = {GND, V5, V33}
# through-hole: soldered by hand, not by JLC
HAND = ('J101', 'J102', 'J103', 'J104', 'J202', 'J301', 'J302', 'J303', 'J304', 'J305')
PWR_FLAG_NETS = {'power': [GND, 'VPACK', 'VMAIN', 'VMAIN_F', V5, 'V3V3_IN'], 'mcu': ['VUSB', 'VDD_SPI'], 'io': []}


def check():
    refs = [p.ref for p in PARTS]
    dup = {r for r in refs if refs.count(r) > 1}
    assert not dup, dup
    nets = {}
    for p in PARTS:
        for pin, n in p.pins.items():
            if n:
                nets.setdefault(n, []).append((p.ref, pin))
    single = {n: v for n, v in nets.items() if len(v) < 2}
    return nets, single


if __name__ == '__main__':
    nets, single = check()
    print(len(PARTS), 'parts', len(nets), 'nets')
    print('single-pin nets:', single)
