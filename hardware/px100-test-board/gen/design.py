"""PX100 test board: every part on the generated sheets, its value, footprint, order code and pin-to-net map.

make_sch.py draws the 'power', 'esp32' and 'pxio' sheets from this file. The fourth sheet, receiver.kicad_sch
("RF chain and PX100"), is drawn by hand: it started as px100-module's receiver sheet (gen/edit_receiver.py made
the one-time changes) and is not regenerated. RECEIVER_NETS lists the nets it shares with the generated sheets, so
they get global labels here too.

Parts follow the boards they come from:
- the PX100, its RF chain, the ADP7142, LEDs and most passives: gnss-px100-18mm-hv (V1 parts already on hand);
- ESP32-S3FN8 with its crystal circuit, USB-C, ESD and the AP2112K: spin-rig-controller (the Beetle's circuits);
- the FSUSB63 USB mux and its slide switch: Tinker-Beetle's U1/S1;
- CP2102N: Silicon Labs' self-powered 3.3 V connection (CP2102N data sheet rev 1.1, figures 2.3 and 2.6).
"""
import re

SHEETS = [
    ('power', 'USB-C, USB switch and power', 'USB-C, the ESP32 / PX100 USB switch, 3.3 V for the ESP32 side and for the PX100'),
    ('esp32', 'ESP32-S3', 'ESP32-S3FN8 with every PX100 pin on its own GPIO, buttons, status light, spare-pin header'),
    ('pxio', 'PX100 USB bridge and I/O', 'CP2102N USB-UART bridge, PX100 reset/boot control, 1 k links to the ESP32, probe header'),
]
STATIC_SHEETS = [('receiver', 'RF chain and PX100', 'Active L1/L5 antenna input, bias tee, SkyTraq L1/L5 split, PX100 and its core')]

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
YAGEO = {'10k': 'RC0402FR-0710KL', '1k': 'RC0402FR-071KL', '100k': 'RC0402FR-07100KL', '22': 'RC0402FR-0722RL',
         '22.1k': 'RC0402FR-0722K1L', '47.5k': 'RC0402FR-0747K5L', '5.1k': 'RC0402FR-075K1L'}


def _uniroyal(v):
    """'100' -> 0402WGF1000TCE, '0' -> 0402WGF0000TCE."""
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


def R(sheet, ref, value, a, b, block, dnp=False, desc=''):
    """0402. Yageo RC0402 where gnss-px100-18mm-hv used it (10 k, 1 k), UNI-ROYAL otherwise."""
    mpn = YAGEO.get(value) or _uniroyal(value)
    v = value.replace('k', ' k').replace('M', ' M')
    if value == '0':
        v = '0R'
    return P(sheet, ref, 'Device:R', v, 'Resistor_SMD:R_0402_1005Metric', {1: a, 2: b}, mpn,
             'UNI-ROYAL' if mpn.startswith('0402WGF') else 'Yageo', dnp, desc, block)


CAPS = {  # value: (footprint, MPN, maker); the gnss-px100-18mm-hv V1 parts where it has one
    '12pF': ('Capacitor_SMD:C_0402_1005Metric', '0402CG120J500NT', 'FH'),
    '2.2nF': ('Capacitor_SMD:C_0402_1005Metric', 'CC0402KRX7R9BB222', 'Yageo'),
    '10nF': ('Capacitor_SMD:C_0402_1005Metric', 'CL05B103KB5NNNC', 'Samsung'),
    '100nF': ('Capacitor_SMD:C_0402_1005Metric', 'CL05B104KO5NNNC', 'Samsung'),
    '1uF': ('Capacitor_SMD:C_0402_1005Metric', 'CL05A105KO5NNNC', 'Samsung'),
    '2.2uF': ('Capacitor_SMD:C_0402_1005Metric', 'CL05A225MQ5NSNC', 'Samsung'),
    '4.7uF': ('Capacitor_SMD:C_0402_1005Metric', 'CL05A475MP5NRNC', 'Samsung'),
    '10uF': ('Capacitor_SMD:C_0402_1005Metric', 'CL05A106MQ5NUNC', 'Samsung'),
    '22uF': ('Capacitor_SMD:C_0805_2012Metric', 'CL21A226MOQNNNE', 'Samsung'),
}


def C(sheet, ref, value, a, b, block, dnp=False, desc=''):
    fp, mpn, mfr = CAPS[value]
    v = re.sub(r'([0-9.]+)([pnu]F)', r'\1 \2', value)
    return P(sheet, ref, 'Device:C', v, fp, {1: a, 2: b}, mpn, mfr, dnp, desc, block)


def LED(sheet, ref, anode, cathode, block, desc=''):
    """The carrier's green 0402 (XL-1005UGC)."""
    return P(sheet, ref, 'Device:LED', 'Green', 'LED_SMD:LED_0402_1005Metric', {1: cathode, 2: anode},
             'XL-1005UGC', 'XINGLIGHT', desc=desc, block=block)


def TP(sheet, ref, net, block):
    return P(sheet, ref, 'Connector:TestPoint', net, 'TestPoint:TestPoint_Pad_D1.0mm', {1: net}, block=block)


def BUTTON(sheet, ref, value, net, block, desc=''):
    return P(sheet, ref, 'Switch:SW_Push', value, 'Button_Switch_SMD:SW_Push_1P1T_XKB_TS-1187A', {1: net, 2: GND},
             'TS-1187A-B-A-B', 'XKB', desc=desc, block=block)


GND = 'GND'
VBUS = 'VBUS'
V33 = '+3V3'            # the PX100's rail (U104 through FL101), the name the receiver sheet uses
V33M = '+3V3_MCU'       # the ESP32, the CP2102N and the USB mux (U103)

# ================================================================ USB-C, USB switch and power
s = 'power'
b = 'USB-C (the Beetle\'s J6 circuit, as on the spin-rig controller)'
P(s, 'J101', 'Custom:USB4110GFA', 'USB4110GFA', 'Footprints:GCT_USB4110GFA',
  {'A1/B12': GND, 'B1/A12': GND, 'S1': GND, 'S2': GND, 'S3': GND, 'S4': GND, 'A4/B9': VBUS, 'B4/A9': VBUS,
   'A5': 'USB_CC1', 'B5': 'USB_CC2', 'A6': 'USB_DP', 'B6': 'USB_DP', 'A7': 'USB_DN', 'B7': 'USB_DN', 'A8': None,
   'B8': None}, 'USB4110-GF-A', 'GCT', desc='The board\'s only power input', block=b)
R(s, 'R101', '5.1k', 'USB_CC1', GND, b)
R(s, 'R102', '5.1k', 'USB_CC2', GND, b)
P(s, 'CR101', 'Custom:SP0503BAHTG', 'SP0503BAHTG', 'Footprints:TVS-SP05_LTF',
  {1: GND, 2: VBUS, 3: 'USB_DP', 4: 'USB_DN'}, 'SP0503BAHTG', 'Littelfuse', block=b)
C(s, 'C101', '1uF', VBUS, GND, b)

b = 'USB switch: S101 picks the ESP32 (port 2) or the PX100\'s CP2102N (port 3); Tinker-Beetle U1/S1'
P(s, 'U101', 'Custom:FSUSB63UMX', 'FSUSB63UMX', 'Footprints:QFN40P180X180X55-12N',
  {1: 'USB_DP', 2: 'USB_DN', 3: GND, 4: 'USB_SEL', 5: None, 6: None, 7: 'ESP_DN', 8: 'ESP_DP', 9: 'CP_DN',
   10: 'CP_DP', 11: 'USB_SEL1', 12: 'USB_MUX_VCC'}, 'FSUSB63UMX', 'onsemi',
  desc='SEL1 high: SEL0 low = port 2 (ESP32), SEL0 high = port 3 (CP2102N)', block=b)
R(s, 'R103', '100k', 'USB_SEL', V33M, b, desc='SEL0 high (PX100 side) when S101 is open')
R(s, 'R104', '100k', 'USB_SEL1', V33M, b)
R(s, 'R105', '22', 'USB_MUX_VCC', V33M, b)
C(s, 'C102', '100nF', 'USB_MUX_VCC', GND, b)
P(s, 'S101', 'Custom:JS202011JCQN', 'JS202011JCQN', 'Footprints:SW_JS202011JCQN',
  {1: None, 2: None, 3: None, 5: None, 6: 'USB_SEL', 7: GND}, 'JS202011JCQN', 'C&K',
  desc='USB to the ESP32 (SEL0 to GND) or to the PX100 bridge (open)', block=b)

b = '3.3 V for the ESP32, the bridge and the mux (the spin-rig controller\'s AP2112K)'
P(s, 'U103', 'Regulator_Linear:AP2112K-3.3', 'AP2112K-3.3TRG1', 'Package_TO_SOT_SMD:SOT-23-5',
  {1: VBUS, 2: GND, 3: VBUS, 4: None, 5: V33M}, 'AP2112K-3.3TRG1', 'Diodes Inc', desc='600 mA', block=b)
C(s, 'C103', '1uF', VBUS, GND, b)
C(s, 'C104', '1uF', V33M, GND, b)
C(s, 'C105', '10uF', V33M, GND, b)
R(s, 'R106', '10k', V33M, 'LED_MCU_A', b)
LED(s, 'D101', 'LED_MCU_A', GND, b, desc='3V3 MCU')

b = '3.3 V for the PX100 and the antenna (gnss-px100-18mm-hv U2 circuit: ADP7142, FL2 bead); the ESP32 can switch it'
P(s, 'U104', 'Custom:ADP7142ARDZ', 'ADP7142ARDZ-3.3', 'Footprints:SOIC127P600X175-9N',
  {1: 'V_LDO_GNSS', 2: 'V_LDO_GNSS', 3: 'V_LDO_GNSS', 4: GND, 5: 'GNSS_EN', 6: 'GNSS_SS', 7: VBUS, 8: VBUS, 9: GND},
  'ADP7142ARDZ-3.3', 'Analog Devices', desc='200 mA: the PX100 (120 mA) and an active antenna', block=b)
C(s, 'C106', '4.7uF', VBUS, GND, b, desc='USB allows 10 uF on VBUS: C101 + C103 + C106 + C107 = 6.8 uF')
C(s, 'C107', '100nF', VBUS, GND, b)
C(s, 'C108', '2.2nF', 'GNSS_SS', GND, b, desc='Soft start, as the carrier')
R(s, 'R107', '10k', 'GNSS_EN', V33M, b, desc='On unless the ESP32 pulls GNSS_EN low (power-cycles the PX100)')
C(s, 'C109', '22uF', 'V_LDO_GNSS', GND, b)
P(s, 'FL101', 'Device:FerriteBead_Small', 'BLM18PG471SN1D', 'Footprints:BEADC1608X95N', {1: 'V_LDO_GNSS', 2: V33},
  'BLM18PG471SN1D', 'Murata', block=b)
C(s, 'C110', '22uF', V33, GND, b)
C(s, 'C111', '1uF', V33, GND, b)
R(s, 'R108', '10k', V33, 'LED_GNSS_A', b)
LED(s, 'D102', 'LED_GNSS_A', GND, b, desc='3V3 PX100')

b = 'Switch readback, test points and mounting holes'
R(s, 'R109', '1k', 'USB_SEL', 'USB_SEL_MCU', b, desc='The ESP32 reads which side has the USB port')
TP(s, 'TP101', VBUS, b)
TP(s, 'TP102', V33M, b)
TP(s, 'TP103', V33, b)
TP(s, 'TP104', GND, b)
TP(s, 'TP105', GND, b)
for r in ('H1', 'H2', 'H3', 'H4'):
    P(s, r, 'Mechanical:MountingHole', 'M3', 'MountingHole:MountingHole_3.2mm_M3', {}, block=b)

# ================================================================ ESP32-S3
s = 'esp32'
b = 'ESP32-S3FN8 (the spin-rig controller\'s circuit; no antenna: the radio stays off)'
# ADC1 (GPIO1-10) takes the nine PX100 GPIOs, ADC2 (GPIO11-16) the four reserved pins and the two bias monitors, so
# every PX100 line can also be read as a voltage. None of them is a strapping pin. RSTN and BOOT_SEL sit on GPIO38/39,
# outside GPIO1-14, which can glitch low while the S3 powers up. GPIO3 (JTAG strap) stays free.
ESP = {1: None, 2: V33M, 3: V33M, 4: 'CHIP_PU', 5: 'BOOT0',
       6: 'PX_GPIO0_MCU', 7: 'PX_GPIO2_MCU', 8: None, 9: 'PX_GPIO6_MCU', 10: 'PX_GPIO9_MCU', 11: 'PX_GPIO10_MCU',
       12: 'PX_GPIO11_MCU', 13: 'PX_GPIO13_MCU', 14: 'PX_GPIO14_MCU', 15: 'PX_GPIO15_MCU',
       16: 'PX_NC8_MCU', 17: 'PX_NC14_MCU', 18: 'PX_NC29_MCU', 19: 'PX_NC36_MCU', 20: V33M,
       21: 'ANT_BIAS_MON', 22: 'VANT_MON', 23: 'GNSS_RX_MCU', 24: 'GNSS_TX_MCU',
       25: 'USB_DN_MCU', 26: 'USB_DP_MCU', 27: 'PPS_MCU', 28: None,
       29: 'VDD_SPI', 30: None, 31: None, 32: None, 33: None, 34: None, 35: None,      # in-package flash
       36: 'IO48', 37: 'IO47', 38: 'IO33', 39: 'IO34', 40: 'IO35', 41: 'IO36', 42: 'USB_SEL_MCU',
       43: 'PX100_RSTN_MCU', 44: 'PX100_BOOT_SEL_MCU', 45: 'GNSS_EN', 46: V33M, 47: 'LED_STAT', 48: None,
       49: 'DBG_TX', 50: 'DBG_RX', 51: None, 52: 'GPIO46', 53: 'XTAL_N', 54: 'XTAL_P', 55: V33M, 56: V33M, 57: GND}
P(s, 'U201', 'MCU_Espressif:ESP32-S3', 'ESP32-S3FN8', 'Footprints:IC_ESP32-S3', ESP, 'ESP32-S3FN8', 'Espressif',
  desc='8 MB flash in the package; the rocket boards\' footprint', block=b)
P(s, 'Y201', 'Device:Crystal_GND24', '40 MHz', 'Footprints:XTAL_ECS-400-10-37B2-CKY-TR',
  {1: 'XTAL_N', 2: GND, 3: 'XTAL_P_L', 4: GND}, 'ECS-400-10-37B2-CKY-TR', 'ECS', block=b)
P(s, 'L201', 'Device:L', '24 nH', 'Inductor_SMD:L_0402_1005Metric', {1: 'XTAL_P_L', 2: 'XTAL_P'}, 'LQW15AN24NH00D',
  'Murata', block=b)
C(s, 'C201', '12pF', 'XTAL_P_L', GND, b)
C(s, 'C202', '12pF', 'XTAL_N', GND, b)
R(s, 'R201', '10k', V33M, 'CHIP_PU', b)
C(s, 'C203', '1uF', 'CHIP_PU', GND, b)
R(s, 'R202', '10k', V33M, 'BOOT0', b)
R(s, 'R203', '10k', 'GPIO46', GND, b)
C(s, 'C204', '10uF', V33M, GND, b)
C(s, 'C205', '100nF', V33M, GND, b)
C(s, 'C206', '100nF', V33M, GND, b)
C(s, 'C207', '1uF', 'VDD_SPI', GND, b)
C(s, 'C208', '100nF', V33M, GND, b)
C(s, 'C209', '100nF', V33M, GND, b)
C(s, 'C210', '10nF', V33M, GND, b)
C(s, 'C211', '10uF', V33M, GND, b)
BUTTON(s, 'SW201', 'BOOT', 'BOOT0', b)
BUTTON(s, 'SW202', 'RESET', 'CHIP_PU', b)
R(s, 'R204', '22', 'ESP_DP', 'USB_DP_MCU', b)
R(s, 'R205', '22', 'ESP_DN', 'USB_DN_MCU', b)

b = 'Status light and spare pins'
R(s, 'R206', '1k', 'LED_STAT', 'LED_STAT_A', b)
LED(s, 'D201', 'LED_STAT_A', GND, b, desc='STATUS')
P(s, 'J201', 'Connector_Generic:Conn_01x10', 'ESP32 SPARE', 'Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical',
  {1: GND, 2: V33M, 3: 'DBG_TX', 4: 'DBG_RX', 5: 'IO33', 6: 'IO34', 7: 'IO35', 8: 'IO36', 9: 'IO47', 10: 'IO48'},
  '', '', desc='UART0 console and six spare GPIOs (hand-fit header)', block=b)

# ================================================================ PX100 USB bridge and I/O
s = 'pxio'
b = 'CP2102N USB-UART bridge to the PX100\'s UART (self-powered at 3.3 V, data sheet figs 2.3/2.6)'
P(s, 'U301', 'Interface_USB:CP2102N-Axx-xQFN24', 'CP2102N-A02-GQFN24',
  'Package_DFN_QFN:QFN-24-1EP_4x4mm_P0.5mm_EP2.6x2.6mm',
  {1: None, 2: GND, 3: 'CP_DP', 4: 'CP_DN', 5: V33M, 6: V33M, 7: V33M, 8: 'CP_VBUS', 9: 'CP_RST_N', 10: None,
   11: None, 12: None, 13: 'CP_RXT', 14: 'CP_TXT', 15: None, 16: None, 17: None, 18: None, 19: 'CP_RTS',
   20: 'CP_RXD', 21: 'CP_TXD', 22: None, 23: 'CP_DTR', 24: None, 25: GND}, 'CP2102N-A02-GQFN24', 'Silicon Labs',
  desc='macOS has the driver built in (AppleUSBSLCOM); up to 3 Mbaud', block=b)
C(s, 'C301', '4.7uF', V33M, GND, b, desc='VDD and VREGIN')
C(s, 'C302', '100nF', V33M, GND, b, desc='VDD and VREGIN')
C(s, 'C303', '4.7uF', V33M, GND, b, desc='VIO')
C(s, 'C304', '100nF', V33M, GND, b, desc='VIO')
R(s, 'R301', '22.1k', VBUS, 'CP_VBUS', b, desc='VBUS sense divider, self-powered')
R(s, 'R302', '47.5k', 'CP_VBUS', GND, b)
R(s, 'R303', '1k', 'CP_RST_N', V33M, b)
P(s, 'D301', 'Device:D_Schottky', 'NSR0320MW2T1G', 'Diode_SMD:D_SOD-323', {1: 'USB_SEL', 2: 'CP_RST_N'},
  'NSR0320MW2T1G', 'onsemi',
  desc='Holds the CP2102N in reset (its pins weak pull-ups only) while S101 gives USB to the ESP32', block=b)
R(s, 'R304', '1k', 'CP_TXD', 'GNSS_RX', b, desc='Into PX100 RXD1')
R(s, 'R305', '1k', 'GNSS_TX', 'CP_RXD', b, desc='From PX100 TXD1')
R(s, 'R306', '1k', V33M, 'LED_TXT_A', b)
LED(s, 'D302', 'LED_TXT_A', 'CP_TXT', b, desc='TX (CP2102N GPIO.0, low while sending)')
R(s, 'R307', '1k', V33M, 'LED_RXT_A', b)
LED(s, 'D303', 'LED_RXT_A', 'CP_RXT', b, desc='RX (CP2102N GPIO.1)')

b = 'PX100 reset and boot: DTR/RTS auto-program (the ESP32 dev-board circuit), buttons'
# esptool-style: DTR high + RTS low pulls RSTN low; DTR low + RTS high pulls BOOT_SEL low (ROM boot loader);
# both equal does nothing, so a terminal that opens the port leaves the PX100 running.
P(s, 'Q301', 'Transistor_BJT:MMBT3904', 'MMBT3904', 'Package_TO_SOT_SMD:SOT-23',
  {1: 'Q301_B', 2: 'CP_RTS', 3: 'PX100_RSTN'}, 'MMBT3904LT1G', 'onsemi', block=b)
R(s, 'R308', '10k', 'CP_DTR', 'Q301_B', b, desc='Remove to take reset away from DTR/RTS')
P(s, 'Q302', 'Transistor_BJT:MMBT3904', 'MMBT3904', 'Package_TO_SOT_SMD:SOT-23',
  {1: 'Q302_B', 2: 'CP_DTR', 3: 'PX100_BOOT_SEL'}, 'MMBT3904LT1G', 'onsemi', block=b)
R(s, 'R309', '10k', 'CP_RTS', 'Q302_B', b, desc='Remove to take BOOT_SEL away from DTR/RTS')
BUTTON(s, 'SW301', 'PX RESET', 'PX100_RSTN', b)
BUTTON(s, 'SW302', 'PX BOOT', 'PX100_BOOT_SEL', b, desc='Hold through a reset: BOOT_SEL low = the PX100\'s ROM boot loader')

b = 'PX100 pins to the ESP32: 1 k in every line (no fight if both sides drive), probe header on the PX100 side'
LINKS = [  # (PX100 pin, PX100-side net, ESP32-side net)
    (8, 'PX_NC8', 'PX_NC8_MCU'), (12, 'PX_GPIO0', 'PX_GPIO0_MCU'), (13, 'PX_GPIO6', 'PX_GPIO6_MCU'),
    (14, 'PX_NC14', 'PX_NC14_MCU'), (15, 'PX_GPIO9', 'PX_GPIO9_MCU'), (17, 'PX100_RSTN', 'PX100_RSTN_MCU'),
    (18, 'GNSS_RX', 'GNSS_RX_MCU'), (19, 'GNSS_TX', 'GNSS_TX_MCU'), (24, 'PX_GPIO11', 'PX_GPIO11_MCU'),
    (25, 'PX_GPIO10', 'PX_GPIO10_MCU'), (26, 'PX_GPIO13', 'PX_GPIO13_MCU'), (27, 'PX_GPIO14', 'PX_GPIO14_MCU'),
    (28, 'PX_GPIO15', 'PX_GPIO15_MCU'), (29, 'PX_NC29', 'PX_NC29_MCU'), (30, 'PX100_BOOT_SEL', 'PX100_BOOT_SEL_MCU'),
    (31, 'PPS', 'PPS_MCU'), (32, 'PX_GPIO2', 'PX_GPIO2_MCU'), (36, 'PX_NC36_LINE', 'PX_NC36_MCU'),
]
for i, (pin, px, mcu) in enumerate(LINKS):
    R(s, 'R%d' % (310 + i), '1k', px, mcu, b, desc='U1 pin %d' % pin)
R(s, 'R330', '0', 'PX_NC36', 'PX_NC36_LINE', b, dnp=True,
  desc='Pin 36 sits between VDD_ANA2 and LNA_IN2: fit at U1 to probe it, otherwise the pin stays open')
C(s, 'C305', '2.2uF', 'PX_NC14', GND, b, desc='SkyTraq\'s reference puts 2.2 uF (their C21) on "NC" pin 14; place at U1')
HDR = {2 * i + 1: px for i, (pin, px, mcu) in enumerate(LINKS[:9])}           # odd pins: U1 pins 8-24
HDR.update({2 * i + 2: px for i, (pin, px, mcu) in enumerate(LINKS[9:])})     # even pins: U1 pins 25-36
HDR.update({19: GND, 20: GND})
P(s, 'J301', 'Connector_Generic:Conn_02x10_Odd_Even', 'PX100 PINS', 'Connector_PinHeader_2.54mm:PinHeader_2x10_P2.54mm_Vertical',
  HDR, '', '', desc='Logic analyser / scope header straight on the PX100 pins (hand-fit header)', block=b)

b = 'Antenna bias monitors (read on ADC2: halves of 3.3 V)'
R(s, 'R331', '100k', 'ANT_BIAS', 'ANT_BIAS_MON', b, desc='PX100 ANT_BIAS (pin 9): is it live?')
R(s, 'R332', '100k', 'ANT_BIAS_MON', GND, b)
C(s, 'C306', '100nF', 'ANT_BIAS_MON', GND, b)
R(s, 'R333', '100k', 'VANT', 'VANT_MON', b, desc='The antenna feed after the bias choice')
R(s, 'R334', '100k', 'VANT_MON', GND, b)
C(s, 'C307', '100nF', 'VANT_MON', GND, b)
TP(s, 'TP301', 'VANT', b)
TP(s, 'TP302', 'ANT_BIAS', b)

# nets the hand-drawn receiver sheet shares with the sheets above
RECEIVER_NETS = {'PX_NC8', 'PX_GPIO0', 'PX_GPIO6', 'PX_NC14', 'PX_GPIO9', 'PX100_RSTN', 'GNSS_RX', 'GNSS_TX',
                 'PX_GPIO11', 'PX_GPIO10', 'PX_GPIO13', 'PX_GPIO14', 'PX_GPIO15', 'PX_NC29', 'PX100_BOOT_SEL', 'PPS',
                 'PX_GPIO2', 'PX_NC36', 'ANT_BIAS', 'VANT'}
POWER_NETS = {GND, VBUS, V33}
PWR_FLAG_NETS = {'power': [VBUS, GND, V33, 'USB_MUX_VCC'], 'esp32': ['VDD_SPI'], 'pxio': []}
