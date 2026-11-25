"""tools/rig_replay: the V9 rig replay's helpers -- the transmitter's log on the file's timeline (tx_log.py), the
flight computer's port from the USB registry (usb_ports.py) and the sky's GPS Klobuchar from a RINEX header
(rinex_gps_iono.py). Made-up inputs in the real formats: hackrf_tx_ram -B's per-second line, `ioreg -r -c
IOUSBHostDevice -l -w0`'s tree, a BKG broadcast file's header. No real serial number or place.
"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools" / "rig_replay"))

import rinex_gps_iono  # noqa: E402
import tx_log  # noqa: E402
import usb_ports  # noqa: E402

MB_S = 2 * 18.48   # the file's MB a second


def stats(mb, under=0, longest=0, host=0):
    return (f"{mb:.1f} MB / 1.000 sec = {mb:.1f} MB/second, average power -11.2 dBfs, 19584 bytes filled in buffer, "
            f"{under} underruns, longest {longest} bytes, ring 63 MiB ahead, host shortfalls {host}, slowest callback "
            f"0.40 ms")


def tx_text(events, seconds=250, sent=True):
    """A whole file at the file's rate, one line a second; events: {second: (underruns, longest, host)} cumulative
    from that second on. A last, short line drains the end with one more underrun, as the HackRF does."""
    L = ["hackrf_tx_ram: signalsim_test.C8, 9.2 GB; ring 2048 MiB (wired), 512 MiB read ahead in 0.30 s"]
    cur = (0, 0, 0)
    for k in range(1, seconds + 1):
        cur = events.get(k, cur)
        L.append(stats(MB_S, *cur))
    if sent:
        L.append(stats(5.0, cur[0] + 1, max(cur[1], 3884160), cur[2]))
        total = (seconds * MB_S + 5.0) / 1e3
        L.append(f"hackrf_tx_ram: sent {total:.3f} GB of {total:.3f} GB, host shortfalls {cur[2]}")
    return "\n".join(L) + "\n"


class TxLogTest(unittest.TestCase):
    def test_each_underrun_lands_on_the_files_timeline(self):
        tx = tx_log.parse(tx_text({100: (1, 253312, 0), 190: (2, 253312, 0)}))
        ev = [e for e in tx["events"] if not e["drain"]]
        self.assertEqual(len(ev), 2)
        self.assertAlmostEqual(ev[0]["file_s_from"], 99.0, delta=0.05)
        self.assertAlmostEqual(ev[0]["file_s_to"], 100.0, delta=0.05)
        self.assertAlmostEqual(ev[0]["slip_ms_longest"], 6.854, delta=0.01)   # 253,312 B at 36.96 MB/s
        self.assertEqual(ev[1]["added"], 1)
        self.assertTrue(tx["events"][-1]["drain"])
        self.assertTrue(tx["complete"])

    def test_the_windows_and_the_verdict(self):
        tx = tx_log.parse(tx_text({20: (1, 1000, 0), 100: (2, 253312, 0), 190: (3, 253312, 0), 240: (4, 253312, 0)}))
        r = tx_log.judge(tx, armed_file_s=60.0, freeze_file_s=175.0, landing_file_s=230.0)
        win = [e["window"] for e in r["events"]]
        self.assertEqual(win, ["before the clock", "between the clock and the freeze", "after the freeze",
                               "after the landing", "the end of the file draining"])
        self.assertEqual(r["verdict"], "VOID")
        self.assertEqual(len(r["void_why"]), 1)
        self.assertEqual(len(r["degraded_why"]), 1)
        # without the one after the freeze: only degraded
        tx2 = tx_log.parse(tx_text({100: (1, 253312, 0)}))
        self.assertEqual(tx_log.judge(tx2, 60.0, 175.0, 230.0)["verdict"], "DEGRADED")
        tx3 = tx_log.parse(tx_text({}))
        self.assertEqual(tx_log.judge(tx3, 60.0, 175.0, 230.0)["verdict"], "clean")

    def test_a_host_shortfall_is_a_slip_too(self):
        tx = tx_log.parse(tx_text({200: (0, 0, 1)}))
        e = [x for x in tx["events"] if x["kind"] == "host shortfall"]
        self.assertEqual(len(e), 1)
        self.assertAlmostEqual(e[0]["slip_ms_max"], 262144 / (MB_S * 1e6) * 1e3, delta=1e-3)
        r = tx_log.judge(tx, 60.0, 175.0, 230.0)
        self.assertEqual(r["verdict"], "VOID")
        self.assertIn("host shortfall", r["void_why"][0])

    def test_a_failed_or_short_transmission_is_void_unless_the_script_stopped_it(self):
        tx = tx_log.parse(tx_text({}, sent=False) + "hackrf_tx_ram: sent 4.581 GB of 9.565 GB, host shortfalls 0\n")
        self.assertFalse(tx["complete"])
        self.assertEqual(tx_log.judge(tx, 60.0, 175.0, 230.0)["verdict"], "VOID")
        self.assertEqual(tx_log.judge(tx, 60.0, 175.0, 230.0, stopped=True)["verdict"], "clean")
        whole = tx_log.parse(tx_text({}))
        self.assertEqual(tx_log.judge(whole, 60.0, 175.0, 230.0, exit_status=1)["verdict"], "VOID")
        no_end = tx_log.parse(tx_text({}, sent=False))
        self.assertIn("never said", tx_log.judge(no_end, 60.0, 175.0, 230.0)["void_why"][0])


FC = "AA:BB:CC:00:11:22"
OC = "AA:BB:CC:00:11:33"


def device(serial, product, location, ports, depth=0):
    """One USB device's subtree as ioreg -r -c IOUSBHostDevice -l -w0 prints it (abridged): the device, its CDC
    interfaces (which repeat its serial number) and a serial client per port."""
    p = " " * depth
    L = [f"{p}+-o {product}@{location:08x}  <class IOUSBHostDevice, id 0x100{location:x}, registered, matched, active>",
         f'{p}  |   "USB Serial Number" = "{serial}"', f'{p}  |   "USB Product Name" = "{product}"',
         f'{p}  |   "locationID" = {location}']
    for k, port in enumerate(ports):
        L += [f"{p}  +-o IOUSBHostInterface@{k}  <class IOUSBHostInterface, id 0x2{k}, registered, matched, active>",
              f'{p}  | |   "USB Serial Number" = "{serial}"',
              f"{p}  | +-o AppleUSBACMData  <class AppleUSBACMData, id 0x3{k}, registered, matched, active>",
              f"{p}  |   +-o IOSerialBSDClient  <class IOSerialBSDClient, id 0x4{k}, registered, matched, active>",
              f'{p}  |         "IOCalloutDevice" = "{port}"']
    return L


def dump(*devs):
    return "\n".join(line for d in devs for line in d) + "\n"


class UsbPortsTest(unittest.TestCase):
    def test_the_fc_beside_the_portapack(self):
        text = dump(device(FC, "USB JTAG_serial debug unit", 0x1100000, ["/dev/cu.usbmodem1101"]),
                    device("Transceiver", "PortaPack Mayhem", 0x2100000, ["/dev/cu.usbmodemTransceiver1"]))
        self.assertEqual(usb_ports.fc_port(text, FC.lower()), ("/dev/cu.usbmodem1101", ""))

    def test_the_portapack_left_in_hackrf_mode_is_allowed(self):
        text = dump(device(FC, "USB JTAG_serial debug unit", 0x1100000, ["/dev/cu.usbmodem1101"]),
                    device("0000000000000000a06063c8235b6a5f", "HackRF One", 0x2100000, []))
        self.assertEqual(usb_ports.fc_port(text, FC)[0], "/dev/cu.usbmodem1101")

    def test_the_port_is_the_fcs_own_not_the_only_one(self):
        # the OC alone on the bus: one usbmodem port, but not the FC's
        port, why = usb_ports.fc_port(dump(device(OC, "USB JTAG_serial debug unit", 0x1100000,
                                                  ["/dev/cu.usbmodem1101"])), FC)
        self.assertIsNone(port)
        self.assertIn("no USB device with the serial number", why)
        # both: refused, another device besides the FC
        port, why = usb_ports.fc_port(dump(device(FC, "USB JTAG_serial debug unit", 0x1100000,
                                                  ["/dev/cu.usbmodem1101"]),
                                           device(OC, "USB JTAG_serial debug unit", 0x2100000,
                                                  ["/dev/cu.usbmodem2101"])), FC)
        self.assertIsNone(port)
        self.assertIn("another USB device", why)

    def test_one_device_one_port(self):
        port, why = usb_ports.fc_port(dump(device(FC, "USB JTAG_serial debug unit", 0x1100000,
                                                  ["/dev/cu.usbmodem1101", "/dev/cu.usbmodem1103"])), FC)
        self.assertIsNone(port)
        self.assertIn("one device with one port", why)
        port, why = usb_ports.fc_port(dump(device(FC, "USB JTAG_serial debug unit", 0x1100000, [])), FC)
        self.assertIsNone(port)

    def test_a_device_printed_twice_is_one(self):
        # ioreg -r prints a device under its hub and again as its own root
        d = device(FC, "USB JTAG_serial debug unit", 0x1100000, ["/dev/cu.usbmodem1101"])
        text = dump(["+-o USB2 Hub@01100000  <class IOUSBHostDevice, id 0x9, registered, matched, active>",
                     '  |   "locationID" = 17825791'], device(FC, "USB JTAG_serial debug unit", 0x1100000,
                                                              ["/dev/cu.usbmodem1101"], depth=2), d)
        self.assertEqual(usb_ports.fc_port(text, FC)[0], "/dev/cu.usbmodem1101")


RINEX3 = """\
     3.05           NAVIGATION DATA     MIXED               RINEX VERSION / TYPE
Converto v3.5.11    BKG Frankfurt       20260830 092015 UTC PGM / RUN BY / DATE
BDSA   3.2596e-08  6.7055e-08 -1.0133e-06  1.5497e-06       IONOSPHERIC CORR
GPSA   1.3039e-08  2.2352e-08 -5.9605e-08 -1.1921E-07       IONOSPHERIC CORR
GPSB   1.0445e+05  9.8304e+04 -1.3107e+05 -3.2768E+05       IONOSPHERIC CORR
                                                            END OF HEADER
GPSA   9.9999e-01  2.2352e-08 -5.9605e-08 -1.1921E-07       IONOSPHERIC CORR
"""

RINEX2 = """\
     2.11           N: GPS NAV DATA                         RINEX VERSION / TYPE
    0.1304D-07  0.2235D-07 -0.5960D-07 -0.1192D-06          ION ALPHA
    0.1044D+06  0.9830D+05 -0.1311D+06 -0.3277D+06          ION BETA
                                                            END OF HEADER
"""


class RinexGpsIonoTest(unittest.TestCase):
    def test_the_broadcast_integers_of_a_rinex3_header(self):
        # the rig flights' day-241 file: alpha 14, 3, -1, -2 and beta 51, 6, -2, -5 in the broadcast's units
        b = rinex_gps_iono.broadcast_bytes(rinex_gps_iono.header_klobuchar(RINEX3))
        self.assertEqual(b, [14, 3, -1, -2, 51, 6, -2, -5])

    def test_a_rinex2_header_gives_the_same(self):
        self.assertEqual(rinex_gps_iono.broadcast_bytes(rinex_gps_iono.header_klobuchar(RINEX2)),
                         [14, 3, -1, -2, 51, 6, -2, -5])

    def test_a_value_that_is_no_broadcast_byte_is_refused(self):
        with self.assertRaises(ValueError):
            rinex_gps_iono.broadcast_bytes([1.0e-6, 0, 0, 0, 0, 0, 0, 0])   # 1074 units of 2^-30: not a byte
        with self.assertRaises(ValueError):
            rinex_gps_iono.header_klobuchar("     3.05           NAVIGATION DATA\n" + " " * 60 + "END OF HEADER\n")


if __name__ == "__main__":
    unittest.main()
