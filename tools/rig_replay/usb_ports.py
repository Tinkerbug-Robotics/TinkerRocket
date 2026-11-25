#!/usr/bin/env python3
"""Which /dev/cu port is the flight computer's, from the USB registry alone: no port is opened (opening one pulses
RTS/DTR, which resets an ESP32), and the port is the one macOS made for the device whose USB serial number is the
flight computer's MAC -- not "the only usbmodem port", which a V9's USB switch or a second board can make another
chip's (the FC and the OC can even get the same port name, the one of the USB socket).

    usb_ports.py --fc 80:F1:B2:D0:94:A7 [--ioreg-file DUMP]     -> prints the FC's port, exit 0; or why not, exit 1
    usb_ports.py --list [--ioreg-file DUMP]                      -> every USB device with a serial number, its ports

It reads `ioreg -r -c IOUSBHostDevice -l -w0` (or DUMP, a saved copy): each serial port (IOSerialBSDClient, its
IOCalloutDevice) belongs to the nearest USB device above it in the tree. The rig allows one other USB device with a
serial number: the PortaPack (Mayhem's "Transceiver", or the HackRF's 32 hex digits once a run left it in HackRF mode).
The flight computer must be the only other one, with exactly one port, and no other usbmodem port may exist.
"""
import argparse
import re
import subprocess
import sys

NODE = re.compile(r"^(?P<indent>[ |]*)\+-o (?P<name>.+?)  <class (?P<cls>[^,>]+)")
PROP = re.compile(r'^[ |]*"(?P<key>[^"]+)" = (?P<val>.*?)\s*$')
USB_DEVICE = ("IOUSBHostDevice", "IOUSBDevice")
HACKRF_SERIAL = re.compile(r"^[0-9a-fA-F]{32}$")


def nodes(text):
    """The registry tree: each node's class, name, properties and parent."""
    out, stack = [], []
    for ln in text.splitlines():
        m = NODE.match(ln)
        if m:
            depth = len(m.group("indent"))
            while stack and stack[-1]["depth"] >= depth:
                stack.pop()
            n = {"depth": depth, "name": m.group("name"), "cls": m.group("cls").strip(), "props": {},
                 "parent": stack[-1] if stack else None}
            out.append(n)
            stack.append(n)
            continue
        m = PROP.match(ln)
        if m and stack:
            v = m.group("val")
            stack[-1]["props"].setdefault(m.group("key"), v[1:-1] if v.startswith('"') and v.endswith('"') else v)
    return out


def devices(text):
    """Every USB device: serial number, product, locationID and the serial ports under it (each once, however
    often ioreg prints a device nested under a hub and again as its own root)."""
    tree = nodes(text)
    devs = {}

    def key(n):
        p = n["props"]
        return (p.get("locationID"), p.get("USB Serial Number"), p.get("USB Product Name") or n["name"])

    for n in tree:
        if n["cls"] in USB_DEVICE:
            k = key(n)
            devs.setdefault(k, {"serial": k[1], "product": k[2], "location": k[0], "ports": []})
    for n in tree:
        port = n["props"].get("IOCalloutDevice")
        if n["cls"] != "IOSerialBSDClient" or not port:
            continue
        up = n["parent"]
        while up is not None and up["cls"] not in USB_DEVICE:
            up = up["parent"]
        if up is None:
            continue
        d = devs[key(up)]
        if port not in d["ports"]:
            d["ports"].append(port)
    return list(devs.values())


def is_portapack(d):
    s = d["serial"] or ""
    return s.lower() == "transceiver" or "portapack" in (d["product"] or "").lower() or bool(HACKRF_SERIAL.match(s))


def fc_port(text, fc_mac):
    """The flight computer's one port, or (None, why)."""
    devs = devices(text)
    others = [d for d in devs if d["serial"] and not is_portapack(d)]
    want = fc_mac.upper()
    fc = [d for d in others if d["serial"].upper() == want]
    strangers = [d for d in others if d["serial"].upper() != want]
    loose = [p for d in devs if not d["serial"] for p in d["ports"]]   # a serial port with no serial number above it
    if not fc:
        return None, f"no USB device with the serial number {fc_mac} (devices with one: " \
                     f"{[d['serial'] for d in devs if d['serial']] or 'none'})"
    if strangers:
        return None, f"another USB device besides the FC and the PortaPack: {[d['serial'] for d in strangers]}"
    if len(fc) != 1 or len(fc[0]["ports"]) != 1:
        return None, f"the FC must be one device with one port: {fc}"
    if loose:
        return None, f"serial ports with no serial number above them: {loose}"
    return fc[0]["ports"][0], ""


def read_ioreg(path=None):
    if path:
        return open(path, errors="replace").read()
    return subprocess.run(["ioreg", "-r", "-c", "IOUSBHostDevice", "-l", "-w0"], capture_output=True, text=True,
                          check=True).stdout


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--fc", help="the flight computer's MAC = its USB serial number")
    g.add_argument("--list", action="store_true")
    ap.add_argument("--ioreg-file", help="a saved `ioreg -r -c IOUSBHostDevice -l -w0`, for tests")
    a = ap.parse_args(argv)
    text = read_ioreg(a.ioreg_file)
    if a.list:
        for d in devices(text):
            if d["serial"]:
                print(f"{d['serial']}\t{d['product']}\t{','.join(d['ports']) or '-'}")
        return 0
    port, why = fc_port(text, a.fc)
    if port is None:
        print(f"MAC check failed: {why}", file=sys.stderr)
        return 1
    print(port)
    return 0


if __name__ == "__main__":
    sys.exit(main())
