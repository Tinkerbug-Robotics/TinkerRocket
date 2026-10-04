#!/usr/bin/env python3
"""Log the PX1105R (TinkerNav bridge) on the real sky through its own antenna. Listens only:
no transmitter is touched.

Cold-starts the receiver first so it forgets the simulated sky (time, position, ephemeris),
then gives it the bench runs' settings -- nav mode 9 (SLR), elevation mask 3 deg, power normal,
all SRAM -- and reads them back, so its C/N0 readings compare with the boost-vs-signal
ladder (cn0_boost_report.html). Every frame is written with host timestamps, in the bench
captures' format, to captures/px1105r_sky_<stamp>.log.

The capture holds the antenna's position: it stays in captures/ (gitignored), and nothing
here prints it.

    ./px1105r_sky_log.py [SECONDS=1200] [--no-cold] [--mac MAC]
"""
import datetime
import sys
import time
from pathlib import Path

SDR = Path(__file__).resolve().parent
sys.path.insert(0, str(SDR.parent))
sys.path.insert(0, str(SDR))
from serial.tools import list_ports                               # noqa: E402
from skytraq_raw import Link, frame                               # noqa: E402
from nav_mode import cold_start, set_mode, read_mode, MODES       # noqa: E402

args = [a for a in sys.argv[1:] if not a.startswith("--")]
SECONDS = float(args[0]) if args else 1200.0
MAC = sys.argv[sys.argv.index("--mac") + 1] if "--mac" in sys.argv else "9C:13:9E:A2:F8:CC"
NAV, ELEV = 9, 3
GNSS_NAMES = {0: "GPS", 1: "SBAS", 2: "GLONASS", 3: "Galileo", 4: "QZSS", 5: "BeiDou", 6: "NavIC"}

port = next((p.device for p in list_ports.comports() if (p.serial_number or "").upper() == MAC), None)
if not port:
    sys.exit(f"!! no PX1105R bridge on USB with serial {MAC}")
link = Link(port)
time.sleep(0.3)
link.frames()


def sub_query(payload, sub_reply, timeout=3.0):
    link.s.write(frame(payload))
    deadline = time.time() + timeout
    while time.time() < deadline:
        for k, x in link.frames():
            if k == "B" and len(x) >= 2 and x[0] == payload[0] and x[1] == sub_reply:
                return x
    return None


if "--no-cold" not in sys.argv:
    cold_start(link)                              # clears time/pos/eph, keeps SRAM config
if not set_mode(link, NAV) or read_mode(link) != NAV:
    link.close(); sys.exit(f"!! nav mode {NAV} not set")
ack, _ = link.command(bytes([0x2B, 0x01, ELEV, 0, 0]), timeout=3.0)        # elevation + CNR mask, SRAM
if not ack:
    link.close(); sys.exit(f"!! receiver refused elevation mask {ELEV}")
ack, _ = link.command(bytes([0x0C, 0x00, 0x00]), timeout=3.0)               # power normal, SRAM
if not ack:
    link.close(); sys.exit("!! receiver refused power mode normal")
_a, pr = link.command(bytes([0x15]), want_reply=0xB9, timeout=3.0)
_a, mr = link.command(bytes([0x2F]), want_reply=0xB0, timeout=3.0)
rtk = sub_query(bytes([0x6A, 0x07]), 0x83)
cm = sub_query(bytes([0x64, 0x1A]), 0x8C)
power = {0: "normal", 1: "save"}.get(pr[1], "?") if pr else "unknown"
mask = f"elev mask {mr[2]} deg, CNR mask {mr[3]} dB-Hz" if mr else "mask unknown"
rtkname = f"RTK mode/function {rtk[2]}/{rtk[3]}" if rtk and len(rtk) >= 4 else "RTK mode unknown"
cmask = f"constellation mask 0x{(cm[2] << 8) | cm[3]:04X}" if cm and len(cm) >= 4 else "constellation mask unknown"
settings = f"nav mode {NAV} {MODES.get(NAV, '?')}; {mask}; power {power}; {rtkname}; {cmask}"
print(f"# {port}: {'cold start, ' if '--no-cold' not in sys.argv else ''}{settings}")

stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M")
OUT = SDR / "captures" / f"px1105r_sky_{stamp}.log"
ids = {}
fix, sats = None, {}
try:
    with open(OUT, "w") as f:
        f.write(f"0.000 # host: SKY, no transmitter; PX1105R {MAC} on its own antenna; {settings}\n")
        t_end = time.time() + SECONDS
        last = time.time()
        while time.time() < t_end:
            for k, x in link.frames():
                t = time.time() - link.t0
                if k == "B":
                    f.write(f"{t:.3f} B {x.hex()}\n")
                    ids[x[0]] = ids.get(x[0], 0) + 1
                    if x[0] == 0xDF and len(x) >= 3:
                        fix = x[2]
                    elif x[0] == 0xE5 and len(x) >= 14:
                        sats = {}
                        for j in range(x[13]):
                            r = x[14 + 31 * j: 14 + 31 * (j + 1)]
                            if len(r) == 31:
                                sats.setdefault(GNSS_NAMES.get(r[0] & 0x0F, "?"), []).append(r[3])
                else:
                    s = x.decode("ascii", "replace")
                    if s.startswith("$") or s.startswith("#"):
                        f.write(f"{t:.3f} {s}\n")
            if time.time() - last > 30:
                last = time.time()
                per = ", ".join(f"{g} {len(v)} (C/N0 {min(v)}-{max(v)})" for g, v in sorted(sats.items()) if v)
                print(f"  t={time.time() - link.t0:5.0f}s  fix state {fix}; measurements: {per or 'none'}",
                      flush=True)
finally:
    link.close()
print(f"# capture -> {OUT}: " + ", ".join(f"0x{k:02X} x{v}" for k, v in sorted(ids.items())))
