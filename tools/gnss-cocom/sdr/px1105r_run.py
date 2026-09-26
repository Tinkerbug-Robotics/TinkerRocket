#!/usr/bin/env python3
"""PX1105R (TinkerNav, RTK kinematic base, raw 0xE5 at 20 Hz) through the smooth
spaceshot to 460 s, radiated at gain 0 in the sealed cage: where does the raw
measurement output stop and restart against the COCOM crossings?"""
import shutil
import sys
import time
from pathlib import Path

SDR = Path(__file__).resolve().parent
sys.path.insert(0, str(SDR))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_radiated import start_tx, stop_tx                       # noqa: E402
from ensure_hackrf import hackrf_idle                           # noqa: E402
from skytraq_raw import Link                                     # noqa: E402
from nav_mode import cold_start, set_mode, read_mode, MODES     # noqa: E402
from serial.tools import list_ports                              # noqa: E402

PX_MAC = "9C:13:9E:A2:F8:CC"
GAIN = int(sys.argv[1]) if len(sys.argv) > 1 else 0
if not 0 <= GAIN <= 6:
    sys.exit("gain is capped at 6 dB for radiated runs")
C8 = SDR / "c8" / (sys.argv[2] if len(sys.argv) > 2 else "spaceshot_smooth460.C8")
SECONDS = float(sys.argv[3]) if len(sys.argv) > 3 else 462.0
NAV = int(sys.argv[4]) if len(sys.argv) > 4 else None      # nav (dynamics) mode; cold-start + set it
tag = f"_nav{NAV}" if NAV is not None else ""
OUT = SDR / "captures" / (f"px1105r_{C8.stem}_gain{GAIN}{tag}.log" if (GAIN or len(sys.argv) > 2) else "px1105r_smooth460_spaceshot.log")
ERR = "/tmp/hackrf_tx_px1105r.err"

port = next((p.device for p in list_ports.comports() if (p.serial_number or "").upper() == PX_MAC), None)
if not port:
    sys.exit(f"!! the TinkerNav ({PX_MAC}) is not on USB")
link = Link(port)
time.sleep(0.3)
link.frames()
navname = "as-is"
if NAV is not None:
    cold_start(link)                                      # clears time/pos/eph, keeps SRAM config
    if not set_mode(link, NAV):
        link.close(); sys.exit(f"!! receiver refused nav mode {NAV}")
    rb = read_mode(link)
    if rb != NAV:
        link.close(); sys.exit(f"!! nav mode set {NAV} but reads back {rb}")
    navname = f"{NAV} {MODES.get(NAV, '?')}"
    print(f"# nav mode {navname}, cold started")
t_tx = time.time() - link.t0
tx = start_tx(C8, 1575420000, 2600000, GAIN, ERR, extra=("-B",))
if tx is None:
    link.close()
    sys.exit("!! hackrf_transfer did not start")
ids, n_e5 = {}, 0
try:
    with open(OUT, "w") as f:
        f.write(f"0.000 # host: tx {C8.name} gain {GAIN}; TX launched {t_tx:.3f} s after the link opened "
                f"(start_tx returns 4 s later); PX1105R {PX_MAC} RTK kinematic base, 0xE5 20 Hz; nav mode {navname}\n")
        t_end = time.time() + SECONDS
        last = 0.0
        while time.time() < t_end:
            for k, x in link.frames():
                t = time.time() - link.t0
                if k == "B":
                    f.write(f"{t:.3f} B {x.hex()}\n")
                    ids[x[0]] = ids.get(x[0], 0) + 1
                else:
                    s = x.decode("ascii", "replace")
                    if s.startswith("$") or s.startswith("#"):
                        f.write(f"{t:.3f} {s}\n")
            if time.time() - last > 30:
                last = time.time()
                print(f"  t={time.time() - link.t0:6.0f}s  frames " +
                      ", ".join(f"0x{k:02X}:{v}" for k, v in sorted(ids.items())), flush=True)
finally:
    stop_tx(tx)
    link.close()
    shutil.copyfile(ERR, str(OUT) + ".hackrf.txt")
    hackrf_idle()                     # radio idle between scenarios (owner's rule)
print(f"# capture -> {OUT}: " + ", ".join(f"0x{k:02X} x{v}" for k, v in sorted(ids.items())))
