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
# hot-start injection: "--hot-start FILET[:SCENARIO]" sends 0x01 hot (kept ephemeris) with
# truth position + UTC at that file time, once, when the played signal reaches it (IMU-aided
# reacquire test). Vertical flight: lat/lon are the scenario origin (exact), alt from truth.
HOT_T = None; HOT_SCEN = None
if "--hot-start" in sys.argv:
    v = sys.argv[sys.argv.index("--hot-start") + 1]
    HOT_T = float(v.split(":")[0]); HOT_SCEN = v.split(":")[1] if ":" in v else "spaceshot"
# "--restart-mode hot|warm|cold" picks the 0x01 start mode for that restart (default hot).
RESTART = sys.argv[sys.argv.index("--restart-mode") + 1] if "--restart-mode" in sys.argv else "hot"
if RESTART not in ("hot", "warm", "cold"):
    sys.exit("--restart-mode is hot, warm or cold")
# "--elev-mask N" sets the elevation mask (AN0037 0x2B, 3-85 deg, SRAM only) after the pad cold start.
# The receiver ships at 15 deg, which keeps every satellite below ~15 deg out of its channel table.
ELEV = int(sys.argv[sys.argv.index("--elev-mask") + 1]) if "--elev-mask" in sys.argv else None
if ELEV is not None and not 3 <= ELEV <= 85:
    sys.exit("--elev-mask must be 3..85 deg (AN0037)")
# "--pad-shift S": seconds of pad added in front of the scenario in this C8 (180 for the
# 360 s-pad files, 420 for spaceshot_pad600); maps file time to the scenario's truth.
PAD_SHIFT = float(sys.argv[sys.argv.index("--pad-shift") + 1]) if "--pad-shift" in sys.argv else 180.0
# "--tag X" appends _X to the capture name, so a repeat run never overwrites an earlier one.
RUN_TAG = sys.argv[sys.argv.index("--tag") + 1] if "--tag" in sys.argv else None
tag = ((f"_nav{NAV}" if NAV is not None else "") + (f"_el{ELEV}" if ELEV is not None else "")
       + (f"_{RESTART}{HOT_T:.0f}" if HOT_T is not None else "") + (f"_{RUN_TAG}" if RUN_TAG else ""))
if (SDR / "captures" / f"px1105r_{C8.stem}_gain{GAIN}{tag}.log").exists():
    sys.exit(f"!! capture px1105r_{C8.stem}_gain{GAIN}{tag}.log exists; pass --tag to keep it")
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
if ELEV is not None:
    ack, _ = link.command(bytes([0x2B, 0x01, ELEV, 0, 0]), timeout=3.0)     # elevation + CNR, CNR 0, SRAM
    if not ack:
        link.close(); sys.exit(f"!! receiver refused elevation mask {ELEV}")
_mack, _mr = link.command(bytes([0x2F]), want_reply=0xB0, timeout=3.0)
maskname = f"elev mask {_mr[2]} deg, CNR mask {_mr[3]} dB-Hz" if _mr else "mask unknown"
if ELEV is not None and (not _mr or _mr[2] != ELEV):
    link.close(); sys.exit(f"!! elevation mask set {ELEV} but reads back {maskname}")
print(f"# {maskname}")
hot_frame = None
if HOT_T is not None:
    import json, struct, bisect, datetime
    sc = json.loads((SDR / "scenarios" / f"{HOT_SCEN}.json").read_text())
    tr = sc["truth"]; tt = [q["t"] for q in tr]; o = sc["origin"]
    ot = HOT_T - PAD_SHIFT                       # truth time (flight is padded by PAD_SHIFT)
    i = min(max(bisect.bisect_left(tt, ot), 1), len(tt) - 1)
    a_, b_ = tr[i - 1], tr[i]; w = (ot - a_["t"]) / (b_["t"] - a_["t"]) if b_["t"] > a_["t"] else 0.0
    alt = a_["alt_m"] + w * (b_["alt_m"] - a_["alt_m"])
    from seed_restart import restart_payload      # AN0037 0x01 layout, altitude clamped to -1000..18300 m
    alt_aid = max(-1000.0, min(18300.0, alt))     # the doc's bound: 18.3 km = the 60,000 ft COCOM altitude
    clamp_note = (f"; alt CLAMPED to {alt_aid/1000:.1f} km (AN0037 bounds 0x01 altitude at 18.3 km), "
                  f"true {alt/1000:.1f} km" if alt_aid != alt else "")
    y, mo, d, hh, mm, ss = 2026, 8, 18, 8, 30, 0
    utc = datetime.datetime(y, mo, d, hh, mm, ss) + datetime.timedelta(seconds=HOT_T)
    body = restart_payload({"hot": 1, "warm": 2, "cold": 3}[RESTART], (utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second),
                           o["lat_deg"], o["lon_deg"], alt)
    hot_frame = body
    print(f"# {RESTART}-start armed for file t={HOT_T:.0f}s: 0x01 {RESTART}, {utc:%Y-%m-%d %H:%M:%S}, "
          f"lat {o['lat_deg']:.2f} lon {o['lon_deg']:.2f} alt {alt_aid/1000:.1f} km{clamp_note}")
t_tx = time.time() - link.t0
tx = start_tx(C8, 1575420000, 2600000, GAIN, ERR, extra=("-B",))
tx_wall = time.time()      # ~4 s into the file (start_tx sleeps 4 s)
if tx is None:
    link.close()
    sys.exit("!! hackrf_transfer did not start")
ids, n_e5 = {}, 0
try:
    with open(OUT, "w") as f:
        f.write(f"0.000 # host: tx {C8.name} gain {GAIN}; TX launched {t_tx:.3f} s after the link opened "
                f"(start_tx returns 4 s later); PX1105R {PX_MAC} RTK kinematic base, 0xE5 20 Hz; nav mode {navname}; {maskname}\n")
        t_end = time.time() + SECONDS
        last = 0.0
        hot_sent = False
        while time.time() < t_end:
            if hot_frame is not None and not hot_sent and (time.time() - tx_wall) + 4.0 >= HOT_T:
                link.s.write(__import__("skytraq_raw").frame(hot_frame))
                hot_sent = True
                f.write(f"{time.time() - link.t0:.3f} # host: {RESTART.upper()}-START 0x01 sent at file t~{HOT_T:.0f}{clamp_note}\n")
                print(f"# {RESTART.upper()}-START sent at file t~{HOT_T:.0f}s (host {time.time()-tx_wall+4:.0f})")
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
