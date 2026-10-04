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
from run_radiated import start_tx, stop_tx, keep_awake           # noqa: E402
from ensure_hackrf import hackrf_idle                           # noqa: E402
from skytraq_raw import Link                                     # noqa: E402
from nav_mode import cold_start, set_mode, read_mode, MODES     # noqa: E402
from serial.tools import list_ports                              # noqa: E402

# Which receiver: "--mac MAC" (the bridge's USB serial number; default the TinkerNav PX1105R)
# or "--port DEV"; "--rx NAME" names the captures (px1105r_..., px1125r_...).
PX_MAC = (sys.argv[sys.argv.index("--mac") + 1] if "--mac" in sys.argv else "9C:13:9E:A2:F8:CC").upper()
PORT_ARG = sys.argv[sys.argv.index("--port") + 1] if "--port" in sys.argv else None
RX = sys.argv[sys.argv.index("--rx") + 1].lower() if "--rx" in sys.argv else "px1105r"
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
# "--rate HZ": the C8's sample rate (default 2.6 Msps, gps-sdr-sim's). SignalSim's multi-GNSS L1
# files are 8.184 Msps (Galileo E1 and BeiDou B1C need the width).
RATE = int(sys.argv[sys.argv.index("--rate") + 1]) if "--rate" in sys.argv else 2600000
# "--freq HZ": the C8's centre frequency (default L1, 1575.42 MHz). SignalSim's L1-plus-B1I files sit at
# 1568.286 MHz so one 18.48 Msps band holds BeiDou B1I (1561.098) and GPS L1 / Galileo E1 (1575.42).
FREQ = int(sys.argv[sys.argv.index("--freq") + 1]) if "--freq" in sys.argv else 1575420000
# "--bb-filter HZ": the HackRF baseband filter (hackrf_transfer -b). Left alone it follows the sample rate
# down to ~0.75x, which at 18.48 Msps clips B1I and L1 at the band edges; 20 MHz keeps both.
BBF = int(sys.argv[sys.argv.index("--bb-filter") + 1]) if "--bb-filter" in sys.argv else None
# "--pad-shift S": seconds of pad added in front of the scenario in this C8 (180 for the
# 360 s-pad files, 420 for spaceshot_pad600); maps file time to the scenario's truth.
PAD_SHIFT = float(sys.argv[sys.argv.index("--pad-shift") + 1]) if "--pad-shift" in sys.argv else 180.0
# "--tag X" appends _X to the capture name, so a repeat run never overwrites an earlier one.
RUN_TAG = sys.argv[sys.argv.index("--tag") + 1] if "--tag" in sys.argv else None
tag = ((f"_nav{NAV}" if NAV is not None else "") + (f"_el{ELEV}" if ELEV is not None else "")
       + (f"_{RESTART}{HOT_T:.0f}" if HOT_T is not None else "") + (f"_{RUN_TAG}" if RUN_TAG else ""))
if (SDR / "captures" / f"{RX}_{C8.stem}_gain{GAIN}{tag}.log").exists():
    sys.exit(f"!! capture {RX}_{C8.stem}_gain{GAIN}{tag}.log exists; pass --tag to keep it")
OUT = SDR / "captures" / (f"{RX}_{C8.stem}_gain{GAIN}{tag}.log" if (GAIN or len(sys.argv) > 2) else f"{RX}_smooth460_spaceshot.log")
ERR = "/tmp/hackrf_tx_px1105r.err"
awake = keep_awake(SECONDS + 120)      # the flight plus the receiver setup before it and the stop after
print(f"# caffeinate pid {awake.pid}: awake and user-active until this run exits"
      if awake else "# caffeinate not found: running without a keep-awake")

port = PORT_ARG or next((p.device for p in list_ports.comports() if (p.serial_number or "").upper() == PX_MAC), None)
if not port:
    sys.exit(f"!! no receiver on USB with serial {PX_MAC} (pass --mac or --port)")
link = Link(port)
time.sleep(0.3)
link.frames()
navname = "as-is"
# "--pad-restart warm|hot|cold" picks the restart before transmitting (default cold). warm/hot
# are seeded with the scenario's start time and origin -- the August PX1125R procedure
# (seed_restart.py via batch_run.py), for a receiver that fights the injected signal.
PAD_RESTART = sys.argv[sys.argv.index("--pad-restart") + 1] if "--pad-restart" in sys.argv else "cold"
if NAV is not None:
    if PAD_RESTART == "cold":
        cold_start(link)                                  # clears time/pos/eph, keeps SRAM config
    else:
        import json as _json
        from seed_restart import restart_payload as _rp
        _sc = _json.loads((SDR / "scenarios" / "spaceshot.json").read_text())
        _o, _alt0 = _sc["origin"], _sc["truth"][0]["alt_m"]
        _ack, _ = link.command(_rp({"hot": 1, "warm": 2}[PAD_RESTART], (2026, 8, 18, 8, 30, 0),
                                   _o["lat_deg"], _o["lon_deg"], _alt0), timeout=3.0)
        if not _ack:
            link.close(); sys.exit(f"!! receiver refused the seeded {PAD_RESTART} restart")
        time.sleep(3.0); link.frames()
        print(f"# seeded {PAD_RESTART} restart: 2026-08-18 08:30:00, {_o['lat_deg']:.2f}, {_o['lon_deg']:.2f}, {_alt0:.0f} m")
    if not set_mode(link, NAV):
        link.close(); sys.exit(f"!! receiver refused nav mode {NAV}")
    rb = read_mode(link)
    if rb != NAV:
        link.close(); sys.exit(f"!! nav mode set {NAV} but reads back {rb}")
    navname = f"{NAV} {MODES.get(NAV, '?')}"
    print(f"# nav mode {navname}, {PAD_RESTART} restart on the pad")
if ELEV is not None:
    ack, _ = link.command(bytes([0x2B, 0x01, ELEV, 0, 0]), timeout=3.0)     # elevation + CNR, CNR 0, SRAM
    if not ack:
        link.close(); sys.exit(f"!! receiver refused elevation mask {ELEV}")
# "--power-mode normal|save" (AN0037 0x0C, SRAM). SkyTraq ships in Power Save, which throttles the
# search engine; with it on, the PX1125R never re-assembled an ephemeris on the bench.
PMODE = sys.argv[sys.argv.index("--power-mode") + 1] if "--power-mode" in sys.argv else None
if PMODE is not None:
    if PMODE not in ("normal", "save"):
        link.close(); sys.exit("--power-mode is normal or save")
    _pa, _ = link.command(bytes([0x0C, 0 if PMODE == "normal" else 1, 0x00]), timeout=3.0)
    if not _pa:
        link.close(); sys.exit(f"!! receiver refused power mode {PMODE}")
# "--gnss gps" restricts the navigation constellations to GPS (AN0037 0x64/0x19) and turns QZSS off
# (0x62/0x03), both SRAM only, after the pad restart: the sim is GPS L1 only, and the receiver
# otherwise keeps channels searching for BeiDou/Galileo/NavIC/QZSS all run. The settings found
# are put back after the run.
# "--gnss mask:0xNNNN" sets that constellation mask instead (bit 0 GPS, 2 Galileo, 3 BeiDou, 4 NavIC as this
# PX1105R reads 0x001D) and leaves QZSS alone -- e.g. BeiDou out for a same-PRN GPS/BeiDou test.
GNSS = sys.argv[sys.argv.index("--gnss") + 1] if "--gnss" in sys.argv else None
GNSS_MASK = int(GNSS.split(":", 1)[1], 16) if GNSS and GNSS.startswith("mask:") else (0x0001 if GNSS == "gps" else None)


def _sub_query(payload, sub_reply, timeout=3.0):
    """Send a 0x62/0x64 query; return the reply frame with the named sub-ID, or None."""
    link.s.write(__import__("skytraq_raw").frame(payload))
    deadline = time.time() + timeout
    while time.time() < deadline:
        for k, x in link.frames():
            if k == "B" and len(x) >= 2 and x[0] == payload[0] and x[1] == sub_reply:
                return x
    return None


gnss_restore = None
gnssname = ""
if GNSS is not None and GNSS_MASK is None:
    link.close(); sys.exit("--gnss takes gps or mask:0xNNNN")


def _restore_gnss():
    """Put back the constellation mask and QZSS setting found before --gnss gps (SRAM)."""
    global gnss_restore
    if gnss_restore is None:
        return
    _m, _qe, _qc = gnss_restore
    _a1, _ = link.command(bytes([0x64, 0x19, _m >> 8, _m & 0xFF, 0x00]), timeout=3.0)
    _a2, _ = link.command(bytes([0x62, 0x03, _qe, max(1, _qc), 0x00]), timeout=3.0)
    print(f"# restored constellation mask 0x{_m:04X} and QZSS {'on' if _qe else 'off'} (SRAM): acks {_a1}/{_a2}")
    gnss_restore = None


_pack, _pr = link.command(bytes([0x15]), want_reply=0xB9, timeout=3.0)
powername = ("power " + {0: "normal", 1: "save"}.get(_pr[1], str(_pr[1]))) if _pr else "power mode unknown"
if PMODE is not None and powername != f"power {PMODE}":
    link.close(); sys.exit(f"!! power mode set {PMODE} but reads back {powername}")
_mack, _mr = link.command(bytes([0x2F]), want_reply=0xB0, timeout=3.0)
maskname = f"elev mask {_mr[2]} deg, CNR mask {_mr[3]} dB-Hz" if _mr else "mask unknown"
if ELEV is not None and (not _mr or _mr[2] != ELEV):
    link.close(); sys.exit(f"!! elevation mask set {ELEV} but reads back {maskname}")
print(f"# {maskname}; {powername}")
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
if GNSS_MASK is not None:
    _c = _sub_query(bytes([0x64, 0x1A]), 0x8C)
    _q = _sub_query(bytes([0x62, 0x04]), 0x81)
    _i = _sub_query(bytes([0x64, 0x07]), 0x83)
    if not _c or not _q:
        link.close(); sys.exit(f"!! no constellation / QZSS readback before --gnss {GNSS}")
    gnss_restore = (int.from_bytes(_c[2:4], "big"), _q[2], _q[3])
    print(f"# found: constellation mask 0x{gnss_restore[0]:04X}, QZSS {'on' if _q[2] else 'off'} "
          f"({_q[3]} ch); interference detection " +
          (f"{'on' if _i[2] else 'off'}, status {_i[3]} (0 unknown 1 none 2 lite 3 critical)" if _i else "no reply"))
    _qz = 0x00 if GNSS == "gps" else _q[2]            # GPS-only also turns QZSS off; a mask leaves it
    _a1, _ = link.command(bytes([0x64, 0x19, GNSS_MASK >> 8, GNSS_MASK & 0xFF, 0x00]), timeout=3.0)
    _a2, _ = link.command(bytes([0x62, 0x03, _qz, max(1, _q[3]), 0x00]), timeout=3.0)
    _c = _sub_query(bytes([0x64, 0x1A]), 0x8C)
    _q = _sub_query(bytes([0x62, 0x04]), 0x81)
    if not (_a1 and _a2 and _c and _q and int.from_bytes(_c[2:4], "big") == GNSS_MASK and _q[2] == _qz):
        _restore_gnss()
        link.close(); sys.exit(f"!! --gnss {GNSS} not taken: acks {_a1}/{_a2}, readback "
                               f"{_c.hex() if _c else None} {_q.hex() if _q else None}")
    # the change must not have cost the SRAM settings above (a reboot would put back flash defaults)
    time.sleep(2.0)
    _, _pr2 = link.command(bytes([0x15]), want_reply=0xB9, timeout=3.0)
    _, _mr2 = link.command(bytes([0x2F]), want_reply=0xB0, timeout=3.0)
    _nav2 = read_mode(link) if NAV is not None else None
    if (not _pr2 or not _mr2 or (_pr and _pr2[1] != _pr[1]) or (_mr and _mr2[2] != _mr[2])
            or (NAV is not None and _nav2 != NAV)):
        _restore_gnss()
        link.close(); sys.exit(f"!! settings changed after --gnss {GNSS}: power {_pr2.hex() if _pr2 else None}, "
                               f"mask {_mr2.hex() if _mr2 else None}, nav mode {_nav2}")
    gnssname = ("; GNSS GPS only, QZSS off (SRAM)" if GNSS == "gps"
                else f"; GNSS mask 0x{GNSS_MASK:04X}, QZSS as found (SRAM)")
    print(f"# constellation{gnssname[6:]}, other settings unchanged; restored after the run")
t_tx = time.time() - link.t0
tx = start_tx(C8, FREQ, RATE, GAIN, ERR, extra=("-B",) + (("-b", str(BBF)) if BBF else ()))
tx_wall = time.time()      # ~4 s into the file (start_tx sleeps 4 s)
if tx is None:
    _restore_gnss()
    link.close()
    sys.exit("!! hackrf_transfer did not start")
ids, n_e5 = {}, 0
try:
    with open(OUT, "w") as f:
        f.write(f"0.000 # host: tx {C8.name} gain {GAIN}{f', {RATE / 1e6:.3f} Msps' if RATE != 2600000 else ''}{f' at {FREQ / 1e6:.3f} MHz' if FREQ != 1575420000 else ''}{f', filter {BBF / 1e6:.0f} MHz' if BBF else ''}; TX launched {t_tx:.3f} s after the link opened "
                f"(start_tx returns 4 s later); {RX.upper()} {PORT_ARG or PX_MAC} RTK kinematic base, 0xE5 20 Hz; nav mode {navname}; {maskname}; {powername}{gnssname}\n")
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
    _restore_gnss()
    link.close()
    shutil.copyfile(ERR, str(OUT) + ".hackrf.txt")
    hackrf_idle()                     # radio idle between scenarios (owner's rule)
print(f"# capture -> {OUT}: " + ", ".join(f"0x{k:02X} x{v}" for k, v in sorted(ids.items())))
