#!/usr/bin/env python3
"""Transmit a static .C8 into the sealed cage at gain 0 and log the LC86G through
lc86_bridge, optionally switching its navigation mode on a schedule.

Radiated only inside the sealed Faraday cage, and never above HackRF gain 0: the
gain is fixed in the code, and a quieter level is made in the file instead
(make_level_steps.py).

    A/B/A:        --c8 c8/pad_static.C8 --seconds 730 --switch 240:5,480:3
    level sweep:  --c8 c8/pad_levels.C8 --seconds 1240
    one mode:     --c8 c8/pad_static.C8 --seconds 184 --start-mode 1

c8/pad_static.C8 is the flights' own pad, extended to 25 minutes (its first 170 s
are byte-identical to spaceshot.C8):

    gps-sdr-sim -e BRDC_2026230.rx2.n -l 0.0,-119.0,1200 -d 1500 -b 8 -s 2600000 \
        -t 2026/08/18,08:30:00 -p -o pad_static.C8

The capture uses run_radiated.py's format (t, text / 'R' + RTCM hex), so
msm_channels.py and the other readers take it as is. Mode switches are written
on the same handle that logs, so their $PAIR001 acks land in the capture, and a
$PAIR081 read-back follows each one.
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

SDR = Path(__file__).resolve().parent
sys.path.insert(0, str(SDR.parent))
sys.path.insert(0, str(SDR))

import serial                                                   # noqa: E402
import rtcm3                                                    # noqa: E402
from gnss_nmea_monitor import _demux                            # noqa: E402
from lc86_config import (Link, NAV_MODES, find_bridge, frame,   # noqa: E402
                         open_bridge, read_state, show_state)
from run_radiated import start_tx, stop_tx                      # noqa: E402

T0_TOW = 203400.0      # 2026/08/18 08:30:00 GPS, the file's t = 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--c8", required=True)
    ap.add_argument("--seconds", type=float, required=True)
    ap.add_argument("--start-mode", type=int, default=3)
    ap.add_argument("--switch", default="", help="T:MODE,... host s after TX start")
    ap.add_argument("--out", required=True)
    ap.add_argument("--no-cold-start", action="store_true")
    ap.add_argument("--dry", action="store_true", help="no transmission")
    a = ap.parse_args()

    port = find_bridge("auto")
    if not port:
        return "the Beetle FC (lc86_bridge) is not on USB"
    ser = open_bridge(port)
    link = Link(ser)
    st = read_state(link)
    show_state(st)
    if st.get("fix_interval_ms") != 100 or st.get("rtcm") != 1 or st.get("PQTMPVT") != 1:
        return "the LC86G is not at 10 Hz with MSM7 and PQTMPVT on"
    if st.get("navmode") != a.start_mode:
        r, _ = link.pair(f"PAIR080,{a.start_mode}", 80)
        r2, f = link.pair("PAIR081", 81, reply="PAIR081")
        print(f"# set nav mode {a.start_mode}: ack {r}, read back {f}")
        if r != 0 or not f or int(f[0]) != a.start_mode:
            return "could not set the starting navigation mode"
    if not a.no_cold_start:
        r, _ = link.pair("PAIR006", 6, timeout=3.0)
        print(f"# cold start ($PAIR006): result {r}")
        if r != 0:
            return "cold start not acknowledged"
        time.sleep(2.0)
        r, f = link.pair("PAIR081", 81, reply="PAIR081")
        print(f"# after the cold start $PAIR081 reads {f}")
        if not f or int(f[0]) != a.start_mode:
            return "the navigation mode did not survive the cold start"
    sched = sorted((float(t), int(m)) for t, m in
                   (x.split(":") for x in a.switch.split(",") if x))

    tx = None
    if not a.dry:
        tx = start_tx(Path(a.c8), 1575420000, 2600000, 0, "/tmp/hackrf_tx_lc86bench.err")
        if tx is None:
            return "hackrf_transfer did not start"
    t0 = time.time()
    lead = 0.0 if a.dry else 4.0          # start_tx waits 4 s before returning

    pending = list(sched)
    queries = []                          # host times to send $PAIR081
    buf = bytearray()
    last_print = -99.0
    fix = used = None
    tow = None
    cells = []
    try:
        with open(a.out, "w") as fh:
            fh.write(f"0.000 # host: tx {Path(a.c8).name} gain 0 from {lead:.1f} s before "
                     f"t=0; start mode {a.start_mode}; switch {a.switch or 'none'}\n")
            while time.time() - t0 < a.seconds:
                t = time.time() - t0
                if pending and t >= pending[0][0]:
                    _, m = pending.pop(0)
                    ser.write(frame(f"PAIR080,{m}"))
                    fh.write(f"{t:.3f} # host: sent $PAIR080,{m} ({NAV_MODES.get(m)}); "
                             f"receiver TOW {tow}\n")
                    print(f"\n>>> t={t:6.1f}s  $PAIR080,{m} ({NAV_MODES.get(m)})\n")
                    queries.append(t + 1.5)
                if queries and t >= queries[0]:
                    queries.pop(0)
                    ser.write(frame("PAIR081"))
                    fh.write(f"{t:.3f} # host: sent $PAIR081\n")
                try:
                    chunk = ser.read(ser.in_waiting or 1)
                except serial.SerialException as exc:
                    fh.write(f"{t:.3f} # host: serial dropped ({exc})\n")
                    print(f"!! serial dropped at {t:.1f} s: {exc}")
                    ser.close()
                    for _ in range(40):
                        time.sleep(0.5)
                        p = find_bridge("auto")
                        if p:
                            try:
                                ser = open_bridge(p)
                                fh.write(f"{time.time() - t0:.3f} # host: reopened {p}\n")
                                break
                            except serial.SerialException:
                                pass
                    else:
                        break
                    continue
                if not chunk:
                    continue
                buf.extend(chunk)
                for kind, data in _demux(buf, rtcm=True):
                    t = time.time() - t0
                    if kind == "rtcm":
                        fh.write(f"{t:.3f} R {data.hex()}\n")
                        if rtcm3.message_type(data) == 1077:
                            r = rtcm3.parse_msm7(data)
                            if r:
                                cells = r[2]
                        continue
                    fh.write(f"{t:.3f} {data}\n")
                    if data.startswith("$PQTMPVT"):
                        f = data.split(",")
                        try:
                            tow = int(f[2]) / 1000.0 - T0_TOW
                            fix, used = int(f[6] or 0), int(f[7] or 0)
                        except (ValueError, IndexError):
                            pass
                    elif data.startswith("$PAIR001,08") or data.startswith("$PAIR081"):
                        print(f"    {data}")
                if t - last_print >= 15:
                    last_print = t
                    cn = [c["cn0"] for c in cells if c.get("cn0")]
                    lk = [c["lock_ms"] / 1000 for c in cells]
                    hc = sum(1 for c in cells if c.get("halfcyc"))
                    print(f"t={t:6.1f}s file~{(tow if tow is not None else t + lead):6.1f}s  "
                          f"fixmode {fix} used {used}  MSM7 cells {len(cells):2d}  "
                          f"C/N0 med {statistics.median(cn) if cn else 0:4.1f}  "
                          f"lock max {max(lk) if lk else 0:5.1f}s  halfcyc {hc}/{len(cells)}")
    except KeyboardInterrupt:
        print("# interrupted")
    finally:
        stop_tx(tx)
        ser.close()
    print(f"# capture -> {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
