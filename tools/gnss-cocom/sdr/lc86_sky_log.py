#!/usr/bin/env python3
"""Log the Beetle's LC86G on the real sky through lc86_bridge. Listens only: no transmitter.

Waits for the flight computer on USB (by MAC), makes sure the module is in the wanted
configuration (10 Hz, flight message set, MSM7, --navmode), re-applying it after any
power cycle, and logs everything with host timestamps until stopped. Survives unplug
and replug. If the configuration survived (battery kept the rail up), the first
connection cold-starts the module so it forgets the simulated sky.

A line written to --cmd-file (e.g. 'PAIR080,3') is sent to the module and logged.
"""
from __future__ import annotations

import argparse
import statistics
import subprocess
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
                         open_bridge, read_state)


def configured(st, navmode):
    rates = st.get("nmea_rates") or {}
    return (st.get("navmode") == navmode and st.get("fix_interval_ms") == 100
            and st.get("rtcm") == 1 and st.get("PQTMPVT") == 1
            and rates.get(0) == 1 and rates.get(3) == 10)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--navmode", type=int, default=5)
    ap.add_argument("--out", required=True)
    ap.add_argument("--cmd-file", required=True)
    a = ap.parse_args()

    t0 = time.time()
    first = True
    Path(a.cmd_file).write_text("")
    fh = open(a.out, "a")
    fh.write(f"{0.0:.3f} # host: sky log start {time.strftime('%Y-%m-%d %H:%M:%S')} "
             f"local, want nav mode {a.navmode} ({NAV_MODES.get(a.navmode)}); listen only\n")
    fh.flush()
    last_wait = -99.0
    while True:
        port = find_bridge("auto")
        t = time.time() - t0
        if not port:
            if t - last_wait >= 20:
                last_wait = t
                print(f"t={t:6.0f}s  waiting for the Beetle FC on USB (S1 on F)", flush=True)
            time.sleep(1.0)
            continue
        try:
            ser = open_bridge(port)
            link = Link(ser)
            st = read_state(link)
        except (serial.SerialException, OSError) as exc:
            print(f"t={t:6.0f}s  {port} not ready ({exc}); retrying", flush=True)
            time.sleep(2.0)
            continue
        if st.get("navmode") is None and st.get("fix_interval_ms") is None:
            print(f"t={t:6.0f}s  the bridge is up but the LC86G does not answer -- is the "
                  f"flight-computer rail on (app, BLE command 8)?", flush=True)
            fh.write(f"{t:.3f} # host: LC86G not answering on {port}\n"); fh.flush()
            ser.close(); time.sleep(5.0)
            continue
        if not configured(st, a.navmode):
            print(f"t={t:6.0f}s  configuration lost (nav {st.get('navmode')}, "
                  f"{st.get('fix_interval_ms')} ms, rtcm {st.get('rtcm')}): re-applying, "
                  f"nav mode {a.navmode}", flush=True)
            fh.write(f"{t:.3f} # host: configuration lost; re-applying nav mode {a.navmode}\n")
            fh.flush()
            ser.close()
            subprocess.run([sys.executable, str(SDR / "lc86_config.py"), "-p", port,
                            "--navmode", str(a.navmode), "--rtcm", "msm7"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=90)
            continue                      # re-read and verify on the next pass
        if first:
            r, _ = link.pair("PAIR006", 6, timeout=3.0)
            print(f"t={t:6.0f}s  configuration intact: cold start to forget the simulated "
                  f"sky, ack {r}", flush=True)
            fh.write(f"{t:.3f} # host: cold start ($PAIR006) ack {r}\n")
        first = False
        print(f"t={t:6.0f}s  logging {port}: nav mode {st['navmode']} "
              f"({NAV_MODES.get(st['navmode'])}), 10 Hz, MSM7", flush=True)
        fh.write(f"{t:.3f} # host: logging {port}, nav mode {st['navmode']}\n"); fh.flush()

        buf = bytearray()
        last = {}
        pvt = (None, None)
        last_print = -99.0
        try:
            while True:
                t = time.time() - t0
                cmd = Path(a.cmd_file).read_text().strip()
                if cmd:
                    Path(a.cmd_file).write_text("")
                    ser.write(frame(cmd))
                    fh.write(f"{t:.3f} # host: sent ${cmd}\n")
                    print(f"t={t:6.0f}s  >>> ${cmd}", flush=True)
                    if cmd.startswith("PAIR080"):
                        time.sleep(1.0)
                        ser.write(frame("PAIR081"))
                        fh.write(f"{time.time() - t0:.3f} # host: sent $PAIR081\n")
                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    buf.extend(chunk)
                    for kind, data in _demux(buf, rtcm=True):
                        tt = time.time() - t0
                        if kind == "rtcm":
                            fh.write(f"{tt:.3f} R {data.hex()}\n")
                            r = rtcm3.parse_msm7(data)
                            if r:
                                last[r[0]] = r[2]
                            continue
                        fh.write(f"{tt:.3f} {data}\n")
                        if data.startswith("$PQTMPVT"):
                            f = data.split(",")
                            try:
                                pvt = (int(f[6] or 0), int(f[7] or 0))
                            except (ValueError, IndexError):
                                pass
                        elif data.startswith(("$PAIR001,080", "$PAIR081")) or "silent" in data:
                            print(f"t={tt:6.0f}s    {data}", flush=True)
                if t - last_print >= 15:
                    last_print = t
                    fh.flush()
                    cells = [c for cs in last.values() for c in cs]
                    cn = [c["cn0"] for c in cells if c.get("cn0")]
                    hc = sum(1 for c in cells if c.get("halfcyc"))
                    lk = [c["lock_ms"] / 1000 for c in cells]
                    per = " ".join(f"{k[:3]}{len(v)}" for k, v in sorted(last.items()))
                    print(f"t={t:6.0f}s  fixmode {pvt[0]} used {pvt[1]}  MSM7 {per or '-'}  "
                          f"C/N0 med {statistics.median(cn) if cn else 0:4.1f}  "
                          f"lock max {max(lk) if lk else 0:5.1f}s  halfcyc {hc}/{len(cells)}",
                          flush=True)
        except (serial.SerialException, OSError) as exc:
            t = time.time() - t0
            print(f"t={t:6.0f}s  USB dropped ({exc}); waiting to reconnect", flush=True)
            fh.write(f"{t:.3f} # host: USB dropped ({exc})\n"); fh.flush()
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(1.0)


if __name__ == "__main__":
    raise SystemExit(main())
