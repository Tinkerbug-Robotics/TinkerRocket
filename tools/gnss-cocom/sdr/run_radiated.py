#!/usr/bin/env python3
"""Run a scenario radiated into a Faraday cage, tapping the receiver's UART.

Differs from the conducted runs in three ways that matter:

* **The receiver is not configured by us.** The rocket computer owns it and sets
  it up at boot, including DYN_MODEL_AIRBORNE4g. This tool only listens, on a
  passive tap of the receiver's TXD line, so it cannot perturb the thing it is
  measuring. There is no seeded restart here; the SkyTraq `0x01` trick has a
  u-blox equivalent (UBX-CFG-RST) but injecting it would mean driving a line the
  rocket computer is also driving.
* **The tap is a generic USB-serial adapter**, so it cannot be found by USB
  vendor the way the passthrough board can. Pass --port, or run --list.
* **Level is set by pad and geometry, not by a cable.** Free-space loss at L1 is
  only 26 dB at 30 cm, so inside a cage the separation is a first-order term. It
  is also fixed by the box, which leaves TX gain as the only live adjustment.

Always confirm the transmitter is actually streaming before believing a null
result: a run where hackrf_transfer failed to open the device looks exactly like
a run where the receiver heard nothing.
"""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import serial                                          # noqa: E402
from serial.tools import list_ports                    # noqa: E402
from gnss_nmea_monitor import Parser, _demux           # noqa: E402
from ublox_binary import (CLS_NAV, MSG_NAV_PVT, MSG_NAV_SAT,   # noqa: E402
                          parse_nav_pvt, parse_nav_sat)
sys.path.insert(0, str(HERE))
from ensure_hackrf import ensure_hackrf                    # noqa: E402
from find_ublox import find_ublox                          # noqa: E402

# Per-scenario, not shared: an orphaned hackrf_transfer from an earlier run
# keeps appending "MB / " progress lines to whatever file it was given, which
# is enough to satisfy a naive "did it start?" check for a process that in
# fact died with hackrf_open() failed. That combination reported a healthy
# transmitter and a failed one in the same breath.
def tx_err_path(scenario):
    return f"/tmp/hackrf_tx_{scenario or 'probe'}.err"


def list_candidate_ports():
    print("serial ports:")
    for p in sorted(list_ports.comports(), key=lambda x: x.device):
        vid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "----:----"
        print(f"  {p.device:<28} {vid}  {p.manufacturer or ''} {p.product or ''}")
    print("\nThe tap is whichever adapter is clipped to the receiver's TXD.\n"
          "A u-blox on the rocket computer runs at 460800 (38400 factory default).")


def start_tx(c8: Path, freq: int, rate: int, gain: int, errfile: str, extra: tuple = ()):
    # The PortaPack boots into Mayhem and hides the radio; this is a no-op when
    # the HackRF is already there.
    if not ensure_hackrf():
        return None
    errf = open(errfile, "w")
    tx = subprocess.Popen(
        ["hackrf_transfer", "-t", str(c8), "-f", str(freq),
         "-s", str(rate), "-a", "0", "-x", str(gain), *extra],
        stdout=errf, stderr=subprocess.STDOUT)
    time.sleep(4.0)
    out = Path(errfile).read_text()
    # Check for the explicit failure first. "Access denied" here almost always
    # means another hackrf_transfer still holds the device -- killing a runner
    # by name leaves its child alive, because the child is not matched by the
    # parent's name.
    failed = ("hackrf_open() failed" in out or "not found" in out.lower()
              or tx.poll() is not None or "MB / " not in out)
    if failed:
        try:
            tx.kill()
        except Exception:
            pass
        print("!! hackrf_transfer is not streaming:")
        print(out[:400] or "(no output)")
        if "hackrf_open() failed" in out:
            print("   Another hackrf_transfer probably still holds the radio:\n"
                  "     pkill -f hackrf_transfer")
        return None
    print(f"# TX confirmed: {out.strip().splitlines()[-1]}")
    return tx


def stop_tx(tx):
    if tx is None:
        return
    tx.send_signal(signal.SIGINT)
    try:
        tx.wait(timeout=6)
    except subprocess.TimeoutExpired:
        tx.kill()


def preflight(port, baud, expect_dynmodel=8):
    """Refuse to fly a receiver that is not in the state we think it is.

    Configuration written to the RAM layer -- which is what this rig does, so a
    bench part goes home as its owner left it -- does not survive a power cycle,
    and a receiver on a USB-UART bridge power-cycles whenever the bridge does.
    A Beitian M10 silently reverted between its gain sweep and its flight: back
    to NMEA, NAV-PVT and NAV-SAT off, and DYNMODEL back to 0 (portable, whose
    ~12 km ceiling would have been measured and written up as an altitude gate.)

    Nothing downstream would have flagged it. correlate.py reads whatever
    arrives, and a portable-model ceiling looks exactly like a gate. Fourteen
    minutes of transmission, and a plausible wrong number at the end of it.
    """
    import serial
    from ublox_binary import iter_frames, CLS_NAV, MSG_NAV_PVT, MSG_NAV_SAT
    try:
        with serial.Serial(port, baud, timeout=0.3) as ser:
            time.sleep(0.3)
            ser.reset_input_buffer()
            buf = bytearray()
            t0 = time.time()
            while time.time() - t0 < 3.0:
                buf.extend(ser.read(ser.in_waiting or 1))
    except Exception as exc:
        return [f"cannot open {port}: {exc}"]

    raw = bytes(buf)
    seen = {(c, m) for c, m, _ in iter_frames(bytearray(raw))}
    problems = []
    if (CLS_NAV, MSG_NAV_PVT) not in seen:
        problems.append("no UBX NAV-PVT on the wire")
    if (CLS_NAV, MSG_NAV_SAT) not in seen:
        problems.append("no UBX NAV-SAT on the wire (no per-satellite C/N0 or elevation)")
    if raw.count(b"$G") > 5:
        problems.append(f"NMEA still enabled ({raw.count(b'$G')} sentences in 3 s)")
    return problems


def preflight_lc86(port, navmode, cold_start=False, rtcm=None):
    """The Tinker-Beetle's LC86G, through lc86_bridge: the same refusal.

    Its configuration is RAM-only, exactly as in flight, and it is lost
    whenever the flight computer's rail cycles -- after which the module
    streams its factory set at 1 Hz in Normal mode and would fly a
    different experiment without a word. So every setting is read back
    against the flight configuration plus the navigation mode this run is
    meant to measure, and a mismatch refuses the run.

    cold_start sends $PAIR006 first: time, position, almanac and ephemeris
    erased, configuration kept. That is how a Beetle comes up on the pad --
    its V_BCKP rides the same switched rail, so nothing survives a power-up
    -- and it stops a restarted scenario file from presenting the receiver
    with its own clock running backwards.
    """
    from lc86_config import (Link, open_bridge, read_state, show_state,
                             check_flight, FLIGHT_RATE_HZ)
    try:
        with open_bridge(port) as ser:
            link = Link(ser)
            if cold_start:
                r, _ = link.pair("PAIR006", 6, timeout=3.0)
                print(f"# cold start ($PAIR006): result {r}")
                if r != 0:
                    return [f"cold start not acknowledged (result {r})"]
                time.sleep(2.0)
            st = read_state(link)
    except Exception as exc:
        return [f"cannot open {port}: {exc}"]
    show_state(st)
    bad = check_flight(st, FLIGHT_RATE_HZ, navmode)
    if rtcm is not None:
        from lc86_config import RTCM_MODES
        if st.get("rtcm") != RTCM_MODES[rtcm]:
            bad.append(f"RTCM mode {st.get('rtcm')}, want {rtcm}")
    return bad


def reopen(ser, args, exc, fh, t):
    """Reopen a port that dropped mid-capture; None if it does not come back.

    Only a bridge found by identity is reopened -- a different device that
    happens to take the same path must never be read as this receiver.
    """
    print(f"\n!! serial dropped at t={t:.1f}s ({exc}); reopening")
    fh.write(f"{t:.3f} # host: serial dropped ({exc}); reopening\n")
    try:
        ser.close()
    except Exception:
        pass
    deadline = time.time() + 20
    while time.time() < deadline:
        time.sleep(0.5)
        port = args.port
        if args.lc86 is not None:
            from lc86_config import find_bridge
            port = find_bridge("auto")
        if not port:
            continue
        try:
            s = serial.Serial(port, args.baud, timeout=0.5)
        except serial.SerialException:
            continue
        fh.write(f"{t:.3f} # host: reopened {port}\n")
        print(f"# reopened {port}")
        return s
    fh.write(f"{t:.3f} # host: port did not come back in 20 s\n")
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("-s", "--scenario", help="scenario name, e.g. spaceshot")
    ap.add_argument("-p", "--port", help="serial port, or 'auto' for a u-blox")
    ap.add_argument("-b", "--baud", type=int, default=460800)
    ap.add_argument("-x", "--gain", type=int, default=40, help="HackRF TX gain 0-47")
    ap.add_argument("--freq", type=int, default=1575420000)
    ap.add_argument("--rate", type=int, default=2600000)
    ap.add_argument("--seconds", type=float, help="override capture length")
    ap.add_argument("--skip-preflight", action="store_true",
                    help="transmit without checking the receiver's output first")
    ap.add_argument("--listen-only", action="store_true",
                    help="tap the receiver without transmitting, to check wiring")
    ap.add_argument("--lc86", type=int, metavar="NAVMODE",
                    help="the Tinker-Beetle's LC86G through lc86_bridge, expected "
                         "in this $PAIR080 navigation mode (3 = Balloon, as flown "
                         "from PR #1500; 0 = Normal, as flown before it). Finds "
                         "the port by the flight computer's MAC and replaces the "
                         "UBX preflight with a full read-back")
    ap.add_argument("--cold-start", action="store_true",
                    help="with --lc86: $PAIR006 before transmitting, as a Beetle "
                         "powering up on the pad")
    ap.add_argument("--rtcm", choices=["off", "msm4", "msm7"],
                    help="with --lc86: the RTCM output mode the module must be "
                         "in (set it with lc86_config.py --rtcm)")
    ap.add_argument("--tag", help="capture name prefix, e.g. lc86g_normal; the "
                         "capture is captures/TAG_SCENARIO.log")
    args = ap.parse_args()

    if args.list:
        list_candidate_ports()
        return 0
    if args.lc86 is not None:
        from lc86_config import find_bridge
        args.port = find_bridge(args.port)
    else:
        args.port = find_ublox(args.port)
    if not args.port:
        list_candidate_ports()
        return "\n--port is required (see the list above)"

    c8 = meta = None
    if not args.listen_only:
        if not args.scenario:
            return "--scenario is required unless --listen-only"
        c8 = HERE / "c8" / f"{args.scenario}.C8"
        sc = HERE / "scenarios" / f"{args.scenario}.json"
        if not c8.exists():
            return f"no {c8}"
        meta = json.loads(sc.read_text()) if sc.exists() else None

    dur = args.seconds or (meta["duration_s"] + 15 if meta else 120)

    if args.lc86 is not None and not args.listen_only and not args.skip_preflight:
        bad = preflight_lc86(args.port, args.lc86, args.cold_start, args.rtcm)
        if bad:
            print("!! the LC86G is not configured for this measurement:")
            for b in bad:
                print(f"     {b}")
            print("   Run lc86_config.py (add --navmode 0 for Normal; the default "
                  "is the flight's Balloon).\n"
                  "   The configuration is RAM-only and a rail cycle loses it.")
            return 1
        print("# preflight: LC86G in the flight configuration, nav mode "
              f"{args.lc86} as expected")
    elif not args.listen_only and not args.skip_preflight:
        bad = preflight(args.port, args.baud)
        if bad:
            print("!! the receiver is not configured for this measurement:")
            for b in bad:
                print(f"     {b}")
            print("   Run ubx_config.py before flying, or pass --skip-preflight to\n"
                  "   transmit anyway. RAM-layer configuration does not survive a\n"
                  "   power cycle, and the bridge power-cycles with the receiver.")
            return 1
        print("# preflight: UBX NAV-PVT and NAV-SAT present, NMEA quiet")

    tx = None
    if not args.listen_only:
        # meta is None whenever the .C8 has no scenario sidecar -- the static
        # probe files have none, and they are exactly what a level check uses.
        span = (f"{meta['duration_s']:.0f}s" if meta
                else f"{c8.stat().st_size/(2*args.rate):.0f}s (no sidecar)")
        print(f"# {args.scenario}: {c8.stat().st_size/1e9:.2f} GB, "
              f"{span}, gain {args.gain}")
        tx = start_tx(c8, args.freq, args.rate, args.gain,
                      tx_err_path(args.scenario))
        if tx is None:
            return 1
    else:
        print("# listen-only: not transmitting")

    name = (f"{args.tag}_{args.scenario}" if args.tag and args.scenario else
            f"{args.scenario}_radiated" if args.scenario else "tap_check")
    cap = HERE / "captures" / f"{name}.log"
    cap.parent.mkdir(exist_ok=True)
    print(f"# tap {args.port} @ {args.baud}, logging to {cap.name}\n")
    print(f"{'t':>6} {'fix':<10} {'verdict':<8} {'sats':>5} {'used':>5} "
          f"{'lat':>10} {'lon':>11} {'alt m':>9} {'3D m/s':>8}")
    print("-" * 78)

    p = Parser()
    nsat = used = 0
    last = -99.0
    nbytes = nframes = 0
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
        with open(cap, "w") as fh:
            buf = bytearray()
            t0 = time.time()
            while time.time() - t0 < dur:
                try:
                    chunk = ser.read(ser.in_waiting or 1)
                except serial.SerialException as exc:
                    # A native-USB bridge (lc86_bridge) re-enumerates if its
                    # chip resets, and a port-state change can do that. The
                    # receiver is not on that chip's reset and keeps running,
                    # so reopen and carry on rather than abandon a flight --
                    # and say so in the capture, where the gap will show.
                    ser = reopen(ser, args, exc, fh, time.time() - t0)
                    if ser is None:
                        break
                    continue
                if not chunk:
                    continue
                nbytes += len(chunk)
                buf.extend(chunk)
                for kind, data in _demux(buf, rtcm=args.lc86 is not None):
                    t = time.time() - t0
                    nframes += 1
                    if kind == "rtcm":
                        fh.write(f"{t:.3f} R {data.hex()}\n")
                        continue
                    if kind == "ubx":
                        fh.write(f"{t:.3f} U {data.hex()}\n")
                        p.feed_ubx(data)
                    elif kind == "bin":
                        fh.write(f"{t:.3f} B {data.hex()}\n")
                        p.feed_binary(data)
                    else:
                        fh.write(f"{t:.3f} {data}\n")
                        p.feed(data)
                        # An NMEA receiver has no NAV-PVT to trigger the row
                        # below, so drive the display off GGA instead -- without
                        # this the run looks dead for its whole duration while
                        # capturing perfectly.
                        if data[3:6] == "GGA" and t - last >= 5:
                            last = t
                            e = p.epoch
                            print(f"{t:>6.0f} {'FIX' if e.verdict()=='FIX' else '-':<10} "
                                  f"{e.verdict():<8} {e.tracked_sats():>5} "
                                  f"{'':>5} {'':>10} {'':>11} {'':>9} "
                                  f"{(e.mean_cn0() or 0):>8.0f}")
                        continue
                    if kind == "ubx" and len(data) >= 2 and \
                            data[0] == CLS_NAV and data[1] == MSG_NAV_SAT:
                        ss = parse_nav_sat(data[2:]) or []
                        best = {}
                        for x in ss:
                            k = (x["gnss_name"], x["svid"])
                            best[k] = max(best.get(k, -99), x["cn0"])
                        nsat = sum(1 for v in best.values() if v >= 30)
                        used = len({(x["gnss_name"], x["svid"])
                                    for x in ss if x["used_in_fix"]})
                    if kind == "ubx" and len(data) >= 2 and \
                            data[0] == CLS_NAV and data[1] == MSG_NAV_PVT:
                        r = parse_nav_pvt(data[2:])
                        if not r or t - last < 5:
                            continue
                        last = t
                        e = p.epoch
                        print(f"{t:>6.0f} {r['fix_type_name']:<10} "
                              f"{e.verdict():<8} {nsat:>5} {used:>5} "
                              f"{r['lat']:>10.5f} {r['lon']:>11.5f} "
                              f"{r['alt_m']:>9.0f} {r['speed_mps']:>8.0f}")
    except serial.SerialException as exc:
        print(f"\n!! serial: {exc}")
    except KeyboardInterrupt:
        print("\n# interrupted")
    finally:
        stop_tx(tx)
        try:
            ser.close()
        except Exception:
            pass

    print(f"\n# {nbytes} bytes, {nframes} frames -> {cap}")
    if nframes == 0:
        print("# Nothing arrived. Check the tap is on the receiver's TXD (not the\n"
              "# rocket computer's), that ground is common, and the baud rate --\n"
              "# a u-blox on this firmware runs 460800, not the 38400 default.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
