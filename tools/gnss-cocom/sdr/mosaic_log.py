#!/usr/bin/env python3
"""Log a Septentrio mosaic-G5 on USB: SBF at 20 Hz with host timestamps.

Configures the receiver on every connection and saves nothing to its boot
configuration, so a power cycle returns it to factory state and the next
connection puts the test setup back:

    srd, High, Unlimited      the most dynamic tracking/PVT setting the receiver has
    Stream1  msec50   MeasEpoch MeasExtra PVTGeodetic (fw 1.0.0 has no PosCov/VelCov)
    Stream2  sec1     ReceiverStatus QualityInd RFStatus SatVisibility ChannelStatus
                      DOP ReceiverTime GALAuthStatus
    Stream3  OnChange navigation data, ReceiverSetup, Commands, RxMessage

--cmd adds commands after that (for the rig: --cmd "sna, Detection" --cmd
"sou, off, , off", so the simulator's signals are not dropped as suspect).

The log matches the rig captures: one line per message, `<host s> S <hex>` for
an SBF block, `# host:` comments, `# rx:` for the receiver's ASCII replies. Host
time is seconds since the logger started; the start's wall-clock time is in the
header. A .sbf copy of the blocks is written alongside for Septentrio tools.

    ./mosaic_log.py --minutes 120 --tag sky     # captures/mosaic_g5_sky_<stamp>.log
"""

from __future__ import annotations

import argparse
import datetime as dt
import re
import signal
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import serial                                           # noqa: E402
from serial.tools import list_ports                     # noqa: E402
import septentrio_sbf as sbf                            # noqa: E402

SEPTENTRIO_VID = 0x152A
PROMPT = re.compile(rb"(COM|USB|IP|NTR|IPS|DSK)\d{1,2}>")   # 'USB1>'; a reply's '---->' is not one

FAST = "MeasEpoch+MeasExtra+PVTGeodetic"     # fw 1.0.0 refuses PosCovGeodetic/VelCovGeodetic
SLOW = ("ReceiverStatus+QualityInd+RFStatus+SatVisibility+ChannelStatus+DOP+"
        "ReceiverTime+GALAuthStatus")
NAV = ("GPSNav+GPSCNav+GALNav+BDSNav+BDSCNav1+GLONav+QZSNav+GPSIon+GPSUtc+GALIon+GALUtc+GALGstGps+"
       "BDSIon+BDSUtc+GLOTime+ReceiverSetup+Commands+RxMessage")


def find_port() -> str | None:
    """First Septentrio CDC port (USB1 on macOS: the lower interface number)."""
    ports = sorted(p.device for p in list_ports.comports()
                   if p.vid == SEPTENTRIO_VID and p.device.startswith("/dev/cu."))
    return ports[0] if ports else None


class Link:
    """Serial link that keeps splitting SBF from text while commands run."""

    def __init__(self, port: str, log):
        self.s = serial.Serial(port, 115200, timeout=0.05, exclusive=True)
        self.split = sbf.Splitter()
        self.log = log
        self.text = b""
        self.prompt = None

    def pump(self) -> list[bytes]:
        """Read what is waiting; log every item; return the SBF blocks."""
        data = self.s.read(65536)
        blocks = []
        if not data:
            return blocks
        for kind, item in self.split.feed(data):
            if kind == "S":
                blocks.append(item)
                self.log.block(item)
            else:
                self.text += item
                *lines, self.text = self.text.replace(b"\r", b"").split(b"\n")
                for ln in lines:
                    if ln.strip():
                        self.log.rx(ln)
                if PROMPT.fullmatch(self.text):
                    self.log.rx(self.text)
                    self.prompt = self.text.decode("latin-1")
                    self.text = b""
        return blocks

    def command(self, line: str, timeout: float = 5.0) -> str:
        """Send one command; return the receiver's reply text ('$R: ...' or '$R? ...')."""
        self.log.note(f"cmd: {line}")
        start = len(self.log.rx_lines)
        self.prompt = None
        self.s.write(line.encode() + b"\r")
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump()
            if self.prompt and len(self.log.rx_lines) > start:
                break
        return "\n".join(self.log.rx_lines[start:])


class Log:
    def __init__(self, path: Path, t0: float):
        self.fh = path.open("w")
        self.sbf = path.with_suffix(".sbf").open("wb")
        self.t0 = t0
        self.rx_lines: list[str] = []
        self.counts: dict[int, int] = {}

    def t(self) -> float:
        return time.time() - self.t0

    def note(self, msg: str):
        self.fh.write(f"{self.t():.3f} # host: {msg}\n")
        self.fh.flush()

    def rx(self, line: bytes):
        text = line.decode("latin-1").rstrip()
        self.rx_lines.append(text)
        self.fh.write(f"{self.t():.3f} # rx: {text}\n")

    def block(self, blk: bytes):
        self.fh.write(f"{self.t():.3f} S {blk.hex()}\n")
        self.sbf.write(blk)
        bid = sbf.block_id(blk)[0]
        self.counts[bid] = self.counts.get(bid, 0) + 1

    def flush(self):
        self.fh.flush()
        self.sbf.flush()


def configure(link: Link, cd: str, args) -> bool:
    """Apply the test setup. Returns False if any command was refused."""
    ok = True
    cmds = [f"srd, {args.dynamics}",
            f"sso, Stream1, {cd}, {FAST}, {args.interval}",
            f"sso, Stream2, {cd}, {SLOW}, sec1",
            f"sso, Stream3, {cd}, {NAV}, OnChange"] + list(args.cmd)
    for c in cmds:
        reply = link.command(c)
        good = "$R:" in reply and "$R?" not in reply
        ok &= good
        print(f"  {'ok ' if good else 'BAD'} {c}" + ("" if good else f"\n      {reply.strip()}"), flush=True)
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="default: the first Septentrio port (USB1)")
    ap.add_argument("--minutes", type=float, default=120.0)
    ap.add_argument("--tag", default="sky")
    ap.add_argument("--dynamics", default="High, Unlimited")
    ap.add_argument("--interval", default="msec50", help="Stream1 rate (msec50 = 20 Hz)")
    ap.add_argument("--cmd", action="append", default=[], help="extra command after the setup")
    ap.add_argument("--keep-streams", action="store_true", help="leave the streams on at exit")
    args = ap.parse_args()

    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)

    t0 = time.time()
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M")
    path = HERE / "captures" / f"mosaic_g5_{args.tag}_{stamp}.log"
    path.parent.mkdir(exist_ok=True)
    log = Log(path, t0)
    log.note(f"start {dt.datetime.now().astimezone().isoformat(timespec='seconds')} (unix {t0:.3f}); "
             f"mosaic_log.py {' '.join(sys.argv[1:])}")
    print(f"logging to {path}", flush=True)

    end = t0 + args.minutes * 60
    link = None
    cd = "USB1"
    window = {"meas": [], "t": time.time(), "bytes": 0}
    last = {}
    try:
        while time.time() < end:
            if link is None:
                port = args.port or find_port()
                if not port:
                    time.sleep(1)
                    continue
                try:
                    link = Link(port, log)
                except (serial.SerialException, OSError) as exc:
                    log.note(f"open {port} failed ({exc})")
                    time.sleep(1)
                    continue
                log.note(f"opened {port}")
                # the prompt names the port; the streams must go to that port
                link.command("")
                if link.prompt:
                    cd = link.prompt.rstrip(">").strip()
                print(f"{time.strftime('%H:%M:%S')} connected {port} ({cd}); configuring", flush=True)
                if not configure(link, cd, args):
                    log.note("configuration refused; stopping")
                    print("configuration refused; see the log", flush=True)
                    return 1
                # OnChange only sends navigation data when it changes: ask for the current set once
                for c in ("lif, Identification", "lif, Permissions", "grc",
                          f"esoc, {cd}, GPSNav+GPSCNav+GALNav+BDSNav+BDSCNav1+GLONav+QZSNav+GPSUtc+GLOTime"):
                    link.command(c)
                log.note("configured")
            try:
                for blk in link.pump():
                    bid = sbf.block_id(blk)[0]
                    window["bytes"] += len(blk)
                    if bid == sbf.MEAS_EPOCH:
                        window["meas"].append(sbf.meas_epoch(blk))
                    elif bid == sbf.PVT_GEODETIC:
                        last["pvt"] = sbf.pvt_geodetic(blk)
                    elif bid == sbf.RECEIVER_STATUS:
                        last["rx"] = sbf.receiver_status(blk)
            except (serial.SerialException, OSError) as exc:
                log.note(f"serial dropped ({exc}); waiting for the receiver")
                print(f"{time.strftime('%H:%M:%S')} serial dropped; waiting", flush=True)
                try:
                    link.s.close()
                except Exception:
                    pass
                link = None
                continue
            if time.time() - window["t"] >= 10:
                log.flush()
                me = window["meas"]
                n = len(me)
                rate = n / (time.time() - window["t"])
                sats = len({m["sv"] for m in me[-1]["meas"]}) if me else 0
                sigs = len(me[-1]["meas"]) if me else 0
                cn = [m["cn0"] for m in me[-1]["meas"] if m["cn0"] is not None] if me else []
                p, r = last.get("pvt"), last.get("rx")
                fix = (f"{p['mode_name']} NrSV {p['nrsv']} H {p['hacc']} m" if p and p["mode"]
                       else f"no fix ({p['error_name']})" if p else "no PVT yet")
                rxs = (f"up {r['uptime']} s CPU {r['cpu']}% {r['temp_c']} C err 0x{r['rx_error']:x}" if r else "")
                print(f"{time.strftime('%H:%M:%S')} meas {rate:4.1f} Hz | sats {sats} sigs {sigs} "
                      f"C/N0 med {statistics.median(cn) if cn else 0:.0f} | {fix} | {rxs} | "
                      f"{window['bytes'] / 10240:.1f} kB/s | bad {link.split.bad}", flush=True)
                window = {"meas": [], "t": time.time(), "bytes": 0}
    except KeyboardInterrupt:
        log.note("stopped by user")
    finally:
        if link is not None and not args.keep_streams:
            try:
                for k in (1, 2, 3):
                    link.command(f"sso, Stream{k}, {cd}, none, off")
            except Exception:
                pass
        log.note("end; blocks " + " ".join(f"{k}:{v}" for k, v in sorted(log.counts.items())))
        log.flush()
    print(f"done: {path}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
