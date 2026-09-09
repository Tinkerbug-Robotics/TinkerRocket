#!/usr/bin/env python3
"""Scripted bench session driver (#1211) — BLE commands + serial console on one timeline.

`bench_ble_cmd.py` connects, sends one command and drops the link.  Most of the
#1211 backlog needs more than that: a second Calibrate tap *inside* the first
one's 10 s window, a sim started *during* a cal, a power-off sent 5 s after the
FC went silent.  Those are sequences on a single connection, and the evidence is
a serial log line landing in the right place relative to a BLE write.

So this holds one BLE connection, tails the serial console at the same time, and
merges both into one timestamped timeline.  Assertions are directives in a
script, so a run produces a PASS/FAIL list rather than a wall of text to read.

The two streams are independent: BLE reaches whichever MCU owns the radio, the
serial console shows whichever MCU S1 selects.  Point S1 at the console whose
log lines the checklist quotes, and drive everything over BLE.

Usage:
  python3 tools/bench_session.py --name TR-R --port /dev/cu.usbmodem2101 \
      --script tests/bench/1131_railoff_readback.txt --out runs/1131

Script directives (one per line, '#' comments, blank lines ignored):
  note <text>                     annotate the timeline
  send <cmd> [hexpayload]         write [cmd][payload] to the COMMAND char
  sleep <seconds>
  expect_serial <timeout> <regex>     PASS when a console line matches
  refute_serial <window> <regex>      PASS when NO line matches for <window> s
  expect_tlm <timeout> <cond>         PASS when a telemetry frame satisfies cond
  refute_tlm <window> <cond>          PASS when no frame satisfies cond
  wait_tlm <timeout> <cond>           like expect_tlm but not scored (setup step)
  expect_tlm_rate <window> <min_hz>   PASS if telemetry keeps arriving (feed alive)
  expect_tlm_nogap <window> <max_gap> PASS if no telemetry gap exceeds max_gap s
  mark                                set the search origin for the three below
  expect_serial_since <regex>         PASS if a line since `mark` matched (history)
  refute_serial_since <regex>         PASS if NO line since `mark` matched
  expect_order <regexA> || <regexB>   PASS if A's first match precedes B's
  expect_fileops <hexprefix> <n>      PASS on exactly n file-ops frames since mark
  wait_quiet <regex> <quiet_s> <timeout>  wait until regex stops appearing
  disconnect / connect                drop and retake the BLE link (#1124)

Conditions read the telemetry JSON:
  st==INFLIGHT        state string
  nsat>=4             any numeric key
  fs.SIM_ACTIVE==1    named bit of the BLE fs field (NOT NonSensorData.flags)
  ps.CH1_FIRED==0     named bit of the BLE ps field (ARMED, then cont/fired pairs)
  h.STORAGE==OK       named 2-bit SensorHealthState field
Ops: == != >= <= > <

Note on `h` and `ps`: the OC omits them when zero to save MTU, and the whole
frame is trimmed tail-first under a small MTU ("tr":1 marks a trimmed frame).
A missing key is therefore NOT a zero — conditions on an absent key never match,
and expect_tlm will time out rather than quietly report a false PASS.
"""

import argparse
import asyncio
import json
import re
import sys
import threading
import time
from pathlib import Path

from bleak import BleakClient, BleakScanner

SERVICE_UUID   = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
TELEMETRY_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
COMMAND_UUID   = "cba1d466-344c-4be3-ab3f-189f80dd7518"
# Some readbacks do not ride telemetry: SENSOR_CAL_STATUS (#132) is a binary
# frame on file-ops with a 0xCB discriminator (mag cal is 0xCA).  Counting
# those is the only way to check "exactly ONE status frame" assertions.
FILE_OPS_UUID  = "8d53dc1d-1db7-4cd3-868b-8a527460aa84"

SCAN_TIMEOUT_S = 15.0

# The BLE telemetry "fs" bitfield — buildTelemetryJSON() in TR_BLE_To_APP.cpp.
#
# NOT NonSensorData.flags.  The generated protocol reference documents a field
# by that name with a DIFFERENT layout, only 8 bits wide; "fs" is a BLE-only
# packing built for the app.  Using the wrong table reads sim_active off
# bs_logging_active and silently reports 0 through an entire sim flight.
NSF_BITS = {
    "LAUNCH": 0, "VEL_APOGEE": 1, "ALT_APOGEE": 2, "ALT_LANDED": 3,
    "PWR_PIN_ON": 4, "CAMERA_RECORDING": 5, "LOGGING_ACTIVE": 6,
    "BS_LOGGING_ACTIVE": 7, "SIM_ACTIVE": 8, "BURNOUT": 9,
}

# The BLE telemetry "ps" bitfield — buildTelemetryJSON() again.  One ARMED bit
# then a (cont, fired) PAIR per channel, so the shifts are NOT one-per-channel.
# ps==10 on a board with dummy loads on 1 and 2 is CH1_CONT|CH2_CONT, and a
# test fire on 2 takes it to 26.
PS_BITS = {
    "ARMED": 0,
    "CH1_CONT": 1, "CH1_FIRED": 2,
    "CH2_CONT": 3, "CH2_FIRED": 4,
    "CH3_CONT": 5, "CH3_FIRED": 6,
    "CH4_CONT": 7, "CH4_FIRED": 8,
}

# Sensor health: two bits per sensor at these shifts.
SH_SHIFTS = {
    "BARO": 0, "IMU": 2, "EKF": 4, "MAG": 6, "GNSS": 8, "BATT": 10,
    "PYRO1": 12, "PYRO2": 14, "PYRO3": 16, "PYRO4": 18,
    "STORAGE": 20, "GNSS_ABSENT": 22,
}
SH_STATES = {0: "NA", 1: "OK", 2: "DEGRADED", 3: "BAD"}


class Timeline:
    """Merged, timestamped record of everything both streams saw."""

    def __init__(self, t0):
        self.t0 = t0
        self.events = []
        self.lock = threading.Lock()

    def add(self, kind, text, data=None):
        ev = {"t": round(time.time() - self.t0, 3), "kind": kind, "text": text}
        if data is not None:
            ev["data"] = data
        with self.lock:
            self.events.append(ev)
        print(f"[{ev['t']:8.3f}] {kind:9s} {text}", flush=True)
        return ev

    def since(self, t_start, kinds):
        with self.lock:
            return [e for e in self.events if e["t"] >= t_start and e["kind"] in kinds]


class SerialTail(threading.Thread):
    """Console reader that does NOT reset the board on attach.

    DTR/RTS must be driven low *before* open() or the USB-serial/JTAG bridge
    pulses the target into a reboot — the trap that cost a V8 board a session.
    """

    daemon = True

    def __init__(self, port, baud, timeline, label=None):
        super().__init__()
        self.port, self.baud, self.timeline = port, baud, timeline
        self.label = label
        self.stop_flag = threading.Event()
        self.error = None

    def run(self):
        try:
            import serial
        except ImportError:
            self.error = "pyserial not installed (pip install pyserial)"
            return
        try:
            s = serial.Serial()
            s.port, s.baudrate, s.timeout = self.port, self.baud, 0.2
            s.dtr = False          # before open(), or the attach resets the target
            s.rts = False
            s.open()
            s.dtr = False
            s.rts = False
        except Exception as exc:                      # noqa: BLE001
            self.error = f"serial open failed on {self.port}: {exc}"
            return

        buf = b""
        while not self.stop_flag.is_set():
            try:
                chunk = s.read(4096)
            except Exception as exc:                  # noqa: BLE001
                self.timeline.add("serial-err", str(exc))
                break
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", "replace").rstrip("\r")
                    if text:
                        # A labelled line reads "[BS] I (123) ...", so a script can
                        # aim an assertion at one console with ^\[BS\] while an
                        # unlabelled single-port run keeps its bare text.
                        if self.label:
                            text = f"[{self.label}] {text}"
                        self.timeline.add("serial", text)
        try:
            s.close()
        except Exception:                             # noqa: BLE001
            pass


def parse_cond(cond):
    """'fs.SIM_ACTIVE==1' -> (key, op, value). Ops are matched longest-first."""
    for op in ("==", "!=", ">=", "<=", ">", "<"):
        if op in cond:
            key, _, val = cond.partition(op)
            return key.strip(), op, val.strip()
    raise SystemExit(f"cannot parse condition: {cond!r}")


def cond_holds(frame, cond):
    """Evaluate a condition against one decoded telemetry frame."""
    key, op, want = parse_cond(cond)

    if key.startswith("fs."):
        name = key[3:].removeprefix("NSF_")
        if name not in NSF_BITS:
            raise SystemExit(f"unknown NSF bit {name!r}; known: {sorted(NSF_BITS)}")
        if "fs" not in frame:
            return False          # absent != zero: the frame may be trimmed
        have = (int(frame["fs"]) >> NSF_BITS[name]) & 1
        return compare(have, op, int(want))

    if key.startswith("ps."):
        name = key[3:]
        if name not in PS_BITS:
            raise SystemExit(f"unknown pyro bit {name!r}; known: {sorted(PS_BITS)}")
        if "ps" not in frame:
            return False          # absent != zero, same trap as "fs"
        have = (int(frame["ps"]) >> PS_BITS[name]) & 1
        return compare(have, op, int(want))

    if key.startswith("h."):
        name = key[2:].removeprefix("SH_")
        if name not in SH_SHIFTS:
            raise SystemExit(f"unknown health field {name!r}; known: {sorted(SH_SHIFTS)}")
        if "h" not in frame:
            return False          # the OC omits "h" entirely when it is zero
        have = (int(frame["h"]) >> SH_SHIFTS[name]) & 0x3
        want_num = {v: k for k, v in SH_STATES.items()}.get(want.upper())
        return compare(have, op, want_num if want_num is not None else int(want))

    if key not in frame:
        return False
    have = frame[key]
    # Config readbacks carry real JSON booleans ("p1e":true), so normalise both
    # sides before comparing — otherwise `p1e==true` falls through to a string
    # compare of "True" against "true" and silently reports a FAIL.
    if isinstance(have, bool) or want.lower() in ("true", "false"):
        return compare(bool(have), op, want.lower() == "true")
    if isinstance(have, str):
        return compare(have, op, want)
    try:
        return compare(float(have), op, float(want))
    except ValueError:
        return compare(str(have), op, want)


def compare(a, op, b):
    return {
        "==": lambda: a == b, "!=": lambda: a != b,
        ">=": lambda: a >= b, "<=": lambda: a <= b,
        ">":  lambda: a > b,  "<":  lambda: a < b,
    }[op]()


def describe_health(frame):
    """Human-readable health decode — the pre-launch scorecard, numerically."""
    if "h" not in frame:
        return "h absent (all NA/zero, or trimmed)"
    h = int(frame["h"])
    return " ".join(f"{n}={SH_STATES[(h >> s) & 3]}" for n, s in SH_SHIFTS.items())


class Session:
    def __init__(self, args, timeline):
        self.args = args
        self.tl = timeline
        self.client = None
        self.dev = None
        self.frames = []          # (t, decoded dict)
        self.mark = 0.0           # time origin for the *_since assertions
        self.mark_frame = 0       # frame-index origin for expect_tlm/refute_tlm
        self.results = []         # (status, directive, detail)

    # ---- BLE plumbing -----------------------------------------------------

    def on_notify(self, _handle, data: bytearray):
        raw = data.decode("utf-8", "replace")
        try:
            frame = json.loads(raw)
        except json.JSONDecodeError:
            self.tl.add("tlm-bad", raw)
            return
        self.frames.append((time.time() - self.tl.t0, frame))
        self.tl.add("tlm", raw, data=frame)

    async def find(self):
        if self.args.address:
            return self.args.address
        found = await BleakScanner.discover(timeout=SCAN_TIMEOUT_S, return_adv=True)
        for d, ad in found.values():
            name = d.name or ""
            svc = SERVICE_UUID.lower() in [u.lower() for u in (ad.service_uuids or [])]
            if self.args.name.lower() in name.lower() or (svc and not self.args.name):
                return d
        raise SystemExit(f"no device matching {self.args.name!r}")

    def on_fileops(self, _handle, data: bytearray):
        self.tl.add("fileops", data.hex())

    async def connect(self):
        # #1124 needs the APP to hold the link (only a real app download stalls
        # loop_oc, and only a real radio-off drops it the way the test means),
        # and BLE is single-connection — so the harness has to stay off the
        # radio and score the console alone.
        if self.args.no_ble:
            self.tl.add("ble", "serial-only run (--no-ble): the link belongs to another central")
            return
        if self.dev is None:
            self.dev = await self.find()
        label = getattr(self.dev, "name", self.dev)
        self.client = BleakClient(self.dev)
        await self.client.connect()
        await self.client.start_notify(TELEMETRY_UUID, self.on_notify)
        try:
            await self.client.start_notify(FILE_OPS_UUID, self.on_fileops)
        except Exception as exc:                      # noqa: BLE001
            # Not fatal: only the frame-counting assertions need this one.
            self.tl.add("ble-err", f"file-ops subscribe failed: {exc}")
        self.tl.add("ble", f"connected to {label}")

    async def disconnect(self):
        if self.args.no_ble:
            return
        if self.client and self.client.is_connected:
            await self.client.disconnect()
        self.tl.add("ble", "disconnected")

    # ---- directives -------------------------------------------------------

    def record(self, status, directive, detail=""):
        self.results.append((status, directive, detail))
        self.tl.add("RESULT", f"{status}: {directive}" + (f" — {detail}" if detail else ""))

    async def wait_serial(self, timeout, pattern, want_match, directive, scored=True):
        rx = re.compile(pattern)
        start = self.mark if want_match else (time.time() - self.tl.t0)
        deadline = time.time() + timeout
        hit = None
        while time.time() < deadline:
            for ev in self.tl.since(start, {"serial"}):
                if rx.search(ev["text"]):
                    hit = ev
                    break
            if hit and want_match:
                break
            if hit and not want_match:
                break
            await asyncio.sleep(0.05)
        if not scored:
            return hit
        if want_match:
            self.record("PASS" if hit else "FAIL", directive,
                        hit["text"] if hit else f"no console line matched in {timeout}s")
        else:
            self.record("FAIL" if hit else "PASS", directive,
                        f"matched: {hit['text']}" if hit else f"absent for {timeout}s")
        return hit

    def serial_since_mark(self):
        return self.tl.since(self.mark, {"serial"})

    def check_serial_since(self, pattern, want_match, directive):
        """Assert over console history since the last `mark`, without waiting.

        The forward-looking expect_serial cannot see a line that already
        scrolled past — and the boot lines a reboot test cares about are always
        in the past by the time the script reaches the assertion.
        """
        rx = re.compile(pattern)
        hits = [e for e in self.serial_since_mark() if rx.search(e["text"])]
        if want_match:
            self.record("PASS" if hits else "FAIL", directive,
                        hits[0]["text"] if hits else
                        f"no console line since mark matched {pattern!r}")
        else:
            self.record("FAIL" if hits else "PASS", directive,
                        f"matched: {hits[0]['text']}" if hits else "absent since mark")
        return hits

    @staticmethod
    def board_ms(text):
        """Milliseconds from an ESP log prefix like 'I (1354) TAG: ...'."""
        # Tolerate the "[LABEL] " prefix a multi-port capture prepends; without
        # this the board clock is never found on a labelled run and ordering
        # silently degrades to wall-clock arrival, which is the very artefact
        # board time exists to avoid.
        m = re.match(r"^(?:\[[^\]]+\]\s*)?[EWIDV] \((\d+)\)", text)
        return int(m.group(1)) if m else None

    def check_order(self, pat_a, pat_b, directive):
        """PASS when the FIRST match of A precedes the first match of B.

        #1131 turns on exactly this: the NVS config load must happen inside
        setup_oc, i.e. BEFORE any rail line — an ordering a regex alone cannot
        express, and the whole point of the fix.
        """
        rx_a, rx_b = re.compile(pat_a), re.compile(pat_b)
        first_a = next((e for e in self.serial_since_mark() if rx_a.search(e["text"])), None)
        first_b = next((e for e in self.serial_since_mark() if rx_b.search(e["text"])), None)
        if first_a is None:
            self.record("FAIL", directive, f"{pat_a!r} never appeared")
        elif first_b is None:
            # B absent is not an ordering failure: on this board the rail-off
            # path may simply not print a rail line at all.
            self.record("PASS", directive,
                        f"{pat_a!r} at t={first_a['t']}; {pat_b!r} never appeared")
        else:
            ms_a, ms_b = self.board_ms(first_a["text"]), self.board_ms(first_b["text"])
            if ms_a is not None and ms_b is not None:
                ok, unit, va, vb = ms_a <= ms_b, "ms board time", ms_a, ms_b
            else:
                ok, unit, va, vb = (first_a["t"] <= first_b["t"], "s wall clock",
                                    first_a["t"], first_b["t"])
            self.record("PASS" if ok else "FAIL", directive,
                        f"{pat_a!r} at {va} {unit} "
                        f"{'precedes' if ok else 'came AFTER'} {pat_b!r} at {vb}")

    async def wait_tlm(self, timeout, cond, want_match, directive, scored=True):
        start_idx = self.mark_frame
        deadline = time.time() + timeout
        hit = None
        while time.time() < deadline:
            for _t, frame in self.frames[start_idx:]:
                if cond_holds(frame, cond):
                    hit = frame
                    break
            if hit:
                break
            await asyncio.sleep(0.05)
        if not scored:
            return hit
        if want_match:
            self.record("PASS" if hit else "FAIL", directive,
                        json.dumps(hit) if hit else
                        f"no frame satisfied it in {timeout}s "
                        f"({len(self.frames) - start_idx} frames seen)")
        else:
            self.record("FAIL" if hit else "PASS", directive,
                        json.dumps(hit) if hit else f"never true over {timeout}s")
        return hit

    async def tlm_gap(self, window, max_gap_s, directive):
        """PASS when no telemetry gap exceeds max_gap_s over the window.

        #1114's defect was a 10 s blackout, not a slow feed — and the bench feed
        idles at 1 Hz, so a rate floor would condemn a healthy board.
        """
        start_idx = len(self.frames)
        await asyncio.sleep(window)
        times = [t for t, f in self.frames[start_idx:] if "st" in f]
        gaps = [round(b - a, 2) for a, b in zip(times, times[1:]) if b - a > max_gap_s]
        n = len(times)
        worst = max([round(b - a, 2) for a, b in zip(times, times[1:])], default=None)
        detail = f"{n} frames in {window}s, worst gap {worst}s (limit {max_gap_s}s)"
        if gaps:
            detail += f"; over limit: {gaps}"
        self.record("PASS" if n >= 2 and not gaps else "FAIL", directive, detail)

    async def tlm_rate(self, window, min_hz, directive):
        """The '#1114: the BLE feed must keep updating through the 10 s' check.

        Counts only real telemetry frames — a config readback burst rides the
        same characteristic, and letting those count would paper over exactly
        the 10 s hole this check exists to find.
        """
        start_idx = len(self.frames)
        await asyncio.sleep(window)
        live = [(t, f) for t, f in self.frames[start_idx:] if "st" in f]
        n = len(live)
        hz = n / window if window else 0.0
        gaps = []
        times = [t for t, _f in live]
        for a, b in zip(times, times[1:]):
            if b - a > 1.0:
                gaps.append(round(b - a, 2))
        detail = f"{n} frames in {window}s = {hz:.2f} Hz"
        if gaps:
            detail += f"; gaps >1s: {gaps}"
        self.record("PASS" if hz >= min_hz and not gaps else "FAIL", directive, detail)

    async def wait_quiet(self, pattern, quiet_s, timeout, directive):
        """Wait until nothing has matched `pattern` for `quiet_s` seconds.

        Synchronises a script with a human action whose timing it cannot
        control: "hold the FC in reset" becomes "wait until the FC's frames
        stop", which is the condition under test anyway.
        """
        rx = re.compile(pattern)
        deadline = time.time() + timeout
        last_hit_t = None          # timeline seconds of the newest match
        while time.time() < deadline:
            now_t = time.time() - self.tl.t0
            for ev in self.tl.since(0.0, {"serial"}):
                if rx.search(ev["text"]) and (last_hit_t is None or ev["t"] > last_hit_t):
                    last_hit_t = ev["t"]
            if last_hit_t is not None and (now_t - last_hit_t) >= quiet_s:
                self.record("PASS", directive,
                            f"{pattern!r} quiet for {now_t - last_hit_t:.1f}s")
                return True
            await asyncio.sleep(0.2)
        self.record("FAIL", directive,
                    f"{pattern!r} never went quiet for {quiet_s}s within {timeout}s")
        return False

    async def send(self, cmd, payload_hex, directive):
        payload = bytes.fromhex(payload_hex) if payload_hex else b""
        frame = bytes([cmd]) + payload
        try:
            await self.client.write_gatt_char(COMMAND_UUID, frame, response=True)
        except Exception as exc:                      # noqa: BLE001
            # A command that restarts the OC (cmd 8 power-off) takes the link
            # down with it, so a write can legitimately fail mid-script.  Record
            # it and carry on — aborting here would throw away the timeline that
            # is the entire point of the run.
            self.tl.add("ble-err", f"cmd {cmd} write failed: {exc}")
            self.record("ERROR", directive, f"write failed: {exc}")
            return
        self.tl.add("ble-tx", f"cmd {cmd} payload={payload_hex or '(none)'} [{frame.hex()}]")

    # ---- script runner ----------------------------------------------------

    async def run_script(self, lines):
        for lineno, raw in enumerate(lines, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split(None, 1)
            verb = parts[0]
            rest = parts[1] if len(parts) > 1 else ""
            where = f"line {lineno}: {line}"

            if verb == "note":
                self.tl.add("note", rest)
            elif verb == "sleep":
                await asyncio.sleep(float(rest))
            elif verb == "send":
                bits = rest.split()
                await self.send(int(bits[0]), bits[1] if len(bits) > 1 else "", where)
            elif verb in ("expect_serial", "refute_serial"):
                num, _, pat = rest.partition(" ")
                await self.wait_serial(float(num), pat.strip(),
                                       verb == "expect_serial", where)
            elif verb in ("expect_tlm", "refute_tlm", "wait_tlm"):
                num, _, cond = rest.partition(" ")
                await self.wait_tlm(float(num), cond.strip(),
                                    verb != "refute_tlm", where,
                                    scored=(verb != "wait_tlm"))
            elif verb == "expect_tlm_rate":
                win, _, hz = rest.partition(" ")
                await self.tlm_rate(float(win), float(hz), where)
            elif verb == "expect_tlm_nogap":
                win, _, gap = rest.partition(" ")
                await self.tlm_gap(float(win), float(gap), where)
            elif verb == "mark":
                self.mark = time.time() - self.tl.t0
                self.mark_frame = len(self.frames)
                self.tl.add("mark", f"search origin set at t={self.mark:.3f} "
                                    f"(frame {self.mark_frame})")
            elif verb in ("expect_serial_since", "refute_serial_since"):
                self.check_serial_since(rest.strip(),
                                        verb == "expect_serial_since", where)
            elif verb == "expect_order":
                a, b = rest.split("||")
                self.check_order(a.strip(), b.strip(), where)
            elif verb == "wait_quiet":
                pat, quiet, tmo = rest.rsplit(" ", 2)
                await self.wait_quiet(pat.strip(), float(quiet), float(tmo), where)
            elif verb == "expect_fileops":
                pre, _, n = rest.rpartition(" ")
                hits = [e for e in self.tl.since(self.mark, {"fileops"})
                        if e["text"].startswith(pre.strip().lower())]
                want = int(n)
                self.record("PASS" if len(hits) == want else "FAIL", where,
                            f"{len(hits)} file-ops frame(s) with prefix "
                            f"{pre.strip()!r} since mark, wanted {want}")
            elif verb == "disconnect":
                await self.disconnect()
            elif verb == "connect":
                await self.connect()
            elif verb == "health":
                latest = self.frames[-1][1] if self.frames else {}
                self.tl.add("health", describe_health(latest))
            else:
                raise SystemExit(f"unknown directive on {where}")


def validate(lines):
    """Parse-check a script without hardware: every directive, regex and condition.

    A typo that only surfaces halfway through a live run costs a board setup, so
    this runs the same parse the executor does and reports every problem at once.
    """
    known = {"note", "sleep", "send", "expect_serial", "refute_serial",
             "expect_tlm", "refute_tlm", "wait_tlm", "expect_tlm_rate",
             "disconnect", "connect", "health", "mark",
             "expect_serial_since", "refute_serial_since", "expect_order",
             "expect_fileops", "expect_tlm_nogap", "wait_quiet"}
    problems, steps = [], 0
    for lineno, raw in enumerate(lines, 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        steps += 1
        parts = line.split(None, 1)
        verb, rest = parts[0], (parts[1] if len(parts) > 1 else "")
        if verb not in known:
            problems.append(f"line {lineno}: unknown directive {verb!r}")
            continue
        try:
            if verb == "sleep":
                float(rest)
            elif verb == "send":
                bits = rest.split()
                cmd = int(bits[0])
                if not 0 <= cmd <= 255:
                    problems.append(f"line {lineno}: cmd {cmd} out of range")
                if len(bits) > 1:
                    bytes.fromhex(bits[1])
            elif verb in ("expect_serial", "refute_serial"):
                num, _, pat = rest.partition(" ")
                float(num)
                re.compile(pat.strip())
            elif verb in ("expect_tlm", "refute_tlm", "wait_tlm"):
                num, _, cond = rest.partition(" ")
                float(num)
                cond_holds({}, cond.strip())      # exercises the key/bit tables
            elif verb in ("expect_tlm_rate", "expect_tlm_nogap"):
                win, _, hz = rest.partition(" ")
                float(win); float(hz)
            elif verb in ("expect_serial_since", "refute_serial_since"):
                re.compile(rest.strip())
            elif verb == "expect_order":
                a, b = rest.split("||")
                re.compile(a.strip()); re.compile(b.strip())
            elif verb == "expect_fileops":
                pre, _, n = rest.rpartition(" ")
                int(n); bytes.fromhex(pre.strip())
            elif verb == "wait_quiet":
                pat, quiet, tmo = rest.rsplit(" ", 2)
                re.compile(pat.strip()); float(quiet); float(tmo)
        except SystemExit as exc:
            problems.append(f"line {lineno}: {exc}")
        except Exception as exc:                      # noqa: BLE001
            problems.append(f"line {lineno}: {type(exc).__name__}: {exc}")
    return steps, problems


async def main_async(args):
    t0 = time.time()
    tl = Timeline(t0)
    script = Path(args.script).read_text().splitlines() if args.script else []

    steps, problems = validate(script)
    for prob in problems:
        print(f"SCRIPT ERROR  {prob}", file=sys.stderr)
    if problems:
        return 2
    # A directive that needs the radio would otherwise sit there doing nothing
    # and be scored a pass, which is the one outcome a bench harness must never
    # produce.  Refuse the run instead.
    if args.no_ble:
        needs_radio = {"send", "expect_tlm", "refute_tlm", "wait_tlm",
                       "expect_tlm_rate", "expect_tlm_nogap", "expect_fileops",
                       "connect", "disconnect", "health"}
        offenders = [(i + 1, ln.split()[0]) for i, ln in enumerate(script)
                     if ln.strip() and not ln.lstrip().startswith("#")
                     and ln.split()[0] in needs_radio]
        for lineno, verb in offenders:
            print(f"SCRIPT ERROR  line {lineno}: '{verb}' needs the BLE link, "
                  f"but this is a --no-ble run", file=sys.stderr)
        if offenders:
            return 2

    if args.dry_run:
        print(f"{args.script}: {steps} steps, no script errors")
        return 0

    tails = []
    for spec in args.port:
        path, _, label = spec.partition(":")
        tail = SerialTail(path, args.baud, tl, label or None)
        tail.start()
        tails.append(tail)
    if tails:
        await asyncio.sleep(0.5)
        for tail in tails:
            if tail.error:
                print(f"WARNING: {tail.error} — continuing without it", file=sys.stderr)

    sess = Session(args, tl)
    aborted = None
    try:
        await sess.connect()
        await sess.run_script(script)
    except Exception as exc:                          # noqa: BLE001
        # Never lose the timeline to a failure partway through: a half-run whose
        # console log survives is still evidence, and the log is usually what
        # explains why the run stopped.
        aborted = exc
        tl.add("ABORT", f"{type(exc).__name__}: {exc}")
    finally:
        try:
            await sess.disconnect()
        except Exception:                             # noqa: BLE001
            pass
        for tail in tails:
            tail.stop_flag.set()
            tail.join(timeout=2)
        if args.out:
            out = Path(args.out)
            out.mkdir(parents=True, exist_ok=True)
            (out / "timeline.jsonl").write_text(
                "\n".join(json.dumps(e) for e in tl.events) + "\n")
            (out / "console.log").write_text(
                "\n".join(e["text"] for e in tl.events if e["kind"] == "serial") + "\n")
            print(f"\nwrote {out}/timeline.jsonl and {out}/console.log")

    print("\n=== RESULTS ===")
    for status, directive, detail in sess.results:
        print(f"  {status}  {directive}")
        if detail:
            print(f"         {detail}")
    failed = sum(1 for s, _d, _x in sess.results if s != "PASS")
    print(f"\n{len(sess.results) - failed} passed, {failed} not passed")
    if aborted:
        print(f"RUN ABORTED: {type(aborted).__name__}: {aborted}")
    return 1 if (failed or aborted) else 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", default="TR-R", help="BLE name substring")
    p.add_argument("--no-ble", action="store_true",
                   help="tail the console only; take no BLE link. For tests where "
                        "another central (the phone app) must own the connection.")
    p.add_argument("--address", help="CoreBluetooth peripheral UUID")
    p.add_argument("--port", action="append", default=[],
                   help="serial console as PATH or PATH:LABEL; repeatable, so one "
                        "run can watch the rocket and the base station together "
                        "(labelled lines read '[BS] I (123) ...')")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--script", required=True, help="test script path")
    p.add_argument("--out", help="directory for timeline.jsonl + console.log")
    p.add_argument("--dry-run", action="store_true",
                   help="parse-check the script and exit; no hardware needed")
    args = p.parse_args()
    sys.exit(asyncio.run(main_async(args)))


if __name__ == "__main__":
    main()
