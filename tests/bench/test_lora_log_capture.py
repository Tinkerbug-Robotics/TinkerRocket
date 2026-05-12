#!/usr/bin/env python3
"""Bench HIL test for the BS LoRa CSV logging path (#137).

Drives a simulated flight on real hardware end-to-end:

    laptop ── BLE ── BS ── LoRa ── flight computer (in SIM mode)

The flight computer's SensorCollectorSim provides synthetic GNSS / IMU /
pressure that drive the rocket state machine through READY → PRELAUNCH →
INFLIGHT → LANDED → READY, while the rocket transmits real LoRa packets
on the operating frequency.  The BS receives them and writes lora_*.csv
to the SD card.  After the cycle, the test downloads the newest LoRa log
via BLE and asserts:

    • the file exists with a header + non-trivial number of data rows
    • the state column shows the expected sequence
    • max_alt grew (the synthetic flight actually went up)
    • there's no in-flight silence gap larger than the silence-close
      threshold (which would indicate the #137 silence-during-INFLIGHT
      regression had returned)

This is a manual bench test — not run from CI.  Run it on hardware before
every flight day, and on every PR that touches LoRa or BS logging code.

Hardware setup:
    1. BS powered, antenna connected, SD card inserted
    2. Flight computer powered (USB or battery)
    3. Both within LoRa range of each other (~1 m on the bench is plenty)
    4. Laptop within BLE range of the BS
    5. No other tinker base station advertising (or pass --address)

Software requirements:
    pip install bleak   # tested with bleak 0.21+
    Python 3.10+

Usage:
    # Fully scripted: Python drives the sim end-to-end
    python3 tests/bench/test_lora_log_capture.py

    # Explicit BS by MAC (when multiple BS are advertising)
    python3 tests/bench/test_lora_log_capture.py --address AA:BB:CC:DD:EE:FF

    # Wipe old logs first (recommended for repeated runs)
    python3 tests/bench/test_lora_log_capture.py --delete-existing

    # Customise the sim profile
    python3 tests/bench/test_lora_log_capture.py --sim-burn 1.5 --sim-thrust 30

    # iOS-driven mode: skip the sim drive, just verify the newest log on the
    # BS SD.  Workflow: drive the sim from the iOS Simulation view, wait for
    # rocket to return to READY, disconnect iOS, then run this.
    python3 tests/bench/test_lora_log_capture.py --skip-sim

    # Verify a specific file (e.g. a recovered log from a past flight)
    python3 tests/bench/test_lora_log_capture.py --skip-sim --file lora_20260512_143000.csv

Exit code 0 = pass, non-zero = fail (with diagnostic output).
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import io
import json
import re
import struct
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Optional

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    sys.stderr.write(
        "error: bleak not installed.  Run: pip install bleak\n"
    )
    sys.exit(2)


# ----------------------------------------------------------------------------
# BLE constants — must match tinkerrocket-idf/components/TR_BLE_To_APP/
# ----------------------------------------------------------------------------
SERVICE_UUID            = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
TELEMETRY_CHAR_UUID     = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
COMMAND_CHAR_UUID       = "cba1d466-344c-4be3-ab3f-189f80dd7518"
FILE_OPS_CHAR_UUID      = "8d53dc1d-1db7-4cd3-868b-8a527460aa84"
FILE_TRANSFER_CHAR_UUID = "1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d"

DEFAULT_BS_NAME = "TinkerBaseStation"

# Command bytes (subset — full list in base_station/main/main.cpp BLE handler)
CMD_FILE_LIST       = 0x02
CMD_DELETE          = 0x03
CMD_DOWNLOAD        = 0x04
CMD_SIM_CONFIG      = 0x05
CMD_SIM_START       = 0x06
CMD_SIM_STOP        = 0x07
CMD_TIME_SYNC       = 0x09


# ----------------------------------------------------------------------------
# Captured state
# ----------------------------------------------------------------------------
@dataclass
class Capture:
    """Aggregates everything we need to observe during the test."""
    states_seen: list[str] = field(default_factory=list)
    last_state: Optional[str] = None
    last_bs_logging_active: Optional[bool] = None
    last_active_file: Optional[str] = None
    # The BS closes the flight log on the INFLIGHT→LANDED transition and
    # then auto-opens a new file for any post-landing telemetry, so the
    # "newest" file on disk after a sim is a tiny post-flight log without
    # the actual flight content.  Capture the active_file while we're
    # observing INFLIGHT so the assertion path can target the right log.
    inflight_active_file: Optional[str] = None
    last_max_alt: float = 0.0
    last_landed_flag: bool = False
    telemetry_count: int = 0

    # Asynchronous file ops
    file_list_event: asyncio.Event = field(default_factory=asyncio.Event)
    file_list_json: Optional[str] = None
    chunks: dict[int, bytes] = field(default_factory=dict)
    download_complete: asyncio.Event = field(default_factory=asyncio.Event)

    def note_state(self, state: str) -> None:
        if state and state != self.last_state:
            self.states_seen.append(state)
            self.last_state = state


# ----------------------------------------------------------------------------
# Notification handlers
# ----------------------------------------------------------------------------
def make_telemetry_handler(cap: Capture):
    def handler(_char, data: bytearray):
        cap.telemetry_count += 1
        try:
            obj = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        state = obj.get("st")
        if isinstance(state, str):
            cap.note_state(state)
        if "bslog" in obj:
            cap.last_bs_logging_active = bool(obj["bslog"])
        if "af" in obj and isinstance(obj["af"], str) and obj["af"]:
            cap.last_active_file = obj["af"]
            if cap.last_state == "INFLIGHT":
                cap.inflight_active_file = obj["af"]
        if "malt" in obj:
            try:
                cap.last_max_alt = float(obj["malt"])
            except (TypeError, ValueError):
                pass
        if "land" in obj:
            cap.last_landed_flag = bool(obj["land"])
    return handler


def make_file_ops_handler(cap: Capture):
    def handler(_char, data: bytearray):
        # Multiple kinds of payload arrive on this characteristic:
        #   0xAA marker  → scan results (ignore)
        #   0xCA marker  → mag-cal status (ignore)
        #   otherwise    → file list JSON (UTF-8 array)
        if len(data) == 0:
            return
        marker = data[0]
        if marker in (0xAA, 0xCA):
            return
        try:
            cap.file_list_json = data.decode("utf-8")
            cap.file_list_event.set()
        except UnicodeDecodeError:
            pass
    return handler


def make_file_transfer_handler(cap: Capture):
    def handler(_char, data: bytearray):
        # Packet format: [offset(4)][length(2)][flags(1)][data(N)]
        if len(data) < 7:
            return
        offset = int.from_bytes(data[0:4], "little")
        length = int.from_bytes(data[4:6], "little")
        eof = bool(data[6] & 0x01)
        payload = bytes(data[7:7 + length])
        cap.chunks[offset] = payload
        if eof:
            cap.download_complete.set()
    return handler


# ----------------------------------------------------------------------------
# Command builders
# ----------------------------------------------------------------------------
def build_time_sync_payload() -> bytes:
    """Cmd 9: [year_lo, year_hi, mo, d, h, mi, s] — matches BS handler at
    main.cpp:3314."""
    now = datetime.now(timezone.utc)
    return bytes([
        CMD_TIME_SYNC,
        now.year & 0xFF, (now.year >> 8) & 0xFF,
        now.month, now.day, now.hour, now.minute, now.second,
    ])


def build_sim_config(mass_g: float, thrust_n: float, burn_s: float,
                     descent_mps: float) -> bytes:
    """Cmd 5: 16 bytes of little-endian floats [mass_g, thrust_n, burn_s,
    descent_mps].  See iOS SimulationView.swift launchSim() for the
    canonical encoder."""
    return bytes([CMD_SIM_CONFIG]) + struct.pack(
        "<ffff", mass_g, thrust_n, burn_s, descent_mps
    )


# ----------------------------------------------------------------------------
# Test driver
# ----------------------------------------------------------------------------
async def find_bs(name_filter: str, address: Optional[str], timeout_s: float):
    if address:
        print(f"[scan] looking up BS by address {address}...")
        device = await BleakScanner.find_device_by_address(
            address, timeout=timeout_s
        )
        return device
    print(f"[scan] scanning for BLE peripheral matching '{name_filter}'...")
    device = await BleakScanner.find_device_by_filter(
        lambda _d, adv: (adv.local_name or "").startswith(name_filter),
        timeout=timeout_s,
    )
    return device


async def wait_for_state(cap: Capture, target: str,
                         timeout_s: float, label: str) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if cap.last_state == target:
            return True
        await asyncio.sleep(0.2)
    print(f"[fail] timeout waiting for {label} (state={cap.last_state}, "
          f"seen={cap.states_seen})", file=sys.stderr)
    return False


async def run_test(args) -> int:
    cap = Capture()

    device = await find_bs(args.name, args.address, args.scan_timeout)
    if device is None:
        print("[fail] no base station found over BLE", file=sys.stderr)
        return 3
    print(f"[scan] found {device.name} @ {device.address}")

    async with BleakClient(device) as client:
        print("[ble]  connected; subscribing to characteristics")
        await client.start_notify(
            TELEMETRY_CHAR_UUID,    make_telemetry_handler(cap))
        await client.start_notify(
            FILE_OPS_CHAR_UUID,     make_file_ops_handler(cap))
        await client.start_notify(
            FILE_TRANSFER_CHAR_UUID, make_file_transfer_handler(cap))

        # Wait for first telemetry — confirms BS is alive and our subscription
        # is wired up.  Bail early if the BS is offline.
        print("[ble]  waiting for first telemetry...")
        deadline = time.monotonic() + 10.0
        while cap.telemetry_count == 0 and time.monotonic() < deadline:
            await asyncio.sleep(0.2)
        if cap.telemetry_count == 0:
            print("[fail] no telemetry received within 10 s — is the BS "
                  "powered and within range?", file=sys.stderr)
            return 4

        if args.skip_sim:
            # iOS-driven mode: the operator already ran the sim from the iOS
            # Simulation view and the flight cycle is complete.  We're just
            # the verifier — connect, request file list, download newest,
            # assert.  Skip time-sync (iOS already sent it on its connect)
            # and the full state-machine wait.
            print("[ble]  --skip-sim: assuming iOS already drove the flight")
            # Brief grace in case the operator triggered the script right at
            # LANDED transition; the BS may still be flushing.
            await asyncio.sleep(2.0)
        else:
            # Time-sync so the new log will be named lora_YYYYMMDD_HHMMSS.csv,
            # making the "newest file" check unambiguous below.
            print("[ble]  sending time-sync")
            await client.write_gatt_char(
                COMMAND_CHAR_UUID, build_time_sync_payload(), response=True)
            await asyncio.sleep(1.0)

            # Optionally clear any stale logs so the post-flight file list has
            # exactly one new entry.  Off by default — the operator may want
            # to preserve previous bench runs for comparison.
            if args.delete_existing:
                print("[ble]  deleting existing lora_*.csv before sim...")
                await delete_all_lora_logs(client, cap)

            # Configure + start sim.  The flight computer's SensorCollectorSim
            # walks PRELAUNCH (5 s) → POWERED (burn_s) → COASTING → DESCENT →
            # LANDED — total ~burn + descent_time seconds for the masses we use.
            print(f"[sim]  config: mass={args.sim_mass}g thrust={args.sim_thrust}N "
                  f"burn={args.sim_burn}s descent={args.sim_descent}m/s")
            await client.write_gatt_char(
                COMMAND_CHAR_UUID,
                build_sim_config(args.sim_mass, args.sim_thrust,
                                 args.sim_burn, args.sim_descent),
                response=True,
            )

            # The iOS app waits 0.3-1.0 s between cfg and start (LoRa uplink
            # retries take ~300 ms).  Match the BS-relay path.
            await asyncio.sleep(1.0)

            print("[sim]  starting sim")
            await client.write_gatt_char(
                COMMAND_CHAR_UUID, bytes([CMD_SIM_START]), response=True)

            # Watch the state machine.  Timeouts are conservative — even a 10 s
            # burn + 30 s descent should comfortably hit READY again within 90 s.
            if not await wait_for_state(
                    cap, "PRELAUNCH",  20, "READY → PRELAUNCH"): return 5
            if not await wait_for_state(
                    cap, "INFLIGHT",   30, "PRELAUNCH → INFLIGHT"): return 6
            if not await wait_for_state(
                    cap, "LANDED",     90, "INFLIGHT → LANDED"): return 7
            # After landing, the BS closes log A on the LANDED transition
            # and immediately auto-opens log B for any post-landing packets
            # (intentional — captures recovery telemetry).  Log A is on
            # disk; log B will close on its own once 5 min of silence
            # elapses (LOG_SILENCE_TIMEOUT_MS, #137).
            await asyncio.sleep(5.0)
            if not await wait_for_state(
                    cap, "READY",      30, "LANDED → READY (sim done)"): return 8

            # Short grace so the LANDED-close fsync has finished and log A
            # is visible in the BLE file listing.  We deliberately do NOT
            # wait for log B's 5-min silence-close — we want the assertion
            # to run against log A (the file with PRELAUNCH/INFLIGHT/LANDED
            # rows).  The newest-by-name pick below selects log B if it's
            # opened with a later timestamp, so we use --file to force log
            # A when running back-to-back tests.
            await asyncio.sleep(5.0)

        # ---- File list ----
        print("[ble]  requesting file list")
        cap.file_list_event.clear()
        cap.file_list_json = None
        await client.write_gatt_char(
            COMMAND_CHAR_UUID, bytes([CMD_FILE_LIST, 0]), response=True)
        try:
            await asyncio.wait_for(cap.file_list_event.wait(), timeout=10.0)
        except asyncio.TimeoutError:
            print("[fail] no file list returned within 10 s", file=sys.stderr)
            return 9

        lora_files = parse_lora_files(cap.file_list_json or "[]")
        if not lora_files:
            print(f"[fail] no lora_*.csv on the BS SD after sim cycle "
                  f"(file list: {cap.file_list_json})", file=sys.stderr)
            return 10

        # Pick which file to verify.  Preference order:
        #   1. --file FILENAME (explicit override, useful with --skip-sim)
        #   2. The active_file we captured during the simulated INFLIGHT.
        #      This is "log A" — the file containing the actual flight.
        #      "log B" (post-landing auto-restart) shows up with a later
        #      timestamp and would mislead the max() heuristic.
        #   3. Newest by filename — fallback when nothing else is known
        #      (e.g. --skip-sim and the iOS-driven flow couldn't be
        #      observed, or this is the very first run).
        if args.file:
            chosen_name = args.file
        elif cap.inflight_active_file:
            chosen_name = cap.inflight_active_file
            print(f"[ble]  selecting flight log observed during INFLIGHT: "
                  f"{chosen_name}")
        else:
            chosen_name = max(lora_files, key=lambda f: f["name"])["name"]
        match = [f for f in lora_files if f["name"] == chosen_name]
        if not match:
            print(f"[fail] target log {chosen_name!r} not present on the "
                  f"BS SD (found: {[f['name'] for f in lora_files]})",
                  file=sys.stderr)
            return 10
        newest = match[0]
        print(f"[ble]  selected log: {newest['name']} ({newest['size']} bytes)")

        if newest["size"] < 200:
            print(f"[fail] newest log is suspiciously small "
                  f"({newest['size']} bytes — header is ~140 bytes alone)",
                  file=sys.stderr)
            return 11

        # ---- Download ----
        print(f"[ble]  downloading {newest['name']}...")
        cap.chunks.clear()
        cap.download_complete.clear()
        await client.write_gatt_char(
            COMMAND_CHAR_UUID,
            bytes([CMD_DOWNLOAD]) + newest["name"].encode("utf-8"),
            response=True,
        )
        try:
            await asyncio.wait_for(
                cap.download_complete.wait(), timeout=180.0)
        except asyncio.TimeoutError:
            print(f"[fail] download did not complete within 180 s "
                  f"(got {sum(len(v) for v in cap.chunks.values())} / "
                  f"{newest['size']} bytes)", file=sys.stderr)
            return 12

        body = assemble_chunks(cap.chunks)
        if len(body) != newest["size"]:
            print(f"[warn] downloaded {len(body)} bytes, file list said "
                  f"{newest['size']} — possibly mid-write")
        return assert_log_quality(body, args)


async def delete_all_lora_logs(client, cap: Capture) -> None:
    """Walk pages until empty, deleting every lora_*.csv we see."""
    page = 0
    while True:
        cap.file_list_event.clear()
        cap.file_list_json = None
        await client.write_gatt_char(
            COMMAND_CHAR_UUID, bytes([CMD_FILE_LIST, page]), response=True)
        try:
            await asyncio.wait_for(cap.file_list_event.wait(), timeout=5.0)
        except asyncio.TimeoutError:
            return
        files = parse_lora_files(cap.file_list_json or "[]")
        if not files:
            return
        for f in files:
            print(f"       delete {f['name']}")
            await client.write_gatt_char(
                COMMAND_CHAR_UUID,
                bytes([CMD_DELETE]) + f["name"].encode("utf-8"),
                response=True,
            )
            await asyncio.sleep(0.3)
        # handleDeleteCommand auto-resends page 0, so we re-poll page 0
        # each round — no need to advance `page`.


def parse_lora_files(json_str: str) -> list[dict]:
    try:
        arr = json.loads(json_str)
    except json.JSONDecodeError:
        return []
    return [f for f in arr
            if isinstance(f, dict)
            and isinstance(f.get("name"), str)
            and f["name"].startswith("lora_")
            and f["name"].endswith(".csv")]


def assemble_chunks(chunks: dict[int, bytes]) -> bytes:
    if not chunks:
        return b""
    sorted_offsets = sorted(chunks.keys())
    out = bytearray()
    for off in sorted_offsets:
        if off != len(out):
            # Gap or overlap — pad with zeros so the parser doesn't choke,
            # but the assertion path will catch it via row count.
            out.extend(b"\x00" * max(0, off - len(out)))
        out[off:off + len(chunks[off])] = chunks[off]
    return bytes(out)


# ----------------------------------------------------------------------------
# Log quality assertions
# ----------------------------------------------------------------------------
EXPECTED_TRANSITIONS = ["PRELAUNCH", "INFLIGHT", "LANDED"]


def assert_log_quality(body: bytes, args) -> int:
    text = body.decode("utf-8", errors="replace")
    reader = csv.reader(io.StringIO(text))
    rows = list(reader)
    if len(rows) < 2:
        print("[fail] log has no data rows", file=sys.stderr)
        return 20
    header, data_rows = rows[0], rows[1:]
    print(f"[csv]  header: {len(header)} columns, {len(data_rows)} data rows")

    try:
        state_col = header.index("state")
        time_col  = header.index("time_ms")
        malt_col  = header.index("max_alt")
    except ValueError as e:
        print(f"[fail] required CSV column missing: {e}", file=sys.stderr)
        return 21

    # State transition sequence
    states = [r[state_col] for r in data_rows if len(r) > state_col]
    transitions = [s for i, s in enumerate(states)
                   if i == 0 or s != states[i - 1]]
    print(f"[csv]  state sequence: {transitions}")
    for target in EXPECTED_TRANSITIONS:
        if target not in transitions:
            print(f"[fail] state '{target}' not seen in log", file=sys.stderr)
            return 22

    # max_alt grew at some point (the synthetic flight actually went up).
    # The "max_alt" column is sticky-max from the rocket — it should rise
    # above the threshold during INFLIGHT.
    max_alts = [float(r[malt_col]) for r in data_rows
                if len(r) > malt_col and is_float(r[malt_col])]
    if not max_alts or max(max_alts) < args.min_max_alt:
        print(f"[fail] peak max_alt={max(max_alts) if max_alts else 0:.1f} m "
              f"< {args.min_max_alt} m — flight didn't register altitude",
              file=sys.stderr)
        return 23
    print(f"[csv]  peak max_alt: {max(max_alts):.1f} m")

    # Silence-gap check.  Skip EVENT rows (state=='EVENT') because they don't
    # have a time_ms aligned to the rocket clock the same way.  We're looking
    # for #137 returning: a >5 s in-flight gap that would have silence-closed
    # the log pre-fix.
    inflight_rows = [r for r in data_rows
                     if len(r) > state_col and r[state_col] == "INFLIGHT"]
    times = []
    for r in inflight_rows:
        if len(r) > time_col and is_float(r[time_col]):
            times.append(float(r[time_col]))
    if len(times) >= 2:
        gaps_ms = [b - a for a, b in zip(times, times[1:])]
        worst_ms = max(gaps_ms)
        print(f"[csv]  worst INFLIGHT inter-row gap: {worst_ms:.0f} ms")
        # 5 s threshold — typical packet cadence is ~200-500 ms; the BS will
        # certainly hold the log open for >5 s with the #137 fix in place.
        if worst_ms > 5000:
            print(f"[fail] INFLIGHT gap {worst_ms:.0f} ms > 5000 ms — "
                  f"silence-close-during-INFLIGHT may have regressed",
                  file=sys.stderr)
            return 24

    print("[pass] BS LoRa log captured the simulated flight end-to-end")
    return 0


def is_float(s: str) -> bool:
    try:
        float(s)
        return True
    except (TypeError, ValueError):
        return False


# ----------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name", default=DEFAULT_BS_NAME,
                   help="BLE name prefix to scan for")
    p.add_argument("--address", default=None,
                   help="explicit BLE MAC address (skips scan)")
    p.add_argument("--scan-timeout", type=float, default=15.0,
                   help="BLE scan timeout in seconds")
    p.add_argument("--sim-mass", type=float, default=500.0,
                   help="sim mass in grams")
    p.add_argument("--sim-thrust", type=float, default=20.0,
                   help="sim motor thrust in N")
    p.add_argument("--sim-burn", type=float, default=2.0,
                   help="sim motor burn time in s")
    p.add_argument("--sim-descent", type=float, default=5.0,
                   help="sim descent rate in m/s")
    p.add_argument("--min-max-alt", type=float, default=20.0,
                   help="minimum acceptable peak altitude in m")
    p.add_argument("--delete-existing", action="store_true",
                   help="delete every lora_*.csv on the BS SD before the sim")
    p.add_argument("--skip-sim", action="store_true",
                   help="don't drive the sim — assume iOS already did the "
                        "flight cycle.  Just connect, list files, download, "
                        "and assert on the newest (or --file)")
    p.add_argument("--file", default=None,
                   help="explicit log filename to verify (default: newest "
                        "lora_*.csv).  Useful with --skip-sim")
    args = p.parse_args()
    if args.file and not args.skip_sim:
        p.error("--file only makes sense with --skip-sim "
                "(without it we don't know which log this run produced)")

    try:
        rc = asyncio.run(run_test(args))
    except KeyboardInterrupt:
        rc = 130
    sys.exit(rc)


if __name__ == "__main__":
    main()
