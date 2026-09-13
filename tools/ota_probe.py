#!/usr/bin/env python3
"""OTA refusal probe (#1425, #1125) — offer the first chunks of an image and
print what the device says, without ever flashing anything.

The motivating case: an FC image built with the WRONG board flag must be
refused, and the operator must be told WHY — `fc_image_identity_mismatch`,
not a timeout (#1267 / #1273).  Checking that through the phone means a
multi-minute push of a whole image before the interesting 300 ms.  It does
not have to: the receiver decides identity as soon as the app descriptor is
complete, which is **288 bytes in** (`APP_DESC_OFFSET 32 + APP_DESC_SIZE
256`, TR_OTA_Receiver.h), so two 220-byte chunks settle it.

    $ python3 tools/ota_probe.py --name TR-R --image build_v8/flight_computer.bin \
          --target fc --expect fc_image_identity_mismatch
    ...
      21.012  -> chunk offset=0   len=220
      21.306  -> chunk offset=220 len=220
      21.353  <- {"type":"ota_status","state":"verify_failed","bytes":220,
                 "err":"fc_image_identity_mismatch"}      [+47 ms]
    PASS: refused with fc_image_identity_mismatch after 220 B

**This tool never sends OTA_FINISH, so it cannot commit an image.**  That is
deliberate and is what makes it safe to point at a flight board: without
FINISH the receiver never verifies, never sets a boot partition and never
reboots.  Flashing for real is the app's job.

The session is always closed with OTA_ABORT on the way out, including on
Ctrl-C, and on the relay path that matters for more than tidiness: a BEGIN
flips the I2S link to master TX for the image pump, so telemetry is down
until something ends the session.  The abort reverts it immediately; walking
away instead means waiting out the OC's 10 s no-chunk stall (#1116).

`bytes` in the refusal reports what was written BEFORE the rejected chunk —
the receiver checks identity before `bytes_written_ += len` — so a refusal on
the second 220-byte chunk correctly reads `bytes=220`, not 440.

Wire facts (BLEDevice.swift, TR_BLE_To_APP.cpp, TR_OTA_Receiver.cpp):
  service        4fafc201-1fb5-459e-8fcc-c5c9c331914b
  command        cba1d466-344c-4be3-ab3f-189f80dd7518   (write)
      OTA_BEGIN  cmd 70, payload [target:1][size:4 LE][sha256:32]
      OTA_FINISH cmd 71   <- never sent by this tool
      OTA_ABORT  cmd 72
  file transfer  1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d   (write-no-response)
      chunk      [offset:4 LE][length:2 LE][flags:1][data:N], flags bit0 = isLast
  file ops       8d53dc1d-1db7-4cd3-868b-8a527460aa84   (notify)
      status     {"type":"ota_status","state":...,"bytes":N[,"err":...][,"fw":...]}

`--target fc` is the OC-relayed Flight Computer path; the OC holds its `ready`
until it has flipped the I2S link to master TX, which is why this waits for
`ready` rather than pumping straight after BEGIN.  `--target oc` flashes the
connected device's own app — same refusal check, tokens without the `fc_`
prefix (`image_identity_mismatch`).

Notes:
  * Disconnect the phone first — the device takes a limited number of centrals.
  * An FC relay BEGIN takes seconds to answer: the FC erases its OTA slot
    before it can say ready.  --begin-timeout covers it.
  * If nothing at all comes back, check you are on the right device: an OC
    running an image older than the relay-status handling is what made #1425
    look like a firmware bug.  Read its version stamp first.
"""

import argparse
import asyncio
import hashlib
import json
import struct
import sys
import time
from pathlib import Path

# bleak is imported lazily by _load_bleak() rather than at module scope, so
# this file imports on a machine without it — which is what lets
# tests/unit/test_ota_probe.py drive the whole flow against a fake client in
# CI instead of skipping there. Tests bind these two names themselves.
BleakClient = None
BleakScanner = None


def _load_bleak() -> None:
    global BleakClient, BleakScanner
    if BleakClient is not None:
        return
    try:
        from bleak import BleakClient as _Client, BleakScanner as _Scanner
    except ImportError as e:
        raise SystemExit("bleak is not installed — `pip install bleak`") from e
    BleakClient, BleakScanner = _Client, _Scanner


SERVICE_UUID       = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
COMMAND_UUID       = "cba1d466-344c-4be3-ab3f-189f80dd7518"
FILE_OPS_UUID      = "8d53dc1d-1db7-4cd3-868b-8a527460aa84"
FILE_TRANSFER_UUID = "1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d"

CMD_OTA_BEGIN  = 70
CMD_OTA_ABORT  = 72

# TR_OTA_Receiver.h: the identity decision needs APP_DESC_OFFSET + APP_DESC_SIZE
# bytes buffered.  Everything this tool does is sized off this number.
IDENTITY_BYTES = 32 + 256

CHUNK_HEADER   = 7      # [offset:4][len:2][flags:1]
ATT_OVERHEAD   = 3      # ATT write-command opcode + handle
SCAN_TIMEOUT_S = 15.0


class Probe:
    def __init__(self):
        self.statuses: list[tuple[float, dict]] = []
        self.t0 = time.monotonic()
        self.last_chunk_at: float | None = None

    def stamp(self) -> float:
        return time.monotonic() - self.t0

    def on_status(self, _h, data: bytearray):
        text = data.decode("utf-8", "replace")
        try:
            msg = json.loads(text)
        except ValueError:
            print(f"  {self.stamp():7.3f}  <- (unparsed) {text}")
            return
        if msg.get("type") != "ota_status":
            return          # file_ops carries other traffic; ignore it
        # Time since the chunk that most likely triggered this is the number
        # #1425 turns on — "the reason reached the app in N ms".
        delta = ""
        if self.last_chunk_at is not None:
            delta = f"      [+{(time.monotonic() - self.last_chunk_at) * 1000:.0f} ms]"
        print(f"  {self.stamp():7.3f}  <- {text}{delta}")
        self.statuses.append((self.stamp(), msg))

    def latest(self, state: str, since: int = 0) -> dict | None:
        """Most recent status with `state`, considering only frames recorded
        at or after index `since`.

        The window matters: a verdict is only a verdict if it arrived AFTER
        the chunks that should have provoked it. Matching a frame from
        earlier in the session would let a stale refusal read as a pass,
        which is the dangerous direction for a tool whose whole job is to
        confirm a refusal."""
        for _, m in reversed(self.statuses[since:]):
            if m.get("state") == state:
                return m
        return None


async def find_device(name_substr: str):
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: name_substr.lower() in (d.name or "").lower(),
        timeout=SCAN_TIMEOUT_S,
    )
    if dev is None:
        raise SystemExit(
            f"device matching '{name_substr}' not found — power it, and make "
            f"sure the phone isn't holding its only BLE connection")
    return dev


async def await_state(probe: Probe, state: str, timeout: float,
                      since: int = 0) -> dict | None:
    """Poll for a status with `state`, or any terminal one, until timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if (m := probe.latest(state, since)) is not None:
            return m
        for terminal in ("verify_failed", "aborted"):
            if (m := probe.latest(terminal, since)) is not None:
                return m
        await asyncio.sleep(0.01)
    return None


async def run(args) -> int:
    _load_bleak()
    try:
        image = Path(args.image).read_bytes()
    except OSError as e:
        raise SystemExit(f"cannot read {args.image}: {e.strerror}")
    sha = hashlib.sha256(image).digest()
    target_is_fc = (args.target == "fc")
    print(f"Image {args.image}: {len(image)} B, sha256 {sha.hex()[:16]}…")
    if len(image) < IDENTITY_BYTES:
        raise SystemExit(f"image is shorter than the {IDENTITY_BYTES} B identity "
                         f"header — nothing to decide on")

    dev = args.address if args.address else await find_device(args.name)
    if args.address:
        print(f"Connecting by address {args.address} …")
    else:
        print(f"Connecting to {dev.name} ({dev.address}) …")

    probe = Probe()
    async with BleakClient(dev) as client:
        # A chunk write is CHUNK_HEADER + payload and must fit one ATT write.
        room = client.mtu_size - ATT_OVERHEAD - CHUNK_HEADER
        chunk = args.chunk_size or min(220, room)
        if chunk > room:
            raise SystemExit(f"--chunk-size {chunk} exceeds what this link can "
                             f"carry ({room} B at MTU {client.mtu_size})")
        if chunk <= 0:
            raise SystemExit(f"MTU {client.mtu_size} leaves no room for a chunk")
        # Enough chunks to COMPLETE the identity header — the decision lands on
        # the chunk that finishes it, not the one after.
        need = -(-IDENTITY_BYTES // chunk)
        count = args.chunks or need
        print(f"MTU {client.mtu_size} → {chunk} B chunks; sending {count} "
              f"({count * chunk} B, identity needs {IDENTITY_BYTES})")
        if count < need:
            print(f"  note: {count} chunk(s) is short of the {need} needed to "
                  f"complete the header — no identity verdict will come")

        await client.start_notify(FILE_OPS_UUID, probe.on_status)
        try:
            payload = bytes([1 if target_is_fc else 0]) + struct.pack("<I", len(image)) + sha
            print(f"  {probe.stamp():7.3f}  -> OTA_BEGIN target="
                  f"{'fc (relay)' if target_is_fc else 'oc (local)'} size={len(image)}")
            await client.write_gatt_char(COMMAND_UUID,
                                         bytes([CMD_OTA_BEGIN]) + payload, response=True)

            ready = await await_state(probe, "ready", args.begin_timeout)
            if ready is None:
                print(f"FAIL: no 'ready' within {args.begin_timeout:.0f}s — the "
                      f"device never accepted OTA_BEGIN")
                return 2
            if ready.get("state") != "ready":
                print(f"FAIL: refused at BEGIN: {json.dumps(ready)}")
                return 2

            # Everything from here on is judged against frames that arrive
            # AFTER the chunks — see Probe.latest.
            mark = len(probe.statuses)
            for i in range(count):
                off = i * chunk
                body = image[off:off + chunk]
                if not body:
                    break
                frame = struct.pack("<IHB", off, len(body), 0) + body
                print(f"  {probe.stamp():7.3f}  -> chunk offset={off:<6} len={len(body)}")
                await client.write_gatt_char(FILE_TRANSFER_UUID, frame, response=False)
                probe.last_chunk_at = time.monotonic()

            verdict = await await_state(probe, "verify_failed", args.listen,
                                        since=mark)
        finally:
            # Always close the session, even on Ctrl-C or an exception: a
            # session left open blocks the next one until the device times it
            # out or reboots.
            try:
                await client.write_gatt_char(COMMAND_UUID, bytes([CMD_OTA_ABORT]),
                                             response=True)
                print(f"  {probe.stamp():7.3f}  -> OTA_ABORT (session closed)")
            except Exception as e:      # noqa: BLE001 — best-effort cleanup
                print(f"  note: OTA_ABORT failed ({e}); power-cycle if the next "
                      f"session refuses")
            await asyncio.sleep(0.2)    # let a trailing status land
            try:
                await client.stop_notify(FILE_OPS_UUID)
            except Exception:           # noqa: BLE001
                pass

    if verdict is None or verdict.get("state") != "verify_failed":
        print(f"FAIL: no refusal within {args.listen:g}s of the last chunk. "
              f"An image that is NOT being refused is the other outcome — check "
              f"you offered the wrong-board build.")
        return 1
    err = verdict.get("err", "(none)")
    written = verdict.get("bytes", "?")
    if args.expect and err != args.expect:
        print(f"FAIL: refused, but with '{err}' — expected '{args.expect}'")
        return 1
    print(f"PASS: refused with {err} after {written} B")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", help="device name substring (e.g. TR-R)")
    p.add_argument("--address", help="connect by CoreBluetooth peripheral UUID")
    p.add_argument("--image", required=True, help="path to the .bin to offer")
    p.add_argument("--target", choices=["fc", "oc"], default="fc",
                   help="fc = OC-relayed Flight Computer OTA (default); "
                        "oc = the connected device's own app")
    p.add_argument("--chunks", type=int,
                   help="how many chunks to send (default: just enough to "
                        "complete the identity header)")
    p.add_argument("--chunk-size", type=int,
                   help="bytes per chunk (default: 220, or what the MTU allows)")
    p.add_argument("--begin-timeout", type=float, default=30.0,
                   help="seconds to wait for 'ready' — an FC relay erases its "
                        "OTA slot first, so this is slow (default 30)")
    p.add_argument("--listen", type=float, default=10.0,
                   help="seconds to wait for a verdict after the last chunk")
    p.add_argument("--expect", help="require this err token for a PASS "
                                    "(e.g. fc_image_identity_mismatch)")
    args = p.parse_args()
    if not args.name and not args.address:
        raise SystemExit("--name <substring> or --address <uuid> required")
    try:
        sys.exit(asyncio.run(run(args)))
    except KeyboardInterrupt:
        sys.exit(130)


if __name__ == "__main__":
    main()
