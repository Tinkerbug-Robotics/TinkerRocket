#!/usr/bin/env python3
"""BLE flight-file downloader for the TinkerRocket rocket computer (OC).

Speaks the app's file-ops protocol (verified against TR_BLE_To_APP.cpp and
out_computer/main.cpp @ d7017c0-v8):

  COMMAND       cba1d466-344c-4be3-ab3f-189f80dd7518  write [cmd][payload]
  FILE_OPS      8d53dc1d-1db7-4cd3-868b-8a527460aa84  notify: file-list JSON
  FILE_TRANSFER 1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d  notify: [off u32][len u16][flags u8][data]

  cmd 2 [page]      -> file list page (5/page, newest first), JSON on FILE_OPS
  cmd 4 [filename]  -> stream whole file as chunks; flags bit0=EOF bit1=ABORT

Usage:
  ble_download.py scan
  ble_download.py list
  ble_download.py fetch <outdir> [name ...]   # no names = all listed files
  ble_download.py recover <outdir>            # rail-on (cmd 8) -> wait for
                                              # brownout/MRAM recovery -> list,
                                              # download flight files -> rail-off

recover exists because in idle mode (rail off) the OC never runs
flightlog.begin(): the index is not loaded and cmd 2 truthfully reports 0
files. flightlog.begin() + scanForBrownoutRecovery() + the #274 MRAM drain
all run from initPeripherals(), which only the rail-on path calls.
"""
import asyncio, hashlib, json, sys, time
from pathlib import Path

from bleak import BleakClient, BleakScanner

SERVICE_UUID       = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
COMMAND_UUID       = "cba1d466-344c-4be3-ab3f-189f80dd7518"
FILE_OPS_UUID      = "8d53dc1d-1db7-4cd3-868b-8a527460aa84"
FILE_TRANSFER_UUID = "1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d"

TARGET_NAME = "RC V8"
SCAN_S = 15.0
FIRST_RESPONSE_TIMEOUT = 25.0   # first file op lazily inits NAND in idle mode
CHUNK_STALL_TIMEOUT = 30.0

FLAG_EOF, FLAG_ABORT = 0x01, 0x02


async def find_device():
    print(f"scanning {SCAN_S:.0f}s for '{TARGET_NAME}' / service uuid ...", flush=True)
    found = await BleakScanner.discover(timeout=SCAN_S, return_adv=True)
    best = None
    for d, ad in found.values():
        name = d.name or ""
        has_svc = SERVICE_UUID.lower() in [u.lower() for u in (ad.service_uuids or [])]
        if name == TARGET_NAME or (has_svc and TARGET_NAME.lower() in name.lower()):
            return d
        if has_svc or name.startswith("TR-"):
            print(f"  other TR device: {d.address} rssi={ad.rssi} name={name!r}")
            if best is None:
                best = d
    return best


class FileOpsClient:
    def __init__(self, client):
        self.client = client
        self.fileops_q = asyncio.Queue()
        self.chunk_q = asyncio.Queue()

    def _on_fileops(self, _h, data: bytearray):
        if data[:1] == b"[":
            self.fileops_q.put_nowait(bytes(data))

    def _on_chunk(self, _h, data: bytearray):
        self.chunk_q.put_nowait(bytes(data))

    async def start(self):
        await self.client.start_notify(FILE_OPS_UUID, self._on_fileops)
        await self.client.start_notify(FILE_TRANSFER_UUID, self._on_chunk)

    async def cmd(self, cmd: int, payload: bytes = b""):
        await self.client.write_gatt_char(COMMAND_UUID, bytes([cmd]) + payload, response=True)

    async def list_page(self, page: int, timeout: float):
        while not self.fileops_q.empty():
            self.fileops_q.get_nowait()
        await self.cmd(2, bytes([page]))
        raw = await asyncio.wait_for(self.fileops_q.get(), timeout)
        return json.loads(raw.decode())

    async def list_all(self):
        entries, page = [], 0
        timeout = FIRST_RESPONSE_TIMEOUT
        while True:
            batch = await self.list_page(page, timeout)
            entries.extend(batch)
            if len(batch) < 5:
                return entries
            page += 1
            timeout = 10.0

    async def download(self, name: str, out_path: Path):
        while not self.chunk_q.empty():
            self.chunk_q.get_nowait()
        buf = bytearray()
        t0 = time.time()
        await self.cmd(4, name.encode())
        eof = aborted = False
        gaps = 0
        started = False
        timeout = FIRST_RESPONSE_TIMEOUT
        last_report = t0
        while not eof:
            pkt = await asyncio.wait_for(self.chunk_q.get(), timeout)
            timeout = CHUNK_STALL_TIMEOUT
            if len(pkt) < 7:
                print(f"  short packet ({len(pkt)} B) ignored", flush=True)
                continue
            off = int.from_bytes(pkt[0:4], "little")
            ln = int.from_bytes(pkt[4:6], "little")
            flags = pkt[6]
            data = pkt[7:7 + ln]
            if not started:
                # A late notification from the PREVIOUS transfer (its EOF tail)
                # can slip in after the pre-send drain; a real stream always
                # opens at offset 0. Discard anything else until it does.
                if off != 0:
                    print(f"  discarding stale pre-stream chunk off={off}", flush=True)
                    continue
                started = True
            if len(data) != ln:
                print(f"  length mismatch: hdr={ln} got={len(data)}", flush=True)
            if off != len(buf):
                gaps += 1
                print(f"  offset discontinuity: expected {len(buf)}, got {off}", flush=True)
                if off > len(buf):
                    buf.extend(b"\x00" * (off - len(buf)))
                    del buf[off:]
            buf.extend(data)
            eof = bool(flags & FLAG_EOF)
            aborted = bool(flags & FLAG_ABORT)
            now = time.time()
            if now - last_report > 5:
                rate = len(buf) / (now - t0) / 1024
                print(f"  {len(buf):>10,} B  {rate:6.1f} kB/s", flush=True)
                last_report = now
        dt = time.time() - t0
        sha = hashlib.sha256(buf).hexdigest()
        status = "ABORTED" if aborted else "ok"
        out_path.write_bytes(buf)
        print(f"  -> {out_path.name}: {len(buf):,} B in {dt:.1f}s "
              f"({len(buf)/max(dt,1e-9)/1024:.1f} kB/s) {status} gaps={gaps} sha256={sha[:16]}",
              flush=True)
        return {"name": name, "bytes": len(buf), "seconds": round(dt, 1),
                "aborted": aborted, "gaps": gaps, "sha256": sha}


def is_flight_target(name: str) -> bool:
    n = name.lower()
    return ("recovered" in n) or ("2026082" in n) or ("2026083" in n)


async def do_downloads(fc, entries, outdir: Path, wanted_names):
    outdir.mkdir(parents=True, exist_ok=True)
    results = []
    for name in wanted_names:
        listed = next((e for e in entries if e["name"] == name), None)
        print(f"\ndownloading {name}"
              + (f" (listed size {listed['size']:,} B)" if listed else ""), flush=True)
        r = await fc.download(name, outdir / name)
        r["listed_size"] = listed["size"] if listed else None
        results.append(r)
        await asyncio.sleep(1.0)  # let the previous transfer's tail notifications flush
    return results


def write_manifest(outdir: Path, dev, entries, results):
    manifest = {"device": dev.name, "listed": entries, "downloads": results,
                "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    (outdir / "download_manifest.json").write_text(json.dumps(manifest, indent=2))
    print("\nmanifest written", flush=True)
    bad = [r for r in results if r["aborted"] or (r["listed_size"] not in (None, r["bytes"]))]
    if bad:
        print("WARNING: incomplete downloads:", [r["name"] for r in bad], flush=True)
        return 1
    return 0


async def recover_mode(fc, dev, outdir: Path):
    # Rail on. initPeripherals() blocks the OC loop for seconds, then
    # flightlog.begin() runs the brownout scan (<=90 s budget) and the #274
    # MRAM drain. Poll the file list until it goes non-empty or we time out.
    print("sending cmd 8 (FC rail ON) to trigger flightlog init + recovery...", flush=True)
    await fc.cmd(8)
    t0 = time.time()
    entries = []
    while time.time() - t0 < 240:
        await asyncio.sleep(6)
        try:
            entries = await fc.list_all()
        except (asyncio.TimeoutError, Exception) as e:  # noqa: BLE001 - keep polling
            print(f"  list attempt: {type(e).__name__} {e} ({time.time()-t0:.0f}s)", flush=True)
            continue
        print(f"  t+{time.time()-t0:5.0f}s: {len(entries)} file(s) listed", flush=True)
        if entries:
            break
    print(f"\n{len(entries)} file(s) on board (newest first):")
    for e in entries:
        print(f"  {e['name']:<34} {e['size']:>12,} B")
    if not entries:
        print("no files appeared after rail-on — see serial log for FLIGHTLOG lines")
        return 3

    names = [e["name"] for e in entries]
    wanted = [n for n in names if is_flight_target(n)] if len(names) > 8 else names
    skipped = [n for n in names if n not in wanted]
    if skipped:
        print(f"skipping {len(skipped)} older file(s): {skipped}")
    results = await do_downloads(fc, entries, outdir, wanted)
    rc = write_manifest(outdir, dev, entries, results)

    print("\nsending cmd 8 (FC rail OFF — OC will esp_restart, disconnect expected)", flush=True)
    try:
        await fc.cmd(8)
        await asyncio.sleep(2)
    except Exception as e:
        print(f"  (rail-off write: {type(e).__name__} — fine if already restarting)", flush=True)
    return rc


async def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "list"
    dev = await find_device()
    if dev is None:
        print("DEVICE NOT FOUND — if the iOS app is connected to the rocket, "
              "close it (it holds the only BLE slot) and re-run.")
        return 2
    print(f"found {dev.name!r} @ {dev.address}", flush=True)
    if mode == "scan":
        return 0

    async with BleakClient(dev, timeout=30.0) as client:
        print("connected", flush=True)
        fc = FileOpsClient(client)
        await fc.start()

        if mode == "recover":
            return await recover_mode(fc, dev, Path(sys.argv[2]))

        entries = await fc.list_all()
        print(f"\n{len(entries)} file(s) on board (newest first):")
        for e in entries:
            print(f"  {e['name']:<34} {e['size']:>12,} B")
        if mode == "list":
            return 0

        outdir = Path(sys.argv[2])
        wanted = sys.argv[3:] or [e["name"] for e in entries]
        results = await do_downloads(fc, entries, outdir, wanted)
        return write_manifest(outdir, dev, entries, results)
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
