#!/usr/bin/env python3
"""#1145 check (1): a config push issued WHILE a download is streaming.

Result on a V9, 2026-09-10 (main e1a4bee):

    I  (27735) BLE: Received command: 34 (queued, depth=1)
    I (216102) BLE: Download complete: 6460942 bytes in 191.4s
    I (216104) OC: phone-IO pause ended: 4 stale FC poll(s) discarded, ...
    I (216137) BLE: Pyro config: ... (NVS wrote 24 bytes)

The command sat in the ring for 188 s and was dispatched 33 ms after the pause
ended. Before PR #1284 the stale polls that queued during the transfer would
have consumed it.

TWO TRAPS, both of which cost a run here:

  * The COMMAND characteristic is BLE_GATT_CHR_F_WRITE only — no
    WRITE_NO_RSP (TR_BLE_To_APP.cpp:946). A write-without-response is silently
    not delivered, and the run looks like the command vanished. Use
    response=True.
  * A write-with-response on a MARGINAL link can time out and take the
    connection with it (seen at rssi avg -82, min -109: reason=531, the CENTRAL
    terminating). The board handles that correctly — it aborts the transfer and
    still logs the pause — but the run is wasted. Check the XFER line's rssi
    before blaming the firmware.

The defect: the command was consumed by the stale FC polls that queued during
the transfer, and silently dropped. It must instead land AFTER the transfer.

Issuing the push BETWEEN two runs of the download tool does not exercise this
at all — the tool disconnects, so the push completes long before the transfer
starts. This drives the download and injects the command on the SAME
connection, a few seconds into the stream.
"""
import asyncio, sys, time, hashlib, importlib.util
from bleak import BleakClient
spec = importlib.util.spec_from_file_location("dl", "/tmp/claude-501/dl_tool.py")
dl = importlib.util.module_from_spec(spec); spec.loader.exec_module(dl)

FILENAME = "flight_6.bin"
PUSH_CMD = 34
PUSH_PAYLOAD = bytes.fromhex("010000000000010166661843010000000040000000000000")

async def main():
    dev = await dl.find_device()
    if dev is None:
        print("DEVICE NOT FOUND"); return 2
    print(f"found {dev.name!r}", flush=True)
    async with BleakClient(dev, timeout=30.0) as client:
        fc = dl.FileOpsClient(client)
        await fc.start()
        print("connected; starting download", flush=True)
        while not fc.chunk_q.empty(): fc.chunk_q.get_nowait()
        buf = bytearray(); t0 = time.time()
        await fc.cmd(4, FILENAME.encode())
        eof = False; injected = False; inject_at = None
        timeout = dl.FIRST_RESPONSE_TIMEOUT
        while not eof:
            pkt = await asyncio.wait_for(fc.chunk_q.get(), timeout)
            timeout = dl.CHUNK_STALL_TIMEOUT
            if len(pkt) < 7: continue
            ln = int.from_bytes(pkt[4:6], "little"); flags = pkt[6]
            buf.extend(pkt[7:7+ln]); eof = bool(flags & dl.FLAG_EOF)
            if not injected and time.time() - t0 > 3.0:
                injected = True; inject_at = len(buf)
                await client.write_gatt_char(dl.COMMAND_UUID, bytes([PUSH_CMD]) + PUSH_PAYLOAD, response=True)
                print(f"  INJECTED cmd {PUSH_CMD} at {inject_at:,} B into the stream", flush=True)
        dt = time.time() - t0
        print(f"  -> {FILENAME}: {len(buf):,} B in {dt:.1f}s "
              f"sha256={hashlib.sha256(buf).hexdigest()[:16]}", flush=True)
        print(f"  injected mid-transfer at {inject_at:,} of {len(buf):,} B", flush=True)
        print("  holding the link 10 s so the OC can dispatch it after the transfer", flush=True)
        await asyncio.sleep(10)
    return 0
sys.exit(asyncio.run(main()))
