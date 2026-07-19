#!/usr/bin/env python3
"""Bench BLE command tool (#150) — drive TinkerRocket BLE commands from a Mac.

The iOS app is the normal command path, but bench work sometimes needs a
command the app has no UI for yet (the #150 hop toggle was the motivating
case).  This connects to a device over CoreBluetooth (via bleak) and writes
the COMMAND characteristic with exactly the bytes the app's
sendRawCommand() would: [cmd_u8][payload...], write-with-response.

Wire facts (TR_BLE_To_APP.h / BLEDevice.swift):
  service    4fafc201-1fb5-459e-8fcc-c5c9c331914b
  telemetry  beb5483e-36e1-4688-b7f5-ea07361b26a8   (notify: JSON)
  command    cba1d466-344c-4be3-ab3f-189f80dd7518   (write)

Examples:
  python3 tools/bench_ble_cmd.py --scan
  python3 tools/bench_ble_cmd.py --name TR-B --hop on --listen 6
  python3 tools/bench_ble_cmd.py --name TR-B --hop off
  python3 tools/bench_ble_cmd.py --name TR-B --lora 915.0 250 9 5 12
  python3 tools/bench_ble_cmd.py --name TR-B --cmd 20 --listen 4   # config readback
  # cmd 60 noise scan 902..928 MHz, 500 kHz step, 30 ms dwell
  # payload = [start f32][stop f32][step_khz u16][dwell_ms u16], little-endian:
  python3 tools/bench_ble_cmd.py --name TR-B --cmd 60 --payload 0080614400006844f4011e00

Notes:
  * The device supports a limited number of BLE centrals — if the iOS app
    is connected to the same device, disconnect it first.
  * --hop on sends cmd 17 payload 0x00 (payload is the hop_DISABLED flag).
  * The firmware refuses a hop enable when the current modulation cannot
    hop legally (config readback shows "lhdw":0) — watch --listen output.
"""

import argparse
import asyncio
import struct
import sys

from bleak import BleakClient, BleakScanner

SERVICE_UUID   = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
TELEMETRY_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
COMMAND_UUID   = "cba1d466-344c-4be3-ab3f-189f80dd7518"

SCAN_TIMEOUT_S = 15.0   # generous: a marginal-RSSI BS at 1 Hz advertising needs headroom


def build_payload(args) -> tuple[int, bytes]:
    """Resolve the (cmd, payload) to send from the CLI sugar options."""
    if args.hop is not None:
        # cmd 17 payload byte = lora_hop_disabled (0 = hopping ON)
        return 17, bytes([0x00 if args.hop == "on" else 0x01])
    if args.lora is not None:
        freq, bw, sf, cr, pwr = args.lora
        # BS BLE cmd 10: [freq f32][bw f32][sf u8][cr u8][pwr i8], little-endian
        return 10, struct.pack("<ffBBb", freq, bw, int(sf), int(cr), int(pwr))
    if args.cmd is not None:
        payload = bytes.fromhex(args.payload) if args.payload else b""
        return args.cmd, payload
    raise SystemExit("nothing to send: use --hop/--lora/--cmd (or --scan)")


async def scan(name_filter: str | None):
    print(f"Scanning {SCAN_TIMEOUT_S:.0f}s ...")
    found = await BleakScanner.discover(timeout=SCAN_TIMEOUT_S, return_adv=True)
    hits = 0
    for d, ad in found.values():
        name = d.name or ""
        # Renamed devices advertise the raw user-set unit name (no TR-
        # prefix), so a name gate hides them — the service UUID is the
        # real membership test (same bug/fix as the app's scanner).
        advertises_service = SERVICE_UUID.lower() in [u.lower() for u in (ad.service_uuids or [])]
        if name_filter:
            if name_filter.lower() not in name.lower():
                continue
        elif not (advertises_service or name.startswith("TR-") or "Base" in name):
            continue
        tag = "svc" if advertises_service else "   "
        print(f"  {d.address}  rssi={ad.rssi}  {tag}  {name}")
        hits += 1
    if hits == 0:
        print("  no matching devices (is it powered? is the app connected to it?)")


async def find_device(name_substr: str):
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: name_substr.lower() in (d.name or "").lower(),
        timeout=SCAN_TIMEOUT_S,
    )
    if dev is None:
        raise SystemExit(
            f"device matching '{name_substr}' not found — power it, and make "
            f"sure the iOS app isn't holding its only BLE connection")
    return dev


async def run(args):
    if args.scan:
        await scan(args.name)
        return

    if not args.name and not args.address:
        raise SystemExit("--name <substring> or --address <uuid> required")

    cmd, payload = build_payload(args)
    if args.address:
        # Connect by cached identifier — on macOS this can reach a known
        # peripheral whose advertisements are too weak to scan reliably.
        dev = args.address
        print(f"Connecting by address {args.address} ...")
    else:
        dev = await find_device(args.name)
        print(f"Connecting to {dev.name} ({dev.address}) ...")

    async with BleakClient(dev) as client:
        if args.listen > 0:
            def on_notify(_h, data: bytearray):
                try:
                    print(f"  <- {data.decode('utf-8', 'replace')}")
                except Exception:
                    print(f"  <- {data.hex()}")
            await client.start_notify(TELEMETRY_UUID, on_notify)

        frame = bytes([cmd]) + payload
        print(f"Writing cmd {cmd} payload={payload.hex() or '(none)'} "
              f"[{frame.hex()}]")
        await client.write_gatt_char(COMMAND_UUID, frame, response=True)
        print("Write OK")

        if args.listen > 0:
            print(f"Listening {args.listen}s for notifications ...")
            await asyncio.sleep(args.listen)
            await client.stop_notify(TELEMETRY_UUID)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scan", action="store_true",
                   help="list nearby TinkerRocket devices and exit")
    p.add_argument("--name", help="device name substring (e.g. TR-B, TR-R)")
    p.add_argument("--address", help="connect by CoreBluetooth peripheral UUID "
                                     "(works at RSSI too weak for name scans)")
    p.add_argument("--cmd", type=int, help="raw BLE command id")
    p.add_argument("--payload", default="", help="raw payload as hex string")
    p.add_argument("--hop", choices=["on", "off"],
                   help="cmd-17 sugar: link mode (on = hopping)")
    p.add_argument("--lora", nargs=5, type=float, metavar=("MHZ", "BW", "SF", "CR", "DBM"),
                   help="cmd-10 sugar: LoRa reconfigure (e.g. 915.0 250 9 5 12)")
    p.add_argument("--listen", type=float, default=0.0,
                   help="after writing, print telemetry-char notifications for N seconds")
    args = p.parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        sys.exit(130)


if __name__ == "__main__":
    main()
