#!/usr/bin/env python3
"""Does the flash size this build DECLARES match the part actually fitted?

Why this exists: the two ways of getting CONFIG_ESPTOOLPY_FLASHSIZE wrong are
not symmetric, and the bad one is invisible.

  declared < fitted   benign.  The board boots and logs "Detected size(16384k)
                      larger than the size in the binary image header(8192k)";
                      the top of the part is simply unreachable.

  declared > fitted   FATAL, and it happens in the second-stage bootloader,
                      before app_main, before any of our logging exists:

                        E spi_flash: Detected size(8192k) smaller than the size
                                     in the binary image header(16384k).
                                     Probe failed.
                        E esp_core_dump_flash: Core dump flash config is
                                     corrupted!

                      Continuous boot loop, on hardware that is perfectly good.

Because that failure precedes the firmware, no runtime assertion can catch it —
the check has to happen before or at flash time, which is what this is. Measured
2026-08-24 on the V7 rocket computer: both MCUs carry an 8 MB W25Q64 while the
then-current defaults declared 16 MB, and a fresh build dir was all it took to
produce an unbootable image.

Usage:
    python tools/check_flash_size.py --port /dev/cu.usbmodem101 \
                                     --build-dir projects/flight_computer/build_v7

    # just read the part, no build to compare against
    python tools/check_flash_size.py --port /dev/cu.usbmodem101

Needs esptool on PATH or importable, i.e. run it inside the IDF environment
(`. $HOME/esp/esp-idf-v6.0/export.sh`).

Exit status: 0 match (or declared-smaller, which only warns), 1 declared LARGER
than fitted, 2 could not determine one of the two.
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

# esp_image_header_t byte 3, high nibble. Values from
# components/esp_bootloader_format/include/esp_bootloader_desc.h and the
# esptool image header docs.
HEADER_NIBBLE_TO_MB = {0x0: 1, 0x1: 2, 0x2: 4, 0x3: 8, 0x4: 16, 0x5: 32,
                       0x6: 64, 0x7: 128}


def declared_from_bootloader(build_dir: Path) -> int | None:
    """MB declared in the image header of the bootloader we would flash.

    Preferred over sdkconfig because it is the byte the ROM actually reads.
    A build dir whose sdkconfig was edited but not rebuilt disagrees with
    itself, and this side is the one that ships.
    """
    bl = build_dir / "bootloader" / "bootloader.bin"
    if not bl.is_file():
        return None
    head = bl.read_bytes()[:4]
    if len(head) < 4 or head[0] != 0xE9:
        return None  # not an image header (magic byte)
    return HEADER_NIBBLE_TO_MB.get(head[3] >> 4)


def declared_from_sdkconfig(build_dir: Path) -> int | None:
    """MB from the generated sdkconfig, as a fallback for an unbuilt dir."""
    cfg = build_dir / "sdkconfig"
    if not cfg.is_file():
        return None
    m = re.search(r'^CONFIG_ESPTOOLPY_FLASHSIZE="(\d+)MB"',
                  cfg.read_text(), re.M)
    return int(m.group(1)) if m else None


def fitted(port: str) -> tuple[int | None, str]:
    """MB reported by the chip itself, plus the raw JEDEC line for the log."""
    try:
        out = subprocess.run([sys.executable, "-m", "esptool", "-p", port,
                              "flash-id"],
                             capture_output=True, text=True, timeout=120).stdout
    except (OSError, subprocess.SubprocessError) as e:
        print(f"ERROR: could not run esptool: {e}", file=sys.stderr)
        return None, ""
    size = re.search(r"Detected flash size:\s*(\d+)MB", out)
    mfr = re.search(r"Manufacturer:\s*(\w+)", out)
    dev = re.search(r"Device:\s*(\w+)", out)
    jedec = f"manufacturer {mfr.group(1)}, device {dev.group(1)}" if mfr and dev else "JEDEC id unread"
    return (int(size.group(1)) if size else None), jedec


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", required=True, help="serial port of the board")
    ap.add_argument("--build-dir", type=Path,
                    help="build dir to check; omit to only report the part")
    args = ap.parse_args()

    actual, jedec = fitted(args.port)
    if actual is None:
        print("ERROR: could not read the flash size from the board.",
              file=sys.stderr)
        return 2
    print(f"fitted:   {actual} MB ({jedec}) on {args.port}")

    if args.build_dir is None:
        return 0

    declared = declared_from_bootloader(args.build_dir)
    source = "bootloader.bin image header"
    if declared is None:
        declared = declared_from_sdkconfig(args.build_dir)
        source = "sdkconfig (bootloader not built yet)"
    if declared is None:
        print(f"ERROR: no flash size found in {args.build_dir} — is it a build "
              f"dir that has been configured?", file=sys.stderr)
        return 2
    print(f"declared: {declared} MB ({source})")

    if declared > actual:
        print(f"\nFAIL: this image declares {declared} MB against a {actual} MB "
              f"part.\nFlashing it will boot-loop the board in the bootloader "
              f"with\n  'Detected size({actual * 1024}k) smaller than the size "
              f"in the binary image header({declared * 1024}k). Probe failed.'"
              f"\nand no firmware log at all. Fix the declaration for this "
              f"board revision before flashing.", file=sys.stderr)
        return 1
    if declared < actual:
        print(f"\nOK (undersized): {actual - declared} MB at the top of the part "
              f"is unreachable and every boot logs a benign\n  'Detected "
              f"size({actual * 1024}k) larger than the size in the binary image "
              f"header({declared * 1024}k)'.\nSafe to flash. This is the "
              f"deliberate default for board revisions whose part is mixed or "
              f"unread.")
        return 0
    print("\nOK: declaration matches the fitted part.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
