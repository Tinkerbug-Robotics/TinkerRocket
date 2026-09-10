#!/usr/bin/env python3
"""Read an ESP-IDF application image's own description of itself (#773).

Every fact a firmware release needs about a `.bin` is already inside it: the
CMake `project()` name, the version string the device will report after
rebooting, the chip it was built for, and the IDF that built it. Nothing has to
be inferred from a filename or a build directory, which is the whole point —
a filename is what a human typed, and the header is what will actually run.

This is the canonical parser. The two apps carry ports of it that must agree
byte for byte:
  TinkerRocketAndroid/core/protocol/.../EspImage.kt
  TinkerRocketApp/TinkerRocketApp/Models/EspImage.swift

Layout (esp_image_format.h), verified against real images built from this repo:
  0   esp_image_header_t, 24 B: [0] magic 0xE9, [12:14] chip_id LE u16,
      [15:17] min_chip_rev_full, [17:19] max_chip_rev_full
  24  esp_image_segment_header_t, 8 B
  32  esp_app_desc_t: magic 0xABCD5432, +16 version[32], +48 project_name[32],
      +80 time[16], +96 date[16], +112 idf_ver[32]

Usage:
    python3 image_info.py <image.bin> [more.bin ...]      # JSON array
    python3 image_info.py --manifest <tag> <image.bin>... # release manifest
"""
from __future__ import annotations

import hashlib
import json
import struct
import sys
from pathlib import Path

IMAGE_MAGIC = 0xE9
APP_DESC_MAGIC = 0xABCD5432
APP_DESC_OFFSET = 32
MIN_LENGTH = APP_DESC_OFFSET + 256

# esp_chip_id_t. An id absent here is reported as its raw number rather than
# guessed at — a new Espressif part must be added deliberately.
CHIP_NAMES = {
    0x0000: "ESP32", 0x0002: "ESP32-S2", 0x0005: "ESP32-C3",
    0x0009: "ESP32-S3", 0x000C: "ESP32-C2", 0x000D: "ESP32-C6",
    0x0010: "ESP32-H2", 0x0012: "ESP32-P4",
}


class NotAnEspImage(ValueError):
    """The file is not an ESP-IDF application image."""


def _field(buf: bytes, off: int, length: int) -> str:
    """NUL-terminated fixed-width ASCII; anything unprintable ends it."""
    out = []
    for b in buf[off:off + length]:
        if b == 0 or b < 0x20 or b > 0x7E:
            break
        out.append(chr(b))
    return "".join(out)


def parse(path: str | Path) -> dict:
    data = Path(path).read_bytes()
    if len(data) < MIN_LENGTH:
        raise NotAnEspImage(f"{path}: too short to be an ESP-IDF image")
    if data[0] != IMAGE_MAGIC:
        raise NotAnEspImage(f"{path}: image magic is {data[0]:#04x}, not 0xE9")
    if struct.unpack_from("<I", data, APP_DESC_OFFSET)[0] != APP_DESC_MAGIC:
        raise NotAnEspImage(f"{path}: no esp_app_desc_t at offset {APP_DESC_OFFSET}")

    chip_id = struct.unpack_from("<H", data, 12)[0]
    return {
        "file": Path(path).name,
        "project": _field(data, APP_DESC_OFFSET + 48, 32),
        "version": _field(data, APP_DESC_OFFSET + 16, 32),
        "chip_id": chip_id,
        "chip": CHIP_NAMES.get(chip_id, f"chip 0x{chip_id:04X}"),
        "min_chip_rev_full": struct.unpack_from("<H", data, 15)[0],
        "max_chip_rev_full": struct.unpack_from("<H", data, 17)[0],
        "idf_version": _field(data, APP_DESC_OFFSET + 112, 32),
        "build_date": _field(data, APP_DESC_OFFSET + 96, 16),
        "build_time": _field(data, APP_DESC_OFFSET + 80, 16),
        "size": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
    }


def board_of(version: str) -> str | None:
    """The board revision the image ASSERTS, from the `-v9` in its version string.

    None when the build carries no suffix. This is the image's own claim, which
    is not the same as the board it is on — a wrongly flashed board reports the
    wrong revision forever — so it is a label, never a gate.

    THE LETTER IS NOT ALWAYS `v`. The projects stamp three shapes, all set in
    their own CMakeLists from the same TR_BOARD_SUFFIX that reaches the
    compiler as TR_BOARD_OTA_SUFFIX:

      -v7 -v8 -v9   flight_computer, out_computer, base_station
      -m1           flight_computer, out_computer, on the rocket-computer-mini
      -b1           rocket_computer_mini, when TR_MINI_BOARD is set

    This matched only `-v` until 2026-09-10, so every `-m1` image came out of
    the manifest with no board at all — labelled "applies everywhere" when it
    is the one image that applies to exactly one board. The firmware's own
    check never had the bug (TR_OTA_Receiver strstr's the whole suffix), so a
    mini image aimed at a V9 was refused by the board while the app offered it
    as a valid choice.
    """
    import re
    m = re.search(r"-([vmb]\d+)(?:[+\-]|$)", version, re.IGNORECASE)
    return m.group(1).lower() if m else None


def manifest(tag: str, paths: list[str]) -> dict:
    images = []
    for p in paths:
        info = parse(p)
        info["board"] = board_of(info["version"])
        images.append(info)
    images.sort(key=lambda i: (i["project"], i["board"] or ""))
    return {
        "manifest_version": 1,
        "tag": tag,
        "images": images,
    }


def main(argv: list[str]) -> int:
    if len(argv) >= 3 and argv[1] == "--manifest":
        print(json.dumps(manifest(argv[2], argv[3:]), indent=2))
        return 0
    if len(argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    out = []
    for p in argv[1:]:
        try:
            out.append(parse(p))
        except NotAnEspImage as e:
            print(f"error: {e}", file=sys.stderr)
            return 1
    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
