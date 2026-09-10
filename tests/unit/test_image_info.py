"""#773: the canonical ESP-IDF image parser used to build a firmware manifest.

The header values here come from real images built out of this repo, so the
offsets are pinned against the actual layout rather than against a reading of
`esp_image_format.h`:

  flight_computer      chip 0x0012  min/max chip rev 100/199  (the #690 v1.x P4)
  out_computer         chip 0x0009  min/max chip rev 0/99
  rocket_computer_mini chip 0x0009  version carries no board suffix

The two apps carry ports of this parser (EspImage.kt / EspImage.swift) with
their own mirrored suites; this one additionally covers the manifest, which is
release-side only.
"""
import json
import struct
import subprocess
import sys
from pathlib import Path

import pytest

TOOL = Path(__file__).resolve().parents[2] / "tinkerrocket-idf" / "tools" / "image_info.py"
sys.path.insert(0, str(TOOL.parent))
import image_info  # noqa: E402


def make_image(tmp_path, name="fw.bin", *, project="out_computer",
               version="7410cdc6-dirty-v9+20260909-2056", chip_id=0x0009,
               magic=0xE9, desc_magic=0xABCD5432, size=4096,
               min_rev=0, max_rev=99):
    b = bytearray(size)
    b[0] = magic
    struct.pack_into("<H", b, 12, chip_id)
    struct.pack_into("<H", b, 15, min_rev)
    struct.pack_into("<H", b, 17, max_rev)
    struct.pack_into("<I", b, 32, desc_magic)

    def put(off, s, length):
        raw = s.encode()[: length - 1]
        b[off:off + len(raw)] = raw

    put(48, version, 32)
    put(80, project, 32)
    put(112, "14:35:47", 16)
    put(128, "Sep  9 2026", 16)
    put(144, "v6.0.1-dirty", 32)
    p = tmp_path / name
    p.write_bytes(bytes(b))
    return p


def test_parses_a_real_flight_computer_header(tmp_path):
    p = make_image(tmp_path, project="flight_computer",
                   version="537dc3ff-dirty-v9+20260909-1835",
                   chip_id=0x0012, min_rev=100, max_rev=199)
    info = image_info.parse(p)
    assert info["project"] == "flight_computer"
    assert info["version"] == "537dc3ff-dirty-v9+20260909-1835"
    assert info["chip"] == "ESP32-P4"
    assert info["chip_id"] == 0x0012
    # #690: the P4s on hand are v1.3, and v1.x/v3.x images are NOT
    # interchangeable. The header is where that targeting is visible.
    assert (info["min_chip_rev_full"], info["max_chip_rev_full"]) == (100, 199)
    assert info["idf_version"] == "v6.0.1-dirty"
    assert info["size"] == 4096
    assert len(info["sha256"]) == 64


def test_refuses_anything_that_is_not_an_esp_idf_image(tmp_path):
    short = tmp_path / "short.bin"
    short.write_bytes(b"\xe9" * 16)
    with pytest.raises(image_info.NotAnEspImage):
        image_info.parse(short)
    with pytest.raises(image_info.NotAnEspImage):
        image_info.parse(make_image(tmp_path, "bad_magic.bin", magic=0x50))
    with pytest.raises(image_info.NotAnEspImage):
        image_info.parse(make_image(tmp_path, "no_desc.bin", desc_magic=0))


def test_board_suffix_is_read_from_the_version_string():
    assert image_info.board_of("537dc3ff-dirty-v9+20260909-1835") == "v9"
    assert image_info.board_of("abc-V8+1") == "v8"
    assert image_info.board_of("abc-v10") == "v10"
    # The mini's single-MCU project carries no suffix — that is not a failure.
    assert image_info.board_of("7410cdc6-dirty+20260909-2043") is None
    # A trailing build stamp must not be mistaken for a board.
    assert image_info.board_of("abc+20260909-1835") is None


def test_manifest_sorts_and_labels(tmp_path):
    a = make_image(tmp_path, "oc.bin", project="out_computer",
                   version="aaa-v9+1", chip_id=0x0009)
    b = make_image(tmp_path, "fc.bin", project="flight_computer",
                   version="aaa-v8+1", chip_id=0x0012)
    m = image_info.manifest("fw-v1.2.3", [str(a), str(b)])
    assert m["manifest_version"] == 1
    assert m["tag"] == "fw-v1.2.3"
    # Sorted by project then board, so a diff between two manifests is readable.
    assert [i["project"] for i in m["images"]] == ["flight_computer", "out_computer"]
    assert m["images"][0]["board"] == "v8"
    assert m["images"][1]["board"] == "v9"


def test_the_sha_is_over_the_whole_file(tmp_path):
    import hashlib
    p = make_image(tmp_path)
    assert image_info.parse(p)["sha256"] == hashlib.sha256(p.read_bytes()).hexdigest()


def test_cli_manifest_mode_emits_json(tmp_path):
    p = make_image(tmp_path)
    out = subprocess.run(
        [sys.executable, str(TOOL), "--manifest", "fw-v9.9.9", str(p)],
        capture_output=True, text=True, check=True,
    )
    m = json.loads(out.stdout)
    assert m["tag"] == "fw-v9.9.9"
    assert m["images"][0]["project"] == "out_computer"


def test_cli_fails_loudly_on_a_bad_image(tmp_path):
    bad = tmp_path / "notfw.bin"
    bad.write_bytes(b"hello world" * 100)
    out = subprocess.run([sys.executable, str(TOOL), str(bad)],
                         capture_output=True, text=True)
    assert out.returncode == 1
    assert "not" in out.stderr.lower() or "magic" in out.stderr.lower()


def test_an_unknown_chip_id_is_reported_not_guessed(tmp_path):
    info = image_info.parse(make_image(tmp_path, chip_id=0x00FE))
    assert info["chip"] == "chip 0x00FE"
    assert info["chip_id"] == 0x00FE
