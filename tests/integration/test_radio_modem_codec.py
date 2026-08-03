"""CI guard for the bench host's copy of the radio-modem wire format (#409).

tools/bench_radio_modem.py reimplements the tr_msg codec in Python so the
daughterboard can be driven from a laptop without a rocket attached. Two
independent implementations of one wire format is exactly the arrangement
that drifts, so both ends are pinned to the same literal frames:

  * the C++ side in tests_cpp/test_uart_link_codec.cpp (PackGoldenFrame)
  * the Python side here, via the tool's own --selftest constants

If someone changes the CRC parameters, the SOF, or a payload struct, one of
those two fails. Neither can be satisfied by "recompute it the same wrong
way", which is the failure mode a computed-expectation test has.

No hardware and no pyserial: the tool imports serial lazily, inside
ModemLink.__init__, precisely so this runs anywhere.
"""
import importlib.util
import struct
from pathlib import Path

import pytest

TOOL = Path(__file__).resolve().parents[2] / "tools" / "bench_radio_modem.py"


@pytest.fixture(scope="module")
def modem():
    spec = importlib.util.spec_from_file_location("bench_radio_modem", TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_tool_exists():
    assert TOOL.is_file(), f"{TOOL} missing — the bench acceptance path needs it"


def test_golden_frames_match_firmware(modem):
    """The frames here were emitted by tr_msg::pack itself, not hand-computed."""
    for msg_type, payload_hex, expect in modem.GOLDEN:
        got = modem.pack(msg_type, bytes.fromhex(payload_hex)).hex().upper()
        assert got == expect, f"pack(0x{msg_type:02X}, {payload_hex!r})"


def test_crc_parameters(modem):
    # Pinned independently of pack() so a SOF change and a CRC change are
    # distinguishable when this file goes red.
    assert modem.crc16(bytes([0x42, 0x03, 0x01, 0x02, 0x03])) == 0x0066
    assert modem.crc16(b"") == 0x0000


def test_deframer_recovers_from_garbage_and_corruption(modem):
    d = modem.Deframer()
    good = modem.pack(0x33, b"\x01\x02")
    assert list(d.feed(b"\x00\xFF\xAA\xAA\x13\x55" + good)) == [(0x33, b"\x01\x02")]
    # 5, not 6 — see the note in Deframer. Confirmed against the on-chip
    # self-test and pinned identically in tests_cpp.
    assert d.resync_bytes == 5

    d = modem.Deframer()
    bad = bytearray(modem.pack(0x44, b"\x0A\x14\x1E"))
    bad[7] ^= 0xFF
    assert list(d.feed(bytes(bad) + modem.pack(0x55, b"\x2A"))) == [(0x55, b"\x2A")]
    assert d.crc_fails == 1


def test_truncated_frame_costs_exactly_two(modem):
    """The behaviour a scan-for-SOF host implementation gets wrong.

    Length-driven framing means a truncated frame eats the next frame's SOF
    as its own payload/CRC. A host that instead re-anchors on the next SOF
    loses one frame where the modem loses two, and would report better link
    quality than the link actually has.
    """
    d = modem.Deframer()
    truncated = modem.pack(0x66, bytes(range(1, 9)))[:-4]
    stream = truncated + modem.pack(0x77, b"\xEE") + modem.pack(0x78, b"\xEF")
    assert list(d.feed(stream)) == [(0x78, b"\xEF")]  # 0x77 is the casualty
    assert d.crc_fails == 1


def test_payload_struct_sizes(modem):
    """Mirrors the static_asserts in RadioModemProtocol.h."""
    for fmt, size in ((modem.FMT_CONFIG, 16), (modem.FMT_HOP, 4),
                      (modem.FMT_RX_HEADER, 8), (modem.FMT_TX_RESULT, 2),
                      (modem.FMT_IDENTITY, 44), (modem.FMT_STATUS, 52),
                      (modem.FMT_SCAN_REQ, 12), (modem.FMT_SCAN_HEADER, 12)):
        assert struct.calcsize(fmt) == size, fmt


def test_selftest_passes(modem, capsys):
    """The tool's own --selftest, so the two never diverge in coverage."""
    rc = modem.selftest()
    assert rc == 0, capsys.readouterr().out
