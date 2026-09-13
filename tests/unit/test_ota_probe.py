"""tools/ota_probe.py — wire framing, the identity arithmetic, and the whole
flow against a fake device.

The probe exists to answer #1425's remaining question on a bench in seconds
rather than through a multi-minute push from the phone, so its two failure
modes both cost bench time: a frame the firmware will not accept, and a
verdict that reads as PASS when it should not.  Both are checked here.

It drives `ota_probe.run()` end to end against a stand-in for the OC — which
is possible only because the module imports bleak lazily; `BleakClient` is
just a module global the test rebinds.  So this runs for real in CI rather
than skipping for a missing BLE stack.
"""

import asyncio
import importlib.util
import json
import struct
import sys
import types
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]


def _load():
    spec = importlib.util.spec_from_file_location(
        "ota_probe", REPO / "tools" / "ota_probe.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


probe_mod = _load()


# --------------------------------------------------------------------------
# Wire framing — these must match BLEDevice.swift byte for byte, because the
# firmware parses them with no version negotiation of any kind.
# --------------------------------------------------------------------------

def test_begin_payload_matches_the_app():
    """[target:1][size:4 LE][sha256:32] — sendOtaBegin's `Data(capacity: 37)`."""
    sha = bytes(range(32))
    payload = bytes([1]) + struct.pack("<I", 648_624) + sha
    assert len(payload) == 37
    assert payload[0] == 1
    assert struct.unpack("<I", payload[1:5])[0] == 648_624
    assert payload[5:] == sha


def test_chunk_frame_matches_the_app():
    """[offset:4 LE][length:2 LE][flags:1][data:N] — sendOtaChunk."""
    body = bytes(range(220))
    frame = struct.pack("<IHB", 220, len(body), 0) + body
    assert len(frame) == 7 + len(body) == probe_mod.CHUNK_HEADER + 220
    assert struct.unpack("<I", frame[0:4])[0] == 220
    assert struct.unpack("<H", frame[4:6])[0] == 220
    assert frame[6] == 0, "flags bit0 is isLast; the probe never sends a last chunk"


def test_identity_constant_tracks_the_receiver():
    """APP_DESC_OFFSET + APP_DESC_SIZE in TR_OTA_Receiver.h.

    Read out of the header rather than restated, because the whole point of
    the probe is that it sends the minimum bytes that force a verdict. If the
    descriptor ever moves, a stale constant here sends too few and the
    operator sees silence that looks like the bug under test.
    """
    hdr = (REPO / "tinkerrocket-idf" / "components" / "TR_OTA" / "include"
           / "TR_OTA_Receiver.h").read_text()
    import re
    off = int(re.search(r"APP_DESC_OFFSET\s*=\s*(\d+)", hdr).group(1))
    size = int(re.search(r"APP_DESC_SIZE\s*=\s*(\d+)", hdr).group(1))
    assert probe_mod.IDENTITY_BYTES == off + size == 288


@pytest.mark.parametrize("chunk,want", [(220, 2), (150, 2), (100, 3), (288, 1), (512, 1)])
def test_chunk_count_completes_the_header(chunk, want):
    need = -(-probe_mod.IDENTITY_BYTES // chunk)
    assert need == want
    assert need * chunk >= probe_mod.IDENTITY_BYTES
    assert (need - 1) * chunk < probe_mod.IDENTITY_BYTES, "sends more than needed"


# --------------------------------------------------------------------------
# The status window
# --------------------------------------------------------------------------

def _frame(**kw):
    return bytearray(json.dumps({"type": "ota_status", **kw}).encode())


def test_a_stale_refusal_is_not_a_verdict():
    """A verdict only counts if it arrived after the chunks that provoked it.

    Matching an earlier frame would report PASS for a refusal that belonged
    to a previous session — a false pass in a tool whose only job is to
    confirm a refusal.
    """
    p = probe_mod.Probe()
    p.on_status(None, _frame(state="verify_failed", bytes=9, err="stale"))
    mark = len(p.statuses)
    assert p.latest("verify_failed", since=mark) is None
    p.on_status(None, _frame(state="verify_failed", bytes=220,
                             err="fc_image_identity_mismatch"))
    got = p.latest("verify_failed", since=mark)
    assert got["err"] == "fc_image_identity_mismatch" and got["bytes"] == 220


def test_other_file_ops_traffic_is_ignored():
    """file_ops carries file listings and pyro refusals too (#132, #1231)."""
    p = probe_mod.Probe()
    p.on_status(None, bytearray(b'{"type":"file_list","files":[]}'))
    p.on_status(None, bytearray(b'not json at all'))
    assert p.statuses == []


# --------------------------------------------------------------------------
# Whole-flow, against a stand-in OC
# --------------------------------------------------------------------------

class FakeOC:
    """Answers BEGIN with `ready`, then refuses on the chunk that COMPLETES
    the identity header, reporting the bytes written before it — which is
    what the receiver does (`bytes_written_ += len` runs after the check).
    """

    def __init__(self, dev, err="fc_image_identity_mismatch", refuse=True):
        self.mtu_size = 527
        self.cb = None
        self.commands: list[int] = []
        self.chunks: list[tuple[int, int]] = []
        self.total = 0
        self.err = err
        self.refuse = refuse
        self.refused = False
        self.begin_target = None      # the byte that picks WHICH processor

    async def __aenter__(self):
        return self

    async def __aexit__(self, *a):
        return False

    async def start_notify(self, uuid, cb):
        self.cb = cb

    async def stop_notify(self, uuid):
        self.cb = None

    def _notify(self, obj):
        self.cb(None, bytearray(json.dumps({"type": "ota_status", **obj}).encode()))

    async def write_gatt_char(self, uuid, data, response=None):
        data = bytes(data)
        if uuid == probe_mod.COMMAND_UUID:
            self.commands.append(data[0])
            if data[0] == probe_mod.CMD_OTA_BEGIN:
                assert len(data) == 38, "cmd byte + 37-byte payload"
                self.begin_target = data[1]
                await asyncio.sleep(0)
                self._notify({"state": "ready", "bytes": 0})
        elif uuid == probe_mod.FILE_TRANSFER_UUID:
            off, ln, flags = struct.unpack("<IHB", data[:7])
            assert off == self.total, "offsets must be contiguous or the FC rejects"
            assert len(data) == 7 + ln
            assert flags == 0
            self.chunks.append((off, ln))
            prev, self.total = self.total, self.total + ln
            if self.refuse and not self.refused and self.total >= probe_mod.IDENTITY_BYTES:
                self.refused = True
                await asyncio.sleep(0)
                self._notify({"state": "verify_failed", "bytes": prev, "err": self.err})


def _args(tmp_path, **over):
    img = tmp_path / "fc.bin"
    img.write_bytes(bytes((i * 7) % 251 for i in range(10_000)))
    base = dict(image=str(img), target="fc", name="TR-R", address=None,
                chunks=None, chunk_size=None, begin_timeout=2.0, listen=1.0,
                expect="fc_image_identity_mismatch")
    base.update(over)
    return types.SimpleNamespace(**base)


def _run(monkeypatch, args, fake_factory):
    holder = {}

    def make(dev):
        holder["oc"] = fake_factory(dev)
        return holder["oc"]

    monkeypatch.setattr(probe_mod, "BleakClient", make)
    monkeypatch.setattr(probe_mod, "BleakScanner", object())   # never reached
    monkeypatch.setattr(probe_mod, "find_device",
                        lambda n: asyncio.sleep(0, result=types.SimpleNamespace(
                            name="TR-R-ab5c", address="FAKE")))
    rc = asyncio.run(probe_mod.run(args))
    return rc, holder["oc"]


def test_refusal_passes_and_never_sends_finish(monkeypatch, tmp_path):
    rc, oc = _run(monkeypatch, _args(tmp_path), FakeOC)
    assert rc == 0
    assert oc.chunks == [(0, 220), (220, 220)], "two chunks settle a 288 B header"
    assert probe_mod.CMD_OTA_ABORT in oc.commands, "session must be closed"
    assert 71 not in oc.commands, "OTA_FINISH must never be sent — it would flash"


def test_a_different_token_is_not_a_pass(monkeypatch, tmp_path):
    rc, oc = _run(monkeypatch, _args(tmp_path),
                  lambda d: FakeOC(d, err="fc_sha_mismatch"))
    assert rc == 1, "--expect must gate the verdict, not just the refusal"
    assert probe_mod.CMD_OTA_ABORT in oc.commands, "abort still runs on failure"


def test_silence_is_not_a_pass(monkeypatch, tmp_path):
    """A correct image is not refused — the probe must say so, not hang."""
    rc, oc = _run(monkeypatch, _args(tmp_path, listen=0.1),
                  lambda d: FakeOC(d, refuse=False))
    assert rc == 1
    assert probe_mod.CMD_OTA_ABORT in oc.commands


def test_chunk_size_is_capped_by_the_link_mtu(monkeypatch, tmp_path):
    """A chunk write has to fit one ATT write or the firmware never sees it."""
    class SmallMTU(FakeOC):
        def __init__(self, dev, **kw):
            super().__init__(dev, **kw)
            self.mtu_size = 100

    rc, oc = _run(monkeypatch, _args(tmp_path), SmallMTU)
    assert rc == 0
    room = 100 - probe_mod.ATT_OVERHEAD - probe_mod.CHUNK_HEADER
    assert all(ln <= room for _, ln in oc.chunks)
    assert sum(ln for _, ln in oc.chunks) >= probe_mod.IDENTITY_BYTES, \
        "still has to reach a verdict on a small MTU"


# --------------------------------------------------------------------------
# #1425: --target picks WHICH PROCESSOR is offered the image.  Everything
# above ran only the `fc` path, and FakeOC checked the BEGIN payload's LENGTH
# but never its first byte -- so a swapped target byte passed the whole suite
# while offering an FC image to the OC.  The probe never sends OTA_FINISH so
# nothing commits either way, but the refusal it reports would be the wrong
# device's, which is precisely the kind of misattribution that made #1425 look
# like a firmware bug for a day.
# --------------------------------------------------------------------------

def test_fc_target_is_byte_one(monkeypatch, tmp_path):
    _, oc = _run(monkeypatch, _args(tmp_path, target="fc"), FakeOC)
    assert oc.begin_target == 1, "target=fc must set the relay byte"


def test_oc_target_is_byte_zero_and_takes_the_unprefixed_token(monkeypatch, tmp_path):
    """Local self-OTA: byte 0, and the firmware's token has no `fc_` prefix.

    The OC emits `image_identity_mismatch` from TR_BLE_To_APP.cpp; only the
    relay path adds `fc_` (out_computer/main.cpp). Expecting the prefixed one
    here would fail against a correctly behaving board.
    """
    rc, oc = _run(
        monkeypatch,
        _args(tmp_path, target="oc", expect="image_identity_mismatch"),
        lambda dev: FakeOC(dev, err="image_identity_mismatch"),
    )
    assert rc == 0
    assert oc.begin_target == 0, "target=oc must NOT set the relay byte"
    assert oc.chunks == [(0, 220), (220, 220)]
    assert 71 not in oc.commands, "OTA_FINISH must never be sent — it would flash"


def test_the_two_targets_do_not_send_the_same_byte(monkeypatch, tmp_path):
    """The regression that matters: a constant, or a dropped conditional."""
    _, fc = _run(monkeypatch, _args(tmp_path, target="fc"), FakeOC)
    _, oc = _run(
        monkeypatch,
        _args(tmp_path, target="oc", expect="image_identity_mismatch"),
        lambda dev: FakeOC(dev, err="image_identity_mismatch"),
    )
    assert fc.begin_target != oc.begin_target
