"""SkyTraq frames in the raw-frame record (GNSS_RTCM_MSG, 0x95): the V9 PX1105R image's.

The FC logs each frame whole (A0 A1, the body's length, the body, its XOR,
0D 0A) in the record the LC86G fills with RTCM3 and the mosaic with SBF. The
bodies here are tests_cpp/fixtures/skytraq_frames.h's: the PX1105R's own 0xE5
and the 0xDF after it, from its 09-26 open-sky capture, with the position made
up (that header says how). Records are cut into GNSS_RTCM_MAX_CHUNK parts as
the FC cuts them.
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

import plot_flight_data_mini as pfd          # noqa: E402

CHUNK = 219                                   # GNSS_RTCM_MAX_CHUNK
E5 = bytes.fromhex("e5015e098521ef081400320000230017c02e4173b7fc0c993e5cc137b941b8999db44510400000000000070000000fc02d417451871d2ee12a41244107cedab17ec4d52000000000000700000012c02f417403642fbbd9eb41106576b170aedec4988000000000000700000018c02f41734b4557ab8a9241215381c6e60afb443e400000000000070000050b501a417719b58bc8583cc11a4514403b92ab44b20000000000001700000520c02f417540a0472c7e91c134407d9738287c44e98000000000000700000529c0254176611691c206c64131550d51cc1fe3c45ec00000000000070000400a001d41755f62bbdf4612c10a5d4be8f4ebdc45314000000000001f00000020c0294176717441408f30c135134a158a48a645679000000000000700004017c0354173488b89b9252ac131d271c4eb0c5844dc0000000000000700004018c0294173adc27567d1b8c1265f466f5aa32c44530000000000000700004012c034417478a9023b8fd64151b896a29800d8c499c00000000000070000051ec02b417610d04dba65e2c11aff44f2962178c345000000000000070000051980234176dc410c6ecded41617e950242e231448c800000000000070000051b60244177a5eb7e998db0413a3846394efd7ec51bb000000000000700000517302a4177e4104d119b2740ed4bec9fb745c0454c7000000000000700004020c02841766beb100d736bc11098411b0d51984530b000000000000700004306c03041769f642095a0b84112b7a5fab0d124c1300000000000000700004304c0304177338997b13bca412da0ecbeffc3d1c474800000000000070000430bc02c4174ffd1a1d1b46c4105baec769ab3d6c461000000000000070000430ac0324178a0e2913e0768411b98a6e5113f29c467000000000000070000431cc02c41796464ef530193413377cfa710b7fac44a8000000000000700004529c02941763fdd348b88d0413940265f9d22ccc3b3800000000000070000451ec02c417697fe98c2b78a4138f4ed5b516c4ec2d8000000000000070000451b902241779c0513c46bfa4133ab3a421b9099c4de8000000000000700004517c02e41778488009e3f50c13164aed90d5f68451e2000000000000700004520c030417596fca20b5a7bc11bb555cec82d4a44758000000000000700004324c02f41772b5461ee0e6e414931ee94036eaf44e4c000000000000700004519101a4176db59c358359f416054d86622e71a44a24000000000001700000304c02c41772282b854539541259b8ed62019ccc410800000000000070000030ac02c417904cbfc71ca2c4122ac7801b099dcc4b32000000000000700000306c02b4176a3456bec3718c1230b8ad26eee1542b8000000000000070000030bc0294174ff9a9c51dcbf40ee89e684cbb2d0c3ee0000000000000700000324c0294176e4616711edc841519206269f9e2c44e8e00000000000070000031cc02741794f1c0b125d8740fa550e81436e18c4a0800000000000070000")
DF = bytes.fromhex("df5e03098541215fc299724b9b415027aadcc25e36c14b1c795a2a970b4148302f5e210d0f3c3d99bb3bfb6ea53b11f4ba40f5f2b018ec9c5ac3391a063f983e423f851e673f307f813f4753753f13bf89")


def frame(body: bytes) -> bytes:
    cs = 0
    for b in body:
        cs ^= b
    return b"\xa0\xa1" + struct.pack(">H", len(body)) + body + bytes([cs]) + b"\r\n"


def records(frames_with_t):
    """The FC's GNSS_RTCM records for these frames, as decode_gnss_rtcm returns them."""
    out = []
    for t, f in frames_with_t:
        parts = (len(f) + CHUNK - 1) // CHUNK
        for k in range(parts):
            out.append({"time_us": t, "part": k, "parts": parts, "data": f[k * CHUNK:(k + 1) * CHUNK]})
    return out


def with_tow(body: bytes, tow_ms: int) -> bytes:
    b = bytearray(body)
    struct.pack_into(">I", b, 5, tow_ms)
    return bytes(b)


def test_skytraq_frames_split_apart_from_rtcm3_and_sbf():
    fs = [(1000, frame(E5)), (1005, frame(DF))]
    joined, dropped = pfd.join_gnss_rtcm(records(fs))
    assert dropped == 0
    raw = pfd.split_gnss_raw_frames(joined)
    assert raw["bad"] == 0 and not raw["rtcm3"] and not raw["sbf"]
    assert [f for _, f in raw["skytraq"]] == [frame(E5), frame(DF)]
    assert pfd.skytraq_frame_body(frame(E5)) == E5


def test_a_corrupt_skytraq_frame_is_bad():
    f = bytearray(frame(DF))
    f[10] ^= 0x01
    raw = pfd.split_gnss_raw_frames([(0, bytes(f))])
    assert raw["bad"] == 1 and not raw["skytraq"]


def test_the_e5_rate_from_the_receivers_own_stamps():
    tow0 = struct.unpack_from(">I", E5, 5)[0]
    fs = [(1000 + 50_000 * k, frame(with_tow(E5, tow0 + 50 * k))) for k in range(41)]
    epochs, span_s, rate_hz = pfd.skytraq_e5_rate(fs)
    assert epochs == 41
    assert abs(span_s - 2.0) < 1e-9
    assert abs(rate_hz - 20.0) < 1e-9


def test_the_summary_names_the_messages(capsys):
    pfd.print_gnss_skytraq_summary(records([(1000, frame(E5)), (1005, frame(DF))]))
    out = capsys.readouterr().out
    assert "SkyTraq, PX1105R" in out
    assert "ext raw meas (E5)" in out and "nav state (DF)" in out
