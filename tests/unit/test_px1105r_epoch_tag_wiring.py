"""#1771: the PX1105R driver and the offline replay date each 0xE5's time-sync sample at its frame's first byte.

The helpers (Px1105rRawEpoch.h arrivalUs, frameStartUs) and the tags they give through a simulated wire, the real
SkyTraq parser and GnssTimeSync are host-tested in tests_cpp/test_px1105r_epoch_tag.cpp. The driver is not host-built,
so these tests read its source and the replay's:
- both of the driver's time-sync samples (rawE5, and onFrame's with the raw outputs off) are at
  frameStartUs(end_us, len, byte_us_), and no sample is at a frame's last byte;
- the driver's arrivalUs is the pure helper the host test runs;
- the replay's sample is at frameStartUs(end_us, len, cfg_.byte_us), its byte time the driver's target baud's, but 0
  (the stamp as given) on a px_events.py file or with --px-stamp last, and it hands the feed each R line's own stamp;
- the replay's time sync takes no latency of its own unless asked, as the driver's (setEpochLatencyS has no caller:
  the filter takes the profile's epoch_latency_s off);
- the host test's byte time is the driver's target baud's.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
DRV_DIR = os.path.join(ROOT, "tinkerrocket-idf", "components", "TR_GNSSReceiverPX1105R_Serial")
DRIVER = os.path.join(DRV_DIR, "TR_GNSSReceiverPX1105R_Serial.cpp")
FEED = os.path.join(ROOT, "tools", "raw_replay", "skytraq_feed.h")
REPLAY = os.path.join(ROOT, "tools", "raw_replay", "raw_replay.cpp")
FW_DIRS = [os.path.join(ROOT, "tinkerrocket-idf", "components"), os.path.join(ROOT, "tinkerrocket-idf", "projects")]
HOST_TEST = os.path.join(ROOT, "tests_cpp", "test_px1105r_epoch_tag.cpp")


def read(path):
    with open(path) as fh:
        return fh.read()


def code(text):
    """The source without comments (string literals kept whole), each whitespace run collapsed to one space, none
    just inside a parenthesis."""
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if text.startswith("//", i):
            i = text.find("\n", i)
            i = n if i < 0 else i
        elif text.startswith("/*", i):
            i = text.index("*/", i + 2) + 2
            out.append(" ")
        elif c in "\"'":
            j = i + 1
            while text[j] != c:
                j += 2 if text[j] == "\\" else 1
            out.append(text[i:j + 1])
            i = j + 1
        else:
            out.append(c)
            i += 1
    text = re.sub(r"\s+", " ", "".join(out))
    return re.sub(r"\s+\)", ")", re.sub(r"\(\s+", "(", text))


def function_body(src, signature):
    i = src.index(signature)
    j = src.index("{", i)
    depth, k = 0, j
    while True:
        if src[k] == "{":
            depth += 1
        elif src[k] == "}":
            depth -= 1
            if depth == 0:
                return src[j + 1:k]
        k += 1


def target_baud():
    m = re.search(r"kTargetBaud\s*=\s*(\d+)U?;", read(DRIVER))
    assert m
    return int(m.group(1))


def test_both_driver_samples_are_at_the_frames_first_byte():
    src = code(read(DRIVER))
    samples = re.findall(r"\baddSample\(([^;]*)\);", src)
    assert len(samples) == 2, samples
    for s in samples:
        assert s.endswith("px1105r::frameStartUs(end_us, len, byte_us_)"), s
    raw = function_body(src, "void TR_GNSSReceiverPX1105RSerial::rawE5(const uint8_t* body, size_t len, int64_t end_us)")
    assert "if (biased) time_sync_.addSample(gps_s, px1105r::frameStartUs(end_us, len, byte_us_));" in raw
    frame = function_body(src, "void TR_GNSSReceiverPX1105RSerial::onFrame(const uint8_t* body, size_t len, size_t index)")
    assert ("time_sync_.addSample(px1105r::epochGpsS(week, tow_ms, dtr), px1105r::frameStartUs(end_us, len, byte_us_));"
            in frame)
    # end_us is the frame's last byte there, and the frame's own body length is what is handed on
    assert "const int64_t end_us = arrivalUs(index);" in frame
    assert "if (raw_ != nullptr) rawE5(body, len, end_us);" in frame


def test_the_driver_dates_bytes_with_the_tested_helper():
    src = code(read(DRIVER))
    body = function_body(src, "int64_t TR_GNSSReceiverPX1105RSerial::arrivalUs(size_t index) const")
    assert body.strip() == "return px1105r::arrivalUs(rd_us_, rd_n_, index, rd_behind_, byte_us_);"
    begin = function_body(src, "bool TR_GNSSReceiverPX1105RSerial::uartBegin(uint32_t baud)")
    assert begin.strip().startswith("byte_us_ = 1.0e7 / static_cast<double>(baud);")
    assert src.count("byte_us_ =") == 1


def test_the_replay_samples_at_the_first_byte_at_the_drivers_baud():
    src = code(read(FEED))
    samples = re.findall(r"\baddSample\(([^;]*)\);", src)
    assert samples == ["gps_s, px1105r::frameStartUs(end_us, len, cfg_.byte_us)"], samples
    m = re.search(r"double byte_us = 1\.0e7 / (\d+)\.0;", src)
    assert m and int(m.group(1)) == target_baud()


def test_the_host_tests_byte_time_is_the_drivers():
    m = re.search(r"constexpr double kByteUs = 1\.0e7 / (\d+)\.0;", read(HOST_TEST))
    assert m and int(m.group(1)) == target_baud()


def test_the_replay_keeps_px_events_stamps_and_offers_the_old_stamp():
    src = code(read(REPLAY))
    assert 'const bool px_events_file = in.meta.count("px_latency_ms") != 0;' in src
    assert 'if (px_events_file || a.px_stamp == "last") pcfg.byte_us = 0.0;' in src
    assert src.count("pcfg.byte_us") == 1
    assert 'else if (k == "--px-stamp") a.px_stamp = val();' in src
    assert 'std::string px_stamp = "first";' in src
    # each body handed the R line's own stamp (the frame's last byte on an FC log)
    assert "pxfeed.onBody(body.data(), body.size(), ev.t_us, te);" in src


def test_no_latency_is_taken_off_twice():
    src = code(read(REPLAY))
    assert "pcfg.epoch_latency_s = a.px_latency_ms >= 0.0 ? a.px_latency_ms * 1.0e-3 : 0.0;" in src
    assert "rx.epoch_latency_s;" not in src.split("pcfg.epoch_latency_s =")[1].split(";")[0] + ";"
    # the driver's own: setEpochLatencyS defined and declared, called nowhere in the firmware
    calls = 0
    for d in FW_DIRS:
        for root, _, files in os.walk(d):
            if "/build" in root:
                continue
            for f in files:
                if f.endswith((".cpp", ".h", ".c")):
                    with open(os.path.join(root, f), errors="replace") as fh:
                        for line in fh:
                            if not line.lstrip().startswith("//"):
                                calls += len(re.findall(r"\bsetEpochLatencyS\(", line))
    assert calls == 2, calls   # the declaration and the definition
