"""#1772: on the ESP32-S3 the PX1105R driver keeps its large blocks out of internal RAM, and the floor counts real stacks.

The Beetle V2's floor (config.h) is host-tested in tests_cpp/test_beetle_v2_raw_floor.cpp. The driver is not
host-built (ESP-IDF), so these tests read its source:
- TR_PX1105R_PSRAM_FIRST comes from the S3 target alone;
- on it the flight log's ring, the three raw queues, the raw state and the Doppler pairs are made in PSRAM first,
  internal RAM otherwise; off it, as before (the ring a static member, the rest internal);
- every raw queue is deleted by the call that matches the one that made it;
- main.cpp creates the raw filter's task and the flight task with the stacks the check counts, and on the Beetle V2
  (RAW_INTERNAL_PROBE) starts the filter only with room for the flight task beside it, probed with blocks the size of
  the filter's stacks that it gives back.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
DRV_DIR = os.path.join(ROOT, "tinkerrocket-idf", "components", "TR_GNSSReceiverPX1105R_Serial")
DRIVER = os.path.join(DRV_DIR, "TR_GNSSReceiverPX1105R_Serial.cpp")
HEADER = os.path.join(DRV_DIR, "TR_GNSSReceiverPX1105R_Serial.h")
MAIN = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")


def read(path):
    with open(path) as fh:
        return fh.read()


def code(text):
    """The source without comments (string literals kept whole), each whitespace run collapsed to one space, none
    just inside a parenthesis. Preprocessor lines are kept, each on its own line."""
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
    lines = []
    for line in "".join(out).split("\n"):
        s = line.strip()
        lines.append("\n" + s + "\n" if s.startswith("#") else " " + s)
    text = re.sub(r"[ \t]+", " ", "".join(lines))
    return re.sub(r"\s+\)", ")", re.sub(r"\(\s+", "(", text))


def flat(s):
    return re.sub(r"\s+", " ", s).strip()


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


def test_psram_first_is_the_s3_alone():
    raw = read(HEADER)
    assert re.search(r"#if defined\(CONFIG_IDF_TARGET_ESP32S3\) && CONFIG_IDF_TARGET_ESP32S3\s*\n"
                     r"#define TR_PX1105R_PSRAM_FIRST 1\s*\n#else\s*\n#define TR_PX1105R_PSRAM_FIRST 0\s*\n#endif", raw)
    assert raw.count("#define TR_PX1105R_PSRAM_FIRST") == 2
    assert "#include <sdkconfig.h>" in raw


def test_the_ring_is_psram_first_on_the_s3_and_static_otherwise():
    hdr = code(read(HEADER))
    i = hdr.index("alignas(8) uint8_t ring_mem_[kRingBytes] = {};")
    assert hdr.rindex("#if !TR_PX1105R_PSRAM_FIRST", 0, i) > hdr.rindex("#endif", 0, i)
    src = code(read(DRIVER))
    begin = src[src.index("if (ring_ == nullptr) {"):]
    begin = begin[:begin.index("if (clock_q_ == nullptr)")]
    psram, other = begin.split("#else", 1)
    assert "#if TR_PX1105R_PSRAM_FIRST" in psram
    assert ("ring_ = xRingbufferCreateWithCaps(kRingBytes, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);"
            in psram)
    assert psram.index("xRingbufferCreateWithCaps") < psram.index("if (ring_ == nullptr) ring_ = xRingbufferCreate(")
    assert "xRingbufferCreateStatic(kRingBytes, RINGBUF_TYPE_NOSPLIT, ring_mem_, &ring_struct_);" in other
    assert len(re.findall(r"xRingbufferCreate\w*\(", src)) == 3


def test_the_raw_blocks_and_queues_are_psram_first():
    src = code(read(DRIVER))
    alloc = function_body(src, "static void* rawAlloc(size_t n)")
    assert flat(alloc) == ("#if TR_PX1105R_PSRAM_FIRST void* m = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | "
                           "MALLOC_CAP_8BIT); if (m != nullptr) return m; #endif return heap_caps_malloc(n, "
                           "MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);"), flat(alloc)
    create = function_body(src, "static QueueHandle_t rawQueueCreate(UBaseType_t depth, UBaseType_t item, bool& caps)")
    assert flat(create) == ("#if TR_PX1105R_PSRAM_FIRST QueueHandle_t q = xQueueCreateWithCaps(depth, item, "
                            "MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); if (q != nullptr) { caps = true; return q; } "
                            "#endif caps = false; return xQueueCreate(depth, item);"), flat(create)
    raw = function_body(src, "bool TR_GNSSReceiverPX1105RSerial::enableRawOutputs()")
    assert "void* mem = rawAlloc(sizeof(RawState));" in raw
    for k, (q, depth, item) in enumerate((("epoch_q", "kRawEpochQDepth", "sizeof(gnss_raw::TimedRawEpoch)"),
                                          ("eph_q", "kEphQDepth", "sizeof(gps_eph_t)"),
                                          ("fix_q", "kFixQDepth", "sizeof(gnss_raw::TimedFix)"))):
        assert f"r->{q} = rawQueueCreate({depth}, {item}, r->q_caps[{k}]);" in raw
    pairs = function_body(src, "bool TR_GNSSReceiverPX1105RSerial::setRawDopplerPairing(")
    assert "void* mem = rawAlloc(sizeof(RawState::Pairing));" in pairs
    # nothing of the raw outputs straight into internal RAM any more
    assert "MALLOC_CAP_INTERNAL" not in raw and "MALLOC_CAP_INTERNAL" not in pairs
    assert "xQueueCreate(" not in raw


def test_each_queue_is_deleted_by_its_own_call():
    src = code(read(DRIVER))
    delete = function_body(src, "static void rawQueueDelete(QueueHandle_t q, bool caps)")
    assert flat(delete) == ("if (q == nullptr) return; #if TR_PX1105R_PSRAM_FIRST if (caps) { vQueueDeleteWithCaps(q); "
                            "return; } #endif (void)caps; vQueueDelete(q);"), flat(delete)
    raw = function_body(src, "bool TR_GNSSReceiverPX1105RSerial::enableRawOutputs()")
    assert "vQueueDelete(" not in raw
    for k, q in enumerate(("epoch_q", "eph_q", "fix_q")):
        assert raw.count(f"rawQueueDelete(r->{q}, r->q_caps[{k}]);") == 2   # the queues' failure, the subframes'


def test_the_tasks_have_the_stacks_the_floor_counts():
    src = code(read(MAIN))
    assert 'xTaskCreatePinnedToCore(rawShadowTask, "Raw GNSS", config::RAW_FILTER_STACK_B,' in src
    assert '"flight", config::FLIGHT_TASK_STACK_B, nullptr, configMAX_PRIORITIES - 1, &flight_task, 1)' in src
    assert 'config::RAW_MEAS_STACK_B, nullptr, 0, &raw_meas_task,' in src


def test_the_beetle_v2_starts_the_filter_only_with_room_for_the_flight_task():
    src = flat(code(read(MAIN)))
    i = src.index("if (config::RAW_INTERNAL_PROBE) {")
    j = src.index("else if (largest < config::RAW_INTERNAL_FLOOR_B) {", i)
    probe = src[i:j]
    steps = ["void* s1 = heap_caps_malloc(config::RAW_FILTER_STACK_B, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);",
             "void* s2 = heap_caps_malloc(config::RAW_MEAS_STACK_B, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);",
             "const size_t largest_after = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);",
             "const size_t free_after = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);",
             "heap_caps_free(s2);", "heap_caps_free(s1);",
             "const bool room = RawNavPolicy::roomForFlightTask(s1 != nullptr && s2 != nullptr, largest_after, "
             "free_after, config::FLIGHT_TASK_STACK_B, config::RAW_INTERNAL_SLACK_B, config::RAW_INTERNAL_MARGIN_B);",
             "if (!room) {"]
    at = [probe.index(s) for s in steps]
    assert at == sorted(at), at
    refuse = probe[probe.index("if (!room) {"):]
    assert "rawNavGiveBack(mem, stage_mem, built_mem, epoch_mem); return;" in refuse
    # before the filter's tasks are made
    assert j < src.index('xTaskCreatePinnedToCore(rawShadowTask, "Raw GNSS",', j)
