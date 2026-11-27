"""#1773: a magnetometer that does not come up no longer keeps the FC in setup forever.

The bounded retry (TR_Sensor_Collector/sensor_bringup_retry.h) is host-tested in tests_cpp/test_sensor_bringup_retry.cpp.
The collector is not host-built (ESP-IDF), so these tests read its source and main.cpp's:
- the MMC5983MA is brought up by SensorBringup::retryUntil with the host test's 2 s, a soft reset between tries, and no
  unbounded loop is left in either magnetometer's bring-up;
- a part that does not answer, or will not configure, leaves the boot with no magnetometer: mmc5983maMissing() set,
  use_mmc5983ma cleared (the poll task skips the part), and its interrupt never attached;
- the I2C slot's part that answers but will not configure is treated as one that does not answer;
- main.cpp reports a missing MMC5983MA as degraded sensors, outside any #if.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
SC_DIR = os.path.join(ROOT, "tinkerrocket-idf", "components", "TR_Sensor_Collector")
COLLECTOR = os.path.join(SC_DIR, "TR_Sensor_Collector.cpp")
HEADER = os.path.join(SC_DIR, "TR_Sensor_Collector.h")
MAIN = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")
HOST_TEST = os.path.join(ROOT, "tests_cpp", "test_sensor_bringup_retry.cpp")


def read(path):
    with open(path) as fh:
        return fh.read()


def code(text):
    """The source without comments (string literals kept whole), all whitespace collapsed to one space, none just
    inside a parenthesis."""
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


def mag_section(src):
    """begin()'s two magnetometer bring-ups: the I2C slot's probe through the MMC5983MA's block."""
    a = src.index("if (use_iis2mdc) {")
    b = src.index('ESP_LOGI(SC_TAG, "Initializing ISM6HG256...");', a)
    return src[a:b]


def braced(src, i):
    """The braced block that opens at src[i] ('{'), braces included."""
    depth, k = 0, i
    while True:
        if src[k] == "{":
            depth += 1
        elif src[k] == "}":
            depth -= 1
            if depth == 0:
                return src[i:k + 1]
        k += 1


def mmc_block(src):
    a = src.index("if (use_mmc5983ma && !iis2mdc_active) {")
    b = src.index('ESP_LOGI(SC_TAG, "Initializing ISM6HG256...");', a)
    return src[a:b]


def test_no_unbounded_loop_is_left_in_either_magnetometers_bring_up():
    sec = mag_section(code(read(COLLECTOR)))
    assert "while (1)" not in sec and "while (true)" not in sec and "for (;;)" not in sec
    assert "while (!mmc5983ma.begin())" not in sec
    assert not re.search(r"\bwhile\s*\(", sec), re.findall(r"while\s*\([^)]*\)", sec)


def test_the_mmc_is_brought_up_by_the_bounded_retry():
    blk = mmc_block(code(read(COLLECTOR)))
    assert "constexpr uint32_t kMmcBringupTimeoutMs = 2000U;" in blk
    assert ("const bool mmc_answered = SensorBringup::retryUntil([this] { return mmc5983ma.begin(); }, [this] {"
            in blk)
    retry = blk[blk.index("SensorBringup::retryUntil("):]
    retry = retry[:retry.index("mmc_budget_ms, mmc_attempts);")]
    assert "(void)mmc5983ma.softReset();" in retry and retry.count("delay_ms(500);") == 2
    assert "[] { return (uint32_t)millis(); }," in retry
    m = re.search(r"constexpr uint32_t kTimeoutMs = (\d+);\s*// TR_Sensor_Collector.cpp kMmcBringupTimeoutMs",
                  read(HOST_TEST))
    assert m and m.group(1) == "2000"
    assert '#include "sensor_bringup_retry.h"' in read(COLLECTOR)


def test_a_missing_or_unconfigurable_mmc_leaves_no_magnetometer():
    blk = mmc_block(code(read(COLLECTOR)))
    assert "bool mmc_ok = mmc_answered;" in blk
    # configured only once it answered
    j = blk.index("if (mmc_answered) {")
    cfg = blk[j:blk.index("if (!mmc_ok) {", j)]
    for call in ("setFilterBandwidth(", "setContinuousModeFrequency(", "enableContinuousMode()",
                 "enableInterrupt()"):
        assert ("mmc_ok = mmc_ok && mmc5983ma." + call) in cfg, call
    k = blk.index("if (!mmc_ok) {")
    fail = braced(blk, blk.index("{", k))
    assert "mmc_missing_ = true; use_mmc5983ma = false;" in fail
    # the part's state and its interrupt only for a part brought up: inside the else branch's own braces
    e = k + len("if (!mmc_ok) ") + len(fail)
    assert blk[e:].startswith(" else {"), blk[e:e + 20]
    ok = braced(blk, blk.index("{", e))
    for line in ("mmc5983ma_instance = this;", "mmc_int_wired_ = (MMC5983MA_INT < GPIO_NUM_MAX);",
                 "gpio_isr_handler_add((gpio_num_t)(MMC5983MA_INT), &onMMC5983MAIntTrampoline, nullptr);",
                 "mmc_poll_period_us_ = 500000UL / (uint32_t)mmc_freq;"):
        assert line in ok, line
        assert blk.count(line) == 1, line
    hdr = code(read(HEADER))
    assert "bool mmc5983maMissing() const { return mmc_missing_; }" in hdr
    assert "bool mmc_missing_ = false;" in hdr


def test_the_poll_task_skips_a_part_not_brought_up():
    src = code(read(COLLECTOR))
    assert "if (self->use_mmc5983ma && !self->iis2mdc_active) {" in src
    hdr = code(read(HEADER))
    assert re.search(r"\bbool use_mmc5983ma;", hdr), "not const: begin() clears it"


def test_the_i2c_slots_part_that_will_not_configure_is_not_used():
    src = code(read(COLLECTOR))
    assert ("else if (iis2mdc.begin(iis2mdc_bus, IIS2MDC_I2C_FREQ_HZ) == MAG_DRIVER_OK && configureI2cMag()) {"
            in src)
    i = src.index("bool SensorCollector::configureI2cMag() {")
    body = braced(src, src.index("{", i))
    assert "constexpr uint32_t kI2cMagConfigTimeoutMs = 100U;" in body
    assert ("if (SensorBringup::retryUntil([this] { return iis2mdc.configure() == MAG_DRIVER_OK; }, "
            "[] { delay_ms(20); }, [] { return (uint32_t)millis(); }, "
            "mag_one_try_ ? 0U : kI2cMagConfigTimeoutMs, tries)) return true;") in body
    assert body.rstrip("} ").endswith("return false;")


def test_a_flight_in_progress_gives_the_magnetometer_one_try():
    src = code(read(COLLECTOR))
    blk = mmc_block(src)
    assert "const uint32_t mmc_budget_ms = mag_one_try_ ? 0U : kMmcBringupTimeoutMs;" in blk
    assert "[] { return (uint32_t)millis(); }, mmc_budget_ms, mmc_attempts);" in blk
    assert "void magOneTryThisBoot() { mag_one_try_ = true; }" in code(read(HEADER))
    main = code(read(MAIN))
    i = main.index("if (raw_boot_ask.valid) {")
    assert main.index("sensor_collector_hw.magOneTryThisBoot();", i) < main.index("}", i)
    assert main.count("magOneTryThisBoot()") == 1
    assert main.index("sensor_collector_hw.magOneTryThisBoot();") < main.index("sensor_collector.begin(config::SENSOR_CORE);")


def test_the_mag_status_reads_bad_and_the_diagnostics_go_quiet_when_it_is_missing():
    src = code(read(MAIN))
    assert ("if (mag_st == SH_NA && config::USE_MMC5983MA && sensor_collector_hw.mmc5983maMissing()) "
            "{ mag_st = SH_BAD; }") in src
    assert src.index("mag_st = SH_BAD;") < src.index("sh = shSet(sh, SH_MAG_SHIFT, mag_st);")
    assert ("if (!sensor_collector.isIIS2MDCActive() && !sensor_collector_hw.mmc5983maMissing()) "
            "{ static MMC5983MADebugSnapshot prev_mmc_snap = {};") in src


def test_main_reports_a_missing_mmc_as_degraded_sensors():
    raw = read(MAIN)
    src = code(raw)
    want = "if (config::USE_MMC5983MA && sensor_collector_hw.mmc5983maMissing()) {"
    assert src.count(want) == 1
    i = src.index(want)
    assert "fcBootStatus(FCB_SENSORS, FCB_DEG_SENSORS);" in src[i:src.index("}", i)]
    pos = raw.index("sensor_collector_hw.mmc5983maMissing()")
    stack = []
    for line in raw[:pos].splitlines():
        t = line.strip()
        if re.match(r"#\s*if", t):
            stack.append(t)
        elif re.match(r"#\s*endif", t):
            stack.pop()
    assert stack == [], stack
