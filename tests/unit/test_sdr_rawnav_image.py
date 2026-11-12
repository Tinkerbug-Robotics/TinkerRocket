"""The V10's gnss-sdr-dev image (-DTR_BOARD_V10=1 -DTR_GNSS_SDR=1): the build switches that make it, end to end.

Its one navigation filter is the raw-GNSS filter on the board's 100 Hz MSM7, as the mosaic-only image's is on the
mosaic-G5. The pieces live in several files that each also serve the other images: the flight computer's CMakeLists
(the driver's define, the overlays, the ephemeris store, the auto image's default, the bench and rig-replay rules),
main/CMakeLists (the PSRAM check), config.h (the paths' macros), main.cpp (startRawShadow's profile, the Doppler
means and their lag, the wind-up) and tools/raw_replay (--receiver sdr flying the image's code). A union that drops a
side builds an image that silently is not this one; these tests read the sources and check that each piece is there.
"""
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
FC = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer")
FC_CMAKE = os.path.join(FC, "CMakeLists.txt")
MAIN_CMAKE = os.path.join(FC, "main", "CMakeLists.txt")
CONFIG_H = os.path.join(FC, "main", "config.h")
MAIN_CPP = os.path.join(FC, "main", "main.cpp")
RR_CPP = os.path.join(ROOT, "tools", "raw_replay", "raw_replay.cpp")
RR_CMAKE = os.path.join(ROOT, "tools", "raw_replay", "CMakeLists.txt")


def read(path):
    with open(path) as fh:
        return fh.read()


def cmake_if_block(src, cond):
    """The body of the first 'if(cond)' at the start of a line, to its matching endif()."""
    m = re.search(r"^if\(" + re.escape(cond) + r"\)\n", src, re.M)
    assert m, "no if(%s)" % cond
    depth, pos = 0, m.start()
    for line in src[m.start():].splitlines(keepends=True):
        s = line.strip()
        if re.match(r"if\(", s):
            depth += 1
        elif s.startswith("endif("):
            depth -= 1
            if depth == 0:
                return src[m.start():pos + len(line)]
        pos += len(line)
    raise AssertionError("no endif for if(%s)" % cond)


def test_the_image_selects_the_board_alone_with_psram_and_128_ephemerides():
    block = cmake_if_block(read(FC_CMAKE), "TR_GNSS_SDR")
    assert "add_compile_definitions(TR_GNSS_DRIVER_SDR=1)" in block
    assert 'list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.defaults.mosaic" "sdkconfig.defaults.rawnav")' in block
    assert "add_compile_definitions(TR_EPH_CAPACITY=128)" in block
    # the V10 only, one image at a time, the raw filter or nothing
    assert "if(NOT TR_BOARD_V10)" in block
    assert "if(TR_GNSS_AUTO OR TR_GNSS_MOSAIC)" in block
    assert "NOT TR_RAW_GNSS_SHADOW" in block and "NOT TR_RAW_GNSS_PRIMARY" in block


def test_the_auto_image_is_not_turned_on_beside_it():
    src = read(FC_CMAKE)
    assert ("if(NOT DEFINED TR_GNSS_AUTO AND (TR_BOARD_V9 OR TR_BOARD_V10) AND NOT TR_GNSS_MOSAIC AND NOT TR_GNSS_SDR)"
            in src)
    # and the SDR block comes before the auto default, so its define is seen first
    assert src.index("if(TR_GNSS_SDR)\n") < src.index("if(NOT DEFINED TR_GNSS_AUTO AND")


def test_its_bench_switches_and_rig_replay_are_allowed():
    src = read(FC_CMAKE)
    m = re.search(r"if\(TR_GNSS_MOSAIC AND \(NOT DEFINED TR_RAW_GNSS_SHADOW OR TR_RAW_GNSS_SHADOW\)\)\n"
                  r"    set\(_tr_mosaic_rawnav ON\)\nelseif\(TR_GNSS_SDR\)\n    set\(_tr_mosaic_rawnav ON\)", src)
    assert m, "the raw-nav bench switches are not allowed on the gnss-sdr-dev image"
    replay = cmake_if_block(src, "TR_SIM_REPLAY_BLOB")
    assert "NOT TR_GNSS_SDR" in replay
    sdr = replay[replay.index("    if(TR_GNSS_SDR)"):]
    assert "add_compile_definitions(TR_RAW_NAV_RIG_REPLAY=1)" in sdr.split("    endif()")[0]


def test_the_psram_check_knows_it():
    src = read(MAIN_CMAKE)
    assert "if((TR_GNSS_MOSAIC AND (NOT DEFINED TR_RAW_GNSS_SHADOW OR TR_RAW_GNSS_SHADOW)) OR TR_GNSS_SDR)" in src


def test_config_h_defines_its_paths_and_keeps_the_auto_image_off_the_raw_filter():
    src = read(CONFIG_H)
    assert re.search(r"#if defined\(TR_GNSS_DRIVER_SDR\) && TR_GNSS_DRIVER_SDR && TR_RAW_GNSS_PRIMARY\n"
                     r"#define TR_RAW_NAV_SDR 1\n#endif", src)
    assert re.search(r"#if \(defined\(TR_RAW_NAV_MOSAIC\) && TR_RAW_NAV_MOSAIC\) \|\| "
                     r"\(defined\(TR_RAW_NAV_SDR\) && TR_RAW_NAV_SDR\)\n#define TR_RAW_NAV_P4 1\n#endif", src)
    assert "#if TR_RAW_GNSS_SHADOW && defined(TR_GNSS_DRIVER_AUTO) && TR_GNSS_DRIVER_AUTO\n#error" in src
    # the bands combined stay the mosaic's
    assert "#define TR_RAW_MULTIBAND TR_RAW_NAV_MOSAIC" in src


def start_raw_shadow(src):
    i = src.index("static void startRawShadow()\n{")
    return src[i:src.index("\n}\n", i)]


def test_start_raw_shadow_flies_the_boards_profile_and_its_doppler_means():
    body = start_raw_shadow(read(MAIN_CPP))
    assert "gnss_raw::sdrDevProfile()" in body
    assert "gnss_raw::applySdrFlightProfile(rx);" in body
    # the driver is told the grid and the means from the profile before the filter is told where the means fall
    cfg = body.index("sensor_collector.setGNSSRawEpochConfig(")
    apply_profile = body.index("gnss_raw::applyReceiverProfile(rx, cfg);")
    means = body.index("gnss_raw::applySdrDopplerMeans(rx, cfg);")
    assert cfg < apply_profile < means
    # the safeguards are the image's own (the mosaic's, no wind-up taken off, the change check standing down in
    # the IMU-free descent: ReceiverProfileApply.h), under TR_RAW_NAV_SDR, the mosaic's call kept for the mosaic
    i = body.index("#if TR_RAW_NAV_SDR\n    // The gnss-sdr-dev image's (ReceiverProfileApply.h")
    block = body[i:body.index("#endif", i)]
    assert "gnss_raw::applySdrFlightSafeguards(cfg);" in block
    assert "#else\n    gnss_raw::applyMosaicFlightSafeguards(cfg);\n" in block
    apply = read(os.path.join(ROOT, "tinkerrocket-idf", "components", "TR_RawGnssShadow", "ReceiverProfileApply.h"))
    fn = apply[apply.index("inline void applySdrFlightSafeguards"):]
    fn = fn[:fn.index("\n}\n")]
    assert "applyMosaicFlightSafeguards(c);" in fn and "c.windup_k = 0.0;" in fn and "c.dv_imu_free_off = true;" in fn


def test_no_ionosphere_on_the_image():
    src = read(MAIN_CPP)
    task = src[src.index("static void rawGnssMeasTask(void*)"):]
    task = task[:task.index("\n}\n")]
    sdr = task[task.index("#elif TR_RAW_NAV_SDR"):task.index("#else", task.index("#elif TR_RAW_NAV_SDR"))]
    assert "setIono" not in sdr and "rawIonoOffer" not in sdr


def test_raw_replay_flies_the_images_code():
    src = read(RR_CPP)
    assert "applySdrFlightProfile(rx);" in src
    assert "applySdrDopplerMeans(rx, cfg);" in src
    assert "sdr_rtcm::lineSampleOf(" in src
    assert "sdr_rtcm::timedEpochOf(" in src
    assert "sdr_acfg.mean.n = static_cast<uint32_t>(sdrDopplerMeanEpochs(rx));" in src
    cm = read(RR_CMAKE)
    assert "TR_GNSSReceiverSdr_Serial/SdrAid.cpp" in cm


def fn_body(src, head):
    i = src.index(head)
    return src[i:src.index("\n}\n", i)]


def test_no_attitude_to_the_board_while_the_filter_is_not_running():
    # RawNavAdapter hands on the start's attitude, frozen, before the filter runs or while a stop is held over:
    # the image sends no attitude and no rests then (the sender's "no attitude" follows, sdr_aid_policy.h)
    body = fn_body(read(MAIN_CPP), "static void sdrAidNoteUpdate(const EkfIMUData& imu)")
    i = body.index("#if TR_RAW_NAV_SDR")
    guard = body[i:body.index("#endif", i)]
    assert "if (!ekf.running())\n        return;" in guard
    # before anything is noted for the sender
    assert i < body.index("sdr_aid_fc::widenUs(")
