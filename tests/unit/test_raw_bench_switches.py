"""The V9 mosaic image's bench switches for more GNSS data (the bench integration, exp/v9-moredata-bench), where they
meet in one firmware and one harness: the cheaper epoch build, the epoch's shared geometry, the Dopplers in pairs and
the satellite cap's governor. Each was merged from its own branch into lists and blocks the others also touch: the
flight computer's CMakeLists (the mosaic-only refusal and the compile definitions), main.cpp's startRawShadow (one
#ifdef block a switch, each setting its own profile field), and tools/raw_replay (its option, its compile definition,
its command-line flag setting the same profile field, its block in the events JSON). A union that dropped a side, or
a block that swallowed its neighbour, builds and runs with that switch silently off; these tests read the sources
and check that each switch keeps its own path end to end, and that none sets another's field."""
import os
import re

import pytest

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
FC_CMAKE = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "CMakeLists.txt")
MAIN_CPP = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer", "main", "main.cpp")
RR_CMAKE = os.path.join(ROOT, "tools", "raw_replay", "CMakeLists.txt")
RR_CPP = os.path.join(ROOT, "tools", "raw_replay", "raw_replay.cpp")

# define: (the profile fields its block sets, the harness's flag, the harness's build option, the events JSON key)
SWITCHES = {
    "TR_BENCH_RAW_BUILD_LITE": (("build_lite",), "--build-lite", "RAW_REPLAY_BUILD_LITE", '\\"build_lite\\"'),
    "TR_BENCH_RAW_SHARE_LOS": (("share_los", "rr_float"), "--share-los", "RAW_REPLAY_SHARELOS", '\\"share_los\\"'),
    "TR_BENCH_RAW_DOP_AVG": (("rr_pair_avg",), "--rr-pair-avg", "RAW_REPLAY_DOPAVG", '\\"pairs\\"'),
    "TR_BENCH_RAW_CAP_MAX": (("sat_cap_max",), "--cap-max", "RAW_REPLAY_CAPGOV", '\\"cap_gov\\"'),
    "TR_BENCH_RAW_SLICE_US": (("epoch_slice_us",), "--slice-us", "RAW_REPLAY_CAPGOV", None),
}
# defines that set no profile field of their own (TR_BENCH_RAW_CAP_PAD: the PRELAUNCH hold, main.cpp's ticks)
OTHER = ("TR_BENCH_RAW_CAP_PAD",)


def read(path):
    with open(path) as fh:
        return fh.read()


def foreach_lists(cmake):
    """The flight computer's two foreach lists of bench defines: the mosaic-only refusal's and the definitions'."""
    out = []
    for m in re.finditer(r"foreach\(_v ([^)]*)\)\s*\n(.*?)endforeach\(\)", cmake, re.S):
        names = set(m.group(1).split())
        body = m.group(2)
        if "TR_BENCH_RAW_FUSE_MS" in names:
            out.append((names, body))
    return out


def ifdef_block(src, define, start=0):
    """The text between '#ifdef define' and its own '#endif' (nested #if counted), from start."""
    i = src.index("#ifdef " + define + "\n", start)
    depth = 0
    pos = i
    for line in src[i:].splitlines(keepends=True):
        s = line.strip()
        if s.startswith("#if"):
            depth += 1
        elif s.startswith("#endif"):
            depth -= 1
            if depth == 0:
                return src[i:pos + len(line)]
        pos += len(line)
    raise AssertionError("no #endif for " + define)


def if_block(src, cond):
    """The '#if cond' branch (to its #else or #endif) of the harness's profile settings."""
    i = src.index("#if " + cond + "\n    if (a.")
    depth = 0
    pos = i
    for line in src[i:].splitlines(keepends=True):
        s = line.strip()
        if s.startswith("#if"):
            depth += 1
        elif s.startswith("#endif") or (s.startswith("#else") and depth == 1):
            if depth == 1:
                return src[i:pos]
            if s.startswith("#endif"):
                depth -= 1
        pos += len(line)
    raise AssertionError("no end for #if " + cond)


def test_each_define_is_refused_off_the_mosaic_image_and_passed_on_it():
    lists = foreach_lists(read(FC_CMAKE))
    assert len(lists) == 2
    refuse = [n for n, body in lists if "FATAL_ERROR" in body]
    define = [n for n, body in lists if "add_compile_definitions" in body]
    assert len(refuse) == 1 and len(define) == 1
    for d in list(SWITCHES) + list(OTHER):
        assert d in refuse[0], d + " is not refused outside the mosaic-only image"
        assert d in define[0], d + " is not passed to the compiler"


def test_each_define_sets_only_its_own_profile_fields_in_startRawShadow():
    main = read(MAIN_CPP)
    start = main.index("static void startRawShadow()")
    end = main.index("gnss_raw::applyReceiverProfile(rx, cfg);", start)
    body = main[start:end]
    all_fields = {f for fields, *_ in SWITCHES.values() for f in fields}
    for d, (fields, *_rest) in SWITCHES.items():
        blk = ifdef_block(body, d)
        for f in fields:
            assert re.search(r"\brx\." + f + r"\s*=", blk), d + " does not set rx." + f
        for other in all_fields - set(fields):
            assert not re.search(r"\brx\." + other + r"\s*=", blk), d + " sets another switch's rx." + other
        # its own block: no other bench define opens inside it
        for o in SWITCHES:
            if o != d:
                assert "#ifdef " + o not in blk, o + " opens inside " + d + "'s block"
    # the pairs' driver is set up after every bench override, from what they left
    assert body.index("rawPairsStart(rx)") > max(body.index("#ifdef " + d) for d in SWITCHES)


@pytest.mark.parametrize("define", sorted(SWITCHES))
def test_the_harness_flag_sets_the_same_profile_field_in_its_own_build_option(define):
    fields, flag, option, key = SWITCHES[define]
    rr = read(RR_CPP)
    cm = read(RR_CMAKE)
    # the option and its compile definition
    assert re.search(r"option\(" + option + r"\b", cm), option
    assert re.search(option + r"=\$<BOOL:\$\{" + option + r"\}>", cm), option + " is not defined for the target"
    # the flag is parsed
    assert 'k == "' + flag + '"' in rr, flag
    # in its option's block, the flag's profile fields, named for the firmware's define
    blk = if_block(rr, option)
    for f in fields:
        assert re.search(r"\brx\." + f + r"\s*=", blk), flag + " does not set rx." + f + " under " + option
    assert "// " + define in blk, option + "'s block does not name " + define
    # the events JSON says what the run had
    if key is not None:
        assert key in rr, "the events JSON has no " + key


def test_no_harness_block_sets_another_switchs_field():
    rr = read(RR_CPP)
    all_fields = {f for fields, *_ in SWITCHES.values() for f in fields}
    for define, (fields, flag, option, key) in SWITCHES.items():
        blk = if_block(rr, option)
        mine = {f for d, (fs, _, o, _) in SWITCHES.items() if o == option for f in fs}
        for other in all_fields - mine:
            assert not re.search(r"\brx\." + other + r"\s*=", blk), option + " sets rx." + other
