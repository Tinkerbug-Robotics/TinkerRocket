"""The flight computer's two partition tables, and the switches that pick the 4 MB one.

The flight computer's counterpart of test_oc_partition_tables.py. Every flight computer boots from at least 8 MB of
flash except the Tinker-Beetle V2's (-DTR_BOARD_M2=1): its ESP32-S3FH4R2 carries 4 MB in the package, like the out
computer beside it. An image that declares more flash than the part has stops in the bootloader, before any firmware
runs, so that board gets partitions_4mb.csv and a 4 MB declaration from sdkconfig.defaults.flash4mb. No CI job builds
an M2 image yet (the board is not ordered), so nothing else would notice a table that overlaps, runs past 4 MB, or is
never selected. These tests read the sources and check each piece.
"""
import csv
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
FC = os.path.join(ROOT, "tinkerrocket-idf", "projects", "flight_computer")
FC_CMAKE = os.path.join(FC, "CMakeLists.txt")
MAIN_CPP = os.path.join(FC, "main", "main.cpp")
OVERLAY = os.path.join(FC, "sdkconfig.defaults.flash4mb")

MB = 1024 * 1024


def read(path):
    with open(path) as fh:
        return fh.read()


def table(name):
    """The rows of a partition CSV as dicts, in file order."""
    rows = []
    with open(os.path.join(FC, name)) as fh:
        for row in csv.reader(fh):
            if not row or row[0].lstrip().startswith("#"):
                continue
            cells = [c.strip() for c in row]
            rows.append({"name": cells[0], "type": cells[1], "subtype": cells[2],
                         "offset": int(cells[3], 0), "size": int(cells[4], 0)})
    return rows


def end(rows):
    return max(r["offset"] + r["size"] for r in rows)


def by_name(rows):
    return {r["name"]: r for r in rows}


def test_the_4mb_table_has_the_same_partitions_in_the_same_order():
    full, small = table("partitions.csv"), table("partitions_4mb.csv")
    assert [(r["name"], r["type"], r["subtype"]) for r in small] == [(r["name"], r["type"], r["subtype"]) for r in full]
    assert [r["name"] for r in small] == ["nvs", "otadata", "ota_0", "ota_1"]


def test_it_fits_4mb_and_needs_more_than_2mb():
    small = table("partitions_4mb.csv")
    assert end(small) <= 4 * MB
    # 4 MB is then the smallest standard size that covers it, which is what the declaration must be (#916)
    assert end(small) > 2 * MB
    # and the shared table is the reason a 4 MB board cannot use it
    assert end(table("partitions.csv")) > 4 * MB


def test_no_partition_overlaps_another_and_each_is_aligned():
    for name in ("partitions.csv", "partitions_4mb.csv"):
        rows = table(name)
        assert rows[0]["offset"] >= 0x9000, name
        for prev, cur in zip(rows, rows[1:]):
            assert cur["offset"] >= prev["offset"] + prev["size"], "%s: %s overlaps %s" % (name, cur["name"], prev["name"])
        for r in rows:
            align = 0x10000 if r["type"] == "app" else 0x1000
            assert r["offset"] % align == 0, "%s: %s is not aligned" % (name, r["name"])
            assert r["size"] % 0x1000 == 0 and r["size"] > 0, "%s: %s" % (name, r["name"])


def test_only_the_app_slots_shrink_and_they_are_as_large_as_4mb_allows():
    full, small = by_name(table("partitions.csv")), by_name(table("partitions_4mb.csv"))
    for name in ("nvs", "otadata"):
        assert (small[name]["offset"], small[name]["size"]) == (full[name]["offset"], full[name]["size"])
    for rows in (full, small):
        assert rows["ota_0"]["size"] == rows["ota_1"]["size"]
        assert rows["ota_0"]["offset"] == 0x10000
    assert small["ota_0"]["size"] == 0x1F0000
    # one more 64 KB step on each slot would push the table past the end of the part
    assert end(table("partitions_4mb.csv")) + 2 * 0x10000 > 4 * MB


def test_the_overlay_declares_4mb_and_names_the_table():
    lines = [ln.strip() for ln in read(OVERLAY).splitlines() if ln.strip() and not ln.lstrip().startswith("#")]
    assert lines == ["CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y", 'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions_4mb.csv"']


def test_only_the_beetle_v2_build_takes_the_overlay_and_takes_it_last():
    src = read(FC_CMAKE)
    append = 'list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.defaults.flash4mb")'
    assert src.count("sdkconfig.defaults.flash4mb\")") == 1
    guard = src.index("if(TR_BOARD_M2)\n    " + append)
    # after every other overlay the M2 build takes (the M1's S3 overlay, the PX1105R image's), so the last file wins,
    # and before the IDF project is entered
    assert src.index('set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.m1")') < guard
    assert src.rindex('list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.defaults.mosaic")') < guard
    assert guard < src.index("include($ENV{IDF_PATH}/tools/cmake/project.cmake)")
    assert 'set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")' in src


def test_the_p4_psram_overlay_never_reaches_the_s3():
    # sdkconfig.defaults.rawnav is the P4's hex PSRAM at 200 MHz; the Beetle V2's S3 runs quad PSRAM from its own overlay
    src = read(FC_CMAKE)
    px = src.index("\nif(TR_GNSS_PX1105R)\n")   # the PX1105R image's block, not the replay check's
    rawnav = src.index('list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.defaults.rawnav")', px)
    assert "if(NOT TR_BOARD_M2)" in src[px:rawnav]


def test_an_m2_image_that_does_not_declare_4mb_does_not_compile():
    src = read(MAIN_CPP)
    guard = re.search(r"^#if TR_BOARD_M2 && !defined\(CONFIG_ESPTOOLPY_FLASHSIZE_4MB\)\n#error ", src, re.M)
    assert guard, "the boot-flash guard for TR_BOARD_M2 is gone from flight_computer/main/main.cpp"
