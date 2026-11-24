"""The out computer's two partition tables, and the switches that pick the 4 MB one.

Every out computer boots from at least 8 MB of flash except the Tinker-Beetle V2's (-DTR_BOARD_M2=1): its
ESP32-S3FH4R2 carries 4 MB in the package. An image that declares more flash than the part has stops in the bootloader,
before any firmware runs, so that board gets partitions_4mb.csv and a 4 MB declaration from sdkconfig.defaults.flash4mb.
No CI job builds an M2 image yet (the board is not ordered), so nothing else would notice a table that overlaps, runs
past 4 MB, or is never selected. These tests read the sources and check each piece.
"""
import csv
import os
import re

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
OC = os.path.join(ROOT, "tinkerrocket-idf", "projects", "out_computer")
OC_CMAKE = os.path.join(OC, "CMakeLists.txt")
MAIN_CPP = os.path.join(OC, "main", "main.cpp")
OVERLAY = os.path.join(OC, "sdkconfig.defaults.flash4mb")

MB = 1024 * 1024


def read(path):
    with open(path) as fh:
        return fh.read()


def table(name):
    """The rows of a partition CSV as dicts, in file order."""
    rows = []
    with open(os.path.join(OC, name)) as fh:
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
    # main.cpp finds the flight token by this name, and the OTA code needs both slots
    assert [r["name"] for r in small] == ["nvs", "otadata", "ota_0", "ota_1", "coredump", "flighttok"]


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
        # after the bootloader (0x0) and the partition table itself (0x8000, one 4 KB sector)
        assert rows[0]["offset"] >= 0x9000, name
        for prev, cur in zip(rows, rows[1:]):
            assert cur["offset"] >= prev["offset"] + prev["size"], "%s: %s overlaps %s" % (name, cur["name"], prev["name"])
        for r in rows:
            align = 0x10000 if r["type"] == "app" else 0x1000
            assert r["offset"] % align == 0, "%s: %s is not aligned" % (name, r["name"])
            assert r["size"] % 0x1000 == 0 and r["size"] > 0, "%s: %s" % (name, r["name"])


def test_only_the_app_slots_shrink():
    full, small = by_name(table("partitions.csv")), by_name(table("partitions_4mb.csv"))
    # same place and size in both: a tool or a note that names them holds for either table
    for name in ("nvs", "otadata"):
        assert (small[name]["offset"], small[name]["size"]) == (full[name]["offset"], full[name]["size"])
    # same size, new place
    for name in ("coredump", "flighttok"):
        assert small[name]["size"] == full[name]["size"]
    for rows in (full, small):
        assert rows["ota_0"]["size"] == rows["ota_1"]["size"]
        assert rows["ota_0"]["offset"] == 0x10000
    assert small["ota_0"]["size"] < full["ota_0"]["size"]


def test_the_slots_are_as_large_as_4mb_allows():
    small = by_name(table("partitions_4mb.csv"))
    slot = small["ota_0"]["size"]
    assert slot == 0x1E0000
    # one more 64 KB step on each slot would push the table past the end of the part
    assert end(table("partitions_4mb.csv")) + 2 * 0x10000 > 4 * MB


def test_the_overlay_declares_4mb_and_names_the_table():
    lines = [ln.strip() for ln in read(OVERLAY).splitlines() if ln.strip() and not ln.lstrip().startswith("#")]
    assert lines == ["CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y", 'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions_4mb.csv"']


def test_only_the_beetle_v2_build_takes_the_overlay_and_takes_it_last():
    src = read(OC_CMAKE)
    assert src.count("sdkconfig.defaults.flash4mb\")") == 1
    append = 'list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.defaults.flash4mb")'
    assert append in src
    # inside the M2 branch of the overlay block, after the list is set: the last file wins
    m2 = src.index("    elseif(TR_BOARD_M2)\n        message(STATUS \"out_computer: sdkconfig overlay sdkconfig.defaults.v9")
    nxt = src.index("    elseif(TR_BOARD_V10)", m2)
    assert m2 < src.index(append) < nxt
    assert src.index('set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.v9")') < src.index(append)
    # and the generated config stays per build dir, or an M1 build could inherit the 4 MB table
    assert 'set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")' in src


def test_an_m2_image_that_does_not_declare_4mb_does_not_compile():
    src = read(MAIN_CPP)
    guard = re.search(r"^#if defined\(ESP_PLATFORM\) && TR_BOARD_M2 && !defined\(CONFIG_ESPTOOLPY_FLASHSIZE_4MB\)\n#error ",
                      src, re.M)
    assert guard, "the boot-flash guard for TR_BOARD_M2 is gone from out_computer/main/main.cpp"
