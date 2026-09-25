"""#1190 shock-gate saturation flags, decoded per SENSOR axis (#552).

The flight computer holds the attitude quaternion while a gyro or high-g
accelerometer axis sits at its rail, because a saturated IMU is not measuring
rotation.  The host replay has to model that or it is replaying a filter that
never flew -- on the 2026-08-29 RIM-66 log the difference is 43.6 vs 29.9 m/s
of median coast velocity error, which is the whole subject of #552.

Two things make this easy to get wrong, and both are pinned here.

**The test must run on raw counts, before any rotation.**  The parser applies a
-45 deg sensor->board rotation and then a board->rocket quaternion, so a
rotated component reaches sqrt(2)x a single axis.  Testing a rotated value
reports "4407 dps" on a +-4000 dps part and calls saturation where there is
none -- and, worse, misses a genuinely railed axis whose partner happens to
cancel it.

**The gyro's specified range is NOT where the int16 word saturates.**  ST gives
the gyro a fixed 0.035 mdps/LSB per dps of full scale (#369), so the specified
+-4000 dps range is +-28571 LSB while the word runs on to +-32767 (114.7% of
it).  Counts above +-28571 are OUT OF SPEC rather than impossible, and the
2026-08-29 bursts logged 32086 and 31480 on a single sensor axis -- verified on
the raw log frames with no scaling or rotation applied.  So a threshold written
as 32767, or as "FS/32768", would never fire.  The gate trips just BELOW the
spec limit precisely because a reading beyond it has unspecified accuracy: it
is not a rate, it is a sensor that has stopped measuring.
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
if str(REPO_ROOT / "Data_Analysis") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "Data_Analysis"))

import plot_flight_data_mini as pfd  # noqa: E402


def frame(msg_type, payload):
    body = bytes([msg_type, len(payload)]) + payload
    crc = pfd.crc16(body)
    return pfd.SYNC + body + bytes([crc >> 8, crc & 0xFF])


def status_query():
    """v5 OutStatusQueryData: +-16 g / +-256 g / +-4000 dps, b2r identity."""
    p = struct.pack(
        "<BHHhhB hhh BB hhhh h",
        16, 256, 4000, -4500, 0, 5,
        0, 0, 0,
        0, 0, 10000, 0, 0, 0,
        0,
    )
    p += struct.pack("<ffhBBB", 0, 0, 0, 0, 0, 0)
    return frame(0xA0, p)


def ism6(t_us, gyro=(0, 0, 0), high=(0, 0, 0), low=(0, 0, 0)):
    return frame(pfd.MSG_ISM6HG256,
                 struct.pack(pfd.FMT_ISM6, t_us, *low, *high, *gyro))


def parse(tmp_path, *frames):
    log = tmp_path / "log.bin"
    log.write_bytes(status_query() + b"".join(frames))
    records, _, _ = pfd.parse_binary_file(str(log))
    return records["ISM6HG256"]


def test_thresholds_match_the_firmware():
    # imu_drain_window.h: gyroFsFractionLsb / accelFsFractionLsb at 0.95.
    assert pfd.ISM6_GYRO_RAIL_LSB == int(0.95 * (1.0 / 0.035e-3)) == 27142
    assert pfd.ISM6_ACCEL_RAIL_LSB == int(0.95 * 32768.0) == 31129
    # The gyro threshold is BELOW the int16 rail, which is the point: the
    # part can output past its nominal full scale without ever saturating
    # the word, and a gate keyed on 32767 would sleep through the burst.
    assert pfd.ISM6_GYRO_RAIL_LSB < 32767


def test_quiet_samples_do_not_trip(tmp_path):
    recs = parse(tmp_path, ism6(1000, gyro=(1000, -2000, 500), high=(100, 100, 100)))
    assert recs[0]["gyro_railed"] is False
    assert recs[0]["accel_railed"] is False


def test_a_near_rail_gyro_axis_trips(tmp_path):
    # 32031 LSB: the real 2026-08-29 nose burst, past the specified +-28571
    # and nowhere near the int16 rail. Quoted as counts, not as a rate -- the
    # part is out of spec there, so "4492 dps" would be reading a number the
    # datasheet does not stand behind.
    recs = parse(tmp_path, ism6(1000, gyro=(0, 32031, 0)))
    assert recs[0]["gyro_railed"] is True


def test_each_gyro_axis_is_tested_independently(tmp_path):
    for axis in range(3):
        g = [0, 0, 0]
        g[axis] = pfd.ISM6_GYRO_RAIL_LSB
        recs = parse(tmp_path, ism6(1000, gyro=tuple(g)))
        assert recs[0]["gyro_railed"] is True, f"axis {axis} not tested"
        g[axis] = -pfd.ISM6_GYRO_RAIL_LSB
        recs = parse(tmp_path, ism6(1000, gyro=tuple(g)))
        assert recs[0]["gyro_railed"] is True, f"axis {axis} negative rail missed"


def test_the_verdict_is_taken_before_rotation(tmp_path):
    """The -45 deg rotation must not be able to create or hide a rail.

    X and Y at 0.71x the threshold each: individually well clear, but the
    rotation sums them to just above it.  A verdict taken after rotating
    would call this saturation. It is not -- neither sensor axis is near its
    rail, and the part is reporting both faithfully.
    """
    sub = int(pfd.ISM6_GYRO_RAIL_LSB * 0.71)
    recs = parse(tmp_path, ism6(1000, gyro=(sub, -sub, 0)))
    rotated_peak = max(abs(recs[0]["gyro_x"]), abs(recs[0]["gyro_y"]))
    assert rotated_peak > pfd.ISM6_GYRO_RAIL_LSB * 0.140, (
        "this case must actually exceed the threshold once rotated, "
        f"else it proves nothing (got {rotated_peak} dps)")
    assert recs[0]["gyro_railed"] is False


def test_a_railed_axis_survives_a_cancelling_partner(tmp_path):
    """The mirror failure: rotation can also CANCEL a genuinely railed axis.

    The chip rotation is -45 deg about +Z, so equal X and Y at the rail give
    out_x = r*(c+s) = sqrt(2)*r and out_y = r*(c-s) = 0.  Two sensor axes are
    pinned at their rail and the rotated Y reads exactly zero: a verdict taken
    after rotating, per component, would see nothing wrong on that axis.
    """
    r = pfd.ISM6_GYRO_RAIL_LSB + 10
    recs = parse(tmp_path, ism6(1000, gyro=(r, r, 0)))
    assert abs(recs[0]["gyro_y"]) < 1e-6, "the rotated Y really should cancel"
    assert recs[0]["gyro_railed"] is True


def test_the_accel_verdict_uses_the_high_g_channel(tmp_path):
    # The low-g channel saturates at 16 g and is expected to sit at its rail
    # through any real boost; only the high-g channel can say "shock".
    recs = parse(tmp_path, ism6(1000, low=(32767, 32767, 32767)))
    assert recs[0]["accel_railed"] is False
    recs = parse(tmp_path, ism6(1000, high=(0, 0, pfd.ISM6_ACCEL_RAIL_LSB)))
    assert recs[0]["accel_railed"] is True


# ---- #1191: the low-g -> high-g switch, judged the same way ----------------
#
# The flight loop hands its estimators the high-g channel once any low-g SENSOR
# axis passes full scale less 0.5 g.  The thrust axis sits between sensor X and
# Y, so a body-frame test gets it wrong both ways: body X reaches sqrt(2)x a
# sensor axis with nothing railed, and one railed axis reads ~11 g on body X/Y.

def test_the_low_g_bar_matches_the_firmware():
    # imu_drain_window.h nearRailLsb(fs, 0.5), with the full scale from the log.
    assert pfd.low_g_near_rail_lsb(16) == 31744
    assert pfd.low_g_near_rail_lsb(8) == 30720


def test_the_low_g_flag_is_strictly_above_the_bar_on_any_axis(tmp_path):
    bar = pfd.low_g_near_rail_lsb(16)
    assert parse(tmp_path, ism6(1000, low=(bar, 0, 0)))[0]["low_g_near_rail"] is False
    assert parse(tmp_path, ism6(1000, low=(0, -(bar + 1), 0)))[0]["low_g_near_rail"] is True
    assert parse(tmp_path, ism6(1000, low=(0, 0, -32768)))[0]["low_g_near_rail"] is True


def test_the_low_g_verdict_is_taken_before_rotation(tmp_path):
    # 28000 LSB (13.7 g) on both sensor axes: 19.3 g on body X, nothing railed.
    recs = parse(tmp_path, ism6(1000, low=(28000, 28000, 0), high=(1750, 1750, 0)))
    assert recs[0]["low_acc_x"] > 15.5 * pfd.G_MS2, "must beat the old body-frame bar"
    assert recs[0]["low_g_near_rail"] is False
    assert pfd.firmware_accel_xyz(recs[0])[0] == recs[0]["low_acc_x"]
    # Sensor X at the rail alone: under 11.4 g on each body axis, but railed.
    recs = parse(tmp_path, ism6(1000, low=(32767, 0, 0), high=(2600, 0, 0)))
    assert max(abs(recs[0]["low_acc_x"]), abs(recs[0]["low_acc_y"])) < 15.5 * pfd.G_MS2
    assert recs[0]["low_g_near_rail"] is True
    assert pfd.firmware_accel_xyz(recs[0])[0] == recs[0]["high_acc_x"]


def test_the_report_picks_its_accelerometer_by_the_flag(tmp_path):
    from flight_report.imu import accel_magnitude

    clean = parse(tmp_path, ism6(1000, low=(28000, 28000, 0), high=(1750, 1750, 0)))
    _, which = accel_magnitude({"ISM6HG256": clean})
    assert which == "low-G accelerometer"

    railed = parse(tmp_path, ism6(1000, low=(32767, 0, 0), high=(2600, 0, 0)))
    mag, which = accel_magnitude({"ISM6HG256": railed})
    assert which == "high-G accelerometer (low-G accelerometer at its rail)"
    assert abs(mag[0] - 2600 * (256 / 32768) * pfd.G_MS2) < 1e-6
