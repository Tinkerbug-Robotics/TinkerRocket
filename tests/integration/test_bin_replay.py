"""
Integration tests that replay binary .bin flight logs.

These tests require test data files in tests/test_data/.
If no .bin files are present, tests are skipped (not failed).

To add test data:
    cp /path/to/bench_log.bin tests/test_data/bench_static_60s.bin
"""
import pytest
from collections import defaultdict
from conftest import (
    parse_bin_file,
    get_bin_files,
    MSG_TYPES,
    board_variant,
    BOARD_OLD,
    BOARD_NEW,
)


bin_files = get_bin_files()
skip_if_no_data = pytest.mark.skipif(
    len(bin_files) == 0,
    reason="No .bin test data files in tests/test_data/"
)


# MRAM ring buffer artifacts: STATUS_QUERY frames with a fixed bogus
# timestamp (0xA000016F...) appear at the boundary of the ring buffer.
# These are not real sensor data and should be excluded from CRC checks.
MRAM_ARTIFACT_TIMESTAMP = 2684420112  # 0xA000_0110 little-endian artifact


def is_mram_artifact(frame):
    """True if this frame is a known MRAM ring buffer boundary artifact."""
    return frame.msg_type == 0xA0 and frame.timestamp_us == MRAM_ARTIFACT_TIMESTAMP


def sensor_frames(frames):
    """Return only real sensor frames (CRC-valid, not MRAM artifacts)."""
    return [f for f in frames if f.crc_valid and not is_mram_artifact(f)]


@skip_if_no_data
@pytest.mark.parametrize("binfile", bin_files, ids=[f.name for f in bin_files])
class TestBinReplay:

    def test_crc_error_rate_below_threshold(self, binfile):
        """CRC error rate should be < 0.1% (excluding MRAM artifacts)."""
        frames = parse_bin_file(binfile)
        assert len(frames) > 0, "No frames parsed"

        # Exclude known MRAM boundary artifacts from both counts
        real_frames = [f for f in frames if not is_mram_artifact(f)]
        invalid = [f for f in real_frames if not f.crc_valid]

        if len(real_frames) == 0:
            pytest.skip("No real sensor frames found")

        error_rate = len(invalid) / len(real_frames)
        assert error_rate < 0.001, (
            f"{len(invalid)} CRC errors in {len(real_frames)} frames "
            f"({error_rate*100:.2f}% > 0.1% threshold)"
        )

    def test_timestamp_monotonic(self, binfile):
        """Per-sensor-type timestamps should be monotonically increasing."""
        frames = sensor_frames(parse_bin_file(binfile))
        by_type = defaultdict(list)
        for f in frames:
            by_type[f.msg_type].append(f.timestamp_us)

        for msg_type, timestamps in by_type.items():
            violations = 0
            for i in range(len(timestamps) - 1):
                if timestamps[i + 1] < timestamps[i]:
                    violations += 1
            name = MSG_TYPES.get(msg_type, f"0x{msg_type:02X}")
            assert violations == 0, (
                f"{name}: {violations} non-monotonic timestamps"
            )

    def test_imu_rate_above_900hz(self, binfile):
        """ISM6HG256 median sample rate should be > 900 Hz (target: 960).

        Uses median interval instead of mean to be robust against
        occasional NAND flush stalls that create large isolated gaps.
        """
        frames = sensor_frames(parse_bin_file(binfile))
        imu_ts = [f.timestamp_us for f in frames if f.msg_type == 0xA2]

        if len(imu_ts) < 100:
            pytest.skip("Insufficient IMU data for rate check")

        diffs = sorted([imu_ts[i+1] - imu_ts[i] for i in range(len(imu_ts)-1)
                        if imu_ts[i+1] > imu_ts[i]])
        median_dt = diffs[len(diffs) // 2]
        rate = 1e6 / median_dt
        assert rate > 900, f"IMU median rate {rate:.0f} Hz < 900 Hz threshold"

    def test_baro_rate_above_400hz(self, binfile):
        """BMP585 median sample rate should be > 400 Hz (target: 500)."""
        frames = sensor_frames(parse_bin_file(binfile))
        ts = [f.timestamp_us for f in frames if f.msg_type == 0xA3]

        if len(ts) < 50:
            pytest.skip("Insufficient baro data")

        diffs = sorted([ts[i+1] - ts[i] for i in range(len(ts)-1)
                        if ts[i+1] > ts[i]])
        median_dt = diffs[len(diffs) // 2]
        rate = 1e6 / median_dt
        assert rate > 400, f"Baro median rate {rate:.0f} Hz < 400 Hz"

    def test_no_large_imu_gaps(self, binfile):
        """IMU timestamp gap budget.

        Two thresholds are checked:
        - Count: at most 2 gaps > 10 ms (lets NAND page writes / file close
          cause one or two isolated stalls).
        - Hard cap: no single gap > 50 ms.  A stall that long means the
          flight loop blocked in something that's not a normal flush, e.g.
          a synchronous log-close, a GNSS parse spike, or an I2C lockup.
          At 1 kHz we lose 50+ IMU samples in that window, which is
          visible as a step in the EKF.  The 4/21 bench run had a single
          746 ms stall on file close; that is the class of failure we
          need to catch here, not tolerate.
        """
        frames = sensor_frames(parse_bin_file(binfile))
        imu_ts = [f.timestamp_us for f in frames if f.msg_type == 0xA2]

        if len(imu_ts) < 100:
            pytest.skip("Insufficient IMU data")

        large_gaps = []
        max_gap_ms = 0.0
        for i in range(len(imu_ts) - 1):
            dt_ms = (imu_ts[i+1] - imu_ts[i]) / 1000.0
            if dt_ms > 10.0:
                large_gaps.append(dt_ms)
            if dt_ms > max_gap_ms:
                max_gap_ms = dt_ms

        assert max_gap_ms <= 50.0, (
            f"IMU stalled for {max_gap_ms:.1f} ms (hard cap 50 ms). "
            f"Log-close / NAND-flush should finish in < 50 ms."
        )
        assert len(large_gaps) <= 2, (
            f"{len(large_gaps)} IMU gaps > 10ms (max 2 allowed): "
            f"{[f'{g:.1f}ms' for g in large_gaps]}"
        )

    def test_mag_frames_match_board_variant(self, binfile):
        """Magnetometer frame counts must match the declared board variant.

        Old board: MMC5983MA over SPI at 200 Hz. Expect MMC frames present,
        zero IIS2MDC frames (the IIS2MDC chip isn't populated).

        New board: IIS2MDC over I2C at 100 Hz. Expect IIS2MDC frames present,
        zero MMC frames (poll path gated on !iis2mdc_active).

        Either-direction non-zero count on the wrong stream is a regression
        of #98's auto-detect / poll-gate.
        """
        frames = sensor_frames(parse_bin_file(binfile))
        mmc_count = sum(1 for f in frames if f.msg_type == 0xA4)
        iis2mdc_count = sum(1 for f in frames if f.msg_type == 0xD1)
        variant = board_variant(binfile)

        if variant == BOARD_NEW:
            assert mmc_count == 0, (
                f"new-board log contains {mmc_count} MMC5983MA frames "
                f"(0xA4); MMC poll path should be gated off when "
                f"iis2mdc_active is true. Likely regression of #98."
            )
            if len(frames) < 1000:
                pytest.skip("Capture too short to assert IIS2MDC presence")
            assert iis2mdc_count > 100, (
                f"new-board log has only {iis2mdc_count} IIS2MDC frames "
                f"(0xD1); the I2C mag stream looks dead. Check that "
                f"SensorCollector::pollIMUdata is firing iis2mdc.readRawXYZ."
            )
        elif variant == BOARD_OLD:
            assert iis2mdc_count == 0, (
                f"old-board log contains {iis2mdc_count} IIS2MDC frames "
                f"(0xD1); the IIS2MDC chip isn't populated on this PCB rev."
            )
            # Sanity gate: a stationary 20+ s capture at 200 Hz target
            # should yield thousands of MMC frames. Use a low floor
            # (100) so a brief stall-truncated capture still passes.
            if len(frames) < 1000:
                pytest.skip("Capture too short to assert MMC presence")
            assert mmc_count > 100, (
                f"old-board log has only {mmc_count} MMC5983MA frames "
                f"(0xA4); the SPI mag stream looks dead."
            )
