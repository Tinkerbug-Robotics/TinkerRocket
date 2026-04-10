"""
Integration tests that replay binary .bin flight logs.

These tests require test data files in tests/test_data/.
If no .bin files are present, tests are skipped (not failed).

To add test data:
    cp /path/to/bench_log.bin tests/test_data/bench_static_60s.bin
"""
import pytest
from collections import defaultdict
from conftest import parse_bin_file, get_bin_files, MSG_TYPES


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
        """No IMU timestamp gaps > 10ms (excluding NAND flush stalls).

        Allows up to 2 gaps > 10ms from NAND page writes / file close.
        A clean bench run should have zero, but the log-stop flush can
        cause one or two isolated stalls.
        """
        frames = sensor_frames(parse_bin_file(binfile))
        imu_ts = [f.timestamp_us for f in frames if f.msg_type == 0xA2]

        if len(imu_ts) < 100:
            pytest.skip("Insufficient IMU data")

        large_gaps = []
        for i in range(len(imu_ts) - 1):
            dt_ms = (imu_ts[i+1] - imu_ts[i]) / 1000.0
            if dt_ms > 10.0:
                large_gaps.append(dt_ms)

        assert len(large_gaps) <= 2, (
            f"{len(large_gaps)} IMU gaps > 10ms (max 2 allowed): "
            f"{[f'{g:.1f}ms' for g in large_gaps]}"
        )
