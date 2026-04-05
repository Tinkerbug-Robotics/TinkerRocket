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


@skip_if_no_data
@pytest.mark.parametrize("binfile", bin_files, ids=[f.name for f in bin_files])
class TestBinReplay:

    def test_all_frames_valid_crc(self, binfile):
        """Every frame in the binary log should have a valid CRC."""
        frames = parse_bin_file(binfile)
        assert len(frames) > 0, "No frames parsed"

        invalid = [f for f in frames if not f.crc_valid]
        assert len(invalid) == 0, (
            f"{len(invalid)} CRC errors in {len(frames)} frames"
        )

    def test_timestamp_monotonic(self, binfile):
        """Per-sensor-type timestamps should be monotonically increasing."""
        frames = parse_bin_file(binfile)
        by_type = defaultdict(list)
        for f in frames:
            if f.crc_valid:
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
        """ISM6HG256 sample rate should be > 900 Hz (target: 960)."""
        frames = parse_bin_file(binfile)
        imu_ts = [f.timestamp_us for f in frames if f.msg_type == 0xA2 and f.crc_valid]

        if len(imu_ts) < 100:
            pytest.skip("Insufficient IMU data for rate check")

        diffs = [imu_ts[i+1] - imu_ts[i] for i in range(len(imu_ts)-1)
                 if imu_ts[i+1] > imu_ts[i]]
        mean_dt = sum(diffs) / len(diffs)
        rate = 1e6 / mean_dt
        assert rate > 900, f"IMU rate {rate:.0f} Hz < 900 Hz threshold"

    def test_baro_rate_above_400hz(self, binfile):
        """BMP585 sample rate should be > 400 Hz (target: 500)."""
        frames = parse_bin_file(binfile)
        ts = [f.timestamp_us for f in frames if f.msg_type == 0xA3 and f.crc_valid]

        if len(ts) < 50:
            pytest.skip("Insufficient baro data")

        diffs = [ts[i+1] - ts[i] for i in range(len(ts)-1) if ts[i+1] > ts[i]]
        rate = 1e6 / (sum(diffs) / len(diffs))
        assert rate > 400, f"Baro rate {rate:.0f} Hz < 400 Hz"

    def test_no_large_imu_gaps(self, binfile):
        """No IMU timestamp gaps > 10ms."""
        frames = parse_bin_file(binfile)
        imu_ts = [f.timestamp_us for f in frames if f.msg_type == 0xA2 and f.crc_valid]

        if len(imu_ts) < 100:
            pytest.skip("Insufficient IMU data")

        max_gap_ms = 0
        for i in range(len(imu_ts) - 1):
            dt = imu_ts[i+1] - imu_ts[i]
            if dt > 0:
                max_gap_ms = max(max_gap_ms, dt / 1000.0)

        assert max_gap_ms < 10.0, f"Max IMU gap {max_gap_ms:.1f} ms > 10 ms"
