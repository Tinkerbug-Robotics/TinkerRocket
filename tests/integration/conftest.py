"""
Shared fixtures for binary log integration tests.

Test data files go in tests/test_data/ (Git LFS for large files).
To add test data:
    git lfs track "tests/test_data/*.bin"
    cp /path/to/bench_log.bin tests/test_data/bench_static_60s.bin
    git add tests/test_data/bench_static_60s.bin

Board variant metadata (optional):
    Each <binfile>.bin may have a companion <binfile>.meta.json with the
    schema {"board": "old" | "new"}. Captures from the legacy MMC5983MA
    PCB are "old"; captures from the IIS2MDC PCB rev are "new". When the
    sidecar is absent the variant defaults to "old" so legacy logs keep
    their existing assertions. The companion <binfile>.json (no .meta)
    is the iOS app's flight summary and is unrelated.
"""
import json
import pytest
import struct
from pathlib import Path
from collections import defaultdict
from dataclasses import dataclass, field


PREAMBLE = bytes([0xAA, 0x55, 0xAA, 0x55])

MSG_TYPES = {
    0xA0: "STATUS_QUERY",
    0xA1: "GNSS",
    0xA2: "ISM6HG256",
    0xA3: "BMP585",
    0xA4: "MMC5983MA",
    0xA5: "NON_SENSOR",
    0xA6: "POWER",
    0xD1: "IIS2MDC",
    0xF1: "LORA",
}


def crc16(data: bytes, poly: int = 0x8001, init: int = 0x0000) -> int:
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class ParsedFrame:
    msg_type: int
    payload: bytes
    timestamp_us: int
    crc_valid: bool


def parse_bin_file(path: Path) -> list:
    """Parse binary flight log into list of ParsedFrames."""
    data = path.read_bytes()
    frames = []
    idx = 0

    while idx < len(data) - 8:
        pos = data.find(PREAMBLE, idx)
        if pos == -1:
            break
        idx = pos + 4

        if idx + 2 > len(data):
            break

        msg_type = data[idx]
        msg_len = data[idx + 1]
        idx += 2

        if idx + msg_len + 2 > len(data):
            break

        payload = data[idx:idx + msg_len]
        crc_recv = (data[idx + msg_len] << 8) | data[idx + msg_len + 1]
        idx += msg_len + 2

        crc_calc = crc16(bytes([msg_type, msg_len]) + payload)
        crc_valid = crc_calc == crc_recv

        ts = struct.unpack_from("<I", payload, 0)[0] if len(payload) >= 4 else 0

        frames.append(ParsedFrame(msg_type, payload, ts, crc_valid))

    # Sort by timestamp so MRAM ring-buffer drain (which can interleave
    # older pre-launch frames with newer ones) doesn't cause spurious
    # monotonicity or gap failures.
    frames.sort(key=lambda f: f.timestamp_us)

    return frames


TEST_DATA_DIR = Path(__file__).parent.parent / "test_data"


# Board variants for sensor-suite differences:
#   "old" — MMC5983MA over SPI (pin 13 CS).  Mag frames type 0xA4 are emitted.
#   "new" — IIS2MDC over I2C  (pins 13 SDA, 20 SCL).  MMC poll path is
#            gated off via SensorCollector::iis2mdc_active, so the new board
#            emits zero 0xA4 frames.  IIS2MDC polling integration is a
#            follow-up; once it lands a new MSG type will be added here.
BOARD_OLD = "old"
BOARD_NEW = "new"
BOARD_DEFAULT = BOARD_OLD
KNOWN_BOARDS = (BOARD_OLD, BOARD_NEW)


def sidecar_path(binfile: Path) -> Path:
    """Return the expected sidecar metadata path for a given binfile."""
    return binfile.with_suffix(".meta.json")


def load_sidecar(binfile: Path) -> dict:
    """Load <binfile>.meta.json, or return {} if missing/unreadable.

    The companion <binfile>.json (no .meta) is the iOS flight-summary
    file written by TinkerRocketApp and intentionally not consulted here.
    """
    path = sidecar_path(binfile)
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except (json.JSONDecodeError, OSError):
        return {}


def board_variant(binfile: Path) -> str:
    """Return the declared board variant for a binfile.

    Reads the sidecar metadata if present, falling back to BOARD_DEFAULT
    so legacy logs (no sidecar) keep their pre-existing assertions. An
    unknown value in the sidecar is treated as missing — log a warning
    via pytest if/when we ever care, for now silently fall through.
    """
    meta = load_sidecar(binfile)
    declared = meta.get("board")
    if declared in KNOWN_BOARDS:
        return declared
    return BOARD_DEFAULT


@pytest.fixture
def test_data_dir():
    return TEST_DATA_DIR


def get_bin_files():
    """Return all .bin files in test_data/."""
    if not TEST_DATA_DIR.exists():
        return []
    return sorted(TEST_DATA_DIR.glob("*.bin"))
