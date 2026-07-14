import XCTest
@testable import TinkerRocketApp

/// Wire-format decode for the 0xCC RocketStorageStats frame. The payload passed
/// to `decode` is the 15 bytes AFTER the discriminator byte, LE-packed, mirroring
/// RocketStorageStatsData in RocketComputerTypes.h. Guards the flags byte in
/// particular: bit0 = initialized, bit1 = auto-evicted (#315).
final class RocketStorageStatsTests: XCTestCase {

    /// Build a 15-byte LE payload from the seven u16 fields + flags byte.
    private func frame(flightRegion: UInt16, used: UInt16, free: UInt16,
                       bad: UInt16, system: UInt16, flightCount: UInt16,
                       blockSizeKB: UInt16, flags: UInt8) -> [UInt8] {
        var b: [UInt8] = []
        for v in [flightRegion, used, free, bad, system, flightCount, blockSizeKB] {
            b.append(UInt8(v & 0xFF))
            b.append(UInt8(v >> 8))
        }
        b.append(flags)
        return b
    }

    func testDecodeFieldsFromFullFrame() {
        let bytes = frame(flightRegion: 2012, used: 800, free: 1200,
                          bad: 3, system: 36, flightCount: 10,
                          blockSizeKB: 256, flags: 0x01)
        let s = RocketStorageStats.decode(bytes)
        XCTAssertNotNil(s)
        XCTAssertEqual(s?.flightRegionBlocks, 2012)
        XCTAssertEqual(s?.usedBlocks, 800)
        XCTAssertEqual(s?.freeBlocks, 1200)
        XCTAssertEqual(s?.badBlocks, 3)
        XCTAssertEqual(s?.systemBlocks, 36)
        XCTAssertEqual(s?.flightCount, 10)
        XCTAssertEqual(s?.blockSizeKB, 256)
        XCTAssertEqual(s?.initialized, true)
        XCTAssertEqual(s?.autoEvicted, false)
        // Byte accounting derives from block count × block size.
        XCTAssertEqual(s?.usedBytes, 800 * 256 * 1024)
        XCTAssertEqual(s?.freeBytes, 1200 * 256 * 1024)
    }

    func testFlagsBitsDecodeIndependently() {
        func flagsOf(_ f: UInt8) -> (Bool, Bool) {
            let s = RocketStorageStats.decode(
                frame(flightRegion: 1, used: 0, free: 1, bad: 0, system: 0,
                      flightCount: 0, blockSizeKB: 256, flags: f))
            return (s?.initialized ?? false, s?.autoEvicted ?? false)
        }
        XCTAssertEqual(flagsOf(0x00).0, false); XCTAssertEqual(flagsOf(0x00).1, false)
        XCTAssertEqual(flagsOf(0x01).0, true);  XCTAssertEqual(flagsOf(0x01).1, false)
        XCTAssertEqual(flagsOf(0x02).0, false); XCTAssertEqual(flagsOf(0x02).1, true)
        XCTAssertEqual(flagsOf(0x03).0, true);  XCTAssertEqual(flagsOf(0x03).1, true)
        // Unknown high bits are ignored (forward-compatible wire format).
        XCTAssertEqual(flagsOf(0xFF).0, true);  XCTAssertEqual(flagsOf(0xFF).1, true)
    }

    func testShortFrameReturnsNil() {
        // 14 bytes is one short of the 15-byte layout.
        XCTAssertNil(RocketStorageStats.decode([UInt8](repeating: 0, count: 14)))
    }
}
