import XCTest
@testable import TinkerRocketApp

/// Wire-format decode for the 0xCD BaseStationStorageStats frame. The payload
/// passed to `decode` is the 26 bytes AFTER the discriminator byte, LE-packed,
/// mirroring BaseStationStorageStatsData in RocketComputerTypes.h.
///
/// Guards the flags byte in particular: bit0 = mounted, bit1 = fallback (#761).
/// The dashboard's demotion warning keys on bit1 rather than inferring the
/// demotion from `backend == 0`, so the bit has to decode independently of the
/// mounted bit sharing the same byte.
final class BaseStationStorageStatsTests: XCTestCase {

    /// Build a 26-byte LE payload from three u64 byte counts + backend + flags.
    private func frame(total: UInt64, used: UInt64, free: UInt64,
                       backend: UInt8, flags: UInt8) -> [UInt8] {
        var b: [UInt8] = []
        for v in [total, used, free] {
            for shift in stride(from: 0, through: 56, by: 8) {
                b.append(UInt8((v >> UInt64(shift)) & 0xFF))
            }
        }
        b.append(backend)
        b.append(flags)
        return b
    }

    func testDecodeFieldsFromFullFrame() {
        // The healthy bench reading: 512 MB NAND, barely used.
        let bytes = frame(total: 481_296_384, used: 104_857, free: 481_191_527,
                          backend: 2, flags: 0x01)
        let s = BaseStationStorageStats.decode(bytes)
        XCTAssertNotNil(s)
        XCTAssertEqual(s?.totalBytes, 481_296_384)
        XCTAssertEqual(s?.usedBytes, 104_857)
        XCTAssertEqual(s?.freeBytes, 481_191_527)
        XCTAssertEqual(s?.backend, 2)
        XCTAssertEqual(s?.backendName, "External NAND")
        XCTAssertEqual(s?.mounted, true)
        XCTAssertEqual(s?.fallback, false)
        XCTAssertEqual(s?.reservedBytes, 0)
    }

    /// u64 byte counts must survive past INT32_MAX — an SD-card base station
    /// reports tens of GB, which a 32-bit read would wrap.
    func testLargeByteCountsSurviveDecode() {
        let bytes = frame(total: 512_000_000_000, used: 100_200_300_400,
                          free: 411_799_699_600, backend: 1, flags: 0x01)
        let s = BaseStationStorageStats.decode(bytes)
        XCTAssertEqual(s?.totalBytes, 512_000_000_000)
        XCTAssertEqual(s?.usedBytes, 100_200_300_400)
        XCTAssertEqual(s?.freeBytes, 411_799_699_600)
        XCTAssertEqual(s?.backendName, "SD card")
    }

    func testFlagsBitsDecodeIndependently() {
        func flagsOf(_ f: UInt8) -> (mounted: Bool, fallback: Bool) {
            let s = BaseStationStorageStats.decode(
                frame(total: 1024, used: 0, free: 1024, backend: 0, flags: f))
            return (s?.mounted ?? false, s?.fallback ?? false)
        }
        XCTAssertEqual(flagsOf(0x00).mounted, false)
        XCTAssertEqual(flagsOf(0x00).fallback, false)
        // Healthy: mounted, no demotion.
        XCTAssertEqual(flagsOf(0x01).mounted, true)
        XCTAssertEqual(flagsOf(0x01).fallback, false)
        // bit1 must not be read off the back of bit0.
        XCTAssertEqual(flagsOf(0x02).mounted, false)
        XCTAssertEqual(flagsOf(0x02).fallback, true)
        // The reported failure: mounted, but demoted to internal flash.
        XCTAssertEqual(flagsOf(0x03).mounted, true)
        XCTAssertEqual(flagsOf(0x03).fallback, true)
        // BSS_FLAG_RETRIED (bit2) is on the wire but not surfaced yet; it must
        // not bleed into either flag above.
        XCTAssertEqual(flagsOf(0x05).mounted, true)
        XCTAssertEqual(flagsOf(0x05).fallback, false)
        // Unknown high bits are ignored (forward-compatible wire format).
        XCTAssertEqual(flagsOf(0xFF).mounted, true)
        XCTAssertEqual(flagsOf(0xFF).fallback, true)
    }

    /// A demoted base station reports the internal partition, so `backendName`
    /// alone reads as a statement of fact — which is why the warning keys on the
    /// flag instead. Pin both so the pair can't drift apart.
    func testDemotedFrameNamesInternalFlashAndSetsFallback() {
        // ~1.9 MB SPIFFS partition, nearly full — the bench reading that opened
        // this: "Internal flash — Used 1.3 MB, Free 0.4 MB".
        let bytes = frame(total: 1_996_800, used: 1_363_148, free: 419_430,
                          backend: 0, flags: 0x03)
        let s = BaseStationStorageStats.decode(bytes)
        XCTAssertEqual(s?.backendName, "Internal flash")
        XCTAssertEqual(s?.fallback, true)
        XCTAssertEqual(s?.mounted, true)
    }

    func testShortFrameReturnsNil() {
        // 25 bytes is one short of the 26-byte layout.
        XCTAssertNil(BaseStationStorageStats.decode([UInt8](repeating: 0, count: 25)))
        XCTAssertNotNil(BaseStationStorageStats.decode([UInt8](repeating: 0, count: 26)))
    }
}
