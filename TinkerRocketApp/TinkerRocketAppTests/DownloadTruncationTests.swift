import XCTest
@testable import TinkerRocketApp

/// #832: a truncated download was saved, cached and reported complete.
///
/// The chunk header is `[offset u32 LE][len u16 LE][flags u8][data]`, flags
/// bit0 = EOF, bit1 = ABORT. #526 closed the explicit EOF|ABORT case; this is
/// the same truncation re-entering through a bare EOF — a notification dropped
/// AFTER the peripheral queued it, which is precisely the case the firmware's
/// redundant EOF exists to compensate for and which it cannot itself detect.
///
/// Nothing covered `handleFileChunk` or `completeDownload` before this file.
final class DownloadTruncationTests: XCTestCase {

    private func chunk(offset: UInt32, payload: [UInt8],
                       eof: Bool = false, abort: Bool = false) -> Data {
        var d = Data()
        d.append(contentsOf: withUnsafeBytes(of: offset.littleEndian, Array.init))
        d.append(contentsOf: withUnsafeBytes(of: UInt16(payload.count).littleEndian, Array.init))
        d.append((eof ? 0x01 : 0x00) | (abort ? 0x02 : 0x00))
        d.append(contentsOf: payload)
        return d
    }

    /// Drive a download through the real notification entry point.
    private func run(_ frames: [Data], expectedSize: UInt32) -> URL? {
        let d = BLEDevice(peripheral: nil, name: "TR-R-Test")
        d.isConnected = true
        d.files = [FileInfo(name: "flight.bin", size: expectedSize)]

        var result: URL?
        var called = false
        d.beginDownloadForTesting(filename: "flight.bin") { url in
            result = url; called = true
        }
        for f in frames { d.handleFileChunkForTesting(f) }
        XCTAssertTrue(called, "the completion handler must always be invoked")
        return result
    }

    /// A clean transfer still succeeds — the regression guard for the fix.
    func testCompleteTransferSucceeds() {
        let url = run([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
            chunk(offset: 100, payload: Array(repeating: 0xBB, count: 100)),
            chunk(offset: 200, payload: [], eof: true),
        ], expectedSize: 200)
        XCTAssertNotNil(url, "a contiguous, correctly-sized transfer must succeed")
    }

    /// The #832 defect: a chunk is dropped, EOF arrives with no abort bit.
    func testDroppedChunkIsDetectedByTheGap() {
        let url = run([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
            // offset 100 never arrives
            chunk(offset: 200, payload: Array(repeating: 0xCC, count: 100)),
            chunk(offset: 300, payload: [], eof: true),
        ], expectedSize: 300)
        XCTAssertNil(url, "a gap in the offsets must fail the download, not splice it")
    }

    /// EOF carries the device's own bytes_sent — a mismatch means missing data
    /// even when every chunk that DID arrive was contiguous.
    func testEofByteCountMismatchFails() {
        let url = run([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
            chunk(offset: 100, payload: [], eof: true),   // device claims 100...
        ], expectedSize: 300)                              // ...listing says 300
        XCTAssertNil(url, "short against the listing must fail on the EOF path too")
    }

    func testEofClaimingMoreThanArrivedFails() {
        let url = run([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
            chunk(offset: 999, payload: [], eof: true),   // device sent 999, we have 100
        ], expectedSize: 999)
        XCTAssertNil(url, "EOF byte count must be checked against what arrived")
    }

    /// A frame shorter than its own length header used to be dropped in
    /// silence, leaving a hole indistinguishable from a clean transfer.
    func testTruncatedFrameFails() {
        var bad = chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100))
        bad = bad.prefix(50)                               // cut mid-payload
        let url = run([Data(bad), chunk(offset: 100, payload: [], eof: true)],
                      expectedSize: 100)
        XCTAssertNil(url, "a frame shorter than its length header must fail")
    }

    /// #526 must keep working.
    func testExplicitAbortStillFails() {
        let url = run([
            chunk(offset: 0, payload: Array(repeating: 0xAA, count: 100)),
            chunk(offset: 100, payload: [], eof: true, abort: true),
        ], expectedSize: 300)
        XCTAssertNil(url)
    }
}
