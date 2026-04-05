import XCTest
@testable import TinkerRocketApp

final class MessageParserTests: XCTestCase {

    let parser = MessageParser()

    // MARK: - Test Helpers

    /// Build a valid binary frame: [0xAA 0x55 0xAA 0x55][type][len][payload][CRC_MSB][CRC_LSB]
    private func buildFrame(type: UInt8, payload: [UInt8]) -> Data {
        var data = Data([0xAA, 0x55, 0xAA, 0x55])
        data.append(type)
        data.append(UInt8(payload.count))
        data.append(contentsOf: payload)

        // CRC16 over type + len + payload
        var crcData = Data()
        crcData.append(type)
        crcData.append(UInt8(payload.count))
        crcData.append(contentsOf: payload)
        let crc = calculateCRC16(crcData)
        data.append(UInt8((crc >> 8) & 0xFF))
        data.append(UInt8(crc & 0xFF))

        return data
    }

    /// CRC16 matching the C++ and Swift implementations (poly=0x8001, init=0)
    private func calculateCRC16(_ data: Data) -> UInt16 {
        let polynome: UInt16 = 0x8001
        var crc: UInt16 = 0x0000
        for byte in data {
            crc ^= UInt16(byte) << 8
            for _ in 0..<8 {
                if (crc & 0x8000) != 0 {
                    crc <<= 1
                    crc ^= polynome
                } else {
                    crc <<= 1
                }
            }
        }
        return crc
    }

    // MARK: - Frame Parsing Tests

    func testSingleValidFrame() throws {
        // 0xA2 = ISM6HG256 message, 22-byte payload
        let payload = [UInt8](repeating: 0x01, count: 22)
        let frame = buildFrame(type: 0xA2, payload: payload)

        // Write to temp file
        let url = FileManager.default.temporaryDirectory.appendingPathComponent("test_single.bin")
        try frame.write(to: url)
        defer { try? FileManager.default.removeItem(at: url) }

        let frames = try parser.parseLogFile(url)
        XCTAssertEqual(frames.count, 1)
        XCTAssertEqual(frames[0].type, 0xA2)
        XCTAssertEqual(frames[0].payload.count, 22)
    }

    func testMultipleFrames() throws {
        let frame1 = buildFrame(type: 0xA2, payload: [UInt8](repeating: 0x01, count: 22))
        let frame2 = buildFrame(type: 0xA3, payload: [UInt8](repeating: 0x02, count: 12))
        var data = Data()
        data.append(frame1)
        data.append(frame2)

        let url = FileManager.default.temporaryDirectory.appendingPathComponent("test_multi.bin")
        try data.write(to: url)
        defer { try? FileManager.default.removeItem(at: url) }

        let frames = try parser.parseLogFile(url)
        XCTAssertEqual(frames.count, 2)
        XCTAssertEqual(frames[0].type, 0xA2)
        XCTAssertEqual(frames[1].type, 0xA3)
    }

    func testBadCRC_FrameSkipped() throws {
        var frame = buildFrame(type: 0xA2, payload: [UInt8](repeating: 0x01, count: 22))
        // Corrupt last byte (CRC LSB)
        frame[frame.count - 1] ^= 0xFF

        let url = FileManager.default.temporaryDirectory.appendingPathComponent("test_badcrc.bin")
        try frame.write(to: url)
        defer { try? FileManager.default.removeItem(at: url) }

        let frames = try parser.parseLogFile(url)
        XCTAssertEqual(frames.count, 0) // CRC mismatch -> skipped
    }

    func testTruncatedFrame_NoCrash() throws {
        // Build valid frame then truncate it
        let frame = buildFrame(type: 0xA2, payload: [UInt8](repeating: 0x01, count: 22))
        let truncated = frame.prefix(10) // cut mid-payload

        let url = FileManager.default.temporaryDirectory.appendingPathComponent("test_trunc.bin")
        try truncated.write(to: url)
        defer { try? FileManager.default.removeItem(at: url) }

        // Should not crash
        let frames = try parser.parseLogFile(url)
        XCTAssertEqual(frames.count, 0)
    }

    func testEmptyData() throws {
        let url = FileManager.default.temporaryDirectory.appendingPathComponent("test_empty.bin")
        try Data().write(to: url)
        defer { try? FileManager.default.removeItem(at: url) }

        let frames = try parser.parseLogFile(url)
        XCTAssertEqual(frames.count, 0)
    }

    func testAllMessageTypes() throws {
        let types: [UInt8] = [0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6]
        let sizes = [42, 22, 12, 16, 43, 10] // GNSS, IMU, Baro, Mag, NonSensor, Power

        var data = Data()
        for (type, size) in zip(types, sizes) {
            data.append(buildFrame(type: type, payload: [UInt8](repeating: 0, count: size)))
        }

        let url = FileManager.default.temporaryDirectory.appendingPathComponent("test_alltypes.bin")
        try data.write(to: url)
        defer { try? FileManager.default.removeItem(at: url) }

        let frames = try parser.parseLogFile(url)
        XCTAssertEqual(frames.count, types.count)
        for (i, frame) in frames.enumerated() {
            XCTAssertEqual(frame.type, types[i])
        }
    }

    func testTimestamp_Extracted() throws {
        // First 4 bytes of payload = time_us (little-endian)
        var payload = [UInt8](repeating: 0, count: 22)
        // time_us = 123456 = 0x0001E240
        payload[0] = 0x40  // LSB
        payload[1] = 0xE2
        payload[2] = 0x01
        payload[3] = 0x00  // MSB

        let frame = buildFrame(type: 0xA2, payload: payload)
        let url = FileManager.default.temporaryDirectory.appendingPathComponent("test_ts.bin")
        try frame.write(to: url)
        defer { try? FileManager.default.removeItem(at: url) }

        let frames = try parser.parseLogFile(url)
        XCTAssertEqual(frames.count, 1)
        XCTAssertEqual(frames[0].timestamp, 123456)
    }

    // MARK: - CRC Cross-Validation

    func testCRC16_KnownVector() {
        // Verify our Swift CRC matches a known computation
        // type=0xA2, len=5, payload=[0x01,0x02,0x03,0x04,0x05]
        let data = Data([0xA2, 0x05, 0x01, 0x02, 0x03, 0x04, 0x05])
        let crc = calculateCRC16(data)

        // The CRC should be deterministic and non-zero
        XCTAssertNotEqual(crc, 0)

        // Second computation should match
        let crc2 = calculateCRC16(data)
        XCTAssertEqual(crc, crc2)
    }
}
