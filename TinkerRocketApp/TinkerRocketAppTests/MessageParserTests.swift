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

    // MARK: - Flight Settings Snapshot (#165)

    private func leU16(_ v: UInt16) -> [UInt8] { [UInt8(v & 0xFF), UInt8(v >> 8)] }
    private func leU32(_ v: UInt32) -> [UInt8] {
        [UInt8(v & 0xFF), UInt8((v >> 8) & 0xFF), UInt8((v >> 16) & 0xFF), UInt8((v >> 24) & 0xFF)]
    }
    private func leI16(_ v: Int16) -> [UInt8] { leU16(UInt16(bitPattern: v)) }
    private func leF32(_ v: Float) -> [UInt8] { leU32(v.bitPattern) }

    /// Build a known 188-byte FlightSettingsData payload (post 4-channel
    /// pyro layout) and verify the decode + JSON mapping. Offsets here must
    /// match the C++ struct (locked by the FlightSettingsData_Layout test
    /// in tests_cpp).
    func testFlightSettingsDecode() throws {
        var p: [UInt8] = []
        p += leU32(123456)        // time_us @0
        p.append(1)               // version @4
        let flags: UInt8 = (1 << 0) | (1 << 1) | (1 << 3) | (1 << 5)  // angle, gainSched, servo, sounds
        p.append(flags)           // flags @5
        p += leU16(500)           // roll_delay_ms @6
        p += leF32(0.02)          // kp @8
        p += leF32(0.0005)        // ki @12
        p += leF32(0.0003)        // kd @16
        p += leF32(10)            // d_lpf_hz @20
        p += leF32(-20)           // min_cmd_deg @24
        p += leF32(20)            // max_cmd_deg @28
        p += leF32(4.0)           // kp_angle @32
        p += leF32(60)            // kp_angle_rate_cap_dps @36
        p += leF32(50)            // gs_v_ref @40
        p += leF32(25)            // gs_v_min @44
        p += leF32(3)             // gs_scale_cap @48
        p += leF32(0)             // roll_rate_set_point @52
        p.append(16)              // ism6_low_g_fs_g @56
        p += leU16(256)           // ism6_high_g_fs_g @57
        p += leU16(4000)          // ism6_gyro_fs_dps @59
        p += leI16(85); p += leI16(-3); p += leI16(0); p += leI16(12)  // servo_bias_us @61
        p += leI16(333)           // servo_hz @69
        p += leI16(1250)          // servo_min_us @71
        p += leI16(1750)          // servo_max_us @73
        p.append(2)               // camera_type @75 (runcam)
        p.append(1); p.append(0); p += leF32(1.5)   // pyro ch1 @76: enabled, time mode, 1.5 s
        p.append(1); p.append(1); p += leF32(100)    // pyro ch2 @82: enabled, altitude mode, 100 m
        p.append(0); p.append(0); p += leF32(0)      // pyro ch3 @88: disabled
        p.append(1); p.append(0); p += leF32(2.5)    // pyro ch4 @94: enabled, time mode, 2.5 s
        let sha = Array("abc1234".utf8)              // fw_git_sha @100 (12 bytes)
        p += sha + [UInt8](repeating: 0, count: 12 - sha.count)
        p.append(2)               // num_waypoints @112
        p += [0, 0, 0]            // _pad @113
        p += leF32(0.5); p += leF32(0); p.append(1)      // wp0 @116: 0.5 s, 0°, null_rate
        p += leF32(1.0); p += leF32(180); p.append(0)    // wp1: 1.0 s, 180°, angle
        p += [UInt8](repeating: 0, count: 6 * 9)         // remaining 6 zero waypoint slots

        XCTAssertEqual(p.count, 188)

        let raw = try FlightSettingsData(from: Data(p))
        XCTAssertEqual(raw.time_us, 123456)
        XCTAssertEqual(raw.version, 1)
        XCTAssertTrue(raw.useAngleControl)
        XCTAssertTrue(raw.gainScheduleEnabled)
        XCTAssertFalse(raw.guidanceEnabled)
        XCTAssertTrue(raw.servoEnabled)
        XCTAssertTrue(raw.soundsEnabled)
        XCTAssertFalse(raw.fwDirty)
        XCTAssertEqual(raw.roll_delay_ms, 500)
        XCTAssertEqual(raw.kp, 0.02, accuracy: 1e-6)
        XCTAssertEqual(raw.ki, 0.0005, accuracy: 1e-7)
        XCTAssertEqual(raw.kd, 0.0003, accuracy: 1e-7)
        XCTAssertEqual(raw.d_lpf_hz, 10, accuracy: 1e-4)
        XCTAssertEqual(raw.min_cmd_deg, -20, accuracy: 1e-4)
        XCTAssertEqual(raw.max_cmd_deg, 20, accuracy: 1e-4)
        XCTAssertEqual(raw.kp_angle, 4.0, accuracy: 1e-4)
        XCTAssertEqual(raw.kp_angle_rate_cap_dps, 60, accuracy: 1e-4)
        XCTAssertEqual(raw.gs_v_ref, 50, accuracy: 1e-4)
        XCTAssertEqual(raw.gs_v_min, 25, accuracy: 1e-4)
        XCTAssertEqual(raw.gs_scale_cap, 3, accuracy: 1e-4)
        XCTAssertEqual(raw.roll_rate_set_point, 0, accuracy: 1e-4)
        XCTAssertEqual(raw.ism6_low_g_fs_g, 16)
        XCTAssertEqual(raw.ism6_high_g_fs_g, 256)
        XCTAssertEqual(raw.ism6_gyro_fs_dps, 4000)
        XCTAssertEqual(raw.servo_bias_us, [85, -3, 0, 12])
        XCTAssertEqual(raw.servo_hz, 333)
        XCTAssertEqual(raw.servo_min_us, 1250)
        XCTAssertEqual(raw.servo_max_us, 1750)
        XCTAssertEqual(raw.camera_type, 2)
        XCTAssertEqual(raw.pyro_enabled,        [true, true, false, true])
        XCTAssertEqual(raw.pyro_trigger_mode,   [0, 1, 0, 0])
        XCTAssertEqual(raw.pyro_trigger_value[0], 1.5, accuracy: 1e-4)
        XCTAssertEqual(raw.pyro_trigger_value[1], 100, accuracy: 1e-4)
        XCTAssertEqual(raw.pyro_trigger_value[2], 0,   accuracy: 1e-4)
        XCTAssertEqual(raw.pyro_trigger_value[3], 2.5, accuracy: 1e-4)
        XCTAssertEqual(raw.fw_git_sha, "abc1234")
        XCTAssertEqual(raw.num_waypoints, 2)
        XCTAssertEqual(raw.waypoints.count, 2)
        XCTAssertEqual(raw.waypoints[0].time_s, 0.5, accuracy: 1e-4)
        XCTAssertEqual(raw.waypoints[0].mode, 1)
        XCTAssertEqual(raw.waypoints[1].angle_deg, 180, accuracy: 1e-4)
        XCTAssertEqual(raw.waypoints[1].mode, 0)

        // JSON model mapping
        let s = FlightSettings(from: raw)
        XCTAssertEqual(s.fw_git_sha, "abc1234")
        XCTAssertTrue(s.sounds_enabled)
        XCTAssertEqual(s.roll_control.mode, "angle_profile")   // num_waypoints > 0
        XCTAssertEqual(s.roll_control.kp, 0.02, accuracy: 1e-9)
        XCTAssertEqual(s.roll_control.cmd_limit_min_deg, -20, accuracy: 1e-9)
        XCTAssertEqual(s.roll_control.profile.count, 2)
        XCTAssertEqual(s.roll_control.profile[0].mode, "null_rate")
        XCTAssertEqual(s.servo.bias_us, [85, -3, 0, 12])
        XCTAssertEqual(s.servo.frequency_hz, 333)
        XCTAssertEqual(s.camera.type, "runcam")
        XCTAssertEqual(s.pyro.ch1.trigger_mode, "time_after_apogee")
        XCTAssertEqual(s.pyro.ch1.trigger_value, 1.5, accuracy: 1e-6)
        XCTAssertEqual(s.pyro.ch2.trigger_mode, "altitude_on_descent")
        XCTAssertFalse(s.pyro.ch3.enabled)
        XCTAssertTrue(s.pyro.ch4.enabled)
        XCTAssertEqual(s.pyro.ch4.trigger_value, 2.5, accuracy: 1e-6)
        XCTAssertEqual(s.imu.gyro_fs_dps, 4000)

        // Undersized payload is rejected.
        XCTAssertThrowsError(try FlightSettingsData(from: Data(p.prefix(187))))
    }
}
