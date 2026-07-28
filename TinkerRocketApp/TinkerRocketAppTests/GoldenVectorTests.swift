import XCTest
@testable import TinkerRocketApp

/// The Swift golden walk over tests_cpp/fixtures/wire/ (android-port plan
/// §2.1).  Every assertion here compares a Swift decoder's output against the
/// .expected.json sidecar the C++ emitter produced by reading the SAME bytes
/// through the real packed firmware structs — so this suite, the C++
/// freshness gtest, and the Kotlin golden walk jointly make silent layout
/// drift between the three codebases unshippable.
///
/// Existing hand-written tests are intentionally untouched: they encode
/// semantic intent; this walk pins bytes.
final class GoldenVectorTests: XCTestCase {

    // MARK: - Corpus integrity

    func testManifestListsNonTrivialCorpus() {
        XCTAssertGreaterThanOrEqual(WireFixtures.manifestEntries().count, 40)
    }

    func testEveryManifestEntryExistsWithRecordedSize() throws {
        for entry in WireFixtures.manifestEntries() {
            let rel = entry.str("file")
            let expected = entry.int("size")
            let attrs = try? FileManager.default.attributesOfItem(
                atPath: WireFixtures.url(rel).path)
            let size = (attrs?[.size] as? NSNumber)?.intValue
            XCTAssertEqual(size, expected, "size drift: \(rel)")
        }
    }

    func testCommandFixturesLeadWithTheirCommandByte() {
        for entry in WireFixtures.manifestEntries() where entry.str("file").hasPrefix("commands/") {
            let rel = entry.str("file")
            let bytes = WireFixtures.data(rel)
            XCTAssertEqual(Int(bytes[0]), WireFixtures.sidecar(rel).int("cmd"),
                           "\(rel) first byte != sidecar cmd")
        }
    }

    // MARK: - Framed stream semantics (MessageParser)

    private func parse(_ rel: String) throws -> [MessageFrame] {
        try MessageParser().parseLogFile(WireFixtures.url(rel))
    }

    func testGoodStreamParsesEveryFrame() throws {
        let side = WireFixtures.sidecar("framed/stream_good.bin")
        let frames = try parse("framed/stream_good.bin")
        XCTAssertEqual(frames.count, side.int("frame_count"))
        XCTAssertEqual(frames.map { Int($0.type) }, side.ints("types"))
        XCTAssertEqual(frames.map { $0.payload.count }, side.ints("payload_lens"))
    }

    func testTimestampConventionFirstFourPayloadBytes() throws {
        let frames = try parse("framed/stream_good.bin")
        let gnss = frames.first { $0.type == 0xA1 }
        XCTAssertEqual(gnss?.timestamp, 3_600_123_456)  // > INT32_MAX by design
    }

    func testCorruptCrcSkipsAndResyncs() throws {
        let side = WireFixtures.sidecar("framed/stream_badcrc_resync.bin")
        let frames = try parse("framed/stream_badcrc_resync.bin")
        XCTAssertEqual(frames.count, side.int("valid_frame_count"))
        XCTAssertEqual(frames.map { Int($0.type) }, side.ints("valid_types"))
    }

    func testInflatedLengthByteWholeFrameSkipOvershoots() throws {
        // The resync discriminator: whole-frame-skip swallows the following
        // frame; an advance-by-one parser would recover it. iOS semantics
        // are the reference the sidecar records.
        let side = WireFixtures.sidecar("framed/stream_badlen_overshoot.bin")
        let frames = try parse("framed/stream_badlen_overshoot.bin")
        XCTAssertEqual(frames.count, side.int("valid_frame_count"))
        XCTAssertEqual(frames.map { Int($0.type) }, side.ints("valid_types"))
    }

    func testTruncatedTailStopsCleanly() throws {
        let side = WireFixtures.sidecar("framed/stream_truncated_tail.bin")
        let frames = try parse("framed/stream_truncated_tail.bin")
        XCTAssertEqual(frames.count, side.int("valid_frame_count"))
        XCTAssertEqual(frames.map { Int($0.type) }, side.ints("valid_types"))
    }

    // MARK: - Sensor payload decoders, field-for-field

    func testGnssDecode() throws {
        let side = WireFixtures.sidecar("logframes/gnss_42.bin")
        let g = try GNSSData(from: WireFixtures.data("logframes/gnss_42.bin"))
        XCTAssertEqual(Int64(g.time_us), side.i64("time_us"))
        XCTAssertEqual(Int(g.year), side.int("year"))
        XCTAssertEqual(Int(g.month), side.int("month"))
        XCTAssertEqual(Int(g.day), side.int("day"))
        XCTAssertEqual(Int(g.hour), side.int("hour"))
        XCTAssertEqual(Int(g.minute), side.int("minute"))
        XCTAssertEqual(Int(g.second), side.int("second"))
        XCTAssertEqual(Int(g.milli_second), side.int("milli_second"))
        XCTAssertEqual(Int(g.fix_mode), side.int("fix_mode"))
        XCTAssertEqual(Int(g.num_sats), side.int("num_sats"))
        XCTAssertEqual(Int(g.pdop_x10), side.int("pdop_x10"))
        XCTAssertEqual(Int(g.lat_e7), side.int("lat_e7"))
        XCTAssertEqual(Int(g.lon_e7), side.int("lon_e7"))
        XCTAssertEqual(Int(g.alt_mm), side.int("alt_mm"))
        XCTAssertEqual(Int(g.vel_e_mmps), side.int("vel_e_mmps"))
        XCTAssertEqual(Int(g.vel_n_mmps), side.int("vel_n_mmps"))
        XCTAssertEqual(Int(g.vel_u_mmps), side.int("vel_u_mmps"))
        XCTAssertEqual(Int(g.h_acc_m), side.int("h_acc_m"))
        XCTAssertEqual(Int(g.v_acc_m), side.int("v_acc_m"))
    }

    func testPowerDecode() throws {
        let side = WireFixtures.sidecar("logframes/power_10.bin")
        let p = try POWERData(from: WireFixtures.data("logframes/power_10.bin"))
        XCTAssertEqual(Int64(p.time_us), side.i64("time_us"))
        XCTAssertEqual(Int(p.voltage_raw), side.int("voltage_raw"))
        XCTAssertEqual(Int(p.current_raw), side.int("current_raw"))
        XCTAssertEqual(Int(p.soc_raw), side.int("soc_raw"))
    }

    func testBaroDecode() throws {
        let side = WireFixtures.sidecar("logframes/baro_bmp585_12.bin")
        let b = try BMP585Data(from: WireFixtures.data("logframes/baro_bmp585_12.bin"))
        XCTAssertEqual(Int64(b.time_us), side.i64("time_us"))
        XCTAssertEqual(Int(b.temp_q16), side.int("temp_q16"))
        XCTAssertEqual(Int64(b.press_q6), side.i64("press_q6"))
    }

    func testImuDecode() throws {
        let side = WireFixtures.sidecar("logframes/imu_ism6_22.bin")
        let m = try ISM6HG256Data(from: WireFixtures.data("logframes/imu_ism6_22.bin"))
        XCTAssertEqual(Int64(m.time_us), side.i64("time_us"))  // exactly 2^31
        XCTAssertEqual([Int(m.acc_low_x), Int(m.acc_low_y), Int(m.acc_low_z)],
                       side.ints("acc_low_raw"))
        XCTAssertEqual([Int(m.acc_high_x), Int(m.acc_high_y), Int(m.acc_high_z)],
                       side.ints("acc_high_raw"))
        XCTAssertEqual([Int(m.gyro_x), Int(m.gyro_y), Int(m.gyro_z)],
                       side.ints("gyro_raw"))
    }

    func testMmcMagDecode() throws {
        let side = WireFixtures.sidecar("logframes/mag_mmc5983_16.bin")
        let m = try MMC5983MAData(from: WireFixtures.data("logframes/mag_mmc5983_16.bin"))
        XCTAssertEqual(Int64(m.time_us), side.i64("time_us"))
        // mag_x > 0x20000 by design — the 18-bit-mask trap for converters.
        XCTAssertEqual(Int64(m.mag_x), side.i64("mag_x"))
        XCTAssertEqual(Int64(m.mag_y), side.i64("mag_y"))
        XCTAssertEqual(Int64(m.mag_z), side.i64("mag_z"))
    }

    func testIisMagDecode() throws {
        let side = WireFixtures.sidecar("logframes/mag_iis2mdc_10.bin")
        let m = try IIS2MDCData(from: WireFixtures.data("logframes/mag_iis2mdc_10.bin"))
        XCTAssertEqual(Int64(m.time_us), side.i64("time_us"))
        XCTAssertEqual([Int(m.mag_x), Int(m.mag_y), Int(m.mag_z)],
                       [side.int("mag_x"), side.int("mag_y"), side.int("mag_z")])
    }

    // MARK: - NonSensor length ladder (43/44/48/50)

    private func assertNonSensorBase(_ n: NonSensorData, _ side: [String: Any],
                                     file: StaticString = #filePath, line: UInt = #line) {
        XCTAssertEqual(Int64(n.time_us), side.i64("time_us"), file: file, line: line)
        XCTAssertEqual([Int(n.q0), Int(n.q1), Int(n.q2), Int(n.q3)],
                       [side.int("q0"), side.int("q1"), side.int("q2"), side.int("q3")],
                       file: file, line: line)
        XCTAssertEqual(Int(n.roll_cmd), side.int("roll_cmd"), file: file, line: line)
        XCTAssertEqual([Int(n.e_pos), Int(n.n_pos), Int(n.u_pos)],
                       [side.int("e_pos"), side.int("n_pos"), side.int("u_pos")],
                       file: file, line: line)
        XCTAssertEqual([Int(n.e_vel), Int(n.n_vel), Int(n.u_vel)],
                       [side.int("e_vel"), side.int("n_vel"), side.int("u_vel")],
                       file: file, line: line)
        XCTAssertEqual(Int(n.flags), side.int("flags"), file: file, line: line)
        XCTAssertEqual(Int(n.rocket_state), side.int("rocket_state"), file: file, line: line)
        XCTAssertEqual(Int(n.baro_alt_rate_dmps), side.int("baro_alt_rate_dmps"),
                       file: file, line: line)
        XCTAssertEqual(Int(n.pyro_status), side.int("pyro_status"), file: file, line: line)
    }

    func testNonSensorLadder() throws {
        for rel in ["logframes/nonsensor_43.bin", "logframes/nonsensor_44.bin",
                    "logframes/nonsensor_48.bin", "logframes/nonsensor_50.bin"] {
            let side = WireFixtures.sidecar(rel)
            let n = try NonSensorData(from: WireFixtures.data(rel))
            assertNonSensorBase(n, side)

            let present = side.int("present_bytes")
            // 43-byte logs decode apogee_flags as 0 (backwards-compat rule).
            XCTAssertEqual(Int(n.apogee_flags), present >= 44 ? side.int("apogee_flags") : 0,
                           "apogee_flags @ \(rel)")
            // #529 nil-vs-value distinction: ekf_ticks only exists at 50 B —
            // 44/48-byte frames must yield nil, never 0.
            if present >= 50 {
                XCTAssertEqual(n.ekf_ticks.map(Int.init), side.int("ekf_ticks"), "ekf_ticks @ \(rel)")
            } else {
                XCTAssertNil(n.ekf_ticks, "ekf_ticks must be nil @ \(rel)")
            }
        }
    }

    // MARK: - MagCal 0xCA ladder (22/26/32/36)

    func testMagCalLadder() {
        for rel in ["fileops/magcal_0xCA_len22.bin", "fileops/magcal_0xCA_len26.bin",
                    "fileops/magcal_0xCA_len32.bin", "fileops/magcal_0xCA_len36.bin"] {
            let side = WireFixtures.sidecar(rel)
            // Fixtures are full frames; decode() takes bytes AFTER the 0xCA.
            let frame = [UInt8](WireFixtures.data(rel))
            XCTAssertEqual(frame.first, 0xCA, rel)
            guard let s = MagCalStatus.decode(Array(frame.dropFirst())) else {
                XCTFail("decode returned nil for \(rel)"); continue
            }
            let present = side.int("present_bytes")
            XCTAssertEqual(Int(s.coverageBins), side.int("coverage_bins"), rel)
            XCTAssertEqual(Int(s.sampleCount), side.int("sample_count"), rel)
            XCTAssertEqual([Int(s.offsetX), Int(s.offsetY), Int(s.offsetZ)],
                           [side.int("offset_x"), side.int("offset_y"), side.int("offset_z")], rel)
            XCTAssertEqual(s.instantaneousFieldUT,
                           Float(side.int("inst_field_uT_x10")) / 10.0, rel)
            // Trailer defaults: absent rungs decode as 0, by contract.
            XCTAssertEqual(Int64(s.coverageMask),
                           present >= 26 ? side.i64("coverage_mask") : 0, rel)
            XCTAssertEqual(Int64(s.partialMask),
                           present >= 36 ? side.i64("partial_mask") : 0, rel)
            if present >= 32 {
                XCTAssertEqual(s.liveX_uT, Float(side.int("inst_x_lsb")) * 0.15, rel)
                XCTAssertEqual(s.liveY_uT, Float(side.int("inst_y_lsb")) * 0.15, rel)
                XCTAssertEqual(s.liveZ_uT, Float(side.int("inst_z_lsb")) * 0.15, rel)
            }
        }
    }

    // MARK: - StatusQuery version ladder (v3/v4/v5)

    func testStatusQueryLadder() throws {
        for rel in ["logframes/statusquery_v3_26.bin", "logframes/statusquery_v4_28.bin",
                    "logframes/statusquery_v5_41.bin"] {
            let side = WireFixtures.sidecar(rel)
            let q = try OutStatusQueryData(from: WireFixtures.data(rel))
            XCTAssertEqual(Int(q.ism6_low_g_fs_g), side.int("ism6_low_g_fs_g"), rel)
            XCTAssertEqual(Int(q.ism6_high_g_fs_g), side.int("ism6_high_g_fs_g"), rel)
            XCTAssertEqual(Int(q.ism6_gyro_fs_dps), side.int("ism6_gyro_fs_dps"), rel)
            XCTAssertEqual(Int(q.ism6_rot_z_cdeg), side.int("ism6_rot_z_cdeg"), rel)
            XCTAssertEqual(Int(q.mmc_rot_z_cdeg), side.int("mmc_rot_z_cdeg"), rel)
            XCTAssertEqual(Int(q.format_version), side.int("format_version"), rel)
            // #204 dual gate: version >= 4 AND length >= 28.
            if side.int("format_version") >= 4 {
                XCTAssertEqual(q.iis2mdc_rot_z_cdeg.map(Int.init),
                               side.int("iis2mdc_rot_z_cdeg"), rel)
            } else {
                XCTAssertNil(q.iis2mdc_rot_z_cdeg,
                             "iis rotation must be nil pre-v4 @ \(rel)")
            }
        }
    }

    // MARK: - FlightSettings version ladder (v1/v2/v3/v5/v6)

    func testFlightSettingsLadder() throws {
        for rel in ["logframes/flightsettings_v1_188.bin", "logframes/flightsettings_v2_200.bin",
                    "logframes/flightsettings_v3_208.bin", "logframes/flightsettings_v5_210.bin",
                    "logframes/flightsettings_v6_219.bin"] {
            let side = WireFixtures.sidecar(rel)
            let s = try FlightSettingsData(from: WireFixtures.data(rel))

            // Base (v1) fields — present at every rung.
            XCTAssertEqual(Int64(s.time_us), side.i64("time_us"), rel)
            XCTAssertEqual(Int(s.version), side.int("version"), rel)
            XCTAssertEqual(Int(s.flags), side.int("flags"), rel)
            XCTAssertEqual(Int(s.roll_delay_ms), side.int("roll_delay_ms"), rel)
            XCTAssertEqual(s.kp, side.f32("kp"), rel)
            XCTAssertEqual(s.ki, side.f32("ki"), rel)
            XCTAssertEqual(s.kd, side.f32("kd"), rel)
            XCTAssertEqual(s.d_lpf_hz, side.f32("d_lpf_hz"), rel)
            XCTAssertEqual(s.min_cmd_deg, side.f32("min_cmd_deg"), rel)
            XCTAssertEqual(s.max_cmd_deg, side.f32("max_cmd_deg"), rel)
            XCTAssertEqual(s.kp_angle, side.f32("kp_angle"), rel)
            XCTAssertEqual(s.kp_angle_rate_cap_dps, side.f32("kp_angle_rate_cap_dps"), rel)
            XCTAssertEqual(s.gs_v_ref, side.f32("gs_v_ref"), rel)
            XCTAssertEqual(s.gs_v_min, side.f32("gs_v_min"), rel)
            XCTAssertEqual(s.gs_scale_cap, side.f32("gs_scale_cap"), rel)
            XCTAssertEqual(s.roll_rate_set_point, side.f32("roll_rate_set_point"), rel)
            XCTAssertEqual(Int(s.ism6_low_g_fs_g), side.int("ism6_low_g_fs_g"), rel)
            XCTAssertEqual(Int(s.ism6_high_g_fs_g), side.int("ism6_high_g_fs_g"), rel)
            XCTAssertEqual(Int(s.ism6_gyro_fs_dps), side.int("ism6_gyro_fs_dps"), rel)
            XCTAssertEqual(s.servo_bias_us.map(Int.init), side.ints("servo_bias_us"), rel)
            XCTAssertEqual(Int(s.servo_hz), side.int("servo_hz"), rel)
            XCTAssertEqual(Int(s.servo_min_us), side.int("servo_min_us"), rel)
            XCTAssertEqual(Int(s.servo_max_us), side.int("servo_max_us"), rel)
            XCTAssertEqual(Int(s.camera_type), side.int("camera_type"), rel)
            XCTAssertEqual(s.fw_git_sha, side.str("fw_git_sha"), rel)
            XCTAssertEqual(Int(s.num_waypoints), side.int("roll_profile_num_waypoints"), rel)

            // Version tails — the dual (version, length) gates.
            let present = side.int("present_bytes")
            if present >= 200 {
                XCTAssertEqual(s.b2r_code.map(Int.init), side.int("b2r_code"), rel)
                XCTAssertEqual(s.b2r_mode.map(Int.init), side.int("b2r_mode"), rel)
                XCTAssertEqual(s.b2r_residual_deg,
                               Float(side.int("b2r_residual_cdeg")) / 100.0, rel)
            } else {
                XCTAssertNil(s.b2r_code, rel)
            }
            if present >= 208 {
                XCTAssertEqual(s.fin_min_deg, side.f32("fin_min_deg"), rel)
                XCTAssertEqual(s.fin_max_deg, side.f32("fin_max_deg"), rel)
            } else {
                XCTAssertNil(s.fin_min_deg, rel)
            }
            if present >= 210 {
                XCTAssertEqual(s.ism6_update_rate_hz.map(Int.init),
                               side.int("ism6_update_rate_hz"), rel)
            } else {
                XCTAssertNil(s.ism6_update_rate_hz, rel)
            }
            if present >= 219 {
                XCTAssertEqual(s.guid_tgt_e_m, side.f32("guid_tgt_e_m"), rel)
                XCTAssertEqual(s.guid_tgt_n_m, side.f32("guid_tgt_n_m"), rel)
                XCTAssertEqual(s.guid_tgt_src.map(Int.init), side.int("guid_tgt_src"), rel)
            } else {
                XCTAssertNil(s.guid_tgt_e_m, rel)
            }
        }
    }
}
