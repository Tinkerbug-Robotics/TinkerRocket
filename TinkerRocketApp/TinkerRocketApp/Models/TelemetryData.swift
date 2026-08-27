//
//  TelemetryData.swift
//  TinkerRocketApp
//
//  Data model matching ESP32 telemetry JSON structure
//

import Foundation

struct TelemetryData: Codable {
    var soc: Float?                   // Battery state of charge %
    var current: Float?               // Battery current mA
    // #850: camera / servo high-side-switch load currents, AMPS (not mA — the
    // battery `current` above is the shunt; these are load rails). nil means
    // the key was absent, i.e. this board has no TPS22811 monitor fitted or
    // the read failed. Render as "--", never as 0.0 A: a camera that is off
    // and a camera with no monitor look identical at zero, and only one of
    // those is something we measured.
    var cam_current: Float?           // Camera rail current A  (nil = not measured)
    var servo_current: Float?         // Servo rail current A   (nil = not measured)
    var voltage: Float?               // Battery voltage V
    var latitude: Double?             // GPS latitude degrees
    var longitude: Double?            // GPS longitude degrees
    var gdop: Float?                  // GPS dilution of precision
    var num_sats: Int = 0             // Number of GPS satellites
    var state: String = "UNKNOWN"     // Rocket state
    var active_file: String = ""
    var rx_kbs: Float?                // I2C RX rate kB/s
    var wr_kbs: Float?                // Flash write rate kB/s
    var frames_rx: UInt32 = 0         // Frames received
    var frames_drop: UInt32 = 0       // Frames dropped
    var max_alt_m: Float?             // Maximum altitude meters
    var max_speed_mps: Float?         // Maximum speed m/s
    var pressure_alt: Float?          // Barometric pressure altitude meters
    var altitude_rate: Float?         // Vertical rate m/s
    var gnss_alt: Float?              // GNSS altitude meters (from ECEF, base station only)

    // EKF launch-relative ENU velocity (#191).  nil = firmware predates the
    // fields or no FC data — the ascent landing prediction gates on these.
    var vel_e: Float?                 // East m/s
    var vel_n: Float?                 // North m/s
    var vel_u: Float?                 // Up m/s (EKF channel; altitude_rate is the baro KF)

    // IMU data (ISM6HG256)
    var low_g_x: Float?               // Low-G accelerometer X m/s²
    var low_g_y: Float?               // Low-G accelerometer Y m/s²
    var low_g_z: Float?               // Low-G accelerometer Z m/s²
    var high_g_x: Float?              // High-G accelerometer X m/s²
    var high_g_y: Float?              // High-G accelerometer Y m/s²
    var high_g_z: Float?              // High-G accelerometer Z m/s²
    var gyro_x: Float?                // Gyroscope X deg/s
    var gyro_y: Float?                // Gyroscope Y deg/s
    var gyro_z: Float?                // Gyroscope Z deg/s

    // Attitude (from FlightComputer onboard estimation)
    var roll_cmd: Float?              // Roll command degrees (PID output)
    var q0: Float?                    // Quaternion w (scalar-first, body-to-NED)
    var q1: Float?                    // Quaternion x
    var q2: Float?                    // Quaternion y
    var q3: Float?                    // Quaternion z

    // Roll/pitch/yaw derived from quaternion (not sent over BLE)
    // Guard: reject malformed quaternions (e.g. all zeros from uninitialized EKF)
    var roll: Float? {
        guard let w = q0, let x = q1, let y = q2, let z = q3 else { return nil }
        let norm = w*w + x*x + y*y + z*z
        guard norm > 0.1 && norm < 2.0 else { return nil }
        // Body-Z roll, matching the FC roll controller (flight_computer/main.cpp:
        // actual_roll_deg = -atan2(z_east, z_north)). Stays well-defined when the
        // rocket is vertical, unlike the ZYX-Euler roll which gimbal-locks and
        // cycles near pitch ±90°. Keep in sync with SensorConverter.convertNonSensor.
        let zNorth = 2.0 * (x * z + w * y)
        let zEast  = 2.0 * (y * z - w * x)
        return -atan2(zEast, zNorth) * 180.0 / .pi
    }
    var pitch: Float? {
        guard let w = q0, let x = q1, let y = q2, let z = q3 else { return nil }
        let norm = w*w + x*x + y*y + z*z
        guard norm > 0.1 && norm < 2.0 else { return nil }
        let sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0 { return copysign(90.0, sinp) }
        return asin(sinp) * 180.0 / .pi
    }
    var yaw: Float? {
        guard let w = q0, let x = q1, let y = q2, let z = q3 else { return nil }
        let norm = w*w + x*x + y*y + z*z
        guard norm > 0.1 && norm < 2.0 else { return nil }
        let siny = 2.0 * (w * z + x * y)
        let cosy = 1.0 - 2.0 * (y * y + z * z)
        return atan2(siny, cosy) * 180.0 / .pi
    }

    // LoRa signal quality (base station only)
    var rssi: Float?                  // LoRa RSSI dBm
    var snr: Float?                   // LoRa SNR dB
    // #150: hop-state surface.  hop_channel only arrives while the BS is
    // following a hop schedule; netid_drops only once the BS has dropped
    // packets on a network-id mismatch (a rising count = a device is on
    // the wrong network id — the failure that used to be silent).
    var hop_channel: Int?
    var netid_drops: Int?
    /// "szd" — LoRa frames dropped because the rocket and this base station
    /// disagree about the protocol (#570, #838 item 4).  Sibling of
    /// `netid_drops` with the same recency window but a different fault: nidd
    /// means somebody else's traffic, szd means OUR rocket is unreadable
    /// because the two were flashed from different builds.  Since #925 it also
    /// counts frames whose LORA_PROTO_VERSION nibble is not ours.
    ///
    /// Emitted by the base station since #570; neither app decoded it, so the
    /// blackout it exists to explain stayed silent.
    var size_drops: Int?

    // Base station (base station only)
    var bs_soc: Float?                // Base station SOC %
    var bs_voltage: Float?            // Base station voltage V
    var bs_current: Float?            // Base station current mA
    // Seconds remaining until the BS silence-timeout closes the active log.
    // Sent only when bs_logging_active is true (BS omits the JSON key
    // otherwise) — nil here = no countdown to show.
    var bs_log_silence_remaining_s: UInt16?

    // #390: rocket board→rocket mounting orientation relayed over LoRa
    // flags2, packed (mode << 5) | code ("imo" key).  nil = not reported:
    // pre-#390 rocket firmware, FC not up yet, or the key fell off the
    // MTU-trimmed tail.  Direct links ignore this and use the richer
    // imu_orient config message on BLEDevice.
    var imu_orient_packed: Int?

    // Packed flight-status bits ("fs" JSON key, decoded in init).  Replaces
    // 8 separate boolean keys to keep the BLE notify payload under MTU —
    // see TR_BLE_To_APP.cpp:buildTelemetryJSON.  Bit layout must stay in
    // sync with the firmware side.
    var flight_status_bits: Int = 0
    var launch_flag: Bool       { (flight_status_bits & 0x01) != 0 }
    var vel_apo: Bool           { (flight_status_bits & 0x02) != 0 }
    var alt_apo: Bool           { (flight_status_bits & 0x04) != 0 }
    var landed_flag: Bool       { (flight_status_bits & 0x08) != 0 }
    var pwr_pin_on: Bool        { (flight_status_bits & 0x10) != 0 }
    var camera_recording: Bool  { (flight_status_bits & 0x20) != 0 }
    var logging_active: Bool    { (flight_status_bits & 0x40) != 0 }
    var bs_logging_active: Bool { (flight_status_bits & 0x80) != 0 }
    // #393: simulated flight in progress, reported by the rocket (NSF_SIM_ACTIVE
    // -> fs bit 8).  Drives the sim banner / Stop-sim control so it survives BLE
    // reconnects, unlike the client-side simLaunched latch.
    var sim_active: Bool        { (flight_status_bits & 0x100) != 0 }
    // #191: motor burnout detected (fs bit 9).  Gates the ascent landing
    // prediction to post-burnout coast, where the ballistic model is valid.
    var burnout_flag: Bool      { (flight_status_bits & 0x200) != 0 }

    // #968: LATCHED "this flight is past apogee" (fs bit 10).
    //
    // `alt_apo` and `vel_apo` above are LIVE VOTES, not phase latches: the FC
    // builds them from a leaky counter whose baro test requires
    // `alt_est > 15.0f`, so both CLEAR below ~15 m AGL and every consumer that
    // read them as "we are past apogee" silently flipped back to "pre-apogee"
    // for the last seconds of every flight (measured: 130.96 s -> 138.84 s on
    // the 2026-08-27 sim flight).  That is what let a stale burnout be
    // announced just before landing (#964), and the same clearing caused #235.
    //
    // The firmware HAS a latched master vote (`NSF2_MASTER_APOGEE`) but does
    // not put it on the wire, and there is no room to add it: LoRa
    // `flags_state` is fully allocated and `num_sats` has only 6 bits left
    // against a real 0-40 range.  So the app latches it itself, into the bit
    // position the firmware would use — if a future protocol version does send
    // it, this reads through unchanged.
    static let pastApogeeBit = 0x400
    var past_apogee: Bool       { (flight_status_bits & Self.pastApogeeBit) != 0 }

    // Source rocket identity (base station relay only, nil for direct BLE)
    var source_rocket_id: Int?        // rocket_id from LoRa header
    var source_unit_name: String?     // rocket unit name from LoRa beacon

    // Telemetry freshness status (#95).  Sent by the BS in periodic-push
    // payloads; absent from RX-path payloads (which are always live).  iOS
    // treats a missing "ds" as live.
    //
    // #831: a DIRECT rocket connection also sends it — SYNCING until the OC
    // has seen the FC's first NonSensorData frame, so stale zeroed continuity
    // cannot render green.  So .syncing does NOT imply "base station".  See
    // DashboardVisibility: the state banner must stay visible through it.
    enum DataStatus: Int, Codable { case live = 0, stale = 1, syncing = 2 }
    var data_status: DataStatus = .live
    var data_age_ms: UInt32 = 0      // only meaningful when .stale

    // #282: base station sets "tr":1 when a worst-case in-flight frame was
    // trimmed to fit the BLE MTU window — low-priority tail fields (link
    // stats, unit name, active filename, some IMU detail) were dropped so the
    // recovery/dashboard-critical fields still got through.  The frame is
    // still valid; this just flags that it's partial so the absence of those
    // fields reads as "trimmed for bandwidth", not "sensor offline".
    var fields_trimmed: Bool = false

    // Pyro channel status (packed bitfield from "ps" JSON key, decoded in init).
    // New PCB: bit 0 = global armed (single shared ARM FET), then per-channel
    // (cont, fired) pairs for channels 1..4. 9 bits total.
    var pyro_status_bits: Int = 0
    var pyro_armed: Bool { (pyro_status_bits & 0x001) != 0 }
    var pyro1_cont:  Bool { (pyro_status_bits & 0x002) != 0 }
    var pyro1_fired: Bool { (pyro_status_bits & 0x004) != 0 }
    var pyro2_cont:  Bool { (pyro_status_bits & 0x008) != 0 }
    var pyro2_fired: Bool { (pyro_status_bits & 0x010) != 0 }
    var pyro3_cont:  Bool { (pyro_status_bits & 0x020) != 0 }
    var pyro3_fired: Bool { (pyro_status_bits & 0x040) != 0 }
    var pyro4_cont:  Bool { (pyro_status_bits & 0x080) != 0 }
    var pyro4_fired: Bool { (pyro_status_bits & 0x100) != 0 }
    func pyroCont(channel: Int) -> Bool {
        switch channel { case 1: return pyro1_cont; case 2: return pyro2_cont
                          case 3: return pyro3_cont; case 4: return pyro4_cont
                          default: return false }
    }
    func pyroFired(channel: Int) -> Bool {
        switch channel { case 1: return pyro1_fired; case 2: return pyro2_fired
                          case 3: return pyro3_fired; case 4: return pyro4_fired
                          default: return false }
    }

    // ── Sensor health scorecard (#303) ────────────────────────────────────
    // Packed "h" bitfield: FC sets sensors + EKF, OC sets battery.  2 bits per
    // item; layout mirrors RocketComputerTypes.h (SH_*_SHIFT).  0 = nothing
    // reported (older firmware or a BS self-frame) → card hidden.
    var sensor_health: Int = 0
    enum SensorHealth: Int { case na = 0, ok = 1, degraded = 2, bad = 3
        var label: String {
            switch self { case .na: return "N/A"; case .ok: return "OK"
                          case .degraded: return "DEGRADED"; case .bad: return "BAD" }
        }
    }
    private func shState(_ shift: Int) -> SensorHealth {
        SensorHealth(rawValue: (sensor_health >> shift) & 0x3) ?? .na
    }
    var baroHealth: SensorHealth { shState(0) }
    var imuHealth:  SensorHealth { shState(2) }
    var ekfHealth:  SensorHealth { shState(4) }   // filter health: init + isHealthy + covariance converged
    var magHealth:  SensorHealth { shState(6) }   // advisory only — never gates go/no-go
    var gnssHealth: SensorHealth { shState(8) }
    var battHealth: SensorHealth { shState(10) }
    var storageHealth: SensorHealth { shState(20) }   // #281/#278: flight-log NAND — BAD = full/failing, won't record
    // #557: GNSS-absent degraded flight (shift 22) — DISTINCT from gnssHealth
    // (fix health, shift 8).  .bad = the FC initialized the EKF on the baro+IMU
    // path because the module failed bring-up (dead/deaf UART), so there is no
    // absolute position and guidance is off.  Rides sensor_health, so it reaches
    // the app on both the direct-BLE and base-station-relay paths.
    var gnssAbsentMode: Bool { shState(22) == .bad }
    func pyroHealth(channel: Int) -> SensorHealth {   // channel 1...4; .na = not configured
        guard (1...4).contains(channel) else { return .na }
        return shState(12 + (channel - 1) * 2)
    }
    /// MEASURED continuity (SH_PYRO_MEAS_SHIFT, bits 24-30), reported for every
    /// channel whether or not it is configured for flight — unlike pyroHealth,
    /// which is config-gated because it feeds the go/no-go rollup.  This is the
    /// ground-test answer: .na = never tested this session, .ok = continuity
    /// present, .bad = tested and open.  Never .degraded.
    ///
    /// Returns nil when the rocket predates this field (all four read NA), so
    /// callers fall back to pyroHealth instead of showing "never tested"
    /// forever against older firmware.
    func pyroMeasuredContinuity(channel: Int) -> SensorHealth? {
        guard (1...4).contains(channel) else { return nil }
        let anyReported = (1...4).contains { shState(24 + ($0 - 1) * 2) != .na }
        guard anyReported else { return nil }
        return shState(24 + (channel - 1) * 2)
    }
    var hasSensorHealth: Bool { sensor_health != 0 }

    // Rows for the pre-launch health card.  Core sensors always shown; a pyro
    // channel appears only when configured for the flight (state != .na).
    struct SensorHealthRow: Identifiable { let name: String; let state: SensorHealth; var id: String { name } }
    var sensorHealthRows: [SensorHealthRow] {
        var rows = [
            SensorHealthRow(name: "Baro",    state: baroHealth),
            SensorHealthRow(name: "IMU",     state: imuHealth),
            SensorHealthRow(name: "EKF",     state: ekfHealth),
            SensorHealthRow(name: "GNSS",    state: gnssHealth),
            SensorHealthRow(name: "Battery", state: battHealth),
            SensorHealthRow(name: "Mag",     state: magHealth),
        ]
        if storageHealth != .na {   // OC-reported only; absent on older firmware / BS self-frames
            rows.append(SensorHealthRow(name: "Storage", state: storageHealth))
        }
        for ch in 1...4 where pyroHealth(channel: ch) != .na {
            rows.append(SensorHealthRow(name: "Pyro \(ch)", state: pyroHealth(channel: ch)))
        }
        return rows
    }

    // Go/no-go rollup (#303).  Red = a hard fault waiting won't fix (baro/IMU/
    // battery BAD, a full/failing flight-log store, or a configured pyro with no
    // continuity).  Green needs every required item OK — including EKF
    // initialized/converged and a GNSS fix.  Mag is advisory and never gates.
    // Storage is OC-reported: absent (N/A) on older firmware → ignored, not red.
    // Amber = anything in between.
    enum FlightReadiness { case unknown, ready, caution, notReady
        var label: String {
            switch self { case .unknown:  return "Waiting for telemetry…"
                          case .ready:    return "Ready to fly"
                          case .caution:  return "Not ready — check sensors"
                          case .notReady: return "Do not fly" }
        }
    }
    var flightReadiness: FlightReadiness {
        guard hasSensorHealth else { return .unknown }
        let hardFault = [baroHealth, imuHealth, battHealth, storageHealth].contains(.bad)
            || (1...4).contains { pyroHealth(channel: $0) == .bad }
        if hardFault { return .notReady }
        var mustBeOK = [baroHealth, imuHealth, battHealth, ekfHealth, gnssHealth]
        if storageHealth != .na { mustBeOK.append(storageHealth) }   // #281/#278: low/full space gates green
        for ch in 1...4 where pyroHealth(channel: ch) != .na { mustBeOK.append(pyroHealth(channel: ch)) }
        return mustBeOK.allSatisfy { $0 == .ok } ? .ready : .caution
    }

    // ── FC boot progress ──────────────────────────────────────────────────
    // The FC spends ~10 s in setup_fc() before its state machine runs, and it
    // transmits no NonSensorData in that window — so the OC's zeroed
    // latest_non_sensor makes rocket_state read 0 == INITIALIZATION, and "FC
    // rail off", "FC still booting" and "FC genuinely initializing" all
    // rendered as the same bare INIT.  The FC now reports each step as it
    // STARTS (FC_BOOT_STATUS_MSG), and the OC forwards it ONLY while the FC is
    // still booting — so the absence of "bs" is itself information: the FC has
    // not spoken at all this session (see RocketStateView).
    var fc_boot_step: Int?          // "bs": FcBootStep now starting (raw wire value)
    var fc_boot_ms: Int?            // "bt": ms since FC reset, saturating at 65535
    var fc_boot_degraded: Int = 0   // "bd": FCB_DEG_* bits; key omitted on a normal boot

    /// setup_fc()'s reportable steps (RocketComputerTypes.h FcBootStep).  The
    /// wire enum is append-only, so an unrecognized value must still render —
    /// a newer FC may report a step this build has never heard of.
    enum FcBootStep: Int {
        case links = 0, nvs, sensors, gnss, servos, complete
        var label: String {
            switch self {
            case .links:    return "Linking to flight computer"
            case .nvs:      return "Loading saved settings"
            case .sensors:  return "Starting sensors"
            case .gnss:     return "Starting GNSS"
            case .servos:   return "Starting servos"
            case .complete: return "Flight computer ready"
            }
        }
    }

    /// One frame's view of the FC boot, rendered as the secondary line under
    /// the rocket state.  Kept a value type so the view rebuilds it per frame.
    struct FcBootProgress: Equatable {
        let step: Int          // raw "bs" — may outrun the FcBootStep this build knows
        let elapsedMs: Int     // "bt"
        let degradedBits: Int  // "bd"

        var knownStep: FcBootStep? { FcBootStep(rawValue: step) }

        // FCB_DEG_* (RocketComputerTypes.h).  A set bit means the step finished
        // DEGRADED and boot CONTINUED — nothing stopped, which is exactly why
        // it has to be said out loud: a dead receiver or an unread config
        // otherwise only shows up as a missing fix or wrong settings in flight.
        static let degSensors = 0x01
        static let degGnss    = 0x02
        static let degServos  = 0x04
        static let degNvs     = 0x08

        var degradedSubsystems: [String] {
            var names: [String] = []
            if degradedBits & Self.degSensors != 0 { names.append("sensors") }
            if degradedBits & Self.degGnss    != 0 { names.append("GNSS") }
            if degradedBits & Self.degServos  != 0 { names.append("servos") }
            if degradedBits & Self.degNvs     != 0 { names.append("saved settings") }
            return names
        }

        /// How long the app watches this SAME step before calling it stuck.
        /// PER STEP, because their legitimate durations differ by four orders
        /// of magnitude — measured on the bench 2026-08-19 (FC boot, healthy
        /// board): LINKS 3 ms, NVS 12 ms, SENSORS ~21 s, GNSS 11 ms,
        /// SERVOS 4.2 s, setup_fc complete at 25.4 s.
        ///
        /// SENSORS is the outlier and must not be judged by the others: GNSS
        /// bootstrap runs INSIDE sensor_collector.begin(), which blocks until a
        /// ~35 s deadline when the module is unresponsive, so a healthy board
        /// legitimately sits here for tens of seconds. A single 12 s threshold
        /// (what this shipped with before the bench run) flagged EVERY normal
        /// boot as stalled — the precise way to train an operator to ignore the
        /// one warning this feature exists to give.
        static func stallAfter(_ step: FcBootStep?) -> TimeInterval {
            switch step {
            case .sensors: return 45   // ~35 s GNSS bring-up deadline + margin
            case .servos:  return 15   // neutral settle measured at 4.2 s
            default:       return 8    // links/NVS/GNSS are all milliseconds
            }
        }

        /// `dwell` is the app's own time on the current step, NOT elapsedMs:
        /// "bt" is stamped when the step STARTS and then repeats unchanged in
        /// every frame, so a wedged boot and a just-started one carry the same
        /// "bt" — only the dwell separates them.  FCB_COMPLETE is exempt: the
        /// FC has finished and the app is just waiting for the first real
        /// NonSensorData to arrive.
        func isStalled(dwell: TimeInterval) -> Bool {
            knownStep != .complete && dwell >= Self.stallAfter(knownStep)
        }

        /// The secondary line.  `dwell` defaults to 0 = "just seen", so a caller
        /// with no clock of its own still gets the step and any degraded bits.
        func line(dwell: TimeInterval = 0) -> String {
            var parts = [knownStep?.label ?? "Starting up…"]
            if isStalled(dwell: dwell) {
                // FCB_SENSORS covers GNSS bring-up too — sensor_collector.begin()
                // blocks to its ~35 s deadline when the module is dead — so a boot
                // parked there IS the dead-GNSS stall, and naming it is the
                // difference between "wait longer" and "check the receiver".
                parts.append(knownStep == .sensors
                    ? "no progress for \(Int(dwell))s, GNSS may be dead"
                    : "no progress for \(Int(dwell))s")
            }
            if !degradedSubsystems.isEmpty {
                parts.append("degraded: " + degradedSubsystems.joined(separator: ", "))
            }
            return parts.joined(separator: " — ")
        }
    }

    /// nil on every frame from a running FC — "bs" rides only the boot window.
    var fcBootProgress: FcBootProgress? {
        guard let step = fc_boot_step else { return nil }
        return FcBootProgress(step: step,
                              elapsedMs: fc_boot_ms ?? 0,
                              degradedBits: fc_boot_degraded)
    }

    // Short JSON keys → Swift property names (saves ~150 bytes in BLE payload)
    enum CodingKeys: String, CodingKey {
        case soc
        case current = "cur"
        case cam_current = "ccur"      // #850
        case servo_current = "scur"    // #850
        case voltage = "vol"
        case latitude = "lat"
        case longitude = "lon"
        case num_sats = "nsat"
        case state = "st"
        case active_file = "af"
        case rx_kbs = "rxk"
        case wr_kbs = "wrk"
        case frames_rx = "frx"
        case frames_drop = "fdr"
        case max_alt_m = "malt"
        case max_speed_mps = "mspd"
        case pressure_alt = "palt"
        case altitude_rate = "arate"
        case gnss_alt = "galt"
        case vel_e = "ve"      // #191 EKF ENU velocity
        case vel_n = "vn"
        case vel_u = "vu"
        case low_g_x = "lx"
        case low_g_y = "ly"
        case low_g_z = "lz"
        case high_g_x = "hx"
        case high_g_y = "hy"
        case high_g_z = "hz"
        case gyro_x = "gx"
        case gyro_y = "gy"
        case gyro_z = "gz"
        case roll_cmd = "rcmd"
        case q0, q1, q2, q3
        case rssi, snr
        case hop_channel = "hch"
        case netid_drops = "nidd"
        case size_drops = "szd"
        case bs_soc = "bsoc"
        case bs_voltage = "bvol"
        case bs_current = "bcur"
        case bs_log_silence_remaining_s = "slrm"
        case imu_orient_packed = "imo"
        // Packed flight-status bitfield (replaces lnch/vapo/aapo/land/pwr/
        // cam/log/bslog).  Bit layout mirrors TR_BLE_To_APP.cpp.
        case flight_status_bits = "fs"
        case pyro_status_bits = "ps"  // packed bitfield: b0=armed (global), then (cont,fired) per channel 1..4
        case sensor_health = "h"      // #303 scorecard: 2 bits/sensor, see RocketComputerTypes.h
        case source_rocket_id = "rid"
        case source_unit_name = "run"
        case data_status = "ds"        // #95
        case data_age_ms = "age"       // #95
        case fields_trimmed = "tr"     // #282: frame trimmed to fit MTU
        // FC boot progress, Tier 1 right after "st" because it QUALIFIES the
        // state; all three are absent once the FC is running.
        case fc_boot_step = "bs"
        case fc_boot_ms = "bt"
        case fc_boot_degraded = "bd"
    }

    // Custom decoder: non-optional fields with defaults need decodeIfPresent
    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        // #293: tolerate a high-value field emitted as a float or string instead
        // of an int — firmware-contract drift on one field would otherwise throw
        // and the catch discards the WHOLE telemetry frame. Mirrors the defensive
        // config decode. Float fields already accept int JSON, so only the ints
        // below need this.
        func flexInt(_ key: CodingKeys) -> Int? {
            if let i = try? c.decodeIfPresent(Int.self, forKey: key) { return i }
            if let d = try? c.decodeIfPresent(Double.self, forKey: key) { return Int(d) }
            if let s = try? c.decodeIfPresent(String.self, forKey: key) {
                return Int(s) ?? Double(s).map { Int($0) }
            }
            return nil
        }
        soc = try c.decodeIfPresent(Float.self, forKey: .soc)
        current = try c.decodeIfPresent(Float.self, forKey: .current)
        // #850: decodeIfPresent, so an absent key stays nil ("no monitor
        // fitted") rather than becoming 0 ("measured zero amps").
        cam_current = try c.decodeIfPresent(Float.self, forKey: .cam_current)
        servo_current = try c.decodeIfPresent(Float.self, forKey: .servo_current)
        voltage = try c.decodeIfPresent(Float.self, forKey: .voltage)
        latitude = try c.decodeIfPresent(Double.self, forKey: .latitude)
        longitude = try c.decodeIfPresent(Double.self, forKey: .longitude)
        // #571: complete the #293 hardening — every integer key goes through
        // flexInt. The stragglers below were still strict decodeIfPresent, so
        // ONE field emitted as float/string (firmware-contract drift) threw in
        // init(from:) and the caller's catch discarded the WHOLE frame — on a
        // BS-relayed stream that made the rocket vanish from the dashboard
        // instead of degrading one field.
        num_sats = flexInt(.num_sats) ?? 0
        state = try c.decodeIfPresent(String.self, forKey: .state) ?? "UNKNOWN"
        active_file = try c.decodeIfPresent(String.self, forKey: .active_file) ?? ""
        rx_kbs = try c.decodeIfPresent(Float.self, forKey: .rx_kbs)
        wr_kbs = try c.decodeIfPresent(Float.self, forKey: .wr_kbs)
        frames_rx = UInt32(clamping: flexInt(.frames_rx) ?? 0)      // #571
        frames_drop = UInt32(clamping: flexInt(.frames_drop) ?? 0)  // #571
        max_alt_m = try c.decodeIfPresent(Float.self, forKey: .max_alt_m)
        max_speed_mps = try c.decodeIfPresent(Float.self, forKey: .max_speed_mps)
        pressure_alt = try c.decodeIfPresent(Float.self, forKey: .pressure_alt)
        altitude_rate = try c.decodeIfPresent(Float.self, forKey: .altitude_rate)
        gnss_alt = try c.decodeIfPresent(Float.self, forKey: .gnss_alt)
        vel_e = try c.decodeIfPresent(Float.self, forKey: .vel_e)
        vel_n = try c.decodeIfPresent(Float.self, forKey: .vel_n)
        vel_u = try c.decodeIfPresent(Float.self, forKey: .vel_u)
        low_g_x = try c.decodeIfPresent(Float.self, forKey: .low_g_x)
        low_g_y = try c.decodeIfPresent(Float.self, forKey: .low_g_y)
        low_g_z = try c.decodeIfPresent(Float.self, forKey: .low_g_z)
        high_g_x = try c.decodeIfPresent(Float.self, forKey: .high_g_x)
        high_g_y = try c.decodeIfPresent(Float.self, forKey: .high_g_y)
        high_g_z = try c.decodeIfPresent(Float.self, forKey: .high_g_z)
        gyro_x = try c.decodeIfPresent(Float.self, forKey: .gyro_x)
        gyro_y = try c.decodeIfPresent(Float.self, forKey: .gyro_y)
        gyro_z = try c.decodeIfPresent(Float.self, forKey: .gyro_z)
        roll_cmd = try c.decodeIfPresent(Float.self, forKey: .roll_cmd)
        q0 = try c.decodeIfPresent(Float.self, forKey: .q0)
        q1 = try c.decodeIfPresent(Float.self, forKey: .q1)
        q2 = try c.decodeIfPresent(Float.self, forKey: .q2)
        q3 = try c.decodeIfPresent(Float.self, forKey: .q3)
        rssi = try c.decodeIfPresent(Float.self, forKey: .rssi)
        snr = try c.decodeIfPresent(Float.self, forKey: .snr)
        hop_channel = flexInt(.hop_channel)                          // #571
        netid_drops = flexInt(.netid_drops)                          // #571
        size_drops = flexInt(.size_drops)                            // #838 item 4
        bs_soc = try c.decodeIfPresent(Float.self, forKey: .bs_soc)
        bs_voltage = try c.decodeIfPresent(Float.self, forKey: .bs_voltage)
        bs_current = try c.decodeIfPresent(Float.self, forKey: .bs_current)
        bs_log_silence_remaining_s = flexInt(.bs_log_silence_remaining_s).map { UInt16(clamping: $0) }  // #571
        imu_orient_packed = flexInt(.imu_orient_packed)   // #293: tolerant, like fs/ps
        flight_status_bits = flexInt(.flight_status_bits) ?? 0   // #293: tolerant
        pyro_status_bits = flexInt(.pyro_status_bits) ?? 0       // #293: tolerant
        sensor_health = flexInt(.sensor_health) ?? 0                 // #571
        source_rocket_id = flexInt(.source_rocket_id)                // #571: rid is the demux key
        source_unit_name = try c.decodeIfPresent(String.self, forKey: .source_unit_name)
        // #95: missing "ds" → .live (older firmware doesn't emit it)
        let dsRaw = flexInt(.data_status) ?? 0                       // #571
        data_status = DataStatus(rawValue: dsRaw) ?? .live
        data_age_ms = UInt32(clamping: flexInt(.data_age_ms) ?? 0)   // #293: tolerant
        fields_trimmed = (flexInt(.fields_trimmed) ?? 0) != 0        // #282
        // Absent on every frame from a running FC, and "bd" is absent on a
        // normal boot as well — lenient like hch/nidd, never an error.
        fc_boot_step = flexInt(.fc_boot_step)
        fc_boot_ms = flexInt(.fc_boot_ms)
        fc_boot_degraded = flexInt(.fc_boot_degraded) ?? 0
    }

    // Default memberwise init (for creating empty telemetry)
    init() {}

    // Computed properties for display
    /// Clamped to 0...100 for display only — the stored value and the CSV keep
    /// whatever came off the wire. SOC is packed as an i16 spanning -25...125%
    /// for headroom, so an exact 0% round-trips to -0.00077 and "%.1f%%" prints
    /// it as "-0.0%", which is what a rocket on USB shows on every line.
    var socDisplay: String {
        if let soc = soc {
            // The `+ 0` is what actually kills "-0.0%", and it is not
            // redundant with the clamp: the OC prints SOC to one decimal, so
            // an exact 0% arrives as the literal -0.0, and -0.0 == 0 means
            // max(0, soc) hands it straight back. Adding positive zero is the
            // IEEE way to drop the sign.
            return String(format: "%.1f%%", min(100, max(0, soc)) + 0)
        }
        return "N/A"
    }

    /// #850: format a high-side-switch load current for display.
    ///
    /// Amps with 2 decimals at 1 A and above, whole milliamps below — the
    /// TPS22811 GIMON spread is +/-13%, so finer absolute precision would be a
    /// fiction, but that error is a stable per-board GAIN term and relative
    /// movement is faithful. Watching for a stalled servo depends on the
    /// latter, not the former.
    ///
    /// nil renders as an em dash: the key was absent, meaning this board has no
    /// monitor fitted. Never render that as 0.
    ///
    /// Android twin: `railAmpsDisplay(Float?)`.
    static func railAmpsDisplay(_ amps: Float?) -> String {
        guard let a = amps else { return "—" }
        return a >= 1.0 ? String(format: "%.2f A", a)
                        : String(format: "%.0f mA", a * 1000)
    }

    var camCurrentDisplay: String { Self.railAmpsDisplay(cam_current) }
    var servoCurrentDisplay: String { Self.railAmpsDisplay(servo_current) }

    var voltageDisplay: String {
        if let voltage = voltage {
            return String(format: "%.2f V", voltage)
        }
        return "N/A"
    }

    var currentDisplay: String {
        if let current = current {
            return String(format: "%.0f mA", current)
        }
        return "N/A"
    }

    var coordinatesDisplay: String {
        if let lat = latitude, let lon = longitude {
            return String(format: "%.6f, %.6f", lat, lon)
        }
        return "N/A"
    }

    var maxAltDisplay: String {
        if let max_alt_m = max_alt_m {
            return UnitFormatter.altitude(Double(max_alt_m))
        }
        return "N/A"
    }

    var maxSpeedDisplay: String {
        if let max_speed_mps = max_speed_mps {
            return UnitFormatter.speed(Double(max_speed_mps))
        }
        return "N/A"
    }

    var pressureAltDisplay: String {
        if let alt = pressure_alt {
            return UnitFormatter.altitude(Double(alt))
        }
        return "N/A"
    }

    var altitudeRateDisplay: String {
        if let rate = altitude_rate {
            return UnitFormatter.speed(Double(rate))
        }
        return "N/A"
    }

    // IMU display helpers.  Values are converted to the display unit (m/s² or
    // g); the unit label is shown separately by the IMU row.
    var lowGDisplay: String {
        if let x = low_g_x, let y = low_g_y, let z = low_g_z {
            let s = UnitSystem.current
            return String(format: "%.2f  %.2f  %.2f",
                          UnitFormatter.accelerationValue(Double(x), system: s),
                          UnitFormatter.accelerationValue(Double(y), system: s),
                          UnitFormatter.accelerationValue(Double(z), system: s))
        }
        return "N/A"
    }

    var highGDisplay: String {
        if let x = high_g_x, let y = high_g_y, let z = high_g_z {
            let s = UnitSystem.current
            return String(format: "%.1f  %.1f  %.1f",
                          UnitFormatter.accelerationValue(Double(x), system: s),
                          UnitFormatter.accelerationValue(Double(y), system: s),
                          UnitFormatter.accelerationValue(Double(z), system: s))
        }
        return "N/A"
    }

    var gyroDisplay: String {
        if let x = gyro_x, let y = gyro_y, let z = gyro_z {
            return String(format: "%.1f  %.1f  %.1f", x, y, z)
        }
        return "N/A"
    }

    // Attitude display helpers
    var rollDisplay: String {
        if let r = roll { return String(format: "%.1f\u{00B0}", r) }
        return "N/A"
    }

    var pitchDisplay: String {
        if let p = pitch { return String(format: "%.1f\u{00B0}", p) }
        return "N/A"
    }

    var yawDisplay: String {
        if let y = yaw { return String(format: "%.1f\u{00B0}", y) }
        return "N/A"
    }

    var rollCmdDisplay: String {
        if let r = roll_cmd { return String(format: "%.1f\u{00B0}", r) }
        return "N/A"
    }

    // LoRa signal display helpers
    var rssiDisplay: String {
        if let rssi = rssi {
            return String(format: "%.0f dBm", rssi)
        }
        return "N/A"
    }

    var snrDisplay: String {
        if let snr = snr {
            return String(format: "%.1f dB", snr)
        }
        return "N/A"
    }

    // Base station battery display helpers
    /// Same clamp and negative-zero guard as `socDisplay` — the base station's
    /// pack reaches 0 the same way the rocket's does, and would print the same
    /// "-0.0%".
    var bsSocDisplay: String {
        if let soc = bs_soc {
            return String(format: "%.1f%%", min(100, max(0, soc)) + 0)
        }
        return "N/A"
    }

    var bsVoltageDisplay: String {
        if let voltage = bs_voltage {
            return String(format: "%.2f V", voltage)
        }
        return "N/A"
    }

    var bsCurrentDisplay: String {
        if let current = bs_current {
            return String(format: "%.0f mA", current)
        }
        return "N/A"
    }

    /// Whether the rocket itself is currently writing its onboard flight
    /// log.  The raw `logging_active` flag comes from the OC's
    /// `logger.isLoggingActive()` which returns true while the end-flight
    /// queue is still draining — so for several seconds post-landing it
    /// stays true, and if the drain wedges (END_FLIGHT lost over I2C, etc.)
    /// it can stick true indefinitely.
    ///
    /// Only suppress the indicator during the post-flight drain window
    /// (state == LANDED).  Trust the flag in every other state — manual
    /// pre-flight bench-test logging from READY/PRELAUNCH is a real
    /// scenario and the button label needs to flip to "Stop Logging"
    /// when it succeeds (#137 originally over-gated this to INFLIGHT only).
    /// A stuck-true that survives the user resetting to READY is a genuine
    /// anomaly; surfacing it is more useful than hiding it.
    var rocketLoggingActive: Bool {
        if state == "LANDED" { return false }
        return logging_active
    }

    // ── Relayed IMU orientation (#390) ─────────────────────────────────────
    // Wire layout mirrors LORA2_ORIENT_* in RocketComputerTypes.h:
    // bits 0-4 = discrete code (31 = auto-exact, no code), bits 5-6 = mode
    // (1 default / 2 manual / 3 auto; 0 never arrives — the BS omits "imo").

    var relayedOrientationMode: IMUOrientationMode {
        guard let packed = imu_orient_packed else { return .unknown }
        let code = packed & 0x1F
        switch (packed >> 5) & 0x3 {
        case 1: return .defaultMounting
        case 2: return .manual
        case 3: return code == 31 ? .autoExact : .autoSnap
        default: return .unknown
        }
    }

    var relayedOrientationName: String {
        guard let packed = imu_orient_packed,
              relayedOrientationMode != .unknown else { return "" }
        let code = UInt8(packed & 0x1F)
        // Auto-exact carries no discrete code — name it so the IMU card's
        // line still renders (it gates on a non-empty name).
        return code < 24 ? FlightSettingsData.b2rName(code: code) : "custom"
    }
}

/// Format an elapsed-time interval as H:MM:SS (or M:SS when under an hour).
/// Used in place of bare seconds so multi-minute waits read as durations
/// the operator can scan at a glance — e.g. "4:32" remaining on the BS
/// silence-close, or "1:03:15" since the last received packet during a
/// long stale stretch.  Negative or implausibly large values clamp to 0.
func formatElapsed(seconds: Int) -> String {
    let s = max(0, seconds)
    let h = s / 3600
    let m = (s % 3600) / 60
    let sec = s % 60
    if h > 0 {
        return String(format: "%d:%02d:%02d", h, m, sec)
    }
    return String(format: "%d:%02d", m, sec)
}

/// Convenience: same as the Int version but for milliseconds, rounded to
/// the nearest second.  Avoids divisor noise at the call site.
func formatElapsed(ms: UInt32) -> String {
    return formatElapsed(seconds: Int((ms + 500) / 1000))
}
