package com.tinkerbug.tinkerrocket.protocol

import java.math.BigDecimal
import java.math.RoundingMode
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Device type auto-detected from binary log data — port of iOS `DeviceType`.
 */
public enum class DeviceType {
    /** ISM6HG256 22-byte IMU payload. */
    MINI,

    /** ICM45686 36-byte IMU payload + separate H3LIS331 high-G. */
    LEGACY,
}

/** Port of iOS `CSVError.noData` — the log contains no parseable rows. */
public class CsvNoDataException : Exception("binary log contains no usable data")

/**
 * Fixed-decimal formatter matching C/Swift `printf("%.Nf")` semantics.
 *
 * ROUNDING POLICY (single place, used for every CSV float column): Swift
 * `String(format:)` is C printf — it rounds the EXACT binary value of the
 * double to N decimals, ties-to-even.  Kotlin `String.format("%.Nf")` uses
 * HALF_UP instead, so it is NOT used here; `BigDecimal(value)` (the exact
 * binary expansion) + `setScale(HALF_EVEN)` reproduces printf.  printf also
 * keeps the sign on negative values that round to zero (`-0.0` → `"-0.00"`),
 * which BigDecimal drops (it has no signed zero) — restored explicitly.
 * Locale note: BigDecimal.toPlainString / Int.toString are locale-independent
 * ('.' decimal separator always) — equivalent to Locale.ROOT, with no
 * formatter involved that could inject a locale-dependent comma.
 */
internal fun formatFixed(value: Double, decimals: Int): String {
    if (value.isNaN()) return "nan"
    if (value.isInfinite()) return if (value > 0) "inf" else "-inf"
    var s = BigDecimal(value).setScale(decimals, RoundingMode.HALF_EVEN).toPlainString()
    val negative = value < 0.0 || (value == 0.0 && 1.0 / value < 0.0)
    if (negative && !s.startsWith("-")) s = "-$s"
    return s
}

/**
 * Generate CSV files (and the flight-summary sidecar) from binary log bytes.
 * One row per IMU update; other sensors forward-filled.  Supports both Mini
 * (ISM6HG256) and Legacy (ICM45686 + H3LIS331) devices.
 *
 * Direct port of iOS TinkerRocketApp/Models/CSVGenerator.swift — the iOS
 * source is the behavioral reference, including regression-pinned oddities
 * (#142 apogee gate fallback, #196 burnout-flag-only, #514 comma-free column
 * names + quat-blank rule, #529 ekf_ticks blank-vs-0).  Pure JVM: output goes
 * to an [Appendable] sink, no file I/O.
 */
public class CsvGenerator {

    /**
     * Detect device type by inspecting the first IMU frame's payload size.
     * Legacy ICM45686 = 36 bytes, Mini ISM6HG256 = 22 bytes.
     */
    public fun detectDeviceType(frames: List<MessageFrame>): DeviceType {
        val imuFrame = frames.firstOrNull { it.type == MsgType.IMU } ?: return DeviceType.MINI
        return if (imuFrame.payload.size >= 36) DeviceType.LEGACY else DeviceType.MINI
    }

    /**
     * Convenience for tests / in-memory use: parse [binary], render the whole
     * CSV into a String and return it with the [FlightSummary].
     */
    public fun writeCsv(binary: ByteArray): Pair<String, FlightSummary> {
        val sb = StringBuilder()
        val summary = generateCsv(binary, sb)
        return sb.toString() to summary
    }

    /** Parse [binary] into frames, then stream the CSV into [sink]. */
    public fun generateCsv(
        binary: ByteArray,
        sink: Appendable,
        progressCallback: ((Double) -> Unit)? = null,
    ): FlightSummary = generateCsv(MessageParser.parse(binary), sink, progressCallback)

    /**
     * Port of iOS `generateCSV(from:to:progressCallback:)`.
     *
     * @throws CsvNoDataException when no frames, no IMU frames, or no IMU
     *   frames survive the pre-launch trim (iOS `CSVError.noData`).
     */
    public fun generateCsv(
        frames: List<MessageFrame>,
        sink: Appendable,
        progressCallback: ((Double) -> Unit)? = null,
    ): FlightSummary {
        if (frames.isEmpty()) throw CsvNoDataException()

        // 2. Detect device type
        val deviceType = detectDeviceType(frames)

        // 3. Configure sensor converter with rotation from status query
        val converter = SensorConverter()
        configureRotation(converter, deviceType, frames)

        // 4. Convert frames to SI units (unparseable frames skipped, like iOS
        //    catch-and-continue — the Kotlin decoders return null instead of
        //    throwing).
        val gnssData = ArrayList<Timed<GnssDataSi>>()
        val imuData = ArrayList<Timed<Ism6Hg256DataSi>>()
        val baroData = ArrayList<Timed<Bmp585DataSi>>()
        val magData = ArrayList<Timed<Mmc5983MaDataSi>>()
        val powerData = ArrayList<Timed<PowerDataSi>>()
        val nonSensorData = ArrayList<Timed<NonSensorDataSi>>()
        val highGData = ArrayList<Timed<SensorConverter.HighGDataSi>>()  // Legacy only

        for (frame in frames) {
            when (frame.type) {
                MsgType.GNSS -> {
                    // GNSS struct is the same for both devices
                    val raw = GnssData.decode(frame.payload) ?: continue
                    gnssData.add(Timed(raw.timeUs, converter.convertGNSS(raw)))
                }

                MsgType.IMU -> when (deviceType) {
                    DeviceType.MINI -> {
                        val raw = Ism6hg256Data.decode(frame.payload) ?: continue
                        imuData.add(Timed(raw.timeUs, converter.convertISM6HG256(raw)))
                    }
                    DeviceType.LEGACY -> {
                        val raw = LegacyIcm45686Data.decode(frame.payload) ?: continue
                        imuData.add(Timed(raw.timeUs, converter.convertLegacyICM45686(raw)))
                    }
                }

                MsgType.BARO -> when (deviceType) {
                    DeviceType.MINI -> {
                        val raw = Bmp585Data.decode(frame.payload) ?: continue
                        baroData.add(Timed(raw.timeUs, converter.convertBMP585(raw)))
                    }
                    DeviceType.LEGACY -> {
                        val raw = LegacyMs5611Data.decode(frame.payload) ?: continue
                        baroData.add(Timed(raw.timeUs, converter.convertLegacyMS5611(raw)))
                    }
                }

                MsgType.MAG -> when (deviceType) {
                    DeviceType.MINI -> {
                        val raw = Mmc5983Data.decode(frame.payload) ?: continue
                        magData.add(Timed(raw.timeUs, converter.convertMMC5983MA(raw)))
                    }
                    DeviceType.LEGACY -> {
                        val raw = LegacyLis3mdlData.decode(frame.payload) ?: continue
                        magData.add(Timed(raw.timeUs, converter.convertLegacyLIS3MDL(raw)))
                    }
                }

                MsgType.IIS2MDC -> {
                    // New Mini PCB rev: IIS2MDC replaces MMC5983MA.  Converter
                    // output type is Mmc5983MaDataSi so the downstream Magnetic
                    // Field X/Y/Z (µT) columns emit the same way regardless of
                    // which mag chip is fitted.
                    val raw = Iis2mdcData.decode(frame.payload) ?: continue
                    magData.add(Timed(raw.timeUs, converter.convertIIS2MDC(raw)))
                }

                MsgType.POWER -> {
                    // Power encoding is the same for both devices
                    val raw = PowerData.decode(frame.payload) ?: continue
                    powerData.add(Timed(raw.timeUs, converter.convertPOWER(raw)))
                }

                MsgType.NON_SENSOR -> when (deviceType) {
                    DeviceType.MINI -> {
                        val raw = NonSensorData.decode(frame.payload) ?: continue
                        nonSensorData.add(Timed(raw.timeUs, converter.convertNonSensor(raw)))
                    }
                    DeviceType.LEGACY -> {
                        val raw = LegacyNonSensorData.decode(frame.payload) ?: continue
                        nonSensorData.add(Timed(raw.timeUs, converter.convertLegacyNonSensor(raw)))
                    }
                }

                MsgType.HIGH_G_LEGACY -> {
                    // Legacy-only high-G accelerometer
                    val raw = LegacyH3lis331Data.decode(frame.payload) ?: continue
                    highGData.add(Timed(raw.timeUs, converter.convertLegacyH3LIS331(raw)))
                }

                else -> {} // Skip non-sensor messages
            }
        }

        // 5. Compute ground pressure from pre-launch baro readings.
        //    NOTE: like iOS this runs on the FILE-ORDER (unsorted) lists —
        //    "suffix(100)" is a file-order window, pinned as-is.
        val groundPressure = computeGroundPressure(baroData, nonSensorData)

        // 6. Determine time range from IMU timestamps (one row per IMU update)
        if (imuData.isEmpty()) throw CsvNoDataException()

        // Sort all sensor data by timestamp
        gnssData.sortBy { it.timeUs }
        imuData.sortBy { it.timeUs }
        baroData.sortBy { it.timeUs }
        magData.sortBy { it.timeUs }
        powerData.sortBy { it.timeUs }
        nonSensorData.sortBy { it.timeUs }
        highGData.sortBy { it.timeUs }

        // If launch was detected, start CSV 4 seconds before launch to avoid
        // hundreds of seconds of dead pre-launch rows when the MRAM buffer
        // spans a long boot period.
        val preLaunchPadding = 4_000_000L  // 4 seconds in μs
        val launchTimestamp = nonSensorData.firstOrNull { it.data.launchFlag }?.timeUs

        val firstImuTime = imuData.first().timeUs
        val firstTime: Long = if (launchTimestamp != null && launchTimestamp > preLaunchPadding) {
            maxOf(firstImuTime, launchTimestamp - preLaunchPadding)
        } else {
            firstImuTime
        }

        // Filter out frames before the valid window
        gnssData.removeAll { it.timeUs < firstTime }
        imuData.removeAll { it.timeUs < firstTime }
        baroData.removeAll { it.timeUs < firstTime }
        magData.removeAll { it.timeUs < firstTime }
        powerData.removeAll { it.timeUs < firstTime }
        nonSensorData.removeAll { it.timeUs < firstTime }
        highGData.removeAll { it.timeUs < firstTime }

        if (imuData.isEmpty()) throw CsvNoDataException()

        // 8. Write CSV header
        sink.append(buildCsvHeader())

        // 9. Generate one row per IMU update, forward-filling other sensors
        val totalRows = imuData.size
        var rowsWritten = 0

        // Track flight summary stats
        var maxPressureAlt: Double? = null
        var maxSpeed: Double? = null
        var launchTimeUs: Long? = null
        var burnoutTimeUs: Long? = null  // first NSF_BURNOUT flag latch (#196)
        var apogeeTimeUs: Long? = null

        // Running indices for forward-fill (O(n+m) total)
        var gnssIdx = -1
        var baroIdx = -1
        var magIdx = -1
        var powerIdx = -1
        var nsIdx = -1
        var highGIdx = -1

        val progressBatch = 500  // iOS batch flush + progress cadence

        for (imuSample in imuData) {
            val t = imuSample.timeUs

            // Advance each index to the last sample at or before t
            while (gnssIdx + 1 < gnssData.size && gnssData[gnssIdx + 1].timeUs <= t) gnssIdx++
            while (baroIdx + 1 < baroData.size && baroData[baroIdx + 1].timeUs <= t) baroIdx++
            while (magIdx + 1 < magData.size && magData[magIdx + 1].timeUs <= t) magIdx++
            while (powerIdx + 1 < powerData.size && powerData[powerIdx + 1].timeUs <= t) powerIdx++
            while (nsIdx + 1 < nonSensorData.size && nonSensorData[nsIdx + 1].timeUs <= t) nsIdx++
            while (highGIdx + 1 < highGData.size && highGData[highGIdx + 1].timeUs <= t) highGIdx++

            val gnss = if (gnssIdx >= 0) gnssData[gnssIdx].data else null
            val baro = if (baroIdx >= 0) baroData[baroIdx].data else null
            val mag = if (magIdx >= 0) magData[magIdx].data else null
            val power = if (powerIdx >= 0) powerData[powerIdx].data else null
            val nonSensor = if (nsIdx >= 0) nonSensorData[nsIdx].data else null
            val highG = if (highGIdx >= 0) highGData[highGIdx].data else null

            // For Legacy: merge H3LIS331 high-G data into the IMU row
            var imu = imuSample.data
            if (highG != null) {
                imu = imu.copy(
                    highGAccX = highG.accX,
                    highGAccY = highG.accY,
                    highGAccZ = highG.accZ,
                )
            }

            // Build CSV row
            val timeMs = (t - firstTime).toDouble() / 1000.0
            sink.append(
                buildCsvRow(
                    timeMs = timeMs,
                    gnss = gnss,
                    imu = imu,
                    baro = baro,
                    groundPressure = groundPressure,
                    mag = mag,
                    power = power,
                    nonSensor = nonSensor,
                ),
            )

            // Track max pressure altitude
            if (baro != null && groundPressure != null) {
                val alt = pressureToAltitude(baro.pressure, groundPressure)
                if (maxPressureAlt == null || alt > maxPressureAlt!!) {
                    maxPressureAlt = alt
                }
            }

            // Track max 3D speed from EKF velocity for the max_speed_mps
            // metric. Only update before apogee — IMU integration drifts
            // after apogee so post-apogee speeds are unreliable.
            //
            // Gate on the master voted apogee_flag — a single per-detector
            // false positive (#142) previously gated off max-speed tracking
            // ~6s before real apogee, missing the burnout peak entirely.
            // Legacy 43-byte logs predate the master flag, so fall back to
            // requiring BOTH baro and velocity detectors to agree.
            if (nonSensor != null) {
                val postApogee = nonSensor.apogeeFlag ||
                    (nonSensor.altApogeeFlag && nonSensor.velUApogeeFlag)
                if (!postApogee) {
                    val speed = sqrt(
                        nonSensor.eVel * nonSensor.eVel +
                            nonSensor.nVel * nonSensor.nVel +
                            nonSensor.uVel * nonSensor.uVel,
                    )
                    if (maxSpeed == null || speed > maxSpeed!!) {
                        maxSpeed = speed
                    }
                }

                if (nonSensor.launchFlag && launchTimeUs == null) {
                    launchTimeUs = t
                }

                // burnout_time_s is the first NSF_BURNOUT flag latch (#196).
                // Legacy logs predating the burnout bit decode as false →
                // burnoutTimeUs stays null → the sidecar omits the key rather
                // than emitting a wrong peak-velocity proxy value.
                if (nonSensor.burnoutFlag && burnoutTimeUs == null) {
                    burnoutTimeUs = t
                }

                // Source apogee timestamp from the master voted flag — not
                // from a per-detector OR — to avoid a single noisy detector
                // (e.g. baro during boost, see #142) polluting the sidecar's
                // canonical apogee_time_s.  Legacy 43-byte logs decode the
                // master flag as false, so apogeeTimeUs stays null.
                if (nonSensor.apogeeFlag && apogeeTimeUs == null) {
                    apogeeTimeUs = t
                }
            }

            rowsWritten++
            if (rowsWritten % progressBatch == 0) {
                progressCallback?.invoke(rowsWritten.toDouble() / totalRows.toDouble())
            }
        }

        progressCallback?.invoke(1.0)

        // Compute event times relative to launch (seconds)
        val launch = launchTimeUs
        val burnoutTime: Double? =
            if (launch != null && burnoutTimeUs != null && burnoutTimeUs!! > launch) {
                (burnoutTimeUs!! - launch).toDouble() / 1_000_000.0
            } else {
                null
            }
        val apogeeTime: Double? =
            if (launch != null && apogeeTimeUs != null && apogeeTimeUs!! > launch) {
                (apogeeTimeUs!! - launch).toDouble() / 1_000_000.0
            } else {
                null
            }

        // Decode the flight settings snapshot (#165), if present.  Exactly
        // like iOS: only the FIRST 0xE1 frame is attempted — if it fails to
        // decode there is no fallback scan to later copies.
        val flightSettings: FlightSettings? = frames
            .firstOrNull { it.type == MsgType.FLIGHT_SETTINGS }
            ?.let { FlightSettingsData.decode(it.payload) }
            ?.let { FlightSettings.from(it) }

        return FlightSummary(
            maxAltitudeM = maxPressureAlt,
            maxSpeedMps = maxSpeed,
            burnoutTimeS = burnoutTime,
            apogeeTimeS = apogeeTime,
            settings = flightSettings,
        )
    }

    // MARK: - Rotation Configuration

    /**
     * Extract rotation config from the binary log and apply to the converter.
     * Mini: reads the FIRST 0xA0 (statusQuery) frame for per-board rotation
     * angles (no fallback scan if it fails to decode, like iOS).
     * Legacy: data is raw (unrotated), apply 180° in app.
     */
    private fun configureRotation(
        converter: SensorConverter,
        deviceType: DeviceType,
        frames: List<MessageFrame>,
    ) {
        when (deviceType) {
            DeviceType.MINI -> {
                val config = frames.firstOrNull { it.type == MsgType.OUT_STATUS_QUERY }
                    ?.let { OutStatusQueryData.decode(it.payload) }
                if (config != null) {
                    converter.configureMiniRotation(
                        imuDeg = config.imuRotationDeg,
                        magDeg = config.magRotationDeg,
                        iisDeg = config.iisRotationDeg,
                    )
                }
                // else: default rotation (0°)
            }
            DeviceType.LEGACY -> {
                // Legacy data is raw (unrotated), apply 180° rotation in app
                converter.configureLegacyRotation()  // defaults to 180°
            }
        }
    }

    // MARK: - Helper Functions

    /**
     * Compute ground pressure by averaging the last up-to-100 baro readings
     * strictly before launch (readings closest to launch avoid warmup
     * transients).  Falls back to the last 100 readings if no launch was
     * detected, and to the last 10 overall if the pre-launch window is empty.
     */
    private fun computeGroundPressure(
        baroData: List<Timed<Bmp585DataSi>>,
        nonSensorData: List<Timed<NonSensorDataSi>>,
    ): Double? {
        if (baroData.isEmpty()) return null

        val launchTime = nonSensorData.firstOrNull { it.data.launchFlag }?.timeUs

        val preLaunchPressures: List<Double> = if (launchTime != null) {
            baroData.filter { it.timeUs < launchTime }
                .map { it.data.pressure }
                .takeLast(100)
        } else {
            baroData.takeLast(100).map { it.data.pressure }
        }

        val pressures = if (preLaunchPressures.isEmpty()) {
            baroData.takeLast(10).map { it.data.pressure }
        } else {
            preLaunchPressures
        }

        if (pressures.isEmpty()) return null
        return pressures.sum() / pressures.size
    }

    /** Convert pressure to altitude AGL using the barometric formula. */
    private fun pressureToAltitude(pressure: Double, groundPressure: Double): Double =
        44330.0 * (1.0 - (pressure / groundPressure).pow(1.0 / 5.255))

    /** Build CSV header row — the exact 62 iOS column names (#514: no commas). */
    internal fun buildCsvHeader(): String {
        val columns = ArrayList<String>(62)

        // Time
        columns.add("Time (ms)")

        // GNSS
        columns.add("Latitude (deg)")
        columns.add("Longitude (deg)")
        columns.add("GNSS Altitude (m)")
        columns.add("Number of Satellites")
        columns.add("PDOP")
        columns.add("GNSS East Velocity (m/s)")
        columns.add("GNSS North Velocity (m/s)")
        columns.add("GNSS Up Velocity (m/s)")
        columns.add("GNSS Horizontal Accuracy (m)")
        columns.add("GNSS Vertical Accuracy (m)")

        // IMU - Low-G Accel
        columns.add("Low-G Acceleration X (m/s2)")
        columns.add("Low-G Acceleration Y (m/s2)")
        columns.add("Low-G Acceleration Z (m/s2)")

        // IMU - High-G Accel
        columns.add("High-G Acceleration X (m/s2)")
        columns.add("High-G Acceleration Y (m/s2)")
        columns.add("High-G Acceleration Z (m/s2)")

        // IMU - Gyro
        columns.add("Gyro X (deg/s)")
        columns.add("Gyro Y (deg/s)")
        columns.add("Gyro Z (deg/s)")

        // Barometer
        columns.add("Pressure (Pa)")
        columns.add("Barometer Temperature (C)")
        columns.add("Pressure Altitude (m)")

        // Magnetometer
        columns.add("Magnetic Field X (uT)")
        columns.add("Magnetic Field Y (uT)")
        columns.add("Magnetic Field Z (uT)")

        // Power
        columns.add("Voltage (V)")
        columns.add("Current (mA)")
        columns.add("State of Charge (%)")

        // NonSensor
        // #514: the quaternion is the ONLY reconstructable attitude in this
        // file.  Roll below is the body-Z azimuth (matches the FC roll
        // controller, doesn't gimbal-lock) while Pitch/Yaw are ZYX-Euler — two
        // different conventions, so Roll/Pitch/Yaw is NOT a valid Euler triple.
        // Column names must never contain a comma: this writer does not quote
        // fields and the readers split rows blind (the original #514 names
        // carried commas and shifted every column after "Quat q3").
        columns.add("Quat q0")
        columns.add("Quat q1")
        columns.add("Quat q2")
        columns.add("Quat q3")
        columns.add("Roll (deg; body-Z azimuth)")
        columns.add("Pitch (deg; ZYX Euler)")
        columns.add("Yaw (deg; ZYX Euler)")
        columns.add("Roll Command (deg)")
        columns.add("Position East (m)")
        columns.add("Position North (m)")
        columns.add("Position Up (m)")
        columns.add("Velocity East (m/s)")
        columns.add("Velocity North (m/s)")
        columns.add("Velocity Up (m/s)")
        columns.add("Altitude Rate (m/s)")
        columns.add("Landed Flag")
        // Per #142/#143: per-detector outputs renamed for clarity and master
        // voted result added alongside.  Old logs (43-byte NonSensorData)
        // emit 0 for the GPS/Pitch/Master columns.
        columns.add("Apogee Detector: Baro")
        columns.add("Apogee Detector: Velocity")
        columns.add("Apogee Detector: GPS")
        columns.add("Apogee Detector: Pitch")
        columns.add("Apogee Flag (Master)")
        columns.add("Launch Flag")

        // Pyro status bits — 4 channels (legacy files emit 0s for ch3/4)
        columns.add("Pyro 1 Continuity")
        columns.add("Pyro 2 Continuity")
        columns.add("Pyro 3 Continuity")
        columns.add("Pyro 4 Continuity")
        columns.add("Pyro 1 Fired")
        columns.add("Pyro 2 Fired")
        columns.add("Pyro 3 Fired")
        columns.add("Pyro 4 Fired")
        columns.add("Reboot Recovery")
        columns.add("FC Guidance Enabled")
        // #529: free-running EKF update-tick counter (uint16 wrap).  Blank on
        // logs predating the 50-byte NonSensorData.
        columns.add("EKF Ticks")

        return columns.joinToString(",") + "\n"
    }

    /** Build a CSV row — format specifiers verbatim from iOS `buildCSVRow`. */
    private fun buildCsvRow(
        timeMs: Double,
        gnss: GnssDataSi?,
        imu: Ism6Hg256DataSi,
        baro: Bmp585DataSi?,
        groundPressure: Double?,
        mag: Mmc5983MaDataSi?,
        power: PowerDataSi?,
        nonSensor: NonSensorDataSi?,
    ): String {
        val values = ArrayList<String>(62)

        // Time — %.3f
        values.add(formatFixed(timeMs, 3))

        // GNSS — %.7f lat/lon, %.3f alt/vel, bare int sats, %.1f pdop/acc
        values.add(gnss?.let { formatFixed(it.lat, 7) } ?: "")
        values.add(gnss?.let { formatFixed(it.lon, 7) } ?: "")
        values.add(gnss?.let { formatFixed(it.alt, 3) } ?: "")
        values.add(gnss?.numSats?.toString() ?: "")
        values.add(gnss?.let { formatFixed(it.pdop, 1) } ?: "")
        values.add(gnss?.let { formatFixed(it.velE, 3) } ?: "")
        values.add(gnss?.let { formatFixed(it.velN, 3) } ?: "")
        values.add(gnss?.let { formatFixed(it.velU, 3) } ?: "")
        values.add(gnss?.let { formatFixed(it.hAcc, 1) } ?: "")
        values.add(gnss?.let { formatFixed(it.vAcc, 1) } ?: "")

        // IMU - Low-G Accel — %.6f
        values.add(formatFixed(imu.lowGAccX, 6))
        values.add(formatFixed(imu.lowGAccY, 6))
        values.add(formatFixed(imu.lowGAccZ, 6))

        // IMU - High-G Accel — %.6f
        values.add(formatFixed(imu.highGAccX, 6))
        values.add(formatFixed(imu.highGAccY, 6))
        values.add(formatFixed(imu.highGAccZ, 6))

        // IMU - Gyro — %.6f
        values.add(formatFixed(imu.gyroX, 6))
        values.add(formatFixed(imu.gyroY, 6))
        values.add(formatFixed(imu.gyroZ, 6))

        // Barometer — %.2f
        values.add(baro?.let { formatFixed(it.pressure, 2) } ?: "")
        values.add(baro?.let { formatFixed(it.temperature, 2) } ?: "")
        if (baro != null && groundPressure != null) {
            values.add(formatFixed(pressureToAltitude(baro.pressure, groundPressure), 2))
        } else {
            values.add("")
        }

        // Magnetometer — %.6f
        values.add(mag?.let { formatFixed(it.magX, 6) } ?: "")
        values.add(mag?.let { formatFixed(it.magY, 6) } ?: "")
        values.add(mag?.let { formatFixed(it.magZ, 6) } ?: "")

        // Power — %.3f V, %.1f mA, %.1f %
        values.add(power?.let { formatFixed(it.voltage, 3) } ?: "")
        values.add(power?.let { formatFixed(it.current, 1) } ?: "")
        values.add(power?.let { formatFixed(it.soc, 1) } ?: "")

        // NonSensor
        // #514: quaternion first — the only unambiguous attitude here.  A zero
        // (non-unit) quaternion means "absent" (legacy log) → write blanks.
        val quatValid = nonSensor != null &&
            (
                nonSensor.q0 * nonSensor.q0 + nonSensor.q1 * nonSensor.q1 +
                    nonSensor.q2 * nonSensor.q2 + nonSensor.q3 * nonSensor.q3
                ) > 0.5
        values.add(if (quatValid) formatFixed(nonSensor!!.q0, 5) else "")
        values.add(if (quatValid) formatFixed(nonSensor!!.q1, 5) else "")
        values.add(if (quatValid) formatFixed(nonSensor!!.q2, 5) else "")
        values.add(if (quatValid) formatFixed(nonSensor!!.q3, 5) else "")
        values.add(nonSensor?.let { formatFixed(it.roll, 2) } ?: "")
        values.add(nonSensor?.let { formatFixed(it.pitch, 2) } ?: "")
        values.add(nonSensor?.let { formatFixed(it.yaw, 2) } ?: "")
        values.add(nonSensor?.let { formatFixed(it.rollCmd, 2) } ?: "")
        values.add(nonSensor?.let { formatFixed(it.ePos, 2) } ?: "")
        values.add(nonSensor?.let { formatFixed(it.nPos, 2) } ?: "")
        values.add(nonSensor?.let { formatFixed(it.uPos, 2) } ?: "")
        values.add(nonSensor?.let { formatFixed(it.eVel, 2) } ?: "")
        values.add(nonSensor?.let { formatFixed(it.nVel, 2) } ?: "")
        values.add(nonSensor?.let { formatFixed(it.uVel, 2) } ?: "")
        values.add(nonSensor?.let { formatFixed(it.altitudeRate, 1) } ?: "")
        values.add(nonSensor?.let { if (it.altLandedFlag) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.altApogeeFlag) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.velUApogeeFlag) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.gpsApogeeFlag) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.pitchApogeeFlag) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.apogeeFlag) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.launchFlag) "1" else "0" } ?: "")

        // Pyro status bits (4 channels)
        values.add(nonSensor?.let { if (it.pyro1Continuity) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.pyro2Continuity) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.pyro3Continuity) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.pyro4Continuity) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.pyro1Fired) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.pyro2Fired) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.pyro3Fired) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.pyro4Fired) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.rebootRecovery) "1" else "0" } ?: "")
        values.add(nonSensor?.let { if (it.guidanceEnabled) "1" else "0" } ?: "")
        // #529: bare int or blank (blank when no NonSensor OR pre-50-B log)
        values.add(nonSensor?.ekfTicks?.toString() ?: "")

        return values.joinToString(",") + "\n"
    }

    /** Timestamped SI sample — mirrors the iOS `(time_us, data)` tuples. */
    private class Timed<T>(val timeUs: Long, val data: T)
}
