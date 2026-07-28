package com.tinkerbug.tinkerrocket.protocol

import java.text.BreakIterator

/**
 * App→device command frame builders — the encoder side of the BLE command
 * characteristic.  Each function returns the COMPLETE frame `[cmd u8][payload]`
 * exactly as it goes onto the wire (and exactly as pinned byte-for-byte by
 * tests_cpp/fixtures/wire/commands/).
 *
 * Behavioral reference: the send* methods in the iOS app's BLEDevice.swift
 * (plus SimulationView.launchSim for cmd 5 and the DashboardView relay call
 * sites for the with-state cmd 1/23 variants).  Byte packing mirrors the
 * Swift `withUnsafeBytes`/append sequences 1:1 — little-endian everywhere,
 * `UInt8(bitPattern:)` for i8 fields, raw IEEE-754 bits for floats.
 *
 * Pure functions only: no connection state, no config-cache side effects
 * (the iOS methods also mutate `rocketConfig`; that mirror-cache behavior
 * belongs to the Android device layer, not the protocol encoder).
 */
public object Commands {

    // ---------------------------------------------------------------- helpers

    /** One-byte frame for the many payload-less commands (power toggle, sim
     *  start/stop, mag-cal singles, OTA finish/abort, ...).  Mirrors iOS
     *  `sendCommand(_:)`. */
    public fun bare(cmdId: Int): ByteArray = byteArrayOf(cmdId.toByte())

    // ------------------------------------------------------------- file ops

    /** cmd 2 — request a page of the on-device file list. `[page u8]`. */
    public fun fileList(page: Int): ByteArray =
        frame(BleCommandId.FILE_LIST) { u8(page) }

    /** cmd 3 — delete a file by name. `[name utf8]` (no terminator). */
    public fun fileDelete(filename: String): ByteArray =
        frame(BleCommandId.FILE_DELETE) { bytes(filename.toByteArray(Charsets.UTF_8)) }

    /** cmd 4 — start a file download. `[name utf8]` (no terminator). */
    public fun fileDownload(filename: String): ByteArray =
        frame(BleCommandId.FILE_DOWNLOAD) { bytes(filename.toByteArray(Charsets.UTF_8)) }

    // ------------------------------------------------------------ simulation

    /**
     * cmd 5 — sim config, 16 B: `[mass_g f32][thrust_n f32][burn_s f32]
     * [descent_rate_mps f32]`.  The wire mass unit is GRAMS: the OC relay
     * (BLE and LoRa paths) divides by 1000 into the firmware-internal
     * `SimConfigData.mass_kg` before forwarding to the FC.  Field order
     * matches `SimConfigData`, the mass unit does not — never pack kg here.
     * Verified against the 2026-07-16 HIL log (500 g → 80 m/s² boost).
     */
    public fun simConfig(
        massGrams: Float,
        thrustN: Float,
        burnTimeS: Float,
        descentRateMps: Float,
    ): ByteArray = frame(BleCommandId.SIM_CONFIG) {
        f32(massGrams); f32(thrustN); f32(burnTimeS); f32(descentRateMps)
    }

    // ------------------------------------------------------- device settings

    /** cmd 9 — time sync, UTC wall-clock: `[year u16][month][day][hour][minute][second]`. */
    public fun timeSync(
        year: Int,
        month: Int,
        day: Int,
        hour: Int,
        minute: Int,
        second: Int,
    ): ByteArray = frame(BleCommandId.TIME_SYNC) {
        u16(year); u8(month); u8(day); u8(hour); u8(minute); u8(second)
    }

    /** cmd 10 — LoRa RF config: `[freq_mhz f32][bw_khz f32][sf u8][cr u8][tx_pwr i8]`. */
    public fun loraConfig(
        freqMHz: Float,
        bwKHz: Float,
        sf: Int,
        cr: Int,
        txPowerDbm: Int,
    ): ByteArray = frame(BleCommandId.LORA_CONFIG) {
        f32(freqMHz); f32(bwKHz); u8(sf); u8(cr); i8(txPowerDbm)
    }

    /** cmd 11 — sounds on/off. `[bool]`. */
    public fun soundsEnable(enabled: Boolean): ByteArray =
        frame(BleCommandId.SOUNDS_ENABLE) { bool(enabled) }

    /** cmd 17 (BS only) — disable/re-enable LoRa frequency hopping. `[bool]`. */
    public fun loraHopDisabled(disabled: Boolean): ByteArray =
        frame(BleCommandId.HOP_DISABLE_BS) { bool(disabled) }

    /**
     * cmd 12 — ServoConfigData, 22 B (#267 grew it from 14):
     * `[bias_us i16 ×4][hz i16][min_us i16][max_us i16][fin_min_deg f32][fin_max_deg f32]`.
     * Missing bias entries pad with 0, extras are ignored (iOS indexes 0..<4).
     */
    public fun servoConfig(
        biasesUs: List<Int>,
        hz: Int,
        minUs: Int,
        maxUs: Int,
        finMinDeg: Float,
        finMaxDeg: Float,
    ): ByteArray = frame(BleCommandId.SERVO_CONFIG) {
        for (i in 0 until 4) i16(if (i < biasesUs.size) biasesUs[i] else 0)
        i16(hz); i16(minUs); i16(maxUs)
        f32(finMinDeg); f32(finMaxDeg)
    }

    /** cmd 13 — PIDConfigData, 20 B: `[kp][ki][kd][min_cmd][max_cmd]` all f32. */
    public fun pidConfig(
        kp: Float,
        ki: Float,
        kd: Float,
        minCmd: Float,
        maxCmd: Float,
    ): ByteArray = frame(BleCommandId.PID_CONFIG) {
        f32(kp); f32(ki); f32(kd); f32(minCmd); f32(maxCmd)
    }

    /** cmd 14 — servo control on/off. `[bool]`. */
    public fun servoEnable(enabled: Boolean): ByteArray =
        frame(BleCommandId.SERVO_ENABLE) { bool(enabled) }

    /** cmd 22 — gain scheduling on/off. `[bool]`. */
    public fun gainScheduleEnable(enabled: Boolean): ByteArray =
        frame(BleCommandId.GAIN_SCHED_ENABLE) { bool(enabled) }

    // ------------------------------------------------------------ roll / test

    /**
     * cmd 24 — ServoTestAnglesData, 8 B: 4 × i16 centi-degrees.  Mirrors the
     * iOS conversion exactly: `Int16(clamping: Int(angle * 100.0))` — truncate
     * toward zero, then saturate to Int16.  Missing entries pad with 0.
     */
    public fun servoTestAngles(anglesDeg: List<Double>): ByteArray =
        frame(BleCommandId.SERVO_TEST) {
            for (i in 0 until 4) {
                val angle = if (i < anglesDeg.size) anglesDeg[i] else 0.0
                val cdeg = (angle * 100.0).toInt()
                    .coerceIn(Short.MIN_VALUE.toInt(), Short.MAX_VALUE.toInt())
                i16(cdeg)
            }
        }

    /**
     * cmd 26 — RollProfileData, 76 B (matches the firmware packed struct):
     * `[num_waypoints u8][pad ×3]` then 8 × `[time_s f32][angle_deg f32][mode u8]`
     * (9 B per waypoint).  Slots past the given list are zero-filled; entries
     * past 8 are dropped (count byte caps at 8, like iOS).
     * mode: 0 = ROLL_SEG_ANGLE, 1 = ROLL_SEG_NULL_RATE.
     */
    public fun rollProfile(waypoints: List<RollWaypoint>): ByteArray =
        frame(BleCommandId.ROLL_PROFILE) {
            u8(minOf(waypoints.size, 8))
            u8(0); u8(0); u8(0)
            for (i in 0 until 8) {
                val w = if (i < waypoints.size) waypoints[i] else null
                f32(w?.timeS ?: 0.0f)
                f32(w?.angleDeg ?: 0.0f)
                u8(w?.mode ?: 0)
            }
        }

    /**
     * cmd 31 — RollControlConfigData, 16 B:
     * `[use_angle_control u8][pad u8][roll_delay_ms u16][rate_cap_dps f32]
     * [kp_angle f32][integral_sep_threshold f32]`.
     */
    public fun rollControlConfig(
        useAngleControl: Boolean,
        rollDelayMs: Int,
        rateCapDps: Float,
        kpAngle: Float,
        integralSepThreshold: Float,
    ): ByteArray = frame(BleCommandId.ROLL_CTRL_CONFIG) {
        bool(useAngleControl); u8(0)
        u16(rollDelayMs)
        f32(rateCapDps); f32(kpAngle); f32(integralSepThreshold)
    }

    // -------------------------------------------------------------- guidance

    /** cmd 32 — guidance on/off (the small toggle, distinct from cmd 65). `[bool]`. */
    public fun guidanceEnable(enabled: Boolean): ByteArray =
        frame(BleCommandId.GUIDANCE_ENABLE) { bool(enabled) }

    /**
     * cmd 28 — GuidancePointData, 20 B: `[lat f64][lon f64][alt_m f32]`
     * geodetic Drift-Cast aim point (#435).  Same payload rides inside the BS
     * relay (see [relayToRocket]); a relayed point can never be confirmed —
     * the guid_target echo only rides the direct OC link.
     */
    public fun guidancePoint(latDeg: Double, lonDeg: Double, altitudeM: Float): ByteArray =
        frame(BleCommandId.GUIDANCE_POINT) { f64(latDeg); f64(lonDeg); f32(altitudeM) }

    /**
     * cmd 65 — GuidanceConfigData, 45 B.  v1 floats LE in struct order, then
     * `coast_delay u16`, `enable u8`, `target_mode u8`; #534 appended
     * `kp_pos f32`, `kd_vel f32`, `guidance_law u8` at offsets 36/40/44.
     * THE FIRST 36 BYTES ARE FROZEN — the FC parses by offset; append only,
     * never insert.  Both laws' parameters are always sent, regardless of
     * which law is selected.  A 45-byte-era FC rejects a 36-byte frame.
     */
    public fun guidanceConfig(
        enabled: Boolean,
        navGain: Float,
        maxAccel: Float,
        accelToFin: Float,
        maxFinDeg: Float,
        minSpeed: Float,
        coastDelayMs: Int,
        targetMode: Int,
        targetE: Float,
        targetN: Float,
        targetAlt: Float,
        kpPos: Float,
        kdVel: Float,
        guidanceLaw: Int,
    ): ByteArray = frame(BleCommandId.GUIDANCE_CONFIG) {
        f32(navGain); f32(maxAccel); f32(accelToFin); f32(maxFinDeg); f32(minSpeed)
        f32(targetE); f32(targetN); f32(targetAlt)
        u16(coastDelayMs)
        bool(enabled)
        u8(targetMode)
        // --- appended #534 (offsets 36/40/44) ---
        f32(kpPos); f32(kdVel)
        u8(guidanceLaw)
    }

    // ------------------------------------------------------------ fin layout

    /**
     * cmd 66 — FinConfigData, 18 B, raw form:
     * `[azimuth_deg f32 ×4][tilt_reverse_mask u8][roll_reverse_mask u8]`.
     * Azimuths are per SERVO index (index 0 = servo 1), in the control frame.
     */
    public fun finConfig(
        azimuthDeg: List<Float>,
        reverseMask: Int,
        rollReverseMask: Int,
    ): ByteArray {
        require(azimuthDeg.size == 4) { "finConfig needs exactly 4 azimuths" }
        return frame(BleCommandId.FIN_CONFIG) {
            for (a in azimuthDeg) f32(a)
            u8(reverseMask)
            u8(rollReverseMask)
        }
    }

    /**
     * cmd 66, ring-GUI form mirroring iOS `sendFinConfig`: derives each
     * servo's CONTROL azimuth from the ring slot it occupies (slot azimuths
     * {0,90,180,270} for "+" ringMode 0, {45,135,225,315} for "×" ringMode 1)
     * and packs the per-servo reverse bitmasks (bit i = servo i+1).
     * Returns null when any list is not exactly 4 long — iOS guards and
     * silently skips the send.
     */
    public fun finConfig(
        ringMode: Int,
        servoAtSlot: List<Int>,
        reverse: List<Boolean>,
        rollReverse: List<Boolean>,
    ): ByteArray? {
        if (servoAtSlot.size != 4 || reverse.size != 4 || rollReverse.size != 4) return null
        val slotAz: List<Float> =
            if (ringMode == 1) listOf(45f, 135f, 225f, 315f) else listOf(0f, 90f, 180f, 270f)
        val az = floatArrayOf(0f, 90f, 180f, 270f)   // per servo index (0 = servo 1)
        for (slot in 0 until 4) {
            val servo = servoAtSlot[slot]            // 1-4
            if (servo in 1..4) az[servo - 1] = slotAz[slot]
        }
        var mask = 0
        for (i in 0 until 4) if (reverse[i]) mask = mask or (1 shl i)
        var rollMask = 0
        for (i in 0 until 4) if (rollReverse[i]) rollMask = rollMask or (1 shl i)
        return finConfig(az.toList(), mask, rollMask)
    }

    // ----------------------------------------------------------- camera / IMU

    /** cmd 33 — camera type select. `[camera_type u8]`. */
    public fun cameraConfig(cameraType: Int): ByteArray =
        frame(BleCommandId.CAMERA_CONFIG) { u8(cameraType) }

    /**
     * cmd 1 with an explicit desired-state byte — ONLY used as the inner
     * frame of a BS relay (#390): the direct link sends bare cmd 1
     * ([bare]).  desired 1 = start recording, 0 = stop.
     */
    public fun cameraToggleWithState(recordOn: Boolean): ByteArray =
        frame(BleCommandId.CAMERA_TOGGLE) { bool(recordOn) }

    /**
     * cmd 23 with an explicit desired-state byte — ONLY used as the inner
     * frame of a BS relay (#390); the direct link sends bare cmd 23.
     */
    public fun toggleLoggingWithState(loggingOn: Boolean): ByteArray =
        frame(BleCommandId.TOGGLE_LOGGING) { bool(loggingOn) }

    /** cmd 64 — IMU mounting orientation: 0xFF = auto (pad-gravity detect),
     *  0..23 = manual board→rocket code. `[setting u8]`. */
    public fun imuOrient(setting: Int): ByteArray =
        frame(BleCommandId.IMU_ORIENT) { u8(setting) }

    /** cmd 67 — IMU logging rate: `[rate_hz u16]`, whitelisted ISM6HG256 ODR
     *  (960/1920/3840). */
    public fun imuRate(rateHz: Int): ByteArray =
        frame(BleCommandId.IMU_RATE) { u16(rateHz) }

    // ------------------------------------------------------------------ pyro

    /** cmd 34 — PyroConfigData, 24 B: 4 × `[enabled u8][mode u8][value f32]`.
     *  Exactly 4 channels required (iOS preconditions). */
    public fun pyroConfig(channels: List<PyroChannelConfig>): ByteArray {
        require(channels.size == 4) { "pyro config requires exactly 4 channels" }
        return frame(BleCommandId.PYRO_CONFIG) {
            for (ch in channels) {
                bool(ch.enabled)
                u8(ch.mode)
                f32(ch.value)
            }
        }
    }

    /** cmd 35 — manual continuity test. `[channel u8]`. */
    public fun pyroContTest(channel: Int): ByteArray =
        frame(BleCommandId.PYRO_CONT_TEST) { u8(channel) }

    /** cmd 36 — manual fire (bench test). `[channel u8]`. */
    public fun pyroFire(channel: Int): ByteArray =
        frame(BleCommandId.PYRO_FIRE_TEST) { u8(channel) }

    // -------------------------------------------------------------- identity

    /**
     * cmd 40 — set unit name, UTF-8, clamped to 20 BYTES (not characters):
     * the firmware hard-rejects plen > 20 with no write and no echo, and 20
     * chars of multibyte UTF-8 can exceed 20 bytes.  Clamps by dropping
     * trailing grapheme clusters (never splits a character), mirroring the
     * iOS `String.utf8Clamped(maxBytes:)` / KnownDeviceStore semantics.
     */
    public fun setUnitName(name: String): ByteArray =
        frame(BleCommandId.SET_NAME) {
            bytes(utf8Clamped(name, maxBytes = 20).toByteArray(Charsets.UTF_8))
        }

    /** cmd 41 — set network id. `[nid u8]`. */
    public fun setNetworkId(nid: Int): ByteArray =
        frame(BleCommandId.SET_NETWORK_ID) { u8(nid) }

    /** cmd 42 — set rocket id (1..254). `[rid u8]`. */
    public fun setRocketId(rid: Int): ByteArray =
        frame(BleCommandId.SET_ROCKET_ID) { u8(rid) }

    /** cmd 45 (BS only, #390) — pin radio focus: `[rid u8]`, 0 = auto.
     *  RAM-only on the BS; re-send on every connect. */
    public fun setFocusRocket(rid: Int): ByteArray =
        frame(BleCommandId.SET_FOCUS_ROCKET_BS) { u8(rid) }

    /** cmd 46 (BS only, #390) — BS CSV logging on/off; never uplinks a
     *  rocket-logging command (unlike legacy cmd 23). `[bool]`. */
    public fun bsLogging(on: Boolean): ByteArray =
        frame(BleCommandId.BS_LOGGING) { bool(on) }

    // ----------------------------------------------------------------- relay

    /**
     * cmd 50 (BS link) — relay envelope: `[target_rid u8][inner cmd+payload]`.
     * [innerFrame] is a complete `[cmd][payload]` frame from any other builder.
     *
     * The inner PAYLOAD (frame minus its cmd byte) is capped at 33 bytes —
     * bs_uplink_queue::kMaxPayload, mirroring iOS `innerPayload.prefix(33)` —
     * but the OC's uplink RX buffer further caps DELIVERED payloads at 26 B,
     * so keep inner payloads ≤ 26 (documented in BleCommandId; the 20-B
     * cmd-28 guidance point is the largest relayed payload in practice).
     */
    public fun relayToRocket(targetRid: Int, innerFrame: ByteArray): ByteArray {
        require(innerFrame.isNotEmpty()) { "inner frame must have at least the cmd byte" }
        return frame(BleCommandId.RELAY_TO_ROCKET_BS) {
            u8(targetRid)
            u8(innerFrame[0].toInt() and 0xFF)
            bytes(innerFrame.copyOfRange(1, minOf(innerFrame.size, 1 + 33)))
        }
    }

    // ----------------------------------------------------------- calibration

    /**
     * cmd 55 — MagCalApplyData, 14 B (#132 profile push):
     * `[cx i16][cy i16][cz i16][R_uT f32][res_uT f32]`.
     */
    public fun magCalApply(
        cx: Int,
        cy: Int,
        cz: Int,
        fieldRuT: Float,
        residualUT: Float,
    ): ByteArray = frame(BleCommandId.MAG_CAL_APPLY_OC) {
        i16(cx); i16(cy); i16(cz)
        f32(fieldRuT); f32(residualUT)
    }

    /**
     * cmd 62 — SensorCalApplyData, 18 B (#132 profile push):
     * `[gyro_x i16][gyro_y i16][gyro_z i16][hg_x f32][hg_y f32][hg_z f32]`.
     */
    public fun sensorCalApply(
        gyroX: Int,
        gyroY: Int,
        gyroZ: Int,
        hgX: Float,
        hgY: Float,
        hgZ: Float,
    ): ByteArray = frame(BleCommandId.SENSOR_CAL_APPLY_OC) {
        i16(gyroX); i16(gyroY); i16(gyroZ)
        f32(hgX); f32(hgY); f32(hgZ)
    }

    // ------------------------------------------------------------------ scan

    /** cmd 60 (BS only) — frequency scan, 12 B:
     *  `[start_mhz f32][stop_mhz f32][step_khz u16][dwell_ms u16]`. */
    public fun frequencyScan(
        startMHz: Float,
        stopMHz: Float,
        stepKHz: Int,
        dwellMs: Int,
    ): ByteArray = frame(BleCommandId.FREQ_SCAN_BS) {
        f32(startMHz); f32(stopMHz)
        u16(stepKHz); u16(dwellMs)
    }

    // ------------------------------------------------------------------- OTA

    /**
     * cmd 70 — OTA_BEGIN, 37 B: `[target u8][size u32][sha256 ×32]`.
     * targetIsFC=false flashes the connected BS/OC device's own app slot;
     * true is reserved for OC-relayed FC OTA (phase 4).
     */
    public fun otaBegin(targetIsFC: Boolean, totalSize: Long, sha256: ByteArray): ByteArray {
        require(sha256.size == 32) { "SHA-256 must be exactly 32 bytes" }
        return frame(BleCommandId.OTA_BEGIN) {
            bool(targetIsFC)
            u32(totalSize)
            bytes(sha256)
        }
    }

    // ------------------------------------------------------------- internals

    private inline fun frame(cmdId: Int, build: LeWriter.() -> Unit): ByteArray {
        val w = LeWriter()
        w.u8(cmdId)
        w.build()
        return w.toByteArray()
    }

    /**
     * Longest prefix of [s] whose UTF-8 encoding fits [maxBytes], never
     * splitting a user-perceived character: drops trailing grapheme clusters
     * (java.text.BreakIterator character boundaries) until it fits — the
     * JVM analogue of the iOS `removeLast()` loop over Swift Characters.
     */
    public fun utf8Clamped(s: String, maxBytes: Int): String {
        var out = s
        while (out.toByteArray(Charsets.UTF_8).size > maxBytes) {
            val it = BreakIterator.getCharacterInstance()
            it.setText(out)
            it.last()
            val prev = it.previous()
            out = if (prev == BreakIterator.DONE) "" else out.substring(0, prev)
        }
        return out
    }
}

/** One cmd-26 roll-profile waypoint: mode 0 = ROLL_SEG_ANGLE (hold angle),
 *  1 = ROLL_SEG_NULL_RATE. */
public data class RollWaypoint(
    val timeS: Float,
    val angleDeg: Float,
    val mode: Int,
)

/** One cmd-34 pyro channel: `mode` is the firmware trigger-mode enum,
 *  `value` its threshold (units depend on mode). */
public data class PyroChannelConfig(
    val enabled: Boolean,
    val mode: Int,
    val value: Float,
)

/**
 * Sequential little-endian writer — [LeBuffer]'s encode-side twin.  Same
 * rules: every multi-byte field LE, unsigned widths narrow from the widened
 * Kotlin types (u8/u16 take Int, u32 takes Long), floats/doubles as raw
 * IEEE-754 bits, and `java.nio` stays out of the module.
 */
internal class LeWriter(initialCapacity: Int = 64) {
    private var buf: ByteArray = ByteArray(initialCapacity)
    private var size: Int = 0

    private fun ensure(extra: Int) {
        if (size + extra > buf.size) {
            buf = buf.copyOf(maxOf(buf.size * 2, size + extra))
        }
    }

    fun u8(v: Int) {
        ensure(1)
        buf[size++] = v.toByte()
    }

    /** i8 — two's-complement low byte, the `UInt8(bitPattern:)` of the Swift side. */
    fun i8(v: Int): Unit = u8(v)

    fun bool(v: Boolean): Unit = u8(if (v) 0x01 else 0x00)

    fun u16(v: Int) {
        ensure(2)
        buf[size++] = v.toByte()
        buf[size++] = (v ushr 8).toByte()
    }

    fun i16(v: Int): Unit = u16(v)

    fun u32(v: Long) {
        ensure(4)
        buf[size++] = v.toByte()
        buf[size++] = (v ushr 8).toByte()
        buf[size++] = (v ushr 16).toByte()
        buf[size++] = (v ushr 24).toByte()
    }

    fun i32(v: Int) {
        ensure(4)
        buf[size++] = v.toByte()
        buf[size++] = (v ushr 8).toByte()
        buf[size++] = (v ushr 16).toByte()
        buf[size++] = (v ushr 24).toByte()
    }

    fun f32(v: Float): Unit = i32(v.toRawBits())

    fun f64(v: Double) {
        val bits = v.toRawBits()
        ensure(8)
        for (i in 0 until 8) buf[size++] = (bits ushr (8 * i)).toByte()
    }

    fun bytes(b: ByteArray) {
        ensure(b.size)
        b.copyInto(buf, size)
        size += b.size
    }

    fun toByteArray(): ByteArray = buf.copyOf(size)
}
