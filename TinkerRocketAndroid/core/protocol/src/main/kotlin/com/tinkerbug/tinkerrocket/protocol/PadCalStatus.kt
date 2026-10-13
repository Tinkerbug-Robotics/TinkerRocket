package com.tinkerbug.tinkerrocket.protocol

/** The gyro/accel window of a pad calibration (PadCalStatusData.cal_state, PAD_CAL_*). */
public enum class PadCalState(public val raw: Int) {
    /** No pad calibration since the flight computer booted. */
    IDLE(0),

    /** The window is open. */
    RUNNING(1),

    /** It closed and the new offsets are in use. */
    DONE(2),

    /** It closed with an implausible gravity; the previous offsets are kept. */
    REJECTED(3),

    /** Not run: launched, landed, a sim running, or no IMU. */
    REFUSED(4);

    public companion object {
        /** Unknown raw values read as [IDLE], as [MagCalSubType.fromRaw] does. */
        public fun fromRaw(raw: Int): PadCalState = entries.firstOrNull { it.raw == raw } ?: IDLE
    }
}

/** What the raw-GNSS filter takes the ionosphere out with (PadCalStatusData.iono_source, PAD_IONO_*). */
public enum class PadIonoSource(public val raw: Int) {
    /** Neither: Klobuchar's night-time 5 ns only. */
    NONE(0),

    /** The broadcast parameters the app sent with a pad calibration. */
    BROADCAST(1),

    /** The pad survey's per-satellite corrections, the backup. */
    PAD_SURVEY(2);

    public companion object {
        public fun fromRaw(raw: Int): PadIonoSource = entries.firstOrNull { it.raw == raw } ?: NONE
    }
}

/**
 * PadCalStatus — what a pad calibration is doing (#1600 phase 5), the flight
 * computer's PadCalStatusData.  Sent once a second while the rocket is on the
 * ground, whether or not a calibration runs, and at each change; the out
 * computer relays it on the file_ops characteristic behind a 0xD0
 * discriminator.  A rocket that sends it has the firmware that knows command
 * 75.  The gyro/accel result itself still comes as the 0xCB
 * [SensorCalStatus] frame when the window closes.
 *
 * Wire (40 bytes, little-endian, packed):
 *   [0..3]   u32 time_us           [4] u8 cal_state     [5] u8 cal_progress
 *   [6..7]   u16 cal_gravity_mg    [8] u8 sats_tracked  [9] u8 sats_no_eph
 *   [10] u8 sats_settling  [11] u8 sats_corrected  [12..13] u16 corr_rms_cm
 *   [14] u8 iono_source    [15] u8 flags           [16..17] u16 iono_gps_day
 *   [18..19] i16 iono_age_days  [20..21] u16 pad_fixes  [22..23] u16 pad_secs
 *   [24..25] u16 pad_scatter_cm [26..29] i32 pad_lat_e7 [30..33] i32 pad_lon_e7
 *   [34..37] i32 pad_alt_mm     [38..39] u16 pos_sigma_cm
 */
public data class PadCalStatus(
    /** The flight computer's micros(); it wraps every ~71 min. */
    val timeUs: Long,                // u32
    val calState: PadCalState,
    /** % of the gyro/accel window gone (100 once it closed). */
    val calProgress: Int,            // u8
    /** |gravity| the window measured, milli-g (0: none yet). */
    val calGravityMg: Int,           // u16
    /** GPS, Galileo and BeiDou satellites with a pseudorange in the last epoch. */
    val satsTracked: Int,            // u8
    /** Of those, without a broadcast ephemeris yet (wait until 0). */
    val satsNoEph: Int,              // u8
    /** Locked under 50 s, not yet surveyed (a relocked pseudorange starts biased). */
    val satsSettling: Int,           // u8
    /** With a pad correction ready. */
    val satsCorrected: Int,          // u8
    /** The corrections' spread about each system's mean, cm. */
    val corrRmsCm: Int,              // u16
    val ionoSource: PadIonoSource,
    /** PCS_* bits; read them through the properties below. */
    val flags: Int,                  // u8
    /** The stored broadcast set's file date, days since 1980-01-06 (0: none stored). */
    val ionoGpsDay: Int,             // u16
    /** That set's age by the receiver's date; meaningful only when [ionoAgeKnown]. */
    val ionoAgeDays: Int,            // i16
    /** Receiver fixes in the pad average (each pad calibration restarts it). */
    val padFixes: Int,               // u16
    /** The span they cover, s. */
    val padSecs: Int,                // u16
    /** Their 3-D RMS about the average, cm. */
    val padScatterCm: Int,           // u16
    /** The pad, degrees × 1e7; the pad_* fields are meaningful only when [padAverage]. */
    val padLatE7: Int,               // i32
    val padLonE7: Int,               // i32
    /** Ellipsoidal height, mm. */
    val padAltMm: Int,               // i32
    /** The raw-GNSS filter's 3-D position sigma, cm (0: not running). */
    val posSigmaCm: Int,             // u16
) {
    /** This board runs the raw-GNSS filter (the LC86G): only then is there a GNSS block to show. */
    public val rawGnss: Boolean get() = (flags and FLAG_RAW_GNSS) != 0

    /** The raw-GNSS filter is running. */
    public val filterRunning: Boolean get() = (flags and FLAG_RUNNING) != 0

    /** [ionoAgeDays] is known: the receiver has given its date. */
    public val ionoAgeKnown: Boolean get() = (flags and FLAG_IONO_AGE) != 0

    /** The pad average has fixes, so the pad_* fields are set. */
    public val padAverage: Boolean get() = (flags and FLAG_PAD) != 0

    /** Launched: the pad average and the corrections are frozen. */
    public val padFrozen: Boolean get() = (flags and FLAG_PAD_FROZEN) != 0

    /**
     * The raw measurements put the pad more than 30 m from the receiver's
     * averaged fix, so the pad corrections are not used (and [ionoSource]
     * reads [PadIonoSource.NONE]).  The bench caught the LC86G's own fix
     * 312 m below sea level while its raw measurements were fine.
     */
    public val padMismatch: Boolean get() = (flags and FLAG_PAD_MISMATCH) != 0

    public companion object {
        public const val SIZE: Int = 40

        public const val FLAG_RAW_GNSS: Int = 0x01
        public const val FLAG_RUNNING: Int = 0x02
        public const val FLAG_IONO_AGE: Int = 0x04
        public const val FLAG_PAD: Int = 0x08
        public const val FLAG_PAD_FROZEN: Int = 0x10
        public const val FLAG_PAD_MISMATCH: Int = 0x20

        /** Decode the payload *after* the 0xD0 discriminator byte; null if too short. */
        public fun decode(payload: ByteArray): PadCalStatus? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return PadCalStatus(
                timeUs = b.u32(),
                calState = PadCalState.fromRaw(b.u8()),
                calProgress = b.u8(),
                calGravityMg = b.u16(),
                satsTracked = b.u8(),
                satsNoEph = b.u8(),
                satsSettling = b.u8(),
                satsCorrected = b.u8(),
                corrRmsCm = b.u16(),
                ionoSource = PadIonoSource.fromRaw(b.u8()),
                flags = b.u8(),
                ionoGpsDay = b.u16(),
                ionoAgeDays = b.i16(),
                padFixes = b.u16(),
                padSecs = b.u16(),
                padScatterCm = b.u16(),
                padLatE7 = b.i32(),
                padLonE7 = b.i32(),
                padAltMm = b.i32(),
                posSigmaCm = b.u16(),
            )
        }
    }
}
