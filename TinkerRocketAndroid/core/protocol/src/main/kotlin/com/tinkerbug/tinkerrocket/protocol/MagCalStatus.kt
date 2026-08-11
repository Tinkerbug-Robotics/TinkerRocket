package com.tinkerbug.tinkerrocket.protocol

import kotlin.math.sqrt

/** Sub-state of the mag-cal flow (MagCalStatusData.sub_type). */
public enum class MagCalSubType(public val raw: Int) {
    IDLE(0),
    SAMPLING(1),
    REVIEW(2),
    APPLIED(3),
    ABORTED(4),

    /**
     * #206 — between Accept and final NVS persist: chip OFFSET regs already
     * programmed, FC sampling ~5 s while the user rotates to verify the
     * corrected |B| stays in a tight band.
     */
    VERIFYING(5);

    /**
     * Is a calibration in flight, such that leaving the screen must abort it?
     *
     * The set matters in both directions.  Leaving during any of these three
     * strands the FC in MAG_CALIBRATION with the magnetometer offsets zeroed,
     * and that state has NO launch-detect failsafe by design — kinematicChecks
     * is skipped in MAG_CALIBRATION, so nothing ever clears it on its own.
     * VERIFYING is the sharpest of the three: a 60 s firmware timeout runs
     * evaluateVerify() whether or not the app is still watching, so walking
     * away can commit a permanent NVS calibration nobody reviewed.
     *
     * APPLIED must NOT be in the set. The FC rests in APPLIED whenever a
     * calibration exists, and MagCalibrator::abort() forces ABORTED from ANY
     * state — so aborting on the way out of a successful save would throw the
     * calibration away.
     */
    public val needsAbortOnLeave: Boolean
        get() = this == SAMPLING || this == REVIEW || this == VERIFYING

    public companion object {
        /** Unknown raw values fall back to [IDLE], mirroring the iOS `?? .idle`. */
        public fun fromRaw(raw: Int): MagCalSubType =
            entries.firstOrNull { it.raw == raw } ?: IDLE
    }
}

/** Fit accept/reject gate result (MagCalStatusData.reject_code). */
public enum class MagCalRejectCode(public val raw: Int) {
    OK(0),
    R_TOO_LOW(1),             // fitted R < 20 µT
    R_TOO_HIGH(2),            // fitted R > 80 µT
    HIGH_RESIDUAL(3),         // RMS residual above threshold (poor sphere fit)
    LOW_COVERAGE(4),          // < min populated wedges (insufficient tumble coverage)

    // #206 — post-accept verification sub-codes; the frame's
    // instantaneousFieldUt carries the worst observed |B| in all cases.
    VERIFY_FAILED(5),         // legacy / kept for back-compat
    VERIFY_TOO_HIGH(6),
    VERIFY_TOO_LOW(7),
    VERIFY_RANGE_WIDE(8),
    VERIFY_LOW_COVERAGE(9),
    VERIFY_FEW_SAMPLES(10);

    public companion object {
        /** Unknown raw values fall back to [OK], mirroring the iOS `?? .ok`. */
        public fun fromRaw(raw: Int): MagCalRejectCode =
            entries.firstOrNull { it.raw == raw } ?: OK
    }
}

/**
 * MagCalStatus — decoded form of the FC's MagCalStatusData binary frame
 * (issue #96), arriving on the BLE file_ops characteristic behind a 0xCA
 * discriminator.  Current wire payload is 36 bytes LE; the format grew over
 * time (22-byte → +coverage_mask 26 → +live LSBs 32 → +partial_mask 36,
 * #148) and [decode] accepts the shorter legacy payloads, defaulting the
 * missing trailers to 0 so older FC builds degrade to timer-based prompts.
 *
 * Field layout mirrors MagCalStatusData in RocketComputerTypes.h
 * (static_assert sizeof == 36).  The wire time_us (bytes 0..3) is not
 * exposed — iOS uses the phone clock for UI, and this port pins iOS.
 */
public data class MagCalStatus(
    /** Sub-state of the cal flow (drives the client UI). */
    val subType: MagCalSubType,

    /** Distinct directional wedges populated, out of 32 (#148). */
    val coverageBins: Int,      // u8

    /** Total raw samples accumulated this run, capped at MAX_SAMPLES. */
    val sampleCount: Int,       // u16

    /**
     * Magnitude of the most recent raw sample, in µT (post-OFFSET subtract
     * during SAMPLING — the *uncalibrated* reading).  0 before first sample.
     */
    val instantaneousFieldUt: Float,   // wire u16 ×10

    /**
     * Fitted hard-iron offset in raw IIS2MDC LSB units (0.15 µT/LSB).
     * Zero in SAMPLING/IDLE/ABORTED, populated in REVIEW/APPLIED.
     */
    val offsetX: Int,           // i16
    val offsetY: Int,
    val offsetZ: Int,

    /** Fitted Earth-field magnitude in µT; should land in [20, 80]. */
    val fieldRUt: Float,        // wire u16 ×10

    /** RMS residual of samples from the fitted sphere, in µT. */
    val residualUt: Float,      // wire u16 ×10

    /** 0 = fit accepted, non-zero = fit failed a gate. */
    val rejectCode: MagCalRejectCode,

    /**
     * Bitmap of populated 3³ wedges; bit 13 (unreachable centre cell) is
     * always 0.  0 on old firmware that ships the 22-byte payload.
     */
    val coverageMask: Long,     // u32

    /** Live raw mag-vector components in µT; all 0 on < 32-byte payloads. */
    val liveXUt: Float,
    val liveYUt: Float,
    val liveZUt: Float,

    /**
     * #148 partial-coverage mask — wedges with 1..(N-1) samples, disjoint
     * from [coverageMask].  0 on < 36-byte payloads.
     */
    val partialMask: Long,      // u32
) {
    /**
     * Magnitude of the fitted hard-iron offset vector, in µT (#207).
     * IIS2MDC: 0.15 µT/LSB, mirrors LSB_TO_UT in [decode].
     */
    public val centerMagnitudeUt: Float
        get() {
            val x = offsetX.toFloat(); val y = offsetY.toFloat(); val z = offsetZ.toFloat()
            return 0.15f * sqrt(x * x + y * y + z * z)
        }

    /**
     * |c| / R as a fraction in [0, ∞).  Ratios approaching 1 mean the
     * applied offset rivals Earth's field — rotating then sweeps |B| across
     * roughly [R − |c|, R + |c|], which can clip the EKF's [15, 80] µT mag
     * input gate.  Zero when no fit has converged yet (R == 0).
     */
    public val centerToRRatio: Float
        get() = if (fieldRUt > 0) centerMagnitudeUt / fieldRUt else 0f

    /**
     * UI severity bucket for the *absolute* center magnitude |c|.  The only
     * failure mode for |c| alone is approaching the chip's ±4915 µT OFFSET
     * register range, beyond which the IIS2MDC can't fully subtract the
     * bias.  Typical fresh new-PCB IIS2MDC sits around 1.7 mT — well under
     * 4500 µT, so the row stays [OK].
     */
    public enum class CenterWarning {
        OK,       // |c| < 4500 µT (chip can subtract it)
        HIGH,     // |c| ≥ 4500 µT (approaching chip OFFSET range)
    }

    public val centerWarning: CenterWarning
        get() = if (centerMagnitudeUt >= 4500f) CenterWarning.HIGH else CenterWarning.OK

    public companion object {
        /** Original (oldest accepted) payload size. */
        public const val MIN_SIZE: Int = 22

        /** IIS2MDC sensitivity, 0.15 µT/LSB (datasheet 9.13); mirrors firmware IIS2MDC_LSB_TO_uT. */
        public const val LSB_TO_UT: Float = 0.15f

        /**
         * Decode the wire payload (bytes *after* the 0xCA discriminator).
         * Accepts the legacy 22-byte layout, the +coverage_mask layout, the
         * +live-LSB layout, and the current 36-byte layout; missing trailers
         * default to 0.  Returns null if the buffer is too short to be even
         * the original 22-byte payload.
         */
        public fun decode(payload: ByteArray): MagCalStatus? {
            if (payload.size < MIN_SIZE) return null
            val b = LeBuffer(payload)

            b.u32()                       // bytes[0..3] time_us — unused (phone clock drives UI)
            val subTypeRaw = b.u8()       // [4]
            val coverage = b.u8()         // [5]
            val sampleCount = b.u16()     // [6..7]
            val instUtX10 = b.u16()       // [8..9]
            val offX = b.i16()            // [10..11]
            val offY = b.i16()            // [12..13]
            val offZ = b.i16()            // [14..15]
            val rUtX10 = b.u16()          // [16..17]
            val resUtX10 = b.u16()        // [18..19]
            val rejectRaw = b.u8()        // [20]
            // [21] = _pad
            // [22..25] u32 coverage_mask  (new in 26-byte payload)
            // [26..31] i16 inst_x/y/z LSB (new in 32-byte payload)
            // [32..35] u32 partial_mask   (new in 36-byte payload, #148)
            val coverageMask = if (payload.size >= 26) LeBuffer(payload, 22).u32() else 0L
            val liveXLsb = if (payload.size >= 32) LeBuffer(payload, 26).i16() else 0
            val liveYLsb = if (payload.size >= 32) LeBuffer(payload, 28).i16() else 0
            val liveZLsb = if (payload.size >= 32) LeBuffer(payload, 30).i16() else 0
            val partialMask = if (payload.size >= 36) LeBuffer(payload, 32).u32() else 0L

            return MagCalStatus(
                subType = MagCalSubType.fromRaw(subTypeRaw),
                coverageBins = coverage,
                sampleCount = sampleCount,
                instantaneousFieldUt = instUtX10 / 10.0f,
                offsetX = offX,
                offsetY = offY,
                offsetZ = offZ,
                fieldRUt = rUtX10 / 10.0f,
                residualUt = resUtX10 / 10.0f,
                rejectCode = MagCalRejectCode.fromRaw(rejectRaw),
                coverageMask = coverageMask,
                liveXUt = liveXLsb * LSB_TO_UT,
                liveYUt = liveYLsb * LSB_TO_UT,
                liveZUt = liveZLsb * LSB_TO_UT,
                partialMask = partialMask,
            )
        }
    }
}
