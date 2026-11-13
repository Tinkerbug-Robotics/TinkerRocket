package com.tinkerbug.tinkerrocket.protocol

/** The board's mounting so far (SdrAlignStatusData.have). */
public enum class SdrAlignHave(public val raw: Int) {
    NONE(0),

    /** One pose: its tilt alone. */
    TILT(1),

    /** Two poses apart or more: the whole rotation. */
    WHOLE(2);

    public companion object {
        public fun fromRaw(raw: Int): SdrAlignHave = entries.firstOrNull { it.raw == raw } ?: NONE
    }
}

/** What the board keeps in its storage (SdrAlignStatusData.saved). */
public enum class SdrAlignSaved(public val raw: Int) {
    NONE(0),

    /** Saved, and the same as the one in use. */
    SAME(1),

    /** Saved, but the one in use differs: not saved yet. */
    DIFFERS(2);

    public companion object {
        public fun fromRaw(raw: Int): SdrAlignSaved = entries.firstOrNull { it.raw == raw } ?: NONE
    }
}

/** What the board did with the last rest the flight computer sent it. */
public enum class SdrRestFate(public val raw: Int) {
    NONE(0), PAIRED(1), MOVING(2), SHORT(3), LATE(4), NOT_ONE_G(5), NO_TIME(6), IMU_OFF(7);

    public companion object {
        public fun fromRaw(raw: Int): SdrRestFate? = entries.firstOrNull { it.raw == raw }
    }
}

/** The board's answer to a command. */
public enum class SdrCommandResult(public val raw: Int) {
    NONE(0), DONE(1), NOTHING_TO_SAVE(2), REFUSED(3), STORAGE_FAILED(4), UNKNOWN_OP(5);

    public companion object {
        public fun fromRaw(raw: Int): SdrCommandResult? = entries.firstOrNull { it.raw == raw }
    }
}

/** The flight computer's part in the app's last command. */
public enum class SdrCommandState(public val raw: Int) {
    IDLE(0), SENDING(1), ECHOED(2), NO_ANSWER(3), REFUSED(4);

    public companion object {
        public fun fromRaw(raw: Int): SdrCommandState? = entries.firstOrNull { it.raw == raw }
    }
}

/**
 * SdrAlignStatus — the gnss-sdr-dev board's IMU on its flight board: how it
 * is mounted on the flight computer and its accelerometer's calibration, as
 * the board reports it and the flight computer relays it (SdrAlignStatusData).
 * The board finds both from the flight computer's rests, paired with its own
 * readings: two poses apart give the whole rotation, the six faces every
 * axis's scale and offset. Behind a 0xD2 discriminator on the file_ops
 * characteristic, once a second on the ground with the board on J1 and at
 * each change; the poses in the flight computer's IMU frame (the six-face
 * board frame). Command 83 ([Commands.sdrAlign]) answers with it, its id
 * echoed. iOS twin: SdrAlignStatus.swift.
 *
 * Wire (111 bytes, little-endian, packed):
 *   [0] ver, [1] flags (FLAG_*), [2] imu (0 off, 1 watching, 2 aiding),
 *   [3] have, [4] from, [5] saved, [6] n_pose, [7] cal_axes, [8] last_rest,
 *   [9] gen, [10] n_rest, [11] n_pair, [12] cmd_id, [13] cmd_op, [14] cmd_res,
 *   [15] fc_cmd_state, [16] fc_cmd_op, [17] fc_cmd_id, [18] rests_sent (u8 each),
 *   [19..20] rest_age_ds, [21..22] board_age_ds (u16, 0.1 s, 65535 none),
 *   [23..34] spread_deg, resid_deg, cal_rms_mps2 (f32), [35..50] q (f32 ×4,
 *   U10 to the flight computer's IMU, scalar first), [51..62] scale (f32 ×3),
 *   [63..74] bias_mps2 (f32 ×3), [75..98] pose_dir (i8 ×3 ×8, 1/127),
 *   [99..106] pose_pairs (u8 ×8), [107..110] moved_deg (f32: from 3 only, the
 *   rail pose off the saved rotation)
 *
 * Flags: the board on J1, its report held, it can place a rest (GPS time or
 * the rest's arrival), the flight computer's rests stamped (else unstamped),
 * the board's own auto-save on this boot (a command turns it off), solved
 * against the body, the flight computer still.
 */
public data class SdrAlignStatus(
    val ver: Int,
    val flags: Int,
    val imu: Int,
    val have: SdrAlignHave,
    val from: Int,
    val saved: SdrAlignSaved,
    val calAxes: Int,
    val lastRestRaw: Int,
    val gen: Int,
    val nRest: Int,
    val nPair: Int,
    val cmdId: Int,
    val cmdOp: Int,
    val cmdResRaw: Int,
    val fcCmdStateRaw: Int,
    val fcCmdOp: Int,
    val fcCmdId: Int,
    val restsSent: Int,
    val restAgeDs: Int,
    val boardAgeDs: Int,
    val spreadDeg: Float,
    val residDeg: Float,
    val calRms: Float,
    val q: List<Float>,
    val scale: List<Float>,
    val bias: List<Float>,
    val poses: List<Pose>,
    /** From 3 only: the rail pose off the saved rotation, deg (5.1: that or more). */
    val movedDeg: Float = 0f,
) {
    /** One pose: the flight computer's reading's direction (its IMU frame) and the rests paired on it. */
    public data class Pose(val dir: List<Double>, val pairs: Int)

    val boardPresent: Boolean get() = flags and FLAG_BOARD != 0
    val reportHeld: Boolean get() = flags and FLAG_REPORT != 0
    /** The board can place a rest: GPS time, or the rest's arrival. */
    val canPlaceRest: Boolean get() = flags and FLAG_TIME != 0
    /** The flight computer's rests go stamped (its clock line); else unstamped. */
    val restsStamped: Boolean get() = flags and FLAG_LINE != 0
    val autosave: Boolean get() = flags and FLAG_AUTOSAVE != 0
    val fcStill: Boolean get() = flags and FLAG_STILL != 0
    val lastRest: SdrRestFate? get() = SdrRestFate.fromRaw(lastRestRaw)
    val cmdResult: SdrCommandResult? get() = SdrCommandResult.fromRaw(cmdResRaw)
    val fcCmdState: SdrCommandState? get() = SdrCommandState.fromRaw(fcCmdStateRaw)
    val allAxesCalibrated: Boolean get() = calAxes and 0x07 == 0x07
    public fun axisCalibrated(j: Int): Boolean = calAxes and (1 shl j) != 0

    /** Whole, every axis calibrated: nothing more for the six faces to give. */
    val isComplete: Boolean get() = have == SdrAlignHave.WHOLE && allAxesCalibrated

    /** Complete and good enough to save by itself: the board's own auto-save rule's quality. */
    val isGoodToSave: Boolean
        get() = isComplete && poses.size >= SAVE_POSES_MIN && residDeg <= SAVE_RESID_MAX_DEG && calRms <= SAVE_RMS_MAX

    /** In use: the saved rotation with this boot's rests (its tilt checked on the rail). */
    val onSavedRotation: Boolean get() = from == 3

    /** How far the board's IMU is turned from the flight computer's, deg. */
    val rotationDeg: Double
        get() = 2.0 * Math.toDegrees(kotlin.math.acos(kotlin.math.abs(q[0].toDouble()).coerceIn(-1.0, 1.0)))

    public companion object {
        public const val SIZE: Int = 111
        public const val POSES: Int = 8

        public const val SAVE_POSES_MIN: Int = 6
        public const val SAVE_RESID_MAX_DEG: Float = 1.0f
        public const val SAVE_RMS_MAX: Float = 0.05f

        /** The rail pose this far off the saved rotation says the board moved since its faces were measured. */
        public const val MOVED_WARN_DEG: Float = 1.0f

        public const val FLAG_BOARD: Int = 0x01
        public const val FLAG_REPORT: Int = 0x02
        public const val FLAG_TIME: Int = 0x04
        public const val FLAG_LINE: Int = 0x08
        public const val FLAG_AUTOSAVE: Int = 0x10
        public const val FLAG_BODY: Int = 0x20
        public const val FLAG_STILL: Int = 0x40

        /** Decode the payload *after* the 0xD2 discriminator byte; null if too short. */
        public fun decode(payload: ByteArray): SdrAlignStatus? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            val ver = b.u8()
            val flags = b.u8()
            val imu = b.u8()
            val have = SdrAlignHave.fromRaw(b.u8())
            val from = b.u8()
            val saved = SdrAlignSaved.fromRaw(b.u8())
            val nPose = minOf(b.u8(), POSES)
            val calAxes = b.u8()
            val lastRest = b.u8()
            val gen = b.u8()
            val nRest = b.u8()
            val nPair = b.u8()
            val cmdId = b.u8()
            val cmdOp = b.u8()
            val cmdRes = b.u8()
            val fcState = b.u8()
            val fcOp = b.u8()
            val fcId = b.u8()
            val restsSent = b.u8()
            val restAge = b.u16()
            val boardAge = b.u16()
            val spread = b.f32()
            val resid = b.f32()
            val rms = b.f32()
            val q = List(4) { b.f32() }
            val scale = List(3) { b.f32() }
            val bias = List(3) { b.f32() }
            val dirs = List(POSES) { List(3) { b.i8() / 127.0 } }
            val pairs = List(POSES) { b.u8() }
            val moved = b.f32()
            return SdrAlignStatus(
                ver = ver, flags = flags, imu = imu, have = have, from = from, saved = saved,
                calAxes = calAxes, lastRestRaw = lastRest, gen = gen, nRest = nRest, nPair = nPair,
                cmdId = cmdId, cmdOp = cmdOp, cmdResRaw = cmdRes, fcCmdStateRaw = fcState,
                fcCmdOp = fcOp, fcCmdId = fcId, restsSent = restsSent, restAgeDs = restAge,
                boardAgeDs = boardAge, spreadDeg = spread, residDeg = resid, calRms = rms,
                q = q, scale = scale, bias = bias,
                poses = List(nPose) { Pose(dirs[it], pairs[it]) }, movedDeg = moved,
            )
        }
    }
}

/**
 * SdrAlignCmdData, the command-83 payload: `[op u8][id u8]`.
 * [Commands.sdrAlign] frames it.
 */
public object SdrAlignCommand {
    public const val OP_SAVE: Int = 1
    public const val OP_CLEAR: Int = 2
    public const val OP_RESTART: Int = 3
    public const val OP_REPORT: Int = 4
}
