package com.tinkerbug.tinkerrocket.protocol

/** One face window of the six-face calibration (AccelCalStatusData.face_state, ACC_FACE_*). */
public enum class AccelFaceState(public val raw: Int) {
    /** No face window since the flight computer booted. */
    IDLE(0),

    /** A window is open: the low-g accelerometer is being averaged for 3 s. */
    MEASURING(1),

    /** It closed still and at one g. */
    DONE(2),

    /**
     * It closed but the board moved (an axis's SD past 0.05 m/s²) or the
     * reading was not one g (|mean| more than 0.5 m/s² from 9.80665); the
     * numbers are sent anyway.  A refused window is FAILED too, with no
     * samples ([AccelCalStatus.faceN] 0).
     */
    FAILED(3);

    public companion object {
        /** Unknown raw values read as [IDLE], as [PadCalState.fromRaw] does. */
        public fun fromRaw(raw: Int): AccelFaceState = entries.firstOrNull { it.raw == raw } ?: IDLE
    }
}

/**
 * AccelCalStatus — the low-g accelerometer's six-face hand calibration
 * (#1629), the flight computer's AccelCalStatusData.  It answers commands 76,
 * 77 and 78 and closes each face window; the flight computer sends it at no
 * other time.  The out computer relays it on the file_ops characteristic
 * behind a 0xD1 discriminator.
 *
 * Board frame throughout: +X forward, +Y left, +Z up out of the component
 * side, whatever way the board is mounted in the rocket.
 *
 * Wire (40 bytes, little-endian, packed):
 *   [0]      u8     flags            ACCF_*: bit0 STORED, bit1 APPLIED
 *   [1]      u8     face_state       ACC_FACE_*
 *   [2..13]  f32 ×3 offset_mps2      the stored offset (0 when none)
 *   [14..25] f32 ×3 face_mean_mps2   the last face window's mean, before any offset
 *   [26..37] f32 ×3 face_sd_mps2     each axis's sample SD over that window
 *   [38..39] u16    face_n           its samples
 */
public data class AccelCalStatus(
    /** ACCF_* bits; read them through [stored] and [applied]. */
    val flags: Int,                 // u8
    val faceState: AccelFaceState,
    /** The stored offset, m/s² (0 when none). */
    val offsetX: Float,
    val offsetY: Float,
    val offsetZ: Float,
    /** The last face window's mean low-g reading, m/s², before any offset is taken off. */
    val faceMeanX: Float,
    val faceMeanY: Float,
    val faceMeanZ: Float,
    /** Each axis's sample SD over that window, m/s². */
    val faceSdX: Float,
    val faceSdY: Float,
    val faceSdZ: Float,
    /** Samples in the window (0 for a refusal). */
    val faceN: Int,                 // u16
) {
    /** An offset is stored on the board (NVS). */
    public val stored: Boolean get() = (flags and FLAG_STORED) != 0

    /** The flight computer takes it off its readings — it does not in a simulation. */
    public val applied: Boolean get() = (flags and FLAG_APPLIED) != 0

    public val offset: List<Float> get() = listOf(offsetX, offsetY, offsetZ)
    public val faceMean: List<Float> get() = listOf(faceMeanX, faceMeanY, faceMeanZ)
    public val faceSd: List<Float> get() = listOf(faceSdX, faceSdY, faceSdZ)

    public companion object {
        public const val SIZE: Int = 40

        public const val FLAG_STORED: Int = 0x01
        public const val FLAG_APPLIED: Int = 0x02

        /** Decode the payload *after* the 0xD1 discriminator byte; null if too short. */
        public fun decode(payload: ByteArray): AccelCalStatus? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            return AccelCalStatus(
                flags = b.u8(),
                faceState = AccelFaceState.fromRaw(b.u8()),
                offsetX = b.f32(), offsetY = b.f32(), offsetZ = b.f32(),
                faceMeanX = b.f32(), faceMeanY = b.f32(), faceMeanZ = b.f32(),
                faceSdX = b.f32(), faceSdY = b.f32(), faceSdZ = b.f32(),
                faceN = b.u16(),
            )
        }
    }
}
