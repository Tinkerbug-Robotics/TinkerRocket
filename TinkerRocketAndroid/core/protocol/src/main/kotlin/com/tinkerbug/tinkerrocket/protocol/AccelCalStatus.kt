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
 * 77, 78 and 80 and closes each face window; the flight computer sends it at no
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
 *
 * A flight computer that knows the high-g sends AccelCalHighGStatus after
 * those 40 bytes (114 in all), relayed whole by an out computer that knows it
 * too; a 40-byte frame is an older one's ([highG] null):
 *   [40..51]   f32 ×3 face_hg_mean_mps2 the face window's mean high-g reading,
 *                                       board frame, before any correction
 *   [52..63]   f32 ×3 face_hg_sd_mps2
 *   [64]       u8     hg_flags          HGF_*: bit0 an alignment stored, bit1
 *                                       applied (never in a sim), bit2 a bias stored
 *   [65]       u8     hg_bias_src       HG_BIAS_SRC_*: what wrote the bias in use
 *   [66..101]  f32 ×9 hg_matrix         the stored alignment M, row-major
 *                                       (identity when none)
 *   [102..113] f32 ×3 hg_bias_mps2      the bias in use, b
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
    /**
     * The high-g part of the frame; null when it carries none — an older
     * flight computer, or an older out computer relaying only the first 40
     * bytes.
     */
    val highG: HighG? = null,
) {
    /** An offset is stored on the board (NVS). */
    public val stored: Boolean get() = (flags and FLAG_STORED) != 0

    /** The flight computer takes it off its readings — it does not in a simulation. */
    public val applied: Boolean get() = (flags and FLAG_APPLIED) != 0

    public val offset: List<Float> get() = listOf(offsetX, offsetY, offsetZ)
    public val faceMean: List<Float> get() = listOf(faceMeanX, faceMeanY, faceMeanZ)
    public val faceSd: List<Float> get() = listOf(faceSdX, faceSdY, faceSdZ)

    /**
     * The high-g part (AccelCalHighGStatus): its corrected reading is
     * high = M · h − b − offset, board frame (RocketComputerTypes.h, HG_CAL_*).
     */
    public data class HighG(
        /** The face window's mean high-g reading, board frame x, y, z, m/s², before any correction. */
        val faceMean: List<Float>,
        /** Each axis's sample SD over that window, m/s². */
        val faceSd: List<Float>,
        /** HGF_* bits; read them through [stored], [applied] and [hasBias]. */
        val flags: Int,                 // u8
        /** HG_BIAS_SRC_* as sent; [source] is null for one newer than this app. */
        val biasSource: Int,            // u8
        /** The stored alignment M, row-major (identity when none). */
        val matrix: List<Float>,
        /** The bias in use, b, board frame x, y, z, m/s². */
        val bias: List<Float>,
    ) {
        /** An alignment is stored on the board (NVS). */
        public val stored: Boolean get() = (flags and FLAG_STORED) != 0

        /** And the flight computer applies it — it does not in a simulation. */
        public val applied: Boolean get() = (flags and FLAG_APPLIED) != 0

        /** A high-g bias is stored on the board. */
        public val hasBias: Boolean get() = (flags and FLAG_BIAS) != 0

        public val source: HighGBiasSource? get() = HighGBiasSource.fromRaw(biasSource)

        public companion object {
            /** AccelCalHighGStatus's size, after the 40 bytes of AccelCalStatusData. */
            public const val SIZE: Int = 74

            public const val FLAG_STORED: Int = 0x01
            public const val FLAG_APPLIED: Int = 0x02
            public const val FLAG_BIAS: Int = 0x04
        }
    }

    public companion object {
        public const val SIZE: Int = 40

        /** The whole frame from a flight computer that sends the high-g. */
        public const val HIGH_G_SIZE: Int = SIZE + HighG.SIZE

        public const val FLAG_STORED: Int = 0x01
        public const val FLAG_APPLIED: Int = 0x02

        /**
         * Decode the payload *after* the 0xD1 discriminator byte; null if too
         * short.  The high-g part only from a payload that holds all of it: a
         * frame cut anywhere inside it is still the low-g alone.
         */
        public fun decode(payload: ByteArray): AccelCalStatus? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            val lowG = AccelCalStatus(
                flags = b.u8(),
                faceState = AccelFaceState.fromRaw(b.u8()),
                offsetX = b.f32(), offsetY = b.f32(), offsetZ = b.f32(),
                faceMeanX = b.f32(), faceMeanY = b.f32(), faceMeanZ = b.f32(),
                faceSdX = b.f32(), faceSdY = b.f32(), faceSdZ = b.f32(),
                faceN = b.u16(),
            )
            if (payload.size < HIGH_G_SIZE) return lowG
            return lowG.copy(
                highG = HighG(
                    faceMean = List(3) { b.f32() },
                    faceSd = List(3) { b.f32() },
                    flags = b.u8(),
                    biasSource = b.u8(),
                    matrix = List(9) { b.f32() },
                    bias = List(3) { b.f32() },
                ),
            )
        }
    }
}

/** What wrote the high-g bias in use (HG_BIAS_SRC_*). */
public enum class HighGBiasSource(public val raw: Int) {
    /** None stored, or written before this was recorded. */
    NONE(0),

    /** The pad calibration (or a saved one the app pushed back, command 62). */
    PAD_CAL(1),

    /** The six-face fit (command 80). */
    SIX_FACES(2);

    public companion object {
        /** Null for a value newer than this app. */
        public fun fromRaw(raw: Int): HighGBiasSource? = entries.firstOrNull { it.raw == raw }
    }
}

/**
 * HighGCalApplyData, the command-80 payload (49 bytes): op (1 store, 0
 * clear), the alignment M row-major and the bias b, little-endian f32.  A
 * clear sets M back to identity and keeps the bias.  The flight computer
 * refuses a scale outside 0.97..1.03, a cross-axis term past 0.03 and a bias
 * past 11 m/s² (and anything in flight).  [Commands.highGCalApply] frames it.
 */
public object HighGCalApply {
    public const val PAYLOAD_LENGTH: Int = 49
    public const val OP_CLEAR: Int = 0
    public const val OP_STORE: Int = 1
    public val IDENTITY: List<Float> = listOf(1f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f)
    public val ZERO_BIAS: List<Float> = listOf(0f, 0f, 0f)
}
