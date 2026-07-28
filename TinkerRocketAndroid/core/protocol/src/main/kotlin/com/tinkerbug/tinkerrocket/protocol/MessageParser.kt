package com.tinkerbug.tinkerrocket.protocol

/**
 * One framed message from a `.bin` flight log:
 * `[AA 55 AA 55][type u8][len u8][payload][CRC16 hi][CRC16 lo]`.
 *
 * @property timestamp first 4 payload bytes as LE u32 (`time_us`), 0 when the
 *   payload is shorter than 4 bytes — the generic convention every payload
 *   honors by placing `time_us` first.
 */
public data class MessageFrame(
    val type: Int,
    val payload: ByteArray,
    val timestamp: Long,
) {
    override fun equals(other: Any?): Boolean =
        other is MessageFrame && type == other.type &&
            timestamp == other.timestamp && payload.contentEquals(other.payload)

    override fun hashCode(): Int = 31 * (31 * type + payload.contentHashCode()) + timestamp.toInt()
}

/**
 * Frame scanner for `.bin` flight logs — a semantics-exact port of iOS
 * `MessageParser.swift` (both must keep parsing the same logs identically):
 *
 *  - preamble search from the current index; none found → done
 *  - fewer than 4 bytes after the preamble for type+len+CRC → done
 *  - frame runs past end of data (truncated tail) → done, keep earlier frames
 *  - CRC mismatch → skip the ENTIRE claimed frame, then resync by preamble
 *    search (NOT advance-by-one; if the length byte itself was corrupted this
 *    can overshoot — iOS accepts that, so this port does too)
 *
 * CRC16 is computed over type + len + payload and carried BIG-endian on the
 * wire — the one big-endian field in an otherwise little-endian protocol.
 *
 * Pinned by the framed/ golden fixtures (good / bad-CRC-resync / truncated).
 */
public object MessageParser {

    private val PREAMBLE = byteArrayOf(0xAA.toByte(), 0x55, 0xAA.toByte(), 0x55)

    public fun parse(bytes: ByteArray): List<MessageFrame> {
        val frames = mutableListOf<MessageFrame>()
        var index = 0

        while (index < bytes.size) {
            val preambleIndex = findPreamble(bytes, index) ?: break
            index = preambleIndex + PREAMBLE.size

            // Need type + len + at least the 2 CRC bytes.
            if (index + 3 >= bytes.size) break

            val type = bytes[index].toInt() and 0xFF
            val length = bytes[index + 1].toInt() and 0xFF
            index += 2

            if (index + length + 2 > bytes.size) break  // truncated tail

            val payloadStart = index
            index += length
            val receivedCrc = ((bytes[index].toInt() and 0xFF) shl 8) or
                (bytes[index + 1].toInt() and 0xFF)
            index += 2

            // CRC over type + len + payload (contiguous in the source buffer).
            val calculated = Crc16.compute(bytes, payloadStart - 2, length + 2)
            if (calculated != receivedCrc) continue  // skip frame, resync by search

            val payload = bytes.copyOfRange(payloadStart, payloadStart + length)
            val timestamp = if (payload.size >= 4) {
                (payload[0].toLong() and 0xFF) or
                    ((payload[1].toLong() and 0xFF) shl 8) or
                    ((payload[2].toLong() and 0xFF) shl 16) or
                    ((payload[3].toLong() and 0xFF) shl 24)
            } else 0L

            frames.add(MessageFrame(type, payload, timestamp))
        }
        return frames
    }

    private fun findPreamble(bytes: ByteArray, startingAt: Int): Int? {
        var i = startingAt
        while (i <= bytes.size - PREAMBLE.size) {
            if (bytes[i] == PREAMBLE[0] && bytes[i + 1] == PREAMBLE[1] &&
                bytes[i + 2] == PREAMBLE[2] && bytes[i + 3] == PREAMBLE[3]
            ) return i
            i++
        }
        return null
    }
}

/** Log-frame message type bytes — mirrors RocketComputerTypes.h. */
public object MsgType {
    public const val OUT_STATUS_QUERY: Int = 0xA0
    public const val GNSS: Int = 0xA1
    public const val IMU: Int = 0xA2            // 22 B = ISM6HG256 (Mini); 36 B = legacy ICM45686
    public const val BARO: Int = 0xA3           // 12 B = BMP585; 10 B = legacy MS5611
    public const val MAG: Int = 0xA4            // 16 B = MMC5983MA; 10 B = legacy LIS3MDL
    public const val NON_SENSOR: Int = 0xA5     // 43/44/48/50 B ladder (65 B = legacy)
    public const val POWER: Int = 0xA6
    public const val START_LOGGING: Int = 0xA7
    public const val END_FLIGHT: Int = 0xA8
    public const val HIGH_G_LEGACY: Int = 0xA9  // H3LIS331, legacy boards only
    public const val CAMERA_START: Int = 0xAA
    public const val CAMERA_STOP: Int = 0xAB
    public const val IIS2MDC: Int = 0xD1
    public const val FLIGHT_SETTINGS: Int = 0xE1  // 188/200/208/210/219 B by version
}
