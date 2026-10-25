package com.tinkerbug.tinkerrocket.protocol

/**
 * OcIdentityData (#1658, 68 bytes, msg 0x87): the out computer that wrote this
 * log, recorded by the OC itself ~1 Hz while logging, so a log names its
 * computer even when no phone downloaded it. Port of iOS `OcIdentityData`;
 * layout pinned in RocketComputerTypes.h.
 *
 * Strings are fixed fields, NUL-padded, and not NUL-terminated when full (a
 * 20-character unit name fills its field).
 */
public data class OcIdentityData(
    val timeUs: Long,          // the OC's clock
    val version: Int,
    val networkId: Int,
    val rocketId: Int,
    val boardRevCode: Int,     // as FlightSettingsData.boardRevCode
    val unitId: String,        // "a1b2c3d4"
    val unitName: String,
    val fwVersion: String,
) {
    val boardRevName: String get() = com.tinkerbug.tinkerrocket.protocol.boardRevName(boardRevCode)

    public companion object {
        public const val SIZE: Int = 68

        /** null unless the payload is exactly [SIZE] bytes. */
        public fun decode(payload: ByteArray): OcIdentityData? {
            if (payload.size != SIZE) return null
            fun u8(at: Int) = payload[at].toInt() and 0xFF
            fun text(at: Int, len: Int): String {
                var end = at
                while (end < at + len && payload[end] != 0.toByte()) end++
                return String(payload, at, end - at, Charsets.UTF_8)
            }
            return OcIdentityData(
                timeUs = (u8(0).toLong()) or (u8(1).toLong() shl 8) or
                    (u8(2).toLong() shl 16) or (u8(3).toLong() shl 24),
                version = u8(4),
                networkId = u8(5),
                rocketId = u8(6),
                boardRevCode = u8(7),
                unitId = text(8, 8),
                unitName = text(16, 20),
                fwVersion = text(36, 32),
            )
        }
    }
}
