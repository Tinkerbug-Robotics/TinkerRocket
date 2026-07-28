package com.tinkerbug.tinkerrocket.protocol

/**
 * The log/I2S frame CRC: polynomial 0x8001 (NOT the common 0x8005/0x1021),
 * init 0x0000, unreflected, no xorOut.  Matches `calcCRC16` defaults in the
 * firmware's components/CRC library and iOS `MessageParser.calculateCRC16`.
 * Hand-ported on purpose — never substitute a library CRC16, every standard
 * variant differs.  Pinned against firmware output by the framed/ golden
 * fixtures.
 */
public object Crc16 {
    private const val POLYNOME = 0x8001

    public fun compute(data: ByteArray, offset: Int = 0, length: Int = data.size - offset): Int {
        var crc = 0
        for (i in offset until offset + length) {
            crc = crc xor ((data[i].toInt() and 0xFF) shl 8)
            repeat(8) {
                crc = if (crc and 0x8000 != 0) {
                    ((crc shl 1) xor POLYNOME) and 0xFFFF
                } else {
                    (crc shl 1) and 0xFFFF
                }
            }
        }
        return crc
    }
}
