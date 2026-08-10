package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertContentEquals

/**
 * The phone fix is the ONLY record of where the base station was — the BS has
 * no GNSS — so a silent encoding bug loses the range for a whole test day and
 * is not recoverable afterwards.  These pin the byte layout the firmware
 * parses in `BLE_BS_CMD_SET_PHONE_FIX` (base_station main.cpp: memcpy of
 * i32/i32/i16 then one byte), case for case with iOS PhoneFixCodecTests.
 */
class PhoneFixCodecTest {

    private fun i32(b: ByteArray, o: Int): Int =
        (b[o].toInt() and 0xFF) or
            ((b[o + 1].toInt() and 0xFF) shl 8) or
            ((b[o + 2].toInt() and 0xFF) shl 16) or
            ((b[o + 3].toInt() and 0xFF) shl 24)

    private fun i16(b: ByteArray, o: Int): Int =
        (((b[o].toInt() and 0xFF) or ((b[o + 1].toInt() and 0xFF) shl 8)).toShort()).toInt()

    private fun u8(b: ByteArray, o: Int): Int = b[o].toInt() and 0xFF

    /** The real 2026-08-08 Kaua'i base-station site, round-tripped. */
    @Test
    fun encodesRealSiteToFirmwareLayout() {
        val d = PhoneFixCodec.encode(
            latDeg = 22.062009925676062,
            lonDeg = -159.35412251709172,
            altitudeM = 12.4,
            accuracyM = 4.6,
        )

        assertEquals(PhoneFixCodec.PAYLOAD_LENGTH, d.size)
        assertEquals(220620099, i32(d, 0))
        assertEquals(-1593541225, i32(d, 4))
        assertEquals(12, i16(d, 8))
        assertEquals(5, u8(d, 10))

        // Decoding the way the firmware does must land back on the site to
        // well under a metre.
        assertEquals(22.062009925676062, i32(d, 0) * 1e-7, 1e-6)
        assertEquals(-159.35412251709172, i32(d, 4) * 1e-7, 1e-6)
    }

    @Test
    fun missingAltitudeAndAccuracyUseSentinels() {
        val d = PhoneFixCodec.encode(0.0, 0.0, altitudeM = null, accuracyM = null)
        assertEquals(0, i16(d, 8))
        assertEquals(PhoneFixCodec.ACCURACY_UNKNOWN, u8(d, 10))
    }

    /**
     * A negative accuracy is the platform's "invalid fix" signal; it must
     * become the sentinel, not wrap to a small plausible number.
     */
    @Test
    fun negativeAccuracyBecomesSentinel() {
        val d = PhoneFixCodec.encode(1.0, 1.0, altitudeM = 0.0, accuracyM = -1.0)
        assertEquals(PhoneFixCodec.ACCURACY_UNKNOWN, u8(d, 10))
    }

    /**
     * Clamping, not wrapping.  A wrapped coordinate reads as a valid position
     * somewhere else on Earth and would be undetectable in the log.
     */
    @Test
    fun extremeValuesClamp() {
        val d = PhoneFixCodec.encode(90.0, -180.0, altitudeM = 1e9, accuracyM = 1e9)
        assertEquals(900000000, i32(d, 0))
        assertEquals(-1800000000, i32(d, 4))
        assertEquals(Short.MAX_VALUE.toInt(), i16(d, 8))
        assertEquals(255, u8(d, 10))
    }

    /** Southern/eastern hemispheres must stay signed through the round trip. */
    @Test
    fun signedHemispheres() {
        val d = PhoneFixCodec.encode(-33.8688, 151.2093, altitudeM = -5.0, accuracyM = 3.0)
        assertEquals(-33.8688, i32(d, 0) * 1e-7, 1e-6)
        assertEquals(151.2093, i32(d, 4) * 1e-7, 1e-6)
        assertEquals(-5, i16(d, 8))
    }

    /** The command frame is `[47][11-byte payload]` — nothing else. */
    @Test
    fun commandFrameLeadsWithCmd47() {
        val payload = PhoneFixCodec.encode(22.0, -159.0, 10.0, 4.0)
        val frame = Commands.setPhoneFix(22.0, -159.0, 10.0, 4.0)
        assertEquals(1 + PhoneFixCodec.PAYLOAD_LENGTH, frame.size)
        assertEquals(BleCommandId.SET_PHONE_FIX_BS, frame[0].toInt() and 0xFF)
        assertContentEquals(payload, frame.copyOfRange(1, frame.size))
    }
}
