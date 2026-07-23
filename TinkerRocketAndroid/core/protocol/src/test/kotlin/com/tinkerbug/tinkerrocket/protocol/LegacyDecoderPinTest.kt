package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

/**
 * Legacy-board decoders have no golden fixtures (the C++ emitter covers the
 * current Mini structs), so these are hand-written pins: exact-size accept /
 * size-minus-one reject per struct, plus hand-built byte payloads that pin
 * each field's offset and width.
 */
class LegacyDecoderPinTest {

    // --- Size pins (throw in Swift == null in Kotlin) ---

    @Test
    fun `icm45686 accepts exactly 36 bytes and rejects 35`() {
        assertNotNull(LegacyIcm45686Data.decode(ByteArray(LegacyIcm45686Data.SIZE)))
        assertNull(LegacyIcm45686Data.decode(ByteArray(LegacyIcm45686Data.SIZE - 1)))
    }

    @Test
    fun `h3lis331 accepts exactly 10 bytes and rejects 9`() {
        assertNotNull(LegacyH3lis331Data.decode(ByteArray(LegacyH3lis331Data.SIZE)))
        assertNull(LegacyH3lis331Data.decode(ByteArray(LegacyH3lis331Data.SIZE - 1)))
    }

    @Test
    fun `ms5611 accepts exactly 10 bytes and rejects 9`() {
        assertNotNull(LegacyMs5611Data.decode(ByteArray(LegacyMs5611Data.SIZE)))
        assertNull(LegacyMs5611Data.decode(ByteArray(LegacyMs5611Data.SIZE - 1)))
    }

    @Test
    fun `lis3mdl accepts exactly 10 bytes and rejects 9`() {
        assertNotNull(LegacyLis3mdlData.decode(ByteArray(LegacyLis3mdlData.SIZE)))
        assertNull(LegacyLis3mdlData.decode(ByteArray(LegacyLis3mdlData.SIZE - 1)))
    }

    @Test
    fun `legacy nonsensor accepts exactly 65 bytes and rejects 64`() {
        assertNotNull(LegacyNonSensorData.decode(ByteArray(LegacyNonSensorData.SIZE)))
        assertNull(LegacyNonSensorData.decode(ByteArray(LegacyNonSensorData.SIZE - 1)))
    }

    // --- Hand-built field-offset pins ---

    /** Little-endian byte builder for hand-made payloads. */
    private class Builder {
        val out = ArrayList<Byte>()
        fun u8(v: Int) { out.add(v.toByte()) }
        fun i16(v: Int) { u8(v and 0xFF); u8((v shr 8) and 0xFF) }
        fun i32(v: Int) { i16(v and 0xFFFF); i16((v shr 16) and 0xFFFF) }
        fun u32(v: Long) { i32(v.toInt()) }
        fun f32(v: Float) { i32(v.toRawBits()) }
        fun bytes(): ByteArray = out.toByteArray()
    }

    @Test
    fun `icm45686 field order - times then acc then gyro then temp`() {
        val b = Builder().apply {
            u32(4_000_000_000L)          // time_us > INT32_MAX
            u32(123456L)                 // time_sync
            i32(-100); i32(200); i32(-300)      // acc
            i32(4000); i32(-5000); i32(6000)    // gyro
            i32(-25)                     // temp
        }
        val d = LegacyIcm45686Data.decode(b.bytes())!!
        assertEquals(4_000_000_000L, d.timeUs)
        assertEquals(123456L, d.timeSync)
        assertEquals(listOf(-100, 200, -300), listOf(d.accX, d.accY, d.accZ))
        assertEquals(listOf(4000, -5000, 6000), listOf(d.gyroX, d.gyroY, d.gyroZ))
        assertEquals(-25, d.temp)
    }

    @Test
    fun `h3lis331 field order - time then acc i16`() {
        val b = Builder().apply {
            u32(1000L)
            i16(-32768); i16(32767); i16(-1)
        }
        val d = LegacyH3lis331Data.decode(b.bytes())!!
        assertEquals(1000L, d.timeUs)
        assertEquals(listOf(-32768, 32767, -1), listOf(d.accX, d.accY, d.accZ))
    }

    @Test
    fun `ms5611 field order - time then u32 pressure then i16 temperature`() {
        val b = Builder().apply {
            u32(2000L)
            u32(4_294_967_295L)          // full-scale pressure code stays positive
            i16(-40)
        }
        val d = LegacyMs5611Data.decode(b.bytes())!!
        assertEquals(2000L, d.timeUs)
        assertEquals(4_294_967_295L, d.pressure)
        assertEquals(-40, d.temperature)
    }

    @Test
    fun `lis3mdl field order - time then mag i16`() {
        val b = Builder().apply {
            u32(3000L)
            i16(-2000); i16(0); i16(1234)
        }
        val d = LegacyLis3mdlData.decode(b.bytes())!!
        assertEquals(3000L, d.timeUs)
        assertEquals(listOf(-2000, 0, 1234), listOf(d.magX, d.magY, d.magZ))
    }

    @Test
    fun `legacy nonsensor field order - floats then i32 blocks then bool flags`() {
        val b = Builder().apply {
            u32(65_000L)
            f32(10.5f); f32(-20.25f); f32(179.0f); f32(3.5f)  // roll/pitch/yaw/roll_cmd (deg)
            i32(100); i32(-200); i32(300)                     // pos cm
            i32(-10); i32(20); i32(-30)                       // vel cm/s
            i32(1500)                                          // pressure_alt m
            i32(-55)                                           // altitude_rate dm/s
            i32(1600)                                          // max_alt m
            i32(240)                                           // max_speed m/s
            u8(1); u8(0); u8(1); u8(0)                        // landed/apogee/velU/launch
            u8(3)                                              // rocket_state
        }
        val d = LegacyNonSensorData.decode(b.bytes())!!
        assertEquals(65_000L, d.timeUs)
        assertEquals(listOf(10.5f, -20.25f, 179.0f, 3.5f),
            listOf(d.roll, d.pitch, d.yaw, d.rollCmd))
        assertEquals(listOf(100, -200, 300), listOf(d.ePos, d.nPos, d.uPos))
        assertEquals(listOf(-10, 20, -30), listOf(d.eVel, d.nVel, d.uVel))
        assertEquals(1500, d.pressureAlt)
        assertEquals(-55, d.altitudeRate)
        assertEquals(1600, d.maxAlt)
        assertEquals(240, d.maxSpeed)
        assertEquals(true, d.altLandedFlag)
        assertEquals(false, d.altApogeeFlag)
        assertEquals(true, d.velUApogeeFlag)
        assertEquals(false, d.launchFlag)
        assertEquals(3, d.rocketState)
    }
}
