package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

/**
 * Field-for-field golden walk of the simple fixed-size sensor payload
 * decoders (power/baro/imu/mag×2) against the C++ emitter sidecars, plus the
 * exact-size accept / size-minus-one reject pins ported from the iOS
 * SensorTypesTests (throw in Swift == null in Kotlin).
 */
class SensorPayloadGoldenTest {

    @Test
    fun `power decodes the golden payload field-for-field`() {
        val side = WireFixtures.sidecar("logframes/power_10.bin")
        val p = PowerData.decode(WireFixtures.bytes("logframes/power_10.bin"))
        assertNotNull(p)
        assertEquals(side["time_us"]!!.jsonPrimitive.long, p.timeUs)
        assertEquals(side["voltage_raw"]!!.jsonPrimitive.int, p.voltageRaw)
        assertEquals(side["current_raw"]!!.jsonPrimitive.int, p.currentRaw)
        assertEquals(side["soc_raw"]!!.jsonPrimitive.int, p.socRaw)
    }

    @Test
    fun `baro decodes the golden payload field-for-field`() {
        val side = WireFixtures.sidecar("logframes/baro_bmp585_12.bin")
        val b = Bmp585Data.decode(WireFixtures.bytes("logframes/baro_bmp585_12.bin"))
        assertNotNull(b)
        assertEquals(side["time_us"]!!.jsonPrimitive.long, b.timeUs)  // 4e9 > INT32_MAX by design
        assertEquals(side["temp_q16"]!!.jsonPrimitive.int, b.tempQ16)
        assertEquals(side["press_q6"]!!.jsonPrimitive.long, b.pressQ6)
    }

    @Test
    fun `imu decodes the golden payload field-for-field`() {
        val side = WireFixtures.sidecar("logframes/imu_ism6_22.bin")
        val m = Ism6hg256Data.decode(WireFixtures.bytes("logframes/imu_ism6_22.bin"))
        assertNotNull(m)
        assertEquals(side["time_us"]!!.jsonPrimitive.long, m.timeUs)  // exactly 2^31
        assertEquals(side["acc_low_raw"]!!.jsonArray.map { it.jsonPrimitive.int },
            listOf(m.accLowX, m.accLowY, m.accLowZ))
        assertEquals(side["acc_high_raw"]!!.jsonArray.map { it.jsonPrimitive.int },
            listOf(m.accHighX, m.accHighY, m.accHighZ))
        assertEquals(side["gyro_raw"]!!.jsonArray.map { it.jsonPrimitive.int },
            listOf(m.gyroX, m.gyroY, m.gyroZ))
    }

    @Test
    fun `mmc mag decodes the golden payload field-for-field`() {
        val side = WireFixtures.sidecar("logframes/mag_mmc5983_16.bin")
        val m = Mmc5983Data.decode(WireFixtures.bytes("logframes/mag_mmc5983_16.bin"))
        assertNotNull(m)
        assertEquals(side["time_us"]!!.jsonPrimitive.long, m.timeUs)
        // mag_x > 0x20000 by design — the 18-bit-mask trap for converters.
        assertEquals(side["mag_x"]!!.jsonPrimitive.long, m.magX)
        assertEquals(side["mag_y"]!!.jsonPrimitive.long, m.magY)
        assertEquals(side["mag_z"]!!.jsonPrimitive.long, m.magZ)
    }

    @Test
    fun `iis mag decodes the golden payload field-for-field`() {
        val side = WireFixtures.sidecar("logframes/mag_iis2mdc_10.bin")
        val m = Iis2mdcData.decode(WireFixtures.bytes("logframes/mag_iis2mdc_10.bin"))
        assertNotNull(m)
        assertEquals(side["time_us"]!!.jsonPrimitive.long, m.timeUs)
        assertEquals(
            listOf(side["mag_x"]!!.jsonPrimitive.int, side["mag_y"]!!.jsonPrimitive.int,
                side["mag_z"]!!.jsonPrimitive.int),
            listOf(m.magX, m.magY, m.magZ))
    }

    // Ported from iOS SensorTypesTests.assertSize: exactly SIZE zero bytes
    // must decode; SIZE-1 must be rejected (null here, throw on iOS).

    @Test
    fun `power accepts exactly 10 bytes and rejects 9`() {
        assertNotNull(PowerData.decode(ByteArray(PowerData.SIZE)))
        assertNull(PowerData.decode(ByteArray(PowerData.SIZE - 1)))
    }

    @Test
    fun `baro accepts exactly 12 bytes and rejects 11`() {
        assertNotNull(Bmp585Data.decode(ByteArray(Bmp585Data.SIZE)))
        assertNull(Bmp585Data.decode(ByteArray(Bmp585Data.SIZE - 1)))
    }

    @Test
    fun `imu accepts exactly 22 bytes and rejects 21`() {
        assertNotNull(Ism6hg256Data.decode(ByteArray(Ism6hg256Data.SIZE)))
        assertNull(Ism6hg256Data.decode(ByteArray(Ism6hg256Data.SIZE - 1)))
    }

    @Test
    fun `mmc mag accepts exactly 16 bytes and rejects 15`() {
        assertNotNull(Mmc5983Data.decode(ByteArray(Mmc5983Data.SIZE)))
        assertNull(Mmc5983Data.decode(ByteArray(Mmc5983Data.SIZE - 1)))
    }

    @Test
    fun `iis mag accepts exactly 10 bytes and rejects 9`() {
        assertNotNull(Iis2mdcData.decode(ByteArray(Iis2mdcData.SIZE)))
        assertNull(Iis2mdcData.decode(ByteArray(Iis2mdcData.SIZE - 1)))
    }
}
