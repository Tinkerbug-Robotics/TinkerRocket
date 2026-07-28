package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

/**
 * First decoder family through the golden walk.  Every field of the decoded
 * GnssData must equal the emitter's sidecar — the sidecar was produced by
 * the C++ compiler reading the same bytes through the real packed struct.
 */
class GnssDecoderGoldenTest {

    @Test
    fun `decodes the golden payload field-for-field`() {
        val side = WireFixtures.sidecar("logframes/gnss_42.bin")
        val g = GnssData.decode(WireFixtures.bytes("logframes/gnss_42.bin"))
        assertNotNull(g)

        assertEquals(side["time_us"]!!.jsonPrimitive.long, g.timeUs)
        assertEquals(side["year"]!!.jsonPrimitive.int, g.year)
        assertEquals(side["month"]!!.jsonPrimitive.int, g.month)
        assertEquals(side["day"]!!.jsonPrimitive.int, g.day)
        assertEquals(side["hour"]!!.jsonPrimitive.int, g.hour)
        assertEquals(side["minute"]!!.jsonPrimitive.int, g.minute)
        assertEquals(side["second"]!!.jsonPrimitive.int, g.second)
        assertEquals(side["milli_second"]!!.jsonPrimitive.int, g.milliSecond)
        assertEquals(side["fix_mode"]!!.jsonPrimitive.int, g.fixMode)
        assertEquals(side["num_sats"]!!.jsonPrimitive.int, g.numSats)
        assertEquals(side["pdop_x10"]!!.jsonPrimitive.int, g.pdopX10)
        assertEquals(side["lat_e7"]!!.jsonPrimitive.int, g.latE7)
        assertEquals(side["lon_e7"]!!.jsonPrimitive.int, g.lonE7)
        assertEquals(side["alt_mm"]!!.jsonPrimitive.int, g.altMm)
        assertEquals(side["vel_e_mmps"]!!.jsonPrimitive.int, g.velEMmps)
        assertEquals(side["vel_n_mmps"]!!.jsonPrimitive.int, g.velNMmps)
        assertEquals(side["vel_u_mmps"]!!.jsonPrimitive.int, g.velUMmps)
        assertEquals(side["h_acc_m"]!!.jsonPrimitive.int, g.hAccM)
        assertEquals(side["v_acc_m"]!!.jsonPrimitive.int, g.vAccM)
    }

    @Test
    fun `u32 time above INT32_MAX stays positive`() {
        val g = GnssData.decode(WireFixtures.bytes("logframes/gnss_42.bin"))!!
        assertEquals(3600123456L, g.timeUs)  // would be negative through a signed Int read
    }

    @Test
    fun `short payload returns null instead of throwing`() {
        assertNull(GnssData.decode(ByteArray(GnssData.SIZE - 1)))
    }
}
