package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

/**
 * NonSensorData 43/44/48/50/52 length ladder — golden walk mirroring the iOS
 * GoldenVectorTests.testNonSensorLadder, plus the hand-written pins ported
 * from SensorTypesTests (#142/#143 apogee_flags back-compat, #529 ekf_ticks
 * and #1190 shock_gate_trips null-vs-0 semantics).
 */
class NonSensorDecoderTest {

    private val ladder = listOf(
        "logframes/nonsensor_43.bin", "logframes/nonsensor_44.bin",
        "logframes/nonsensor_48.bin", "logframes/nonsensor_50.bin",
        "logframes/nonsensor_52.bin",
    )

    @Test
    fun `ladder decodes base fields field-for-field at every rung`() {
        for (rel in ladder) {
            val side = WireFixtures.sidecar(rel)
            val n = NonSensorData.decode(WireFixtures.bytes(rel))
            assertNotNull(n, rel)

            assertEquals(side["time_us"]!!.jsonPrimitive.long, n.timeUs, rel)
            assertEquals(
                listOf(side["q0"]!!.jsonPrimitive.int, side["q1"]!!.jsonPrimitive.int,
                    side["q2"]!!.jsonPrimitive.int, side["q3"]!!.jsonPrimitive.int),
                listOf(n.q0, n.q1, n.q2, n.q3), rel)
            assertEquals(side["roll_cmd"]!!.jsonPrimitive.int, n.rollCmd, rel)
            assertEquals(
                listOf(side["e_pos"]!!.jsonPrimitive.int, side["n_pos"]!!.jsonPrimitive.int,
                    side["u_pos"]!!.jsonPrimitive.int),
                listOf(n.ePos, n.nPos, n.uPos), rel)
            assertEquals(
                listOf(side["e_vel"]!!.jsonPrimitive.int, side["n_vel"]!!.jsonPrimitive.int,
                    side["u_vel"]!!.jsonPrimitive.int),
                listOf(n.eVel, n.nVel, n.uVel), rel)
            assertEquals(side["flags"]!!.jsonPrimitive.int, n.flags, rel)
            assertEquals(side["rocket_state"]!!.jsonPrimitive.int, n.rocketState, rel)
            assertEquals(side["baro_alt_rate_dmps"]!!.jsonPrimitive.int, n.baroAltRateDmps, rel)
            assertEquals(side["pyro_status"]!!.jsonPrimitive.int, n.pyroStatus, rel)

            val present = side["present_bytes"]!!.jsonPrimitive.int
            // 43-byte logs decode apogee_flags as 0 (backwards-compat rule).
            assertEquals(
                if (present >= 44) side["apogee_flags"]!!.jsonPrimitive.int else 0,
                n.apogeeFlags, "apogee_flags @ $rel")
            // #529 null-vs-value distinction: ekf_ticks only exists at 50 B —
            // 43/44/48-byte frames must yield null, never 0.
            if (present >= 50) {
                assertEquals(side["ekf_ticks"]!!.jsonPrimitive.int, n.ekfTicks, "ekf_ticks @ $rel")
            } else {
                assertNull(n.ekfTicks, "ekf_ticks must be null @ $rel")
            }
            // #1190 null-vs-value distinction: shock_gate_trips only exists at 52 B.
            if (present >= 52) {
                assertEquals(side["shock_gate_trips"]!!.jsonPrimitive.int, n.shockGateTrips,
                    "shock_gate_trips @ $rel")
            } else {
                assertNull(n.shockGateTrips, "shock_gate_trips must be null @ $rel")
            }
        }
    }

    // --- Hand-written pins ported from iOS SensorTypesTests ---

    @Test
    fun `43-byte legacy accepted, 42 rejected`() {
        assertNotNull(NonSensorData.decode(ByteArray(43)))
        assertNull(NonSensorData.decode(ByteArray(42)))
    }

    @Test
    fun `44-byte current layout accepted`() {
        assertNotNull(NonSensorData.decode(ByteArray(44)))
    }

    @Test
    fun `apogee_flags decodes from byte 43`() {
        val bytes = ByteArray(44)
        bytes[43] = (1 shl 2).toByte()  // NSF2_MASTER_APOGEE
        val raw = NonSensorData.decode(bytes)
        assertNotNull(raw)
        assertEquals(1 shl 2, raw.apogeeFlags)
    }

    @Test
    fun `43-byte legacy decodes apogee_flags as 0`() {
        val raw = NonSensorData.decode(ByteArray(43))
        assertNotNull(raw)
        assertEquals(0, raw.apogeeFlags)
    }

    @Test
    fun `50-byte layout decodes ekf_ticks after skipping sensor_health`() {
        // #529: 50-byte layout appends u16 ekf_ticks after the u32
        // sensor_health (#303, offset 44, never surfaced by the app).
        val bytes = ByteArray(50)
        bytes[48] = 0x34  // little-endian 0x1234 = 4660
        bytes[49] = 0x12
        val raw = NonSensorData.decode(bytes)
        assertNotNull(raw)
        assertEquals(0x1234, raw.ekfTicks)
    }

    @Test
    fun `44 and 48-byte layouts decode ekf_ticks as null`() {
        // Pre-#529 layouts: null (not 0 — 0 is a real counter value,
        // meaning "EKF not yet initialized").
        assertNull(NonSensorData.decode(ByteArray(44))!!.ekfTicks)
        assertNull(NonSensorData.decode(ByteArray(48))!!.ekfTicks)
    }

    @Test
    fun `52-byte layout decodes shock_gate_trips after ekf_ticks`() {
        // #1190: 52-byte layout appends u16 shock_gate_trips at offset 50.
        val bytes = ByteArray(52)
        bytes[48] = 0x34; bytes[49] = 0x12   // ekf_ticks = 0x1234
        bytes[50] = 0x0E; bytes[51] = 0x00   // shock_gate_trips = 14
        val raw = NonSensorData.decode(bytes)
        assertNotNull(raw)
        assertEquals(0x1234, raw.ekfTicks)
        assertEquals(14, raw.shockGateTrips)
    }

    @Test
    fun `50-byte and shorter layouts decode shock_gate_trips as null`() {
        // Pre-#1190 layouts: null, not 0 — 0 is a real value (a clean flight).
        assertNull(NonSensorData.decode(ByteArray(50))!!.shockGateTrips)
        assertNull(NonSensorData.decode(ByteArray(48))!!.shockGateTrips)
    }
}
