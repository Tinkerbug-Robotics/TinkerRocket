package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

/**
 * ota_status JSON parse — semantics from iOS OTAStatusUpdate.parse
 * (OTAStatus.swift): type-gated, per-field tolerant, unmapped states →
 * UNKNOWN.
 */
class OtaStatusMsgTest {

    @Test
    fun `all wire states map to their enum cases`() {
        val cases = mapOf(
            "ready" to OtaStatusUpdate.State.READY,
            "writing" to OtaStatusUpdate.State.WRITING,
            "ready_to_boot" to OtaStatusUpdate.State.READY_TO_BOOT,
            "verify_failed" to OtaStatusUpdate.State.VERIFY_FAILED,
            "aborted" to OtaStatusUpdate.State.ABORTED,
            "idle" to OtaStatusUpdate.State.IDLE,
        )
        for ((wire, expected) in cases) {
            val s = OtaStatusUpdate.parse("""{"type":"ota_status","state":"$wire"}""")
            assertNotNull(s, wire)
            assertEquals(expected, s.state, wire)
        }
    }

    @Test
    fun `writing frame carries cumulative bytes`() {
        val s = OtaStatusUpdate.parse("""{"type":"ota_status","state":"writing","bytes":123456}""")
        assertNotNull(s)
        assertEquals(OtaStatusUpdate.State.WRITING, s.state)
        assertEquals(123456L, s.bytes)
        assertNull(s.err)
        assertNull(s.fw)
    }

    @Test
    fun `verify_failed carries the stable error token`() {
        val s = OtaStatusUpdate.parse(
            """{"type":"ota_status","state":"verify_failed","err":"sha_mismatch"}""",
        )
        assertNotNull(s)
        assertEquals(OtaStatusUpdate.State.VERIFY_FAILED, s.state)
        assertEquals("sha_mismatch", s.err)
    }

    @Test
    fun `ready_to_boot carries the running-image version stamp`() {
        val s = OtaStatusUpdate.parse(
            """{"type":"ota_status","state":"ready_to_boot","bytes":204800,"fw":"abc1234+20260722-0930"}""",
        )
        assertNotNull(s)
        assertEquals(OtaStatusUpdate.State.READY_TO_BOOT, s.state)
        assertEquals(204800L, s.bytes)
        assertEquals("abc1234+20260722-0930", s.fw)
    }

    @Test
    fun `unmapped or missing state reads as UNKNOWN not a parse failure`() {
        assertEquals(
            OtaStatusUpdate.State.UNKNOWN,
            OtaStatusUpdate.parse("""{"type":"ota_status","state":"flashing"}""")?.state,
        )
        assertEquals(
            OtaStatusUpdate.State.UNKNOWN,
            OtaStatusUpdate.parse("""{"type":"ota_status"}""")?.state,
        )
        // Non-string state → "" → UNKNOWN (iOS `as? String ?? ""`).
        assertEquals(
            OtaStatusUpdate.State.UNKNOWN,
            OtaStatusUpdate.parse("""{"type":"ota_status","state":7}""")?.state,
        )
    }

    @Test
    fun `missing or non-integral bytes default to zero`() {
        assertEquals(0L, OtaStatusUpdate.parse("""{"type":"ota_status","state":"ready"}""")?.bytes)
        assertEquals(
            0L,
            OtaStatusUpdate.parse("""{"type":"ota_status","bytes":"12"}""")?.bytes,
        )
    }

    @Test
    fun `wrong type malformed json and non-object all return null`() {
        assertNull(OtaStatusUpdate.parse("""{"type":"config"}"""))
        assertNull(OtaStatusUpdate.parse("""{"state":"writing"}"""))
        assertNull(OtaStatusUpdate.parse("not json"))
        assertNull(OtaStatusUpdate.parse("[1,2]"))
    }
}
