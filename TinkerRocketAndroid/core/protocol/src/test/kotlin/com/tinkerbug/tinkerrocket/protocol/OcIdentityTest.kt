package com.tinkerbug.tinkerrocket.protocol

import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import kotlinx.serialization.json.int
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull

/**
 * #1658 step 3: the out computer names itself in the log (OC_IDENTITY_MSG,
 * 0x87), so a flight's `source` no longer depends on which phone downloaded
 * it. Same cases as iOS OcIdentityTests.
 */
class OcIdentityTest {

    @Test
    fun `decodes the shared fixture`() {
        val side = WireFixtures.sidecar("logframes/ocidentity_68.bin")
        val id = assertNotNull(OcIdentityData.decode(WireFixtures.bytes("logframes/ocidentity_68.bin")))
        assertEquals(side["time_us"]!!.jsonPrimitive.long, id.timeUs)
        assertEquals(side["version"]!!.jsonPrimitive.int, id.version)
        assertEquals(side["network_id"]!!.jsonPrimitive.int, id.networkId)
        assertEquals(side["rocket_id"]!!.jsonPrimitive.int, id.rocketId)
        assertEquals(side["board_rev_code"]!!.jsonPrimitive.int, id.boardRevCode)
        assertEquals(side["unit_id"]!!.jsonPrimitive.content, id.unitId)
        // The fixture's name fills all 20 bytes with no NUL; it still reads whole.
        assertEquals("Twenty-char unitname", id.unitName)
        assertEquals(side["unit_name"]!!.jsonPrimitive.content, id.unitName)
        assertEquals(side["fw_version"]!!.jsonPrimitive.content, id.fwVersion)
        assertEquals("M1", id.boardRevName)
    }

    @Test
    fun `a wrong size is refused`() {
        assertNull(OcIdentityData.decode(ByteArray(67)))
        assertNull(OcIdentityData.decode(ByteArray(69)))
    }

    @Test
    fun `the log wins for what it knows`() {
        // The unit was renamed after the flight: the log keeps the name it flew
        // under, and the app still adds the profile, the FC and when.
        val log = FlightSource(unitId = "9e8a1b2c", unitName = "Beetle", rocketId = 2,
            ocBoard = "M1", ocFw = "41b2ab92")
        val live = FlightSource(unitId = "9e8a1b2c", unitName = "Beetle (renamed)",
            rocketProfile = "RIM66", rocketId = 3, fcBoard = "M1", ocBoard = "M1 (asserted)",
            fcFw = "41b2ab92", ocFw = "ffffffff", fcGnss = "LC86G",
            downloadedAt = "2026-10-24T18:03:00Z")
        val m = assertNotNull(FlightSource.merged(log, live))
        assertEquals("Beetle", m.unitName)
        assertEquals(2, m.rocketId)
        assertEquals("M1", m.ocBoard)
        assertEquals("41b2ab92", m.ocFw)
        assertEquals("RIM66", m.rocketProfile)
        assertEquals("LC86G", m.fcGnss)
        assertEquals("2026-10-24T18:03:00Z", m.downloadedAt)
    }

    @Test
    fun `either side alone survives`() {
        val live = FlightSource(unitName = "Atlas")
        assertEquals(live, FlightSource.merged(null, live))
        assertEquals(live, FlightSource.merged(live, null))
        assertNull(FlightSource.merged(null, null))
    }

    @Test
    fun `an unknown board is left out, not written as unknown`() {
        val id = OcIdentityData(0, 1, 0, 0, 0, "9e8a1b2c", "", "")
        val s = FlightSource.fromLog(id)
        assertNull(s.ocBoard)
        assertNull(s.unitName)
        assertNull(s.ocFw)
        assertEquals("9e8a1b2c", s.unitId)
    }
}
