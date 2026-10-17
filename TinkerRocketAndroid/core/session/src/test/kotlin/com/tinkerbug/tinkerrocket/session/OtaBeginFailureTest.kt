package com.tinkerbug.tinkerrocket.session

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * The words an operator reads after trying to flash a flight computer that
 * did not answer: most likely one that is stuck booting.  iOS twin:
 * OTABeginFailureTests.swift.
 */
class OtaBeginFailureTest {

    @Test
    fun aSilentFlightComputerIsNamedAndTheWayOutIsSaid() {
        val s = OtaBeginFailure.notAccepted(targetIsFc = true, timeoutS = 20)
        assertTrue(s.startsWith("The flight computer did not answer within 20s."), s)
        assertTrue(s.contains("rescue mode"), s)
        assertTrue(s.contains("USB"), s)
    }

    @Test
    fun theLocalWordingIsUnchanged() {
        assertEquals("Device did not accept OTA_BEGIN within 5s",
            OtaBeginFailure.notAccepted(targetIsFc = false, timeoutS = 5))
    }

    @Test
    fun bothPlatformsSayTheSameThing() {
        // Pinned verbatim so a wording edit on one platform fails here until the
        // iOS twin (OTABeginFailureTests.testWordingMatchesAndroid) matches.
        assertEquals(
            "The flight computer did not answer within 20s. It takes updates once it is " +
                "running, or in rescue mode, but not while it is still starting up. Firmware " +
                "with rescue mode switches to it about two minutes after a stalled start; try " +
                "again then. Older firmware needs a USB flash.",
            OtaBeginFailure.notAccepted(targetIsFc = true, timeoutS = 20),
        )
    }
}
