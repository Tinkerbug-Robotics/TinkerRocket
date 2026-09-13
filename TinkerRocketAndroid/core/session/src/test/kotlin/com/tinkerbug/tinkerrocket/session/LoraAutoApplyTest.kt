package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.RocketConfig
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * #624/#150. The states themselves need a base station, a rocket and a real
 * link, which no unit test can arrange — so what is pinned here is the part
 * that can be: which refusal a set of facts means, and in which order.
 *
 * The order is the substance. Each refusal names a different thing for the
 * operator to go and do, and naming a symptom instead of the cause sends them
 * to the wrong place.
 */
class LoraAutoApplyTest {

    private val NOW = 1_000_000L

    /** A readback complete enough for cmd 10 to be built from. */
    private fun fullConfig() = RocketConfig(
        loraFreqMHz = 915.0f, loraBwKHz = 250.0f, loraSF = 8, loraCR = 5, loraTxPower = 17,
    )

    private fun refusal(
        isBaseStation: Boolean = true,
        isConnected: Boolean = true,
        config: RocketConfig? = fullConfig(),
        lastSeen: List<Long> = listOf(NOW - 1_000L),
    ) = LoraAutoApply.refusalReason(isBaseStation, isConnected, config, lastSeen, NOW)

    @Test
    fun `a healthy base station link with a live rocket is allowed`() {
        assertNull(refusal())
    }

    @Test
    fun `a rocket link is refused before anything else is considered`() {
        // Cmd 17 is rejected by rocket-side firmware and cmd 10 belongs to the
        // base station, so "connect to the base station" is the only useful
        // thing to say — even when everything else is also wrong.
        assertEquals(LoraApplyRefusal.NOT_BASE_STATION, refusal(isBaseStation = false))
        assertEquals(
            LoraApplyRefusal.NOT_BASE_STATION,
            refusal(isBaseStation = false, isConnected = false, config = null, lastSeen = emptyList()),
        )
    }

    @Test
    fun `a disconnected base station outranks a missing readback`() {
        // The readback is missing BECAUSE the link is down; reporting
        // "waiting for config" would send the reader off to wait for
        // something that cannot arrive.
        assertEquals(
            LoraApplyRefusal.NOT_CONNECTED,
            refusal(isConnected = false, config = null, lastSeen = emptyList()),
        )
    }

    @Test
    fun `every field cmd 10 needs is required, not just any one of them`() {
        // Cmd 10 resends all five values whichever one the user changed, so a
        // readback missing any single field cannot produce a frame.
        assertEquals(LoraApplyRefusal.CONFIG_MISSING, refusal(config = null))
        val full = fullConfig()
        for ((name, cfg) in listOf(
            "bandwidth" to full.copy(loraBwKHz = null),
            "spreading factor" to full.copy(loraSF = null),
            "coding rate" to full.copy(loraCR = null),
            "tx power" to full.copy(loraTxPower = null),
        )) {
            assertEquals(
                LoraApplyRefusal.CONFIG_MISSING, refusal(config = cfg),
                "a readback with no $name cannot build a cmd 10",
            )
        }
    }

    @Test
    fun `frequency alone missing is not a refusal here`() {
        // Deliberate, and it matches iOS: the frequency field is not part of
        // the readiness check because a frequency CHANGE supplies its own
        // value. Only a TX-power push needs the stored one, and that path
        // checks it separately.
        assertNull(refusal(config = fullConfig().copy(loraFreqMHz = null)))
    }

    @Test
    fun `a rocket that has gone quiet blocks the push`() {
        // This is the whole point of the gate: pushing a new link setting to a
        // base station whose rocket is off strands the rocket on the old one.
        assertEquals(LoraApplyRefusal.NO_ROCKET_PRESENT, refusal(lastSeen = emptyList()))
        assertEquals(
            LoraApplyRefusal.NO_ROCKET_PRESENT,
            refusal(lastSeen = listOf(NOW - LoraAutoApply.MAX_BEACON_AGE_MS - 1)),
        )
    }

    @Test
    fun `a beacon exactly on the window edge still counts`() {
        assertNull(refusal(lastSeen = listOf(NOW - LoraAutoApply.MAX_BEACON_AGE_MS)))
    }

    @Test
    fun `one live rocket is enough when others have gone quiet`() {
        // A base station can track several; the transaction needs any one of
        // them to answer on the new settings.
        assertNull(refusal(lastSeen = listOf(NOW - 60_000L, NOW - 500L)))
    }

    @Test
    fun `every refusal says what to do about it`() {
        for (r in LoraApplyRefusal.entries) {
            assertTrue(r.message.isNotBlank(), "$r must name a next step, not grey out in silence")
        }
    }

    @Test
    fun `hopping is unavailable only when the firmware says the dwell is zero`() {
        // #150: 0 means the dwell rules cannot be met at this modulation and
        // the firmware will refuse the enable.
        assertFalse(LoraAutoApply.hoppingAvailable(RocketConfig(loraHopDwell = 0)))
        assertTrue(LoraAutoApply.hoppingAvailable(RocketConfig(loraHopDwell = 4)))
    }

    @Test
    fun `firmware that never reports a dwell is not treated as incapable`() {
        // Pre-#150 firmware omits the key. Greying the control on a device
        // that never claimed the limit would remove a working feature; the
        // firmware stays the final authority and refuses if it must.
        assertTrue(LoraAutoApply.hoppingAvailable(RocketConfig()))
        assertTrue(LoraAutoApply.hoppingAvailable(null))
    }
}
