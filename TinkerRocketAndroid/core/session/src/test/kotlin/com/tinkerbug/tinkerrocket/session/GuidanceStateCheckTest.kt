package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.RocketConfig
import com.tinkerbug.tinkerrocket.protocol.RocketGuidanceExtras
import com.tinkerbug.tinkerrocket.protocol.TelemetryCharMessage
import com.tinkerbug.tinkerrocket.protocol.TelemetryDispatch
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertIs
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * GTV Blue, 2026-11-01: the app showed station-keeping ON and the flight
 * computer flew with it OFF.  The flight computer's own on/off now rides
 * config_guid ("gen"); the app compares against that alone (iOS
 * GuidanceStateReadbackTests).
 */
class GuidanceStateCheckTest {

    private fun guidFrame(law: Int = 0, genTail: String): String {
        val p = RocketProfile.makeDefault("x", 0)
        return """{"type":"config_guid","gng":${p.pnNavGain},"gma":${p.pnMaxAccel},""" +
            """"gaf":${p.pnAccelToFin},"gmf":${p.pnMaxFinDeg},"gms":${p.pnMinSpeed},""" +
            """"gcd":${p.pnCoastDelayMs},"gtm":${p.pnTargetMode},"gte":${p.pnTargetE},""" +
            """"gtn":${p.pnTargetN},"gta":${p.pnTargetAltM},"gkp":${p.pnKpPos},""" +
            """"gkd":${p.pnKdVel},"glw":$law$genTail}"""
    }

    private fun parsedExtras(frame: String): RocketGuidanceExtras? {
        val msg = TelemetryDispatch.parse(frame)
        assertIs<TelemetryCharMessage.ConfigGuid>(msg)
        return msg.extras
    }

    private fun extras(enabled: Boolean?, law: Int = 0): RocketGuidanceExtras {
        val p = RocketProfile.makeDefault("x", 0)
        return RocketGuidanceExtras(
            navGain = p.pnNavGain, maxAccel = p.pnMaxAccel, accelToFin = p.pnAccelToFin,
            maxFinDeg = p.pnMaxFinDeg, minSpeed = p.pnMinSpeed, coastDelayMs = p.pnCoastDelayMs,
            targetMode = p.pnTargetMode, targetE = p.pnTargetE, targetN = p.pnTargetN,
            targetAltM = p.pnTargetAltM, kpPos = p.pnKpPos, kdVel = p.pnKdVel,
            guidanceLaw = law, enabled = enabled,
        )
    }

    private fun profile(guidance: Boolean, law: Int = 0): RocketProfile =
        RocketProfile.makeDefault("GTV", 0).copy(guidanceEnabled = guidance, pnGuidanceLaw = law)

    private fun config(fc: Boolean?, law: Int = 0, ge: Boolean = true): RocketConfig =
        RocketConfig(guidanceEnabled = ge, guidanceExtras = extras(fc, law))

    private fun check(
        p: RocketProfile?,
        c: RocketConfig?,
        connected: Boolean = true,
        baseStation: Boolean = false,
    ) = GuidanceStateCheck.evaluate(p, c, connected, baseStation)

    // ── Parsing ──

    @Test fun genIsTheFlightComputersOnOff() {
        assertEquals(false, parsedExtras(guidFrame(genTail = ""","gen":false"""))?.enabled)
        assertEquals(true, parsedExtras(guidFrame(genTail = ""","gen":true"""))?.enabled)
    }

    @Test fun withoutGenTheFrameStillParsesButIsUnvouched() {
        val e = parsedExtras(guidFrame(genTail = ""))
        assertEquals(0, e?.guidanceLaw)
        assertNull(e?.enabled)
        assertNull(RocketConfig(guidanceEnabled = true, guidanceExtras = e).guidanceEnabledOnFlightComputer)
    }

    // ── The check ──

    @Test fun gtvBlueProfileOnRocketOff() {
        val c = check(profile(true, law = 1), config(fc = false, law = 1))
        assertEquals(GuidanceStateCheck.RocketOff, c)
        assertTrue(c.isMismatch)
        assertEquals("Rocket guidance OFF — profile says ON", c.message)
    }

    @Test fun profileOffRocketOn() {
        assertEquals(GuidanceStateCheck.RocketOn, check(profile(false), config(fc = true)))
    }

    @Test fun agreementSaysNothing() {
        assertEquals(GuidanceStateCheck.Agrees, check(profile(true, 1), config(fc = true, law = 1)))
        assertEquals(GuidanceStateCheck.Agrees, check(profile(false), config(fc = false)))
        assertNull(GuidanceStateCheck.Agrees.message)
    }

    @Test fun bothOnDifferentLaws() {
        val c = check(profile(true, law = 1), config(fc = true, law = 0))
        assertEquals(GuidanceStateCheck.LawDiffers(rocket = 0, profile = 1), c)
        assertEquals("Rocket flies PN — profile says station-keep", c.message)
    }

    @Test fun aGuidanceProfileWithoutTheFlightComputersWordIsUnverified() {
        val p = profile(true)
        listOf(
            check(p, config(fc = true), connected = false) to "not connected",
            check(p, config(fc = true), baseStation = true) to "connect to the rocket directly to check",
            check(p, null) to "rocket settings not read yet",
            check(p, RocketConfig()) to "flight computer not reporting (power it on)",
            check(p, config(fc = null)) to "this rocket's firmware doesn't report it",
        ).forEach { (c, why) ->
            assertEquals(GuidanceStateCheck.Unverified(why), c)
            assertFalse(c.isMismatch)
        }
    }

    @Test fun aRollProfileWithNothingToCompareIsQuiet() {
        val p = profile(false)
        assertEquals(GuidanceStateCheck.NotApplicable, check(p, null))
        assertEquals(GuidanceStateCheck.NotApplicable, check(p, config(fc = null)))
        assertEquals(GuidanceStateCheck.NotApplicable, check(null, config(fc = true)))
    }

    // ── Adoption ──

    @Test fun adoptionTakesOnlyTheFlightComputersOnOff() {
        val stale = ActiveRocketSyncer.adopt(profile(false), config(fc = null, ge = true))
        assertFalse(stale.profile.guidanceEnabled, "the out computer's copy is not adopted")
        assertFalse(ActiveRocketSyncer.GROUP_GUIDANCE_ENABLE in stale.changed)

        val fc = ActiveRocketSyncer.adopt(profile(false), config(fc = true))
        assertTrue(fc.profile.guidanceEnabled, "the flight computer's own word wins (#915)")
        assertTrue(ActiveRocketSyncer.GROUP_GUIDANCE_ENABLE in fc.changed)
    }

    // ── Preflight ──

    @Test fun settingsStepBlocksOnAMismatch() {
        val base = PreflightAutoContext(
            isConnected = true, hasTelemetry = true,
            syncState = ActiveRocketSyncer.SyncState.Synced,
        )
        assertEquals(
            PreflightAutoStatus.Pending("Rocket guidance OFF — profile says ON"),
            PreflightChecklist.autoStatus(
                PreflightItemKind.SETTINGS_SYNCED, base.copy(guidance = GuidanceStateCheck.RocketOff),
            ),
        )
        assertEquals(
            PreflightAutoStatus.Satisfied,
            PreflightChecklist.autoStatus(
                PreflightItemKind.SETTINGS_SYNCED, base.copy(guidance = GuidanceStateCheck.Agrees),
            ),
        )
        assertEquals(
            PreflightAutoStatus.Satisfied,
            PreflightChecklist.autoStatus(
                PreflightItemKind.SETTINGS_SYNCED,
                base.copy(guidance = GuidanceStateCheck.Unverified("this rocket's firmware doesn't report it")),
            ),
            "older out computers never say; the dashboard carries that line",
        )
    }
}
