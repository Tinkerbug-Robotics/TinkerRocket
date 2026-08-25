package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import java.io.File
import java.util.UUID
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertIs
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * THE defaults pin: every RocketProfile default must equal the firmware
 * config.h factory default — the profile is pushed on connect and OVERRIDES
 * the FC, so a wrong default silently re-tunes the rocket.  Values cite
 * tinkerrocket-idf/projects/flight_computer/main/config.h.
 */
class FirmwareDefaultsTest {

    private val p = RocketProfile.makeDefault("x", nowMs = 0)

    @Test
    fun pidDefaults_matchConfigH() {
        assertEquals(0.12f, p.pidKp)                 // config.h:158 KP
        assertEquals(0.01f, p.pidKi)                 // config.h:159 KI
        assertEquals(0.0f, p.pidKd)                  // config.h:160 KD
        assertEquals(-20.0f, p.pidMinCmd)            // config.h:161 MIN_CMD
        assertEquals(20.0f, p.pidMaxCmd)             // config.h:162 MAX_CMD
        assertEquals(40f, p.integralSepThreshold)    // config.h:184 INTEGRAL_SEP_THRESHOLD_DPS
    }

    @Test
    fun servoDefaults_matchConfigH() {
        assertEquals(0, p.servoBias1)                // config.h:139-142 SERVO_BIAS_N (#561)
        assertEquals(0, p.servoBias4)
        assertEquals(333, p.servoHz)                 // config.h:144 SERVO_HZ
        assertEquals(1000, p.servoMinUs)             // config.h:145 SERVO_MIN_US
        assertEquals(2000, p.servoMaxUs)             // config.h:146 SERVO_MAX_US
    }

    @Test
    fun angleLoopDefaults_matchConfigH() {
        assertEquals(2.0f, p.kpAngle)                // config.h:280 KP_ANGLE
        assertEquals(60f, p.rateCapDps)              // config.h:288 KP_ANGLE_RATE_CAP_DPS
    }

    @Test
    fun guidanceDefaults_matchConfigH() {
        assertEquals(5.0f, p.pnNavGain)              // config.h:306 PN_NAV_GAIN
        assertEquals(20.0f, p.pnMaxAccel)            // config.h:308 PN_MAX_ACCEL_MPS2
        assertEquals(600.0f, p.pnTargetAltM)         // config.h:310 PN_TARGET_ALT_M
        assertEquals(15.0f, p.pnMaxFinDeg)           // config.h:322 PN_MAX_FIN_DEG
        assertEquals(15.0f, p.pnMinSpeed)            // config.h:345 PN_MIN_SPEED_MPS
        assertEquals(0, p.pnCoastDelayMs)            // config.h:347 PN_COAST_DELAY_MS
        assertEquals(4.0f, p.pnAccelToFin)           // config.h:349 PN_ACCEL_TO_FIN_DEG
        assertEquals(0, p.pnGuidanceLaw)             // config.h:363 GUIDANCE_LAW_DEFAULT
        assertEquals(0.8f, p.pnKpPos)                // config.h:378 PN_KP_POS_PER_S2
        assertEquals(1.5f, p.pnKdVel)                // config.h:379 PN_KD_VEL_PER_S
    }

    @Test
    fun cameraDefault_matchesConfigH() {
        assertEquals(2, p.cameraType)                // config.h:98 CAMERA_TYPE (RunCam)
    }

    @Test
    fun finCalibration_isDerived449() {
        assertEquals(120.0f, p.finTravelDeg)
        assertEquals(-60.0f, p.finMinDeg)
        assertEquals(60.0f, p.finMaxDeg)
    }
}

class RocketProfileCodecTest {

    @Test
    fun roundTrip_preservesEverything() {
        val original = RocketProfile.makeDefault("Atlas", nowMs = 1_753_000_000_000).copy(
            notes = "test",
            lastUsedUnitID = "a1b2",
            rollWaypoints = listOf(ProfileRollWaypoint(timeSeconds = 1.5f, angleDeg = 90f)),
            magCal = MagCalData(-321, 456, -789, 48.2f, 3.7f, "a1b2", 1_753_000_000_000),
            pidKp = 0.5f,
            finTravelDeg = 90f,
        )
        val decoded = RocketProfileCodec.decode(RocketProfileCodec.encode(original), nowMs = 0)
        assertNotNull(decoded)
        assertEquals(original.id, decoded.id)
        assertEquals(original.name, decoded.name)
        assertEquals(original.pidKp, decoded.pidKp)
        assertEquals(original.finTravelDeg, decoded.finTravelDeg)
        assertEquals(original.magCal, decoded.magCal)
        assertEquals(original.rollWaypoints.single().angleDeg, decoded.rollWaypoints.single().angleDeg)
        // Apple-epoch date round trip (ms precision within 1 ms of double math)
        assertTrue(kotlin.math.abs(original.createdAtMs - decoded.createdAtMs) <= 1)
    }

    @Test
    fun minimalJson_getsAllDefaults() {
        val decoded = RocketProfileCodec.decode("""{"name":"Old"}""", nowMs = 42)
        assertNotNull(decoded)
        assertEquals("Old", decoded.name)
        assertEquals(0.12f, decoded.pidKp)
        assertEquals(333, decoded.servoHz)
        assertNull(decoded.magCal)
    }

    @Test
    fun missingName_failsDecode() {
        assertNull(RocketProfileCodec.decode("""{"pidKp":0.5}""", nowMs = 0))
        assertNull(RocketProfileCodec.decode("not json", nowMs = 0))
    }

    @Test
    fun legacyFinKeys_deliberatelyIgnored449() {
        // The pre-#449 pathology: 1250/1750 µs declared as ±60° — no physical
        // servo.  The legacy angle keys must be IGNORED and travel derived
        // from the endpoints on the standard-servo line: 500 µs → 60° total.
        val decoded = RocketProfileCodec.decode(
            """{"name":"Legacy","servoMinUs":1250,"servoMaxUs":1750,
               "finMinDeg":-60.0,"finMaxDeg":60.0}""",
            nowMs = 0,
        )
        assertNotNull(decoded)
        assertEquals(60.0f, decoded.finTravelDeg)
        assertEquals(-30.0f, decoded.finMinDeg, "±30°, NOT the stored ±60 lie")
        assertEquals(30.0f, decoded.finMaxDeg)
    }

    @Test
    fun wrongLengthFinArrays_fallBackToDefaults() {
        val decoded = RocketProfileCodec.decode(
            """{"name":"X","finServoAtSlot":[1,2],"finReverse":[true]}""",
            nowMs = 0,
        )
        assertNotNull(decoded)
        assertEquals(listOf(1, 2, 3, 4), decoded.finServoAtSlot)
        assertEquals(listOf(false, false, false, false), decoded.finReverse)
    }

    @Test
    fun outOfRangeEnumInts_normalizeOnDecode() {
        // Older builds edited these as free-typed numbers.  The pickers that
        // replaced those fields can only DISPLAY a valid choice, so a stored
        // out-of-range value would show one thing while the syncer pushed
        // another — and an out-of-range pyro mode is a channel the FC (which
        // matches modes with ==) silently never fires.
        val decoded = RocketProfileCodec.decode(
            """{"name":"Legacy","cameraType":5,"imuOrientSetting":30,"imuRateHz":500,
               "pyro1TriggerMode":2,"pyro2TriggerMode":1}""",
            nowMs = 0,
        )
        assertNotNull(decoded)
        assertEquals(2, decoded.cameraType)          // clamped into 0..2
        assertEquals(0, decoded.imuOrientSetting)    // not 0..23 and not auto
        assertEquals(1920, decoded.imuRateHz)        // off the ODR whitelist
        assertEquals(1, decoded.pyro1TriggerMode)    // clamped into 0..1
        assertEquals(1, decoded.pyro2TriggerMode)    // valid, untouched
    }

    @Test
    fun validEnumInts_surviveDecodeUnchanged() {
        val decoded = RocketProfileCodec.decode(
            """{"name":"OK","cameraType":1,"imuOrientSetting":255,"imuRateHz":3840,
               "pyro1TriggerMode":0}""",
            nowMs = 0,
        )
        assertNotNull(decoded)
        assertEquals(1, decoded.cameraType)
        assertEquals(0xFF, decoded.imuOrientSetting)  // pad auto-detect
        assertEquals(3840, decoded.imuRateHz)
        assertEquals(0, decoded.pyro1TriggerMode)
    }
}

class RocketProfileStoreTest {

    private class MemActive : ActiveProfileStorage {
        var id: String? = null
        override fun loadActiveId() = id
        override fun saveActiveId(id: String?) { this.id = id }
    }

    private fun tempStore(active: MemActive = MemActive()): Pair<RocketProfileStore, File> {
        val dir = File.createTempFile("profiles", "").let { f -> f.delete(); File(f.path).apply { mkdirs() } }
        return RocketProfileStore(dir, active) { 1000 } to dir
    }

    @Test
    fun addSaveReload_roundTrips() {
        val active = MemActive()
        val (store, dir) = tempStore(active)
        val p = store.add("Atlas")
        store.setActive(p.id)

        val reloaded = RocketProfileStore(dir, active) { 2000 }
        assertEquals(1, reloaded.profiles.value.size)
        assertEquals("Atlas", reloaded.profiles.value.single().name)
        assertEquals(p.id, reloaded.activeId.value)
        assertEquals(p.id, reloaded.activeProfile?.id)
    }

    @Test
    fun corruptFile_losesOneProfileNotTheSet() {
        val (store, dir) = tempStore()
        store.add("Good")
        File(dir, "${UUID.randomUUID().toString().uppercase()}.json").writeText("{corrupt")

        val reloaded = RocketProfileStore(dir, MemActive()) { 0 }
        assertEquals(listOf("Good"), reloaded.profiles.value.map { it.name })
    }

    @Test
    fun delete_removesFileAndClearsActive() {
        val (store, dir) = tempStore()
        val p = store.add("Gone")
        store.setActive(p.id)
        store.delete(p.id)
        assertTrue(store.profiles.value.isEmpty())
        assertNull(store.activeId.value)
        assertEquals(0, dir.listFiles { f: File -> f.extension == "json" }.orEmpty().size)
    }

    @Test
    fun update_bumpsUpdatedAtAndPersists() {
        val active = MemActive()
        val (store, dir) = tempStore(active)
        val p = store.add("Tune")
        store.update(p.id) { it.copy(pidKp = 0.5f) }
        val reloaded = RocketProfileStore(dir, active) { 0 }
        assertEquals(0.5f, reloaded.profiles.value.single().pidKp)
    }

    @Test
    fun duplicateNames_deduped() {
        val (store, _) = tempStore()
        store.add("Kit")
        val second = store.add("Kit")
        assertEquals("Kit 2", second.name)
    }
}

/**
 * Syncer over a REAL DeviceSession + FakeFirmware — iOS ActiveRocketSyncerTests
 * semantics: push-on-ready, whole-profile command set, optimistic synced,
 * BS never pushed, cal advisories, suggestion.
 */
class ActiveRocketSyncerTest {

    private class Rig(
        val syncer: ActiveRocketSyncer,
        val session: DeviceSession,
        val fw: FakeFirmware,
        val store: RocketProfileStore,
    )

    private fun TestScope.rig(
        identityJson: String? =
            """{"type":"config_identity","uid":"boardA","un":"Atlas","nid":5,"rid":1,"dt":"R"}""",
        makeProfile: Boolean = true,
        mutate: (RocketProfile) -> RocketProfile = { it },
    ): Rig {
        val fw = FakeFirmware(backgroundScope).apply {
            configIdentityJson = identityJson
        }
        val session = DeviceSession(
            scope = backgroundScope,
            transport = fw,
            connectedDeviceName = "TR-R-Atlas",
            clock = { currentTime },
        )
        session.start()
        runCurrent()

        val dir = File.createTempFile("prof", "").let { f -> f.delete(); File(f.path).apply { mkdirs() } }
        val store = RocketProfileStore(
            dir,
            object : ActiveProfileStorage {
                var v: String? = null
                override fun loadActiveId() = v
                override fun saveActiveId(id: String?) { v = id }
            },
        ) { currentTime }
        if (makeProfile) {
            val p = store.add("Atlas Profile")
            store.update(p.id, mutate)
            store.setActive(p.id)
        }

        val syncer = ActiveRocketSyncer(backgroundScope)
        return Rig(syncer, session, fw, store)
    }

    private fun Rig.sentCommandIds(): List<Int> = fw.commandFrames.map { it[0].toInt() }

    /**
     * #915: connecting is not a command.  A board the app has never seen is
     * adopted as its own profile and NOTHING flight-affecting is written to
     * it — only the two cal READs, which ask rather than tell.
     */
    @Test
    fun connect_writesNoConfig_andAdoptsTheBoardAsItsOwnProfile() = runTest {
        val r = rig()
        r.syncer.attach(r.session, r.store)
        runCurrent()
        assertIs<ActiveRocketSyncer.SyncState.AwaitingSync>(r.syncer.syncState.value)

        advanceTimeBy(1100)   // choreography: config + identity land
        runCurrent()

        val sent = r.sentCommandIds().toSet()
        // The fourteen config frames #915 removed from the connect path.
        for (cmd in listOf(12, 13, 14, 22, 31, 26, 65, 66, 33, 64, 67, 11, 34)) {
            assertTrue(cmd !in sent, "cmd $cmd was pushed on connect (sent: $sent)")
        }
        // Cal is still READ (a question, not a write) — the profile has none.
        assertTrue(61 in sent && 63 in sent, "cal reads still go out")

        // The unknown board became its own profile, named from its identity.
        assertEquals("boardA", r.store.activeProfile?.lastUsedUnitID)
        assertEquals("Atlas", r.syncer.createdProfileName.value)
        assertIs<ActiveRocketSyncer.SyncState.Synced>(r.syncer.syncState.value)
    }

    /** A board with a bound profile makes that profile active, silently. */
    @Test
    fun knownBoard_bindsItsOwnProfile_insteadOfPushingTheActiveOne() = runTest {
        val r = rig()
        val mine = r.store.add("Atlas").let { p ->
            r.store.update(p.id) { it.copy(lastUsedUnitID = "boardA", pidKp = 0.5f) }
            p.id
        }
        val other = r.store.activeId.value
        assertTrue(other != mine, "a different profile starts active")

        r.syncer.attach(r.session, r.store)
        advanceTimeBy(1100)
        runCurrent()

        assertEquals(mine, r.store.activeId.value, "the board's own profile wins")
        assertNull(r.syncer.createdProfileName.value, "nothing was created")
        for (cmd in listOf(12, 13, 14, 22, 31, 26, 65, 66, 33, 64, 67, 11, 34)) {
            assertTrue(cmd !in r.sentCommandIds(), "binding must not push cmd $cmd")
        }
    }

    /** The old #132 behaviour, now reachable only on purpose. */
    @Test
    fun explicitPush_sendsTheWholeProfile() = runTest {
        val r = rig()
        r.syncer.attach(r.session, r.store)
        advanceTimeBy(1100)
        runCurrent()

        val before = r.fw.commandFrames.size
        r.syncer.pushProfileToRocket()
        runCurrent()

        val sent = r.sentCommandIds().drop(before).toSet()
        for (cmd in listOf(12, 13, 14, 22, 31, 26, 65, 66, 33, 64, 67, 11, 34)) {
            assertTrue(cmd in sent, "cmd $cmd missing from the explicit push (sent: $sent)")
        }
        assertIs<ActiveRocketSyncer.SyncState.Syncing>(r.syncer.syncState.value)
        advanceTimeBy(ActiveRocketSyncer.SYNCED_DELAY_MS)
        runCurrent()
        assertIs<ActiveRocketSyncer.SyncState.Synced>(r.syncer.syncState.value)
        assertEquals("boardA", r.store.activeProfile?.lastUsedUnitID)
    }

    @Test
    fun roleFlip_afterIdentityLands_reattachesAsBaseStation() = runTest {
        // The SUBSONIC case (#375): the advertised name parses as a rocket,
        // but the config_identity readback says base station.  The syncer
        // must fall through to a re-attach on its own — not stick in
        // AwaitingSync forever (nobody re-calls attach for it).
        val r = rig(
            identityJson =
                """{"type":"config_identity","uid":"bs1","un":"SUBSONIC","nid":5,"dt":"B"}""",
        )
        val before = r.fw.commandFrames.size
        r.syncer.attach(r.session, r.store)
        runCurrent()
        assertIs<ActiveRocketSyncer.SyncState.AwaitingSync>(r.syncer.syncState.value)

        advanceTimeBy(2000)   // identity lands, role corrects to BS
        runCurrent()
        assertIs<ActiveRocketSyncer.SyncState.Idle>(r.syncer.syncState.value)
        // The session's own connect choreography still runs (cmd 20 etc.) —
        // only PROFILE writes are forbidden on a BS link.
        val profileCmds = setOf(12, 13, 14, 22, 31, 26, 65, 66, 33, 64, 67, 11, 34, 55, 61, 62, 63)
        val sentAfter = r.sentCommandIds().drop(before)
        assertTrue(
            sentAfter.none { it in profileCmds },
            "profile writes to a BS: $sentAfter",
        )
    }

    @Test
    fun pushGroup_sendsOnlyThatGroup_andKeepsSynced() = runTest {
        val r = rig()
        r.syncer.attach(r.session, r.store)
        advanceTimeBy(1100)
        runCurrent()
        assertIs<ActiveRocketSyncer.SyncState.Synced>(r.syncer.syncState.value)

        val before = r.fw.commandFrames.size
        r.syncer.pushGroup(ActiveRocketSyncer.ConfigGroup.PID)
        runCurrent()
        // Self-apply (#144): the edit sends exactly one PID frame and the
        // badge stays Synced — no whole-profile re-push, no state churn.
        assertEquals(listOf(13), r.sentCommandIds().drop(before))
        assertIs<ActiveRocketSyncer.SyncState.Synced>(r.syncer.syncState.value)
    }

    /**
     * #915 replaced the "connected but no profile" dead end: a rocket that
     * reports an identity always ends up with a profile, seeded from its own
     * settings, so there is something to hold them in.
     */
    @Test
    fun noProfilesAtAll_theBoardStillGetsOne() = runTest {
        val r = rig(makeProfile = false)
        r.syncer.attach(r.session, r.store)
        advanceTimeBy(1100)
        runCurrent()
        assertNotNull(r.store.activeProfile)
        assertEquals("boardA", r.store.activeProfile?.lastUsedUnitID)
        assertIs<ActiveRocketSyncer.SyncState.Synced>(r.syncer.syncState.value)
    }

    @Test
    fun baseStation_neverPushed() = runTest {
        val r = rig(
            identityJson = """{"type":"config_identity","uid":"bs1","un":"BS","nid":5,"dt":"B"}""",
        )
        advanceTimeBy(1100)   // identity lands FIRST so the role is known
        runCurrent()
        val before = r.fw.commandFrames.size
        r.syncer.attach(r.session, r.store)
        advanceTimeBy(2000)
        runCurrent()
        assertIs<ActiveRocketSyncer.SyncState.Idle>(r.syncer.syncState.value)
        assertEquals(before, r.fw.commandFrames.size, "no profile writes to a BS")
    }

    @Test
    fun calOnAnotherBoard_warnsInsteadOfPushing() = runTest {
        // Bound to this board (#915) so it is the profile the connect reconciles;
        // its cal, though, was captured on a different one.
        val r = rig(
            mutate = {
                it.copy(
                    lastUsedUnitID = "boardA",
                    magCal = MagCalData(1, 2, 3, 48f, 3f, "OTHERBOARD", 0),
                )
            },
        )
        r.syncer.attach(r.session, r.store)
        advanceTimeBy(1100)
        runCurrent()
        val adv = assertIs<ActiveRocketSyncer.CalAdvisory.BoardMismatch>(r.syncer.magCalAdvisory.value)
        assertEquals("OTHERBOARD", adv.savedOn)
        assertEquals("boardA", adv.current)
        // Mismatch → no cmd 55 apply, and no cmd 61 read either.
        assertTrue(55 !in r.sentCommandIds())
        assertTrue(61 !in r.sentCommandIds())
    }

    @Test
    fun rocketAppliedCal_offersImport() = runTest {
        // Bound profile with no cal → connect-time READ.
        val r = rig(mutate = { it.copy(lastUsedUnitID = "boardA") })
        r.syncer.attach(r.session, r.store)
        advanceTimeBy(1100)
        runCurrent()
        assertTrue(61 in r.sentCommandIds(), "mag-cal READ sent")

        // The rocket answers: cal APPLIED → advisory offers import.
        r.fw.emitFileOpsFrame(
            byteArrayOf(0xCA.toByte()) + r.fw.magCalStatusFrame(subType = 3).drop(1).toByteArray(),
        )
        runCurrent()
        assertIs<ActiveRocketSyncer.CalAdvisory.RocketHasUnsavedCal>(r.syncer.magCalAdvisory.value)
    }

    @Test
    fun profileSwitch_rePushes() = runTest {
        val r = rig()
        r.syncer.attach(r.session, r.store)
        advanceTimeBy(1100)
        runCurrent()
        val afterFirst = r.fw.commandFrames.size

        val second = r.store.add("Backup")
        r.store.setActive(second.id)
        runCurrent()
        assertTrue(r.fw.commandFrames.size > afterFirst, "switch re-pushed the profile")
        assertNull(r.syncer.suggestedProfileId.value, "user chose — hint dropped")
    }

    // ── #915: the precedence rule (pure) ─────────────────────────────────

    /**
     * A readback whose values all match RocketProfile's factory defaults, so
     * a test only has to state the field it cares about.
     */
    private fun matchingConfig(): com.tinkerbug.tinkerrocket.protocol.RocketConfig {
        val p = RocketProfile.makeDefault("x", nowMs = 0)
        return com.tinkerbug.tinkerrocket.protocol.RocketConfig(
            servoBias1 = p.servoBias1, servoHz = p.servoHz,
            servoMinUs = p.servoMinUs, servoMaxUs = p.servoMaxUs,
            pidKp = p.pidKp, pidKi = p.pidKi, pidKd = p.pidKd,
            pidMinCmd = p.pidMinCmd, pidMaxCmd = p.pidMaxCmd,
            servoEnabled = p.servoControlEnabled,
            gainScheduleEnabled = p.gainScheduleEnabled,
            useAngleControl = p.useAngleControl, rollDelayMs = p.rollDelayMs,
            rateCapDps = p.rateCapDps, kpAngle = p.kpAngle,
            integralSepThreshold = p.integralSepThreshold,
            rollGainsReported = true,
            guidanceEnabled = p.guidanceEnabled, cameraType = p.cameraType,
            imuOrientSetting = p.imuOrientSetting, imuRateHz = p.imuRateHz,
            pyro1Enabled = p.pyro1Enabled, pyro1TriggerMode = p.pyro1TriggerMode,
            pyro1TriggerValue = p.pyro1TriggerValue,
            pyro2Enabled = p.pyro2Enabled, pyro2TriggerMode = p.pyro2TriggerMode,
            pyro2TriggerValue = p.pyro2TriggerValue,
            pyro3Enabled = p.pyro3Enabled, pyro3TriggerMode = p.pyro3TriggerMode,
            pyro3TriggerValue = p.pyro3TriggerValue,
            pyro4Enabled = p.pyro4Enabled, pyro4TriggerMode = p.pyro4TriggerMode,
            pyro4TriggerValue = p.pyro4TriggerValue,
        )
    }

    @Test
    fun agreement_reportsNothingChanged() {
        val p = RocketProfile.makeDefault("Rolly Polly", 0)
        assertEquals(emptyList(), ActiveRocketSyncer.adopt(p, matchingConfig()).changed)
    }

    @Test
    fun rocketValue_overwritesTheProfile() {
        val p = RocketProfile.makeDefault("Rolly Polly", 0).copy(pidKp = 0.30f)
        val r = ActiveRocketSyncer.adopt(p, matchingConfig().copy(pidKp = 0.12f))
        assertEquals(listOf(ActiveRocketSyncer.GROUP_PID_GAINS), r.changed)
        assertEquals(0.12f, r.profile.pidKp)
    }

    /**
     * The whole point of #915: the rocket the phone connected to must come
     * away flying what it already had, and the profile must say so.
     */
    @Test
    fun anotherAirframesOrientation_doesNotSurvive() {
        val p = RocketProfile.makeDefault("Wrong airframe", 0).copy(imuOrientSetting = 7)
        val r = ActiveRocketSyncer.adopt(p, matchingConfig().copy(imuOrientSetting = 0xFF))
        assertEquals(listOf(ActiveRocketSyncer.GROUP_IMU_ORIENTATION), r.changed)
        assertEquals(0xFF, r.profile.imuOrientSetting)
    }

    /** Wire rounding must not manufacture a diff on every single connect. */
    @Test
    fun wireRounding_isNotADifference() {
        val p = RocketProfile.makeDefault("x", 0)
            .copy(pidKp = 0.120004f, kpAngle = 2.001f, pyro2TriggerValue = 100.04f)
        assertEquals(emptyList(), ActiveRocketSyncer.adopt(p, matchingConfig()).changed)
    }

    /**
     * #253 sentinels mean "the firmware is on its own default", and
     * RocketConfig then holds the APP's defaults — adopting those would
     * overwrite a deliberately-tuned profile with a number nobody chose.
     */
    @Test
    fun unreportedRollGains_areLeftAlone() {
        val p = RocketProfile.makeDefault("x", 0).copy(rateCapDps = 120f, kpAngle = 5f)
        val r = ActiveRocketSyncer.adopt(p, matchingConfig().copy(rollGainsReported = false))
        assertEquals(emptyList(), r.changed)
        assertEquals(120f, r.profile.rateCapDps)
        assertEquals(5f, r.profile.kpAngle)
    }

    /** Firmware too old to report a field leaves the profile's value alone. */
    @Test
    fun fieldsThisFirmwareNeverReports_areKept() {
        val p = RocketProfile.makeDefault("x", 0).copy(imuOrientSetting = 7, imuRateHz = 1920)
        val r = ActiveRocketSyncer.adopt(
            p, matchingConfig().copy(imuOrientSetting = null, imuRateHz = null),
        )
        assertEquals(emptyList(), r.changed)
        assertEquals(7, r.profile.imuOrientSetting)
        assertEquals(1920, r.profile.imuRateHz)
    }

    /** Adoption may not silently reset what it cannot see. */
    @Test
    fun unreportedGroups_surviveAdoption() {
        val p = RocketProfile.makeDefault("x", 0).copy(
            servoBias2 = 40, finTravelDeg = 90f, finRingMode = 1,
            soundsEnabled = true, pnNavGain = 9f,
            rollWaypoints = listOf(ProfileRollWaypoint(timeSeconds = 1f, angleDeg = 90f)),
        )
        val r = ActiveRocketSyncer.adopt(p, matchingConfig())
        assertEquals(40, r.profile.servoBias2)
        assertEquals(90f, r.profile.finTravelDeg)
        assertEquals(1, r.profile.finRingMode)
        assertTrue(r.profile.soundsEnabled)
        assertEquals(9f, r.profile.pnNavGain)
        assertEquals(1, r.profile.rollWaypoints.size)
    }

    @Test
    fun eachPyroChannel_isNamedSeparately() {
        val p = RocketProfile.makeDefault("x", 0)
        val cfg = matchingConfig().copy(
            pyro1Enabled = !p.pyro1Enabled,
            pyro3TriggerValue = p.pyro3TriggerValue + 50f,
        )
        val r = ActiveRocketSyncer.adopt(p, cfg)
        assertEquals(listOf("Pyro 1", "Pyro 3"), r.changed)
    }

    @Test
    fun adoption_isIdempotent() {
        val p = RocketProfile.makeDefault("x", 0).copy(cameraType = 0)
        val cfg = matchingConfig().copy(cameraType = 1)
        val first = ActiveRocketSyncer.adopt(p, cfg)
        assertEquals(listOf(ActiveRocketSyncer.GROUP_CAMERA), first.changed)
        assertEquals(
            emptyList(), ActiveRocketSyncer.adopt(first.profile, cfg).changed,
        )
    }

    // -- #915 firmware config report (pure) --------------------------------

    private fun servoExtras() = com.tinkerbug.tinkerrocket.protocol.RocketServoExtras(
        bias2 = 0, bias3 = 0, bias4 = 0,
        finMinDeg = -60f, finMaxDeg = 60f,
        finAzimuths = listOf(0f, 90f, 180f, 270f),
        finReverseMask = 0, finRollReverseMask = 0, soundsEnabled = false,
    )

    private fun guidanceExtras(): com.tinkerbug.tinkerrocket.protocol.RocketGuidanceExtras {
        val p = RocketProfile.makeDefault("x", 0)
        return com.tinkerbug.tinkerrocket.protocol.RocketGuidanceExtras(
            navGain = p.pnNavGain, maxAccel = p.pnMaxAccel,
            accelToFin = p.pnAccelToFin, maxFinDeg = p.pnMaxFinDeg,
            minSpeed = p.pnMinSpeed, coastDelayMs = p.pnCoastDelayMs,
            targetMode = p.pnTargetMode, targetE = p.pnTargetE, targetN = p.pnTargetN,
            targetAltM = p.pnTargetAltM, kpPos = p.pnKpPos, kdVel = p.pnKdVel,
            guidanceLaw = p.pnGuidanceLaw,
        )
    }

    /** A rocket that reports everything and agrees about all of it. */
    private fun fullyReportingConfig() = matchingConfig().copy(
        servoExtras = servoExtras(),
        guidanceExtras = guidanceExtras(),
        rollWaypoints = emptyList(),
    )

    @Test
    fun fullReport_withNoDisagreement_changesNothing() {
        val p = RocketProfile.makeDefault("x", 0)
        assertEquals(emptyList(), ActiveRocketSyncer.adopt(p, fullyReportingConfig()).changed)
    }

    /**
     * The group that motivated the firmware work: a fin layout from another
     * airframe used to be invisible AND unverifiable.
     */
    @Test
    fun finLayout_isAdoptedFromTheRocket() {
        val p = RocketProfile.makeDefault("x", 0).copy(
            finServoAtSlot = listOf(4, 3, 2, 1),
            finRollReverse = listOf(true, false, false, false),
        )
        val r = ActiveRocketSyncer.adopt(p, fullyReportingConfig())
        assertEquals(listOf(ActiveRocketSyncer.GROUP_FIN_LAYOUT), r.changed)
        assertEquals(listOf(1, 2, 3, 4), r.profile.finServoAtSlot)
        assertEquals(listOf(false, false, false, false), r.profile.finRollReverse)
        assertEquals(0, r.profile.finRingMode)

        val rotated = fullyReportingConfig().copy(
            servoExtras = servoExtras().copy(finAzimuths = listOf(45f, 135f, 225f, 315f)),
        )
        assertEquals(1, ActiveRocketSyncer.adopt(r.profile, rotated).profile.finRingMode)
    }

    @Test
    fun servoTrim_finTravel_andSounds_areAdopted() {
        val p = RocketProfile.makeDefault("x", 0)
            .copy(servoBias3 = 55, finTravelDeg = 90f, soundsEnabled = true)
        val r = ActiveRocketSyncer.adopt(p, fullyReportingConfig())
        assertEquals(
            listOf(
                ActiveRocketSyncer.GROUP_SERVO_TRIM_24,
                ActiveRocketSyncer.GROUP_FIN_TRAVEL,
                ActiveRocketSyncer.GROUP_SOUNDS,
            ),
            r.changed,
        )
        assertEquals(0, r.profile.servoBias3)
        assertEquals(120f, r.profile.finTravelDeg)
        assertEquals(false, r.profile.soundsEnabled)
    }

    @Test
    fun guidanceParameters_areAdopted() {
        val p = RocketProfile.makeDefault("x", 0).copy(pnNavGain = 9f, pnGuidanceLaw = 1)
        val r = ActiveRocketSyncer.adopt(p, fullyReportingConfig())
        assertEquals(listOf(ActiveRocketSyncer.GROUP_GUIDANCE_PARAMS), r.changed)
        assertEquals(5.0f, r.profile.pnNavGain)
        assertEquals(0, r.profile.pnGuidanceLaw)
    }

    /**
     * "No waypoints" and "we can't see the waypoints" must not be the same
     * thing: one clears the profile's roll profile, the other must not.
     */
    @Test
    fun emptyWaypointList_isAnAnswer_butNullIsNot() {
        val withWps = RocketProfile.makeDefault("x", 0).copy(
            rollWaypoints = listOf(ProfileRollWaypoint(timeSeconds = 1f, angleDeg = 90f)),
        )
        val reported = ActiveRocketSyncer.adopt(withWps, fullyReportingConfig())
        assertEquals(listOf(ActiveRocketSyncer.GROUP_ROLL_PROFILE), reported.changed)
        assertTrue(reported.profile.rollWaypoints.isEmpty(), "the rocket flies rate-only")

        val unreported = ActiveRocketSyncer.adopt(
            withWps, fullyReportingConfig().copy(rollWaypoints = null),
        )
        assertEquals(emptyList(), unreported.changed)
        assertEquals(
            1, unreported.profile.rollWaypoints.size,
            "a rocket that can't report waypoints must not erase them",
        )
    }

    /**
     * Pre-report firmware, and the mini (no servo/fin/guidance hardware): the
     * profile keeps its own and the app says it cannot check.
     */
    @Test
    fun aGroupTheRocketCannotReport_isNamedAndLeftAlone() {
        val p = RocketProfile.makeDefault("x", 0).copy(servoBias2 = 40, pnNavGain = 9f)
        val cfg = matchingConfig()   // no extras at all
        val r = ActiveRocketSyncer.adopt(p, cfg)
        assertEquals(emptyList(), r.changed)
        assertEquals(40, r.profile.servoBias2)
        assertEquals(9f, r.profile.pnNavGain)
        assertEquals(
            listOf(
                "Servo trim 2-4", "Fin travel", "Fin layout", "Sounds",
                "Guidance parameters", "Roll profile",
            ),
            cfg.unreportedGroups,
        )
        assertEquals(
            emptyList(), fullyReportingConfig().unreportedGroups,
            "a reporting rocket leaves nothing unverifiable",
        )
    }

    @Test
    fun azimuthsThatDoNotDescribeFourSlots_areRefused() {
        // Two servos claiming the same slot can't be inverted into a mapping;
        // guessing one would put the wrong servo on the wrong fin.
        assertNull(ActiveRocketSyncer.slotsFromAzimuths(listOf(0f, 0f, 180f, 270f)))
        assertNull(ActiveRocketSyncer.slotsFromAzimuths(listOf(0f, 90f, 180f)))
        assertEquals(
            listOf(2, 3, 4, 1),
            ActiveRocketSyncer.slotsFromAzimuths(listOf(270f, 0f, 90f, 180f)),
        )
    }

    // -- Binding is exclusive (#915 bench regression) ----------------------

    /**
     * Bench 2026-08-25: assign a second profile to a rocket, connect to a
     * different rocket, come back — and the selection had reverted. Both
     * profiles still claimed the board, and the lookup takes the first match
     * in a list sorted by NAME, so the winner was decided alphabetically
     * instead of by what the user chose.
     */
    @Test
    fun assigningASecondProfile_releasesTheFirst() = runTest {
        val r = rig()
        val alpha = r.store.add("Alpha")   // sorts FIRST by name
        val zulu = r.store.add("Zulu")
        r.store.bind(alpha.id, "BOARD1")
        r.store.bind(zulu.id, "BOARD1")    // the user's later choice

        assertNull(
            r.store.profiles.value.first { it.id == alpha.id }.lastUsedUnitID,
            "the earlier profile must release the board",
        )
        assertEquals(
            "BOARD1",
            r.store.profiles.value.first { it.id == zulu.id }.lastUsedUnitID,
        )
        assertEquals(
            1, r.store.profiles.value.count { it.lastUsedUnitID == "BOARD1" },
            "a board is claimed by exactly one profile",
        )
    }

    /**
     * The symptom as reported: come back to the rocket and get the profile you
     * actually chose, not the alphabetically-earlier one.
     */
    @Test
    fun reconnect_bindsTheChosenProfile_notTheAlphabeticallyFirst() = runTest {
        val r = rig(makeProfile = false)
        val alpha = r.store.add("Alpha")
        val zulu = r.store.add("Zulu")
        r.store.bind(alpha.id, "boardA")
        r.store.bind(zulu.id, "boardA")    // chosen later, so it wins
        r.store.setActive(alpha.id)        // simulate being elsewhere

        r.syncer.attach(r.session, r.store)
        advanceTimeBy(1100)
        runCurrent()

        assertEquals(
            zulu.id, r.store.activeId.value,
            "the board comes back on the profile the user put on it",
        )
    }

    @Test
    fun suggestion_pure() {
        val a = RocketProfile.makeDefault("A", 0).copy(lastUsedUnitID = "u1")
        val b = RocketProfile.makeDefault("B", 0).copy(lastUsedUnitID = "u2")
        assertEquals(a.id, ActiveRocketSyncer.suggestedProfile(listOf(a, b), active = b.id, unitId = "u1"))
        assertNull(ActiveRocketSyncer.suggestedProfile(listOf(a, b), active = a.id, unitId = "u1"))
        assertNull(ActiveRocketSyncer.suggestedProfile(listOf(a, b), active = null, unitId = ""))
    }
}

/**
 * The connect-time sync gate (#836 item 4).
 *
 * The gate read `cfg != null && id.unitId != null`. `DeviceIdentity.unitId` is
 * a non-nullable String defaulting to "", so the id half was a CONSTANT TRUE
 * and the gate collapsed to the config half — which, by the gate's own comment,
 * arrives before the identity readback.
 *
 * The consequence was not a late sync but a WRONG one.  onReadyToSync ran with
 * unitId == "": bindProfileToBoard bailed on the empty id, so adoptRocketConfig
 * wrote the connected rocket's settings into whatever profile was ACTIVE, and
 * syncCal took magCalSyncAction's mismatch branch — the mag and sensor apply
 * frames were never sent and a BoardMismatch advisory was raised against an
 * empty id.  The collector is a first(), so nothing re-ran it when the real id
 * landed — the rocket flew on whatever calibration was in its NVS.
 */
class SyncGateTest {

    @Test
    fun gateStaysShutUntilBothHalvesArePresent() {
        // The ordering that broke it: config first, identity later.
        assertTrue(!ActiveRocketSyncer.readyToSync(hasConfig = false, unitId = ""))
        assertTrue(!ActiveRocketSyncer.readyToSync(hasConfig = true, unitId = ""))
        assertTrue(ActiveRocketSyncer.readyToSync(hasConfig = true, unitId = "A1B2C3"))
    }

    @Test
    fun aMissingIdentityIsEmptyNotNull() {
        // The reason a null check could never work: this is what an
        // un-received identity readback actually looks like.
        assertEquals("", DeviceIdentity().unitId)
    }

    @Test
    fun configWithoutIdentityDoesNotOpenTheGate() {
        // Named separately because this is the exact case that shipped: the
        // cmd-20 config frame lands ~1 s in, config_identity a beat later.
        assertTrue(
            !ActiveRocketSyncer.readyToSync(hasConfig = true, unitId = ""),
            "syncing on the config frame alone pushes calibration against an empty unit id",
        )
    }

    @Test
    fun syncingAgainstAnEmptyUnitIdWouldMismatchNotPush() {
        // Why the gate matters, rather than just when it opens. A stored cal is
        // never calibratedOnUnitID == "", so an empty id takes the else branch.
        val cal = MagCalData(
            offsetX = 1, offsetY = 2, offsetZ = 3,
            fieldRuT = 50f, residualUT = 1f,
            calibratedOnUnitID = "A1B2C3", calibratedAtMs = 0,
        )
        assertIs<ActiveRocketSyncer.Companion.CalAction.WarnMismatch>(
            ActiveRocketSyncer.magCalSyncAction(cal, ""),
        )
        assertIs<ActiveRocketSyncer.Companion.CalAction.Push>(
            ActiveRocketSyncer.magCalSyncAction(cal, "A1B2C3"),
        )
    }
}
