package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.protocol.GuidanceSendFlow
import com.tinkerbug.tinkerrocket.protocol.IMUOrientationMode
import com.tinkerbug.tinkerrocket.protocol.MagCalStatus
import com.tinkerbug.tinkerrocket.protocol.MagCalSubType
import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlinx.coroutines.async
import kotlinx.coroutines.launch
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertIs
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * DeviceSession behavior against the scripted [FakeFirmware], all under
 * runTest virtual time.  The iOS BLEDevice.swift flows are the reference:
 * connect choreography ORDER, readback folds, #377/#159/#385/#390 rules,
 * the #435 confirmation round-trip, and the disconnect wipe list.
 */
class DeviceSessionTest {

    private class Harness(val session: DeviceSession, val fw: FakeFirmware)

    private fun TestScope.startedSession(
        type: BleDeviceType = BleDeviceType.ROCKET,
        focusSeed: Int? = null,
        configure: FakeFirmware.() -> Unit = {},
    ): Harness {
        val fw = FakeFirmware(backgroundScope).apply(configure)
        val session = DeviceSession(
            scope = backgroundScope,
            transport = fw,
            connectedDeviceName = "TR-Test",
            initialDeviceType = type,
            clock = { currentTime },
        )
        focusSeed?.let { session.seedFocusRocket(it) }
        session.start()
        return Harness(session, fw)
    }

    // ── Connect choreography ─────────────────────────────────────────────

    @Test
    fun connectChoreographyOrder_timeSyncBeforeFileCccds() = runTest {
        val h = startedSession()
        runCurrent()
        // The exact op order up to the 1 s settle — time sync (cmd 9) lands
        // the INSTANT the command char is usable, BEFORE the file CCCDs.
        assertEquals(
            listOf(
                "mtu:517",
                "cccd:TELEMETRY",
                "write:COMMAND:9",
                "cccd:FILE_OPS",
                "cccd:FILE_TRANSFER",
            ),
            h.fw.ops.toList(),
        )
        // Time-sync payload is the injected clock (t=0 → 1970-01-01T00:00:00Z).
        val ts = h.fw.commandFrames.first { it[0].toInt() == BleCommandId.TIME_SYNC }
        assertContentEquals(Commands.timeSync(1970, 1, 1, 0, 0, 0), ts)

        // cmd 20 waits the full 1.0 s.
        advanceTimeBy(999)
        runCurrent()
        assertTrue(h.fw.ops.none { it == "write:COMMAND:20" })
        advanceTimeBy(1)
        runCurrent()
        assertTrue(h.fw.ops.contains("write:COMMAND:20"))
        // A rocket link never sends the BS focus pin.
        assertTrue(h.fw.ops.none { it == "write:COMMAND:45" })
    }

    @Test
    fun connectChoreography_baseStationRePinsFocusAfterConfig() = runTest {
        val h = startedSession(type = BleDeviceType.BASE_STATION, focusSeed = 7) {
            configIdentityJson =
                """{"type":"config_identity","uid":"b5","un":"BS","nid":1,"dt":"B"}"""
        }
        advanceTimeBy(1000)
        runCurrent()
        val commandOps = h.fw.ops.filter { it.startsWith("write:COMMAND") }
        assertEquals(
            listOf("write:COMMAND:9", "write:COMMAND:20", "write:COMMAND:45"),
            commandOps,
        )
        assertEquals(listOf(7), h.fw.focusPins)
    }

    // ── Config readback fold ─────────────────────────────────────────────

    @Test
    fun configReadback_sentinelsKeepDefaults_andValuesLand() = runTest {
        val h = startedSession {
            configJson =
                """{"type":"config","sb1":42,"rcap":0.0,"kpang":-1.0,"iwind":-0.5,"lf":915.5}"""
            configPyroJson = null
        }
        advanceTimeBy(1000)
        runCurrent()
        val cfg = assertNotNull(h.session.rocketConfig.value)
        assertEquals(42, cfg.servoBias1)
        // #253 sentinels: rcap <= 0, kpang <= 0, iwind < 0 → keep defaults.
        assertEquals(60f, cfg.rateCapDps)
        assertEquals(2.0f, cfg.kpAngle)
        assertEquals(40f, cfg.integralSepThreshold)
        assertEquals(915.5f, cfg.loraFreqMHz)
    }

    @Test
    fun configReadback_rebuildsFromDefaults_butPreservesPyro() = runTest {
        val h = startedSession {
            configJson = """{"type":"config","sb1":42}"""
            configPyroJson = null
        }
        advanceTimeBy(1000)
        runCurrent()
        assertEquals(42, h.session.rocketConfig.value?.servoBias1)

        h.fw.emitTelemetryJson("""{"type":"config_pyro","p1e":true,"p1v":123.0}""")
        runCurrent()
        assertEquals(true, h.session.rocketConfig.value?.pyro1Enabled)

        // A fresh config frame with NO keys: non-pyro fields reset to
        // DEFAULTS (absent key never keeps the previous value), pyro carried.
        h.fw.emitTelemetryJson("""{"type":"config"}""")
        runCurrent()
        val cfg = assertNotNull(h.session.rocketConfig.value)
        assertEquals(0, cfg.servoBias1)          // NOT the previous 42
        assertEquals(true, cfg.pyro1Enabled)     // preserved from config_pyro
        assertEquals(123f, cfg.pyro1TriggerValue)
    }

    // ── Pyro continuity test (cmd 35) ────────────────────────────────────

    @Test
    fun pyroContTest_sendsFrameAndOpensTestingWindow() = runTest {
        val h = startedSession()
        advanceTimeBy(1000)
        runCurrent()

        h.session.sendPyroContTest(2)
        runCurrent()

        // Wire bytes: [35, channel] (golden-pinned in CommandsGoldenTest).
        assertContentEquals(
            Commands.pyroContTest(2),
            h.fw.commandFrames.first { it[0].toInt() == BleCommandId.PYRO_CONT_TEST },
        )
        // Per-channel window — only the tested channel shows TESTING.
        assertTrue(h.session.contTestPending(2))
        assertFalse(h.session.contTestPending(1))

        // Window is exactly the iOS 2.5 s round-trip allowance.
        advanceTimeBy(DeviceSession.CONT_TEST_PENDING_WINDOW_MS - 1)
        assertTrue(h.session.contTestPending(2))
        advanceTimeBy(1)
        assertFalse(h.session.contTestPending(2))

        // A re-test reopens (iOS: newer tap extends the deadline).
        h.session.sendPyroContTest(2)
        runCurrent()
        assertTrue(h.session.contTestPending(2))
    }

    @Test
    fun mirrorPyroConfig_appliesAllFourChannels_onlyOverAReadback() = runTest {
        val h = startedSession { configPyroJson = null; configJson = null }
        runCurrent()
        val channels = List(4) { i ->
            com.tinkerbug.tinkerrocket.protocol.PyroChannelConfig(
                enabled = i == 1, mode = 1, value = 42f,
            )
        }

        // No readback yet: the mirror must NOT fabricate a config, or the
        // dashboard would render defaults as device-reported truth (iOS
        // mirrors only inside `if var cfg = device.rocketConfig`).
        h.session.mirrorPyroConfig(channels)
        runCurrent()
        assertNull(h.session.rocketConfig.value)

        // Once a readback exists, the mirror lands — all four channels, the
        // same payload cmd 34 carried.
        h.fw.emitTelemetryJson("""{"type":"config_pyro","p1e":true,"p1v":1.0}""")
        runCurrent()
        h.session.mirrorPyroConfig(channels)
        runCurrent()
        val cfg = assertNotNull(h.session.rocketConfig.value)
        assertFalse(cfg.pyro1Enabled)            // the readback's true, overwritten
        assertTrue(cfg.pyro2Enabled)
        assertEquals(42f, cfg.pyro4TriggerValue)
        assertEquals(1, cfg.pyro3TriggerMode)
    }

    // ── Identity readback ────────────────────────────────────────────────

    @Test
    fun identityReadback_appliesFields_andDeviceTypeRules() = runTest {
        val h = startedSession()
        advanceTimeBy(1000)
        runCurrent()
        val id = h.session.identity.value
        assertEquals("a1b2c3d4", id.unitId)
        assertEquals("Atlas", id.unitName)
        assertEquals(1, id.networkId)
        assertEquals(3, id.rocketId)
        assertEquals(BleDeviceType.ROCKET, id.deviceType)
        assertEquals("abc1234+20260722-0900", id.firmwareVersion)

        // Unrecognized "dt" keeps the previous type (iOS rawValue init fails)…
        h.fw.emitTelemetryJson("""{"type":"config_identity","dt":"X"}""")
        runCurrent()
        assertEquals(BleDeviceType.ROCKET, h.session.identity.value.deviceType)
        // …but "?" is a VALID raw value and really sets UNKNOWN (iOS parity).
        h.fw.emitTelemetryJson("""{"type":"config_identity","dt":"?"}""")
        runCurrent()
        assertEquals(BleDeviceType.UNKNOWN, h.session.identity.value.deviceType)
    }

    @Test
    fun fcIdentity_updatesOnlyFcFirmware() = runTest {
        val h = startedSession { fcIdentityJson = """{"type":"fc_identity","fc_fw":"fc99"}""" }
        advanceTimeBy(1000)
        runCurrent()
        assertEquals("fc99", h.session.identity.value.fcFirmwareVersion)
        assertEquals("abc1234+20260722-0900", h.session.identity.value.firmwareVersion)
    }

    // ── imu_orient caching gate ──────────────────────────────────────────

    @Test
    fun imuOrient_settingCachedOnlyWhenConfigExists() = runTest {
        val h = startedSession {
            configJson = null
            configPyroJson = null
            configIdentityJson = null
            pushEchoWithConfig = false
        }
        runCurrent()
        // Before any config readback: name/mode land, "set" is NOT cached.
        // NOTE: the imu_orient JSON 'mode' uses the IMUOrientationMode enum
        // raws (0 default / 1 manual / 2 autoSnap / 3 autoExact) — NOT the
        // packed telemetry 'imo' bitfield encoding (1 default / 2 manual /
        // 3 auto).  Two different wire vocabularies for the same concept.
        h.fw.emitTelemetryJson("""{"type":"imu_orient","name":"+X","mode":1,"set":255}""")
        runCurrent()
        assertEquals("+X", h.session.imuOrientationName.value)
        assertEquals(IMUOrientationMode.MANUAL, h.session.imuOrientationMode.value)
        assertNull(h.session.rocketConfig.value)

        h.fw.emitTelemetryJson("""{"type":"config"}""")
        runCurrent()
        assertNull(h.session.rocketConfig.value?.imuOrientSetting)

        h.fw.emitTelemetryJson("""{"type":"imu_orient","name":"-Z r90","mode":3,"set":3}""")
        runCurrent()
        assertEquals(3, h.session.rocketConfig.value?.imuOrientSetting)
        assertEquals("-Z r90", h.session.imuOrientationName.value)
    }

    // ── #377 telemetry gate ──────────────────────────────────────────────

    @Test
    fun hasReceivedTelemetry_falseUntilFirstRealFrame() = runTest {
        val h = startedSession()
        advanceTimeBy(1000)
        runCurrent()
        // Config/identity/echo readbacks arrived on the telemetry char — the
        // gate must NOT flip for them (they are not telemetry frames).
        assertNotNull(h.session.rocketConfig.value)
        assertFalse(h.session.hasReceivedTelemetry.value)

        h.fw.emitTelemetry(state = "READY")
        runCurrent()
        assertTrue(h.session.hasReceivedTelemetry.value)
    }

    @Test
    fun hasReceivedTelemetry_relayedFrameAlsoConfirms() = runTest {
        // iOS PowerStateGateTests.RelayedFrame_AlsoConfirms: the gate flips
        // BEFORE the relay early-return.
        val h = startedSession(type = BleDeviceType.BASE_STATION) {
            configIdentityJson = null
        }
        runCurrent()
        h.fw.emitTelemetry(state = "READY", sourceRocketId = 1)
        runCurrent()
        assertTrue(h.session.hasReceivedTelemetry.value)
    }

    @Test
    fun hasReceivedTelemetry_garbageFrameDoesNotConfirm() = runTest {
        // iOS PowerStateGateTests.GarbageFrame_DoesNotConfirm.
        val h = startedSession { configJson = null; configPyroJson = null; configIdentityJson = null; pushEchoWithConfig = false }
        runCurrent()
        h.fw.emitTelemetryJson("not json at all")
        runCurrent()
        assertFalse(h.session.hasReceivedTelemetry.value)
    }

    @Test
    fun hasReceivedTelemetry_firstFrameConfirmsPowerOff() = runTest {
        // iOS PowerStateGateTests.FirstFrame_ConfirmsPowerOff: a frame with
        // pwr_pin OFF still confirms (the point is KNOWING, not being on).
        val h = startedSession()
        runCurrent()
        h.fw.emitTelemetry(state = "READY", pwrPinOn = false)
        runCurrent()
        assertTrue(h.session.hasReceivedTelemetry.value)
        assertFalse(h.session.telemetry.value.pwrPinOn)
    }

    @Test
    fun relayedEmptyUnitName_neverClobbersLearnedName() = runTest {
        // iOS RemoteRocket.updateTelemetry guards !name.isEmpty.
        val h = startedSession(type = BleDeviceType.BASE_STATION) {
            configIdentityJson = null
        }
        runCurrent()
        h.fw.emitTelemetry(state = "READY", sourceRocketId = 1, sourceUnitName = "Atlas")
        runCurrent()
        h.fw.emitTelemetry(state = "READY", sourceRocketId = 1, sourceUnitName = "")
        runCurrent()
        assertEquals("Atlas", h.session.remoteRockets.value.single().unitName)
    }

    // ── #159 power-on watchdog ───────────────────────────────────────────

    @Test
    fun poweringOn_watchdogClearsAfter180s() = runTest {
        val h = startedSession()
        runCurrent()
        h.session.beginPowerOn()
        runCurrent()
        assertTrue(h.session.poweringOn.value)
        assertTrue(h.fw.ops.contains("write:COMMAND:8"))
        advanceTimeBy(179_999)
        runCurrent()
        assertTrue(h.session.poweringOn.value)
        advanceTimeBy(1)
        runCurrent()
        assertFalse(h.session.poweringOn.value)
    }

    @Test
    fun poweringOn_clearedByPwrPinOnFrame() = runTest {
        val h = startedSession { powerToggleReplyDelayMs = 5_000 }
        runCurrent()
        h.session.beginPowerOn()
        runCurrent()
        assertTrue(h.session.poweringOn.value)
        advanceTimeBy(5_000)
        runCurrent()
        assertFalse(h.session.poweringOn.value)
        assertTrue(h.session.telemetry.value.pwrPinOn)
    }

    // ── File list paging ─────────────────────────────────────────────────

    @Test
    fun fileList_writeThenDelayedExplicitRead_hasMoreEdges() = runTest {
        val h = startedSession {
            repeat(12) { deviceFiles += FakeFirmware.FakeFile("flight_$it.bin", ByteArray(10)) }
        }
        runCurrent()
        h.session.requestFileList(0)
        runCurrent()
        assertTrue(h.fw.ops.contains("write:COMMAND:2"))
        // The list rides the EXPLICIT read 0.5 s later — nothing lands early.
        assertTrue(h.session.files.value.isEmpty())
        assertTrue(h.fw.ops.none { it == "read:FILE_OPS" })
        advanceTimeBy(500)
        runCurrent()
        assertTrue(h.fw.ops.contains("read:FILE_OPS"))
        assertEquals(5, h.session.files.value.size)
        assertTrue(h.session.hasMoreFiles.value)
        assertEquals(0, h.session.currentPage.value)

        // Last, partial page: 12 files → page 2 has 2 entries, no more.
        h.session.requestFileList(2)
        advanceTimeBy(500)
        runCurrent()
        assertEquals(2, h.session.files.value.size)
        assertFalse(h.session.hasMoreFiles.value)
        assertEquals(2, h.session.currentPage.value)
    }

    @Test
    fun fileList_exactMultipleOfPageSize_showsPhantomMorePage() = runTest {
        val h = startedSession {
            repeat(10) { deviceFiles += FakeFirmware.FakeFile("flight_$it.bin", ByteArray(10)) }
        }
        runCurrent()
        // Page 1 is FULL (files 5..9) → the count==5 heuristic claims more…
        h.session.requestFileList(1)
        advanceTimeBy(500)
        runCurrent()
        assertEquals(5, h.session.files.value.size)
        assertTrue(h.session.hasMoreFiles.value)
        // …and the final page is empty (pinned iOS behavior).
        h.session.requestFileList(2)
        advanceTimeBy(500)
        runCurrent()
        assertEquals(0, h.session.files.value.size)
        assertFalse(h.session.hasMoreFiles.value)
    }

    // ── #390 staleness worsen-only ───────────────────────────────────────

    @Test
    fun effectiveDataStatus_worsensOnlyPast3000ms_onRssiTick() = runTest {
        val h = startedSession(type = BleDeviceType.BASE_STATION) {
            configIdentityJson = null   // keep the seeded BS type
        }
        runCurrent()
        // Emit OFF the ticker epoch so the 2 s ticks observe ages 1000/3000/
        // 5000 — age 3000 must stay LIVE, pinning the strict `> 3000` rule
        // (an `>= 3000` or any 2000-4000 threshold fails this test).
        advanceTimeBy(1000)
        h.fw.emitTelemetry(state = "READY", sourceRocketId = 1, sourceUnitName = "Atlas", dataStatusRaw = 0)
        runCurrent()
        assertEquals(1, h.session.focusRocketId.value)
        assertEquals(TelemetryData.DataStatus.LIVE, h.session.effectiveDataStatus.value)

        // Tick at 2 s: age 1000 ≤ 3000 → LIVE.
        advanceTimeBy(1000)
        runCurrent()
        assertEquals(1000L, h.session.focusedRelayAgeMs.value)
        assertEquals(TelemetryData.DataStatus.LIVE, h.session.effectiveDataStatus.value)

        // Tick at 4 s: age EXACTLY 3000 → still LIVE (strictly greater).
        advanceTimeBy(2000)
        runCurrent()
        assertEquals(3000L, h.session.focusedRelayAgeMs.value)
        assertEquals(TelemetryData.DataStatus.LIVE, h.session.effectiveDataStatus.value)

        // Tick at 6 s: age 5000 > 3000 → worsened to STALE while the frame
        // still SAYS live (the BS re-push freeze this overlay exists for).
        advanceTimeBy(2000)
        runCurrent()
        assertEquals(TelemetryData.DataStatus.STALE, h.session.effectiveDataStatus.value)
        assertEquals(TelemetryData.DataStatus.LIVE, h.session.telemetry.value.dataStatus)
        assertEquals(5000L, h.session.effectiveDataAgeMs.value)

        // SYNCING is never overridden by the overlay.
        h.fw.emitTelemetry(sourceRocketId = 1, dataStatusRaw = 2)
        runCurrent()
        assertEquals(TelemetryData.DataStatus.SYNCING, h.session.effectiveDataStatus.value)

        // A fresh LIVE frame does NOT improve the overlay mid-tick (the age
        // only refreshes on the 2 s tick — iOS parity)…
        h.fw.emitTelemetry(sourceRocketId = 1, dataStatusRaw = 0)
        runCurrent()
        assertEquals(TelemetryData.DataStatus.STALE, h.session.effectiveDataStatus.value)
        // …and the next tick recomputes the age from the fresh frame → LIVE.
        advanceTimeBy(2000)
        runCurrent()
        assertEquals(TelemetryData.DataStatus.LIVE, h.session.effectiveDataStatus.value)

        // A frame-carried STALE stays STALE even with a fresh age (worsen-only).
        h.fw.emitTelemetry(sourceRocketId = 1, dataStatusRaw = 1, dataAgeMs = 9000)
        runCurrent()
        assertEquals(TelemetryData.DataStatus.STALE, h.session.effectiveDataStatus.value)
        assertEquals(9000L, h.session.effectiveDataAgeMs.value)
    }

    @Test
    fun relayedTelemetry_onlyFocusedRocketMirrors() = runTest {
        val h = startedSession(type = BleDeviceType.BASE_STATION) {
            configIdentityJson = null
        }
        runCurrent()
        h.fw.emitTelemetry(state = "READY", sourceRocketId = 1, sourceUnitName = "Atlas")
        runCurrent()
        h.fw.emitTelemetry(state = "BOOST", sourceRocketId = 2, sourceUnitName = "Nova")
        runCurrent()
        // Focus latched on the FIRST rocket heard; rocket 2 tracked but not mirrored.
        assertEquals(1, h.session.focusRocketId.value)
        assertEquals("READY", h.session.telemetry.value.state)
        assertEquals(
            listOf(1, 2),
            h.session.remoteRockets.value.map { it.rocketId },
        )
        // Explicit switch re-pins immediately from the roster cache + cmd 45.
        h.session.setFocusRocket(2)
        runCurrent()
        assertEquals("BOOST", h.session.telemetry.value.state)
        assertEquals(listOf(2), h.fw.focusPins)
    }

    // ── Sim banner latch ─────────────────────────────────────────────────

    @Test
    fun simBanner_clearsOnlyAfterNonReadyThenReady() = runTest {
        val h = startedSession()
        runCurrent()
        h.session.markSimLaunched()
        runCurrent()
        assertTrue(h.session.simLaunched.value)

        // Still READY (sim not started on the FC yet) — banner stays.
        h.fw.emitTelemetry(state = "READY")
        runCurrent()
        assertTrue(h.session.simLaunched.value)

        h.fw.emitTelemetry(state = "BOOST")
        runCurrent()
        assertTrue(h.session.simLaunched.value)

        // Sim flight finished, FC back to READY → banner clears.
        h.fw.emitTelemetry(state = "READY")
        runCurrent()
        assertFalse(h.session.simLaunched.value)
    }

    // ── Mag-cal frames: SharedFlow, never conflated ──────────────────────

    @Test
    fun magCalFrames_everyLadderFrameDelivered() = runTest {
        val h = startedSession {
            magCalLadder = listOf(
                magCalStatusFrame(subType = 1, sampleCount = 10),
                magCalStatusFrame(subType = 1, sampleCount = 20),
                magCalStatusFrame(subType = 2, sampleCount = 30),
            )
        }
        runCurrent()
        val seen = mutableListOf<MagCalStatus>()
        backgroundScope.launch { h.session.magCalFrames.collect { seen += it } }
        runCurrent()
        h.session.sendBareCommand(BleCommandId.MAG_CAL_START_OC)
        runCurrent()
        // All three frames observed IN ORDER — a StateFlow would have
        // conflated the two SAMPLING heartbeats away.
        assertEquals(listOf(10, 20, 30), seen.map { it.sampleCount })
        assertEquals(MagCalSubType.REVIEW, h.session.magCalStatus.value?.subType)
    }

    // ── Frequency scan ───────────────────────────────────────────────────

    @Test
    fun frequencyScan_resultClearsSpinner_wedgeTimeoutAt75s() = runTest {
        val h = startedSession {
            scanResponse = scanFrame(902.0f, 500.0f, listOf(-120, -95, -110))
            scanReplyDelayMs = 10_000
        }
        runCurrent()
        h.session.startFrequencyScan(902.0f, 928.0f, 500, 10)
        runCurrent()
        assertTrue(h.session.isScanning.value)
        advanceTimeBy(10_000)
        runCurrent()
        assertFalse(h.session.isScanning.value)
        assertEquals(3, h.session.scanSamples.value.size)
        assertEquals(-95, h.session.scanSamples.value[1].rssiDbm)

        // Lost result: #385 wedge timeout clears the spinner at 75 s.
        h.fw.scanResponse = null
        h.session.startFrequencyScan(902.0f, 928.0f, 500, 10)
        runCurrent()
        assertTrue(h.session.isScanning.value)
        advanceTimeBy(74_999)
        runCurrent()
        assertTrue(h.session.isScanning.value)
        advanceTimeBy(1)
        runCurrent()
        assertFalse(h.session.isScanning.value)
    }

    // ── #435 guidance echo round-trip ────────────────────────────────────

    @Test
    fun guidancePoint_confirmedViaEchoRoundTrip() = runTest {
        val h = startedSession { echoSeq = 5 }
        advanceTimeBy(1000)
        runCurrent()
        // Connect-time echo push seeds the baseline.
        assertEquals(5, h.session.guidanceEcho.value?.seq)

        val verdict = async {
            h.session.sendGuidancePointConfirming(37.123456, -122.654321, 150f)
        }
        runCurrent()
        assertEquals(GuidanceSendFlow.Verdict.Confirmed, verdict.await())
        assertEquals(6, h.session.guidanceEcho.value?.seq)
        assertEquals(1, h.session.guidanceEcho.value?.lastRc)
    }

    @Test
    fun guidancePoint_rejectIsProvisional_settlesFailedAtTimeout() = runTest {
        val h = startedSession { echoSeq = 5 }
        advanceTimeBy(1000)
        runCurrent()
        h.fw.guidanceRc = 2   // GUID_RC_REJ_RADIUS

        val verdict = async {
            h.session.sendGuidancePointConfirming(37.0, -122.0, 150f)
        }
        runCurrent()
        // The reject frame arrived but is only PROVISIONAL (an interleaved
        // cmd-65 apply could carry a stale rc) — no verdict before timeout.
        assertFalse(verdict.isCompleted)
        advanceTimeBy(6_000)
        runCurrent()
        val failed = assertIs<GuidanceSendFlow.Verdict.Failed>(verdict.await())
        assertTrue(failed.message.contains("100 m"), failed.message)
    }

    // ── Disconnect wipe ──────────────────────────────────────────────────

    @Test
    fun disconnect_wipesExactlyWhatIosWipes() = runTest {
        val h = startedSession {
            repeat(3) { deviceFiles += FakeFirmware.FakeFile("flight_$it.bin", ByteArray(10)) }
        }
        advanceTimeBy(1000)
        runCurrent()
        // Populate everything on the wipe list (and some keep-list state).
        h.fw.emitTelemetry(state = "BOOST")
        h.fw.emitFileOpsFrame(h.fw.magCalStatusFrame(subType = 2, sampleCount = 40))
        runCurrent()
        h.session.markSimLaunched()
        h.session.beginPowerOn()
        h.session.requestFileList(0)
        h.session.sendPyroContTest(3)
        advanceTimeBy(500)
        runCurrent()
        advanceTimeBy(1500)   // an RSSI tick populates connectedRssi
        runCurrent()

        assertNotNull(h.session.rocketConfig.value)
        assertNotNull(h.session.magCalStatus.value)
        assertNotNull(h.session.guidanceEcho.value)
        assertEquals(3, h.session.files.value.size)
        assertEquals(-50, h.session.connectedRssi.value)
        assertTrue(h.session.hasReceivedTelemetry.value)

        h.fw.fireDisconnect()
        runCurrent()

        // Wiped (iOS onDisconnect list):
        assertFalse(h.session.isConnected.value)
        assertTrue(h.session.files.value.isEmpty())
        assertEquals(0, h.session.currentPage.value)
        assertFalse(h.session.hasMoreFiles.value)
        assertFalse(h.session.simLaunched.value)
        assertFalse(h.session.poweringOn.value)
        assertNull(h.session.rocketConfig.value)
        assertTrue(h.session.contTestPendingUntil.value.isEmpty())
        assertNull(h.session.magCalStatus.value)
        assertNull(h.session.sensorCalStatus.value)
        assertNull(h.session.guidanceEcho.value)
        assertFalse(h.session.hasReceivedTelemetry.value)
        assertNull(h.session.connectedRssi.value)

        // NOT wiped (iOS keeps these on the instance):
        assertEquals("BOOST", h.session.telemetry.value.state)
        assertEquals("a1b2c3d4", h.session.identity.value.unitId)
        assertEquals("Atlas", h.session.identity.value.unitName)

        // The RSSI ticker is stopped — no further polls after disconnect.
        val rssiOps = h.fw.ops.count { it == "rssi" }
        advanceTimeBy(10_000)
        runCurrent()
        assertEquals(rssiOps, h.fw.ops.count { it == "rssi" })
    }

    // ── #634 bulk delete ─────────────────────────────────────────────────

    @Test
    fun deleteFiles_issuesEveryDeleteInOrderAndStripsThemLocally() = runTest {
        val h = startedSession {
            repeat(5) { i ->
                deviceFiles += FakeFirmware.FakeFile("flight_$i.bin", ByteArray(64))
            }
        }
        advanceTimeBy(1_200); runCurrent()
        h.session.requestFileList(0)
        advanceTimeBy(500); runCurrent()
        val listed = h.session.files.value.map { it.name }
        assertTrue(listed.size >= 3, "need a populated list to delete from, got $listed")

        val doomed = listOf(listed[0], listed[2])
        h.session.deleteFiles(doomed)
        advanceTimeBy(2_000); runCurrent()

        // Every name went out as its own cmd 3, in the order given — one
        // coroutine feeding the op queue, not N racing ones.
        val deletes = h.fw.commandFrames
            .filter { it[0].toInt() == BleCommandId.FILE_DELETE }
            .map { String(it.copyOfRange(1, it.size), Charsets.UTF_8) }
        assertEquals(doomed, deletes)

        // ...and the list is optimistically stripped of exactly those.
        val left = h.session.files.value.map { it.name }
        for (name in doomed) assertFalse(name in left, "$name still listed after delete")
        assertEquals(listed.size - doomed.size, left.size)
    }

    @Test
    fun deleteFiles_emptyListIsANoOp() = runTest {
        val h = startedSession()
        advanceTimeBy(1_200); runCurrent()
        val before = h.fw.commandFrames.count { it[0].toInt() == BleCommandId.FILE_DELETE }
        h.session.deleteFiles(emptyList())
        advanceTimeBy(500); runCurrent()
        assertEquals(
            before,
            h.fw.commandFrames.count { it[0].toInt() == BleCommandId.FILE_DELETE },
        )
    }
}
