package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.protocol.ConfigIdentityMsg
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNotSame
import kotlin.test.assertNull
import kotlin.test.assertSame
import kotlin.test.assertTrue

/**
 * FleetManager scanner policy (#547/#394), connect flow, identity precedence
 * (registry > TR- prefix > legacy substring, overridden by config_identity
 * "dt"), fleet-scoped focus (#390), and relay-aware routing (cmd-50 duality).
 * Behavioral reference: iOS BLEFleet.swift (+ the seedTypeFromRegistry /
 * DiscoveredDevice precedence and RocketFocusPinningTests' fleet-scoped
 * cases).
 */
class FleetManagerTest {

    private fun identity(
        uid: String? = null,
        un: String? = null,
        nid: Int? = null,
        rid: Int? = null,
        dt: String? = null,
        fw: String? = null,
    ) = ConfigIdentityMsg(
        unitId = uid, unitName = un, networkId = nid,
        rocketId = rid, deviceType = dt, firmwareVersion = fw,
    )

    // ------------------------------------------------------------- scanning

    @Test
    fun scan_usesScanRecordName_neverFiltersOnIt() = runTest {
        val h = fleetHarness()
        h.fleet.scan()
        runCurrent()
        assertTrue(h.fleet.isScanning.value)
        assertEquals("Scanning...", h.fleet.statusMessage.value)

        // #547: a device renamed through My Devices advertises the raw name
        // ("SUBSONIC" — no TR- prefix, would fail any name filter) and MUST
        // still be listed: the service UUID already proved it's ours.
        h.scanner.emissions.emit(BleAdvertisement("aa:01", "SUBSONIC", -42))
        runCurrent()
        val listed = h.fleet.discoveredDevices.value.single()
        assertEquals("SUBSONIC", listed.name)
        assertEquals(-42, listed.rssi)
        assertEquals("Found 1 device", h.fleet.statusMessage.value)
    }

    @Test
    fun scan_advertisementWithoutLocalName_listsAsUnknown() = runTest {
        // No stack-cached-name fallback on Android (#547): a scan record
        // with no local name displays as "Unknown" but is still listed.
        val h = fleetHarness()
        h.fleet.scan()
        runCurrent()
        h.scanner.emissions.emit(BleAdvertisement("aa:02", null, -60))
        runCurrent()
        assertEquals("Unknown", h.fleet.discoveredDevices.value.single().name)
    }

    @Test
    fun scan_rediscovery_updatesRssiKeepsFirstName() = runTest {
        val h = fleetHarness()
        h.fleet.scan()
        runCurrent()
        h.scanner.emissions.emit(BleAdvertisement("aa:01", "TR-R-Atlas", -42))
        h.scanner.emissions.emit(BleAdvertisement("aa:01", "TR-R-Atlas", -55))
        runCurrent()
        val listed = h.fleet.discoveredDevices.value
        assertEquals(1, listed.size, "Re-discovery must not duplicate the row")
        assertEquals(-55, listed[0].rssi)
        assertEquals("TR-R-Atlas", listed[0].name)
        assertEquals("Found 1 device", h.fleet.statusMessage.value)
    }

    @Test
    fun scan_statusPluralizes() = runTest {
        val h = fleetHarness()
        h.fleet.scan()
        runCurrent()
        h.scanner.emissions.emit(BleAdvertisement("aa:01", "A", -40))
        h.scanner.emissions.emit(BleAdvertisement("aa:02", "B", -50))
        runCurrent()
        assertEquals("Found 2 devices", h.fleet.statusMessage.value)
    }

    @Test
    fun scan_timesOutAfter15s_noDevicesFound() = runTest {
        val h = fleetHarness()
        h.fleet.scan()
        runCurrent()
        advanceTimeBy(FleetManager.SCAN_TIMEOUT_MS)
        runCurrent()
        assertFalse(h.fleet.isScanning.value)
        assertEquals(0, h.scanner.activeCollections, "Platform scan must stop on timeout")
        assertEquals("No devices found", h.fleet.statusMessage.value)
    }

    @Test
    fun scan_timesOutAfter15s_scanCompleteWhenSomethingDiscovered() = runTest {
        val h = fleetHarness()
        h.fleet.scan()
        runCurrent()
        h.scanner.emissions.emit(BleAdvertisement("aa:01", "TR-R-Atlas", -40))
        runCurrent()
        advanceTimeBy(FleetManager.SCAN_TIMEOUT_MS)
        runCurrent()
        assertFalse(h.fleet.isScanning.value)
        assertEquals("Scan complete", h.fleet.statusMessage.value)
    }

    @Test
    fun scan_timeoutWithConnectedDevice_doesNotClobberStatus() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.fleet.scan()
        runCurrent()
        advanceTimeBy(FleetManager.SCAN_TIMEOUT_MS)
        runCurrent()
        assertFalse(h.fleet.isScanning.value)
        // iOS parity: the end-of-scan result messages only apply when
        // nothing is connected.
        assertEquals("Scanning...", h.fleet.statusMessage.value)
    }

    @Test
    fun userInitiatedScan_onlyForExplicitAdd_clearedOnStop() = runTest {
        // #394: only the "Add" button drives the Add-Device sheet.
        val h = fleetHarness()
        h.fleet.scan(userInitiated = true)
        runCurrent()
        assertTrue(h.fleet.userInitiatedScan.value)
        h.fleet.stopScanning()
        assertFalse(h.fleet.userInitiatedScan.value)

        h.fleet.scan()  // background/default
        runCurrent()
        assertFalse(h.fleet.userInitiatedScan.value)
    }

    @Test
    fun scan_restart_clearsPreviousResults() = runTest {
        val h = fleetHarness()
        h.fleet.scan()
        runCurrent()
        h.scanner.emissions.emit(BleAdvertisement("aa:01", "TR-R-Atlas", -40))
        runCurrent()
        h.fleet.scan()
        runCurrent()
        assertTrue(h.fleet.discoveredDevices.value.isEmpty())
    }

    // ----------------------------------------------------------- connecting

    @Test
    fun connect_stopsScanning_adoptsSession() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        assertFalse(h.fleet.isScanning.value)
        val dev = assertNotNull(h.fleet.devices.value["aa:01"])
        assertEquals(1, dev.generation)
        assertEquals("TR-R-Atlas", dev.advertisedName)
        assertEquals("Connected to TR-R-Atlas", h.fleet.statusMessage.value)
        assertEquals("aa:01", h.fleet.activeDeviceId.value)
        assertSame(h.sessions.created.single(), dev.session)
    }

    @Test
    fun connect_failure_reportsConnectionFailed() = runTest {
        val h = fleetHarness()
        h.fleet.scan()
        runCurrent()
        h.scanner.emissions.emit(BleAdvertisement("aa:01", "TR-R-Atlas", -40))
        runCurrent()
        h.transports.failNextConnects = 1
        h.fleet.connect("aa:01")
        runCurrent()
        assertEquals("Connection failed", h.fleet.statusMessage.value)
        assertTrue(h.fleet.devices.value.isEmpty())
    }

    // -------------------------------------- device-type precedence (#547)

    @Test
    fun typeAtConnect_registryBeatsLegacySubstring() = runTest {
        val h = fleetHarness()
        // A rocket renamed "SUBSONIC": the legacy substring rule misparses
        // it as a base station ("BS" substring) — the registry must win.
        h.knownDevices.deviceDidReportIdentity(
            unitID = "u-1", name = "SUBSONIC", deviceType = BleDeviceType.ROCKET,
            networkID = 1, rocketID = 1, pusher = null,
        )
        discoverAndConnect(h, "aa:01", "SUBSONIC")
        assertEquals(BleDeviceType.ROCKET, h.fleet.devices.value["aa:01"]!!.deviceType)
        // The scan row carried the hint too.
        assertEquals(BleDeviceType.ROCKET, h.fleet.discoveredDevices.value.single().knownType)
    }

    @Test
    fun typeAtConnect_factoryPrefixes() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        discoverAndConnect(h, "aa:02", "TR-B-Ground")
        assertEquals(BleDeviceType.ROCKET, h.fleet.devices.value["aa:01"]!!.deviceType)
        assertEquals(BleDeviceType.BASE_STATION, h.fleet.devices.value["aa:02"]!!.deviceType)
    }

    @Test
    fun typeAtConnect_legacySubstringOddity_pinned() = runTest {
        // Regression-pinned iOS oddity: with no registry record, "SUBSONIC"
        // parses as BASE_STATION via the "BS" substring.  Do not "fix" here
        // without changing iOS too.
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "SUBSONIC")
        assertEquals(BleDeviceType.BASE_STATION, h.fleet.devices.value["aa:01"]!!.deviceType)
    }

    @Test
    fun identityReadback_dtOverridesNameSeed() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "SUBSONIC")   // mis-seeded BASE_STATION
        h.fleet.onIdentityReadback("aa:01", identity(uid = "u-1", un = "SUBSONIC", dt = "R"), null)
        assertEquals(
            BleDeviceType.ROCKET, h.fleet.devices.value["aa:01"]!!.deviceType,
            "config_identity dt is device truth and overrides every heuristic",
        )
    }

    @Test
    fun identityReadback_unknownOrAbsentDt_keepsPreviousType() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.fleet.onIdentityReadback("aa:01", identity(uid = "u-1", dt = "X"), null)
        assertEquals(BleDeviceType.ROCKET, h.fleet.devices.value["aa:01"]!!.deviceType)
        h.fleet.onIdentityReadback("aa:01", identity(uid = "u-1", dt = null), null)
        assertEquals(BleDeviceType.ROCKET, h.fleet.devices.value["aa:01"]!!.deviceType)
    }

    @Test
    fun identityReadback_populatesMetadataAndRegistry() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.fleet.onIdentityReadback(
            "aa:01",
            identity(uid = "u-77", un = "Atlas", nid = 5, rid = 3, dt = "R", fw = "abc1234+20260701-1200"),
            null,
        )
        val dev = h.fleet.devices.value["aa:01"]!!
        assertEquals("u-77", dev.unitId)
        assertEquals("Atlas", dev.unitName)
        assertEquals(5, dev.networkId)
        assertEquals(3, dev.rocketId)
        assertEquals("abc1234+20260701-1200", dev.firmwareVersion)
        assertEquals("Atlas", dev.displayName)

        // Folded into the known-device registry (renamed-device recovery for
        // the NEXT scan).
        val rec = assertNotNull(h.knownDevices.device("u-77"))
        assertEquals("Atlas", rec.name)
        assertEquals(BleDeviceType.ROCKET, rec.deviceType)
        assertEquals(BleDeviceType.ROCKET, h.knownDevices.deviceTypeForAdvertisedName("Atlas"))
    }

    @Test
    fun connectedDevice_byUnitId_emptyIsNull() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        assertNull(h.fleet.connectedDevice("u-77"), "No identity readback yet")
        h.fleet.onIdentityReadback("aa:01", identity(uid = "u-77"), null)
        assertEquals("aa:01", h.fleet.connectedDevice("u-77")?.deviceId)
        assertNull(h.fleet.connectedDevice(""), "Empty unitID must never match")
    }

    // ----------------------------------------------------------- disconnect

    @Test
    fun userDisconnect_cleansUpWithoutReconnect() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        val session = h.fleet.devices.value["aa:01"]!!.session
        h.fleet.disconnect("aa:01")
        runCurrent()
        assertTrue(h.fleet.devices.value.isEmpty())
        assertEquals("Disconnected", h.fleet.statusMessage.value)
        assertTrue(h.fleet.discoveredDevices.value.isEmpty())
        assertSame(session, h.sessions.closed.single())
        assertNull(h.fleet.activeDeviceId.value)
        advanceTimeBy(120_000)
        assertEquals(1, h.transports.calls.size, "No reconnect after a user disconnect")
    }

    @Test
    fun disconnectAll_suppressesReconnectForEveryDevice() = runTest {
        // Per-device suppression — the iOS single global flag would only
        // suppress the FIRST device's disconnect event (see docs/android-parity-ledger.md).
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        discoverAndConnect(h, "aa:02", "TR-B-Ground")
        h.fleet.disconnectAll()
        runCurrent()
        assertTrue(h.fleet.devices.value.isEmpty())
        advanceTimeBy(120_000)
        assertEquals(2, h.transports.calls.size, "No reconnects after disconnectAll")
    }

    @Test
    fun activeDevice_fallsToNextOnDisconnect() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        discoverAndConnect(h, "aa:02", "TR-B-Ground")
        assertEquals("aa:01", h.fleet.activeDeviceId.value)
        h.fleet.disconnect("aa:01")
        runCurrent()
        assertEquals("aa:02", h.fleet.activeDeviceId.value)
    }

    // ------------------------------------------------- bsFocus (#390)

    @Test
    fun noteAutoFocus_stickyFirstHeard_pushesCmd45() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "bs:01", "TR-B-Ground")
        h.fleet.noteAutoFocus("bs:01", rocketId = 1)
        runCurrent()
        assertEquals(1, h.fleet.focusFor("bs:01"))
        val transport = h.transports.lastFor("bs:01")
        assertEquals(1, transport.writes.size)
        assertEquals(TrCharacteristic.COMMAND, transport.writes[0].first)
        assertContentEquals(Commands.setFocusRocket(1), transport.writes[0].second)

        // Second rocket heard later must NOT steal focus (stickiness is the
        // point — the old last-heard behavior is the #390 bug).
        h.fleet.noteAutoFocus("bs:01", rocketId = 2)
        runCurrent()
        assertEquals(1, h.fleet.focusFor("bs:01"))
        assertEquals(1, transport.writes.size, "No second cmd-45 push")
    }

    @Test
    fun setFocus_userSwitch_overwritesAndPushes() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "bs:01", "TR-B-Ground")
        h.fleet.noteAutoFocus("bs:01", rocketId = 1)
        h.fleet.setFocus("bs:01", rocketId = 2)
        runCurrent()
        assertEquals(2, h.fleet.focusFor("bs:01"))
        val transport = h.transports.lastFor("bs:01")
        assertContentEquals(Commands.setFocusRocket(2), transport.writes.last().second)
    }

    @Test
    fun bsFocus_survivesReconnect_andIsRepushedOnAdopt() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "bs:01", "TR-B-Ground")
        h.fleet.setFocus("bs:01", rocketId = 3)
        runCurrent()

        // Unexpected drop → ladder reconnects 1 s later on a NEW transport.
        h.transports.lastFor("bs:01").dropUnexpectedly()
        runCurrent()
        advanceTimeBy(1_000)
        runCurrent()

        val dev = assertNotNull(h.fleet.devices.value["bs:01"])
        assertEquals(2, dev.generation)
        assertEquals(3, h.fleet.focusFor("bs:01"), "Focus map survives session recreation")
        // #390: the pin is SEEDED into the fresh session (iOS BLEFleet.adopt);
        // the session's own choreography sends the single cmd 45 after cmd 20.
        // The fleet must NOT write it — an adopt-time push double-sent and
        // could land before cmd 9 (Phase 2 review).
        assertEquals(3, h.sessions.created.last().seededFocus)
        val newTransport = h.transports.lastFor("bs:01")
        assertTrue(newTransport.writes.isEmpty(), "fleet must not push cmd 45 at adopt")
    }

    // ------------------------------------------- foreground BS pair (#390)

    @Test
    fun foregroundBaseStation_autoIsFirstConnectedBS() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")   // rocket — never a candidate
        assertNull(h.fleet.foregroundBaseStation())
        discoverAndConnect(h, "bs:01", "TR-B-One")
        discoverAndConnect(h, "bs:02", "TR-B-Two")
        assertEquals("bs:01", h.fleet.foregroundBaseStation()?.deviceId)
    }

    @Test
    fun foregroundBaseStation_explicitPick_thenFallbackWhenItDrops() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "bs:01", "TR-B-One")
        discoverAndConnect(h, "bs:02", "TR-B-Two")
        h.fleet.setForegroundBS("bs:02")
        assertEquals("bs:02", h.fleet.foregroundBaseStation()?.deviceId)

        h.fleet.disconnect("bs:02")
        runCurrent()
        // Pick kept, but a disconnected BS can't be foreground — fall back
        // to the first connected one.
        assertEquals("bs:01", h.fleet.foregroundBaseStation()?.deviceId)
        assertEquals("bs:02", h.fleet.foregroundBSID.value)
    }

    // ------------------------------------------------ hidden rockets (#390)

    @Test
    fun hiddenRocketKeys_pureViewFilter() = runTest {
        val h = fleetHarness()
        val key = RocketKey(networkId = 5, rocketId = 1)
        h.fleet.hideRocket(key)
        assertTrue(key in h.fleet.hiddenRocketKeys.value)
        h.fleet.unhideRocket(key)
        assertFalse(key in h.fleet.hiddenRocketKeys.value)
    }

    // -------------------------------------- relay-aware routing (cmd 50)

    @Test
    fun relayAwareFrame_bsWithFocus_wrapsInCmd50() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "bs:01", "TR-B-Ground")
        h.fleet.noteAutoFocus("bs:01", rocketId = 4)
        val inner = byteArrayOf(23)  // toggle logging
        assertContentEquals(
            Commands.relayToRocket(4, inner),
            h.fleet.relayAwareFrame("bs:01", inner),
        )
        assertContentEquals(
            byteArrayOf(50, 4, 23),
            h.fleet.relayAwareFrame("bs:01", inner),
            "Envelope is [50][target_rid][inner cmd+payload]",
        )
    }

    @Test
    fun relayAwareFrame_rocketLink_passesThrough_cmd50Duality() = runTest {
        // Cmd-50 duality: on a rocket (OC) link, wire id 50 IS mag-cal
        // start — a bare [50] frame must arrive unwrapped, and no rocket
        // command may ever be relay-wrapped on a direct link.
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        val magCalStart = byteArrayOf(50)
        assertContentEquals(magCalStart, h.fleet.relayAwareFrame("aa:01", magCalStart))
    }

    @Test
    fun relayAwareFrame_bsWithoutFocus_passesThrough() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "bs:01", "TR-B-Ground")
        val inner = byteArrayOf(23)
        assertContentEquals(inner, h.fleet.relayAwareFrame("bs:01", inner))
    }

    @Test
    fun sendRelayAware_writesWrappedFrameOnCommandChar() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "bs:01", "TR-B-Ground")
        h.fleet.noteAutoFocus("bs:01", rocketId = 4)
        runCurrent()
        h.fleet.sendRelayAware("bs:01", byteArrayOf(23))
        runCurrent()
        val transport = h.transports.lastFor("bs:01")
        val last = transport.writes.last()
        assertEquals(TrCharacteristic.COMMAND, last.first)
        assertContentEquals(byteArrayOf(50, 4, 23), last.second)
    }

    // ------------------------------------------------- generation counter

    @Test
    fun generation_incrementsPerConnection_newSessionObject() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        val first = h.fleet.devices.value["aa:01"]!!
        assertEquals(1, first.generation)

        h.fleet.disconnect("aa:01")
        runCurrent()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        val second = h.fleet.devices.value["aa:01"]!!
        assertEquals(2, second.generation)
        assertNotSame(first.session, second.session,
            "Every connection gets a NEW session object — survivors live on the fleet")
    }

    // ── scanner failure must not be fatal ────────────────────────────────

    @Test
    fun scannerFailure_reportsAndStopsInsteadOfCrashing() = runTest {
        // The real scanner throws "BLE scanner unavailable (adapter off?)" when
        // the adapter is off.  Uncaught, it escaped scan()'s launch and the
        // default handler killed the process — tapping Scan with Bluetooth off
        // crashed the app outright (bench 2026-07-29, APP CRASH(EXCEPTION)).
        val h = fleetHarness()
        h.scanner.failWith = IllegalStateException("BLE scanner unavailable (adapter off?)")

        h.fleet.scan(userInitiated = true)
        advanceTimeBy(1_000); runCurrent()

        // Survived, and says so rather than pretending to scan forever.
        assertFalse(h.fleet.isScanning.value, "should stop scanning on failure")
        assertFalse(h.fleet.userInitiatedScan.value, "spinner flag must clear too")
        assertTrue(
            h.fleet.statusMessage.value.contains("unavailable", ignoreCase = true),
            "status should surface the reason, was '${h.fleet.statusMessage.value}'",
        )
    }

    @Test
    fun scannerRecoversOnASubsequentScan_afterAFailure() = runTest {
        val h = fleetHarness()
        h.scanner.failWith = IllegalStateException("BLE scanner unavailable (adapter off?)")
        h.fleet.scan(userInitiated = true)
        advanceTimeBy(1_000); runCurrent()
        assertFalse(h.fleet.isScanning.value)

        // Adapter comes back — the next scan must work, not stay poisoned.
        h.scanner.failWith = null
        h.fleet.scan(userInitiated = true)
        runCurrent()
        assertTrue(h.fleet.isScanning.value, "a later scan should start normally")
    }

    // ── #633 cold-start resume ───────────────────────────────────────────

    @Test
    fun connectRecordsTheResumeHint_andUserDisconnectClearsIt() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Rocket")
        assertEquals("aa:01", h.lastSession.loadLastConnected()?.deviceId)

        // Walking away is the one case we must NOT resume from.
        h.fleet.disconnect("aa:01")
        runCurrent()
        assertNull(h.lastSession.loadLastConnected(), "user disconnect must clear the hint")
    }

    @Test
    fun unexpectedDropKeepsTheHint_soARestartCanResume() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Rocket")
        // Not user-initiated: a crash/OEM kill/reboot looks like this.
        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()
        assertEquals(
            "aa:01",
            h.lastSession.loadLastConnected()?.deviceId,
            "an unexpected drop must leave the hint for a cold start",
        )
    }

    @Test
    fun resumeLastSession_scansThenConnectsOnceSighted() = runTest {
        val h = fleetHarness()
        h.lastSession.saveLastConnected("aa:01", "TR-R-Rocket")

        h.fleet.resumeLastSession()
        runCurrent()
        // Scanning, and NOT user-initiated — a resume must not pop a modal (#394).
        assertTrue(h.fleet.isScanning.value, "resume should start a scan")
        assertFalse(h.fleet.userInitiatedScan.value, "resume scan must be silent")
        assertTrue(h.fleet.devices.value.isEmpty(), "must not connect before a sighting")

        // Only a real advertisement unlocks the connect — never blind from a MAC.
        h.scanner.emissions.emit(BleAdvertisement("aa:01", "TR-R-Rocket", rssi = -40))
        advanceTimeBy(500); runCurrent()
        assertTrue(h.fleet.devices.value.containsKey("aa:01"), "should have reconnected")
    }

    @Test
    fun resumeLastSession_givesUpBoundedWhenTheDeviceIsAbsent() = runTest {
        val h = fleetHarness()
        h.lastSession.saveLastConnected("aa:01", "TR-R-Rocket")
        h.fleet.resumeLastSession()
        runCurrent()

        // Never sighted: bounded wait, unlike the mid-flight endgame.
        advanceTimeBy(FleetManager.RESUME_SIGHTING_TIMEOUT_MS + 1_000)
        runCurrent()
        assertTrue(h.fleet.devices.value.isEmpty())
        assertTrue(
            h.fleet.statusMessage.value.contains("not found", ignoreCase = true),
            "should say so plainly, was '${h.fleet.statusMessage.value}'",
        )
    }

    @Test
    fun resumeLastSession_isANoOpWithNoHintOrWhenAlreadyConnected() = runTest {
        val h = fleetHarness()
        // No hint at all.
        h.fleet.resumeLastSession()
        runCurrent()
        assertFalse(h.fleet.isScanning.value, "nothing to resume — must not scan")

        // Already connected: a hint exists but there's nothing to do.
        discoverAndConnect(h, "aa:01", "TR-R-Rocket")
        val before = h.fleet.statusMessage.value
        h.fleet.resumeLastSession()
        runCurrent()
        assertEquals(before, h.fleet.statusMessage.value, "should not disturb a live session")
    }
}
