package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertIs
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertSame
import kotlin.test.assertTrue

/**
 * #390 roster merge + command routing + freshness — the full port of iOS
 * RocketRosterTests (10 cases), over REAL DeviceSessions fed by FakeFirmware
 * (iOS constructs peripheral-less BLEDevices; the transport seam is our
 * equivalent).  Closes the parity-ledger Phase-3 lag entry.
 */
class RocketRosterTest {

    private class Link(val device: FleetDevice<DeviceSession>, val fw: FakeFirmware)

    private fun TestScope.link(
        name: String,
        identityJson: String?,
    ): Link {
        val fw = FakeFirmware(backgroundScope).apply {
            configJson = null
            configPyroJson = null
            configIdentityJson = identityJson
            pushEchoWithConfig = false
        }
        val session = DeviceSession(
            scope = backgroundScope,
            transport = fw,
            connectedDeviceName = name,
            clock = { currentTime },
        )
        session.start()
        runCurrent()
        advanceTimeBy(1100)   // let the choreography settle + identity land
        runCurrent()
        val device = FleetDevice(
            deviceId = "id:$name",
            advertisedName = name,
            generation = 1,
            session = session,
            transport = fw,
            deviceType = session.identity.value.deviceType,
        )
        return Link(device, fw)
    }

    private fun TestScope.baseStation(name: String = "TR-B-Ridge", nid: Int = 5): Link =
        link(
            name,
            """{"type":"config_identity","uid":"$name","un":"$name","nid":$nid,"dt":"B"}""",
        )

    private fun TestScope.directRocket(
        rid: Int,
        nid: Int = 5,
        name: String = "TR-R-Atlas",
    ): Link =
        link(
            name,
            """{"type":"config_identity","uid":"$name","un":"$name","nid":$nid,"rid":$rid,"dt":"R"}""",
        )

    private fun Link.relay(rocketId: Int, name: String = "") {
        val run = if (name.isEmpty()) "" else ",\"run\":\"$name\""
        fw.emitTelemetryJson("{\"rid\":$rocketId$run,\"st\":\"READY\",\"palt\":1.0}")
    }

    // ── Merge ────────────────────────────────────────────────────────────

    @Test
    fun relayedRockets_oneSubjectEach() = runTest {
        val bs = baseStation()
        bs.relay(1, "Booster")
        bs.relay(2, "Sustainer")
        runCurrent()

        val roster = RocketRoster.build(listOf(bs.device))
        assertEquals(2, roster.size)
        assertEquals(listOf("Booster", "Sustainer"), roster.map { it.name })
        assertTrue(roster.all { it.isRelayOnly })
    }

    @Test
    fun directAndRelaySameKey_mergeIntoOneSubject() = runTest {
        val bs = baseStation(nid = 5)
        bs.relay(1)
        val direct = directRocket(rid = 1, nid = 5)
        runCurrent()

        val roster = RocketRoster.build(listOf(bs.device, direct.device))
        assertEquals(1, roster.size, "Same (nid, rid) via BS and direct BLE is ONE rocket")
        val subject = roster[0]
        assertNotNull(subject.direct)
        assertEquals(1, subject.relays.size)
        assertFalse(subject.isRelayOnly)
    }

    @Test
    fun sameRidDifferentNetwork_staySeparate() = runTest {
        val bsA = baseStation("TR-B-A", nid = 5)
        bsA.relay(1, "PairA")
        val bsB = baseStation("TR-B-B", nid = 9)
        bsB.relay(1, "PairB")
        runCurrent()

        val roster = RocketRoster.build(listOf(bsA.device, bsB.device))
        assertEquals(2, roster.size, "rid is only unique per network — never merge across nids")
        assertEquals(setOf("PairA", "PairB"), roster.map { it.name }.toSet())
    }

    @Test
    fun twoBaseStationsSameNetwork_mergeRelaysForOneRocket() = runTest {
        val bsA = baseStation("TR-B-A", nid = 5)
        bsA.relay(1)
        val bsB = baseStation("TR-B-B", nid = 5)
        bsB.relay(1)
        runCurrent()

        val roster = RocketRoster.build(listOf(bsA.device, bsB.device))
        assertEquals(1, roster.size)
        assertEquals(2, roster[0].relays.size, "Both BSes carry the same rocket — one subject, two paths")
    }

    @Test
    fun directRocketWithoutIdentity_showsAsIdentifying() = runTest {
        // No identity readback: dt stays the name-derived ROCKET, rid null.
        val direct = link("TR-R-New", identityJson = null)

        val roster = RocketRoster.build(listOf(direct.device))
        assertEquals(1, roster.size)
        assertIs<RocketSubject.Identity.Identifying>(
            roster[0].id, "rid 0 must not mint a known (nid,0) identity",
        )
        assertNotNull(roster[0].direct)
    }

    @Test
    fun disconnectedDevices_notInRoster() = runTest {
        val direct = directRocket(rid = 1)
        direct.fw.fireDisconnect()
        runCurrent()
        assertTrue(RocketRoster.build(listOf(direct.device)).isEmpty())
    }

    // ── Command routing ──────────────────────────────────────────────────

    @Test
    fun commandLink_directWinsOverRelay() = runTest {
        val bs = baseStation(nid = 5)
        bs.relay(1)
        val direct = directRocket(rid = 1, nid = 5)
        runCurrent()

        val subject = RocketRoster.build(listOf(bs.device, direct.device))[0]
        val linkChoice = assertNotNull(subject.commandLink())
        val d = assertIs<RocketCommandLink.Direct>(linkChoice, "Connected direct link must win")
        assertSame(direct.device, d.device)
    }

    @Test
    fun commandLink_relayOnly_targetsRocketId() = runTest {
        val bs = baseStation()
        bs.relay(3)
        runCurrent()

        val subject = RocketRoster.build(listOf(bs.device))[0]
        val r = assertIs<RocketCommandLink.Relay>(
            assertNotNull(subject.commandLink()), "Relay-only rocket routes via the BS",
        )
        assertSame(bs.device, r.baseStation)
        assertEquals(3, r.rocketId)
    }

    @Test
    fun commandLink_prefersFreshestRelay() = runTest {
        val bsStale = baseStation("TR-B-Stale", nid = 5)
        bsStale.relay(1)
        runCurrent()
        advanceTimeBy(30_000)   // 30 s later the second BS hears it
        val bsFresh = baseStation("TR-B-Fresh", nid = 5)
        bsFresh.relay(1)
        runCurrent()

        val subject = RocketRoster.build(listOf(bsStale.device, bsFresh.device))[0]
        val r = assertIs<RocketCommandLink.Relay>(assertNotNull(subject.commandLink()))
        assertSame(
            bsFresh.device, r.baseStation,
            "With no foreground preference, the freshest relay carries commands",
        )
    }

    // ── Freshness ────────────────────────────────────────────────────────

    @Test
    fun freshness_tiers() {
        val now = 1_000_000L
        assertEquals(RocketFreshness.Live, RocketFreshness.from(now - 1_000, now))
        val stale = RocketFreshness.from(now - 10_000, now)
        assertEquals(RocketFreshness.Stale(10_000), stale, "10 s old should be stale")
        assertIs<RocketFreshness.Lost>(
            RocketFreshness.from(now - 120_000, now), "2 min old should be lost",
        )
        assertEquals(
            RocketFreshness.Lost(null), RocketFreshness.from(null, now),
            "Never seen should be lost",
        )
    }
}
