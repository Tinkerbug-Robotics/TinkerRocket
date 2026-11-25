package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.Commands
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals
import kotlin.test.assertIs
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertSame
import kotlin.test.assertTrue

/**
 * #1650: the multi-rocket dashboard's roster → sections, scoping, hiding and
 * per-section command routing ([FleetDashboard], [DashboardSection]), over
 * real DeviceSessions fed by FakeFirmware like [RocketRosterTest].
 */
class FleetDashboardTest {

    private class Link(val device: FleetDevice<DeviceSession>, val fw: FakeFirmware)

    private fun TestScope.link(name: String, identityJson: String?): Link {
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
        link(name, """{"type":"config_identity","uid":"$name","un":"$name","nid":$nid,"dt":"B"}""")

    private fun TestScope.directRocket(rid: Int, nid: Int = 5, name: String = "TR-R-Atlas"): Link =
        link(name, """{"type":"config_identity","uid":"$name","un":"$name","nid":$nid,"rid":$rid,"dt":"R"}""")

    private fun Link.relay(rocketId: Int, name: String = "", extra: String = "") {
        val run = if (name.isEmpty()) "" else ",\"run\":\"$name\""
        val more = if (extra.isEmpty()) "" else ",$extra"
        fw.emitTelemetryJson("{\"rid\":$rocketId$run,\"st\":\"READY\",\"palt\":1.0$more}")
    }

    private fun layout(
        vararg links: Link,
        foregroundId: String? = null,
        hidden: Set<RocketKey> = emptySet(),
    ): FleetDashboard.Layout {
        val devices = links.map { it.device }
        val fg = FleetDashboard.foregroundBaseStation(devices, foregroundId)
        return FleetDashboard.layout(
            RocketRoster.build(devices), fg, fg?.session?.focusRocketId?.value, hidden,
        )
    }

    private fun DashboardSection.rid(): Int? = subject.key?.rocketId

    // ── Roster → sections ────────────────────────────────────────────────

    @Test
    fun followedRocketGetsTheFullDashboard_othersTheirOwnCards() = runTest {
        val bs = baseStation()
        bs.relay(1, "Booster")    // first heard: the base station follows it
        bs.relay(2, "Sustainer")
        runCurrent()

        val l = layout(bs)
        assertEquals(listOf("Booster", "Sustainer"), l.scoped.map { it.name })
        assertEquals(2, l.sections.size)
        val focused = assertIs<DashboardSection.FocusedRelay>(l.sections[0])
        assertEquals(1, focused.rocketId)
        val other = assertIs<DashboardSection.Relay>(l.sections[1])
        assertEquals(2, other.remote.rocketId)
        assertTrue(l.collapsible, "two rockets displayed: headers and collapse")
        assertNull(l.searchingBaseStation)
    }

    @Test
    fun directAndRelayedSameRocket_isOneDirectSection() = runTest {
        val bs = baseStation(nid = 5)
        bs.relay(1, "Atlas")
        bs.relay(2, "Dart")
        val direct = directRocket(rid = 1, nid = 5)
        runCurrent()

        val l = layout(bs, direct)
        assertEquals(2, l.sections.size, "(5, 1) heard both ways is ONE rocket")
        val d = assertIs<DashboardSection.Direct>(l.sections[0])
        assertSame(direct.device, d.device)
        assertEquals(1, d.subject.relays.size, "the relay path is merged into it")
        assertEquals(1, bs.device.session.focusRocketId.value)
        // The base station follows rocket 1, but the direct link wins it.
        assertIs<DashboardSection.Relay>(l.sections[1])
    }

    @Test
    fun hiddenRocket_staysInTheBar_leavesTheDashboard() = runTest {
        val bs = baseStation(nid = 5)
        bs.relay(1)
        bs.relay(2)
        runCurrent()

        val l = layout(bs, hidden = setOf(RocketKey(5, 2)))
        assertEquals(2, l.scoped.size, "a hidden rocket keeps its chip")
        assertEquals(listOf(1), l.sections.map { it.rid() })
        assertEquals(false, l.collapsible, "one rocket displayed: no collapse")
    }

    @Test
    fun identifyingLink_showsAndCannotBeHidden() = runTest {
        val direct = link("TR-R-New", identityJson = null)
        val l = layout(direct, hidden = setOf(RocketKey(0, 0)))
        val d = assertIs<DashboardSection.Direct>(l.sections.single())
        assertIs<RocketSubject.Identity.Identifying>(d.subject.id)
    }

    @Test
    fun theForegroundPairScopesTheRoster_directLinksAlwaysShow() = runTest {
        val bsA = baseStation("TR-B-A", nid = 5)
        bsA.relay(1, "PairA")
        val bsB = baseStation("TR-B-B", nid = 9)
        bsB.relay(1, "PairB")
        val direct = directRocket(rid = 3, nid = 7, name = "TR-R-Solo")
        runCurrent()

        assertEquals(
            listOf("PairA", "TR-R-Solo"),
            layout(bsA, bsB, direct).scoped.map { it.name },
            "no pick: the first base station",
        )
        assertEquals(
            listOf("TR-R-Solo", "PairB"),
            layout(bsA, bsB, direct, foregroundId = bsB.device.deviceId).scoped.map { it.name },
            "roster order: by network, then rocket ID",
        )
        assertEquals(
            listOf("PairA", "TR-R-Solo"),
            layout(bsA, direct, foregroundId = bsB.device.deviceId).scoped.map { it.name },
            "a pick that is no longer connected falls back",
        )
    }

    @Test
    fun aBaseStationThatHearsNothing_rendersItsSearchingState() = runTest {
        val bs = baseStation()
        val l = layout(bs)
        assertTrue(l.sections.isEmpty())
        assertSame(bs.device, l.searchingBaseStation)
    }

    // ── Command target per section ───────────────────────────────────────

    @Test
    fun cameraInARelayedRocketsSection_addressesThatRocketOnly() = runTest {
        val bs = baseStation()
        bs.relay(1, "Booster")
        bs.relay(2, "Sustainer")
        runCurrent()
        val (focused, other) = layout(bs).sections
        bs.fw.commandFrames.clear()

        other.send(Commands.cameraToggleWithState(true))
        focused.send(Commands.toggleLoggingWithState(false))
        runCurrent()

        val relayed = bs.fw.commandFrames.filter { it[0].toInt() == BleCommandId.RELAY_TO_ROCKET_BS }
        assertEquals(2, relayed.size)
        assertContentEquals(
            Commands.relayToRocket(2, Commands.cameraToggleWithState(true)), relayed[0],
            "rocket 2's own camera, though the base station follows rocket 1",
        )
        assertContentEquals(
            Commands.relayToRocket(1, Commands.toggleLoggingWithState(false)), relayed[1],
        )
        assertEquals(1, bs.device.session.focusRocketId.value, "a command never moves the focus")
    }

    @Test
    fun directSection_sendsBareOnItsOwnLink() = runTest {
        val bs = baseStation(nid = 5)
        bs.relay(1)
        val direct = directRocket(rid = 1, nid = 5)
        runCurrent()
        val section = layout(bs, direct).sections.single()
        bs.fw.commandFrames.clear()
        direct.fw.commandFrames.clear()

        assertSame(direct.device, section.commandDevice)
        section.send(Commands.cameraToggleWithState(true))
        runCurrent()

        assertContentEquals(Commands.cameraToggleWithState(true), direct.fw.commandFrames.single())
        assertTrue(bs.fw.commandFrames.isEmpty(), "a direct link wins the command too")
    }

    // ── Chips and header ─────────────────────────────────────────────────

    @Test
    fun focusBadgeAndFollow_onlyForTheForegroundPair() = runTest {
        val bs = baseStation()
        bs.relay(1)
        bs.relay(2)
        val direct = directRocket(rid = 9, name = "TR-R-Solo")
        runCurrent()
        val devices = listOf(bs.device, direct.device)
        val roster = RocketRoster.build(devices)
        val fg = FleetDashboard.foregroundBaseStation(devices, null)
        val focus = fg?.session?.focusRocketId?.value
        val (one, two, solo) = roster

        assertTrue(FleetDashboard.isFocused(one, fg, focus))
        assertNull(FleetDashboard.followTarget(one, fg, focus), "already followed")
        assertEquals(false, FleetDashboard.isFocused(two, fg, focus))
        assertSame(fg, FleetDashboard.followTarget(two, fg, focus))
        assertNull(FleetDashboard.followTarget(solo, fg, focus), "no base station carries it")
    }

    @Test
    fun sourceLineAndStaleLine_followTheClock() = runTest {
        val bs = baseStation()
        bs.relay(1)
        runCurrent()
        val subject = RocketRoster.build(listOf(bs.device)).single()
        val heard = assertNotNull(subject.lastSeenMs)

        assertEquals("via TR-B-Ridge", FleetDashboard.sourceLine(subject, heard + 1_000))
        assertEquals("via TR-B-Ridge · 10 s ago", FleetDashboard.sourceLine(subject, heard + 10_000))
        assertEquals(
            "via TR-B-Ridge · lost, last heard 2 min ago",
            FleetDashboard.sourceLine(subject, heard + 125_000),
        )
        assertNull(FleetDashboard.staleAdvisory(subject.freshness(heard + 1_000), heard + 1_000))
        assertEquals(
            "Telemetry 10 s old",
            FleetDashboard.staleAdvisory(subject.freshness(heard + 10_000), heard + 10_000),
        )
        assertEquals(
            "Link lost — last heard 2 min ago",
            FleetDashboard.staleAdvisory(subject.freshness(heard + 125_000), heard + 125_000),
        )
        assertEquals(0.35f, subject.freshness(heard + 125_000).cardOpacity)

        val direct = directRocket(rid = 4)
        assertEquals("Direct BLE", FleetDashboard.sourceLine(RocketRoster.build(listOf(direct.device)).single(), currentTime))
    }

    @Test
    fun aSharedRocketId_putsTheWarningInTheHeaderEvenSolo() = runTest {
        val bs = baseStation()
        bs.relay(2, "Dart", extra = "\"dup\":1")
        runCurrent()
        val subject = RocketRoster.build(listOf(bs.device)).single()

        val warning = assertNotNull(FleetDashboard.duplicateIdWarning(subject, currentTime))
        assertTrue("rocket ID 2" in warning)
        assertTrue(FleetDashboard.needsHeaderWhenSolo(subject, currentTime))

        bs.relay(2, "Dart")   // the base station omits dup once it clears
        runCurrent()
        val cleared = RocketRoster.build(listOf(bs.device)).single()
        assertNull(FleetDashboard.duplicateIdWarning(cleared, currentTime))
    }
}
