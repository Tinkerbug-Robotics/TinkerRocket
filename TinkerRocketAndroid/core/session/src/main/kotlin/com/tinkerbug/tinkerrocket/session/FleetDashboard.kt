package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.Commands
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.flowOf

/**
 * #1650: the multi-rocket dashboard's decisions — port of iOS
 * `ConnectedFleetView` (DashboardView.swift), `BLEFleet`'s roster scoping and
 * the fleet chrome in FleetDashboardComponents.swift.  Pulled out of Compose
 * so they can be pinned by tests: `:app` renders, this decides.
 *
 * Base stations are compared by deviceId, never by reference: a
 * [FleetDevice] is a data class the fleet re-copies on every identity
 * readback, where iOS compares `BLEDevice` objects with `===`.
 */
public object FleetDashboard {

    /** What the dashboard renders right now. */
    public data class Layout(
        /** Units-bar chips: the foreground pair's rockets plus every direct link. */
        val scoped: List<RocketSubject>,
        /** One per displayed (scoped and not hidden) rocket, in roster order. */
        val sections: List<DashboardSection>,
        /** More than one rocket displayed: headers and the collapse control. */
        val collapsible: Boolean,
        /**
         * The foreground base station has heard no rocket yet, so its own
         * dashboard renders the searching state (iOS's trailing
         * `ConnectedDashboardView`); null otherwise.
         */
        val searchingBaseStation: FleetDevice<DeviceSession>?,
    )

    /**
     * The roster, rebuilt whenever a link connects or drops, an identity
     * readback lands, or a base station's relayed roster changes.  Direct
     * links' telemetry is not an input: sections collect it themselves.
     */
    public fun rosterFlow(devices: Collection<FleetDevice<DeviceSession>>): Flow<List<RocketSubject>> {
        if (devices.isEmpty()) return flowOf(emptyList())
        val inputs: List<Flow<Any?>> = devices.flatMap { d ->
            listOf(d.session.isConnected, d.session.identity, d.session.remoteRockets)
        }
        return combine(inputs) { RocketRoster.build(devices) }
    }

    /** Connected base-station links, in connection order. */
    public fun baseStations(devices: Collection<FleetDevice<DeviceSession>>): List<FleetDevice<DeviceSession>> =
        devices.filter { it.session.isConnected.value && it.session.isBaseStation }

    /**
     * The base station whose pair is on screen (iOS `foregroundBaseStation`):
     * the user's pick while it is still connected, else the first one.
     */
    public fun foregroundBaseStation(
        devices: Collection<FleetDevice<DeviceSession>>,
        foregroundBSID: String?,
    ): FleetDevice<DeviceSession>? {
        val stations = baseStations(devices)
        return stations.firstOrNull { it.deviceId == foregroundBSID } ?: stations.firstOrNull()
    }

    /**
     * @param foreground from [foregroundBaseStation]
     * @param focusRocketId the rocket [foreground]'s radio follows
     */
    public fun layout(
        roster: List<RocketSubject>,
        foreground: FleetDevice<DeviceSession>?,
        focusRocketId: Int?,
        hidden: Set<RocketKey>,
    ): Layout {
        // iOS scopedRockets: an explicit direct connection always shows.
        val scoped = roster.filter { s ->
            s.direct != null || (foreground != null && s.carriedBy(foreground))
        }
        // iOS dashboardRockets: an identifying link has no key to hide by.
        val displayed = scoped.filter { s -> s.key?.let { it !in hidden } ?: true }
        return Layout(
            scoped = scoped,
            sections = displayed.mapNotNull { section(it, foreground, focusRocketId) },
            collapsible = displayed.size > 1,
            searchingBaseStation = foreground?.takeIf { it.session.remoteRockets.value.isEmpty() },
        )
    }

    /**
     * Which section a displayed rocket gets (iOS `ConnectedFleetView`'s
     * ForEach): a direct link wins; then the rocket the foreground base
     * station follows rides that link's full dashboard; any other relayed
     * rocket gets the read-only card stack via its freshest relay.
     */
    public fun section(
        subject: RocketSubject,
        foreground: FleetDevice<DeviceSession>?,
        focusRocketId: Int?,
    ): DashboardSection? {
        subject.direct?.let { return DashboardSection.Direct(subject, it) }
        if (foreground != null && focusRocketId != null &&
            subject.key?.rocketId == focusRocketId && subject.carriedBy(foreground)
        ) {
            return DashboardSection.FocusedRelay(subject, foreground, focusRocketId)
        }
        val relay = subject.freshestRelay ?: return null
        return DashboardSection.Relay(subject, relay.baseStation, relay.remote)
    }

    /** The chip's focus badge: the foreground base station follows this rocket. */
    public fun isFocused(
        subject: RocketSubject,
        foreground: FleetDevice<DeviceSession>?,
        focusRocketId: Int?,
    ): Boolean =
        foreground != null && focusRocketId != null &&
            subject.key?.rocketId == focusRocketId && subject.carriedBy(foreground)

    /**
     * The base station whose radio "Focus base station here" (chip menu) and
     * Follow (section header) would move to this rocket, or null to hide
     * both: only the foreground one, only when it carries the rocket and is
     * not already following it.
     */
    public fun followTarget(
        subject: RocketSubject,
        foreground: FleetDevice<DeviceSession>?,
        focusRocketId: Int?,
    ): FleetDevice<DeviceSession>? {
        val key = subject.key ?: return null
        if (foreground == null || !subject.carriedBy(foreground)) return null
        return foreground.takeIf { focusRocketId != key.rocketId }
    }

    // ── Section header text (iOS RocketSectionHeader) ────────────────────

    /** "Direct BLE", "via Base 1", "via Base 1 · 12 s ago", "… · lost, last heard 3 min ago". */
    public fun sourceLine(subject: RocketSubject, nowMs: Long): String {
        if (subject.direct?.session?.isConnected?.value == true) return "Direct BLE"
        val relay = subject.freshestRelay ?: return "No link"
        val via = "via ${relay.baseStation.session.displayName}"
        return when (val f = subject.freshness(nowMs)) {
            RocketFreshness.Live -> via
            is RocketFreshness.Stale -> "$via · ${f.ageMs / 1000} s ago"
            is RocketFreshness.Lost -> f.lastSeenMs?.let { seen ->
                "$via · lost, last heard ${(nowMs - seen) / 60_000} min ago"
            } ?: "$via · lost"
        }
    }

    /**
     * The red header line while this rocket's ID looks shared by two rockets
     * (the base station's "dup", or the relayed name alternating), or null.
     */
    public fun duplicateIdWarning(subject: RocketSubject, nowMs: Long): String? {
        val key = subject.key ?: return null
        if (subject.relays.none { it.remote.duplicateIdSuspected(nowMs) }) return null
        return "Two rockets may be using rocket ID ${key.rocketId} — pyro commands are blocked. " +
            "Give one a different ID in My Devices."
    }

    /**
     * Whether a section that hides its header when it is the only one on
     * screen must show it anyway: the duplicate-ID warning has nowhere else
     * to go.  (iOS also shows it for a flown rocket gone quiet; Android has
     * no flown latch yet — parity ledger.)
     */
    public fun needsHeaderWhenSolo(subject: RocketSubject, nowMs: Long): Boolean =
        duplicateIdWarning(subject, nowMs) != null

    /**
     * iOS `StaleTelemetryAdvisoryRow`: one quiet line under the state banner
     * while a relayed rocket's telemetry is not live; null while live.
     */
    public fun staleAdvisory(freshness: RocketFreshness, nowMs: Long): String? = when (freshness) {
        RocketFreshness.Live -> null
        is RocketFreshness.Stale -> "Telemetry ${RocketFreshness.ageText(freshness.ageMs)} old"
        is RocketFreshness.Lost -> freshness.lastSeenMs?.let { seen ->
            "Link lost — last heard ${RocketFreshness.ageText(nowMs - seen)} ago"
        } ?: "No telemetry received"
    }

    private fun RocketSubject.carriedBy(bs: FleetDevice<DeviceSession>): Boolean =
        relays.any { it.baseStation.deviceId == bs.deviceId }
}

/**
 * One rocket's section on the dashboard, and where its rocket commands go.
 * The target is the section's own rocket: a Camera tap in a rocket's section
 * changes that rocket, whichever one the base station happens to follow.
 */
public sealed interface DashboardSection {
    public val subject: RocketSubject

    /** The link this section's rocket commands are written on. */
    public val commandDevice: FleetDevice<DeviceSession>

    /** Wrap a rocket-targeted `[cmd][payload]` frame for [commandDevice]. */
    public fun commandFrame(inner: ByteArray): ByteArray

    /** [commandFrame] + write on [commandDevice]'s command characteristic. */
    public fun send(inner: ByteArray) {
        commandDevice.session.sendCommandFrame(commandFrame(inner))
    }

    /** A directly connected rocket (or a link still identifying): bare commands on its own link. */
    public data class Direct(
        override val subject: RocketSubject,
        val device: FleetDevice<DeviceSession>,
    ) : DashboardSection {
        override val commandDevice: FleetDevice<DeviceSession> get() = device
        override fun commandFrame(inner: ByteArray): ByteArray = inner
    }

    /** The rocket [baseStation] follows: that link's full, mirrored dashboard. */
    public data class FocusedRelay(
        override val subject: RocketSubject,
        val baseStation: FleetDevice<DeviceSession>,
        val rocketId: Int,
    ) : DashboardSection {
        override val commandDevice: FleetDevice<DeviceSession> get() = baseStation
        override fun commandFrame(inner: ByteArray): ByteArray = Commands.relayToRocket(rocketId, inner)
    }

    /** Any other relayed rocket: its card stack and Camera/Logging, addressed through [baseStation]. */
    public data class Relay(
        override val subject: RocketSubject,
        val baseStation: FleetDevice<DeviceSession>,
        val remote: RelayedRocket,
    ) : DashboardSection {
        override val commandDevice: FleetDevice<DeviceSession> get() = baseStation
        override fun commandFrame(inner: ByteArray): ByteArray =
            Commands.relayToRocket(remote.rocketId, inner)
    }
}
