package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData

/**
 * #390: the rocket-centric model — port of iOS RocketRoster.swift.  A
 * [RocketSubject] is one logical rocket merged across every link that can
 * currently reach it: a direct BLE connection and/or one or more base
 * stations relaying it over LoRa.  Identity is (networkId, rocketId) —
 * rocket ids are only unique within a network, and two BS/rocket pairs on
 * different network ids may legitimately reuse the same rocket id.
 *
 * The roster holds REFERENCES ([FleetDevice]/[RelayedRocket]), not copies:
 * UI observes the leaf flows for per-frame updates and only re-derives the
 * roster when the set of links/rockets changes (never snapshot telemetry
 * into roster items — the iOS rule).
 */
public object RocketRoster {

    /**
     * Build the merged roster from the currently connected links.  Pure with
     * respect to its inputs; keys derive from the LIVE networkId so an entry
     * migrates from (0, rid) to (nid, rid) when the identity readback lands —
     * the roster never splits one rocket in two.
     */
    public fun build(devices: Collection<FleetDevice<DeviceSession>>): List<RocketSubject> {
        val direct = mutableMapOf<RocketKey, FleetDevice<DeviceSession>>()
        val identifying = mutableListOf<Pair<String, FleetDevice<DeviceSession>>>()
        val relays = mutableMapOf<RocketKey, MutableList<RelayPath>>()

        for (device in devices) {
            val session = device.session
            if (!session.isConnected.value) continue
            val identity = session.identity.value
            when (identity.deviceType) {
                BleDeviceType.ROCKET, BleDeviceType.UNKNOWN -> {
                    val rid = identity.rocketId ?: 0
                    // UNKNOWN still gets a provisional row: the link exists
                    // and the user connected it on purpose; the role usually
                    // resolves within a beat (#330).  rid 0 must not mint a
                    // known (nid, 0) identity.
                    if (identity.deviceType == BleDeviceType.ROCKET && rid > 0) {
                        direct[RocketKey(identity.networkId ?: 0, rid)] = device
                    } else {
                        identifying += device.deviceId to device
                    }
                }
                BleDeviceType.BASE_STATION -> {
                    for (remote in session.remoteRockets.value) {
                        val key = RocketKey(identity.networkId ?: 0, remote.rocketId)
                        relays.getOrPut(key) { mutableListOf() } += RelayPath(device, remote)
                    }
                }
            }
        }

        val subjects = mutableListOf<RocketSubject>()
        val knownKeys = (direct.keys + relays.keys)
            .sortedWith(compareBy({ it.networkId }, { it.rocketId }))
        for (key in knownKeys) {
            val dev = direct[key]
            val paths = relays[key].orEmpty()
            subjects += RocketSubject(
                id = RocketSubject.Identity.Known(key),
                name = resolveName(key, dev, paths),
                direct = dev,
                relays = paths,
            )
        }
        for ((deviceId, device) in identifying) {
            subjects += RocketSubject(
                id = RocketSubject.Identity.Identifying(deviceId),
                name = device.session.displayName,
                direct = device,
                relays = emptyList(),
            )
        }
        return subjects
    }

    private fun resolveName(
        key: RocketKey,
        direct: FleetDevice<DeviceSession>?,
        relays: List<RelayPath>,
    ): String {
        direct?.session?.displayName?.takeIf { it.isNotEmpty() }?.let { return it }
        relays.firstOrNull { it.remote.unitName.isNotEmpty() }?.let { return it.remote.unitName }
        return "Rocket ${key.rocketId}"
    }
}

/** One base station currently relaying a rocket. */
public data class RelayPath(
    val baseStation: FleetDevice<DeviceSession>,
    val remote: RelayedRocket,
)

/** The transport a rocket-targeted command should take. */
public sealed interface RocketCommandLink {
    public data class Direct(val device: FleetDevice<DeviceSession>) : RocketCommandLink

    /** Send via this base station's targeted relay (BS cmd 50 → rid). */
    public data class Relay(
        val baseStation: FleetDevice<DeviceSession>,
        val rocketId: Int,
    ) : RocketCommandLink
}

/**
 * How recently a rocket was heard, across its best link.  Thresholds mirror
 * the BS firmware's BLE_TELEMETRY_STALE_MS (3 s) so the app and the base
 * station agree on what "stale" means.
 */
public sealed interface RocketFreshness {
    public data object Live : RocketFreshness
    public data class Stale(val ageMs: Long) : RocketFreshness
    public data class Lost(val lastSeenMs: Long?) : RocketFreshness

    public companion object {
        public const val STALE_AFTER_MS: Long = 3_000
        public const val LOST_AFTER_MS: Long = 60_000

        public fun from(lastSeenMs: Long?, nowMs: Long): RocketFreshness {
            if (lastSeenMs == null) return Lost(null)
            val age = nowMs - lastSeenMs
            return when {
                age <= STALE_AFTER_MS -> Live
                age <= LOST_AFTER_MS -> Stale(age)
                else -> Lost(lastSeenMs)
            }
        }
    }
}

/** One logical rocket as the app sees it right now. */
public data class RocketSubject(
    val id: Identity,
    val name: String,
    val direct: FleetDevice<DeviceSession>?,
    val relays: List<RelayPath>,
) {
    /**
     * Stable identity: network-scoped for identified rockets; a direct link
     * whose config_identity readback hasn't arrived yet (rocketId 0) is
     * provisionally identified by its deviceId so it can render as
     * "identifying…" without colliding with anything.
     */
    public sealed interface Identity {
        public data class Known(val key: RocketKey) : Identity
        public data class Identifying(val deviceId: String) : Identity
    }

    public val key: RocketKey? get() = (id as? Identity.Known)?.key

    /** True while only reachable through a base station. */
    public val isRelayOnly: Boolean get() = direct == null && relays.isNotEmpty()

    public val freshestRelay: RelayPath?
        get() = relays.maxByOrNull { it.remote.lastSeenMs }

    /**
     * Latest telemetry from the best source: a connected direct link wins,
     * otherwise the most recently heard relay.
     */
    public val telemetry: TelemetryData?
        get() {
            direct?.let { if (it.session.isConnected.value) return it.session.telemetry.value }
            return freshestRelay?.remote?.telemetry
        }

    /** When this rocket was last heard on ANY link. */
    public val lastSeenMs: Long?
        get() {
            val candidates = relays.map { it.remote.lastSeenMs }.toMutableList()
            direct?.let { d ->
                if (d.session.isConnected.value) {
                    d.session.lastTelemetryAtMs?.let { candidates += it }
                }
            }
            return candidates.maxOrNull()
        }

    public fun freshness(nowMs: Long): RocketFreshness =
        RocketFreshness.from(lastSeenMs, nowMs)

    /**
     * Preferred transport for commands: a live direct BLE link
     * unconditionally (writes work even between telemetry frames), else the
     * foreground base station when it carries this rocket, else the relay
     * that heard it most recently.  The foreground preference is honoured
     * ONLY when one is actually set — a bare `id == foregroundBSID` would
     * match null == null and pick an arbitrary relay instead of the
     * freshest (the iOS nil-guard, pinned by test).
     */
    public fun commandLink(foregroundBSID: String? = null): RocketCommandLink? {
        direct?.let { if (it.session.isConnected.value) return RocketCommandLink.Direct(it) }
        if (relays.isEmpty()) return null
        val foreground = foregroundBSID?.let { fg ->
            relays.firstOrNull { it.baseStation.deviceId == fg }
        }
        val relay = foreground ?: freshestRelay ?: return null
        return RocketCommandLink.Relay(relay.baseStation, relay.remote.rocketId)
    }
}
