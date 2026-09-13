package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.RocketConfig

/**
 * #624/#150: whether a LoRa radio change may be pushed, and why not when it
 * may not.
 *
 * The base station owns the link. Cmd 10 (RF config) and cmd 17 (link mode)
 * are transactional on the firmware side: the BS relays the change to every
 * tracked rocket on the OLD settings, switches, listens for a beacon on the
 * new ones, and commits or rolls back. That only works if a rocket is
 * actually on air to answer — pushing to a base station while the rocket is
 * off or out of range is precisely what strands the rocket on the old
 * channel, which is what this gate exists to prevent.
 *
 * Kept free of Android imports so the rules can be tested without a device or
 * a live link, and stated once rather than duplicated between the session and
 * the screen. The iOS twin is `BLEDevice.autoApplyRefusalReason` — same four
 * cases, same order, same 10 s window.
 */
public enum class LoraApplyRefusal {
    NOT_BASE_STATION,
    NOT_CONNECTED,
    CONFIG_MISSING,
    NO_ROCKET_PRESENT,
    ;

    /**
     * The line shown under the control. A concrete next step, never a bare
     * "unavailable" — iOS surfaces these verbatim rather than grey out in
     * silence, and a greyed control with no reason is the thing that makes a
     * user think the app is broken.
     */
    public val message: String
        get() = when (this) {
            NOT_BASE_STATION -> "Connect to the base station first."
            NOT_CONNECTED -> "Base station is not connected over BLE."
            CONFIG_MISSING -> "Waiting for base-station config readback."
            NO_ROCKET_PRESENT ->
                "No rocket has beaconed recently — power it on and wait for it to show up."
        }
}

public object LoraAutoApply {

    /**
     * How stale the newest rocket beacon may be. Beacons fire at roughly
     * 0.5–2 Hz in READY/PRELAUNCH/INITIALIZATION, so silence for this long is
     * strong evidence the rocket is not on air. Matches iOS's
     * `autoApplyMaxBeaconAgeSeconds`.
     */
    public const val MAX_BEACON_AGE_MS: Long = 10_000L

    /**
     * Null when a push may proceed, otherwise the reason it may not.
     *
     * Order is load-bearing and matches iOS: the cheapest and most
     * fundamental disqualifier first, so the message a user sees names the
     * thing furthest upstream rather than a symptom of it.
     *
     * [rocketLastSeenMs] and [nowMs] share the session clock that stamps
     * `RelayedRocket.lastSeenMs`; passing wall-clock for one and session
     * millis for the other silently disables the gate.
     */
    public fun refusalReason(
        isBaseStation: Boolean,
        isConnected: Boolean,
        config: RocketConfig?,
        rocketLastSeenMs: List<Long>,
        nowMs: Long,
    ): LoraApplyRefusal? {
        if (!isBaseStation) return LoraApplyRefusal.NOT_BASE_STATION
        if (!isConnected) return LoraApplyRefusal.NOT_CONNECTED
        // Cmd 10 carries the WHOLE radio config, so a change to any one field
        // still has to send the other four. Without the readback there is
        // nothing to send them from, and inventing defaults here would push a
        // modulation nobody chose.
        if (config == null ||
            config.loraBwKHz == null || config.loraSF == null ||
            config.loraCR == null || config.loraTxPower == null
        ) {
            return LoraApplyRefusal.CONFIG_MISSING
        }
        val cutoff = nowMs - MAX_BEACON_AGE_MS
        if (rocketLastSeenMs.none { it >= cutoff }) return LoraApplyRefusal.NO_ROCKET_PRESENT
        return null
    }

    /**
     * Whether frequency hopping is legally possible at the current modulation.
     *
     * #150: the firmware computes packets-per-channel from real airtime and
     * reports it as `lhdw`; 0 means the dwell rules cannot be met at this
     * SF/BW/CR, and the firmware will refuse a cmd-17 enable. Pre-#150
     * firmware does not report the key at all, and the app must not grey out
     * a control on a device that never claimed the limit — so null reads as
     * available and the firmware stays the final authority.
     */
    public fun hoppingAvailable(config: RocketConfig?): Boolean =
        (config?.loraHopDwell ?: 1) != 0
}
