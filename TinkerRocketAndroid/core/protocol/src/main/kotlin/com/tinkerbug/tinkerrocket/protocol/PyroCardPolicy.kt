package com.tinkerbug.tinkerrocket.protocol

/**
 * What the dashboard's pyro card shows, and on which link (#838 item 7).
 *
 * The card used to be wrapped in `if (!session.isBaseStation)`, commented
 * "direct rocket links only — iOS hides pyro on BS links". iOS does the
 * opposite: it renders `PyroChannelsView(relayMode: true)` on a base-station
 * link, with continuity from the relayed health scorecard. So the comment both
 * misdescribed the reference implementation and disguised a missing safety
 * display as deliberate parity.
 *
 * The cost was concrete. In the standard pad configuration — rocket relaying
 * over LoRa to a base station the phone is paired to over BLE — the Android
 * operator had NO continuity readout at all, on the only link available at the
 * pad. [pyroContinuityOf] already resolved the relay path correctly; nothing
 * on that screen ever called it with `isBaseStation = true`, because nothing
 * rendered.
 *
 * These are the display decisions that differ by link, pulled out of the
 * Compose tree so they can be pinned by a test — `:app` has no test source
 * set, which is how the gap survived.
 */
public object PyroCardPolicy {

    /** The card is shown on BOTH link types. */
    public fun showCard(): Boolean = true

    /**
     * The LoRa downlink carries no `pyro_status`, so the armed bit does not
     * exist on a relay. Render it false rather than let a zeroed field read
     * as a measurement.
     */
    public fun armed(isBaseStation: Boolean, telemetryArmed: Boolean): Boolean =
        !isBaseStation && telemetryArmed

    /** Same reasoning as [armed] — fired is direct-link only. */
    public fun fired(isBaseStation: Boolean, telemetryFired: Boolean): Boolean =
        !isBaseStation && telemetryFired

    /**
     * Whether a tile's continuity badge is visible.
     *
     * On a direct link it reveals while armed, or for 5 s after that tile's
     * manual test. On a relay it is ALWAYS revealed: `armed` is unavailable
     * there, so gating on it would hide the readout permanently — and the
     * readout is the entire point of the card at the pad. Matches iOS
     * `relayMode || armed || contTestChannel == channel`.
     */
    public fun revealed(
        isBaseStation: Boolean,
        armed: Boolean,
        testedChannel: Int,
        channel: Int,
    ): Boolean = isBaseStation || armed || testedChannel == channel

    /**
     * The manual continuity-test button.
     *
     * Never on a relay: the stand-back LoRa test (uplink cmds 35/36) is
     * iOS-only, pending Android's own pyro-safety pass, so a button here would
     * send a direct-link cmd 35 into a base station — nothing would happen,
     * and the silence would read as a fault.
     *
     * On a direct link the rail gate matches the iOS tile: the OC refuses
     * cmd 35 with the rail off, because a queued ARM pulse would deliver at
     * the next power-on.
     */
    public fun showTestButton(
        isBaseStation: Boolean,
        armed: Boolean,
        fired: Boolean,
        inflight: Boolean,
        powerPinOn: Boolean,
    ): Boolean = !isBaseStation && !armed && !fired && !inflight && powerPinOn

    /**
     * Where the per-channel trigger config comes from.
     *
     * A relay never receives the cmd-20 readback — config does not travel over
     * LoRa — so the active profile stands in, exactly as iOS's `configProvider`
     * does on that branch. It describes what the app believes it pushed rather
     * than what the rocket echoed, which is why the card says "(via LoRa)".
     */
    public enum class ConfigSource { ROCKET_READBACK, ACTIVE_PROFILE }

    public fun configSource(isBaseStation: Boolean): ConfigSource =
        if (isBaseStation) ConfigSource.ACTIVE_PROFILE else ConfigSource.ROCKET_READBACK

    /** Card title; the relay form says where the numbers came from. */
    public fun title(isBaseStation: Boolean, armed: Boolean): String = when {
        isBaseStation -> "Pyro Channels (via LoRa)"
        armed -> "Pyro Channels — ARMED"
        else -> "Pyro Channels"
    }
}
