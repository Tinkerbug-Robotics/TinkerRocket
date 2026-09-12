package com.tinkerbug.tinkerrocket.protocol

/**
 * #1085: the frame said it was trimmed, so say so rather than letting the
 * dropped fields read as a dead rocket.
 *
 * `TR_BLE_To_APP` holds seven bytes of payload back specifically so the `"tr"`
 * flag always fits when it has to drop fields to stay under the MTU window —
 * "so the partial frame isn't a silent blackout", in its own words. Both apps
 * decode the flag and neither looked at it; outside the two decoders the only
 * references in either tree were unit tests.
 *
 * What the operator saw instead: both apps replace the whole telemetry object
 * per frame, so every key the firmware dropped falls back to its decode default
 * and renders as an ABSENT reading — battery "—", `numSats` 0, state UNKNOWN.
 * On a small MTU window the Tier 1 floor suppresses everything below it and the
 * frame carries only the Tier 1 prefix that fit plus `"tr":1`, so the dashboard
 * looks like a rocket that stopped talking, in flight, while the one field
 * guaranteed to arrive is the one saying the frame was trimmed.
 *
 * Scope, honestly: the flag also sets on ordinary frames where only the Tier
 * 2/3 tail drops (filename, unit name, link stats), which is the benign case
 * the tiering exists for. So this is an advisory and never a verdict — one
 * quiet line, never a recolour of the state banner, under the same rule the
 * hold-up and LoRa-off lines already follow.
 */
public object TrimAdvisory {

    /**
     * How long the line stays up after the last trimmed frame.
     *
     * The flag toggles with payload size frame to frame — a frame that fits is
     * followed by one that does not — so rendering it per frame would flicker
     * at telemetry rate. Holding it makes it readable, and the cost of holding
     * is only that the line outlives the condition by at most this long, which
     * for an advisory is the right direction to be wrong in.
     */
    public const val HOLD_MS: Long = 5_000L

    /**
     * Wording. Says what happened and, more importantly, what it does NOT mean
     * — the missing values are a link-bandwidth decision, not sensors that
     * stopped. iOS twin uses this same string.
     */
    public const val TEXT: String =
        "Frame trimmed for link bandwidth — blank values are not sensor failures"

    /**
     * Should the advisory render?
     *
     * @param lastTrimmedAtMs when a frame last arrived with the flag set, or
     *   null if none has this session.
     * @param nowMs current time on the same clock.
     */
    public fun isShowing(lastTrimmedAtMs: Long?, nowMs: Long): Boolean {
        if (lastTrimmedAtMs == null) return false
        return (nowMs - lastTrimmedAtMs) <= HOLD_MS
    }
}
