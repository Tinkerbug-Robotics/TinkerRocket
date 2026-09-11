package com.tinkerbug.tinkerrocket.protocol

/**
 * #1039: when the go/no-go verdict must stop being asserted.
 *
 * `TelemetryData.flightReadiness` is derived from the sensor-health bits and
 * nothing else — it has no freshness term. The out computer's
 * `latest_non_sensor_valid` latches once at first receipt and is never
 * cleared, so a flight computer that dies on the pad leaves the OC
 * republishing the last good frame's health bits forever, while separately
 * stamping the frame STALE past FC_FRAME_STALE_MS (3 s).
 *
 * Both facts ride the SAME frame. Read the bits alone and the card that exists
 * to answer "is it safe to fly right now" answers yes about a rocket that has
 * stopped talking — in green, undimmed, indistinguishable from a live verdict.
 *
 * These are pure functions in :core:protocol rather than view-private branches
 * for the reason iOS pinned its half with `static func isHeld`: the rule is
 * testable, and the two platforms can be shown to agree.
 *
 * iOS twin: `DashboardView.swift` `RocketFreshness.ageText`,
 * `HealthCardView.isHeld` / `readinessLabel`.
 */
public object ReadinessHold {

    /**
     * The verdict is HELD whenever a staleness age is present at all.
     *
     * The caller supplies a non-null age only when the stream is not live, so
     * the presence of the value IS the condition — matching iOS's
     * `isHeld(staleAgeSec:) { staleAgeSec != nil }`. Kept as a named function
     * rather than an inline null check so both apps point at one rule.
     */
    public fun isHeld(staleAgeSec: Double?): Boolean = staleAgeSec != null

    /**
     * "47 s" / "3 min". iOS `RocketFreshness.ageText`.
     *
     * Negative and non-finite ages read "unknown" rather than throwing or
     * rendering nonsense: a lost stream can hand us an age derived from a
     * missing last-seen stamp.
     */
    public fun ageText(ageSec: Double): String {
        if (!ageSec.isFinite() || ageSec < 0.0) return "unknown"
        return if (ageSec < 60.0) "${ageSec.toInt()} s" else "${ageSec.toInt() / 60} min"
    }

    /**
     * The banner's words.
     *
     * "Held" is deliberately not a verdict word. "Unknown" reads as a
     * measurement that was taken and failed; "held" says the app stopped
     * judging because the input stopped arriving — which is the true
     * statement, and the one that tells an operator to go look at the rocket
     * rather than to trust a stale green.
     */
    public fun label(readiness: TelemetryData.FlightReadiness, staleAgeSec: Double?): String {
        val age = staleAgeSec ?: return readiness.label
        if (!age.isFinite()) return "Held — no telemetry received"
        return "Held — data ${ageText(age)} old"
    }

    /**
     * How far to fade the health card while held. Matches the iOS dashboard's
     * own stale opacity so a card does not change appearance for any other
     * reason than the hold.
     */
    public const val HELD_OPACITY: Float = 0.5f
}
