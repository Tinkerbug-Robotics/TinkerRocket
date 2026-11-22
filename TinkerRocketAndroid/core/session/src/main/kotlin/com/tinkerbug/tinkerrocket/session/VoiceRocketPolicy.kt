package com.tinkerbug.tinkerrocket.session

/**
 * Which relayed rocket the callouts follow on a base-station link: the
 * computer the dashboard is showing. iOS twin: `VoiceRocketPolicy.swift`;
 * the two must not diverge.
 *
 * Voice used to follow the base station's radio focus and nothing else. On
 * 2026-11-21 (iOS) that left Rolly Polly V's flight silent: the 65 mm rocket
 * (rocket 8) had flown through the same base station earlier, the pin stayed
 * on it, and the two computers on screen (rockets 1 and 7) each showed a
 * Follow button while voice waited for a rocket already recovered. The radio
 * pin has its own reasons to stay put; what speaks should be what is on
 * screen.
 *
 * The rule, in order:
 * 1. The followed rocket (the radio focus) keeps voice while it is on the
 *    dashboard (heard on this link and not hidden). The exception is a
 *    followed rocket gone quiet (landed, or unheard for [QUIET_AFTER_MS])
 *    while another rocket on the dashboard is flying: then that flight speaks.
 * 2. A pin applied moments ago (re-seeded on reconnect, or a Follow) holds
 *    voice for its grace window before its rocket is heard again, unless the
 *    user hid it.
 * 3. Otherwise the first rocket on the dashboard: one flying, else one being
 *    heard, else any. With nothing on the dashboard, the focus.
 *
 * Where several rockets qualify, the one voice already follows (`current`)
 * keeps it, so two computers in one rocket going INFLIGHT a frame apart do
 * not hand voice back and forth at launch.
 */
public object VoiceRocketPolicy {

    /** One relayed rocket as the policy sees it. */
    public data class Candidate(
        val rocketId: Int,
        /** The user hid it from the dashboard. */
        val hidden: Boolean,
        /** Millis since its last live frame. */
        val silentForMs: Long,
        val state: String,
        val landed: Boolean,
        /** A simulated flight never claims voice from another rocket. */
        val simulated: Boolean,
    )

    /** A rocket unheard this long has gone quiet: the focus pin's own staleness bound. */
    public const val QUIET_AFTER_MS: Long = DeviceSession.FOCUS_PIN_STALE_AFTER_MS

    public fun isQuiet(r: Candidate): Boolean =
        r.landed || r.state == "LANDED" || r.silentForMs >= QUIET_AFTER_MS

    public fun isFlying(r: Candidate): Boolean =
        !r.simulated && r.state == "INFLIGHT" && !isQuiet(r)

    /**
     * @param focus the base station's radio focus (the followed rocket).
     * @param focusHidden the user hid the followed rocket from the dashboard.
     * @param focusPinFresh the pin was applied within its grace window.
     * @param current the rocket voice follows now, if any.
     * @param rockets every rocket heard on this link.
     */
    public fun choose(
        focus: Int?,
        focusHidden: Boolean,
        focusPinFresh: Boolean,
        current: Int?,
        rockets: List<Candidate>,
    ): Int? {
        val shown = rockets.filter { !it.hidden }.sortedBy { it.rocketId }
        val flying = shown.filter(::isFlying)

        fun pick(list: List<Candidate>): Int? {
            if (current != null && list.any { it.rocketId == current }) return current
            return list.firstOrNull()?.rocketId
        }

        if (focus != null) {
            val followed = shown.firstOrNull { it.rocketId == focus }
            if (followed != null) {
                if (isQuiet(followed)) pick(flying)?.let { return it }
                return focus
            }
            if (focusPinFresh && !focusHidden) return focus
        }
        return pick(flying)
            ?: pick(shown.filter { it.silentForMs < QUIET_AFTER_MS })
            ?: pick(shown)
            ?: focus
    }
}
