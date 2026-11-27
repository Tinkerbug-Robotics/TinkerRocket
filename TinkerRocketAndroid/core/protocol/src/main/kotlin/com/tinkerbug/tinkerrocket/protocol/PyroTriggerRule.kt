package com.tinkerbug.tinkerrocket.protocol

import kotlin.math.ceil

/**
 * #1775: the check the flight computer and the out computer apply to an
 * enabled pyro channel's trigger (firmware `PyroConfigGate.h`; iOS
 * `PyroTriggerRule`). The rocket refuses a cmd-34 config that fails it and
 * keeps the one it holds; the app checks it first, so a value the rocket would
 * refuse is neither stored in the profile from a field nor sent.
 *
 * - Time after apogee (mode 0): 0 to 120 s. A delay that outlasts the descent
 *   never fires, since the flight computer services the triggers only in
 *   flight.
 * - Altitude on descent (mode 1): at least 30 m above the pad. At 0 m or below
 *   a channel fires at touchdown or never.
 * - The value must be finite, and the mode 0 or 1.
 *
 * A disabled channel is never checked: the rocket never reads its trigger.
 * `:app` has no test source set, so the field's parse and the range text live
 * here too.
 */
public object PyroTriggerRule {
    public const val MAX_DELAY_S: Float = 120f
    public const val MIN_ALTITUDE_M: Float = 30f

    private const val FEET_PER_METER: Double = 1.0 / 0.3048

    /** Whether an enabled channel with this trigger passes; `value` in
     *  seconds (mode 0) or meters (mode 1). */
    public fun accepts(mode: Int, value: Float): Boolean {
        if (!value.isFinite()) return false
        return when (mode) {
            0 -> value >= 0f && value <= MAX_DELAY_S
            1 -> value >= MIN_ALTITUDE_M
            else -> false
        }
    }

    /** The first enabled channel that fails, 1-4, or null. */
    public fun firstRefused(channels: List<PyroChannelConfig>): Int? =
        channels.indexOfFirst { it.enabled && !accepts(it.mode, it.value) }.takeIf { it >= 0 }?.plus(1)

    /** Whether any enabled channel fails. Such a config is not sent: the
     *  rocket would refuse the whole frame and keep what it holds. */
    public fun anyRefused(channels: List<PyroChannelConfig>): Boolean = firstRefused(channels) != null

    /** The whole-profile push's status when it held the pyro config back. */
    public fun notSentText(channel: Int): String =
        "Pyro settings not sent: channel $channel's trigger is outside what the rocket accepts, " +
            "so it kept its own. Everything else was sent."

    /** A field's text as a number, or null. `toFloatOrNull()` alone takes
     *  "NaN" and "Infinity", and an overflowing literal comes back infinite.
     *  A comma is read as the decimal point, as a comma-decimal keyboard
     *  types it. */
    public fun entry(text: String): Float? =
        text.trim().replace(',', '.').toFloatOrNull()?.takeIf { it.isFinite() }

    /** The range the rocket accepts, in the display unit. The altitude is
     *  rounded UP, so the number shown is itself accepted (30 m is 98.4 ft,
     *  and 98 ft is under it). */
    public fun rangeText(mode: Int, imperial: Boolean): String = when (mode) {
        0 -> "The rocket accepts 0 to ${MAX_DELAY_S.toInt()} s."
        1 -> if (imperial) {
            "The rocket accepts ${ceil(MIN_ALTITUDE_M * FEET_PER_METER).toInt()} ft or more."
        } else {
            "The rocket accepts ${ceil(MIN_ALTITUDE_M.toDouble()).toInt()} m or more."
        }
        else -> "Choose a trigger."
    }
}
