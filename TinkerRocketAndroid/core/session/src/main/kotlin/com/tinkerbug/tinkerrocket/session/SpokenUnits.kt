package com.tinkerbug.tinkerrocket.session

import kotlin.math.roundToLong

/**
 * Spoken-number formatting for voice callouts, ported from the iOS
 * `UnitFormatter.spoken*` family (#160).
 *
 * Wire, log and CSV values stay SI everywhere — this converts only at the point
 * of speech, so nothing about the recorded flight changes with the setting.
 *
 * Scope note (corrected #1101): this used to say "Android currently has no unit
 * toggle anywhere, so the app always passes [UnitSystem.METRIC] today". Both
 * halves are stale. Android has had a unit toggle since the display-units port,
 * and AppContainer wires the live setting straight through —
 * `unitSystem = { units.system.value }` — so callouts follow the user's choice
 * on both platforms. The METRIC default on [FlightAnnouncer]'s parameter is a
 * test convenience, not what the app passes.
 */
public enum class UnitSystem {
    METRIC,
    IMPERIAL,
    ;

    public companion object {
        /** Exact, not 3.28 — iOS uses `1.0 / 0.3048`. */
        internal const val FEET_PER_METER: Double = 1.0 / 0.3048
    }
}

/**
 * Whole-number spoken quantities. Deliberately integer: a synthesizer reading
 * "one hundred twenty point five three meters" over a launch PA is worse than
 * useless, and the operator only needs the magnitude.
 */
public object SpokenUnits {

    public fun altitude(meters: Double, system: UnitSystem = UnitSystem.METRIC): String =
        when (system) {
            UnitSystem.METRIC -> "${meters.roundToLong()} meters"
            UnitSystem.IMPERIAL -> "${(meters * UnitSystem.FEET_PER_METER).roundToLong()} feet"
        }

    public fun speed(mps: Double, system: UnitSystem = UnitSystem.METRIC): String =
        when (system) {
            UnitSystem.METRIC -> "${mps.roundToLong()} meters per second"
            UnitSystem.IMPERIAL ->
                "${(mps * UnitSystem.FEET_PER_METER).roundToLong()} feet per second"
        }

    /**
     * Same words as [altitude] — kept separate because iOS has a separate
     * `spokenDistance`, and because a future change (miles past some threshold,
     * say) belongs to one and not the other.
     */
    public fun distance(meters: Double, system: UnitSystem = UnitSystem.METRIC): String =
        when (system) {
            UnitSystem.METRIC -> "${meters.roundToLong()} meters"
            UnitSystem.IMPERIAL -> "${(meters * UnitSystem.FEET_PER_METER).roundToLong()} feet"
        }
}
