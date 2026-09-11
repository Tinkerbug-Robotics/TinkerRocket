package com.tinkerbug.tinkerrocket.session

import java.util.Locale
import com.tinkerbug.tinkerrocket.protocol.formatFixed

/**
 * Display-unit formatting, ported from the iOS `UnitFormatter` (#160) —
 * format-string for format-string, so the two apps print identical values in
 * either system.  Wire, log and CSV stay SI everywhere; conversion happens
 * only at the point of display.  The spoken twin is [SpokenUnits].
 */
public object UnitFormatter {

    private const val FEET_PER_METER: Double = 1.0 / 0.3048
    private const val FEET_PER_MILE: Double = 5280.0
    private const val STANDARD_GRAVITY: Double = 9.80665

    // #1086: round ties-to-even (printf / iOS), not HALF_UP. String.format
    // rounds `%f` HALF_UP, so a frame carrying an exact .5 printed one metre
    // higher on Android than iOS. `fmt` here is always "%.<N>f<suffix>" (every
    // call site below), so split the decimals and the unit suffix off and run
    // the number through the shared formatFixed (BigDecimal HALF_EVEN, the same
    // routine the CSV uses). Falls back to String.format for any fmt that is
    // not that shape, which none currently is.
    private val fixedFmt = Regex("""%\.(\d+)f(.*)""", RegexOption.DOT_MATCHES_ALL)
    private fun f(fmt: String, v: Double): String {
        val m = fixedFmt.matchEntire(fmt)
            ?: return String.format(Locale.ROOT, fmt, v)
        val decimals = m.groupValues[1].toInt()
        val suffix = m.groupValues[2]
        return formatFixed(v, decimals) + suffix
    }

    // ── Altitude ─────────────────────────────────────────────────────────
    // Never switches to miles: rocket apogees routinely exceed 5280 ft and
    // "1.2 mi" is useless for altitude.  Horizontal distances use distance().

    public fun altitude(meters: Double, system: UnitSystem): String = when (system) {
        UnitSystem.METRIC ->
            if (meters >= 1000) f("%.2f km", meters / 1000.0) else f("%.0f m", meters)
        UnitSystem.IMPERIAL -> f("%.0f ft", meters * FEET_PER_METER)
    }

    /** Base altitude unit label ("m" / "ft") — no km/mi promotion. */
    public fun altitudeUnit(system: UnitSystem): String =
        if (system == UnitSystem.METRIC) "m" else "ft"

    /** Scale an altitude into the display unit without formatting (iOS
     *  `metersToDisplay`) — for callers with their own format string. */
    public fun altitudeValue(meters: Double, system: UnitSystem): Double =
        if (system == UnitSystem.METRIC) meters else meters * FEET_PER_METER

    /** Inverse of [altitudeValue] (iOS `displayToMeters`) — parse a user
     *  entry in the display unit back to canonical meters. */
    public fun altitudeToMeters(display: Double, system: UnitSystem): Double =
        if (system == UnitSystem.METRIC) display else display / FEET_PER_METER

    // ── Horizontal distance (promotes to km / mi when large) ─────────────

    public fun distance(meters: Double, system: UnitSystem): String = when (system) {
        UnitSystem.METRIC ->
            if (meters >= 1000) f("%.1f km", meters / 1000.0) else f("%.0f m", meters)
        UnitSystem.IMPERIAL -> {
            val feet = meters * FEET_PER_METER
            if (feet >= FEET_PER_MILE) f("%.2f mi", feet / FEET_PER_MILE) else f("%.0f ft", feet)
        }
    }

    // ── Speed ────────────────────────────────────────────────────────────

    public fun speed(mps: Double, system: UnitSystem, decimals: Int = 1): String = when (system) {
        UnitSystem.METRIC -> f("%.${decimals}f m/s", mps)
        UnitSystem.IMPERIAL -> f("%.${decimals}f ft/s", mps * FEET_PER_METER)
    }

    public fun speedUnit(system: UnitSystem): String =
        if (system == UnitSystem.METRIC) "m/s" else "ft/s"

    // ── Acceleration (g in imperial) ─────────────────────────────────────

    public fun acceleration(mps2: Double, system: UnitSystem, decimals: Int = 2): String =
        when (system) {
            UnitSystem.METRIC -> f("%.${decimals}f m/s²", mps2)
            UnitSystem.IMPERIAL -> f("%.${decimals}f g", mps2 / STANDARD_GRAVITY)
        }

    public fun accelerationUnit(system: UnitSystem): String =
        if (system == UnitSystem.METRIC) "m/s²" else "g"

    /** Scale an acceleration triplet into the display unit without formatting. */
    public fun accelerationValue(mps2: Double, system: UnitSystem): Double =
        if (system == UnitSystem.METRIC) mps2 else mps2 / STANDARD_GRAVITY

    // ── Temperature ──────────────────────────────────────────────────────

    public fun temperature(celsius: Double, system: UnitSystem, decimals: Int = 1): String =
        when (system) {
            UnitSystem.METRIC -> f("%.${decimals}f °C", celsius)
            UnitSystem.IMPERIAL -> f("%.${decimals}f °F", celsius * 9.0 / 5.0 + 32.0)
        }
}
