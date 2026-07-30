package com.tinkerbug.tinkerrocket.session

import kotlin.math.abs

/**
 * Compass-heading arithmetic for the direction-to-rocket arrow.
 *
 * Extracted from `PhoneLocationManager`'s `SensorEventListener` and
 * `DashboardScreen`'s arrow rotation (Checkpoint A Group 7): all three of these
 * are pure functions that were buried in an Android sensor callback and a
 * Composable, where nothing could test them. Every one has a wrap-around case
 * that a visual check at the bench would not catch — an arrow that is 359° off
 * looks identical to one that is 1° off.
 *
 * Angles are degrees, compass convention: 0 = true north, increasing clockwise.
 */
public object HeadingMath {

    /**
     * Magnetic heading + geomagnetic declination → true heading, normalised to
     * [0, 360).
     *
     * Declination is signed (east positive). `mod` rather than `%` matters here:
     * Kotlin's `%` keeps the sign of the dividend, so a westerly declination
     * that pushes the result negative would yield e.g. -5° instead of 355°.
     */
    public fun trueHeading(magneticDeg: Double, declinationDeg: Double): Double =
        (magneticDeg + declinationDeg).mod(360.0)

    /**
     * Smallest angular distance between two headings, always in [0, 180].
     *
     * The published-heading guard uses this so 359.5° → 0.5° reads as a 1°
     * move rather than a 359° one. Naive subtraction would make every pass
     * through north look like a huge jump — which, with a "publish on ≥1°
     * change" rule, would publish constantly while the phone sits still
     * pointing north, and iOS memory growth from exactly that kind of
     * over-publishing is why the guard exists.
     */
    public fun angularDeltaDeg(a: Double, b: Double): Double =
        abs((a - b + 180.0).mod(360.0) - 180.0)

    /** Publish a new heading only once it has moved at least [minDeltaDeg]. */
    public fun shouldPublish(newDeg: Double, currentDeg: Double, minDeltaDeg: Double = 1.0): Boolean =
        angularDeltaDeg(newDeg, currentDeg) >= minDeltaDeg

    /**
     * Rotation for the on-screen arrow: where the rocket is, relative to where
     * the phone is pointing. Signed, in **[-180, 180)** — negative is left of
     * the phone's nose, positive is right — so a rotation applied to the glyph
     * takes the short way round instead of spinning 350° the wrong way.
     *
     * The exact antipode (rocket directly behind) lands at **-180, not +180**.
     * Harmless for a rotation — the two are the same direction — but it is the
     * half-open end of the range, so don't assert +180 for it.
     */
    public fun arrowAngleDeg(bearingToRocketDeg: Double, phoneHeadingDeg: Double): Double =
        (bearingToRocketDeg - phoneHeadingDeg + 180.0).mod(360.0) - 180.0
}
