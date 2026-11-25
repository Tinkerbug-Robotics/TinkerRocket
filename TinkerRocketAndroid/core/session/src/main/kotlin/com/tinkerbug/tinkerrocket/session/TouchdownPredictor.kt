package com.tinkerbug.tinkerrocket.session

import kotlin.math.max
import kotlin.math.min

/**
 * Predicts touchdown from the last live descent frames, so the announcer can
 * make the landing call when the LoRa link dies before the landed frame gets
 * through. Kotlin twin of the iOS `TouchdownPredictor` — same model, same
 * constants; the two announcers are a parity pair.
 *
 * **Why.** The flight computer sets its landed flag within about half a
 * second of touchdown, but at touchdown the antenna reaches the ground and
 * the link usually dies with it. On four of the five 2026-10-24 BARC flights
 * the last live frame came at most 4 m above pad level, still descending,
 * and the landed frame reached the base station 2–4 minutes later, when
 * someone walked into range. No landing call was spoken until then.
 *
 * **The model.** Remaining height over the mean descent rate of the last
 * [RATE_WINDOW_MS] of live frames. When the active profile's pyro channels say
 * the rocket fires a main on descent, and it is still falling faster than
 * [MAIN_OPEN_RATIO] times the profile's main rate, the main is taken as still
 * to come: the observed rate down to the main altitude, the profile's main
 * rate below it. The main altitude comes from the pyro channels, not from the
 * profile's recovery fields: those always hold a value (700 ft by default),
 * so they cannot tell a motor-ejection rocket from a dual-deploy one, while an
 * enabled "altitude on descent" channel is what the flight computer will
 * actually fire.
 *
 * **Scored on the flight logs** (base-station LoRa logs against the flight
 * computers' own touchdown, 22 flights from 2026-03 to 2026-10;
 * `Data_Analysis/touchdown_call_eval.py`). Losing the link at touchdown, the
 * call came 0.4 s early to 3.0 s late on the six flights that really lost it.
 * Cutting the link at 100 m or lower gives a median error near 1 s. Above the
 * main on the one dual-deploy flight (Rolly Polly V, main at 800 ft), the
 * plain rate was 14–23 s early; the main leg brought that to 1–7 s.
 *
 * Fleet thread only, like [DeploymentWatcher].
 */
public class TouchdownPredictor {

    private val samples = ArrayDeque<Pair<Long, Double>>()   // (ms, descent m/s)

    /** The last live descent frame, ms. */
    public var lastLiveMs: Long? = null
        private set

    /** When the rocket is expected on the ground, ms, or null before any
     *  prediction could be made. */
    public var predictedTouchdownMs: Long? = null
        private set

    public fun reset() {
        samples.clear()
        lastLiveMs = null
        predictedTouchdownMs = null
    }

    /**
     * One live descent frame.
     *
     * @param altAglM barometric altitude above the pad, metres.
     * @param altitudeRateMps signed, negative descending — as the wire carries it.
     * @param profile the active rocket's recovery figures, or null.
     */
    public fun observe(nowMs: Long, altAglM: Float?, altitudeRateMps: Float?, profile: RecoveryProfile?) {
        lastLiveMs = nowMs
        if (altitudeRateMps != null) samples.addLast(nowMs to -altitudeRateMps.toDouble())
        while (samples.isNotEmpty() && nowMs - samples.first().first > RATE_WINDOW_MS) samples.removeFirst()
        if (altAglM == null || samples.isEmpty()) return
        val rate = samples.sumOf { it.second } / samples.size
        if (rate < MIN_DESCENT_RATE_MPS) return
        predictedTouchdownMs = nowMs +
            (secondsToTouchdown(altAglM.toDouble(), rate, profile) * 1000).toLong()
    }

    /** When the landing call is due: the predicted touchdown, but never before
     *  the link has been silent for [SILENCE_BEFORE_CALL_MS]. Null until a
     *  prediction exists. */
    public val callDueMs: Long?
        get() {
            val touchdown = predictedTouchdownMs ?: return null
            val last = lastLiveMs ?: return null
            return max(touchdown, last + SILENCE_BEFORE_CALL_MS)
        }

    public companion object {
        /** Live frames whose rate is averaged: six at the 2 Hz relay. The
         *  downlink rate is a rounded integer and jitters by a few m/s. */
        public const val RATE_WINDOW_MS: Long = 3_000

        /** The landing call waits until the link has been silent this long.
         *  Gaps longer than 1.5 s were under 1 % of the descent gaps in the
         *  logs, all but one of them over 130 m up, where the predicted
         *  touchdown is still far off, so a gap there does not bring the call
         *  forward. */
        public const val SILENCE_BEFORE_CALL_MS: Long = 1_500

        /** Below this the rocket is not yet really falling (just past apogee)
         *  and height over rate means nothing; the previous prediction stands. */
        public const val MIN_DESCENT_RATE_MPS: Double = 1.0

        /** Falling faster than this multiple of the profile's main rate means
         *  the main is not out yet. */
        public const val MAIN_OPEN_RATIO: Double = 2.0

        /**
         * Seconds from a frame at [h] metres AGL, falling at [r] m/s
         * (positive), to the ground. At or below pad level it is now: the
         * landing site can sit lower than the pad, and the rocket is on its
         * last metres either way.
         */
        public fun secondsToTouchdown(h: Double, r: Double, profile: RecoveryProfile?): Double {
            if (h <= 0 || r <= 0) return 0.0
            val mainAlt = profile?.pyroMainAglM
            if (profile != null && mainAlt != null && profile.mainMps > 0 &&
                r > MAIN_OPEN_RATIO * profile.mainMps
            ) {
                return max(h - mainAlt, 0.0) / r + min(h, mainAlt) / profile.mainMps
            }
            return h / r
        }
    }
}
