package com.tinkerbug.tinkerrocket.session

import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sqrt

/**
 * #552: how far the flight computer's velocity has drifted from what its own
 * GNSS fixes say — measured on the phone, from the telemetry frame alone.
 *
 * The issue is that the EKF throws large transient horizontal-velocity
 * excursions during coast (47 m/s on RIM-66 with twelve satellites locked),
 * and the ascent landing prediction integrates that velocity to apogee, so a
 * wrong velocity becomes a wrong landing point with a lever arm of
 * seconds-to-apogee.  The phone cannot fix the filter, but it can *notice*,
 * because the frame carries the filter's velocity and the receiver's position
 * and those two have to agree.
 *
 * The frame carries no GNSS velocity, so this differences successive GNSS
 * positions to make one.  That works — correlation +0.98 to +1.00 against the
 * true disagreement on three of the four 2026-08-29 BARC L2 flights — with one
 * failure mode, and it is the one this class exists to handle.
 *
 * ## Why `hacc` sets the differencing interval
 *
 * Differencing two positions each uncertain by sigma over an interval dt gives
 * a velocity uncertain by `sqrt(2) * sigma / dt`.  That blows up as dt shrinks.
 * Rolly Polly V flew on four satellites with 29 m fixes; differenced over the
 * 0.5 s frame interval that is 82 m/s of pure noise, and its measured
 * "disagreement" read 19.8 m/s where the truth was 9.9 — a 2x over-read that
 * inflated the uncertainty radius to 266 m around a 115 m error.
 *
 * The cure is not to subtract the noise (measured: worth almost nothing,
 * because the over-read is driven by outlier fixes rather than by a Gaussian
 * at the sigma level).  It is to *difference over a longer interval*, because
 * the noise falls as 1/dt while a real disagreement does not.  So `hacc` picks
 * the interval: long enough that the differencing noise sits under
 * [TARGET_NOISE_MPS], and never longer than [MAX_BASELINE_S], past which the
 * average smooths away the excursion we are trying to see.
 *
 * Measured over 195 ascent predictions on those four flights, holding
 * everything else fixed, this is worth a 44% tighter radius on the bad-GNSS
 * flight (266 m -> 149 m) at slightly BETTER coverage (68% -> 71%), and
 * changes nothing at all on the three healthy flights, whose `hacc` is 0-1 m
 * and whose interval therefore stays at the frame rate.
 *
 * ## When `hacc` is absent
 *
 * Legacy firmware and the mini send no `hacc`.  Here, and only here, absent
 * and 0 may be treated alike: both pick the shortest interval, which yields
 * the LARGEST disagreement and so the largest radius.  Being wrong in that
 * direction is safe, and it is exactly the behaviour that shipped before
 * `hacc` existed.
 *
 * That equivalence does not generalise.  Anywhere `hacc` is read as a quality
 * bound rather than as an interval input, absent must stay absent — 0 means
 * "better than half a metre", the most trusting value on the scale, so
 * decoding absence as 0 would rate the least trustworthy fixes the most
 * trustworthy.  The decoders keep it nullable for that reason.
 */
public object GnssVelocityCheck {

    /** One telemetry frame's worth of what this needs. */
    public data class Sample(
        val tMs: Long,
        val latDeg: Double,
        val lonDeg: Double,
        /** GNSS horizontal accuracy in metres, or null when not sent. */
        val hAccM: Int?,
        /** Filter velocity, ENU, m/s. */
        val velE: Double,
        val velN: Double,
    )

    public data class Result(
        /** |filter velocity - GNSS-derived velocity|, m/s. */
        val disagreementMps: Double,
        /** The interval actually differenced over, seconds. */
        val baselineS: Double,
        /** Differencing noise floor at that interval, m/s. */
        val noiseFloorMps: Double,
    )

    /** Differencing noise we are willing to tolerate before lengthening the interval. */
    public const val TARGET_NOISE_MPS: Double = 3.0

    /**
     * Longest interval we will difference over.  Measured: 6 s costs coverage
     * (68% -> 59% on the flight it is meant to help) because the chord
     * velocity starts averaging across the excursion instead of resolving it.
     */
    public const val MAX_BASELINE_S: Double = 4.0

    /**
     * Sigma contributed by the wire's own rounding.  `lat`/`lon` ship at 5
     * decimal places (TR_BLE_To_APP.cpp `addDouble("lat", ..., 5)`), a 1.11 m
     * step, so a uniform rounding error of 1.11/sqrt(12).  It is what stops a
     * reported `hacc` of 0 from claiming a noiseless difference.
     */
    public const val QUANTIZATION_SIGMA_M: Double = 1.11 / 3.4641016151377544  // sqrt(12)

    /** Metres per degree of latitude; good to ~0.1% over any one flight. */
    private const val M_PER_DEG_LAT = 111_320.0

    /**
     * Drop samples older than this.  Comfortably past [MAX_BASELINE_S] so the
     * longest interval still has both ends available.
     */
    public const val HISTORY_S: Double = 12.0

    /**
     * The disagreement, or null when it cannot honestly be measured — too few
     * samples, or every sample inside the minimum interval.
     *
     * [history] must be in ascending time order; the last entry is "now".
     */
    public fun evaluate(history: List<Sample>): Result? {
        if (history.size < 2) return null
        val latest = history.last()

        // Absent hacc falls back to the wire's own rounding alone, which keeps
        // the interval at the frame rate and the estimate deliberately
        // conservative -- see "When `hacc` is absent" above for why that is
        // sound here and nowhere else.
        val sigma = hypot(max((latest.hAccM ?: 0).toDouble(), 0.0), QUANTIZATION_SIGMA_M)
        val want = sqrt(2.0) * sigma / TARGET_NOISE_MPS

        // Oldest sample that is at least `want` old, but never reaching past
        // MAX_BASELINE_S. Walking back from the newest finds the shortest
        // interval that satisfies the target, so a healthy flight keeps the
        // frame rate and only a noisy one pays the lag.
        var chosen: Sample? = null
        for (i in history.size - 2 downTo 0) {
            val s = history[i]
            val ageS = (latest.tMs - s.tMs) / 1000.0
            if (ageS > MAX_BASELINE_S) break
            chosen = s
            if (ageS >= want) break
        }
        val old = chosen ?: return null
        val dt = (latest.tMs - old.tMs) / 1000.0
        if (dt <= 0.0) return null

        val cosLat = cos(Math.toRadians(latest.latDeg))
        val dN = (latest.latDeg - old.latDeg) * M_PER_DEG_LAT
        val dE = (latest.lonDeg - old.lonDeg) * M_PER_DEG_LAT * cosLat
        val gnssVelE = dE / dt
        val gnssVelN = dN / dt

        // A chord velocity is an AVERAGE over the interval, so it has to be
        // compared with the filter's average over the same interval -- not
        // with the instantaneous value at one end. Comparing an average to an
        // instant reads the excursion's own slope as disagreement.
        var sumE = 0.0
        var sumN = 0.0
        var n = 0
        for (s in history) {
            if (s.tMs >= old.tMs && s.tMs <= latest.tMs) {
                sumE += s.velE; sumN += s.velN; n++
            }
        }
        if (n == 0) return null
        val ekfVelE = sumE / n
        val ekfVelN = sumN / n

        return Result(
            disagreementMps = hypot(ekfVelE - gnssVelE, ekfVelN - gnssVelN),
            baselineS = dt,
            noiseFloorMps = sqrt(2.0) * sigma / dt,
        )
    }

    /** Append [s] and drop anything older than [HISTORY_S]. */
    public fun trimmed(history: List<Sample>, s: Sample): List<Sample> {
        val cutoffMs = s.tMs - (HISTORY_S * 1000.0).toLong()
        // Guard against a clock that jumped backwards: a sample older than the
        // newest would otherwise sit in the list forever and be differenced
        // against, producing a nonsense interval.
        return (history.filter { it.tMs in cutoffMs..s.tMs } + s)
    }
}
