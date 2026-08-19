package com.tinkerbug.tinkerrocket.session

/**
 * Ground-side recovery-event detector (#813): drogue and main deployment, seen
 * from the descent-rate curve alone, plus a verdict on whether the canopy that
 * came out is actually holding the vehicle back.
 *
 * **Why this lives on the ground and not on the flight computer.** The FC
 * already has a deployment detector (`DeploymentDetector.h`) that is strictly
 * better informed — it sees a 3 ms ejection shock on the accelerometer and a
 * raw baro step at 500 Hz. None of that reaches the operator. Its verdict is
 * exported as `NSF2_DEPLOYED`, which is carried in the flight log and nowhere
 * else: the LoRa downlink has no spare bit for it (`flags_state` and `flags2`
 * are both full, and the 65-byte frame is at a regulatory ceiling), and the
 * base station leaves the per-channel pyro-fired bits zeroed when it builds
 * the BLE JSON. So on the link a flyer actually uses at a mile downrange, the
 * app receives `altitudeRate` at 2 Hz and nothing else about recovery.
 *
 * A detector built on that one signal is the only one that works on **both**
 * links, which is why it is the primary mechanism rather than a fallback.
 * The FC latch and the pyro-fired bits can sharpen this later where the link
 * carries them; they cannot replace it.
 *
 * **What it keys on.** A canopy is a step change in descent rate that holds.
 * The detector tracks the fastest sustained descent seen so far in the current
 * phase and waits for the rate to fall below [TRANSITION_RATIO] of it and
 * *stay* there for [TRANSITION_HOLD_MS]. The ratio is the same 0.6 the
 * post-flight report uses to call a drogue→main transition, so a live callout
 * and the report cannot disagree about the same flight. The hold is expressed
 * in milliseconds rather than frames on purpose: BLE delivers roughly ten
 * times as many frames per second as the 2 Hz LoRa downlink, and a frame count
 * would silently mean two different things on the two links.
 *
 * **Deliberately conservative.** A false "main out" is worse than silence —
 * it tells the flyer to stop watching the sky. Three guards, all cheap:
 *
 *  * Nothing is called below [MIN_AGL_M]. At touchdown the descent rate
 *    genuinely does collapse, and that is indistinguishable from a canopy on
 *    rate alone. No real deployment happens that low, so the floor costs
 *    nothing and removes the whole class of impact false-positives.
 *  * A phase must have been descending at [MIN_REFERENCE_MPS] before any drop
 *    counts, so ordinary rate wobble near zero cannot trip a transition.
 *  * At most two transitions per flight. A third "out" callout could only be
 *    noise or the ground.
 *
 * Not guarded, because it cannot be from rate alone: a single-deploy flight
 * that goes straight to main never descends ballistically, so the first (and
 * only) transition is the main. [RecoveryProfile.mainDeployAglM] is what
 * distinguishes that case — see [DeploymentKind].
 */
public enum class DeploymentKind {
    /** First canopy, opened above the configured main altitude. */
    DROGUE,

    /** The main: either the second transition, or a first one at/below the
     *  configured main-deploy altitude (single-deploy, or motor ejection). */
    MAIN,

    /** A canopy opened but there is no configured main altitude to judge it
     *  against, so which one it is cannot be known. Spoken as "chute". */
    CHUTE,
}

/**
 * The flyer's own numbers for what a good descent looks like, from the active
 * [RocketProfile]. Absent when no profile is selected — the detector still
 * reports deployments, it just cannot grade them.
 */
public data class RecoveryProfile(
    /** Expected descent rate under drogue, m/s. */
    val drogueMps: Double,
    /** Expected descent rate under main, m/s. */
    val mainMps: Double,
    /** AGL altitude the main is configured to fire at, metres. */
    val mainDeployAglM: Double,
) {
    public companion object {
        /** The profile stores recovery figures in feet and fps; this is SI. */
        private const val M_PER_FT: Double = 0.3048

        public fun from(p: RocketProfile): RecoveryProfile = RecoveryProfile(
            drogueMps = p.drogueRateFps * M_PER_FT,
            mainMps = p.mainRateFps * M_PER_FT,
            mainDeployAglM = p.mainDeployAltAglFt * M_PER_FT,
        )
    }
}

/** What [DeploymentWatcher.step] reports, at most one per frame. */
public sealed interface DeploymentEvent {
    /** A canopy just came out. Announced immediately. */
    public data class Deployed(
        val kind: DeploymentKind,
        val altAglM: Double,
    ) : DeploymentEvent

    /**
     * The settled descent rate [DeploymentWatcher.VERDICT_WINDOW_MS] after a
     * deployment. [nominal] is true when the rate is consistent with the
     * profile's expectation for that canopy.
     */
    public data class Verdict(
        val kind: DeploymentKind,
        val rateMps: Double,
        val nominal: Boolean,
    ) : DeploymentEvent
}

/**
 * Caller-owned, one per flight. [reset] it when the announcer resets.
 *
 * Threading: same single-writer contract as [FlightAnnouncer] — stepped only
 * from the fleet dispatcher.
 */
public class DeploymentWatcher {

    private enum class Phase { BALLISTIC, FIRST, SECOND }

    private var phase = Phase.BALLISTIC

    /** Fastest sustained descent seen in the current phase, m/s (positive). */
    private var reference = 0.0

    /** When the rate first dropped below the ratio, or null if it has not. */
    private var slowSinceMs: Long? = null

    // Verdict window: open from a deployment until VERDICT_WINDOW_MS later.
    private var verdictKind: DeploymentKind? = null
    private var verdictDueMs: Long = 0
    private var verdictSum = 0.0
    private var verdictCount = 0

    /** Expected rate for the canopy currently being graded, m/s. */
    private var verdictExpected = 0.0

    public fun reset() {
        phase = Phase.BALLISTIC
        reference = 0.0
        slowSinceMs = null
        verdictKind = null
        verdictDueMs = 0
        verdictSum = 0.0
        verdictCount = 0
        verdictExpected = 0.0
    }

    /**
     * Advance by one telemetry frame.
     *
     * @param nowMs monotonic clock, same source as the announcer's cadence gates.
     * @param altitudeRateMps signed vertical rate as the wire carries it —
     *        negative is descending. Null when the frame has no rate; the frame
     *        is skipped without disturbing any accumulated state.
     * @param altAglM barometric altitude above the pad, metres.
     * @param afterApogee the vehicle is past apogee. Nothing is detected before.
     * @param landed touchdown declared — stops the detector dead.
     * @param profile the flyer's expected rates, or null when no profile is
     *        active. Absent only costs the grade, never the deployment call.
     */
    public fun step(
        nowMs: Long,
        altitudeRateMps: Float?,
        altAglM: Float?,
        afterApogee: Boolean,
        landed: Boolean,
        profile: RecoveryProfile?,
    ): DeploymentEvent? {
        if (landed) return null
        if (!afterApogee) return null
        val rateSigned = altitudeRateMps ?: return null
        val alt = altAglM ?: return null

        // Positive = descending, which is how the rest of this reads.
        val rate = -rateSigned.toDouble()
        val altM = alt.toDouble()

        // A verdict is due first: it is timed from the deployment that opened
        // it, and closing it must not be delayed by the transition logic below.
        collectVerdictSample(rate)
        closingVerdict(nowMs)?.let { return it }

        // Below the floor nothing is a deployment — see the class comment.
        if (altM < MIN_AGL_M) return null

        // Two transitions is the whole vocabulary; a third could only be noise.
        if (phase == Phase.SECOND) return null

        if (rate > reference) reference = rate

        // Until the vehicle has genuinely been falling there is no baseline to
        // measure a drop against, and near-zero wobble would trip the ratio.
        if (reference < MIN_REFERENCE_MPS) {
            slowSinceMs = null
            return null
        }

        if (rate > reference * TRANSITION_RATIO) {
            // Back up to speed — whatever slowed it was not a canopy.
            slowSinceMs = null
            return null
        }

        val since = slowSinceMs
        if (since == null) {
            slowSinceMs = nowMs
            return null
        }
        if (nowMs - since < TRANSITION_HOLD_MS) return null

        // Held below the ratio for the full window: a canopy is out.
        val kind = classify(altM, profile)
        phase = if (phase == Phase.BALLISTIC) Phase.FIRST else Phase.SECOND
        // A first transition already at/below the main altitude IS the main,
        // so there is no second one to wait for.
        if (kind == DeploymentKind.MAIN) phase = Phase.SECOND
        reference = rate
        slowSinceMs = null
        openVerdict(kind, nowMs, profile)
        return DeploymentEvent.Deployed(kind, altM)
    }

    /**
     * First transition above the configured main altitude is the drogue; at or
     * below it, the main has just fired (or the vehicle is single-deploy and
     * this is its only canopy). With no profile there is nothing to compare
     * against and the honest answer is "a chute".
     */
    private fun classify(altM: Double, profile: RecoveryProfile?): DeploymentKind {
        if (phase == Phase.FIRST) return DeploymentKind.MAIN
        val main = profile?.mainDeployAglM ?: return DeploymentKind.CHUTE
        return if (altM > main) DeploymentKind.DROGUE else DeploymentKind.MAIN
    }

    private fun openVerdict(kind: DeploymentKind, nowMs: Long, profile: RecoveryProfile?) {
        // No profile, no grade — the deployment is still announced.
        if (profile == null) {
            verdictKind = null
            return
        }
        verdictKind = kind
        verdictDueMs = nowMs + VERDICT_WINDOW_MS
        verdictSum = 0.0
        verdictCount = 0
        verdictExpected = expectedFor(kind, profile)
    }

    private fun collectVerdictSample(rate: Double) {
        if (verdictKind == null) return
        verdictSum += rate
        verdictCount++
    }

    private fun closingVerdict(nowMs: Long): DeploymentEvent.Verdict? {
        val kind = verdictKind ?: return null
        if (nowMs < verdictDueMs) return null
        verdictKind = null
        if (verdictCount == 0) return null
        val mean = verdictSum / verdictCount
        return DeploymentEvent.Verdict(kind, mean, isNominal(kind, mean))
    }

    private fun isNominal(kind: DeploymentKind, meanMps: Double): Boolean {
        val expected = verdictExpected
        if (expected <= 0.0) return false
        if (meanMps > expected * NOMINAL_TOLERANCE) return false
        // The report's hard limit is about the FINAL descent — a drogue
        // legitimately falls faster than this, so it only binds under main.
        if (kind != DeploymentKind.DROGUE && meanMps > HARD_DESCENT_MPS) return false
        return true
    }

    private fun expectedFor(kind: DeploymentKind, profile: RecoveryProfile): Double =
        if (kind == DeploymentKind.DROGUE) profile.drogueMps else profile.mainMps

    public companion object {
        /**
         * Rate must fall to this fraction of the phase's fastest sustained
         * descent. Same constant as `MAIN_TRANSITION_RATIO` in the post-flight
         * report's deployment module, deliberately — the live callout and the
         * report should not grade the same flight differently.
         */
        public const val TRANSITION_RATIO: Double = 0.6

        /**
         * How long the slower rate must hold. In milliseconds, not frames: the
         * BLE link runs an order of magnitude faster than the 2 Hz LoRa
         * downlink, and a frame count would mean two different durations.
         */
        public const val TRANSITION_HOLD_MS: Long = 1_500

        /** A phase must reach this descent rate before a drop can count. */
        public const val MIN_REFERENCE_MPS: Double = 5.0

        /**
         * No deployment is called below this AGL. Touchdown collapses the
         * descent rate exactly like a canopy does, and nothing real deploys
         * this low, so the floor is free.
         */
        public const val MIN_AGL_M: Double = 15.0

        /** Settling time before the descent rate is worth grading. */
        public const val VERDICT_WINDOW_MS: Long = 4_000

        /** How far past the profile's expectation still counts as nominal. */
        public const val NOMINAL_TOLERANCE: Double = 1.5

        /**
         * Descent under a main faster than this is a partial or failed
         * recovery — the same threshold the post-flight report warns at.
         */
        public const val HARD_DESCENT_MPS: Double = 10.0
    }
}
