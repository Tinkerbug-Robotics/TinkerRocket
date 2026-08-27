package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlin.math.abs

/**
 * Subset of [FlightAnnouncer] that [DeviceSession] calls during telemetry
 * dispatch — the Kotlin twin of the iOS `TelemetryAnnouncer` protocol.
 * A seam rather than a concrete class so dispatch tests substitute a spy
 * without any speech engine, and so the dispatch logic in [DeviceSession]
 * can be exercised end-to-end (#138 regression coverage).
 */
public interface TelemetryAnnouncer {
    public fun processTelemetry(telemetry: TelemetryData)
    public fun reset()
}

/**
 * The speech engine seam. `:app` implements this over Android `TextToSpeech`
 * with audio-focus ducking; tests implement it with a list.
 *
 * iOS splits the same way in spirit (`AVSpeechSynthesizer` + delegate), but
 * keeps engine and policy in one class, which is why its announcer *policy*
 * has no unit tests. This seam is what buys them here.
 */
public interface AnnouncerSpeech {
    /** True while an utterance is playing (or queued in the engine). */
    public val isBusy: Boolean

    /**
     * Speak [text]. With [interrupt] the engine cancels anything currently
     * playing first (iOS `announceImmediate`); without it the engine just
     * starts — the policy has already checked [isBusy] and chosen to speak.
     */
    public fun speak(text: String, interrupt: Boolean)

    /** Cancel any current utterance immediately. */
    public fun stop()
}

/**
 * Voice-callout policy for a rocket flight, ported from the iOS
 * `FlightAnnouncer` (minus `AVAudioSession`, which has no Android twin —
 * ducking is per-utterance audio focus and lives in the `:app` speaker).
 *
 * Announces burnout speed, altitude, apogee, recovery deployments, descent
 * rate/distance, and landing. Everything time-based runs off the injected
 * [clock] (milliseconds), same convention as [DeviceSession], so the whole
 * flight profile is testable under virtual time.
 *
 * Threading: [processTelemetry]/[reset] are called on the fleet dispatcher
 * (single-writer contract); [setEnabled]/[testVoice] may be called from the
 * UI. The mutable flight state is only ever touched from the fleet thread;
 * the enabled flag crosses threads via [StateFlow].
 *
 * Callout policy, in flight order (all matching iOS):
 *  - **Burnout**: max_speed stops increasing for [BURNOUT_STABLE_THRESHOLD]
 *    consecutive frames (0.5 m/s jitter tolerance), above 10 m/s.
 *  - **Altitude** every 5 s, only INFLIGHT pre-apogee and only after burnout —
 *    during powered flight the values change too fast to be useful.
 *  - **Apogee**: on the alt_apo rising edge, with max altitude.
 *  - **Deployment** (#813): drogue and main, from the descent-rate step
 *    ([DeploymentWatcher]) — the only recovery signal carried on both the
 *    direct BLE link and the base-station relay. Interrupts, like apogee.
 *  - **Chute verdict**: the settled descent rate a few seconds after each
 *    deployment. "Good chute" only when it matches the profile; otherwise the
 *    bare rate, never an alarm word.
 *  - **Descent** every 10 s after apogee (first one 5 s after apogee).
 *  - **Landed**: on the landed_flag rising edge (state=LANDED fallback), with
 *    horizontal distance from the captured launch location.
 *
 * The direction word ("climbing"/"descending") comes from the SIGN of the
 * altitude rate, never from the apogee-phase flag — #235's bug was "climbing"
 * spoken during descent because `alt_apo` clears below ~15 m AGL near landing.
 */
public class FlightAnnouncer(
    private val speech: AnnouncerSpeech,
    initialEnabled: Boolean = false,
    /** Persistence hook (iOS: UserDefaults `voiceAnnouncementsEnabled`). */
    private val onEnabledChanged: (Boolean) -> Unit = {},
    private val clock: () -> Long = { System.currentTimeMillis() },
    /** Read per callout so a future settings toggle applies immediately. */
    private val unitSystem: () -> UnitSystem = { UnitSystem.METRIC },
    /**
     * The active rocket's expected descent rates (#813), read per event so a
     * profile switch applies immediately. Null when no profile is selected —
     * deployments are still announced, they just go ungraded.
     */
    private val recovery: () -> RecoveryProfile? = { null },
) : TelemetryAnnouncer {

    private val _enabled = MutableStateFlow(initialEnabled)
    public val enabled: StateFlow<Boolean> = _enabled.asStateFlow()

    // ── Flight state (fleet thread only) ─────────────────────────────────

    /** Previous frame for edge detection. Survives [reset], like iOS. */
    private var previous: TelemetryData? = null

    /** Launch position for the landed-callout horizontal distance. */
    private var launchLocation: Pair<Double, Double>? = null

    private var lastAltitudeAnnounceMs: Long = Long.MIN_VALUE / 2
    private var lastDescentAnnounceMs: Long = Long.MIN_VALUE / 2

    // One-shot events (reset on the PRELAUNCH transition)
    private var burnoutAnnounced = false
    private var apogeeAnnounced = false
    private var landedAnnounced = false

    // Burnout detection: consecutive frames where max_speed didn't increase
    // A burnout is a RISE followed by a PLATEAU, and both halves have to be
    // observed. `maxSpeedMps` is a running maximum that never decreases, so
    // "it stopped increasing" is true at every instant AFTER burnout —
    // including minutes later under canopy. Keying on the plateau alone
    // announces a long-past burnout to anyone who starts watching late (voice
    // toggled on mid-flight, or a reconnect). `null` = no baseline sampled
    // yet; `sawSpeedIncrease` = we watched it climb.
    private var lastMaxSpeed: Float? = null
    private var sawSpeedIncrease = false
    private var maxSpeedStableCount: Int = 0

    /** Recovery-event detector (#813). Self-guards on apogee/landing. */
    private val deployment = DeploymentWatcher()

    // ── Enable / test ────────────────────────────────────────────────────

    public fun setEnabled(value: Boolean) {
        if (_enabled.value == value) return
        _enabled.value = value
        onEnabledChanged(value)
        if (value) {
            // Audible confirmation at the current phone volume — without it
            // the toggle is silent and the operator can't tell voice works
            // until the first in-flight callout.
            announceImmediate("Voice ready")
        } else {
            speech.stop()
        }
    }

    /** Spoken volume/voice check for the pre-flight ritual (long-press). */
    public fun testVoice() {
        announceImmediate("Voice check. Announcements are enabled.")
    }

    // ── Main entry point ─────────────────────────────────────────────────

    /** Called for every decoded telemetry frame of the voice device. */
    override fun processTelemetry(telemetry: TelemetryData) {
        if (!_enabled.value) {
            previous = telemetry
            return
        }

        // Skip stale/syncing frames. When the rocket goes silent mid-flight
        // the BS keeps forwarding last-known telemetry with dataStatus=STALE —
        // the time-gated callouts would otherwise re-announce the same frozen
        // reading every interval forever.
        if (telemetry.dataStatus != TelemetryData.DataStatus.LIVE) {
            previous = telemetry
            return
        }

        val prev = previous
        val state = telemetry.state

        // Reset on the PRELAUNCH transition; capture launch position.
        if (state == "PRELAUNCH" && prev?.state != "PRELAUNCH") {
            resetFlightState()
            captureLaunchLocation(telemetry)
        }

        // Late capture on the launch flag if PRELAUNCH had no fix yet.
        if (telemetry.launchFlag && launchLocation == null) {
            captureLaunchLocation(telemetry)
        }

        // INFLIGHT, pre-apogee.
        if (state == "INFLIGHT" && !telemetry.altApo) {
            checkBurnout(telemetry)
            if (burnoutAnnounced) checkPeriodicAltitude(telemetry)
        }

        // Apogee rising edge.
        if (telemetry.altApo && !(prev?.altApo ?: false) && !apogeeAnnounced) {
            apogeeAnnounced = true
            val alt = telemetry.maxAltM
                ?.let { SpokenUnits.altitude(it.toDouble(), unitSystem()) }
                ?: "altitude unknown"
            announceImmediate("Apogee. $alt")
            // First descent callout 5 s after apogee, not the full 10 s.
            lastDescentAnnounceMs = clock() - (DESCENT_INTERVAL_MS - 5_000)
        }

        // Recovery events (#813) before the cadence callout: a deployment
        // should pre-empt the periodic descent readout, not race it.
        checkDeployment(telemetry, state)

        // Descent callouts (after apogee, before landed).
        if (telemetry.altApo && !telemetry.landedFlag && state == "INFLIGHT") {
            checkDescentCallout(telemetry)
        }

        // Landing: flag edge, else the state transition as fallback.
        if (telemetry.landedFlag && !(prev?.landedFlag ?: false) && !landedAnnounced) {
            landedAnnounced = true
            announceImmediate(landedPhrase(telemetry))
        } else if (state == "LANDED" && prev?.state != "LANDED" && !landedAnnounced) {
            landedAnnounced = true
            announceImmediate(landedPhrase(telemetry))
        }

        previous = telemetry
    }

    /** Reset when disconnecting or starting a new flight. */
    override fun reset() {
        resetFlightState()
        speech.stop()
    }

    // ── Event detectors ──────────────────────────────────────────────────

    private fun checkBurnout(telemetry: TelemetryData) {
        if (burnoutAnnounced) return
        val maxSpeed = telemetry.maxSpeedMps ?: return
        if (maxSpeed <= BURNOUT_MIN_SPEED_MPS) return

        val last = lastMaxSpeed
        if (last == null) {
            // First sample we've seen. A running maximum reads identically
            // during the burn and long after it, so one frame proves nothing.
            // Adopt it as the baseline and wait for evidence either way.
            lastMaxSpeed = maxSpeed
            return
        }

        if (maxSpeed > last + 0.5f) {
            // Still accelerating — reset (0.5 m/s tolerance for jitter).
            lastMaxSpeed = maxSpeed
            sawSpeedIncrease = true
            maxSpeedStableCount = 0
            return
        }

        // Plateau. Only means burnout if we actually watched the climb that
        // came before it — otherwise we are just late to a flight already in
        // progress, and the "max speed" we'd read out is minutes stale.
        if (!sawSpeedIncrease) return
        maxSpeedStableCount++
        if (maxSpeedStableCount >= BURNOUT_STABLE_THRESHOLD) {
            burnoutAnnounced = true
            announceImmediate(
                "Burnout. Max speed " +
                    SpokenUnits.speed(last.toDouble(), unitSystem()),
            )
        }
    }

    /**
     * Drogue and main deployment, detected from the descent-rate curve — the
     * only signal present on BOTH the direct BLE link and the base-station
     * relay (see [DeploymentWatcher]).
     *
     * The deployment itself interrupts, because it is the callout the flyer is
     * waiting for. The verdict arrives a few seconds later on the ordinary
     * path, once the rate has settled enough to be worth quoting.
     */
    private fun checkDeployment(telemetry: TelemetryData, state: String) {
        val event = deployment.step(
            nowMs = clock(),
            altitudeRateMps = telemetry.altitudeRate,
            altAglM = telemetry.pressureAlt,
            afterApogee = telemetry.altApo,
            landed = telemetry.landedFlag || state == "LANDED",
            profile = recovery(),
        ) ?: return

        when (event) {
            is DeploymentEvent.Deployed -> {
                announceImmediate(deployedPhrase(event.kind))
                // Hold the cadence off so the readout that follows is the
                // verdict, not a periodic callout carrying the same number.
                lastDescentAnnounceMs = clock()
            }
            is DeploymentEvent.Verdict -> {
                announce(verdictPhrase(event))
                lastDescentAnnounceMs = clock()
            }
        }
    }

    private fun deployedPhrase(kind: DeploymentKind): String = when (kind) {
        DeploymentKind.DROGUE -> "Drogue out."
        DeploymentKind.MAIN -> "Main out."
        DeploymentKind.CHUTE -> "Chute out."
    }

    /**
     * A nominal rate earns the words "good chute"; anything else is reported as
     * the bare number (#813). Saying "no chute" at 200 m changes nothing the
     * flyer can do and is the callout most likely to be wrong — the rate itself
     * is the honest signal, and they can judge it.
     */
    private fun verdictPhrase(v: DeploymentEvent.Verdict): String {
        val rate = SpokenUnits.speed(v.rateMps, unitSystem())
        return if (v.nominal) "Good chute, descending $rate" else "Descending $rate"
    }

    private fun checkPeriodicAltitude(telemetry: TelemetryData) {
        val now = clock()
        if (now - lastAltitudeAnnounceMs < ALTITUDE_INTERVAL_MS) return
        val alt = telemetry.pressureAlt ?: return
        lastAltitudeAnnounceMs = now
        announce(altitudeWithRatePhrase(alt, telemetry.altitudeRate))
    }

    private fun checkDescentCallout(telemetry: TelemetryData) {
        val now = clock()
        if (now - lastDescentAnnounceMs < DESCENT_INTERVAL_MS) return
        val alt = telemetry.pressureAlt ?: return
        lastDescentAnnounceMs = now
        announce(altitudeWithRatePhrase(alt, telemetry.altitudeRate))
    }

    // ── Phrasing ─────────────────────────────────────────────────────────

    private fun altitudeWithRatePhrase(alt: Float, rate: Float?): String {
        val altStr = SpokenUnits.altitude(alt.toDouble(), unitSystem())
        val word = rate?.let { climbDescendWord(it) }
        return if (rate != null && word != null) {
            "$altStr, $word ${SpokenUnits.speed(abs(rate).toDouble(), unitSystem())}"
        } else {
            altStr
        }
    }

    private fun landedPhrase(telemetry: TelemetryData): String {
        val launch = launchLocation
        val lat = telemetry.latitude
        val lon = telemetry.longitude
        if (launch == null || lat == null || lon == null || lat.isNaN() || lon.isNaN()) {
            return "Landed."
        }
        val dist = DriftCast.haversineM(launch.first, launch.second, lat, lon)
        return "Landed. ${SpokenUnits.distance(dist, unitSystem())} away"
    }

    private fun captureLaunchLocation(telemetry: TelemetryData) {
        val lat = telemetry.latitude ?: return
        val lon = telemetry.longitude ?: return
        if (lat.isNaN() || lon.isNaN()) return
        launchLocation = lat to lon
    }

    // ── Speech routing ───────────────────────────────────────────────────

    /**
     * Periodic announcement: skipped if the engine is mid-utterance — the
     * next scheduled callout will carry fresher data than a queued stale one.
     */
    private fun announce(message: String) {
        if (speech.isBusy) return
        speech.speak(message, interrupt = false)
    }

    /** Critical announcement: cancels current speech and speaks now. */
    private fun announceImmediate(message: String) {
        speech.speak(message, interrupt = true)
    }

    // ── State reset ──────────────────────────────────────────────────────

    private fun resetFlightState() {
        burnoutAnnounced = false
        apogeeAnnounced = false
        landedAnnounced = false
        lastMaxSpeed = null
        sawSpeedIncrease = false
        maxSpeedStableCount = 0
        lastAltitudeAnnounceMs = Long.MIN_VALUE / 2
        lastDescentAnnounceMs = Long.MIN_VALUE / 2
        launchLocation = null
        deployment.reset()
    }

    public companion object {
        private const val ALTITUDE_INTERVAL_MS: Long = 5_000
        private const val DESCENT_INTERVAL_MS: Long = 10_000
        private const val BURNOUT_MIN_SPEED_MPS: Float = 10f
        private const val BURNOUT_STABLE_THRESHOLD: Int = 3

        /**
         * Direction word for a SIGNED altitude rate, or null within the
         * deadband (near-level → omit). Single source of truth so a spoken
         * direction can never contradict the rate's sign (#235).
         */
        public fun climbDescendWord(rate: Float, deadband: Float = 1.0f): String? = when {
            rate > deadband -> "climbing"
            rate < -deadband -> "descending"
            else -> null
        }
    }
}
