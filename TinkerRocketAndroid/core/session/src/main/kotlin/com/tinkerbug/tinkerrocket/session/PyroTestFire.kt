package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.PyroContinuity
import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.drop
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

/**
 * One test fire of one pyro channel (#1650; iOS `PyroTestView`'s state
 * machine, without the slow-motion video, which is #1673).
 *
 * FIRE starts a 10 s countdown with SAFE available throughout.  Rocket
 * logging starts at T-5 if it is off.  At T-0 the fire goes out only if
 * every guard still holds, re-checked at that instant:
 * - the command path is ready ([DeviceSession.pyroCommandPathReady]): on a
 *   base-station link, the followed rocket was heard live within 3 s and no
 *   second rocket shares its ID;
 * - on a base-station link, it still follows the rocket this test was opened
 *   for: the #1052 heal can re-pin the focus to another rocket on the air;
 * - on a direct link, the flight computer's rail is on: the out computer
 *   refuses cmd 36 with it off, since a queued fire would go off at the next
 *   power-on.
 *
 * Five seconds after the fire, logging this test started is stopped, and on
 * a base-station link continuity is tested again.  A fired charge burns the
 * match open, and the LoRa downlink has no fired bit, so that re-test is what
 * tells the operator whether it is safe to walk up ([verdictOf]).
 *
 * Everything runs on [scope]; the screen owns it and calls [close] when it
 * goes away.
 */
public class PyroTestFire(
    private val session: DeviceSession,
    public val channel: Int,
    private val scope: CoroutineScope,
) {
    public enum class Phase {
        IDLE,

        /** Counting down to the fire; SAFE aborts. */
        COUNTDOWN,

        /** The fire command went out; waiting before the re-test. */
        AFTER_FIRE,

        /** This run is over: fired, or stopped short with [State.error]. */
        DONE,
    }

    public data class State(
        val phase: Phase = Phase.IDLE,
        val secondsRemaining: Int = COUNTDOWN_S,
        val afterFireRemaining: Int = AFTER_FIRE_S,
        /** Why this run stopped short, or null. */
        val error: String? = null,
        /** The fire command went out this run. */
        val fired: Boolean = false,
        /** Rocket logging is on because this test turned it on. */
        val startedLogging: Boolean = false,
    )

    /** Why FIRE is disabled, or null when it is not. */
    public enum class FireBlock { NOT_CONNECTED, RAIL_OFF, ID_SHARED }

    /** The stand-back answer after a fire over LoRa. */
    public enum class Verdict {
        /** Continuity is being tested again. */
        RETESTING,

        /** A measured open on a live link to the same rocket: it fired. */
        FIRED,

        /** Continuity is still measured: the charge may not have fired. */
        STILL_PRESENT,

        /** The link went stale or the focus moved: nothing can be said. */
        LINK_LOST,

        /** Live, but no fresh measurement came back. */
        NO_READING,
    }

    private val _state = MutableStateFlow(State())
    public val state: StateFlow<State> = _state.asStateFlow()

    /** Base-station link: the rocket this test is FOR, latched by [open]. */
    public var latchedTargetRid: Int? = null
        private set

    private var ticker: Job? = null
    private var refusalWatch: Job? = null

    /**
     * Latch the target and test continuity once, before anything can fire
     * (iOS onAppear).  The refusal watch starts here: a cmd-36 refusal that
     * arrives mid-run aborts it.
     */
    public fun open() {
        latchedTargetRid = session.focusRocketId.value
        refusalWatch?.cancel()
        refusalWatch = scope.launch {
            // drop(1): a refusal from before this screen opened is not ours.
            session.pyroTestRefusal.drop(1).filterNotNull().collect { onRefusal(it) }
        }
        session.sendPyroContTest(channel)
    }

    /** Why FIRE must stay disabled right now (iOS `canFire`), or null. */
    public fun fireBlock(): FireBlock? = fireBlockOf(
        connected = session.isConnected.value,
        relay = session.isBaseStation,
        powerPinOn = session.telemetry.value.pwrPinOn,
        idShared = session.focusedRocketIdShared(),
    )

    /** Start the countdown.  Does nothing while blocked or already running. */
    public fun fire() {
        if (fireBlock() != null) return
        val phase = _state.value.phase
        if (phase == Phase.COUNTDOWN || phase == Phase.AFTER_FIRE) return
        // Per run: an old error, fired flag or count must not carry over.
        _state.update { State(phase = Phase.COUNTDOWN, startedLogging = it.startedLogging) }
        ticker?.cancel()
        ticker = scope.launch {
            while (true) {
                delay(1_000)
                if (!tick()) break
            }
        }
    }

    /** Abort the countdown: nothing is sent, and logging this test started stops. */
    public fun safe() {
        ticker?.cancel()
        ticker = null
        stopLoggingIfStarted()
        _state.update { it.copy(phase = Phase.IDLE, secondsRemaining = COUNTDOWN_S) }
    }

    /** The screen is going away: stop the clock and any logging this test started. */
    public fun close() {
        ticker?.cancel()
        ticker = null
        refusalWatch?.cancel()
        refusalWatch = null
        stopLoggingIfStarted()
    }

    /**
     * The post-fire answer on a base-station link, or null when there is
     * none to give: not a relay, not done, or nothing was fired.
     */
    public fun verdict(continuity: PyroContinuity): Verdict? = verdict(
        retesting = session.contTestPending(channel),
        connected = session.isConnected.value,
        dataStatus = session.effectiveDataStatus.value,
        focusRocketId = session.focusRocketId.value,
        continuity = continuity,
    )

    /** [verdict] from values a screen has already collected. */
    public fun verdict(
        retesting: Boolean,
        connected: Boolean,
        dataStatus: TelemetryData.DataStatus,
        focusRocketId: Int?,
        continuity: PyroContinuity,
    ): Verdict? {
        val s = _state.value
        if (!session.isBaseStation || s.phase != Phase.DONE || !s.fired) return null
        if (retesting) return Verdict.RETESTING
        val liveAndSameRocket = connected && dataStatus == TelemetryData.DataStatus.LIVE &&
            focusRocketId == latchedTargetRid
        return verdictOf(liveAndSameRocket, continuity)
    }

    /** One second of the run; false ends the clock. */
    private fun tick(): Boolean {
        val s = _state.value
        return when (s.phase) {
            Phase.COUNTDOWN -> {
                val left = s.secondsRemaining - 1
                _state.update { it.copy(secondsRemaining = left) }
                if (left == LOGGING_AT_S) startLoggingIfOff()
                if (left <= 0) fireNow() else true
            }
            Phase.AFTER_FIRE -> {
                val left = s.afterFireRemaining - 1
                _state.update { it.copy(afterFireRemaining = left) }
                if (left > 0) return true
                stopLoggingIfStarted()
                // Only while the base station still follows the rocket that
                // fired: a re-test would otherwise arm the other rocket, and
                // the verdict already says it cannot confirm.
                if (session.isBaseStation && session.focusRocketId.value == latchedTargetRid) {
                    session.sendPyroContTest(channel)
                }
                _state.update { it.copy(phase = Phase.DONE) }
                false
            }
            else -> false
        }
    }

    /** T-0: every guard again, at the instant, then the fire. */
    private fun fireNow(): Boolean {
        val relay = session.isBaseStation
        if (!session.pyroCommandPathReady()) {
            stopShort(
                if (relay) "Rocket LoRa link not ready — no fire command sent"
                else "Link lost before fire — no fire command sent",
            )
            return false
        }
        if (relay && session.focusRocketId.value != latchedTargetRid) {
            stopShort("Focused rocket changed — no fire command sent")
            return false
        }
        if (!relay && !session.telemetry.value.pwrPinOn) {
            safe()
            _state.update { it.copy(error = "Rocket power is off — no fire command sent") }
            return false
        }
        session.sendPyroFire(channel)
        _state.update {
            it.copy(phase = Phase.AFTER_FIRE, fired = true, afterFireRemaining = AFTER_FIRE_S)
        }
        return true
    }

    private fun stopShort(error: String) {
        ticker = null
        stopLoggingIfStarted()
        _state.update { it.copy(phase = Phase.DONE, error = error) }
    }

    /**
     * The out computer refused this channel's fire (rail off at dispatch).
     * The guards should make it unreachable; if it comes, abort through SAFE
     * and say why.
     */
    private fun onRefusal(refusal: PyroTestRefusal) {
        val phase = _state.value.phase
        if (refusal.cmd != 36 || refusal.channel != channel) return
        if (phase != Phase.COUNTDOWN && phase != Phase.AFTER_FIRE) return
        safe()
        _state.update { it.copy(error = "Rocket refused the fire — power is off") }
    }

    /**
     * Only if the rocket is not logging already, and remembered, so the stop
     * afterwards never ends a log the operator started (#385, #560).
     */
    private fun startLoggingIfOff() {
        if (session.telemetry.value.rocketLoggingActive) return
        session.setRocketLoggingForTest(true)
        _state.update { it.copy(startedLogging = true) }
    }

    private fun stopLoggingIfStarted() {
        if (!_state.value.startedLogging) return
        session.setRocketLoggingForTest(false)
        _state.update { it.copy(startedLogging = false) }
    }

    public companion object {
        public const val COUNTDOWN_S: Int = 10

        /** Rocket logging starts at T-5, so the fire is in the log. */
        public const val LOGGING_AT_S: Int = 5

        /** How long after the fire before the re-test. */
        public const val AFTER_FIRE_S: Int = 5

        /** iOS `canFire`, inverted: why FIRE is disabled, or null. */
        public fun fireBlockOf(
            connected: Boolean,
            relay: Boolean,
            powerPinOn: Boolean,
            idShared: Boolean,
        ): FireBlock? = when {
            !connected -> FireBlock.NOT_CONNECTED
            // A base station has no rail signal: the relay never carries it.
            !relay && !powerPinOn -> FireBlock.RAIL_OFF
            idShared -> FireBlock.ID_SHARED
            else -> null
        }

        /**
         * The stand-back verdict (iOS `pyroFireVerdict`).  Only a measured
         * open on a live stream from the same rocket says "fired"; every
         * absence of data says "treat it as live", never green.
         */
        public fun verdictOf(liveAndSameRocket: Boolean, continuity: PyroContinuity): Verdict = when {
            !liveAndSameRocket -> Verdict.LINK_LOST
            continuity == PyroContinuity.OPEN -> Verdict.FIRED
            continuity == PyroContinuity.PRESENT -> Verdict.STILL_PRESENT
            else -> Verdict.NO_READING
        }
    }
}
