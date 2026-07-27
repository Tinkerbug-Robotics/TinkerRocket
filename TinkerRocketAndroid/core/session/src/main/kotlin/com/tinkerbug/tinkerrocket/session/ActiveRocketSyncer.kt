package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.protocol.PyroChannelConfig
import com.tinkerbug.tinkerrocket.protocol.RollWaypoint
import com.tinkerbug.tinkerrocket.protocol.MagCalSubType
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.drop
import kotlinx.coroutines.flow.filter
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch
import java.util.UUID

/**
 * Pushes the active rocket profile to the connected flight computer (#132) —
 * port of iOS ActiveRocketSyncer.swift.  The app is source-of-truth: on
 * connect the WHOLE active profile is pushed so the rocket always flies the
 * profile's settings, never whatever happened to be in its NVS — the fix for
 * "flew the wrong rocket's settings".  Whole-profile (not a diff) because
 * the config readback doesn't echo every field; connects are infrequent and
 * the writes idempotent.  Offline edits ride out on the next connect free.
 *
 * Single-writer: all entry points run on the fleet dispatcher via [scope].
 */
public class ActiveRocketSyncer(private val scope: CoroutineScope) {

    public sealed interface SyncState {
        public data object Idle : SyncState          // not attached / base station
        public data object AwaitingSync : SyncState  // readback not in yet (#375)
        public data object NoProfile : SyncState
        public data object Syncing : SyncState
        public data object Synced : SyncState
        public data class Failed(val message: String) : SyncState
    }

    /** Cal is board-specific — it can't be blindly pushed. */
    public sealed interface CalAdvisory {
        public data object None : CalAdvisory
        public data object Missing : CalAdvisory
        public data class BoardMismatch(val savedOn: String, val current: String) : CalAdvisory
        public data object RocketHasUnsavedCal : CalAdvisory
    }

    private val _syncState = MutableStateFlow<SyncState>(SyncState.Idle)
    public val syncState: StateFlow<SyncState> = _syncState.asStateFlow()

    private val _magCalAdvisory = MutableStateFlow<CalAdvisory>(CalAdvisory.None)
    public val magCalAdvisory: StateFlow<CalAdvisory> = _magCalAdvisory.asStateFlow()

    private val _sensorCalAdvisory = MutableStateFlow<CalAdvisory>(CalAdvisory.None)
    public val sensorCalAdvisory: StateFlow<CalAdvisory> = _sensorCalAdvisory.asStateFlow()

    /** A profile previously flown on this board — suggested, never auto-applied. */
    private val _suggestedProfileId = MutableStateFlow<UUID?>(null)
    public val suggestedProfileId: StateFlow<UUID?> = _suggestedProfileId.asStateFlow()

    private var session: DeviceSession? = null
    private var store: RocketProfileStore? = null
    private var jobs = mutableListOf<Job>()
    private var magCalReadPending = false
    private var sensorCalReadPending = false
    private var syncedTimer: Job? = null

    /**
     * Role at attach time.  A renamed device can mis-parse its type from the
     * BLE name until config_identity lands (the SUBSONIC case), so the role
     * isn't final — a role flip falls through to a full re-attach (#375).
     */
    private var attachedAsBaseStation = false

    public fun attach(session: DeviceSession, store: RocketProfileStore) {
        // Idempotent for the same pair under the same role (#375): redundant
        // re-attaches must not reset a Synced state; a NEW session object
        // (reconnect) or a corrected role falls through to full re-attach.
        if (session === this.session && store === this.store &&
            attachedAsBaseStation == session.isBaseStation
        ) return

        detach()
        this.session = session
        this.store = store
        attachedAsBaseStation = session.isBaseStation

        if (session.isBaseStation) {
            // BS is a read-only display of the active rocket; never pushed.
            _syncState.value = SyncState.Idle
            return
        }

        // Visible from the first moment we own this rocket: "connected but
        // not yet pushed" must never render as silent Idle (#375).
        _syncState.value = SyncState.AwaitingSync

        // Sync ONCE when the config readback AND the hardware id are both in
        // (unitId arrives in a later readback than the main config).
        jobs += scope.launch {
            combine(session.rocketConfig, session.identity) { cfg, id -> cfg != null && id.unitId != null }
                .filter { it }
                .first()
            onReadyToSync()
        }

        // Connect-time cal READ replies (only honoured right after we ask).
        jobs += scope.launch {
            session.magCalStatus.filterNotNull().collect { status ->
                if (!magCalReadPending) return@collect
                magCalReadPending = false
                _magCalAdvisory.value =
                    if (status.subType == MagCalSubType.APPLIED) CalAdvisory.RocketHasUnsavedCal
                    else CalAdvisory.Missing
            }
        }
        jobs += scope.launch {
            session.sensorCalStatus.filterNotNull().collect { status ->
                if (!sensorCalReadPending) return@collect
                sensorCalReadPending = false
                _sensorCalAdvisory.value =
                    if (status.valid) CalAdvisory.RocketHasUnsavedCal else CalAdvisory.Missing
            }
        }

        // Re-push when the user switches the active profile mid-connection.
        jobs += scope.launch {
            store.activeId.drop(1).collect {
                val s = this@ActiveRocketSyncer.session ?: return@collect
                if (!s.isConnected.value || s.isBaseStation) return@collect
                _suggestedProfileId.value = null   // user has chosen
                performSync()
            }
        }
    }

    public fun detach() {
        jobs.forEach { it.cancel() }
        jobs.clear()
        syncedTimer?.cancel()
        syncedTimer = null
        session = null
        store = null
        attachedAsBaseStation = false
        magCalReadPending = false
        sensorCalReadPending = false
        _syncState.value = SyncState.Idle
        _magCalAdvisory.value = CalAdvisory.None
        _sensorCalAdvisory.value = CalAdvisory.None
        _suggestedProfileId.value = null
    }

    private fun onReadyToSync() {
        val s = session ?: return
        if (s.isBaseStation) return
        val st = store ?: return
        _suggestedProfileId.value = suggestedProfile(
            st.profiles.value, st.activeId.value, s.identity.value.unitId.orEmpty(),
        )
        performSync()
    }

    private fun performSync() {
        val s = session
        val st = store
        if (s == null || st == null || !s.isConnected.value) {
            _syncState.value = SyncState.Idle
            return
        }
        val profile = st.activeProfile ?: run {
            _syncState.value = SyncState.NoProfile
            return
        }

        _syncState.value = SyncState.Syncing

        s.sendCommandFrame(
            Commands.servoConfig(
                biasesUs = listOf(profile.servoBias1, profile.servoBias2, profile.servoBias3, profile.servoBias4),
                hz = profile.servoHz, minUs = profile.servoMinUs, maxUs = profile.servoMaxUs,
                finMinDeg = profile.finMinDeg, finMaxDeg = profile.finMaxDeg,
            ),
        )
        s.sendCommandFrame(
            Commands.pidConfig(
                kp = profile.pidKp, ki = profile.pidKi, kd = profile.pidKd,
                minCmd = profile.pidMinCmd, maxCmd = profile.pidMaxCmd,
            ),
        )
        s.sendCommandFrame(Commands.servoEnable(profile.servoControlEnabled))
        s.sendCommandFrame(Commands.gainScheduleEnable(profile.gainScheduleEnabled))
        s.sendCommandFrame(
            Commands.rollControlConfig(
                useAngleControl = profile.useAngleControl,
                rollDelayMs = profile.rollDelayMs,
                rateCapDps = profile.rateCapDps,
                kpAngle = profile.kpAngle,
                integralSepThreshold = profile.integralSepThreshold,
            ),
        )
        // mode byte is legacy wire (pre-v4); always .angle = 0.
        s.sendCommandFrame(
            Commands.rollProfile(
                profile.rollWaypoints.map {
                    RollWaypoint(timeS = it.timeSeconds, angleDeg = it.angleDeg, mode = 0)
                },
            ),
        )
        s.sendCommandFrame(
            Commands.guidanceConfig(
                enabled = profile.guidanceEnabled,
                navGain = profile.pnNavGain, maxAccel = profile.pnMaxAccel,
                accelToFin = profile.pnAccelToFin, maxFinDeg = profile.pnMaxFinDeg,
                minSpeed = profile.pnMinSpeed,
                coastDelayMs = profile.pnCoastDelayMs, targetMode = profile.pnTargetMode,
                targetE = profile.pnTargetE, targetN = profile.pnTargetN,
                targetAlt = profile.pnTargetAltM,
                kpPos = profile.pnKpPos, kdVel = profile.pnKdVel,
                guidanceLaw = profile.pnGuidanceLaw,
            ),
        )
        Commands.finConfig(
            ringMode = profile.finRingMode,
            servoAtSlot = profile.finServoAtSlot,
            reverse = profile.finReverse,
            rollReverse = profile.finRollReverse,
        )?.let { s.sendCommandFrame(it) }
        s.sendCommandFrame(Commands.cameraConfig(profile.cameraType))
        s.sendCommandFrame(Commands.imuOrient(profile.imuOrientSetting))
        s.sendCommandFrame(Commands.imuRate(profile.imuRateHz))
        s.sendCommandFrame(Commands.soundsEnable(profile.soundsEnabled))
        s.sendCommandFrame(
            Commands.pyroConfig(
                listOf(
                    PyroChannelConfig(profile.pyro1Enabled, profile.pyro1TriggerMode, profile.pyro1TriggerValue),
                    PyroChannelConfig(profile.pyro2Enabled, profile.pyro2TriggerMode, profile.pyro2TriggerValue),
                    PyroChannelConfig(profile.pyro3Enabled, profile.pyro3TriggerMode, profile.pyro3TriggerValue),
                    PyroChannelConfig(profile.pyro4Enabled, profile.pyro4TriggerMode, profile.pyro4TriggerValue),
                ),
            ),
        )

        syncCal(profile, s)

        // Record which board this profile last flew on (soft pre-select).
        st.update(profile.id) { it.copy(lastUsedUnitID = s.identity.value.unitId) }

        // Optimistic (no per-command ack on this link); hold "syncing"
        // briefly so the badge is visible.
        syncedTimer?.cancel()
        syncedTimer = scope.launch {
            delay(SYNCED_DELAY_MS)
            if (_syncState.value == SyncState.Syncing) _syncState.value = SyncState.Synced
        }
    }

    private fun syncCal(profile: RocketProfile, s: DeviceSession) {
        val unitId = s.identity.value.unitId.orEmpty()
        when (val action = magCalSyncAction(profile.magCal, unitId)) {
            is CalAction.Push -> {
                _magCalAdvisory.value = CalAdvisory.None
                val cal = action.mag!!
                s.sendCommandFrame(
                    Commands.magCalApply(cal.offsetX, cal.offsetY, cal.offsetZ, cal.fieldRuT, cal.residualUT),
                )
            }
            is CalAction.WarnMismatch ->
                _magCalAdvisory.value = CalAdvisory.BoardMismatch(action.savedOn, action.current)
            CalAction.ReadRocket -> {
                magCalReadPending = true
                s.sendBareCommand(BleCommandId.MAG_CAL_READ_OC)
            }
        }
        when (val action = sensorCalSyncAction(profile.sensorCal, unitId)) {
            is CalAction.Push -> {
                _sensorCalAdvisory.value = CalAdvisory.None
                val cal = action.sensor!!
                s.sendCommandFrame(
                    Commands.sensorCalApply(cal.gyroX, cal.gyroY, cal.gyroZ, cal.hgX, cal.hgY, cal.hgZ),
                )
            }
            is CalAction.WarnMismatch ->
                _sensorCalAdvisory.value = CalAdvisory.BoardMismatch(action.savedOn, action.current)
            CalAction.ReadRocket -> {
                sensorCalReadPending = true
                s.sendBareCommand(BleCommandId.SENSOR_CAL_READ_OC)
            }
        }
    }

    /** Import the rocket's applied mag cal into the active profile (advisory accept). */
    public fun importRocketCalIntoActiveProfile(nowMs: Long) {
        val s = session ?: return
        val st = store ?: return
        val profile = st.activeProfile ?: return
        val status = s.magCalStatus.value ?: return
        if (status.subType != MagCalSubType.APPLIED) return
        st.update(profile.id) {
            it.copy(
                magCal = MagCalData(
                    offsetX = status.offsetX, offsetY = status.offsetY, offsetZ = status.offsetZ,
                    fieldRuT = status.fieldRUt, residualUT = status.residualUt,
                    calibratedOnUnitID = s.identity.value.unitId.orEmpty(),
                    calibratedAtMs = nowMs,
                ),
            )
        }
        _magCalAdvisory.value = CalAdvisory.None
    }

    public companion object {
        public const val SYNCED_DELAY_MS: Long = 800

        /** Pure cal-sync decision, shared by mag + sensor cal (testable). */
        public sealed interface CalAction {
            public data class Push(val mag: MagCalData? = null, val sensor: SensorCalData? = null) : CalAction
            public data class WarnMismatch(val savedOn: String, val current: String) : CalAction
            public data object ReadRocket : CalAction
        }

        public fun magCalSyncAction(cal: MagCalData?, deviceUnitId: String): CalAction =
            when {
                cal == null -> CalAction.ReadRocket
                cal.calibratedOnUnitID == deviceUnitId -> CalAction.Push(mag = cal)
                else -> CalAction.WarnMismatch(cal.calibratedOnUnitID, deviceUnitId)
            }

        public fun sensorCalSyncAction(cal: SensorCalData?, deviceUnitId: String): CalAction =
            when {
                cal == null -> CalAction.ReadRocket
                cal.calibratedOnUnitID == deviceUnitId -> CalAction.Push(sensor = cal)
                else -> CalAction.WarnMismatch(cal.calibratedOnUnitID, deviceUnitId)
            }

        /** A profile previously flown on [unitId] other than the active one. */
        public fun suggestedProfile(
            profiles: List<RocketProfile>,
            active: UUID?,
            unitId: String,
        ): UUID? {
            if (unitId.isEmpty()) return null
            return profiles.firstOrNull { it.lastUsedUnitID == unitId && it.id != active }?.id
        }
    }
}
