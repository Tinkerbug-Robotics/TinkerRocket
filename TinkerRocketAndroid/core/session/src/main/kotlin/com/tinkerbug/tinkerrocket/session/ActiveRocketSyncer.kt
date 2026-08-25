package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.protocol.PyroChannelConfig
import com.tinkerbug.tinkerrocket.protocol.RollWaypoint
import com.tinkerbug.tinkerrocket.protocol.MagCalSubType
import com.tinkerbug.tinkerrocket.protocol.RocketConfig
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.drop
import kotlinx.coroutines.flow.filter
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.launch
import java.util.UUID

/**
 * Reconciles the connected rocket's own settings with the app's profile for
 * that airframe (#132, #915) — port of iOS ActiveRocketSyncer.swift.  The
 * ROCKET is source-of-truth on connect: it reports what it has, the app
 * adopts it into the profile bound to that board by `lastUsedUnitID`, and
 * nothing is written to the vehicle unless the user asks.
 *
 * Why this way round (#915 reverses #132): the rocket persists essentially
 * its whole config in NVS and reloads it at boot, so it already remembers
 * what it was configured as.  Under the old rule, connecting while the phone
 * held a different airframe's profile silently reconfigured the vehicle —
 * fourteen config frames inside two seconds, no user action, no UI notice.
 *
 * The whole profile still goes out on an explicit act: switching the active
 * profile onto a connected rocket, or [pushProfileToRocket] from the UI.
 * The readback covers only about half the editable surface, so
 * [unreportedGroups] names what the app cannot verify.
 *
 * Single-writer: all entry points run on the fleet dispatcher via [scope].
 */
public class ActiveRocketSyncer(private val scope: CoroutineScope) {

    public sealed interface SyncState {
        public data object Idle : SyncState          // not attached / base station
        public data object AwaitingSync : SyncState  // readback not in yet (#375)
        public data object NoProfile : SyncState
        public data object Syncing : SyncState       // explicit whole-profile push in flight
        public data object Synced : SyncState        // profile and rocket agree on all reported
        /**
         * The rocket reported settings the profile disagreed with; the profile
         * was updated to match and these groups changed (#915).  Informational,
         * not a fault — the rocket kept flying what it had.
         */
        public data class Adopted(val groups: List<String>) : SyncState
        public data class Failed(val message: String) : SyncState
    }

    /**
     * One editable settings group = the command frame(s) that carry it.
     * Enum order IS the connect-time wire order — keep it stable.
     */
    public enum class ConfigGroup {
        SERVO, PID, SERVO_ENABLE, GAIN_SCHEDULE, ROLL_CONTROL, ROLL_PROFILE,
        GUIDANCE, FIN_LAYOUT, CAMERA, IMU, SOUNDS, PYRO,
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

    /**
     * A profile previously flown on this board — suggested, never auto-applied.
     * Since #915 binds the board's profile automatically this is normally null
     * (the bound profile IS the active one, and this excludes the active id);
     * it still fires if a second profile claims the same board.
     */
    private val _suggestedProfileId = MutableStateFlow<UUID?>(null)
    public val suggestedProfileId: StateFlow<UUID?> = _suggestedProfileId.asStateFlow()

    /**
     * Setting groups the CONNECTED rocket does not report back, so the app
     * shows the profile's values for them and cannot verify them against the
     * vehicle.  Empty once the firmware's config report has landed; the full
     * pre-report list on older firmware, and the servo/fin/guidance groups on
     * a mini, which has none of that hardware.
     */
    private val _unreportedGroups = MutableStateFlow<List<String>>(emptyList())
    public val unreportedGroups: StateFlow<List<String>> = _unreportedGroups.asStateFlow()

    /**
     * Name of a profile this syncer created for a board it had never seen,
     * seeded from the rocket's own reported settings (#915).  Surfaced as an
     * info line so a silently-created profile is never a surprise.
     */
    private val _createdProfileName = MutableStateFlow<String?>(null)
    public val createdProfileName: StateFlow<String?> = _createdProfileName.asStateFlow()

    private var session: DeviceSession? = null
    private var store: RocketProfileStore? = null
    private var jobs = mutableListOf<Job>()
    private var magCalReadPending = false
    private var sensorCalReadPending = false
    private var syncedTimer: Job? = null

    /**
     * The profile id THIS syncer made active while binding the board.  The
     * activeId collection is delivered on a later dispatcher turn, so it
     * cannot tell our own bind from a user tap — without this, binding a
     * board would push the profile straight back at the rocket, reinstating
     * the exact #915 behaviour the bind exists to remove.
     */
    private var selfSelectedProfileId: UUID? = null

    /**
     * The orientation setting rides a LATER readback frame than the main
     * config, so the first adoption pass usually runs without it.  Armed once
     * per attach; never re-armed, so a later user edit isn't mistaken for a
     * rocket report.
     */
    private var orientAdoptArmed = false

    /** Armed once per attach, for the same reason as [orientAdoptArmed]. */
    private var extrasAdoptArmed = false

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

        // #375 (the SUBSONIC case): the role is parsed from the BLE name and
        // isn't final until config_identity lands ~1 s in — a "nimble"-named
        // BS attaches as a rocket and would stick in AwaitingSync forever.
        // Watch for the flip ourselves; a fresh coroutine re-attaches so the
        // detach() inside doesn't cancel the job mid-collect.
        jobs += scope.launch {
            session.identity
                .map { it.deviceType == BleDeviceType.BASE_STATION }
                .distinctUntilChanged()
                .drop(1)
                .collect {
                    scope.launch {
                        val s = this@ActiveRocketSyncer.session ?: return@launch
                        val st = this@ActiveRocketSyncer.store ?: return@launch
                        attach(s, st)
                    }
                }
        }

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

        // Push when the USER switches the active profile mid-connection.
        jobs += scope.launch {
            store.activeId.drop(1).collect {
                val s = this@ActiveRocketSyncer.session ?: return@collect
                if (!s.isConnected.value || s.isBaseStation) return@collect
                // Our own bind, arriving late — not a user choice, so no push.
                val mine = selfSelectedProfileId
                if (mine != null && store.activeId.value == mine) {
                    selfSelectedProfileId = null
                    return@collect
                }
                selfSelectedProfileId = null
                _suggestedProfileId.value = null   // user has chosen
                _createdProfileName.value = null
                // Choosing a different profile for a CONNECTED rocket is the
                // explicit "make this rocket fly these settings" act, so it
                // still pushes — the one place #915 keeps the old behaviour.
                pushProfileToRocket()
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
        selfSelectedProfileId = null
        orientAdoptArmed = false
        extrasAdoptArmed = false
        _syncState.value = SyncState.Idle
        _magCalAdvisory.value = CalAdvisory.None
        _sensorCalAdvisory.value = CalAdvisory.None
        _suggestedProfileId.value = null
        _createdProfileName.value = null
        _unreportedGroups.value = emptyList()
    }

    private fun onReadyToSync() {
        val s = session ?: return
        if (s.isBaseStation) return
        val st = store ?: return
        bindProfileToBoard()
        _suggestedProfileId.value = suggestedProfile(
            st.profiles.value, st.activeId.value, s.identity.value.unitId.orEmpty(),
        )
        adoptRocketConfig()
    }

    /**
     * Make the profile bound to the connected board the active one, creating
     * one from the rocket's own settings if this board is new to the app.
     *
     * `lastUsedUnitID` was already a per-board binding; before #915 it only
     * drove a suggestion the user had to accept while the *active* profile was
     * pushed regardless.  Reading the binding instead of ignoring it is what
     * lets the phone connect to any rocket in the field without changing it.
     */
    private fun bindProfileToBoard() {
        val s = session ?: return
        val st = store ?: return
        val unitId = s.identity.value.unitId.orEmpty()
        if (unitId.isEmpty()) return

        val bound = st.profiles.value.firstOrNull { it.lastUsedUnitID == unitId }
        if (bound != null) {
            if (st.activeId.value != bound.id) {
                selfSelectedProfileId = bound.id
                st.setActive(bound.id)
            }
            return
        }

        // Board we have never seen: adopt it as its own profile.  Pushing the
        // active profile here instead would be exactly the #915 failure —
        // connect to an unfamiliar rocket, reconfigure it.
        val name = s.identity.value.unitName?.takeIf { it.isNotEmpty() }
            ?: "Rocket ${unitId.takeLast(4)}"
        val created = st.add(name)
        st.update(created.id) { it.copy(lastUsedUnitID = unitId) }
        selfSelectedProfileId = created.id
        st.setActive(created.id)
        _createdProfileName.value = st.profiles.value.firstOrNull { it.id == created.id }?.name
    }

    /**
     * Take the rocket's reported settings into the bound profile.  Nothing is
     * written to the rocket.
     */
    private fun adoptRocketConfig() {
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
        val cfg = s.rocketConfig.value ?: run {
            _syncState.value = SyncState.AwaitingSync
            return
        }

        _unreportedGroups.value = cfg.unreportedGroups

        val result = adopt(profile, cfg)
        if (result.changed.isNotEmpty()) st.update(profile.id) { result.profile }

        // A profile we just created from this same readback matches the rocket
        // by construction — reporting its every factory-default field as
        // "changed" would be noise.  createdProfileName covers it.
        _syncState.value =
            if (result.changed.isEmpty() || _createdProfileName.value != null) SyncState.Synced
            else SyncState.Adopted(result.changed)

        armOrientationAdopt(s, cfg)
        armExtrasAdopt(s, cfg)

        syncCal(profile, s)
    }

    /**
     * The orientation setting arrives in a later readback frame than the main
     * config, so adopt it again when it lands.  Only while it is still
     * missing, only once, and only on firmware that reports it at all
     * (pre-v3-orientation FCs never send it — the collect simply never fires
     * and the profile keeps its own value).
     */
    private fun armOrientationAdopt(s: DeviceSession, cfg: RocketConfig) {
        if (cfg.imuOrientSetting != null || orientAdoptArmed) return
        orientAdoptArmed = true
        jobs += scope.launch {
            val setting = s.rocketConfig
                .map { it?.imuOrientSetting }
                .filterNotNull()
                .first()
            val st = this@ActiveRocketSyncer.store ?: return@launch
            if (!s.isConnected.value) return@launch
            val profile = st.activeProfile ?: return@launch
            if (profile.imuOrientSetting == setting) return@launch
            st.update(profile.id) { it.copy(imuOrientSetting = setting) }
            if (_createdProfileName.value != null) return@launch
            val existing = (_syncState.value as? SyncState.Adopted)?.groups.orEmpty()
            if (GROUP_IMU_ORIENTATION !in existing) {
                _syncState.value = SyncState.Adopted(existing + GROUP_IMU_ORIENTATION)
            }
        }
    }

    /**
     * The three #915 config-report frames land after the main config readback,
     * in their own paced slots.  Re-adopt as they arrive so the profile picks
     * up the fin layout and guidance parameters without a reconnect.  Armed
     * once per attach, so a later user edit — which also flows back through
     * the report — is not reported as the rocket disagreeing with the phone.
     */
    private fun armExtrasAdopt(s: DeviceSession, cfg: RocketConfig) {
        if (cfg.unreportedGroups.isEmpty() || extrasAdoptArmed) return
        extrasAdoptArmed = true
        jobs += scope.launch {
            s.rocketConfig
                .filterNotNull()
                .filter { it.unreportedGroups.isEmpty() }
                .first()
            adoptRocketConfig()
        }
    }

    /**
     * Write the whole active profile to the connected rocket.  This is the old
     * #132 connect-time behaviour, kept as a deliberate user action:
     * provisioning a fresh board, cloning a known-good setup onto a
     * replacement computer, or making the unreported groups true after editing
     * them offline.
     */
    public fun pushProfileToRocket() {
        val s = session
        val st = store
        if (s == null || st == null || !s.isConnected.value || s.isBaseStation) {
            _syncState.value = SyncState.Idle
            return
        }
        val profile = st.activeProfile ?: run {
            _syncState.value = SyncState.NoProfile
            return
        }

        _syncState.value = SyncState.Syncing

        ConfigGroup.entries.forEach { pushGroupFrames(s, profile, it) }

        syncCal(profile, s)

        // This profile now owns this board.
        st.update(profile.id) { it.copy(lastUsedUnitID = s.identity.value.unitId) }

        // Optimistic (no per-command ack on this link); hold "syncing"
        // briefly so the badge is visible.
        syncedTimer?.cancel()
        syncedTimer = scope.launch {
            delay(SYNCED_DELAY_MS)
            if (_syncState.value == SyncState.Syncing) _syncState.value = SyncState.Synced
        }
    }

    /**
     * Push ONE group of the active profile — settings self-apply (#144):
     * an edit reaches a connected rocket immediately, never waits behind an
     * Apply button.  No-op when detached / disconnected / base station; the
     * offline edit rides out on the next connect via the whole-profile push.
     * Call on the fleet dispatcher; doesn't touch syncState (a field edit
     * while Synced stays Synced).
     */
    public fun pushGroup(group: ConfigGroup) {
        val s = session ?: return
        val profile = store?.activeProfile ?: return
        if (!s.isConnected.value || s.isBaseStation) return
        pushGroupFrames(s, profile, group)
    }

    private fun pushGroupFrames(s: DeviceSession, profile: RocketProfile, group: ConfigGroup) {
        when (group) {
            ConfigGroup.SERVO -> s.sendCommandFrame(
                Commands.servoConfig(
                    biasesUs = listOf(profile.servoBias1, profile.servoBias2, profile.servoBias3, profile.servoBias4),
                    hz = profile.servoHz, minUs = profile.servoMinUs, maxUs = profile.servoMaxUs,
                    finMinDeg = profile.finMinDeg, finMaxDeg = profile.finMaxDeg,
                ),
            )
            ConfigGroup.PID -> s.sendCommandFrame(
                Commands.pidConfig(
                    kp = profile.pidKp, ki = profile.pidKi, kd = profile.pidKd,
                    minCmd = profile.pidMinCmd, maxCmd = profile.pidMaxCmd,
                ),
            )
            ConfigGroup.SERVO_ENABLE -> s.sendCommandFrame(Commands.servoEnable(profile.servoControlEnabled))
            ConfigGroup.GAIN_SCHEDULE -> s.sendCommandFrame(Commands.gainScheduleEnable(profile.gainScheduleEnabled))
            ConfigGroup.ROLL_CONTROL -> s.sendCommandFrame(
                Commands.rollControlConfig(
                    useAngleControl = profile.useAngleControl,
                    rollDelayMs = profile.rollDelayMs,
                    rateCapDps = profile.rateCapDps,
                    kpAngle = profile.kpAngle,
                    integralSepThreshold = profile.integralSepThreshold,
                ),
            )
            // mode byte is legacy wire (pre-v4); always .angle = 0.
            ConfigGroup.ROLL_PROFILE -> s.sendCommandFrame(
                Commands.rollProfile(
                    profile.rollWaypoints.map {
                        RollWaypoint(timeS = it.timeSeconds, angleDeg = it.angleDeg, mode = 0)
                    },
                ),
            )
            ConfigGroup.GUIDANCE -> s.sendCommandFrame(
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
            ConfigGroup.FIN_LAYOUT -> Commands.finConfig(
                ringMode = profile.finRingMode,
                servoAtSlot = profile.finServoAtSlot,
                reverse = profile.finReverse,
                rollReverse = profile.finRollReverse,
            )?.let { s.sendCommandFrame(it) }
            ConfigGroup.CAMERA -> s.sendCommandFrame(Commands.cameraConfig(profile.cameraType))
            ConfigGroup.IMU -> {
                s.sendCommandFrame(Commands.imuOrient(profile.imuOrientSetting))
                s.sendCommandFrame(Commands.imuRate(profile.imuRateHz))
            }
            ConfigGroup.SOUNDS -> s.sendCommandFrame(Commands.soundsEnable(profile.soundsEnabled))
            ConfigGroup.PYRO -> {
                val channels = listOf(
                    PyroChannelConfig(profile.pyro1Enabled, profile.pyro1TriggerMode, profile.pyro1TriggerValue),
                    PyroChannelConfig(profile.pyro2Enabled, profile.pyro2TriggerMode, profile.pyro2TriggerValue),
                    PyroChannelConfig(profile.pyro3Enabled, profile.pyro3TriggerMode, profile.pyro3TriggerValue),
                    PyroChannelConfig(profile.pyro4Enabled, profile.pyro4TriggerMode, profile.pyro4TriggerValue),
                )
                s.sendCommandFrame(Commands.pyroConfig(channels))
                // iOS applyPyroConfig step 3, from the ONE source the push
                // used: the firmware never echoes config_pyro after a write,
                // so without this the dashboard tiles show the last readback
                // until the next connect.
                s.mirrorPyroConfig(channels)
            }
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

        // Group labels for the "what the rocket disagreed about" line.  Groups,
        // not fields: "PID gains" reads better on a phone than five numbers,
        // and the settings screen is organised the same way.
        public const val GROUP_SERVO_TRIM: String = "Servo trim"
        public const val GROUP_SERVO_TIMING: String = "Servo timing"
        public const val GROUP_PID_GAINS: String = "PID gains"
        public const val GROUP_SERVO_CONTROL: String = "Servo control"
        public const val GROUP_GAIN_SCHEDULE: String = "Gain schedule"
        public const val GROUP_ROLL_CONTROL: String = "Roll control"
        public const val GROUP_GUIDANCE_ENABLE: String = "Guidance"
        public const val GROUP_CAMERA: String = "Camera"
        public const val GROUP_IMU_ORIENTATION: String = "IMU orientation"
        public const val GROUP_IMU_RATE: String = "IMU rate"

        // #915 firmware follow-up: reachable only once the rocket reports them.
        public const val GROUP_SERVO_TRIM_24: String = "Servo trim 2-4"
        public const val GROUP_FIN_TRAVEL: String = "Fin travel"
        public const val GROUP_FIN_LAYOUT: String = "Fin layout"
        public const val GROUP_SOUNDS: String = "Sounds"
        public const val GROUP_GUIDANCE_PARAMS: String = "Guidance parameters"
        public const val GROUP_ROLL_PROFILE: String = "Roll profile"


        /** Outcome of [adopt]: the reconciled profile and what differed. */
        public data class AdoptionResult(
            val profile: RocketProfile,
            val changed: List<String>,
        )

        /**
         * Reconcile a profile against what the rocket reported — rocket wins —
         * and name the groups that differed.  Pure, so the precedence rule is
         * testable without a peripheral (mirrors [magCalSyncAction]).
         *
         * Only fields the readback actually echoes are touched.  Anything the
         * rocket doesn't report (see [unreportedGroups]) is left exactly as the
         * profile has it, because the alternative — resetting it to a default
         * we invented — would be a silent change of its own.
         *
         * Comparisons run at the precision the value crossed the wire at, so a
         * profile Kp of 0.12 doesn't "differ" from a rocket reporting 0.1200
         * every single connect.
         */
        public fun adopt(profile: RocketProfile, cfg: RocketConfig): AdoptionResult {
            var p = profile
            val changed = mutableListOf<String>()

            if (p.servoBias1 != cfg.servoBias1) {
                p = p.copy(servoBias1 = cfg.servoBias1)
                changed += GROUP_SERVO_TRIM
            }

            if (p.servoHz != cfg.servoHz || p.servoMinUs != cfg.servoMinUs ||
                p.servoMaxUs != cfg.servoMaxUs
            ) {
                p = p.copy(
                    servoHz = cfg.servoHz,
                    servoMinUs = cfg.servoMinUs,
                    servoMaxUs = cfg.servoMaxUs,
                )
                changed += GROUP_SERVO_TIMING
            }

            if (!same(p.pidKp, cfg.pidKp, 4) || !same(p.pidKi, cfg.pidKi, 4) ||
                !same(p.pidKd, cfg.pidKd, 4) || !same(p.pidMinCmd, cfg.pidMinCmd, 1) ||
                !same(p.pidMaxCmd, cfg.pidMaxCmd, 1)
            ) {
                p = p.copy(
                    pidKp = cfg.pidKp, pidKi = cfg.pidKi, pidKd = cfg.pidKd,
                    pidMinCmd = cfg.pidMinCmd, pidMaxCmd = cfg.pidMaxCmd,
                )
                changed += GROUP_PID_GAINS
            }

            if (p.servoControlEnabled != cfg.servoEnabled) {
                p = p.copy(servoControlEnabled = cfg.servoEnabled)
                changed += GROUP_SERVO_CONTROL
            }

            if (p.gainScheduleEnabled != cfg.gainScheduleEnabled) {
                p = p.copy(gainScheduleEnabled = cfg.gainScheduleEnabled)
                changed += GROUP_GAIN_SCHEDULE
            }

            // The three roll gains ride sentinels meaning "firmware default"
            // (#253).  When the rocket sends those, RocketConfig holds the
            // app's defaults rather than the vehicle's values — adopting them
            // would overwrite the profile with a number nobody chose.
            var rollDiffers = p.useAngleControl != cfg.useAngleControl ||
                p.rollDelayMs != cfg.rollDelayMs
            if (cfg.rollGainsReported) {
                rollDiffers = rollDiffers ||
                    !same(p.rateCapDps, cfg.rateCapDps, 1) ||
                    !same(p.kpAngle, cfg.kpAngle, 2) ||
                    !same(p.integralSepThreshold, cfg.integralSepThreshold, 1)
            }
            if (rollDiffers) {
                p = p.copy(
                    useAngleControl = cfg.useAngleControl,
                    rollDelayMs = cfg.rollDelayMs,
                )
                if (cfg.rollGainsReported) {
                    p = p.copy(
                        rateCapDps = cfg.rateCapDps,
                        kpAngle = cfg.kpAngle,
                        integralSepThreshold = cfg.integralSepThreshold,
                    )
                }
                changed += GROUP_ROLL_CONTROL
            }

            // Enable flag only — the PN parameters behind it aren't reported.
            if (p.guidanceEnabled != cfg.guidanceEnabled) {
                p = p.copy(guidanceEnabled = cfg.guidanceEnabled)
                changed += GROUP_GUIDANCE_ENABLE
            }

            if (p.cameraType != cfg.cameraType) {
                p = p.copy(cameraType = cfg.cameraType)
                changed += GROUP_CAMERA
            }

            // null = this firmware doesn't report it; keep what the profile has.
            cfg.imuOrientSetting?.let {
                if (p.imuOrientSetting != it) {
                    p = p.copy(imuOrientSetting = it)
                    changed += GROUP_IMU_ORIENTATION
                }
            }
            cfg.imuRateHz?.let {
                if (p.imuRateHz != it) {
                    p = p.copy(imuRateHz = it)
                    changed += GROUP_IMU_RATE
                }
            }

            val rocketPyro = listOf(
                Triple(cfg.pyro1Enabled, cfg.pyro1TriggerMode, cfg.pyro1TriggerValue),
                Triple(cfg.pyro2Enabled, cfg.pyro2TriggerMode, cfg.pyro2TriggerValue),
                Triple(cfg.pyro3Enabled, cfg.pyro3TriggerMode, cfg.pyro3TriggerValue),
                Triple(cfg.pyro4Enabled, cfg.pyro4TriggerMode, cfg.pyro4TriggerValue),
            )
            val profilePyro = listOf(
                Triple(p.pyro1Enabled, p.pyro1TriggerMode, p.pyro1TriggerValue),
                Triple(p.pyro2Enabled, p.pyro2TriggerMode, p.pyro2TriggerValue),
                Triple(p.pyro3Enabled, p.pyro3TriggerMode, p.pyro3TriggerValue),
                Triple(p.pyro4Enabled, p.pyro4TriggerMode, p.pyro4TriggerValue),
            )
            for (i in 0..3) {
                if (profilePyro[i].first != rocketPyro[i].first ||
                    profilePyro[i].second != rocketPyro[i].second ||
                    !same(profilePyro[i].third, rocketPyro[i].third, 1)
                ) {
                    changed += "Pyro ${i + 1}"
                }
            }
            p = p.copy(
                pyro1Enabled = rocketPyro[0].first,
                pyro1TriggerMode = rocketPyro[0].second,
                pyro1TriggerValue = rocketPyro[0].third,
                pyro2Enabled = rocketPyro[1].first,
                pyro2TriggerMode = rocketPyro[1].second,
                pyro2TriggerValue = rocketPyro[1].third,
                pyro3Enabled = rocketPyro[2].first,
                pyro3TriggerMode = rocketPyro[2].second,
                pyro3TriggerValue = rocketPyro[2].third,
                pyro4Enabled = rocketPyro[3].first,
                pyro4TriggerMode = rocketPyro[3].second,
                pyro4TriggerValue = rocketPyro[3].third,
            )

            // ── The #915 firmware follow-up groups ─────────────────────────
            // Each is null on a rocket that cannot report it (pre-report
            // firmware, or the mini, which has none of this hardware).  null
            // means "we don't know", and the only honest thing to do with that
            // is leave the profile alone — resetting it to an invented default
            // would be a silent change dressed up as a reconciliation.

            cfg.servoExtras?.let { e ->
                if (p.servoBias2 != e.bias2 || p.servoBias3 != e.bias3 ||
                    p.servoBias4 != e.bias4
                ) {
                    p = p.copy(servoBias2 = e.bias2, servoBias3 = e.bias3, servoBias4 = e.bias4)
                    changed += GROUP_SERVO_TRIM_24
                }

                // #449 made the profile store ONE travel number, from which the
                // endpoints are derived symmetrically.  An asymmetric
                // rocket-side cal therefore collapses to its span — the profile
                // has no way to express it.
                val travel = e.finMaxDeg - e.finMinDeg
                if (!same(p.finTravelDeg, travel, 2)) {
                    p = p.copy(finTravelDeg = travel)
                    changed += GROUP_FIN_TRAVEL
                }

                val slots = slotsFromAzimuths(e.finAzimuths)
                val reverse = (0..3).map { (e.finReverseMask shr it) and 1 == 1 }
                val rollReverse = (0..3).map { (e.finRollReverseMask shr it) and 1 == 1 }
                val ringMode = ringModeFromAzimuths(e.finAzimuths)
                if (slots != null && (
                        p.finServoAtSlot != slots || p.finReverse != reverse ||
                            p.finRollReverse != rollReverse || p.finRingMode != ringMode
                        )
                ) {
                    p = p.copy(
                        finServoAtSlot = slots, finReverse = reverse,
                        finRollReverse = rollReverse, finRingMode = ringMode,
                    )
                    changed += GROUP_FIN_LAYOUT
                }

                if (p.soundsEnabled != e.soundsEnabled) {
                    p = p.copy(soundsEnabled = e.soundsEnabled)
                    changed += GROUP_SOUNDS
                }
            }

            cfg.guidanceExtras?.let { g ->
                if (!same(p.pnNavGain, g.navGain, 2) ||
                    !same(p.pnMaxAccel, g.maxAccel, 1) ||
                    !same(p.pnAccelToFin, g.accelToFin, 2) ||
                    !same(p.pnMaxFinDeg, g.maxFinDeg, 1) ||
                    !same(p.pnMinSpeed, g.minSpeed, 1) ||
                    p.pnCoastDelayMs != g.coastDelayMs ||
                    p.pnTargetMode != g.targetMode ||
                    !same(p.pnTargetE, g.targetE, 1) ||
                    !same(p.pnTargetN, g.targetN, 1) ||
                    !same(p.pnTargetAltM, g.targetAltM, 1) ||
                    !same(p.pnKpPos, g.kpPos, 2) ||
                    !same(p.pnKdVel, g.kdVel, 2) ||
                    p.pnGuidanceLaw != g.guidanceLaw
                ) {
                    p = p.copy(
                        pnNavGain = g.navGain, pnMaxAccel = g.maxAccel,
                        pnAccelToFin = g.accelToFin, pnMaxFinDeg = g.maxFinDeg,
                        pnMinSpeed = g.minSpeed, pnCoastDelayMs = g.coastDelayMs,
                        pnTargetMode = g.targetMode, pnTargetE = g.targetE,
                        pnTargetN = g.targetN, pnTargetAltM = g.targetAltM,
                        pnKpPos = g.kpPos, pnKdVel = g.kdVel,
                        pnGuidanceLaw = g.guidanceLaw,
                    )
                    changed += GROUP_GUIDANCE_PARAMS
                }
            }

            cfg.rollWaypoints?.let { wps ->
                val differs = wps.size != p.rollWaypoints.size ||
                    wps.zip(p.rollWaypoints).any { (r, mine) ->
                        !same(r.timeS, mine.timeSeconds, 2) ||
                            !same(r.angleDeg, mine.angleDeg, 1)
                    }
                if (differs) {
                    p = p.copy(
                        rollWaypoints = wps.map {
                            ProfileRollWaypoint(timeSeconds = it.timeS, angleDeg = it.angleDeg)
                        },
                    )
                    changed += GROUP_ROLL_PROFILE
                }
            }

            return AdoptionResult(p, changed)
        }

        /**
         * Ring mode from the reported azimuths: "+" is the on-axis set
         * {0,90,180,270}, "×" the 45°-rotated one.  Anything else reads as "+"
         * — the profile has no third mode, and the azimuths still round-trip
         * through finServoAtSlot.
         */
        public fun ringModeFromAzimuths(az: List<Float>): Int {
            val rotated = az.any { a ->
                val m = ((a % 90) + 90) % 90
                kotlin.math.abs(m - 45f) < 1f
            }
            return if (rotated) 1 else 0
        }

        /**
         * Invert the app's slot→servo mapping from the azimuths the FC reports.
         * Returns null if they don't describe four distinct slots, in which
         * case the layout is left alone rather than guessed at — putting the
         * wrong servo on the wrong fin is worse than not knowing.
         */
        public fun slotsFromAzimuths(az: List<Float>): List<Int>? {
            if (az.size != 4) return null
            val slots = IntArray(4)
            val filled = mutableSetOf<Int>()
            az.forEachIndexed { servoIdx, a ->
                val norm = ((a % 360f) + 360f) % 360f
                // FLOOR, not round-to-nearest: a slot IS a quadrant of the
                // ring, and the "x" layout's azimuths (45/135/225/315) land
                // exactly on rounding ties — where Kotlin breaks to even and
                // Swift breaks away from zero, so the two platforms derived
                // DIFFERENT fin mappings from the same rocket.  Flooring has
                // no tie to break and gives 0..3 for both ring modes.
                val slot = kotlin.math.floor(norm / 90f).toInt().coerceIn(0, 3)
                if (!filled.add(slot)) return null      // two servos, one slot
                slots[slot] = servoIdx + 1              // servos are 1-based
            }
            return if (filled.size == 4) slots.toList() else null
        }

        /**
         * Equal once both sides are rounded to the decimals the value is
         * serialised at in the readback JSON.
         */
        public fun same(a: Float, b: Float, decimals: Int): Boolean {
            var scale = 1.0
            repeat(decimals) { scale *= 10.0 }
            return Math.round(a * scale) == Math.round(b * scale)
        }

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
