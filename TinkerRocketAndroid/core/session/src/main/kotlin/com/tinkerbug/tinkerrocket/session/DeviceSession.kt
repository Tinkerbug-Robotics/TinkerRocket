package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.BaseStationStorageStats
import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.protocol.ConfigIdentityMsg
import com.tinkerbug.tinkerrocket.protocol.FileInfo
import com.tinkerbug.tinkerrocket.protocol.PyroContinuity
import com.tinkerbug.tinkerrocket.protocol.pyroContinuityOf
import com.tinkerbug.tinkerrocket.protocol.FileOpsDispatch
import com.tinkerbug.tinkerrocket.protocol.FileOpsMessage
import com.tinkerbug.tinkerrocket.protocol.FrequencyScanSample
import com.tinkerbug.tinkerrocket.protocol.GuidanceSendFlow
import com.tinkerbug.tinkerrocket.protocol.GuidanceTargetEcho
import com.tinkerbug.tinkerrocket.protocol.IMUOrientationMode
import com.tinkerbug.tinkerrocket.protocol.ImuOrientMsg
import com.tinkerbug.tinkerrocket.protocol.MagCalStatus
import com.tinkerbug.tinkerrocket.protocol.OtaStatusUpdate
import com.tinkerbug.tinkerrocket.protocol.PyroChannelConfig
import com.tinkerbug.tinkerrocket.protocol.RocketConfig
import com.tinkerbug.tinkerrocket.protocol.RocketStorageStats
import com.tinkerbug.tinkerrocket.protocol.SensorCalStatus
import com.tinkerbug.tinkerrocket.protocol.TelemetryCharMessage
import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import com.tinkerbug.tinkerrocket.protocol.TelemetryDispatch
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.TimeoutCancellationException
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.onSubscription
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeout
import java.io.ByteArrayOutputStream
import java.time.Instant
import java.time.ZoneOffset

/**
 * Per-connection device session — the Kotlin port of the iOS `BLEDevice`
 * per-peripheral lifecycle (BLEDevice.swift is the behavioral reference),
 * running over the abstract [BleTransport] seam.
 *
 * Concurrency contract (android-port plan §3, single-writer):
 *  - ALL state mutation happens on [scope]'s dispatcher.  Public mutator
 *    methods launch into [scope]; suspend methods ([downloadFile],
 *    [sendGuidancePointConfirming]) must be called from a coroutine on the
 *    SAME dispatcher.
 *  - All timing is `delay()`-driven and the clock is injected, so
 *    kotlinx-coroutines-test virtual time drives everything.
 *
 * iOS ↔ Kotlin lifecycle mapping:
 *  - CoreBluetooth's service/characteristic discovery collapses into the
 *    transport seam; the ORDER of the connect choreography is preserved
 *    exactly (see [start]).
 *  - The Timer-based behaviors (RSSI 2 s poll, download 3 s stall, #159
 *    power-on watchdog, #385 scan-result wedge timeout) become `delay()`
 *    jobs with the same durations.
 */
public class DeviceSession(
    private val scope: CoroutineScope,
    private val transport: BleTransport,
    /** BLE advertised name at connect time (iOS `connectedDeviceName`). */
    public val connectedDeviceName: String = "",
    /**
     * Device-type seed (fleet registry / name-prefix parse — iOS BLEDevice.init
     * + BLEFleet.seedTypeFromRegistry).  The config_identity "dt" readback
     * overrides it ~1 s after connect.
     */
    initialDeviceType: BleDeviceType = BleDeviceType.fromName(connectedDeviceName),
    /** Injected clock (epoch millis) — never read the wall clock directly. */
    private val clock: () -> Long = System::currentTimeMillis,
    /**
     * Optional registry to fold config_identity readbacks into (iOS:
     * `fleet?.knownDevices.deviceDidReportIdentity`); this session acts as the
     * [DeviceIdentityPusher] for queued offline edits.
     */
    private val knownDevices: KnownDeviceStore? = null,
    /**
     * #390 sticky-focus escalation hook (iOS `fleet?.noteAutoFocus`): invoked
     * once when a base-station link auto-latches onto the first relayed
     * rocket heard this session.
     */
    private val onAutoFocus: ((Int) -> Unit)? = null,
    /**
     * #140 per-packet fix recording (iOS `fleet?.recordRocketFix`): called
     * synchronously for EVERY relayed rocket packet and for direct-rocket
     * packets, so the fleet can latch the last VALID GPS fix per (nid, rid).
     * Returns the latched fix (null when the packet carried no valid fix and
     * nothing was latched before).  Wire to [FleetManager.recordRocketFix].
     */
    private val onRocketFix: ((TelemetryData, RocketKey) -> LastValidRocketFix?)? = null,
    /**
     * #140 re-latch/hydrate lookup (iOS `fleet?.lastValidRocketFix(for:)`):
     * consulted on focus switches and when identity becomes known, so a
     * reconnected session shows the last known position immediately instead
     * of blanking until the next valid fix.
     */
    private val fixLookup: ((RocketKey) -> LastValidRocketFix?)? = null,
) : DeviceIdentityPusher {

    // ── Connection / link state ──────────────────────────────────────────

    /**
     * Voice-callout sink (iOS `BLEDevice.flightAnnouncer`, #138). Dispatch is
     * DIRECT, not via the [telemetry] StateFlow: a StateFlow dedups equal
     * frames, and burnout detection counts *consecutive unchanged* max-speed
     * frames — conflation would eat exactly the frames it counts. Relay path
     * feeds only the FOCUSED rocket (#390 — two interleaved flights' callouts
     * are noise). The app attaches/detaches this on the fleet dispatcher.
     */
    public var telemetryAnnouncer: TelemetryAnnouncer? = null

    private val _isConnected = MutableStateFlow(false)
    public val isConnected: StateFlow<Boolean> = _isConnected.asStateFlow()

    /**
     * ATT MTU currently negotiated on this link.  The OTA pump sizes its
     * chunks from it; 23 is the BLE default before negotiation lands.
     */
    private val _negotiatedMtu = MutableStateFlow(23)
    public val negotiatedMtu: StateFlow<Int> = _negotiatedMtu.asStateFlow()

    private val _connectedRssi = MutableStateFlow<Int?>(null)
    public val connectedRssi: StateFlow<Int?> = _connectedRssi.asStateFlow()

    // ── Telemetry ────────────────────────────────────────────────────────

    private val _telemetry = MutableStateFlow(TelemetryData())
    public val telemetry: StateFlow<TelemetryData> = _telemetry.asStateFlow()

    /**
     * #377: false until the FIRST decoded telemetry frame of this session.
     * Until then the power state is UNKNOWN (a default frame reads as
     * pwr_pin_on == false) and the UI must not offer the blind cmd-8 toggle.
     */
    private val _hasReceivedTelemetry = MutableStateFlow(false)
    public val hasReceivedTelemetry: StateFlow<Boolean> = _hasReceivedTelemetry.asStateFlow()

    /** When the last telemetry frame (any kind) was decoded on this link. */
    public var lastTelemetryAtMs: Long? = null
        private set

    /**
     * #140: the latched last-valid GPS fix for THIS session's rocket (direct
     * link) or its focused relayed rocket (BS link).  Consumers read this,
     * never per-frame telemetry lat/lon — after landing, frames stop carrying
     * fresh GPS and a per-frame read blanks the map marker.  Survives
     * disconnect by design (it is the fleet cache's mirror).
     */
    private val _lastValidRocketFix = MutableStateFlow<LastValidRocketFix?>(null)
    public val lastValidRocketFix: StateFlow<LastValidRocketFix?> = _lastValidRocketFix.asStateFlow()

    // ── Identity ─────────────────────────────────────────────────────────

    private val _identity = MutableStateFlow(DeviceIdentity(deviceType = initialDeviceType))
    public val identity: StateFlow<DeviceIdentity> = _identity.asStateFlow()

    public val isBaseStation: Boolean
        get() = _identity.value.deviceType == BleDeviceType.BASE_STATION

    /** Display name: unitName if set, otherwise the advertised name. */
    public val displayName: String
        get() = _identity.value.unitName.ifEmpty { connectedDeviceName }

    // ── Config ───────────────────────────────────────────────────────────

    private val _rocketConfig = MutableStateFlow<RocketConfig?>(null)
    public val rocketConfig: StateFlow<RocketConfig?> = _rocketConfig.asStateFlow()

    private val _imuOrientationName = MutableStateFlow("")
    public val imuOrientationName: StateFlow<String> = _imuOrientationName.asStateFlow()

    private val _imuOrientationMode = MutableStateFlow(IMUOrientationMode.UNKNOWN)
    public val imuOrientationMode: StateFlow<IMUOrientationMode> = _imuOrientationMode.asStateFlow()

    /**
     * iOS `contTestPendingUntil` twin: per-channel deadline (session-clock
     * millis) while a manual continuity test round-trips BLE→OC→I2C→FC and
     * back through telemetry "ps" bits (#411).  Displays show TESTING
     * instead of the cached reading until the deadline — the pre-test value
     * is exactly the number the user asked to refresh.  Expired entries stay
     * in the map (same as iOS); readers compare against the clock.
     */
    private val _contTestPendingUntil = MutableStateFlow<Map<Int, Long>>(emptyMap())
    public val contTestPendingUntil: StateFlow<Map<Int, Long>> = _contTestPendingUntil.asStateFlow()

    // ── Files / downloads ────────────────────────────────────────────────

    private val _files = MutableStateFlow<List<FileInfo>>(emptyList())
    public val files: StateFlow<List<FileInfo>> = _files.asStateFlow()

    private val _currentPage = MutableStateFlow(0)
    public val currentPage: StateFlow<Int> = _currentPage.asStateFlow()

    private val _hasMoreFiles = MutableStateFlow(false)
    public val hasMoreFiles: StateFlow<Boolean> = _hasMoreFiles.asStateFlow()

    private val _downloadState = MutableStateFlow(DownloadProgress())
    public val downloadState: StateFlow<DownloadProgress> = _downloadState.asStateFlow()

    // ── Cal / storage / OTA frames (file_ops characteristic) ─────────────

    /**
     * EVERY mag-cal frame, in order — the 5 Hz SAMPLING heartbeat drives
     * progress UI, so frames must never be conflated away (hence a
     * SharedFlow, not a StateFlow).  [magCalStatus] carries the latest.
     */
    private val _magCalFrames = MutableSharedFlow<MagCalStatus>(extraBufferCapacity = 256)
    public val magCalFrames: SharedFlow<MagCalStatus> = _magCalFrames.asSharedFlow()

    private val _magCalStatus = MutableStateFlow<MagCalStatus?>(null)
    public val magCalStatus: StateFlow<MagCalStatus?> = _magCalStatus.asStateFlow()

    private val _sensorCalStatus = MutableStateFlow<SensorCalStatus?>(null)
    public val sensorCalStatus: StateFlow<SensorCalStatus?> = _sensorCalStatus.asStateFlow()

    private val _rocketStorage = MutableStateFlow<RocketStorageStats?>(null)
    public val rocketStorage: StateFlow<RocketStorageStats?> = _rocketStorage.asStateFlow()

    private val _bsStorage = MutableStateFlow<BaseStationStorageStats?>(null)
    public val bsStorage: StateFlow<BaseStationStorageStats?> = _bsStorage.asStateFlow()

    private val _otaStatus = MutableStateFlow<OtaStatusUpdate?>(null)
    public val otaStatus: StateFlow<OtaStatusUpdate?> = _otaStatus.asStateFlow()

    // ── Frequency scan ───────────────────────────────────────────────────

    private val _scanSamples = MutableStateFlow<List<FrequencyScanSample>>(emptyList())
    public val scanSamples: StateFlow<List<FrequencyScanSample>> = _scanSamples.asStateFlow()

    private val _isScanning = MutableStateFlow(false)
    public val isScanning: StateFlow<Boolean> = _isScanning.asStateFlow()

    /** #385: monotonically identifies the latest scan request so the wedge
     *  timeout can't clear a newer scan's spinner. */
    private var scanGeneration = 0

    // ── Guidance echo (#435) ─────────────────────────────────────────────

    private val _guidanceEcho = MutableStateFlow<GuidanceTargetEcho?>(null)
    public val guidanceEcho: StateFlow<GuidanceTargetEcho?> = _guidanceEcho.asStateFlow()

    /** Per-frame echo stream feeding [sendGuidancePointConfirming]. */
    private val _echoFrames = MutableSharedFlow<GuidanceTargetEcho>(extraBufferCapacity = 64)

    // ── Sim banner latch ─────────────────────────────────────────────────

    private val _simLaunched = MutableStateFlow(false)
    public val simLaunched: StateFlow<Boolean> = _simLaunched.asStateFlow()
    private var simSawNonReady = false

    // ── Power-on watchdog (#159) ─────────────────────────────────────────

    private val _poweringOn = MutableStateFlow(false)
    public val poweringOn: StateFlow<Boolean> = _poweringOn.asStateFlow()
    private var poweringOnJob: Job? = null

    // ── Relayed rockets + focus (#390) ───────────────────────────────────

    private val _focusRocketId = MutableStateFlow<Int?>(null)
    public val focusRocketId: StateFlow<Int?> = _focusRocketId.asStateFlow()

    private val remoteMap = LinkedHashMap<Int, RelayedRocket>()
    private val _remoteRockets = MutableStateFlow<List<RelayedRocket>>(emptyList())
    public val remoteRockets: StateFlow<List<RelayedRocket>> = _remoteRockets.asStateFlow()

    /**
     * #390: app-computed age of the FOCUSED rocket's relayed stream,
     * refreshed on the 2 s RSSI tick (NOT on frame arrival — mirrors iOS,
     * where only the tick and an explicit focus switch recompute it).
     */
    private val _focusedRelayAgeMs = MutableStateFlow<Long?>(null)
    public val focusedRelayAgeMs: StateFlow<Long?> = _focusedRelayAgeMs.asStateFlow()

    /**
     * Freshness the dashboard should trust for this link's rocket stream:
     * the frame-carried status, worsened by the app-computed focused-rocket
     * age when that is staler — never improved (a frame-carried STALE stays,
     * and SYNCING is never overridden).  iOS `effectiveDataStatus`.
     */
    private val _effectiveDataStatus = MutableStateFlow(TelemetryData.DataStatus.LIVE)
    public val effectiveDataStatus: StateFlow<TelemetryData.DataStatus> =
        _effectiveDataStatus.asStateFlow()

    private val _effectiveDataAgeMs = MutableStateFlow(0L)
    public val effectiveDataAgeMs: StateFlow<Long> = _effectiveDataAgeMs.asStateFlow()

    // ── Internal jobs / download state ───────────────────────────────────

    private var started = false
    private var rssiJob: Job? = null
    private var downloadStallJob: Job? = null
    private var activeDownload: ActiveDownload? = null

    private class ActiveDownload(
        val filename: String,
        val expectedSize: Int,
        val result: CompletableDeferred<DownloadResult>,
    ) {
        val buffer = ByteArrayOutputStream()
    }

    // ─────────────────────────────────────────────────────────────────────
    // Lifecycle
    // ─────────────────────────────────────────────────────────────────────

    /**
     * Drive the connect choreography.  The op ORDER is the iOS contract:
     *
     *   connect → MTU 517 → CCCD telemetry → **time sync (cmd 9) the instant
     *   the command char is usable — BEFORE the file_ops / file_transfer
     *   CCCDs** → CCCD file_ops → CCCD file_transfer → RSSI ticker →
     *   1.0 s delay → requestConfig (cmd 20) → cmd 45 focus re-pin when this
     *   is a base station with a seeded focus pin (#390: the BS keeps the pin
     *   in RAM only, so every (re)connect must re-assert it).
     *
     * (iOS: didDiscoverCharacteristics loop + the 1.0 s asyncAfter block.)
     */
    public fun start(): Job {
        check(!started) { "DeviceSession.start() may only be called once" }
        started = true
        // The TRANSPORT IS ALREADY CONNECTED by contract: the fleet owns
        // connect() (iOS parity — CBCentralManager connects, BLEDevice never
        // does).  Double-connecting here raced/violated the transport
        // contract (Phase 2 review finding).  Standalone users (replay
        // feeds, tests) must connect the transport before start().
        // Subscribe first so no event can be dropped.
        scope.launch { transport.events.collect { onTransportEvent(it) } }
        return scope.launch {
            _isConnected.value = true
            transport.requestMtu(REQUESTED_MTU)
            transport.enableNotifications(TrCharacteristic.TELEMETRY)
            // Time sync FIRST — the firmware stamps log/file timestamps from
            // it, so it must land before anything else can start file I/O.
            sendTimeSyncNow()
            transport.enableNotifications(TrCharacteristic.FILE_OPS)
            transport.enableNotifications(TrCharacteristic.FILE_TRANSFER)
            startRssiTicker()
            delay(CONNECT_CONFIG_DELAY_MS)
            writeCommand(Commands.bare(BleCommandId.REQUEST_CONFIG))
            val focus = _focusRocketId.value
            if (isBaseStation && focus != null) {
                writeCommand(Commands.setFocusRocket(focus))
            }
        }
    }

    /**
     * Wipe exactly what iOS `onDisconnect()` wipes.  NOT wiped (also exactly
     * like iOS): telemetry mirror, identity, remote-rocket roster, scan
     * results, storage stats, otaStatus, IMU orientation, focus pin.
     */
    private fun onDisconnect() {
        _isConnected.value = false
        rssiJob?.cancel()
        rssiJob = null
        _connectedRssi.value = null
        _files.value = emptyList()
        _currentPage.value = 0
        _hasMoreFiles.value = false
        clearSimBannerNow()
        clearPoweringOnNow()
        _rocketConfig.value = null
        // iOS equivalent is implicit: BLEDevice is recreated per connection,
        // taking its contTestPendingUntil map with it.
        _contTestPendingUntil.value = emptyMap()
        // Stale REVIEW must not bleed across sessions (#96); the FC
        // republishes IDLE on reconnect anyway.
        _magCalStatus.value = null
        _sensorCalStatus.value = null
        // #435: per-connection live state — a stale echo could "confirm" a
        // send made over a new connection to a rebooted FC.
        _guidanceEcho.value = null
        // Power state unknown again until the next session's first frame (#377).
        _hasReceivedTelemetry.value = false
        // Stop mid-utterance speech + clear one-shot flags (iOS onDisconnect
        // does the same).  The previous-frame snapshot survives inside the
        // announcer, so edges that already fired don't replay on reconnect.
        telemetryAnnouncer?.reset()
        // Fail any in-flight download (the transport can't deliver more
        // chunks; the suspended caller must not hang forever).
        downloadStallJob?.cancel()
        downloadStallJob = null
        activeDownload?.let { d ->
            activeDownload = null
            _downloadState.value = DownloadProgress()
            d.result.complete(DownloadResult.Disconnected)
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Transport event routing
    // ─────────────────────────────────────────────────────────────────────

    private fun onTransportEvent(event: TransportEvent) {
        when (event) {
            is TransportEvent.Connected -> {
                _isConnected.value = true
                if (event.mtu > 0) _negotiatedMtu.value = event.mtu
            }
            is TransportEvent.MtuChanged -> if (event.mtu > 0) _negotiatedMtu.value = event.mtu
            is TransportEvent.Disconnected -> onDisconnect()
            is TransportEvent.Rssi -> _connectedRssi.value = event.dbm
            is TransportEvent.WriteCompleted -> Unit
            is TransportEvent.Notification -> when (event.char) {
                TrCharacteristic.TELEMETRY -> onTelemetryFrame(event.bytes)
                TrCharacteristic.FILE_OPS -> onFileOpsFrame(event.bytes)
                TrCharacteristic.FILE_TRANSFER -> onFileChunk(event.bytes)
                TrCharacteristic.COMMAND -> Unit
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Telemetry characteristic (JSON demux ladder)
    // ─────────────────────────────────────────────────────────────────────

    private fun onTelemetryFrame(bytes: ByteArray) {
        when (val msg = TelemetryDispatch.parse(bytes)) {
            is TelemetryCharMessage.Config -> {
                _rocketConfig.value = msg.msg.applyTo(_rocketConfig.value)
                // iOS calls triggerAutoChannelSelectIfNeeded() here — a
                // deliberate no-op since #136; not ported.
            }
            is TelemetryCharMessage.ConfigPyro ->
                _rocketConfig.value = msg.msg.applyTo(_rocketConfig.value)
            // #915: a malformed frame leaves the group as it was — "not
            // reported" — rather than applying half of it as verified.
            is TelemetryCharMessage.ConfigServo -> msg.extras?.let { e ->
                _rocketConfig.value = (_rocketConfig.value ?: RocketConfig())
                    .copy(servoExtras = e)
            }
            is TelemetryCharMessage.ConfigGuid -> msg.extras?.let { e ->
                _rocketConfig.value = (_rocketConfig.value ?: RocketConfig())
                    .copy(guidanceExtras = e)
            }
            is TelemetryCharMessage.ConfigRoll -> msg.waypoints?.let { w ->
                _rocketConfig.value = (_rocketConfig.value ?: RocketConfig())
                    .copy(rollWaypoints = w)
            }
            is TelemetryCharMessage.ConfigIdentity -> onConfigIdentity(msg.msg)
            is TelemetryCharMessage.FcIdentity -> {
                msg.msg.fcFirmwareVersion?.let {
                    _identity.value = _identity.value.copy(fcFirmwareVersion = it)
                }
            }
            is TelemetryCharMessage.ImuOrient -> onImuOrient(msg.msg)
            is TelemetryCharMessage.GuidTarget -> {
                _guidanceEcho.value = msg.echo
                _echoFrames.tryEmit(msg.echo)
            }
            is TelemetryCharMessage.Telemetry -> onTelemetry(msg.data)
            is TelemetryCharMessage.Unknown -> Unit   // iOS logs and discards
        }
    }

    private fun onTelemetry(data: TelemetryData) {
        // #377: the flags are now confirmed — flips exactly once per session.
        if (!_hasReceivedTelemetry.value) _hasReceivedTelemetry.value = true
        lastTelemetryAtMs = clock()

        // Relayed via base station → route to the roster; only the FOCUSED
        // rocket mirrors into this session's own telemetry (#390).
        val rid = data.sourceRocketId
        if (rid != null && rid > 0 && isBaseStation) {
            val now = clock()
            val existing = remoteMap[rid]
            remoteMap[rid] = if (existing != null) {
                existing.copy(
                    telemetry = data,
                    // iOS RemoteRocket.updateTelemetry guards !name.isEmpty —
                    // an empty relayed name never clobbers a learned one.
                    unitName = data.sourceUnitName?.takeIf { it.isNotEmpty() }
                        ?: existing.unitName,
                    lastSeenMs = now,
                )
            } else {
                RelayedRocket(
                    rocketId = rid,
                    unitName = data.sourceUnitName ?: "",
                    telemetry = data,
                    lastSeenMs = now,
                )
            }
            _remoteRockets.value = remoteMap.values.toList()

            // #140: record the fix for EVERY relayed rocket, keyed by the
            // BS's OWN network id (it only forwards its own network) —
            // rocket ids are only unique per network (#390).
            val fix = onRocketFix?.invoke(
                data, RocketKey(_identity.value.networkId ?: 0, rid))

            // #390: sticky first-heard focus — never moves on its own after
            // this; the user (or the fleet re-seed) switches it.
            if (_focusRocketId.value == null) {
                _focusRocketId.value = rid
                onAutoFocus?.invoke(rid)
            }
            if (rid == _focusRocketId.value) {
                _telemetry.value = data
                // Mirror the latched fix only when non-null — a GPS-less
                // packet must not blank the marker (#140).
                if (fix != null) _lastValidRocketFix.value = fix
                // The relayed JSON carries the full rocket state. Without
                // this, voice callouts only fire when paired directly to the
                // rocket, never during a real flight (#138). Focused rocket
                // only (#390).
                telemetryAnnouncer?.processTelemetry(data)
            }
            recomputeEffective()
            return
        }

        // Sim-banner latch: launched → wait for a non-READY state, then the
        // return to READY clears the banner (iOS markSimLaunched flow).
        if (_simLaunched.value) {
            if (data.state != "READY" && data.state != "INITIALIZATION") {
                simSawNonReady = true
            }
            if (simSawNonReady && data.state == "READY") {
                _simLaunched.value = false
                simSawNonReady = false
            }
        }
        _telemetry.value = data
        // #159: the rocket finished flushing and powered on — drop the busy
        // state (guarded inside clearPoweringOn via StateFlow dedup).
        if (data.pwrPinOn && _poweringOn.value) clearPoweringOnNow()
        // #140 direct path: record + mirror using this device's own identity.
        // Skip when rocketId is unset so a BS-self packet (no source rocket
        // id, falls through to here) can't blank a relay-mirrored fix.
        val id = _identity.value
        val ownRid = id.rocketId
        if (ownRid != null && ownRid > 0) {
            onRocketFix?.invoke(data, RocketKey(id.networkId ?: 0, ownRid))
                ?.let { _lastValidRocketFix.value = it }
        }
        // Direct-rocket and BS-self frames both dispatch to voice from here
        // (iOS parseTelemetryData's fall-through path).
        telemetryAnnouncer?.processTelemetry(data)
        recomputeEffective()
    }

    private fun onConfigIdentity(msg: ConfigIdentityMsg) {
        val cur = _identity.value
        val next = cur.copy(
            unitId = msg.unitId ?: cur.unitId,
            unitName = msg.unitName ?: cur.unitName,
            networkId = msg.networkId ?: cur.networkId,
            rocketId = msg.rocketId ?: cur.rocketId,
            // iOS `BLEDeviceType(rawValue: dt) ?? deviceType`: "?" is a VALID
            // raw value and sets UNKNOWN; only unrecognized strings keep the
            // previous type.
            deviceType = msg.deviceType?.let { BleDeviceType.fromRaw(it) } ?: cur.deviceType,
            firmwareVersion = msg.firmwareVersion ?: cur.firmwareVersion,
        )
        _identity.value = next
        recomputeEffective()
        // Fold into the registry + push queued offline edits (iOS:
        // fleet?.knownDevices.deviceDidReportIdentity; empty-uid guard is in
        // the store).
        knownDevices?.deviceDidReportIdentity(
            unitID = next.unitId,
            name = next.unitName,
            deviceType = next.deviceType,
            networkID = next.networkId,
            rocketID = next.rocketId,
            pusher = this,
        )
    }

    private fun onImuOrient(msg: ImuOrientMsg) {
        msg.name?.let { _imuOrientationName.value = it }
        msg.mode?.let { _imuOrientationMode.value = it }
        // iOS caches "set" into rocketConfig ONLY when a config already
        // exists (`if let set..., var cfg = rocketConfig`).
        val setting = msg.setting
        val cfg = _rocketConfig.value
        if (setting != null && cfg != null) {
            _rocketConfig.value = cfg.copy(imuOrientSetting = setting)
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // File-ops characteristic (binary discriminator demux)
    // ─────────────────────────────────────────────────────────────────────

    private fun onFileOpsFrame(bytes: ByteArray) {
        when (val msg = FileOpsDispatch.parse(bytes)) {
            is FileOpsMessage.Scan -> {
                _scanSamples.value = msg.result.samples
                _isScanning.value = false
                // (iOS pendingAutoApply flow is dead since #136 — not ported.)
            }
            FileOpsMessage.ScanFailed -> _isScanning.value = false   // #385
            is FileOpsMessage.MagCal -> {
                _magCalStatus.value = msg.status
                _magCalFrames.tryEmit(msg.status)
            }
            is FileOpsMessage.SensorCal -> _sensorCalStatus.value = msg.status
            is FileOpsMessage.RocketStorage -> _rocketStorage.value = msg.stats
            is FileOpsMessage.BsStorage -> _bsStorage.value = msg.stats
            is FileOpsMessage.PyroRefusal -> {
                // Rail-off refusal: no reading is coming back, so close the
                // TESTING window instead of letting the spinner run out its
                // 2.5 s implying a round trip happened.  (No test-fire UI on
                // Android yet — cmd-36 refusals have nothing to clear.)
                if (msg.cmd == 35) {
                    _contTestPendingUntil.value = _contTestPendingUntil.value - msg.channel
                }
            }
            is FileOpsMessage.Ota -> _otaStatus.value = msg.status
            is FileOpsMessage.FileList -> {
                _files.value = msg.page.files
                _hasMoreFiles.value = msg.page.hasMore
            }
            null -> Unit   // malformed non-scan frame: silently dropped
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // File list / delete
    // ─────────────────────────────────────────────────────────────────────

    /**
     * iOS requestFileList: write `[2, page]`, then 0.5 s later EXPLICITLY
     * READ the file_ops characteristic (the list rides a read, not a
     * notification, on this path) and parse it through the same demux.
     */
    public fun requestFileList(page: Int = 0) {
        scope.launch {
            if (!writeCommand(Commands.fileList(page))) return@launch
            _currentPage.value = page
            launch {
                delay(FILE_LIST_READ_DELAY_MS)
                val bytes = runCatching { transport.read(TrCharacteristic.FILE_OPS) }
                    .getOrNull() ?: return@launch
                onFileOpsFrame(bytes)
            }
        }
    }

    /** cmd 3 + optimistic local removal (iOS deleteFile). */
    public fun deleteFile(filename: String) {
        scope.launch {
            if (!writeCommand(Commands.fileDelete(filename))) return@launch
            _files.value = _files.value.filterNot { it.name == filename }
        }
    }

    /**
     * Bulk delete (#634) — the Kotlin twin of iOS `deleteFiles`, which is
     * likewise a loop rather than a bulk wire command; the firmware has no
     * multi-delete opcode.
     *
     * Issued from ONE coroutine so the writes go out in the caller's order and
     * the op queue serialises them, instead of N racing coroutines landing in
     * arbitrary order.  A failed write stops the run rather than pressing on:
     * the usual cause is the link dropping, and continuing would optimistically
     * strip rows for files still on the board.
     */
    public fun deleteFiles(filenames: List<String>) {
        if (filenames.isEmpty()) return
        scope.launch {
            for (name in filenames) {
                if (!writeCommand(Commands.fileDelete(name))) return@launch
                _files.value = _files.value.filterNot { it.name == name }
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Download engine
    // ─────────────────────────────────────────────────────────────────────

    /**
     * Download a file over the file_transfer characteristic and return the
     * assembled bytes (or a typed failure — iOS returns nil for all of
     * them).  Chunk semantics are EXACTLY the iOS `handleFileChunk` engine:
     *
     *  - chunks are `[offset u32 LE][len u16 LE][flags u8][data]`.  The
     *    offset is CHECKED (#854, following iOS #832): on a data chunk it is
     *    the position that data belongs at, so a gap fails the download
     *    instead of splicing; on the EOF chunk it carries the device's own
     *    bytes_sent, which is checked against what arrived.  The shortfall
     *    check against the listing size now guards BOTH the EOF and stall
     *    paths — it used to guard only the stall path, so the common path
     *    returned Success with whatever had turned up;
     *  - `flags & 0x01` = EOF, `flags & 0x02` = ABORT.  EOF|ABORT (#526)
     *    fails the download outright — the bytes are a truncated fragment;
     *  - every non-EOF chunk re-arms a 3 s stall timer.  On stall: shortfall
     *    against a known expected size → [DownloadResult.Incomplete];
     *    otherwise the transfer completes with what arrived (legacy
     *    firmware sends no EOF at all — the stall IS its completion path;
     *    an unknown expected size also completes, matching iOS);
     *  - a duplicate EOF after completion is ignored (no active download);
     *  - NO timer runs before the first chunk (iOS parity): a device that
     *    never sends anything leaves the download pending until disconnect.
     *
     * Expected size comes from the current [files] list entry (>0), else 0.
     */
    public suspend fun downloadFile(filename: String): DownloadResult {
        if (!_isConnected.value) return DownloadResult.NotConnected
        if (activeDownload != null) return DownloadResult.Busy
        val expected = _files.value.firstOrNull { it.name == filename }
            ?.size?.takeIf { it > 0 }?.toInt() ?: 0
        val d = ActiveDownload(filename, expected, CompletableDeferred())
        activeDownload = d
        _downloadState.value = DownloadProgress(filename = filename, progress = 0.0, active = true)
        if (!writeCommand(Commands.fileDownload(filename))) {
            activeDownload = null
            _downloadState.value = DownloadProgress()
            return DownloadResult.NotConnected
        }
        return d.result.await()
    }

    private fun onFileChunk(data: ByteArray) {
        val d = activeDownload ?: return          // incl. duplicate-EOF tolerance
        if (data.size < CHUNK_HEADER_SIZE) return // malformed: ignored, timer untouched
        // #854/#832: bytes 0-3 are the offset, and they used to be ignored
        // entirely. On a data chunk the offset IS the position that data
        // belongs at, so contiguity is free; on the EOF chunk it carries the
        // device's own bytes_sent. Both were on the wire and thrown away.
        val offset = (data[0].toLong() and 0xFF) or
            ((data[1].toLong() and 0xFF) shl 8) or
            ((data[2].toLong() and 0xFF) shl 16) or
            ((data[3].toLong() and 0xFF) shl 24)
        val length = (data[4].toInt() and 0xFF) or ((data[5].toInt() and 0xFF) shl 8)
        val flags = data[6].toInt() and 0xFF
        val isEof = (flags and 0x01) != 0
        val isAbort = (flags and 0x02) != 0

        // #526: EOF|ABORT — the device could not finish; fail, save nothing.
        if (isEof && isAbort) {
            cancelStallTimer()
            finishDownload(d, DownloadResult.Aborted, finalProgress = null)
            return
        }

        if (length > 0) {
            if (data.size < CHUNK_HEADER_SIZE + length) {
                // A frame shorter than its own length header used to be
                // dropped in silence, leaving a hole indistinguishable from a
                // clean transfer.
                cancelStallTimer()
                finishDownload(d, DownloadResult.Incomplete, finalProgress = null)
                return
            }
            if (offset != d.buffer.size().toLong()) {
                // A notification dropped AFTER the peripheral queued it — the
                // case the firmware's redundant EOF exists for and cannot
                // itself detect. Appending past it splices the file silently.
                cancelStallTimer()
                finishDownload(d, DownloadResult.Incomplete, finalProgress = null)
                return
            }
            d.buffer.write(data, CHUNK_HEADER_SIZE, length)
        }
        val received = d.buffer.size()
        if (d.expectedSize > 0) {
            _downloadState.value = _downloadState.value.copy(
                progress = minOf(received.toDouble() / d.expectedSize, 1.0),
            )
        }
        if (isEof) {
            cancelStallTimer()
            // The EOF frame carries the device's bytes_sent; a mismatch means
            // we are missing bytes it believes it sent.
            // The EOF frame may carry DATA as well as the flag, so its offset
            // is that data's position, not the total (the base station's last
            // frame is offset=18190 len=132 for an 18322-byte file). A
            // zero-length EOF has offset == total; offset+length covers both.
            if (offset + length.toLong() != received.toLong()) {
                finishDownload(d, DownloadResult.Incomplete, finalProgress = null)
                return
            }
            _downloadState.value = _downloadState.value.copy(progress = 1.0)
            completeDownload(d, fromStallTimer = false)
        } else {
            resetStallTimer()
        }
    }

    private fun resetStallTimer() {
        downloadStallJob?.cancel()
        downloadStallJob = scope.launch {
            delay(DOWNLOAD_STALL_MS)
            val d = activeDownload ?: return@launch
            completeDownload(d, fromStallTimer = true)
        }
    }

    private fun cancelStallTimer() {
        downloadStallJob?.cancel()
        downloadStallJob = null
    }

    private fun completeDownload(d: ActiveDownload, fromStallTimer: Boolean) {
        cancelStallTimer()
        // #854/#832: this was gated on fromStallTimer, so the EOF path — the
        // common one — returned Success with whatever had arrived. The
        // listing's size is just as authoritative on either path.
        if (d.expectedSize > 0 && d.buffer.size() < d.expectedSize) {
            finishDownload(d, DownloadResult.Incomplete, finalProgress = null)
            return
        }
        finishDownload(d, DownloadResult.Success(d.buffer.toByteArray()), finalProgress = 1.0)
    }

    private fun finishDownload(d: ActiveDownload, result: DownloadResult, finalProgress: Double?) {
        activeDownload = null
        _downloadState.value = DownloadProgress(
            filename = null,
            progress = finalProgress ?: _downloadState.value.progress,
            active = false,
        )
        d.result.complete(result)
    }

    // ─────────────────────────────────────────────────────────────────────
    // RSSI tick + relayed-stream freshness
    // ─────────────────────────────────────────────────────────────────────

    private fun startRssiTicker() {
        rssiJob?.cancel()
        rssiJob = scope.launch {
            while (isActive) {
                delay(RSSI_POLL_MS)
                runCatching { transport.readRssi() }.getOrNull()?.let {
                    _connectedRssi.value = it
                }
                // #390: piggyback the focused-rocket staleness overlay on the
                // same tick so it advances even when no frames arrive.
                refreshFocusedRelayFreshness()
                recomputeEffective()
            }
        }
    }

    private fun refreshFocusedRelayFreshness(now: Long = clock()) {
        val focus = _focusRocketId.value
        val remote = if (isBaseStation && focus != null) remoteMap[focus] else null
        if (remote == null) {
            if (_focusedRelayAgeMs.value != null) _focusedRelayAgeMs.value = null
            return
        }
        val age = (now - remote.lastSeenMs).coerceIn(0L, 0xFFFF_FFFFL)   // UInt32 clamping
        if (_focusedRelayAgeMs.value != age) _focusedRelayAgeMs.value = age
    }

    private fun recomputeEffective() {
        val t = _telemetry.value
        val age = _focusedRelayAgeMs.value
        _effectiveDataStatus.value =
            if (isBaseStation && t.dataStatus != TelemetryData.DataStatus.SYNCING &&
                age != null && age > RELAY_STALE_THRESHOLD_MS
            ) {
                TelemetryData.DataStatus.STALE
            } else {
                t.dataStatus
            }
        _effectiveDataAgeMs.value =
            if (isBaseStation && age != null && age > t.dataAgeMs) age else t.dataAgeMs
    }

    // ─────────────────────────────────────────────────────────────────────
    // Focus (#390)
    // ─────────────────────────────────────────────────────────────────────

    /**
     * Fleet re-seed BEFORE [start] (iOS BLEFleet.adopt): restores the sticky
     * pin across the session recreation on reconnect, without sending cmd 45
     * (the connect choreography re-asserts it once the link is up).
     */
    public fun seedFocusRocket(rocketId: Int) {
        _focusRocketId.value = rocketId
    }

    /**
     * User-driven focus switch (iOS BLEFleet.setFocus tail that lives on the
     * device): re-pin the mirrored stream immediately from the roster cache,
     * refresh the staleness overlay, and push the pin (cmd 45) to the BS.
     */
    public fun setFocusRocket(rocketId: Int) {
        scope.launch {
            _focusRocketId.value = rocketId
            remoteMap[rocketId]?.let { _telemetry.value = it.telemetry }
            // #140 re-latch from the fleet cache (iOS BLEFleet.setFocus):
            // the newly focused rocket's last known position shows
            // immediately instead of blanking until its next valid fix.
            fixLookup?.invoke(RocketKey(_identity.value.networkId ?: 0, rocketId))
                ?.let { _lastValidRocketFix.value = it }
            refreshFocusedRelayFreshness()
            recomputeEffective()
            writeCommand(Commands.setFocusRocket(rocketId))
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Commands
    // ─────────────────────────────────────────────────────────────────────

    /** Fire-and-forget single-byte command (iOS sendCommand). */
    public fun sendBareCommand(cmdId: Int) {
        scope.launch { writeCommand(Commands.bare(cmdId)) }
    }

    /**
     * Write one raw OTA chunk frame on the FILE_TRANSFER characteristic and
     * SUSPEND until the stack reports it complete.  That completion is the
     * Android backpressure mechanism (one write per onCharacteristicWrite —
     * the analogue of iOS parking on canSendWriteWithoutResponse), so the
     * pump can never outrun the link by looping on this.
     *
     * write-without-response: there is no per-chunk ack on the wire, and a
     * dropped chunk is caught by the firmware's SHA check at OTA_FINISH.
     */
    public suspend fun writeOtaChunk(frame: ByteArray) {
        transport.write(TrCharacteristic.FILE_TRANSFER, frame, withResponse = false)
    }

    /** Tighter connection interval for the OTA pump only; always released. */
    public fun requestConnectionPriorityHigh(): Unit = transport.requestConnectionPriorityHigh()
    public fun releaseConnectionPriority(): Unit = transport.releaseConnectionPriority()

    /** Fire-and-forget prebuilt `[cmd][payload]` frame (iOS sendRawCommand). */
    public fun sendCommandFrame(frame: ByteArray) {
        scope.launch { writeCommand(frame) }
    }

    /**
     * cmd 8 with the desired rail state.  Explicit rather than a blind
     * toggle: a desync would otherwise cut the FC's rail when the operator
     * asked to power it up.  The UI still gates on [hasReceivedTelemetry]
     * (#377) so [railOn] is derived from a state we have actually seen.
     */
    public fun sendPowerState(railOn: Boolean): Unit =
        sendCommandFrame(Commands.powerState(railOn))

    /**
     * #159 power-on press: lights the busy state and arms a 3 min watchdog
     * before commanding the rail ON.  Cleared by the first pwr_pin_on
     * telemetry frame, by the watchdog (a genuinely-dropped command), or by
     * disconnect.  3 min because the OC can block its loop up to ~90 s
     * flushing the flight log before acting on cmd 8 — the watchdog must
     * outlast that.  The command carries the desired state, so a repeat
     * while busy is now idempotent rather than powering the rocket back off.
     */
    public fun beginPowerOn() {
        scope.launch {
            _poweringOn.value = true
            poweringOnJob?.cancel()
            poweringOnJob = scope.launch {
                delay(POWER_ON_WATCHDOG_MS)
                clearPoweringOnNow()
            }
            writeCommand(Commands.powerState(railOn = true))
        }
    }

    // Single-writer contract: the public mutators launch onto the session
    // dispatcher like every other public API; the *Now variants are the
    // direct implementations for on-dispatcher internal call sites that need
    // synchronous semantics (the atomic disconnect wipe, the watchdog, the
    // telemetry fold).  (Phase 2 review finding — these three previously
    // mutated on the caller's thread.)

    public fun clearPoweringOn() {
        scope.launch { clearPoweringOnNow() }
    }

    internal fun clearPoweringOnNow() {
        _poweringOn.value = false
        poweringOnJob?.cancel()
        poweringOnJob = null
    }

    public fun requestConfig(): Unit = sendBareCommand(BleCommandId.REQUEST_CONFIG)

    /**
     * Manual pyro continuity test — cmd 35, `[channel 1..4]` (direct rocket
     * links only; the pyro card never renders on BS links).  Opens the
     * per-channel TESTING window; the result arrives implicitly in later
     * telemetry frames' "ps" bits — no dedicated reply, timeout, or failure
     * state on the wire.
     */
    public fun sendPyroContTest(channel: Int) {
        scope.launch {
            _contTestPendingUntil.value = _contTestPendingUntil.value +
                (channel to clock() + CONT_TEST_PENDING_WINDOW_MS)
            writeCommand(Commands.pyroContTest(channel))
        }
    }

    /** True while [channel]'s TESTING window is open. */
    public fun contTestPending(channel: Int): Boolean =
        (_contTestPendingUntil.value[channel] ?: 0L) > clock()

    /**
     * Four-state continuity for [channel], as a flow so Compose recomposes
     * only when the VERDICT changes rather than on every telemetry frame.
     * iOS twin: `BLEDevice.pyroContinuity(channel:)`.
     *
     * [isBaseStation] is read once per emission rather than observed — it is
     * a plain getter off the identity, and both call sites already treat the
     * link type as fixed for the lifetime of the session.
     */
    public fun pyroContinuityFlow(channel: Int): Flow<PyroContinuity> =
        combine(telemetry, effectiveDataStatus, isConnected) { t, ds, connected ->
            pyroContinuityOf(t, channel, connected, ds, isBaseStation)
        }.distinctUntilChanged()

    /**
     * Snapshot of [pyroContinuityFlow] for non-Compose callers. Never use the
     * Bool-ish shorthand `== PRESENT` to render a verdict — that is the
     * collapse the four states exist to prevent (#828).
     */
    public fun pyroContinuity(channel: Int): PyroContinuity =
        pyroContinuityOf(
            telemetry.value,
            channel,
            isConnected.value,
            effectiveDataStatus.value,
            isBaseStation,
        )

    /**
     * iOS applyPyroConfig step 3: mirror an optimistic cmd-34 push into
     * [rocketConfig] so the dashboard pyro tiles update live — the firmware
     * does NOT re-send config_pyro after a pyro-config write, only on the
     * next connect/readback.
     *
     * Mirrors the SAME four channels the push carries, and only when a
     * readback already exists (iOS `if var cfg = device.rocketConfig`):
     * fabricating a config here would render the un-pushed fields as
     * device-reported truth when the device has reported nothing yet.
     */
    public fun mirrorPyroConfig(channels: List<PyroChannelConfig>) {
        if (channels.size != 4) return
        scope.launch {
            val cfg = _rocketConfig.value ?: return@launch
            _rocketConfig.value = cfg.copy(
                pyro1Enabled = channels[0].enabled,
                pyro1TriggerMode = channels[0].mode,
                pyro1TriggerValue = channels[0].value,
                pyro2Enabled = channels[1].enabled,
                pyro2TriggerMode = channels[1].mode,
                pyro2TriggerValue = channels[1].value,
                pyro3Enabled = channels[2].enabled,
                pyro3TriggerMode = channels[2].mode,
                pyro3TriggerValue = channels[2].value,
                pyro4Enabled = channels[3].enabled,
                pyro4TriggerMode = channels[3].mode,
                pyro4TriggerValue = channels[3].value,
            )
        }
    }

    public fun markSimLaunched() {
        scope.launch {
            _simLaunched.value = true
            simSawNonReady = false
        }
    }

    public fun clearSimBanner() {
        scope.launch { clearSimBannerNow() }
    }

    internal fun clearSimBannerNow() {
        _simLaunched.value = false
        simSawNonReady = false
    }

    /**
     * Kick off a base-station frequency scan (cmd 60).  #385: a 75 s wedge
     * timeout (generation-guarded) clears the spinner if the 0xAA result
     * notification is lost — the BS runs 5 passes, ~10-45 s.
     */
    public fun startFrequencyScan(startMHz: Float, stopMHz: Float, stepKHz: Int, dwellMs: Int) {
        scope.launch {
            _scanSamples.value = emptyList()
            _isScanning.value = true
            scanGeneration += 1
            val gen = scanGeneration
            scope.launch {
                delay(SCAN_RESULT_TIMEOUT_MS)
                if (_isScanning.value && scanGeneration == gen) {
                    _isScanning.value = false
                }
            }
            writeCommand(Commands.frequencyScan(startMHz, stopMHz, stepKHz, dwellMs))
        }
    }

    /**
     * #435 fire-and-forget Drift-Cast aim point (cmd 28).  Sending proves
     * NOTHING — success is gated on the guid_target echo; use
     * [sendGuidancePointConfirming] for the confirmed flow.
     */
    public fun sendGuidancePoint(lat: Double, lon: Double, altitudeM: Float) {
        scope.launch { writeCommand(Commands.guidancePoint(lat, lon, altitudeM)) }
    }

    /**
     * Cmd-28 round-trip with the #435 confirmation state machine
     * ([GuidanceSendFlow] owns the attribution rules; iOS DriftCastSendButton
     * owns this loop).  Captures the echo-seq baseline, sends, then feeds
     * every echo frame to the flow until it settles or [timeoutMs] elapses
     * (6 s on iOS) — a provisional reject settles as Failed only at timeout.
     */
    public suspend fun sendGuidancePointConfirming(
        lat: Double,
        lon: Double,
        altitudeM: Float,
        timeoutMs: Long = GUIDANCE_CONFIRM_TIMEOUT_MS,
    ): GuidanceSendFlow.Verdict {
        val flow = GuidanceSendFlow(
            baselineSeq = _guidanceEcho.value?.seq ?: -1,
            sentLat = lat,
            sentLon = lon,
        )
        try {
            withTimeout(timeoutMs) {
                _echoFrames
                    .onSubscription { writeCommand(Commands.guidancePoint(lat, lon, altitudeM)) }
                    .first { echo ->
                        flow.onEcho(echo)
                        flow.verdict != GuidanceSendFlow.Verdict.Waiting
                    }
            }
        } catch (_: TimeoutCancellationException) {
            flow.onTimeout()
        }
        return flow.verdict
    }

    // ── DeviceIdentityPusher (#547 registry push-through) ────────────────

    override fun sendSetUnitName(name: String) {
        scope.launch { writeCommand(Commands.setUnitName(name)) }
    }

    override fun sendSetNetworkId(nid: Int) {
        scope.launch { writeCommand(Commands.setNetworkId(nid)) }
    }

    override fun sendSetRocketId(rid: Int) {
        scope.launch { writeCommand(Commands.setRocketId(rid)) }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Internals
    // ─────────────────────────────────────────────────────────────────────

    /** iOS "Cannot send command: not connected" → silently drop. */
    private suspend fun writeCommand(frame: ByteArray): Boolean =
        runCatching {
            transport.write(TrCharacteristic.COMMAND, frame, withResponse = true)
        }.isSuccess

    /** Fresh phone time (unique sim filenames — iOS sendTimeSync before cmd 5). */
    public fun sendTimeSync() {
        scope.launch { sendTimeSyncNow() }
    }

    private suspend fun sendTimeSyncNow() {
        val t = Instant.ofEpochMilli(clock()).atOffset(ZoneOffset.UTC)
        writeCommand(
            Commands.timeSync(
                year = t.year,
                month = t.monthValue,
                day = t.dayOfMonth,
                hour = t.hour,
                minute = t.minute,
                second = t.second,
            ),
        )
    }

    public companion object {
        /** BLE 5 max — the OC/BS grant what they can (iOS negotiates ~185). */
        public const val REQUESTED_MTU: Int = 517

        /** iOS asyncAfter(1.0) between discovery and the cmd-20 readback. */
        public const val CONNECT_CONFIG_DELAY_MS: Long = 1_000

        /** iOS `BLEDevice.contTestPendingWindow` — the continuity-test
         *  round-trip allowance before displays trust "ps" bits again. */
        public const val CONT_TEST_PENDING_WINDOW_MS: Long = 2_500

        /** iOS asyncAfter(0.5) between the cmd-2 write and the explicit read. */
        public const val FILE_LIST_READ_DELAY_MS: Long = 500

        /** iOS 3.0 s download stall timer. */
        public const val DOWNLOAD_STALL_MS: Long = 3_000

        /** iOS 2.0 s RSSI poll (also drives the #390 staleness overlay). */
        public const val RSSI_POLL_MS: Long = 2_000

        /** Mirrors the BS firmware's BLE_TELEMETRY_STALE_MS. */
        public const val RELAY_STALE_THRESHOLD_MS: Long = 3_000

        /** iOS 180 s #159 power-on watchdog. */
        public const val POWER_ON_WATCHDOG_MS: Long = 180_000

        /** iOS 75 s #385 scan-result wedge timeout. */
        public const val SCAN_RESULT_TIMEOUT_MS: Long = 75_000

        /** iOS 6 s Drift-Cast confirmation window. */
        public const val GUIDANCE_CONFIRM_TIMEOUT_MS: Long = 6_000

        /** `[offset u32][len u16][flags u8]`. */
        public const val CHUNK_HEADER_SIZE: Int = 7
    }
}

/** Identity readback mirror (iOS: the @Published identity fields). */
public data class DeviceIdentity(
    val unitId: String = "",
    val unitName: String = "",
    val networkId: Int = 0,       // u8
    val rocketId: Int = 0,        // u8
    val deviceType: BleDeviceType = BleDeviceType.UNKNOWN,
    /** Connected device's fw stamp (#8) — the OC's, on a rocket link. */
    val firmwareVersion: String = "",
    /** FC's own fw stamp, OC-relayed (#8 Phase 4); FC OTA rollback compares THIS. */
    val fcFirmwareVersion: String = "",
)

/** One rocket seen via this base-station link's LoRa relay (iOS RemoteRocket). */
public data class RelayedRocket(
    val rocketId: Int,
    val unitName: String,
    val telemetry: TelemetryData,
    /** Session-clock millis of the last relayed frame. */
    val lastSeenMs: Long,
)

/** Download progress mirror (iOS isDownloading/downloadingFilename/downloadProgress). */
public data class DownloadProgress(
    val filename: String? = null,
    val progress: Double = 0.0,
    val active: Boolean = false,
)

/**
 * Typed outcome of [DeviceSession.downloadFile] — iOS collapses all failures
 * to a nil URL; the reasons are preserved here.
 */
public sealed interface DownloadResult {
    /** Assembled file bytes (EOF, or a stall with no known shortfall). */
    public class Success(public val bytes: ByteArray) : DownloadResult {
        override fun equals(other: Any?): Boolean =
            other is Success && bytes.contentEquals(other.bytes)

        override fun hashCode(): Int = bytes.contentHashCode()

        override fun toString(): String = "Success(${bytes.size} bytes)"
    }

    /** #526 EOF|ABORT — the device could not finish; nothing is kept. */
    public data object Aborted : DownloadResult

    /** 3 s stall with fewer bytes than the file list promised. */
    public data object Incomplete : DownloadResult

    /** The link dropped mid-transfer. */
    public data object Disconnected : DownloadResult

    /** A download is already in flight on this session. */
    public data object Busy : DownloadResult

    /** Not connected / the cmd-4 write failed. */
    public data object NotConnected : DownloadResult
}
