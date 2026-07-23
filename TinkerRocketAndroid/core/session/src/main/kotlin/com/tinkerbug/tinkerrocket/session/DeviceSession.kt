package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.BaseStationStorageStats
import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.protocol.ConfigIdentityMsg
import com.tinkerbug.tinkerrocket.protocol.FileInfo
import com.tinkerbug.tinkerrocket.protocol.FileOpsDispatch
import com.tinkerbug.tinkerrocket.protocol.FileOpsMessage
import com.tinkerbug.tinkerrocket.protocol.FrequencyScanSample
import com.tinkerbug.tinkerrocket.protocol.GuidanceSendFlow
import com.tinkerbug.tinkerrocket.protocol.GuidanceTargetEcho
import com.tinkerbug.tinkerrocket.protocol.IMUOrientationMode
import com.tinkerbug.tinkerrocket.protocol.ImuOrientMsg
import com.tinkerbug.tinkerrocket.protocol.MagCalStatus
import com.tinkerbug.tinkerrocket.protocol.OtaStatusUpdate
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
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
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
) : DeviceIdentityPusher {

    // ── Connection / link state ──────────────────────────────────────────

    private val _isConnected = MutableStateFlow(false)
    public val isConnected: StateFlow<Boolean> = _isConnected.asStateFlow()

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
        // Subscribe BEFORE connecting so no event can be dropped.
        scope.launch { transport.events.collect { onTransportEvent(it) } }
        return scope.launch {
            transport.connect()
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
        clearSimBanner()
        clearPoweringOn()
        _rocketConfig.value = null
        // Stale REVIEW must not bleed across sessions (#96); the FC
        // republishes IDLE on reconnect anyway.
        _magCalStatus.value = null
        _sensorCalStatus.value = null
        // #435: per-connection live state — a stale echo could "confirm" a
        // send made over a new connection to a rebooted FC.
        _guidanceEcho.value = null
        // Power state unknown again until the next session's first frame (#377).
        _hasReceivedTelemetry.value = false
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
            is TransportEvent.Connected -> _isConnected.value = true
            is TransportEvent.MtuChanged -> Unit
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
                    unitName = data.sourceUnitName ?: existing.unitName,
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

            // #390: sticky first-heard focus — never moves on its own after
            // this; the user (or the fleet re-seed) switches it.
            if (_focusRocketId.value == null) {
                _focusRocketId.value = rid
                onAutoFocus?.invoke(rid)
            }
            if (rid == _focusRocketId.value) {
                _telemetry.value = data
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
        if (data.pwrPinOn && _poweringOn.value) clearPoweringOn()
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

    // ─────────────────────────────────────────────────────────────────────
    // Download engine
    // ─────────────────────────────────────────────────────────────────────

    /**
     * Download a file over the file_transfer characteristic and return the
     * assembled bytes (or a typed failure — iOS returns nil for all of
     * them).  Chunk semantics are EXACTLY the iOS `handleFileChunk` engine:
     *
     *  - chunks are `[offset u32 LE][len u16 LE][flags u8][data]`; bytes are
     *    appended in ARRIVAL order — the offset field is never used for
     *    reassembly (bug-compat pin: a dropped chunk followed by EOF still
     *    "succeeds" with truncated data — the stall-shortfall check only
     *    guards the stall path);
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

        if (length > 0 && data.size >= CHUNK_HEADER_SIZE + length) {
            d.buffer.write(data, CHUNK_HEADER_SIZE, length)
        }
        val received = d.buffer.size()
        if (d.expectedSize > 0) {
            _downloadState.value = _downloadState.value.copy(
                progress = minOf(received.toDouble() / d.expectedSize, 1.0),
            )
        }
        if (isEof) {
            _downloadState.value = _downloadState.value.copy(progress = 1.0)
            cancelStallTimer()
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
        if (fromStallTimer && d.expectedSize > 0 && d.buffer.size() < d.expectedSize) {
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

    /** Fire-and-forget prebuilt `[cmd][payload]` frame (iOS sendRawCommand). */
    public fun sendCommandFrame(frame: ByteArray) {
        scope.launch { writeCommand(frame) }
    }

    /** Blind cmd-8 toggle — UI must gate on [hasReceivedTelemetry] (#377). */
    public fun sendPowerToggle(): Unit = sendBareCommand(BleCommandId.POWER_TOGGLE)

    /**
     * #159 power-on press: lights the busy state and arms a 3 min watchdog
     * before sending the toggle.  Cleared by the first pwr_pin_on telemetry
     * frame, by the watchdog (a genuinely-dropped command), or by disconnect.
     * 3 min because the OC can block its loop up to ~90 s flushing the
     * flight log before acting on cmd 8 — the watchdog must outlast that.
     */
    public fun beginPowerOn() {
        scope.launch {
            _poweringOn.value = true
            poweringOnJob?.cancel()
            poweringOnJob = scope.launch {
                delay(POWER_ON_WATCHDOG_MS)
                clearPoweringOn()
            }
            writeCommand(Commands.bare(BleCommandId.POWER_TOGGLE))
        }
    }

    public fun clearPoweringOn() {
        _poweringOn.value = false
        poweringOnJob?.cancel()
        poweringOnJob = null
    }

    public fun requestConfig(): Unit = sendBareCommand(BleCommandId.REQUEST_CONFIG)

    public fun markSimLaunched() {
        _simLaunched.value = true
        simSawNonReady = false
    }

    public fun clearSimBanner() {
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
