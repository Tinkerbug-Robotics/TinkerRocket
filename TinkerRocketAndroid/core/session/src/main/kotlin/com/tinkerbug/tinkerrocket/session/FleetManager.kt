package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.protocol.ConfigIdentityMsg
import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.CoroutineStart
import kotlinx.coroutines.Job
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.TimeoutCancellationException
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.filterIsInstance
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeout
import kotlinx.coroutines.withTimeoutOrNull
import kotlin.math.min

/**
 * Multi-device fleet manager — scanning policy, connect/reconnect
 * choreography, identity precedence, and the fleet-scoped survivors that must
 * outlive any single connection.  Semantics-exact port of iOS `BLEFleet.swift`
 * (the CBCentralManager owner), with the CoreBluetooth-specific parts replaced
 * by their Android analogs per android-port plan §3.
 *
 * ## Concurrency model (single-writer)
 * ALL state mutation happens on [scope]'s dispatcher.  Public mutating
 * functions must be called from that dispatcher (in production the fleet's
 * single confined dispatcher; in tests the `runTest` scheduler).  Timed
 * behavior (scan timeout, reconnect backoff) runs as coroutines on [scope]
 * using `delay`, so `kotlinx-coroutines-test` virtual time drives it.  State
 * is exposed as [StateFlow]s, safe to read from anywhere.
 *
 * ## Dropped iOS-isms (documented, no port)
 *  - **#173 strong-ref bookkeeping** (`connectingPeripherals`): CoreBluetooth
 *    retains `CBPeripheral`s weakly, so iOS must hold its own strong reference
 *    from `connect()` until the connection resolves or CB cancels with "API
 *    MISUSE: … unused peripheral".  On Android the `BluetoothGatt` object is
 *    owned by the `RealBleTransport` implementation and this manager holds the
 *    [BleTransport] strongly in the device record / connect job — there is
 *    nothing weakly-referenced to protect.  No analog needed.
 *  - **#571 `willRestoreState` / CB state restoration**: iOS-only.  Android
 *    has no OS-level BLE state restoration for a relaunched process; the
 *    analog is a foreground service keeping the process alive plus a rescan
 *    on cold start (app layer, not this class).  The #571 lesson that DOES
 *    carry over — never keep only the first of several restored links — is
 *    moot here because every device gets its own record and watcher.
 *  - **CBCentralManager power-state gating** ("Bluetooth not ready" /
 *    "poweredOff" statuses): adapter state is owned by the platform
 *    [BleScanner]/[TransportFactory] implementations.
 *  - **Idle-timer (screen sleep) toggling**: UI-layer concern on Android
 *    (window flag), not session-layer.
 */
public class FleetManager<S : Any>(
    private val scope: CoroutineScope,
    private val scanner: BleScanner,
    private val transportFactory: TransportFactory,
    private val sessionFactory: FleetSessionFactory<S>,
    /**
     * Registry of every device ever provisioned/seen, keyed by hardware
     * unitID.  Lives on the fleet (not a view) because identity readbacks
     * fold into it for every connected device (iOS parity).
     */
    public val knownDevices: KnownDeviceStore,
    /** Injected clock (no wall-clock reads in the session layer). */
    private val nowMillis: () -> Long = System::currentTimeMillis,
    /**
     * Where the "we were connected when the app went away" hint lives (#633).
     * Defaults to an in-memory no-op so tests and any existing construction
     * keep working; the app supplies a persistent one.
     */
    private val lastSession: LastSessionStore = LastSessionStore.InMemory(),
) {

    // ------------------------------------------------------------- published

    private val _isScanning = MutableStateFlow(false)
    public val isScanning: StateFlow<Boolean> = _isScanning.asStateFlow()

    /**
     * #394: true only when the user explicitly asked to add a device (the
     * "Add" button); false for automatic/background scans (reconnect
     * fallback, endgame search).  The dashboard's "Add Device" sheet gates on
     * this so a returning unit re-pairing silently doesn't pop an unsolicited
     * modal.
     */
    private val _userInitiatedScan = MutableStateFlow(false)
    public val userInitiatedScan: StateFlow<Boolean> = _userInitiatedScan.asStateFlow()

    private val _statusMessage = MutableStateFlow("Not connected")
    public val statusMessage: StateFlow<String> = _statusMessage.asStateFlow()

    private val _discoveredDevices = MutableStateFlow<List<DiscoveredDevice>>(emptyList())
    public val discoveredDevices: StateFlow<List<DiscoveredDevice>> = _discoveredDevices.asStateFlow()

    /**
     * Currently connected devices, keyed by [FleetDevice.deviceId], in
     * connection order (insertion-ordered map — "first connected" semantics
     * mirror the iOS array).  A NEW session object (and a bumped
     * [FleetDevice.generation]) appears on every (re)connection; anything
     * that must survive reconnects lives on this fleet instead
     * ([lastValidRocketFixes], [bsFocus], [hiddenRocketKeys]).
     */
    private val _devices = MutableStateFlow<Map<String, FleetDevice<S>>>(linkedMapOf())
    public val devices: StateFlow<Map<String, FleetDevice<S>>> = _devices.asStateFlow()

    private val _linkActive = MutableStateFlow(false)

    /**
     * True while the app still INTENDS to hold a link: a device is connected,
     * or a reconnect ladder is still trying to get one back.
     *
     * #829: the foreground service keys off this rather than [devices].
     * [handleDisconnect] removes the device from [devices] BEFORE the ladder
     * starts, so on a single-device fleet every transient drop looked like
     * "fleet is empty" and tore down the service that exists to keep the
     * process — and its BLE links — alive, at exactly the moment the link
     * needed recovering.
     *
     * Note there is no "the ladder gave up" state to wait for: the endgame in
     * [launchReconnectLadder] is an unbounded sighting wait, deliberately, so
     * a flight in progress is never abandoned. What clears this is a user
     * disconnect, which cancels the ladder — or the ladder's job dying with
     * the scope.
     */
    public val linkActive: StateFlow<Boolean> = _linkActive.asStateFlow()

    /**
     * Recompute [linkActive] after any change to [_devices] or [reconnectJobs].
     *
     * A completed or cancelled Job reports `isActive == false`, so stale
     * entries left in [reconnectJobs] are inert here and need no bookkeeping
     * of their own — which keeps this correct without a
     * remove-the-right-generation dance against a ladder that was just
     * superseded.
     */
    private fun refreshLinkActive() {
        _linkActive.value = _devices.value.isNotEmpty() ||
            reconnectJobs.values.any { it.isActive }
    }

    /** Which device the dashboard is currently showing. */
    private val _activeDeviceId = MutableStateFlow<String?>(null)
    public val activeDeviceId: StateFlow<String?> = _activeDeviceId.asStateFlow()

    /**
     * Latest usable rocket GPS fix per rocket (#140).  Lives on the fleet
     * because the per-connection session is destroyed+recreated on every
     * BLE disconnect/reconnect — without an outer cache the map would blank
     * every time the phone briefly loses the base station.  In-memory only.
     * Keyed by (networkID, rocketID) — rocket IDs are only unique per network
     * (#390).  The key uses the LIVE networkID at record time, so a fix
     * recorded before the identity readback populates networkID can land
     * under nid 0 and be superseded once the real nid is known — harmless.
     */
    private val _lastValidRocketFixes =
        MutableStateFlow<Map<RocketKey, LastValidRocketFix>>(emptyMap())
    public val lastValidRocketFixes: StateFlow<Map<RocketKey, LastValidRocketFix>> =
        _lastValidRocketFixes.asStateFlow()

    /**
     * #390: roster rockets the user chose to hide from the dashboard.
     * Hiding is a pure view filter — it never changes radios or command
     * routing.
     */
    private val _hiddenRocketKeys = MutableStateFlow<Set<RocketKey>>(emptySet())
    public val hiddenRocketKeys: StateFlow<Set<RocketKey>> = _hiddenRocketKeys.asStateFlow()

    /**
     * #390: which base station's "pair" the dashboard foregrounds.
     * null = auto (the first connected BS).  Only set explicitly by the user
     * via the pair switcher; survives that BS disconnecting (fallback is
     * computed, the pick is kept).
     */
    private val _foregroundBSID = MutableStateFlow<String?>(null)
    public val foregroundBSID: StateFlow<String?> = _foregroundBSID.asStateFlow()

    /**
     * #390: per-base-station radio focus (rocketID), keyed by [deviceId] so
     * it survives session recreation on reconnect.  Auto-latched to the first
     * rocket a BS hears ([noteAutoFocus]); switched explicitly by the user
     * ([setFocus]).  Pushed to the BS via cmd 45 (RAM-only on the BS, so it
     * is re-pushed on every (re)connect).
     */
    private val _bsFocus = MutableStateFlow<Map<String, Int>>(emptyMap())
    public val bsFocus: StateFlow<Map<String, Int>> = _bsFocus.asStateFlow()

    // --------------------------------------------------------------- private

    /** Per-device monotonic connection-generation counter.  Replaces iOS
     *  object identity (a fresh BLEDevice per connect): same id + higher
     *  generation = "same physical device, new connection". */
    private val generations = mutableMapOf<String, Int>()

    /** Devices whose next Disconnected event is user-requested (suppresses
     *  the reconnect ladder).  Per-device, unlike the iOS single global
     *  `userInitiatedDisconnect` flag — see docs/android-parity-ledger.md (the global flag drops
     *  suppression for the 2nd device of a disconnectAll). */
    private val pendingUserDisconnects = mutableSetOf<String>()

    private var scanJob: Job? = null
    private var scanEpoch: Int = 0
    private val reconnectJobs = mutableMapOf<String, Job>()
    private var resumeJob: Job? = null
    private val connectJobs = mutableMapOf<String, Job>()

    public companion object {
        public const val SCAN_TIMEOUT_MS: Long = 15_000

        /** #291: was 3 (~7 s); raised for flight dropouts. */
        public const val MAX_RECONNECT_ATTEMPTS: Int = 8

        /**
         * How long a cold-start resume waits to SEE the last device before
         * giving up (#633).  Bounded on purpose — the mid-flight endgame waits
         * forever because a flight is in the air; at startup nothing is, and an
         * endless background scan is just battery.  20 s covers a board that is
         * already powered and advertising, which is the case worth serving.
         */
        public const val RESUME_SIGHTING_TIMEOUT_MS: Long = 20_000

        /**
         * Backoff before reconnect attempt `n` (1-based): `min(8, 2^(n-1))` s
         * (#291 capped — 2^(n-1) blows up fast once the cap is raised).
         * Sequence: 1, 2, 4, 8, 8, 8, 8, 8 s.
         */
        public fun reconnectBackoffMs(attempt: Int): Long =
            min(8L, 1L shl (attempt - 1)) * 1000L
    }

    // -------------------------------------------------------------- scanning

    /**
     * Start (or restart) a scan.  The scan filter is the TinkerRocket
     * SERVICE UUID and nothing else (#547) — see the [BleScanner] contract.
     * No name filter is applied to results either: anything the scanner
     * delivers IS a TinkerRocket board.  The old belt-and-suspenders
     * TR-* / Tinker name guard silently dropped every device renamed through
     * My Devices — the firmware advertises the raw user-set unit name; only
     * factory defaults carry the TR-R-/TR-B- prefix.
     *
     * Times out after [SCAN_TIMEOUT_MS] (15 s, iOS parity), with the same
     * end-of-scan status messages.
     *
     * @param userInitiated #394 — true ONLY for the explicit "Add Device"
     *   action; reconnect/background scans pass false so UI gated on
     *   [userInitiatedScan] doesn't pop.
     */
    public fun scan(userInitiated: Boolean = false) {
        _discoveredDevices.value = emptyList()
        _statusMessage.value = "Scanning..."
        _isScanning.value = true
        _userInitiatedScan.value = userInitiated
        scanJob?.cancel()
        // Epoch-guarded timeout: an already-queued stale timeout resumption
        // (cancel() can't unqueue it) must not truncate the NEW scan.
        // Deliberate divergence from iOS, whose unguarded 15 s timer DOES
        // truncate a restarted scan — see the parity ledger (Phase 2 review).
        val epoch = ++scanEpoch
        scanJob = scope.launch {
            try {
                withTimeout(SCAN_TIMEOUT_MS) {
                    scanner.advertisements().collect { onAdvertisement(it) }
                }
            } catch (_: TimeoutCancellationException) {
                if (epoch == scanEpoch) onScanTimeout()
            } catch (e: CancellationException) {
                throw e          // structured cancellation is not a failure
            } catch (e: Exception) {
                // The scanner flow can fail rather than time out — most often
                // "BLE scanner unavailable (adapter off?)" when the adapter is
                // off, but any transport error lands here.  Left uncaught this
                // escapes the launch and the default handler KILLS THE PROCESS:
                // tapping Scan with Bluetooth off crashed the app outright
                // (Checkpoint A Group 1, bench 2026-07-29 — APP CRASH(EXCEPTION)).
                //
                // Report it and stop scanning instead; the adapter coming back
                // is a normal thing to recover from, not a fatal condition.
                if (epoch == scanEpoch) {
                    _isScanning.value = false
                    _userInitiatedScan.value = false
                    _statusMessage.value = e.message ?: "Scan failed"
                }
            }
        }
    }

    public fun stopScanning() {
        scanEpoch++   // invalidate any queued timeout resumption
        scanJob?.cancel()
        scanJob = null
        _isScanning.value = false
        _userInitiatedScan.value = false  // #394
    }

    private fun onScanTimeout() {
        // iOS parity: stopScanning + result status only when nothing is
        // connected (a live session's status must not be clobbered).
        _isScanning.value = false
        _userInitiatedScan.value = false
        scanJob = null
        if (_devices.value.isEmpty()) {
            _statusMessage.value =
                if (_discoveredDevices.value.isEmpty()) "No devices found" else "Scan complete"
        }
    }

    private fun onAdvertisement(adv: BleAdvertisement) {
        // #547: the name comes from the SCAN RECORD (advertisement/scan
        // response payload), NEVER a stack-cached device name — the Android
        // analog of preferring CBAdvertisementDataLocalNameKey over the
        // iOS-cached `peripheral.name` (stale/nil after a re-flash).  A
        // record with no local name displays as "Unknown"; it is still
        // listed (the service UUID already proved it's ours).
        val name = adv.advertisedName ?: "Unknown"
        val list = _discoveredDevices.value
        val idx = list.indexOfFirst { it.deviceId == adv.deviceId }
        _discoveredDevices.value = if (idx >= 0) {
            // Re-discovery updates RSSI only; the first-seen name sticks
            // (iOS parity).
            list.toMutableList().also { it[idx] = it[idx].copy(rssi = adv.rssi) }
        } else {
            list + DiscoveredDevice(
                deviceId = adv.deviceId,
                name = name,
                rssi = adv.rssi,
                // Renames happen through this app, so a renamed device's
                // advertised name matches its registry record — recover the
                // type the name prefix no longer carries (#547).
                knownType = knownDevices.deviceTypeForAdvertisedName(name),
            )
        }
        val n = _discoveredDevices.value.size
        _statusMessage.value = "Found $n device${if (n == 1) "" else "s"}"
    }

    // ------------------------------------------------------------ connection

    /**
     * Connect to a discovered device.  Uses the scan-time [DiscoveredDevice]
     * identity when available (name + registry type hint); falls back to
     * "Unknown" for a caller-supplied id that was never discovered this
     * session.
     */
    public fun connect(deviceId: String) {
        stopScanning()
        val discovered = _discoveredDevices.value.firstOrNull { it.deviceId == deviceId }
        val name = discovered?.name ?: "Unknown"
        _statusMessage.value = "Connecting to $name..."
        connectJobs[deviceId]?.cancel()
        // A user-driven connect supersedes any in-flight reconnect ladder —
        // two concurrent tryConnects for one device would race adoption
        // (Phase 2 review).
        reconnectJobs.remove(deviceId)?.cancel()
        refreshLinkActive()
        connectJobs[deviceId] = scope.launch {
            if (!tryConnect(deviceId, name, autoConnect = false)) {
                _statusMessage.value = "Connection failed"
            }
        }
    }

    /**
     * User-initiated disconnect of one device.  Suppresses the reconnect
     * ladder for THIS device's next Disconnected event (per-device — see
     * docs/android-parity-ledger.md, iOS global-flag divergence).
     */
    public fun disconnect(deviceId: String) {
        val dev = _devices.value[deviceId] ?: return
        pendingUserDisconnects.add(deviceId)
        dev.transport.disconnect()
    }

    /** Disconnect all devices (convenience for single-device mode). */
    public fun disconnectAll() {
        for (dev in _devices.value.values) {
            pendingUserDisconnects.add(dev.deviceId)
            dev.transport.disconnect()
        }
    }

    /**
     * Create a transport, connect it, and adopt the session on success.
     * Returns false on failure (the caller decides retry policy).
     */
    private suspend fun tryConnect(deviceId: String, name: String, autoConnect: Boolean): Boolean {
        val transport = transportFactory.create(deviceId, autoConnect)
        // Disconnect watcher: registered UNDISPATCHED *before* connect() so a
        // drop in the connect window can never be missed — transport.events
        // has NO replay guarantee (BleTransport contract; Phase 2 review).
        // The watcher is scoped to THIS transport: handleDisconnect ignores
        // ghosts (see there).  One event drives the full teardown+reconnect
        // decision (iOS didDisconnectPeripheral).
        val watcher = scope.launch(start = CoroutineStart.UNDISPATCHED) {
            transport.events.filterIsInstance<TransportEvent.Disconnected>().first()
            handleDisconnect(deviceId, transport)
        }
        return try {
            transport.connect()
            adopt(deviceId, name, transport)
            // #633: remember what we were talking to, so a cold start can
            // resume it.  Written on every successful connect, cleared only by
            // an explicit user disconnect — so "was connected" survives a
            // crash, an OEM kill, or a phone reboot, which are exactly the
            // cases the field cares about.
            lastSession.saveLastConnected(deviceId, name)
            true
        } catch (_: Exception) {
            watcher.cancel()
            false
        }
    }

    /**
     * Wire a freshly connected device into the fleet (iOS `didConnect` +
     * `adopt`): device-type precedence seed, a NEW session at generation+1,
     * a disconnect watcher, and the connect-time cmd-45 focus re-push.
     */
    private fun adopt(deviceId: String, advertisedName: String, transport: BleTransport) {
        val generation = (generations[deviceId] ?: 0) + 1
        generations[deviceId] = generation

        // #547 identity precedence at connect (seedTypeFromRegistry):
        //   1. the scan-time DiscoveredDevice registry hint,
        //   2. a live registry lookup by advertised name,
        //   3. the TR-R-/TR-B- prefix, then the legacy substring heuristic
        //      (BleDeviceType.fromName — regression-pinned oddities and all).
        // The config_identity readback's "dt" overrides everything ~1 s
        // later via [onIdentityReadback].
        val discovered = _discoveredDevices.value.firstOrNull { it.deviceId == deviceId }
        val deviceType = discovered?.knownType
            ?: knownDevices.deviceTypeForAdvertisedName(advertisedName)
            ?: BleDeviceType.fromName(advertisedName)

        // #390: the sticky focus pin is SEEDED into the session (iOS
        // BLEFleet.adopt), never pushed from here — the session's connect
        // choreography sends the single cmd 45 after cmd 20.  A fleet-side
        // push raced the choreography and double-sent (Phase 2 review).
        val session = sessionFactory.create(
            deviceId, advertisedName, generation, transport,
            seedFocusRocket = _bsFocus.value[deviceId],
        )
        val device = FleetDevice(
            deviceId = deviceId,
            advertisedName = advertisedName,
            generation = generation,
            session = session,
            transport = transport,
            deviceType = deviceType,
        )
        _devices.value = LinkedHashMap(_devices.value).also { it[deviceId] = device }
        refreshLinkActive()
        if (_activeDeviceId.value == null) _activeDeviceId.value = deviceId
        _statusMessage.value = "Connected to $advertisedName"
    }

    private fun handleDisconnect(deviceId: String, transport: BleTransport) {
        val dev = _devices.value[deviceId]
        // Connection-scoped: a GHOST event (a replaced/failed transport's
        // late Disconnected) must not tear down the CURRENT connection.
        // Only the transport actually adopted for this device may act; a
        // ghost's own cleanup is just its watcher ending (Phase 2 review).
        if (dev != null && dev.transport !== transport) return
        if (dev != null) {
            sessionFactory.close(dev.session)
            _devices.value = LinkedHashMap(_devices.value).also { it.remove(deviceId) }
        }
        if (_activeDeviceId.value == deviceId) {
            _activeDeviceId.value = _devices.value.keys.firstOrNull()
        }

        if (pendingUserDisconnects.remove(deviceId)) {
            reconnectJobs.remove(deviceId)?.cancel()
            // The user walked away from THIS device — but another may still be
            // connected or mid-ladder, so recompute rather than clearing.
            refreshLinkActive()
            _statusMessage.value = "Disconnected"
            _discoveredDevices.value = emptyList()
            // Deliberate: walking away is the ONE case we don't resume from.
            // Anything else (crash, OEM kill, reboot, battery pull) leaves the
            // hint in place (#633).
            lastSession.clearLastConnected()
            return
        }

        // Unexpected drop → reconnect ladder.
        launchReconnectLadder(deviceId, dev?.advertisedName ?: "rocket")
    }

    // -------------------------------------------------------- cold-start resume

    /**
     * Reconnect on a cold start to whatever we were connected to (#633).
     *
     * The gap this closes: mid-session recovery was already solid — a board
     * power-cycle comes back in under 6 s — but nothing resumed after the app
     * itself restarted.  `scan()` was only ever called from the Scan button, so
     * a crash, an OEM kill or a phone reboot left the user tapping Scan then
     * Connect at exactly the moment they'd rather be watching the pad.  iOS
     * never had this: state restoration hands its connections back, and it
     * re-scans on Bluetooth power-on.
     *
     * Reuses the ladder's endgame shape — scan, wait for a real advertisement,
     * THEN autoConnect — because the rule that made that safe still applies: an
     * `autoConnect=true` transport for an address the adapter hasn't recently
     * seen can silently never fire, so we never autoConnect blind from a MAC.
     *
     * BOUNDED, unlike the mid-flight endgame's unbounded wait.  That one is
     * unbounded on purpose: a flight is in progress and abandoning it is worse
     * than scanning. Here nothing is in the air, and a cold start that scans
     * forever is just a battery drain — so it gives up after
     * [RESUME_SIGHTING_TIMEOUT_MS] and leaves the user on a normal scanner.
     *
     * Safe to call unconditionally at startup: no hint, or already connected,
     * and it does nothing.
     */
    public fun resumeLastSession() {
        val last = lastSession.loadLastConnected() ?: return
        if (_devices.value.isNotEmpty()) return          // already connected
        resumeJob?.cancel()
        resumeJob = scope.launch {
            _statusMessage.value = "Looking for ${last.name}…"
            // Background scan — NOT user-initiated, so no modal pops (#394).
            scan(userInitiated = false)
            val seen = withTimeoutOrNull(RESUME_SIGHTING_TIMEOUT_MS) {
                scanner.advertisements().first { it.deviceId == last.deviceId }
            }
            if (seen == null) {
                // Not here — say so plainly instead of implying we're still
                // trying.  The device list keeps whatever the scan turned up.
                if (_devices.value.isEmpty()) {
                    _statusMessage.value = "${last.name} not found"
                }
                return@launch
            }
            // Connected by other means while we waited (user tapped it).
            if (_devices.value.containsKey(last.deviceId)) return@launch
            tryConnect(last.deviceId, last.name, autoConnect = true)
        }
    }

    // ----------------------------------------------------- reconnect ladder

    /**
     * Per-device reconnect coroutine.  Up to [MAX_RECONNECT_ATTEMPTS]
     * direct-connect attempts spaced [reconnectBackoffMs] apart
     * (1,2,4,8,8,8,8,8 s), then the endgame.
     *
     * iOS endgame (#291): park a CoreBluetooth `connect()` forever (CB
     * connects whenever the peripheral returns to range) + fall back to
     * scanning.  Android has no trustworthy parked-connect from a bare MAC —
     * `autoConnect=true` against an address the adapter hasn't recently seen
     * can silently never fire.  Android analog (plan §3): **re-scan, THEN
     * autoConnect** — keep a background scan-sighting wait alive, and only
     * once the device is seen advertising again ask [TransportFactory] for an
     * `autoConnect=true` transport.  Never autoConnect blind from a MAC.
     */
    private fun launchReconnectLadder(deviceId: String, name: String) {
        reconnectJobs[deviceId]?.cancel()
        reconnectJobs[deviceId] = scope.launch {
            for (attempt in 1..MAX_RECONNECT_ATTEMPTS) {
                _statusMessage.value = "Reconnecting ($attempt/$MAX_RECONNECT_ATTEMPTS)..."
                kotlinx.coroutines.delay(reconnectBackoffMs(attempt))
                // Reconnected by other means (user tapped it in a scan)
                // while we were backing off — stand down (iOS guard).
                if (_devices.value.containsKey(deviceId)) return@launch
                if (tryConnect(deviceId, name, autoConnect = false)) return@launch
            }

            // Endgame: don't abandon the flight after a handful of tries.
            _statusMessage.value = "Searching for $name…"
            // Background UI scan so a returning device re-pairs / relists
            // automatically — NOT user-initiated (#394: no modal).
            scan(userInitiated = false)
            while (true) {
                // Unbounded sighting wait — the Android analog of the
                // parked CB connect().  Only a fresh advertisement unlocks
                // the autoConnect attempt (never blind from a MAC).
                scanner.advertisements().first { it.deviceId == deviceId }
                if (_devices.value.containsKey(deviceId)) return@launch
                if (tryConnect(deviceId, name, autoConnect = true)) return@launch
            }
        }
        // The ladder is now the thing keeping the link "intended" — set it
        // before anyone can observe the empty fleet, and recompute when the
        // job ends however it ends (reconnected, cancelled, scope torn down).
        refreshLinkActive()
        reconnectJobs[deviceId]?.invokeOnCompletion { refreshLinkActive() }
    }

    // --------------------------------------------------------- device lookup

    /**
     * The live device for a hardware unitID, if currently connected AND past
     * its identity readback.  Used by the device manager to decide
     * push-now vs queue-for-next-connect.
     */
    public fun connectedDevice(unitId: String): FleetDevice<S>? {
        if (unitId.isEmpty()) return null
        return _devices.value.values.firstOrNull { it.unitId == unitId }
    }

    /** Convenience: the active device, or the first connected one. */
    public fun activeDevice(): FleetDevice<S>? {
        val id = _activeDeviceId.value ?: return _devices.value.values.firstOrNull()
        return _devices.value[id]
    }

    public fun setActiveDevice(deviceId: String?) {
        _activeDeviceId.value = deviceId
    }

    /** Connected base-station links, in connection order. */
    public fun baseStations(): List<FleetDevice<S>> =
        _devices.value.values.filter { it.isBaseStation }

    /**
     * The base station whose pair is on screen: the user's explicit pick
     * while that BS is still connected, else the first connected BS (#390).
     */
    public fun foregroundBaseStation(): FleetDevice<S>? {
        _foregroundBSID.value?.let { id ->
            _devices.value[id]?.takeIf { it.isBaseStation }?.let { return it }
        }
        return baseStations().firstOrNull()
    }

    /** #390: explicit pair pick (null = auto / first connected BS). */
    public fun setForegroundBS(deviceId: String?) {
        _foregroundBSID.value = deviceId
    }

    // -------------------------------------------------------- identity intake

    /**
     * Fold a `config_identity` readback into the fleet record and the
     * known-device registry (iOS parseTelemetryData's config_identity
     * branch + BLEFleet's registry hookup).  Called by the device session
     * when its telemetry demux hits [ConfigIdentityMsg].
     *
     * Only present keys apply; the "dt" field overrides the connect-time
     * name/registry seed (device truth beats every heuristic), and an
     * unknown "dt" keeps the previous type.
     *
     * @param pusher the session's [DeviceIdentityPusher] so the registry can
     *   push offline-queued edits; null skips the push (edits stay queued).
     */
    public fun onIdentityReadback(
        deviceId: String,
        msg: ConfigIdentityMsg,
        pusher: DeviceIdentityPusher?,
    ) {
        val dev = _devices.value[deviceId] ?: return
        val updated = dev.copy(
            unitId = msg.unitId ?: dev.unitId,
            unitName = msg.unitName ?: dev.unitName,
            networkId = msg.networkId ?: dev.networkId,
            rocketId = msg.rocketId ?: dev.rocketId,
            deviceType = msg.deviceType?.let { BleDeviceType.fromRaw(it) } ?: dev.deviceType,
            firmwareVersion = msg.firmwareVersion ?: dev.firmwareVersion,
        )
        _devices.value = LinkedHashMap(_devices.value).also { it[deviceId] = updated }
        knownDevices.deviceDidReportIdentity(
            unitID = updated.unitId,
            name = updated.unitName,
            deviceType = updated.deviceType,
            networkID = updated.networkId,
            rocketID = updated.rocketId,
            pusher = pusher,
        )
    }

    // ------------------------------------------------- last-valid fix (#140)

    /**
     * Called by the device session on every telemetry packet.  Updates the
     * cached fix when the packet carries a usable GPS solution, and returns
     * the resulting fix (new or pre-existing) so the caller can mirror it
     * onto its own state — including the hydrate-on-reconnect case where a
     * brand-new session's first packet has no GPS but a prior session cached
     * one.
     */
    public fun recordRocketFix(telemetry: TelemetryData, key: RocketKey): LastValidRocketFix? {
        val fresh = LastValidRocketFix.fromTelemetry(telemetry, key.rocketId, nowMillis())
        if (fresh != null) {
            _lastValidRocketFixes.value = _lastValidRocketFixes.value + (key to fresh)
            return fresh
        }
        return _lastValidRocketFixes.value[key]
    }

    public fun lastValidRocketFix(key: RocketKey): LastValidRocketFix? =
        _lastValidRocketFixes.value[key]

    // -------------------------------------------------------- rockets (#390)

    /** #390: hide a roster rocket — pure view filter, never touches radios. */
    public fun hideRocket(key: RocketKey) {
        _hiddenRocketKeys.value = _hiddenRocketKeys.value + key
    }

    public fun unhideRocket(key: RocketKey) {
        _hiddenRocketKeys.value = _hiddenRocketKeys.value - key
    }

    /**
     * Record a base station's sticky auto-focus (first rocket heard) so it
     * survives the session being recreated on reconnect.  Never overwrites
     * an existing choice — that's the point of stickiness.  Pushes the pin
     * to the BS too (cmd 45): its own auto pick is the same first-heard
     * rocket, but pinning makes the agreement explicit and disables the
     * BS-side silent fallback while the app is attached.
     */
    public fun noteAutoFocus(deviceId: String, rocketId: Int) {
        if (_bsFocus.value.containsKey(deviceId)) return
        _bsFocus.value = _bsFocus.value + (deviceId to rocketId)
        sendCommandFrame(deviceId, Commands.setFocusRocket(rocketId))
    }

    /**
     * User-driven focus switch for one base station.  Updates the sticky map
     * and pushes the pin to the BS (cmd 45) so hop-follow / stale re-push /
     * uplink targeting move with it.  The session-side re-mirror (cached
     * telemetry + latched fix, so the dashboard doesn't wait for the next
     * frame) is the DeviceSession's job — it observes [bsFocus].
     */
    public fun setFocus(deviceId: String, rocketId: Int) {
        _bsFocus.value = _bsFocus.value + (deviceId to rocketId)
        sendCommandFrame(deviceId, Commands.setFocusRocket(rocketId))
    }

    /** The pinned focus rocket for a BS link, or null if none heard yet. */
    public fun focusFor(deviceId: String): Int? = _bsFocus.value[deviceId]

    // ------------------------------------------------------- command routing

    /**
     * #390 relay-aware routing: wrap a rocket-targeted `[cmd][payload]`
     * frame for the link it will be sent on.  On a base-station link with a
     * pinned focus rocket the frame is wrapped in the cmd-50 relay envelope
     * targeting that rocket; otherwise (direct rocket link, or a BS that has
     * not heard a rocket yet) the frame passes through unchanged.
     *
     * Cmd-50 duality: wire id 50 is the RELAY envelope only on BS links —
     * on a rocket (OC) link the same id is MAG-CAL START.  That is why the
     * wrap decision keys on the link's device type: a bare `[50]` frame sent
     * on a rocket link must arrive unwrapped as mag-cal start, and a rocket
     * command on a BS link must be wrapped or the BS would misinterpret it.
     */
    public fun relayAwareFrame(deviceId: String, innerFrame: ByteArray): ByteArray {
        val dev = _devices.value[deviceId] ?: return innerFrame
        val focus = _bsFocus.value[deviceId]
        return if (dev.isBaseStation && focus != null) {
            Commands.relayToRocket(focus, innerFrame)
        } else {
            innerFrame
        }
    }

    /** [relayAwareFrame] + write on the COMMAND characteristic. */
    public fun sendRelayAware(deviceId: String, innerFrame: ByteArray) {
        sendCommandFrame(deviceId, relayAwareFrame(deviceId, innerFrame))
    }

    /** Fire-and-forget command write on a device's COMMAND characteristic. */
    public fun sendCommandFrame(deviceId: String, frame: ByteArray) {
        val dev = _devices.value[deviceId] ?: return
        scope.launch {
            try {
                dev.transport.write(TrCharacteristic.COMMAND, frame, withResponse = true)
            } catch (_: Exception) {
                // Write failures surface through the transport's own event
                // stream / disconnect; command sends are fire-and-forget
                // (iOS parity: writeValue has no error path at the call
                // site).
            }
        }
    }
}

// --------------------------------------------------------------------- seams

/**
 * Platform scanner seam.  The ONLY filter is the TinkerRocket service UUID
 * ([TrCharacteristic.SERVICE_UUID]) — NEVER filter or gate on the advertised
 * name (#547: renamed devices advertise the raw user-set name; a name filter
 * silently drops them).
 */
public interface BleScanner {
    /**
     * Cold flow: collection starts the platform scan, cancellation stops it.
     * Implementations must support concurrent collectors (ref-count the
     * platform scan) — the fleet may hold a UI scan and an endgame
     * sighting-wait at once.
     *
     * [BleAdvertisement.advertisedName] MUST come from the scan record
     * (advertising/scan-response payload), never from a stack-cached device
     * name — the Android analog of iOS preferring
     * `CBAdvertisementDataLocalNameKey` over the cached `peripheral.name`
     * (which is nil/stale after a firmware re-flash).
     */
    public fun advertisements(): Flow<BleAdvertisement>
}

/** One scan sighting. */
public data class BleAdvertisement(
    /** Stable platform id (MAC on Android). */
    val deviceId: String,
    /** Local name from the scan record, or null when absent. */
    val advertisedName: String?,
    val rssi: Int,
)

/**
 * Transport factory seam, carrying the reconnect policy (plan §3):
 *
 * - `autoConnect = false`: direct connect attempt — used for user connects
 *   and the timed ladder attempts against a recently-connected device.
 * - `autoConnect = true`: background/parked connect — the fleet requests it
 *   ONLY immediately after the device was sighted in a live scan (endgame
 *   RE-SCAN-THEN-autoConnect).  Implementations may trust that contract;
 *   the fleet never autoConnects blind from a stored MAC.
 */
public interface TransportFactory {
    public fun create(deviceId: String, autoConnect: Boolean): BleTransport
}

/**
 * Session factory seam: the fleet creates a NEW session object per
 * connection (iOS creates a fresh BLEDevice in didConnect) and closes it on
 * disconnect.  The concrete session type is the DeviceSession port; the
 * fleet treats it opaquely so the two layers can evolve independently.
 */
public interface FleetSessionFactory<S : Any> {
    /**
     * Build (and start) the per-connection session.  [seedFocusRocket] is the
     * fleet's sticky BS focus pin for this device (null when none): the
     * factory MUST seed it into the session BEFORE starting it (iOS
     * BLEFleet.adopt seeds `focusRocketID`; the session's connect
     * choreography then sends the single cmd 45 after cmd 20 — the fleet
     * itself never writes it, so exactly ONE push happens per (re)connect,
     * in the iOS order [9, 20, 45]).
     */
    public fun create(
        deviceId: String,
        advertisedName: String,
        generation: Int,
        transport: BleTransport,
        seedFocusRocket: Int?,
    ): S

    /** Teardown for a session whose connection dropped (iOS onDisconnect). */
    public fun close(session: S)
}

/**
 * One scan result row (iOS DiscoveredDevice minus the CBPeripheral).
 *
 * @property knownType best-effort registry type recovered from the
 *   advertised name at discovery time (#547) — null when unknown/ambiguous.
 */
public data class DiscoveredDevice(
    val deviceId: String,
    val name: String,
    val rssi: Int,
    val knownType: BleDeviceType?,
)

/**
 * A connected device as the fleet sees it: the per-connection session plus
 * fleet-tracked metadata.  Immutable snapshot — identity readbacks and
 * reconnects publish new copies through [FleetManager.devices].
 */
public data class FleetDevice<S : Any>(
    val deviceId: String,
    /** Scan-record name at connect time (never a stack-cached name, #547). */
    val advertisedName: String,
    /**
     * Connection generation for this deviceId (1 = first connect this
     * process, +1 per reconnect).  Replaces iOS object identity: a consumer
     * holding (deviceId, generation) can detect that its session object is
     * stale after a reconnect.
     */
    val generation: Int,
    val session: S,
    val transport: BleTransport,
    /** Precedence-resolved at connect (registry > prefix > legacy substring),
     *  overridden by the config_identity "dt" readback ~1 s later. */
    val deviceType: BleDeviceType,

    // Populated by the config_identity readback (empty/0 until then).
    val unitId: String = "",
    val unitName: String = "",
    val networkId: Int = 0,
    val rocketId: Int = 0,
    val firmwareVersion: String = "",
) {
    public val isBaseStation: Boolean get() = deviceType == BleDeviceType.BASE_STATION

    /** Display name: unitName if set, otherwise the advertised name. */
    public val displayName: String get() = unitName.ifEmpty { advertisedName }
}
