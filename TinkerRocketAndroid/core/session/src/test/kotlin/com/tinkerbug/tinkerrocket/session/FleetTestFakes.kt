package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.onCompletion
import kotlinx.coroutines.flow.onStart
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.runCurrent
import java.io.IOException

/**
 * Shared fakes for the FleetManager test suite.  Everything is confined to
 * the runTest scheduler (single-writer, virtual time).
 */

/** Scan seam fake: a hot shared flow + a collector ref-count so tests can
 *  assert "a scan is running" without touching platform APIs. */
class FakeScanner : BleScanner {
    val emissions = MutableSharedFlow<BleAdvertisement>(extraBufferCapacity = 64)
    var activeCollections = 0
        private set

    /**
     * Set to make the flow FAIL instead of emitting — what the real scanner
     * does when the adapter is off ("BLE scanner unavailable"). Left uncaught
     * this used to kill the process (Checkpoint A Group 1).
     */
    var failWith: Throwable? = null

    override fun advertisements(): Flow<BleAdvertisement> = emissions
        .onStart { activeCollections++; failWith?.let { throw it } }
        .onCompletion { activeCollections-- }
}

/** Transport fake: scripted connect success/failure, recorded writes, and a
 *  manual disconnect-event trigger. */
class FakeTransport(val deviceId: String) : BleTransport {
    private val eventFlow = MutableSharedFlow<TransportEvent>(extraBufferCapacity = 16)
    override val events: Flow<TransportEvent> = eventFlow

    var failConnect = false
    var connectCalls = 0
    val writes = mutableListOf<Pair<TrCharacteristic, ByteArray>>()
    var disconnectCalls = 0

    override suspend fun connect() {
        connectCalls++
        if (failConnect) throw IOException("scripted connect failure")
    }

    override suspend fun requestMtu(target: Int): Int = target
    override suspend fun enableNotifications(char: TrCharacteristic) {}
    override suspend fun write(char: TrCharacteristic, bytes: ByteArray, withResponse: Boolean) {
        writes += char to bytes
    }
    override suspend fun read(char: TrCharacteristic): ByteArray = ByteArray(0)
    override suspend fun readRssi(): Int = -50

    override fun disconnect() {
        disconnectCalls++
        // The real transport surfaces the resulting GATT callback as an
        // event; reason 0 = local/clean.
        eventFlow.tryEmit(TransportEvent.Disconnected(reason = 0))
    }

    /** Simulate an unexpected link drop (remote timeout etc.). */
    fun dropUnexpectedly(reason: Int = 8) {
        eventFlow.tryEmit(TransportEvent.Disconnected(reason))
    }
}

/** Factory fake: records every (deviceId, autoConnect, virtualTime) request
 *  and scripts how many upcoming connects fail. */
class FakeTransportFactory(private val currentTime: () -> Long) : TransportFactory {
    data class Call(val deviceId: String, val autoConnect: Boolean, val atMillis: Long)

    val calls = mutableListOf<Call>()
    val created = mutableListOf<FakeTransport>()

    /** Next N created transports fail their connect(). */
    var failNextConnects = 0

    /** Every created transport fails connect() while true. */
    var failAllConnects = false

    override fun create(deviceId: String, autoConnect: Boolean): BleTransport {
        calls += Call(deviceId, autoConnect, currentTime())
        val t = FakeTransport(deviceId)
        if (failAllConnects) {
            t.failConnect = true
        } else if (failNextConnects > 0) {
            failNextConnects--
            t.failConnect = true
        }
        created += t
        return t
    }

    /** The transport backing the currently live connection for [deviceId]. */
    fun lastFor(deviceId: String): FakeTransport =
        created.last { it.deviceId == deviceId }
}

/** Opaque per-connection session stand-in (the DeviceSession port slots in
 *  behind the same FleetSessionFactory seam). */
class FakeSession(val deviceId: String, val generation: Int, val seededFocus: Int? = null)

class FakeSessionFactory : FleetSessionFactory<FakeSession> {
    val created = mutableListOf<FakeSession>()
    val closed = mutableListOf<FakeSession>()

    override fun create(
        deviceId: String,
        advertisedName: String,
        generation: Int,
        transport: BleTransport,
        seedFocusRocket: Int?,
    ): FakeSession = FakeSession(deviceId, generation, seedFocusRocket).also { created += it }

    override fun close(session: FakeSession) {
        closed += session
    }
}

class InMemoryKnownStorage : KnownDeviceStorage {
    private var devicesJson: String? = null
    private var legacyIds: List<String>? = null
    override fun loadDevicesJson(): String? = devicesJson
    override fun saveDevicesJson(json: String) { devicesJson = json }
    override fun loadLegacyKnownIds(): List<String>? = legacyIds
    override fun removeLegacyKnownIds() { legacyIds = null }
}

class FleetHarness(scope: CoroutineScope, currentTime: () -> Long) {
    val scanner = FakeScanner()
    val transports = FakeTransportFactory(currentTime)
    val sessions = FakeSessionFactory()
    val knownDevices = KnownDeviceStore(InMemoryKnownStorage(), nowEpochMillis = { 0L })
    /** #633 resume hint; exposed so tests can seed or inspect it. */
    val lastSession = LastSessionStore.InMemory()
    val fleet: FleetManager<FakeSession> = FleetManager(
        scope = scope,
        scanner = scanner,
        transportFactory = transports,
        sessionFactory = sessions,
        knownDevices = knownDevices,
        nowMillis = currentTime,
        lastSession = lastSession,
    )
}

/** Harness on the runTest scheduler: backgroundScope = the fleet's single
 *  dispatcher, virtual time = the injected clock. */
fun TestScope.fleetHarness(): FleetHarness =
    FleetHarness(backgroundScope) { testScheduler.currentTime }

/** Scan → advertise → connect, the standard way a device joins the fleet. */
suspend fun TestScope.discoverAndConnect(
    h: FleetHarness,
    deviceId: String,
    name: String?,
    userInitiatedScan: Boolean = false,
) {
    h.fleet.scan(userInitiated = userInitiatedScan)
    runCurrent()
    h.scanner.emissions.emit(BleAdvertisement(deviceId, name, rssi = -40))
    runCurrent()
    h.fleet.connect(deviceId)
    runCurrent()
}
