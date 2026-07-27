package com.tinkerbug.tinkerrocket.ble

import android.util.Log
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.KnownDeviceStorage
import com.tinkerbug.tinkerrocket.session.KnownDeviceStore
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.withTimeout
import kotlinx.coroutines.withTimeoutOrNull
import org.junit.Assume.assumeTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import kotlin.test.assertNotNull

/**
 * THE PHASE 2 BENCH SEAM — runs ON the Pixel against a REAL bench OC/BS.
 *
 * Deliberately gated: it does nothing unless launched with the bench flag, so
 * `connectedAndroidTest` in CI or a casual run never touches radios.  Run it
 * with a powered bench board in range:
 *
 * ```
 * ./gradlew :core:ble:connectedDebugAndroidTest \
 *   -Pandroid.testInstrumentationRunnerArguments.bench=1
 * ```
 *
 * What it proves (the items the 398 JVM tests cannot): the svc-UUID scan
 * filter finds real firmware, RealBleTransport's op queue survives the real
 * stack (MTU 517 negotiation, CCCD writes, the [9, 20(, 45)] choreography
 * timing), real telemetry decodes, config readback round-trips, and a clean
 * disconnect.  Results are logged under the "BenchSeam" tag —
 * `adb logcat -s BenchSeam` for the session record.
 */
@RunWith(AndroidJUnit4::class)
class BenchSeamTest {

    /** Instrumented tests cannot show permission dialogs — self-grant. */
    @get:Rule
    val permissions: androidx.test.rule.GrantPermissionRule =
        androidx.test.rule.GrantPermissionRule.grant(
            android.Manifest.permission.BLUETOOTH_SCAN,
            android.Manifest.permission.BLUETOOTH_CONNECT,
        )

    private val benchEnabled: Boolean
        get() = InstrumentationRegistry.getArguments().getString("bench") == "1"

    @Test
    fun scanConnectChoreographyTelemetryConfig() {
        assumeTrue("bench flag not set — skipping radio test", benchEnabled)
        val context = InstrumentationRegistry.getInstrumentation().targetContext

        // Single-writer dispatcher: the session contract (plan §3).
        val dispatcher = Dispatchers.Default.limitedParallelism(1)
        val scope = CoroutineScope(SupervisorJob() + dispatcher)
        val tap: (String) -> Unit = { Log.i(TAG, it) }

        try {
            runBlocking(dispatcher) {
                // 1. Scan: the svc-UUID filter must find the bench board.
                Log.i(TAG, "scanning for TinkerRocket service...")
                val adv = withTimeout(SCAN_TIMEOUT_MS) {
                    AndroidBleScanner(context).advertisements().first()
                }
                Log.i(TAG, "found ${adv.deviceId} name=${adv.advertisedName} rssi=${adv.rssi}")

                // 2. Connect (fleet-owned in production; direct here).
                val transport = AndroidTransportFactory(context, tap)
                    .create(adv.deviceId, autoConnect = false)
                transport.connect()
                Log.i(TAG, "connected + services discovered")

                // 3. Real DeviceSession over the real transport.
                val session = DeviceSession(
                    scope = scope,
                    transport = transport,
                    connectedDeviceName = adv.advertisedName ?: "Unknown",
                    knownDevices = KnownDeviceStore(EphemeralStorage()),
                )
                session.start()

                // 4. Telemetry within 15 s (boards stream continuously).
                val telemetryOk = withTimeoutOrNull(TELEMETRY_TIMEOUT_MS) {
                    session.hasReceivedTelemetry.first { it }
                }
                assertNotNull(telemetryOk, "no telemetry within ${TELEMETRY_TIMEOUT_MS} ms")
                Log.i(TAG, "telemetry: state=${session.telemetry.value.state}")

                // 5. Config readback (cmd 20 fires at +1 s in the choreography).
                val cfg = withTimeoutOrNull(CONFIG_TIMEOUT_MS) {
                    session.rocketConfig.first { it != null }
                }
                assertNotNull(cfg, "no config readback within ${CONFIG_TIMEOUT_MS} ms")
                Log.i(TAG, "config: loraFreq=${cfg?.loraFreqMHz} identity=${session.identity.value}")

                // 6. RSSI ticker alive (2 s cadence).
                delay(4_500)
                val rssi = session.connectedRssi.value
                assertNotNull(rssi, "RSSI ticker produced nothing")
                Log.i(TAG, "rssi=$rssi")

                // 7. Clean disconnect: the async Disconnected event must land.
                transport.disconnect()
                val disconnected = withTimeoutOrNull(5_000) {
                    session.isConnected.first { !it }
                }
                assertNotNull(disconnected, "no Disconnected event after disconnect()")
                Log.i(TAG, "clean disconnect — bench seam PASSED")
            }
        } finally {
            scope.cancel()
        }
    }

    /**
     * DIAGNOSTIC: unfiltered 10 s scan logging everything in range —
     * distinguishes "board not advertising" / "advertising without the
     * service UUID" / "nothing heard at all".  Same bench gate.
     */
    @Test
    fun diagUnfilteredScan() {
        assumeTrue("bench flag not set", benchEnabled)
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val manager = context.getSystemService(android.content.Context.BLUETOOTH_SERVICE)
            as android.bluetooth.BluetoothManager
        val scanner = manager.adapter?.bluetoothLeScanner ?: error("adapter off")
        val seen = java.util.concurrent.ConcurrentHashMap<String, String>()
        val cb = object : android.bluetooth.le.ScanCallback() {
            override fun onScanResult(t: Int, r: android.bluetooth.le.ScanResult) {
                val uuids = r.scanRecord?.serviceUuids?.joinToString() ?: "-"
                seen[r.device.address] =
                    "name=${r.scanRecord?.deviceName ?: "-"} rssi=${r.rssi} uuids=$uuids"
            }
        }
        scanner.startScan(
            null,
            android.bluetooth.le.ScanSettings.Builder()
                .setScanMode(android.bluetooth.le.ScanSettings.SCAN_MODE_LOW_LATENCY).build(),
            cb,
        )
        Thread.sleep(10_000)
        scanner.stopScan(cb)
        Log.i(TAG, "DIAG: ${seen.size} distinct devices heard")
        seen.forEach { (addr, info) -> Log.i(TAG, "DIAG: $addr $info") }
    }

    /** Bench runs never persist device registrations. */
    private class EphemeralStorage : KnownDeviceStorage {
        private var json: String? = null
        override fun loadDevicesJson(): String? = json
        override fun saveDevicesJson(json: String) { this.json = json }
        override fun loadLegacyKnownIds(): List<String>? = null
        override fun removeLegacyKnownIds() = Unit
    }

    private companion object {
        const val TAG = "BenchSeam"
        const val SCAN_TIMEOUT_MS = 20_000L
        const val TELEMETRY_TIMEOUT_MS = 15_000L
        const val CONFIG_TIMEOUT_MS = 15_000L
    }
}
