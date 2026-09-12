package com.tinkerbug.tinkerrocket.ble

import android.util.Log
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import org.junit.Assume.assumeTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * #863 — ON the Pixel, against the real platform scanner.
 *
 * `BleScanner`'s contract says implementations must ref-count the platform
 * scan. `AndroidBleScanner` did not: every collector got its own `startScan`
 * and its own `ScanCallback`. Android throttles an app that starts more than
 * 5 scans per 30 s, so the violation manufactured the very
 * SCAN_FAILED_SCANNING_TOO_FREQUENTLY it then had to survive.
 *
 * This cannot be tested against FakeScanner: the thing being fixed is how many
 * times the PLATFORM is asked to scan. AndroidBleScanner therefore exposes
 * platformScanStarts / platformScanStops / activeCollectors, and this walks the
 * concurrency the two real call sites produce — a UI scan plus a sighting-wait.
 *
 * Needs BLE on; does NOT need a bench board in range (it counts scan
 * lifecycle, not sightings). Gated like BenchSeamTest so CI never touches
 * radios:
 *
 * ```
 * ./gradlew :core:ble:connectedDebugAndroidTest \
 *   -Pandroid.testInstrumentationRunnerArguments.bench=1
 * ```
 */
@RunWith(AndroidJUnit4::class)
class ScanRefCountBenchTest {

    @get:Rule
    val permissions: androidx.test.rule.GrantPermissionRule =
        androidx.test.rule.GrantPermissionRule.grant(
            android.Manifest.permission.BLUETOOTH_SCAN,
            android.Manifest.permission.BLUETOOTH_CONNECT,
        )

    private val benchEnabled: Boolean
        get() = InstrumentationRegistry.getArguments().getString("bench") == "1"

    private companion object { const val TAG = "ScanRefCount" }

    @Test
    fun concurrentCollectorsShareOnePlatformScan() {
        assumeTrue("bench flag not set — skipping radio test", benchEnabled)
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val scanner = AndroidBleScanner(context)
        val scope = CoroutineScope(SupervisorJob() + Dispatchers.Default)

        runBlocking {
            // 1. First collector — the UI scan. Starts the platform scan.
            val uiScan = scope.launch { scanner.advertisements().collect { } }
            delay(1500)
            assertEquals(1, scanner.platformScanStarts, "first collector must start one scan")
            assertEquals(1, scanner.activeCollectors)
            Log.i(TAG, "1 collector: starts=${scanner.platformScanStarts}")

            // 2. Second collector — the endgame sighting-wait, concurrent with
            //    it. THE regression: this used to be a second startScan.
            val sighting = scope.launch { scanner.advertisements().collect { } }
            delay(1500)
            assertEquals(1, scanner.platformScanStarts, "a second collector must NOT start a second scan")
            assertEquals(2, scanner.activeCollectors)
            Log.i(TAG, "2 collectors: starts=${scanner.platformScanStarts} (shared)")

            // 3. Cancelling ONE must not stop the platform scan — the other is
            //    still collecting and used to lose its scan to this.
            sighting.cancelAndJoin()
            delay(800)
            assertEquals(0, scanner.platformScanStops, "cancelling one collector must not stop the scan")
            assertEquals(1, scanner.activeCollectors)
            Log.i(TAG, "after 1 cancel: stops=${scanner.platformScanStops} collectors=${scanner.activeCollectors}")

            // 4. Cancelling the LAST one must stop it.
            uiScan.cancelAndJoin()
            delay(800)
            assertEquals(1, scanner.platformScanStops, "the last collector must stop the scan")
            assertEquals(0, scanner.activeCollectors)
            Log.i(TAG, "after last cancel: stops=${scanner.platformScanStops}")

            // 5. And the scanner is reusable afterwards — a later scan starts a
            //    fresh platform scan rather than finding a stale callback.
            val again = scope.launch { scanner.advertisements().collect { } }
            delay(1200)
            assertEquals(2, scanner.platformScanStarts, "a later collector must start a fresh scan")
            again.cancelAndJoin()
            delay(500)
            assertEquals(2, scanner.platformScanStops)
        }
        scope.cancel()
    }

    /**
     * Adapter off: every collector must still be closed with
     * [BleTransportException], and no platform scan may be left behind.
     *
     * Run with BLE OFF — the other two need it on, so this is its own gate:
     *
     * ```
     * adb shell svc bluetooth disable
     * ./gradlew :core:ble:connectedDebugAndroidTest \
     *   -Pandroid.testInstrumentationRunnerArguments.bench=1 \
     *   -Pandroid.testInstrumentationRunnerArguments.btOff=1
     * adb shell svc bluetooth enable
     * ```
     */
    @Test
    fun adapterOffClosesEveryCollectorAndStartsNoScan() {
        assumeTrue("bench flag not set", benchEnabled)
        assumeTrue(
            "btOff flag not set — this one needs the adapter OFF",
            InstrumentationRegistry.getArguments().getString("btOff") == "1",
        )
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val scanner = AndroidBleScanner(context)

        runBlocking {
            val errors = mutableListOf<Throwable>()
            repeat(2) {
                runCatching { scanner.advertisements().collect { } }
                    .onFailure { e -> errors.add(e) }
            }
            Log.i(TAG, "adapter off: errors=${errors.map { it::class.simpleName }} " +
                       "starts=${scanner.platformScanStarts} collectors=${scanner.activeCollectors}")
            assertEquals(2, errors.size, "both collectors must be closed with an error")
            assertTrue(errors.all { it is BleTransportException },
                       "must be BleTransportException, got ${errors.map { it::class.simpleName }}")
            assertEquals(0, scanner.platformScanStarts, "no platform scan may be started with the adapter off")
            assertEquals(0, scanner.activeCollectors, "a refused collector must not stay registered")
        }
    }

    /**
     * The throttle this issue is about: Android allows 5 scan starts per 30 s.
     * The reconnect endgame collects once per loop iteration, so the old
     * implementation doubled every one of those. With the scan shared, holding
     * one long-lived collector across many short-lived ones costs ONE start.
     */
    @Test
    fun manySequentialWaitsUnderOneHeldScanCostOneStart() {
        assumeTrue("bench flag not set — skipping radio test", benchEnabled)
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val scanner = AndroidBleScanner(context)
        val scope = CoroutineScope(SupervisorJob() + Dispatchers.Default)

        runBlocking {
            val held = scope.launch { scanner.advertisements().collect { } }
            delay(1000)
            assertEquals(1, scanner.platformScanStarts)

            // Eight sighting-waits in a row — comfortably past the 5-per-30 s
            // throttle if each were its own scan.
            repeat(8) { i ->
                val w = scope.launch { scanner.advertisements().collect { } }
                delay(300)
                w.cancelAndJoin()
                Log.i(TAG, "wait $i done: starts=${scanner.platformScanStarts}")
            }
            assertEquals(1, scanner.platformScanStarts, "8 sequential waits under a held scan must not re-start it")
            assertEquals(0, scanner.platformScanStops, "the held collector keeps the scan alive throughout")

            held.cancelAndJoin()
            delay(500)
            assertEquals(1, scanner.platformScanStops)
            assertTrue(scanner.activeCollectors == 0)
        }
        scope.cancel()
    }
}
