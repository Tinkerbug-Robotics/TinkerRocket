package com.tinkerbug.tinkerrocket.ble

import android.util.Log
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.GnssVelocityCheck
import com.tinkerbug.tinkerrocket.session.KnownDeviceStorage
import com.tinkerbug.tinkerrocket.session.KnownDeviceStore
import com.tinkerbug.tinkerrocket.session.LandingCast
import com.tinkerbug.tinkerrocket.session.RocketProfile
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
import kotlin.test.assertTrue

/**
 * #552 ON THE BENCH: `hacc` reaches the phone and actually moves the landing
 * radius, measured on a real rocket over a real link.
 *
 * The JVM tests pin the estimator and the wiring against synthetic samples.
 * What they cannot show is that the firmware puts `hacc` on the wire where the
 * decoder expects it, that it survives the link, and that a flight the phone
 * did not author drives the adaptive baseline. This flies the FC's own
 * simulation and reads the answer off the real telemetry stream.
 *
 * The sim reports `h_acc_m = 2` (TR_Sensor_Collector_Sim.cpp), which is the
 * interesting case rather than a trivial one: sqrt(2)*2.03/3 = 0.96 s of
 * wanted baseline, so the check must reach PAST the frame interval — exactly
 * the behaviour #552 added — without going anywhere near the 4 s cap.
 *
 * ```
 * ./gradlew :core:ble:connectedDebugAndroidTest \
 *   -Pandroid.testInstrumentationRunnerArguments.bench=1 \
 *   -Pandroid.testInstrumentationRunnerArguments.benchAddr=<MAC> \
 *   -Pandroid.testInstrumentationRunnerArguments.class=\
 * com.tinkerbug.tinkerrocket.ble.LandingRadiusBenchTest
 * adb logcat -d -s LandingRadius
 * ```
 */
@RunWith(AndroidJUnit4::class)
class LandingRadiusBenchTest {

    @get:Rule
    val permissions: androidx.test.rule.GrantPermissionRule =
        androidx.test.rule.GrantPermissionRule.grant(
            android.Manifest.permission.BLUETOOTH_SCAN,
            android.Manifest.permission.BLUETOOTH_CONNECT,
        )

    private val benchEnabled: Boolean
        get() = InstrumentationRegistry.getArguments().getString("bench") == "1"

    /** Bench runs never persist device registrations. */
    private class BenchStorage : KnownDeviceStorage {
        private var json: String? = null
        override fun loadDevicesJson(): String? = json
        override fun saveDevicesJson(json: String) { this.json = json }
        override fun loadLegacyKnownIds(): List<String>? = null
        override fun removeLegacyKnownIds() = Unit
    }

    private companion object {
        const val TAG = "LandingRadius"

        const val SCAN_TIMEOUT_MS = 20_000L
        const val TELEMETRY_TIMEOUT_MS = 15_000L
        const val FLIGHT_WATCH_MS = 60_000L
    }

    @Test
    fun haccReachesThePhoneAndMovesTheLandingRadius() {
        assumeTrue("bench flag not set — skipping radio test", benchEnabled)
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val dispatcher = Dispatchers.Default.limitedParallelism(1)
        val scope = CoroutineScope(SupervisorJob() + dispatcher)
        val profile = RocketProfile(name = "bench", createdAtMs = 0L, updatedAtMs = 0L)

        try {
            runBlocking(dispatcher) {
                val wantAddr = InstrumentationRegistry.getArguments().getString("benchAddr")
                Log.i(TAG, "scanning (benchAddr=${wantAddr ?: "any"})...")
                val adv = withTimeout(SCAN_TIMEOUT_MS) {
                    AndroidBleScanner(context).advertisements()
                        .first { wantAddr == null || it.deviceId.equals(wantAddr, ignoreCase = true) }
                }
                Log.i(TAG, "found ${adv.deviceId} name=${adv.advertisedName} rssi=${adv.rssi}")

                val transport = AndroidTransportFactory(context) { }
                    .create(adv.deviceId, autoConnect = false)
                transport.connect()
                val session = DeviceSession(
                    scope = scope, transport = transport,
                    connectedDeviceName = adv.advertisedName ?: "Unknown",
                    knownDevices = KnownDeviceStore(BenchStorage()),
                )
                session.start()
                assertNotNull(
                    withTimeoutOrNull(TELEMETRY_TIMEOUT_MS) { session.hasReceivedTelemetry.first { it } },
                    "no telemetry within $TELEMETRY_TIMEOUT_MS ms",
                )

                // ── 1. hacc is on the wire at all ────────────────────────────
                val idle = session.telemetry.value
                Log.i(TAG, "pre-flight: state=${idle.state} sats=${idle.numSats} hacc=${idle.gnssHAccM}")

                // ── 2. settle first ──────────────────────────────────────────
                // A previous run's sim may still be flying, and SIM_START is
                // refused INFLIGHT (and through the post-flight lockout). Left
                // unhandled that reads as "the flight never reached ascent"
                // rather than "there was already one in progress".
                if (idle.state == "INFLIGHT") {
                    Log.i(TAG, "a flight is already in progress — stopping it")
                    session.stopSimulation()
                }
                val settled = withTimeoutOrNull(90_000) {
                    session.telemetry.first { it.state == "READY" || it.state == "PRELAUNCH" }
                }
                assertNotNull(
                    settled,
                    "rocket never returned to READY/PRELAUNCH — last state ${session.telemetry.value.state}",
                )
                Log.i(TAG, "settled at ${session.telemetry.value.state}")
                delay(2_000)

                // ── 3. fly the FC's own simulation ───────────────────────────
                Log.i(TAG, "starting simulated flight...")
                session.startSimulation(
                    Commands.simConfig(
                        massGrams = 883f, thrustN = 220f, burnTimeS = 1.5f, descentRateMps = 5f,
                    ),
                    gapMs = if (session.isBaseStation) 1000L else 300L,
                ).join()

                // ── 4. run the real check over the real stream ───────────────
                var history = emptyList<GnssVelocityCheck.Sample>()
                var sawHacc = false
                var sawAdaptiveBaseline = false
                var frames = 0
                var ascentSamples = 0
                var maxSpread = 0.0
                var lastState = ""
                val deadline = System.currentTimeMillis() + FLIGHT_WATCH_MS

                // Wait for a DISTINCT frame each pass. `first {}` on a StateFlow
                // returns the current value when it already matches, so testing
                // against a fixed reference would spin and stuff the history with
                // duplicate positions -- which differences to zero movement and
                // would fake agreement.
                var last = idle
                while (System.currentTimeMillis() < deadline) {
                    val t = withTimeoutOrNull(3_000) { session.telemetry.first { it !== last } } ?: continue
                    last = t
                    frames++
                    if (t.state != lastState) {
                        Log.i(TAG, "state -> ${t.state}")
                        lastState = t.state ?: ""
                    }
                    val lat = t.latitude; val lon = t.longitude
                    val ve = t.velE; val vn = t.velN
                    if (lat == null || lon == null || ve == null || vn == null) continue
                    if (t.gnssHAccM != null) sawHacc = true

                    history = GnssVelocityCheck.trimmed(
                        history,
                        GnssVelocityCheck.Sample(
                            tMs = System.currentTimeMillis(), latDeg = lat, lonDeg = lon,
                            hAccM = t.gnssHAccM, velE = ve.toDouble(), velN = vn.toDouble(),
                        ),
                    )
                    val r = GnssVelocityCheck.evaluate(history) ?: continue

                    // A/B on the SAME real stream -- real positions, real
                    // velocities, real link timing -- with only hacc swapped.
                    // Needed because the sim reports a fixed h_acc_m = 2, which
                    // at this telemetry rate asks for less baseline than one
                    // frame interval already gives: the adaptive path has no
                    // room to act, so the received-hacc baseline alone proves
                    // nothing. 29 m is Rolly Polly V's real value, the case
                    // #552 exists for.
                    val poor = GnssVelocityCheck.evaluate(
                        history.dropLast(1) + history.last().copy(hAccM = 29),
                    )
                    if (poor != null && poor.baselineS > r.baselineS + 0.25) sawAdaptiveBaseline = true

                    val vu = t.velU?.toDouble() ?: 0.0
                    val ascending = t.launchFlag && t.burnoutFlag && !t.pastApogee && vu > 0.5
                    if (!ascending) continue
                    ascentSamples++
                    val altFt = LandingCast.let { _ ->
                        com.tinkerbug.tinkerrocket.session.DriftCast.mToFt((t.pressureAlt ?: 0f).toDouble())
                    }
                    val nominal = LandingCast.simulateAscentThenDescent(
                        startLat = lat, startLon = lon, currentAltAglFt = altFt,
                        velE = ve.toDouble(), velN = vn.toDouble(), velU = vu,
                        profile = profile, dragK = profile.ballisticDragK, wind = null,
                    ).first.lastOrNull()
                    val spread = LandingCast.ascentVelocitySpreadMeters(
                        startLat = lat, startLon = lon, currentAltAglFt = altFt,
                        velE = ve.toDouble(), velN = vn.toDouble(), velU = vu,
                        profile = profile, dragK = profile.ballisticDragK, wind = null,
                        nominalLanding = nominal, disagreementMps = r.disagreementMps,
                    )
                    if (spread > maxSpread) maxSpread = spread
                    Log.i(
                        TAG,
                        ("ASCENT hacc=${t.gnssHAccM} baseline=%.2fs noise=%.1f disagree=%.1f m/s " +
                            "-> spread=%.0f m   | same frames at hacc=29: baseline=%.2fs noise=%.1f")
                            .format(
                                r.baselineS, r.noiseFloorMps, r.disagreementMps, spread,
                                poor?.baselineS ?: Double.NaN, poor?.noiseFloorMps ?: Double.NaN,
                            ),
                    )
                    if (t.pastApogee) break
                }

                Log.i(
                    TAG,
                    "SUMMARY frames=$frames ascentSamples=$ascentSamples sawHacc=$sawHacc " +
                        "adaptiveBaseline=$sawAdaptiveBaseline maxSpread=%.0f m".format(maxSpread),
                )
                transport.disconnect()

                assertTrue(sawHacc, "no frame carried hacc — the firmware or the decoder dropped it")
                assertTrue(ascentSamples > 0, "the simulated flight never reached ascent")
                assertTrue(
                    sawAdaptiveBaseline,
                    "a 29 m hacc never lengthened the baseline on real telemetry — the adaptive " +
                        "path is inert, which is the whole of what #552 added",
                )
                Log.i(TAG, "#552 bench check PASSED")
            }
        } finally {
            scope.cancel()
        }
    }
}
