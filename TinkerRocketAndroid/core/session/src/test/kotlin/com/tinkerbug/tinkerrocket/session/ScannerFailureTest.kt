package com.tinkerbug.tinkerrocket.session

import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * #830: a scanner failure must not escape the fleet scope.
 *
 * `scan()` has caught this since the 2026-07-29 bench, where tapping Scan with
 * Bluetooth off crashed the app outright ("APP CRASH(EXCEPTION)"). Two other
 * call sites collected the same flow with no guard: [FleetManager.resumeLastSession]
 * — where `withTimeoutOrNull` only converts TimeoutCancellationException, not a
 * BleTransportException — and the reconnect ladder's endgame sighting-wait,
 * which had no timeout at all.
 *
 * `AndroidBleScanner` closes its flow with a BleTransportException when the
 * adapter is off (`bluetoothLeScanner` is null) and on any `onScanFailed`,
 * including the SCAN_FAILED_SCANNING_TOO_FREQUENTLY that the endgame loop's own
 * per-iteration scan provokes. A plain Exception out of `scope.launch` reaches
 * Android's default handler and kills the process — with the rocket airborne,
 * in the endgame case.
 *
 * These run on a TestScope: an exception escaping a child coroutine fails the
 * test, which is exactly the "it killed the process" signal.
 */
class ScannerFailureTest {

    private class AdapterOff : Exception("BLE scanner unavailable (adapter off?)")

    // ------------------------------------------------------- resumeLastSession

    @Test
    fun resumeSurvivesAnAdapterOffScanner() = runTest {
        val h = fleetHarness()
        h.lastSession.saveLastConnected("aa:01", "TR-R-Atlas")
        h.scanner.failWith = AdapterOff()

        h.fleet.resumeLastSession()
        runCurrent()

        // Reaching here at all is the assertion: an escaping exception would
        // have failed this test the way it kills the app.
        assertTrue(h.fleet.devices.value.isEmpty())
        assertEquals(
            "BLE scanner unavailable (adapter off?)",
            h.fleet.statusMessage.value,
            "the failure should be reported, not swallowed silently",
        )
    }

    @Test
    fun resumeStillReportsNotFoundWhenTheScannerIsHealthy() = runTest {
        val h = fleetHarness()
        h.lastSession.saveLastConnected("aa:01", "TR-R-Atlas")

        h.fleet.resumeLastSession()
        advanceTimeBy(FleetManager.RESUME_SIGHTING_TIMEOUT_MS + 1_000)
        runCurrent()

        assertEquals("TR-R-Atlas not found", h.fleet.statusMessage.value)
    }

    // ------------------------------------------------------------- the endgame

    @Test
    fun endgameSurvivesAScannerFailureAndKeepsTrying() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.transports.failAllConnects = true

        // Adapter is down before the endgame opens its sighting-wait.
        h.scanner.failWith = AdapterOff()
        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()

        // Walk past the 8 direct rungs, into the endgame and around its loop
        // many times — every iteration's collection fails at onStart.
        advanceTimeBy(120_000)
        runCurrent()

        // Survived — and the ladder is still the thing holding the link open,
        // rather than having been silently killed along with the process.
        assertTrue(
            h.fleet.linkActive.value,
            "a scanner failure must not abandon a flight in progress",
        )
    }

    @Test
    fun endgameRecoversOnceTheAdapterComesBack() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.transports.failAllConnects = true
        h.scanner.failWith = AdapterOff()
        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()
        advanceTimeBy(120_000)
        runCurrent()

        // Adapter back, device advertising, connects succeed again.
        h.scanner.failWith = null
        h.transports.failAllConnects = false
        advanceTimeBy(FleetManager.ENDGAME_RETRY_DELAY_MS + 1_000)
        runCurrent()
        h.scanner.emissions.emit(BleAdvertisement("aa:01", "TR-R-Atlas", -40))
        advanceTimeBy(1_000)
        runCurrent()

        assertTrue(
            h.fleet.devices.value.containsKey("aa:01"),
            "the endgame should reconnect once the scanner is healthy again",
        )
    }

    /**
     * The loop must not spin: each iteration opens its own platform scan, and
     * >5 starts per 30 s is what trips SCAN_FAILED_SCANNING_TOO_FREQUENTLY in
     * the first place.
     */
    @Test
    fun endgamePacesItsRetriesWhileTheScannerIsFailing() = runTest {
        val h = fleetHarness()
        discoverAndConnect(h, "aa:01", "TR-R-Atlas")
        h.transports.failAllConnects = true
        h.scanner.failWith = AdapterOff()
        h.transports.lastFor("aa:01").dropUnexpectedly()
        runCurrent()
        // Past the direct rungs, into the endgame.
        advanceTimeBy(60_000)
        runCurrent()

        val before = h.scanner.totalCollections
        advanceTimeBy(30_000)
        runCurrent()
        val attempts = h.scanner.totalCollections - before

        assertTrue(
            attempts <= 5,
            "at most ~4 scan starts per 30 s; saw $attempts (a spin would be thousands)",
        )
        assertFalse(attempts == 0, "it must still be retrying, not stalled")
    }
}
