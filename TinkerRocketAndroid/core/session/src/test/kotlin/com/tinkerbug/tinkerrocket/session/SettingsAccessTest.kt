package com.tinkerbug.tinkerrocket.session

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.boolean
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import java.io.File
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue
import kotlin.test.fail

/**
 * Which settings the Settings screen offers while the flight computer is off,
 * booting, stuck or in rescue mode.  Reported 2026-10-16: an FC parked on
 * "Starting sensors" locked iOS Settings, Firmware included, behind an
 * "Initializing…" overlay; Android locked its whole Settings screen the same
 * way, the out computer's own radio and network settings with it.  iOS twin:
 * SettingsAccessTests.swift, which runs the same shared fixture.
 */
class SettingsAccessTest {

    private fun fixture(): JsonObject {
        val f = File(System.getProperty("tr.appBehaviorFixtures") ?: "")
            .resolve("settings_access.json")
        if (!f.isFile) fail("missing shared fixture at ${f.absolutePath}")
        return Json.parseToJsonElement(f.readText()) as JsonObject
    }

    private fun fixtureName(a: SettingsAccess): String = when (a) {
        SettingsAccess.FULL -> "full"
        SettingsAccess.ROCKET_OFF -> "rocketOff"
        SettingsAccess.FLIGHT_COMPUTER_NOT_RUNNING -> "flightComputerNotRunning"
    }

    @Test
    fun matchesTheSharedFixture() {
        val cases = fixture()["cases"]!!.jsonArray
        assertTrue(cases.size >= 10, "the fixture lost its cases")
        for (c in cases) {
            val o = c.jsonObject
            fun b(k: String) = o[k]!!.jsonPrimitive.boolean
            val name = o["name"]!!.jsonPrimitive.content
            val got = settingsAccess(
                isConnected = b("connected"),
                isBaseStation = b("baseStation"),
                hasReceivedTelemetry = b("telemetry"),
                pwrPinOn = b("railOn"),
                state = o["state"]!!.jsonPrimitive.content,
                bootStepReported = b("bootStep"),
            )
            assertEquals(o["expect"]!!.jsonPrimitive.content, fixtureName(got), name)
        }
    }

    @Test
    fun aStuckBootLocksOnlyTheFlightComputersSettings() {
        val a = settingsAccess(
            isConnected = true, isBaseStation = false, hasReceivedTelemetry = true,
            pwrPinOn = true, state = "INITIALIZATION", bootStepReported = true,
        )
        assertEquals(SettingsAccess.FLIGHT_COMPUTER_NOT_RUNNING, a)
        assertFalse(a.allowsFlightComputerSettings)
        assertTrue(a.allowsOutComputerSettings, "radio, network ID and firmware stay")
    }

    @Test
    fun theOutComputersSettingsAreNeverLocked() {
        for (a in SettingsAccess.entries) {
            assertTrue(a.allowsOutComputerSettings, a.name)
        }
        assertEquals(listOf(SettingsAccess.FULL),
            SettingsAccess.entries.filter { it.allowsFlightComputerSettings })
    }

    @Test
    fun theNoticeNamesWhatTheFlightComputerIsDoing() {
        // Same titles as iOS FlightComputerNotRunningSection.title.
        assertEquals("Waiting for flight computer", FlightComputerNotice.title(false, false))
        assertEquals("Flight computer starting", FlightComputerNotice.title(true, false))
        assertEquals("Flight computer in rescue mode", FlightComputerNotice.title(true, true))
        assertEquals("Flight computer not running", FlightComputerNotice.title(true, false, true))
    }

    @Test
    fun theNoticeSaysTheWayOut() {
        val rescue = FlightComputerNotice.explanation(true, true)
        assertTrue(rescue.contains("Update the flight computer from Firmware below"), rescue)
        assertTrue(rescue.contains("power-cycle"), rescue)
        val starting = FlightComputerNotice.explanation(true, false)
        assertTrue(starting.contains("whether the flight computer answered"), starting)
        // The app's update cannot reach a flight computer with no flight loop
        // (the loop serves the update relay), so it must not point there.
        val noLoop = FlightComputerNotice.explanation(true, false, true)
        assertTrue(noLoop.contains("over USB"), noLoop)
        assertTrue(noLoop.contains("can't reach it"), noLoop)
        for (s in listOf(starting, FlightComputerNotice.explanation(false, false))) {
            assertTrue(s.contains("radio, network ID and firmware below"), s)
        }
    }
}
