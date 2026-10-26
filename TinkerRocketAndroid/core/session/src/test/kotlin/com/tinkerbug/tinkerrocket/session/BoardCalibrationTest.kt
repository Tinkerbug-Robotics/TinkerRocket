package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * #1629: what a flight computer holds for calibration, per board — the
 * wording ([BoardCalibration]) and the session that reads it on connect and
 * folds the board's answers in, then into the registry.
 */
class BoardCalibrationTest {

    // ── The record and its wording ───────────────────────────────────────

    @Test
    fun notReadIsNeverNotCalibrated() {
        val none = BoardCalibration()
        assertTrue(none.missing.isEmpty())
        assertNull(none.notCalibratedLine)
        assertEquals("Calibration not read yet", none.summary)
        assertEquals("Not read yet", BoardCalibration.rowText(null))
    }

    @Test
    fun missingNamesEachSensorInOrder() {
        val c = BoardCalibration(mag = false, accel = false, gyro = false, highGAligned = false)
        assertEquals("Not calibrated: magnetometer, accelerometer, gyro", c.notCalibratedLine)
        assertEquals(c.notCalibratedLine, c.summary)
        assertEquals(
            "Not calibrated: accelerometer",
            BoardCalibration(mag = true, accel = false, gyro = true).notCalibratedLine,
        )
    }

    @Test
    fun theHighGAlignmentIsNeverMissing() {
        val c = BoardCalibration(mag = true, accel = true, gyro = true, highGAligned = false)
        assertNull(c.notCalibratedLine)
        assertTrue(c.fullyCalibrated)
        assertEquals("Calibrated", c.summary)
        assertEquals("Not aligned (flies on its bias)", BoardCalibration.highGRowText(false))
    }

    @Test
    fun partlyReadSaysSo() {
        assertEquals("Calibration not fully read", BoardCalibration(mag = true, accel = true).summary)
    }

    @Test
    fun aReadingChangesOnlyWhatItAnswers() {
        val old = BoardCalibration(mag = true, accel = false, gyro = true, highGAligned = false, readAtMs = 1)
        val now = old.merged(BoardCalibration(accel = true, highGAligned = true, readAtMs = 2))
        assertEquals(BoardCalibration(mag = true, accel = true, gyro = true, highGAligned = true, readAtMs = 2), now)
    }

    // ── The session ──────────────────────────────────────────────────────

    private class Storage : KnownDeviceStorage {
        var json: String? = null
        override fun loadDevicesJson(): String? = json
        override fun saveDevicesJson(json: String) { this.json = json }
        override fun loadLegacyKnownIds(): List<String>? = null
        override fun removeLegacyKnownIds() = Unit
    }

    private fun TestScope.link(
        dt: String,
        store: KnownDeviceStore? = null,
        fs: Int? = null,
    ): Pair<DeviceSession, FakeFirmware> {
        val fw = FakeFirmware(backgroundScope).apply {
            configJson = null
            configPyroJson = null
            configIdentityJson =
                """{"type":"config_identity","uid":"a1b2c3d4","un":"Atlas","nid":5,"rid":1,"dt":"$dt"}"""
            pushEchoWithConfig = false
        }
        val session = DeviceSession(
            scope = backgroundScope,
            transport = fw,
            connectedDeviceName = "TR-test",
            clock = { currentTime },
            knownDevices = store,
        )
        session.start()
        runCurrent()
        fs?.let { fw.emitTelemetryJson("""{"st":"READY","fs":$it}""") }
        advanceTimeBy(1100)
        runCurrent()
        return session to fw
    }

    private fun FakeFirmware.reads(): List<Int> =
        commandFrames.map { it[0].toInt() and 0xFF }.filter {
            it == BleCommandId.MAG_CAL_READ_OC || it == BleCommandId.SENSOR_CAL_READ_OC ||
                it == BleCommandId.ACC_CAL_READ
        }

    /** 0xCA: MagCalStatusData, sub_type at payload byte 4. */
    private fun FakeFirmware.mag(subType: Int) =
        emitFileOpsFrame(ByteArray(1 + 36).also { it[0] = 0xCA.toByte(); it[1 + 4] = subType.toByte() })

    /** 0xCB: SensorCalStatusData, valid first. */
    private fun FakeFirmware.sensor(valid: Boolean) =
        emitFileOpsFrame(ByteArray(1 + 19).also { it[0] = 0xCB.toByte(); it[1] = if (valid) 1 else 0 })

    /** 0xD1: AccelCalStatusData flags first; the high-g tail's flags 24 bytes in. */
    private fun FakeFirmware.accel(stored: Boolean, hgStored: Boolean) =
        emitFileOpsFrame(
            ByteArray(1 + 40 + 74).also {
                it[0] = 0xD1.toByte()
                it[1] = if (stored) 0x03 else 0
                it[1 + 40 + 24] = if (hgStored) 0x07 else 0x04
            },
        )

    @Test
    fun aRocketIsReadOnConnect_aBaseStationIsNot() = runTest {
        val (_, rocket) = link("R")
        assertEquals(listOf(61, 63, 77), rocket.reads(), "one read of each, once the role is known")
        val (_, bs) = link("B")
        assertTrue(bs.reads().isEmpty(), "a base station has nothing to calibrate")
    }

    @Test
    fun theBoardsAnswersFillTheRecord() = runTest {
        val (session, fw) = link("R")
        fw.mag(subType = 3)               // APPLIED
        fw.sensor(valid = false)
        fw.accel(stored = true, hgStored = false)
        runCurrent()
        val c = session.calibration.value
        assertEquals(true, c.mag)
        assertEquals(false, c.gyro)
        assertEquals(true, c.accel)
        assertEquals(false, c.highGAligned)
        assertEquals("Not calibrated: gyro", c.notCalibratedLine)
        assertNotNull(c.readAtMs)
    }

    @Test
    fun anIdleMagFrameIsAnAnswerOnlyRightAfterTheRead() = runTest {
        val (session, fw) = link("R")
        fw.mag(subType = 0)               // IDLE, answering the connect read
        runCurrent()
        assertEquals(false, session.calibration.value.mag)

        fw.mag(subType = 3)               // a calibration stored
        runCurrent()
        assertEquals(true, session.calibration.value.mag)

        advanceTimeBy(DeviceSession.CAL_READ_WINDOW_MS + 1)
        fw.mag(subType = 0)               // a session ending, long after any read
        runCurrent()
        assertEquals(true, session.calibration.value.mag, "an IDLE nobody asked for says nothing")
    }

    @Test
    fun railOnAsksAgainForWhatIsStillUnanswered() = runTest {
        val (session, fw) = link("R", fs = 0)
        assertEquals(3, fw.reads().size)
        fw.emitTelemetryJson("""{"st":"READY","fs":16}""")
        runCurrent()
        assertEquals(6, fw.reads().size, "nothing came back before the rail came up")

        fw.mag(3); fw.sensor(true); fw.accel(true, true)
        fw.emitTelemetryJson("""{"st":"READY","fs":0}""")
        fw.emitTelemetryJson("""{"st":"READY","fs":16}""")
        runCurrent()
        assertEquals(6, fw.reads().size, "all answered: no need to ask again")
        assertTrue(session.calibration.value.fullyCalibrated)
    }

    @Test
    fun eachAnswerIsKeptOnTheBoardsRecord() = runTest {
        val storage = Storage()
        val store = KnownDeviceStore(storage)
        val (_, fw) = link("R", store = store)
        fw.mag(3)
        fw.accel(stored = false, hgStored = false)
        runCurrent()

        val rec = assertNotNull(store.device("a1b2c3d4"))
        assertEquals(BoardCalibration(mag = true, accel = false, highGAligned = false, readAtMs = currentTime), rec.calibration)

        // And it survives a restart of the app.
        val reloaded = KnownDeviceStore(storage).device("a1b2c3d4")?.calibration
        assertEquals(rec.calibration, reloaded)
        assertFalse(reloaded!!.fullyCalibrated)
    }

    @Test
    fun aRelayedRocketsRecordIsFoundByNetworkAndRocketId() {
        val a = KnownDevice(unitID = "a", name = "Atlas", deviceType = BleDeviceType.ROCKET, networkID = 5, rocketID = 1)
        val b = KnownDevice(unitID = "b", name = "Twin", deviceType = BleDeviceType.ROCKET, networkID = 5, rocketID = 1)
        val bs = KnownDevice(unitID = "c", deviceType = BleDeviceType.BASE_STATION, networkID = 5, rocketID = 1)
        assertEquals("a", KnownDeviceStore.rocketIn(listOf(a, bs), 5, 1)?.unitID)
        assertNull(KnownDeviceStore.rocketIn(listOf(a, b), 5, 1), "two boards on one ID: no guess")
        assertEquals("b", KnownDeviceStore.rocketIn(listOf(a, b), 5, 1, name = "Twin")?.unitID)
    }
}
