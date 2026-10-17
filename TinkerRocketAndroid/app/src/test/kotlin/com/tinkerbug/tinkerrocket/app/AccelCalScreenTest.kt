package com.tinkerbug.tinkerrocket.app

import androidx.compose.ui.test.assertIsEnabled
import androidx.compose.ui.test.assertCountEquals
import androidx.compose.ui.test.assertIsNotEnabled
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onAllNodesWithText
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performScrollTo
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.tinkerbug.tinkerrocket.protocol.AccelCalStatus
import com.tinkerbug.tinkerrocket.protocol.AccelFaceState
import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.LeBuffer
import com.tinkerbug.tinkerrocket.session.AccelSixFace
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Face
import com.tinkerbug.tinkerrocket.session.BleDeviceType
import com.tinkerbug.tinkerrocket.session.BleTransport
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.TrCharacteristic
import com.tinkerbug.tinkerrocket.session.TransportEvent
import java.util.concurrent.CopyOnWriteArrayList
import kotlin.math.roundToInt
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith

/**
 * The six-face accelerometer calibration screen (#1629), composed over a real
 * session on a link that answers commands 76/77/78 the way the flight computer
 * does: the opening read, six faces in any order, the solve, Save with its
 * read-back, a refusal, and Clear behind its confirmation.
 */
@RunWith(AndroidJUnit4::class)
class AccelCalScreenTest {

    @get:Rule val compose = createComposeRule()

    /** A flight computer holding [stored] that measures [faces] in turn. */
    private class Fc : BleTransport {
        private val fromFc = MutableSharedFlow<TransportEvent>(extraBufferCapacity = 64)
        override val events: Flow<TransportEvent> = fromFc
        val commands = CopyOnWriteArrayList<Int>()

        /** The offset in its NVS, m/s²; null when none is stored. */
        var stored: List<Float>? = listOf(0.0412f, -0.0273f, 0.0156f)

        /** The face windows' mean readings, in the order they will be measured. */
        val faces = ArrayDeque<List<Float>>()

        /** Answer 78 as in flight: FAILED, no samples, no window. */
        var refuse = false

        private var faceState = 0
        private var faceMean = listOf(0f, 0f, 0f)
        private var faceN = 0

        val accelCommands: List<Int>
            get() = commands.filter { it in BleCommandId.ACC_CAL_APPLY..BleCommandId.ACC_FACE_MEASURE }

        override suspend fun connect() = Unit
        override suspend fun requestMtu(target: Int): Int = 185
        override suspend fun enableNotifications(char: TrCharacteristic) = Unit
        override suspend fun read(char: TrCharacteristic): ByteArray = ByteArray(0)
        override suspend fun readRssi(): Int = -60
        override fun disconnect() = Unit

        override suspend fun write(char: TrCharacteristic, bytes: ByteArray, withResponse: Boolean) {
            if (char != TrCharacteristic.COMMAND || bytes.isEmpty()) return
            val cmd = bytes[0].toInt() and 0xFF
            commands += cmd
            when (cmd) {
                BleCommandId.ACC_CAL_READ -> report()
                BleCommandId.ACC_FACE_MEASURE -> if (refuse) {
                    window(AccelFaceState.FAILED.raw, listOf(0f, 0f, 0f), 0)
                } else {
                    window(AccelFaceState.MEASURING.raw, listOf(0f, 0f, 0f), 0)
                    window(AccelFaceState.DONE.raw, faces.removeFirst(), 2880)
                }
                BleCommandId.ACC_CAL_APPLY -> {
                    val b = LeBuffer(bytes, position = 1)
                    val o = listOf(b.f32(), b.f32(), b.f32())
                    stored = if (o.all { it == 0f }) null else o
                    report()
                }
            }
        }

        fun telemetry(json: String) {
            fromFc.tryEmit(TransportEvent.Notification(TrCharacteristic.TELEMETRY, json.encodeToByteArray()))
        }

        private fun window(state: Int, mean: List<Float>, n: Int) {
            faceState = state
            faceMean = mean
            faceN = n
            report()
        }

        /** `[0xD1][AccelCalStatusData]`: flags, face state, offset, mean, SD, n. */
        private fun report() {
            val p = ByteArray(1 + AccelCalStatus.SIZE)
            p[0] = 0xD1.toByte()
            val o = stored
            p[1] = (if (o != null) AccelCalStatus.FLAG_STORED or AccelCalStatus.FLAG_APPLIED else 0).toByte()
            p[2] = faceState.toByte()
            fun f32(at: Int, v: Float) {
                val bits = v.toRawBits()
                for (i in 0 until 4) p[at + i] = (bits ushr (8 * i)).toByte()
            }
            for (i in 0 until 3) {
                f32(3 + 4 * i, o?.get(i) ?: 0f)
                f32(15 + 4 * i, faceMean[i])
                f32(27 + 4 * i, 0.012f)
            }
            p[39] = faceN.toByte()
            p[40] = (faceN ushr 8).toByte()
            fromFc.tryEmit(TransportEvent.Notification(TrCharacteristic.FILE_OPS, p))
        }
    }

    private val sessions = mutableListOf<DeviceSession>()

    private fun connect(fc: Fc): DeviceSession = DeviceSession(
        scope = CoroutineScope(Dispatchers.Unconfined),
        transport = fc,
        connectedDeviceName = "TR-R-Test",
        initialDeviceType = BleDeviceType.ROCKET,
    ).also { sessions += it; it.start() }

    @After fun closeSessions() = sessions.forEach { it.close() }

    /** Telemetry with the rail on, in [state]. */
    private fun powered(fc: Fc, state: String = "READY") = fc.telemetry("""{"st":"$state","fs":16}""")

    private fun open(session: DeviceSession) {
        compose.setContent { AccelCalScreen(session, deviceKey = "rocket", onBack = {}) }
        compose.waitForIdle()
    }

    private fun tap(text: String) {
        compose.onNodeWithText(text).performScrollTo().performClick()
        compose.waitForIdle()
    }

    @Test
    fun sixFacesInAnyOrder_solveSaveAndReadBack_thenStartOver() {
        val fc = Fc()
        // A board with 30 / -46 / 12 mg of offset, set square on each face.
        val off = listOf(30.0, -46.0, 12.0).map { it / 1e3 * AccelSixFace.G0 }
        for (face in listOf(Face.Z_UP, Face.X_DOWN, Face.Y_UP, Face.Z_DOWN, Face.X_UP, Face.Y_DOWN)) {
            fc.faces += (0..2).map { (off[it] + if (it == face.axis) face.sign * AccelSixFace.G0 else 0.0).toFloat() }
        }
        val session = connect(fc)
        powered(fc)
        open(session)
        // The opening read shows what the board holds, axis by axis.
        for (v in listOf("+4.20 mg", "-2.78 mg", "+1.59 mg")) compose.onNodeWithText(v).assertExists()
        compose.onNodeWithText("In use by the flight computer").assertExists()

        tap("Measure This Face")
        // The offset itself leans each face: 55 mg sideways is 3.1° here.
        compose.onNodeWithText("+Z UP, 3.1° off vertical").assertExists()
        compose.onNodeWithText("(+0.294, -0.451, +9.924) m/s², SD max 1.2 mg, 2880 samples").assertExists()
        compose.onNodeWithText("3.1° off").assertExists()
        repeat(5) { tap("Measure This Face") }
        compose.onNodeWithText("Measure a Face Again").assertExists()
        compose.onNodeWithText("Offset").assertExists()
        for (v in listOf("+30.00 mg", "-46.00 mg", "+12.00 mg")) compose.onNodeWithText(v).assertExists()

        tap("Save to the Flight Computer")
        compose.onNodeWithText("Saved and read back").assertExists()
        // The read-back is now the stored offset, beside the solve's own row.
        compose.onAllNodesWithText("+30.00 mg").assertCountEquals(2)
        // The read, six windows, the save, and its read-back — in that order.
        assertEquals(listOf(77, 78, 78, 78, 78, 78, 78, 76, 77), fc.accelCommands)
        assertEquals(
            listOf(30, -46, 12),
            fc.stored!!.map { (AccelSixFace.mg(it.toDouble()) * 100).roundToInt() / 100 },
        )

        tap("Start Over")
        compose.onNodeWithText("Offset").assertDoesNotExist()
        compose.onNodeWithText("Saved and read back").assertDoesNotExist()
        compose.onNodeWithText("Start Over").assertDoesNotExist()
        compose.onNodeWithText("Measure This Face").assertIsEnabled()
    }

    @Test
    fun aRefusalIsSaid_andCapturesNothing_andNothingIsSentUntilThePowerIsOn() {
        val fc = Fc().apply { refuse = true }
        val session = connect(fc)
        open(session)
        // No telemetry yet: the rocket reads as off. As on iOS, not even the
        // opening read goes out.
        compose.onNodeWithText("Measure This Face").assertIsNotEnabled()
        compose.onNodeWithText("Connect to the rocket and power it on to calibrate it.").assertExists()
        compose.onNodeWithText("Not read yet").assertExists()
        assertEquals(emptyList<Int>(), fc.accelCommands)

        powered(fc)
        compose.waitForIdle()
        assertEquals("the gate opened with nothing read: read now", listOf(77), fc.accelCommands)
        compose.onNodeWithText("Measure This Face").assertIsEnabled()
        tap("Measure This Face")
        compose.onNodeWithText(
            "The flight computer refused: it measures no face in flight, after a flight, in a sim " +
                "or during a magnetometer calibration",
        ).assertExists()
        compose.onNodeWithText("Start Over").assertDoesNotExist()
    }

    @Test
    fun afterAFlight_theOpeningReadWaitsForARestart() {
        val fc = Fc()
        val session = connect(fc)
        powered(fc, state = "LANDED")
        open(session)
        compose.onNodeWithText(
            "After a flight the flight computer takes no calibration until it restarts: power " +
                "the rocket off and on.",
        ).assertExists()
        compose.onNodeWithText("Measure This Face").assertIsNotEnabled()
        assertEquals(emptyList<Int>(), fc.accelCommands)

        // Power-cycled: the flight computer is back on the pad.
        powered(fc, state = "READY")
        compose.waitForIdle()
        assertEquals(listOf(77), fc.accelCommands)
        compose.onNodeWithText("+4.20 mg").assertExists()
    }

    @Test
    fun clearAsksFirst_thenReadsBackNoOffset() {
        val fc = Fc()
        val session = connect(fc)
        powered(fc)
        open(session)
        tap("Clear the Stored Offset…")
        compose.onNodeWithText("Clear the stored offset?").assertExists()
        assertEquals("nothing is sent before the confirmation", listOf(77), fc.accelCommands)
        compose.onNodeWithText("Clear Offset").performClick()
        compose.waitForIdle()
        compose.onNodeWithText("Cleared: no offset stored").assertExists()
        compose.onNodeWithText("No offset stored").assertExists()
        assertEquals(listOf(77, 76, 77), fc.accelCommands)
        assertNull(fc.stored)
        // Nothing stored: nothing to clear.
        compose.onNodeWithText("Clear the Stored Offset…").assertIsNotEnabled()
    }
}
