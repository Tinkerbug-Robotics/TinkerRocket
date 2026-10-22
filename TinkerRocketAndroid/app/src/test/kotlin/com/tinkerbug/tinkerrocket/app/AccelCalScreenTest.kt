package com.tinkerbug.tinkerrocket.app

import androidx.compose.ui.test.assertIsEnabled
import androidx.compose.ui.test.assertIsNotSelected
import androidx.compose.ui.test.assertIsSelected
import androidx.compose.ui.test.hasClickAction
import androidx.compose.ui.test.hasText
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
import com.tinkerbug.tinkerrocket.protocol.HighGCalApply
import com.tinkerbug.tinkerrocket.protocol.LeBuffer
import com.tinkerbug.tinkerrocket.session.AccelSixFace
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Face
import com.tinkerbug.tinkerrocket.session.ActiveProfileStorage
import com.tinkerbug.tinkerrocket.session.ActiveRocketSyncer
import com.tinkerbug.tinkerrocket.session.BleDeviceType
import com.tinkerbug.tinkerrocket.session.BleTransport
import com.tinkerbug.tinkerrocket.session.DeviceSession
import com.tinkerbug.tinkerrocket.session.RocketProfileStore
import com.tinkerbug.tinkerrocket.session.SensorCalData
import com.tinkerbug.tinkerrocket.session.TrCharacteristic
import com.tinkerbug.tinkerrocket.session.TransportEvent
import java.util.concurrent.CopyOnWriteArrayList
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin
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
 * does: the opening read, six faces in any order, the solve and the save
 * that follows the sixth by itself, Save Again, a refusal, and Clear behind
 * its confirmation.  A flight computer
 * that sends the high-g answers 80 too, with 114-byte frames: the High-g
 * section, its save after the offset, the stored rows, and the clear that
 * takes both.
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

        /**
         * A flight and out computer that send the high-g: 114-byte frames, and
         * 80 answered.  False is an older one: 40 bytes, and 80 unanswered.
         */
        var sendsHighG = false

        /** The high-g windows' mean readings, beside [faces]. */
        val hgFaces = ArrayDeque<List<Float>>()

        /** The high-g alignment M in NVS, row-major; null when none is stored. */
        var hgMatrix: List<Float>? = null

        /** The high-g bias in use, and what wrote it (HG_BIAS_SRC_*); null when none. */
        var hgBias: List<Float>? = null
        var hgSource = 0

        private var faceState = 0
        private var faceMean = listOf(0f, 0f, 0f)
        private var hgMean = listOf(0f, 0f, 0f)
        private var faceN = 0

        val accelCommands: List<Int>
            get() = commands.filter {
                it in BleCommandId.ACC_CAL_APPLY..BleCommandId.ACC_FACE_MEASURE || it == BleCommandId.HG_CAL_APPLY
            }

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
                    window(AccelFaceState.DONE.raw, faces.removeFirst(), 2880, hgFaces.removeFirstOrNull())
                }
                BleCommandId.ACC_CAL_APPLY -> {
                    val b = LeBuffer(bytes, position = 1)
                    val o = listOf(b.f32(), b.f32(), b.f32())
                    stored = if (o.all { it == 0f }) null else o
                    report()
                }
                BleCommandId.HG_CAL_APPLY -> if (sendsHighG) {
                    val b = LeBuffer(bytes, position = 1)
                    val op = b.u8()
                    val m = List(9) { b.f32() }
                    val bias = List(3) { b.f32() }
                    if (op == HighGCalApply.OP_STORE) {
                        hgMatrix = m
                        hgBias = bias
                        hgSource = 2
                    } else {
                        hgMatrix = null   // the flight computer keeps the bias
                    }
                    report()
                }
            }
        }

        fun telemetry(json: String) {
            fromFc.tryEmit(TransportEvent.Notification(TrCharacteristic.TELEMETRY, json.encodeToByteArray()))
        }

        private fun window(state: Int, mean: List<Float>, n: Int, hg: List<Float>? = null) {
            faceState = state
            faceMean = mean
            hgMean = hg ?: listOf(0f, 0f, 0f)
            faceN = n
            report()
        }

        /**
         * `[0xD1][AccelCalStatusData]`: flags, face state, offset, mean, SD,
         * n; and AccelCalHighGStatus after it when [sendsHighG].
         */
        private fun report() {
            val p = ByteArray(1 + if (sendsHighG) AccelCalStatus.HIGH_G_SIZE else AccelCalStatus.SIZE)
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
            if (sendsHighG) {
                val m = hgMatrix
                val b = hgBias
                for (i in 0 until 3) {
                    f32(41 + 4 * i, hgMean[i])
                    f32(53 + 4 * i, 0.56f)
                    f32(103 + 4 * i, b?.get(i) ?: 0f)
                }
                p[65] = ((if (m != null) 0x03 else 0) or (if (b != null) 0x04 else 0)).toByte()
                p[66] = hgSource.toByte()
                for (i in 0 until 9) f32(67 + 4 * i, (m ?: HighGCalApply.IDENTITY)[i])
            }
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

    private fun open(session: DeviceSession, syncer: ActiveRocketSyncer? = null) {
        compose.setContent { AccelCalScreen(session, deviceKey = "rocket", syncer = syncer, onBack = {}) }
        compose.waitForIdle()
    }

    private fun tap(text: String) {
        compose.onNodeWithText(text).performScrollTo().performClick()
        compose.waitForIdle()
    }

    @Test
    fun sixFacesInAnyOrder_solveAndSaveByThemselves_thenStartOver() {
        val fc = Fc()
        // A board with 30 / -46 / 12 mg of offset, set square on each face,
        // then +Z UP measured again.
        val off = listOf(30.0, -46.0, 12.0).map { it / 1e3 * AccelSixFace.G0 }
        for (face in listOf(Face.Z_UP, Face.X_DOWN, Face.Y_UP, Face.Z_DOWN, Face.X_UP, Face.Y_DOWN, Face.Z_UP)) {
            fc.faces += (0..2).map { (off[it] + if (it == face.axis) face.sign * AccelSixFace.G0 else 0.0).toFloat() }
        }
        val session = connect(fc)
        powered(fc)
        open(session)
        // The opening read shows what the board holds, axis by axis.
        for (v in listOf("+4.20 mg", "-2.78 mg", "+1.59 mg")) compose.onNodeWithText(v).assertExists()
        compose.onNodeWithText("In use by the flight computer").assertExists()
        // This link's frames are the low-g's 40 bytes: an older flight computer.
        compose.onNodeWithText(
            "This firmware sends no high-g calibration: update the flight computer and the out " +
                "computer to calibrate the high-g.",
        ).assertExists()

        tap("Measure This Face")
        // The offset itself leans each face: 55 mg sideways is 3.1° here.
        compose.onNodeWithText("+Z UP, 3.1° off vertical").assertExists()
        compose.onNodeWithText("(+0.294, -0.451, +9.924) m/s², SD max 1.2 mg, 2880 samples").assertExists()
        compose.onNodeWithText("3.1° off").assertExists()
        repeat(5) { tap("Measure This Face") }
        compose.onNodeWithText("Measure a Face Again").assertExists()
        compose.onNodeWithText("Offset").assertExists()
        // The sixth face saved the offset by itself and read it back: each
        // axis shows twice, the solve's row and the stored one.
        compose.onNodeWithText("Saved and read back").assertExists()
        for (v in listOf("+30.00 mg", "-46.00 mg", "+12.00 mg")) compose.onAllNodesWithText(v).assertCountEquals(2)
        // The read, six windows, the save, and its read-back — in that order,
        // with no tap on Save.
        assertEquals(listOf(77, 78, 78, 78, 78, 78, 78, 76, 77), fc.accelCommands)
        assertEquals(
            listOf(30, -46, 12),
            fc.stored!!.map { (AccelSixFace.mg(it.toDouble()) * 100).roundToInt() / 100 },
        )
        compose.onNodeWithText(
            "All six faces are solved together, so a face a few degrees off square does not bias the offset.",
        ).assertExists()
        // The faces came without the high-g: the section says why, and the
        // offset still saves on its own.
        compose.onNodeWithText("High-g Accelerometer").assertExists()
        compose.onNodeWithText(
            "The faces came without the high-g reading. Update the flight computer and the out " +
                "computer to calibrate the high-g; the low-g offset saves as before.",
        ).assertExists()

        // Save Again sends it once more; so does a face measured again.
        tap("Save Again")
        compose.onNodeWithText("Saved and read back").assertExists()
        tap("Measure a Face Again")
        compose.onNodeWithText("+Z UP, 3.1° off vertical — replaced the earlier one").assertExists()
        compose.onNodeWithText("Saved and read back").assertExists()
        assertEquals(listOf(77, 78, 78, 78, 78, 78, 78, 76, 77, 76, 77, 78, 76, 77), fc.accelCommands)

        tap("Start Over")
        compose.onNodeWithText("Offset").assertDoesNotExist()
        compose.onNodeWithText("Saved and read back").assertDoesNotExist()
        compose.onNodeWithText("Start Over").assertDoesNotExist()
        compose.onNodeWithText("Measure This Face").assertIsEnabled()
    }

    @Test
    fun anOffsetPastTheLimit_isNotSentAfterTheSixthFace() {
        val fc = Fc()
        // 1.2 m/s² on Y: each face 7° off vertical, and past the 1.0 the flight computer takes.
        val off = listOf(0.0, 1.2, 0.0)
        for (face in Face.entries) {
            fc.faces += (0..2).map { (off[it] + if (it == face.axis) face.sign * AccelSixFace.G0 else 0.0).toFloat() }
        }
        val session = connect(fc)
        powered(fc)
        open(session)
        repeat(6) { tap("Measure This Face") }
        compose.onNode(hasText("Offset past 1.0 m/s² on Y", substring = true)).assertExists()
        compose.onNodeWithText("Save to the Flight Computer").assertIsNotEnabled()
        assertEquals("no save after the sixth face", listOf(77, 78, 78, 78, 78, 78, 78), fc.accelCommands)
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

    /**
     * Telemetry with the rail on and the low-g reading (rocket frame) the
     * board gives at [tiltDeg] off its +Y axis, leaning toward +X, |v| = [g].
     * As the rocket's own link sends it: no "imo" in the frame — the
     * mounting (here the identity, the pad auto-detect's snap) comes in the
     * "imu_orient" message.
     */
    private fun leaningOnPlusY(fc: Fc, tiltDeg: Double, g: Double = AccelSixFace.G0) {
        val t = Math.toRadians(tiltDeg)
        fun n(v: Double) = String.format(java.util.Locale.ROOT, "%.4f", v)
        fc.telemetry("""{"type":"imu_orient","code":0,"mode":2,"name":"+X","set":255}""")
        fc.telemetry("""{"st":"READY","fs":16,"lx":${n(g * sin(t))},"ly":${n(g * cos(t))},"lz":0.0}""")
        compose.waitForIdle()
    }

    private fun tapRow(face: String) {
        compose.onNode(hasText(face) and hasClickAction()).performScrollTo().performClick()
        compose.waitForIdle()
    }

    @Test
    fun theLiveFacePreselectsItsRow_namesTheButton_andARowTapShowsHowToPlaceIt() {
        val fc = Fc()
        fc.faces += listOf(0.85f, 9.77f, 0.0f)   // the window, if one is measured: +Y UP
        val session = connect(fc)
        powered(fc)
        open(session)
        compose.onNodeWithText("Waiting for the rocket's reading…").assertExists()
        compose.onNodeWithText("You don't pick the face: set the board on a side, and the app sees which one it is.")
            .assertExists()

        leaningOnPlusY(fc, tiltDeg = 5.0)
        compose.onNodeWithText("On now: +Y UP (left side up), 5° off vertical").assertExists()
        compose.onNode(hasText("+Y UP") and hasText("Now")).assertIsSelected()
        compose.onNode(hasText("+X UP") and hasClickAction()).assertIsNotSelected()
        compose.onNodeWithText("Measure +Y UP").assertIsEnabled()

        // Askew: the row stays the live one, the button stops naming it.
        leaningOnPlusY(fc, tiltDeg = 15.0)
        compose.onNodeWithText("Set it squarer: +Y UP is 15° off vertical").assertExists()
        compose.onNode(hasText("+Y UP") and hasText("Now")).assertIsSelected()
        compose.onNodeWithText("Measure This Face").assertIsEnabled()

        // Moving: no live row at all.
        leaningOnPlusY(fc, tiltDeg = 0.0, g = 8.5)
        compose.onNodeWithText("Hold it still").assertExists()
        compose.onNodeWithText("Now").assertDoesNotExist()
        compose.onNodeWithText("Measure This Face").assertIsEnabled()

        // A tap on a row says how to place the board; the next tap replaces it.
        tapRow("+X DOWN")
        compose.onNodeWithText("To measure +X DOWN: stand it on its forward end, forward end down.").assertExists()
        tapRow("+Z UP")
        compose.onNodeWithText("To measure +Z UP: lay it flat, component side up.").assertExists()
        compose.onNodeWithText("To measure +X DOWN: stand it on its forward end, forward end down.")
            .assertDoesNotExist()

        // A measurement clears it.
        leaningOnPlusY(fc, tiltDeg = 5.0)
        tap("Measure +Y UP")
        compose.onNodeWithText("To measure +Z UP: lay it flat, component side up.").assertDoesNotExist()
        compose.onNodeWithText("+Y UP, 5.0° off vertical").assertExists()
    }

    /**
     * The user's Beetle on its bench: the pad auto-detect in exact mode
     * (mode 3), code 16 (+Z toward the nose), gravity along the nose.  The
     * face is named and preselected, with no tilt to tell.
     */
    @Test
    fun anAutoExactMounting_namesAndPreselectsTheFace_withoutATilt() {
        val fc = Fc()
        val session = connect(fc)
        powered(fc)
        open(session)
        fc.telemetry("""{"type":"imu_orient","code":16,"mode":3,"name":"+Z","set":255}""")
        fc.telemetry("""{"st":"READY","fs":16,"lx":9.8,"ly":0.0,"lz":0.0}""")
        compose.waitForIdle()
        compose.onNodeWithText("On now: +Z UP (component side up)").assertExists()
        compose.onNode(hasText("+Z UP") and hasText("Now")).assertIsSelected()
        compose.onNodeWithText("Measure +Z UP").assertIsEnabled()
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

    // ── The high-g ────────────────────────────────────────────────────────

    /**
     * A board whose two accelerometers differ as real ones do (iOS
     * HighGCalTests' synthetic board): the low-g with 30 / -46 / 12 mg of
     * offset, the high-g through a full 3x3 and its own zero-g.  Square faces,
     * in the order the Fc measures them.
     */
    private fun loadBoard(fc: Fc) {
        val off = listOf(30.0, -46.0, 12.0).map { it / 1e3 * AccelSixFace.G0 }
        val sHG = listOf(0.9921, -0.0047, 0.0030, 0.0029, 1.0068, -0.0054, -0.0016, 0.0051, 0.9880)
        val cHG = listOf(0.98, -1.47, 0.62)
        for (face in listOf(Face.Z_UP, Face.X_DOWN, Face.Y_UP, Face.Z_DOWN, Face.X_UP, Face.Y_DOWN)) {
            val a = (0..2).map { if (it == face.axis) face.sign * AccelSixFace.G0 else 0.0 }
            fc.faces += (0..2).map { (off[it] + a[it]).toFloat() }
            fc.hgFaces += (0..2).map { i ->
                (sHG[3 * i] * a[0] + sHG[3 * i + 1] * a[1] + sHG[3 * i + 2] * a[2] + cHG[i]).toFloat()
            }
        }
    }

    /** A syncer attached with [session] to a profile store whose active profile holds [cal]. */
    private fun syncerWith(session: DeviceSession, cal: SensorCalData): Pair<ActiveRocketSyncer, RocketProfileStore> {
        val dir = java.nio.file.Files.createTempDirectory("accelcal").toFile()
        val store = RocketProfileStore(
            dir,
            object : ActiveProfileStorage {
                var v: String? = null
                override fun loadActiveId() = v
                override fun saveActiveId(id: String?) { v = id }
            },
        )
        val p = store.add("Atlas")
        store.update(p.id) { it.copy(sensorCal = cal) }
        store.setActive(p.id)
        val syncer = ActiveRocketSyncer(CoroutineScope(Dispatchers.Unconfined))
        syncer.attach(session, store)
        return syncer to store
    }

    @Test
    fun withTheHighG_theSectionShowsItsFit_andTheSaveSendsItAfterTheOffset() {
        val fc = Fc().apply {
            sendsHighG = true
            hgBias = listOf(0.9f, -1.4f, 0.6f)
            hgSource = 1   // the pad calibration's
        }
        loadBoard(fc)
        val session = connect(fc)
        fc.telemetry("""{"type":"config_identity","uid":"boardA","un":"Atlas","nid":5,"rid":1,"dt":"R"}""")
        val cal = SensorCalData(3, -2, 1, 0.9f, -1.4f, 0.6f, calibratedOnUnitID = "boardA", calibratedAtMs = 5)
        val (syncer, store) = syncerWith(session, cal)
        powered(fc)
        open(session, syncer)
        // The stored rows: no alignment yet, and the pad calibration's bias.
        compose.onNodeWithText("No high-g alignment stored").assertExists()
        compose.onNodeWithText("High-g bias: X +91.8 mg, Y -142.8 mg, Z +61.2 mg").assertExists()
        compose.onNodeWithText("Measured by the pad calibration").assertExists()
        compose.onNodeWithText("Clear the Stored Offset…").assertIsEnabled()

        repeat(6) { tap("Measure This Face") }
        compose.onNodeWithText("High-g Accelerometer").assertExists()
        // Per axis the scale and the offset, each with its 1-sigma.
        compose.onAllNodes(hasText(" % ± ", substring = true)).assertCountEquals(3)
        compose.onAllNodes(hasText(" mg ± ", substring = true)).assertCountEquals(3)
        compose.onNode(hasText("Cross-axis: up to ", substring = true)).assertExists()
        compose.onNode(hasText("Fit to the low-g: ", substring = true) and hasText("(limit 3 mg)", substring = true))
            .assertExists()
        compose.onNodeWithText("Its own gravity fit agrees with the alignment to the low-g within their noise.")
            .assertExists()
        compose.onNodeWithText("Passed: saved with the offset").assertExists()
        compose.onNodeWithText(
            "The high-g is fitted to the low-g, the more accurate channel on the bench, so the flight " +
                "computer reads the same acceleration when it switches to the high-g near 16 g. ± is one " +
                "standard deviation from these faces. The pad calibration keeps this alignment and measures " +
                "the high-g's offset again on the day.",
        ).assertExists()

        // The sixth face saved both, with no tap on Save.
        compose.onNodeWithText("Saved and read back").assertExists()
        compose.onNodeWithText("High-g alignment saved and read back").assertExists()
        // The read, six windows, the offset and its read-back, then the high-g and its own.
        assertEquals(listOf(77, 78, 78, 78, 78, 78, 78, 76, 77, 80, 77), fc.accelCommands)
        val m = requireNotNull(fc.hgMatrix) { "no high-g alignment stored" }
        assertEquals(1 / 0.9921, m[0].toDouble(), 2e-4)   // the low-g's scale over the high-g's
        assertEquals(2, fc.hgSource)
        // The stored rows follow the read-back.
        compose.onNode(hasText("High-g alignment in use: X +", substring = true)).assertExists()
        compose.onNodeWithText("From the six faces").assertExists()
        compose.onNodeWithText("Clear the Stored Calibration…").assertIsEnabled()
        // This board's profile sensor cal now holds the bias the flight computer uses.
        val saved = requireNotNull(store.activeProfile?.sensorCal) { "the profile lost its sensor cal" }
        assertEquals(fc.hgBias, listOf(saved.hgX, saved.hgY, saved.hgZ))
        assertEquals(3, saved.gyroX)
    }

    @Test
    fun aClearWithAHighGStored_asksAboutTheCalibration_andClearsBoth() {
        val fc = Fc().apply {
            sendsHighG = true
            hgMatrix = listOf(1.01f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f)
            hgBias = listOf(0.98f, -1.47f, 0.62f)
            hgSource = 2
        }
        val session = connect(fc)
        powered(fc)
        open(session)
        compose.onNodeWithText("High-g alignment in use: X +1.000 %, Y +0.000 %, Z +0.000 %").assertExists()
        compose.onNodeWithText(
            "Removes the offset and the high-g alignment from the flight computer, which then uses the " +
                "low-g reading as it is and the high-g with its bias alone.",
        ).assertExists()
        tap("Clear the Stored Calibration…")
        compose.onNodeWithText("Clear the stored calibration?").assertExists()
        compose.onNodeWithText(
            "The flight computer goes back to the low-g reading as it is and to the high-g without its " +
                "alignment (it keeps the high-g's bias until the next pad calibration). You can measure and " +
                "save again at any time.",
        ).assertExists()
        assertEquals("nothing is sent before the confirmation", listOf(77), fc.accelCommands)
        compose.onNodeWithText("Clear Calibration").performClick()
        compose.waitForIdle()
        assertEquals(listOf(77, 76, 77, 80, 77), fc.accelCommands)
        compose.onNodeWithText("Cleared: no offset stored").assertExists()
        compose.onNodeWithText("Cleared: no high-g alignment stored").assertExists()
        compose.onNodeWithText("No high-g alignment stored").assertExists()
        assertNull(fc.stored)
        assertNull(fc.hgMatrix)
        // The bias stays until the next pad calibration, and there is nothing left to clear.
        compose.onNodeWithText("High-g bias: X +99.9 mg, Y -149.9 mg, Z +63.2 mg").assertExists()
        compose.onNodeWithText("Clear the Stored Offset…").assertIsNotEnabled()
    }

    @Test
    fun aClearWithOnlyTheOffsetStored_sendsNo80() {
        val fc = Fc().apply { sendsHighG = true }
        val session = connect(fc)
        powered(fc)
        open(session)
        compose.onNodeWithText("No high-g alignment stored").assertExists()
        tap("Clear the Stored Offset…")
        compose.onNodeWithText("Clear the stored offset?").assertExists()
        compose.onNodeWithText("Clear Offset").performClick()
        compose.waitForIdle()
        assertEquals(listOf(77, 76, 77), fc.accelCommands)
        compose.onNodeWithText("Cleared: no offset stored").assertExists()
        compose.onNodeWithText("Cleared: no high-g alignment stored").assertDoesNotExist()
    }
}
