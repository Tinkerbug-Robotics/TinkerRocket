package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.AccelCalStatus
import com.tinkerbug.tinkerrocket.protocol.AccelFaceState
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Access
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Face
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Measurement
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Verdict
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertIs
import kotlin.test.assertNull
import kotlin.test.assertSame
import kotlin.test.assertTrue

/**
 * #1629: the six-face calibration's rules and math, against synthetic faces
 * whose answer is known.  tools/accel_cal_six_face.py is the reference.
 */
class AccelSixFaceTest {

    private val g = AccelSixFace.G0

    // The board under test: 30 / -46 / 12 mg of offset and scales a few
    // tenths of a percent off 1, as the ISM6HG256 can be.
    private val offMg = listOf(30.0, -46.0, 12.0)
    private val off = offMg.map { it / 1e3 * g }
    private val scale = listOf(1.0020, 0.9970, 1.0015)

    /**
     * What a still accelerometer reads on [face] with its axis [tiltDeg] off
     * vertical, leaning toward board axis [toward] ([towardSign]): +g along
     * the axis that points up, through the board's scale and offset, and
     * through the wire's f32 like a real 0xD1 window.
     */
    private fun reading(
        face: Face,
        tiltDeg: Double = 0.0,
        toward: Int = (face.axis + 1) % 3,
        towardSign: Int = 1,
        offset: List<Double> = off,
        scl: List<Double> = scale,
    ): List<Double> {
        val t = Math.toRadians(tiltDeg)
        val u = DoubleArray(3)
        u[face.axis] = face.sign * cos(t)
        u[toward] = towardSign * sin(t)
        return (0..2).map { (offset[it] + scl[it] * g * u[it]).toFloat().toDouble() }
    }

    private fun window(
        mean: List<Double>,
        state: AccelFaceState = AccelFaceState.DONE,
        n: Int = 2880,
        flags: Int = 0,
        offset: List<Float> = listOf(0f, 0f, 0f),
    ) = AccelCalStatus(
        flags = flags, faceState = state,
        offsetX = offset[0], offsetY = offset[1], offsetZ = offset[2],
        faceMeanX = mean[0].toFloat(), faceMeanY = mean[1].toFloat(), faceMeanZ = mean[2].toFloat(),
        faceSdX = 0.012f, faceSdY = 0.013f, faceSdZ = 0.015f, faceN = n,
    )

    private fun allSquare(offset: List<Double> = off, scl: List<Double> = scale): Map<Face, List<Double>> =
        Face.entries.associateWith { reading(it, offset = offset, scl = scl) }

    // A perfect accelerometer, for the angle checks: any scale error skews
    // the angle it reports.
    private val flat = listOf(0.0, 0.0, 0.0)
    private val unit = listOf(1.0, 1.0, 1.0)

    // ── which face ───────────────────────────────────────────────────────

    @Test
    fun everyFaceIsToldFromItsReading() {
        for (face in Face.entries) {
            // Square, and leaning 6° every way: the face stays the face.
            assertEquals(face, AccelSixFace.identify(reading(face)).face, face.label)
            for (toward in (0..2).filter { it != face.axis }) {
                for (s in listOf(1, -1)) {
                    assertEquals(
                        face,
                        AccelSixFace.identify(reading(face, 6.0, toward, s)).face,
                        "${face.label} leaning toward ${if (s > 0) "+" else "-"}$toward",
                    )
                }
            }
        }
        assertEquals(
            listOf("+X UP", "+X DOWN", "+Y UP", "+Y DOWN", "+Z UP", "+Z DOWN"),
            Face.entries.map { it.label },
        )
    }

    @Test
    fun theTiltIsTheAxisAngleOffVertical() {
        val r = AccelSixFace.identify(reading(Face.Y_DOWN, 7.0, toward = 2, offset = flat, scl = unit))
        assertEquals(Face.Y_DOWN, r.face)
        assertEquals(7.0, r.tiltDeg, 1e-4)
        // The wire fixture's window (accelcal_0xD1_40): the board flat, component side up.
        val fixture = AccelSixFace.identify(listOf(-0.0830999985, 0.1149, 9.82129955))
        assertEquals(Face.Z_UP, fixture.face)
        assertEquals(0.827, fixture.tiltDeg, 1e-3)
        // A reading of nothing has no vertical axis.
        assertEquals(90.0, AccelSixFace.identify(listOf(0.0, 0.0, 0.0)).tiltDeg)
    }

    // ── judging a window ─────────────────────────────────────────────────

    @Test
    fun aStillSquarishFaceIsKept_andSaysWhenItReplacesOne() {
        val w = window(reading(Face.X_DOWN, 4.0, offset = flat, scl = unit))
        val first = assertIs<Verdict.Accepted>(AccelSixFace.judge(w, emptySet()))
        assertEquals(Face.X_DOWN, first.face)
        assertEquals(4.0, first.tiltDeg, 1e-4)
        assertFalse(first.replaced)
        val again = assertIs<Verdict.Accepted>(AccelSixFace.judge(w, setOf(Face.Z_UP, Face.X_DOWN)))
        assertTrue(again.replaced, "measuring a face already captured replaces it")
    }

    @Test
    fun aFaceMoreThanTenDegreesOffIsMeasuredAgain() {
        assertIs<Verdict.Accepted>(
            AccelSixFace.judge(window(reading(Face.Z_UP, 9.5, offset = flat, scl = unit)), emptySet()),
        )
        val tilted = assertIs<Verdict.Tilted>(
            AccelSixFace.judge(window(reading(Face.Z_UP, 10.5, offset = flat, scl = unit)), emptySet()),
        )
        assertEquals(Face.Z_UP, tilted.face)
        assertEquals(10.5, tilted.tiltDeg, 1e-4)
    }

    @Test
    fun aFailedWindowIsNeverUsed_whateverItReads() {
        // Moved, or not one g: the flight computer sends the numbers anyway,
        // and a perfect-looking reading must not sneak through on them.
        val w = window(reading(Face.Y_UP), state = AccelFaceState.FAILED)
        assertSame(Verdict.Moved, AccelSixFace.judge(w, emptySet()))
    }

    // ── the solve ────────────────────────────────────────────────────────

    @Test
    fun squareFacesGiveTheOffsetAndScaleBack() {
        val s = AccelSixFace.solve(allSquare())
        for (i in 0..2) {
            assertEquals(offMg[i], s.offsetMg[i], 1e-3, "offset axis $i")
            assertEquals(scale[i], s.scale[i], 1e-6, "scale axis $i")
            assertEquals((scale[i] - 1) * 100, s.scalePct[i], 1e-4)
        }
        for (face in Face.entries) assertEquals(0.0, s.residualMg.getValue(face), 1e-2, face.label)
        assertTrue(s.saveAllowed)
        assertFalse(s.residualNote)
        assertEquals(s.offset.map { it.toFloat() }, s.wireOffset)
    }

    /**
     * Faces held by hand, up to 4° off.  Each pair leaves
     * g·(cos t_up − cos t_down)/2 in its axis's offset, so pairs tilted
     * alike cancel: 4° against 3.5° is 0.29 mg, 2° against 3° 0.38 mg,
     * 2.5° against 3° 0.21 mg.
     */
    @Test
    fun handHeldFacesUpToFourDegreesOffGiveTheOffsetToHalfAMilliG() {
        val faces = mapOf(
            Face.X_UP to reading(Face.X_UP, 4.0, toward = 1, towardSign = 1),
            Face.X_DOWN to reading(Face.X_DOWN, 3.5, toward = 2, towardSign = -1),
            Face.Y_UP to reading(Face.Y_UP, 2.0, toward = 2, towardSign = 1),
            Face.Y_DOWN to reading(Face.Y_DOWN, 3.0, toward = 0, towardSign = 1),
            Face.Z_UP to reading(Face.Z_UP, 2.5, toward = 0, towardSign = -1),
            Face.Z_DOWN to reading(Face.Z_DOWN, 3.0, toward = 1, towardSign = -1),
        )
        // Every one is still told apart and kept.
        for ((face, mean) in faces) {
            val v = assertIs<Verdict.Accepted>(AccelSixFace.judge(window(mean), emptySet()), face.label)
            assertEquals(face, v.face)
        }
        val s = AccelSixFace.solve(faces)
        for (i in 0..2) assertEquals(offMg[i], s.offsetMg[i], 0.5, "offset axis $i, mg")
        // The scale is the script's formula, whatever the tilt did to it.
        for (i in 0..2) {
            val p = faces.getValue(Face.of(i, 1))[i]
            val m = faces.getValue(Face.of(i, -1))[i]
            assertEquals((p - m) / (2 * g), s.scale[i], 1e-12)
        }
        assertTrue(s.worstResidualMg < AccelSixFace.RESIDUAL_NOTE_MG, "worst ${s.worstResidualMg} mg")
        assertFalse(s.residualNote)
        assertTrue(s.saveAllowed)
    }

    @Test
    fun anOffsetPastOneMetrePerSecondSquaredIsNotSaved() {
        // A face measured the wrong way up, or moving, solves to nonsense the
        // flight computer would refuse: the screen says "redo the faces".
        val s = AccelSixFace.solve(allSquare(offset = listOf(0.2, 1.2, -0.1)))
        assertEquals(1.2, s.offset[1], 1e-5)
        assertEquals(listOf(1), s.axesPastLimit)
        assertFalse(s.saveAllowed)
        // The script's test is `> 1.0`: exactly the limit still saves.
        val edge = AccelSixFace.Solution(listOf(1.0, -1.0, 0.0), unit, emptyMap())
        assertTrue(edge.saveAllowed)
        val past = edge.copy(offset = listOf(0.0, -1.0000001, 0.0))
        assertEquals(listOf(1), past.axesPastLimit)
    }

    @Test
    fun aFaceFarOffOneGAfterTheCorrectionIsANoteNotARefusal() {
        // +X UP set 9° off and +X DOWN square: the X pair no longer agrees on
        // the scale, and +X UP ends ~12 mg off one g after the correction.
        val faces = allSquare(offset = flat, scl = unit) +
            (Face.X_UP to reading(Face.X_UP, 9.0, toward = 1, offset = flat, scl = unit))
        val s = AccelSixFace.solve(faces)
        assertTrue(s.worstResidualMg > AccelSixFace.RESIDUAL_NOTE_MG, "worst ${s.worstResidualMg} mg")
        assertEquals(s.worstResidualMg, abs(s.residualMg.getValue(Face.X_UP)), 1e-9)
        assertTrue(s.residualNote)
        assertTrue(s.saveAllowed, "the note never blocks the save")
    }

    @Test
    fun theSolveNeedsAllSixFaces() {
        assertFailsWith<IllegalArgumentException> {
            AccelSixFace.solve(allSquare() - Face.Y_DOWN)
        }
    }

    // ── which frame answers a face measurement ───────────────────────────

    @Test
    fun aStaleDoneBeforeTheWindowsOwnMeasuringIsIgnored() {
        // A late answer to the screen's opening 77 carries the previous
        // window's DONE; taking it would capture the last face twice.
        val seq = AccelSixFace.Sequencer()
        assertNull(seq.onFrame(window(reading(Face.Z_UP))))
        assertFalse(seq.windowOpen)
        assertNull(seq.onFrame(window(listOf(0.0, 0.0, 0.0), state = AccelFaceState.MEASURING, n = 0)))
        assertTrue(seq.windowOpen)
        val fresh = window(reading(Face.X_UP))
        val result = assertIs<Measurement.Finished>(seq.onFrame(fresh))
        assertEquals(fresh, result.window)
        // Settled: nothing later changes it.
        assertSame(result, seq.onFrame(window(reading(Face.Y_UP))))
    }

    @Test
    fun aFailedWithNoSamplesInPlaceOfMeasuringIsARefusal() {
        val seq = AccelSixFace.Sequencer()
        val refusal = window(listOf(0.0, 0.0, 0.0), state = AccelFaceState.FAILED, n = 0)
        val out = assertIs<Measurement.Refused>(seq.onFrame(refusal))
        assertEquals(refusal, out.status)
        assertFalse(seq.windowOpen)
    }

    @Test
    fun aStaleFailedWithSamplesIsNotARefusal() {
        // An earlier window that moved still has its samples; it is old news,
        // not this window's answer.
        val seq = AccelSixFace.Sequencer()
        assertNull(seq.onFrame(window(reading(Face.Z_UP), state = AccelFaceState.FAILED, n = 2880)))
        assertNull(seq.outcome)
    }

    @Test
    fun afterMeasuringAFailedIsTheResult_andASecondMeasuringIsNot() {
        val seq = AccelSixFace.Sequencer()
        assertNull(seq.onFrame(window(listOf(0.0, 0.0, 0.0), state = AccelFaceState.MEASURING, n = 0)))
        // A repeated 78 inside the window answers MEASURING again.
        assertNull(seq.onFrame(window(listOf(0.0, 0.0, 0.0), state = AccelFaceState.MEASURING, n = 0)))
        assertNull(seq.onFrame(window(listOf(0.0, 0.0, 0.0), state = AccelFaceState.IDLE, n = 0)))
        val moved = window(reading(Face.Y_UP), state = AccelFaceState.FAILED)
        assertEquals(moved, assertIs<Measurement.Finished>(seq.onFrame(moved)).window)
    }

    // ── the save's read-back ─────────────────────────────────────────────

    @Test
    fun aSaveTakesWhenStoredAndEachComponentReadsBackWithin1e5() {
        val sent = listOf(0.0412f, -0.0273f, 0.0156f)
        val back = window(reading(Face.Z_UP), flags = AccelCalStatus.FLAG_STORED or AccelCalStatus.FLAG_APPLIED, offset = sent)
        assertTrue(AccelSixFace.savedAsSent(sent, back))
        assertFalse(AccelSixFace.savedAsSent(sent, back.copy(offsetY = -0.0273f + 2e-5f)))
        assertFalse(AccelSixFace.savedAsSent(sent, back.copy(flags = 0)), "not stored")
        assertFalse(AccelSixFace.savedAsSent(sent, null), "no answer at all")
        assertTrue(AccelSixFace.cleared(back.copy(flags = 0, offsetX = 0f, offsetY = 0f, offsetZ = 0f)))
        assertFalse(AccelSixFace.cleared(back))
        assertFalse(AccelSixFace.cleared(null))
        // The read-back is preferred; the answer to 76 stands in without one.
        assertSame(back, AccelSixFace.Applied(answer = null, readBack = back).final)
        assertSame(back, AccelSixFace.Applied(answer = back, readBack = null).final)
    }

    @Test
    fun aWriteReportConfirmsBySaveOrClear() {
        val sent = listOf(0.0412f, -0.0273f, 0.0156f)
        val stored = window(reading(Face.Z_UP), flags = AccelCalStatus.FLAG_STORED, offset = sent)
        val none = stored.copy(flags = 0, offsetX = 0f, offsetY = 0f, offsetZ = 0f)
        val zeros = listOf(0f, 0f, 0f)
        assertTrue(AccelSixFace.WriteReport(isClear = false, sent = sent, readBack = stored).confirmed)
        assertFalse(AccelSixFace.WriteReport(isClear = false, sent = sent, readBack = none).confirmed)
        assertTrue(AccelSixFace.WriteReport(isClear = true, sent = zeros, readBack = none).confirmed)
        assertFalse(AccelSixFace.WriteReport(isClear = true, sent = zeros, readBack = stored).confirmed)
        assertFalse(AccelSixFace.WriteReport(isClear = false, sent = sent, readBack = null).confirmed)
        assertFalse(AccelSixFace.WriteReport(isClear = true, sent = zeros, readBack = null).confirmed)
    }

    // ── the gate ─────────────────────────────────────────────────────────

    /** iOS AccelCalibrationView.blockedReason, case for case and in its order. */
    @Test
    fun theGateIsIosBlockedReason() {
        fun gate(
            connected: Boolean = true,
            bs: Boolean = false,
            power: Boolean = true,
            state: String = "READY",
            booting: Boolean = false,
            sim: Boolean = false,
        ) = AccelSixFace.access(connected, bs, power, state, booting, sim)

        for (state in listOf("READY", "PRELAUNCH", "UNKNOWN")) assertEquals(Access.READY, gate(state = state), state)
        // A base station first, whatever else holds.
        assertEquals(Access.BASE_STATION, gate(bs = true, connected = false, power = false))
        // Not connected and not powered are one case.
        assertEquals(Access.OFF, gate(connected = false))
        assertEquals(Access.OFF, gate(power = false))
        // Starting: the state, or a boot step still reported.
        assertEquals(Access.STARTING, gate(state = "INITIALIZATION"))
        assertEquals(Access.STARTING, gate(booting = true, state = "LANDED"))
        assertEquals(Access.FLYING, gate(state = "INFLIGHT"))
        assertEquals(Access.LANDED, gate(state = "LANDED"))
        assertEquals(Access.MAG_CAL, gate(state = "MAG_CAL"))
        // The state is read before the sim latch.
        assertEquals(Access.SIMULATING, gate(sim = true))
        assertEquals(Access.FLYING, gate(sim = true, state = "INFLIGHT"))

        // The entries: iOS's canCalibrate, the rocket itself, connected and powered.
        assertTrue(AccelSixFace.canCalibrate(isConnected = true, isBaseStation = false, pwrPinOn = true))
        assertFalse(AccelSixFace.canCalibrate(isConnected = true, isBaseStation = false, pwrPinOn = false))
        assertFalse(AccelSixFace.canCalibrate(isConnected = false, isBaseStation = false, pwrPinOn = true))
        assertFalse(AccelSixFace.canCalibrate(isConnected = true, isBaseStation = true, pwrPinOn = true))
    }
}
