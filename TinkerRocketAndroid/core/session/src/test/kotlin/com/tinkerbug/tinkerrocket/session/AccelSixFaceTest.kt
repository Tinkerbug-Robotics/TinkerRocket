package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.AccelCalStatus
import com.tinkerbug.tinkerrocket.protocol.AccelFaceState
import com.tinkerbug.tinkerrocket.protocol.BoardOrientation
import com.tinkerbug.tinkerrocket.protocol.IMUOrientationMode
import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Access
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Face
import com.tinkerbug.tinkerrocket.session.AccelSixFace.LiveFace
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

    // ── the solve: a sphere fit over all six faces ───────────────────────

    /**
     * A face's reading with its axis [tiltDeg] off vertical, leaning toward
     * [azimuthRad] around it, through the board's [scl] and [offset] and the
     * wire's f32.
     */
    private fun leaning(
        face: Face,
        tiltDeg: Double,
        azimuthRad: Double,
        offset: List<Double>,
        scl: List<Double>,
    ): List<Double> {
        val t = Math.toRadians(tiltDeg)
        val u = DoubleArray(3)
        u[face.axis] = face.sign * cos(t)
        u[(face.axis + 1) % 3] = sin(t) * cos(azimuthRad)
        u[(face.axis + 2) % 3] = sin(t) * sin(azimuthRad)
        return (0..2).map { (offset[it] + scl[it] * g * u[it]).toFloat().toDouble() }
    }

    /** The user's first hand-held run on the Beetle: faces 3.7-9.1° off square, unequal within a pair. */
    private val userTilts = mapOf(
        Face.X_UP to 9.1, Face.X_DOWN to 5.0,
        Face.Y_UP to 6.9, Face.Y_DOWN to 4.5,
        Face.Z_UP to 3.7, Face.Z_DOWN to 9.0,
    )

    @Test
    fun squareFacesGiveTheOffsetAndScaleBack() {
        val s = AccelSixFace.solve(allSquare())
        assertTrue(s.converged)
        for (i in 0..2) {
            assertEquals(offMg[i], s.offsetMg[i], 1e-3, "offset axis $i")
            assertEquals(scale[i], s.scale[i], 1e-6, "scale axis $i")
            assertEquals((scale[i] - 1) * 100, s.scalePct[i], 1e-4)
        }
        assertTrue(s.maxResidualMps2 < AccelSixFace.FIT_RESIDUAL_TOL_MPS2)
        assertTrue(s.saveAllowed)
        assertEquals(s.offset.map { it.toFloat() }, s.wireOffset)
    }

    /**
     * The fit over the user's tilts: 200 boards (~15 mg offsets, ~0.4 % scale
     * errors), each face leaning a random way.  The per-axis average — the
     * old solve, and the fit's seed — leaves g·(cos t_up − cos t_down)/2 in
     * each offset, several mg with these tilts; six faces on one sphere leave
     * nothing.
     */
    @Test
    fun theUsersTiltsDoNotBiasTheFit_theyBiasedThePerAxisAverage() {
        val rnd = java.util.Random(1629)
        var worstFit = 0.0
        var worstSquare = 0.0
        var worstAverage = 0.0
        var mostSteps = 0
        repeat(200) { trial ->
            val o = List(3) { rnd.nextGaussian() * 15e-3 * g }
            val k = List(3) { 1 + rnd.nextGaussian() * 0.004 }
            val faces = Face.entries.associateWith { f ->
                leaning(f, userTilts.getValue(f), rnd.nextDouble() * 2 * Math.PI, o, k)
            }
            val s = AccelSixFace.solve(faces)
            assertTrue(s.converged, "trial $trial")
            assertTrue(s.saveAllowed, "trial $trial")
            mostSteps = maxOf(mostSteps, s.iterations)
            worstFit = maxOf(worstFit, (0..2).maxOf { abs(s.offsetMg[it] - AccelSixFace.mg(o[it])) })
            worstAverage = maxOf(
                worstAverage,
                (0..2).maxOf {
                    val average = 0.5 * (faces.getValue(Face.of(it, 1))[it] + faces.getValue(Face.of(it, -1))[it])
                    abs(AccelSixFace.mg(average - o[it]))
                },
            )
            val square = AccelSixFace.solve(Face.entries.associateWith { f -> leaning(f, 0.0, 0.0, o, k) })
            assertTrue(square.converged, "trial $trial, square")
            worstSquare = maxOf(worstSquare, (0..2).maxOf { abs(square.offsetMg[it] - AccelSixFace.mg(o[it])) })
        }
        assertTrue(worstFit < 1.5, "the fit's worst offset error $worstFit mg")
        assertTrue(worstSquare < 0.1, "square faces' worst $worstSquare mg")
        assertTrue(worstAverage > 1.5, "the per-axis average's worst $worstAverage mg: these tilts must defeat it")
        assertTrue(mostSteps <= 6, "converges in a few steps, took $mostSteps")
    }

    @Test
    fun anOffsetPastOneMetrePerSecondSquaredIsNotSaved() {
        // A face measured the wrong way up, or moving, solves to nonsense the
        // flight computer would refuse: the screen says "redo the faces".
        val s = AccelSixFace.solve(allSquare(offset = listOf(0.2, 1.2, -0.1)))
        assertTrue(s.converged)
        assertEquals(1.2, s.offset[1], 1e-5)
        assertEquals(listOf(1), s.axesPastLimit)
        assertFalse(s.saveAllowed)
        // The limit itself still saves (|offset| <= 1.0).
        val edge = AccelSixFace.Solution(
            offset = listOf(1.0, -1.0, 0.0), scale = unit, converged = true, iterations = 3, maxResidualMps2 = 0.0,
        )
        assertTrue(edge.saveAllowed)
        assertEquals(listOf(1), edge.copy(offset = listOf(0.0, -1.0000001, 0.0)).axesPastLimit)
    }

    @Test
    fun aScaleFarOutOfRange_orFacesThatCannotBeSolved_doNotSave() {
        // A 6 % scale fits — the faces agree with each other — but is no
        // accelerometer this board carries: redo the faces.
        val far = AccelSixFace.solve(allSquare(scl = listOf(1.06, 1.0, 1.0)))
        assertTrue(far.converged)
        assertEquals(1.06, far.scale[0], 1e-6)
        assertEquals(listOf(0), far.axesScaleOutOfRange)
        assertFalse(far.saveAllowed)
        // +X DOWN read the same as +X UP: nothing to solve the X axis from.
        val faces = allSquare() + (Face.X_DOWN to allSquare().getValue(Face.X_UP))
        val bad = AccelSixFace.solve(faces)
        assertFalse(bad.converged)
        assertFalse(bad.saveAllowed)
        // 0.97..1.03 is inclusive; an unconverged fit never saves.
        val edge = AccelSixFace.Solution(
            offset = flat, scale = listOf(0.97, 1.0, 1.03), converged = true, iterations = 3, maxResidualMps2 = 0.0,
        )
        assertTrue(edge.saveAllowed)
        assertEquals(listOf(2), edge.copy(scale = listOf(0.97, 1.0, 1.0300001)).axesScaleOutOfRange)
        assertFalse(edge.copy(converged = false).saveAllowed)
    }

    @Test
    fun theSolveNeedsAllSixFaces() {
        assertFailsWith<IllegalArgumentException> {
            AccelSixFace.solve(allSquare() - Face.Y_DOWN)
        }
    }

    // ── the live face, from telemetry ────────────────────────────────────

    private fun frame(x: Float?, y: Float?, z: Float?, imo: Int?) =
        TelemetryData(lowGX = x, lowGY = y, lowGZ = z, imuOrientPacked = imo)

    /** "imo" as the out computer packs it: (mode << 5) | code, here the pad auto-detect (3). */
    private fun imo(code: Int) = (3 shl 5) or code

    private fun assertOn(face: Face, tiltDeg: Double, live: LiveFace) {
        val on = assertIs<LiveFace.On>(live)
        assertEquals(face, on.face)
        assertEquals(tiltDeg, on.tiltDeg, 1e-4)
    }

    @Test
    fun theLiveFaceTurnsTheReadingBackIntoTheBoardFrame() {
        // +Z toward the nose (code 16) and +g along the rocket's nose: the board's +Z is up.
        assertOn(Face.Z_UP, 0.0, AccelSixFace.liveFace(frame(9.8f, 0f, 0f, imo(16))))
        // Identity mounting (code 0): the rocket's +Y is the board's.
        assertOn(Face.Y_UP, 0.0, AccelSixFace.liveFace(frame(0f, 9.8f, 0f, imo(0))))
        // +Y toward the nose (code 8) and -g along the nose: the board's +Y points down.
        assertOn(Face.Y_DOWN, 0.0, AccelSixFace.liveFace(frame(-9.8f, 0f, 0f, imo(8))))
        // Mode does not matter, only the code: a manual mounting (mode 2) reads the same.
        assertOn(Face.Z_UP, 0.0, AccelSixFace.liveFace(frame(9.8f, 0f, 0f, (2 shl 5) or 16)))
        assertEquals(Face.Z_UP, AccelSixFace.liveFace(frame(9.8f, 0f, 0f, imo(16))).face)
    }

    /**
     * The rocket's own link carries no "imo": its frames are turned back with
     * the "imu_orient" message's code, as on iOS.  A frame's own "imo" wins
     * (the base station relays it with the reading).  Auto-exact (mode 3)
     * keeps its code, marked exact: it names the axis nearest "up".
     */
    @Test
    fun theMountingIsTheFramesImo_elseTheImuOrientMessagesCode() {
        assertEquals(
            AccelSixFace.LiveMounting(0, exact = false),
            AccelSixFace.liveMounting(imo(0), 16, IMUOrientationMode.AUTO_EXACT),
        )
        for (mode in listOf(IMUOrientationMode.DEFAULT_MOUNTING, IMUOrientationMode.MANUAL, IMUOrientationMode.AUTO_SNAP)) {
            assertEquals(AccelSixFace.LiveMounting(16, exact = false), AccelSixFace.liveMounting(null, 16, mode), mode.name)
        }
        assertEquals(
            AccelSixFace.LiveMounting(16, exact = true),
            AccelSixFace.liveMounting(null, 16, IMUOrientationMode.AUTO_EXACT),
        )
        assertNull(AccelSixFace.liveMounting(null, 16, IMUOrientationMode.UNKNOWN))
        assertNull(AccelSixFace.liveMounting(null, null, IMUOrientationMode.AUTO_SNAP))
        // A direct link's frame: no imo, the message said +Z toward the nose.
        val direct = frame(9.8f, 0f, 0f, imo = null)
        assertOn(Face.Z_UP, 0.0, AccelSixFace.liveFace(direct, reportedCode = 16, reportedMode = IMUOrientationMode.AUTO_SNAP))
        assertSame(LiveFace.Waiting, AccelSixFace.liveFace(direct))
    }

    /**
     * Auto-exact: the flight computer applies the exact rotation and reports
     * beside it only the board axis nearest "up" (orientNearestNoseCode), so
     * the face is right but a reading turned back through that code always
     * looks square.  The face, then, without a tilt — never askew — while
     * modes 1 and 2 keep theirs; and only once the reading sits within 5° of
     * rocket +X, as it does whenever the flight computer has read the board
     * where it now lies.
     */
    @Test
    fun anExactMountingNamesTheFace_butNotItsTilt() {
        val exact = IMUOrientationMode.AUTO_EXACT
        assertEquals(
            LiveFace.OnNoTilt(Face.Z_UP),
            AccelSixFace.liveFace(frame(9.8f, 0f, 0f, imo = null), reportedCode = 16, reportedMode = exact),
        )
        // 4° off rocket +X under code 16: settled, the face alone in mode 3, the tilt in modes 1 and 2.
        val settled = Math.toRadians(4.0)
        val near = frame((g * cos(settled)).toFloat(), (g * sin(settled)).toFloat(), 0f, imo = null)
        assertEquals(LiveFace.OnNoTilt(Face.Z_UP), AccelSixFace.liveFace(near, 16, exact))
        assertOn(Face.Z_UP, 4.0, AccelSixFace.liveFace(near, 16, IMUOrientationMode.AUTO_SNAP))
        // 15° off: askew in modes 1 and 2; in mode 3 a board the flight computer has not re-read yet.
        val t = Math.toRadians(15.0)
        val leaning = frame((g * cos(t)).toFloat(), (g * sin(t)).toFloat(), 0f, imo = null)
        assertSame(LiveFace.Moving, AccelSixFace.liveFace(leaning, 16, exact))
        val askew = assertIs<LiveFace.Askew>(AccelSixFace.liveFace(leaning, 16, IMUOrientationMode.AUTO_SNAP))
        assertEquals(15.0, askew.tiltDeg, 1e-4)
        assertIs<LiveFace.Askew>(AccelSixFace.liveFace(leaning, 16, IMUOrientationMode.MANUAL))
        // |v| is the same in any frame: a board being moved still says so.
        assertSame(LiveFace.Moving, AccelSixFace.liveFace(frame(8.5f, 0f, 0f, imo = null), 16, exact))
        assertEquals(Face.Z_UP, LiveFace.OnNoTilt(Face.Z_UP).face)
    }

    /**
     * Why mode 3 waits for the reading to settle: the flight computer turns
     * the reading with the exact rotation it last read (orientMatrixFromNoseVector,
     * ported below) but names the nearest axis's code beside it, so a board set
     * on another face reads, through that code, as some other face until the
     * flight computer re-reads it (after 2 s still).  Over exact mountings
     * 16-40° off each axis and the board then set on any face: never a face it
     * is not on, where the bare turn-back names a wrong one.
     */
    @Test
    fun anExactMountingNeverNamesAFaceTheBoardIsNotOn() {
        val exact = IMUOrientationMode.AUTO_EXACT
        var wrongWithoutTheWait = 0
        for (nose in Face.entries) for (lean in listOf(16.0, 25.0, 40.0)) for (az in 0 until 360 step 30) {
            val n = leaned(nose, lean, az.toDouble())
            val r = exactRotation(n)
            val code = nearestNoseCode(n)
            // Settled: the reading is the nose the flight computer read.
            assertEquals(LiveFace.OnNoTilt(nose), AccelSixFace.liveFace(turned(r, n), code, exact), "$nose $lean $az")
            for (now in Face.entries) for (tilt in listOf(0.0, 6.0)) {
                val v = leaned(now, tilt, az + 45.0)
                val live = AccelSixFace.liveFace(turned(r, v), code, exact)
                assertTrue(live === LiveFace.Moving || live.face == now, "$nose $lean $az on $now: $live")
                val bare = AccelSixFace.identify(BoardOrientation.toBoard(code, rotate(r, v)))
                if (bare.face != now) wrongWithoutTheWait++
            }
        }
        assertTrue(wrongWithoutTheWait > 0, "the case the wait is for")
    }

    /** A unit vector [leanDeg] off [face]'s up direction, at [azDeg] around it. */
    private fun leaned(face: Face, leanDeg: Double, azDeg: Double): List<Double> {
        val u = DoubleArray(3).also { it[face.axis] = face.sign.toDouble() }
        val p1 = DoubleArray(3).also { it[(face.axis + 1) % 3] = 1.0 }
        val p2 = DoubleArray(3).also { it[(face.axis + 2) % 3] = 1.0 }
        val l = Math.toRadians(leanDeg)
        val a = Math.toRadians(azDeg)
        return (0..2).map { cos(l) * u[it] + sin(l) * (cos(a) * p1[it] + sin(a) * p2[it]) }
    }

    /** orientMatrixFromNoseVector: the shortest arc taking unit [a] to rocket +X, row-major. */
    private fun exactRotation(a: List<Double>): DoubleArray {
        val c = a[0]
        if (c < -1.0 + 1e-6) return doubleArrayOf(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0)
        val ky = a[2]
        val kz = -a[1]
        val f = 1.0 / (1.0 + c)
        return doubleArrayOf(
            1.0 - f * (ky * ky + kz * kz), -kz, ky,
            kz, 1.0 - f * kz * kz, f * ky * kz,
            -ky, f * ky * kz, 1.0 - f * ky * ky,
        )
    }

    /** orientNearestNoseCode: the clock-0 code of the board axis nearest [v]. */
    private fun nearestNoseCode(v: List<Double>): Int {
        var axis = 0
        for (i in 1..2) if (abs(v[i]) > abs(v[axis])) axis = i
        return (axis * 2 + if (v[axis] < 0) 1 else 0) * 4
    }

    private fun rotate(r: DoubleArray, v: List<Double>): List<Double> =
        (0..2).map { i -> g * (r[3 * i] * v[0] + r[3 * i + 1] * v[1] + r[3 * i + 2] * v[2]) }

    /** A direct-link frame whose lx/ly/lz are the board's one g along [v], turned by [r]. */
    private fun turned(r: DoubleArray, v: List<Double>): TelemetryData {
        val w = rotate(r, v)
        return frame(w[0].toFloat(), w[1].toFloat(), w[2].toFloat(), imo = null)
    }

    @Test
    fun theLiveFaceSaysAskewMovingOrWaiting() {
        val t = Math.toRadians(15.0)
        val askew = assertIs<LiveFace.Askew>(
            AccelSixFace.liveFace(frame((g * sin(t)).toFloat(), 0f, (g * cos(t)).toFloat(), imo(0))),
        )
        assertEquals(Face.Z_UP, askew.face)
        assertEquals(15.0, askew.tiltDeg, 1e-4)
        val square = Math.toRadians(7.0)
        assertOn(Face.Z_UP, 7.0, AccelSixFace.liveFace(frame((g * sin(square)).toFloat(), 0f, (g * cos(square)).toFloat(), imo(0))))
        // |v| 8.5: more than 0.1 g short of one g.
        assertSame(LiveFace.Moving, AccelSixFace.liveFace(frame(0f, 0f, 8.5f, imo(0))))
        assertNull(LiveFace.Moving.face)
        // No frame yet, a field missing from the newest, or no discrete code (31 = exact auto).
        assertSame(LiveFace.Waiting, AccelSixFace.liveFace(TelemetryData()))
        assertSame(LiveFace.Waiting, AccelSixFace.liveFace(frame(0f, null, 9.8f, imo(0))))
        assertSame(LiveFace.Waiting, AccelSixFace.liveFace(frame(0f, 0f, 9.8f, null)))
        assertSame(LiveFace.Waiting, AccelSixFace.liveFace(frame(0f, 0f, 9.8f, imo(31))))
        assertSame(LiveFace.Waiting, AccelSixFace.liveFace(frame(0f, 0f, 9.8f, imo(24))))
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
