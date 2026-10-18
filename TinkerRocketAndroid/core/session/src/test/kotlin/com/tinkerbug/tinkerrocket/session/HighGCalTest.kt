package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.AccelCalStatus
import com.tinkerbug.tinkerrocket.protocol.AccelFaceState
import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.protocol.HighGBiasSource
import com.tinkerbug.tinkerrocket.protocol.HighGCalApply
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Face
import com.tinkerbug.tinkerrocket.session.AccelSixFace.HighGProblem
import com.tinkerbug.tinkerrocket.session.AccelSixFace.HighGResult
import com.tinkerbug.tinkerrocket.session.AccelSixFace.HighGSolution
import com.tinkerbug.tinkerrocket.session.AccelSixFace.HighGWrite
import com.tinkerbug.tinkerrocket.session.AccelSixFace.HighGWriteReport
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.async
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.currentTime
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import java.util.Random
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertIs
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * The high-g accelerometer's six-face alignment to the low-g
 * (RocketComputerTypes.h, HG_CAL_*): the fit (the cases
 * tests/unit/test_accel_cal_six_face.py pins on the reference script and iOS
 * HighGCalTests.swift on the app), its words, the save and clear on the wire
 * through a session, and the rocket profile's bias after a save.  The frames'
 * decode and command 80's bytes are pinned against the goldens in
 * :core:protocol (FileOpsDispatchGoldenTest, CommandsGoldenTest).
 */
class HighGCalTest {

    private val g = AccelSixFace.G0
    private val mg = AccelSixFace.G0 / 1000

    // ── Synthetic boards ─────────────────────────────────────────────────

    // A board's two accelerometers as real ones differ: reading = S a + offset,
    // the high-g with a full 3x3 S.  The fit must find l = M h − b with
    // M = S_lg S_hg⁻¹ and b = M c_hg − o_lg.
    private val sLG = listOf(1.0006, 0.9991, 1.0004)
    private val oLG = listOf(-0.0155, -0.1324, -0.0306)
    private val sHG = listOf(0.9921, -0.0047, 0.0030, 0.0029, 1.0068, -0.0054, -0.0016, 0.0051, 0.9880)
    private val cHG = listOf(0.98, -1.47, 0.62)

    /**
     * One 3 s window at 3780 Hz on the V9 (the 2026-10-17 bench log): the
     * low-g's and the high-g's sample SD, m/s² (1.9 and 57 mg).
     */
    private val sdLG = 0.019
    private val sdHG = 0.56
    private val nSamples = 11340

    /** The user's first run on the Beetle: each face's tilt off its axis, deg. */
    private val userTilts = mapOf(
        Face.X_UP to 9.1, Face.X_DOWN to 5.0, Face.Y_UP to 6.9, Face.Y_DOWN to 4.5,
        Face.Z_UP to 3.7, Face.Z_DOWN to 9.0,
    )

    private fun mul(m: List<Double>, v: List<Double>): List<Double> =
        (0..2).map { i -> m[3 * i] * v[0] + m[3 * i + 1] * v[1] + m[3 * i + 2] * v[2] }

    private fun inv3(a: List<Double>): List<Double> {
        val c00 = a[4] * a[8] - a[5] * a[7]
        val c01 = a[5] * a[6] - a[3] * a[8]
        val c02 = a[3] * a[7] - a[4] * a[6]
        val det = a[0] * c00 + a[1] * c01 + a[2] * c02
        val r0 = listOf(c00, a[2] * a[7] - a[1] * a[8], a[1] * a[5] - a[2] * a[4])
        val r1 = listOf(c01, a[0] * a[8] - a[2] * a[6], a[2] * a[3] - a[0] * a[5])
        val r2 = listOf(c02, a[1] * a[6] - a[0] * a[7], a[0] * a[4] - a[1] * a[3])
        return (r0 + r1 + r2).map { it / det }
    }

    /** M and b the fit should find on this board. */
    private fun truth(): Pair<List<Double>, List<Double>> {
        val si = inv3(sHG)
        val m = (0..8).map { k -> sLG[k / 3] * si[k] }
        val mc = mul(m, cHG)
        return m to (0..2).map { mc[it] - oLG[it] }
    }

    /**
     * Six face windows: gravity tilted off each face's axis (towards an
     * azimuth that turns face by face), each channel's mean of it with
     * optional Gaussian noise of SD / √n, and the high-g's even-order
     * nonlinearity quad_i · a_i².  Through the wire's f32, like real 0xD1
     * windows.
     */
    private fun board(
        tilts: Map<Face, Double>,
        noiseSeed: Long? = null,
        sHG: List<Double> = this.sHG,
        cHG: List<Double> = this.cHG,
        quad: List<Double> = listOf(0.0, 0.0, 0.0),
        withHighG: Boolean = true,
    ): Map<Face, AccelCalStatus> {
        val noise = noiseSeed?.let { Random(it) }
        val rootN = sqrt(nSamples.toDouble())
        return Face.entries.withIndex().associate { (k, face) ->
            val t = Math.toRadians(tilts[face] ?: 0.0)
            val az = Math.toRadians(30.0 + 50 * k)
            val a = DoubleArray(3)
            a[face.axis] = face.sign * g * cos(t)
            a[(face.axis + 1) % 3] = g * sin(t) * cos(az)
            a[(face.axis + 2) % 3] = g * sin(t) * sin(az)
            var l = (0..2).map { sLG[it] * a[it] + oLG[it] }
            val sa = mul(sHG, a.toList())
            var h = (0..2).map { sa[it] + cHG[it] + quad[it] * a[it] * a[it] }
            if (noise != null) {
                l = l.map { it + noise.nextGaussian() * sdLG / rootN }
                h = h.map { it + noise.nextGaussian() * sdHG / rootN }
            }
            face to window(l, h.takeIf { withHighG })
        }
    }

    /** A DONE window: low-g mean [l], and the high-g's [h] when the frame carries it. */
    private fun window(l: List<Double>, h: List<Double>?) = AccelCalStatus(
        flags = 0, faceState = AccelFaceState.DONE,
        offsetX = 0f, offsetY = 0f, offsetZ = 0f,
        faceMeanX = l[0].toFloat(), faceMeanY = l[1].toFloat(), faceMeanZ = l[2].toFloat(),
        faceSdX = sdLG.toFloat(), faceSdY = sdLG.toFloat(), faceSdZ = sdLG.toFloat(),
        faceN = nSamples,
        highG = h?.let {
            AccelCalStatus.HighG(
                faceMean = it.map { v -> v.toFloat() },
                faceSd = List(3) { sdHG.toFloat() },
                flags = 0, biasSource = 0,
                matrix = HighGCalApply.IDENTITY, bias = HighGCalApply.ZERO_BIAS,
            )
        },
    )

    private fun lowG(faces: Map<Face, AccelCalStatus>): AccelSixFace.Solution =
        AccelSixFace.solve(faces.mapValues { (_, w) -> w.faceMean.map { it.toDouble() } }).also {
            assertTrue(it.converged)
        }

    private fun fit(faces: Map<Face, AccelCalStatus>): HighGSolution =
        assertIs<HighGResult.Solved>(AccelSixFace.solveHighG(faces, lowG(faces))).solution

    // ── The fit ──────────────────────────────────────────────────────────

    @Test
    fun theAlignmentRecoversScaleCrossAxisAndOffset_whateverTheTilts() {
        val (wantM, wantB) = truth()
        for (tilts in listOf(userTilts, emptyMap())) {
            val s = fit(board(tilts))
            assertEquals(emptyList(), s.problems)
            assertEquals(emptyList(), s.disagreeAxes)
            // the means went through f32 on the wire: a few parts in 1e7
            for (k in 0..8) assertEquals(wantM[k], s.matrix[k], 2e-6, "M[$k]")
            for (i in 0..2) assertEquals(wantB[i], s.bias[i], 0.01 * mg, "b[$i]")
            assertTrue(s.residualMax < 0.01 * mg)
            for (i in 0..2) {
                assertEquals(cHG[i], s.ownOffset[i], 0.2 * mg, "the sphere fit has no cross-axis terms")
                assertEquals(cHG[i], s.impliedZero[i], 0.01 * mg)
            }
            assertTrue(s.saveAllowed)
        }
    }

    @Test
    fun withBenchNoise_theErrorsMatchTheReportedSigma() {
        val (wantM, wantB) = truth()
        val zM = ArrayList<Double>()
        val zB = ArrayList<Double>()
        var falseAlarms = 0
        val trials = 200
        var last: HighGSolution? = null
        for (trial in 0 until trials) {
            val s = fit(board(userTilts, noiseSeed = 1629L + trial * 1_000_003L))
            assertEquals(emptyList(), s.problems)
            for (k in 0..8) zM += (s.matrix[k] - wantM[k]) / s.sigmaMatrix[k]
            for (i in 0..2) zB += (s.bias[i] - wantB[i]) / s.sigmaBias[i]
            if (s.disagreeAxes.isNotEmpty()) falseAlarms++
            last = s
        }
        fun rms(z: List<Double>) = sqrt(z.sumOf { it * it } / z.size)
        assertTrue(rms(zM) in 0.3..1.1, "M's z-scores rms ${rms(zM)}")
        assertTrue(rms(zB) in 0.3..1.1, "b's z-scores rms ${rms(zB)}")
        assertTrue(falseAlarms.toDouble() / trials < 0.02, "$falseAlarms false alarms in $trials")
        // what a 3 s face buys: the scale to a few hundredths of a percent, the offset to a few tenths of a mg
        val s = assertNotNull(last)
        assertTrue(listOf(0, 4, 8).maxOf { s.sigmaMatrix[it] } < 5e-4)
        assertTrue(s.sigmaBias.max() < 0.5 * mg)
    }

    @Test
    fun aDisagreeingHighGIsFlagged_andTheLowGAlignmentStillSaved() {
        // an even-order nonlinearity of 4 mg at one g on the high-g's Z: its own
        // gravity fit takes it all as offset, the alignment a third of it
        val q = 4 * mg / (g * g)
        val s = fit(board(userTilts, quad = listOf(0.0, 0.0, q)))
        assertEquals(listOf(2), s.disagreeAxes)
        assertTrue(abs(s.diffOffset[2]) > 2 * mg)
        assertEquals(emptyList(), s.problems, "the alignment to the low-g is what is saved")
        val agreement = AccelCalReadout.hgAgreement(s)
        assertEquals(AccelCalReadout.Tone.WARN, agreement.tone)
        assertTrue(agreement.text.startsWith("Its own gravity fit differs on Z by "), agreement.text)
        assertTrue(
            agreement.text.endsWith("the alignment to the low-g, the bench's reference, is what gets saved."),
            agreement.text,
        )
        assertEquals(AccelCalReadout.Line("Passed: saved with the offset", AccelCalReadout.Tone.OK), AccelCalReadout.hgVerdict(s))
        // a worse one leaves a face further from the low-g than the limit
        val worse = fit(board(userTilts, quad = listOf(0.0, 0.0, 1.5 * q)))
        assertTrue(HighGProblem.RESIDUAL in worse.problems)
        assertFalse(worse.saveAllowed)
        assertEquals(
            AccelCalReadout.Line(
                "Not saved: A face fits the low-g worse than 3 mg — did it move, or warm up between faces? " +
                    "Measure the faces again.",
                AccelCalReadout.Tone.BAD,
            ),
            AccelCalReadout.hgVerdict(worse),
        )
    }

    @Test
    fun theHighGRefusals() {
        val scale = fit(board(userTilts, sHG = listOf(1.045, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)))
        assertEquals(listOf(HighGProblem.SCALE), scale.problems)
        assertEquals("A scale is outside 0.97–1.03.", AccelCalReadout.hgProblem(scale))
        val cross = fit(board(userTilts, sHG = listOf(1.0, 0.05, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)))
        assertEquals(listOf(HighGProblem.CROSS), cross.problems)
        assertEquals("A cross-axis term is past 0.03 (1.7°).", AccelCalReadout.hgProblem(cross))
        val bias = fit(board(userTilts, cHG = listOf(12.0, 0.0, 0.0)))
        assertEquals(listOf(HighGProblem.BIAS), bias.problems)
        assertEquals("An offset is past 11 m/s².", AccelCalReadout.hgProblem(bias))
        val faces = board(userTilts, withHighG = false)
        assertEquals(HighGResult.NoReading, AccelSixFace.solveHighG(faces, lowG(faces)))
        assertEquals(
            "The faces came without the high-g reading. Update the flight computer and the out computer " +
                "to calibrate the high-g; the low-g offset saves as before.",
            AccelCalReadout.hgUnavailable(HighGResult.NoReading),
        )
        assertEquals(
            "The high-g fit failed: are the six faces six different sides?",
            AccelCalReadout.hgUnavailable(HighGResult.Singular),
        )
        // a refused fit is not sent, an older one has none to send
        for (s in listOf(scale, cross, bias)) assertNull(AccelSixFace.highGAfterSave(HighGResult.Solved(s)))
        assertNull(AccelSixFace.highGAfterSave(HighGResult.NoReading))
        assertNull(AccelSixFace.highGAfterSave(null))
    }

    @Test
    fun aMissingFaceIsNoReading_andFacesThatSpanTooFewAxesAreSingular() {
        val faces = board(userTilts)
        val lowG = lowG(faces)
        assertEquals(HighGResult.NoReading, AccelSixFace.solveHighG(faces - Face.Y_DOWN, lowG))
        // every face's high-g reading the same: no spread to fit M on
        val flat = faces.mapValues { (_, w) -> w.copy(highG = w.highG!!.copy(faceMean = listOf(0f, 0f, 9.8f))) }
        assertEquals(HighGResult.Singular, AccelSixFace.solveHighG(flat, lowG))
    }

    @Test
    fun theScreensNumbers() {
        val s = fit(board(emptyMap()))
        val (wantM, wantB) = truth()
        fun f(v: Double, d: Int) = String.format(java.util.Locale.ROOT, "%.${d}f", v)
        fun signed(v: Double, d: Int) = String.format(java.util.Locale.ROOT, "%+.${d}f", v)
        assertEquals("scale ${signed((wantM[0] - 1) * 100, 3)} % ± ${f(s.sigmaMatrix[0] * 100, 3)}", AccelCalReadout.hgScale(s, 0))
        assertEquals("offset ${signed(wantB[1] / mg, 1)} mg ± ${f(s.sigmaBias[1] / mg, 2)}", AccelCalReadout.hgOffset(s, 1))
        assertTrue(AccelCalReadout.hgCross(s).startsWith("Cross-axis: up to 0."), AccelCalReadout.hgCross(s))
        assertTrue(AccelCalReadout.hgResidual(s).startsWith("Fit to the low-g: "), AccelCalReadout.hgResidual(s))
        assertTrue(AccelCalReadout.hgResidual(s).endsWith("(limit 3 mg)"), AccelCalReadout.hgResidual(s))
        assertNull(AccelCalReadout.hgProblem(s))
        assertEquals(
            AccelCalReadout.Line("Its own gravity fit agrees with the alignment to the low-g within their noise."),
            AccelCalReadout.hgAgreement(s),
        )
        // the wire carries the fit narrowed to f32
        assertEquals(HighGWrite.Save(s.wireMatrix, s.wireBias), AccelSixFace.highGAfterSave(HighGResult.Solved(s)))
    }

    // ── The stored rows and the reports ──────────────────────────────────

    /** A status frame saying what is stored, low-g and high-g. */
    private fun stored(
        offset: List<Float>?,
        matrix: List<Float>?,
        bias: List<Float> = listOf(0f, 0f, 0f),
        source: Int = HighGBiasSource.SIX_FACES.raw,
    ) = AccelCalStatus(
        flags = if (offset != null) AccelCalStatus.FLAG_STORED or AccelCalStatus.FLAG_APPLIED else 0,
        faceState = AccelFaceState.IDLE,
        offsetX = offset?.get(0) ?: 0f, offsetY = offset?.get(1) ?: 0f, offsetZ = offset?.get(2) ?: 0f,
        faceMeanX = 0f, faceMeanY = 0f, faceMeanZ = 0f, faceSdX = 0f, faceSdY = 0f, faceSdZ = 0f, faceN = 0,
        highG = AccelCalStatus.HighG(
            faceMean = listOf(0f, 0f, 0f), faceSd = listOf(0f, 0f, 0f),
            flags = (if (matrix == null) 0 else 0x03) or 0x04,
            biasSource = source,
            matrix = matrix ?: HighGCalApply.IDENTITY, bias = bias,
        ),
    )

    @Test
    fun theStoredRows() {
        // The golden's numbers (fileops/accelcal_0xD1_114.bin).
        val m = listOf(1.0081f, 0.0042f, -0.0031f, -0.0027f, 0.9934f, 0.0056f, 0.0018f, -0.0049f, 1.0122f)
        val h = stored(null, matrix = m, bias = listOf(-0.6512f, 1.48f, -0.3127f)).highG!!
        assertEquals("High-g alignment in use: X +0.810 %, Y -0.660 %, Z +1.220 %", AccelCalReadout.hgAlignment(h))
        assertEquals("High-g bias: X -66.4 mg, Y +150.9 mg, Z -31.9 mg", AccelCalReadout.hgBiasRow(h))
        assertEquals("from the six faces", AccelCalReadout.hgSource(h))
        assertEquals("From the six faces", AccelCalReadout.hgSourceRow(h))
        assertEquals(
            "High-g alignment stored, but not in use while a sim runs",
            AccelCalReadout.hgAlignment(h.copy(flags = 0x01 or 0x04)),
        )
        assertEquals("No high-g alignment stored", AccelCalReadout.hgAlignment(h.copy(flags = 0x04)))
        assertEquals("Measured by the pad calibration", AccelCalReadout.hgSourceRow(h.copy(biasSource = 1)))
        assertEquals("Source not recorded", AccelCalReadout.hgSourceRow(h.copy(biasSource = 0)))
        assertEquals("Source not recorded", AccelCalReadout.hgSourceRow(h.copy(biasSource = 9)), "a newer source")
    }

    @Test
    fun aHighGReadBackConfirmsBySaveOrClear() {
        val m = listOf(1.0081f, 0.0042f, -0.0031f, -0.0027f, 0.9934f, 0.0056f, 0.0018f, -0.0049f, 1.0122f)
        val b = listOf(-0.6512f, 1.48f, -0.3127f)
        val save = HighGWrite.Save(m, b)
        val o = listOf(0.0412f, -0.0273f, 0.0156f)
        assertTrue(HighGWriteReport(save, stored(o, m, b)).confirmed)
        assertEquals(
            AccelCalReadout.Line("High-g alignment saved and read back", AccelCalReadout.Tone.OK),
            AccelCalReadout.hgWrite(HighGWriteReport(save, stored(o, m, b))),
        )
        // a bias the pad calibration wrote since is not this save's
        val padCal = HighGWriteReport(save, stored(o, m, b, source = HighGBiasSource.PAD_CAL.raw))
        assertFalse(padCal.confirmed)
        assertEquals(
            AccelCalReadout.Line("The high-g read-back does not match what was sent", AccelCalReadout.Tone.WARN),
            AccelCalReadout.hgWrite(padCal),
        )
        // M within 1e-6, b within 1e-5 m/s²
        assertFalse(HighGWriteReport(save, stored(o, m.mapIndexed { i, v -> if (i == 4) v + 2e-6f else v }, b)).confirmed)
        assertFalse(HighGWriteReport(save, stored(o, m, b.map { it + 2e-5f })).confirmed)
        assertFalse(HighGWriteReport(save, stored(o, null, b)).confirmed, "nothing stored")

        assertTrue(HighGWriteReport(HighGWrite.Clear, stored(null, null)).confirmed)
        assertEquals(
            AccelCalReadout.Line("Cleared: no high-g alignment stored", AccelCalReadout.Tone.OK),
            AccelCalReadout.hgWrite(HighGWriteReport(HighGWrite.Clear, stored(null, null))),
        )
        assertEquals(
            AccelCalReadout.Line("The flight computer still has a high-g alignment stored", AccelCalReadout.Tone.WARN),
            AccelCalReadout.hgWrite(HighGWriteReport(HighGWrite.Clear, stored(null, m))),
        )
        // no answer, and an answer with no high-g in it
        assertEquals(
            AccelCalReadout.Line(
                "No answer about the high-g from the flight computer — read it back to check",
                AccelCalReadout.Tone.WARN,
            ),
            AccelCalReadout.hgWrite(HighGWriteReport(save, null)),
        )
        val old = stored(o, null).copy(highG = null)
        for (w in listOf(save, HighGWrite.Clear)) {
            assertFalse(HighGWriteReport(w, old).confirmed)
            assertEquals(
                AccelCalReadout.Line(
                    "The answer has no high-g in it: update the flight computer and the out computer",
                    AccelCalReadout.Tone.WARN,
                ),
                AccelCalReadout.hgWrite(HighGWriteReport(w, old)),
            )
        }
    }

    @Test
    fun aClearTakesTheHighGOnlyWhenOneIsStored() {
        assertEquals(HighGWrite.Clear, AccelSixFace.highGAfterClear(stored(null, listOf(1.01f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f))))
        assertNull(AccelSixFace.highGAfterClear(stored(null, null)), "a bias alone is the pad calibration's")
        assertNull(AccelSixFace.highGAfterClear(stored(null, null).copy(highG = null)), "an older flight computer")
        assertNull(AccelSixFace.highGAfterClear(null))
        // what command 80 carries for each
        assertEquals(HighGCalApply.OP_CLEAR, HighGWrite.Clear.op)
        assertEquals(HighGCalApply.IDENTITY, HighGWrite.Clear.matrix)
        assertEquals(listOf(0f, 0f, 0f), HighGWrite.Clear.bias)
        assertEquals(HighGCalApply.OP_STORE, HighGWrite.Save(HighGCalApply.IDENTITY, listOf(1f, 2f, 3f)).op)
    }

    // ── On the wire, through a session ───────────────────────────────────

    @Test
    fun theFakesFrame_decodesToWhatItWasGiven() {
        // The decoder is pinned to the C++ goldens in :core:protocol; the
        // session tests' frames must go through it unchanged.
        val m = listOf(1.0081f, 0.0042f, -0.0031f, -0.0027f, 0.9934f, 0.0056f, 0.0018f, -0.0049f, 1.0122f)
        val h = stored(listOf(0.0412f, -0.0273f, 0.0156f), m, listOf(-0.6512f, 1.48f, -0.3127f)).highG!!
            .copy(faceMean = listOf(0.9012f, -1.3577f, 10.4419f), faceSd = listOf(0.5618f, 0.5613f, 0.5252f))
        val fw = FakeFirmware(CoroutineScope(Dispatchers.Unconfined))
        val frame = fw.accelCalStatusFrame(flags = 3, faceState = 2, faceN = 2880, highG = h)
        assertEquals(1 + AccelCalStatus.HIGH_G_SIZE, frame.size)
        val s = assertNotNull(AccelCalStatus.decode(frame.copyOfRange(1, frame.size)))
        assertEquals(h, s.highG)
        assertEquals(2880, s.faceN)
        assertNull(AccelCalStatus.decode(fw.accelCalStatusFrame().copyOfRange(1, 41))?.highG)
    }

    private class Harness(val session: DeviceSession, val fw: FakeFirmware) {
        fun sentSince(before: Int): List<Int> = fw.commandFrames.drop(before).map { it[0].toInt() and 0xFF }
    }

    private fun TestScope.connected(board: FakeFirmware.AccelCalBoard?): Harness {
        val fw = FakeFirmware(backgroundScope).apply { accelCal = board }
        val session = DeviceSession(
            scope = backgroundScope,
            transport = fw,
            connectedDeviceName = "TR-Test",
            initialDeviceType = BleDeviceType.ROCKET,
            clock = { currentTime },
        )
        session.start()
        advanceTimeBy(1000)
        runCurrent()
        return Harness(session, fw)
    }

    /** The six faces' offset and the high-g write that follows its save. */
    private fun saved(faces: Map<Face, AccelCalStatus>): Pair<List<Float>, HighGWrite?> {
        val lowG = lowG(faces)
        return lowG.wireOffset to AccelSixFace.highGAfterSave(AccelSixFace.solveHighG(faces, lowG))
    }

    @Test
    fun aSaveSendsTheOffsetThenTheHighG_andReadsBothBack() = runTest {
        val board = FakeFirmware.AccelCalBoard()
        val h = connected(board)
        val (o, hw) = saved(board(userTilts))
        val save = assertIs<HighGWrite.Save>(hw)
        val before = h.fw.commandFrames.size
        var atOffset: List<Int>? = null
        val w = h.session.writeAccelCal(isClear = false, sent = o, highG = save) { atOffset = h.sentSince(before) }
        // the offset's report is in before 80 goes out
        assertEquals(listOf(BleCommandId.ACC_CAL_APPLY, BleCommandId.ACC_CAL_READ), atOffset)
        assertEquals(
            listOf(BleCommandId.ACC_CAL_APPLY, BleCommandId.ACC_CAL_READ, BleCommandId.HG_CAL_APPLY, BleCommandId.ACC_CAL_READ),
            h.sentSince(before),
        )
        val sent80 = h.fw.commandFrames.drop(before)[2]
        assertEquals(1 + HighGCalApply.PAYLOAD_LENGTH, sent80.size)
        assertContentEquals(Commands.highGCalApply(HighGCalApply.OP_STORE, save.matrix, save.bias), sent80)
        assertTrue(w.offset.confirmed)
        assertEquals(AccelCalReadout.Line("Saved and read back", AccelCalReadout.Tone.OK), AccelCalReadout.write(w.offset))
        val hg = assertNotNull(w.highG)
        assertTrue(hg.confirmed)
        assertEquals("High-g alignment saved and read back", AccelCalReadout.hgWrite(hg).text)
        assertEquals(save.matrix, board.hgMatrix)
        assertEquals(save.bias, board.hgBias)
        assertEquals(true, h.session.accelCalStatus.value?.highG?.stored)
    }

    @Test
    fun anOlderFlightComputerSavesTheOffset_andNoHighGGoesOut() = runTest {
        // its frames carry no high-g: the fit is unavailable and only 76 goes out
        val h = connected(FakeFirmware.AccelCalBoard(knowsHighG = false))
        val (o, hw) = saved(board(userTilts, withHighG = false))
        assertNull(hw)
        val before = h.fw.commandFrames.size
        val w = h.session.writeAccelCal(isClear = false, sent = o, highG = hw)
        assertEquals(listOf(BleCommandId.ACC_CAL_APPLY, BleCommandId.ACC_CAL_READ), h.sentSince(before))
        assertTrue(w.offset.confirmed)
        assertNull(w.highG)
    }

    @Test
    fun aNewFlightComputerBehindAnOldOutComputer_saysWhyNotTheHighG() = runTest {
        // the faces did carry the high-g, but 80 gets no answer and the
        // read-back is the old out computer's 40 bytes
        val h = connected(FakeFirmware.AccelCalBoard(knowsHighG = false))
        val (o, hw) = saved(board(userTilts))
        val before = h.fw.commandFrames.size
        val run = async { h.session.writeAccelCal(isClear = false, sent = o, highG = hw) }
        runCurrent()
        assertEquals(
            listOf(BleCommandId.ACC_CAL_APPLY, BleCommandId.ACC_CAL_READ, BleCommandId.HG_CAL_APPLY),
            h.sentSince(before),
        )
        advanceTimeBy(AccelSixFace.APPLY_TIMEOUT_MS)
        runCurrent()
        val w = run.await()
        assertEquals(
            listOf(BleCommandId.ACC_CAL_APPLY, BleCommandId.ACC_CAL_READ, BleCommandId.HG_CAL_APPLY, BleCommandId.ACC_CAL_READ),
            h.sentSince(before),
        )
        assertTrue(w.offset.confirmed)
        val hg = assertNotNull(w.highG)
        assertNotNull(hg.readBack)
        assertFalse(hg.confirmed)
        assertEquals(
            AccelCalReadout.Line(
                "The answer has no high-g in it: update the flight computer and the out computer",
                AccelCalReadout.Tone.WARN,
            ),
            AccelCalReadout.hgWrite(hg),
        )
    }

    @Test
    fun aHighGThatFailsItsChecksIsNotSent_butTheOffsetIs() = runTest {
        val h = connected(FakeFirmware.AccelCalBoard())
        val faces = board(emptyMap(), quad = listOf(0.0, 0.0, 6 * mg / (g * g)))
        val fit = assertIs<HighGResult.Solved>(AccelSixFace.solveHighG(faces, lowG(faces))).solution
        assertFalse(fit.saveAllowed)
        val (o, hw) = saved(faces)
        assertNull(hw)
        val before = h.fw.commandFrames.size
        val w = h.session.writeAccelCal(isClear = false, sent = o, highG = hw)
        assertEquals(listOf(BleCommandId.ACC_CAL_APPLY, BleCommandId.ACC_CAL_READ), h.sentSince(before))
        assertTrue(w.offset.confirmed)
        assertNull(w.highG)
    }

    @Test
    fun aClearTakesTheHighGWithIt_whenOneIsStored() = runTest {
        val board = FakeFirmware.AccelCalBoard(
            offset = listOf(0.0412f, -0.0273f, 0.0156f),
            hgMatrix = listOf(1.01f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 1f),
            hgBias = listOf(0.98f, -1.47f, 0.62f),
            hgBiasSource = HighGBiasSource.SIX_FACES.raw,
        )
        val h = connected(board)
        val status = assertNotNull(h.session.readAccelCal())
        val hw = assertNotNull(AccelSixFace.highGAfterClear(status))
        val before = h.fw.commandFrames.size
        val w = h.session.writeAccelCal(isClear = true, sent = listOf(0f, 0f, 0f), highG = hw)
        assertEquals(
            listOf(BleCommandId.ACC_CAL_APPLY, BleCommandId.ACC_CAL_READ, BleCommandId.HG_CAL_APPLY, BleCommandId.ACC_CAL_READ),
            h.sentSince(before),
        )
        assertContentEquals(Commands.highGCalApply(HighGCalApply.OP_CLEAR), h.fw.commandFrames.drop(before)[2])
        assertTrue(w.offset.confirmed)
        val hg = assertNotNull(w.highG)
        assertTrue(hg.confirmed)
        assertEquals("Cleared: no high-g alignment stored", AccelCalReadout.hgWrite(hg).text)
        assertNull(board.hgMatrix)
        assertEquals(listOf(0.98f, -1.47f, 0.62f), board.hgBias, "the flight computer keeps the bias")

        // without one stored, 76 alone, as before
        val after = assertNotNull(h.session.readAccelCal())
        assertNull(AccelSixFace.highGAfterClear(after))
        val again = h.fw.commandFrames.size
        val w2 = h.session.writeAccelCal(isClear = true, sent = listOf(0f, 0f, 0f), highG = null)
        assertEquals(listOf(BleCommandId.ACC_CAL_APPLY, BleCommandId.ACC_CAL_READ), h.sentSince(again))
        assertTrue(w2.offset.confirmed)
        assertNull(w2.highG)
    }

    @Test
    fun noAnswerToTheHighG_thenReadItBack() = runTest {
        val board = FakeFirmware.AccelCalBoard()
        val h = connected(board)
        val (o, hw) = saved(board(userTilts))
        val save = assertIs<HighGWrite.Save>(hw)
        // The link drops every answer once the offset is in: the high-g
        // arrives and is stored, but neither its answer nor its read-back
        // comes back.
        val run = async {
            h.session.writeAccelCal(isClear = false, sent = o, highG = save) {
                h.fw.accelCal = null
                board.hgMatrix = save.matrix
                board.hgBias = save.bias
                board.hgBiasSource = HighGBiasSource.SIX_FACES.raw
            }
        }
        runCurrent()
        advanceTimeBy(AccelSixFace.APPLY_TIMEOUT_MS)
        runCurrent()
        assertFalse(run.isCompleted, "the read-back waits its own 6 s")
        advanceTimeBy(AccelSixFace.READ_TIMEOUT_MS)
        runCurrent()
        val w = run.await()
        assertTrue(w.offset.confirmed)
        val hg = assertNotNull(w.highG)
        assertNull(hg.readBack)
        assertEquals(
            "No answer about the high-g from the flight computer — read it back to check",
            AccelCalReadout.hgWrite(hg).text,
        )
        // The screen's Read It Back: one 77, its answer in place of the missing one.
        h.fw.accelCal = board
        val again = hg.copy(readBack = h.session.readAccelCal())
        assertTrue(again.confirmed)
        assertEquals("High-g alignment saved and read back", AccelCalReadout.hgWrite(again).text)
    }

    // ── The rocket profile ───────────────────────────────────────────────

    @Test
    fun aConfirmedSaveGivesThisBoardsProfileTheNewBias() {
        val cal = SensorCalData(
            gyroX = 3, gyroY = -2, gyroZ = 1, hgX = -0.65f, hgY = -0.57f, hgZ = 0.91f,
            calibratedOnUnitID = "AA:BB", calibratedAtMs = 0,
        )
        val updated = assertNotNull(
            ActiveRocketSyncer.sensorCalAfterHighGSave(cal, "AA:BB", listOf(-0.6512f, 1.48f, -0.3127f)),
        )
        assertEquals(-0.6512f, updated.hgX)
        assertEquals(1.48f, updated.hgY)
        assertEquals(-0.3127f, updated.hgZ)
        assertEquals(3, updated.gyroX, "the gyro's offsets stay the pad calibration's")
        assertEquals(cal.calibratedAtMs, updated.calibratedAtMs)
        assertEquals("AA:BB", updated.calibratedOnUnitID)
        // another board's cal is never pushed, and no cal stays none
        assertNull(ActiveRocketSyncer.sensorCalAfterHighGSave(cal, "CC:DD", listOf(0f, 0f, 0f)))
        assertNull(ActiveRocketSyncer.sensorCalAfterHighGSave(null, "AA:BB", listOf(0f, 0f, 0f)))
        // nor before the board's id is in
        assertNull(ActiveRocketSyncer.sensorCalAfterHighGSave(cal.copy(calibratedOnUnitID = ""), "", listOf(0f, 0f, 0f)))
    }
}
