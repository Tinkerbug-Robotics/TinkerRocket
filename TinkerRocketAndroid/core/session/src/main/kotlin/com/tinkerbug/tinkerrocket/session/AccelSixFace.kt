package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.AccelCalStatus
import com.tinkerbug.tinkerrocket.protocol.AccelFaceState
import com.tinkerbug.tinkerrocket.protocol.BoardOrientation
import com.tinkerbug.tinkerrocket.protocol.HighGBiasSource
import com.tinkerbug.tinkerrocket.protocol.HighGCalApply
import com.tinkerbug.tinkerrocket.protocol.IMUOrientationMode
import com.tinkerbug.tinkerrocket.protocol.TelemetryData
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.atan
import kotlin.math.sqrt

/**
 * The low-g accelerometer's six-face hand calibration (#1629): every rule and
 * every number the screen acts on, kept out of the composable so they can be
 * tested.  The reference is tools/accel_cal_six_face.py; this does what it
 * does.
 *
 * On a still pad the flight computer finds "level" from gravity, and an
 * accelerometer offset there looks exactly like a tilt.  Turn each board axis
 * straight up and then straight down: gravity flips sign on that axis and the
 * offset does not.  The per-axis average of the two,
 *
 *     offset_i = (reading with axis i up + reading with axis i down) / 2
 *
 * cancels a face's tilt only when both faces of the axis lean alike: it
 * leaves g·(cos t_up − cos t_down)/2 behind, which reached ~6 mg on the
 * user's first hand-held run (faces 3.7-9.1° off square).  So the solve fits
 * all six faces together to |reading| = g — a sphere fit with a per-axis
 * scale, [solve] — which no tilt biases; the average only seeds it.
 *
 * Board frame throughout: +X forward, +Y left, +Z up out of the component
 * side, whatever way the board sits in the rocket.  The offset is a property
 * of the board, stored on it; nothing here touches a profile.
 *
 * The same windows average the high-g accelerometer (a flight and out
 * computer that send it).  Its correction is fitted to the low-g, the
 * accurate channel on the bench: the matrix M (scale and cross-axis) and the
 * offset b with
 *
 *     l = M · h − b   on every face (least squares)
 *
 * l the raw low-g, h the raw high-g, so the high-g reads as the calibrated
 * low-g and takes over at the low-g's rail with no step (RocketComputerTypes.h,
 * HG_CAL_*).  The high-g's own sphere fit is shown beside it as a check; when
 * the two disagree beyond their noise, the alignment to the low-g is what is
 * saved ([solveHighG]).
 */
public object AccelSixFace {

    /** Standard gravity, m/s²: the solve's g (the script's, without --lat). */
    public const val G0: Double = 9.80665

    /** m/s²: the flight computer refuses a larger component (ACC_CAL_OFFSET_MAX_MPS2). */
    public const val OFFSET_MAX_MPS2: Double = 1.0

    /** A face whose axis is further than this off vertical is measured again. */
    public const val TILT_MAX_DEG: Double = 10.0

    /** The fitted scales Save accepts: outside this the faces did not make sense. */
    public const val SCALE_MIN: Double = 0.97
    public const val SCALE_MAX: Double = 1.03

    /** The fit's Gauss-Newton steps: at most this many. */
    public const val FIT_MAX_ITERATIONS: Int = 20

    /** It stops when no unknown moves more than this in a step. */
    public const val FIT_STEP_TOL: Double = 1e-9

    /** m/s²: converged also needs every face this close to one g at the end. */
    public const val FIT_RESIDUAL_TOL_MPS2: Double = 1e-6

    /** m/s²: how close the read-back must give back each component that was sent. */
    public const val READBACK_TOL_MPS2: Double = 1e-5

    /** The flight computer's face window (ACC_FACE_WINDOW_US). */
    public const val FACE_WINDOW_MS: Long = 3_000

    /** How long a face measurement waits for its result, as the script does. */
    public const val FACE_TIMEOUT_MS: Long = 10_000

    /** How long a read (command 77) waits for its answer. */
    public const val READ_TIMEOUT_MS: Long = 6_000

    /** How long a save or clear (command 76) waits for its answer before the read-back. */
    public const val APPLY_TIMEOUT_MS: Long = 8_000

    private const val AXES = "XYZ"

    /** milli-g, the unit the screen shows offsets and residuals in. */
    public fun mg(mps2: Double): Double = mps2 / G0 * 1e3

    /** The six faces, in the checklist's order. */
    public enum class Face(public val axis: Int, public val sign: Int) {
        X_UP(0, 1), X_DOWN(0, -1),
        Y_UP(1, 1), Y_DOWN(1, -1),
        Z_UP(2, 1), Z_DOWN(2, -1);

        /** "+X UP": the board's +X axis pointing up — the script's face_name. */
        public val label: String get() = "+${AXES[axis]} ${if (sign > 0) "UP" else "DOWN"}"

        public companion object {
            public fun of(axis: Int, sign: Int): Face =
                entries.first { it.axis == axis && (it.sign > 0) == (sign > 0) }
        }
    }

    /** The face a still reading is, and how far its axis is off vertical. */
    public data class Reading(val face: Face, val tiltDeg: Double)

    /**
     * The script's which_face: the axis nearest vertical (the largest |m_i|;
     * the first of equal ones), pointing up when the reading along it is
     * positive — a still accelerometer reads +g along the axis that points up
     * — and tilt = acos(|m_axis| / |m|).
     */
    public fun identify(mean: List<Double>): Reading {
        require(mean.size == 3) { "a reading has three axes, got ${mean.size}" }
        var axis = 0
        for (i in 1..2) if (abs(mean[i]) > abs(mean[axis])) axis = i
        val sign = if (mean[axis] > 0) 1 else -1
        val norm = sqrt(mean.sumOf { it * it })
        val tilt = if (norm > 0) Math.toDegrees(acos(minOf(1.0, abs(mean[axis]) / norm))) else 90.0
        return Reading(Face.of(axis, sign), tilt)
    }

    /** m/s²: a live reading this far from one g (0.1 g) is a board being moved. */
    public const val LIVE_G_TOL_MPS2: Double = 0.98

    /**
     * Degrees: under an auto-exact mounting, a still reading further than this
     * from rocket +X is a board the flight computer has not re-read yet — it
     * re-reads after 2 s still (its ORIENT_ACCEPT_TOL_DEG), and until then the
     * code beside the reading still names the face the board was on before.
     */
    public const val EXACT_SETTLED_DEG: Double = 5.0

    /**
     * Which face the board is on now, from the newest telemetry frame.  Only a
     * hint, to preselect the checklist row and name the Measure button: a
     * measurement still tells its face from its own window.
     */
    public sealed interface LiveFace {
        /** The face it reads, square or askew; null while waiting or moving. */
        public val face: Face? get() = null

        /** No frame yet, a field missing from the newest one, or a mounting with no discrete code. */
        public data object Waiting : LiveFace

        /**
         * |reading| more than [LIVE_G_TOL_MPS2] from one g, or, under an
         * auto-exact mounting, more than [EXACT_SETTLED_DEG] from rocket +X:
         * a board the flight computer has not re-read since it was moved.
         */
        public data object Moving : LiveFace

        /** Still, but its axis more than [TILT_MAX_DEG] off vertical. */
        public data class Askew(override val face: Face, val tiltDeg: Double) : LiveFace

        /** On [face], square enough to measure. */
        public data class On(override val face: Face, val tiltDeg: Double) : LiveFace

        /**
         * On [face], under an auto-exact mounting, where the tilt cannot be
         * read back: the flight computer applies the exact rotation, and the
         * code beside it only names the board axis nearest "up", so the
         * reading turned back through that code always looks square.  The
         * face is right; the measurement judges the tilt.
         */
        public data class OnNoTilt(override val face: Face) : LiveFace
    }

    /**
     * The mounting a reading is in: a discrete [code], and whether the flight
     * computer actually applies an [exact] rotation beside it (auto-exact,
     * where the code is only the board axis nearest "up").
     */
    public data class LiveMounting(val code: Int, val exact: Boolean)

    /**
     * The live face from a rocket-frame low-g reading and the mounting it is
     * in ([BoardOrientation]): turned back into the board frame (Rᵀ), then
     * told as a measurement's face is ([identify]).  Waiting without a
     * reading or without a code that names a rotation; under an exact
     * mounting, the face without its tilt ([LiveFace.OnNoTilt]), never askew,
     * and only once the reading has settled onto rocket +X.
     */
    public fun liveFace(rocket: List<Double>?, mounting: LiveMounting?): LiveFace {
        if (rocket == null || mounting == null || !BoardOrientation.isValid(mounting.code)) {
            return LiveFace.Waiting
        }
        val board = BoardOrientation.toBoard(mounting.code, rocket)
        val norm = sqrt(board.sumOf { it * it })
        if (abs(norm - G0) > LIVE_G_TOL_MPS2) return LiveFace.Moving
        if (mounting.exact && Math.toDegrees(acos((rocket[0] / norm).coerceIn(-1.0, 1.0))) > EXACT_SETTLED_DEG) {
            return LiveFace.Moving
        }
        val r = identify(board)
        return when {
            mounting.exact -> LiveFace.OnNoTilt(r.face)
            r.tiltDeg > TILT_MAX_DEG -> LiveFace.Askew(r.face, r.tiltDeg)
            else -> LiveFace.On(r.face, r.tiltDeg)
        }
    }

    /**
     * The mounting a telemetry frame's lx/ly/lz are in.  The frame's own
     * "imo" (code = imo & 0x1F) when it carries one, which the base station's
     * relay does; the reading and its code then arrive together and always
     * agree, and an exact auto rotation there has no code at all (31).  The
     * rocket's own link leaves "imo" out and reports the mounting in its
     * "imu_orient" message instead, re-sent as the pad auto-detect re-racks:
     * default, manual and snapped modes as they are, and auto-exact (mode 3)
     * with its code marked [LiveMounting.exact].  Null: no hint.
     */
    public fun liveMounting(imo: Int?, reportedCode: Int?, reportedMode: IMUOrientationMode): LiveMounting? {
        if (imo != null) return LiveMounting(imo and 0x1F, exact = false)
        val code = reportedCode ?: return null
        return when (reportedMode) {
            IMUOrientationMode.DEFAULT_MOUNTING, IMUOrientationMode.MANUAL, IMUOrientationMode.AUTO_SNAP ->
                LiveMounting(code, exact = false)
            IMUOrientationMode.AUTO_EXACT -> LiveMounting(code, exact = true)
            IMUOrientationMode.UNKNOWN -> null
        }
    }

    /**
     * The live face from one telemetry frame (lx/ly/lz; a field missing from
     * it is no hint) and, for a link whose frames carry no "imo", the mounting
     * the "imu_orient" message last reported.
     */
    public fun liveFace(
        t: TelemetryData,
        reportedCode: Int? = null,
        reportedMode: IMUOrientationMode = IMUOrientationMode.UNKNOWN,
    ): LiveFace {
        val x = t.lowGX ?: return LiveFace.Waiting
        val y = t.lowGY ?: return LiveFace.Waiting
        val z = t.lowGZ ?: return LiveFace.Waiting
        return liveFace(
            listOf(x.toDouble(), y.toDouble(), z.toDouble()),
            liveMounting(t.imuOrientPacked, reportedCode, reportedMode),
        )
    }

    /** What a finished face window comes to. */
    public sealed interface Verdict {
        /** Kept.  [replaced]: this face was captured before, and this one replaces it. */
        public data class Accepted(val face: Face, val tiltDeg: Double, val replaced: Boolean) : Verdict

        /** The window FAILED: the board moved, or the reading is not one g. */
        public data object Moved : Verdict

        /** Its axis is more than [TILT_MAX_DEG] off vertical. */
        public data class Tilted(val face: Face, val tiltDeg: Double) : Verdict
    }

    /**
     * A finished window ([Measurement.Finished]: DONE or FAILED), judged in
     * the script's order: FAILED is never used, whatever it reads; otherwise
     * the face it reads is kept if its axis is within [TILT_MAX_DEG] of
     * vertical.  [captured] is what the checklist holds already.
     */
    public fun judge(window: AccelCalStatus, captured: Set<Face>): Verdict {
        if (window.faceState == AccelFaceState.FAILED) return Verdict.Moved
        val r = identify(window.faceMean.map { it.toDouble() })
        if (r.tiltDeg > TILT_MAX_DEG) return Verdict.Tilted(r.face, r.tiltDeg)
        return Verdict.Accepted(r.face, r.tiltDeg, replaced = r.face in captured)
    }

    /** The fit, in m/s² and the board frame. */
    public data class Solution(
        /** The offset b — what is saved. */
        val offset: List<Double>,
        /** Per axis 1 / k: shown for information, never sent. */
        val scale: List<Double>,
        /**
         * The fit stopped on its step size, not its iteration cap, with every
         * face within [FIT_RESIDUAL_TOL_MPS2] of one g.
         */
        val converged: Boolean,
        /** Gauss-Newton steps taken. */
        val iterations: Int,
        /** The largest | |k ⊙ (m − b)| − g | over the six faces at the end, m/s². */
        val maxResidualMps2: Double,
    ) {
        public val offsetMg: List<Double> get() = offset.map { mg(it) }

        /** Each scale as a % from 1. */
        public val scalePct: List<Double> get() = scale.map { (it - 1.0) * 100.0 }

        /** The axes (0 = X) whose offset the flight computer would refuse. */
        public val axesPastLimit: List<Int> get() = (0..2).filter { !(abs(offset[it]) <= OFFSET_MAX_MPS2) }

        /** The axes whose scale is outside [SCALE_MIN]..[SCALE_MAX]. */
        public val axesScaleOutOfRange: List<Int>
            get() = (0..2).filter { !(scale[it] in SCALE_MIN..SCALE_MAX) }

        /** Converged, every offset within the limit and every scale in range; otherwise redo the faces. */
        public val saveAllowed: Boolean
            get() = converged && axesPastLimit.isEmpty() && axesScaleOutOfRange.isEmpty()

        /** The offset as command 76 carries it. */
        public val wireOffset: List<Float> get() = offset.map { it.toFloat() }
    }

    /**
     * The sphere fit over all six faces' mean readings, m/s²: the offset b
     * and per-axis k = 1 / scale that put every face at |k ⊙ (m − b)| = g.
     * Six faces, six unknowns, so it fits exactly, and a face that is a few
     * degrees off square is still a point on that sphere: nothing to bias.
     * Gauss-Newton, seeded with b = the per-axis average and k = 1; each step
     * solves the normal equations (JᵀJ) dp = −Jᵀr with partial pivoting.
     * Identical in the iOS app and tools/accel_cal_six_face.py.
     */
    public fun solve(faces: Map<Face, List<Double>>, g: Double = G0): Solution {
        val missing = Face.entries.filter { it !in faces }
        require(missing.isEmpty()) { "the solve needs all six faces; missing ${missing.map { it.label }}" }
        val means = Face.entries.map { faces.getValue(it) }
        // p = (b_x, b_y, b_z, k_x, k_y, k_z).
        val p = DoubleArray(6)
        for (ax in 0..2) {
            p[ax] = 0.5 * (faces.getValue(Face.of(ax, 1))[ax] + faces.getValue(Face.of(ax, -1))[ax])
            p[3 + ax] = 1.0
        }
        var stoppedOnStep = false
        var iterations = 0
        while (iterations < FIT_MAX_ITERATIONS) {
            iterations++
            val (r, jac) = residualsAndJacobian(means, p, g) ?: break
            val jtj = Array(6) { i -> DoubleArray(6) { j -> (0..5).sumOf { f -> jac[f][i] * jac[f][j] } } }
            val rhs = DoubleArray(6) { i -> -(0..5).sumOf { f -> jac[f][i] * r[f] } }
            val dp = solveLinear(jtj, rhs) ?: break
            for (i in 0..5) p[i] += dp[i]
            if (dp.maxOf { abs(it) } < FIT_STEP_TOL) {
                stoppedOnStep = true
                break
            }
        }
        val maxResidual = residualsAndJacobian(means, p, g)?.first?.maxOf { abs(it) } ?: Double.POSITIVE_INFINITY
        return Solution(
            offset = listOf(p[0], p[1], p[2]),
            scale = listOf(1.0 / p[3], 1.0 / p[4], 1.0 / p[5]),
            converged = stoppedOnStep && maxResidual < FIT_RESIDUAL_TOL_MPS2,
            iterations = iterations,
            maxResidualMps2 = maxResidual,
        )
    }

    /**
     * Each face's r = |v| − g with v = k ⊙ (m − b), and its Jacobian row
     * dr/db_i = −k_i·v_i / |v|, dr/dk_i = (m_i − b_i)·v_i / |v|; null when a
     * face sits exactly on the offset (|v| = 0).
     */
    private fun residualsAndJacobian(
        means: List<List<Double>>,
        p: DoubleArray,
        g: Double,
    ): Pair<DoubleArray, Array<DoubleArray>>? {
        val r = DoubleArray(means.size)
        val jac = Array(means.size) { DoubleArray(6) }
        for ((f, m) in means.withIndex()) {
            val v = DoubleArray(3) { p[3 + it] * (m[it] - p[it]) }
            val n = sqrt(v.sumOf { it * it })
            if (!(n > 0.0)) return null
            r[f] = n - g
            for (i in 0..2) {
                jac[f][i] = -p[3 + i] * v[i] / n
                jac[f][3 + i] = (m[i] - p[i]) * v[i] / n
            }
        }
        return r to jac
    }

    /** A·x = b by Gaussian elimination with partial pivoting; null when A is singular. */
    private fun solveLinear(a: Array<DoubleArray>, b: DoubleArray): DoubleArray? {
        val n = b.size
        val m = Array(n) { a[it].copyOf() }
        val x = b.copyOf()
        val scale = m.maxOf { row -> row.maxOf { abs(it) } }
        for (col in 0 until n) {
            var piv = col
            for (row in col + 1 until n) if (abs(m[row][col]) > abs(m[piv][col])) piv = row
            if (!(abs(m[piv][col]) > 1e-12 * scale)) return null
            if (piv != col) {
                val t = m[piv]; m[piv] = m[col]; m[col] = t
                val tx = x[piv]; x[piv] = x[col]; x[col] = tx
            }
            for (row in col + 1 until n) {
                val factor = m[row][col] / m[col][col]
                for (k in col until n) m[row][k] -= factor * m[col][k]
                x[row] -= factor * x[col]
            }
        }
        val out = DoubleArray(n)
        for (row in n - 1 downTo 0) {
            var s = x[row]
            for (k in row + 1 until n) s -= m[row][k] * out[k]
            out[row] = s / m[row][row]
        }
        return out
    }

    // ── The high-g, aligned to the low-g ─────────────────────────────────

    /**
     * What the flight computer refuses (HG_CAL_SCALE_MIN..MAX,
     * HG_CAL_CROSS_MAX, HG_CAL_BIAS_MAX_MPS2) and what the screen refuses on
     * top: a face fitted worse than [HG_RESIDUAL_MAX_MPS2], ~7x the residual
     * of a still 3 s face.
     */
    public const val HG_SCALE_MIN: Double = 0.97
    public const val HG_SCALE_MAX: Double = 1.03
    public const val HG_CROSS_MAX: Double = 0.03
    public const val HG_BIAS_MAX_MPS2: Double = 11.0
    public const val HG_RESIDUAL_MAX_MPS2: Double = 0.03

    /**
     * m/s²: each face mean's error beyond its samples' SD / √n (the offset's
     * drift while the board is handled, ~2 mg/°C, and slow noise).
     */
    public const val HG_NOISE_FLOOR_MPS2: Double = 0.002

    /** The floors under the two fits' agreement test, and its width. */
    public const val HG_AGREE_FLOOR_OFFSET_MPS2: Double = 0.005
    public const val HG_AGREE_FLOOR_SCALE: Double = 2e-4
    public const val HG_AGREE_SIGMAS: Double = 3.0

    /** How close a read-back's matrix must be to what was sent. */
    public const val MATRIX_READBACK_TOL: Double = 1e-6

    /** The off-diagonal entries of a row-major 3x3. */
    public val CROSS_INDICES: List<Int> = listOf(1, 2, 3, 5, 6, 7)

    /** What the high-g's fit came to. */
    public sealed interface HighGResult {
        /** The faces came without the high-g reading: an older flight or out computer. */
        public data object NoReading : HighGResult

        /** The fit could not be solved (the faces do not span three axes). */
        public data object Singular : HighGResult

        public data class Solved(val solution: HighGSolution) : HighGResult
    }

    /** Why a high-g alignment may not be saved. */
    public enum class HighGProblem { RESIDUAL, SCALE, CROSS, BIAS }

    /** The high-g's alignment to the low-g, in m/s² and the board frame. */
    public data class HighGSolution(
        /** M, row-major, and b: l = M h − b. */
        val matrix: List<Double>,
        val bias: List<Double>,
        /** m/s², over the six faces' 18 equations. */
        val residualRms: Double,
        val residualMax: Double,
        /** 1-sigma of each M entry and of b, from the faces' noise. */
        val sigmaMatrix: List<Double>,
        val sigmaBias: List<Double>,
        /**
         * The high-g's own sphere fit, and the zero-g the alignment implies
         * for it, M⁻¹ (b + the low-g's offset).
         */
        val ownOffset: List<Double>,
        val ownScale: List<Double>,
        val impliedZero: List<Double>,
        /**
         * Per axis: the alignment less the own fit (offset, m/s²; scale), the
         * difference's 1-sigma, and the axes where it is past 3 sigma.
         */
        val diffOffset: List<Double>,
        val diffScale: List<Double>,
        val sigmaDiffOffset: List<Double>,
        val sigmaDiffScale: List<Double>,
        val disagreeAxes: List<Int>,
        val problems: List<HighGProblem>,
    ) {
        /**
         * Within the limits: saved with the offset.  A disagreement is not a
         * problem: the alignment to the low-g is saved either way.
         */
        public val saveAllowed: Boolean get() = problems.isEmpty()

        /** M and b as command 80 carries them. */
        public val wireMatrix: List<Float> get() = matrix.map { it.toFloat() }
        public val wireBias: List<Float> get() = bias.map { it.toFloat() }

        /** The largest cross-axis term, as an angle, degrees. */
        public val crossDeg: Double
            get() = Math.toDegrees(atan(CROSS_INDICES.maxOf { abs(matrix[it]) }))

        public val sigmaCrossDeg: Double get() = Math.toDegrees(CROSS_INDICES.maxOf { sigmaMatrix[it] })
    }

    /**
     * The high-g's alignment to the low-g from the six face windows, with the
     * low-g's own fit ([lowG], for the agreement check): one row of M (and b)
     * at a time, [h_x h_y h_z −1] (M_i, b_i) = l_i over the six faces — four
     * unknowns, six equations.  Its 1-sigma comes from the faces' noise (each
     * mean's SD / √n, the high-g's through M, and the noise floor), not the
     * residuals, two degrees of freedom a row being too few to judge by.
     * tools/accel_cal_six_face.py's solve_high_g, number for number, as iOS
     * AccelSixFace.solveHighG is.
     */
    public fun solveHighG(faces: Map<Face, AccelCalStatus>, lowG: Solution, g: Double = G0): HighGResult {
        val windows = Face.entries.map { faces[it] ?: return HighGResult.NoReading }
        val high = windows.map { it.highG ?: return HighGResult.NoReading }
        val l = windows.map { w -> w.faceMean.map { it.toDouble() } }
        val sl = windows.map { w -> w.faceSd.map { it.toDouble() } }
        val n = windows.map { maxOf(it.faceN, 1).toDouble() }
        val h = high.map { hg -> hg.faceMean.map { it.toDouble() } }
        val sh = high.map { hg -> hg.faceSd.map { it.toDouble() } }

        val x = h.map { it + -1.0 }
        val xtx = Array(4) { i -> DoubleArray(4) { j -> (0..5).sumOf { f -> x[f][i] * x[f][j] } } }
        val xtxInv = invert(xtx) ?: return HighGResult.Singular
        val m = DoubleArray(9)
        val b = DoubleArray(3)
        for (i in 0..2) {
            val rhs = DoubleArray(4) { j -> (0..5).sumOf { f -> x[f][j] * l[f][i] } }
            for (j in 0..3) {
                val t = (0..3).sumOf { k -> xtxInv[j][k] * rhs[k] }
                if (j < 3) m[3 * i + j] = t else b[i] = t
            }
        }
        if (!(m + b).all { it.isFinite() }) return HighGResult.Singular

        var sumSq = 0.0
        var worst = 0.0
        for (f in 0..5) {
            val mh = mul(m, h[f])
            for (i in 0..2) {
                val r = mh[i] - b[i] - l[f][i]
                sumSq += r * r
                worst = maxOf(worst, abs(r))
            }
        }
        val rms = sqrt(sumSq / 18)
        // each row's noise: the mean over the faces of every reading's variance in that row's equation
        val sigRow = (0..2).map { i ->
            var v = 0.0
            for (f in 0..5) {
                val hv = (0..2).sumOf { j -> (m[3 * i + j] * sh[f][j]).let { it * it } }
                v += (sl[f][i] * sl[f][i] + hv) / n[f] + HG_NOISE_FLOOR_MPS2 * HG_NOISE_FLOOR_MPS2
            }
            sqrt(v / 6)
        }
        val sigM = (0..8).map { k -> sigRow[k / 3] * sqrt(xtxInv[k % 3][k % 3]) }
        val sigB = (0..2).map { sigRow[it] * sqrt(xtxInv[3][3]) }

        // the high-g's own gravity fit, and what the alignment says it should have found
        val own = solve(Face.entries.withIndex().associate { (f, face) -> face to h[f] }, g)
        val mInv = invert(Array(3) { r -> DoubleArray(3) { c -> m[3 * r + c] } }) ?: return HighGResult.Singular
        val zero = mul(DoubleArray(9) { mInv[it / 3][it % 3] }, (0..2).map { b[it] + lowG.offset[it] })
        val dOff = (0..2).map { zero[it] - own.offset[it] }
        val dScl = (0..2).map { m[4 * it] - lowG.scale[it] / own.scale[it] }
        val sdOff = ArrayList<Double>(3)
        val sdScl = ArrayList<Double>(3)
        val disagree = ArrayList<Int>()
        for (i in 0..2) {
            val up = Face.entries.indexOf(Face.of(i, 1))
            val down = Face.entries.indexOf(Face.of(i, -1))
            val ownVar = sh[up][i] * sh[up][i] / n[up] + sh[down][i] * sh[down][i] / n[down] +
                2 * HG_NOISE_FLOOR_MPS2 * HG_NOISE_FLOOR_MPS2
            sdOff += sqrt(ownVar / 4 + sigB[i] * sigB[i] + HG_AGREE_FLOOR_OFFSET_MPS2 * HG_AGREE_FLOOR_OFFSET_MPS2)
            sdScl += sqrt(ownVar / (4 * g * g) + sigM[4 * i] * sigM[4 * i] + HG_AGREE_FLOOR_SCALE * HG_AGREE_FLOOR_SCALE)
            if (!(abs(dOff[i]) <= HG_AGREE_SIGMAS * sdOff[i] && abs(dScl[i]) <= HG_AGREE_SIGMAS * sdScl[i])) {
                disagree += i
            }
        }

        val problems = buildList {
            if (worst > HG_RESIDUAL_MAX_MPS2) add(HighGProblem.RESIDUAL)
            if ((0..2).any { !(m[4 * it] >= HG_SCALE_MIN && m[4 * it] <= HG_SCALE_MAX) }) add(HighGProblem.SCALE)
            if (CROSS_INDICES.any { abs(m[it]) > HG_CROSS_MAX }) add(HighGProblem.CROSS)
            if (b.any { abs(it) > HG_BIAS_MAX_MPS2 }) add(HighGProblem.BIAS)
        }
        return HighGResult.Solved(
            HighGSolution(
                matrix = m.toList(), bias = b.toList(), residualRms = rms, residualMax = worst,
                sigmaMatrix = sigM, sigmaBias = sigB,
                ownOffset = own.offset, ownScale = own.scale, impliedZero = zero,
                diffOffset = dOff, diffScale = dScl, sigmaDiffOffset = sdOff, sigmaDiffScale = sdScl,
                disagreeAxes = disagree, problems = problems,
            ),
        )
    }

    /** [m] · [v], m row-major 3x3. */
    private fun mul(m: DoubleArray, v: List<Double>): List<Double> =
        (0..2).map { i -> m[3 * i] * v[0] + m[3 * i + 1] * v[1] + m[3 * i + 2] * v[2] }

    /** The inverse of a small square matrix, column by column; null when singular. */
    private fun invert(a: Array<DoubleArray>): Array<DoubleArray>? {
        val n = a.size
        val cols = (0 until n).map { c -> solveLinear(a, DoubleArray(n) { if (it == c) 1.0 else 0.0 }) ?: return null }
        return Array(n) { r -> DoubleArray(n) { c -> cols[c][r] } }
    }

    /** What one face measurement (command 78) came to. */
    public sealed interface Measurement {
        /** The window closed, DONE or FAILED: [judge] says what to do with it. */
        public data class Finished(val window: AccelCalStatus) : Measurement

        /** The flight computer would not open a window: in flight, after landing, or in a sim. */
        public data class Refused(val status: AccelCalStatus) : Measurement

        /** Nothing settled it inside [FACE_TIMEOUT_MS]. */
        public data object NoAnswer : Measurement

        /** Not connected: nothing was sent. */
        public data object NotConnected : Measurement
    }

    /**
     * Which 0xD1 frame answers a face measurement: the rule that keeps an old
     * frame from passing for a new face.  Feed it every frame that arrives
     * after command 78 goes out.
     *
     *  - Until the window's own MEASURING arrives, frames are ignored: a late
     *    answer to an earlier 77 carries the previous face_state, DONE
     *    included, and must not be taken as this face's result.
     *  - A FAILED frame with no samples in place of that MEASURING is a
     *    refusal (the flight computer opens no window in flight, after
     *    landing, or in a sim).
     *  - After the MEASURING, the first DONE or FAILED is the result.
     */
    public class Sequencer {
        /** The window's MEASURING frame has arrived. */
        public var windowOpen: Boolean = false
            private set

        /** The result or the refusal, once settled. */
        public var outcome: Measurement? = null
            private set

        /** Take one frame; the outcome once settled (and kept from then on). */
        public fun onFrame(frame: AccelCalStatus): Measurement? {
            if (outcome != null) return outcome
            val state = frame.faceState
            when {
                windowOpen -> if (state == AccelFaceState.DONE || state == AccelFaceState.FAILED) {
                    outcome = Measurement.Finished(frame)
                }
                state == AccelFaceState.MEASURING -> windowOpen = true
                state == AccelFaceState.FAILED && frame.faceN == 0 -> outcome = Measurement.Refused(frame)
                else -> Unit   // stale: an earlier window's state
            }
            return outcome
        }
    }

    /** What a save or clear found: command 76's answer, then the read-back (77). */
    public data class Applied(
        /** The answer to 76 — none when the flight computer refused it in flight or after landing. */
        val answer: AccelCalStatus?,
        /** The answer to the 77 sent once 76 was settled. */
        val readBack: AccelCalStatus?,
    ) {
        /** What the board holds as far as the app can tell: the read-back, else the answer to 76. */
        public val final: AccelCalStatus? get() = readBack ?: answer
    }

    /** A save took: an offset is stored, and each component reads back within [READBACK_TOL_MPS2] of [sent]. */
    public fun savedAsSent(sent: List<Float>, back: AccelCalStatus?): Boolean =
        back != null && back.stored &&
            (0..2).all { abs(back.offset[it].toDouble() - sent[it].toDouble()) < READBACK_TOL_MPS2 }

    /** A clear took: no offset stored. */
    public fun cleared(back: AccelCalStatus?): Boolean = back != null && !back.stored

    /** A save or a clear, and what it read back — iOS's AccelCalFlow.WriteReport. */
    public data class WriteReport(
        /** True for a clear (76 with zeros), false for a save. */
        val isClear: Boolean,
        /** The offset sent: zeros for a clear. */
        val sent: List<Float>,
        /** The read-back, else the answer to 76; null when neither came. */
        val readBack: AccelCalStatus?,
    ) {
        public val confirmed: Boolean
            get() = if (isClear) cleared(readBack) else savedAsSent(sent, readBack)
    }

    /**
     * A high-g save took: an alignment stored from the six faces, M within
     * [MATRIX_READBACK_TOL] and b within [READBACK_TOL_MPS2] of what was sent.
     * A bias the pad calibration wrote since is not this save's.
     */
    public fun highGSavedAsSent(matrix: List<Float>, bias: List<Float>, back: AccelCalStatus?): Boolean {
        val h = back?.highG ?: return false
        return h.stored && h.source == HighGBiasSource.SIX_FACES &&
            (0..8).all { abs(h.matrix[it].toDouble() - matrix[it].toDouble()) < MATRIX_READBACK_TOL } &&
            (0..2).all { abs(h.bias[it].toDouble() - bias[it].toDouble()) < READBACK_TOL_MPS2 }
    }

    /** A high-g clear took: the answer carries the high-g, and no alignment is stored. */
    public fun highGCleared(back: AccelCalStatus?): Boolean = back?.highG?.let { !it.stored } ?: false

    /** A write of the high-g's alignment (command 80) — iOS's AccelCalFlow.HighGWrite. */
    public sealed interface HighGWrite {
        /** [HighGCalApply.OP_STORE] or [HighGCalApply.OP_CLEAR]. */
        public val op: Int
        public val matrix: List<Float>
        public val bias: List<Float>

        /** Store and apply M and b. */
        public data class Save(override val matrix: List<Float>, override val bias: List<Float>) : HighGWrite {
            override val op: Int get() = HighGCalApply.OP_STORE
        }

        /** M back to identity; the flight computer keeps b. */
        public data object Clear : HighGWrite {
            override val op: Int get() = HighGCalApply.OP_CLEAR
            override val matrix: List<Float> get() = HighGCalApply.IDENTITY
            override val bias: List<Float> get() = HighGCalApply.ZERO_BIAS
        }
    }

    /** The high-g's save or clear, and what its read-back said — iOS's HighGWriteReport. */
    public data class HighGWriteReport(
        val write: HighGWrite,
        /** The read-back, else the answer to 80; null when neither came. */
        val readBack: AccelCalStatus?,
    ) {
        public val confirmed: Boolean
            get() = when (write) {
                is HighGWrite.Save -> highGSavedAsSent(write.matrix, write.bias, readBack)
                HighGWrite.Clear -> highGCleared(readBack)
            }
    }

    /**
     * The high-g write that follows a save: the alignment, when the faces
     * carried the high-g and its fit is within its limits; null otherwise, and
     * the offset is saved alone.
     */
    public fun highGAfterSave(fit: HighGResult?): HighGWrite? =
        ((fit as? HighGResult.Solved)?.solution)?.takeIf { it.saveAllowed }
            ?.let { HighGWrite.Save(it.wireMatrix, it.wireBias) }

    /**
     * The high-g write that follows a clear: a clear only when the board says
     * an alignment is stored ([status], the last 0xD1 frame) — an older
     * flight computer would not answer 80.
     */
    public fun highGAfterClear(status: AccelCalStatus?): HighGWrite? =
        if (status?.highG?.stored == true) HighGWrite.Clear else null

    /** A save's or a clear's two reports: the offset's, and the high-g's when one followed. */
    public data class Written(val offset: WriteReport, val highG: HighGWriteReport?)

    /**
     * Why the screen may not measure, save or clear right now (and so does
     * not send its opening read either) — iOS AccelCalibrationView's
     * blockedReason, case for case and in its order.
     */
    public enum class Access {
        READY,

        /** A base-station link: it does not relay these commands. */
        BASE_STATION,

        /** Not connected, or the rocket is off, so nothing would answer. */
        OFF,

        /** INITIALIZATION, or a boot step still reported: the flight computer is starting. */
        STARTING,

        /** INFLIGHT: it refuses these commands. */
        FLYING,

        /** LANDED: the post-flight lockout holds until the flight computer restarts. */
        LANDED,

        /** MAG_CAL: it refuses these commands. */
        MAG_CAL,

        /** A simulation is running: the flight computer opens no face window. */
        SIMULATING,
    }

    /** The gate.  [simLaunched] is the session's launch latch, as iOS reads `simLaunched`. */
    public fun access(
        isConnected: Boolean,
        isBaseStation: Boolean,
        pwrPinOn: Boolean,
        state: String,
        bootStepReported: Boolean,
        simLaunched: Boolean,
    ): Access = when {
        isBaseStation -> Access.BASE_STATION
        !isConnected || !pwrPinOn -> Access.OFF
        state == "INITIALIZATION" || bootStepReported -> Access.STARTING
        state == "INFLIGHT" -> Access.FLYING
        state == "LANDED" -> Access.LANDED
        state == "MAG_CAL" -> Access.MAG_CAL
        simLaunched -> Access.SIMULATING
        else -> Access.READY
    }

    /**
     * Whether the screen's entries (Settings, the dashboard's Tools row) open
     * it: the rocket itself, connected and powered — iOS SettingsView's
     * canCalibrate, the magnetometer calibration's gate.
     */
    public fun canCalibrate(isConnected: Boolean, isBaseStation: Boolean, pwrPinOn: Boolean): Boolean =
        isConnected && !isBaseStation && pwrPinOn
}
