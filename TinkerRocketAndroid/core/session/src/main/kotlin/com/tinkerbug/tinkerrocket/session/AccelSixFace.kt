package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.AccelCalStatus
import com.tinkerbug.tinkerrocket.protocol.AccelFaceState
import kotlin.math.abs
import kotlin.math.acos
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
 * offset does not, so
 *
 *     offset_i = (reading with axis i up + reading with axis i down) / 2
 *
 * A face a few degrees off its axis reads g·cos(tilt) along it, so a pair
 * leaves g·(cos t_up − cos t_down)/2 in the offset: nothing when the two
 * tilts match, 0.3 mg for 4° against 3.5°.
 *
 * Board frame throughout: +X forward, +Y left, +Z up out of the component
 * side, whatever way the board sits in the rocket.  The offset is a property
 * of the board, stored on it; nothing here touches a profile.
 */
public object AccelSixFace {

    /** Standard gravity, m/s²: the solve's g (the script's, without --lat). */
    public const val G0: Double = 9.80665

    /** m/s²: the flight computer refuses a larger component (ACC_CAL_OFFSET_MAX_MPS2). */
    public const val OFFSET_MAX_MPS2: Double = 1.0

    /** A face whose axis is further than this off vertical is measured again. */
    public const val TILT_MAX_DEG: Double = 10.0

    /** mg: a face this far off one g after the correction earns a note, not a refusal. */
    public const val RESIDUAL_NOTE_MG: Double = 5.0

    /** The same limit in m/s², computed as the script computes it. */
    private const val RESIDUAL_NOTE_MPS2: Double = 5e-3 * G0

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

    /** The solve, in m/s² and the board frame. */
    public data class Solution(
        /** offset_i = (m_up[i] + m_down[i]) / 2 — what is saved. */
        val offset: List<Double>,
        /** scale_i = (m_up[i] − m_down[i]) / 2g — for information only, never sent. */
        val scale: List<Double>,
        /** Each face's |(m − offset) / scale| − g, elementwise division. */
        val residual: Map<Face, Double>,
    ) {
        public val offsetMg: List<Double> get() = offset.map { mg(it) }

        /** Each scale as a % from 1. */
        public val scalePct: List<Double> get() = scale.map { (it - 1.0) * 100.0 }

        public val residualMg: Map<Face, Double> get() = residual.mapValues { mg(it.value) }

        /** The largest |residual|, mg. */
        public val worstResidualMg: Double get() = mg(residual.values.maxOf { abs(it) })

        /** The axes (0 = X) whose offset the flight computer would refuse. */
        public val axesPastLimit: List<Int> get() = (0..2).filter { abs(offset[it]) > OFFSET_MAX_MPS2 }

        /** Nothing past the limit.  Otherwise there is nothing to save: redo the faces. */
        public val saveAllowed: Boolean get() = axesPastLimit.isEmpty()

        /**
         * A face is more than [RESIDUAL_NOTE_MG] off one g after the
         * correction.  Worth a note — the offset is still good if the faces
         * were squarish — never a refusal.
         */
        public val residualNote: Boolean get() = residual.values.maxOf { abs(it) } > RESIDUAL_NOTE_MPS2

        /** The offset as command 76 carries it. */
        public val wireOffset: List<Float> get() = offset.map { it.toFloat() }
    }

    /** The script's solve over all six faces' mean readings, m/s². */
    public fun solve(faces: Map<Face, List<Double>>, g: Double = G0): Solution {
        val missing = Face.entries.filter { it !in faces }
        require(missing.isEmpty()) { "the solve needs all six faces; missing ${missing.map { it.label }}" }
        val off = DoubleArray(3)
        val scl = DoubleArray(3)
        for (ax in 0..2) {
            val p = faces.getValue(Face.of(ax, 1))[ax]
            val m = faces.getValue(Face.of(ax, -1))[ax]
            off[ax] = 0.5 * (p + m)
            scl[ax] = (p - m) / (2.0 * g)
        }
        val resid = LinkedHashMap<Face, Double>()
        for (face in Face.entries) {
            val mean = faces.getValue(face)
            var sum = 0.0
            for (i in 0..2) {
                val c = (mean[i] - off[i]) / scl[i]
                sum += c * c
            }
            resid[face] = sqrt(sum) - g
        }
        return Solution(off.toList(), scl.toList(), resid)
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
