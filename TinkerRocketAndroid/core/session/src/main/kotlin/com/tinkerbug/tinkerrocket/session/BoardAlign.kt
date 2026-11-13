package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.SdrAlignCommand
import com.tinkerbug.tinkerrocket.protocol.SdrAlignHave
import com.tinkerbug.tinkerrocket.protocol.SdrAlignSaved
import com.tinkerbug.tinkerrocket.protocol.SdrAlignStatus
import com.tinkerbug.tinkerrocket.protocol.SdrCommandResult
import com.tinkerbug.tinkerrocket.protocol.SdrCommandState
import com.tinkerbug.tinkerrocket.protocol.SdrRestFate
import com.tinkerbug.tinkerrocket.protocol.formatFixed
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Face
import kotlin.math.abs

/**
 * The gnss-sdr-dev board's part of the six-face calibration — iOS
 * SdrAlignStatus.swift (BoardAlignFlow and the screen's words), which is the
 * reference: the same rules, sentences, capitals and punctuation.
 *
 * The six faces the flight computer measures are the board's poses too: it
 * pairs each rest the flight computer sends while the rocket is held still.
 * The first face measured starts the board afresh (restart: its saved
 * mounting stays until the new one is saved; the command also turns the
 * board's own auto-save off for that boot, so the app saves). Each time the
 * flight computer's offset is saved and read back, the board's mounting is
 * saved too, once it is whole with every axis calibrated and passes the
 * board's own auto-save quality ([SdrAlignStatus.isGoodToSave]; at once, or
 * when the board's last pair comes in after it) — unless the flight computer's offset
 * moved by more than [OFFSET_MOVE_MAX] since the board began, when the
 * board's calibration was made against the old one: then nothing is saved,
 * and the next face starts the board afresh for the faces once more.
 */
public object BoardAlign {

    /** m/s² on any axis (2 mg): a larger move of the flight computer's offset calls for the faces again. */
    public const val OFFSET_MOVE_MAX: Float = 0.02f

    /** The flight computer tries the board for 3 s, then says so. */
    public const val TIMEOUT_MS: Long = 6_000

    /** A pose further than this from a face's axis is no face's. */
    public const val FACE_TOLERANCE_DEG: Double = 20.0

    /** The face a pose was held on: its reading's axis nearest vertical, within [FACE_TOLERANCE_DEG]. */
    public fun faceOf(pose: SdrAlignStatus.Pose): Face? {
        val r = AccelSixFace.identify(pose.dir)
        return if (r.tiltDeg <= FACE_TOLERANCE_DEG) r.face else null
    }

    /** The rests the board paired on each face (faces with none left out). */
    public fun pairsByFace(s: SdrAlignStatus): Map<Face, Int> {
        val out = mutableMapOf<Face, Int>()
        for (p in s.poses) faceOf(p)?.let { out[it] = (out[it] ?: 0) + p.pairs }
        return out
    }

    /** What the screen sends: command 83. */
    public data class Command(val op: Int, val id: Int)

    /** How the last command ended. */
    public sealed interface Outcome {
        public val op: Int

        public data class Done(override val op: Int) : Outcome

        /** The board answered but did not do it. */
        public data class NotDone(override val op: Int, val result: SdrCommandResult?) : Outcome

        /** The flight computer would not send it: in flight, after a flight, during a magnetometer calibration, or no board. */
        public data class Refused(override val op: Int) : Outcome

        /** The flight computer sent it for 3 s and the board never echoed it. */
        public data class NoAnswer(override val op: Int) : Outcome

        /** Nothing came back from the flight computer at all. */
        public data class NoStatus(override val op: Int) : Outcome
    }

    /** Why the faces must be measured once more for the board: the flight computer's offset's move, m/s². */
    public data class Again(val move: List<Float>)

    /**
     * The flow, without the BLE and the timers: the screen sends the
     * [Command] each step returns, arms a timer for [TIMEOUT_MS] with the
     * [generation] it had, and hands back every 0xD2 status ([take]) and every
     * timer that fires ([timedOut]). One command at a time.
     */
    public class Flow {
        public var status: SdrAlignStatus? = null
            private set
        public var pending: Command? = null
            private set
        public var generation: Int = 0
            private set
        public var lastOutcome: Outcome? = null
            private set

        /** The board was started afresh for these faces. */
        public var started: Boolean = false
            private set

        /** The flight computer's offset in use when the board began. */
        public var offsetAtStart: List<Float>? = null
            private set

        /** The faces must be measured again for the board. */
        public var again: Again? = null
            private set

        private var lastId = 0

        /** The flight computer's offset saved since the board's last save. */
        private var savedOffset: List<Float>? = null

        public val isBusy: Boolean get() = pending != null
        public val boardPresent: Boolean get() = status?.boardPresent == true

        /** A command of the user's (or the flow's own); null while another is out. */
        public fun request(op: Int): Command? {
            if (isBusy) return null
            lastId = if (lastId == 255) 1 else lastId + 1
            val c = Command(op, lastId)
            pending = c
            lastOutcome = null
            generation++
            return c
        }

        /**
         * A face is about to be measured. The first one of a fresh set
         * ([fresh]: no face captured yet, or the faces asked for again) starts
         * the board afresh, remembering the flight computer's offset in use.
         */
        public fun faceStarting(fresh: Boolean, offsetInUse: List<Float>): Command? {
            if (!boardPresent || !(fresh || !started)) return null
            val c = request(SdrAlignCommand.OP_RESTART) ?: return null
            started = true
            offsetAtStart = offsetInUse
            again = null
            savedOffset = null
            return c
        }

        /** The flight computer's offset was saved and read back. */
        public fun flightComputerSaved(offset: List<Float>): Command? {
            if (!boardPresent || !started) return null
            val o0 = offsetAtStart
            if (o0 != null) {
                val move = offset.zip(o0) { a, b -> a - b }
                if (move.any { abs(it) > OFFSET_MOVE_MAX }) {
                    again = Again(move)
                    started = false
                    savedOffset = null
                    return null
                }
            }
            savedOffset = offset
            return saveIfDue()
        }

        private fun saveIfDue(): Command? {
            if (savedOffset == null || !started || status?.isGoodToSave != true) return null
            val c = request(SdrAlignCommand.OP_SAVE) ?: return null
            savedOffset = null
            return c
        }

        /** A 0xD2 status: the pending command's answer when it carries its id in a final state; then the save if due. */
        public fun take(s: SdrAlignStatus): Command? {
            status = s
            val c = pending
            if (c != null) {
                if (s.fcCmdId != c.id || s.fcCmdOp != c.op) return null
                lastOutcome = when (s.fcCmdState) {
                    SdrCommandState.ECHOED ->
                        if (s.cmdResult == SdrCommandResult.DONE) Outcome.Done(c.op) else Outcome.NotDone(c.op, s.cmdResult)
                    SdrCommandState.NO_ANSWER -> Outcome.NoAnswer(c.op)
                    SdrCommandState.REFUSED -> Outcome.Refused(c.op)
                    else -> return null
                }
                pending = null
                generation++
            }
            return saveIfDue()
        }

        /** A timer armed at [generation] fired. */
        public fun timedOut(generation: Int) {
            val c = pending
            if (generation != this.generation || c == null) return
            lastOutcome = Outcome.NoStatus(c.op)
            pending = null
            this.generation++
        }
    }

    // ── What the screen says (iOS SdrAlignStatus.swift, verbatim) ────────

    public const val HEADER: String = "GNSS Board IMU"
    public const val FOOTER: String =
        "The GNSS board on the flight computer has its own IMU. The same six faces find how it is " +
            "mounted and calibrate its accelerometer against the flight computer's: hold each face " +
            "still for 3 s while the board pairs the readings. The first face starts the board afresh; " +
            "its saved mounting stays until the new one is saved, after the flight computer's."
    public const val SAVE: String = "Save to the GNSS Board"
    public const val CLEAR: String = "Clear the Board's Mounting…"
    public const val BUSY: String = "Talking to the GNSS board…"
    public const val CLEAR_TITLE: String = "Clear the GNSS board's mounting?"
    public const val CLEAR_CONFIRM: String = "Clear Mounting"
    public const val CLEAR_MESSAGE: String =
        "The GNSS board forgets its mounting and its accelerometer calibration, the saved ones too. " +
            "Until the faces are measured again it finds only its tilt on the pad."

    /** "Whole: 6 poses, 90° apart, the worst 0.14° off". */
    public fun mounting(s: SdrAlignStatus): String = when (s.have) {
        SdrAlignHave.NONE -> "No mounting yet"
        SdrAlignHave.TILT -> "Tilt only: one pose so far. Turn it to another face for the whole rotation."
        SdrAlignHave.WHOLE -> "Whole: ${s.poses.size} poses, ${formatFixed(s.spreadDeg.toDouble(), 0)}° apart, " +
            "the worst ${formatFixed(s.residDeg.toDouble(), 2)}° off"
    }

    /** "Turned 45.0° from the flight computer's IMU". */
    public fun rotation(s: SdrAlignStatus): String =
        "Turned ${formatFixed(s.rotationDeg, 1)}° from the flight computer's IMU"

    /** "Board X". */
    public fun axisLabel(j: Int): String = "Board ${"XYZ"[j]}"

    /** One axis's calibration: "scale +1.20 %, offset +15.60 mg", or "not calibrated yet". */
    public fun axis(s: SdrAlignStatus, j: Int): String {
        if (!s.axisCalibrated(j)) return "not calibrated yet"
        val pct = formatFixed((s.scale[j].toDouble() - 1) * 100, 2).let { if (it.startsWith("-")) it else "+$it" }
        return "scale $pct %, offset ${AccelCalReadout.mg(s.bias[j].toDouble())}"
    }

    /** What is saved on the board. */
    public fun saved(s: SdrAlignStatus): String = when (s.saved) {
        SdrAlignSaved.NONE -> "Nothing saved on the board"
        SdrAlignSaved.SAME -> "Saved on the board"
        SdrAlignSaved.DIFFERS -> "The board has a saved mounting, but uses another now: not saved yet"
    }

    /** What the board did with the last rest, when it says why a face is not counting. */
    public fun lastRest(s: SdrAlignStatus): String? = when (s.lastRest) {
        SdrRestFate.MOVING -> "The board was moving: hold it still."
        SdrRestFate.SHORT -> "The board was not still long enough: hold it still a little longer."
        SdrRestFate.LATE -> "The flight computer's reading came too late for the board."
        SdrRestFate.NOT_ONE_G -> "The readings were not 1 g: was it moving?"
        SdrRestFate.NO_TIME -> "The GNSS board could not place that reading in time."
        SdrRestFate.IMU_OFF -> "The GNSS board's IMU is off."
        else -> null
    }

    /** Why no face can count for the board now, if none can. */
    public fun blocked(s: SdrAlignStatus): String? = when {
        !s.reportHeld -> "The GNSS board is not reporting its mounting: is its firmware current?"
        s.imu == 0 -> "The GNSS board's IMU is off: its readings are not paired."
        !s.canPlaceRest ->
            "The GNSS board cannot place the flight computer's readings yet: update its firmware, or give it " +
                "the sky for a minute."
        else -> null
    }

    /** On the rail with a saved mounting: how far the board sits off it, and a warning past the limit. */
    public fun moved(s: SdrAlignStatus): String? {
        if (!s.onSavedRotation) return null
        return if (s.movedDeg > SdrAlignStatus.MOVED_WARN_DEG) {
            "The GNSS board sits ${formatFixed(s.movedDeg.toDouble(), 1)}° off its saved mounting: did it move " +
                "since its faces were measured? Measure the six faces again."
        } else {
            "On the saved mounting, ${formatFixed(s.movedDeg.toDouble(), 2)}° off it here"
        }
    }

    /** The moved line is a warning. */
    public fun movedWarns(s: SdrAlignStatus): Boolean = s.onSavedRotation && s.movedDeg > SdrAlignStatus.MOVED_WARN_DEG

    /** Whole and calibrated, but not good enough for the save to go by itself. */
    public fun poorFit(s: SdrAlignStatus): String? {
        if (!s.isComplete || s.isGoodToSave) return null
        return "Not saved by itself: ${s.poses.size} poses, the worst ${formatFixed(s.residDeg.toDouble(), 2)}° off, " +
            "the fit ${formatFixed(s.calRms.toDouble(), 3)} m/s² rms. Measure the faces again, or save it as it is."
    }

    private fun opName(op: Int): String = when (op) {
        SdrAlignCommand.OP_SAVE -> "Save"
        SdrAlignCommand.OP_CLEAR -> "Clear"
        SdrAlignCommand.OP_RESTART -> "Start over"
        else -> "Report"
    }

    public fun isGood(o: Outcome): Boolean = o is Outcome.Done

    public fun outcome(o: Outcome): String = when (o) {
        is Outcome.Done -> when (o.op) {
            SdrAlignCommand.OP_SAVE -> "Saved on the GNSS board"
            SdrAlignCommand.OP_CLEAR -> "Cleared on the GNSS board"
            SdrAlignCommand.OP_RESTART -> "The GNSS board started its mounting afresh"
            else -> "The GNSS board reported"
        }
        is Outcome.NotDone -> when (o.result) {
            SdrCommandResult.NOTHING_TO_SAVE -> "${opName(o.op)}: the GNSS board had nothing to save"
            SdrCommandResult.REFUSED -> "${opName(o.op)}: the GNSS board refused it (in flight, or its IMU off)"
            SdrCommandResult.STORAGE_FAILED -> "${opName(o.op)}: the GNSS board could not write its storage"
            else -> "${opName(o.op)}: the GNSS board did not do it"
        }
        is Outcome.Refused ->
            "${opName(o.op)}: the flight computer would not send it (in flight, after a flight, or no GNSS board)"
        is Outcome.NoAnswer -> "${opName(o.op)}: the GNSS board did not answer"
        is Outcome.NoStatus -> "${opName(o.op)}: no answer from the flight computer: is its firmware current?"
    }

    /** "The flight computer's calibration moved (X +15.60 mg, ...), ...". */
    public fun again(a: Again): String {
        val parts = (0..2).joinToString(", ") { "${"XYZ"[it]} ${AccelCalReadout.mg(a.move[it].toDouble())}" }
        return "The flight computer's calibration moved ($parts), and the GNSS board calibrates against it: " +
            "measure the six faces once more for the board."
    }
}
