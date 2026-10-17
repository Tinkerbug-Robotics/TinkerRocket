package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.AccelCalStatus
import com.tinkerbug.tinkerrocket.protocol.formatFixed
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Access
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Face
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Measurement
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Solution
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Verdict
import com.tinkerbug.tinkerrocket.session.AccelSixFace.WriteReport

/**
 * Every word the six-face accelerometer calibration shows (#1629), as the iOS
 * app words it: AccelCalibrationView.swift, the text extensions in
 * AccelCalStatus.swift and AccelSixFace.swift, and the Settings entry in
 * SettingsView.swift.  iOS is the reference, so wording changes there first
 * and is copied here verbatim — the same sentences, capitals and punctuation,
 * as the pad calibration keeps both apps identical.  Numbers follow iOS's
 * String(format:): `%+.2f mg`, `%+.3f %`, `%.1f°`, `%+.3f` m/s².  Kept out of
 * the composable so it can be tested ([PadCalReadout]'s twin).
 */
public object AccelCalReadout {

    public enum class Tone { NORMAL, OK, WARN, BAD }

    public data class Line(val text: String, val tone: Tone = Tone.NORMAL)

    // ── The screen, section by section ────────────────────────────────────

    public const val TITLE: String = "Accelerometer Calibration"

    public const val WHY_TITLE: String = "Six-Face Accelerometer Calibration"
    public const val WHY_TILT: String =
        "On the pad the flight computer finds level from gravity, and a small accelerometer " +
            "offset looks just like a small tilt. In flight, with GNSS correcting the attitude, " +
            "that false tilt becomes a heading error in the boost."
    public const val WHY_FACES: String =
        "Turning each axis straight up and then straight down separates the two: gravity flips " +
            "sign, the offset does not. Do it once per flight computer; the offset is stored on " +
            "the board."

    public const val STORED_TITLE: String = "Stored Offset"
    public const val READING: String = "Reading the flight computer…"
    public const val NO_READ: String =
        "No answer. This flight computer's firmware may not have the six-face calibration: " +
            "update it, or read again."
    public const val READ_AGAIN: String = "Read Again"
    public const val NOT_READ: String = "Not read yet"

    public const val FACES_TITLE: String = "Six Faces"
    public const val FACES_HOW: String =
        "Board axes: +X points forward (to the nose when the board is mounted nose-forward), " +
            "+Y to the left, +Z up out of the component side. Set the board, or the whole " +
            "rocket, on each face and hold it still for 3 s. Any order: the app tells the face " +
            "from the reading."
    public const val MEASURE: String = "Measure This Face"
    public const val MEASURE_AGAIN: String = "Measure a Face Again"
    public const val STARTING: String = "Starting…"
    public const val HOLD_STILL: String = "Hold still…"
    public const val FINISHING: String = "Finishing…"
    public const val START_OVER: String = "Start Over"

    public const val RESULT_TITLE: String = "Offset"
    public const val RESIDUALS: String = "Each face after the correction, off one g"
    public const val SAVE: String = "Save to the Flight Computer"
    public const val SAVING: String = "Saving…"
    public const val RESULT_FOOTER: String = "The scale is shown for information; only the offset is saved."

    public const val CLEAR: String = "Clear the Stored Offset…"
    public const val CLEARING: String = "Clearing…"
    public const val CLEAR_FOOTER: String =
        "Removes the offset from the flight computer, which then uses the accelerometer's " +
            "reading as it is."
    public const val CLEAR_CONFIRM_TITLE: String = "Clear the stored offset?"
    public const val CLEAR_CONFIRM_TEXT: String =
        "The flight computer goes back to the accelerometer's reading as it is. You can " +
            "measure and save again at any time."
    public const val CLEAR_CONFIRM: String = "Clear Offset"
    public const val CANCEL: String = "Cancel"
    public const val READ_IT_BACK: String = "Read It Back"

    // ── The Settings entry (iOS SettingsView's Calibration section) ───────

    public const val ENTRY: String = "Accelerometer Calibration (Six Faces)"
    public const val ENTRY_CAPTION: String =
        "Measures the accelerometer's offset with the board on each of its six faces. Once " +
            "per flight computer; stored on the board."
    public const val ENTRY_POWER: String = "Power on the rocket to run calibrations."

    // ── Lines from the numbers ────────────────────────────────────────────

    /** "+4.20 mg" from m/s²: milli-g against standard gravity. */
    public fun mg(mps2: Double): String = signed(mps2 / AccelSixFace.G0 * 1000, 2) + " mg"

    /** "X +4.20 mg, Y -2.78 mg, Z +1.59 mg", or "no offset stored". */
    public fun offsets(s: AccelCalStatus): String =
        if (!s.stored) {
            "no offset stored"
        } else {
            (0..2).joinToString(", ") { "${AXES[it]} ${mg(s.offset[it].toDouble())}" }
        }

    /** What the flight computer does with the stored offset. */
    public fun use(s: AccelCalStatus): String = when {
        s.stored && s.applied -> "In use by the flight computer"
        s.stored -> "Stored, but not in use while a sim runs"
        s.applied -> "In use, but not stored on the board"
        else -> "No offset stored"
    }

    /** A checklist face in plain words: "Forward end up". */
    public fun howTo(face: Face): String =
        listOf("Forward end", "Left side", "Component side")[face.axis] + if (face.sign > 0) " up" else " down"

    /** A captured face's tilt, beside it in the checklist: "3.1° off". */
    public fun captured(tiltDeg: Double): String = formatFixed(tiltDeg, 1) + "° off"

    /** The result line under the Measure button for a window that closed. */
    public fun verdict(v: Verdict): Line = when (v) {
        is Verdict.Accepted -> Line(
            "${v.face.label}, ${formatFixed(v.tiltDeg, 1)}° off vertical" +
                if (v.replaced) " — replaced the earlier one" else "",
            Tone.OK,
        )
        Verdict.Moved -> Line("It moved, or it is not one g — hold it still and measure again", Tone.WARN)
        is Verdict.Tilted -> Line(
            "${v.face.label} is ${formatFixed(v.tiltDeg, 0)}° off vertical — set it squarer and measure again",
            Tone.WARN,
        )
    }

    /** The result line for a measurement with no window to judge; null for one with ([verdict]). */
    public fun notMeasured(m: Measurement): Line? = when (m) {
        is Measurement.Finished -> null
        is Measurement.Refused -> Line(
            "The flight computer refused: it measures no face in flight, after a flight, in a " +
                "sim or during a magnetometer calibration",
            Tone.WARN,
        )
        // No link to send on: iOS's send fails quietly and times out to the same words.
        Measurement.NoAnswer, Measurement.NotConnected ->
            Line("No answer from the flight computer — measure again", Tone.WARN)
    }

    /** A window's numbers: "(-0.083, +0.115, +9.821) m/s², SD max 1.5 mg, 2880 samples". */
    public fun detail(w: AccelCalStatus): String {
        val mean = w.faceMean.joinToString(", ", "(", ")") { signed(it.toDouble(), 3) }
        val sdMaxMg = w.faceSd.maxOf { it }.toDouble() / AccelSixFace.G0 * 1000
        return "$mean m/s², SD max ${formatFixed(sdMaxMg, 1)} mg, ${w.faceN} samples"
    }

    /** "scale +0.123 %": the scale's difference from 1, under an axis's offset. */
    public fun scale(s: Solution, axis: Int): String = "scale " + signed((s.scale[axis] - 1) * 100, 3) + " %"

    /** Why the offset cannot be saved, if it cannot. */
    public fun tooLarge(s: Solution): Line? {
        if (s.saveAllowed) return null
        val axes = s.axesPastLimit.joinToString(", ") { AXES[it].toString() }
        return Line(
            "Offset past 1.0 m/s² on $axes — the flight computer would refuse it. " +
                "Redo the faces: was one of them not still, or the wrong way up?",
            Tone.BAD,
        )
    }

    /** The note for a face far off one g after the correction. */
    public fun residualNote(s: Solution): Line? {
        if (!s.residualNote) return null
        return Line(
            "A face is ${formatFixed(s.worstResidualMg, 1)} mg off one g after the correction " +
                "(over 5). The offset is still good if the faces were squarish; the scale is not used.",
            Tone.WARN,
        )
    }

    /** A save's or a clear's outcome. */
    public fun write(r: WriteReport): Line = when {
        r.readBack == null -> Line("No answer from the flight computer — read it back to check", Tone.WARN)
        r.isClear ->
            if (r.confirmed) Line("Cleared: no offset stored", Tone.OK)
            else Line("The flight computer still has an offset stored", Tone.WARN)
        else ->
            if (r.confirmed) Line("Saved and read back", Tone.OK)
            else Line("The read-back does not match what was sent", Tone.WARN)
    }

    /** What came back, under an outcome that did not confirm; null otherwise. */
    public fun readBack(r: WriteReport): String? {
        val back = r.readBack ?: return null
        return if (r.confirmed) null else "Read back: " + offsets(back)
    }

    /** Why the screen cannot calibrate now; null when it can. */
    public fun blocked(a: Access): String? = when (a) {
        Access.READY -> null
        Access.BASE_STATION -> "Connect to the rocket itself to calibrate it."
        Access.OFF -> "Connect to the rocket and power it on to calibrate it."
        Access.STARTING -> "Wait for the flight computer to finish starting."
        Access.FLYING -> "Not while the rocket is flying."
        Access.LANDED ->
            "After a flight the flight computer takes no calibration until it restarts: power " +
                "the rocket off and on."
        Access.MAG_CAL -> "Not while a magnetometer calibration is running."
        Access.SIMULATING -> "Not while a simulation is running."
    }

    private const val AXES = "XYZ"

    /** iOS's `%+` format: a sign on every value, zero included. */
    private fun signed(v: Double, decimals: Int): String =
        formatFixed(v, decimals).let { if (it.startsWith("-")) it else "+$it" }
}
