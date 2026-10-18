package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.AccelCalStatus
import com.tinkerbug.tinkerrocket.protocol.HighGBiasSource
import com.tinkerbug.tinkerrocket.protocol.formatFixed
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Access
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Face
import com.tinkerbug.tinkerrocket.session.AccelSixFace.HighGProblem
import com.tinkerbug.tinkerrocket.session.AccelSixFace.HighGResult
import com.tinkerbug.tinkerrocket.session.AccelSixFace.HighGSolution
import com.tinkerbug.tinkerrocket.session.AccelSixFace.HighGWrite
import com.tinkerbug.tinkerrocket.session.AccelSixFace.HighGWriteReport
import com.tinkerbug.tinkerrocket.session.AccelSixFace.LiveFace
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Measurement
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Solution
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Verdict
import com.tinkerbug.tinkerrocket.session.AccelSixFace.WriteReport
import kotlin.math.abs

/**
 * Every word the six-face accelerometer calibration shows (#1629), as the iOS
 * app words it: AccelCalibrationView.swift, the text extensions in
 * AccelCalStatus.swift and AccelSixFace.swift, and the Settings entry in
 * SettingsView.swift.  iOS is the reference, so wording changes there first
 * and is copied here verbatim — the same sentences, capitals and punctuation,
 * as the pad calibration keeps both apps identical.  Numbers follow iOS's
 * String(format:): `%+.2f mg`, `%+.3f %`, `%.1f°`, `%+.3f` m/s².  Kept out of
 * the composable so it can be tested ([PadCalReadout]'s twin).  The live face,
 * the "Now" tag, the named Measure button and the placement lines came in a
 * second round, with the same strings specified for both apps at once; the
 * high-g's section, rows and clear wording in a third, from iOS's.
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
    public const val FACES_NO_PICK: String =
        "You don't pick the face: set the board on a side, and the app sees which one it is."
    public const val NOW: String = "Now"
    /**
     * The rows' accessibility action — iOS's hint "Shows how to place the
     * board for this face", as TalkBack phrases an action: "Double-tap to
     * show how to place the board for this face".
     */
    public const val ROW_ACTION: String = "show how to place the board for this face"
    public const val MEASURE: String = "Measure This Face"
    public const val MEASURE_AGAIN: String = "Measure a Face Again"
    public const val STARTING: String = "Starting…"
    public const val HOLD_STILL: String = "Hold still…"
    public const val FINISHING: String = "Finishing…"
    public const val START_OVER: String = "Start Over"

    public const val RESULT_TITLE: String = "Offset"
    public const val SOLVED_TOGETHER: String =
        "All six faces are solved together, so a face a few degrees off square does not bias the offset."
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

    // With a high-g alignment stored, the clear takes it too, and says so.
    public const val CLEAR_ALL: String = "Clear the Stored Calibration…"
    public const val CLEAR_ALL_FOOTER: String =
        "Removes the offset and the high-g alignment from the flight computer, which then uses " +
            "the low-g reading as it is and the high-g with its bias alone."
    public const val CLEAR_ALL_CONFIRM_TITLE: String = "Clear the stored calibration?"
    public const val CLEAR_ALL_CONFIRM_TEXT: String =
        "The flight computer goes back to the low-g reading as it is and to the high-g without " +
            "its alignment (it keeps the high-g's bias until the next pad calibration). You can " +
            "measure and save again at any time."
    public const val CLEAR_ALL_CONFIRM: String = "Clear Calibration"

    // ── The high-g ────────────────────────────────────────────────────────

    public const val HG_TITLE: String = "High-g Accelerometer"
    public const val HG_SAVING: String = "Saving the high-g…"
    public const val HG_PASSED: String = "Passed: saved with the offset"
    public const val HG_FOOTER: String =
        "The high-g is fitted to the low-g, the more accurate channel on the bench, so the " +
            "flight computer reads the same acceleration when it switches to the high-g near " +
            "16 g. ± is one standard deviation from these faces. The pad calibration keeps this " +
            "alignment and measures the high-g's offset again on the day."
    /** Under the stored offset, from a frame with no high-g part. */
    public const val HG_NOT_SENT: String =
        "This firmware sends no high-g calibration: update the flight computer and the out " +
            "computer to calibrate the high-g."

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

    /** The live line at the top of the checklist, from the newest telemetry frame. */
    public fun live(l: LiveFace): Line = when (l) {
        is LiveFace.On -> Line(
            "On now: ${l.face.label} (${hint(l.face)}), ${formatFixed(l.tiltDeg, 0)}° off vertical",
            Tone.OK,
        )
        // An auto-exact mounting: the face, with no tilt to tell.
        is LiveFace.OnNoTilt -> Line("On now: ${l.face.label} (${hint(l.face)})", Tone.OK)
        is LiveFace.Askew ->
            Line("Set it squarer: ${l.face.label} is ${formatFixed(l.tiltDeg, 0)}° off vertical", Tone.WARN)
        LiveFace.Moving -> Line("Hold it still", Tone.WARN)
        LiveFace.Waiting -> Line("Waiting for the rocket's reading…")
    }

    /**
     * The Measure button: the live face's name while the board sits square on
     * one, or on one under an auto-exact mounting ([onFace]), otherwise as
     * before.  Only a hint — the button works in every live state, and the
     * window judges its own face.
     */
    public fun measureLabel(onFace: Face?, allCaptured: Boolean): String = when {
        onFace != null -> "Measure ${onFace.label}"
        allCaptured -> MEASURE_AGAIN
        else -> MEASURE
    }

    /** A tapped row's line under the checklist: how to place the board for [face]. */
    public fun placement(face: Face): String = "To measure ${face.label}: " + when (face) {
        Face.X_UP -> "stand it on its back end, forward end up."
        Face.X_DOWN -> "stand it on its forward end, forward end down."
        Face.Y_UP -> "lay it on its right side, left side up."
        Face.Y_DOWN -> "lay it on its left side, left side down."
        Face.Z_UP -> "lay it flat, component side up."
        Face.Z_DOWN -> "lay it flat, component side down."
    }

    /** {hint}: the face's description in lower case, "left side up". */
    private fun hint(face: Face): String = howTo(face).replaceFirstChar { it.lowercaseChar() }

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

    /**
     * Why the offset cannot be saved, if it cannot: past the flight
     * computer's limit (the axes named), or a fit that did not converge or
     * found a scale outside 0.97..1.03 — then the redo line alone.
     */
    public fun cannotSave(s: Solution): Line? {
        if (s.saveAllowed) return null
        if (s.axesPastLimit.isEmpty()) return Line(REDO, Tone.BAD)
        val axes = s.axesPastLimit.joinToString(", ") { AXES[it].toString() }
        return Line("Offset past 1.0 m/s² on $axes — the flight computer would refuse it. $REDO", Tone.BAD)
    }

    private const val REDO: String = "Redo the faces: was one of them not still, or the wrong way up?"

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

    // ── The high-g's lines ───────────────────────────────────────────────

    /** "scale +0.812 % ± 0.041": the scale's difference from 1 and its 1-sigma. */
    public fun hgScale(s: HighGSolution, axis: Int): String =
        "scale " + signed((s.matrix[4 * axis] - 1) * 100, 3) + " % ± " + formatFixed(s.sigmaMatrix[4 * axis] * 100, 3)

    /** "offset +66.4 mg ± 0.24": the offset against the low-g. */
    public fun hgOffset(s: HighGSolution, axis: Int): String =
        "offset " + signed(s.bias[axis] / AccelSixFace.G0 * 1000, 1) + " mg ± " +
            formatFixed(s.sigmaBias[axis] / AccelSixFace.G0 * 1000, 2)

    public fun hgCross(s: HighGSolution): String =
        "Cross-axis: up to ${formatFixed(s.crossDeg, 2)}° (± ${formatFixed(s.sigmaCrossDeg, 3)}°)"

    public fun hgResidual(s: HighGSolution): String =
        "Fit to the low-g: ${formatFixed(s.residualRms / AccelSixFace.G0 * 1000, 2)} mg rms, " +
            "${formatFixed(s.residualMax / AccelSixFace.G0 * 1000, 2)} mg at most " +
            "(limit ${formatFixed(AccelSixFace.HG_RESIDUAL_MAX_MPS2 / AccelSixFace.G0 * 1000, 0)} mg)"

    /**
     * The two fits' agreement: the high-g's own gravity fit against what the
     * alignment to the low-g implies.  WARN (iOS's orange) when they disagree.
     */
    public fun hgAgreement(s: HighGSolution): Line {
        val first = s.disagreeAxes.firstOrNull()
            ?: return Line("Its own gravity fit agrees with the alignment to the low-g within their noise.")
        val axes = s.disagreeAxes.joinToString(", ") { AXES[it].toString() }
        val off = abs(s.diffOffset[first]) / AccelSixFace.G0 * 1000
        val lim = AccelSixFace.HG_AGREE_SIGMAS * s.sigmaDiffOffset[first] / AccelSixFace.G0 * 1000
        val scl = abs(s.diffScale[first]) * 100
        val sLim = AccelSixFace.HG_AGREE_SIGMAS * s.sigmaDiffScale[first] * 100
        val numbers = "${formatFixed(off, 1)} mg (3σ ${formatFixed(lim, 1)}) and " +
            "${formatFixed(scl, 3)} % (3σ ${formatFixed(sLim, 3)})"
        return Line(
            "Its own gravity fit differs on $axes by $numbers: the alignment to the low-g, the " +
                "bench's reference, is what gets saved.",
            Tone.WARN,
        )
    }

    /** Why the alignment is not saved, if it is not: the first problem. */
    public fun hgProblem(s: HighGSolution): String? = when (s.problems.firstOrNull()) {
        null -> null
        HighGProblem.RESIDUAL ->
            "A face fits the low-g worse than 3 mg — did it move, or warm up between faces? Measure the faces again."
        HighGProblem.SCALE -> "A scale is outside 0.97–1.03."
        HighGProblem.CROSS -> "A cross-axis term is past 0.03 (1.7°)."
        HighGProblem.BIAS -> "An offset is past 11 m/s²."
    }

    /** The section's verdict: saved with the offset, or why not. */
    public fun hgVerdict(s: HighGSolution): Line =
        hgProblem(s)?.let { Line("Not saved: $it", Tone.BAD) } ?: Line(HG_PASSED, Tone.OK)

    /** The line when there is nothing to show: no high-g with the faces, or no fit. */
    public fun hgUnavailable(r: HighGResult): String? = when (r) {
        HighGResult.NoReading ->
            "The faces came without the high-g reading. Update the flight computer and the out " +
                "computer to calibrate the high-g; the low-g offset saves as before."
        HighGResult.Singular -> "The high-g fit failed: are the six faces six different sides?"
        is HighGResult.Solved -> null
    }

    /** A high-g save's or clear's outcome. */
    public fun hgWrite(r: HighGWriteReport): Line {
        val back = r.readBack
            ?: return Line("No answer about the high-g from the flight computer — read it back to check", Tone.WARN)
        if (back.highG == null) {
            return Line("The answer has no high-g in it: update the flight computer and the out computer", Tone.WARN)
        }
        return when (r.write) {
            is HighGWrite.Save ->
                if (r.confirmed) Line("High-g alignment saved and read back", Tone.OK)
                else Line("The high-g read-back does not match what was sent", Tone.WARN)
            HighGWrite.Clear ->
                if (r.confirmed) Line("Cleared: no high-g alignment stored", Tone.OK)
                else Line("The flight computer still has a high-g alignment stored", Tone.WARN)
        }
    }

    /** "X +0.812 %, Y -0.664 %, Z +1.224 %": the stored alignment's scales. */
    public fun hgScales(h: AccelCalStatus.HighG): String =
        (0..2).joinToString(", ") { "${AXES[it]} " + signed((h.matrix[4 * it].toDouble() - 1) * 100, 3) + " %" }

    /** "X -66.4 mg, Y +150.9 mg, Z -31.9 mg": the bias in use. */
    public fun hgBias(h: AccelCalStatus.HighG): String =
        (0..2).joinToString(", ") { "${AXES[it]} " + signed(h.bias[it].toDouble() / AccelSixFace.G0 * 1000, 1) + " mg" }

    /** The stored-section row: "High-g bias: X -66.4 mg, …". */
    public fun hgBiasRow(h: AccelCalStatus.HighG): String = "High-g bias: " + hgBias(h)

    /** What wrote the bias in use, lower case as iOS's sourceText. */
    public fun hgSource(h: AccelCalStatus.HighG): String = when (h.source) {
        HighGBiasSource.PAD_CAL -> "measured by the pad calibration"
        HighGBiasSource.SIX_FACES -> "from the six faces"
        HighGBiasSource.NONE, null -> "source not recorded"
    }

    /** The stored-section row under the bias: [hgSource] capitalized. */
    public fun hgSourceRow(h: AccelCalStatus.HighG): String = hgSource(h).replaceFirstChar { it.uppercaseChar() }

    /** What the flight computer does with the stored alignment. */
    public fun hgAlignment(h: AccelCalStatus.HighG): String = when {
        h.stored && h.applied -> "High-g alignment in use: " + hgScales(h)
        h.stored -> "High-g alignment stored, but not in use while a sim runs"
        else -> "No high-g alignment stored"
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
