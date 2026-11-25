package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.AccelCalStatus
import com.tinkerbug.tinkerrocket.protocol.AccelFaceState
import com.tinkerbug.tinkerrocket.session.AccelCalReadout.Line
import com.tinkerbug.tinkerrocket.session.AccelCalReadout.Tone
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Access
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Face
import com.tinkerbug.tinkerrocket.session.AccelSixFace.LiveFace
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Measurement
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Verdict
import com.tinkerbug.tinkerrocket.session.AccelSixFace.WriteReport
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull

/**
 * #1629: what the six-face calibration screen says — iOS's words
 * (AccelCalibrationView.swift and the text extensions in AccelCalStatus.swift
 * and AccelSixFace.swift), to the character.
 */
class AccelCalReadoutTest {

    /** The wire fixture accelcal_0xD1_40's values: stored and applied, a +Z UP window done. */
    private fun fixture(
        flags: Int = 3,
        faceState: AccelFaceState = AccelFaceState.DONE,
        offset: List<Float> = listOf(0.0412f, -0.0273f, 0.0156f),
    ) = AccelCalStatus(
        flags = flags, faceState = faceState,
        offsetX = offset[0], offsetY = offset[1], offsetZ = offset[2],
        faceMeanX = -0.0831f, faceMeanY = 0.1149f, faceMeanZ = 9.8213f,
        faceSdX = 0.0123f, faceSdY = 0.0134f, faceSdZ = 0.0145f, faceN = 2880,
    )

    private val none = fixture(flags = 0, offset = listOf(0f, 0f, 0f))

    @Test
    fun theStoredOffset() {
        // 0.0412 m/s² is 4.2012 mg: m/s² / 9.80665 × 1000, `%+.2f mg`.
        assertEquals(listOf("+4.20 mg", "-2.78 mg", "+1.59 mg"), fixture().offset.map { AccelCalReadout.mg(it.toDouble()) })
        assertEquals("X +4.20 mg, Y -2.78 mg, Z +1.59 mg", AccelCalReadout.offsets(fixture()))
        assertEquals("no offset stored", AccelCalReadout.offsets(none))
        assertEquals("In use by the flight computer", AccelCalReadout.use(fixture()))
        assertEquals("Stored, but not in use while a sim runs", AccelCalReadout.use(fixture(flags = 1)))
        assertEquals("In use, but not stored on the board", AccelCalReadout.use(fixture(flags = 2)))
        assertEquals("No offset stored", AccelCalReadout.use(none))
    }

    @Test
    fun theChecklist() {
        assertEquals(
            listOf(
                "Forward end up", "Forward end down", "Left side up", "Left side down",
                "Component side up", "Component side down",
            ),
            Face.entries.map { AccelCalReadout.howTo(it) },
        )
        assertEquals("3.1° off", AccelCalReadout.captured(3.106))
    }

    @Test
    fun eachMeasurementSaysWhatBecameOfIt() {
        assertEquals(
            Line("+Z UP, 0.8° off vertical", Tone.OK),
            AccelCalReadout.verdict(Verdict.Accepted(Face.Z_UP, 0.83, replaced = false)),
        )
        assertEquals(
            "+Z UP, 0.8° off vertical — replaced the earlier one",
            AccelCalReadout.verdict(Verdict.Accepted(Face.Z_UP, 0.83, replaced = true)).text,
        )
        assertEquals(
            Line("+X DOWN is 14° off vertical — set it squarer and measure again", Tone.WARN),
            AccelCalReadout.verdict(Verdict.Tilted(Face.X_DOWN, 13.8)),
        )
        assertEquals(
            Line("It moved, or it is not one g — hold it still and measure again", Tone.WARN),
            AccelCalReadout.verdict(Verdict.Moved),
        )
        assertEquals(
            "The flight computer refused: it measures no face in flight, after a flight, in a sim " +
                "or during a magnetometer calibration",
            AccelCalReadout.notMeasured(Measurement.Refused(fixture(faceState = AccelFaceState.FAILED)))?.text,
        )
        assertEquals(
            "No answer from the flight computer — measure again",
            AccelCalReadout.notMeasured(Measurement.NoAnswer)?.text,
        )
        assertEquals(
            AccelCalReadout.notMeasured(Measurement.NoAnswer),
            AccelCalReadout.notMeasured(Measurement.NotConnected),
        )
        assertNull(AccelCalReadout.notMeasured(Measurement.Finished(fixture())))
        assertEquals(
            "(-0.083, +0.115, +9.821) m/s², SD max 1.5 mg, 2880 samples",
            AccelCalReadout.detail(fixture()),
        )
    }

    @Test
    fun theSolvedOffset() {
        val s = AccelSixFace.Solution(
            offset = listOf(0.0412, -0.0273, 0.0156),
            scale = listOf(1.002, 0.997, 1.0),
            converged = true,
            iterations = 3,
            maxResidualMps2 = 1e-12,
        )
        assertEquals("+4.20 mg", AccelCalReadout.mg(s.offset[0]))
        assertEquals(
            listOf("scale +0.200 %", "scale -0.300 %", "scale +0.000 %"),
            (0..2).map { AccelCalReadout.scale(s, it) },
        )
        assertEquals(
            "All six faces are solved together, so a face a few degrees off square does not bias the offset.",
            AccelCalReadout.SOLVED_TOGETHER,
        )
        assertNull(AccelCalReadout.cannotSave(s))

        assertEquals(
            Line(
                "Offset past 1.0 m/s² on Y, Z — the flight computer would refuse it. " +
                    "Redo the faces: was one of them not still, or the wrong way up?",
                Tone.BAD,
            ),
            AccelCalReadout.cannotSave(s.copy(offset = listOf(0.0412, 1.3, -1.1))),
        )
        // A fit that did not converge, or a scale out of range: the redo line alone.
        val redo = Line("Redo the faces: was one of them not still, or the wrong way up?", Tone.BAD)
        assertEquals(redo, AccelCalReadout.cannotSave(s.copy(converged = false)))
        assertEquals(redo, AccelCalReadout.cannotSave(s.copy(scale = listOf(1.06, 1.0, 1.0))))
    }

    @Test
    fun theLiveFaceLine() {
        assertEquals(
            Line("On now: +Y UP (left side up), 5° off vertical", Tone.OK),
            AccelCalReadout.live(LiveFace.On(Face.Y_UP, 5.3)),
        )
        assertEquals(
            "On now: +X DOWN (forward end down), 0° off vertical",
            AccelCalReadout.live(LiveFace.On(Face.X_DOWN, 0.2)).text,
        )
        assertEquals(
            Line("Set it squarer: +Z UP is 15° off vertical", Tone.WARN),
            AccelCalReadout.live(LiveFace.Askew(Face.Z_UP, 14.6)),
        )
        // Auto-exact: the face, and no tilt to tell.
        assertEquals(
            Line("On now: +Z UP (component side up)", Tone.OK),
            AccelCalReadout.live(LiveFace.OnNoTilt(Face.Z_UP)),
        )
        assertEquals(Line("Hold it still", Tone.WARN), AccelCalReadout.live(LiveFace.Moving))
        assertEquals(Line("Waiting for the rocket's reading…"), AccelCalReadout.live(LiveFace.Waiting))
        assertEquals(
            "You don't pick the face: set the board on a side, and the app sees which one it is.",
            AccelCalReadout.FACES_NO_PICK,
        )
        assertEquals("Now", AccelCalReadout.NOW)
    }

    @Test
    fun theMeasureButtonNamesTheLiveFace() {
        assertEquals("Measure +Y UP", AccelCalReadout.measureLabel(Face.Y_UP, allCaptured = false))
        assertEquals("Measure +Y UP", AccelCalReadout.measureLabel(Face.Y_UP, allCaptured = true))
        assertEquals("Measure This Face", AccelCalReadout.measureLabel(null, allCaptured = false))
        assertEquals("Measure a Face Again", AccelCalReadout.measureLabel(null, allCaptured = true))
    }

    @Test
    fun aTappedRowSaysHowToPlaceTheBoard() {
        assertEquals(
            listOf(
                "To measure +X UP: stand it on its back end, forward end up.",
                "To measure +X DOWN: stand it on its forward end, forward end down.",
                "To measure +Y UP: lay it on its right side, left side up.",
                "To measure +Y DOWN: lay it on its left side, left side down.",
                "To measure +Z UP: lay it flat, component side up.",
                "To measure +Z DOWN: lay it flat, component side down.",
            ),
            Face.entries.map { AccelCalReadout.placement(it) },
        )
    }

    @Test
    fun theSaveAndTheClear() {
        val sent = listOf(0.0412f, -0.0273f, 0.0156f)
        val zeros = listOf(0f, 0f, 0f)
        fun save(back: AccelCalStatus?) = WriteReport(isClear = false, sent = sent, readBack = back)
        fun clear(back: AccelCalStatus?) = WriteReport(isClear = true, sent = zeros, readBack = back)

        assertEquals(Line("Saved and read back", Tone.OK), AccelCalReadout.write(save(fixture())))
        assertNull(AccelCalReadout.readBack(save(fixture())))
        val off = save(fixture(offset = listOf(0.0412f, -0.0273f, 0.0166f)))
        assertEquals(Line("The read-back does not match what was sent", Tone.WARN), AccelCalReadout.write(off))
        assertEquals("Read back: X +4.20 mg, Y -2.78 mg, Z +1.69 mg", AccelCalReadout.readBack(off))

        assertEquals(Line("Cleared: no offset stored", Tone.OK), AccelCalReadout.write(clear(none)))
        assertEquals(
            Line("The flight computer still has an offset stored", Tone.WARN),
            AccelCalReadout.write(clear(fixture())),
        )
        assertEquals("Read back: X +4.20 mg, Y -2.78 mg, Z +1.59 mg", AccelCalReadout.readBack(clear(fixture())))

        for (r in listOf(save(null), clear(null))) {
            assertEquals(
                Line("No answer from the flight computer — read it back to check", Tone.WARN),
                AccelCalReadout.write(r),
            )
            assertNull(AccelCalReadout.readBack(r))
        }
    }

    @Test
    fun whyItCannotCalibrate() {
        assertNull(AccelCalReadout.blocked(Access.READY))
        assertEquals(
            mapOf(
                Access.BASE_STATION to "Connect to the rocket itself to calibrate it.",
                Access.OFF to "Connect to the rocket and power it on to calibrate it.",
                Access.STARTING to "Wait for the flight computer to finish starting.",
                Access.FLYING to "Not while the rocket is flying.",
                Access.LANDED to "After a flight the flight computer takes no calibration until it " +
                    "restarts: power the rocket off and on.",
                Access.MAG_CAL to "Not while a magnetometer calibration is running.",
                Access.SIMULATING to "Not while a simulation is running.",
            ),
            (Access.entries - Access.READY).associateWith { AccelCalReadout.blocked(it) },
        )
    }
}
