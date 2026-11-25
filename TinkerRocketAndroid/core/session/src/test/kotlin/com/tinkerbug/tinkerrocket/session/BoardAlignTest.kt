package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.SdrAlignCommand
import com.tinkerbug.tinkerrocket.protocol.SdrAlignHave
import com.tinkerbug.tinkerrocket.protocol.SdrAlignSaved
import com.tinkerbug.tinkerrocket.protocol.SdrAlignStatus
import com.tinkerbug.tinkerrocket.protocol.SdrCommandResult
import com.tinkerbug.tinkerrocket.protocol.SdrCommandState
import com.tinkerbug.tinkerrocket.protocol.SdrRestFate
import com.tinkerbug.tinkerrocket.session.AccelSixFace.Face
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * The gnss-sdr-dev board's part of the six-face calibration — iOS
 * SdrAlignStatusTests' flow and words, case for case: the poses matched to
 * the faces, the restart with the first face, the save after the flight
 * computer's, the faces asked for again when its offset moved, and the
 * command answers.
 */
class BoardAlignTest {

    /** The golden's numbers (fileops/sdralign_0xD2_107.bin): six faces, whole, every axis calibrated. */
    private val golden = SdrAlignStatus(
        ver = 1, flags = 0x5F, imu = 2, have = SdrAlignHave.WHOLE, from = 1, saved = SdrAlignSaved.SAME,
        calAxes = 7, lastRestRaw = SdrRestFate.PAIRED.raw, gen = 17, nRest = 44, nPair = 13, cmdId = 42,
        cmdOp = SdrAlignCommand.OP_SAVE, cmdResRaw = SdrCommandResult.DONE.raw,
        fcCmdStateRaw = SdrCommandState.ECHOED.raw, fcCmdOp = SdrAlignCommand.OP_SAVE, fcCmdId = 9,
        restsSent = 51, restAgeDs = 23, boardAgeDs = 4, spreadDeg = 90f, residDeg = 0.14f, calRms = 0.012f,
        q = listOf(0.70710677f, 0.0012f, -0.0034f, 0.70710677f), scale = listOf(1.012f, 0.987f, 1.003f),
        bias = listOf(0.153f, -0.271f, 0.0089f),
        poses = listOf(
            listOf(127, 0, 0) to 2, listOf(-127, 1, -2) to 3, listOf(0, 127, 3) to 2,
            listOf(-4, -127, 0) to 1, listOf(5, 0, 127) to 4, listOf(0, -6, -127) to 255,
        ).map { (d, n) -> SdrAlignStatus.Pose(d.map { it / 127.0 }, n) },
    )

    /** The board on J1 with nothing yet, as it reports at the start. */
    private val fresh = golden.copy(
        have = SdrAlignHave.NONE, calAxes = 0, poses = emptyList(),
        fcCmdStateRaw = SdrCommandState.IDLE.raw, fcCmdId = 0, fcCmdOp = 0,
    )

    private fun answer(
        c: BoardAlign.Command,
        state: SdrCommandState,
        result: SdrCommandResult = SdrCommandResult.DONE,
        from: SdrAlignStatus,
    ): SdrAlignStatus = from.copy(fcCmdStateRaw = state.raw, fcCmdId = c.id, fcCmdOp = c.op, cmdResRaw = result.raw)

    // ── The faces and the words ───────────────────────────────────────────

    @Test
    fun eachPoseIsMatchedToItsFace() {
        val byFace = BoardAlign.pairsByFace(golden)
        assertEquals(2, byFace[Face.X_UP])
        assertEquals(3, byFace[Face.X_DOWN])
        assertEquals(2, byFace[Face.Y_UP])
        assertEquals(1, byFace[Face.Y_DOWN])
        assertEquals(4, byFace[Face.Z_UP])
        assertEquals(255, byFace[Face.Z_DOWN])
    }

    @Test
    fun aPoseFarFromEveryFaceIsNoFaces() {
        assertNull(BoardAlign.faceOf(SdrAlignStatus.Pose(listOf(sin(PI / 6), 0.0, cos(PI / 6)), 1)))
        assertEquals(Face.Z_UP, BoardAlign.faceOf(SdrAlignStatus.Pose(listOf(sin(PI / 12), 0.0, cos(PI / 12)), 1)))
    }

    @Test
    fun theWords() {
        assertTrue(golden.isComplete)
        assertEquals("Whole: 6 poses, 90° apart, the worst 0.14° off", BoardAlign.mounting(golden))
        assertEquals("Saved on the board", BoardAlign.saved(golden))
        assertEquals("Turned 90.0° from the flight computer's IMU", BoardAlign.rotation(golden))
        assertEquals("scale +1.20 %, offset +15.60 mg", BoardAlign.axis(golden, 0))
        assertEquals("scale -1.30 %, offset -27.63 mg", BoardAlign.axis(golden, 1))
        assertEquals("not calibrated yet", BoardAlign.axis(golden.copy(calAxes = 3), 2))
        assertEquals("Board Y", BoardAlign.axisLabel(1))
        assertNull(BoardAlign.blocked(golden))
        assertNull(BoardAlign.lastRest(golden))
        assertEquals(
            "The GNSS board is not reporting its mounting: is its firmware current?",
            BoardAlign.blocked(golden.copy(flags = SdrAlignStatus.FLAG_BOARD)),
        )
        assertEquals("The GNSS board's IMU is off: its readings are not paired.", BoardAlign.blocked(golden.copy(imu = 0)))
        assertEquals(
            "The GNSS board cannot place the flight computer's readings yet: update its firmware, or give it " +
                "the sky for a minute.",
            BoardAlign.blocked(
                golden.copy(flags = SdrAlignStatus.FLAG_BOARD or SdrAlignStatus.FLAG_REPORT or SdrAlignStatus.FLAG_LINE),
            ),
        )
        assertNull(
            BoardAlign.blocked(
                golden.copy(flags = SdrAlignStatus.FLAG_BOARD or SdrAlignStatus.FLAG_REPORT or SdrAlignStatus.FLAG_TIME),
            ),
            "unstamped rests: the board places them at their arrival",
        )
        assertTrue(golden.isGoodToSave)
        assertNull(BoardAlign.poorFit(golden))
        assertNull(BoardAlign.moved(golden), "from 1: this boot's rests")
        assertEquals(
            "The board was moving: hold it still.",
            BoardAlign.lastRest(golden.copy(lastRestRaw = SdrRestFate.MOVING.raw)),
        )
        assertEquals(
            "Tilt only: one pose so far. Turn it to another face for the whole rotation.",
            BoardAlign.mounting(golden.copy(have = SdrAlignHave.TILT)),
        )
        assertEquals(
            "The board has a saved mounting, but uses another now: not saved yet",
            BoardAlign.saved(golden.copy(saved = SdrAlignSaved.DIFFERS)),
        )
        assertEquals(90.0, golden.rotationDeg, 0.01)
        assertEquals(sqrt(0.5).toFloat(), golden.q[0], 1e-6f)
    }

    @Test
    fun theRailPoseOffTheSavedMounting() {
        assertEquals("On the saved mounting, 0.42° off it here", BoardAlign.moved(golden.copy(from = 3, movedDeg = 0.42f)))
        assertFalse(BoardAlign.movedWarns(golden.copy(from = 3, movedDeg = 0.42f)))
        assertEquals(
            "The GNSS board sits 2.5° off its saved mounting: did it move since its faces were measured? " +
                "Measure the six faces again.",
            BoardAlign.moved(golden.copy(from = 3, movedDeg = 2.5f)),
        )
        assertTrue(BoardAlign.movedWarns(golden.copy(from = 3, movedDeg = 2.5f)))
        assertNull(BoardAlign.moved(golden.copy(from = 2, movedDeg = 2.5f)), "moved is from 3's")
    }

    @Test
    fun aPoorFitIsNotSavedByItself() {
        assertFalse(golden.copy(residDeg = 1.5f).isGoodToSave)
        assertFalse(golden.copy(calRms = 0.06f).isGoodToSave)
        assertFalse(golden.copy(poses = golden.poses.take(5)).isGoodToSave)
        assertEquals(
            "Not saved by itself: 6 poses, the worst 1.50° off, the fit 0.012 m/s² rms. Measure the faces again, " +
                "or save it as it is.",
            BoardAlign.poorFit(golden.copy(residDeg = 1.5f)),
        )
        val f = BoardAlign.Flow()
        f.take(fresh)
        val r = assertNotNull(f.faceStarting(fresh = true, offsetInUse = listOf(0f, 0f, 0f)))
        val poor = golden.copy(
            residDeg = 1.5f, fcCmdStateRaw = SdrCommandState.ECHOED.raw, fcCmdId = r.id, fcCmdOp = r.op,
        )
        f.take(poor)
        assertNull(f.flightComputerSaved(listOf(0f, 0f, 0f)), "the board's own rule's quality")
        val s = assertNotNull(f.take(poor.copy(residDeg = 0.4f)), "measured better: saved")
        assertEquals(SdrAlignCommand.OP_SAVE, s.op)
    }

    // ── The flow ──────────────────────────────────────────────────────────

    @Test
    fun theFirstFaceStartsTheBoardAfresh() {
        val f = BoardAlign.Flow()
        assertNull(f.take(fresh))
        val c = assertNotNull(f.faceStarting(fresh = true, offsetInUse = listOf(0.01f, 0f, 0f)))
        assertEquals(SdrAlignCommand.OP_RESTART, c.op)
        assertEquals(1, c.id)
        assertTrue(f.isBusy)
        assertNull(f.faceStarting(fresh = false, offsetInUse = listOf(0f, 0f, 0f)), "one restart for the set")
        assertNull(f.take(answer(c, SdrCommandState.SENDING, from = fresh)))
        assertTrue(f.isBusy)
        assertNull(f.take(answer(c, SdrCommandState.ECHOED, from = fresh)))
        assertFalse(f.isBusy)
        assertEquals(BoardAlign.Outcome.Done(SdrAlignCommand.OP_RESTART), f.lastOutcome)
        assertNull(f.faceStarting(fresh = false, offsetInUse = listOf(0f, 0f, 0f)), "not again for the second face")
        assertEquals(listOf(0.01f, 0f, 0f), f.offsetAtStart)
    }

    @Test
    fun noBoardNoCommands() {
        val f = BoardAlign.Flow()
        assertNull(f.faceStarting(fresh = true, offsetInUse = listOf(0f, 0f, 0f)), "no status: no board")
        f.take(fresh.copy(flags = 0))
        assertNull(f.faceStarting(fresh = true, offsetInUse = listOf(0f, 0f, 0f)), "a rocket without the board")
    }

    @Test
    fun theBoardSavesAfterTheFlightComputer() {
        val f = BoardAlign.Flow()
        f.take(fresh)
        val r = assertNotNull(f.faceStarting(fresh = true, offsetInUse = listOf(0.04f, -0.03f, 0.01f)))
        f.take(answer(r, SdrCommandState.ECHOED, from = fresh))
        val complete = golden.copy(
            saved = SdrAlignSaved.DIFFERS, fcCmdStateRaw = SdrCommandState.ECHOED.raw, fcCmdId = r.id, fcCmdOp = r.op,
        )
        assertNull(f.take(complete), "no save before the flight computer's")
        val s = assertNotNull(f.flightComputerSaved(listOf(0.0410f, -0.0300f, 0.0095f)))
        assertEquals(SdrAlignCommand.OP_SAVE, s.op)
        assertEquals(2, s.id)
        f.take(answer(s, SdrCommandState.ECHOED, from = complete))
        assertEquals(BoardAlign.Outcome.Done(SdrAlignCommand.OP_SAVE), f.lastOutcome)
        assertEquals("Saved on the GNSS board", BoardAlign.outcome(f.lastOutcome!!))
        assertNull(f.take(complete), "once for that save")
    }

    @Test
    fun theBoardSavesWhenItCompletesAfterTheFlightComputer() {
        val f = BoardAlign.Flow()
        f.take(fresh)
        val r = assertNotNull(f.faceStarting(fresh = true, offsetInUse = listOf(0f, 0f, 0f)))
        val five = golden.copy(calAxes = 3, fcCmdStateRaw = SdrCommandState.ECHOED.raw, fcCmdId = r.id, fcCmdOp = r.op)
        f.take(five)
        assertNull(f.flightComputerSaved(listOf(0.001f, 0f, 0f)), "the board's z not calibrated yet")
        val s = assertNotNull(f.take(five.copy(calAxes = 7)), "the last pair came in: save")
        assertEquals(SdrAlignCommand.OP_SAVE, s.op)
    }

    @Test
    fun aMovedOffsetAsksForTheFacesAgain() {
        val f = BoardAlign.Flow()
        f.take(fresh)
        val r = assertNotNull(f.faceStarting(fresh = true, offsetInUse = listOf(0f, 0f, 0f)))
        val complete = golden.copy(fcCmdStateRaw = SdrCommandState.ECHOED.raw, fcCmdId = r.id, fcCmdOp = r.op)
        f.take(complete)
        assertNull(f.flightComputerSaved(listOf(-0.0155f, -0.1569f, 0.0088f)))
        val again = assertNotNull(f.again)
        assertEquals(-0.1569f, again.move[1], 1e-6f)
        assertTrue(
            BoardAlign.again(again).startsWith(
                "The flight computer's calibration moved (X -1.58 mg, Y -16.00 mg, Z +0.90 mg)",
            ),
        )
        assertNull(f.take(complete), "nothing saved against the old offset")
        val r2 = assertNotNull(f.faceStarting(fresh = false, offsetInUse = listOf(-0.0155f, -0.1569f, 0.0088f)))
        assertEquals(SdrAlignCommand.OP_RESTART, r2.op)
        assertNull(f.again)
        f.take(answer(r2, SdrCommandState.ECHOED, from = fresh))
        f.take(complete.copy(fcCmdId = r2.id, fcCmdOp = r2.op))
        val s = assertNotNull(f.flightComputerSaved(listOf(-0.0150f, -0.1580f, 0.0090f)))
        assertEquals(SdrAlignCommand.OP_SAVE, s.op)
    }

    @Test
    fun refusedNoAnswerAndSilence() {
        val f = BoardAlign.Flow()
        f.take(fresh)
        var c = assertNotNull(f.request(SdrAlignCommand.OP_CLEAR))
        f.take(answer(c, SdrCommandState.REFUSED, from = fresh))
        assertEquals(BoardAlign.Outcome.Refused(SdrAlignCommand.OP_CLEAR), f.lastOutcome)
        c = assertNotNull(f.request(SdrAlignCommand.OP_SAVE))
        f.take(answer(c, SdrCommandState.NO_ANSWER, from = fresh))
        assertEquals(BoardAlign.Outcome.NoAnswer(SdrAlignCommand.OP_SAVE), f.lastOutcome)
        assertEquals("Save: the GNSS board did not answer", BoardAlign.outcome(f.lastOutcome!!))
        c = assertNotNull(f.request(SdrAlignCommand.OP_SAVE))
        f.take(answer(c, SdrCommandState.ECHOED, SdrCommandResult.NOTHING_TO_SAVE, from = fresh))
        assertEquals(
            BoardAlign.Outcome.NotDone(SdrAlignCommand.OP_SAVE, SdrCommandResult.NOTHING_TO_SAVE), f.lastOutcome,
        )
        assertNotNull(f.request(SdrAlignCommand.OP_REPORT))
        val gen = f.generation
        f.timedOut(gen - 1)
        assertTrue(f.isBusy, "an older timer does nothing")
        f.timedOut(gen)
        assertEquals(BoardAlign.Outcome.NoStatus(SdrAlignCommand.OP_REPORT), f.lastOutcome)
        assertFalse(f.isBusy)
    }

    @Test
    fun anAnswerToAnotherIdIsNotThisOnes() {
        val f = BoardAlign.Flow()
        f.take(fresh)
        val c = assertNotNull(f.request(SdrAlignCommand.OP_SAVE))
        assertNull(f.request(SdrAlignCommand.OP_CLEAR), "one at a time")
        f.take(fresh.copy(fcCmdStateRaw = SdrCommandState.ECHOED.raw, fcCmdId = c.id + 1, fcCmdOp = c.op))
        assertTrue(f.isBusy)
        f.take(fresh.copy(fcCmdStateRaw = SdrCommandState.ECHOED.raw, fcCmdId = c.id, fcCmdOp = SdrAlignCommand.OP_CLEAR))
        assertTrue(f.isBusy, "same id, another op: an older answer")
    }

    @Test
    fun idsWrapPastZero() {
        val f = BoardAlign.Flow()
        f.take(fresh)
        var last = 0
        repeat(256) {
            val c = assertNotNull(f.request(SdrAlignCommand.OP_REPORT))
            last = c.id
            assertTrue(c.id in 1..255)
            f.timedOut(f.generation)
        }
        assertEquals(1, last)
    }
}
