package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * The port of TR_Orientation.cpp's orientCodeToMatrix: every code a proper
 * rotation that takes its nose axis to the rocket nose, and three codes
 * against the C++ formula worked by hand.
 */
class BoardOrientationTest {

    private fun at(r: IntArray, row: Int, col: Int): Int = r[3 * row + col]

    /** The board axis code / 4 names: +X, -X, +Y, -Y, +Z, -Z. */
    private fun noseAxis(code: Int): IntArray {
        val sel = code / 4
        return IntArray(3).also { it[sel / 2] = if (sel % 2 == 0) 1 else -1 }
    }

    @Test
    fun everyCodeIsAProperRotation() {
        for (code in 0 until BoardOrientation.CODE_COUNT) {
            val r = BoardOrientation.matrix(code)
            // Orthonormal: Rᵀ·R = I.
            for (i in 0..2) for (j in 0..2) {
                val dot = (0..2).sumOf { k -> at(r, k, i) * at(r, k, j) }
                assertEquals(if (i == j) 1 else 0, dot, "code $code: column $i · column $j")
            }
            val det = at(r, 0, 0) * (at(r, 1, 1) * at(r, 2, 2) - at(r, 1, 2) * at(r, 2, 1)) -
                at(r, 0, 1) * (at(r, 1, 0) * at(r, 2, 2) - at(r, 1, 2) * at(r, 2, 0)) +
                at(r, 0, 2) * (at(r, 1, 0) * at(r, 2, 1) - at(r, 1, 1) * at(r, 2, 0))
            assertEquals(1, det, "code $code: a rotation, not a reflection")
        }
    }

    @Test
    fun theNoseAxisGoesToTheRocketNose_atEveryClock() {
        for (code in 0 until BoardOrientation.CODE_COUNT) {
            val r = BoardOrientation.matrix(code)
            val nose = noseAxis(code)
            val v = IntArray(3) { row -> (0..2).sumOf { k -> at(r, row, k) * nose[k] } }
            assertContentEquals(intArrayOf(1, 0, 0), v, "code $code (${b2rOrientationName(code)})")
        }
    }

    @Test
    fun threeCodesAgainstTheFormulaByHand() {
        // 0: +X, clock 0 — B is the identity, Rx(0) is the identity.
        assertContentEquals(intArrayOf(1, 0, 0, 0, 1, 0, 0, 0, 1), BoardOrientation.matrix(0))
        // 16: +Z, clock 0 — R = B = kNoseBase[4].
        assertContentEquals(intArrayOf(0, 0, 1, 0, 1, 0, -1, 0, 0), BoardOrientation.matrix(16))
        // 23: -Z, clock 3 — B = kNoseBase[5] = [0,0,-1; 0,1,0; 1,0,0], c = 0, s = -1:
        // row0 = B row0 = (0, 0, -1); row1 = c·B1 − s·B2 = (1, 0, 0);
        // row2 = s·B1 + c·B2 = (0, -1, 0).
        assertContentEquals(intArrayOf(0, 0, -1, 1, 0, 0, 0, -1, 0), BoardOrientation.matrix(23))
    }

    @Test
    fun anInvalidCodeIsTheFirmwaresIdentity_andSaysSo() {
        for (code in listOf(24, 31, -1)) {
            assertFalse(BoardOrientation.isValid(code), "code $code")
            assertContentEquals(BoardOrientation.matrix(0), BoardOrientation.matrix(code), "code $code")
        }
        assertTrue((0 until 24).all { BoardOrientation.isValid(it) })
    }

    @Test
    fun toBoardUndoesTheMounting() {
        val board = listOf(0.3, -9.7, 1.1)
        for (code in 0 until BoardOrientation.CODE_COUNT) {
            val r = BoardOrientation.matrix(code)
            val rocket = (0..2).map { row -> (0..2).sumOf { k -> at(r, row, k) * board[k] } }
            val back = BoardOrientation.toBoard(code, rocket)
            for (i in 0..2) assertEquals(board[i], back[i], 1e-12, "code $code axis $i")
        }
    }
}
