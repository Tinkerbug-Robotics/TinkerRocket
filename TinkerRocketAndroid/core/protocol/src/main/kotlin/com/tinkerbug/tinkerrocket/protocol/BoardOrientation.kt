package com.tinkerbug.tinkerrocket.protocol

/**
 * The board→rocket mounting rotation of a discrete orientation code — a port
 * of `orientCodeToMatrix` in tinkerrocket-idf/components/TR_Orientation/
 * TR_Orientation.cpp, entry for entry.  Board and rocket frames are both FLU
 * (+X forward/nose, +Y left, +Z up).
 *
 *   code = nose_sel * 4 + clock          (24 proper rotations, 0 = identity)
 *   nose_sel: the BOARD axis toward the rocket nose, 0=+X 1=-X 2=+Y 3=-Y 4=+Z 5=-Z
 *   clock:    quarter-turns about rocket +X (right-hand rule) after the nose
 *             alignment
 *
 * Matrices are row-major 3×3, v_rocket = R · v_board, so v_board = Rᵀ · v_rocket.
 * Telemetry's "imo" packs (mode << 5) | code; 31 there is an exact
 * (non-discrete) auto rotation, which has no code and so no matrix here.
 */
public object BoardOrientation {

    /** Discrete codes 0 until this (ORIENT_CODE_COUNT). */
    public const val CODE_COUNT: Int = 24

    // clock=0 base rotations: the shortest arc taking the selected board axis
    // to the rocket nose (+X); -X, where the arc is ambiguous, is 180° about
    // board +Z.  kNoseBase in the firmware.
    private val NOSE_BASE: Array<IntArray> = arrayOf(
        intArrayOf(1, 0, 0, 0, 1, 0, 0, 0, 1),     // +X: identity
        intArrayOf(-1, 0, 0, 0, -1, 0, 0, 0, 1),   // -X: 180° about board +Z
        intArrayOf(0, 1, 0, -1, 0, 0, 0, 0, 1),    // +Y: shortest arc +Y→+X (90° about -Z)
        intArrayOf(0, -1, 0, 1, 0, 0, 0, 0, 1),    // -Y: shortest arc -Y→+X (90° about +Z)
        intArrayOf(0, 0, 1, 0, 1, 0, -1, 0, 0),    // +Z: shortest arc +Z→+X (90° about +Y)
        intArrayOf(0, 0, -1, 0, 1, 0, 1, 0, 0),    // -Z: shortest arc -Z→+X (90° about -Y)
    )

    // Quarter-turn about rocket +X, applied after the nose alignment:
    // kClockCos / kClockSin for clock = 0..3.
    private val CLOCK_COS: IntArray = intArrayOf(1, 0, -1, 0)
    private val CLOCK_SIN: IntArray = intArrayOf(0, 1, 0, -1)

    /**
     * R for [code], row-major, v_rocket = R · v_board.  As in the firmware, a
     * code outside 0 until [CODE_COUNT] gives the identity — callers that
     * must not guess check [isValid] first.
     */
    public fun matrix(code: Int): IntArray {
        val c0 = if (isValid(code)) code else 0
        val b = NOSE_BASE[c0 / 4]
        val c = CLOCK_COS[c0 % 4]
        val s = CLOCK_SIN[c0 % 4]
        // R = Rx(clock) · B with Rx = [[1,0,0],[0,c,-s],[0,s,c]]:
        // row0 = B row0, row1 = c·B row1 − s·B row2, row2 = s·B row1 + c·B row2.
        val r = IntArray(9)
        for (j in 0..2) {
            r[j] = b[j]
            r[3 + j] = c * b[3 + j] - s * b[6 + j]
            r[6 + j] = s * b[3 + j] + c * b[6 + j]
        }
        return r
    }

    public fun isValid(code: Int): Boolean = code in 0 until CODE_COUNT

    /** A rocket-frame vector back in the board frame: Rᵀ · [rocket]. */
    public fun toBoard(code: Int, rocket: List<Double>): List<Double> {
        require(rocket.size == 3) { "a vector has three components, got ${rocket.size}" }
        val r = matrix(code)
        return (0..2).map { j -> r[j] * rocket[0] + r[3 + j] * rocket[1] + r[6 + j] * rocket[2] }
    }
}
