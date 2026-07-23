package com.tinkerbug.tinkerrocket.protocol

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Pins for the LTTB port of iOS `CSVParser.lttbDecimate` (no iOS test twin —
 * the expectations here are hand-computed against the Swift bucket
 * arithmetic, so any Kotlin drift from the Swift rounding shows up as a
 * changed selection).
 */
class LttbTest {

    /** count <= targetCount → passthrough, zipped unchanged. */
    @Test
    fun `passthrough when at or under target`() {
        val x = listOf(0.0, 1.0, 2.0, 3.0)
        val y = listOf(5.0, 6.0, 7.0, 8.0)
        assertEquals(
            listOf(LttbPoint(0.0, 5.0), LttbPoint(1.0, 6.0), LttbPoint(2.0, 7.0), LttbPoint(3.0, 8.0)),
            Lttb.decimate(x, y, 4),
        )
        assertEquals(4, Lttb.decimate(x, y, 100).size)
    }

    /** targetCount < 3 → passthrough even when count is larger. */
    @Test
    fun `passthrough when target under three`() {
        val x = (0 until 10).map { it.toDouble() }
        val y = (0 until 10).map { it * 2.0 }
        val out = Lttb.decimate(x, y, 2)
        assertEquals(10, out.size)
        assertEquals(LttbPoint(9.0, 18.0), out.last())
    }

    /** Passthrough zip truncates to the shorter input, like Swift `zip`. */
    @Test
    fun `passthrough zip truncates to shorter series`() {
        val out = Lttb.decimate(listOf(0.0, 1.0, 2.0), listOf(9.0, 8.0), 100)
        assertEquals(listOf(LttbPoint(0.0, 9.0), LttbPoint(1.0, 8.0)), out)
    }

    /**
     * Hand-computed bucket case (count=10 → target=5, bucketSize=8/3):
     * buckets are [1..2], [3..5], [6..8]; the y-spikes at indices 2 and 7 and
     * the post-spike pivot at index 3 win their triangles, so the selection
     * is exactly indices [0, 2, 3, 7, 9].
     */
    @Test
    fun `hand computed selection for ten to five`() {
        val x = (0 until 10).map { it.toDouble() }
        val y = listOf(0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0)
        val out = Lttb.decimate(x, y, 5)
        assertEquals(
            listOf(
                LttbPoint(0.0, 0.0),
                LttbPoint(2.0, 10.0),
                LttbPoint(3.0, 0.0),
                LttbPoint(7.0, 5.0),
                LttbPoint(9.0, 0.0),
            ),
            out,
        )
    }

    /** Decimation always returns exactly targetCount points, keeping first and last. */
    @Test
    fun `first and last always kept at target size`() {
        val n = 1000
        val x = (0 until n).map { it * 0.01 }
        val y = (0 until n).map { kotlin.math.sin(it * 0.05) * it }
        for (target in listOf(3, 10, 137, 999)) {
            val out = Lttb.decimate(x, y, target)
            assertEquals(target, out.size, "target=$target")
            assertEquals(LttbPoint(x.first(), y.first()), out.first(), "target=$target")
            assertEquals(LttbPoint(x.last(), y.last()), out.last(), "target=$target")
            // Selected x values strictly increase (indices move forward).
            assertTrue(out.zipWithNext().all { (a, b) -> a.x < b.x }, "target=$target")
        }
    }

    /** An extreme spike survives decimation (the point of LTTB over striding). */
    @Test
    fun `spike survives decimation`() {
        val n = 500
        val x = (0 until n).map { it.toDouble() }
        val y = (0 until n).map { if (it == 250) 1000.0 else 0.0 }
        val out = Lttb.decimate(x, y, 20)
        assertTrue(out.any { it.y == 1000.0 })
    }
}
