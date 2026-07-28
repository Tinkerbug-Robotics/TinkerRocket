package com.tinkerbug.tinkerrocket.protocol

import com.tinkerbug.tinkerrocket.protocol.ChartWindow.Domain
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Chart window math over the GOLDEN flight (the Phase 7 exit criterion
 * "chart demo vs golden CSV"): the emitter's tiny_flight.bin → CsvGenerator
 * → CsvParser → ChartWindow, so the whole post-flight chart pipeline runs
 * against pinned data.  Plus the iOS edge-case pins (safeDomain #236,
 * zoom floor, flat-data Y range, raw-at-deep-zoom).
 */
class ChartWindowTest {

    private val golden: FlightCsvData by lazy {
        val bin = WireFixtures.bytes("csv/tiny_flight.bin")
        CsvParser.parse(CsvGenerator().writeCsv(bin).first)
    }

    private fun goldenSeries(column: String): Pair<List<Double>, List<Double>> {
        val timeMs = golden.columns.getValue("Time (ms)")
        val values = golden.columns.getValue(column)
        val x = mutableListOf<Double>()
        val y = mutableListOf<Double>()
        for (i in 0 until minOf(timeMs.size, values.size)) {
            if (timeMs[i].isFinite() && values[i].isFinite()) {
                x += timeMs[i] / 1000.0
                y += values[i]
            }
        }
        return x to y
    }

    @Test
    fun goldenFlight_fullWindow_smallDataStaysRaw() {
        val (x, y) = goldenSeries("Pressure Altitude (m)")
        assertTrue(x.isNotEmpty(), "golden CSV must have altitude rows")
        val full = ChartWindow.safeDomain(x.first(), x.last())
        val pts = ChartWindow.redecimate(x, y, full, maxPoints = 2000)
        // The golden flight is small (<2000 rows) → raw pass-through,
        // one point per row, exact values.
        assertEquals(x.size, pts.size)
        assertEquals(y.first(), pts.first().y)
        assertEquals(y.last(), pts.last().y)
    }

    @Test
    fun goldenFlight_decimationCapsPoints_andKeepsExtremes() {
        val (x, y) = goldenSeries("Pressure Altitude (m)")
        val full = ChartWindow.safeDomain(x.first(), x.last())
        val pts = ChartWindow.redecimate(x, y, full, maxPoints = 40)
        assertTrue(pts.size <= 40)
        // LTTB keeps the visually important extreme: apogee survives.
        val apogee = y.max()
        assertTrue(pts.any { it.y == apogee }, "apogee must survive decimation")
        // Endpoints always survive.
        assertEquals(x.first(), pts.first().x)
        assertEquals(x.last(), pts.last().x)
    }

    @Test
    fun goldenFlight_zoomWindow_returnsOnlyVisiblePlusEdges() {
        val (x, y) = goldenSeries("Pressure Altitude (m)")
        val mid = x[x.size / 2]
        val window = ChartWindow.safeDomain(mid, x[x.size / 2 + 5])
        val pts = ChartWindow.redecimate(x, y, window, maxPoints = 2000)
        // 6 visible rows + 1 edge point each side.
        assertEquals(8, pts.size)
        assertTrue(pts.first().x <= window.lower)
        assertTrue(pts.last().x >= window.upper)
    }

    @Test
    fun safeDomain_neverReversed_neverZeroWidth_neverNaN() {
        // #236: the inverted-ClosedRange trap.
        val d1 = ChartWindow.safeDomain(5.0, 2.0)
        assertEquals(2.0, d1.lower)
        assertEquals(5.0, d1.upper)
        val d2 = ChartWindow.safeDomain(3.0, 3.0)
        assertTrue(d2.span >= 1e-6)
        val d3 = ChartWindow.safeDomain(Double.NaN, 1.0)
        assertEquals(Domain(0.0, 1.0), d3)
        val d4 = ChartWindow.safeDomain(Double.POSITIVE_INFINITY, 0.0)
        assertEquals(Domain(0.0, 1.0), d4)
    }

    @Test
    fun zoom_floorsAtThousandthOfFull_andClampsToEdges() {
        val full = Domain(0.0, 100.0)
        // Absurd zoom-in floors at full/1000.
        val deep = ChartWindow.zoomAround(Domain(40.0, 60.0), full, scale = 1e9)
        assertEquals(0.1, deep.span, 1e-9)
        // Zoom-out from a window at the right edge clamps inside full.
        val out = ChartWindow.zoomAround(Domain(90.0, 100.0), full, scale = 0.1)
        assertTrue(out.lower >= full.lower && out.upper <= full.upper)
        assertEquals(100.0, out.span, 1e-9) // capped at full span
    }

    @Test
    fun pan_clampsAtBothEdges() {
        val full = Domain(0.0, 100.0)
        val left = ChartWindow.pan(Domain(10.0, 20.0), full, dataOffset = -50.0)
        assertEquals(Domain(0.0, 10.0), left)
        val right = ChartWindow.pan(Domain(80.0, 90.0), full, dataOffset = 50.0)
        assertEquals(Domain(90.0, 100.0), right)
    }

    @Test
    fun visibleY_padsFivePercent_flatGetsPlusMinusOne_emptyGetsUnit() {
        val series = listOf(listOf(LttbPoint(0.0, 10.0), LttbPoint(1.0, 20.0)))
        val y = ChartWindow.visibleYRange(series, Domain(0.0, 1.0))
        assertEquals(10.0 - 0.5, y.lower, 1e-9)
        assertEquals(20.0 + 0.5, y.upper, 1e-9)

        val flat = listOf(listOf(LttbPoint(0.0, 7.0), LttbPoint(1.0, 7.0)))
        val yf = ChartWindow.visibleYRange(flat, Domain(0.0, 1.0))
        assertEquals(6.0, yf.lower, 1e-9)
        assertEquals(8.0, yf.upper, 1e-9)

        val none = ChartWindow.visibleYRange(series, Domain(5.0, 6.0))
        assertEquals(Domain(0.0, 1.0), none)
    }

    @Test
    fun isZoomed_toleratesFloatNoise() {
        val full = Domain(0.0, 100.0)
        assertTrue(!ChartWindow.isZoomed(full, Domain(0.0, 99.5)))
        assertTrue(ChartWindow.isZoomed(full, Domain(0.0, 50.0)))
    }
}
