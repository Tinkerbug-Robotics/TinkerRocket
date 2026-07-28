package com.tinkerbug.tinkerrocket.protocol

import kotlin.math.max
import kotlin.math.min

/**
 * Pure zoom/pan/re-decimation math for the flight chart — port of the
 * window logic in iOS FlightChartView.swift, extracted so the golden CSV
 * can pin it on the JVM (the Compose Canvas on top is presentation only).
 */
public object ChartWindow {

    public data class Domain(val lower: Double, val upper: Double) {
        public val span: Double get() = upper - lower
        public fun contains(x: Double): Boolean = x in lower..upper
    }

    /**
     * A chart-safe domain: never reversed (#236's inverted-range trap),
     * never zero-width, always finite.  Every domain and zoom window is
     * built through this.
     */
    public fun safeDomain(a: Double, b: Double, minWidth: Double = 1e-6): Domain {
        if (!a.isFinite() || !b.isFinite()) return Domain(0.0, 1.0)
        val lo = min(a, b)
        val hi = max(a, b)
        return if (hi - lo < minWidth) Domain(lo, lo + minWidth) else Domain(lo, hi)
    }

    /** Zoomed = visible span meaningfully smaller than full (float tolerance). */
    public fun isZoomed(full: Domain, visible: Domain): Boolean {
        if (full.span <= 0) return false
        return visible.span < full.span * 0.99
    }

    /**
     * Zoom about the window center by [scale] (>1 zooms in), clamped to
     * [full] and floored at full/1000 span (enough to see 1 ms samples) —
     * the iOS pinch math verbatim.
     */
    public fun zoomAround(start: Domain, full: Domain, scale: Double): Domain {
        if (full.span <= 0) return start
        val newSpan = max(full.span / 1000.0, min(full.span, start.span / scale))
        val center = (start.lower + start.upper) / 2.0
        var newLower = center - newSpan / 2.0
        var newUpper = center + newSpan / 2.0
        if (newLower < full.lower) {
            newLower = full.lower
            newUpper = newLower + newSpan
        }
        if (newUpper > full.upper) {
            newUpper = full.upper
            newLower = newUpper - newSpan
        }
        return safeDomain(max(newLower, full.lower), min(newUpper, full.upper))
    }

    /** Pan by a data-space offset, clamped so the window stays inside full. */
    public fun pan(start: Domain, full: Domain, dataOffset: Double): Domain {
        var newLower = start.lower + dataOffset
        var newUpper = start.upper + dataOffset
        if (newLower < full.lower) {
            newLower = full.lower
            newUpper = newLower + start.span
        }
        if (newUpper > full.upper) {
            newUpper = full.upper
            newLower = newUpper - start.span
        }
        return safeDomain(newLower, newUpper)
    }

    /**
     * Y-domain fitting the points visible in [window]: 5% vertical padding;
     * flat data gets ±1; nothing visible → 0..1.
     */
    public fun visibleYRange(series: List<List<LttbPoint>>, window: Domain): Domain {
        var minY = Double.POSITIVE_INFINITY
        var maxY = Double.NEGATIVE_INFINITY
        for (s in series) {
            for (p in s) {
                if (window.contains(p.x)) {
                    if (p.y < minY) minY = p.y
                    if (p.y > maxY) maxY = p.y
                }
            }
        }
        if (!minY.isFinite() || !maxY.isFinite()) return Domain(0.0, 1.0)
        val span = maxY - minY
        if (span < 1e-9) return safeDomain(minY - 1, maxY + 1)
        val padding = span * 0.05
        return safeDomain(minY - padding, maxY + padding)
    }

    /**
     * Progressive-LOD re-decimation for one series (iOS
     * redecimateForVisibleRange): binary-search the visible index range,
     * widen by one point each side for smooth line edges, then show RAW
     * points if ≤ [maxPoints] (true 1 kHz resolution at deep zoom) else
     * LTTB the visible subset.
     */
    public fun redecimate(
        x: List<Double>,
        y: List<Double>,
        window: Domain,
        maxPoints: Int,
    ): List<LttbPoint> {
        if (x.isEmpty()) return emptyList()
        val startIdx = lowerBound(x, window.lower)
        val endIdx = upperBound(x, window.upper)
        if (startIdx > endIdx) return emptyList()

        val safeStart = max(0, startIdx - 1)
        val safeEnd = min(x.size - 1, endIdx + 1)
        val rangeCount = safeEnd - safeStart + 1

        return if (rangeCount <= maxPoints) {
            (safeStart..safeEnd).map { LttbPoint(x[it], y[it]) }
        } else {
            Lttb.decimate(
                x.subList(safeStart, safeEnd + 1),
                y.subList(safeStart, safeEnd + 1),
                maxPoints,
            )
        }
    }

    /** First index with arr[i] >= target (sorted ascending); size if none. */
    internal fun lowerBound(arr: List<Double>, target: Double): Int {
        var lo = 0
        var hi = arr.size
        while (lo < hi) {
            val mid = (lo + hi) / 2
            if (arr[mid] < target) lo = mid + 1 else hi = mid
        }
        return lo
    }

    /** Last index with arr[i] <= target (sorted ascending); -1 if none. */
    internal fun upperBound(arr: List<Double>, target: Double): Int {
        var lo = 0
        var hi = arr.size
        while (lo < hi) {
            val mid = (lo + hi) / 2
            if (arr[mid] <= target) lo = mid + 1 else hi = mid
        }
        return lo - 1
    }
}
