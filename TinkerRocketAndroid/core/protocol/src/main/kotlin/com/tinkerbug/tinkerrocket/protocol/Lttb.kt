package com.tinkerbug.tinkerrocket.protocol

import kotlin.math.abs

// Port of the LTTB downsampler in TinkerRocketApp/Models/CSVParser.swift
// (`CSVParser.lttbDecimate`).  The bucket-boundary arithmetic (truncating
// Int conversions, the +1/min clamps) is behavior we chart against on iOS —
// keep it token-for-token, do not "clean up" the rounding.

/** One decimated sample. Mirrors the Swift `(x: Double, y: Double)` tuple. */
public data class LttbPoint(
    public val x: Double,
    public val y: Double,
)

public object Lttb {

    /**
     * Largest Triangle Three Buckets downsampling.
     * Preserves visually important peaks and troughs (e.g., apogee, max-g)
     * that naive stride-based decimation would miss.
     *
     * Passthrough (zip of x/y, truncated to the shorter, exactly like Swift
     * `zip`) when `x.size <= targetCount` or `targetCount < 3`; otherwise the
     * result has exactly [targetCount] points and always keeps the first and
     * last input points.
     */
    public fun decimate(
        x: List<Double>,
        y: List<Double>,
        targetCount: Int,
    ): List<LttbPoint> {
        val count = x.size
        if (count <= targetCount || targetCount < 3) {
            // No decimation needed
            return x.zip(y) { xv, yv -> LttbPoint(xv, yv) }
        }

        val result = ArrayList<LttbPoint>(targetCount)

        // Always include first point
        result.add(LttbPoint(x[0], y[0]))

        val bucketSize = (count - 2).toDouble() / (targetCount - 2).toDouble()
        var prevIndex = 0

        for (i in 1 until (targetCount - 1)) {
            // Current bucket range (Swift Int(Double) truncates toward zero,
            // as does Kotlin toInt())
            val bucketStart = ((i - 1).toDouble() * bucketSize).toInt() + 1
            val bucketEnd = minOf((i.toDouble() * bucketSize).toInt(), count - 2)

            // Next bucket range (for computing average)
            val nextBucketStart = (i.toDouble() * bucketSize).toInt() + 1
            val nextBucketEnd = minOf(((i + 1).toDouble() * bucketSize).toInt() + 1, count)

            // Average of next bucket
            var avgX = 0.0
            var avgY = 0.0
            var nextCount = 0
            for (j in nextBucketStart until nextBucketEnd) {
                avgX += x[j]; avgY += y[j]; nextCount += 1
            }
            if (nextCount > 0) {
                avgX /= nextCount.toDouble(); avgY /= nextCount.toDouble()
            }

            // Find point in current bucket that makes largest triangle
            // with the previous selected point and the next bucket average
            var maxArea = -1.0
            var bestIndex = bucketStart
            val px = x[prevIndex]
            val py = y[prevIndex]

            for (j in bucketStart..bucketEnd) {
                val area = abs((px - avgX) * (y[j] - py) - (px - x[j]) * (avgY - py))
                if (area > maxArea) {
                    maxArea = area
                    bestIndex = j
                }
            }

            result.add(LttbPoint(x[bestIndex], y[bestIndex]))
            prevIndex = bestIndex
        }

        // Always include last point
        result.add(LttbPoint(x[count - 1], y[count - 1]))

        return result
    }
}
