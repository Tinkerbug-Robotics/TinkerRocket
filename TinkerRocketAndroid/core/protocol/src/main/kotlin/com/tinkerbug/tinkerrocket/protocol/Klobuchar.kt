package com.tinkerbug.tinkerrocket.protocol

import java.time.LocalDate
import java.time.temporal.ChronoUnit
import java.util.Locale
import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * The GPS broadcast ionosphere (Klobuchar) parameters a pad calibration
 * carries to the flight computer (#1600 phase 5). Port of iOS
 * `Klobuchar.swift` — keep the two in step.
 *
 * The raw-GNSS filter takes the ionosphere out of each pseudorange with the
 * eight numbers GPS broadcasts. The LC86G does not pass them on (RTCM has no
 * message for them), so the app reads them from the day's broadcast
 * navigation file — NOAA's CORS archive, `brdcDDD0.YYn.gz`, public and
 * without a login, updated through the day — and keeps the last set it read,
 * so a launch site with no signal still has one.
 *
 * On the wire they go as GPS broadcasts them: eight signed bytes, alpha0..3
 * then beta0..3, at the scales of IS-GPS-200 20.3.3.5.1.7, then the file's
 * date as a GPS day (days since 1980-01-06), u16 little-endian. The flight
 * computer judges their age against the receiver's own date.
 */
public data class Klobuchar(
    /** alpha0..3, beta0..3 in broadcast units, each -128..127. */
    val raw: List<Int>,
    /** The navigation file's date: days since 1980-01-06. */
    val gpsDay: Int,
) {
    init {
        require(raw.size == 8 && raw.all { it in -128..127 }) { "eight signed bytes" }
        require(gpsDay in 1..0xFFFF) { "a GPS day" }
    }

    /** alpha0..3 (s, s/semicircle, ...) then beta0..3 (s, s/semicircle, ...). */
    public fun values(): List<Double> = raw.mapIndexed { k, b -> b * SCALES[k] }

    /** `[raw i8 x8][gps_day u16 LE]`. */
    public fun payload(): ByteArray {
        val out = ByteArray(PAYLOAD_LENGTH)
        for (k in 0 until 8) out[k] = raw[k].toByte()
        out[8] = (gpsDay and 0xFF).toByte()
        out[9] = ((gpsDay shr 8) and 0xFF).toByte()
        return out
    }

    public companion object {
        public const val PAYLOAD_LENGTH: Int = 10

        /** 2^-30, 2^-27, 2^-24, 2^-24, 2^11, 2^14, 2^16, 2^16. */
        public val SCALES: DoubleArray = doubleArrayOf(
            9.313225746154785e-10, 7.450580596923828e-09, 5.960464477539063e-08, 5.960464477539063e-08,
            2048.0, 16384.0, 65536.0, 65536.0,
        )

        private val GPS_EPOCH: LocalDate = LocalDate.of(1980, 1, 6)

        /** Days since 1980-01-06 of a UTC calendar date. */
        public fun gpsDay(utcDate: LocalDate): Int = ChronoUnit.DAYS.between(GPS_EPOCH, utcDate).toInt()

        /** The calendar date of a GPS day. */
        public fun date(gpsDay: Int): LocalDate = GPS_EPOCH.plusDays(gpsDay.toLong())

        /** NOAA CORS's GPS broadcast navigation file for a UTC date. */
        public fun noaaUrl(utcDate: LocalDate): String {
            val doy = utcDate.dayOfYear
            val yy = utcDate.year % 100
            return String.format(
                Locale.ROOT,
                "https://geodesy.noaa.gov/corsdata/rinex/%04d/%03d/brdc%03d0.%02dn.gz",
                utcDate.year, doy, doy, yy,
            )
        }

        /**
         * The parameters from a navigation file's header: RINEX 2's
         * `ION ALPHA` / `ION BETA`, or RINEX 3's `GPSA` / `GPSB` lines of
         * `IONOSPHERIC CORR`. Null without both, or if a value is not a whole
         * number of its broadcast unit (the file prints five digits, so a
         * genuine one is within a few thousandths of one).
         */
        public fun fromRinexHeader(text: String, gpsDay: Int): Klobuchar? {
            var alpha: List<Double>? = null
            var beta: List<Double>? = null
            for (line in text.lineSequence()) {
                if (line.length < 61) continue
                val label = line.substring(60).trim()
                when {
                    label == "END OF HEADER" -> break
                    label == "ION ALPHA" -> alpha = fields(line, 2)
                    label == "ION BETA" -> beta = fields(line, 2)
                    label == "IONOSPHERIC CORR" && line.startsWith("GPSA") -> alpha = fields(line, 5)
                    label == "IONOSPHERIC CORR" && line.startsWith("GPSB") -> beta = fields(line, 5)
                }
            }
            val a = alpha ?: return null
            val b = beta ?: return null
            val values = a + b
            val raw = ArrayList<Int>(8)
            for (k in 0 until 8) {
                val x = values[k] / SCALES[k]
                val r = x.roundToInt()
                if (abs(x - r) > 0.05 || r !in -128..127) return null
                raw.add(r)
            }
            if (gpsDay !in 1..0xFFFF) return null
            return Klobuchar(raw, gpsDay)
        }

        /** Four D12.4 fields from column [start]; null if any is unreadable. */
        private fun fields(line: String, start: Int): List<Double>? {
            val out = ArrayList<Double>(4)
            for (k in 0 until 4) {
                val from = start + 12 * k
                val to = from + 12
                if (to > line.length) return null
                val v = line.substring(from, to).trim().replace('D', 'E').replace('d', 'E')
                    .toDoubleOrNull() ?: return null
                if (!v.isFinite()) return null      // "NaN" parses, and would throw at roundToInt
                out.add(v)
            }
            return out
        }
    }
}
