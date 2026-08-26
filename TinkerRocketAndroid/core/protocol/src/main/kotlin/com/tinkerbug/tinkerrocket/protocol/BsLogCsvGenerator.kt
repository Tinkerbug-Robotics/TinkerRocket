package com.tinkerbug.tinkerrocket.protocol

import java.math.BigDecimal
import java.math.RoundingMode
import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.withSign

/**
 * Base-station binary log -> CSV (#850).
 *
 * The base station used to write this CSV itself, one row per packet, formatting
 * every float on the MCU.  Since #850 it logs the bytes it received — framed
 * exactly like the rocket computer's log — and the CSV is generated here.
 *
 * THE FORWARD-FILL is the load-bearing part.  A log holds two interleaved frame
 * types: a 55-byte FAST frame five slots out of six and a 22-byte SLOW frame on
 * the sixth.  Each carries a subset, so a row is built by merging the frame into
 * a running accumulator per rocket, exactly as the base station does in RAM.
 * That is what keeps the rows rectangular; without it a slow frame would blank
 * the position and a fast frame would blank the battery, and the CSV would
 * alternate between half-empty rows.
 *
 * Ports of [ecefToGeodetic] and [eulerFromQuat] are deliberately literal
 * transcriptions of `TR_Coordinates::ecefToGeodetic` and
 * `SensorConverter::eulerFromQuat` rather than tidier equivalents, so the
 * Python, Kotlin and Swift renderings cannot drift in the last decimal.  All
 * three are pinned against the same golden, which is packed by the REAL
 * firmware packers (tests_cpp/fixtures/wire/csv/bs_tiny.bin).
 *
 * iOS twin: `BSLogCSVGenerator.swift`.
 */
public class BsLogCsvGenerator {

    public class NotABaseStationLogException :
        Exception("not a base-station binary log (missing TRBSLOG magic)")

    public class UnsupportedVersionException(version: Int) :
        Exception("base-station log format v$version, this build speaks v$FORMAT_VERSION")

    public companion object {
        internal val MAGIC = "TRBSLOG".toByteArray(Charsets.US_ASCII)
        internal const val FORMAT_VERSION = 1

        internal const val BS_LORA_RX_MSG = 0xFC
        internal const val BS_EVENT_MSG = 0xFD

        internal const val SIZE_OF_LORA_FAST = 55
        internal const val SIZE_OF_LORA_SLOW = 22
        internal const val LORA_PROTO_VERSION = 5
        internal const val LORA_FRAME_SLOW = 0x1

        internal const val LORA_HDR_LEN = 7
        internal const val BS_RX_HDR_LEN = 12
        internal const val BS_EVT_HDR_LEN = 9

        /** INT16_MIN: the radio reported no reading.  Distinct from 0 dBm. */
        internal const val RSSI_UNKNOWN = -32768

        private val PREAMBLE = byteArrayOf(0xAA.toByte(), 0x55, 0xAA.toByte(), 0x55)

        private const val WGS84_A = 6378137.0
        private const val WGS84_F = 1.0 / 298.257223563
        private const val WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F

        private val STATE_NAMES = arrayOf(
            "INIT", "READY", "PRELAUNCH", "INFLIGHT", "LANDED", "MAG_CAL", "6", "7",
        )

        /**
         * The firmware's 39 columns verbatim, with `frame` appended.  Appending
         * is this repo's convention for column additions (it is how rocket_id
         * and cam_a/servo_a arrived), so anything keying columns by name is
         * unaffected while the fast/slow information that only exists post-#850
         * is not thrown away.
         */
        public val COLUMNS: List<String> = listOf(
            "time_ms", "state", "num_sats", "pdop", "lat", "lon", "alt_m", "h_acc",
            "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z",
            "pressure_alt", "alt_rate", "max_alt", "max_speed",
            "voltage", "current", "soc", "cam_a", "servo_a",
            "roll", "pitch", "yaw", "speed",
            "launch", "vel_apo", "alt_apo", "landed", "rssi", "snr",
            "next_ch", "rx_freq_mhz", "seq", "gap", "event", "rocket_id", "frame",
        )

        /** True when [bytes] opens with the base-station log magic. */
        public fun isBaseStationLog(bytes: ByteArray): Boolean =
            bytes.size >= MAGIC.size && MAGIC.indices.all { bytes[it] == MAGIC[it] }

        /**
         * printf("%.Nf") semantics.
         *
         * Kotlin's String.format uses HALF_UP; C and Swift round the EXACT
         * binary value ties-to-even, so BigDecimal(value) (the exact binary
         * expansion) + HALF_EVEN reproduces printf.  printf also keeps the sign
         * on a negative that rounds to zero (-0.0 -> "-0.0"), which BigDecimal
         * drops — restored explicitly.  Same policy as CsvGenerator; kept in
         * both rather than shared because they are independent formats.
         */
        internal fun fmt(value: Double, decimals: Int): String {
            if (value.isNaN()) return "nan"
            val s = BigDecimal(value).setScale(decimals, RoundingMode.HALF_EVEN).toPlainString()
            val negZero = (value < 0.0 || (value == 0.0 && 1.0 / value < 0)) && !s.startsWith("-")
            return if (negZero) "-$s" else s
        }
    }

    // ---- decoded record ---------------------------------------------------

    /** One received packet, forward-filled to a complete picture. */
    internal class Row {
        var timeMs: Long = 0
        var rssi: Double = Double.NaN
        var snr: Double = Double.NaN
        var rxFreqMhz: Double = 0.0
        var frame: String = ""

        // header (both frames)
        var networkId = 0
        var rocketId = 0
        var nextCh = 255
        var seq = 0
        var state = ""
        var launch = 0
        var velApo = 0
        var altApo = 0
        var landed = 0

        // fast frame
        var numSats = 0
        var pdop = 0.0
        var hAcc = 0.0
        var ecefX = 0.0
        var ecefY = 0.0
        var ecefZ = 0.0
        var accX = 0.0; var accY = 0.0; var accZ = 0.0
        var gyroX = 0.0; var gyroY = 0.0; var gyroZ = 0.0
        var q0 = 0.0; var q1 = 0.0; var q2 = 0.0; var q3 = 0.0
        var pressureAlt = 0.0
        var altRate = 0.0
        var speed = 0.0
        var hasQuat = false

        // slow frame
        var maxAlt = 0.0
        var maxSpeed = 0.0
        var voltage = 0.0
        var current = 0.0
        var soc = 0.0
        var camA = 0.0
        var servoA = 0.0

        var gap = -1

        fun copy(): Row {
            val r = Row()
            r.networkId = networkId; r.rocketId = rocketId; r.nextCh = nextCh; r.seq = seq
            r.state = state; r.launch = launch; r.velApo = velApo
            r.altApo = altApo; r.landed = landed
            r.numSats = numSats; r.pdop = pdop; r.hAcc = hAcc
            r.ecefX = ecefX; r.ecefY = ecefY; r.ecefZ = ecefZ
            r.accX = accX; r.accY = accY; r.accZ = accZ
            r.gyroX = gyroX; r.gyroY = gyroY; r.gyroZ = gyroZ
            r.q0 = q0; r.q1 = q1; r.q2 = q2; r.q3 = q3; r.hasQuat = hasQuat
            r.pressureAlt = pressureAlt; r.altRate = altRate; r.speed = speed
            r.maxAlt = maxAlt; r.maxSpeed = maxSpeed
            r.voltage = voltage; r.current = current; r.soc = soc
            r.camA = camA; r.servoA = servoA
            return r
        }
    }

    internal class Event(val timeMs: Long, val rxFreqMhz: Double, val text: String)

    // ---- little-endian readers -------------------------------------------

    private fun u8(b: ByteArray, o: Int) = b[o].toInt() and 0xFF
    private fun i8(b: ByteArray, o: Int) = b[o].toInt()
    private fun u16(b: ByteArray, o: Int) = u8(b, o) or (u8(b, o + 1) shl 8)
    private fun i16(b: ByteArray, o: Int) = u16(b, o).toShort().toInt()
    private fun u32(b: ByteArray, o: Int): Long =
        (u8(b, o).toLong()) or (u8(b, o + 1).toLong() shl 8) or
            (u8(b, o + 2).toLong() shl 16) or (u8(b, o + 3).toLong() shl 24)

    /** Signed little-endian 24-bit, matching i24le_t on the wire. */
    private fun i24(b: ByteArray, o: Int): Int {
        val v = u8(b, o) or (u8(b, o + 1) shl 8) or (u8(b, o + 2) shl 16)
        return if (v and 0x800000 != 0) v - 0x1000000 else v
    }

    private fun decodeVoltage(u: Int) = 2.0 + (u / 255.0) * 8.0

    // ---- frame decode -----------------------------------------------------

    /** Returns ver_type; writes the fields BOTH frames carry. */
    private fun unpackHeader(f: ByteArray, r: Row): Int {
        r.networkId = u8(f, 0)
        r.rocketId = u8(f, 1)
        r.nextCh = u8(f, 2)
        r.seq = u16(f, 3)
        val verType = u8(f, 5)
        val flags = u8(f, 6)
        r.launch = if (flags and 0x01 != 0) 1 else 0
        r.velApo = if (flags and 0x02 != 0) 1 else 0
        r.altApo = if (flags and 0x04 != 0) 1 else 0
        r.landed = if (flags and 0x08 != 0) 1 else 0
        r.state = STATE_NAMES[(flags shr 4) and 0x07]
        return verType
    }

    /** Writes ONLY what the fast frame carries — never clears the rest. */
    private fun unpackFast(f: ByteArray, r: Row) {
        val o = LORA_HDR_LEN
        val sats = u8(f, o)
        r.numSats = sats and 0x3F
        r.pdop = u8(f, o + 1).toDouble()
        r.hAcc = u8(f, o + 2).toDouble()
        r.ecefX = i24(f, o + 3).toDouble()
        r.ecefY = i24(f, o + 6).toDouble()
        r.ecefZ = i24(f, o + 9).toDouble()
        r.accX = i16(f, o + 12) / 10.0
        r.accY = i16(f, o + 14) / 10.0
        r.accZ = i16(f, o + 16) / 10.0
        r.gyroX = i16(f, o + 18) / 10.0
        r.gyroY = i16(f, o + 20) / 10.0
        r.gyroZ = i16(f, o + 22) / 10.0
        r.q0 = i16(f, o + 24) / 10000.0
        r.q1 = i16(f, o + 26) / 10000.0
        r.q2 = i16(f, o + 28) / 10000.0
        r.q3 = i16(f, o + 30) / 10000.0
        r.hasQuat = true
        r.pressureAlt = i24(f, o + 32).toDouble()
        r.altRate = i16(f, o + 35).toDouble()
        val ve = i16(f, o + 37) / 10.0
        val vn = i16(f, o + 39) / 10.0
        val vu = i16(f, o + 41) / 10.0
        // Derived on the ground since #191; both inputs ride THIS frame, so the
        // derivation is self-consistent rather than mixing fresh with stale.
        r.speed = sqrt(ve * ve + vn * vn + vu * vu)
    }

    /** Writes ONLY what the slow frame carries. */
    private fun unpackSlow(f: ByteArray, r: Row) {
        val o = LORA_HDR_LEN
        r.maxAlt = i24(f, o).toDouble()
        r.maxSpeed = i16(f, o + 3).toDouble()
        r.voltage = decodeVoltage(u8(f, o + 7))
        r.current = i16(f, o + 8).toDouble()
        r.soc = i8(f, o + 10).toDouble()
        r.camA = u16(f, o + 11) / 1000.0
        r.servoA = u16(f, o + 13) / 1000.0
    }

    // ---- geodesy / attitude ----------------------------------------------

    internal fun ecefToGeodetic(x: Double, y: Double, z: Double): Triple<Double, Double, Double> {
        val lon = atan2(y, x)
        val p = sqrt(x * x + y * y)
        var lat = atan2(z, p * (1 - WGS84_E2))
        var alt = 0.0
        while (true) {
            val prev = lat
            val n = WGS84_A / sqrt(1 - WGS84_E2 * sin(lat) * sin(lat))
            alt = p / cos(lat) - n
            lat = atan2(z + n * WGS84_E2 * sin(lat), p)
            if (abs(lat - prev) <= 1e-10) break
        }
        return Triple(Math.toDegrees(lat), Math.toDegrees(lon), alt)
    }

    internal fun eulerFromQuat(qw: Double, qx: Double, qy: Double, qz: Double):
        Triple<Double, Double, Double> {
        // Roll — azimuth of body Z in the NED horizontal plane (gimbal-lock-free)
        val zN = 2.0 * (qx * qz + qw * qy)
        val zE = 2.0 * (qy * qz - qw * qx)
        val roll = -Math.toDegrees(atan2(zE, zN))
        val sinp = 2.0 * (qw * qy - qz * qx)
        val pitch = if (abs(sinp) >= 1.0) 90.0.withSign(sinp) else Math.toDegrees(asin(sinp))
        val yaw = Math.toDegrees(
            atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)),
        )
        return Triple(roll, pitch, yaw)
    }

    // ---- parse ------------------------------------------------------------

    private fun indexOfPreamble(b: ByteArray, from: Int): Int {
        var i = from
        while (i + 4 <= b.size) {
            if (b[i] == PREAMBLE[0] && b[i + 1] == PREAMBLE[1] &&
                b[i + 2] == PREAMBLE[2] && b[i + 3] == PREAMBLE[3]
            ) {
                return i
            }
            i++
        }
        return -1
    }

    internal fun parse(bytes: ByteArray): Pair<List<Row>, List<Event>> {
        if (!isBaseStationLog(bytes)) throw NotABaseStationLogException()
        val version = if (bytes.size > MAGIC.size) u8(bytes, MAGIC.size) else 0
        if (version != FORMAT_VERSION) throw UnsupportedVersionException(version)

        val rows = ArrayList<Row>()
        val events = ArrayList<Event>()
        val accum = HashMap<Int, Row>()
        val lastSeq = HashMap<Int, Int>()

        var i = 0
        while (true) {
            val p = indexOfPreamble(bytes, i)
            if (p < 0) break
            var j = p + 4
            if (j + 3 >= bytes.size) break
            val mtype = u8(bytes, j)
            val len = u8(bytes, j + 1)
            j += 2
            if (j + len + 2 > bytes.size) break
            val payload = bytes.copyOfRange(j, j + len)
            i = j + len + 2

            if (mtype == BS_EVENT_MSG) {
                if (payload.size < BS_EVT_HDR_LEN) continue
                val tMs = u32(payload, 0)
                val freq = u32(payload, 4)
                val tlen = u8(payload, 8)
                val end = minOf(BS_EVT_HDR_LEN + tlen, payload.size)
                val text = String(payload, BS_EVT_HDR_LEN, end - BS_EVT_HDR_LEN, Charsets.UTF_8)
                events.add(Event(tMs, freq / 1e6, text))
                continue
            }
            if (mtype != BS_LORA_RX_MSG || payload.size <= BS_RX_HDR_LEN) continue

            val tMs = u32(payload, 0)
            val rssiX10 = i16(payload, 4)
            val snrX10 = i16(payload, 6)
            val freqHz = u32(payload, 8)
            val frame = payload.copyOfRange(BS_RX_HDR_LEN, payload.size)
            if (frame.size != SIZE_OF_LORA_FAST && frame.size != SIZE_OF_LORA_SLOW) continue

            val probe = Row()
            val verType = unpackHeader(frame, probe)
            if ((verType shr 4) != LORA_PROTO_VERSION) continue
            val ftype = verType and 0x0F

            // Forward-fill: start from this rocket's last known state and let
            // the frame overwrite only what it actually contains.
            val row = accum[probe.rocketId]?.copy() ?: Row()
            unpackHeader(frame, row)
            if (ftype == LORA_FRAME_SLOW) unpackSlow(frame, row) else unpackFast(frame, row)

            row.timeMs = tMs
            row.rssi = if (rssiX10 == RSSI_UNKNOWN) Double.NaN else rssiX10 / 10.0
            row.snr = if (snrX10 == RSSI_UNKNOWN) Double.NaN else snrX10 / 10.0
            row.rxFreqMhz = freqHz / 1e6
            row.frame = if (ftype == LORA_FRAME_SLOW) "slow" else "fast"

            val prev = lastSeq[row.rocketId]
            row.gap = if (prev == null) -1 else ((row.seq - prev - 1) and 0xFFFF)
            lastSeq[row.rocketId] = row.seq

            accum[row.rocketId] = row
            rows.add(row)
        }
        return Pair(rows, events)
    }

    // ---- render -----------------------------------------------------------

    /** Binary base-station log -> CSV text. */
    public fun writeCsv(bytes: ByteArray): String {
        val (rows, events) = parse(bytes)
        val sb = StringBuilder()
        sb.append(COLUMNS.joinToString(",")).append('\n')

        // Merge events in by time so one pass sees telemetry and events in
        // arrival order — the property the firmware's padded EVENT rows gave.
        data class Item(val timeMs: Long, val row: Row?, val ev: Event?)
        val merged = ArrayList<Item>(rows.size + events.size)
        rows.forEach { merged.add(Item(it.timeMs, it, null)) }
        events.forEach { merged.add(Item(it.timeMs, null, it)) }
        merged.sortBy { it.timeMs }

        for (item in merged) {
            val ev = item.ev
            if (ev != null) {
                val cells = MutableList(COLUMNS.size) { "" }
                cells[0] = ev.timeMs.toString()
                cells[1] = "EVENT"
                cells[COLUMNS.indexOf("rx_freq_mhz")] = fmt(ev.rxFreqMhz, 3)
                cells[COLUMNS.indexOf("event")] = ev.text
                sb.append(cells.joinToString(",")).append('\n')
                continue
            }
            val r = item.row!!

            // lat/lon only where the rocket claims a fix — nonzero ECEF with
            // num_sats == 0 is a stale register read (#95) and would render a
            // valid-looking position for an invalid fix.
            var lat = Double.NaN; var lon = Double.NaN; var alt = Double.NaN
            if (r.numSats > 0 && (r.ecefX != 0.0 || r.ecefY != 0.0 || r.ecefZ != 0.0)) {
                val g = ecefToGeodetic(r.ecefX, r.ecefY, r.ecefZ)
                lat = g.first; lon = g.second; alt = g.third
            }
            var roll = 0.0; var pitch = 0.0; var yaw = 0.0
            if (r.hasQuat) {
                val e = eulerFromQuat(r.q0, r.q1, r.q2, r.q3)
                roll = e.first; pitch = e.second; yaw = e.third
            }

            sb.append(
                listOf(
                    r.timeMs.toString(), r.state, r.numSats.toString(), fmt(r.pdop, 1),
                    fmt(lat, 7), fmt(lon, 7), fmt(alt, 1), fmt(r.hAcc, 1),
                    fmt(r.accX, 2), fmt(r.accY, 2), fmt(r.accZ, 2),
                    fmt(r.gyroX, 1), fmt(r.gyroY, 1), fmt(r.gyroZ, 1),
                    fmt(r.pressureAlt, 1), fmt(r.altRate, 1),
                    fmt(r.maxAlt, 1), fmt(r.maxSpeed, 1),
                    fmt(r.voltage, 2), fmt(r.current, 0), fmt(r.soc, 1),
                    fmt(r.camA, 3), fmt(r.servoA, 3),
                    fmt(roll, 1), fmt(pitch, 1), fmt(yaw, 1), fmt(r.speed, 1),
                    r.launch.toString(), r.velApo.toString(),
                    r.altApo.toString(), r.landed.toString(),
                    fmt(r.rssi, 0), fmt(r.snr, 1),
                    r.nextCh.toString(), fmt(r.rxFreqMhz, 3),
                    r.seq.toString(), r.gap.toString(),
                    "", r.rocketId.toString(), r.frame,
                ).joinToString(","),
            ).append('\n')
        }
        return sb.toString()
    }
}
