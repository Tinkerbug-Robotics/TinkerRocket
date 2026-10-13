package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.Klobuchar
import com.tinkerbug.tinkerrocket.protocol.PadCalState
import com.tinkerbug.tinkerrocket.protocol.PadCalStatus
import com.tinkerbug.tinkerrocket.protocol.PadIonoSource
import com.tinkerbug.tinkerrocket.protocol.formatFixed
import java.time.Instant
import java.time.ZoneId
import java.time.format.DateTimeFormatter
import java.util.Locale

/**
 * What the pad calibration screen says (#1600 phase 5): the flight computer's
 * 0xD0 status and the ionosphere the phone holds, as text.  Kept out of the
 * composable so the wording can be tested.
 */
public object PadCalReadout {

    public enum class Tone { NORMAL, OK, WARN }

    public data class Line(val text: String, val tone: Tone = Tone.NORMAL)

    private val DATE: DateTimeFormatter = DateTimeFormatter.ofPattern("d MMM yyyy", Locale.US)
    private val STAMP: DateTimeFormatter = DateTimeFormatter.ofPattern("d MMM HH:mm", Locale.US)

    /** A navigation file's date, from its GPS day. */
    public fun date(gpsDay: Int): String = Klobuchar.date(gpsDay).format(DATE)

    /** The gyro/accel window, from cal_state. */
    public fun gyroAccel(s: PadCalStatus): Line = when (s.calState) {
        PadCalState.IDLE -> Line("Gyro & accel: no pad calibration since the rocket powered up")
        PadCalState.RUNNING ->
            Line("Gyro & accel: calibrating, ${s.calProgress} % — keep the rocket still")
        PadCalState.DONE -> Line(
            "Gyro & accel calibrated" + (gravity(s.calGravityMg, 3)?.let { " ($it)" } ?: ""),
            Tone.OK,
        )
        PadCalState.REJECTED -> Line(
            "Rejected: " + (gravity(s.calGravityMg, 2)?.let { "gravity $it — " } ?: "") +
                "keep the rocket still and try again; the previous calibration is kept",
            Tone.WARN,
        )
        PadCalState.REFUSED ->
            Line("Not run: the rocket is not on the pad (or a sim is running)", Tone.WARN)
    }

    /** The GNSS survey, line by line; none on a board without the raw-GNSS filter. */
    public fun gnss(s: PadCalStatus): List<Line> {
        if (!s.rawGnss) return emptyList()
        val out = ArrayList<Line>()
        if (s.padMismatch) {
            out += Line(
                "Receiver's fix disagrees with its raw measurements — pad corrections off",
                Tone.WARN,
            )
        }
        out += Line("Satellites tracked: ${s.satsTracked}")
        if (s.satsNoEph > 0) out += Line("Waiting for ephemeris: ${s.satsNoEph}")
        out += Line("Settling: ${s.satsSettling}")
        out += Line(
            "Pad corrections: ${s.satsCorrected}" +
                if (s.satsCorrected > 0) " (spread ${metres(s.corrRmsCm)})" else "",
        )
        out += Line("Ionosphere: " + ionosphere(s))
        out += Line(
            if (s.padAverage) {
                "Pad position: ${s.padFixes} fixes over ${minSec(s.padSecs)}, " +
                    "scatter ${metres(s.padScatterCm)}" + if (s.padFrozen) ", frozen at launch" else ""
            } else {
                "Pad position: no fixes yet"
            },
        )
        out += Line(
            if (s.filterRunning) {
                "Raw-GNSS filter: running" +
                    if (s.posSigmaCm > 0) ", position sigma ${metres(s.posSigmaCm)}" else ""
            } else {
                "Raw-GNSS filter: not running yet"
            },
        )
        return out
    }

    /** What the raw-GNSS filter takes the ionosphere out with. */
    public fun ionosphere(s: PadCalStatus): String = when (s.ionoSource) {
        PadIonoSource.BROADCAST -> "broadcast set" +
            (if (s.ionoGpsDay != 0) " of ${date(s.ionoGpsDay)}" else "") +
            if (s.ionoAgeKnown) ", ${age(s.ionoAgeDays)}" else ""
        PadIonoSource.PAD_SURVEY -> "pad survey (backup)"
        PadIonoSource.NONE -> "none yet"
    }

    /** The set the next pad calibration sends from this phone. */
    public fun phoneIonosphere(held: IonosphereCache.Entry?, zone: ZoneId): String =
        if (held == null) {
            "Ionosphere on this phone: none — the rocket will use its pad survey"
        } else {
            "Ionosphere on this phone: set of ${date(held.klobuchar.gpsDay)} " +
                "(fetched ${STAMP.format(Instant.ofEpochMilli(held.fetchedAtMs).atZone(zone))})"
        }

    /** milli-g as g; null for 0, which is "none measured". */
    private fun gravity(mg: Int, decimals: Int): String? =
        if (mg <= 0) null else formatFixed(mg / 1000.0, decimals) + " g"

    private fun metres(cm: Int): String = formatFixed(cm / 100.0, 1) + " m"

    private fun minSec(secs: Int): String = String.format(Locale.ROOT, "%d:%02d", secs / 60, secs % 60)

    /**
     * By the receiver's date, which can sit a day behind the phone's around
     * midnight UTC: a set dated "ahead" of it is as fresh as one can be.
     */
    private fun age(days: Int): String = when {
        days <= 0 -> "0 days old"
        days == 1 -> "1 day old"
        else -> "$days days old"
    }
}
