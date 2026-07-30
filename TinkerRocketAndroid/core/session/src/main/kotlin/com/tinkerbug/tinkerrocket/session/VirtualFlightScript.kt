package com.tinkerbug.tinkerrocket.session

import java.util.Locale
import kotlin.math.max
import kotlin.math.min

/**
 * The Virtual Rocket's canned flight, one JSON frame per tick — pure so the
 * announcer regression test can replay the exact frames the demo emits
 * (`VirtualFlightAnnouncerTest`).  iOS twin: `VirtualRocketDriver.flightFrameJSON`.
 *
 * The script speaks fluent firmware, because every consumer in the app keys
 * off real conventions (voice-callout outage, 2026-07-30):
 *  - The countdown phase is **PRELAUNCH** — the state a real FC arms through,
 *    and the announcer's reset edge.  Without it the one-shot callout flags
 *    stay latched from the previous flight and the second demo flight is mute.
 *  - Descent is **INFLIGHT** + the apogee flag — there is no "DESCENT" state
 *    in firmware, and both announcers gate descent callouts on INFLIGHT.
 *  - **mspd** ramps to a stable max — burnout detection watches max-speed
 *    stability, and ascent altitude callouts are gated behind burnout.
 *  - The rocket **drifts east under canopy** so the landed callout has a real
 *    horizontal distance (and the bearing arrow moves).
 *
 * Full callout sequence per flight: burnout ~15 s, altitude cadence to
 * apogee at 22 s (400 m), descent callouts at 27 s and 37 s, landed at 38 s.
 */
public object VirtualFlightScript {

    /** Frames per flight (42 s at 5 Hz: 12 s countdown, 10 s up, 16 s down). */
    public const val TICKS: Int = 210

    /** Telemetry cadence during the flight. */
    public const val TICK_MS: Long = 200

    /** Launch pad position (also the idle frames' fix). */
    public const val PAD_LAT: Double = 34.6572
    public const val PAD_LON: Double = -118.2015

    /** OK (01) at baro/imu/ekf/mag/gnss/batt/storage shifts; pyro NA. */
    public const val HEALTH: Int = 0x100555

    /** The focused rocket's frame for `tick` (0 until [TICKS]). */
    public fun frameJson(tick: Int): String {
        val t = tick / 5.0f    // seconds into the flight
        val phase = when {
            t < 12f -> "PRELAUNCH"
            t < 38f -> "INFLIGHT"
            else -> "LANDED"
        }
        val alt = when {
            t < 12f -> 0f
            t < 22f -> {
                val tt = t - 12f
                max(0f, 80f * tt - 4f * tt * tt)   // apogee 400 m at t=22
            }
            t < 38f -> max(0f, 400f - (t - 22f) * 25f)
            else -> 0f
        }
        // Running max the same way the FC reports it.
        val maxAlt = when {
            t < 12f -> 0f
            t < 22f -> alt
            else -> 400f
        }
        val maxSpeed = if (t < 12f) 0f else min(95f, (t - 12f) * 38f)
        var fs = 0x10 or 0x40                      // pwr on + logging
        if (t >= 12f) fs = fs or 0x01              // launch
        if (t >= 14.5f) fs = fs or 0x200           // burnout
        if (t >= 22f) fs = fs or (0x02 or 0x04)    // apogee votes
        if (t >= 38f) fs = fs or 0x08              // landed
        val rate = when {
            t < 12f -> 0f
            t < 14.5f -> 95f
            t < 22f -> 30f - (t - 14.5f) * 4f
            t < 38f -> -25f
            else -> 0f
        }
        // ~73 m of easterly drift under canopy → "Landed. 75 meters away".
        val lon = PAD_LON + 0.00005 * (min(t, 38f).coerceAtLeast(22f) - 22f)
        return String.format(
            Locale.ROOT,
            """{"rid":1,"run":"Booster","st":"%s","fs":%d,"ps":%d,"h":%d,""" +
                """"nsat":9,"vol":%.2f,"cur":%.1f,"soc":%.1f,"palt":%.1f,""" +
                """"malt":%.1f,"arate":%.1f,"mspd":%.1f,"lat":%.6f,"lon":%.6f}""",
            phase, fs,
            0x00A or 0x080,        // ch1+ch4 continuity (demo)
            HEALTH,
            8.2f - t * 0.004f,
            // High-rate logging draw until deployment drops the rate (#623).
            120f + (if (t >= 12f && t < 22f) 900f else 0f),
            87f - t * 0.1f, alt, maxAlt, rate, maxSpeed,
            PAD_LAT, lon,
        )
    }
}
