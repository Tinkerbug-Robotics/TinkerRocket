package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.Klobuchar
import com.tinkerbug.tinkerrocket.protocol.PadCalState
import com.tinkerbug.tinkerrocket.protocol.PadCalStatus
import com.tinkerbug.tinkerrocket.protocol.PadIonoSource
import com.tinkerbug.tinkerrocket.session.PadCalReadout.Line
import com.tinkerbug.tinkerrocket.session.PadCalReadout.Tone
import java.time.Instant
import java.time.ZoneOffset
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/** #1600 phase 5: what the pad calibration screen says about a 0xD0 frame. */
class PadCalReadoutTest {

    /** The wire fixture padcal_0xD0_40's values, with the state and flags open. */
    private fun status(
        calState: PadCalState = PadCalState.RUNNING,
        calProgress: Int = 42,
        calGravityMg: Int = 1003,
        flags: Int = 0x0F,
        ionoSource: PadIonoSource = PadIonoSource.BROADCAST,
        ionoAgeDays: Int = -1,
        satsNoEph: Int = 2,
        satsCorrected: Int = 9,
        posSigmaCm: Int = 215,
    ) = PadCalStatus(
        timeUs = 61_234_567, calState = calState, calProgress = calProgress,
        calGravityMg = calGravityMg, satsTracked = 14, satsNoEph = satsNoEph, satsSettling = 3,
        satsCorrected = satsCorrected, corrRmsCm = 87, ionoSource = ionoSource, flags = flags,
        ionoGpsDay = 17082, ionoAgeDays = ionoAgeDays, padFixes = 1500, padSecs = 240,
        padScatterCm = 180, padLatE7 = 399_716_265, padLonE7 = -749_363_464, padAltMm = -31_234,
        posSigmaCm = posSigmaCm,
    )

    @Test
    fun theGyroAndAccelLineForEachState() {
        assertEquals(
            Line("Gyro & accel: calibrating, 42 % — keep the rocket still"),
            PadCalReadout.gyroAccel(status()),
        )
        assertEquals(
            Line("Gyro & accel calibrated (1.003 g)", Tone.OK),
            PadCalReadout.gyroAccel(status(PadCalState.DONE, 100)),
        )
        assertEquals(
            Line(
                "Rejected: gravity 0.87 g — keep the rocket still and try again; " +
                    "the previous calibration is kept",
                Tone.WARN,
            ),
            PadCalReadout.gyroAccel(status(PadCalState.REJECTED, 100, calGravityMg = 870)),
        )
        // A window with no samples is rejected too, with no gravity to quote.
        assertEquals(
            "Rejected: keep the rocket still and try again; the previous calibration is kept",
            PadCalReadout.gyroAccel(status(PadCalState.REJECTED, 100, calGravityMg = 0)).text,
        )
        assertEquals(
            Line("Not run: the rocket is not on the pad (or a sim is running)", Tone.WARN),
            PadCalReadout.gyroAccel(status(PadCalState.REFUSED, 0, calGravityMg = 0)),
        )
        assertEquals(
            "Gyro & accel: no pad calibration since the rocket powered up",
            PadCalReadout.gyroAccel(status(PadCalState.IDLE, 0, calGravityMg = 0)).text,
        )
    }

    @Test
    fun theGnssBlockReadsTheSurvey() {
        assertEquals(
            listOf(
                "Satellites tracked: 14",
                "Waiting for ephemeris: 2",
                "Settling: 3",
                "Pad corrections: 9 (spread 0.9 m)",
                // iono_age_days -1: a day "ahead" of the receiver is as fresh as can be.
                "Ionosphere: broadcast set of 13 Oct 2026, 0 days old",
                "Pad position: 1500 fixes over 4:00, scatter 1.8 m",
                // 2.15 m is 2.1499... in binary: ties-to-even printf, as iOS prints it.
                "Raw-GNSS filter: running, position sigma 2.1 m",
            ),
            PadCalReadout.gnss(status()).map { it.text },
        )
        // Once every satellite has its ephemeris the wait line goes.
        assertTrue(PadCalReadout.gnss(status(satsNoEph = 0)).none { it.text.startsWith("Waiting") })
        // Launched: the pad average is frozen.
        assertEquals(
            "Pad position: 1500 fixes over 4:00, scatter 1.8 m, frozen at launch",
            PadCalReadout.gnss(status(flags = 0x1F))[5].text,
        )
    }

    @Test
    fun noGnssBlockOnABoardWithoutTheRawGnssFilter() {
        assertEquals(emptyList(), PadCalReadout.gnss(status(flags = 0x0E)))
    }

    @Test
    fun aFixThatDisagreesWithTheRawMeasurementsLeadsTheBlockAsAWarning() {
        // PCS_PAD_MISMATCH: the pad corrections are off and the source reads none.
        val lines = PadCalReadout.gnss(
            status(flags = 0x01 or 0x20, ionoSource = PadIonoSource.NONE, satsCorrected = 0, posSigmaCm = 0),
        )
        assertEquals(
            Line("Receiver's fix disagrees with its raw measurements — pad corrections off", Tone.WARN),
            lines.first(),
        )
        assertTrue(Line("Pad corrections: 0") in lines)
        assertTrue(Line("Ionosphere: none yet") in lines)
        assertTrue(Line("Pad position: no fixes yet") in lines)
        assertTrue(Line("Raw-GNSS filter: not running yet") in lines)
        assertTrue(PadCalReadout.gnss(status()).none { it.tone == Tone.WARN })
    }

    @Test
    fun theIonosphereSources() {
        assertEquals("pad survey (backup)", PadCalReadout.ionosphere(status(ionoSource = PadIonoSource.PAD_SURVEY)))
        assertEquals("none yet", PadCalReadout.ionosphere(status(ionoSource = PadIonoSource.NONE)))
        assertEquals(
            "broadcast set of 13 Oct 2026, 1 day old",
            PadCalReadout.ionosphere(status(ionoAgeDays = 1)),
        )
        assertEquals(
            "broadcast set of 13 Oct 2026, 12 days old",
            PadCalReadout.ionosphere(status(ionoAgeDays = 12)),
        )
        // Before the receiver has given its date the age is not known.
        assertEquals("broadcast set of 13 Oct 2026", PadCalReadout.ionosphere(status(flags = 0x0B)))
    }

    @Test
    fun thePhonesOwnSet() {
        assertEquals(
            "Ionosphere on this phone: none — the rocket will use its pad survey",
            PadCalReadout.phoneIonosphere(null, ZoneOffset.UTC),
        )
        val held = IonosphereCache.Entry(
            Klobuchar(listOf(21, 0, -2, 0, 60, -1, -4, 3), 17082),
            Instant.parse("2026-10-13T10:45:00Z").toEpochMilli(),
        )
        assertEquals(
            "Ionosphere on this phone: set of 13 Oct 2026 (fetched 13 Oct 10:45)",
            PadCalReadout.phoneIonosphere(held, ZoneOffset.UTC),
        )
    }
}
