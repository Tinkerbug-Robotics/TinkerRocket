package com.tinkerbug.tinkerrocket.protocol

import java.time.LocalDate
import kotlin.test.Test
import kotlin.test.assertContentEquals
import kotlin.test.assertEquals
import kotlin.test.assertNull

/** Port of iOS `KlobucharTests.swift` — keep the two in step. */
class KlobucharTest {

    // The header of NOAA's brdc2860.26n (2026-10-13), as served.
    private val noaaHeader = """
     2.11           N: GPS NAV DATA                         RINEX VERSION / TYPE
teqc  2019Feb25     CORS-ADM Account    20261013 10:45:02UTCPGM / RUN BY / DATE
    1.9558D-08  0.0000D+00 -1.1921D-07  0.0000D+00          ION ALPHA
    1.2288D+05 -1.6384D+04 -2.6214D+05  1.9661D+05          ION BETA
    0.000000000000D+00 3.552713678801D-15   405504     2440 DELTA-UTC: A0,A1,T,W
                                                            END OF HEADER
 1 26 10 13  0  0  0.0 3.508096560836D-04 1.136868377216D-12 0.000000000000D+00
""".trimStart('\n')

    // The same parameters as a RINEX 3 file states them.
    private val rinex3Header = """
     3.04           N: GNSS NAV DATA    M: MIXED            RINEX VERSION / TYPE
GPSA   1.9558E-08  0.0000E+00 -1.1921E-07  0.0000E+00       IONOSPHERIC CORR
GPSB   1.2288E+05 -1.6384E+04 -2.6214E+05  1.9661E+05       IONOSPHERIC CORR
GAL    2.5250E+01  2.3438E-01  4.3640E-03  0.0000E+00       IONOSPHERIC CORR
                                                            END OF HEADER
""".trimStart('\n')

    @Test
    fun readsTheBroadcastBytesFromNoaasHeader() {
        val k = Klobuchar.fromRinexHeader(noaaHeader, 17082)!!
        assertEquals(listOf(21, 0, -2, 0, 60, -1, -4, 3), k.raw)
        assertEquals(17082, k.gpsDay)
        assertEquals(1.9558e-08, k.values()[0], 1e-12)
        assertEquals(196608.0, k.values()[7], 0.0)
    }

    @Test
    fun readsRinex3sGpsLinesAndNotGalileos() {
        assertEquals(Klobuchar.fromRinexHeader(noaaHeader, 17082), Klobuchar.fromRinexHeader(rinex3Header, 17082))
    }

    @Test
    fun refusesAHeaderWithoutBothOrWithAValueOffItsUnit() {
        val noBeta = noaaHeader.lines().filterNot { it.endsWith("ION BETA") }.joinToString("\n")
        assertNull(Klobuchar.fromRinexHeader(noBeta, 17082))
        // 1.5 of alpha0's unit is not a value GPS can broadcast
        val off = noaaHeader.replace("    1.9558D-08", "    1.3970D-09")
        assertNull(Klobuchar.fromRinexHeader(off, 17082))
        // nor is one past a signed byte
        val big = noaaHeader.replace("    1.2288D+05", "    2.6419D+05")
        assertNull(Klobuchar.fromRinexHeader(big, 17082))
        // a field that parses but is no number
        val nan = noaaHeader.replace("    1.9558D-08", "           NaN")
        assertNull(Klobuchar.fromRinexHeader(nan, 17082))
        // the ionosphere lines after the header are not read
        val late = noaaHeader.lines().filterNot { it.contains("ION ") }.joinToString("\n") +
            "\n    1.9558D-08  0.0000D+00 -1.1921D-07  0.0000D+00          ION ALPHA\n"
        assertNull(Klobuchar.fromRinexHeader(late, 17082))
    }

    @Test
    fun gpsDayAndNoaasFileName() {
        assertEquals(0, Klobuchar.gpsDay(LocalDate.of(1980, 1, 6)))
        assertEquals(17082, Klobuchar.gpsDay(LocalDate.of(2026, 10, 13)))
        assertEquals(LocalDate.of(2026, 10, 13), Klobuchar.date(17082))
        assertEquals(
            "https://geodesy.noaa.gov/corsdata/rinex/2026/286/brdc2860.26n.gz",
            Klobuchar.noaaUrl(LocalDate.of(2026, 10, 13)),
        )
        assertEquals(
            "https://geodesy.noaa.gov/corsdata/rinex/2027/001/brdc0010.27n.gz",
            Klobuchar.noaaUrl(LocalDate.of(2027, 1, 1)),
        )
    }

    @Test
    fun payloadIsTheBytesThenTheDay() {
        val k = Klobuchar(listOf(21, 0, -2, 0, 60, -1, -4, 3), 17082)
        assertContentEquals(
            byteArrayOf(21, 0, -2, 0, 60, -1, -4, 3, 0xBA.toByte(), 0x42),
            k.payload(),
        )
    }
}
