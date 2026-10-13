package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.Klobuchar
import kotlinx.coroutines.async
import kotlinx.coroutines.delay
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import java.io.ByteArrayOutputStream
import java.io.File
import java.nio.file.Files
import java.time.Instant
import java.time.LocalDate
import java.util.Locale
import java.util.zip.CRC32
import java.util.zip.Deflater
import kotlin.random.Random
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertSame
import kotlin.test.assertTrue

/**
 * #1600 phase 5: the broadcast ionosphere on the phone — NOAA's file for
 * today, yesterday's when that fails, the cache when both do, and a pad
 * calibration that never waits on the network for long.
 */
class IonosphereSourceTest {

    // The header of NOAA's brdc2860.26n (2026-10-13), as KlobucharTest has it.
    private val header = """
     2.11           N: GPS NAV DATA                         RINEX VERSION / TYPE
teqc  2019Feb25     CORS-ADM Account    20261013 10:45:02UTCPGM / RUN BY / DATE
    1.9558D-08  0.0000D+00 -1.1921D-07  0.0000D+00          ION ALPHA
    1.2288D+05 -1.6384D+04 -2.6214D+05  1.9661D+05          ION BETA
    0.000000000000D+00 3.552713678801D-15   405504     2440 DELTA-UTC: A0,A1,T,W
                                                            END OF HEADER
 1 26 10 13  0  0  0.0 3.508096560836D-04 1.136868377216D-12 0.000000000000D+00
""".trimStart('\n')

    private val broadcast = listOf(21, 0, -2, 0, 60, -1, -4, 3)

    private val now = Instant.parse("2026-10-13T12:00:00Z").toEpochMilli()
    private val todayUrl = Klobuchar.noaaUrl(LocalDate.of(2026, 10, 13))
    private val yesterdayUrl = Klobuchar.noaaUrl(LocalDate.of(2026, 10, 12))

    /**
     * gzip as NOAA serves it: FNAME set, so a NUL-terminated file name sits
     * between the 10-byte header and the DEFLATE body.
     */
    private fun gzip(text: String, name: String = "brdc2860.26n"): ByteArray {
        val data = text.encodeToByteArray()
        val deflater = Deflater(Deflater.DEFAULT_COMPRESSION, true)   // raw DEFLATE
        deflater.setInput(data)
        deflater.finish()
        val body = ByteArrayOutputStream()
        val buf = ByteArray(4096)
        while (!deflater.finished()) body.write(buf, 0, deflater.deflate(buf))
        deflater.end()
        val out = ByteArrayOutputStream()
        out.write(byteArrayOf(0x1f, 0x8b.toByte(), 8, 0x08, 0, 0, 0, 0, 0, 3))   // FLG = FNAME
        out.write(name.encodeToByteArray())
        out.write(0)
        out.write(body.toByteArray())
        fun le32(v: Long) { for (i in 0 until 4) out.write(((v ushr (8 * i)) and 0xFF).toInt()) }
        le32(CRC32().apply { update(data) }.value)
        le32(data.size.toLong())
        return out.toByteArray()
    }

    private fun tmp(): File = Files.createTempDirectory("iono").toFile()

    private fun cacheIn(dir: File) = IonosphereCache(File(dir, "klobuchar.json"))

    private fun TestScope.source(
        responses: Map<String, ByteArray?>,
        dir: File = tmp(),
        fetched: MutableList<String> = mutableListOf(),
    ) = IonosphereSource(
        scope = backgroundScope,
        cache = cacheIn(dir),
        fetch = { url -> fetched += url; responses[url] },
        clock = { now },
    )

    @Test
    fun `today's file is read, stored and dated today`() = runTest {
        val dir = tmp()
        val fetched = mutableListOf<String>()
        val s = source(mapOf(todayUrl to gzip(header)), dir, fetched)
        assertFalse(s.isCurrent())

        val got = assertNotNull(s.refreshIfStale()).await()
        val held = assertNotNull(s.held.value)
        assertEquals(got, held)
        assertEquals(broadcast, held.klobuchar.raw)
        assertEquals(17082, held.klobuchar.gpsDay)
        assertEquals(now, held.fetchedAtMs)
        assertEquals(listOf(todayUrl), fetched)
        // Kept for the field, and read back on the next start.
        assertEquals(held, cacheIn(dir).load())
        assertEquals(held, source(emptyMap(), dir).held.value)
        // Today's set is never fetched again.
        assertTrue(s.isCurrent())
        assertNull(s.refreshIfStale())
        assertEquals(broadcast, s.forPadCalibration()?.raw)
        assertEquals(listOf(todayUrl), fetched)
    }

    @Test
    fun `when today's file fails, yesterday's is read and dated yesterday`() = runTest {
        val fetched = mutableListOf<String>()
        val s = source(mapOf(todayUrl to null, yesterdayUrl to gzip(header)), fetched = fetched)
        assertNotNull(s.refreshIfStale()).await()
        assertEquals(listOf(todayUrl, yesterdayUrl), fetched)
        assertEquals(17081, s.held.value?.klobuchar?.gpsDay, "the date of the file it came from")
        assertFalse(s.isCurrent(), "the next refresh tries today's again")
    }

    @Test
    fun `when both fail, the cache stays`() = runTest {
        val dir = tmp()
        val old = IonosphereCache.Entry(Klobuchar(listOf(1, 2, 3, 4, 5, 6, 7, 8), 17000), 1234L)
        assertTrue(cacheIn(dir).store(old))
        val fetched = mutableListOf<String>()
        val s = source(emptyMap(), dir, fetched)
        assertEquals(old, s.held.value, "the cache is what the app starts with")

        assertNull(assertNotNull(s.refreshIfStale()).await())
        assertEquals(listOf(todayUrl, yesterdayUrl), fetched)
        assertEquals(old, s.held.value)
        assertEquals(old, cacheIn(dir).load())
        assertEquals(old.klobuchar, s.forPadCalibration(), "and a pad calibration sends it")
    }

    @Test
    fun `a header without the ION lines stores nothing`() = runTest {
        val noIono = header.lines().filterNot { it.contains("ION ") }.joinToString("\n")
        val dir = tmp()
        // Yesterday's is not gzip at all: no file either.
        val s = source(
            mapOf(todayUrl to gzip(noIono), yesterdayUrl to noIono.encodeToByteArray()),
            dir,
        )
        assertNull(assertNotNull(s.refreshIfStale()).await())
        assertNull(s.held.value)
        assertFalse(File(dir, "klobuchar.json").exists())
        assertNull(s.forPadCalibration(), "nothing to send: the rocket uses its pad survey")
    }

    @Test
    fun `a pad calibration waits on a slow refresh only briefly, then sends the cache`() = runTest {
        val dir = tmp()
        val cached = IonosphereCache.Entry(Klobuchar(broadcast, 17081), 1L)
        cacheIn(dir).store(cached)
        val s = IonosphereSource(
            scope = backgroundScope,
            cache = cacheIn(dir),
            fetch = { url -> delay(30_000); if (url == todayUrl) gzip(header) else null },
            clock = { now },
        )
        val sent = async { s.forPadCalibration() }
        advanceTimeBy(IonosphereSource.QUICK_REFRESH_MS - 1)
        runCurrent()
        assertFalse(sent.isCompleted, "it does give the refresh its few seconds")
        advanceTimeBy(1)
        runCurrent()
        assertEquals(cached.klobuchar, sent.await())

        // The refresh was not abandoned: it lands for the next calibration.
        assertEquals(cached, s.held.value)
        advanceTimeBy(30_000)
        runCurrent()
        assertEquals(17082, s.held.value?.klobuchar?.gpsDay)
        assertEquals(17082, cacheIn(dir).load()?.klobuchar?.gpsDay)
    }

    @Test
    fun `a refresh already running is shared, not started twice`() = runTest {
        val fetched = mutableListOf<String>()
        val s = IonosphereSource(
            scope = backgroundScope,
            cache = cacheIn(tmp()),
            fetch = { url -> fetched += url; delay(1_000); gzip(header) },
            clock = { now },
        )
        val first = s.refreshIfStale()
        assertSame(first, s.refreshIfStale())
        advanceTimeBy(1_001)
        runCurrent()
        assertEquals(listOf(todayUrl), fetched)
    }

    @Test
    fun `a refresh that throws is a failed refresh, not a crash`() = runTest {
        val s = IonosphereSource(
            scope = backgroundScope,
            cache = cacheIn(tmp()),
            fetch = { throw IllegalStateException("a fetcher breaking its contract") },
            clock = { now },
        )
        assertNull(s.forPadCalibration())
    }

    @Test
    fun `an unreadable cache file is no set`() {
        val f = File(tmp(), "klobuchar.json")
        f.writeText("""{"raw":[1,2,3],"gps_day":17082,"fetched_at_ms":1}""")   // five bytes short
        assertNull(IonosphereCache(f).load())
        f.writeText("""{"raw":[1,2,3,4,5,6,7,300],"gps_day":17082,"fetched_at_ms":1}""")   // not a byte
        assertNull(IonosphereCache(f).load())
        f.writeText("not json")
        assertNull(IonosphereCache(f).load())
        assertNull(IonosphereCache(File(tmp(), "absent.json")).load())
    }

    @Test
    fun `gunzip keeps the header of a file cut short, caps, and refuses what is not gzip`() {
        // A day's records after the header: digits that do not compress away.
        val rnd = Random(1600)
        val records = (0 until 2_000).joinToString("\n") {
            (0 until 4).joinToString(" ") { String.format(Locale.ROOT, "%.12fD+00", rnd.nextDouble()) }
        }
        val gz = gzip(header + records)
        val cut = gz.copyOf(gz.size / 2)
        val text = assertNotNull(IonosphereSource.gunzip(cut, IonosphereSource.MAX_TEXT_BYTES))
            .decodeToString()
        assertTrue(text.length < header.length + records.length, "it really was cut")
        assertEquals(broadcast, Klobuchar.fromRinexHeader(text, 17082)?.raw)

        assertEquals(64, IonosphereSource.gunzip(gz, 64)?.size)
        assertNull(IonosphereSource.gunzip("plain text".encodeToByteArray(), 1_000))
        assertNull(IonosphereSource.gunzip(ByteArray(0), 1_000))
    }
}
