package com.tinkerbug.tinkerrocket.app

import android.app.Application
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Before
import org.junit.Test
import org.junit.runner.RunWith

/**
 * Two boards can both name a log `flight_3.bin` (a board without a phone time
 * sync numbers its own logs), and the cache used to key on that name alone:
 * the second download replaced the first board's flight, CSV and summary. The
 * cache name now carries the unit ID ("flight_3__a1cae150.bin"). Same cases as
 * iOS FlightCacheNamesTests.
 */
@RunWith(AndroidJUnit4::class)
class FlightCacheNamesTest {

    private val context = ApplicationProvider.getApplicationContext<Application>()

    @Before
    fun clean() {
        FlightCache.binDir(context).deleteRecursively()
        FlightCache.csvDir(context).deleteRecursively()
    }

    @Test
    fun rocketFlightsCarryTheUnitId() {
        assertEquals("flight_3__a1cae150.bin", FlightCache.cacheName("flight_3.bin", "a1cae150"))
        assertEquals(
            "flight_20261024_161431__a1cae150.bin",
            FlightCache.cacheName("flight_20261024_161431.bin", "a1cae150"),
        )
        assertEquals("flight_3__a1cae150.bin", FlightCache.cacheName("flight_3__a1cae150.bin", "a1cae150"))
    }

    @Test
    fun otherNamesAndUnknownUnitsKeepTheBoardsName() {
        assertEquals("lora_003.bin", FlightCache.cacheName("lora_003.bin", "a1cae150"))
        assertEquals("flight_3.bin", FlightCache.cacheName("flight_3.bin", ""))
        assertEquals("flight_3.bin", FlightCache.cacheName("flight_3.bin", "../x"))
    }

    @Test
    fun theUnitReadsBackOutOfTheName() {
        assertEquals("a1cae150", FlightCache.unitIdInCacheName("flight_3__a1cae150.bin"))
        assertNull(FlightCache.unitIdInCacheName("flight_20261024_161431.bin"))
    }

    /** The bug: the same board filename from two units. Both copies survive. */
    @Test
    fun twoBoardsSameNameBothKept() {
        val a = FlightCache.cacheName("flight_3.bin", "a1cae150")
        val b = FlightCache.cacheName("flight_3.bin", "9e8a1b2c")
        FlightCache.binFileFor(context, a).writeText("board A's flight")
        FlightCache.binFileFor(context, b).writeText("board B's flight, longer")
        FlightCache.csvFileFor(context, a).writeText("t\n0\n")
        FlightCache.csvFileFor(context, b).writeText("t\n0\n")
        assertEquals("board A's flight", FlightCache.binFileFor(context, a).readText())
        assertEquals("board B's flight, longer", FlightCache.binFileFor(context, b).readText())
        assertEquals(
            listOf("flight_3 · 9e8a1b2c", "flight_3 · a1cae150"),
            FlightCache.listSavedFlights(context).map { it.displayName }.sorted(),
        )
    }

    /**
     * A copy cached before unit IDs (the board's own name) still counts as this
     * unit's, unless its summary says another unit downloaded it.
     */
    @Test
    fun aLegacyCopyResolvesUnlessItIsAnotherUnits() {
        FlightCache.binFileFor(context, "flight_3.bin").writeText("old")
        FlightCache.csvFileFor(context, "flight_3.bin").writeText("t\n0\n")
        assertEquals("flight_3.bin", FlightCache.cachedName(context, "flight_3.bin", "a1cae150"))

        FlightCache.summaryFileFor(context, "flight_3.bin")
            .writeText("""{ "source" : { "unit_id" : "9e8a1b2c" } }""")
        assertEquals("flight_3.bin", FlightCache.cachedName(context, "flight_3.bin", "9e8a1b2c"))
        assertEquals("flight_3__a1cae150.bin", FlightCache.cachedName(context, "flight_3.bin", "a1cae150"))
    }

    @Test
    fun aQualifiedCopyWinsOverALegacyOne() {
        FlightCache.binFileFor(context, "flight_3.bin").writeText("old")
        FlightCache.binFileFor(context, "flight_3__a1cae150.bin").writeText("new")
        assertEquals("flight_3__a1cae150.bin", FlightCache.cachedName(context, "flight_3.bin", "a1cae150"))
    }

    @Test
    fun aNewFlightResolvesToTheQualifiedName() {
        assertEquals("flight_7__a1cae150.bin", FlightCache.cachedName(context, "flight_7.bin", "a1cae150"))
        assertEquals("lora_001.bin", FlightCache.cachedName(context, "lora_001.bin", "a1cae150"))
    }
}
