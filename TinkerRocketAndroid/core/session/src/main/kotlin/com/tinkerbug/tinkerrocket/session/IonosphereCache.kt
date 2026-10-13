package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.Klobuchar
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Deferred
import kotlinx.coroutines.async
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.withTimeoutOrNull
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.long
import kotlinx.serialization.json.put
import java.io.ByteArrayInputStream
import java.io.ByteArrayOutputStream
import java.io.File
import java.io.IOException
import java.time.Instant
import java.time.LocalDate
import java.time.ZoneOffset
import java.util.zip.GZIPInputStream

/**
 * The GPS broadcast ionosphere kept on the phone (#1600 phase 5): the last
 * [Klobuchar] set read from NOAA's navigation file, with the file's date and
 * when it was fetched, so a launch site with no signal still has the set
 * fetched at home.
 *
 * One small JSON file, written beside and renamed into place as
 * [FirmwareCache] writes, and read back through [Klobuchar]'s own checks: a
 * file that does not make a valid set is no set.
 */
public class IonosphereCache(private val file: File) {

    public data class Entry(
        val klobuchar: Klobuchar,
        /** When the phone read it, epoch millis. */
        val fetchedAtMs: Long,
    )

    /** The stored set, or null if there is none or it is unreadable. */
    public fun load(): Entry? = try {
        val o = Json.parseToJsonElement(file.readText()).jsonObject
        Entry(
            Klobuchar(
                raw = o["raw"]!!.jsonArray.map { it.jsonPrimitive.int },
                gpsDay = o["gps_day"]!!.jsonPrimitive.int,
            ),
            fetchedAtMs = o["fetched_at_ms"]!!.jsonPrimitive.long,
        )
    } catch (_: Exception) {
        null
    }

    public fun store(entry: Entry): Boolean = try {
        file.parentFile?.mkdirs()
        val json = buildJsonObject {
            put("raw", JsonArray(entry.klobuchar.raw.map { JsonPrimitive(it) }))
            put("gps_day", entry.klobuchar.gpsDay)
            put("fetched_at_ms", entry.fetchedAtMs)
        }
        // A killed app must not leave half a file under the real name.
        val tmp = File(file.path + ".part")
        tmp.writeText(json.toString())
        tmp.renameTo(file)
    } catch (_: Exception) {
        false
    }
}

/**
 * Keeps an [IonosphereCache] current from NOAA's CORS archive and decides
 * what a pad calibration sends (#1600 phase 5).
 *
 * The file is `brdcDDD0.YYn.gz` ([Klobuchar.noaaUrl]): GPS RINEX 2, gzip,
 * with `ION ALPHA` / `ION BETA` in its header.  Today's (by the UTC date)
 * exists from early in the day and grows through it; yesterday's is the
 * fallback for a 404 just after midnight or a failed fetch.  Nothing is
 * fetched while the held set is today's.
 *
 * A pad calibration never waits on the network for long:
 * [forPadCalibration] gives a stale set one short refresh, then sends what
 * the phone holds, then none.  The rocket judges the set's age by its own
 * receiver and falls back to its pad survey.
 */
public class IonosphereSource(
    /** Where refreshes run: they outlive whoever asked for them. */
    private val scope: CoroutineScope,
    private val cache: IonosphereCache,
    /**
     * An HTTP GET: the body, or null on any failure.  [HttpIonosphereFetch]
     * in the app; tests hand in gzip bytes they built.
     */
    private val fetch: suspend (String) -> ByteArray?,
    /** Epoch millis — injected, never the wall clock directly. */
    private val clock: () -> Long = System::currentTimeMillis,
) {
    private val _held = MutableStateFlow(cache.load())

    /** The set on the phone; null if it has never read one. */
    public val held: StateFlow<IonosphereCache.Entry?> = _held.asStateFlow()

    private var refresh: Deferred<IonosphereCache.Entry?>? = null

    /** The UTC date by the phone's clock: whose file a refresh wants. */
    public fun todayUtc(): LocalDate =
        Instant.ofEpochMilli(clock()).atOffset(ZoneOffset.UTC).toLocalDate()

    /** Is the held set from today's file? */
    public fun isCurrent(): Boolean =
        (_held.value?.klobuchar?.gpsDay ?: 0) >= Klobuchar.gpsDay(todayUtc())

    /**
     * Fetch in the background unless the held set is today's.  The app calls
     * it at start and when the pad calibration screen opens; it never blocks.
     * Returns the refresh to wait on — the running one, if there is one — or
     * null when there is nothing to do.
     */
    @Synchronized
    public fun refreshIfStale(): Deferred<IonosphereCache.Entry?>? {
        if (isCurrent()) return null
        refresh?.takeIf { it.isActive }?.let { return it }
        return scope.async {
            // A refresh that throws must not take a pad calibration with it.
            try {
                fetchLatest()
            } catch (e: CancellationException) {
                throw e
            } catch (_: Exception) {
                null
            }
        }.also { refresh = it }
    }

    /**
     * The set a pad calibration sends: the held one, after one quick refresh
     * if it is not today's.  That refresh gets [timeoutMs] and then carries
     * on in the background for the next calibration; the cache is sent
     * meanwhile, or none.
     */
    public suspend fun forPadCalibration(timeoutMs: Long = QUICK_REFRESH_MS): Klobuchar? {
        refreshIfStale()?.let { pending -> withTimeoutOrNull(timeoutMs) { pending.await() } }
        return _held.value?.klobuchar
    }

    /** Today's file, else yesterday's; the first that reads is stored. */
    private suspend fun fetchLatest(): IonosphereCache.Entry? {
        val today = todayUtc()
        for (date in listOf(today, today.minusDays(1))) {
            val gz = fetch(Klobuchar.noaaUrl(date)) ?: continue
            val text = gunzip(gz, MAX_TEXT_BYTES)?.decodeToString() ?: continue
            val k = Klobuchar.fromRinexHeader(text, Klobuchar.gpsDay(date)) ?: continue
            val entry = IonosphereCache.Entry(k, clock())
            cache.store(entry)
            _held.value = entry
            return entry
        }
        return null
    }

    public companion object {
        /** How long a starting pad calibration waits on a refresh. */
        public const val QUICK_REFRESH_MS: Long = 5_000

        /**
         * The inflated file is read no further than this: the header is at
         * the top, a whole day's file is a few hundred KB, and a wrong URL
         * must not fill the heap.
         */
        public const val MAX_TEXT_BYTES: Int = 4 * 1024 * 1024

        /**
         * The content of [gz], at most [maxBytes] of it; null if it is not
         * gzip at all.  A body cut short keeps what inflated before the cut,
         * since the header it is read for comes first.
         */
        public fun gunzip(gz: ByteArray, maxBytes: Int): ByteArray? {
            val input = try {
                GZIPInputStream(ByteArrayInputStream(gz))
            } catch (_: IOException) {
                return null
            }
            val out = ByteArrayOutputStream()
            try {
                input.use { copyCapped(it, out, maxBytes) }
            } catch (_: IOException) {
                // truncated or corrupt past this point: keep what came out
            }
            return out.toByteArray()
        }
    }
}
