package com.tinkerbug.tinkerrocket.maps

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.NonCancellable
import kotlinx.coroutines.withContext
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Semaphore
import kotlinx.coroutines.sync.withPermit
import java.net.HttpURLConnection
import java.net.URL

/** Rough bytes per imagery tile — only used for the pre-download estimate. */
public const val ESTIMATED_TILE_BYTES: Long = 25_000

/**
 * Downloads every tile in a [RegionSpec] to the [OfflineTileCache] with
 * bounded concurrency, publishing progress — port of iOS TileDownloader.
 * Resumable (already-cached tiles are counted instantly, not re-fetched)
 * and cancelable.  Missing/4xx tiles count as done with 0 bytes so
 * progress always completes.
 */
public class TileDownloader(
    private val cache: OfflineTileCache,
    private val scope: CoroutineScope = CoroutineScope(Dispatchers.IO),
    /**
     * source key → upstream URL template; injectable for tests.  Sources
     * with a null template (online-only, e.g. Google) are absent, so
     * [start] no-ops for them — they can never be region-downloaded.
     */
    private val upstreamTemplates: Map<String, String> =
        TileSource.entries.mapNotNull { s -> s.urlTemplate?.let { s.key to it } }.toMap(),
    private val userAgent: String = "TinkerRocketApp/1.0 (offline map cache)",
) {
    /**
     * [FAILED] means NOTHING was retrieved — every tile errored or 404'd and
     * none was already cached.  It is a distinct outcome from [FINISHED]
     * because a run with no connectivity used to reach FINISHED with 0
     * bytes and save a 0 MB region: a saved area that looks real in the
     * list, and is discovered empty at the field with no signal, which is
     * the one place it was meant to work.
     */
    public enum class Phase { IDLE, DOWNLOADING, FINISHED, CANCELLED, FAILED }

    private val _phase = MutableStateFlow(Phase.IDLE)
    public val phase: StateFlow<Phase> = _phase.asStateFlow()

    private val _done = MutableStateFlow(0)
    public val done: StateFlow<Int> = _done.asStateFlow()

    private val _total = MutableStateFlow(0)
    public val total: StateFlow<Int> = _total.asStateFlow()

    private val _bytes = MutableStateFlow(0L)
    public val bytes: StateFlow<Long> = _bytes.asStateFlow()

    /**
     * Tiles that errored or answered non-200.  Surfaced so a partly-covered
     * area can say so instead of looking complete — the count is the only
     * evidence, since a missing tile is indistinguishable from one that was
     * never requested once the run ends.
     */
    private val _failed = MutableStateFlow(0)
    public val failed: StateFlow<Int> = _failed.asStateFlow()

    @Volatile private var cancelFlag = false
    private var job: Job? = null

    /**
     * Tiles THIS run wrote — cache hits are excluded, because [fetchOne]
     * returns early on them.  It is the undo log for a cancel: see [start].
     */
    private val created = mutableListOf<TileXYZ>()

    public val isRunning: Boolean get() = _phase.value == Phase.DOWNLOADING

    /**
     * Download [region] and, on success, hand the totals to [onFinished] so
     * the caller can record it.
     *
     * Both terminal outcomes are settled HERE rather than by whoever is
     * watching [phase], because this runs on its own scope and outlives the
     * screen that started it.  When the UI owned them, backing out mid-run
     * left the download to finish into a manifest nobody wrote — a whole
     * region of tiles on disk that the storage total could not see and
     * Delete could never reclaim.
     *
     * A cancel deletes exactly what this run created, which is a precise
     * undo: tiles that were already on disk are never re-fetched, so an
     * overlapping saved region keeps every tile it had.
     */
    public fun start(
        region: RegionSpec,
        sourceKey: String,
        onFinished: ((tileCount: Int, bytes: Long) -> Unit)? = null,
    ) {
        val template = upstreamTemplates[sourceKey] ?: return
        if (isRunning) return
        val tiles = TileMath.tiles(region)

        cancelFlag = false
        synchronized(created) { created.clear() }
        _phase.value = Phase.DOWNLOADING
        _total.value = tiles.size
        _done.value = 0
        _bytes.value = 0
        _failed.value = 0

        job = scope.launch {
            val sem = Semaphore(MAX_CONCURRENT)
            val workers = mutableListOf<Job>()
            for (tile in tiles) {
                if (cancelFlag) break
                workers += launch {
                    sem.withPermit {
                        if (!cancelFlag) report(fetchOne(tile, sourceKey, template))
                    }
                }
            }
            // Drain first: cancel() only flips a flag and the fetches are
            // blocking, so workers that already passed the check will finish
            // and store AFTER cancel() returned.  Rolling back before this
            // join would miss exactly those tiles.
            workers.forEach { it.join() }
            // NonCancellable so the rollback still runs if this job is ever
            // bound to a lifecycle scope and cancelled for real — leaking the
            // tiles is worse than a few file deletes during teardown.
            withContext(NonCancellable) {
                val mine = synchronized(created) { created.toList().also { created.clear() } }
                // Nothing new was retrieved and something failed, so this run
                // accomplished nothing and there is no area to record.
                //
                // Not `failed == total`: that misses the case this was found
                // in on the bench.  The preview map caches tiles through the
                // same proxy, so with the network down 13 of 2991 tiles were
                // already on disk, `failed` came in 13 short of `total`, and a
                // 13 km area was saved holding 0.2% of itself.
                //
                // Not zero bytes either — a cache hit reports its size, so
                // those 13 tiles counted as ~200 KB.  `created` holds exactly
                // the tiles this run pulled off the network, which is the
                // question being asked.  Empty with no failures is the honest
                // opposite — every tile was already cached — and still saves.
                val nothingArrived = mine.isEmpty() && _failed.value > 0
                when {
                    cancelFlag -> {
                        cache.removeTiles(sourceKey, mine)
                        _phase.value = Phase.CANCELLED
                    }
                    nothingArrived -> {
                        cache.removeTiles(sourceKey, mine)
                        _phase.value = Phase.FAILED
                    }
                    else -> {
                        onFinished?.invoke(_total.value, _bytes.value)
                        _phase.value = Phase.FINISHED
                    }
                }
            }
        }
    }

    public fun cancel() {
        cancelFlag = true
    }

    public fun reset() {
        if (isRunning) return
        _phase.value = Phase.IDLE
        _done.value = 0
        _total.value = 0
        _bytes.value = 0
        _failed.value = 0
    }

    /** @param byteCount bytes retrieved, or null when the tile failed. */
    private fun report(byteCount: Int?) {
        _done.update { it + 1 }
        if (byteCount == null) _failed.update { it + 1 } else _bytes.update { it + byteCount }
    }

    /**
     * Fetch (or read from cache) one tile.  Returns its byte size, or null
     * when it errored or answered non-200 — distinct from a zero-byte body,
     * because "nothing came back" is exactly what a failed run needs to
     * count.  Progress still advances either way, so it always completes.
     */
    private fun fetchOne(t: TileXYZ, key: String, template: String): Int? {
        cache.tileData(key, t.z, t.x, t.y)?.let { return it.size }

        val url = template
            .replace("{z}", t.z.toString())
            .replace("{y}", t.y.toString())
            .replace("{x}", t.x.toString())
        return runCatching {
            val conn = URL(url).openConnection() as HttpURLConnection
            conn.setRequestProperty("User-Agent", userAgent)
            conn.connectTimeout = 10_000
            conn.readTimeout = 15_000
            try {
                if (conn.responseCode != 200) return@runCatching null
                val data = conn.inputStream.readBytes()
                cache.store(data, key, t.z, t.x, t.y)
                synchronized(created) { created += t }
                data.size
            } finally {
                conn.disconnect()
            }
        }.getOrNull() // unreachable/4xx — counted as failed, progress continues
    }

    private companion object {
        const val MAX_CONCURRENT = 5
    }
}
