package com.tinkerbug.tinkerrocket.maps

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
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
    /** source key → upstream URL template; injectable for tests. */
    private val upstreamTemplates: Map<String, String> =
        TileSource.entries.associate { it.key to it.urlTemplate },
    private val userAgent: String = "TinkerRocketApp/1.0 (offline map cache)",
) {
    public enum class Phase { IDLE, DOWNLOADING, FINISHED, CANCELLED }

    private val _phase = MutableStateFlow(Phase.IDLE)
    public val phase: StateFlow<Phase> = _phase.asStateFlow()

    private val _done = MutableStateFlow(0)
    public val done: StateFlow<Int> = _done.asStateFlow()

    private val _total = MutableStateFlow(0)
    public val total: StateFlow<Int> = _total.asStateFlow()

    private val _bytes = MutableStateFlow(0L)
    public val bytes: StateFlow<Long> = _bytes.asStateFlow()

    @Volatile private var cancelFlag = false
    private var job: Job? = null

    public val isRunning: Boolean get() = _phase.value == Phase.DOWNLOADING

    public fun start(region: RegionSpec, sourceKey: String) {
        val template = upstreamTemplates[sourceKey] ?: return
        if (isRunning) return
        val tiles = TileMath.tiles(region)

        cancelFlag = false
        _phase.value = Phase.DOWNLOADING
        _total.value = tiles.size
        _done.value = 0
        _bytes.value = 0

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
            workers.forEach { it.join() }
            _phase.value = if (cancelFlag) Phase.CANCELLED else Phase.FINISHED
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
    }

    private fun report(byteCount: Int) {
        _done.update { it + 1 }
        _bytes.update { it + byteCount }
    }

    /** Fetch (or read from cache) one tile; returns its byte size (0 = miss). */
    private fun fetchOne(t: TileXYZ, key: String, template: String): Int {
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
                if (conn.responseCode != 200) return@runCatching 0
                val data = conn.inputStream.readBytes()
                cache.store(data, key, t.z, t.x, t.y)
                data.size
            } finally {
                conn.disconnect()
            }
        }.getOrDefault(0) // missing/4xx tile — count it so progress completes
    }

    private companion object {
        const val MAX_CONCURRENT = 5
    }
}
