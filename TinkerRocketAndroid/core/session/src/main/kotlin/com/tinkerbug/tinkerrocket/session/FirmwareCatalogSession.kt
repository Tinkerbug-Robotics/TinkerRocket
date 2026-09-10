package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.EspImage
import com.tinkerbug.tinkerrocket.protocol.FirmwareCatalog
import com.tinkerbug.tinkerrocket.protocol.FirmwareFetch
import com.tinkerbug.tinkerrocket.protocol.FirmwareImage
import com.tinkerbug.tinkerrocket.protocol.FirmwareRelease
import com.tinkerbug.tinkerrocket.protocol.FirmwareRepository
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

/**
 * Finding a published firmware image for the unit in front of you, and getting
 * its bytes (#773 step 4c). iOS twin: `Models/FirmwareCatalogSession.swift`.
 *
 * This is the sequencing only — which request follows which, and what each
 * outcome means to an operator. Deciding WHICH images fit is
 * [FirmwareCatalog]'s job and proving the bytes are [FirmwareRepository]'s, so
 * neither is repeated here. The UI renders [state] and nothing else.
 *
 * It ends at [State.Downloaded]. Handing the bytes to the OTA flow is the
 * screen's business, and it goes through exactly the same path as a file the
 * operator picked by hand — including `EspImage.check`, which reads the
 * downloaded image's own header rather than trusting the manifest that
 * described it.
 */
public class FirmwareCatalogSession(
    private val repository: FirmwareRepository,
    private val scope: CoroutineScope,
    /**
     * Where downloaded firmware is kept, or null for a session that never
     * stores anything. With a cache, a phone that fetched at home can still
     * list and flash at a field with no signal — which #773's acceptance line
     * asks for and is the ordinary condition at a launch site, not an edge.
     */
    private val cache: FirmwareCache? = null,
) {
    public sealed interface State {
        public data object Idle : State

        /** Talking to GitHub. One API call, then one asset fetch. */
        public data object Checking : State

        /**
         * A release was found and [images] are the ones for this unit, best
         * first. [best] is null when the unit's board is known and nothing in
         * the release matches it — the list is still offered, because refusing
         * to guess is not the same as refusing to show.
         */
        public data class Ready(
            val release: FirmwareRelease,
            val images: List<FirmwareImage>,
            val best: FirmwareImage?,
            /** True when [best] is already what the unit is running. */
            val alreadyRunning: Boolean,
            /**
             * Whether the unit's board revision could be established at all —
             * provisioned, or read out of its running version. When it could
             * not, a null [best] means "nobody knows what this is", which is a
             * different thing to tell an operator than "this release has
             * nothing for your board".
             */
            val boardKnown: Boolean,
            /**
             * True when this came from the cache because the network could not
             * be reached. The images are still real and still verified; the
             * catalog may simply be older than what has since been published,
             * and the operator should be told which they are looking at.
             */
            val offline: Boolean,
            /** SHAs already held on the phone, for a "have this" marker. */
            val held: Set<String>,
        ) : State

        public data class Downloading(val image: FirmwareImage) : State

        /** Fetching [index] of [total] for offline use. */
        public data class Prefetching(
            val image: FirmwareImage,
            val index: Int,
            val total: Int,
        ) : State

        /** Terminal for [prefetch]: how many landed, and what did not. */
        public data class Prefetched(
            val release: FirmwareRelease,
            val stored: Int,
            val failed: List<String>,
            val bytesHeld: Long,
        ) : State

        /** Bytes that matched the manifest's size and SHA-256. Terminal. */
        public data class Downloaded(
            val release: FirmwareRelease,
            val image: FirmwareImage,
            val bytes: ByteArray,
        ) : State {
            override fun equals(other: Any?): Boolean =
                other is Downloaded && other.image == image && other.bytes.contentEquals(bytes)
            override fun hashCode(): Int = 31 * image.hashCode() + bytes.contentHashCode()
        }

        public data class Failed(val reason: String) : State
    }

    private val _state = MutableStateFlow<State>(State.Idle)
    public val state: StateFlow<State> = _state.asStateFlow()

    private var job: Job? = null
    private var release: FirmwareRelease? = null

    /**
     * Look for a release carrying an image for this unit.
     *
     * [expectedProject] is the hard filter, exactly as when a file is picked by
     * hand — it is the only field separating a base station from an out
     * computer, both ESP32-S3 with byte-identical app slots.
     */
    public fun check(
        expectedProject: String,
        provisionedBoard: String?,
        runningVersion: String? = null,
    ) {
        job?.cancel()
        _state.value = State.Checking
        job = scope.launch {
            val fetched = repository.latestManifest()
            // No signal is the ordinary condition at a launch site, so it is a
            // fallback rather than a failure: anything cached is still real and
            // still verified on the way out.
            val offline = fetched == null
            val pair = if (fetched != null) {
                cache?.putCatalog(fetched)
                fetched.release to fetched.manifest
            } else {
                cache?.catalog()
            }
            if (pair == null) {
                // Deliberately not "no firmware available": the overwhelmingly
                // likely cause at a launch site is no signal, and telling an
                // operator their firmware is missing when their phone simply
                // cannot reach GitHub sends them looking in the wrong place.
                _state.value = State.Failed(
                    "Could not reach the firmware releases, and nothing has been " +
                        "downloaded on this phone yet. Check the connection — or " +
                        "fetch a release before leaving for the field.",
                )
                return@launch
            }
            val (rel, manifest) = pair
            release = rel
            // The board the unit is provisioned with, or failing that the one
            // its running firmware claims to be. EspImage.check has always
            // used exactly this fallback; the catalog was never given it, so
            // on a board provisioned before #773 step 2 — which is every board
            // in the field — it ranked with nothing to rank on. See
            // FirmwareCatalog.best for what that produced on the bench.
            val effectiveBoard = provisionedBoard?.trim()?.ifEmpty { null }
                ?: EspImage.boardSuffix(runningVersion)

            val images = FirmwareCatalog.forUnit(manifest, expectedProject, effectiveBoard)
            if (images.isEmpty()) {
                _state.value = State.Failed(
                    "Release ${rel.tag} carries no $expectedProject image.",
                )
                return@launch
            }
            val best = FirmwareCatalog.best(manifest, expectedProject, effectiveBoard)
            _state.value = State.Ready(
                release = rel,
                images = images,
                best = best,
                alreadyRunning = best != null && runningVersion != null &&
                    best.version == runningVersion,
                boardKnown = effectiveBoard != null,
                offline = offline,
                held = cache?.heldShas() ?: emptySet(),
            )
        }
    }

    /** Download one image and prove it against the manifest before showing it. */
    public fun download(image: FirmwareImage) {
        val rel = release ?: run {
            _state.value = State.Failed("No release selected — check for updates first.")
            return
        }
        job?.cancel()
        // A cached hit is instantaneous and needs no network — the whole point
        // of having fetched at home. `cache.image` re-hashes before handing
        // anything back, so this is not a shortcut past verification.
        cache?.image(image)?.let {
            _state.value = State.Downloaded(rel, image, it)
            return
        }
        _state.value = State.Downloading(image)
        job = scope.launch {
            when (val got = repository.download(rel, image)) {
                is FirmwareFetch.Ok -> {
                    cache?.putImage(image, got.bytes)
                    _state.value = State.Downloaded(rel, image, got.bytes)
                }
                // The split matters to whoever is standing at the pad: one is
                // "try again on better signal", the other is "do not flash
                // this". Neither hands any bytes back.
                is FirmwareFetch.Unreachable ->
                    _state.value = State.Failed("Download did not finish: ${got.reason}")
                is FirmwareFetch.Corrupt ->
                    _state.value = State.Failed(
                        "Downloaded image does not match the release: ${got.reason}",
                    )
            }
        }
    }

    /**
     * Fetch several images and keep them, without flashing anything — the
     * "do this at home" action.
     *
     * Reports progress as [State.Prefetching] and ends on [State.Prefetched]
     * with what it managed. Deliberately does NOT fail the whole run on one
     * bad image: getting three of four onto the phone before leaving is worth
     * more than an all-or-nothing refusal.
     */
    public fun prefetch(images: List<FirmwareImage>) {
        val rel = release ?: run {
            _state.value = State.Failed("No release selected — check for updates first.")
            return
        }
        val store = cache ?: run {
            _state.value = State.Failed("This device has nowhere to keep downloads.")
            return
        }
        job?.cancel()
        job = scope.launch {
            var done = 0
            val failed = mutableListOf<String>()
            for ((i, img) in images.withIndex()) {
                _state.value = State.Prefetching(img, i + 1, images.size)
                if (store.hasImage(img) && store.image(img) != null) { done++; continue }
                when (val got = repository.download(rel, img)) {
                    is FirmwareFetch.Ok -> { store.putImage(img, got.bytes); done++ }
                    is FirmwareFetch.Unreachable -> failed += img.file
                    is FirmwareFetch.Corrupt -> failed += img.file
                }
            }
            // Superseded images age out here rather than filling the phone one
            // release at a time. Nothing dropped is precious; it can be
            // fetched again with a signal.
            store.prune(images.map { it.sha256 }.toSet())
            _state.value = State.Prefetched(rel, done, failed, store.bytesHeld())
        }
    }

    public fun reset() {
        job?.cancel()
        job = null
        release = null
        _state.value = State.Idle
    }
}
