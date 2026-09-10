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
        ) : State

        public data class Downloading(val image: FirmwareImage) : State

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
            val found = repository.latestManifest()
            if (found == null) {
                // Deliberately not "no firmware available": the overwhelmingly
                // likely cause at a launch site is no signal, and telling an
                // operator their firmware is missing when their phone simply
                // cannot reach GitHub sends them looking in the wrong place.
                _state.value = State.Failed(
                    "Could not reach the firmware releases. Check the phone's " +
                        "connection — a published release cannot be seen without one.",
                )
                return@launch
            }
            val (rel, manifest) = found
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
        _state.value = State.Downloading(image)
        job = scope.launch {
            when (val got = repository.download(rel, image)) {
                is FirmwareFetch.Ok ->
                    _state.value = State.Downloaded(rel, image, got.bytes)
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

    public fun reset() {
        job?.cancel()
        job = null
        release = null
        _state.value = State.Idle
    }
}
