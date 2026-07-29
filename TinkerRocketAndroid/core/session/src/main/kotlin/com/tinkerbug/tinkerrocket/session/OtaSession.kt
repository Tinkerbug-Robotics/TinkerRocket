package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.Commands
import com.tinkerbug.tinkerrocket.protocol.OtaStatusUpdate
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import java.security.MessageDigest

/**
 * Drives an OTA upload end-to-end — port of iOS OTASession.swift:
 * hash image → OTA_BEGIN → chunk pump → OTA_FINISH → await reboot →
 * await reconnect → compare the new firmware version against the
 * pre-flash one.
 *
 * **Lifetime**: keyed by device id and owned ABOVE the [DeviceSession], because
 * the session object is destroyed and recreated across the post-OTA
 * disconnect/reconnect (#140).  The flow therefore never captures a session —
 * it re-resolves one through [sessionLookup] on every poll, which is what lets
 * it survive the reboot and still see the new firmware version.
 *
 * All timing is `delay()`-driven off [scope] so virtual time drives the tests.
 */
public class OtaSession(
    private val scope: CoroutineScope,
    /** Resolves the CURRENT session for this device, or null between reconnects. */
    private val sessionLookup: () -> DeviceSession?,
    private val sha256: (ByteArray) -> ByteArray = { MessageDigest.getInstance("SHA-256").digest(it) },
    /**
     * Wall clock for the #627 FC-relay pacing only.  Injectable because the
     * pacer credits real elapsed time against the rate budget, and under
     * `runTest` a real clock would read ~0 while virtual time races ahead —
     * tests supply their own so the pacing is deterministic.
     */
    private val nanoTime: () -> Long = System::nanoTime,
) {

    public sealed interface State {
        public data object Idle : State
        /** Reading the image + computing its SHA-256. */
        public data object Loading : State
        public data class Uploading(val bytesSent: Long, val totalBytes: Long) : State
        /** OTA_FINISH sent, awaiting ready_to_boot. */
        public data object Verifying : State
        /** Disconnected; waiting for the device to come back. */
        public data object Rebooting : State
        public data class Verified(val newVersion: String) : State
        /** Post-reboot firmware is the SAME as pre-flash — the image didn't take. */
        public data class RollbackDetected(val version: String) : State
        public data class Failed(val reason: String) : State
    }

    private val _state = MutableStateFlow<State>(State.Idle)
    public val state: StateFlow<State> = _state.asStateFlow()

    public var preFlashVersion: String = ""
        private set
    public var imageSize: Int = 0
        private set
    public var imageSha256Hex: String = ""
        private set

    private var job: Job? = null

    public val isRunning: Boolean
        get() = when (_state.value) {
            is State.Loading, is State.Uploading, is State.Verifying, is State.Rebooting -> true
            else -> false
        }

    /** Start (or restart) an OTA run for [image]. */
    public fun start(image: ByteArray, targetIsFc: Boolean = false) {
        job?.cancel()
        job = scope.launch { runFlow(image, targetIsFc) }
    }

    /** User cancel: OTA_ABORT + tear down. Always safe on the firmware side. */
    public fun cancel() {
        job?.cancel()
        job = null
        sessionLookup()?.let {
            it.releaseConnectionPriority()
            it.sendBareCommand(BleCommandId.OTA_ABORT)
        }
        _state.value = State.Failed("Cancelled")
    }

    /** Back to Idle so the same instance can drive another run. */
    public fun reset() {
        job?.cancel()
        job = null
        preFlashVersion = ""
        imageSize = 0
        imageSha256Hex = ""
        _state.value = State.Idle
    }

    // ── Flow ─────────────────────────────────────────────────────────────

    private suspend fun runFlow(image: ByteArray, targetIsFc: Boolean) {
        // 1. Hash + sanity-check the image.
        _state.value = State.Loading
        if (image.size < MIN_IMAGE_BYTES) {
            _state.value = State.Failed(
                "File looks too small to be firmware (${image.size} bytes)",
            )
            return
        }
        val sha = sha256(image)
        imageSize = image.size
        imageSha256Hex = sha.joinToString("") { "%02x".format(it) }

        // An FC OTA changes the FC's version (relayed as fcFirmwareVersion);
        // the OC's own firmwareVersion is untouched — so the rollback check
        // must read the version belonging to the CHOSEN TARGET, else an FC
        // OTA always looks like "version unchanged" (#8 P4).
        preFlashVersion = versionFor(targetIsFc).orEmpty()

        val begin = sessionLookup()
        if (begin == null || !begin.isConnected.value) {
            _state.value = State.Failed("Device disconnected before OTA_BEGIN")
            return
        }

        // 2. OTA_BEGIN, then wait for the firmware to accept it.
        begin.sendCommandFrame(Commands.otaBegin(targetIsFc, image.size.toLong(), sha))
        val beginTimeout = if (targetIsFc) BEGIN_TIMEOUT_FC_MS else BEGIN_TIMEOUT_MS
        if (!awaitOtaState(OtaStatusUpdate.State.READY, beginTimeout)) {
            _state.value = State.Failed(
                "Device did not accept OTA_BEGIN within ${beginTimeout / 1000}s",
            )
            return
        }

        // 3. Chunk pump.  Tighter connection interval for the pump ONLY —
        // the firmware owns connection-parameter policy the rest of the
        // time (#519/#524), so this is released on every exit path.
        val pumpSession = sessionLookup()
        pumpSession?.requestConnectionPriorityHigh()
        try {
            val chunkSize = Commands.otaMaxChunkSize(pumpSession?.negotiatedMtu?.value ?: 23)
            var offset = 0
            // #627: the relay path has a drain rate the OC can actually sustain;
            // exceed it and its BLE stack wedges (see FC_RELAY_MAX_BYTES_PER_SEC).
            // A local OC OTA has no relay and stays uncapped.
            val pumpStartNs = if (targetIsFc) nanoTime() else 0L
            _state.value = State.Uploading(0, image.size.toLong())
            while (offset < image.size) {
                if (!scope.isActive) return

                // A rejected chunk shows up as verify_failed mid-pump; bail
                // rather than pushing the remaining megabyte at a dead session.
                val st = sessionLookup()?.otaStatus?.value
                if (st?.state == OtaStatusUpdate.State.VERIFY_FAILED) {
                    _state.value = State.Failed("Device rejected chunk: ${st.err ?: "unknown"}")
                    sessionLookup()?.sendBareCommand(BleCommandId.OTA_ABORT)
                    return
                }

                val live = sessionLookup()
                if (live == null || !live.isConnected.value) {
                    _state.value = State.Failed("Device disconnected mid-upload at offset $offset")
                    return
                }

                val end = minOf(offset + chunkSize, image.size)
                val chunk = image.copyOfRange(offset, end)
                val isLast = end == image.size
                val ok = runCatching {
                    live.writeOtaChunk(Commands.otaChunkFrame(offset.toLong(), chunk, isLast))
                }.isSuccess
                if (!ok) {
                    _state.value = State.Failed("Chunk write failed at offset $offset")
                    sessionLookup()?.sendBareCommand(BleCommandId.OTA_ABORT)
                    return
                }
                offset = end
                _state.value = State.Uploading(offset.toLong(), image.size.toLong())

                if (targetIsFc && offset < image.size) {
                    val elapsedMs = (nanoTime() - pumpStartNs) / 1_000_000L
                    val pause = OtaTimeouts.fcRelayPaceDelayMs(offset.toLong(), elapsedMs)
                    if (pause > 0) delay(pause)
                }
            }
        } finally {
            sessionLookup()?.releaseConnectionPriority()
        }

        // 4. OTA_FINISH → the firmware verifies the SHA over what it stored.
        _state.value = State.Verifying
        sessionLookup()?.sendBareCommand(BleCommandId.OTA_FINISH)
        val finishTimeout = if (targetIsFc) FINISH_TIMEOUT_FC_MS else FINISH_TIMEOUT_MS
        if (!awaitOtaState(OtaStatusUpdate.State.READY_TO_BOOT, finishTimeout)) {
            val st = sessionLookup()?.otaStatus?.value
            _state.value = if (st?.state == OtaStatusUpdate.State.VERIFY_FAILED) {
                State.Failed("Verify failed: ${st.err ?: "unknown"}")
            } else {
                State.Failed("Device did not finalize OTA within ${finishTimeout / 1000}s")
            }
            return
        }

        // 5. Device reboots ~500 ms after ready_to_boot.  A destroyed session
        // (lookup returns null) counts as disconnected.
        _state.value = State.Rebooting
        awaitPredicate(DISCONNECT_TIMEOUT_MS) {
            val s = sessionLookup()
            s == null || !s.isConnected.value
        }

        // 6. Reconnect — the fleet builds a NEW session for the same device,
        // which sessionLookup() starts returning.
        if (!awaitPredicate(RECONNECT_TIMEOUT_MS) { sessionLookup()?.isConnected?.value == true }) {
            _state.value = State.Failed(
                "Device did not reconnect within ${RECONNECT_TIMEOUT_MS / 1000}s — try power-cycling",
            )
            return
        }

        // 7. Wait for the new version to publish.  A local OTA republishes
        // config_identity on reconnect (fast); an FC OTA keeps the OC link up
        // and can't report until the FC itself finishes rebooting.
        val fwTimeout = if (targetIsFc) FW_TIMEOUT_FC_MS else FW_TIMEOUT_MS
        val pre = preFlashVersion
        val gotNew = awaitPredicate(fwTimeout) {
            val fw = versionFor(targetIsFc)
            !fw.isNullOrEmpty() && fw != pre
        }
        val post = versionFor(targetIsFc).orEmpty()
        _state.value = when {
            gotNew -> State.Verified(post)
            // Same version back = the bootloader rolled us back to the old image.
            post.isNotEmpty() && post == pre -> State.RollbackDetected(pre)
            else -> State.Failed(
                "Reconnected but device didn't publish a new firmware version " +
                    "within ${fwTimeout / 1000}s",
            )
        }
    }

    private fun versionFor(targetIsFc: Boolean): String? = sessionLookup()?.identity?.value?.let {
        if (targetIsFc) it.fcFirmwareVersion else it.firmwareVersion
    }

    /** Poll until the firmware reports [expected]; verify_failed fails fast. */
    private suspend fun awaitOtaState(expected: OtaStatusUpdate.State, timeoutMs: Long): Boolean {
        var waited = 0L
        while (waited < timeoutMs) {
            if (!scope.isActive) return false
            val st = sessionLookup()?.otaStatus?.value
            if (st?.state == expected) return true
            if (st?.state == OtaStatusUpdate.State.VERIFY_FAILED && expected != OtaStatusUpdate.State.VERIFY_FAILED) {
                return false
            }
            delay(POLL_MS)
            waited += POLL_MS
        }
        return false
    }

    private suspend fun awaitPredicate(timeoutMs: Long, predicate: () -> Boolean): Boolean {
        var waited = 0L
        while (waited < timeoutMs) {
            if (!scope.isActive) return false
            if (predicate()) return true
            delay(POLL_MS)
            waited += POLL_MS
        }
        return predicate()
    }

    /**
     * Named views onto [OtaTimeouts], which is the single source of truth and
     * the thing the shared fixture checks.  These read better at the call
     * sites than `millis(OtaStage.X, targetIsFc)` and keep the stage/path
     * pairing in one place instead of at every await.
     */
    public companion object {
        /** ESP32 app images carry a non-trivial header; anything smaller is not firmware. */
        public const val MIN_IMAGE_BYTES: Int = 64

        public const val POLL_MS: Long = OtaTimeouts.POLL_MS

        public val BEGIN_TIMEOUT_MS: Long = OtaTimeouts.millis(OtaStage.BEGIN, false)
        public val BEGIN_TIMEOUT_FC_MS: Long = OtaTimeouts.millis(OtaStage.BEGIN, true)
        public val FINISH_TIMEOUT_MS: Long = OtaTimeouts.millis(OtaStage.FINISH, false)
        public val FINISH_TIMEOUT_FC_MS: Long = OtaTimeouts.millis(OtaStage.FINISH, true)
        public val DISCONNECT_TIMEOUT_MS: Long = OtaTimeouts.millis(OtaStage.DISCONNECT, false)
        public val RECONNECT_TIMEOUT_MS: Long = OtaTimeouts.millis(OtaStage.RECONNECT, false)
        public val FW_TIMEOUT_MS: Long = OtaTimeouts.millis(OtaStage.FW_PUBLISH, false)
        public val FW_TIMEOUT_FC_MS: Long = OtaTimeouts.millis(OtaStage.FW_PUBLISH, true)
    }
}
