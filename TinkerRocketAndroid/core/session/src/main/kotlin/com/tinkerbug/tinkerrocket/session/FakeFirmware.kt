package com.tinkerbug.tinkerrocket.session

import com.tinkerbug.tinkerrocket.protocol.BleCommandId
import com.tinkerbug.tinkerrocket.protocol.LeBuffer
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.launch

/**
 * Scripted [BleTransport] modeling an OC / BS peripheral for virtual-time
 * session tests — the "device side" of the seam, so DeviceSession's connect
 * choreography, demux routing, and download engine run against deterministic,
 * configurable firmware behavior.
 *
 * Behavior modeled (mirroring the real firmware paths the iOS app exercises):
 *  - CCCD enables gate notification delivery per characteristic (a
 *    notification to an un-subscribed characteristic is dropped, like BLE);
 *  - cmd 20 answers with canned config + config_pyro + config_identity JSON
 *    (each individually nullable) plus an optional fc_identity and the OC's
 *    connect-time guid_target echo push;
 *  - cmd 28 echoes guid_target with seq bump + rc rules (accept → GEO_ACTIVE
 *    + f32-rounded coordinates; reject → rc set, target state untouched);
 *  - cmd 2 stages a 5-per-page file-list JSON for the EXPLICIT read that
 *    iOS/DeviceSession issue 0.5 s later (NOT a notification);
 *  - cmd 4 serves file bytes as `[offset u32][len u16][flags u8][data]`
 *    chunk notifications with EOF, with scripted failure modes (dropped
 *    chunk, duplicate EOF, EOF|ABORT, legacy no-EOF, mid-transfer stall,
 *    scrambled offset fields);
 *  - cmd 50 emits a scripted mag-cal ladder of 0xCA frames;
 *  - cmd 60 emits a scripted 0xAA scan result;
 *  - cmd 8 optionally answers with a pwr_pin_on telemetry frame.
 *
 * Every reply has a per-op latency knob so tests can order events
 * deterministically under runTest virtual time.  All GATT ops are recorded
 * in [ops] (plus raw command frames in [commandFrames]) for choreography
 * ORDER assertions.
 */
public class FakeFirmware(
    /** Scope for delayed replies — the test's backgroundScope. */
    private val scope: CoroutineScope,
) : BleTransport {

    // ── Op recording ─────────────────────────────────────────────────────

    /**
     * Every transport op in call order, formatted:
     * `connect`, `mtu:<target>`, `cccd:<CHAR>`, `write:<CHAR>:<first byte>`,
     * `read:<CHAR>`, `rssi`, `disconnect`.
     */
    public val ops: MutableList<String> = mutableListOf()

    /** Raw frames written to the COMMAND characteristic, in order. */
    public val commandFrames: MutableList<ByteArray> = mutableListOf()

    /** cmd-45 focus pins received, in order. */
    public val focusPins: MutableList<Int> = mutableListOf()

    /** Notifications dropped because the characteristic had no CCCD enable. */
    public var droppedNotifications: Int = 0
        private set

    // ── Script knobs ─────────────────────────────────────────────────────

    /** MTU granted on requestMtu. */
    public var mtu: Int = 517

    /** RSSI returned by readRssi(). */
    public var rssi: Int = -50

    // cmd 20 replies (each nullable = "device doesn't send it").
    public var configReplyDelayMs: Long = 0
    public var configJson: String? = DEFAULT_CONFIG_JSON
    public var configPyroJson: String? = DEFAULT_CONFIG_PYRO_JSON
    public var configIdentityJson: String? = DEFAULT_IDENTITY_JSON
    public var fcIdentityJson: String? = null

    /** OC pushes the guid_target echo in the connect-time readback (#435). */
    public var pushEchoWithConfig: Boolean = true

    // Guidance-echo state (the FC-authoritative aim point).
    public var echoSeq: Int = 5
    public var echoStatus: Int = 0        // GUID_TGT_NONE
    public var echoLastRc: Int = 0        // GUID_RC_NONE
    public var echoLat: Double = 0.0
    public var echoLon: Double = 0.0
    public var echoAltM: Int = 0
    public var echoReplyDelayMs: Long = 0

    /** rc for the NEXT processed cmd 28 (1 = ACCEPTED; 2..6 = reject codes). */
    public var guidanceRc: Int = 1

    /** false models pre-#435 firmware: cmd 28 is silently ignored. */
    public var respondToGuidancePoint: Boolean = true

    // Files.
    public val deviceFiles: MutableList<FakeFile> = mutableListOf()
    public var downloadScript: DownloadScript = DownloadScript()

    // Mag-cal ladder served on cmd 50 (raw 0xCA frames, in order).
    public var magCalLadder: List<ByteArray> = emptyList()
    public var magCalFrameDelayMs: Long = 0

    // Frequency scan reply served on cmd 60.
    public var scanResponse: ByteArray? = null
    public var scanReplyDelayMs: Long = 0

    /** Non-null: cmd 8 answers with a pwr_pin_on telemetry frame after this delay. */
    public var powerToggleReplyDelayMs: Long? = null

    // ── Transport implementation ─────────────────────────────────────────

    private val _events = MutableSharedFlow<TransportEvent>(extraBufferCapacity = 1024)
    override val events: Flow<TransportEvent> = _events

    private val notifying = mutableSetOf<TrCharacteristic>()
    private var stagedFileOpsRead: ByteArray = ByteArray(0)

    override suspend fun connect() {
        ops += "connect"
    }

    override suspend fun requestMtu(target: Int): Int {
        ops += "mtu:$target"
        check(_events.tryEmit(TransportEvent.MtuChanged(mtu)))
        return mtu
    }

    override suspend fun enableNotifications(char: TrCharacteristic) {
        ops += "cccd:${char.name}"
        notifying += char
    }

    override suspend fun write(char: TrCharacteristic, bytes: ByteArray, withResponse: Boolean) {
        ops += "write:${char.name}:${bytes.firstOrNull()?.toInt()?.and(0xFF) ?: -1}"
        if (char == TrCharacteristic.FILE_TRANSFER) {
            if (chunkWriteDelayMs > 0) kotlinx.coroutines.delay(chunkWriteDelayMs)
            otaChunks += bytes.copyOf()
            // Scripted mid-pump rejection: real firmware answers a bad chunk
            // with verify_failed while the pump is still running.
            failOtaAtChunk?.let { n ->
                if (otaChunks.size >= n) {
                    failOtaAtChunk = null
                    emitOtaStatus("verify_failed", err = failOtaError)
                }
            }
            return
        }
        if (char == TrCharacteristic.COMMAND && bytes.isNotEmpty()) {
            commandFrames += bytes.copyOf()
            handleCommand(bytes[0].toInt() and 0xFF, bytes.copyOfRange(1, bytes.size))
        }
    }

    /** Raw OTA chunk frames written to FILE_TRANSFER, in order. */
    public val otaChunks: MutableList<ByteArray> = mutableListOf()

    /** Emit verify_failed once this many chunks have landed (null = never). */
    public var failOtaAtChunk: Int? = null
    public var failOtaError: String = "bad_offset"

    /**
     * Per-chunk write cost.  0 keeps tests fast, but a real link takes a
     * millisecond or two per write — set this when a test needs status
     * notifications to interleave with the pump the way they do on air.
     */
    public var chunkWriteDelayMs: Long = 0

    /** Emit an `ota_status` notification on the file-ops characteristic. */
    public fun emitOtaStatus(
        state: String,
        bytes: Long = 0,
        err: String? = null,
        fw: String? = null,
    ) {
        val sb = StringBuilder("""{"type":"ota_status","state":"""")
        sb.append(state).append('"').append(",\"bytes\":").append(bytes)
        if (err != null) sb.append(",\"err\":\"").append(err).append('"')
        if (fw != null) sb.append(",\"fw\":\"").append(fw).append('"')
        sb.append('}')
        emitFileOpsFrame(sb.toString().toByteArray())
    }

    /** Connection-priority calls the OTA pump makes, for assertions. */
    public val priorityCalls: MutableList<String> = mutableListOf()

    override fun requestConnectionPriorityHigh() {
        priorityCalls += "high"
    }

    override fun releaseConnectionPriority() {
        priorityCalls += "release"
    }

    override suspend fun read(char: TrCharacteristic): ByteArray {
        ops += "read:${char.name}"
        return if (char == TrCharacteristic.FILE_OPS) stagedFileOpsRead else ByteArray(0)
    }

    override suspend fun readRssi(): Int {
        ops += "rssi"
        return rssi
    }

    override fun disconnect() {
        ops += "disconnect"
        fireDisconnect()
    }

    // ── Test controls ────────────────────────────────────────────────────

    /** Emit a Disconnected event (link drop / device reboot). */
    public fun fireDisconnect(reason: Int = 0) {
        check(_events.tryEmit(TransportEvent.Disconnected(reason)))
    }

    /** Emit a raw notification, gated on the characteristic's CCCD. */
    public fun emitNotification(char: TrCharacteristic, bytes: ByteArray) {
        if (char !in notifying) {
            droppedNotifications += 1
            return
        }
        check(_events.tryEmit(TransportEvent.Notification(char, bytes)))
    }

    public fun emitTelemetryJson(json: String): Unit =
        emitNotification(TrCharacteristic.TELEMETRY, json.encodeToByteArray())

    public fun emitFileOpsFrame(frame: ByteArray): Unit =
        emitNotification(TrCharacteristic.FILE_OPS, frame)

    public fun emitFileTransferFrame(frame: ByteArray): Unit =
        emitNotification(TrCharacteristic.FILE_TRANSFER, frame)

    /**
     * Emit a minimal live telemetry frame ("fs" packs pwr_pin_on at bit 4;
     * "rid"/"run" mark a BS-relayed frame; "ds"/"age" the #95 freshness).
     */
    public fun emitTelemetry(
        state: String = "READY",
        pwrPinOn: Boolean = false,
        sourceRocketId: Int? = null,
        sourceUnitName: String? = null,
        dataStatusRaw: Int? = null,
        dataAgeMs: Long? = null,
    ) {
        val sb = StringBuilder("""{"st":"$state","fs":${if (pwrPinOn) 0x10 else 0}""")
        sourceRocketId?.let { sb.append(""","rid":$it""") }
        sourceUnitName?.let { sb.append(""","run":"$it"""") }
        dataStatusRaw?.let { sb.append(""","ds":$it""") }
        dataAgeMs?.let { sb.append(""","age":$it""") }
        sb.append("}")
        emitTelemetryJson(sb.toString())
    }

    // ── Frame builders ───────────────────────────────────────────────────

    /** `[offset u32 LE][len u16 LE][flags u8][data]` file-transfer chunk. */
    public fun chunkFrame(offset: Long, data: ByteArray, flags: Int): ByteArray {
        val out = ByteArray(7 + data.size)
        out[0] = offset.toByte()
        out[1] = (offset ushr 8).toByte()
        out[2] = (offset ushr 16).toByte()
        out[3] = (offset ushr 24).toByte()
        out[4] = data.size.toByte()
        out[5] = (data.size ushr 8).toByte()
        out[6] = flags.toByte()
        data.copyInto(out, 7)
        return out
    }

    /** `[0xCA][22-byte legacy MagCalStatusData payload]` frame. */
    public fun magCalStatusFrame(
        subType: Int,
        sampleCount: Int = 0,
        coverageBins: Int = 0,
        rejectCode: Int = 0,
        instFieldUtX10: Int = 0,
    ): ByteArray {
        val p = ByteArray(23)
        p[0] = 0xCA.toByte()
        // payload[0..3] time_us = 0
        p[1 + 4] = subType.toByte()
        p[1 + 5] = coverageBins.toByte()
        p[1 + 6] = sampleCount.toByte()
        p[1 + 7] = (sampleCount ushr 8).toByte()
        p[1 + 8] = instFieldUtX10.toByte()
        p[1 + 9] = (instFieldUtX10 ushr 8).toByte()
        // offsets / R / residual left 0
        p[1 + 20] = rejectCode.toByte()
        return p
    }

    /** `[0xAA][start f32][step f32][n u8][rssi i8 × n]` scan-result frame. */
    public fun scanFrame(startMhz: Float, stepKhz: Float, rssiDbm: List<Int>): ByteArray {
        val out = ByteArray(10 + rssiDbm.size)
        out[0] = 0xAA.toByte()
        writeF32(out, 1, startMhz)
        writeF32(out, 5, stepKhz)
        out[9] = rssiDbm.size.toByte()
        for ((i, r) in rssiDbm.withIndex()) out[10 + i] = r.toByte()
        return out
    }

    private fun writeF32(out: ByteArray, at: Int, v: Float) {
        val bits = v.toRawBits()
        out[at] = bits.toByte()
        out[at + 1] = (bits ushr 8).toByte()
        out[at + 2] = (bits ushr 16).toByte()
        out[at + 3] = (bits ushr 24).toByte()
    }

    // ── Command handling ─────────────────────────────────────────────────

    private fun handleCommand(cmd: Int, payload: ByteArray) {
        when (cmd) {
            BleCommandId.REQUEST_CONFIG -> scope.launch {
                if (configReplyDelayMs > 0) delay(configReplyDelayMs)
                configJson?.let { emitTelemetryJson(it) }
                configPyroJson?.let { emitTelemetryJson(it) }
                configIdentityJson?.let { emitTelemetryJson(it) }
                fcIdentityJson?.let { emitTelemetryJson(it) }
                if (pushEchoWithConfig) emitGuidEcho()
            }

            BleCommandId.FILE_LIST -> {
                val page = payload.firstOrNull()?.toInt()?.and(0xFF) ?: 0
                stagedFileOpsRead = fileListPageJson(page).encodeToByteArray()
            }

            BleCommandId.FILE_DOWNLOAD -> serveDownload(payload.decodeToString())

            BleCommandId.GUIDANCE_POINT -> handleGuidancePoint(payload)

            BleCommandId.SET_FOCUS_ROCKET_BS ->
                focusPins += (payload.firstOrNull()?.toInt()?.and(0xFF) ?: 0)

            BleCommandId.MAG_CAL_START_OC -> scope.launch {
                for (frame in magCalLadder) {
                    if (magCalFrameDelayMs > 0) delay(magCalFrameDelayMs)
                    emitFileOpsFrame(frame)
                }
            }

            BleCommandId.FREQ_SCAN_BS -> scope.launch {
                if (scanReplyDelayMs > 0) delay(scanReplyDelayMs)
                scanResponse?.let { emitFileOpsFrame(it) }
            }

            BleCommandId.POWER_TOGGLE -> powerToggleReplyDelayMs?.let { ms ->
                scope.launch {
                    if (ms > 0) delay(ms)
                    emitTelemetry(pwrPinOn = true)
                }
            }

            else -> Unit
        }
    }

    private fun handleGuidancePoint(payload: ByteArray) {
        if (!respondToGuidancePoint) return
        if (payload.size < 20) return
        val b = LeBuffer(payload)
        val lat = b.f64()
        val lon = b.f64()
        val alt = b.f32()
        // The FC bumps seq on EVERY processed cmd 28, accept or reject.
        echoSeq += 1
        echoLastRc = guidanceRc
        if (guidanceRc == 1) {   // GUID_RC_ACCEPTED
            echoStatus = 1       // GUID_TGT_GEO_ACTIVE
            // The wire echo is f32-rounded (ulp ~0.4 m — see GuidanceTargetEcho).
            echoLat = lat.toFloat().toDouble()
            echoLon = lon.toFloat().toDouble()
            echoAltM = alt.toInt()
        }
        scope.launch {
            if (echoReplyDelayMs > 0) delay(echoReplyDelayMs)
            emitGuidEcho()
        }
    }

    private fun emitGuidEcho() {
        emitTelemetryJson(
            """{"type":"guid_target","seq":$echoSeq,"st":$echoStatus,"rc":$echoLastRc,""" +
                """"lat":$echoLat,"lon":$echoLon,"alt":$echoAltM}""",
        )
    }

    private fun fileListPageJson(page: Int): String {
        val slice = deviceFiles.drop(page * FILES_PER_PAGE).take(FILES_PER_PAGE)
        return slice.joinToString(prefix = "[", postfix = "]", separator = ",") {
            """{"name":"${it.name}","size":${it.size}}"""
        }
    }

    private fun serveDownload(name: String) {
        val file = deviceFiles.firstOrNull { it.name == name }
        val script = downloadScript
        scope.launch {
            if (script.firstChunkDelayMs > 0) delay(script.firstChunkDelayMs)
            if (file == null) {
                // Unknown file: the firmware aborts the transfer (#526).
                emitFileTransferFrame(chunkFrame(0, ByteArray(0), FLAG_EOF or FLAG_ABORT))
                return@launch
            }
            val chunks = file.bytes.toList().chunked(script.chunkSize).map { it.toByteArray() }
            var offset = 0L
            for ((i, chunk) in chunks.withIndex()) {
                script.stallAfterChunks?.let { if (i >= it) return@launch }  // wedged forever
                script.abortAfterChunks?.let {
                    if (i >= it) {
                        if (script.interChunkDelayMs > 0) delay(script.interChunkDelayMs)
                        emitFileTransferFrame(
                            chunkFrame(offset, ByteArray(0), FLAG_EOF or FLAG_ABORT),
                        )
                        return@launch
                    }
                }
                if (i > 0 && script.interChunkDelayMs > 0) delay(script.interChunkDelayMs)
                if (script.dropChunkIndex == i) {
                    offset += chunk.size   // lost in transit; later offsets stay truthful
                    continue
                }
                val isLast = i == chunks.lastIndex
                val flags = if (isLast && !script.legacyNoEof) FLAG_EOF else 0
                val offField = if (script.scrambleOffsets) 0L else offset
                emitFileTransferFrame(chunkFrame(offField, chunk, flags))
                offset += chunk.size
            }
            if (script.duplicateEof && !script.legacyNoEof) {
                emitFileTransferFrame(chunkFrame(offset, ByteArray(0), FLAG_EOF))
            }
        }
    }

    // ── Nested types / canned data ───────────────────────────────────────

    /** One on-device file; [reportedSize] overrides the list's "size" field. */
    public class FakeFile(
        public val name: String,
        public val bytes: ByteArray,
        public val reportedSize: Long? = null,
    ) {
        public val size: Long get() = reportedSize ?: bytes.size.toLong()
    }

    /**
     * cmd-4 serving script.  All failure modes compose with the chunking:
     *  - [dropChunkIndex]: that chunk is never sent (lost in transit);
     *  - [duplicateEof]: a second zero-length EOF frame follows the last chunk;
     *  - [abortAfterChunks]: after N data chunks, an EOF|ABORT frame (#526);
     *  - [legacyNoEof]: no EOF flag ever — the client's stall timer completes;
     *  - [stallAfterChunks]: transfer wedges after N chunks, never resumes;
     *  - [scrambleOffsets]: every offset field is 0 — pins the client's
     *    append-in-ARRIVAL-order behavior (offsets are never used).
     */
    public data class DownloadScript(
        val chunkSize: Int = 100,
        val firstChunkDelayMs: Long = 0,
        val interChunkDelayMs: Long = 0,
        val dropChunkIndex: Int? = null,
        val duplicateEof: Boolean = false,
        val abortAfterChunks: Int? = null,
        val legacyNoEof: Boolean = false,
        val stallAfterChunks: Int? = null,
        val scrambleOffsets: Boolean = false,
    )

    public companion object {
        public const val FILES_PER_PAGE: Int = 5
        public const val FLAG_EOF: Int = 0x01
        public const val FLAG_ABORT: Int = 0x02

        public const val DEFAULT_CONFIG_JSON: String =
            """{"type":"config","sb1":0,"shz":333,"smn":1000,"smx":2000,""" +
                """"kp":0.08,"ki":0.005,"kd":0.003,"pmn":-10.0,"pmx":10.0,""" +
                """"sen":true,"gs":true,"ac":false,"rdly":0,"rmspd":0.0,"rcap":60.0,"kpang":2.0,""" +
                """"iwind":40.0,"ge":false,"camt":2,"irate":1920,"lf":915.0,"lsf":8,""" +
                """"lbw":250.0,"lcr":5,"lpw":17,"lhd":false,"lhdw":400,"ltxd":false}"""

        public const val DEFAULT_CONFIG_PYRO_JSON: String =
            """{"type":"config_pyro","p1e":true,"p1m":0,"p1v":1.0,"p2e":false,"p2m":1,""" +
                """"p2v":100.0,"p3e":false,"p3m":0,"p3v":0.0,"p4e":false,"p4m":0,"p4v":0.0}"""

        public const val DEFAULT_IDENTITY_JSON: String =
            """{"type":"config_identity","uid":"a1b2c3d4","un":"Atlas","nid":1,"rid":3,""" +
                """"dt":"R","fw":"abc1234+20260722-0900"}"""
    }
}
