package com.tinkerbug.tinkerrocket.protocol

/**
 * Demux for the FILE_OPS characteristic — multiple binary frame kinds plus
 * two JSON shapes share it, disambiguated by the FIRST byte.  JSON always
 * starts with '{' or '[', so the binary discriminators (0xAA/0xCA..0xCD)
 * can never collide with it.  A port of the switch in iOS
 * `BLEDevice.peripheral(_:didUpdateValueFor:)` plus each branch's parse
 * helper, including the outer length gates those helpers apply BEFORE
 * handing the payload (discriminator stripped) to the shared decoders:
 *
 *   0xAA → frequency-scan result   (full frame → [FreqScanResult.decode])
 *   0xCA → mag-cal status          (gate: frame >= 23 → [MagCalStatus.decode])
 *   0xCB → sensor-cal status       (gate: frame >= 20 → [SensorCalStatus.decode])
 *   0xCC → rocket storage stats    (gate: frame >= 15 → [RocketStorageStats.decode])
 *   0xCD → BS storage stats        (gate: frame >= 27 → [BaseStationStorageStats.decode])
 *   0xCE → pyro-test refusal       (gate: frame >= 4, decoded inline)
 *   0x7B '{' → try ota_status JSON, else file-list parse (which fails on '{')
 *   anything else → file-list JSON array
 *
 * Failure behavior mirrors iOS per branch:
 *  - a malformed 0xAA frame returns [FileOpsMessage.ScanFailed] — iOS clears
 *    the scan-in-progress spinner there (#385), so the session layer needs
 *    the distinction;
 *  - every other malformed frame is silently dropped (iOS logs only) → null.
 *
 * (The 0xCC/0xCD outer gates are one byte WEAKER than the decoders need —
 * frame >= 15/27 but payload SIZE 15/26 — so a frame at exactly the gate
 * still null-drops inside decode, same as iOS.)
 */
public object FileOpsDispatch {

    public fun parse(frame: ByteArray): FileOpsMessage? {
        val first = frame.firstOrNull()?.toInt()?.and(0xFF)
        return when (first) {
            0xAA -> FreqScanResult.decode(frame)
                ?.let { FileOpsMessage.Scan(it) }
                ?: FileOpsMessage.ScanFailed
            0xCA -> {
                if (frame.size < 23) return null
                MagCalStatus.decode(frame.copyOfRange(1, frame.size))
                    ?.let { FileOpsMessage.MagCal(it) }
            }
            0xCB -> {
                if (frame.size < 20) return null
                SensorCalStatus.decode(frame.copyOfRange(1, frame.size))
                    ?.let { FileOpsMessage.SensorCal(it) }
            }
            0xCC -> {
                if (frame.size < 15) return null
                RocketStorageStats.decode(frame.copyOfRange(1, frame.size))
                    ?.let { FileOpsMessage.RocketStorage(it) }
            }
            0xCD -> {
                if (frame.size < 27) return null
                BaseStationStorageStats.decode(frame.copyOfRange(1, frame.size))
                    ?.let { FileOpsMessage.BsStorage(it) }
            }
            0xCE -> {
                if (frame.size < 4) return null
                FileOpsMessage.PyroRefusal(
                    cmd = frame[1].toInt() and 0xFF,
                    channel = frame[2].toInt() and 0xFF,
                    reason = frame[3].toInt() and 0xFF,
                )
            }
            0x7B -> {   // '{' → JSON object
                OtaStatusUpdate.parse(frame)?.let { return FileOpsMessage.Ota(it) }
                fileList(frame)
            }
            else -> fileList(frame)   // '[' or anything else (file-list arrays)
        }
    }

    private fun fileList(frame: ByteArray): FileOpsMessage? =
        FileListPage.decode(frame)?.let { FileOpsMessage.FileList(it) }
}

/** One decoded file_ops-characteristic frame. */
public sealed interface FileOpsMessage {

    /** 0xAA — base-station frequency-scan result. */
    public data class Scan(val result: FreqScanResult) : FileOpsMessage

    /**
     * A frame that CLAIMED to be a scan result (leading 0xAA) but failed to
     * decode.  iOS clears `isScanning` on this path so a garbled result
     * can't wedge the scan UI (#385) — session layers must do the same.
     */
    public data object ScanFailed : FileOpsMessage

    /** 0xCA — magnetometer hard-iron cal status (#96). */
    public data class MagCal(val status: MagCalStatus) : FileOpsMessage

    /** 0xCB — gyro + high-g sensor cal readback (#132). */
    public data class SensorCal(val status: SensorCalStatus) : FileOpsMessage

    /** 0xCC — rocket NAND flight-log storage stats. */
    public data class RocketStorage(val stats: RocketStorageStats) : FileOpsMessage

    /** 0xCD — base-station filesystem storage stats. */
    public data class BsStorage(val stats: BaseStationStorageStats) : FileOpsMessage

    /**
     * 0xCE — rail-off pyro-test refusal: the OC refused to queue a pyro
     * test ([cmd] 35 continuity / 36 fire, [channel] 1..4) because the FC
     * power rail is off — a held pyro command would deliver its fire/ARM
     * pulse at the next power-on.  [reason] 1 = FC rail off.
     */
    public data class PyroRefusal(
        val cmd: Int,
        val channel: Int,
        val reason: Int,
    ) : FileOpsMessage

    /** '{' JSON with `"type":"ota_status"` (#8 phase 2). */
    public data class Ota(val status: OtaStatusUpdate) : FileOpsMessage

    /** JSON array — one page of the device file list. */
    public data class FileList(val page: FileListPage) : FileOpsMessage
}
