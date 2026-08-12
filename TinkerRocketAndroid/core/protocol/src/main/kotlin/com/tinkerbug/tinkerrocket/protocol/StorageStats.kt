package com.tinkerbug.tinkerrocket.protocol

/**
 * Decoded flash-space stats for the storage bar.  Two frames arrive on the
 * file_ops characteristic, each behind a discriminator byte:
 *   0xCC  [RocketStorageStats]       (rocket NAND flight log — direct rocket link)
 *   0xCD  [BaseStationStorageStats]  (base station's own filesystem — BS link)
 * Wire layouts mirror RocketStorageStatsData / BaseStationStorageStatsData
 * in RocketComputerTypes.h.
 */

/**
 * Rocket NAND flight-log usage, in 256 KB blocks (wire = 15 bytes, LE packed):
 *   [0..1] u16 flight_region_blocks  [2..3] u16 used  [4..5] u16 free
 *   [6..7] u16 bad  [8..9] u16 system  [10..11] u16 flight_count
 *   [12..13] u16 block_size_kb  [14] u8 flags(bit0=initialized, bit1=auto-evicted)
 */
public data class RocketStorageStats(
    val flightRegionBlocks: Int,
    val usedBlocks: Int,
    val freeBlocks: Int,
    val badBlocks: Int,
    val systemBlocks: Int,
    val flightCount: Int,
    val blockSizeKb: Int,
    val initialized: Boolean,
    /**
     * #315: the rolling-buffer auto-evict has deleted at least one oldest
     * flight this session to reclaim space at arm time
     * (RSS_FLAG_AUTO_EVICTED, bit1).
     */
    val autoEvicted: Boolean,
) {
    private val blockBytes: Long get() = blockSizeKb.toLong() * 1024

    /** Full managed chip = flight region + fixed system overhead. */
    public val totalBytes: Long get() = (flightRegionBlocks + systemBlocks) * blockBytes
    public val usedBytes: Long get() = usedBlocks * blockBytes

    /** Reserved = bad blocks + fixed system overhead (LFS + metadata). */
    public val reservedBytes: Long get() = (badBlocks + systemBlocks) * blockBytes
    public val freeBytes: Long get() = freeBlocks * blockBytes

    public companion object {
        public const val SIZE: Int = 15

        /** Decode the payload *after* the 0xCC discriminator byte; null if too short. */
        public fun decode(payload: ByteArray): RocketStorageStats? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            val flightRegionBlocks = b.u16()
            val usedBlocks = b.u16()
            val freeBlocks = b.u16()
            val badBlocks = b.u16()
            val systemBlocks = b.u16()
            val flightCount = b.u16()
            val blockSizeKb = b.u16()
            val flags = b.u8()
            return RocketStorageStats(
                flightRegionBlocks = flightRegionBlocks,
                usedBlocks = usedBlocks,
                freeBlocks = freeBlocks,
                badBlocks = badBlocks,
                systemBlocks = systemBlocks,
                flightCount = flightCount,
                blockSizeKb = blockSizeKb,
                initialized = (flags and 0x01) != 0,
                autoEvicted = (flags and 0x02) != 0,
            )
        }
    }
}

/**
 * Base-station filesystem usage, in bytes (wire = 26 bytes, LE packed):
 *   [0..7] u64 total  [8..15] u64 used  [16..23] u64 free
 *   [24] u8 backend(0=SPIFFS, 1=SD, 2=ext-NAND)
 *   [25] u8 flags(bit0=mounted, bit1=fallback, bit2=retried)
 *
 * bit2 (BSS_FLAG_RETRIED — primary storage came up, but only after a failed
 * bring-up attempt) is defined on the wire and deliberately not decoded here
 * yet: nothing renders it, and a decoded-but-unread field is exactly the trap
 * `mounted` fell into for two releases.  Add it together with its consumer.
 */
public data class BaseStationStorageStats(
    val totalBytes: Long,   // u64 (read as signed 64-bit, same as iOS Int(truncatingIfNeeded:))
    val usedBytes: Long,
    val freeBytes: Long,
    val backend: Int,       // u8
    val mounted: Boolean,
    /**
     * #761: BSS_FLAG_FALLBACK — the board is fitted with primary storage (512 MB
     * NAND, or an SD slot) and its bring-up failed, so the whole session is
     * logging to the ~2 MB internal SPIFFS partition.  This is the firmware
     * saying so, rather than the app inferring it from `backend == 0`: the
     * backend field reports where logging ended up, not that somewhere better
     * was expected and lost, and only the firmware knows which board it is on.
     */
    val fallback: Boolean,
) {
    /** FS overhead the data accounting doesn't attribute to used or free. */
    public val reservedBytes: Long get() = maxOf(0L, totalBytes - usedBytes - freeBytes)

    public companion object {
        public const val SIZE: Int = 26

        /** Decode the payload *after* the 0xCD discriminator byte; null if too short. */
        public fun decode(payload: ByteArray): BaseStationStorageStats? {
            if (payload.size < SIZE) return null
            val b = LeBuffer(payload)
            val total = b.u64()
            val used = b.u64()
            val free = b.u64()
            val backend = b.u8()
            val flags = b.u8()
            return BaseStationStorageStats(
                totalBytes = total,
                usedBytes = used,
                freeBytes = free,
                backend = backend,
                mounted = (flags and 0x01) != 0,
                fallback = (flags and 0x02) != 0,
            )
        }
    }
}
