#ifndef TR_LOG_TO_FLASH_H
#define TR_LOG_TO_FLASH_H

#include <compat.h>
#include "nand_geometry.h"
#include <RocketComputerTypes.h>
#include "lfs.h"
#include "mram_dirty_policy.h"
#include "bad_block_scan_policy.h"
#include "flush_iter_ledger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

struct TR_LogToFlashConfig
{
    int nand_cs = -1;
    uint32_t spi_hz_nand = 40'000'000;
    uint8_t spi_mode_nand = SPI_MODE0;
    uint32_t ring_buffer_size = 65536;  // RAM ring buffer size (bytes)
    bool debug = false;

    // LittleFS block-count. Defaults to the full chip for backward compat.
    // When sharing the NAND with TR_FlightLog (issue #50 Stage 2c) the caller
    // shrinks this to just the blocks LFS owns (32 on both firmwares; the
    // BYTE size is 32 x the runtime block size — 8 MB on the legacy part,
    // 4 MB on the 128 KB-block GD5F parts) so the flight-log allocator can
    // safely use the remainder.
    uint32_t lfs_block_count = 0;  // 0 = full chip (runtime block count)

    // Optional hot-path write override (issue #50 Stage 2c-3c). When set,
    // the flush task calls `write_sink(write_sink_ctx, payload, len)` for
    // each page drained from the ring instead of lfs_file_write. `len` is
    // fixed at (runtime page size - 16) — sinkPayloadSize(), 4080 on the
    // legacy 4 KB part and 2032 on the 2 KB GD5F parts (#671) — so the sink
    // can prepend a 16-byte TR_FlightLog PageHeader and still land on a NAND
    // page boundary. The
    // sink should return true on success; false drops the page (same as an
    // LFS write failure). When the sink is set, periodic lfs_file_sync calls
    // are suppressed — each page is self-describing via its PageHeader CRC.
    bool (*write_sink)(void* ctx, const uint8_t* payload, size_t payload_len) = nullptr;
    void* write_sink_ctx = nullptr;

    // Optional periodic hook called once per flushTaskLoop iteration on the
    // flush task's core. Used to drive deferred work that needs to run on
    // Core 0 instead of the original requesting task — specifically, the
    // TR_FlightLog deferred prepareFlight (issue #77). The hook fires after
    // the prepareLogFile request is processed and before the startLogging
    // request is processed, so a deferred prepareFlight completes between
    // file_open=true and logging_active=true.
    void (*flush_task_hook)(void* ctx) = nullptr;
    void* flush_task_hook_ctx = nullptr;

    // MRAM ring buffer (optional — set mram_cs >= 0 to enable).
    // When enabled the ring buffer lives in SPI MRAM instead of ESP32 RAM,
    // providing larger capacity (128 KB) and power-loss survivability.
    int mram_cs = -1;                   // chip-select pin (-1 = use RAM ring)

    // #822: when MRAM is absent, prefer a ring this large in SPI PSRAM instead
    // of `ring_buffer_size` in internal RAM. 0 = don't try PSRAM (the default,
    // and what every MRAM-fitted board passes). The two sizes are separate on
    // purpose: if the PSRAM allocation fails — part absent, CONFIG_SPIRAM off,
    // fragmentation — begin() falls back to `ring_buffer_size` from INTERNAL
    // RAM, and retrying a PSRAM-sized ring there would just fail again on a
    // chip with 512 KB of SRAM total.
    uint32_t psram_ring_size = 0;
    uint32_t spi_hz_mram = 40'000'000;
    uint8_t spi_mode_mram = SPI_MODE0;
    uint32_t mram_size = 131072;        // 128 KB (MR25H10)

    // #274: absolute MRAM address of the 4-byte sink-mode "dirty" marker, in the
    // caller's reserved region above mram_size. 0 = disabled. When set, markDirty
    // writes a magic here instead of the LittleFS marker (no superblock churn), so
    // a dirty boot is detected and the surviving ring replayed through the sink.
    uint32_t dirty_marker_addr = 0;
};

struct TR_LogToFlashStats
{
    // Ring capacity in bytes (MRAM region when MRAM is wired, else RAM ring).
    // Reported alongside ring_fill / ring_highwater so callers can derive the
    // actual utilization without knowing which backing is active — useful for
    // deciding whether the current 1 Mbit MRAM is over-provisioned.
    uint32_t ring_size = 0;
    uint32_t ring_fill = 0;
    uint32_t ring_highwater = 0;
    uint32_t ring_overruns = 0;
    uint32_t ring_drop_oldest_bytes = 0;
    uint32_t ring_bad_sof_clears = 0;   // Issue #46: count of clearRing fires from
                                        //   the drop-oldest path due to a corrupt
                                        //   tail (bad SOF or truncated frame).
                                        //   Distinct from happy-path drop-oldest.
    uint64_t bytes_received = 0;
    uint32_t frames_received = 0;
    uint32_t frames_dropped = 0;
    uint64_t bytes_written_nand = 0;
    uint32_t nand_prog_fail = 0;
    uint32_t nand_erase_fail = 0;
    uint32_t nand_prog_ops = 0;
    uint32_t nand_erase_ops = 0;
    bool logging_active = false;
    uint32_t nand_page = 0;
    uint32_t nand_block = 0;

    // Interval-peak wall times for slow NAND/LFS operations (µs).
    // Useful for catching the cause of a multi-hundred-ms stall — each of
    // these is wrapped around its underlying call and the max seen since
    // the last resetIntervalTimings() is reported here.
    uint32_t write_max_us = 0;         // max lfs_file_write duration
    uint32_t sync_max_us = 0;          // max lfs_file_sync duration
    uint32_t erase_max_us = 0;         // max lfsBlockErase (NAND block erase) duration
    uint32_t open_max_us = 0;          // max lfs_file_open duration
    uint32_t close_max_us = 0;         // max lfs_file_close duration
    uint32_t activate_max_us = 0;      // max activateLogging() wall time
    uint32_t clear_ring_max_us = 0;    // max clearRing() wall time
    uint32_t flush_iter_max_us = 0;    // max single flushTaskLoop iteration
    // #398: shared NAND/MRAM SPI-bus mutex contention.  spi_wait is the max
    // time any task spent BLOCKED acquiring the mutex (the I2S parser's MRAM
    // pushes starving behind flush-side work shows up here); spi_hold is the
    // max single hold.  Together they pinpoint which window causes the
    // multi-second parser_max stalls.
    uint32_t spi_wait_max_us = 0;      // max time blocked acquiring spi_mutex_
    uint32_t spi_hold_max_us = 0;      // max single spi_mutex_ hold
    // #510: per-report-window SUMS complementing the maxima above — a burst
    // of sub-threshold ops (e.g. the activation ring drain's ~24 page
    // writes) is invisible to a max but shows up here.
    uint32_t drain_pages = 0;          // pages shipped to NAND this window
    uint32_t drain_bytes = 0;          // ring bytes drained this window
    uint32_t pop_sum_us = 0;           // summed MRAM ringPop wall time
    uint32_t write_sum_us = 0;         // summed write_sink/lfs_file_write time
    uint32_t syncs_performed = 0;      // cumulative lfs_file_sync calls since begin()

    // Persistent bad-block avoidance (#47): once a NAND block is known bad,
    // the LFS callbacks short-circuit with LFS_ERR_CORRUPT in ~µs instead of
    // paying the several-hundred-ms LFS remap cost every encounter.
    uint32_t known_bad_blocks = 0;     // blocks marked bad in the persistent bitmap
    uint32_t bad_block_skips = 0;      // cumulative short-circuits since begin()
};

struct TR_LogToFlashRecoveryInfo
{
    bool recovered = false;
    uint32_t recovered_bytes = 0;
    char filename[64] = {};
};

struct TR_LogFileInfo
{
    char filename[64] = {};
    uint32_t size_bytes = 0;
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    bool has_timestamp = false;  // Flag to indicate if timestamp is valid
};

class TR_LogToFlash
{
public:
    TR_LogToFlash();

    bool begin(SPIClass& spi, const TR_LogToFlashConfig& cfg);
    bool enqueueFrame(const uint8_t* frame, size_t len);

    // --- Raw NAND access (bridge for TR_FlightLog, issue #50 Stage 2) --------
    // These expose the private nand* primitives as full-page ops keyed
    // by (block, page_in_block); "full page" is the RUNTIME page size
    // (nandGeometry().page_size — 4096 on the V8 bench part, 2048 on the
    // GD5F parts), and caller buffers must be at least that large. They share the same SPI bus + bad-block
    // bitmap as the LFS-backed flush path, so both layers stay consistent.
    // Intended only for the TR_NandBackend_esp adapter; internal callers keep
    // using the rowPageAddr variants.
    bool readNandPage(uint32_t block, uint32_t page_in_block, uint8_t* out);
    bool programNandPage(uint32_t block, uint32_t page_in_block, const uint8_t* data);
    bool eraseNandBlock(uint32_t block);
    bool isNandBlockBad(uint32_t block) const;
    bool markNandBlockBad(uint32_t block);

    // Bytes drained from the ring into the current flight so far (reset at
    // openLogSession, incremented as pages are pushed to the sink/LFS).
    uint32_t currentFileBytes() const { return current_file_bytes; }

    // Bytes drained during the most recently closed session. Captured by
    // closeLogSession just before current_file_bytes is reset, so callers
    // that finalize the flight on a later flush-task iteration (after the
    // close runs) can still recover the final byte count.
    uint32_t lastClosedSessionBytes() const { return last_closed_session_bytes_; }
    void startLogging();
    void endLogging(); // request close after buffered data is flushed
    void prepareLogFile();  // Pre-create log file (call during PRELAUNCH to avoid NAND stall at launch)
    void service();
    void startFlushTask(uint8_t core = 0, uint32_t stackSize = 8192, uint8_t priority = 1);

    void getStats(TR_LogToFlashStats& out) const;
    /// Zero the *_max_us fields so the next getStats() reflects only peaks
    /// seen since this call.  Intended to be called right after the caller
    /// has read and logged the interval stats.
    void resetIntervalTimings();
    void getRecoveryInfo(TR_LogToFlashRecoveryInfo& out) const;
    size_t listFiles(TR_LogFileInfo* out, size_t max_files) const;
    bool readFileChunk(const char* filename,
                       uint32_t offset,
                       uint8_t* out,
                       size_t max_len,
                       size_t& out_len,
                       bool& eof);
    bool deleteFile(const char* filename);
    bool formatFilesystem();  // WARNING: Erases all files!
    const char* currentFilename() const;
    bool isLoggingActive() const;
    void setFileTimestamp(const char* filename, uint16_t year, uint8_t month, uint8_t day,
                          uint8_t hour, uint8_t minute, uint8_t second);

    // ─── Raw MRAM access (for clients reserving a region outside the ring) ──
    // Used by the FlightSnapshot store (#104 follow-up): caller carves out
    // a region above ring_size_ and uses these to read/write directly without
    // the ring's modulo wrapping.  Returns false if MRAM is not enabled.
    // Acquires the SPI bus mutex internally — safe to call from any task.
    bool mramRawWrite(uint32_t addr, const uint8_t* data, uint32_t len);
    bool mramRawRead(uint32_t addr, uint8_t* out, uint32_t len);
    bool isMramEnabled() const { return use_mram_; }
    // #826: true when cfg.mram_cs named a pin but no MRAM answered on it, so
    // the ring silently fell back to RAM. Distinct from !isMramEnabled(),
    // which is also true on a board that correctly has no MRAM (mram_cs < 0).
    // Callers should surface this — it means a fitted part is dead or the
    // wrong board revision flag was built.
    bool mramProbeFailed() const { return mram_probe_failed_; }
    // #822: true when the RAM ring was allocated from SPI PSRAM rather than
    // internal RAM. Meaningless while isMramEnabled() — there is no ring_buf_.
    bool isRingInPsram() const { return ring_in_psram_; }
    // #671: the RDID-resolved chip geometry (legacy fallback until begin()).
    // TR_FlightLog consumes this through TR_NandBackend_esp; the write-sink
    // payload quantum is sinkPayloadSize() = page size - PageHeader (16 B).
    const NandGeometry& nandGeometry() const { return geom_; }
    uint32_t sinkPayloadSize() const { return geom_.page_size - 16u; }

    // #274 MRAM-ring recovery (sink mode). begin() sets "pending" if the previous
    // session was dirty and the non-volatile ring survived; the OC then opens a
    // recovered flight, calls drainMramToSink() to replay the ring through the
    // write_sink, and finishMramRecovery() to clear the ring + the dirty marker.
    bool hasPendingMramRecovery() const { return pending_mram_recovery_; }
    uint32_t drainMramToSink();   // dump the whole ring through write_sink; returns bytes sunk
    void finishMramRecovery();    // clearRing() + clear the dirty marker

private:
    // --- NAND ---
    static constexpr uint8_t NAND_RDID = 0x9F;
    static constexpr uint8_t NAND_WREN = 0x06;
    static constexpr uint8_t NAND_GETFEAT = 0x0F;
    static constexpr uint8_t NAND_SETFEAT = 0x1F;
    static constexpr uint8_t NAND_BLKERASE = 0xD8;
    static constexpr uint8_t NAND_PROGLOAD = 0x02;
    static constexpr uint8_t NAND_PROGEXEC = 0x10;
    static constexpr uint8_t NAND_PAGEREAD = 0x13;
    static constexpr uint8_t NAND_READCACHE = 0x03;

    static constexpr uint8_t FEAT_STAT = 0xC0;
    static constexpr uint8_t FEAT_PROT = 0xA0;
    static constexpr uint8_t STAT_OIP = 0x01;
    static constexpr uint8_t STAT_PFAIL = 0x08;
    static constexpr uint8_t STAT_EFAIL = 0x04;

    // #671: geometry is RUNTIME, resolved from RDID in nandInit() — see
    // nand_geometry.h for the part table and the legacy fallback. geom_ is
    // valid from nandInit() on; everything before that (ring setup, NVS
    // bitmap load) must not do geometry arithmetic. The old compile-time
    // NAND_PAGE_SIZE/NAND_BLOCK_COUNT constants were deliberately retired so
    // stale uses fail to compile; static buffers size themselves with the
    // *_MAX constants from nand_geometry.h instead.
    NandGeometry geom_ = NAND_GEOMETRY_LEGACY;

    struct __attribute__((packed)) LogMeta
    {
        uint32_t magic;
        uint16_t next_file_index;
        uint16_t reserved;
        uint16_t crc16;
    };
    static constexpr uint32_t LOG_META_MAGIC = 0x4C4F4746; // "F G O L"
    static constexpr uint16_t LOG_META_VERSION = 2;  // Version 2 adds timestamps
    static constexpr uint32_t LOG_META_PAGE = 0;
    static constexpr uint32_t LOG_META_BLOCK = 0;

    // SPI/context
    SPIClass* spi = nullptr;
    TR_LogToFlashConfig cfg = {};
    SPISettings spi_nand;

    // Ring buffer storage (RAM fallback or MRAM via SPI)
    bool use_mram_ = false;
    bool mram_probe_failed_ = false;   // #826
    uint8_t* ring_buf_ = nullptr;       // Used only when use_mram_ == false
    bool ring_in_psram_ = false;        // #822: ring_buf_ came from PSRAM
    uint32_t ring_size_ = 0;
    uint32_t rb_head = 0;
    uint32_t rb_tail = 0;
    uint32_t rb_count = 0;
    uint32_t rb_overruns = 0;
    uint32_t rb_drop_oldest_bytes = 0;
    uint32_t rb_highwater = 0;
    uint32_t rb_bad_sof_clears = 0;

    // Issue #74 diagnostic: compare total bytes pushed vs popped. A healthy
    // ring has pop <= push at all times (residual frames left unflushed at
    // end of flight). pop > push proves the same MRAM region is being drained
    // twice — i.e. a ring-pointer race is re-exposing stale prelaunch data.
    uint64_t ringpush_bytes_ = 0;
    uint64_t ringpop_bytes_ = 0;
    // Remaining per-drain diagnostic prints allowed after the most recent
    // activateLogging(). Set to 20 on activate, decremented per drain.
    uint32_t flush_log_remaining_ = 0;

    // Serializes ringPush against itself (parser and oc_loop can both push
    // from Core 1 and preempt each other on priority boundaries) and against
    // clearRing (Core 0 flush task). Without this, rb_head read-write pairs
    // race with the pointer reset in clearRing — the snapshot of rb_head
    // taken before mramWriteBytes gets written back AFTER clearRing has
    // reset it to 0, clobbering the reset with a prelaunch value.
    // See #74 bench trace (CR1 h=0 → CR2 h=50771 despite zero-sweep).
    SemaphoreHandle_t push_mutex_ = nullptr;
    // Before logging starts, cap the ring at 75% (was 50%) so drop-oldest
    // keeps ~1 s of pre-ignition history at the 1920 Hz stream (~99 KB/s)
    // while still leaving launch-transient headroom: at activation the
    // flush task drains the prelaunch backlog to NAND at ~310 KB/s against
    // ~99 KB/s inflow, so a 97 KB backlog clears in ~0.5 s and the free
    // quarter (~33 KB = ~0.3 s of inflow) absorbs flush-task stalls during
    // the drain. The historical 100-500 ms LFS stalls that motivated the
    // 50% cap no longer occur in the boost window (sink mode skips LFS,
    // the flight range is pre-erased at PRELAUNCH, the rename is deferred
    // to end-of-flight); the worst bounded stall left is a ~7 ms page
    // program plus NimBLE contention on core 0. Raised to full size once
    // logging is active (see activateLogging / closeLogSession).
    uint32_t prelaunchCap() const { return (ring_size_ / 4) * 3; }
    uint32_t ring_prelaunch_cap_ = 0;  // Set in begin() from prelaunchCap()

    // MRAM write staging (2026-07-09 bench): a per-frame ringPush costs two
    // SPI transactions (WREN + WRITE) under the bus mutex — ~325 us/frame
    // with flush-side contention — which caps ingest at ~2.1k frames/s and
    // silently dropped ~30% of the 1920 Hz stream at the DMA ISR.  Frames
    // stage in RAM (memcpy) and reach MRAM as one batched write per
    // STAGING_SIZE, or after STAGING_MAX_AGE_US via the flush task, so the
    // brownout-durability window is bounded.  MRAM path only; guarded by
    // push_mutex_ like the ring pointers.  FIFO order is preserved because
    // every producer funnels through the same staging under the same mutex.
    static constexpr uint32_t STAGING_SIZE = 2048;
    static constexpr int64_t  STAGING_MAX_AGE_US = 50000;
    uint8_t  staging_buf_[STAGING_SIZE] = {};
    uint32_t staged_len_ = 0;
    uint32_t staged_frames_ = 0;
    int64_t  staged_first_us_ = 0;
    uint32_t staging_flushes_ = 0;

    // NAND/log state
    uint32_t nand_page = 0;
    uint32_t nand_block = 0;
    uint64_t nand_bytes_written = 0;
    uint32_t nand_prog_fail = 0;
    uint32_t nand_erase_fail = 0;
    uint32_t nand_prog_ops = 0;
    uint32_t nand_erase_ops = 0;
    uint16_t file_index = 0;
    char filename[64] = {};
    // Cross-core flags (#365): logging_active is written by the Core-0 flush
    // task and read per-frame on Core 1; the request flags are set on Core 1
    // (launch/land edges, BLE) and consumed on Core 0.  volatile so neither
    // side caches a stale value.  Requests are CONSUME-ON-OBSERVE: the
    // servicing side may clear one only after reading it true — a blind
    // else-clear destroys a request that lands between the load and the
    // store (ns window per ~1 ms flush iteration), which for the start
    // request means the entire flight never logs.
    volatile bool logging_active = false;
    volatile bool start_logging_requested = false;
    volatile bool end_flight_requested = false;
    uint32_t log_start_block = 1;
    uint32_t log_curr_block = 1;
    bool log_block_erased = false;

    // page staging. Statically MAX-sized (#671): only the first
    // geom_.page_size bytes are ever staged per chunk; the flush arithmetic
    // is denominated in chunk_target, never in sizeof(page_buf).
    uint8_t page_buf[NAND_PAGE_SIZE_MAX];
    uint32_t page_buf_idx = 0;

    // Periodic sync — commits LittleFS metadata to NAND every N pages.  The
    // sync interval bounds the data-loss window on an unexpected reset: any
    // pages written since the last sync exist in NAND but are orphaned
    // because the FS metadata doesn't point at them.
    //
    // Initially tried 16 pages (0.45 s loss window) but that forced LFS to
    // commit its internal metadata cache every iteration, and each commit
    // has a large fixed cost (rewrite a metadata block) that doesn't scale
    // down — produced continuous 200-600 ms per-write stalls and doubled
    // NAND write amplification.  64 pages is the compromise: still cuts
    // the pre-patch 3.7 s loss window in half while letting LFS batch its
    // metadata commits.
    // #671: byte-denominated so the tuned cadence survives page-size changes;
    // 256 KB / page_size = 64 pages on the legacy 4 KB part (byte-identical
    // to the old SYNC_INTERVAL_PAGES = 64), 128 pages on the 2 KB GD5F parts.
    // Set in begin() from geom_. LFS path only — sink mode never syncs.
    static constexpr uint32_t SYNC_INTERVAL_BYTES = 256 * 1024;
    uint32_t sync_interval_pages_ = SYNC_INTERVAL_BYTES / NAND_GEOMETRY_LEGACY.page_size;
    uint32_t pages_since_sync_ = 0;

    // Interval-peak timing instrumentation (µs) — reset by
    // resetIntervalTimings() after each stats dump.  Any single op taking
    // longer than STALL_THRESHOLD_US is ESP_LOGW'd immediately so we can
    // spot the guilty op without having to wait for the next stats window.
    static constexpr uint64_t STALL_THRESHOLD_US = 100'000;  // 100 ms
    uint32_t write_max_us_ = 0;
    uint32_t sync_max_us_ = 0;
    uint32_t erase_max_us_ = 0;
    uint32_t open_max_us_ = 0;
    uint32_t close_max_us_ = 0;
    uint32_t activate_max_us_ = 0;
    uint32_t clear_ring_max_us_ = 0;
    uint32_t flush_iter_max_us_ = 0;
    // #398: spi_mutex_ contention peaks.  Written from multiple tasks; the
    // read-modify-write max update is unsynchronized (a lost update is
    // acceptable for diagnostics).  hold_start is only touched by the
    // current mutex holder, so it needs no extra guard.
    volatile uint32_t spi_wait_max_us_ = 0;
    volatile uint32_t spi_hold_max_us_ = 0;
    int64_t  spi_hold_start_us_ = 0;
    uint32_t syncs_performed_ = 0;

    // #510: per-iteration section ledger (reset at the top of every
    // flushTaskLoop pass) + per-report-window SUMS (reset alongside the
    // maxima above). The maxima hide a burst of sub-threshold ops — the
    // 7/14 activation drain (~24 page writes, 588 ms total) read as 11 ms.
    FlushIterLedger iter_ledger_;
    uint32_t drain_pages_win_ = 0;
    uint32_t drain_bytes_win_ = 0;
    uint32_t pop_us_win_ = 0;
    uint32_t write_us_win_ = 0;

    // Cumulative LFS-callback op counters (#47 follow-up).  Used to break
    // down *what* a slow lfs_file_write actually did — lots of reads with
    // few writes points at a lookahead scan or CTZ walk; lots of writes
    // points at metadata compaction.  The LFS callbacks each increment
    // their own counter; flushRingToNand snapshots before+after around
    // each lfs_file_write so a stall can log the delta.
    uint32_t lfs_cb_reads_  = 0;
    uint32_t lfs_cb_progs_  = 0;
    uint32_t lfs_cb_erases_ = 0;

    // --- Persistent bad-block bitmap (#47) -------------------------------
    // One bit per NAND block: 0 = unknown-or-good, 1 = known-bad.
    // Loaded from NVS at begin().  Written back at closeLogSession and by
    // persistBadBlocksIfDirty() (called only outside the hot path).
    // #671: array is MAX-sized; the live blob length is geom_.bitmapBytes()
    // (256 B legacy/V9, 128 B on the mini's 1024-block part). NVS load runs
    // BEFORE the RDID read, so it loads up to the max and the length check
    // is deferred to nandInit() where the expected size is known.
    static constexpr uint32_t BAD_BLOCK_BITMAP_BYTES_MAX = NAND_BLOCK_COUNT_MAX / 8;
    uint8_t bad_block_bitmap_[BAD_BLOCK_BITMAP_BYTES_MAX] = {};
    size_t  bad_block_blob_len_ = 0;   // bytes actually read from NVS "map"
    bool     bad_block_bitmap_dirty_ = false;
    uint32_t bad_block_skips_ = 0;   // cumulative short-circuits

    // Persisted alongside the bitmap. RDID = (MID << 8) | DID. When the
    // RDID read at boot doesn't match this, the chip has been replaced
    // (or this is a first boot under firmware that records it) and the
    // bitmap is wiped — see nandInit().
    uint16_t bad_block_chip_id_ = 0;

    // #511 boot-scan gate inputs. current_chip_id_/nand_dead_bus_ are set by
    // nandInit()'s RDID read; the other two are captured by
    // loadBadBlocksFromNVS() BEFORE nandInit() can rewrite bad_block_chip_id_
    // (the "chip" key is written pre-scan and must not be trusted as a
    // scan-completed signal — see bad_block_scan_policy.h).
    uint16_t current_chip_id_ = 0;
    bool     nand_dead_bus_ = false;
    bool     bad_block_map_blob_ok_ = false;   // NVS "map" present, exact size
    uint16_t bad_block_scanned_chip_ = 0;      // NVS "scanned" (0 = never)
    // #671: NVS "gpage" — the page size the persisted scan ran under (missing
    // key loads as 4096 = the pre-#671 scan arithmetic). Folded into
    // bad_block_map_blob_ok_ so a map scanned under the wrong geometry (old
    // firmware on a GD5F chip probed the factory-marker column at 4096, which
    // aliases to main-array byte 0 there) is rescanned, not trusted.
    uint16_t bad_block_scan_page_size_ = 0;

    bool isBlockBad(uint32_t block) const;
    void markBlockBad(uint32_t block);
    void loadBadBlocksFromNVS();
    void persistBadBlocksIfDirty();
    /// #511: record (NVS "bblk"/"scanned") that a boot scan completed for the
    /// current chip. Written strictly AFTER scanBadBlocksAtBoot() so a power
    /// cut mid-scan leaves the marker stale and the next boot rescans.
    void markBootScanComplete();
    uint32_t countBadBlocks() const;
    /// Boot-time non-destructive bad-block scan: for every block not yet in
    /// the persistent map, PAGEREAD page 0 to catch NAND-reported read
    /// errors (Option A), and read the first byte of the spare area on
    /// pages 0 and 1 to detect factory bad-block markers (Option B).
    /// Any new finds get `markBlockBad`'d and will be persisted to NVS.
    /// Returns the number of newly-discovered bad blocks; logs the total
    /// and wall time.
    uint32_t scanBadBlocksAtBoot();

    // counters
    uint64_t bytes_received = 0;
    uint32_t frames_received = 0;
    uint32_t frames_dropped = 0;
    bool     drop_warned_ = false;  // #278: one-shot loud warning on first in-flight drop

    // startup recovery
    bool recovery_performed = false;
    uint32_t recovery_bytes = 0;
    char recovery_filename[64] = {};
    bool pending_mram_recovery_ = false;  // #274: dirty sink-mode boot — ring preserved for deferred replay

    // LittleFS filesystem
    lfs_t lfs;
    lfs_file_t file;
    bool file_open = false;

    // LittleFS config and buffers (heap allocated to avoid stack overflow)
    lfs_config* lfs_cfg = nullptr;
    uint8_t* lfs_read_buffer = nullptr;
    uint8_t* lfs_prog_buffer = nullptr;
    uint8_t* lfs_lookahead_buffer = nullptr;

    // Current file tracking
    char current_filename[64] = {};
    uint32_t current_file_bytes = 0;
    // Sticky byte count from the most recently closed session (see
    // lastClosedSessionBytes()). Updated by closeLogSession before
    // current_file_bytes is reset.
    uint32_t last_closed_session_bytes_ = 0;
    bool current_file_has_timestamp = false;

    // Deferred timestamp (set from Core 1, applied by flush task on Core 0)
    volatile bool pending_timestamp_ = false;
    uint16_t pending_ts_year_ = 0;
    uint8_t pending_ts_month_ = 0;
    uint8_t pending_ts_day_ = 0;
    uint8_t pending_ts_hour_ = 0;
    uint8_t pending_ts_minute_ = 0;
    uint8_t pending_ts_second_ = 0;
    char pending_ts_filename_[64] = {};
    void applyPendingTimestamp();  // Called by flush task
    void applyPendingTimestamp_impl(const char* filename, uint16_t year, uint8_t month, uint8_t day,
                                     uint8_t hour, uint8_t minute, uint8_t second);

    // Pre-create log file support
    volatile bool prepare_file_requested_ = false;

    // SPI bus mutex — always created.  Both cores share the bus even without
    // MRAM (flush task programs NAND pages while the BLE download path reads).
    SemaphoreHandle_t spi_mutex_ = nullptr;
    void spiAcquire();
    void spiRelease();

    // MRAM SPI interface (MR25H10)
    SPISettings spi_mram;
    static constexpr uint8_t MRAM_WREN  = 0x06;
    static constexpr uint8_t MRAM_READ  = 0x03;
    static constexpr uint8_t MRAM_WRITE = 0x02;
    // #826: used only by mramProbe(). WEL is bit 1 of the status register and
    // is the one piece of MRAM state that can be changed without touching a
    // byte of memory.
    static constexpr uint8_t MRAM_WRDI  = 0x04;
    static constexpr uint8_t MRAM_RDSR  = 0x05;
    static constexpr uint8_t MRAM_SR_WEL = 0x02;
    void mramWriteBytes(uint32_t addr, const uint8_t* data, uint32_t len);
    void mramReadBytes(uint32_t addr, uint8_t* out, uint32_t len);

    // FreeRTOS flush task (Core 0)
    portMUX_TYPE ring_mux_ = portMUX_INITIALIZER_UNLOCKED;  // Spinlock for rb_head/rb_tail/rb_count
    TaskHandle_t flush_task_ = nullptr;
    volatile bool flush_task_running_ = false;
    volatile bool flush_task_stop_ = false;     // Signal flush task to exit

    static void flushTaskEntry(void* param);    // FreeRTOS task entry (static)
    void flushTaskLoop();                        // Instance method called by entry

    // helpers
    inline void csLow(int pin) { digitalWrite(pin, LOW); }
    inline void csHigh(int pin) { digitalWrite(pin, HIGH); }

    bool ringPush(const uint8_t* data, uint32_t len);
    bool ringPushLocked(const uint8_t* data, uint32_t len);  // caller holds push_mutex_
    bool flushStagingLocked();                               // caller holds push_mutex_
    void flushStagingIfStale(int64_t max_age_us);
    uint32_t ringPop(uint8_t* out, uint32_t len);
    void ringPeekAt(uint32_t offset, uint8_t* out, uint32_t len);

    // #826: confirm a real MRAM answers on cfg.mram_cs before trusting the
    // pin number. Non-destructive — see the implementation for why it cannot
    // be a scratch write/read-back.
    bool mramProbe();
    void nandWREN();
    uint8_t nandGetFeature(uint8_t addr);
    void nandSetFeature(uint8_t addr, uint8_t val);
    bool nandWaitReady(uint32_t timeout_us = 2'000'000);
    bool nandEraseBlock(uint32_t blockIndex);
    bool nandProgramPage(uint32_t rowPageAddr, const uint8_t* data, uint32_t len);
    bool nandReadPage(uint32_t rowPageAddr, uint8_t* out, uint32_t len);
    /// Read `len` bytes from `column` within a page — used to access the OOB /
    /// spare area (column ≥ runtime page size — 4096 on the legacy part, 2048
    /// on the GD5F parts) where factory bad-block markers live.  Returns
    /// false if the PAGEREAD status polls out (suspect block).
    bool nandReadBytesAt(uint32_t rowPageAddr, uint32_t column,
                         uint8_t* out, uint32_t len);
    bool nandInit();

    // LittleFS block device adapter callbacks (static to access from C API)
    static int lfsBlockRead(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, void *buffer, lfs_size_t size);
    static int lfsBlockProg(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, const void *buffer, lfs_size_t size);
    static int lfsBlockErase(const struct lfs_config *c, lfs_block_t block);
    static int lfsBlockSync(const struct lfs_config *c);

    void openLogSession();
    void activateLogging();
    void closeLogSession();
    void flushRingToNand();

    void markDirty();           // Write LittleFS marker file on log start
    void clearDirty();          // Remove LittleFS marker file on log close
    bool checkDirtyOnStartup(); // Check if previous session was dirty
    bool checkMramDirty();      // #274: read the sink-mode MRAM dirty marker

    // #417: single source of truth for the dirty-marker intent across the log
    // session lifecycle. syncDirtyMarker() mirrors it to the physical marker.
    MramDirtyPolicy dirty_policy_;
    void syncDirtyMarker();
    void clearRing();
    // Internal variant: caller holds push_mutex_. Used by ringPush()'s
    // drop-oldest fallback so it doesn't re-acquire the mutex it already
    // holds (regular FreeRTOS mutexes are not recursive).
    void clearRingLocked();
    void runStartupRecovery();
};

#endif
