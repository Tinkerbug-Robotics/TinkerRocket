#ifndef TR_LOG_TO_FLASH_H
#define TR_LOG_TO_FLASH_H

#include <compat.h>
#include <RocketComputerTypes.h>
#include "lfs.h"
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
    bool force_format = false;  // Erase entire NAND before mount (recovery from corruption)

    // LittleFS block-count. Defaults to the full chip for backward compat.
    // When sharing the NAND with TR_FlightLog (issue #50 Stage 2c) the caller
    // shrinks this to just the blocks LFS owns (e.g. 32 for a 4 MB LFS
    // partition) so the flight-log allocator can safely use the remainder.
    uint32_t lfs_block_count = 0;  // 0 = full chip (NAND_BLOCK_COUNT)

    // Optional hot-path write override (issue #50 Stage 2c-3c). When set,
    // the flush task calls `write_sink(write_sink_ctx, payload, len)` for
    // each page drained from the ring instead of lfs_file_write. `len` is
    // fixed at NAND_PAGE_SIZE - 16 (2032) so the sink can prepend a 16-byte
    // TR_FlightLog PageHeader and still land on a NAND page boundary. The
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
    uint32_t spi_hz_mram = 40'000'000;
    uint8_t spi_mode_mram = SPI_MODE0;
    uint32_t mram_size = 131072;        // 128 KB (MR25H10)
};

struct TR_LogToFlashStats
{
    uint32_t ring_fill = 0;
    uint32_t ring_highwater = 0;
    uint32_t ring_overruns = 0;
    uint32_t ring_drop_oldest_bytes = 0;
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
    // These expose the private nand* primitives as full-page (2 KB) ops keyed
    // by (block, page_in_block). They share the same SPI bus + bad-block
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

    static constexpr uint32_t NAND_PAGE_SIZE = 2048;
    static constexpr uint32_t NAND_PAGES_PER_BLK = 64;
    static constexpr uint32_t NAND_BLOCK_SIZE = NAND_PAGE_SIZE * NAND_PAGES_PER_BLK;  // 128KB
    static constexpr uint32_t NAND_BLOCK_COUNT = 1024;

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
    uint8_t* ring_buf_ = nullptr;       // Used only when use_mram_ == false
    uint32_t ring_size_ = 0;
    uint32_t rb_head = 0;
    uint32_t rb_tail = 0;
    uint32_t rb_count = 0;
    uint32_t rb_overruns = 0;
    uint32_t rb_drop_oldest_bytes = 0;
    uint32_t rb_highwater = 0;

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
    // Before logging starts, cap the ring at 50% so the initial flush at
    // launch detection doesn't stall the main loop.  Raised to full size
    // once logging is active (see openLogSession / closeLogSession).
    uint32_t ring_prelaunch_cap_ = 0;  // Set in begin() from ring_size_

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
    bool logging_active = false;
    bool start_logging_requested = false;
    bool end_flight_requested = false;
    uint32_t log_start_block = 1;
    uint32_t log_curr_block = 1;
    bool log_block_erased = false;

    // page staging
    uint8_t page_buf[NAND_PAGE_SIZE];
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
    static constexpr uint32_t SYNC_INTERVAL_PAGES = 64;   // ~128 KB / 1.8 s
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
    uint32_t syncs_performed_ = 0;

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
    static constexpr uint32_t BAD_BLOCK_BITMAP_BYTES = NAND_BLOCK_COUNT / 8;
    uint8_t bad_block_bitmap_[BAD_BLOCK_BITMAP_BYTES] = {};
    bool     bad_block_bitmap_dirty_ = false;
    uint32_t bad_block_skips_ = 0;   // cumulative short-circuits

    bool isBlockBad(uint32_t block) const;
    void markBlockBad(uint32_t block);
    void loadBadBlocksFromNVS();
    void persistBadBlocksIfDirty();
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

    // startup recovery
    bool recovery_performed = false;
    uint32_t recovery_bytes = 0;
    char recovery_filename[64] = {};

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

    // SPI bus mutex (active when MRAM is enabled — both cores share SPI bus)
    SemaphoreHandle_t spi_mutex_ = nullptr;
    void spiAcquire();
    void spiRelease();

    // MRAM SPI interface (MR25H10)
    SPISettings spi_mram;
    static constexpr uint8_t MRAM_WREN  = 0x06;
    static constexpr uint8_t MRAM_READ  = 0x03;
    static constexpr uint8_t MRAM_WRITE = 0x02;
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
    uint32_t ringPop(uint8_t* out, uint32_t len);
    void ringPeekAt(uint32_t offset, uint8_t* out, uint32_t len);

    void nandWREN();
    uint8_t nandGetFeature(uint8_t addr);
    void nandSetFeature(uint8_t addr, uint8_t val);
    bool nandWaitReady(uint32_t timeout_us = 2'000'000);
    bool nandEraseBlock(uint32_t blockIndex);
    bool nandProgramPage(uint32_t rowPageAddr, const uint8_t* data, uint32_t len);
    bool nandReadPage(uint32_t rowPageAddr, uint8_t* out, uint32_t len);
    /// Read `len` bytes from `column` within a page — used to access the OOB /
    /// spare area (column ≥ NAND_PAGE_SIZE) where factory bad-block markers
    /// live.  Returns false if the PAGEREAD status polls out (suspect block).
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
    void clearRing();
    // Internal variant: caller holds push_mutex_. Used by ringPush()'s
    // drop-oldest fallback so it doesn't re-acquire the mutex it already
    // holds (regular FreeRTOS mutexes are not recursive).
    void clearRingLocked();
    void runStartupRecovery();
};

#endif
