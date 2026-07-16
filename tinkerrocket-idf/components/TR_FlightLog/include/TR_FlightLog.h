#pragma once

#include "BlockStateBitmap.h"
#include "FlightIndex.h"
#include "FlightLogLock.h"
#include "TR_BitmapStore.h"
#include "TR_FlightLog_types.h"
#include "TR_NandBackend.h"

namespace tr_flightlog {

// Custom append-only NAND flight-log layer. Replaces LittleFS on the write
// hot path; preserves BLE cmd 2 / 3 / 4 wire formats on the read side.
//
// Owns blocks [flight_region_start, flight_region_end) of the NAND as raw
// storage, plus four metadata blocks for the flight index + bitmap shadow.
//
// Stage 1 scaffold: only types and the interface are defined. Method bodies
// land in subsequent steps (see docs/plans/50-custom-nand-flight-log.md).
class TR_FlightLog {
public:
    struct Config {
        // ~10 MB (~2.7 min at the ~64 KB/s log rate) covers the vast majority of
        // flights without an overflow extend.  Was 256 (32 MB / ~8.5 min): with
        // recovered flights now trimmed (scanForBrownoutRecovery) that headroom
        // is pure waste/fragmentation, and a brownout flight no longer keeps it.
        // Block is now 256 KB (F35SQB004G), so these are halved vs the old 128 KB
        // block count to keep the same byte footprint / erase-stall duration.
        uint16_t prealloc_blocks     = 40;    // ~10 MB
        uint16_t extend_blocks       = 32;    // overflow step (~200 ms stall)
        uint16_t flight_region_start = 32;    // first block owned by this layer
        uint16_t flight_region_end   = 2044;  // one past last flight block
        uint16_t metadata_blocks[4]  = {2044, 2045, 2046, 2047};
        // #277: wall-clock cap on one brownout-recovery pass so a chip with many
        // large orphaned ranges can't scan for minutes and look hung past the
        // app's 180 s power-on watchdog. Checked at run boundaries (never
        // mid-run), so any unscanned runs simply defer to the next boot. 0 = off.
        uint32_t recovery_budget_ms  = 90000;  // 90 s, comfortably under 180 s
        // Monotonic-ms source for the budget; nullptr uses the built-in source
        // (esp_timer on hardware, 0 on host so the budget is inert in tests).
        uint32_t (*now_ms)()         = nullptr;

        // #315: rolling-buffer auto-eviction. When true, prepareFlight reclaims
        // space by deleting the oldest finalized flight(s) at arm time until the
        // new flight fits, an index slot is free, AND at least
        // auto_evict_target_free_blocks remain free. Default OFF (opt-in):
        // deleting an un-downloaded flight is destructive, so it stays gated
        // until bench-trusted. The active/in-progress flight is never a candidate
        // (it isn't in the index until finalizeFlight), and eviction runs only
        // here — before launch — never mid-flight. Evictions are counted
        // (autoEvictedCount) so the OC can surface them; never silent.
        bool     auto_evict_oldest             = false;
        // Free-block headroom to restore at arm time when auto_evict_oldest is on.
        // Eviction continues oldest-first until FREE blocks reach this floor, so
        // the card keeps a rolling buffer instead of evicting one flight per arm.
        // Clamped to the flight region; 0 means "reclaim only just enough to fit".
        uint16_t auto_evict_target_free_blocks = 0;
    };

    TR_FlightLog() = default;

    // `bitmap_store` is optional — pass nullptr for volatile (test / first-boot)
    // operation. On hardware, supply an NVS-backed store so the bitmap survives
    // reboots.
    Status begin(TR_NandBackend& nand, const Config& cfg,
                 TR_BitmapStore* bitmap_store = nullptr);

    // Mark a block as bad (sticky; persists immediately if a store is attached).
    Status markBlockBad(uint32_t block);

    // Inspect the bitmap — exposed for tests and for diagnostic reporting.
    const BlockStateBitmap& bitmap() const { return bitmap_; }
    const FlightIndex&      index()  const { return index_; }

    // Active-flight accessors (stable after prepareFlight, before finalizeFlight).
    bool      isFlightActive()     const { return flight_active_; }
    uint32_t  activeFlightId()     const { return active_flight_id_; }
    uint32_t  activeStartBlock()   const { return active_start_block_; }
    uint32_t  activeBlockCount()   const { return active_n_blocks_; }
    uint32_t  activePagesWritten() const { return active_next_page_; }

    // Number of times the pre-allocated range has been extended mid-flight.
    // Useful for post-flight log analysis to spot overflow events.
    uint32_t  overflowExtensionCount() const { return extension_count_; }

    // #371 diagnostics. salvagedBlockCount(): runtime bad blocks that had
    // already-written pages relocated to a good block (distinguishes a
    // mid-block program failure — where data was preserved — from a clean
    // page-0 skip-ahead, which relocates nothing). unrecoverablePageCount():
    // salvaged pages that could not be re-read from the retired block and were
    // replaced with a zeroed placeholder to keep the stream aligned (expected
    // to stay 0; a non-zero value flags genuine, unavoidable loss).
    uint32_t  salvagedBlockCount()    const { return salvaged_block_count_; }
    uint32_t  unrecoverablePageCount() const { return unrecoverable_page_count_; }

    // #315 rolling-buffer eviction telemetry. autoEvictedCount(): cumulative
    // number of finalized flights auto-deleted since begin() to make room at arm
    // time (stays 0 unless Config::auto_evict_oldest). lastEvictedFlightId():
    // flight_id of the most recently auto-evicted flight (0 if none). The OC
    // surfaces both in the storage-stats telemetry + a log line so a rolling
    // eviction is observable, never silent.
    uint32_t  autoEvictedCount()      const { return auto_evicted_count_; }
    uint32_t  lastEvictedFlightId()   const { return last_evicted_flight_id_; }

    // #510 (#388 contention): wall time writeFrame spent BLOCKED acquiring
    // mutex_, per stats window — the flush task's per-page waits behind a
    // concurrent BLE listFlights/readFlightPage are otherwise folded
    // invisibly into the logger's write timing. Max single wait + window sum;
    // reset alongside the logger's resetIntervalTimings(). On host builds the
    // clock is inert (0), matching monotonic_ms().
    uint32_t  writeLockWaitMaxUs() const { return wf_lock_wait_max_us_; }
    uint32_t  writeLockWaitSumUs() const { return wf_lock_wait_sum_us_; }
    void      resetLockWaitStats()       { wf_lock_wait_max_us_ = 0; wf_lock_wait_sum_us_ = 0; }

    // Pre-launch: pick + erase a free contiguous range. May stall ~770 ms.
    // Writes the assigned flight_id to `flight_id_out` on success.
    //
    // This is the synchronous form. On hardware, prefer
    // requestPrepareFlight() + servicePendingPrepareFlight() so the 256-block
    // erase loop runs on the flush task's core (Core 0) instead of blocking
    // sensor ingest on the calling core (issue #77).
    Status prepareFlight(uint32_t& flight_id_out);

    // Defer a prepareFlight call. Sets a flag and returns immediately; the
    // ~770 ms erase loop is performed by a later servicePendingPrepareFlight()
    // call (typically from the flush task on Core 0). Idempotent — calling
    // multiple times before service queues at most one prepare. No-op if a
    // flight is already active.
    void requestPrepareFlight();

    // True iff requestPrepareFlight() has been called and the request has not
    // yet been picked up by servicePendingPrepareFlight().
    bool isPrepareFlightPending() const { return prepare_request_pending_; }

    // If a request is pending and no flight is currently active, runs
    // prepareFlight() inline and reports its outcome via id_out / status_out.
    // Returns true when work was done; false when there was nothing to do
    // (no request pending, or a flight is already active). The pending flag
    // is cleared in either case so a stale request can't fire after the
    // flight has been started by some other path.
    //
    // Intended call site: once per flush-task iteration on the same core that
    // owns expensive NAND I/O.
    bool servicePendingPrepareFlight(uint32_t& id_out, Status& status_out);

    // Hot path: deterministic page write. Called by flush task.
    // `page` must be exactly NAND_PAGE_SIZE bytes. No allocation, no metadata.
    // The caller owns the page layout — recovery requires a PageHeader at the
    // start; consider `writeFrame` instead for that case.
    Status writePage(const uint8_t* page);

    // Hot-path helper: wrap `payload` (up to NAND_PAGE_SIZE - sizeof(PageHeader)
    // = 4080 bytes) in a PageHeader (active flight_id + monotonic seq + CRC32)
    // and program it. This is the recommended write API — pages written via
    // writeFrame are recoverable by the brownout scanner if finalizeFlight is
    // never called.
    Status writeFrame(const uint8_t* payload, size_t payload_len);

    // Post-flight: append index entry. May stall.
    Status finalizeFlight(const char* filename, uint32_t final_bytes);

    // BLE-facing read surface.
    size_t listFlights(FlightIndexEntry* out, size_t max, size_t page, size_t per_page);
    Status readFlightPage(const char* filename, uint32_t offset, uint8_t* buf,
                          size_t buf_len, size_t& out_len);
    Status deleteFlight(const char* filename);    // releases blocks for reuse
    Status renameFlight(const char* old_name, const char* new_name);

    // --- Introspection / test surface ---
    bool          isInitialized() const { return initialized_; }
    const Config& config() const        { return cfg_; }

private:
    TR_NandBackend*    nand_          = nullptr;
    TR_BitmapStore*    bitmap_store_  = nullptr;
    BlockStateBitmap   bitmap_        = {};
    FlightIndex        index_         = {};
    Config             cfg_           = {};
    bool               initialized_   = false;

    uint32_t active_flight_id_    = 0;
    uint32_t active_start_block_  = 0;
    uint32_t active_n_blocks_     = 0;
    uint32_t active_next_page_    = 0;  // absolute page index within the flight range
    uint32_t extension_count_     = 0;
    uint32_t salvaged_block_count_    = 0;  // #371: bad blocks whose pages were relocated
    uint32_t unrecoverable_page_count_ = 0; // #371: salvage pages that would not re-read
    uint32_t auto_evicted_count_      = 0;  // #315: flights auto-deleted to make room
    uint32_t last_evicted_flight_id_  = 0;  // #315: id of the most recent auto-evict
    bool     flight_active_       = false;

    // Page-sized scratch buffers kept OFF the task stack. At the 4 KB
    // F35SQB004G page size, the nested writeFrame -> writePageLocked ->
    // retireBlockAndSalvage path would otherwise put two 4 KB page[] arrays on
    // the flush task's 8 KB stack at once and overflow it. All uses are
    // serialized by mutex_ (or run at boot before the flush task exists);
    // salvage needs its own buffer because it is live while page_buf_ is still
    // held by the in-progress writeFrame/writePage call.
    uint8_t  page_buf_[NAND_PAGE_SIZE]    = {};  // writeFrame / readFlightPage / recovery
    uint8_t  salvage_buf_[NAND_PAGE_SIZE] = {};  // retireBlockAndSalvage only

    // Single-producer (any task) / single-consumer (flush task) request flag
    // for the deferred prepareFlight path. `volatile` is sufficient because
    // the consumer is idempotent: a missed clear at worst causes one extra
    // service call that no-ops via the flight_active_ check.
    volatile bool prepare_request_pending_ = false;

    // Guards all mutation/read of index_ and bitmap_ across the two cores that
    // touch this instance (#388). Held for the whole logical operation.
    // requestPrepareFlight() deliberately does NOT take it — it only sets the
    // volatile flag above, so the high-priority I2S task can never block on a
    // ~770 ms prepareFlight erase that holds the lock.
    FlightLogMutex mutex_;

    // #510: writeFrame's mutex_-acquire wall time (see writeLockWaitMaxUs).
    // Written only by the flush task, read cross-task for the stats print —
    // same unsynchronized-diagnostics convention as the logger's spi peaks.
    volatile uint32_t wf_lock_wait_max_us_ = 0;
    volatile uint32_t wf_lock_wait_sum_us_ = 0;

    // Lock-held implementations, called by the public wrappers and by the two
    // internal call chains (servicePendingPrepareFlight -> prepareFlightLocked,
    // writeFrame -> writePageLocked) so the non-recursive mutex is never taken
    // twice on one thread.
    Status prepareFlightLocked(uint32_t& flight_id_out);
    Status writePageLocked(const uint8_t* page);

    // Lock-held delete: frees the flight's blocks, removes its index entry, and
    // persists both. The public deleteFlight() wraps this; it is also the
    // eviction primitive (evictOldestLocked deletes via this path). Assumes
    // mutex_ is held so the non-recursive lock is never taken twice.
    Status deleteFlightLocked(const char* filename);

    // #315 rolling buffer. evictOldestLocked deletes the single oldest eligible
    // (finalized, non-active) flight and returns true, or false when the index
    // holds nothing eligible. evictOldestToTargetLocked repeats it oldest-first
    // until the pending prealloc fits, an index slot is free, AND free blocks
    // reach cfg_.auto_evict_target_free_blocks — or nothing eligible remains;
    // it returns the number of flights evicted. Both assume mutex_ is held and
    // are reached only from prepareFlightLocked (arm time — no active flight to
    // hit, since the active allocation is never in the index).
    bool     evictOldestLocked();
    uint32_t evictOldestToTargetLocked();

    // On a mid-block program failure, retire `bad_block` (relative index
    // `rel_block`) and relocate the `valid_pages` pages already written at its
    // start to the next good block(s), advancing active_next_page_ to just past
    // them so the caller can retry the failed write there. The retired block
    // stays readable for its committed pages (the failure was on a later page),
    // so each is re-read and reprogrammed; a destination that also fails is
    // retired and the whole relocation restarts into the next block, so no
    // salvaged page is dropped. Returns false only on NoSpace. (#371)
    bool retireBlockAndSalvage(uint32_t bad_block, uint32_t rel_block,
                               uint32_t valid_pages);

    void persistBitmap();
    void seedBitmapFromBackend();  // initial bad-block scan into the fresh bitmap

    // Extend the active flight by cfg_.extend_blocks contiguous blocks after
    // the current range. Returns true on success; false if the adjacent slot
    // isn't free or erase fails.
    bool extendActiveRange();

    // After loading bitmap + index, scan for allocated ranges not covered by
    // any index entry — those are flights interrupted by a brownout before
    // finalizeFlight ran. For each, walk the PageHeader magic/CRC32 to find
    // the last valid page, synthesize a partial FlightIndexEntry, and persist.
    // Orphaned ranges with no valid pages are released back to FREE.
    Status scanForBrownoutRecovery();
};

}  // namespace tr_flightlog
