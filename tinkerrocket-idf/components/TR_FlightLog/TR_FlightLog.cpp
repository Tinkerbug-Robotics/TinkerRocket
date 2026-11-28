#include "TR_FlightLog.h"

#include "CRC.h"

#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>
#define FL_LOGW(...) ESP_LOGW("FLTLOG", __VA_ARGS__)
#define FL_LOGI(...) ESP_LOGI("FLTLOG", __VA_ARGS__)
#else
#define FL_LOGW(...) ((void)0)
#define FL_LOGI(...) ((void)0)
#endif

#include <cstdio>
#include <cstring>

namespace tr_flightlog {

namespace {

// CRC32 of a page that starts with a PageHeader: covers bytes[4..page_size-1].
// #671: the span is the RUNTIME page size — a page written at one size can
// never validate at another, which is fine because geometry is a pure
// function of the physical chip (see nand_geometry.h).
uint32_t page_crc(const uint8_t* page, uint32_t page_size) {
    constexpr size_t skip = sizeof(uint32_t);  // skip the crc32 field itself
    return calcCRC32(page + skip, page_size - skip);
}

bool page_is_all_ones(const uint8_t* page, uint32_t page_size) {
    for (size_t i = 0; i < page_size; ++i) {
        if (page[i] != 0xFF) return false;
    }
    return true;
}

// Brownout scan can sweep thousands of NAND pages on a single boot. Yield to
// other tasks every block's worth of reads so IDLE1 (task_wdt) doesn't starve.
// On host / in tests this is a no-op.
inline void yield_to_scheduler() {
#ifdef ESP_PLATFORM
    vTaskDelay(1);  // ~10 ms; allows IDLE to run and resets the watchdog
#endif
}

// Monotonic milliseconds since boot for the recovery time budget (#277). On
// host / in tests there is no wall clock, so this returns 0 and the budget is
// inert unless a test injects a clock via Config::now_ms.
inline uint32_t monotonic_ms() {
#ifdef ESP_PLATFORM
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
#else
    return 0;
#endif
}

// Monotonic microseconds for the #510 lock-wait metric. Host builds have no
// clock, so waits measure 0 and the metric is inert — same as monotonic_ms().
inline int64_t monotonic_us() {
#ifdef ESP_PLATFORM
    return esp_timer_get_time();
#else
    return 0;
#endif
}

}  // namespace

Status TR_FlightLog::begin(TR_NandBackend& nand, const Config& cfg,
                            TR_BitmapStore* bitmap_store) {
    nand_         = &nand;
    cfg_          = cfg;
    bitmap_store_ = bitmap_store;

    // #671: capture the chip's runtime geometry — the single source of truth
    // for every page/block computation below. sanity-cap at the static maxima
    // so a lying backend can't overrun the MAX-sized buffers.
    g_page_   = nand_->pageSize();
    g_ppb_    = nand_->pagesPerBlock();
    g_blocks_ = nand_->blockCount();
    if (g_page_ == 0 || g_page_ > NAND_PAGE_SIZE_MAX ||
        g_ppb_ == 0 || g_blocks_ == 0 || g_blocks_ > NAND_BLOCK_COUNT_MAX) {
        return Status::OutOfRange;
    }
    bitmap_.setBlockCount(g_blocks_);

    // #671: derive the sentinel defaults from the chip — metadata is the top
    // four blocks, flight region runs up to them. Explicit caller values pass
    // through untouched (and get validated either way).
    if (cfg_.flight_region_end == 0) {
        cfg_.flight_region_end = static_cast<uint16_t>(g_blocks_ - 4);
    }
    if (cfg_.metadata_blocks[0] == 0 && cfg_.metadata_blocks[1] == 0 &&
        cfg_.metadata_blocks[2] == 0 && cfg_.metadata_blocks[3] == 0) {
        for (int i = 0; i < 4; ++i) {
            cfg_.metadata_blocks[i] = static_cast<uint16_t>(g_blocks_ - 4 + i);
        }
    }

    if (cfg_.flight_region_start >= cfg_.flight_region_end ||
        cfg_.flight_region_end > g_blocks_) {
        return Status::OutOfRange;
    }
    // #671: this is the gate that was blind on the mini — the legacy defaults
    // (metadata 2044-2047) validated against a compile-time 2048 and then
    // addressed blocks past the end of a 1024-block die.
    for (int i = 0; i < 4; ++i) {
        if (cfg_.metadata_blocks[i] >= g_blocks_) return Status::OutOfRange;
        // Metadata must sit outside the flight region, or the allocator can
        // hand a flight the block the index/bitmap lives in and erase it.
        if (cfg_.metadata_blocks[i] >= cfg_.flight_region_start &&
            cfg_.metadata_blocks[i] < cfg_.flight_region_end) {
            return Status::OutOfRange;
        }
    }
    if (cfg_.prealloc_blocks == 0 ||
        cfg_.prealloc_blocks > (cfg_.flight_region_end - cfg_.flight_region_start)) {
        return Status::OutOfRange;
    }
    // The dual-copy index snapshot must fit one metadata BLOCK at this
    // geometry (it strides in pages of g_page_).
    if (FlightIndex::MAX_SERIALIZED_BYTES > g_page_ * g_ppb_) {
        return Status::OutOfRange;
    }

    // Try to restore bitmap from persistent store. If nothing saved (or wrong
    // size for THIS chip's serialized length) start fresh and seed from the
    // NAND backend's bad-block oracle. When fresh, defer persisting until
    // after the index reconciliation below.
    uint8_t buf[BlockStateBitmap::SERIALIZED_SIZE_MAX];
    const size_t ser_len = bitmap_.serializedSize();
    bool restored = bitmap_store_ && bitmap_store_->load(buf, ser_len);
    if (restored) {
        bitmap_.deserializeFrom(buf, ser_len);
    } else {
        bitmap_.clear();
        seedBitmapFromBackend();
    }

    FL_LOGI("begin: bitmap %s (%lu blocks, %lu B)",
            restored ? "restored" : "seeded fresh",
            (unsigned long)g_blocks_, (unsigned long)ser_len);

    Status idx_st = index_.load(*nand_, cfg_.metadata_blocks[0], cfg_.metadata_blocks[1]);
    if (idx_st != Status::Ok && idx_st != Status::CrcMismatch) return idx_st;
    // CrcMismatch on a fresh chip (or both copies corrupt) is recoverable;
    // the index is already cleared and we proceed with an empty one.

    // Reconcile the bitmap against the index on every boot: any block an
    // indexed flight owns but the bitmap thinks is FREE gets promoted to
    // ALLOCATED. A restored bitmap is normally already consistent, but a fresh
    // seed (fresh chip / NVS wipe / the one-time NVS->NAND migration, #398) — or
    // a bitmap that was persisted before this reconciliation existed — has lost
    // those allocations. findContiguousFree is bitmap-only, so without this the
    // next flight would allocate over an indexed one and overwrite it. Running
    // every boot self-heals such a bitmap (e.g. a device already migrated by an
    // earlier build). BAD blocks are left untouched. Persist only when we
    // actually changed something, or on a fresh seed (to store the seeded bad
    // blocks).
    size_t promoted = 0;
    for (size_t i = 0; i < index_.size(); ++i) {
        const FlightIndexEntry& e = index_.at(i);
        const uint32_t end = static_cast<uint32_t>(e.start_block) + e.n_blocks;
        for (uint32_t b = e.start_block; b < end && b < g_blocks_; ++b) {
            if (bitmap_.get(b) == BLOCK_FREE) {
                bitmap_.set(b, BLOCK_ALLOCATED);
                ++promoted;
            }
        }
    }
    if (!restored || promoted > 0) persistBitmap();

    initialized_ = true;

    // #1279: one line per phase. A hang anywhere in this function used to be
    // completely silent — the last thing on the console was the caller's
    // banner — which is why localising one needed four boots and a source
    // read. Each of these is once per boot.
    FL_LOGI("begin: index loaded (%zu flight(s)), %zu block(s) promoted; "
            "recovery scan next", index_.size(), promoted);

    // #1279 follow-up: hand the scan back to the caller to run on its own
    // time. Everything above is done — the index and bitmap are loaded and
    // reconciled — so the object is fully usable for reads, writes and
    // allocation; only the reconciliation of orphaned runs is outstanding.
    if (cfg_.defer_recovery) {
        recovery_deferred_ = true;
        recovery_failed_   = false;
        orphan_next_id_    = orphanNextIdAtBoot();
        FL_LOGI("begin: recovery scan DEFERRED to the caller (#1279)");
        return Status::Ok;
    }

    // Recover any flights that were in progress when the last boot was cut
    // short. Safe to run every boot — a no-op when nothing is orphaned.
    Status recov_st = scanForBrownoutRecovery(/*enforce_budget=*/true);
    if (recov_st != Status::Ok) {
        // #1127: report the failure, but do NOT un-initialise. initialized_
        // gates listFlights/readFlightPage/deleteFlight/renameFlight as well
        // as the write path, so clearing it took the stored flights offline
        // AND removed the delete that frees the space causing an index-full
        // failure — self-sustaining across every subsequent boot, recoverable
        // only by a reflash. The write path is gated on recovery_failed_
        // instead, which is the half that is genuinely unsafe: the orphaned
        // range is unresolved, so a new allocation could land on it.
        recovery_failed_ = true;
        return recov_st;
    }
    recovery_failed_ = false;
    return Status::Ok;
}

Status TR_FlightLog::runDeferredRecovery() {
    if (!initialized_)       return Status::NotInitialized;
    if (!recovery_deferred_) return Status::Ok;   // nothing was deferred
    {
        FlightLogLockGuard guard(mutex_);
        forgetBootOrphan();   // #1717: the scan indexes and frees blocks
    }

    // Budget OFF on purpose. The 90 s ceiling exists so the scan cannot hold
    // up the caller that is waiting on it; here nobody is waiting, and letting
    // it run to completion is the entire point — a run larger than the budget
    // is exactly the case that could never resolve itself before (#1279).
    const Status st = scanForBrownoutRecovery(/*enforce_budget=*/false);

    recovery_deferred_ = false;
    recovery_failed_   = (st != Status::Ok);
    if (st == Status::Ok) {
        FL_LOGI("deferred recovery complete: %zu flight(s) in index",
                index_.size());
    } else {
        FL_LOGW("deferred recovery FAILED (%d) — no new flight can be armed "
                "(#1127); one already logging continues", (int)st);
    }
    return st;
}

Status TR_FlightLog::newestOrphanTail(uint32_t window_pages, TailSink sink, void* ctx,
                                      OrphanTail& out) {
    out = OrphanTail{};
    if (!initialized_) return Status::NotInitialized;

    uint8_t* const page = page_buf_;
    uint32_t reads = 0;
    auto is_active = [&](uint32_t block) {
        return flight_active_ && block >= active_start_block_ &&
               block < active_start_block_ + active_n_blocks_;
    };
    // One page, one burst under the object lock, as the scan reads: true with
    // its header when it is a valid flight-log page (magic and CRC). With
    // `orphan_only`, blocks that are not orphaned are skipped unread.
    auto readHeader = [&](uint32_t block, uint32_t p, PageHeader& hdr, bool orphan_only) {
        FlightLogLockGuard guard(mutex_);
        if (orphan_only && (bitmap_.get(block) != BLOCK_ALLOCATED ||
                            blockInIndexLocked(block) || is_active(block))) {
            return false;
        }
        ++reads;
        if (!nand_->readPage(block, p, page)) return false;
        std::memcpy(&hdr, page, sizeof(hdr));
        return hdr.magic == FPAG_MAGIC && hdr.crc32 == page_crc(page, g_page_);
    };

    // The newest stored flight: an orphan at or below it is history.
    bool     any_stored    = false;
    uint32_t newest_stored = 0;
    {
        FlightLogLockGuard guard(mutex_);
        for (size_t i = 0; i < index_.size(); ++i) {
            any_stored = true;
            if (index_.at(i).flight_id > newest_stored) newest_stored = index_.at(i).flight_id;
        }
    }

    // Page 0 of every orphaned block: the newest flight id, and its lowest and
    // highest blocks. A flight's range is contiguous and written in block
    // order, so its highest block with a written page 0 is its last. begin()'s
    // sweep has the answer while nothing has changed since.
    bool     have      = false;
    uint32_t id        = 0;
    uint32_t first_blk = 0;
    uint32_t last_blk  = 0;
    bool     swept     = false;
    {
        FlightLogLockGuard guard(mutex_);
        if (boot_orphan_.valid) {
            swept     = true;
            have      = boot_orphan_.found;
            id        = boot_orphan_.id;
            first_blk = boot_orphan_.first_blk;
            last_blk  = boot_orphan_.last_blk;
        }
    }
    for (uint32_t b = cfg_.flight_region_start; !swept && b < cfg_.flight_region_end; ++b) {
        PageHeader hdr;
        if (!readHeader(b, 0, hdr, /*orphan_only=*/true)) continue;
        if (!have || hdr.flight_id > id) {
            have      = true;
            id        = hdr.flight_id;
            first_blk = b;
            last_blk  = b;
        } else if (hdr.flight_id == id) {
            last_blk = b;   // b only grows
        }
    }
    out.pages_read = reads;
    if (!have || (any_stored && id <= newest_stored)) return Status::NotFound;

    // The last block's last valid page, from the top. A block's pages program
    // in order, so the first valid one from the top is the flight's end; a
    // torn page above it (a cut mid-program) fails its CRC and is passed over.
    // (Not by seq_number: a page that failed to program is retried in the next
    // block with the header it was built with, so seq_number can jump within
    // a block. Only page order is sure.)
    int32_t  last_p   = -1;
    uint32_t last_seq = 0;
    for (int32_t p = static_cast<int32_t>(g_ppb_) - 1; p >= 0; --p) {
        PageHeader hdr;
        if (!readHeader(last_blk, static_cast<uint32_t>(p), hdr, /*orphan_only=*/false)) continue;
        if (hdr.flight_id == id) {
            last_p   = p;
            last_seq = hdr.seq_number;
            break;
        }
    }
    out.pages_read = reads;
    if (last_p < 0) return Status::NotFound;   // its page 0 read valid a moment ago

    out.flight_id   = id;
    out.start_block = first_blk;
    out.last_seq    = last_seq;
    if (sink == nullptr || window_pages == 0) return Status::Ok;

    // The flight's blocks that hold the last `window_pages` pages, newest
    // first: walk down from the last block to its first, four blocks at most,
    // so a previous flight's blank prealloc below it is never read. A bad
    // block between two of them is not orphaned and is passed over (its pages
    // were moved on when it failed); a block of another flight, which cannot
    // sit inside the range, would end it.
    uint32_t blks[4];
    size_t   n = 0;
    blks[n++] = last_blk;
    uint32_t have_pages = static_cast<uint32_t>(last_p) + 1u;
    for (uint32_t b = last_blk; b > first_blk && n < 4 && have_pages < window_pages;) {
        --b;
        bool orphan = false;
        {
            FlightLogLockGuard guard(mutex_);
            orphan = bitmap_.get(b) == BLOCK_ALLOCATED && !blockInIndexLocked(b) && !is_active(b);
        }
        if (!orphan) continue;   // a bad block: its pages were moved on when it failed
        PageHeader hdr;
        if (!readHeader(b, 0, hdr, /*orphan_only=*/false)) {
            // A block of the flight whose page 0 reads back bad: its pages
            // cannot be placed, so they are lost to the window.
            out.gaps += g_ppb_;
            have_pages += g_ppb_;
            continue;
        }
        if (hdr.flight_id != id) break;
        blks[n++] = b;
        have_pages += g_ppb_;
    }
    // Where the window starts, in the oldest block taken.
    uint32_t start_page = 0;
    if (have_pages > window_pages) start_page = have_pages - window_pages;

    // Oldest first into the sink, under the lock with the page in page_buf_:
    // the sink must not call back into this object.
    bool gap = false;
    for (size_t k = n; k-- > 0;) {
        const uint32_t blk   = blks[k];
        const uint32_t first = (k == n - 1) ? start_page : 0u;
        const uint32_t last  = (k == 0) ? static_cast<uint32_t>(last_p) : g_ppb_ - 1u;
        for (uint32_t p = first; p <= last; ++p) {
            FlightLogLockGuard guard(mutex_);
            ++reads;
            bool ok = nand_->readPage(blk, p, page);
            if (ok) {
                PageHeader hdr;
                std::memcpy(&hdr, page, sizeof(hdr));
                ok = hdr.magic == FPAG_MAGIC && hdr.crc32 == page_crc(page, g_page_) &&
                     hdr.flight_id == id;
            }
            if (!ok) {
                gap = true;
                ++out.gaps;
                continue;
            }
            sink(ctx, page + sizeof(PageHeader), payloadPerPage(), gap);
            gap = false;
            if (k == 0 && p == static_cast<uint32_t>(last_p)) out.end_fed = true;
        }
    }
    out.pages_read = reads;
    return Status::Ok;
}

bool TR_FlightLog::blockInIndexLocked(uint32_t block) const {
    for (size_t i = 0; i < index_.size(); ++i) {
        const auto& e = index_.at(i);
        if (block >= e.start_block &&
            block <  static_cast<uint32_t>(e.start_block) + e.n_blocks) {
            return true;
        }
    }
    return false;
}

uint32_t TR_FlightLog::orphanNextIdAtBoot() {
    // A flight armed before the deferred scan adopts the orphans takes
    // index_.nextFlightId(), which an orphan may already hold: the flight cut
    // off by the last reboot got that very id. Two flights sharing an id gave
    // two same-named flight_recovered_<id>.bin after another reboot, and let
    // the scan merge two adjacent orphans into one. So read the id on page 0
    // of every orphaned block (each block holds one flight, its first page
    // written first): 1/64 of the scan's reads, at begin(), before anything
    // else runs, so page_buf_ is free.
    uint32_t next = 0;
    uint8_t* const page = page_buf_;
    BootOrphan bo;   // #1717: the newest orphan, as newestOrphanTail sweeps for it
    for (uint32_t b = cfg_.flight_region_start; b < cfg_.flight_region_end; ++b) {
        if (bitmap_.get(b) != BLOCK_ALLOCATED || blockInIndexLocked(b)) continue;
        if (!nand_->readPage(b, 0, page)) continue;
        PageHeader hdr;
        std::memcpy(&hdr, page, sizeof(hdr));
        if (hdr.magic != FPAG_MAGIC || hdr.crc32 != page_crc(page, g_page_)) continue;
        if (hdr.flight_id >= next) next = hdr.flight_id + 1;
        if (!bo.found || hdr.flight_id > bo.id) {
            bo.found = true;
            bo.id = hdr.flight_id;
            bo.first_blk = b;
            bo.last_blk = b;
        } else if (hdr.flight_id == bo.id) {
            bo.last_blk = b;   // b only grows
        }
    }
    boot_orphan_ = bo;
    boot_orphan_.valid = true;   // valid when it found none, too
    return next;
}

Status TR_FlightLog::scanForBrownoutRecovery(bool enforce_budget) {
    if (!initialized_) return Status::NotInitialized;

    // A block is "orphaned" if it's ALLOCATED but not covered by any index
    // entry, nor by the flight being logged. We walk the flight region
    // looking for contiguous orphaned runs.
    //
    // The active flight is ALLOCATED and not in the index until
    // finalizeFlight, which is exactly what an orphan looks like. A
    // synchronous scan runs before any flight can start, but a deferred one
    // runs beside the live log (#1279 follow-up), and it used to adopt the
    // flight being logged as flight_recovered_<id>.bin, free its unwritten
    // tail under it, and merge it with an orphan just before it — the usual
    // layout, because a new flight starts right after the last allocated
    // range. Read under the lock on every check: the range grows with each
    // extension, and a flight can start or finalize between two bursts. A
    // flight that starts mid-scan takes only FREE blocks, so it can never be
    // inside a run already found.
    auto is_active = [&](uint32_t block) {
        return flight_active_ && block >= active_start_block_ &&
               block < active_start_block_ + active_n_blocks_;
    };
    auto is_in_index = [&](uint32_t block) { return blockInIndexLocked(block); };

    // #1279 follow-up: every NAND read below, and the check of what it read,
    // is one burst under the object lock. A deferred scan runs beside
    // writeFrame and readFlightPage, which build and read their pages in
    // page_buf_ under that lock, and the NAND driver's page read is not
    // atomic: it drops the SPI lock between the read-to-cache command and the
    // cache readout. Unlocked, the scan's reads interleaved with theirs —
    // through the buffer and through the chip's one cache register — so a
    // download could return the orphan's page, a logged frame could be
    // judged as the orphan's, and the scan could judge theirs. One page per
    // burst (a page read plus a CRC, about the cost of one download page)
    // keeps the flush task's waits short; a burst across a whole block would
    // hold it ~0.6 s.
    uint8_t* const page = page_buf_;

    bool index_dirty  = false;
    bool bitmap_dirty = false;

    // #277: progress + a wall-clock budget so a near-full chip's scan is visible
    // and can't approach the app's 180 s power-on watchdog.
    auto now = [&]() -> uint32_t { return cfg_.now_ms ? cfg_.now_ms() : monotonic_ms(); };
    const uint32_t t0            = now();
    const uint32_t region_blocks = static_cast<uint32_t>(cfg_.flight_region_end -
                                                          cfg_.flight_region_start);
    uint32_t blocks_scanned = 0;

    // Say which regime this is. Printing the budget unconditionally is worse
    // than useless on a deferred scan: it is the one line someone reads to
    // work out why a scan stopped, and there the budget is not enforced at all.
    if (enforce_budget && cfg_.recovery_budget_ms != 0) {
        FL_LOGI("recovery: scanning blocks %lu..%lu (budget %lu ms)",
                (unsigned long)cfg_.flight_region_start,
                (unsigned long)cfg_.flight_region_end,
                (unsigned long)cfg_.recovery_budget_ms);
    } else {
        FL_LOGI("recovery: scanning blocks %lu..%lu (no budget — %s)",
                (unsigned long)cfg_.flight_region_start,
                (unsigned long)cfg_.flight_region_end,
                enforce_budget ? "budget disabled by config"
                               : "deferred, so nothing is waiting on it");
    }

    uint32_t b = cfg_.flight_region_start;
    while (b < cfg_.flight_region_end) {
        // Budget is checked only at run boundaries: each run is scanned to
        // completion (a partial scan would synthesize a wrong-length entry), so
        // an over-budget pass leaves the remaining orphaned runs for next boot.
        if (enforce_budget && cfg_.recovery_budget_ms != 0 &&
            now() - t0 > cfg_.recovery_budget_ms) {
            FL_LOGW("recovery: %lu ms budget hit at block %lu/%lu — deferring the "
                    "rest to next boot",
                    (unsigned long)cfg_.recovery_budget_ms,
                    (unsigned long)(b - cfg_.flight_region_start),
                    (unsigned long)region_blocks);
            break;
        }
        // #1279 follow-up: this runs concurrently with writeFrame and the
        // download path once it is deferred to a task, so every touch of
        // bitmap_/index_, and every page read (above), is under the object
        // lock, in short bursts.
        {
            FlightLogLockGuard guard(mutex_);
            if (bitmap_.get(b) != BLOCK_ALLOCATED || is_in_index(b) || is_active(b)) {
                ++b;
                continue;
            }
        }
        // Found the start of an orphaned run. Walk forward.
        const uint32_t run_start = b;
        // Span BLOCK_BAD blocks as well as ALLOCATED ones. A block that failed
        // to program mid-flight is marked BAD and skipped to the next block by
        // writePage — it's an interior gap in ONE flight, not a boundary between
        // flights. Stopping the run at it would split the flight into two
        // same-named flight_recovered_<id>.bin entries with the data torn in
        // half (#276). The bad block is skipped when scanning pages (below) and
        // walked over on read (readFlightPage). Recovery runs every boot, so two
        // distinct orphaned flights never coexist to be wrongly merged here; the
        // one un-indexed range that can sit beside an orphan is the flight being
        // logged, and the run stops at it.
        {
            FlightLogLockGuard guard(mutex_);
            while (b < cfg_.flight_region_end &&
                   (bitmap_.get(b) == BLOCK_ALLOCATED || bitmap_.get(b) == BLOCK_BAD) &&
                   !is_in_index(b) && !is_active(b)) {
                ++b;
            }
        }
        const uint32_t run_len = b - run_start;
        // The start check and the walk test the same blocks, so a run is never
        // empty; if a change ever lets them disagree, step on rather than
        // re-test this block for ever.
        if (run_len == 0) {
            ++b;
            continue;
        }

        // Scan the run for pages with valid PageHeader CRC. Track the highest
        // seq_number + associated flight_id.
        int32_t  last_good_page_rel = -1;   // physical page index within the run
        uint32_t last_seq            = 0;
        uint32_t last_flight_id      = 0;
        uint32_t valid_pages         = 0;   // #276: count of magic+CRC pages
        bool     saw_any             = false;
        uint32_t run_flight_id       = 0;   // the flight of the run's first valid page
        uint32_t split_at            = run_len;   // blocks of this run that are that flight's

        bool over_budget = false;
        for (uint32_t i = 0; i < run_len; ++i) {
            // #1279: the budget must bite INSIDE the run as well. Checked only
            // at run boundaries it cannot interrupt a single long one, and a
            // boot that meets one never reaches loop_oc's BLE command dispatch
            // — the board then advertises, accepts a connection, and queues the
            // app's power-on for ever while looking perfectly alive. Observed
            // on the bench 2026-09-09: still scanning at 300 s against a 90 s
            // budget, with the FC rail never asserted.
            if (enforce_budget && cfg_.recovery_budget_ms != 0 &&
                now() - t0 > cfg_.recovery_budget_ms) {
                FL_LOGW("recovery: %lu ms budget hit %lu block(s) into the run at "
                        "%lu — abandoning it un-indexed for the next boot",
                        (unsigned long)cfg_.recovery_budget_ms,
                        (unsigned long)i, (unsigned long)run_start);
                over_budget = true;
                break;
            }
            if (++blocks_scanned % 256 == 0) {
                FL_LOGI("recovery: scanned %lu/%lu blocks (%lu ms)",
                        (unsigned long)blocks_scanned, (unsigned long)region_blocks,
                        (unsigned long)(now() - t0));
            }
            const uint32_t blk = run_start + i;
            // Fast-path: read just the first page of the block. If its header
            // doesn't carry FPAG_MAGIC, no writeFrame ever ran in this block
            // (the rest is 0xFF by contract — each block in an allocated
            // range was erased before use). Skip the 63 remaining reads.
            //
            // A block whose first page belongs to another flight starts the
            // next orphan: two can sit back to back when the OC reboots twice
            // before a deferred scan reaches the first (the comment on the
            // run walk), and before this the walk merged them, and freed the
            // second's blocks as the first's unused tail.
            bool blank = false, next_flight = false;
            {
                FlightLogLockGuard guard(mutex_);
                if (bitmap_.get(blk) == BLOCK_BAD) continue;
                if (!nand_->readPage(blk, 0, page)) continue;
                PageHeader hdr0;
                std::memcpy(&hdr0, page, sizeof(hdr0));
                if (hdr0.magic != FPAG_MAGIC) {
                    blank = true;   // erased but never programmed — nothing to find
                } else if (hdr0.crc32 == page_crc(page, g_page_)) {
                    if (saw_any && hdr0.flight_id != run_flight_id) {
                        next_flight = true;
                    } else {
                        ++valid_pages;
                        if (!saw_any || hdr0.seq_number > last_seq) {
                            last_seq           = hdr0.seq_number;
                            last_flight_id     = hdr0.flight_id;
                            last_good_page_rel = static_cast<int32_t>(i * g_ppb_);
                        }
                        if (!saw_any) run_flight_id = hdr0.flight_id;
                        saw_any = true;
                    }
                }
            }
            if (blank) {
                yield_to_scheduler();
                continue;
            }
            if (next_flight) {
                split_at = i;
                break;
            }
            for (uint32_t p = 1; p < g_ppb_; ++p) {
                FlightLogLockGuard guard(mutex_);
                if (!nand_->readPage(blk, p, page)) continue;
                if (page_is_all_ones(page, g_page_)) continue;   // unwritten

                PageHeader hdr;
                std::memcpy(&hdr, page, sizeof(hdr));
                if (hdr.magic != FPAG_MAGIC) continue;
                if (hdr.crc32 != page_crc(page, g_page_)) continue;
                if (saw_any && hdr.flight_id != run_flight_id) continue;   // not this flight's
                ++valid_pages;

                if (!saw_any || hdr.seq_number > last_seq) {
                    last_seq           = hdr.seq_number;
                    last_flight_id     = hdr.flight_id;
                    last_good_page_rel = static_cast<int32_t>(i * g_ppb_ + p);
                }
                if (!saw_any) run_flight_id = hdr.flight_id;
                saw_any = true;
            }
            // Yield after each block so the per-block scan cost can be
            // absorbed without starving IDLE1 and tripping task_wdt.
            yield_to_scheduler();
        }

        // A partially scanned run tells us nothing: `saw_any` false means "not
        // found YET", not "not there", so neither branch below may run. Leave
        // the blocks ALLOCATED and un-indexed — precisely the state this scan
        // looks for — and the next boot picks the run up again. Same contract
        // the boundary check already had, just honoured mid-run too.
        if (over_budget) break;

        // The rest of the run is the next orphan: resume the walk there.
        const uint32_t span = split_at;
        if (span < run_len) b = run_start + span;

        if (!saw_any) {
            // No recoverable data in this range — release it.
            FlightLogLockGuard guard(mutex_);
            bitmap_.markFreeRange(run_start, span);
            bitmap_dirty = true;
            continue;
        }

        // Synthesize a partial flight entry. final_bytes is the logical PAYLOAD
        // byte count actually flushed to NAND — #273: PAYLOAD_PER_PAGE (4080),
        // NOT the raw page size, or the downloader walks past the last real
        // page into PageHeader/0xFF bytes (corrupt tail, size ~0.8% too large).
        const uint32_t PAYLOAD_PER_PAGE = payloadPerPage();
        // #276: length is the count of *valid* pages actually present, NOT the
        // physical span of the highest-seq page. If the flight hit a bad block
        // mid-write, writePage marked it BAD and skipped to the next block, so
        // the span over-counts by the skipped block(s) — deriving bytes from it
        // injects interior 0xFF garbage and the wrong length. The physical span
        // (used_pages) still sizes the block range to keep allocated; the gap is
        // walked on read (readFlightPage).
        const uint32_t used_pages  = static_cast<uint32_t>(last_good_page_rel + 1);
        uint32_t used_blocks = (used_pages + g_ppb_ - 1) / g_ppb_;
        if (used_blocks > span) used_blocks = span;

        FlightIndexEntry entry{};
        entry.magic       = FLGT_MAGIC;
        entry.flight_id   = last_flight_id;
        std::snprintf(entry.filename, sizeof(entry.filename),
                      "flight_recovered_%lu.bin",
                      static_cast<unsigned long>(last_flight_id));
        entry.start_block = static_cast<uint16_t>(run_start);
        entry.n_blocks    = static_cast<uint16_t>(used_blocks);  // trimmed; was run_len (full ~32 MB)
        entry.final_bytes = valid_pages * PAYLOAD_PER_PAGE;

        Status st;
        {
            FlightLogLockGuard guard(mutex_);
            st = index_.append(entry);
            if (st == Status::Ok) index_dirty = true;
        }
        if (st != Status::Ok) return st;

        // Trim the unused tail of the recovered range back to FREE, exactly like
        // finalizeFlight. Without this a brownout flight permanently holds its
        // full prealloc — ~4 such flights fill the chip despite <10 MB of real
        // data each (the 2026-06-25 silent log-loss / #281 root cause).
        const uint32_t free_count = span - used_blocks;
        if (free_count > 0) {
            FlightLogLockGuard guard(mutex_);
            bitmap_.markFreeRange(run_start + used_blocks, free_count);
            bitmap_dirty = true;
        }
    }

    if (index_dirty) {
        FlightLogLockGuard guard(mutex_);
        Status st = index_.save(*nand_, cfg_.metadata_blocks[0], cfg_.metadata_blocks[1]);
        if (st != Status::Ok) return st;
    }
    if (bitmap_dirty) {
        FlightLogLockGuard guard(mutex_);
        persistBitmap();
    }
    return Status::Ok;
}

Status TR_FlightLog::markBlockBad(uint32_t block) {
    FlightLogLockGuard guard(mutex_);
    if (!initialized_) return Status::NotInitialized;
    forgetBootOrphan();   // #1717
    if (block >= g_blocks_) return Status::OutOfRange;
    bitmap_.set(block, BLOCK_BAD);
    if (nand_) nand_->markBlockBad(block);
    persistBitmap();
    return Status::Ok;
}

void TR_FlightLog::seedBitmapFromBackend() {
    if (!nand_) return;
    for (uint32_t b = 0; b < g_blocks_; ++b) {
        if (nand_->isBlockBad(b)) bitmap_.set(b, BLOCK_BAD);
    }
}

void TR_FlightLog::persistBitmap() {
    if (!bitmap_store_) return;
    // #671: save exactly the runtime serialized length — the load side (and
    // the NVS store's exact-length match) expects (block_count+3)/4 bytes,
    // 256 on the mini's 1024-block chip. Saving sizeof(buf) = the 512-byte
    // MAX made every reboot restore fail there (blob length mismatch ->
    // fresh reseed -> brownout recovery finds no allocated ranges).
    uint8_t buf[BlockStateBitmap::SERIALIZED_SIZE_MAX];
    const size_t len = bitmap_.serializedSize();
    bitmap_.serializeTo(buf, len);
    bitmap_store_->save(buf, len);
}

void TR_FlightLog::requestPrepareFlight() {
    // Idempotent: extra calls collapse into one pending flag.
    // Skip when a flight is already active so the consumer doesn't see a
    // request that would immediately be rejected anyway.
    if (flight_active_) return;
    prepare_request_pending_ = true;
}

bool TR_FlightLog::servicePendingPrepareFlight(uint32_t& id_out, Status& status_out) {
    FlightLogLockGuard guard(mutex_);
    if (!prepare_request_pending_) return false;
    // Always clear the flag — even when we won't run prepareFlight — so a
    // stale request can't fire on a later iteration after the flight has
    // started through another path (e.g. a host-test direct call).
    prepare_request_pending_ = false;
    if (flight_active_) return false;

    status_out = prepareFlightLocked(id_out);
    return true;
}

bool TR_FlightLog::evictOldestLocked() {
    forgetBootOrphan();   // #1717
    // Pick the smallest flight_id in the index — the oldest finalized flight.
    // Skip the active flight defensively: it is never appended to the index
    // until finalizeFlight, so in practice there is nothing to skip here, but
    // the guard documents that an in-progress allocation is never a candidate.
    // Among equal ids (a flight armed before the deferred recovery adopted an
    // orphan shares its id), the first in the index: planEvictionsLocked()
    // plans in exactly this order (#1758).
    size_t   best_idx = index_.size();
    uint32_t best_id  = UINT32_MAX;
    for (size_t i = 0; i < index_.size(); ++i) {
        const FlightIndexEntry& e = index_.at(i);
        if (flight_active_ && e.flight_id == active_flight_id_) continue;
        if (best_idx == index_.size() || e.flight_id < best_id) { best_id = e.flight_id; best_idx = i; }
    }
    if (best_idx >= index_.size()) return false;  // nothing eligible to evict

    // Copy the filename out before delete: removeByFilename shifts + zeroes the
    // entry array, invalidating the reference returned by index_.at().
    char name[sizeof(index_.at(best_idx).filename)];
    std::strncpy(name, index_.at(best_idx).filename, sizeof(name));
    name[sizeof(name) - 1] = '\0';

    if (deleteFlightLocked(name) != Status::Ok) return false;
    ++auto_evicted_count_;
    last_evicted_flight_id_ = best_id;
    FL_LOGW("#315 auto-evict: deleted oldest flight id=%lu (%s)",
            (unsigned long)best_id, name);
    return true;
}

uint32_t TR_FlightLog::regionFreeBlocks() const {
    return static_cast<uint32_t>(bitmap_.countInStateRange(
        BLOCK_FREE, cfg_.flight_region_start, cfg_.flight_region_end));
}

TR_FlightLog::FlightRoom TR_FlightLog::planEvictionsLocked() const {
    const uint32_t rs = cfg_.flight_region_start;
    const uint32_t re = cfg_.flight_region_end;
    // Cap the requested headroom and room at the region size so a mis-set
    // target can't plan more space than the region holds.
    const uint32_t region_blocks = re - rs;
    uint32_t target_free = cfg_.auto_evict_target_free_blocks;
    if (target_free > region_blocks) target_free = region_blocks;
    uint32_t target_room = cfg_.auto_evict_target_room_blocks;
    if (target_room > region_blocks) target_room = region_blocks;

    // The plan runs on a copy, freed the way deleteFlightLocked frees. Free
    // blocks are counted WITHIN the flight region — not countInState(FREE),
    // which also counts the always-free pre-region (LFS) and metadata blocks
    // and would let the target be met without ever evicting (the count
    // regionFreeBlocks() and the OC's storage verdict use, #1235 item 3).
    BlockStateBitmap bm = bitmap_;
    bool taken[FlightIndex::MAX_ENTRIES] = {};
    const size_t n = index_.size() < FlightIndex::MAX_ENTRIES ? index_.size()
                                                              : FlightIndex::MAX_ENTRIES;
    // Frees the next oldest eligible flight on `bm`, in evictOldestLocked's
    // order: the smallest flight_id, the first in the index among equals (the
    // active flight is never in the index). False when none remains.
    auto evictNext = [&]() -> bool {
        size_t next = n;
        for (size_t i = 0; i < n; ++i) {
            if (taken[i]) continue;
            const FlightIndexEntry& e = index_.at(i);
            if (flight_active_ && e.flight_id == active_flight_id_) continue;
            if (next == n || e.flight_id < index_.at(next).flight_id) next = i;
        }
        if (next == n) return false;
        taken[next] = true;
        bm.markFreeRange(index_.at(next).start_block, index_.at(next).n_blocks);
        return true;
    };

    FlightRoom best;                // the fewest evictions giving the most room
    bool       have_best = false;
    FlightRoom first;               // no evictions at all
    for (uint32_t k = 0;; ++k) {
        // Every goal must hold for the pending flight to record cleanly:
        //  - a contiguous prealloc run exists (else prepareFlight returns NoSpace)
        //  - an index slot is free (else finalizeFlight can't append the entry)
        //  - free-block headroom is at/above the rolling-buffer floor
        //  - #1758: the stretch the flight starts in holds the room target. A
        //    flight is one contiguous range and nothing evicts in flight, so
        //    the total-free floor alone let a long flight's log end at the next
        //    stored flight, wherever eviction's hole happened to end.
        uint32_t start = 0, room = 0;
        const bool have_run   = bm.findRoomiestFree(cfg_.prealloc_blocks, rs, re, start, &room);
        const bool index_room = index_.size() - k < FlightIndex::MAX_ENTRIES;
        FlightRoom here;
        here.evictions          = k;
        here.room_blocks        = have_run ? room : 0;
        here.region_free_blocks = static_cast<uint32_t>(bm.countInStateRange(BLOCK_FREE, rs, re));
        if (k == 0) first = here;
        if (!cfg_.auto_evict_oldest) return here;
        if (have_run && index_room) {
            if (here.region_free_blocks >= target_free && here.room_blocks >= target_room) return here;
            if (!have_best || here.room_blocks > best.room_blocks) { best = here; have_best = true; }
        }
        if (!evictNext()) break;
    }
    // Every eligible flight planned away and the goals still unmet (more BAD
    // blocks than the target leaves, or orphaned runs not yet recovered split
    // the region): delete only the oldest flights that buy the most room
    // there is, and none if no flight can start even on an empty index (the
    // arm then fails loudly with NoSpace and keeps the flights).
    return have_best ? best : first;
}

uint32_t TR_FlightLog::evictOldestToTargetLocked() {
    // The plan (made in its own frame, so its bitmap copy is off the stack
    // before the deletes run), then that many of the oldest: the same order
    // and the same block frees, so the flight lands where the plan put it.
    const uint32_t planned = planEvictionsLocked().evictions;
    uint32_t evicted = 0;
    while (evicted < planned && evictOldestLocked()) ++evicted;
    return evicted;
}

uint32_t TR_FlightLog::planKeyLocked() const {
    // FNV-1a over the bitmap and the index entries' ranges: everything the
    // plan reads that can change at run time.
    uint32_t h = bitmap_.fingerprint();
    auto mix = [&h](uint32_t v) {
        for (int i = 0; i < 4; ++i) { h ^= (v >> (8 * i)) & 0xFFu; h *= 16777619u; }
    };
    mix(static_cast<uint32_t>(index_.size()));
    for (size_t i = 0; i < index_.size(); ++i) {
        const FlightIndexEntry& e = index_.at(i);
        mix(e.flight_id);
        mix((static_cast<uint32_t>(e.start_block) << 16) | e.n_blocks);
    }
    return h;
}

TR_FlightLog::FlightRoom TR_FlightLog::flightRoom() {
    FlightLogLockGuard guard(mutex_);
    FlightRoom r;
    if (!initialized_ || recovery_failed_) return r;
    if (flight_active_) {
        // The range it holds (less its BAD blocks), plus the FREE blocks
        // extendActiveRange can still take before a stored flight or the end.
        const uint32_t range_end = active_start_block_ + active_n_blocks_;
        uint32_t room = active_n_blocks_ -
            static_cast<uint32_t>(bitmap_.countInStateRange(BLOCK_BAD, active_start_block_, range_end));
        for (uint32_t b = range_end; b < cfg_.flight_region_end; ++b) {
            const BlockState st = bitmap_.get(b);
            if (st == BLOCK_BAD) continue;
            if (st != BLOCK_FREE) break;
            ++room;
        }
        r.room_blocks        = room;
        r.region_free_blocks = regionFreeBlocks();
        return r;
    }
    // The OC asks several times a second; the plan only changes when the
    // bitmap or the index does.
    const uint32_t key = planKeyLocked();
    if (!room_cache_valid_ || key != room_cache_key_) {
        room_cache_       = planEvictionsLocked();
        room_cache_key_   = key;
        room_cache_valid_ = true;
    }
    return room_cache_;
}

Status TR_FlightLog::prepareFlight(uint32_t& flight_id_out) {
    FlightLogLockGuard guard(mutex_);
    return prepareFlightLocked(flight_id_out);
}

Status TR_FlightLog::prepareFlightLocked(uint32_t& flight_id_out) {
    if (!initialized_) return Status::NotInitialized;
    forgetBootOrphan();   // #1717: blocks are allocated, an active flight begins
    if (recovery_failed_) return Status::RecoveryPending;   // #1127
    if (flight_active_)  return Status::Error;  // already flying — never evict an active flight

    // #315: rolling-buffer auto-eviction. Before picking a range, reclaim space
    // by deleting the oldest finalized flight(s) — enough for this prealloc to
    // fit, for an index slot to be free, to restore the free-block headroom
    // floor, and (#1758) to give the flight its room target unbroken. Off by
    // default (opt-in). No flight is active here (guarded above),
    // so eviction can never touch an in-progress allocation. Best-effort: if the
    // index holds nothing eligible the findRoomiestFree below still fails
    // loudly with NoSpace rather than arming a flight that can't be logged.
    if (cfg_.auto_evict_oldest) {
        evictOldestToTargetLocked();
    }

    uint32_t start = 0;
    if (!bitmap_.findRoomiestFree(cfg_.prealloc_blocks,
                                  cfg_.flight_region_start,
                                  cfg_.flight_region_end,
                                  start)) {
        return Status::NoSpace;
    }

    // Erase each block in the chosen range. On failure, mark the offending
    // block bad (persist + propagate), leave the bitmap otherwise untouched,
    // and return — caller can retry to pick a new range.
    for (uint32_t i = 0; i < cfg_.prealloc_blocks; ++i) {
        uint32_t b = start + i;
        if (!nand_->eraseBlock(b)) {
            bitmap_.set(b, BLOCK_BAD);
            nand_->markBlockBad(b);
            persistBitmap();
            return Status::BackendFailed;
        }
    }

    bitmap_.markAllocatedRange(start, cfg_.prealloc_blocks);
    persistBitmap();

    // Above every orphan's id while the deferred scan has not adopted them
    // (orphanNextIdAtBoot); once it has, the index covers them.
    active_flight_id_   = index_.nextFlightId();
    if (active_flight_id_ < orphan_next_id_) active_flight_id_ = orphan_next_id_;
    active_start_block_ = start;
    active_n_blocks_    = cfg_.prealloc_blocks;
    active_next_page_   = 0;
    extension_count_    = 0;
    flight_active_      = true;
    flight_id_out       = active_flight_id_;
    return Status::Ok;
}

bool TR_FlightLog::extendActiveRange() {
    const uint32_t ext_start = active_start_block_ + active_n_blocks_;

    // Require contiguous FREE blocks — keeps the flight as a single range
    // so FlightIndexEntry's start_block + n_blocks model stays valid.  A BAD
    // block in the window is part of the range and stepped over, as the
    // writer, the reader and recovery already step over a block that goes bad
    // mid-flight (V10 pre-flight review, 2026-11-18: one factory-bad block in
    // the way used to end the log for the rest of the flight).
    //
    // #1758: the window is up to extend_blocks FREE blocks, BAD ones stepped
    // over without counting, and it stops short at a stored flight or the
    // region end instead of being refused whole. Refusing it left up to
    // extend_blocks - 1 FREE blocks unused before either one, and a run of
    // extend_blocks BAD blocks ended the log; both are room flightRoom() and
    // findRoomiestFree count. A trailing BAD block is left out of the range.
    uint32_t ext_end = ext_start;   // one past the last FREE block taken
    uint32_t taken   = 0;
    for (uint32_t b = ext_start; b < cfg_.flight_region_end && taken < cfg_.extend_blocks; ++b) {
        const BlockState st = bitmap_.get(b);
        if (st == BLOCK_BAD) continue;
        if (st != BLOCK_FREE) break;
        ext_end = b + 1;
        ++taken;
    }
    if (ext_end == ext_start) return false;
    for (uint32_t b = ext_start; b < ext_end; ++b) {
        if (bitmap_.get(b) == BLOCK_BAD) continue;
        if (!nand_->eraseBlock(b)) {
            bitmap_.set(b, BLOCK_BAD);
            nand_->markBlockBad(b);
            persistBitmap();
            return false;
        }
    }
    const uint32_t n = ext_end - ext_start;
    bitmap_.markAllocatedRange(ext_start, n);   // skips BAD
    persistBitmap();
    active_n_blocks_ += n;
    ++extension_count_;
    return true;
}

Status TR_FlightLog::writeFrame(const uint8_t* payload, size_t payload_len) {
    // #510: time the lock acquire — the flush task's per-page wait behind a
    // concurrent listFlights/readFlightPage (#388) is otherwise invisible.
    const int64_t lk_t0 = monotonic_us();
    FlightLogLockGuard guard(mutex_);
    const uint32_t lk_dt = static_cast<uint32_t>(monotonic_us() - lk_t0);
    if (lk_dt > wf_lock_wait_max_us_) wf_lock_wait_max_us_ = lk_dt;
    wf_lock_wait_sum_us_ = wf_lock_wait_sum_us_ + lk_dt;
    if (!initialized_)   return Status::NotInitialized;
    // #1127: a failed recovery refuses new flights (prepareFlight). A flight
    // already being logged when a DEFERRED scan fails keeps logging: its range
    // is FREE blocks it took at arm, apart from every orphan, and stopping it
    // would lose the rest of a flight to protect nothing.
    if (recovery_failed_ && !flight_active_) return Status::RecoveryPending;
    if (!flight_active_) return Status::Error;
    if (payload == nullptr && payload_len != 0) return Status::OutOfRange;
    if (payload_len > payloadPerPage()) return Status::OutOfRange;

    uint8_t* const page = page_buf_;  // off-stack (member); guarded by mutex_ above
    std::memset(page, 0xFF, g_page_);

    PageHeader hdr;
    hdr.crc32      = 0;  // placeholder
    hdr.magic      = FPAG_MAGIC;
    hdr.seq_number = active_next_page_;
    hdr.flight_id  = active_flight_id_;
    std::memcpy(page, &hdr, sizeof(hdr));
    if (payload_len > 0) std::memcpy(page + sizeof(hdr), payload, payload_len);

    const uint32_t crc = page_crc(page, g_page_);
    std::memcpy(page, &crc, sizeof(crc));

    // Lock already held; go straight to the locked impl (writePage would
    // re-take the non-recursive mutex and deadlock).
    return writePageLocked(page);
}

Status TR_FlightLog::writePage(const uint8_t* page) {
    FlightLogLockGuard guard(mutex_);
    return writePageLocked(page);
}

Status TR_FlightLog::writePageLocked(const uint8_t* page) {
    if (!initialized_)  return Status::NotInitialized;
    if (!flight_active_) return Status::Error;
    if (page == nullptr) return Status::OutOfRange;

    // Retry bounded by the number of blocks available, so a long run of bad
    // blocks can't cause an infinite loop.
    const uint32_t max_block_attempts = active_n_blocks_ + cfg_.extend_blocks;
    for (uint32_t attempt = 0; attempt <= max_block_attempts; ++attempt) {
        const uint32_t rel_block = active_next_page_ / g_ppb_;
        if (rel_block >= active_n_blocks_) {
            if (!extendActiveRange()) return Status::NoSpace;
            continue;  // re-check bounds with the extended range
        }
        const uint32_t abs_block  = active_start_block_ + rel_block;
        const uint32_t page_in_blk = active_next_page_ % g_ppb_;

        // A BAD block inside the range (an extension stepped over it): skip it
        // whole, as the reader does. The cursor only ever reaches it at page 0.
        if (bitmap_.get(abs_block) == BLOCK_BAD) {
            active_next_page_ = (rel_block + 1) * g_ppb_;
            continue;
        }

        if (nand_->programPage(abs_block, page_in_blk, page)) {
            ++active_next_page_;
            return Status::Ok;
        }

        // Program failure: runtime bad-block discovery. Retire the block and
        // relocate the `page_in_blk` pages already written in it (#371) so the
        // reader's dense-page model stays correct and no logged data is
        // orphaned. active_next_page_ is left just past the relocated pages, in
        // a good block, so the loop retries `page` there.
        if (!retireBlockAndSalvage(abs_block, rel_block, page_in_blk)) {
            return Status::NoSpace;
        }
    }
    return Status::BackendFailed;
}

bool TR_FlightLog::retireBlockAndSalvage(uint32_t bad_block, uint32_t rel_block,
                                         uint32_t valid_pages) {
    bitmap_.set(bad_block, BLOCK_BAD);
    nand_->markBlockBad(bad_block);
    persistBitmap();
    active_next_page_ = (rel_block + 1) * g_ppb_;

    // page-0 failure: nothing was written in the block, so it's a clean
    // skip-ahead with nothing to relocate (the pre-#371 behavior).
    if (valid_pages == 0) return true;
    ++salvaged_block_count_;
    FL_LOGW("bad block %lu mid-flight: relocating %lu already-written pages (#371)",
            (unsigned long)bad_block, (unsigned long)valid_pages);

    // Relocate pages [0, valid_pages) of the retired block to the write cursor.
    // The retired block stays readable for its committed pages (the failure was
    // on a *later* page), so each is re-read and reprogrammed densely into the
    // next good block(s). If a destination block ALSO fails mid-relocation, it
    // is retired and the relocation restarts from page 0 into the next block —
    // the source is still readable, so no salvaged page is dropped. Restarts
    // are bounded by the number of blocks available.
    uint8_t* const buf = salvage_buf_;  // off-stack (member); live while page_buf_ is held
    const uint32_t max_restarts = active_n_blocks_ + cfg_.extend_blocks + 1;
    uint32_t restarts = 0;
    uint32_t i = 0;
    while (i < valid_pages) {
        const uint32_t rel = active_next_page_ / g_ppb_;
        if (rel >= active_n_blocks_) {
            if (!extendActiveRange()) return false;
            continue;
        }
        const uint32_t dst_block = active_start_block_ + rel;
        const uint32_t dst_page  = active_next_page_ % g_ppb_;
        if (bitmap_.get(dst_block) == BLOCK_BAD) {   // stepped over by an extension
            active_next_page_ = (rel + 1) * g_ppb_;
            if (++restarts > max_restarts) return false;
            continue;
        }

        if (!nand_->readPage(bad_block, i, buf)) {
            // A committed page that will not read back — genuinely
            // unrecoverable. Write a zeroed placeholder so the stream stays
            // aligned (one lost page beats orphaning + shifting everything
            // after it) and flag it.
            std::memset(buf, 0x00, g_page_);
            ++unrecoverable_page_count_;
            FL_LOGW("salvage: page %lu of retired block %lu unreadable — wrote "
                    "zeroed placeholder to keep the stream aligned (#371)",
                    (unsigned long)i, (unsigned long)bad_block);
        }

        if (nand_->programPage(dst_block, dst_page, buf)) {
            ++active_next_page_;
            ++i;
            continue;
        }

        // Destination is bad too. Retire it and restart the relocation from
        // page 0 into the next block (partial copies in dst_block are abandoned
        // — the reader skips the BAD block; the source is re-read intact).
        bitmap_.set(dst_block, BLOCK_BAD);
        nand_->markBlockBad(dst_block);
        persistBitmap();
        active_next_page_ = (rel + 1) * g_ppb_;
        i = 0;
        if (++restarts > max_restarts) return false;
    }
    return true;
}

Status TR_FlightLog::finalizeFlight(const char* filename, uint32_t final_bytes) {
    FlightLogLockGuard guard(mutex_);
    if (!initialized_)    return Status::NotInitialized;
    forgetBootOrphan();   // #1717
    if (!flight_active_)  return Status::Error;
    if (filename == nullptr) return Status::OutOfRange;

    // Trim unused tail blocks from the preallocated range. prepareFlight
    // reserves cfg_.prealloc_blocks of headroom for long flights — 40 blocks,
    // which is ~10 MB or ~5 MB depending on the chip's block size (#671) —
    // but most flights are far shorter. Without trimming, the bitmap caps out
    // at floor(region / prealloc) simultaneous flights regardless of their
    // actual size. Round up to cover any partial trailing page.
    //
    // #837 item 10: this said "32 MB", the flat size of the retired 256-block
    // preallocation, which mis-stated the reservation by 3x and hardcoded a
    // number that stopped being fixed at all once geometry went runtime.
    // Size the kept range to cover BOTH the logical length (final_bytes) AND the
    // PHYSICAL span actually consumed (active_next_page_). They match for a
    // normal flight; a runtime bad block makes the physical span larger
    // (writePage marked it BAD and skipped ahead), so trimming by the logical
    // count alone would free post-gap blocks that still hold data (silent loss +
    // reuse) and shrink n_blocks below the span readFlightPage walks (#276).
    // final_bytes stays the logical byte length.
    const uint32_t PAYLOAD_PER_PAGE = payloadPerPage();
    const uint32_t logical_pages = (final_bytes == 0) ? 0
        : (final_bytes + PAYLOAD_PER_PAGE - 1) / PAYLOAD_PER_PAGE;
    const uint32_t span_pages = (active_next_page_ > logical_pages)
                                    ? active_next_page_ : logical_pages;
    uint32_t used_blocks = (span_pages + g_ppb_ - 1) / g_ppb_;
    if (used_blocks > active_n_blocks_) used_blocks = active_n_blocks_;

    // #1155 item 19: refuse a name the entry cannot hold rather than strncpy
    // it down to 27 characters — the OC's old MRAM-recovery name overflowed
    // from id 10 and lost its ".bin" mid-extension. The flight stays active
    // (nothing is lost: an unindexed range with valid page headers is what
    // the boot-time orphan recovery re-adopts), and the caller gets a status
    // it can log instead of a file the apps cannot recognise.
    if (filename == nullptr || std::strlen(filename) > sizeof(FlightIndexEntry::filename) - 1) {
        return Status::OutOfRange;
    }
    FlightIndexEntry entry{};
    entry.magic       = FLGT_MAGIC;
    entry.flight_id   = active_flight_id_;
    std::strncpy(entry.filename, filename, sizeof(entry.filename) - 1);
    entry.start_block = static_cast<uint16_t>(active_start_block_);
    entry.n_blocks    = static_cast<uint16_t>(used_blocks);
    entry.final_bytes = final_bytes;

    // Release the tail in the in-memory bitmap before the index save so the
    // two stay consistent; roll back if the save fails.
    const uint32_t free_start = active_start_block_ + used_blocks;
    const uint32_t free_count = active_n_blocks_ - used_blocks;
    if (free_count > 0) {
        bitmap_.markFreeRange(free_start, free_count);
    }

    Status st = index_.append(entry);
    if (st != Status::Ok) {
        if (free_count > 0) bitmap_.markAllocatedRange(free_start, free_count);
        return st;
    }
    st = index_.save(*nand_, cfg_.metadata_blocks[0], cfg_.metadata_blocks[1]);
    if (st != Status::Ok) {
        index_.removeByFilename(entry.filename);  // roll back the in-memory add
        if (free_count > 0) bitmap_.markAllocatedRange(free_start, free_count);
        return st;
    }

    // Both RAM structures are committed; persist the bitmap change to NAND.
    // If this fails the on-disk bitmap just keeps the tail marked allocated;
    // startup recovery handles orphan-allocated blocks by detecting them as
    // not-in-index and releasing them on the next boot.
    if (free_count > 0) persistBitmap();

    flight_active_    = false;
    active_flight_id_ = 0;
    active_next_page_ = 0;
    return Status::Ok;
}

size_t TR_FlightLog::listFlights(FlightIndexEntry* out, size_t max,
                                 size_t page, size_t per_page) {
    FlightLogLockGuard guard(mutex_);
    if (!initialized_ || out == nullptr || max == 0 || per_page == 0) return 0;
    const size_t total = index_.size();
    const size_t start = page * per_page;
    if (start >= total) return 0;
    size_t copied = 0;
    for (size_t i = start; i < total && copied < per_page && copied < max; ++i) {
        out[copied++] = index_.at(i);
    }
    return copied;
}

Status TR_FlightLog::readFlightPage(const char* filename, uint32_t offset,
                                    uint8_t* buf, size_t buf_len,
                                    size_t& out_len) {
    FlightLogLockGuard guard(mutex_);
    out_len = 0;
    if (!initialized_)       return Status::NotInitialized;
    if (filename == nullptr) return Status::OutOfRange;
    if (buf == nullptr || buf_len == 0) return Status::OutOfRange;

    const FlightIndexEntry* entry = index_.findByFilename(filename);
    if (entry == nullptr) return Status::NotFound;
    if (offset >= entry->final_bytes) return Status::Ok;  // EOF, 0 bytes

    // Pages written via writeFrame have a 16 B PageHeader at the start, so the
    // *logical* payload size per NAND page is PAYLOAD_PER_PAGE = 4080 bytes.
    // `offset` is a byte index into the concatenated payload stream (what the
    // caller originally enqueued via writeFrame / what iOS expects to
    // download). The mapping hops the 16 B header on each physical page.
    const uint32_t PAYLOAD_PER_PAGE = payloadPerPage();

    const uint32_t logical_page = offset / PAYLOAD_PER_PAGE;
    const uint32_t byte_in_pl   = offset % PAYLOAD_PER_PAGE;

    // Map the logical payload page to a physical (block, page). Pages are laid
    // down sequentially, but a block that failed to program mid-flight was
    // marked BLOCK_BAD and skipped whole at write time (writePage), leaving a
    // physical gap. Walk the entry's blocks skipping BLOCK_BAD ones so logical
    // page L lands on the L-th *valid* page instead of hopping into a 0xFF gap
    // and returning interior garbage (#276). Gaps are block-aligned and recorded
    // in the bitmap, so this stays O(blocks) with no extra NAND reads.
    const uint32_t end_block = entry->start_block + entry->n_blocks;
    uint32_t abs_block  = entry->start_block;
    uint32_t pages_left = logical_page;
    while (abs_block < end_block && bitmap_.get(abs_block) == BLOCK_BAD) ++abs_block;
    while (pages_left >= g_ppb_) {
        pages_left -= g_ppb_;
        ++abs_block;
        while (abs_block < end_block && bitmap_.get(abs_block) == BLOCK_BAD) ++abs_block;
    }
    if (abs_block >= end_block) return Status::Ok;  // logical page past the data
    const uint32_t page_in_blk = pages_left;

    uint8_t* const page = page_buf_;  // off-stack (member); guarded by mutex_ above
    if (!nand_->readPage(abs_block, page_in_blk, page)) return Status::BackendFailed;

    // Payload starts right after the PageHeader on the physical page.
    const uint32_t src_off = static_cast<uint32_t>(sizeof(PageHeader)) + byte_in_pl;
    const uint32_t avail_in_page = PAYLOAD_PER_PAGE - byte_in_pl;
    const uint32_t avail_in_file = entry->final_bytes - offset;
    size_t copy_len = buf_len;
    if (copy_len > avail_in_page) copy_len = avail_in_page;
    if (copy_len > avail_in_file) copy_len = avail_in_file;

    std::memcpy(buf, page + src_off, copy_len);
    out_len = copy_len;
    return Status::Ok;
}

Status TR_FlightLog::deleteFlight(const char* filename) {
    FlightLogLockGuard guard(mutex_);
    return deleteFlightLocked(filename);
}

Status TR_FlightLog::deleteFlightLocked(const char* filename) {
    if (!initialized_)       return Status::NotInitialized;
    forgetBootOrphan();   // #1717
    if (filename == nullptr) return Status::OutOfRange;

    const FlightIndexEntry* entry = index_.findByFilename(filename);
    if (entry == nullptr) return Status::NotFound;

    const uint16_t start_block = entry->start_block;
    const uint16_t n_blocks    = entry->n_blocks;
    // #1155 item 13: keep a copy for rollback — removeByFilename shifts the
    // array and invalidates `entry`.
    const FlightIndexEntry saved = *entry;

    if (!index_.removeByFilename(filename)) return Status::NotFound;

    Status st = index_.save(*nand_, cfg_.metadata_blocks[0], cfg_.metadata_blocks[1]);
    if (st != Status::Ok) {
        // Mirror finalizeFlight: a failed persist must leave the RAM index, the
        // on-NAND index and the bitmap all agreeing the flight still exists.
        // Before this the entry was gone from RAM with its blocks still
        // ALLOCATED, and the cmd-3 handler rebuilt the listing from that RAM
        // index, so the app saw the file vanish on a delete that failed.
        // Re-appended at the end: eviction picks the smallest flight_id, not
        // a position, so order does not matter.
        (void)index_.append(saved);
        return st;
    }

    // Release the blocks for reuse. markFreeRange skips BAD blocks so any
    // runtime-discovered bad blocks inside the flight remain excluded.
    bitmap_.markFreeRange(start_block, n_blocks);
    persistBitmap();
    return Status::Ok;
}

Status TR_FlightLog::renameFlight(const char* old_name, const char* new_name) {
    FlightLogLockGuard guard(mutex_);
    if (!initialized_)      return Status::NotInitialized;
    if (old_name == nullptr || new_name == nullptr) return Status::OutOfRange;

    if (!index_.rename(old_name, new_name)) return Status::NotFound;

    Status st = index_.save(*nand_, cfg_.metadata_blocks[0], cfg_.metadata_blocks[1]);
    if (st != Status::Ok) return st;
    return Status::Ok;
}

}  // namespace tr_flightlog
