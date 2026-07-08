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

// CRC32 of a page that starts with a PageHeader: covers bytes[4..end].
uint32_t page_crc(const uint8_t* page) {
    constexpr size_t skip = sizeof(uint32_t);  // skip the crc32 field itself
    return calcCRC32(page + skip, NAND_PAGE_SIZE - skip);
}

bool page_is_all_ones(const uint8_t* page) {
    for (size_t i = 0; i < NAND_PAGE_SIZE; ++i) {
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

}  // namespace

Status TR_FlightLog::begin(TR_NandBackend& nand, const Config& cfg,
                            TR_BitmapStore* bitmap_store) {
    nand_         = &nand;
    cfg_          = cfg;
    bitmap_store_ = bitmap_store;

    if (cfg_.flight_region_start >= cfg_.flight_region_end ||
        cfg_.flight_region_end > NAND_BLOCK_COUNT) {
        return Status::OutOfRange;
    }
    if (cfg_.prealloc_blocks == 0 ||
        cfg_.prealloc_blocks > (cfg_.flight_region_end - cfg_.flight_region_start)) {
        return Status::OutOfRange;
    }

    // Try to restore bitmap from persistent store. If nothing saved (or wrong
    // size) start fresh and seed from the NAND backend's bad-block oracle.
    uint8_t buf[BlockStateBitmap::SERIALIZED_SIZE];
    bool restored = bitmap_store_ && bitmap_store_->load(buf, sizeof(buf));
    if (restored) {
        bitmap_.deserializeFrom(buf, sizeof(buf));
    } else {
        bitmap_.clear();
        seedBitmapFromBackend();
        persistBitmap();
    }

    Status idx_st = index_.load(*nand_, cfg_.metadata_blocks[0], cfg_.metadata_blocks[1]);
    if (idx_st != Status::Ok && idx_st != Status::CrcMismatch) return idx_st;
    // CrcMismatch on a fresh chip (or both copies corrupt) is recoverable;
    // the index is already cleared and we proceed with an empty one.

    initialized_ = true;

    // Recover any flights that were in progress when the last boot was cut
    // short. Safe to run every boot — a no-op when nothing is orphaned.
    Status recov_st = scanForBrownoutRecovery();
    if (recov_st != Status::Ok) {
        initialized_ = false;
        return recov_st;
    }
    return Status::Ok;
}

Status TR_FlightLog::scanForBrownoutRecovery() {
    if (!initialized_) return Status::NotInitialized;

    // A block is "orphaned" if it's ALLOCATED but not covered by any index
    // entry. We walk the flight region looking for contiguous orphaned runs.
    auto is_in_index = [&](uint32_t block) {
        for (size_t i = 0; i < index_.size(); ++i) {
            const auto& e = index_.at(i);
            if (block >= e.start_block &&
                block <  static_cast<uint32_t>(e.start_block) + e.n_blocks) {
                return true;
            }
        }
        return false;
    };

    uint8_t page[NAND_PAGE_SIZE];
    bool index_dirty  = false;
    bool bitmap_dirty = false;

    // #277: progress + a wall-clock budget so a near-full chip's scan is visible
    // and can't approach the app's 180 s power-on watchdog.
    auto now = [&]() -> uint32_t { return cfg_.now_ms ? cfg_.now_ms() : monotonic_ms(); };
    const uint32_t t0            = now();
    const uint32_t region_blocks = static_cast<uint32_t>(cfg_.flight_region_end -
                                                          cfg_.flight_region_start);
    uint32_t blocks_scanned = 0;

    uint32_t b = cfg_.flight_region_start;
    while (b < cfg_.flight_region_end) {
        // Budget is checked only at run boundaries: each run is scanned to
        // completion (a partial scan would synthesize a wrong-length entry), so
        // an over-budget pass leaves the remaining orphaned runs for next boot.
        if (cfg_.recovery_budget_ms != 0 && now() - t0 > cfg_.recovery_budget_ms) {
            FL_LOGW("recovery: %lu ms budget hit at block %lu/%lu — deferring the "
                    "rest to next boot",
                    (unsigned long)cfg_.recovery_budget_ms,
                    (unsigned long)(b - cfg_.flight_region_start),
                    (unsigned long)region_blocks);
            break;
        }
        if (bitmap_.get(b) != BLOCK_ALLOCATED || is_in_index(b)) {
            ++b;
            continue;
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
        // distinct un-indexed flights never coexist to be wrongly merged here.
        while (b < cfg_.flight_region_end &&
               (bitmap_.get(b) == BLOCK_ALLOCATED || bitmap_.get(b) == BLOCK_BAD) &&
               !is_in_index(b)) {
            ++b;
        }
        const uint32_t run_len = b - run_start;

        // Scan the run for pages with valid PageHeader CRC. Track the highest
        // seq_number + associated flight_id.
        int32_t  last_good_page_rel = -1;   // physical page index within the run
        uint32_t last_seq            = 0;
        uint32_t last_flight_id      = 0;
        uint32_t valid_pages         = 0;   // #276: count of magic+CRC pages
        bool     saw_any             = false;

        for (uint32_t i = 0; i < run_len; ++i) {
            if (++blocks_scanned % 256 == 0) {
                FL_LOGI("recovery: scanned %lu/%lu blocks (%lu ms)",
                        (unsigned long)blocks_scanned, (unsigned long)region_blocks,
                        (unsigned long)(now() - t0));
            }
            const uint32_t blk = run_start + i;
            if (bitmap_.get(blk) == BLOCK_BAD) continue;

            // Fast-path: read just the first page of the block. If its header
            // doesn't carry FPAG_MAGIC, no writeFrame ever ran in this block
            // (the rest is 0xFF by contract — each block in an allocated
            // range was erased before use). Skip the 63 remaining reads.
            if (!nand_->readPage(blk, 0, page)) continue;
            {
                PageHeader hdr0;
                std::memcpy(&hdr0, page, sizeof(hdr0));
                if (hdr0.magic != FPAG_MAGIC)
                {
                    // Block was erased but never programmed — nothing to find.
                    yield_to_scheduler();
                    continue;
                }
                // First page looked valid; handle it below along with the rest.
                if (hdr0.crc32 == page_crc(page))
                {
                    ++valid_pages;
                    if (!saw_any || hdr0.seq_number > last_seq) {
                        last_seq           = hdr0.seq_number;
                        last_flight_id     = hdr0.flight_id;
                        last_good_page_rel = static_cast<int32_t>(i * NAND_PAGES_PER_BLK);
                        saw_any            = true;
                    }
                }
            }
            for (uint32_t p = 1; p < NAND_PAGES_PER_BLK; ++p) {
                if (!nand_->readPage(blk, p, page)) continue;
                if (page_is_all_ones(page)) continue;   // unwritten

                PageHeader hdr;
                std::memcpy(&hdr, page, sizeof(hdr));
                if (hdr.magic != FPAG_MAGIC) continue;
                if (hdr.crc32 != page_crc(page)) continue;
                ++valid_pages;

                if (!saw_any || hdr.seq_number > last_seq) {
                    last_seq           = hdr.seq_number;
                    last_flight_id     = hdr.flight_id;
                    last_good_page_rel = static_cast<int32_t>(i * NAND_PAGES_PER_BLK + p);
                    saw_any            = true;
                }
            }
            // Yield after each block so the per-block scan cost can be
            // absorbed without starving IDLE1 and tripping task_wdt.
            yield_to_scheduler();
        }

        if (!saw_any) {
            // No recoverable data in this range — release it.
            bitmap_.markFreeRange(run_start, run_len);
            bitmap_dirty = true;
            continue;
        }

        // Synthesize a partial flight entry. final_bytes is the logical PAYLOAD
        // byte count actually flushed to NAND — #273: PAYLOAD_PER_PAGE (2032),
        // NOT NAND_PAGE_SIZE (2048), or the downloader walks past the last real
        // page into PageHeader/0xFF bytes (corrupt tail, size ~0.8% too large).
        constexpr uint32_t PAYLOAD_PER_PAGE = NAND_PAGE_SIZE - sizeof(PageHeader);
        // #276: length is the count of *valid* pages actually present, NOT the
        // physical span of the highest-seq page. If the flight hit a bad block
        // mid-write, writePage marked it BAD and skipped to the next block, so
        // the span over-counts by the skipped block(s) — deriving bytes from it
        // injects interior 0xFF garbage and the wrong length. The physical span
        // (used_pages) still sizes the block range to keep allocated; the gap is
        // walked on read (readFlightPage).
        const uint32_t used_pages  = static_cast<uint32_t>(last_good_page_rel + 1);
        uint32_t used_blocks = (used_pages + NAND_PAGES_PER_BLK - 1) / NAND_PAGES_PER_BLK;
        if (used_blocks > run_len) used_blocks = run_len;

        FlightIndexEntry entry{};
        entry.magic       = FLGT_MAGIC;
        entry.flight_id   = last_flight_id;
        std::snprintf(entry.filename, sizeof(entry.filename),
                      "flight_recovered_%lu.bin",
                      static_cast<unsigned long>(last_flight_id));
        entry.start_block = static_cast<uint16_t>(run_start);
        entry.n_blocks    = static_cast<uint16_t>(used_blocks);  // trimmed; was run_len (full ~32 MB)
        entry.final_bytes = valid_pages * PAYLOAD_PER_PAGE;

        Status st = index_.append(entry);
        if (st != Status::Ok) return st;
        index_dirty = true;

        // Trim the unused tail of the recovered range back to FREE, exactly like
        // finalizeFlight. Without this a brownout flight permanently holds its
        // full prealloc — ~4 such flights fill the chip despite <10 MB of real
        // data each (the 2026-06-25 silent log-loss / #281 root cause).
        const uint32_t free_count = run_len - used_blocks;
        if (free_count > 0) {
            bitmap_.markFreeRange(run_start + used_blocks, free_count);
            bitmap_dirty = true;
        }
    }

    if (index_dirty) {
        Status st = index_.save(*nand_, cfg_.metadata_blocks[0], cfg_.metadata_blocks[1]);
        if (st != Status::Ok) return st;
    }
    if (bitmap_dirty) persistBitmap();
    return Status::Ok;
}

Status TR_FlightLog::markBlockBad(uint32_t block) {
    FlightLogLockGuard guard(mutex_);
    if (!initialized_) return Status::NotInitialized;
    if (block >= NAND_BLOCK_COUNT) return Status::OutOfRange;
    bitmap_.set(block, BLOCK_BAD);
    if (nand_) nand_->markBlockBad(block);
    persistBitmap();
    return Status::Ok;
}

void TR_FlightLog::seedBitmapFromBackend() {
    if (!nand_) return;
    for (uint32_t b = 0; b < NAND_BLOCK_COUNT; ++b) {
        if (nand_->isBlockBad(b)) bitmap_.set(b, BLOCK_BAD);
    }
}

void TR_FlightLog::persistBitmap() {
    if (!bitmap_store_) return;
    uint8_t buf[BlockStateBitmap::SERIALIZED_SIZE];
    bitmap_.serializeTo(buf, sizeof(buf));
    bitmap_store_->save(buf, sizeof(buf));
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

Status TR_FlightLog::prepareFlight(uint32_t& flight_id_out) {
    FlightLogLockGuard guard(mutex_);
    return prepareFlightLocked(flight_id_out);
}

Status TR_FlightLog::prepareFlightLocked(uint32_t& flight_id_out) {
    if (!initialized_) return Status::NotInitialized;
    if (flight_active_)  return Status::Error;  // already flying

    uint32_t start = 0;
    if (!bitmap_.findContiguousFree(cfg_.prealloc_blocks,
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

    active_flight_id_   = index_.nextFlightId();
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
    const uint32_t ext_end   = ext_start + cfg_.extend_blocks;
    if (ext_end > cfg_.flight_region_end) return false;

    // Require contiguous FREE blocks — keeps the flight as a single range
    // so FlightIndexEntry's start_block + n_blocks model stays valid.
    for (uint32_t b = ext_start; b < ext_end; ++b) {
        if (bitmap_.get(b) != BLOCK_FREE) return false;
    }
    for (uint32_t b = ext_start; b < ext_end; ++b) {
        if (!nand_->eraseBlock(b)) {
            bitmap_.set(b, BLOCK_BAD);
            nand_->markBlockBad(b);
            persistBitmap();
            return false;
        }
    }
    bitmap_.markAllocatedRange(ext_start, cfg_.extend_blocks);
    persistBitmap();
    active_n_blocks_ += cfg_.extend_blocks;
    ++extension_count_;
    return true;
}

Status TR_FlightLog::writeFrame(const uint8_t* payload, size_t payload_len) {
    FlightLogLockGuard guard(mutex_);
    if (!initialized_)   return Status::NotInitialized;
    if (!flight_active_) return Status::Error;
    if (payload == nullptr && payload_len != 0) return Status::OutOfRange;
    if (payload_len > NAND_PAGE_SIZE - sizeof(PageHeader)) return Status::OutOfRange;

    uint8_t page[NAND_PAGE_SIZE];
    std::memset(page, 0xFF, sizeof(page));

    PageHeader hdr;
    hdr.crc32      = 0;  // placeholder
    hdr.magic      = FPAG_MAGIC;
    hdr.seq_number = active_next_page_;
    hdr.flight_id  = active_flight_id_;
    std::memcpy(page, &hdr, sizeof(hdr));
    if (payload_len > 0) std::memcpy(page + sizeof(hdr), payload, payload_len);

    const uint32_t crc = page_crc(page);
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
        const uint32_t rel_block = active_next_page_ / NAND_PAGES_PER_BLK;
        if (rel_block >= active_n_blocks_) {
            if (!extendActiveRange()) return Status::NoSpace;
            continue;  // re-check bounds with the extended range
        }
        const uint32_t abs_block  = active_start_block_ + rel_block;
        const uint32_t page_in_blk = active_next_page_ % NAND_PAGES_PER_BLK;

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
    active_next_page_ = (rel_block + 1) * NAND_PAGES_PER_BLK;

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
    uint8_t buf[NAND_PAGE_SIZE];
    const uint32_t max_restarts = active_n_blocks_ + cfg_.extend_blocks + 1;
    uint32_t restarts = 0;
    uint32_t i = 0;
    while (i < valid_pages) {
        const uint32_t rel = active_next_page_ / NAND_PAGES_PER_BLK;
        if (rel >= active_n_blocks_) {
            if (!extendActiveRange()) return false;
            continue;
        }
        const uint32_t dst_block = active_start_block_ + rel;
        const uint32_t dst_page  = active_next_page_ % NAND_PAGES_PER_BLK;

        if (!nand_->readPage(bad_block, i, buf)) {
            // A committed page that will not read back — genuinely
            // unrecoverable. Write a zeroed placeholder so the stream stays
            // aligned (one lost page beats orphaning + shifting everything
            // after it) and flag it.
            std::memset(buf, 0x00, sizeof(buf));
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
        active_next_page_ = (rel + 1) * NAND_PAGES_PER_BLK;
        i = 0;
        if (++restarts > max_restarts) return false;
    }
    return true;
}

Status TR_FlightLog::finalizeFlight(const char* filename, uint32_t final_bytes) {
    FlightLogLockGuard guard(mutex_);
    if (!initialized_)    return Status::NotInitialized;
    if (!flight_active_)  return Status::Error;
    if (filename == nullptr) return Status::OutOfRange;

    // Trim unused tail blocks from the preallocated range. prepareFlight
    // reserves 32 MB of headroom for long flights, but most flights are far
    // shorter — without trimming, the bitmap caps out at floor(region / 32MB)
    // simultaneous flights regardless of their actual size. Round up to cover
    // any partial trailing page.
    // Size the kept range to cover BOTH the logical length (final_bytes) AND the
    // PHYSICAL span actually consumed (active_next_page_). They match for a
    // normal flight; a runtime bad block makes the physical span larger
    // (writePage marked it BAD and skipped ahead), so trimming by the logical
    // count alone would free post-gap blocks that still hold data (silent loss +
    // reuse) and shrink n_blocks below the span readFlightPage walks (#276).
    // final_bytes stays the logical byte length.
    constexpr uint32_t PAYLOAD_PER_PAGE = NAND_PAGE_SIZE - sizeof(PageHeader);
    const uint32_t logical_pages = (final_bytes == 0) ? 0
        : (final_bytes + PAYLOAD_PER_PAGE - 1) / PAYLOAD_PER_PAGE;
    const uint32_t span_pages = (active_next_page_ > logical_pages)
                                    ? active_next_page_ : logical_pages;
    uint32_t used_blocks = (span_pages + NAND_PAGES_PER_BLK - 1) / NAND_PAGES_PER_BLK;
    if (used_blocks > active_n_blocks_) used_blocks = active_n_blocks_;

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
    // *logical* payload size per NAND page is PAYLOAD_PER_PAGE = 2032 bytes.
    // `offset` is a byte index into the concatenated payload stream (what the
    // caller originally enqueued via writeFrame / what iOS expects to
    // download). The mapping hops the 16 B header on each physical page.
    constexpr uint32_t PAYLOAD_PER_PAGE = NAND_PAGE_SIZE - sizeof(PageHeader);

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
    while (pages_left >= NAND_PAGES_PER_BLK) {
        pages_left -= NAND_PAGES_PER_BLK;
        ++abs_block;
        while (abs_block < end_block && bitmap_.get(abs_block) == BLOCK_BAD) ++abs_block;
    }
    if (abs_block >= end_block) return Status::Ok;  // logical page past the data
    const uint32_t page_in_blk = pages_left;

    uint8_t page[NAND_PAGE_SIZE];
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
    if (!initialized_)       return Status::NotInitialized;
    if (filename == nullptr) return Status::OutOfRange;

    const FlightIndexEntry* entry = index_.findByFilename(filename);
    if (entry == nullptr) return Status::NotFound;

    const uint16_t start_block = entry->start_block;
    const uint16_t n_blocks    = entry->n_blocks;

    if (!index_.removeByFilename(filename)) return Status::NotFound;

    Status st = index_.save(*nand_, cfg_.metadata_blocks[0], cfg_.metadata_blocks[1]);
    if (st != Status::Ok) return st;

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
