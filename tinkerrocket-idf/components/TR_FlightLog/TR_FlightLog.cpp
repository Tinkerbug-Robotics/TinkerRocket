#include "TR_FlightLog.h"

#include "CRC.h"

#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
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

    uint32_t b = cfg_.flight_region_start;
    while (b < cfg_.flight_region_end) {
        if (bitmap_.get(b) != BLOCK_ALLOCATED || is_in_index(b)) {
            ++b;
            continue;
        }
        // Found the start of an orphaned run. Walk forward.
        const uint32_t run_start = b;
        while (b < cfg_.flight_region_end &&
               bitmap_.get(b) == BLOCK_ALLOCATED &&
               !is_in_index(b)) {
            ++b;
        }
        const uint32_t run_len = b - run_start;

        // Scan the run for pages with valid PageHeader CRC. Track the highest
        // seq_number + associated flight_id.
        int32_t  last_good_page_rel = -1;   // logical page index within the run
        uint32_t last_seq            = 0;
        uint32_t last_flight_id      = 0;
        bool     saw_any             = false;

        for (uint32_t i = 0; i < run_len; ++i) {
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

        // Synthesize a partial flight entry. final_bytes tracks the logical
        // byte count of the data that actually got flushed to NAND.
        FlightIndexEntry entry{};
        entry.magic       = FLGT_MAGIC;
        entry.flight_id   = last_flight_id;
        std::snprintf(entry.filename, sizeof(entry.filename),
                      "flight_recovered_%lu.bin",
                      static_cast<unsigned long>(last_flight_id));
        entry.start_block = static_cast<uint16_t>(run_start);
        entry.n_blocks    = static_cast<uint16_t>(run_len);
        entry.final_bytes = static_cast<uint32_t>(
            (last_good_page_rel + 1) * static_cast<int32_t>(NAND_PAGE_SIZE));

        Status st = index_.append(entry);
        if (st != Status::Ok) return st;
        index_dirty = true;
    }

    if (index_dirty) {
        Status st = index_.save(*nand_, cfg_.metadata_blocks[0], cfg_.metadata_blocks[1]);
        if (st != Status::Ok) return st;
    }
    if (bitmap_dirty) persistBitmap();
    return Status::Ok;
}

Status TR_FlightLog::markBlockBad(uint32_t block) {
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

Status TR_FlightLog::prepareFlight(uint32_t& flight_id_out) {
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

    return writePage(page);
}

Status TR_FlightLog::writePage(const uint8_t* page) {
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

        // Program failure: runtime bad-block discovery. Mark bad, skip to
        // the next block, retry the same data.
        bitmap_.set(abs_block, BLOCK_BAD);
        nand_->markBlockBad(abs_block);
        persistBitmap();
        // Round up to the next block boundary.
        active_next_page_ = (rel_block + 1) * NAND_PAGES_PER_BLK;
    }
    return Status::BackendFailed;
}

Status TR_FlightLog::finalizeFlight(const char* filename, uint32_t final_bytes) {
    if (!initialized_)    return Status::NotInitialized;
    if (!flight_active_)  return Status::Error;
    if (filename == nullptr) return Status::OutOfRange;

    FlightIndexEntry entry{};
    entry.magic       = FLGT_MAGIC;
    entry.flight_id   = active_flight_id_;
    std::strncpy(entry.filename, filename, sizeof(entry.filename) - 1);
    entry.start_block = static_cast<uint16_t>(active_start_block_);
    entry.n_blocks    = static_cast<uint16_t>(active_n_blocks_);
    entry.final_bytes = final_bytes;

    Status st = index_.append(entry);
    if (st != Status::Ok) return st;
    st = index_.save(*nand_, cfg_.metadata_blocks[0], cfg_.metadata_blocks[1]);
    if (st != Status::Ok) {
        index_.removeByFilename(entry.filename);  // roll back the in-memory add
        return st;
    }

    flight_active_    = false;
    active_flight_id_ = 0;
    active_next_page_ = 0;
    return Status::Ok;
}

size_t TR_FlightLog::listFlights(FlightIndexEntry* out, size_t max,
                                 size_t page, size_t per_page) {
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
    const uint32_t abs_block    = entry->start_block + logical_page / NAND_PAGES_PER_BLK;
    const uint32_t page_in_blk  = logical_page % NAND_PAGES_PER_BLK;

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
    if (!initialized_)      return Status::NotInitialized;
    if (old_name == nullptr || new_name == nullptr) return Status::OutOfRange;

    if (!index_.rename(old_name, new_name)) return Status::NotFound;

    Status st = index_.save(*nand_, cfg_.metadata_blocks[0], cfg_.metadata_blocks[1]);
    if (st != Status::Ok) return st;
    return Status::Ok;
}

}  // namespace tr_flightlog
