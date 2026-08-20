#include "FlightIndex.h"

#include "CRC.h"

#include <cstring>
#include <memory>
#include <new>

namespace tr_flightlog {

namespace {

constexpr size_t serialized_size(size_t entry_count) {
    return sizeof(MetadataHeader) + entry_count * sizeof(FlightIndexEntry);
}

// Whole-page-aligned scratch for a full serialized snapshot, rounded up to a
// NAND page so readPage/programPage (which always touch a full runtime page)
// never run past the buffer. Allocated on the HEAP per call (#281): the snapshot
// is ~6 KB at MAX_ENTRIES=128 — too large for the 4 KB main-task stack — and the
// callers (main at boot, flush task at finalize, BLE task at delete) each get
// their own buffer, so there's no stack pressure and no cross-task shared-buffer
// race. Returns a zero-filled buffer (pads the last page deterministically), or
// nullptr on OOM (caller degrades gracefully — the data stays on NAND).
// #671: rounded to the MAX page size, which also covers every smaller page
// size exactly (6160 B -> 8192 = 2 x 4096 = 4 x 2048). The per-chip stride is
// runtime (nand.pageSize()); only the buffer allocation is compile-time.
constexpr size_t SNAPSHOT_BUF_BYTES =
    ((FlightIndex::MAX_SERIALIZED_BYTES + NAND_PAGE_SIZE_MAX - 1) / NAND_PAGE_SIZE_MAX) * NAND_PAGE_SIZE_MAX;
static_assert(SNAPSHOT_BUF_BYTES % 2048 == 0, "buffer must hold whole pages at every supported page size");

inline std::unique_ptr<uint8_t[]> allocSnapshotBuf() {
    return std::unique_ptr<uint8_t[]>(new (std::nothrow) uint8_t[SNAPSHOT_BUF_BYTES]());
}

// CRC32 over everything after the crc32 field of MetadataHeader, through the
// last entry byte. Must match the layout expected by save/load.
uint32_t compute_crc(const uint8_t* serialized, size_t total_bytes) {
    constexpr size_t crc_field_bytes = sizeof(uint32_t);
    if (total_bytes <= crc_field_bytes) return 0;
    return calcCRC32(serialized + crc_field_bytes,
                     total_bytes - crc_field_bytes);
}

}  // namespace

Status FlightIndex::append(const FlightIndexEntry& entry) {
    if (count_ >= MAX_ENTRIES) return Status::NoSpace;
    entries_[count_++] = entry;
    return Status::Ok;
}

bool FlightIndex::removeByFilename(const char* filename) {
    for (size_t i = 0; i < count_; ++i) {
        if (std::strncmp(entries_[i].filename, filename,
                         sizeof(entries_[i].filename)) == 0) {
            for (size_t j = i + 1; j < count_; ++j) {
                entries_[j - 1] = entries_[j];
            }
            --count_;
            std::memset(&entries_[count_], 0, sizeof(FlightIndexEntry));
            return true;
        }
    }
    return false;
}

bool FlightIndex::rename(const char* old_name, const char* new_name) {
    for (size_t i = 0; i < count_; ++i) {
        if (std::strncmp(entries_[i].filename, old_name,
                         sizeof(entries_[i].filename)) == 0) {
            std::memset(entries_[i].filename, 0, sizeof(entries_[i].filename));
            std::strncpy(entries_[i].filename, new_name,
                         sizeof(entries_[i].filename) - 1);
            return true;
        }
    }
    return false;
}

const FlightIndexEntry* FlightIndex::findByFilename(const char* filename) const {
    for (size_t i = 0; i < count_; ++i) {
        if (std::strncmp(entries_[i].filename, filename,
                         sizeof(entries_[i].filename)) == 0) {
            return &entries_[i];
        }
    }
    return nullptr;
}

uint32_t FlightIndex::nextFlightId() const {
    uint32_t highest = 0;
    for (size_t i = 0; i < count_; ++i) {
        if (entries_[i].flight_id > highest) highest = entries_[i].flight_id;
    }
    return highest + 1;
}

void FlightIndex::clear() {
    count_ = 0;
    std::memset(entries_, 0, sizeof(entries_));
}

FlightIndex::SnapshotInfo FlightIndex::inspect(TR_NandBackend& nand, uint32_t block) const {
    SnapshotInfo info;
    auto buf = allocSnapshotBuf();
    if (!buf) return info;
    if (!nand.readPage(block, 0, buf.get())) return info;

    MetadataHeader hdr;
    std::memcpy(&hdr, buf.get(), sizeof(hdr));
    if (hdr.magic != META_MAGIC) return info;
    if (hdr.entry_count > MAX_ENTRIES) return info;

    const size_t total_bytes = serialized_size(hdr.entry_count);
    if (total_bytes > MAX_SERIALIZED_BYTES) return info;

    // Read the remaining pages of the serialized blob (page 0 already in buf).
    const size_t page_size = nand.pageSize();
    const size_t pages_needed = (total_bytes + page_size - 1) / page_size;
    for (size_t p = 1; p < pages_needed; ++p) {
        if (!nand.readPage(block, static_cast<uint32_t>(p),
                           buf.get() + p * page_size)) {
            return info;
        }
    }

    const uint32_t want_crc = hdr.crc32;
    const uint32_t got_crc  = compute_crc(buf.get(), total_bytes);
    if (want_crc != got_crc) return info;

    info.valid       = true;
    info.sequence    = hdr.sequence;
    info.entry_count = hdr.entry_count;
    return info;
}

Status FlightIndex::readSnapshot(TR_NandBackend& nand, uint32_t block) {
    clear();
    auto buf = allocSnapshotBuf();
    if (!buf) return Status::BackendFailed;
    if (!nand.readPage(block, 0, buf.get())) return Status::BackendFailed;

    MetadataHeader hdr;
    std::memcpy(&hdr, buf.get(), sizeof(hdr));
    if (hdr.magic != META_MAGIC) return Status::CrcMismatch;
    if (hdr.entry_count > MAX_ENTRIES) return Status::Error;

    const size_t total_bytes = serialized_size(hdr.entry_count);
    const size_t page_size = nand.pageSize();
    const size_t pages_needed = (total_bytes + page_size - 1) / page_size;
    for (size_t p = 1; p < pages_needed; ++p) {
        if (!nand.readPage(block, static_cast<uint32_t>(p),
                           buf.get() + p * page_size)) {
            return Status::BackendFailed;
        }
    }
    if (compute_crc(buf.get(), total_bytes) != hdr.crc32) return Status::CrcMismatch;

    count_         = hdr.entry_count;
    last_sequence_ = hdr.sequence;
    std::memcpy(entries_, buf.get() + sizeof(MetadataHeader),
                count_ * sizeof(FlightIndexEntry));
    return Status::Ok;
}

Status FlightIndex::load(TR_NandBackend& nand,
                         uint32_t block_active, uint32_t block_shadow) {
    SnapshotInfo a = inspect(nand, block_active);
    SnapshotInfo s = inspect(nand, block_shadow);

    clear();
    last_sequence_ = 0;

    if (!a.valid && !s.valid) return Status::Ok;  // fresh chip, empty index

    uint32_t pick_block;
    if (a.valid && s.valid) {
        pick_block = (a.sequence >= s.sequence) ? block_active : block_shadow;
    } else {
        pick_block = a.valid ? block_active : block_shadow;
    }
    return readSnapshot(nand, pick_block);
}

Status FlightIndex::writeSnapshot(TR_NandBackend& nand,
                                  uint32_t block, uint32_t sequence) const {
    auto buf = allocSnapshotBuf();   // zero-filled: pads the last page deterministically
    if (!buf) return Status::BackendFailed;
    const size_t total_bytes = serialized_size(count_);

    MetadataHeader hdr;
    hdr.crc32       = 0;
    hdr.magic       = META_MAGIC;
    hdr.sequence    = sequence;
    hdr.entry_count = static_cast<uint32_t>(count_);
    std::memcpy(buf.get(), &hdr, sizeof(hdr));
    std::memcpy(buf.get() + sizeof(hdr), entries_,
                count_ * sizeof(FlightIndexEntry));

    const uint32_t crc = compute_crc(buf.get(), total_bytes);
    std::memcpy(buf.get(), &crc, sizeof(crc));

    const size_t page_size = nand.pageSize();
    const size_t pages_needed = (total_bytes + page_size - 1) / page_size;
    for (size_t p = 0; p < pages_needed; ++p) {
        if (!nand.programPage(block, static_cast<uint32_t>(p),
                              buf.get() + p * page_size)) {
            return Status::BackendFailed;
        }
    }
    return Status::Ok;
}

Status FlightIndex::save(TR_NandBackend& nand,
                         uint32_t block_active, uint32_t block_shadow) {
    SnapshotInfo a = inspect(nand, block_active);
    SnapshotInfo s = inspect(nand, block_shadow);

    // Target the older-or-invalid copy so readers always see a valid snapshot
    // during the window between erase and program.
    uint32_t target_block;
    if (!a.valid)      target_block = block_active;
    else if (!s.valid) target_block = block_shadow;
    else               target_block = (a.sequence <= s.sequence) ? block_active : block_shadow;

    const uint32_t next_seq =
        (a.valid || s.valid ? (a.sequence > s.sequence ? a.sequence : s.sequence) : 0) + 1;

    if (!nand.eraseBlock(target_block)) return Status::BackendFailed;
    Status st = writeSnapshot(nand, target_block, next_seq);
    if (st != Status::Ok) return st;

    last_sequence_ = next_seq;
    return Status::Ok;
}

}  // namespace tr_flightlog
