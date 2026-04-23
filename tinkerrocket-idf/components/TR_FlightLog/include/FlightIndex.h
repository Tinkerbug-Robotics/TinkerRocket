#pragma once

#include "TR_FlightLog_types.h"
#include "TR_NandBackend.h"

#include <stdint.h>
#include <stddef.h>

namespace tr_flightlog {

// Header for a serialized flight index snapshot. Lives at the start of a
// metadata block. CRC32 covers every byte after the crc32 field (i.e.
// [magic..end_of_entries]).
struct __attribute__((packed)) MetadataHeader {
    uint32_t crc32;
    uint32_t magic;          // META_MAGIC
    uint32_t sequence;       // monotonic; higher wins on dual-copy read
    uint32_t entry_count;
};
static_assert(sizeof(MetadataHeader) == 16, "MetadataHeader must be 16 bytes");

constexpr uint32_t META_MAGIC = 0x4154454D;  // 'META' little-endian

// Append-only flight index. In-RAM collection of FlightIndexEntry, persisted
// as whole-block snapshots to two dedicated NAND blocks (active + shadow).
// Load picks whichever block has the higher valid sequence number; save
// writes the older copy and increments the sequence.
//
// MAX_ENTRIES is conservative — practical chip usage is tens of flights.
class FlightIndex {
public:
    // 64 entries × 48 B + 16 B header = 3088 B serialized. The serialize/
    // deserialize helpers allocate this as a stack buffer, so it must comfortably
    // fit the smallest task stack that calls begin()/save()/load() — that's the
    // ESP-IDF main task at 4 KB default. 256 entries (the Stage 1 figure) put
    // ~12 KB on the stack and crashed on the bench with the classic 0x55aa...
    // canary smear. Chip capacity is ~8 full flights anyway; 64 is still ample.
    static constexpr size_t MAX_ENTRIES = 64;
    static constexpr size_t MAX_SERIALIZED_BYTES =
        sizeof(MetadataHeader) + MAX_ENTRIES * sizeof(FlightIndexEntry);

    FlightIndex() = default;

    // Pull the freshest valid snapshot from the backend. If neither copy is
    // valid (fresh chip), leaves the index empty and returns Ok.
    Status load(TR_NandBackend& nand, uint32_t block_active, uint32_t block_shadow);

    // Persist to the older of the two copies; increments the sequence counter.
    Status save(TR_NandBackend& nand, uint32_t block_active, uint32_t block_shadow);

    // ---- In-memory collection ----
    Status   append(const FlightIndexEntry& entry);
    bool     removeByFilename(const char* filename);
    bool     rename(const char* old_name, const char* new_name);

    size_t                  size() const { return count_; }
    const FlightIndexEntry& at(size_t i) const { return entries_[i]; }
    const FlightIndexEntry* findByFilename(const char* filename) const;
    uint32_t                nextFlightId() const;

    void clear();

    // Test / diagnostic hooks.
    uint32_t sequence() const { return last_sequence_; }

private:
    FlightIndexEntry entries_[MAX_ENTRIES] = {};
    size_t           count_         = 0;
    uint32_t         last_sequence_ = 0;

    // Returns sequence number + entry count if the snapshot at `block` parses
    // and its CRC matches; otherwise returns false with out_valid=false.
    struct SnapshotInfo {
        bool     valid = false;
        uint32_t sequence = 0;
        uint32_t entry_count = 0;
    };
    SnapshotInfo inspect(TR_NandBackend& nand, uint32_t block) const;

    // Fully materialize a snapshot at `block` into this->entries_.
    Status readSnapshot(TR_NandBackend& nand, uint32_t block);

    // Serialize + program into `block`. Assumes the block has been erased.
    Status writeSnapshot(TR_NandBackend& nand, uint32_t block, uint32_t sequence) const;
};

}  // namespace tr_flightlog
