#pragma once

#include "TR_FlightLog_types.h"

#include <stdint.h>
#include <stddef.h>

namespace tr_flightlog {

// 2-bit-per-block state map. Backing array is MAX-sized (2048 blocks = 512 B);
// the ACTIVE block count is runtime chip geometry (#671) — 2048 on the legacy
// and V9 parts, 1024 on the mini — set via setBlockCount() before use. Blocks
// at or past the active count read BLOCK_BAD (the existing out-of-range
// convention), so off-die blocks can never be allocated. serializedSize() is
// derived from the active count: 512 B at 2048 blocks, 256 B at 1024.
//
// Byte layout (little-endian within the byte):
//   buf[N / 4] bits [(N % 4)*2 .. (N % 4)*2 + 1] = state of block N
//
// Encoding matches the BlockState enum (FREE=0, ALLOCATED=1, BAD=2, RESERVED=3).
// A freshly-zeroed bitmap represents "all blocks free", which is what we want
// after a wipe-on-first-boot.
class BlockStateBitmap {
public:
    static constexpr size_t SERIALIZED_SIZE_MAX = (NAND_BLOCK_COUNT_MAX + 3) / 4;  // 512 B

    BlockStateBitmap();

    // #671: set the active block count (chip geometry). Clamped to the max.
    // Does NOT clear existing state — call before any get/set for sane bounds.
    void   setBlockCount(uint32_t count);
    uint32_t blockCount() const { return block_count_; }
    // Serialized length for the ACTIVE count ((count+3)/4).
    size_t serializedSize() const { return (block_count_ + 3) / 4; }

    // Single-block ops
    BlockState get(uint32_t block) const;
    void       set(uint32_t block, BlockState state);

    // Range ops — skip over BAD blocks (sticky). Return number of blocks affected.
    size_t markAllocatedRange(uint32_t start, uint32_t n_blocks);
    size_t markFreeRange(uint32_t start, uint32_t n_blocks);

    // Find first contiguous run of FREE blocks of `n_blocks` size in
    // [range_start, range_end). BAD blocks break the run (as expected). Returns
    // true + writes the starting block to `out_start` on success; false if no
    // such run exists.
    bool findContiguousFree(uint32_t n_blocks,
                            uint32_t range_start,
                            uint32_t range_end,
                            uint32_t& out_start) const;

    // Aggregate counters for health reporting.
    size_t countInState(BlockState state) const;

    // Serialization (round-trip safe).
    void serializeTo(uint8_t* buf, size_t len) const;
    bool deserializeFrom(const uint8_t* buf, size_t len);

    // Reset everything to FREE.
    void clear();

private:
    uint8_t  data_[SERIALIZED_SIZE_MAX] = {0};
    uint32_t block_count_ = NAND_BLOCK_COUNT_MAX;   // legacy default
};

}  // namespace tr_flightlog
