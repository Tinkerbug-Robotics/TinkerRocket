#pragma once

#include "TR_FlightLog_types.h"

#include <stdint.h>
#include <stddef.h>

namespace tr_flightlog {

// 2-bit-per-block state map. 1024 blocks = 256 bytes.
//
// Byte layout (little-endian within the byte):
//   buf[N / 4] bits [(N % 4)*2 .. (N % 4)*2 + 1] = state of block N
//
// Encoding matches the BlockState enum (FREE=0, ALLOCATED=1, BAD=2, RESERVED=3).
// A freshly-zeroed bitmap represents "all blocks free", which is what we want
// after a wipe-on-first-boot.
class BlockStateBitmap {
public:
    static constexpr size_t SERIALIZED_SIZE = (NAND_BLOCK_COUNT + 3) / 4;  // 256 B

    BlockStateBitmap();

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
    uint8_t data_[SERIALIZED_SIZE] = {0};
};

}  // namespace tr_flightlog
