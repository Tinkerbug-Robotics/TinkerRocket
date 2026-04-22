#include "BlockStateBitmap.h"

#include <cstring>

namespace tr_flightlog {

BlockStateBitmap::BlockStateBitmap() { clear(); }

void BlockStateBitmap::clear() {
    std::memset(data_, 0, sizeof(data_));  // all BLOCK_FREE (0)
}

BlockState BlockStateBitmap::get(uint32_t block) const {
    if (block >= NAND_BLOCK_COUNT) return BLOCK_BAD;  // out-of-range == skip
    const uint32_t byte_idx = block >> 2;
    const uint32_t bit_off  = (block & 3) << 1;
    return static_cast<BlockState>((data_[byte_idx] >> bit_off) & 0x3);
}

void BlockStateBitmap::set(uint32_t block, BlockState state) {
    if (block >= NAND_BLOCK_COUNT) return;
    const uint32_t byte_idx = block >> 2;
    const uint32_t bit_off  = (block & 3) << 1;
    data_[byte_idx] = static_cast<uint8_t>(
        (data_[byte_idx] & ~(0x3u << bit_off)) |
        ((static_cast<uint8_t>(state) & 0x3u) << bit_off));
}

size_t BlockStateBitmap::markAllocatedRange(uint32_t start, uint32_t n_blocks) {
    size_t touched = 0;
    for (uint32_t i = 0; i < n_blocks; ++i) {
        uint32_t b = start + i;
        if (b >= NAND_BLOCK_COUNT) break;
        if (get(b) == BLOCK_BAD) continue;   // sticky
        set(b, BLOCK_ALLOCATED);
        ++touched;
    }
    return touched;
}

size_t BlockStateBitmap::markFreeRange(uint32_t start, uint32_t n_blocks) {
    size_t touched = 0;
    for (uint32_t i = 0; i < n_blocks; ++i) {
        uint32_t b = start + i;
        if (b >= NAND_BLOCK_COUNT) break;
        if (get(b) == BLOCK_BAD) continue;   // sticky
        set(b, BLOCK_FREE);
        ++touched;
    }
    return touched;
}

bool BlockStateBitmap::findContiguousFree(uint32_t n_blocks,
                                          uint32_t range_start,
                                          uint32_t range_end,
                                          uint32_t& out_start) const {
    if (n_blocks == 0) return false;
    if (range_start >= range_end) return false;
    if (range_end > NAND_BLOCK_COUNT) range_end = NAND_BLOCK_COUNT;

    uint32_t run_start = range_start;
    uint32_t run_len   = 0;
    for (uint32_t b = range_start; b < range_end; ++b) {
        if (get(b) == BLOCK_FREE) {
            if (run_len == 0) run_start = b;
            if (++run_len == n_blocks) {
                out_start = run_start;
                return true;
            }
        } else {
            run_len = 0;
        }
    }
    return false;
}

size_t BlockStateBitmap::countInState(BlockState state) const {
    size_t count = 0;
    for (uint32_t b = 0; b < NAND_BLOCK_COUNT; ++b) {
        if (get(b) == state) ++count;
    }
    return count;
}

void BlockStateBitmap::serializeTo(uint8_t* buf, size_t len) const {
    if (len < SERIALIZED_SIZE) return;
    std::memcpy(buf, data_, SERIALIZED_SIZE);
}

bool BlockStateBitmap::deserializeFrom(const uint8_t* buf, size_t len) {
    if (len < SERIALIZED_SIZE) return false;
    std::memcpy(data_, buf, SERIALIZED_SIZE);
    return true;
}

}  // namespace tr_flightlog
