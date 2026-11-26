#include "BlockStateBitmap.h"

#include <cstring>

namespace tr_flightlog {

BlockStateBitmap::BlockStateBitmap() { clear(); }

void BlockStateBitmap::setBlockCount(uint32_t count) {
    block_count_ = (count > NAND_BLOCK_COUNT_MAX) ? NAND_BLOCK_COUNT_MAX : count;
}

void BlockStateBitmap::clear() {
    std::memset(data_, 0, sizeof(data_));  // all BLOCK_FREE (0)
}

BlockState BlockStateBitmap::get(uint32_t block) const {
    if (block >= block_count_) return BLOCK_BAD;  // out-of-range/off-die == skip
    const uint32_t byte_idx = block >> 2;
    const uint32_t bit_off  = (block & 3) << 1;
    return static_cast<BlockState>((data_[byte_idx] >> bit_off) & 0x3);
}

void BlockStateBitmap::set(uint32_t block, BlockState state) {
    if (block >= block_count_) return;
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
        if (b >= block_count_) break;
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
        if (b >= block_count_) break;
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
    if (range_end > block_count_) range_end = block_count_;

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

bool BlockStateBitmap::findRoomiestFree(uint32_t n_blocks,
                                        uint32_t range_start,
                                        uint32_t range_end,
                                        uint32_t& out_start,
                                        uint32_t* out_room) const {
    if (n_blocks == 0) return false;
    if (range_start >= range_end) return false;
    if (range_end > block_count_) range_end = block_count_;

    bool     found     = false;
    uint32_t best_room = 0;
    uint32_t b = range_start;
    while (b < range_end) {
        if (get(b) != BLOCK_FREE && get(b) != BLOCK_BAD) { ++b; continue; }
        const uint32_t stretch_start = b;
        while (b < range_end && (get(b) == BLOCK_FREE || get(b) == BLOCK_BAD)) ++b;
        uint32_t start = 0;
        if (!findContiguousFree(n_blocks, stretch_start, b, start)) continue;
        // #1758: the room is what lies AFTER the start: FREE blocks before it
        // (cut off by a BAD block inside the first n) are never reached.
        const uint32_t room = static_cast<uint32_t>(countInStateRange(BLOCK_FREE, start, b));
        if (room > best_room) {
            best_room = room;
            out_start = start;
            found     = true;
        }
    }
    if (found && out_room != nullptr) *out_room = best_room;
    return found;
}

size_t BlockStateBitmap::countInState(BlockState state) const {
    return countInStateRange(state, 0, block_count_);
}

size_t BlockStateBitmap::countInStateRange(BlockState state,
                                           uint32_t start, uint32_t end) const {
    if (end > block_count_) end = block_count_;
    size_t count = 0;
    for (uint32_t b = start; b < end; ++b) {
        if (get(b) == state) ++count;
    }
    return count;
}

uint32_t BlockStateBitmap::fingerprint() const {
    uint32_t h = 2166136261u;
    const size_t n = serializedSize();
    for (size_t i = 0; i < n; ++i) {
        h ^= data_[i];
        h *= 16777619u;
    }
    return h ^ block_count_;
}

void BlockStateBitmap::serializeTo(uint8_t* buf, size_t len) const {
    if (len < serializedSize()) return;
    std::memcpy(buf, data_, serializedSize());
}

bool BlockStateBitmap::deserializeFrom(const uint8_t* buf, size_t len) {
    if (len < serializedSize()) return false;
    std::memcpy(data_, buf, serializedSize());
    return true;
}

}  // namespace tr_flightlog
