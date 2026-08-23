#pragma once

#include "TR_NandBackend.h"
#include "TR_FlightLog_types.h"

#include <cstdint>
#include <set>
#include <unordered_map>
#include <vector>

namespace tr_flightlog_test {

// In-memory NAND simulator with accurate semantics:
//   * All bytes start at 0xFF.
//   * programPage() is bit-and-only (a bit can only go 1->0 without an erase).
//   * eraseBlock() resets all bytes in the block to 0xFF.
//   * Bad-block marking is persistent for the instance.
//   * Fault injection hooks cover: deterministic-bad blocks, program-fail
//     and read-error sets keyed by (block, page).
//
// This class is the central primitive every TR_FlightLog test builds on, so
// its semantics must match the real chip closely.
class FakeNandBackend : public tr_flightlog::TR_NandBackend {
public:
    // #671: geometry is a constructor parameter — the default is the legacy
    // 4 Gbit part (4096/64/2048) so pre-existing tests stay byte-identical.
    // Pass the GD5F figures (2048/64/2048 or 2048/64/1024) to model the
    // V9 / mini chips.
    explicit FakeNandBackend(uint32_t page_size = 4096,
                             uint32_t pages_per_block = 64,
                             uint32_t block_count = 2048);

    // TR_NandBackend overrides
    uint32_t pageSize() const override { return page_size_; }
    uint32_t pagesPerBlock() const override { return pages_per_blk_; }
    uint32_t blockCount() const override { return block_count_; }
    bool readPage(uint32_t block, uint32_t page_in_block, uint8_t* out) override;
    bool programPage(uint32_t block, uint32_t page_in_block, const uint8_t* data) override;
    bool eraseBlock(uint32_t block) override;
    bool isBlockBad(uint32_t block) override;
    bool markBlockBad(uint32_t block) override;

    // ---- Test helpers ----

    // Force a block to be marked bad out of the gate (as if factory-marked).
    void injectFactoryBadBlock(uint32_t block);

    // Make programPage() at (block, page) return false on the next call.
    // Consumed after one firing.
    void injectProgramFailOnce(uint32_t block, uint32_t page_in_block);

    // Make readPage() at (block, page) return false every time.
    void injectReadErrorPersistent(uint32_t block, uint32_t page_in_block);

    // Inspect raw storage (for tests that need to check what was actually written).
    const uint8_t* peekPage(uint32_t block, uint32_t page_in_block) const;

    // Counters for stall / throughput tests.
    uint64_t programCount() const { return program_count_; }
    uint64_t eraseCount()   const { return erase_count_; }
    uint64_t readCount()    const { return read_count_; }

    // Reset everything to "freshly manufactured chip" state (all 0xFF, no bad,
    // no fault injection).
    void reset();

private:
    const uint32_t page_size_;
    const uint32_t pages_per_blk_;
    const uint32_t block_count_;

    std::vector<uint8_t> storage_;   // block_count_ * pages_per_blk_ * page_size_ bytes
    std::vector<bool>    bad_blocks_;

    struct PageKey {
        uint32_t block;
        uint32_t page;
        bool operator<(const PageKey& o) const {
            return (block < o.block) || (block == o.block && page < o.page);
        }
    };
    std::set<PageKey> program_fail_once_;
    std::set<PageKey> read_error_persistent_;

    uint64_t program_count_ = 0;
    uint64_t erase_count_   = 0;
    uint64_t read_count_    = 0;

    uint8_t* pageStorage(uint32_t block, uint32_t page);
    const uint8_t* pageStorage(uint32_t block, uint32_t page) const;
};

}  // namespace tr_flightlog_test
