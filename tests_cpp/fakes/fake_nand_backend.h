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
    FakeNandBackend();

    // TR_NandBackend overrides
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
    static constexpr uint32_t TOTAL_PAGES =
        tr_flightlog::NAND_BLOCK_COUNT * tr_flightlog::NAND_PAGES_PER_BLK;

    std::vector<uint8_t> storage_;   // TOTAL_PAGES * NAND_PAGE_SIZE bytes
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
