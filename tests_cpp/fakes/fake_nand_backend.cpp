#include "fake_nand_backend.h"

#include <cstring>

namespace tr_flightlog_test {

using tr_flightlog::NAND_BLOCK_COUNT;
using tr_flightlog::NAND_PAGES_PER_BLK;
using tr_flightlog::NAND_PAGE_SIZE;

FakeNandBackend::FakeNandBackend() { reset(); }

void FakeNandBackend::reset() {
    storage_.assign(static_cast<size_t>(TOTAL_PAGES) * NAND_PAGE_SIZE, 0xFF);
    bad_blocks_.assign(NAND_BLOCK_COUNT, false);
    program_fail_once_.clear();
    read_error_persistent_.clear();
    program_count_ = 0;
    erase_count_   = 0;
    read_count_    = 0;
}

uint8_t* FakeNandBackend::pageStorage(uint32_t block, uint32_t page) {
    const size_t offset =
        (static_cast<size_t>(block) * NAND_PAGES_PER_BLK + page) * NAND_PAGE_SIZE;
    return &storage_[offset];
}

const uint8_t* FakeNandBackend::pageStorage(uint32_t block, uint32_t page) const {
    const size_t offset =
        (static_cast<size_t>(block) * NAND_PAGES_PER_BLK + page) * NAND_PAGE_SIZE;
    return &storage_[offset];
}

bool FakeNandBackend::readPage(uint32_t block, uint32_t page_in_block, uint8_t* out) {
    if (block >= NAND_BLOCK_COUNT || page_in_block >= NAND_PAGES_PER_BLK) return false;
    if (read_error_persistent_.count({block, page_in_block})) return false;
    std::memcpy(out, pageStorage(block, page_in_block), NAND_PAGE_SIZE);
    ++read_count_;
    return true;
}

bool FakeNandBackend::programPage(uint32_t block, uint32_t page_in_block, const uint8_t* data) {
    if (block >= NAND_BLOCK_COUNT || page_in_block >= NAND_PAGES_PER_BLK) return false;
    auto it = program_fail_once_.find({block, page_in_block});
    if (it != program_fail_once_.end()) {
        program_fail_once_.erase(it);
        return false;
    }
    uint8_t* dst = pageStorage(block, page_in_block);
    for (size_t i = 0; i < NAND_PAGE_SIZE; ++i) {
        dst[i] = dst[i] & data[i];
    }
    ++program_count_;
    return true;
}

bool FakeNandBackend::eraseBlock(uint32_t block) {
    if (block >= NAND_BLOCK_COUNT) return false;
    uint8_t* start = pageStorage(block, 0);
    std::memset(start, 0xFF, static_cast<size_t>(NAND_PAGES_PER_BLK) * NAND_PAGE_SIZE);
    ++erase_count_;
    return true;
}

bool FakeNandBackend::isBlockBad(uint32_t block) {
    if (block >= NAND_BLOCK_COUNT) return true;
    return bad_blocks_[block];
}

bool FakeNandBackend::markBlockBad(uint32_t block) {
    if (block >= NAND_BLOCK_COUNT) return false;
    bad_blocks_[block] = true;
    return true;
}

void FakeNandBackend::injectFactoryBadBlock(uint32_t block) {
    if (block < NAND_BLOCK_COUNT) bad_blocks_[block] = true;
}

void FakeNandBackend::injectProgramFailOnce(uint32_t block, uint32_t page_in_block) {
    program_fail_once_.insert({block, page_in_block});
}

void FakeNandBackend::injectReadErrorPersistent(uint32_t block, uint32_t page_in_block) {
    read_error_persistent_.insert({block, page_in_block});
}

const uint8_t* FakeNandBackend::peekPage(uint32_t block, uint32_t page_in_block) const {
    return pageStorage(block, page_in_block);
}

}  // namespace tr_flightlog_test
