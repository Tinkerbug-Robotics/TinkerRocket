#pragma once

#include <stdint.h>
#include <stddef.h>

namespace tr_flightlog {

// Abstract NAND interface — the real backend wraps the low-level nand* helpers
// from TR_LogToFlash; the test fake implements it in-memory with accurate
// NAND semantics (bit 1->0 only, erase-to-0xFF).
//
// Addressing uses (block, page_in_block). Block is 0..blockCount()-1,
// page_in_block is 0..pagesPerBlock()-1. Buffers are pageSize() bytes for
// read/program (at most NAND_PAGE_SIZE_MAX — callers may size statically to
// the max and use the runtime length).
class TR_NandBackend {
public:
    virtual ~TR_NandBackend() = default;

    // #671: the chip's RDID-resolved geometry, valid for the backend's whole
    // lifetime (geometry is a pure function of the physical chip). This is
    // the single source of truth TR_FlightLog denominates all page/block
    // arithmetic in — there is no compile-time geometry any more.
    virtual uint32_t pageSize() const = 0;
    virtual uint32_t pagesPerBlock() const = 0;
    virtual uint32_t blockCount() const = 0;

    virtual bool readPage(uint32_t block, uint32_t page_in_block, uint8_t* out) = 0;
    virtual bool programPage(uint32_t block, uint32_t page_in_block, const uint8_t* data) = 0;
    virtual bool eraseBlock(uint32_t block) = 0;

    // Bad-block bookkeeping lives in the bitmap owned by TR_FlightLog (NVS-backed
    // on hardware). The backend exposes hooks so the allocator can query/update
    // without caring where the bitmap is stored.
    virtual bool isBlockBad(uint32_t block) = 0;
    virtual bool markBlockBad(uint32_t block) = 0;
};

}  // namespace tr_flightlog
