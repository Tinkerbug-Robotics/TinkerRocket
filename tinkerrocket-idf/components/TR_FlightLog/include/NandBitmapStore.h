#pragma once

#include "TR_BitmapStore.h"
#include "TR_NandBackend.h"

#include <stdint.h>
#include <stddef.h>

namespace tr_flightlog {

// TR_BitmapStore backed by two NAND metadata blocks (dual-copy, newest-sequence
// wins) — mirrors FlightIndex's power-safe scheme.
//
// #398: persisting the 3-state block bitmap here instead of NVS keeps every
// prepareFlight / extend / bad-block persist OFF internal flash. The NVS path
// (Preferences.putBytes + nvs_commit) periodically compacted a 4 KB sector,
// and an internal-flash erase disables the CPU cache on BOTH cores (IDF spins
// an esp_ipc stub on the other core), which froze core 1 — the parser and
// loop_oc stalled ~100 ms. A NAND write is on the external SPI bus and never
// touches the cache. The serialized bitmap (<= runtime page size - header; checked at begin()) fits in
// one NAND page, so each save is one erase + one page program.
//
// Migration: on the first boot after switching from NVS, both metadata blocks
// read back with no valid magic, so load() returns false; TR_FlightLog::begin
// then seeds a fresh bitmap from the backend's bad-block oracle and persists it
// here. The physical bad-block history lives in the backend (its own map), so
// nothing is lost — only the in-RAM ALLOCATED state is reset, which is correct
// on a clean boot.
class NandBitmapStore : public TR_BitmapStore {
public:
    NandBitmapStore() = default;
    NandBitmapStore(TR_NandBackend* nand, uint32_t block_a, uint32_t block_b)
        : nand_(nand), block_a_(block_a), block_b_(block_b) {}

    // main.cpp constructs the store as a static before the NAND backend exists,
    // so binding is deferred to just before flightlog.begin().
    void bind(TR_NandBackend* nand, uint32_t block_a, uint32_t block_b) {
        nand_ = nand; block_a_ = block_a; block_b_ = block_b;
    }

    bool load(uint8_t* buf, size_t len) override;
    bool save(const uint8_t* buf, size_t len) override;

    // Newest sequence number seen at the last successful load/save (diagnostic).
    uint32_t lastSequence() const { return last_sequence_; }

private:
    // page 0 layout: [Header][payload...]. crc32 is first so it can cover the
    // rest of the page exactly like FlightIndex's MetadataHeader.
    struct Header {
        uint32_t crc32;     // CRC32 over bytes [4 .. sizeof(Header)+length-1]
        uint32_t magic;     // kMagic
        uint32_t sequence;  // monotonic; higher wins
        uint32_t length;    // payload byte count
    };
    static constexpr uint32_t kMagic = 0x424D5031u;  // 'BMP1'

    // Byte span the crc32 field covers for a given payload length.
    static size_t crcSpan(size_t payload_len);

    // Read page 0 of `block`. On a valid, CRC-good snapshot: set seq_out and,
    // when out != nullptr, copy up to `len` payload bytes (zero-padding any
    // tail beyond the stored length). Returns false on any invalidity.
    bool readSlot(uint32_t block, uint32_t& seq_out, uint8_t* out, size_t len);

    TR_NandBackend* nand_ = nullptr;
    uint32_t block_a_ = 0;
    uint32_t block_b_ = 0;
    uint32_t last_sequence_ = 0;
};

}  // namespace tr_flightlog
