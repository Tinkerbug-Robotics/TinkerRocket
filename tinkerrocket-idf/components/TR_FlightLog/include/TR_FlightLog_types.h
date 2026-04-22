#pragma once

#include <stdint.h>
#include <stddef.h>

namespace tr_flightlog {

// ---- NAND geometry (ISM6HG256 — matches TR_LogToFlash) ----
constexpr uint32_t NAND_PAGE_SIZE      = 2048;
constexpr uint32_t NAND_PAGES_PER_BLK  = 64;
constexpr uint32_t NAND_BLOCK_SIZE     = NAND_PAGE_SIZE * NAND_PAGES_PER_BLK;  // 128 KB
constexpr uint32_t NAND_BLOCK_COUNT    = 1024;

// ---- Block state (2 bits per block in the persistent bitmap) ----
enum BlockState : uint8_t {
    BLOCK_FREE      = 0,
    BLOCK_ALLOCATED = 1,
    BLOCK_BAD       = 2,
    BLOCK_RESERVED  = 3,
};

// ---- Per-page self-description, first 16 bytes of every flight page ----
// CRC covers everything after the crc32 field (magic + seq + flight_id +
// payload) so a bit-flip anywhere in the page is detected by the scanner.
struct __attribute__((packed)) PageHeader {
    uint32_t crc32;         // CRC32 over bytes[4..NAND_PAGE_SIZE-1]
    uint32_t magic;         // FPAG_MAGIC
    uint32_t seq_number;    // monotonic within the flight
    uint32_t flight_id;     // matches FlightIndexEntry.flight_id
};
static_assert(sizeof(PageHeader) == 16, "PageHeader must be 16 bytes");

constexpr uint32_t FPAG_MAGIC = 0x47415046;  // 'FPAG' little-endian

// ---- Flight index entry, appended on finalize ----
// `filename` is 28 bytes to fit the longest legal form
// "flight_YYYYMMDD_HHMMSS.bin\0" (27 chars incl null).
struct __attribute__((packed)) FlightIndexEntry {
    uint32_t magic;              // FLGT_MAGIC
    uint32_t flight_id;          // monotonic, assigned at prepareFlight
    char     filename[28];       // "flight_YYYYMMDD_HHMMSS.bin\0" or "flight_NNN.bin\0"
    uint16_t start_block;
    uint16_t n_blocks;
    uint32_t final_bytes;
    uint16_t crc16;              // reserved; dual-copy metadata uses its own CRC
    uint16_t reserved;
};
static_assert(sizeof(FlightIndexEntry) == 48, "FlightIndexEntry layout check");

constexpr uint32_t FLGT_MAGIC = 0x54474C46;  // 'FLGT' little-endian

// ---- Public status / error code ----
enum class Status : uint8_t {
    Ok = 0,
    Error,
    NoSpace,          // no free contiguous range of requested size
    NotFound,         // filename / flight_id not in index
    CrcMismatch,
    OutOfRange,
    NotInitialized,
    BackendFailed,    // underlying NAND op failed
    PartialFlight,    // recovery: last flight had no finalized index entry
};

constexpr const char* to_string(Status s) {
    switch (s) {
        case Status::Ok:              return "Ok";
        case Status::Error:           return "Error";
        case Status::NoSpace:         return "NoSpace";
        case Status::NotFound:        return "NotFound";
        case Status::CrcMismatch:     return "CrcMismatch";
        case Status::OutOfRange:      return "OutOfRange";
        case Status::NotInitialized:  return "NotInitialized";
        case Status::BackendFailed:   return "BackendFailed";
        case Status::PartialFlight:   return "PartialFlight";
    }
    return "?";
}

}  // namespace tr_flightlog
