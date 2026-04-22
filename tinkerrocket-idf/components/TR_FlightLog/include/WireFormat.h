#pragma once

#include "TR_FlightLog_types.h"

#include <stdint.h>
#include <stddef.h>

// Byte-level encoders for the BLE wire formats the iOS app consumes.
//
// These are authoritative — the Stage 3 BLE command handlers call into this
// namespace to produce cmd 2 (file-list JSON) and cmd 4 (download chunk) bytes
// identical to the legacy LFS-backed implementation. Golden fixtures in
// tests_cpp/fixtures/ble_wire_format/ lock the bytes in place.

namespace tr_flightlog::wire_format {

// BLE download chunk size / offset constants (match TR_BLE_To_APP).
constexpr size_t CHUNK_HEADER_SIZE = 7;   // offset(4) + length(2) + flags(1)
constexpr uint8_t CHUNK_FLAG_EOF   = 0x01;

// Encode a list of flights as JSON:
//   [{"name":"FILENAME","size":NUMBER},{"name":"...","size":...}]
// No whitespace; filename is copied verbatim (no escaping — filenames are
// restricted to the `flight_*` character set).
//
// Returns number of bytes written (excluding any null terminator). Returns 0
// if `out_max` is too small to fit the full payload.
size_t encodeFileListJson(const FlightIndexEntry* entries, size_t count,
                          char* out, size_t out_max);

// Encode a download chunk packet:
//   [offset(4 LE)][length(2 LE)][flags(1)][data(N)]
// Returns total packet size on success; 0 if `out_max` is insufficient.
size_t encodeFileChunk(uint32_t offset, const uint8_t* data, size_t data_len,
                       bool eof, uint8_t* out, size_t out_max);

// Decode the header of a chunk packet; `*data_out` points into `packet`.
// Returns false if `packet_len` is too small or the encoded `length` field
// exceeds available bytes.
bool decodeFileChunk(const uint8_t* packet, size_t packet_len,
                     uint32_t& offset_out, uint16_t& length_out,
                     bool& eof_out, const uint8_t** data_out);

}  // namespace tr_flightlog::wire_format
