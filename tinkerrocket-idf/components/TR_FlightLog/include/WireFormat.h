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

// ---- #1144: how many entries may be put in one notification ---------------
//
// The page size used to be a hardcoded 5 on every side — firmware, iOS and
// Android — with no reference to the link MTU. #1284 stopped the OC sending an
// over-MTU page (NimBLE trimmed it to the ATT MTU and returned success, so the
// app parsed garbage), but skipping is not the same as working: at an ATT MTU
// of 185 a five-entry page is ~271 B against a 182 B budget, so a phone with
// four or more flights received no list at ALL.
//
// The page size is therefore negotiated: cmd 2 carries an optional per_page
// byte the app derives from its own MTU, and the firmware clamps it to what
// actually fits. These constants are the arithmetic both ends agree on.

// Worst-case bytes for one encoded entry, from the struct's own bounds:
//   {"name":"  =  9      filename  <= 27 (FlightIndexEntry::filename is 28
//   ","size":  =  9                      bytes incl. the NUL)
//   digits     <= 10     final_bytes is uint32_t, so 4294967295
//   }          =  1
constexpr size_t kFileListEntryMaxBytes = 9 + 27 + 9 + 10 + 1;   // 56

// Entries that fit a notification budget, with the brackets and separators:
//   n*E + (n-1) + 2 <= budget   ->   n <= (budget - 1) / (E + 1)
// Never returns 0 — a budget too small for even one entry still gets a
// one-entry page, which the caller's own maxNotifyBytes() guard then refuses.
// Returning 0 would silently turn "MTU too small" into "no flights", which is
// the failure this whole issue is about.
constexpr size_t fileListEntriesThatFit(size_t budget_bytes) {
    return (budget_bytes < (kFileListEntryMaxBytes + 2))
               ? 1u
               : ((budget_bytes - 1) / (kFileListEntryMaxBytes + 1));
}

// What an app that sends no per_page byte gets, and the ceiling for one that
// does. 5 is what every shipped app assumes, so it stays the default.
constexpr size_t kFileListDefaultPerPage = 5;
constexpr size_t kFileListMaxPerPage     = 16;

// Resolve what the app asked for against what the link can carry.
//
//   requested == 0    the app sent no per_page byte -> the historical 5
//   requested > 0     honour it, bounded by kFileListMaxPerPage
//   either way        never more than the notification budget fits
//
// Clamping DOWN on the firmware side rather than trusting the app is the point:
// the app knows its own ATT MTU, but only this side knows the worst-case entry
// width, and a page that does not fit is refused outright by sendFileList().
constexpr size_t clampFileListPerPage(size_t requested, size_t max_notify_bytes) {
    const size_t want = (requested == 0)
                            ? kFileListDefaultPerPage
                            : (requested > kFileListMaxPerPage ? kFileListMaxPerPage
                                                               : requested);
    const size_t fits = fileListEntriesThatFit(max_notify_bytes);
    return (want < fits) ? want : fits;
}

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
