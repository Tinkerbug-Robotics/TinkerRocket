#include "WireFormat.h"

#include <cstdio>
#include <cstring>

namespace tr_flightlog::wire_format {

size_t encodeFileListJson(const FlightIndexEntry* entries, size_t count,
                          char* out, size_t out_max) {
    if (out == nullptr || out_max == 0) return 0;

    size_t pos = 0;
    auto put = [&](char c) -> bool {
        if (pos >= out_max) return false;
        out[pos++] = c;
        return true;
    };
    auto put_str = [&](const char* s) -> bool {
        while (*s) { if (!put(*s++)) return false; }
        return true;
    };

    if (!put('[')) return 0;
    for (size_t i = 0; i < count; ++i) {
        if (i > 0 && !put(',')) return 0;
        if (!put_str("{\"name\":\"")) return 0;
        if (!put_str(entries[i].filename)) return 0;
        if (!put_str("\",\"size\":")) return 0;
        char numbuf[16];
        int written = std::snprintf(numbuf, sizeof(numbuf), "%u",
                                    static_cast<unsigned>(entries[i].final_bytes));
        if (written < 0) return 0;
        if (!put_str(numbuf)) return 0;
        if (!put('}')) return 0;
    }
    if (!put(']')) return 0;
    return pos;
}

size_t encodeFileChunk(uint32_t offset, const uint8_t* data, size_t data_len,
                       bool eof, uint8_t* out, size_t out_max) {
    const size_t total = CHUNK_HEADER_SIZE + data_len;
    if (out == nullptr || out_max < total) return 0;
    if (data_len > 0xFFFF) return 0;  // length field is 2 bytes

    out[0] = static_cast<uint8_t>(offset >> 0);
    out[1] = static_cast<uint8_t>(offset >> 8);
    out[2] = static_cast<uint8_t>(offset >> 16);
    out[3] = static_cast<uint8_t>(offset >> 24);
    out[4] = static_cast<uint8_t>(data_len >> 0);
    out[5] = static_cast<uint8_t>(data_len >> 8);
    out[6] = eof ? CHUNK_FLAG_EOF : 0;

    if (data_len > 0 && data != nullptr) {
        std::memcpy(out + CHUNK_HEADER_SIZE, data, data_len);
    }
    return total;
}

bool decodeFileChunk(const uint8_t* packet, size_t packet_len,
                     uint32_t& offset_out, uint16_t& length_out,
                     bool& eof_out, const uint8_t** data_out) {
    if (packet == nullptr || packet_len < CHUNK_HEADER_SIZE) return false;
    offset_out = static_cast<uint32_t>(packet[0])       |
                 (static_cast<uint32_t>(packet[1]) << 8) |
                 (static_cast<uint32_t>(packet[2]) << 16)|
                 (static_cast<uint32_t>(packet[3]) << 24);
    length_out = static_cast<uint16_t>(packet[4] |
                                        (packet[5] << 8));
    eof_out    = (packet[6] & CHUNK_FLAG_EOF) != 0;
    if (packet_len < CHUNK_HEADER_SIZE + length_out) return false;
    if (data_out != nullptr) *data_out = packet + CHUNK_HEADER_SIZE;
    return true;
}

}  // namespace tr_flightlog::wire_format
