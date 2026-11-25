#pragma once
// SBF block builders for host tests: the same helpers test_sbf_parser.cpp
// keeps in its own anonymous namespace, shared here for the raw-GNSS tests
// (tests_cpp/v9raw). A block is header, time stamp, body, padding to 4 and the
// CRC, exactly as the mosaic-G5 sends it.
#include <cstdint>
#include <cstring>
#include <vector>

#include "SbfParser.h"

namespace sbftest {

struct Bytes
{
    std::vector<uint8_t> v;
    Bytes& u8(uint8_t x) { v.push_back(x); return *this; }
    Bytes& i8(int8_t x) { return u8(static_cast<uint8_t>(x)); }
    Bytes& u16(uint16_t x) { u8(x & 0xFF); return u8(x >> 8); }
    Bytes& i16(int16_t x) { return u16(static_cast<uint16_t>(x)); }
    Bytes& u32(uint32_t x) { u16(x & 0xFFFF); return u16(x >> 16); }
    Bytes& i32(int32_t x) { return u32(static_cast<uint32_t>(x)); }
    Bytes& f32(float f) { uint32_t x; std::memcpy(&x, &f, 4); return u32(x); }
    Bytes& f64(double d) { uint64_t x; std::memcpy(&x, &d, 8); u32(static_cast<uint32_t>(x)); return u32(static_cast<uint32_t>(x >> 32)); }
    Bytes& pad(size_t n, uint8_t x = 0) { v.insert(v.end(), n, x); return *this; }
};

// A whole block: header, time stamp, body, padding to 4, CRC.
inline std::vector<uint8_t> block(uint16_t number, uint8_t rev, uint32_t tow, uint16_t wnc, const std::vector<uint8_t>& body)
{
    Bytes b;
    b.u8('$').u8('@').u16(0).u16(static_cast<uint16_t>(number | (rev << 13))).u16(0).u32(tow).u16(wnc);
    b.v.insert(b.v.end(), body.begin(), body.end());
    while (b.v.size() % 4) b.v.push_back(0);
    const uint16_t len = static_cast<uint16_t>(b.v.size());
    b.v[6] = len & 0xFF;
    b.v[7] = len >> 8;
    const uint16_t crc = mosaic::crc16Ccitt(b.v.data() + 4, b.v.size() - 4);
    b.v[2] = crc & 0xFF;
    b.v[3] = crc >> 8;
    return b.v;
}

}  // namespace sbftest
