#pragma once

// #1485: the frame CRC, table-driven.
//
// Every FC<->OC frame carries a CRC16 over type, length and payload, computed
// with the CRC library's defaults: polynomial 0x8001, initial 0, no reflection,
// no final XOR. The library does it one bit at a time through a generic,
// parameterised loop. At 8 kHz IMU rates the OC computes it twice per sample
// (checking the batch, then packing each record for the log), and on the
// mini's ESP32-S3 that loop was a large share of the parser's core.
//
// This is the same CRC, a byte per table lookup. The table is built at compile
// time; tests_cpp/test_crc16_compat.cpp pins it bit-identical to calcCRC16().
//
// Header-only and IDF-free.

#include <stddef.h>
#include <stdint.h>

namespace crc16fast
{
struct Table
{
    uint16_t v[256];
};

constexpr Table makeTable()
{
    Table t{};
    for (int i = 0; i < 256; ++i)
    {
        uint16_t crc = (uint16_t)(i << 8);
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x8001u) : (uint16_t)(crc << 1);
        t.v[i] = crc;
    }
    return t;
}

inline constexpr Table kTable = makeTable();
}  // namespace crc16fast

// Bit-identical to calcCRC16(data, len) with the library's default parameters.
inline uint16_t crc16Frame(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < len; ++i)
        crc = (uint16_t)((crc << 8) ^ crc16fast::kTable.v[(uint8_t)((crc >> 8) ^ data[i])]);
    return crc;
}
