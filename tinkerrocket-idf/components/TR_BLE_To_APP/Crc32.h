#pragma once

#include <stddef.h>
#include <stdint.h>

// ============================================================================
// CRC-32 (ISO 3309 / zlib / PNG): reflected poly 0xEDB88320, init 0xFFFFFFFF,
// final XOR 0xFFFFFFFF. Canonical check value: crc32("123456789") == 0xCBF43926.
//
// #526: the L2CAP file-transfer stream ends with an END record carrying a CRC32
// over every DATA byte. The download completes ONLY if that CRC matches — it is
// the first end-to-end integrity check this path has ever had. The firmware
// computes it incrementally as it streams (there is no 8.5 MB buffer to checksum
// at the end), so the API is a running state, and iOS has a byte-identical twin
// (Crc32.swift) pinned to the SAME check value in both test suites.
//
// Bitwise, table-free: 8 iterations per byte. A download is ~18k SDUs of ~1 KB;
// the CRC is a rounding error next to the bit-banged NAND read that produced the
// bytes, and a 1 KB table is not worth the RAM on a device already tight on it.
// ============================================================================

namespace tr_crc32
{

// Running state before any bytes have been folded in.
inline uint32_t init() { return 0xFFFFFFFFu; }

// Fold `len` bytes into the running CRC. Chain calls across arbitrarily-split
// buffers — the result depends only on the concatenated byte sequence.
inline uint32_t update(uint32_t crc, const uint8_t* data, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            // Branchless reflected step: subtract-and-mask yields 0xEDB88320
            // exactly when the low bit is set, 0 otherwise.
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return crc;
}

// Finalise the running CRC into the transmitted value.
inline uint32_t finalize(uint32_t crc) { return crc ^ 0xFFFFFFFFu; }

// One-shot convenience for a single contiguous buffer.
inline uint32_t compute(const uint8_t* data, size_t len)
{
    return finalize(update(init(), data, len));
}

}  // namespace tr_crc32
