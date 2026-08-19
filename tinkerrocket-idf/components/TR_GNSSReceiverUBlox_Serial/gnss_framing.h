#pragma once

#include <cstddef>
#include <cstdint>

// Pure framing test for GNSS bring-up, split out of TR_GNSSReceiverUBlox_Serial
// so it can be exercised on the host (tests_cpp/test_gnss_framing.cpp).
//
// WHY THIS EXISTS.  Baud detection used to accept any byte at all
// (`uartAvailable() > 0`).  A real 9600 NMEA stream sampled at 38400 still
// delivers bytes, so every wrong baud looked alive and ran the expensive
// recovery branch — 1.5 s begin + 250 ms drain + 1.5 s assumeSuccess begin.
// Measured on the bench 2026-08-19: a healthy SAM-M10Q talking 9600 was
// false-positived at 38400 and burned 9.4 s before the sweep returned to 9600.
//
// Asking whether the bytes actually FRAME as GNSS traffic collapses a wrong
// baud to a single window.  A baud mismatch shreds byte boundaries, so it
// essentially never yields UBX sync or a valid NMEA checksum, while a genuine
// stream yields one within an output period.
//
// Deliberately conservative: this answers "is this plausibly a GNSS stream at
// this baud", NOT "is the receiver healthy".  A false negative costs one probe
// window and falls through to the full sweep; a false positive costs the ~3 s
// recovery branch this exists to avoid.

namespace tr {

/// True when `buf` contains UBX sync bytes or one checksum-valid NMEA sentence.
inline bool gnssFramingDetected(const uint8_t* buf, size_t n)
{
    if (buf == nullptr) return false;

    // UBX: every frame opens 0xB5 0x62.  Two bytes is weak on its own, but it
    // only has to survive alongside the NMEA path below, and a wrong-baud
    // sample producing that exact ordered pair is rare enough to accept.
    for (size_t i = 0; i + 1 < n; i++)
    {
        if (buf[i] == 0xB5 && buf[i + 1] == 0x62) return true;
    }

    // NMEA: '$' <payload> '*' HH, checksum = XOR over <payload>.
    for (size_t i = 0; i < n; i++)
    {
        if (buf[i] != '$') continue;
        uint8_t sum = 0;
        for (size_t j = i + 1; j < n; j++)
        {
            if (buf[j] == '*')
            {
                if (j + 2 >= n) break;          // checksum digits not in yet
                auto hexVal = [](uint8_t c) -> int {
                    if (c >= '0' && c <= '9') return c - '0';
                    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                    return -1;
                };
                const int hi = hexVal(buf[j + 1]);
                const int lo = hexVal(buf[j + 2]);
                if (hi < 0 || lo < 0) break;
                if (static_cast<uint8_t>((hi << 4) | lo) == sum) return true;
                break;                          // checksum present but wrong
            }
            // A second '$' means the first sentence was truncated; the outer
            // loop will pick this one up on its own iteration.
            if (buf[j] == '$') break;
            sum ^= buf[j];
        }
    }
    return false;
}

}  // namespace tr
