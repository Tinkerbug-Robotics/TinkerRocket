// Rtcm3Msm.cpp — see Rtcm3Msm.h. NO ESP-IDF includes in this file: it must
// compile on the host for tests_cpp/test_lc86_parser.cpp.
#include "Rtcm3Msm.h"

#include <cmath>

namespace rtcm3
{

// Light travel per millisecond: MSM ranges are in light-milliseconds.
static constexpr double kRangeMsToM = 299792458.0 / 1000.0;

// Big-endian bit reader over one payload. Reads past the end return zeros,
// but decodeMsm7() checks the length against the masks before it reads any
// per-satellite field, so that never happens on a message it accepts.
namespace
{
class Bits
{
public:
    Bits(const uint8_t* p, size_t len) : p_(p), nbits_(len * 8) {}

    uint32_t u(unsigned k)
    {
        uint32_t v = 0;
        for (unsigned i = 0; i < k; i++, pos_++)
        {
            const uint32_t bit = (pos_ < nbits_)
                ? (uint32_t)((p_[pos_ >> 3] >> (7 - (pos_ & 7))) & 1u) : 0u;
            v = (v << 1) | bit;
        }
        return v;
    }

    uint64_t u64(unsigned k)
    {
        uint64_t v = 0;
        for (unsigned i = 0; i < k; i++) v = (v << 1) | u(1);
        return v;
    }

    int32_t s(unsigned k)
    {
        const uint32_t v = u(k);
        return (v & (1u << (k - 1))) ? (int32_t)(v - (1u << k)) : (int32_t)v;
    }

    size_t pos() const { return pos_; }

private:
    const uint8_t* p_;
    size_t nbits_;
    size_t pos_ = 0;
};
}  // namespace

uint32_t crc24q(const uint8_t* data, size_t len)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint32_t)data[i] << 16;
        for (int b = 0; b < 8; b++)
        {
            crc <<= 1;
            if (crc & 0x1000000u) crc ^= 0x1864CFBu;
        }
    }
    return crc & 0xFFFFFFu;
}

uint16_t messageType(const uint8_t* payload, size_t len)
{
    if (len < 2) return 0;
    return (uint16_t)((payload[0] << 4) | (payload[1] >> 4));
}

uint8_t msm7GnssId(uint16_t msg_type)
{
    switch (msg_type)
    {
        case 1077: return 0;   // GPS
        case 1087: return 6;   // GLONASS
        case 1097: return 2;   // Galileo
        case 1117: return 5;   // QZSS
        case 1127: return 3;   // BeiDou
        default:   return 0xFF;
    }
}

bool isEphemeris(uint16_t msg_type)
{
    return msg_type == 1019 || msg_type == 1020 || msg_type == 1042 ||
           msg_type == 1044 || msg_type == 1046;
}

uint32_t lockTimeMs(uint16_t i)
{
    // DF407 (extended resolution): 0-63 count milliseconds one to one, then
    // each band of 32 doubles the step, continuous at every band edge
    // (64 -> 64, 96 -> 128, 128 -> 256 ...); 704 and up saturate.
    if (i < 64) return i;
    if (i >= 704) return 67108864u;
    const uint32_t b = (uint32_t)(i - 64) / 32u;
    return (2u << b) * (uint32_t)(i - (64 + 32 * b)) + (64u << b);
}

// The header fields in front of the satellite mask; leaves `b` at DF394.
static void readHeader(Bits& b, uint16_t mt, uint8_t gnss, MsmHeader& hdr)
{
    b.u(12);                           // DF002 message number
    b.u(12);                           // DF003 reference station ID
    const uint32_t epoch = b.u(30);    // DF004 / DF248 / DF427 / DF428, or DF416 + DF034
    const bool multiple = b.u(1) != 0; // DF393
    b.u(3);                            // DF409 IODS
    b.u(7);                            // DF001 reserved
    b.u(2);                            // DF411 clock steering
    b.u(2);                            // DF412 external clock
    b.u(1);                            // DF417 smoothing
    b.u(3);                            // DF418 smoothing interval

    hdr.msg_type    = mt;
    hdr.gnss_id     = gnss;
    hdr.epoch_field = epoch;
    hdr.multiple    = multiple;
    hdr.n_sats      = 0;
    hdr.n_cells     = 0;
    switch (gnss)
    {
        case 6:   // GLONASS: day of week + ms of day in UTC(SU) + 3 h
            hdr.gps_tow_ms = kInvalidTow;
            break;
        case 3:   // BeiDou time runs 14 s behind GPS time
            hdr.gps_tow_ms = (epoch + 14000u) % 604800000u;
            break;
        default:  // GPS, Galileo and QZSS count GPS time of week
            hdr.gps_tow_ms = epoch;
            break;
    }
}

// 73 header bits, then the 64-bit satellite and 32-bit signal masks.
static constexpr size_t kMsmFixedBits = 73 + 64 + 32;

bool peekMsm7(const uint8_t* payload, size_t len, MsmHeader& hdr)
{
    const uint16_t mt = messageType(payload, len);
    const uint8_t gnss = msm7GnssId(mt);
    if (gnss == 0xFF || len * 8 < kMsmFixedBits) return false;
    Bits b(payload, len);
    readHeader(b, mt, gnss, hdr);
    return true;
}

bool decodeMsm7(const uint8_t* payload, size_t len, MsmHeader& hdr,
                MsmCell* cells, size_t cap, size_t* dropped)
{
    const uint16_t mt = messageType(payload, len);
    const uint8_t gnss = msm7GnssId(mt);
    if (gnss == 0xFF || len * 8 < kMsmFixedBits) return false;

    Bits b(payload, len);
    MsmHeader h;
    readHeader(b, mt, gnss, h);
    const uint64_t sat_mask = b.u64(64);   // DF394
    const uint32_t sig_mask = b.u(32);     // DF395

    uint8_t sats[64];
    uint8_t sigs[32];
    unsigned ns = 0, ng = 0;
    for (unsigned i = 0; i < 64; i++)
    {
        if ((sat_mask >> (63 - i)) & 1u) sats[ns++] = (uint8_t)(i + 1);
    }
    for (unsigned i = 0; i < 32; i++)
    {
        if ((sig_mask >> (31 - i)) & 1u) sigs[ng++] = (uint8_t)(i + 1);
    }
    // The cell mask (DF396) is at most 64 bits [RTCM 10403.3 3.5.12.3].
    if (ns * ng > 64) return false;

    uint8_t cell_sat[64];
    uint8_t cell_sig[64];
    unsigned nc = 0;
    for (unsigned s = 0; s < ns; s++)
    {
        for (unsigned g = 0; g < ng; g++)
        {
            if (b.u(1))
            {
                cell_sat[nc] = (uint8_t)s;
                cell_sig[nc] = (uint8_t)g;
                nc++;
            }
        }
    }

    // MSM7: 36 bits per satellite, 80 per cell, the payload padded to a byte.
    const size_t need_bits = b.pos() + 36u * ns + 80u * nc;
    if (len * 8 < need_bits || len * 8 - need_bits >= 8) return false;

    uint8_t  rough_ms[64];
    uint8_t  ext[64];
    uint16_t rough_mod[64];
    int16_t  rough_rate[64];
    for (unsigned s = 0; s < ns; s++) rough_ms[s]   = (uint8_t)b.u(8);    // DF397
    for (unsigned s = 0; s < ns; s++) ext[s]        = (uint8_t)b.u(4);    // DF419
    for (unsigned s = 0; s < ns; s++) rough_mod[s]  = (uint16_t)b.u(10);  // DF398
    for (unsigned s = 0; s < ns; s++) rough_rate[s] = (int16_t)b.s(14);   // DF399

    int32_t  fine_pr[64];
    int32_t  fine_cp[64];
    uint16_t lock[64];
    uint8_t  half[64];
    uint16_t cnr[64];
    int16_t  fine_rate[64];
    for (unsigned c = 0; c < nc; c++) fine_pr[c]   = b.s(20);              // DF405
    for (unsigned c = 0; c < nc; c++) fine_cp[c]   = b.s(24);              // DF406
    for (unsigned c = 0; c < nc; c++) lock[c]      = (uint16_t)b.u(10);    // DF407
    for (unsigned c = 0; c < nc; c++) half[c]      = (uint8_t)b.u(1);      // DF420
    for (unsigned c = 0; c < nc; c++) cnr[c]       = (uint16_t)b.u(10);    // DF408
    for (unsigned c = 0; c < nc; c++) fine_rate[c] = (int16_t)b.s(15);     // DF404

    hdr = h;
    hdr.n_sats  = (uint8_t)ns;
    hdr.n_cells = (uint8_t)nc;

    const double nan = std::nan("");
    size_t out = 0;
    for (unsigned c = 0; c < nc; c++)
    {
        if (out >= cap)
        {
            if (dropped) (*dropped)++;
            continue;
        }
        const unsigned s = cell_sat[c];
        // The rough range is shared by the satellite's code and phase; 255
        // in DF397 says it is invalid (DF405/DF406 have their own sentinels).
        const bool rough_ok = rough_ms[s] != 255;
        const double rough  = rough_ms[s] + rough_mod[s] / 1024.0;

        MsmCell& m = cells[out++];
        m.pseudorange_m = (rough_ok && fine_pr[c] != -524288)
            ? (rough + fine_pr[c] * std::ldexp(1.0, -29)) * kRangeMsToM : nan;
        m.phase_range_m = (rough_ok && fine_cp[c] != -8388608)
            ? (rough + fine_cp[c] * std::ldexp(1.0, -31)) * kRangeMsToM : nan;
        m.rate_mps = (rough_rate[s] != -8192 && fine_rate[c] != -16384)
            ? (float)(rough_rate[s] + fine_rate[c] * 1e-4) : (float)nan;
        m.cn0_dbhz   = cnr[c] / 16.0f;
        m.lock_ms    = lockTimeMs(lock[c]);
        m.gnss_id    = gnss;
        m.sv_id      = sats[s];
        m.sig_id     = sigs[cell_sig[c]];
        m.ext_info   = ext[s];
        m.half_cycle = half[c] != 0;
    }
    return true;
}

}  // namespace rtcm3
