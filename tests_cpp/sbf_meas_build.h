#pragma once
// MeasEpoch (4027) blocks for host tests, built from their sub-block fields
// (mosaic-G5 Reference Guide, firmware 1.0.1, 4.2.1) with sbf_build.h's
// block(). Fields are given as the receiver's raw parts (MSB and LSB), so a
// test can set any do-not-use pair, or from whole values with the setters.
#include <cmath>
#include <cstdint>
#include <vector>

#include "sbf_build.h"

namespace sbftest {

inline int floorDiv(int64_t a, int64_t b) { return static_cast<int>(a >= 0 ? a / b : -((-a + b - 1) / b)); }

// A Type2 sub-block: a slave signal as offsets from its Type1.
struct MeasT2
{
    uint8_t  type = 0;        // SigIdxLo | antenna << 5
    uint8_t  lock = 20;       // s; 255 do-not-use
    uint8_t  cn0 = 128;       // 0.25 dB-Hz (+10): 42 dB-Hz
    int      code_msb = 0;    // 3-bit two's complement, -4..3
    uint16_t code_lsb = 0;
    int      dop_msb = 0;     // 5-bit two's complement, -16..15
    uint16_t dop_lsb = 0;
    int8_t   car_msb = 0;
    uint16_t car_lsb = 0;
    uint8_t  obs = 0;

    MeasT2& sig(uint8_t s, uint8_t antenna = 0) { type = static_cast<uint8_t>((s & 0x1F) | (antenna << 5)); return *this; }
    MeasT2& codeOffsetMm(int mm)
    {
        code_msb = floorDiv(mm, 65536);
        code_lsb = static_cast<uint16_t>(mm - code_msb * 65536);
        return *this;
    }
    MeasT2& dopplerOffset(int tenth_mhz)
    {
        dop_msb = floorDiv(tenth_mhz, 65536);
        dop_lsb = static_cast<uint16_t>(tenth_mhz - dop_msb * 65536);
        return *this;
    }
    MeasT2& carrier(int32_t mcyc)
    {
        car_msb = static_cast<int8_t>(floorDiv(mcyc, 65536));
        car_lsb = static_cast<uint16_t>(mcyc - car_msb * 65536);
        return *this;
    }
};

// A Type1 sub-block: one satellite's master signal, then its Type2s.
struct MeasT1
{
    uint8_t  rx_channel = 1;
    uint8_t  type = 0;        // SigIdxLo | antenna << 5
    uint8_t  svid = 1;
    uint8_t  code_msb = 0;    // Misc bits 0-3
    uint32_t code_lsb = 0;
    int32_t  doppler = 0;     // 0.1 mHz
    uint16_t car_lsb = 0;
    int8_t   car_msb = 0;
    uint8_t  cn0 = 128;
    uint16_t lock = 100;      // s; 65535 do-not-use
    uint8_t  obs = 0;
    std::vector<MeasT2> t2;

    MeasT1& sig(uint8_t s, uint8_t antenna = 0) { type = static_cast<uint8_t>((s & 0x1F) | (antenna << 5)); return *this; }
    MeasT1& codeMm(int64_t mm)
    {
        code_msb = static_cast<uint8_t>(mm >> 32);
        code_lsb = static_cast<uint32_t>(mm & 0xFFFFFFFF);
        return *this;
    }
    MeasT1& prM(double m) { return codeMm(static_cast<int64_t>(std::llround(m * 1000.0))); }
    MeasT1& dopplerHz(double hz) { doppler = static_cast<int32_t>(std::lround(hz * 1.0e4)); return *this; }
    MeasT1& carrier(int32_t mcyc)
    {
        car_msb = static_cast<int8_t>(floorDiv(mcyc, 65536));
        car_lsb = static_cast<uint16_t>(mcyc - car_msb * 65536);
        return *this;
    }
};

struct MeasEpochSpec
{
    uint32_t tow = 507606000;
    uint16_t wnc = 2440;
    uint8_t  rev = 1;
    uint8_t  flags = 0x25;    // the 10/16 captures': multipath mitigation, bit 2, high dynamics
    uint8_t  cum = 0;         // CumClkJumps
    uint8_t  sb1 = 20;        // SB1Length; more pads each Type1
    uint8_t  sb2 = 12;
    int      n1 = -1;         // N1 as written; -1: the number of Type1s given
    std::vector<MeasT1> t1;
};

inline std::vector<uint8_t> measEpoch(const MeasEpochSpec& s)
{
    Bytes b;
    b.u8(static_cast<uint8_t>(s.n1 >= 0 ? s.n1 : static_cast<int>(s.t1.size())))
     .u8(s.sb1).u8(s.sb2).u8(s.flags).u8(s.cum).u8(0);
    for (const MeasT1& t : s.t1)
    {
        const size_t at = b.v.size();
        b.u8(t.rx_channel).u8(t.type).u8(t.svid).u8(t.code_msb & 0x0F).u32(t.code_lsb).i32(t.doppler)
         .u16(t.car_lsb).i8(t.car_msb).u8(t.cn0).u16(t.lock).u8(t.obs).u8(static_cast<uint8_t>(t.t2.size()));
        while (b.v.size() - at < s.sb1) b.u8(0xA5);   // padding: never looked at
        for (const MeasT2& u : t.t2)
        {
            const size_t at2 = b.v.size();
            const uint8_t off = static_cast<uint8_t>((u.code_msb & 0x07) | ((u.dop_msb & 0x1F) << 3));
            b.u8(u.type).u8(u.lock).u8(u.cn0).u8(off).i8(u.car_msb).u8(u.obs)
             .u16(u.code_lsb).u16(u.car_lsb).u16(u.dop_lsb);
            while (b.v.size() - at2 < s.sb2) b.u8(0x5A);
        }
    }
    return block(4027, s.rev, s.tow, s.wnc, b.v);
}

}  // namespace sbftest
