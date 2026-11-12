// SdrRtcm.cpp — see SdrRtcm.h. Pure: no ESP-IDF, host-tested.
#include "SdrRtcm.h"

#include <algorithm>
#include <cstring>

#include "Rtcm3Msm.h"   // TR_GNSSReceiverLC86_Serial: rtcm3::crc24q

namespace sdr_rtcm
{

namespace
{

// `len` bits (at most 32) from bit `pos` of `p`, most significant first, as
// RTCM 3 packs them.
uint32_t bits(const uint8_t* p, size_t pos, int len)
{
    uint32_t v = 0;
    for (int i = 0; i < len; i++, pos++) v = (v << 1) | ((p[pos / 8] >> (7 - pos % 8)) & 1U);
    return v;
}

// A signed field of `len` bits (at most 64), two's complement.
int64_t bitsS(const uint8_t* p, size_t pos, int len)
{
    uint64_t v = 0;
    for (int i = 0; i < len; i++, pos++) v = (v << 1) | ((p[pos / 8] >> (7 - pos % 8)) & 1U);
    return (v >> (len - 1)) & 1U ? static_cast<int64_t>(v) - (int64_t{1} << len) : static_cast<int64_t>(v);
}

// The payload of a 4001 frame of sub-type `sub` long enough for `need_bits`.
const uint8_t* trPayload(const uint8_t* frame, size_t len, uint8_t sub, size_t need_bits)
{
    if (trSubType(frame, len) != sub || (len - 6) * 8 < need_bits) return nullptr;
    return frame + 3;
}

}  // namespace

uint16_t messageNumber(const uint8_t* frame, size_t len)
{
    if (frame == nullptr || len < 3 + 2 + 3) return 0;
    return static_cast<uint16_t>((frame[3] << 4) | (frame[4] >> 4));
}

uint8_t trSubType(const uint8_t* frame, size_t len)
{
    return messageNumber(frame, len) == kTrMessage ? static_cast<uint8_t>(frame[4] & 0x0F) : 0;
}

bool decodeTrEpoch(const uint8_t* frame, size_t len, TrEpoch& out)
{
    // number 12, sub-type 4, week 16, tow 30, shift 20, clock steps 16, their
    // sum 48, mode 2, observables 6, sample 48
    const uint8_t* m = trPayload(frame, len, kTrEpoch, 202);
    if (m == nullptr) return false;
    out.week   = static_cast<uint16_t>(bits(m, 16, 16));
    out.tow_ms = bits(m, 32, 30);
    out.mode   = static_cast<uint8_t>(bits(m, 146, 2));
    out.nobs   = static_cast<uint8_t>(bits(m, 148, 6));
    return true;
}

bool decodeTrFix(const uint8_t* frame, size_t len, TrFix& out)
{
    // number 12, sub-type 4, week 16, tow 30, sub-ms 20, valid 1, velocity
    // valid 1, satellites 6, then the fix itself (374 bits in all): its ECEF
    // position first, three signed 38-bit fields of 0.1 mm (the board's
    // rtcm_tr_fix_encode)
    const uint8_t* m = trPayload(frame, len, kTrFix, 374);
    if (m == nullptr) return false;
    out.week   = static_cast<uint16_t>(bits(m, 16, 16));
    out.tow_ms = bits(m, 32, 30);
    out.valid  = bits(m, 82, 1) != 0;
    out.nsat   = static_cast<uint8_t>(bits(m, 84, 6));
    for (int k = 0; k < 3; k++) out.pos[k] = static_cast<double>(bitsS(m, 90 + 38 * k, 38)) * 1e-4;
    return true;
}

bool decodeTrStatus(const uint8_t* frame, size_t len, TrStatus& out)
{
    // number 12, sub-type 4, the commit 32, dirty 1, uptime 20, tracking 6,
    // locked 6, time 1, fix 1 (83 bits)
    const uint8_t* m = trPayload(frame, len, kTrStatus, 83);
    if (m == nullptr) return false;
    out.fw         = bits(m, 16, 32);
    out.fw_dirty   = bits(m, 48, 1) != 0;
    out.uptime_s   = bits(m, 49, 20);
    out.ntrack     = static_cast<uint8_t>(bits(m, 69, 6));
    out.nlock      = static_cast<uint8_t>(bits(m, 75, 6));
    out.time_valid = bits(m, 81, 1) != 0;
    out.fix_valid  = bits(m, 82, 1) != 0;
    return true;
}

// ── Rtcm3Framer ─────────────────────────────────────────────────────────

void Rtcm3Framer::feed(const uint8_t* data, size_t n, Sink& sink)
{
    if (data == nullptr) return;
    size_t i = 0;
    while (i < n)
    {
        // parse() always leaves room: it keeps less than one frame
        const size_t take = std::min(n - i, kBuf - n_);
        std::memcpy(buf_ + n_, data + i, take);
        n_ += take;
        i += take;
        fed_after_ = n - i;
        parse(sink);
    }
    fed_after_ = 0;
}

void Rtcm3Framer::parse(Sink& sink)
{
    size_t p = 0;
    for (;;)
    {
        size_t q = p;
        while (q < n_ && buf_[q] != rtcm3::kPreamble) q++;
        skipped_ += static_cast<uint32_t>(q - p);
        p = q;
        if (n_ - p < 3) break;
        if ((buf_[p + 1] & 0xFC) != 0)   // the six reserved bits: no frame starts here
        {
            skipped_++;
            p++;
            continue;
        }
        const size_t len = 6 + ((static_cast<size_t>(buf_[p + 1] & 0x03) << 8) | buf_[p + 2]);
        if (n_ - p < len) break;
        const uint32_t want = (static_cast<uint32_t>(buf_[p + len - 3]) << 16) |
                              (static_cast<uint32_t>(buf_[p + len - 2]) << 8) | buf_[p + len - 1];
        if (rtcm3::crc24q(buf_ + p, len - 3) == want)
        {
            frames_++;
            sink.onFrame(buf_ + p, len, (n_ - (p + len)) + fed_after_);
            p += len;
        }
        else
        {
            // Not a frame here; one may begin inside it.
            crc_fails_++;
            skipped_++;
            p++;
        }
    }
    drop(p);
}

void Rtcm3Framer::drop(size_t k)
{
    if (k == 0) return;
    std::memmove(buf_, buf_ + k, n_ - k);
    n_ -= k;
}

}  // namespace sdr_rtcm
