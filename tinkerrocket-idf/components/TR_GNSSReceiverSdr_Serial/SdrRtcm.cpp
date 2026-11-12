// SdrRtcm.cpp — see SdrRtcm.h. Pure: no ESP-IDF, host-tested.
#include "SdrRtcm.h"

#include <algorithm>
#include <cstring>

#include "Rtcm3Msm.h"   // TR_GNSSReceiverLC86_Serial: rtcm3::crc24q

namespace sdr_rtcm
{

namespace
{

// `len` bits (at most 64) from bit `pos` of `p`, most significant first, as
// RTCM 3 packs them.
uint64_t bits64(const uint8_t* p, size_t pos, int len)
{
    uint64_t v = 0;
    for (int i = 0; i < len; i++, pos++) v = (v << 1) | ((p[pos / 8] >> (7 - pos % 8)) & 1U);
    return v;
}
uint32_t bits(const uint8_t* p, size_t pos, int len) { return static_cast<uint32_t>(bits64(p, pos, len)); }
// The same, two's complement.
int64_t sbits(const uint8_t* p, size_t pos, int len)
{
    const uint64_t v = bits64(p, pos, len);
    return (v >> (len - 1)) & 1U ? static_cast<int64_t>(v) - (static_cast<int64_t>(1) << len) : static_cast<int64_t>(v);
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
    // sum 48 (mm), mode 2, observables 6, sample 48; then (where the frame has
    // them) ranges left out 6, marks 6, and each mark: system 2, PRN 6, flags 2
    const uint8_t* m = trPayload(frame, len, kTrEpoch, 202);
    if (m == nullptr) return false;
    out = TrEpoch{};
    out.week       = static_cast<uint16_t>(bits(m, 16, 16));
    out.tow_ms     = bits(m, 32, 30);
    out.shift_ns   = static_cast<int32_t>(sbits(m, 62, 20));
    out.clk_steps  = static_cast<uint16_t>(bits(m, 82, 16));
    out.clk_step_m = static_cast<double>(sbits(m, 98, 48)) * 1.0e-3;
    out.mode       = static_cast<uint8_t>(bits(m, 146, 2));
    out.nobs       = static_cast<uint8_t>(bits(m, 148, 6));
    out.sample     = bits64(m, 154, 48);
    const size_t have = (len - 6) * 8;
    if (have >= 214)
    {
        const uint8_t n = static_cast<uint8_t>(bits(m, 208, 6));
        if (have >= 214 + 10 * static_cast<size_t>(n) && n <= kMaxMarks)
        {
            out.has_tail = true;
            out.ndrop = static_cast<uint8_t>(bits(m, 202, 6));
            out.nmark = n;
            for (int k = 0; k < n; k++)
            {
                const size_t at = 214 + 10 * static_cast<size_t>(k);
                out.mark[k].sys   = static_cast<uint8_t>(bits(m, at, 2));
                out.mark[k].prn   = static_cast<uint8_t>(bits(m, at + 2, 6));
                out.mark[k].flags = static_cast<uint8_t>(bits(m, at + 8, 2));
            }
        }
    }
    return true;
}

bool decodeTrFix(const uint8_t* frame, size_t len, TrFix& out)
{
    // number 12, sub-type 4, week 16, tow 30, sub-ms 20, valid 1, velocity
    // valid 1, satellites 6, then the fix itself (374 bits in all)
    // then position 3 x 38 (0.1 mm), velocity 3 x 28 (0.1 mm/s), clock bias 32
    // (mm), drift 28 (0.1 mm/s), PDOP 10 (0.1), residual rms 16 (cm)
    const uint8_t* m = trPayload(frame, len, kTrFix, 374);
    if (m == nullptr) return false;
    out = TrFix{};
    out.week      = static_cast<uint16_t>(bits(m, 16, 16));
    out.tow_ms    = bits(m, 32, 30);
    out.sub_ns    = static_cast<int32_t>(sbits(m, 62, 20));
    out.valid     = bits(m, 82, 1) != 0;
    out.vel_valid = bits(m, 83, 1) != 0;
    out.nsat      = static_cast<uint8_t>(bits(m, 84, 6));
    for (int k = 0; k < 3; k++)
    {
        out.pos[k] = static_cast<double>(sbits(m, 90 + 38 * static_cast<size_t>(k), 38)) * 1.0e-4;
        out.vel[k] = static_cast<double>(sbits(m, 204 + 28 * static_cast<size_t>(k), 28)) * 1.0e-4;
    }
    out.clk_bias_m    = static_cast<double>(sbits(m, 288, 32)) * 1.0e-3;
    out.clk_drift_mps = static_cast<double>(sbits(m, 320, 28)) * 1.0e-4;
    out.pdop          = bits(m, 348, 10) * 0.1;
    out.resid_rms_m   = bits(m, 358, 16) * 0.01;
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

bool decodeTrIono(const uint8_t* frame, size_t len, TrIono& out)
{
    // number 12, sub-type 4, week 16, second of week 20, source 2, then
    // alpha0..3 and beta0..3 8 each, two's complement (118 bits; 2 reserved)
    const uint8_t* m = trPayload(frame, len, kTrIono, 118);
    if (m == nullptr) return false;
    out.week  = static_cast<uint16_t>(bits(m, 16, 16));
    out.tow_s = bits(m, 32, 20);
    out.src   = static_cast<uint8_t>(bits(m, 52, 2));
    for (int k = 0; k < 8; k++) out.raw[k] = static_cast<int8_t>(sbits(m, 54 + 8 * k, 8));
    return out.src != 0;
}

bool decodeTrAlign(const uint8_t* frame, size_t len, TrAlign& out)
{
    // number 12, sub-type 4, seq 8, gen 8, imu 2, time 1, have 2, from 2,
    // saved 2, autosave 1, frame 1, poses 4, calibrated axes 3, the last
    // rest 3, rests 8, paired 8, the command's id 8, op 3, result 3, spread
    // 8, residual 8, fit 8 (107 bits), the mounting 4 x 16, scales 3 x 16,
    // biases 3 x 16, moved 8 (275), then each pose: its direction 3 x 8 and
    // its pairs 8
    const uint8_t* m = trPayload(frame, len, kTrAlign, 275);
    if (m == nullptr) return false;
    const uint8_t n_pose = static_cast<uint8_t>(bits(m, 43, 4));
    if (n_pose > TrAlign::kPoses || (len - 6) * 8 < 275 + 32 * static_cast<size_t>(n_pose)) return false;
    TrAlign a;
    a.seq       = static_cast<uint8_t>(bits(m, 16, 8));
    a.gen       = static_cast<uint8_t>(bits(m, 24, 8));
    a.imu       = static_cast<uint8_t>(bits(m, 32, 2));
    a.time      = bits(m, 34, 1) != 0;
    a.have      = static_cast<uint8_t>(bits(m, 35, 2));
    a.from      = static_cast<uint8_t>(bits(m, 37, 2));
    a.saved     = static_cast<uint8_t>(bits(m, 39, 2));
    a.autosave  = bits(m, 41, 1) != 0;
    a.frame     = static_cast<uint8_t>(bits(m, 42, 1));
    a.n_pose    = n_pose;
    a.cal_axes  = static_cast<uint8_t>(bits(m, 47, 3));
    a.last_rest = static_cast<uint8_t>(bits(m, 50, 3));
    a.n_rest    = static_cast<uint8_t>(bits(m, 53, 8));
    a.n_pair    = static_cast<uint8_t>(bits(m, 61, 8));
    a.cmd_id    = static_cast<uint8_t>(bits(m, 69, 8));
    a.cmd_op    = static_cast<uint8_t>(bits(m, 77, 3));
    a.cmd_res   = static_cast<uint8_t>(bits(m, 80, 3));
    a.spread_deg = static_cast<double>(bits(m, 83, 8));
    a.resid_deg  = static_cast<double>(bits(m, 91, 8)) * 0.02;
    a.cal_rms    = static_cast<double>(bits(m, 99, 8)) * 1e-3;
    for (int k = 0; k < 4; k++) a.q[k] = static_cast<double>(sbits(m, 107 + 16 * k, 16)) / 16384.0;
    for (int k = 0; k < 3; k++) a.scale[k] = 1.0 + static_cast<double>(sbits(m, 171 + 16 * k, 16)) * 1e-5;
    for (int k = 0; k < 3; k++) a.bias[k] = static_cast<double>(sbits(m, 219 + 16 * k, 16)) * 1e-4;
    a.moved_deg = static_cast<double>(bits(m, 267, 8)) * 0.02;
    for (int i = 0; i < n_pose; i++)
    {
        const size_t at = 275 + 32 * static_cast<size_t>(i);
        for (int k = 0; k < 3; k++) a.pose[i].dir[k] = static_cast<double>(sbits(m, at + 8 * k, 8)) / 127.0;
        a.pose[i].pairs = static_cast<uint8_t>(bits(m, at + 24, 8));
    }
    out = a;
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
