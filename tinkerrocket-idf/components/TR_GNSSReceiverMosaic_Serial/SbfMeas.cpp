// SbfMeas.cpp — see SbfMeas.h. Pure: no ESP-IDF, host-tested.
#include "SbfMeas.h"

#include <cmath>

#include "SbfFields.h"

namespace mosaic
{

using sbf::f32;
using sbf::i16;
using sbf::i32;
using sbf::u16;
using sbf::u32;

namespace
{

constexpr size_t kHeaderLen = 20;       // through MeasEpoch's Reserved / MeasExtra's DopplerVarFactor
constexpr uint8_t kMinSb1Len = 20;      // a Type1 through N2
constexpr uint8_t kMinSb2Len = 12;      // a Type2 through DopplerOffsetLSB
constexpr uint8_t kMinExtraLen = 12;    // a MeasExtra sub-block through LockTime (revision 0)
constexpr int32_t kDopplerDnu = -2147483647 - 1;

// The signal number from a Type field's SigIdxLo, or from ObsInfo/Misc bits
// 3-7 (+32) when SigIdxLo is 31.
inline uint8_t signalNumber(uint8_t type, uint8_t ext)
{
    const uint8_t lo = type & 0x1F;
    return lo == 31 ? static_cast<uint8_t>((ext >> 3) + 32) : lo;
}

inline bool isGloFdma(uint8_t sig) { return sig >= 8 && sig <= 11; }

// Type1 ObsInfo bits 3-7 for GLONASS L1/L2: FreqNr, the frequency number
// plus 8 (1 = -7 ... 14 = +6; 0 is FreqNr's do-not-use).
inline int8_t gloFn(uint8_t obs_info)
{
    const int v = obs_info >> 3;
    return (v >= 1 && v <= 14) ? static_cast<int8_t>(v - 8) : kGloFnUnknown;
}

// A Header's common part: sync, block number and Length against the bytes.
SbfStatus readCommon(const uint8_t* b, size_t len, uint16_t number, uint32_t& tow_ms, uint16_t& wnc,
                     uint8_t& revision, uint16_t& length)
{
    if (b == nullptr || len < 8) return SbfStatus::SHORT;
    if (b[0] != '$' || b[1] != '@') return SbfStatus::WRONG_BLOCK;
    const uint16_t id = u16(b + 4);
    if ((id & 0x1FFF) != number) return SbfStatus::WRONG_BLOCK;
    revision = static_cast<uint8_t>(id >> 13);
    length = u16(b + 6);
    if (length > len || length < kHeaderLen) return SbfStatus::SHORT;
    tow_ms = u32(b + 8);
    wnc = u16(b + 12);
    return SbfStatus::OK;
}

}  // namespace

// ── Signals and satellites ──────────────────────────────────────────────

SbfSys sbfSvid(uint8_t svid, uint8_t& prn)
{
    prn = 0;
    if (svid >= 1 && svid <= 37) { prn = svid; return SbfSys::GPS; }
    if (svid >= 38 && svid <= 61) { prn = static_cast<uint8_t>(svid - 37); return SbfSys::GLO; }
    if (svid == 62) return SbfSys::GLO;   // slot not known (may repeat within an epoch)
    if (svid >= 63 && svid <= 68) { prn = static_cast<uint8_t>(svid - 38); return SbfSys::GLO; }
    if (svid >= 71 && svid <= 106) { prn = static_cast<uint8_t>(svid - 70); return SbfSys::GAL; }
    if (svid >= 107 && svid <= 119) { prn = svid; return SbfSys::LBAND; }
    if (svid >= 120 && svid <= 140) { prn = svid; return SbfSys::SBAS; }
    if (svid >= 141 && svid <= 180) { prn = static_cast<uint8_t>(svid - 140); return SbfSys::BDS; }
    if (svid >= 181 && svid <= 190) { prn = static_cast<uint8_t>(svid - 180); return SbfSys::QZS; }
    if (svid >= 191 && svid <= 197) { prn = static_cast<uint8_t>(svid - 190); return SbfSys::NAVIC; }
    if (svid >= 198 && svid <= 215) { prn = static_cast<uint8_t>(svid - 57); return SbfSys::SBAS; }
    if (svid >= 216 && svid <= 222) { prn = static_cast<uint8_t>(svid - 208); return SbfSys::NAVIC; }
    if (svid >= 223 && svid <= 245) { prn = static_cast<uint8_t>(svid - 182); return SbfSys::BDS; }
    return SbfSys::NONE;
}

bool sbfSignalDefined(uint8_t sig)
{
    return sig <= 39 && sig != 16 && sig != 18 && sig != 31 && sig != 35 && sig != 36;
}

double sbfCarrierHz(uint8_t sig, int8_t glo_fn)
{
    switch (sig)
    {
        case 0: case 1: case 5: case 6: case 13: case 17: case 24: case 32: case 33: case 37: case 38:
            return 1575.42e6;
        case 2: case 3: case 7:
            return 1227.60e6;
        case 4: case 14: case 15: case 20: case 25: case 26: case 39:
            return 1176.45e6;
        case 8: case 9:
            return glo_fn == kGloFnUnknown ? 0.0 : 1602.0e6 + glo_fn * 562.5e3;   // 9/16 MHz a channel
        case 10: case 11:
            return glo_fn == kGloFnUnknown ? 0.0 : 1246.0e6 + glo_fn * 437.5e3;   // 7/16 MHz a channel
        case 12: return 1202.025e6;
        case 19: case 27: return 1278.75e6;
        case 21: case 29: case 34: return 1207.14e6;
        case 22: return 1191.795e6;
        case 28: return 1561.098e6;
        case 30: return 1268.52e6;
        default: return 0.0;   // reserved, or L-band (beam specific)
    }
}

// ── MeasEpoch ───────────────────────────────────────────────────────────

SbfStatus SbfMeasReader::begin(const uint8_t* b, size_t len)
{
    b_ = b;
    hdr_ = SbfMeasHeader{};
    ok_ = false;
    t1_left_ = 0;
    t2_left_ = 0;
    skipped_ = 0;
    m_ = Master{};

    const SbfStatus st = readCommon(b, len, SBF_MEAS_EPOCH, hdr_.tow_ms, hdr_.wnc, hdr_.revision, hdr_.length);
    if (st != SbfStatus::OK) return st;
    hdr_.n1 = b[14];
    hdr_.sb1_len = b[15];
    hdr_.sb2_len = b[16];
    hdr_.common_flags = b[17];
    hdr_.has_cum_clk_jumps = hdr_.revision >= 1;
    hdr_.cum_clk_jumps = hdr_.has_cum_clk_jumps ? b[18] : 0;

    // Walk the two levels once against Length before handing anything out.
    // SB1Length and SB2Length only matter when there are sub-blocks of that
    // kind (an empty epoch, N1 = 0, is Length 20).
    if (hdr_.n1 > 0 && hdr_.sb1_len < kMinSb1Len) return SbfStatus::BAD_LAYOUT;
    size_t at = kHeaderLen;
    for (uint8_t i = 0; i < hdr_.n1; i++)
    {
        if (at + hdr_.sb1_len > hdr_.length) return SbfStatus::BAD_LAYOUT;
        const uint8_t n2 = b[at + 19];
        if (n2 > 0 && hdr_.sb2_len < kMinSb2Len) return SbfStatus::BAD_LAYOUT;
        at += hdr_.sb1_len + static_cast<size_t>(n2) * hdr_.sb2_len;
        if (at > hdr_.length) return SbfStatus::BAD_LAYOUT;
    }
    if (hdr_.common_flags & 0x80) return SbfStatus::SCRAMBLED;
    if (hdr_.tow_ms == kTowDnu || hdr_.wnc == kWncDnu) return SbfStatus::NO_TIME;

    ok_ = true;
    t1_at_ = kHeaderLen;
    t1_left_ = hdr_.n1;
    return SbfStatus::OK;
}

bool SbfMeasReader::next(SbfMeasSignal& out)
{
    if (!ok_) return false;
    for (;;)
    {
        if (t2_left_ > 0)
        {
            const uint8_t* q = b_ + t2_at_;
            t2_at_ += hdr_.sb2_len;
            t2_left_--;
            if (!m_.defined)
            {
                skipped_++;
                continue;
            }
            const uint8_t type = q[0];
            const uint8_t obs = q[5];
            const uint8_t sig = signalNumber(type, obs);
            if (!sbfSignalDefined(sig))
            {
                skipped_++;
                continue;
            }
            out = SbfMeasSignal{};
            out.rx_channel = m_.rx_channel;
            out.svid = m_.svid;
            out.sig = sig;
            out.antenna = static_cast<uint8_t>(type >> 5);
            // a Type2's ObsInfo bits 3-7 are reserved for GLONASS (0 on the
            // 10/16 captures): its frequency number is the master's
            out.glo_fn = isGloFdma(sig) ? m_.glo_fn : kGloFnUnknown;
            out.master = false;
            out.half_cycle = (obs & 0x04) != 0;
            out.smoothed = (obs & 0x01) != 0;
            out.cn0_raw = q[2];
            out.lock_s = q[1] == 255 ? 65535 : q[1];

            // OffsetsMSB: code bits 0-2 and Doppler bits 3-7, both two's complement
            const uint8_t off = q[3];
            const int code_msb = (off & 0x07) >= 4 ? (off & 0x07) - 8 : (off & 0x07);
            const int dop_msb = (off >> 3) >= 16 ? (off >> 3) - 32 : (off >> 3);
            const uint16_t code_lsb = u16(q + 6);
            const uint16_t dop_lsb = u16(q + 10);
            const int8_t car_msb = static_cast<int8_t>(q[4]);
            const uint16_t car_lsb = u16(q + 8);

            out.pr_valid = m_.pr_valid && !(code_msb == -4 && code_lsb == 0);
            out.pr_mm = m_.pr_mm + static_cast<int64_t>(code_msb) * 65536 + code_lsb;
            const double f2 = sbfCarrierHz(sig, out.glo_fn);
            out.dop_valid = m_.dop_valid && !(dop_msb == -16 && dop_lsb == 0) && m_.carrier_hz > 0.0 && f2 > 0.0;
            if (out.dop_valid)
                out.doppler_hz = m_.doppler_hz * f2 / m_.carrier_hz + (dop_msb * 65536 + dop_lsb) * 1.0e-4;
            out.carrier_valid = !(car_msb == -128 && car_lsb == 0);
            out.carrier_mcyc = car_msb * 65536 + car_lsb;
            return true;
        }
        if (t1_left_ == 0) return false;

        const uint8_t* p = b_ + t1_at_;
        const uint8_t n2 = p[19];
        t2_at_ = t1_at_ + hdr_.sb1_len;
        t2_left_ = n2;
        t1_at_ = t2_at_ + static_cast<size_t>(n2) * hdr_.sb2_len;
        t1_left_--;

        const uint8_t type = p[1];
        const uint8_t obs = p[18];
        m_ = Master{};
        uint8_t prn;
        if (sbfSvid(p[2], prn) == SbfSys::NONE)
        {
            skipped_++;   // its Type2s go with it, counted as they pass
            continue;
        }
        m_.defined = true;
        m_.rx_channel = p[0];
        m_.svid = p[2];
        const uint8_t sig = signalNumber(type, obs);
        if ((type & 0x1F) != 31 && isGloFdma(sig)) m_.glo_fn = gloFn(obs);

        const uint8_t code_msb = p[3] & 0x0F;
        const uint32_t code_lsb = u32(p + 4);
        m_.pr_valid = !(code_msb == 0 && code_lsb == 0);
        m_.pr_mm = (static_cast<int64_t>(code_msb) << 32) + code_lsb;
        const int32_t dop = i32(p + 8);
        m_.dop_valid = dop != kDopplerDnu;
        m_.doppler_hz = dop * 1.0e-4;
        m_.carrier_hz = sbfSignalDefined(sig) ? sbfCarrierHz(sig, m_.glo_fn) : 0.0;

        if (!sbfSignalDefined(sig))
        {
            // the master is passed over; its Type2s still read (their code
            // is an offset from its), but none can scale a Doppler from it
            skipped_++;
            continue;
        }
        out = SbfMeasSignal{};
        out.rx_channel = m_.rx_channel;
        out.svid = m_.svid;
        out.sig = sig;
        out.antenna = static_cast<uint8_t>(type >> 5);
        out.glo_fn = m_.glo_fn;
        out.master = true;
        out.half_cycle = (obs & 0x04) != 0;
        out.smoothed = (obs & 0x01) != 0;
        out.cn0_raw = p[15];
        out.lock_s = u16(p + 16);
        out.pr_valid = m_.pr_valid;
        out.pr_mm = m_.pr_mm;
        out.dop_valid = m_.dop_valid;
        out.doppler_hz = m_.dop_valid ? m_.doppler_hz : 0.0;
        const int8_t car_msb = static_cast<int8_t>(p[14]);
        const uint16_t car_lsb = u16(p + 12);
        out.carrier_valid = !(car_msb == -128 && car_lsb == 0);
        out.carrier_mcyc = car_msb * 65536 + car_lsb;
        return true;
    }
}

// ── MeasExtra ───────────────────────────────────────────────────────────

SbfStatus SbfExtraReader::begin(const uint8_t* b, size_t len)
{
    b_ = b;
    hdr_ = SbfExtraHeader{};
    ok_ = false;
    next_ = 0;
    skipped_ = 0;

    const SbfStatus st = readCommon(b, len, SBF_MEAS_EXTRA, hdr_.tow_ms, hdr_.wnc, hdr_.revision, hdr_.length);
    if (st != SbfStatus::OK) return st;
    const uint8_t n = b[14];
    hdr_.sb_len = b[15];
    hdr_.doppler_var_factor = f32(b + 16);
    if (hdr_.sb_len < kMinExtraLen) return SbfStatus::BAD_LAYOUT;
    // N is the count mod 256: NrSB = ((Length / SBLength - N) / 256) * 256 + N,
    // integer divisions (4.2.1)
    const int nr_sb = ((hdr_.length / hdr_.sb_len - n) / 256) * 256 + n;
    if (nr_sb < 0 || kHeaderLen + static_cast<size_t>(nr_sb) * hdr_.sb_len > hdr_.length)
        return SbfStatus::BAD_LAYOUT;
    hdr_.n_sb = static_cast<uint16_t>(nr_sb);
    if (hdr_.tow_ms == kTowDnu || hdr_.wnc == kWncDnu) return SbfStatus::NO_TIME;
    ok_ = true;
    return SbfStatus::OK;
}

bool SbfExtraReader::next(SbfExtraSignal& out)
{
    if (!ok_) return false;
    while (next_ < hdr_.n_sb)
    {
        const uint8_t* p = b_ + kHeaderLen + static_cast<size_t>(next_) * hdr_.sb_len;
        next_++;
        const uint8_t type = p[1];
        // Misc (and the SigIdxLo 31 extension in it) came in revision 3
        const bool has_misc = hdr_.revision >= 3 && hdr_.sb_len >= 16;
        if ((type & 0x1F) == 31 && !has_misc)
        {
            skipped_++;
            continue;
        }
        const uint8_t sig = signalNumber(type, has_misc ? p[15] : 0);
        if (!sbfSignalDefined(sig))
        {
            skipped_++;
            continue;
        }
        out = SbfExtraSignal{};
        out.rx_channel = p[0];
        out.sig = sig;
        out.antenna = static_cast<uint8_t>(type >> 5);
        out.mp_correction_mm = i16(p + 2);
        out.smoothing_corr_mm = i16(p + 4);
        out.code_var = u16(p + 6);
        out.carrier_var = u16(p + 8);
        out.lock_s = u16(p + 10);
        if (hdr_.revision >= 1 && hdr_.sb_len >= 13)
        {
            out.has_cum_loss_cont = true;
            out.cum_loss_cont = p[12];
        }
        if (hdr_.revision >= 1 && hdr_.sb_len >= 14)
        {
            out.has_car_mp_corr = true;
            out.car_mp_corr = static_cast<int8_t>(p[13]);
        }
        if (has_misc)
        {
            out.has_cn0_hires = true;
            out.cn0_hires = p[15] & 0x07;
        }
        return true;
    }
    return false;
}

double sbfDopplerSigmaMps(uint16_t carrier_var, float doppler_var_factor, double lambda_m)
{
    if (carrier_var == 65535 || !(doppler_var_factor > 0.0f)) return -1.0;
    // sigma^2 [mHz^2] = CarrierVar [mcycle^2] * DopplerVarFactor
    return std::sqrt(static_cast<double>(carrier_var) * doppler_var_factor) * 1.0e-3 * lambda_m;
}

}  // namespace mosaic
