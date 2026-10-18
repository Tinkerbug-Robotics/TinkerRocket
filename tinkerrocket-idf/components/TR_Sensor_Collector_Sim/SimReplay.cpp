// SimReplay.cpp — see SimReplay.h
#include "SimReplay.h"

#include <cmath>
#include <cstring>

namespace sim_replay {

namespace {

constexpr uint8_t kMagic[4] = {'T', 'R', 'R', 'P'};
constexpr uint16_t kVersion = 1;
constexpr uint8_t kPhasePad = 0;
constexpr uint8_t kPhaseLanded = 6;

template <typename T>
T rd(const uint8_t* p, size_t off)
{
    T v;
    std::memcpy(&v, p + off, sizeof(T));
    return v;
}

bool positive(double v) { return std::isfinite(v) && v > 0.0; }

double posMod(double a, double m)
{
    double r = std::fmod(a, m);
    if (r < 0.0) r += m;
    return r;
}

}  // namespace

bool SimReplay::load(const uint8_t* blob, size_t len)
{
    blob_ = nullptr;
    h_ = Header();
    if (blob == nullptr || len < kHeaderBytes)
    {
        why_ = "shorter than a header";
        return false;
    }
    if (std::memcmp(blob, kMagic, 4) != 0)
    {
        why_ = "not a replay (magic)";
        return false;
    }
    Header h;
    h.version = rd<uint16_t>(blob, 4);
    const uint16_t header_bytes = rd<uint16_t>(blob, 6);
    const uint16_t sample_bytes = rd<uint16_t>(blob, 8);
    h.rate_hz = rd<uint16_t>(blob, 10);
    h.samples = rd<uint32_t>(blob, 12);
    h.t0_s = rd<double>(blob, 16);
    h.ignition_gps_s = rd<double>(blob, 24);
    h.lat_deg = rd<double>(blob, 32);
    h.lon_deg = rd<double>(blob, 40);
    h.alt_m = rd<double>(blob, 48);
    h.scen_start_s = rd<double>(blob, 56);
    h.scen_end_s = rd<double>(blob, 64);
    h.acc_lsb = rd<float>(blob, 72);
    h.gyro_lsb = rd<float>(blob, 76);
    h.p_ref_pa = rd<float>(blob, 80);
    h.p_lsb_pa = rd<float>(blob, 84);
    h.pad_loop_s = rd<float>(blob, 88);
    h.land_loop_s = rd<float>(blob, 92);
    h.decimal_year = rd<float>(blob, 96);
    std::memcpy(h.name, blob + 100, sizeof(h.name));
    h.name[sizeof(h.name) - 1] = '\0';

    if (h.version != kVersion || header_bytes != kHeaderBytes || sample_bytes != kSampleBytes)
    {
        why_ = "another version of the format";
        return false;
    }
    if (h.rate_hz == 0 || h.samples < 4 || len < kHeaderBytes + static_cast<size_t>(h.samples) * kSampleBytes)
    {
        why_ = "samples missing";
        return false;
    }
    if (!positive(h.acc_lsb) || !positive(h.gyro_lsb) || !positive(h.p_lsb_pa) || !positive(h.p_ref_pa)
        || !positive(h.ignition_gps_s) || !std::isfinite(h.t0_s) || !(h.scen_end_s > h.scen_start_s)
        || !std::isfinite(h.lat_deg) || !std::isfinite(h.lon_deg) || !std::isfinite(h.alt_m))
    {
        why_ = "a header value out of range";
        return false;
    }
    const double span = (h.samples - 1) / static_cast<double>(h.rate_hz);
    if (!positive(h.pad_loop_s) || !positive(h.land_loop_s) || h.pad_loop_s + h.land_loop_s > span)
    {
        why_ = "its pad and landed loops do not fit";
        return false;
    }
    // the loops must be the pad and the ground, or the replay would jump
    auto phaseOf = [&](uint32_t i) { return blob[kHeaderBytes + static_cast<size_t>(i) * kSampleBytes + 22]; };
    const uint32_t pad_n = static_cast<uint32_t>(std::ceil(h.pad_loop_s * h.rate_hz)) + 1;
    const uint32_t land_n = static_cast<uint32_t>(std::ceil(h.land_loop_s * h.rate_hz)) + 1;
    for (uint32_t i = 0; i < pad_n && i < h.samples; ++i)
    {
        if (phaseOf(i) != kPhasePad)
        {
            why_ = "its pad loop is not all pad";
            return false;
        }
    }
    for (uint32_t i = h.samples > land_n ? h.samples - land_n : 0; i < h.samples; ++i)
    {
        if (phaseOf(i) != kPhaseLanded)
        {
            why_ = "its landed loop is not all landed";
            return false;
        }
    }
    blob_ = blob;
    h_ = h;
    why_ = "";
    return true;
}

double SimReplay::tLast() const
{
    return h_.t0_s + (h_.samples - 1) / static_cast<double>(h_.rate_hz);
}

void SimReplay::raw(uint32_t i, Sample& out) const
{
    const uint8_t* p = blob_ + kHeaderBytes + static_cast<size_t>(i) * kSampleBytes;
    for (int k = 0; k < 3; ++k)
    {
        out.f[k] = rd<int16_t>(p, 2 * k) * h_.acc_lsb;
        out.w[k] = rd<int16_t>(p, 6 + 2 * k) * h_.gyro_lsb;
    }
    for (int k = 0; k < 4; ++k)
        out.q[k] = rd<int16_t>(p, 12 + 2 * k) * (1.0f / 32767.0f);
    out.p_pa = h_.p_ref_pa + rd<int16_t>(p, 20) * h_.p_lsb_pa;
    out.phase = p[22];
    out.high_g = (p[23] & 0x01) != 0;
}

void SimReplay::interp(double idx, Sample& out) const
{
    const double last = h_.samples - 1;
    if (!(idx > 0.0))
        idx = 0.0;
    if (idx > last)
        idx = last;
    uint32_t i = static_cast<uint32_t>(idx);
    if (i >= h_.samples - 1)
        i = h_.samples - 2;
    const float u = static_cast<float>(idx - i);
    Sample a, b;
    raw(i, a);
    raw(i + 1, b);
    for (int k = 0; k < 3; ++k)
    {
        out.f[k] = a.f[k] + u * (b.f[k] - a.f[k]);
        out.w[k] = a.w[k] + u * (b.w[k] - a.w[k]);
    }
    float dot = 0.0f;
    for (int k = 0; k < 4; ++k)
        dot += a.q[k] * b.q[k];
    const float sb = dot < 0.0f ? -1.0f : 1.0f;
    float n2 = 0.0f;
    for (int k = 0; k < 4; ++k)
    {
        out.q[k] = a.q[k] + u * (sb * b.q[k] - a.q[k]);
        n2 += out.q[k] * out.q[k];
    }
    const float inv = n2 > 0.0f ? 1.0f / std::sqrt(n2) : 0.0f;
    for (int k = 0; k < 4; ++k)
        out.q[k] *= inv;
    out.p_pa = a.p_pa + u * (b.p_pa - a.p_pa);
    const Sample& near = u < 0.5f ? a : b;
    out.phase = near.phase;
    out.high_g = near.high_g;
}

void SimReplay::at(double t_s, Sample& out) const
{
    const double rate = h_.rate_hz;
    if (t_s < h_.t0_s)
    {
        interp(posMod(t_s - h_.t0_s, h_.pad_loop_s) * rate, out);
        return;
    }
    const double t_last = tLast();
    if (t_s > t_last)
    {
        const double tau = posMod(t_s - t_last, h_.land_loop_s);
        interp((t_last - h_.land_loop_s + tau - h_.t0_s) * rate, out);
        return;
    }
    interp((t_s - h_.t0_s) * rate, out);
}

void SimReplay::padAt(double any_s, Sample& out) const
{
    interp(posMod(any_s, h_.pad_loop_s) * h_.rate_hz, out);
}

void ReplayClock::reset(const Header& h)
{
    ignition_gps_s_ = h.ignition_gps_s;
    lo_s_ = h.scen_start_s - cfg_.margin_s;
    hi_s_ = h.scen_end_s + cfg_.margin_s;
    base_s_ = 0.0;
    state_ = kWaiting;
    fixes_ = outside_ = restarts_ = 0;
    last_fix_t_ = 0.0;
    freeze_t_ = 0.0;
    secs_[0] = secs_[1] = Second{};
    sec_next_ = 0;
    check_ = kNotChecked;
    check_t_ = 0.0;
    check_res_us_ = 0;
    check_move_us_ = 0.0;
    tag_checks_ = 0;
}

void ReplayClock::onFix(double gps_s, int64_t mcu_us)
{
    if (!(gps_s > 0.0) || mcu_us == 0)
        return;
    const double t = gps_s - ignition_gps_s_;
    if (t < lo_s_ || t > hi_s_)
    {
        outside_++;
        return;
    }
    fixes_++;
    last_fix_t_ = t;
    const double base = t - static_cast<double>(mcu_us) * 1.0e-6;
    switch (state_)
    {
    case kWaiting:
        if (t < cfg_.arm_by_s)
        {
            base_s_ = base;
            state_ = kFollowing;
        }
        else
        {
            state_ = kMissed;
        }
        break;
    case kFollowing:
        if (std::fabs(base - base_s_) > cfg_.restart_s)
            restarts_++;
        base_s_ = base;
        if (t >= cfg_.arm_by_s)
        {
            state_ = kFrozen;
            freeze_t_ = t;
        }
        break;
    case kFrozen:
        noteCheckDeadline(t);
        break;
    default:
        break;
    }
}

void ReplayClock::noteCheckDeadline(double t)
{
    if (check_ == kNotChecked && t > cfg_.check_by_s)
        check_ = kUnchecked;
}

void ReplayClock::onEpoch(uint16_t gps_week, double tow_s, int64_t mcu_us, double dtr_s)
{
    if (gps_week == 0 || mcu_us == 0 || !(tow_s >= 0.0 && tow_s < 604800.0) || !std::isfinite(dtr_s))
        return;
    const double gps_s = gps_week * 604800.0 + tow_s - dtr_s;
    const uint32_t counted = fixes_;
    onFix(gps_s, mcu_us);
    // A whole second the clock counted: kept for its 1PPS check (onTagCheck),
    // which comes with the receiver's PVT of the same second
    const double whole = std::round(tow_s);
    if (fixes_ != counted && std::fabs(tow_s - whole) < 1.0e-4)
    {
        Second& s = secs_[sec_next_];
        s.sec = static_cast<int64_t>(gps_week) * 604800 + static_cast<int64_t>(whole);
        s.t = gps_s - ignition_gps_s_;
        s.base = s.t - static_cast<double>(mcu_us) * 1.0e-6;
        sec_next_ = static_cast<uint8_t>((sec_next_ + 1) % 2);
    }
}

void ReplayClock::onTagCheck(double sec_gps_s, int32_t residual_us)
{
    const int64_t sec = static_cast<int64_t>(std::llround(sec_gps_s));
    const Second* s = nullptr;
    for (const Second& c : secs_)
        if (c.sec == sec)
            s = &c;
    if (s == nullptr)
        return;   // not a second the clock counted (another time, or the epoch not drained yet)
    tag_checks_++;
    if (state_ != kFrozen)
        return;
    noteCheckDeadline(s->t);
    // the freeze's own second on: a check of one before it says nothing of a
    // sky that slipped between the two
    if (check_ != kNotChecked || s->t < freeze_t_ - 1.0e-3)
        return;
    // the tag sat residual_us late of the edge: the edge's MCU instant is the
    // tag's less it, so the sky's own clock at that second is
    const double edge_base = s->base + static_cast<double>(residual_us) * 1.0e-6;
    check_t_ = s->t;
    check_res_us_ = residual_us;
    check_move_us_ = (edge_base - base_s_) * 1.0e6;
    if (std::fabs(check_move_us_) > cfg_.check_bar_us)
    {
        base_s_ = edge_base;
        check_ = kMoved;
    }
    else
    {
        check_ = kKept;
    }
}

bool ReplayClock::tAt(int64_t mcu_us, double& t_s) const
{
    if (state_ != kFollowing && state_ != kFrozen)
        return false;
    t_s = base_s_ + static_cast<double>(mcu_us) * 1.0e-6;
    return true;
}

}  // namespace sim_replay
