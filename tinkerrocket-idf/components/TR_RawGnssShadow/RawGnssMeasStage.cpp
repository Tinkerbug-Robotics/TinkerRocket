// RawGnssMeasStage.cpp — see RawGnssMeasStage.h
#include "RawGnssMeasStage.h"

#include "RtcmEphemeris.h"
#include "gnss/atmo.h"
#include "gnss/types.h"

namespace gnss_raw
{

namespace
{

int16_t sat16s(int v)
{
    return static_cast<int16_t>(v < -32767 ? -32767 : (v > 32767 ? 32767 : v));
}

}  // namespace

RawGnssMeasStage::RawGnssMeasStage() : RawGnssMeasStage(Config()) {}

RawGnssMeasStage::RawGnssMeasStage(const Config& cfg) : cfg_(cfg), builder_(cfg.meas) {}

bool RawGnssMeasStage::onEphemeris(const uint8_t* payload, size_t len)
{
    gps_eph_t e;
    if (!decodeEphemeris(payload, len, week_ != 0 ? week_ : cfg_.default_week, e))
    {
        st_.eph_bad++;
        return false;
    }
    if (store_.add(e))
    {
        st_.eph_added++;
        st_.eph_sats = sat16s(store_.satellites(GNSS_SYS_GPS) + store_.satellites(GNSS_SYS_GAL)
                              + store_.satellites(GNSS_SYS_BDS));
    }
    return true;
}

void RawGnssMeasStage::setIono(const ShadowIono& io)
{
    iono_in_ = io;
    have_iono_ = true;
    iono_dirty_ = true;
}

bool RawGnssMeasStage::ionoAgeDays(int& days) const
{
    if (!have_iono_ || week_ == 0 || last_tow_s_ < 0.0)
        return false;
    const int today = static_cast<int>(week_) * 7 + static_cast<int>(last_tow_s_ / 86400.0);
    days = today - static_cast<int>(iono_in_.gps_day);
    return true;
}

void RawGnssMeasStage::see(const TimedRawEpoch& e)
{
    st_.epochs++;
    if (e.gps_week != 0)
        week_ = e.gps_week;
    last_tow_s_ = e.epoch.tow_s;
    st_.last_timed = e.mcu_us != 0 ? 1 : 0;
    // which satellites the receiver tracks that it has not sent the ephemeris of
    int tracked = 0, no_eph = 0;
    for (int i = 0; i < e.epoch.n; ++i)
    {
        const RawObs& o = e.epoch.obs[i];
        if (!o.has_pr || o.sys > GNSS_SYS_BDS)
            continue;
        tracked++;
        const double to_sys = o.sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0;   // as the builder picks
        if (store_.select(o.sys, o.prn, e.epoch.tow_s + to_sys) == nullptr)
            no_eph++;
    }
    st_.tracked = sat16s(tracked);
    st_.tracked_no_eph = sat16s(no_eph);
    int age = 0;
    st_.iono_age_known = ionoAgeDays(age) ? 1 : 0;
    st_.iono_age_days = sat16s(age);
}

void RawGnssMeasStage::chooseIono(bool in_flight, BuiltEpoch& out)
{
    out.iono_changed = 0;
    // made on the pad and kept through the flight: a change there would
    // start the survey over with no pad left to survey
    if (!in_flight)
    {
        int age = 0;
        const bool use = have_iono_ && (!ionoAgeDays(age) || age <= cfg_.iono_max_age_days);
        if (iono_dirty_ || use != use_broadcast_)
        {
            builder_.setIono(use ? RawMeasBuilder::ionoFromBroadcast(iono_in_.raw) : gps_iono_t{});
            use_broadcast_ = use;
            iono_dirty_ = false;
            out.iono_changed = 1;         // the pad survey's residuals were against the other model
        }
    }
    out.use_broadcast = use_broadcast_ ? 1 : 0;
}

void RawGnssMeasStage::begin(const TimedRawEpoch& e, const double* r_approx, bool in_flight, BuiltEpoch& out)
{
    e_ = &e;
    chooseIono(in_flight, out);
    // No stress on the Doppler here: the filter adds it from the IMU's own
    // largest specific force since the last epoch (RawGnssShadow's checks).
    builder_.begin(e.epoch, r_approx, 0.0);
    out.mcu_us = e.mcu_us;
    out.tow_s = e.epoch.tow_s;
    out.gps_week = e.gps_week;
    out.n = 0;
    for (int k = 0; k < 3; ++k)
        out.r_approx[k] = r_approx != nullptr ? r_approx[k] : 0.0;
    out.build_us = 0;
}

bool RawGnssMeasStage::next(BuiltEpoch& out, int max_obs)
{
    if (e_ == nullptr)
        return true;
    const int n0 = out.n;
    out.n = builder_.next(e_->epoch, store_, out.meas, out.n, RawEpoch::kMaxObs, max_obs);
    for (int i = n0; i < out.n; ++i)
        out.lock_age_s[i] = builder_.relockAge(out.meas[i].sys, out.meas[i].prn, out.tow_s);
    if (!builder_.done(e_->epoch))
        return false;
    e_ = nullptr;
    return true;
}

void RawGnssMeasStage::build(const TimedRawEpoch& e, const double* r_approx, bool in_flight, BuiltEpoch& out)
{
    const int64_t c0 = now();
    begin(e, r_approx, in_flight, out);
    while (!next(out, RawEpoch::kMaxObs))
    {
    }
    out.build_us = static_cast<uint32_t>(now() - c0);
    st_.build_us = out.build_us;
    if (out.build_us > st_.build_us_max)
        st_.build_us_max = out.build_us;
}

bool RawGnssMeasStage::offer(const TimedRawEpoch& e, const ShadowNav& nav, BuiltEpoch& out)
{
    see(e);
    // an epoch the filter will not take still tells which satellites are tracked
    if (e.mcu_us == 0)
    {
        st_.untimed++;
        builder_.noteEpoch(e.epoch);
        return false;
    }
    if (!nav.running)
    {
        st_.idle++;
        builder_.noteEpoch(e.epoch);
        return false;
    }
    double r[3];
    geo_to_ecef(nav.lla[0], nav.lla[1], nav.lla[2], r);
    build(e, r, nav.in_flight != 0, out);
    st_.built++;
    return true;
}

void RawGnssMeasStage::skip(const TimedRawEpoch& e)
{
    see(e);
    st_.dropped++;
    builder_.noteEpoch(e.epoch);
}

}  // namespace gnss_raw
