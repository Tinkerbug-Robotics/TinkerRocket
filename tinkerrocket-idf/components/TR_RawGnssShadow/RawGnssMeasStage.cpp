// RawGnssMeasStage.cpp — see RawGnssMeasStage.h
#include "RawGnssMeasStage.h"

#include <cmath>

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

bool RawGnssMeasStage::addEphemeris(const gps_eph_t& e)
{
    // The RTCM decode gives only what the builder can place, and a driver's
    // is held to the same: a GPS, Galileo or BeiDou MEO/IGSO satellite
    // (gnss/eph.h has no GEO orbit), PRN 1-63.
    bool ok = e.valid != 0 && e.sys >= GNSS_SYS_GPS && e.sys <= GNSS_SYS_BDS && e.prn >= 1 && e.prn < kPrns
              && !(e.sys == GNSS_SYS_BDS && (e.prn <= 5 || e.prn >= 59));
    // And, where the last epoch named its week, its toe within
    // kEphMaxOffsetS of the receiver's time: the store goes by second of week
    // alone, so one a whole number of weeks old (a receiver's stored copy)
    // would be taken as today's. (That epoch's own week, not the last one
    // named: past the week's turn the two differ.) BeiDou's toe is in BeiDou
    // time, 14 s behind GPS.
    if (ok && last_week_ != 0 && e.week > 0)
    {
        const double toe_gps = e.toe - (e.sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0);
        const double d = (e.week - static_cast<int>(last_week_)) * 604800.0 + (toe_gps - last_tow_s_);
        ok = std::fabs(d) <= kEphMaxOffsetS;
    }
    if (!ok)
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

bool RawGnssMeasStage::hasEphemeris(int sys, int prn, double tow_gps) const
{
    const double to_sys = sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0;   // as the builder picks
    return store_.select(sys, prn, tow_gps + to_sys) != nullptr;
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
    last_week_ = e.gps_week;
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
    if (cfg_.sat_cap > 0)
    {
        // every satellite's lock age from the whole sky, then the build from
        // the cap's picks (begin() notes them again at the same time: no gap)
        builder_.noteEpoch(e.epoch);
        capEpoch(e);
        build(sel_, r, nav.in_flight != 0, out);
        capBuilt(out);
    }
    else
    {
        build(e, r, nav.in_flight != 0, out);
    }
    st_.built++;
    return true;
}

// The cap's picks from e into sel_: of the satellites the builder would build
// (a pseudorange at or above its system's C/N0 floor, an ephemeris), up to
// kCapGps GPS ones first, then any, to sat_cap. Each by its C/N0, kCapHoldDb
// more if it was built the last epoch; a tie goes to the lower system, then
// the lower PRN, so the same sky always gives the same picks. The cap knows
// no elevations: a satellite its build dropped under the mask (capBuilt) is
// picked only after every other, and not among the GPS ones, until it is
// tried again. sel_ keeps e's order.
void RawGnssMeasStage::capEpoch(const TimedRawEpoch& e)
{
    static constexpr int8_t kNot = -1, kCand = 0, kPicked = 1, kMasked = 2;
    const RawEpoch& in = e.epoch;
    const int n = in.n < RawEpoch::kMaxObs ? in.n : RawEpoch::kMaxObs;
    // a satellite rises through the mask in minutes: those under it are
    // tried again, as any other, kCapMaskRetryS after the first of them
    if ((masked_[0] | masked_[1] | masked_[2]) != 0 && std::fabs(gps_time_diff(in.tow_s, masked_tow_)) >= kCapMaskRetryS)
        masked_[0] = masked_[1] = masked_[2] = 0;
    float score[RawEpoch::kMaxObs];
    int8_t state[RawEpoch::kMaxObs];
    int cand_sats = 0;                    // the candidates not under the mask
    for (int i = 0; i < n; ++i)
    {
        const RawObs& o = in.obs[i];
        state[i] = kNot;
        score[i] = 0.0f;
        if (!o.has_pr || o.sys > GNSS_SYS_BDS || o.prn >= kPrns || o.cn0_dbhz < cfg_.meas.sig[o.sys].cn0_min_pr
            || !hasEphemeris(o.sys, o.prn, in.tow_s))
            continue;
        bool first = true;                // a satellite's first observation counts it
        for (int j = 0; j < i; ++j)
            if (state[j] != kNot && in.obs[j].sys == o.sys && in.obs[j].prn == o.prn)
                first = false;
        const bool masked = ((masked_[o.sys] >> o.prn) & 1u) != 0;
        cand_sats += first && !masked ? 1 : 0;
        state[i] = masked ? kMasked : kCand;
        score[i] = o.cn0_dbhz + (((picked_[o.sys] >> o.prn) & 1u) != 0 ? kCapHoldDb : 0.0f);
    }
    auto better = [&](int a, int b) {
        if (score[a] != score[b])
            return score[a] > score[b];
        if (in.obs[a].sys != in.obs[b].sys)
            return in.obs[a].sys < in.obs[b].sys;
        return in.obs[a].prn < in.obs[b].prn;
    };
    uint64_t picked[3] = {};
    int sats = 0, kept = 0;               // picked; of them, not under the mask
    for (int pass = 0; pass < 3; ++pass)
    {
        // GPS first, then any, then those under the mask
        const int want = pass == 0 ? (cfg_.sat_cap < kCapGps ? cfg_.sat_cap : kCapGps) : cfg_.sat_cap;
        const int8_t from = pass < 2 ? kCand : kMasked;
        while (sats < want)
        {
            int best = -1;
            for (int i = 0; i < n; ++i)
                if (state[i] == from && (pass != 0 || in.obs[i].sys == GNSS_SYS_GPS) && (best < 0 || better(i, best)))
                    best = i;
            if (best < 0)
                break;
            const uint8_t sys = in.obs[best].sys, prn = in.obs[best].prn;
            for (int i = 0; i < n; ++i)   // the satellite, every observation of it
                if (state[i] == from && in.obs[i].sys == sys && in.obs[i].prn == prn)
                    state[i] = kPicked;
            picked[sys] |= uint64_t{1} << prn;
            sats++;
            kept += from == kCand ? 1 : 0;
        }
    }
    sel_.epoch.tow_s = in.tow_s;
    int k = 0;
    for (int i = 0; i < n; ++i)
        if (state[i] == kPicked)
            sel_.epoch.obs[k++] = in.obs[i];
    sel_.epoch.n = k;
    sel_.mcu_us = e.mcu_us;
    sel_.arrival_us = e.arrival_us;
    sel_.gps_week = e.gps_week;
    for (int s = 0; s < 3; ++s)
        picked_[s] = picked[s];
    st_.capped_out += static_cast<uint32_t>(cand_sats - kept);
}

// After the build of the cap's picks: the hold is for the satellites built,
// and a pick the build dropped -- under the elevation mask, the one test the
// cap cannot make -- is picked last from now on (capEpoch), so it does not
// keep a place a satellite that can be built is waiting for.
void RawGnssMeasStage::capBuilt(const BuiltEpoch& out)
{
    uint64_t built[3] = {};
    for (int i = 0; i < out.n; ++i)
        if (out.meas[i].sys <= GNSS_SYS_BDS && out.meas[i].prn < kPrns)
            built[out.meas[i].sys] |= uint64_t{1} << out.meas[i].prn;
    const bool none = (masked_[0] | masked_[1] | masked_[2]) == 0;
    for (int s = 0; s < 3; ++s)
    {
        masked_[s] = (masked_[s] | picked_[s]) & ~built[s];
        picked_[s] = built[s];
    }
    if (none && (masked_[0] | masked_[1] | masked_[2]) != 0)
        masked_tow_ = out.tow_s;
}

#ifdef TR_EPH_CAPACITY
bool RawGnssMeasStage::ephCapacityAgrees(size_t caller_capacity)
{
    return caller_capacity == EphemerisStore::kCapacity && EphemerisStore::builtCapacity() == EphemerisStore::kCapacity;
}
#endif

void RawGnssMeasStage::skip(const TimedRawEpoch& e)
{
    see(e);
    st_.dropped++;
    builder_.noteEpoch(e.epoch);
}

}  // namespace gnss_raw
