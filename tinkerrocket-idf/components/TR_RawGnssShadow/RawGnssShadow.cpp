// RawGnssShadow.cpp — see RawGnssShadow.h
#include "RawGnssShadow.h"

#include <cmath>
#include <cstring>

#include "RtcmEphemeris.h"
#include "gnss/types.h"

namespace gnss_raw
{

namespace
{

constexpr double kRad2Deg = 57.29577951308232;

uint16_t sat16(double v)
{
    if (!(v > 0.0))
        return 0;
    return v >= 65535.0 ? 65535 : static_cast<uint16_t>(std::lround(v));
}

int32_t sat32(double v)
{
    if (!std::isfinite(v))
        return 0;
    if (v >= 2147483647.0)
        return 2147483647;
    if (v <= -2147483647.0)
        return -2147483647;
    return static_cast<int32_t>(std::llround(v));
}

}  // namespace

RawGnssShadow::RawGnssShadow() : RawGnssShadow(Config()) {}

RawGnssShadow::RawGnssShadow(const Config& cfg)
    : cfg_(cfg), ekf_(cfg.ekf), builder_(cfg.meas)
{
}

void RawGnssShadow::reset()
{
    if (running_)
        stop(kStopNone);
    npending_ = 0;
    have_seed_ = false;
}

void RawGnssShadow::onSeed(const ShadowSeed& s)
{
    seed_ = s;
    have_seed_ = true;
}

void RawGnssShadow::onEphemeris(const uint8_t* payload, size_t len)
{
    gps_eph_t e;
    if (!decodeEphemeris(payload, len, week_ != 0 ? week_ : cfg_.default_week, e))
    {
        st_.eph_bad++;
        return;
    }
    if (store_.add(e))
    {
        st_.eph_added++;
        eph_sats_ = store_.satellites(GNSS_SYS_GPS) + store_.satellites(GNSS_SYS_GAL)
                  + store_.satellites(GNSS_SYS_BDS);
    }
}

void RawGnssShadow::onEpoch(const TimedRawEpoch& e)
{
    st_.epochs++;
    if (e.gps_week != 0)
        week_ = e.gps_week;
    last_timed_ = e.mcu_us != 0;
    // an epoch the filter will not take still tells which satellites are tracked
    if (e.mcu_us == 0)
    {
        st_.epochs_untimed++;
        builder_.noteEpoch(e.epoch);
        return;
    }
    if (!running_)
    {
        st_.epochs_idle++;
        builder_.noteEpoch(e.epoch);
        return;
    }
    if (npending_ == 2)
    {
        dropFront();
        st_.epochs_stale++;
        pend_flags_ |= RGE_STALE;
    }
    pending_[npending_++] = e;
}

void RawGnssShadow::dropFront()
{
    builder_.noteEpoch(pending_[0].epoch);
    pending_[0] = pending_[1];
    npending_--;
}

void RawGnssShadow::onImu(const ShadowImu& s, int64_t time64_us)
{
    st_.imu++;
    const double t = static_cast<double>(time64_us) * 1.0e-6;
    if (s.flags & kShadowSim)
    {
        if (running_)
            stop(kStopSim);
        return;
    }
    if (running_)
    {
        if (t <= t_tick_)
        {
            // a repeated stamp (the flight filter skips those too, #440) is
            // not a step; time running back by a second or more is a restart
            if (t_tick_ - t < 1.0)
            {
                st_.imu_repeats++;
                return;
            }
            stop(kStopGap);
        }
        else if (t - t_tick_ > cfg_.max_gap_s)
        {
            stop(kStopGap);
        }
    }
    if (!running_)
    {
        if (have_seed_ && seed_.on_pad)
        {
            const double ts = static_cast<double>(unwrap32(seed_.time_us, time64_us)) * 1.0e-6;
            if (std::fabs(t - ts) <= cfg_.seed_max_age_s)
                start(t);
        }
        return;
    }

    // Each tick's specific force and rate, weighted by its time, until
    // min_step_s has passed: the filter steps on their mean (the ticks are
    // themselves boxcar means of the raw IMU, so this is one longer boxcar).
    const double w = t - t_tick_;
    for (int i = 0; i < 3; ++i)
    {
        acc_sum_[i] += w * static_cast<double>(s.acc[i]);
        gyro_sum_[i] += w * static_cast<double>(s.gyro[i]);
    }
    t_tick_ = t;
    if (t - t_prev_ < cfg_.min_step_s - 1.0e-6)   // a microsecond's slack: 4 x 5 ms is 0.02 s
        return;
    const double span = t - t_prev_;
    float acc[3], gyro[3];
    for (int i = 0; i < 3; ++i)
    {
        acc[i] = static_cast<float>(acc_sum_[i] / span);
        gyro[i] = static_cast<float>(gyro_sum_[i] / span);
        acc_sum_[i] = gyro_sum_[i] = 0.0;
    }

    const int64_t c0 = now();
    ekf_.propagate(acc, gyro, static_cast<float>(span));
    t_prev_ = t;
    if (s.flags & kShadowLevel)
        ekf_.updateAccelLevel(acc);
    const int64_t c1 = now();
    const uint32_t step_us = static_cast<uint32_t>(c1 - c0);
    st_.step_us_sum += step_us;
    st_.step_n++;
    if (step_us > st_.step_us_max)
        st_.step_us_max = step_us;
    prop_sum_ += step_us;
    prop_n_++;

    // the pad: the flight filter's attitude (and, before an epoch of our
    // own, its position and velocity), once a resync_period_s
    if (have_seed_ && seed_.on_pad && !(s.flags & kShadowInFlight)
        && t - last_resync_t_ >= cfg_.resync_period_s)
    {
        const double ts = static_cast<double>(unwrap32(seed_.time_us, time64_us)) * 1.0e-6;
        if (std::fabs(t - ts) <= cfg_.seed_max_age_s)
            resync(t);
    }

    applyDueEpochs(t);

    const double* lla = ekf_.lla();
    const float* v = ekf_.velNed();
    if (!std::isfinite(lla[0]) || !std::isfinite(lla[1]) || !std::isfinite(lla[2])
        || !std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2]))
    {
        stop(kStopNotFinite);
        return;
    }
    if (have_seed_ && seed_.on_pad && !(s.flags & kShadowInFlight))
    {
        // a rough distance: the pad is never near a pole
        const double dn = (lla[0] - seed_.lla[0]) * 6.371e6;
        const double de = (lla[1] - seed_.lla[1]) * 6.371e6 * std::cos(seed_.lla[0]);
        const double dd = lla[2] - seed_.lla[2];
        if (dn * dn + de * de + dd * dd > cfg_.pad_reset_m * cfg_.pad_reset_m)
        {
            stop(kStopFar);
            return;
        }
    }

    if (t >= next_log_t_)
    {
        fillLog(s.time_us, s.flags);
        next_log_t_ += cfg_.log_period_s;
        if (next_log_t_ <= t)
            next_log_t_ = t + cfg_.log_period_s;
    }
}

void RawGnssShadow::start(double t)
{
    const float v[3] = {seed_.vel_ned[0], seed_.vel_ned[1], seed_.vel_ned[2]};
    ekf_.init(seed_.lla, v, seed_.quat, t);
    ekf_.setQuaternion(seed_.quat);
    ekf_.seedGyroBias(seed_.gyro_bias);
    running_ = true;
    own_fix_ = false;
    t_prev_ = t;
    t_tick_ = t;
    for (int i = 0; i < 3; ++i)
        acc_sum_[i] = gyro_sum_[i] = 0.0;
    last_resync_t_ = t;
    next_log_t_ = t;
    npending_ = 0;
    prop_sum_ = 0;
    prop_n_ = 0;
    st_.starts++;
}

void RawGnssShadow::stop(uint8_t why)
{
    running_ = false;
    for (int i = 0; i < npending_; ++i)
        builder_.noteEpoch(pending_[i].epoch);
    npending_ = 0;
    log_ready_ = false;
    st_.stops++;
    st_.last_stop = why;
}

void RawGnssShadow::resync(double t)
{
    if (!own_fix_)
    {
        // nothing of its own yet: the whole state from the flight filter
        const float v[3] = {seed_.vel_ned[0], seed_.vel_ned[1], seed_.vel_ned[2]};
        ekf_.init(seed_.lla, v, seed_.quat, t);
    }
    ekf_.setQuaternion(seed_.quat);
    ekf_.seedGyroBias(seed_.gyro_bias);
    last_resync_t_ = t;
    pend_flags_ |= RGE_RESYNC;
    st_.resyncs++;
}

void RawGnssShadow::applyDueEpochs(double t)
{
    while (npending_ > 0)
    {
        const TimedRawEpoch& e = pending_[0];
        const double te = static_cast<double>(e.mcu_us) * 1.0e-6 - cfg_.epoch_latency_s;
        if (te > t)
            return;                       // the IMU has not reached it yet
        const double age = t - te;
        if (age > cfg_.max_epoch_age_s)
        {
            st_.epochs_stale++;
            pend_flags_ |= RGE_STALE;
            dropFront();
            continue;
        }
        apply(e, age);
        pending_[0] = pending_[1];
        npending_--;
    }
}

void RawGnssShadow::apply(const TimedRawEpoch& e, double age)
{
    double r[3], v[3];
    ekf_.receiverEcef(r, v);
    const int64_t c0 = now();
    const int n = builder_.build(e.epoch, store_, r, static_cast<double>(ekf_.takeGPeak()), meas_,
                                 RawEpoch::kMaxObs);
    const int64_t c1 = now();
    const RawGnssStats rs = ekf_.updateGnssRaw(meas_, n, age);
    const int64_t c2 = now();
    st_.build_us = static_cast<uint32_t>(c1 - c0);
    st_.update_us = static_cast<uint32_t>(c2 - c1);
    if (st_.build_us > st_.build_us_max)
        st_.build_us_max = st_.build_us;
    if (st_.update_us > st_.update_us_max)
        st_.update_us_max = st_.update_us;
    st_.last = rs;
    st_.last_age_s = age;
    st_.last_meas = n;
    st_.last_tow_ms = static_cast<uint32_t>(std::llround(std::fmod(e.epoch.tow_s, 604800.0) * 1000.0));
    st_.epochs_applied++;
    if (rs.used_pr > 0)
        own_fix_ = true;
    if (rs.clock_jump_ms != 0 || rs.clock_step_m != 0.0 || rs.drift_step_mps != 0.0)
        pend_flags_ |= RGE_CLOCK_STEP;
}

void RawGnssShadow::fillLog(uint32_t time_us, uint8_t imu_flags)
{
    RawGnssEkfData& d = log_;
    std::memset(&d, 0, sizeof(d));
    const double* lla = ekf_.lla();
    const float* v = ekf_.velNed();
    const float* q = ekf_.quat();
    d.time_us = time_us;
    d.lat_e7 = sat32(lla[0] * kRad2Deg * 1.0e7);
    d.lon_e7 = sat32(lla[1] * kRad2Deg * 1.0e7);
    d.alt_mm = sat32(lla[2] * 1000.0);
    for (int i = 0; i < 3; ++i)
    {
        d.vel_ned[i] = v[i];
        d.pos_sigma_cm[i] = sat16(static_cast<double>(ekf_.sigma(RawGnssEKF<float>::kPos + i)) * 100.0);
        d.vel_sigma_mmps[i] = sat16(static_cast<double>(ekf_.sigma(RawGnssEKF<float>::kVel + i)) * 1000.0);
    }
    for (int i = 0; i < 4; ++i)
        d.quat[i] = q[i];
    d.clk_bias_m = static_cast<float>(ekf_.clockBias());
    d.clk_drift_mps = ekf_.clockDrift();
    d.isb_gal_m = ekf_.isb(GNSS_SYS_GAL);
    d.isb_bds_m = ekf_.isb(GNSS_SYS_BDS);
    d.epoch_tow_ms = st_.last_tow_ms;
    const double age_ms = st_.last_age_s * 1000.0;
    d.epoch_age_ms = static_cast<int16_t>(age_ms > 32767.0 ? 32767 : std::lround(age_ms));
    auto u8 = [](int x) { return static_cast<uint8_t>(x < 0 ? 0 : (x > 255 ? 255 : x)); };
    d.used_pr = u8(st_.last.used_pr);
    d.used_rr = u8(st_.last.used_rr);
    d.rejected_pr = u8(st_.last.rejected_pr);
    d.rejected_rr = u8(st_.last.rejected_rr);
    d.eph_sats = u8(eph_sats_);
    d.flags = static_cast<uint8_t>(RGE_RUNNING | (ekf_.clockReady() ? RGE_CLOCK : 0)
                                   | (last_timed_ ? RGE_TIMED : 0)
                                   | ((imu_flags & kShadowLevel) ? RGE_LEVEL : 0)
                                   | ((imu_flags & kShadowInFlight) ? RGE_IN_FLIGHT : 0) | pend_flags_);
    pend_flags_ = 0;
    d.build_us = sat16(st_.build_us);
    d.update_us = sat16(st_.update_us);
    d.prop_us = sat16(prop_n_ ? static_cast<double>(prop_sum_) / prop_n_ : 0.0);
    prop_sum_ = 0;
    prop_n_ = 0;
    log_ready_ = true;
}

bool RawGnssShadow::takeLog(RawGnssEkfData& out)
{
    if (!log_ready_)
        return false;
    out = log_;
    log_ready_ = false;
    return true;
}

}  // namespace gnss_raw
