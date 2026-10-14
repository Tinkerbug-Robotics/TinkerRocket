// RawGnssShadow.cpp — see RawGnssShadow.h
#include "RawGnssShadow.h"

#include <cmath>
#include <cstring>

#include "RtcmEphemeris.h"
#include "gnss/atmo.h"
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
    : cfg_(cfg), ekf_(cfg.ekf), builder_(cfg.meas), survey_(cfg.survey), satcheck_(cfg.sat_check),
      fixcheck_(cfg.fix_check), level_gate_(cfg.level)
{
}

void RawGnssShadow::reset()
{
    if (running_)
        stop(kStopNone);
    npending_ = 0;
    nfix_ = 0;
    have_seed_ = false;
}

void RawGnssShadow::onBaro(const ShadowBaro& b)
{
    st_.baro++;
    baro_ = b;
    have_baro_ = true;
}

void RawGnssShadow::onMag(const ShadowMag& m)
{
    st_.mags++;
    mag_ = m;
    have_mag_ = true;
}

void RawGnssShadow::applyMag(const float acc[3])
{
    if (!have_seed_ || !seed_.have_declination)
        return;                           // no north to set it against
    const float* m = mag_.mag;
    const double mm = std::sqrt(static_cast<double>(m[0]) * m[0] + static_cast<double>(m[1]) * m[1]
                                + static_cast<double>(m[2]) * m[2]);
    if (!std::isfinite(mm))
        return;
    float err = -1.0f;
    if (seed_.field_ut > 1.0f)
    {
        err = static_cast<float>(std::fabs(mm - seed_.field_ut));
        if (err > cfg_.mag_field_tol * seed_.field_ut)
        {
            st_.mag_suspect++;            // a hard iron that size: no heading in it (#1304)
            return;
        }
    }
    if (ekf_.updateMagHeading(acc, m, true, seed_.declination_rad, err))
        st_.mag_fused++;
}

void RawGnssShadow::navState(ShadowNav& out) const
{
    out = ShadowNav{};
    out.running = running_ ? 1 : 0;
    if (!running_)
        return;
    const double t = ekf_.time();
    out.time_us = static_cast<uint32_t>(static_cast<int64_t>(std::llround(t * 1.0e6)));
    for (int i = 0; i < 3; ++i)
    {
        out.lla[i] = ekf_.lla()[i];
        out.vel_ned[i] = ekf_.velNed()[i];
        out.accel_bias[i] = ekf_.accelBias()[i];
        out.gyro_bias[i] = ekf_.gyroBias()[i];
    }
    for (int i = 0; i < 4; ++i)
        out.quat[i] = ekf_.quat()[i];
    for (int i = 0; i < 15; ++i)
        out.var[i] = ekf_.P(i, i);
    out.gnss_aided = t - last_aided_t_ <= 2.0 ? 1 : 0;
    out.baro_aided = t - last_baro_fused_t_ <= 1.0 ? 1 : 0;
    out.epochs = st_.epochs_applied;
}

void RawGnssShadow::applyBaro(double t, uint8_t imu_flags)
{
    const double h_baro = static_cast<double>(baro_.height_m);
    if (!std::isfinite(h_baro))
        return;
    const bool in_flight = (imu_flags & kShadowInFlight) != 0;
    const double dt = last_baro_t_ >= 0.0 ? std::fmin(t - last_baro_t_, 1.0) : 0.0;
    last_baro_t_ = t;
    // On the pad, once the raw measurements lead: the offset from the
    // barometer's height to the filter's, a running mean for its first
    // baro_offset_tau_s, then a low-pass that slow. Frozen at launch.
    if (!baro_frozen_ && !in_flight && have_seed_ && seed_.on_pad && (own_fix_ || cfg_.primary)
        && dt >= 0.0)
    {
        const double d = ekf_.lla()[2] - h_baro;
        baro_learn_s_ += dt;
        const double span = std::fmin(baro_learn_s_, cfg_.baro_offset_tau_s);
        const double w = span > 0.0 ? std::fmin(1.0, dt / span) : 1.0;
        baro_offset_ += w * (d - baro_offset_);
        baro_seen_ = true;
    }
    // In flight, frozen: what was learned however short the pad was (a
    // sim's 5 s, a start just before launch), or with nothing learned -- a
    // restored flight -- this sample at the filter's own height. A flight
    // without the barometer has an unbounded vertical.
    if (in_flight && !baro_frozen_)
    {
        if (!baro_seen_)
            baro_offset_ = ekf_.lla()[2] - h_baro;
        baro_seen_ = true;
        baro_learn_s_ = std::fmax(baro_learn_s_, cfg_.baro_offset_min_s);
        baro_frozen_ = true;
        baro_launch_t_ = t;
    }
    if (baro_learn_s_ < cfg_.baro_offset_min_s)
        return;                           // on the pad: not learned long enough yet
    st_.baro_offset_m = baro_offset_;
    st_.baro_offset_frozen = baro_frozen_ ? 1 : 0;

    // Its variance: the barometer's own, and the static port's error at the
    // dynamic pressure the filter's speed and height give (#1579)
    const float* v = ekf_.velNed();
    const double spd2 = static_cast<double>(v[0]) * v[0] + static_cast<double>(v[1]) * v[1]
                        + static_cast<double>(v[2]) * v[2];
    const double rho = 1.225 * std::exp(-ekf_.lla()[2] / 8500.0);
    const double port_m = cfg_.baro_port_k * 0.5 * rho * spd2;
    // In flight, while that error is large, not at all: it is a bias, not
    // noise. On the rig replay (r3) the barometer read 1.4 times the term low
    // through boost and coast, 67 m at burnout, and fifty samples a second at
    // that sigma weigh as one a second at a seventh of it: the raw epochs'
    // vertical lost to it, the filter 21 m low at the top of the coast (6-8 m
    // without the barometer, r1). Judged on the vertical speed -- drifting
    // with the wind under a chute moves no air past the port -- and for no
    // longer than baro_hold_max_s after launch, so a wrong speed cannot keep
    // the barometer out for good.
    const double vd = static_cast<double>(v[2]);
    if (in_flight && cfg_.baro_port_k * 0.5 * rho * vd * vd > cfg_.baro_hold_port_m
        && t - baro_launch_t_ < cfg_.baro_hold_max_s)
    {
        st_.baro_held++;
        return;
    }
    const double var = cfg_.baro_sigma_m * cfg_.baro_sigma_m + port_m * port_m;
    ekf_.updateHeight(h_baro + baro_offset_, static_cast<float>(var));
    st_.baro_fused++;
    last_baro_fused_t_ = t;
}

void RawGnssShadow::onFix(const TimedFix& f)
{
    st_.fixes++;
    if (f.fix_mode < 3 || !running_)
        return;                           // what the flight filter would take, while there is an IMU history
    if (f.mcu_us == 0)
    {
        st_.fixes_untimed++;
        return;
    }
    if (nfix_ == kFixQ)
    {
        for (int i = 1; i < kFixQ; ++i)
            fixq_[i - 1] = fixq_[i];
        nfix_--;
        st_.fixes_dropped++;
    }
    fixq_[nfix_++] = f;
}

void RawGnssShadow::applyDueFixes(double t)
{
    while (nfix_ > 0)
    {
        const double tf = static_cast<double>(fixq_[0].mcu_us) * 1.0e-6;
        if (tf > t)
            return;                       // the IMU has not reached it yet
        judge(fixq_[0], tf);
        for (int i = 1; i < nfix_; ++i)
            fixq_[i - 1] = fixq_[i];
        nfix_--;
    }
}

void RawGnssShadow::judge(const TimedFix& f, double t_fix)
{
    double ta = 0.0;
    float dv[3], dp[3], dva[3], dpa[3], acc[3];
    const bool have = fixcheck_.anchorTime(ta) && ekf_.imuMotion(ta, t_fix, dv, dp);
    const bool have_acc = ekf_.imuMotion(t_fix - cfg_.acc_window_s, t_fix, dva, dpa);
    if (have_acc)
        for (int i = 0; i < 3; ++i)
            acc[i] = dva[i] / static_cast<float>(cfg_.acc_window_s);
    const FixCheck::Result r = fixcheck_.onFix(f, t_fix, have ? dv : nullptr, have ? dp : nullptr,
                                               have_acc ? acc : nullptr);
    GnssCheckData d = {};
    d.time_us = static_cast<uint32_t>(f.mcu_us);
    d.verdict = r.verdict;
    d.sats_jumped = static_cast<uint8_t>(st_.last_sats_jumped > 255 ? 255 : st_.last_sats_jumped);
    d.span_ms = sat16(static_cast<double>(r.span_s) * 1000.0);
    d.pos_err_m = r.pos_err_m;
    d.pos_tol_m = r.pos_tol_m;
    d.vel_err_mps = r.vel_err_mps;
    d.vel_tol_mps = r.vel_tol_mps;
    if (nchk_ == kFixQ)
    {
        for (int i = 1; i < kFixQ; ++i)
            chk_[i - 1] = chk_[i];
        nchk_--;
    }
    chk_[nchk_++] = d;
}

bool RawGnssShadow::takeCheckLog(GnssCheckData& out)
{
    if (nchk_ == 0)
        return false;
    out = chk_[0];
    for (int i = 1; i < nchk_; ++i)
        chk_[i - 1] = chk_[i];
    nchk_--;
    return true;
}

void RawGnssShadow::onSeed(const ShadowSeed& s)
{
    // the navigation filter: the flight computer started over on the pad
    if (cfg_.primary && running_ && s.on_pad && s.start_gen != start_gen_)
        stop(kStopNone);
    seed_ = s;
    have_seed_ = true;
}

void RawGnssShadow::onEphemeris(const uint8_t* payload, size_t len)
{
    eph_frames_.add(payload, len);           // as sent, for the pad context
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

void RawGnssShadow::setIono(const ShadowIono& io)
{
    iono_in_ = io;
    have_iono_ = true;
    iono_dirty_ = true;
    st_.iono_sets++;
}

bool RawGnssShadow::ionoAgeDays(int& days) const
{
    if (!have_iono_ || week_ == 0 || last_tow_s_ < 0.0)
        return false;
    const int today = static_cast<int>(week_) * 7 + static_cast<int>(last_tow_s_ / 86400.0);
    days = today - static_cast<int>(iono_in_.gps_day);
    return true;
}

void RawGnssShadow::padStatus(ShadowPadStatus& out) const
{
    auto u8 = [](int x) { return static_cast<uint8_t>(x < 0 ? 0 : (x > 255 ? 255 : x)); };
    const PadSurvey::Summary& sv = survey_.summary();
    out.running = running_ ? 1 : 0;
    out.tracked = u8(st_.tracked);
    out.no_eph = u8(st_.tracked_no_eph);
    out.settling = sv.settling;
    out.corrected = sv.corrected;
    out.iono_source = ionoSource();
    int age = 0;
    out.iono_age_known = ionoAgeDays(age) ? 1 : 0;
    out.iono_age_days = static_cast<int16_t>(age < -32767 ? -32767 : (age > 32767 ? 32767 : age));
    out.corr_rms_m = sv.rms_m;
    // the raw measurements put the pad far from the receiver's average
    out.pad_mismatch = (sv.corrected > 0 && !sv.pad_ok && sv.pad_offset_m > 0.0f) ? 1 : 0;
    out.pad_offset_m = sv.pad_offset_m;
    out.fix_suspect = fixcheck_.last().verdict == FixCheck::kFlagged ? 1 : 0;
    double v = 0.0;
    if (running_)
        for (int i = 0; i < 3; ++i)
        {
            const double s = static_cast<double>(ekf_.sigma(RawGnssEKF<float>::kPos + i));
            v += s * s;
        }
    out.pos_sigma_m = static_cast<float>(std::sqrt(v));
}

uint8_t RawGnssShadow::ionoSource() const
{
    if (use_broadcast_)
        return kIonoBroadcast;
    const PadSurvey::Summary& sv = survey_.summary();
    return (sv.corrected > 0 && sv.pad_ok) ? kIonoPad : kIonoNone;
}

void RawGnssShadow::chooseIono(bool in_flight)
{
    // made on the pad and kept through the flight: a change there would
    // start the survey over with no pad left to survey
    if (in_flight)
        return;
    int age = 0;
    const bool use = have_iono_ && (!ionoAgeDays(age) || age <= cfg_.iono_max_age_days);
    if (!iono_dirty_ && use == use_broadcast_)
        return;
    builder_.setIono(use ? RawMeasBuilder::ionoFromBroadcast(iono_in_.raw) : gps_iono_t{});
    use_broadcast_ = use;
    iono_dirty_ = false;
    survey_.reset();                  // its residuals were against the other model
}

void RawGnssShadow::onEpoch(const TimedRawEpoch& e)
{
    st_.epochs++;
    if (e.gps_week != 0)
        week_ = e.gps_week;
    last_tow_s_ = e.epoch.tow_s;
    last_timed_ = e.mcu_us != 0;
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
    st_.tracked = tracked;
    st_.tracked_no_eph = no_eph;
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
    // The launch edge: the pad context, once a flight (PAD_CONTEXT_MSG). Only
    // after a ground tick, so a flight restored in the air takes none.
    if (s.flags & kShadowInFlight)
    {
        if (pcx_armed_)
        {
            pcx_armed_ = false;
            capturePadContext(s.time_us);
        }
    }
    else
    {
        pcx_armed_ = true;
    }
    const double t = static_cast<double>(time64_us) * 1.0e-6;
    // A sensor sim's IMU is not the sky's: the shadow stops. The navigation
    // filter flies it -- the sim drops the receiver's raw output, so it has
    // the sim's IMU and barometer and nothing to disagree with them.
    if ((s.flags & kShadowSim) && !cfg_.primary)
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
            // The navigation filter bridges a hole with one long step (on the
            // next tick's specific force and rate, as GpsInsEKF's clamped
            // step does): stopped in flight it could not start again. The
            // shadow stops.
            if (cfg_.primary && t - t_tick_ <= cfg_.max_bridge_s)
                st_.gaps_bridged++;
            else
                stop(kStopGap);
        }
    }
    if (!running_)
    {
        // from the pad; the navigation filter also from a restored flight
        if (have_seed_ && (seed_.on_pad || (cfg_.primary && seed_.restore)))
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
    // Where the flight filter levels: on the ground at g's size (not on a
    // motor burning before the launch detector says so); in flight (under a
    // chute) only once steady and at g's size (LevelGate.h)
    levelled_ = false;
    const bool ground = !(s.flags & kShadowInFlight);
    level_gate_.onStep(t, acc);
    if (s.flags & kShadowLevel)
    {
        if (ground ? level_gate_.gSized(acc) : level_gate_.allow(t, acc))
        {
            ekf_.updateAccelLevel(acc);
            levelled_ = true;
            st_.levels++;
        }
        else
        {
            st_.levels_held++;
        }
    }
    // The barometer's latest sample
    if (have_baro_)
    {
        have_baro_ = false;
        applyBaro(t, s.flags);
    }
    // The navigation filter on the ground: the magnetometer as a heading
    if (have_mag_)
    {
        have_mag_ = false;
        // the accelerometer is its tilt reference: gravity only, g's size
        if (cfg_.primary && ground && level_gate_.gSized(acc))
            applyMag(acc);
    }
    // Zero velocity once landed, and the navigation filter's on the pad while
    // still: on the rig (r4) the receiver lost the sky 87 s before launch and,
    // with nothing to hold it, the filter's horizontal velocity drifted to
    // 1.5 m/s on the pad, 150 m before it left. Still is the levelling's g's
    // size (not a motor burning before the launch verdict) and little rotation
    // (not the rocket being carried to the rail).
    const bool landed = (s.flags & kShadowLanded) != 0;
    const bool still_on_pad = cfg_.primary && cfg_.pad_zupt && ground && !landed && level_gate_.gSized(acc)
                              && gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]
                                     < static_cast<float>(cfg_.still_rate_rps * cfg_.still_rate_rps);
    if ((landed || still_on_pad) && t >= next_zupt_t_)
    {
        // on a schedule, so the steps' own spacing averages out
        ekf_.updateZeroVelocity(static_cast<float>(cfg_.zupt_sigma_mps));
        next_zupt_t_ = t - next_zupt_t_ > cfg_.zupt_period_s ? t + cfg_.zupt_period_s
                                                            : next_zupt_t_ + cfg_.zupt_period_s;
        if (landed)
            st_.zupts++;
        else
            st_.pad_zupts++;
    }
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
    if (!cfg_.primary && have_seed_ && seed_.on_pad && !(s.flags & kShadowInFlight)
        && t - last_resync_t_ >= cfg_.resync_period_s)
    {
        const double ts = static_cast<double>(unwrap32(seed_.time_us, time64_us)) * 1.0e-6;
        if (std::fabs(t - ts) <= cfg_.seed_max_age_s)
            resync(t);
    }

    applyDueEpochs(t, s.flags);
    applyDueFixes(t);

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
    // The flight filter's attitude is taken as known. The navigation filter's
    // seed on the pad is only the accelerometer's tilt and a guess at the
    // heading: its covariance stays the start's (20 degrees, any heading)
    // for the levelling and the magnetometer to settle.
    restored_ = cfg_.primary && !seed_.on_pad && seed_.restore;
    start_gen_ = seed_.start_gen;
    if (!cfg_.primary || restored_)
        ekf_.setQuaternion(seed_.quat);
    ekf_.seedGyroBias(seed_.gyro_bias);
    if (restored_)
        st_.restores++;
    running_ = true;
    own_fix_ = false;
    t_prev_ = t;
    t_tick_ = t;
    for (int i = 0; i < 3; ++i)
        acc_sum_[i] = gyro_sum_[i] = 0.0;
    last_resync_t_ = t;
    next_log_t_ = t;
    npending_ = 0;
    nfix_ = 0;
    satcheck_.reset();                    // a new filter: nothing to take differences from
    fixcheck_.reset();
    level_gate_.reset();
    levelled_ = false;
    have_baro_ = false;
    baro_offset_ = 0.0;
    baro_learn_s_ = 0.0;
    baro_frozen_ = false;
    baro_launch_t_ = 0.0;
    baro_seen_ = false;
    last_baro_t_ = -1.0;
    next_zupt_t_ = 0.0;
    have_mag_ = false;
    last_aided_t_ = -1.0e9;
    last_baro_fused_t_ = -1.0e9;
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
    att_log_ready_ = false;
    st_.stops++;
    st_.last_stop = why;
}

void RawGnssShadow::resync(double t)
{
    if (!own_fix_)
    {
        // nothing of its own yet: the whole state from the flight filter, and
        // nothing for the satellite check to take a difference across
        const float v[3] = {seed_.vel_ned[0], seed_.vel_ned[1], seed_.vel_ned[2]};
        ekf_.init(seed_.lla, v, seed_.quat, t);
        satcheck_.reset();
        level_gate_.reset();
    }
    ekf_.setQuaternion(seed_.quat);
    ekf_.seedGyroBias(seed_.gyro_bias);
    last_resync_t_ = t;
    pend_flags_ |= RGE_RESYNC;
    st_.resyncs++;
}

void RawGnssShadow::applyDueEpochs(double t, uint8_t imu_flags)
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
        apply(e, age, t, imu_flags);
        pending_[0] = pending_[1];
        npending_--;
    }
}

void RawGnssShadow::apply(const TimedRawEpoch& e, double age, double t, uint8_t imu_flags)
{
    const bool in_flight = (imu_flags & kShadowInFlight) != 0;
    chooseIono(in_flight);
    double r[3], v[3];
    ekf_.receiverEcef(r, v);
    const int64_t c0 = now();
    const int n = builder_.build(e.epoch, store_, r, static_cast<double>(ekf_.takeGPeak()), meas_,
                                 RawEpoch::kMaxObs);
    // each satellite against the IMU since the last epoch (#1508): one that
    // jumped is taken as relocked, its sigma the builder's relock sigma
    double r_prior[3], v_prior[3];
    ekf_.receiverEcefAt(age, r_prior, v_prior);
    const int jumped = satcheck_.check(meas_, n, e.epoch.tow_s, r_prior, jumped_);
    for (int i = 0; i < n && jumped > 0; ++i)
    {
        if (!jumped_[i])
            continue;
        builder_.markRelock(meas_[i].sys, meas_[i].prn, e.epoch.tow_s);
        meas_[i].sigma_pr_corr = static_cast<float>(
            std::hypot(static_cast<double>(meas_[i].sigma_pr_corr), cfg_.meas.relock_sigma_m));
    }
    st_.last_sats_jumped = jumped;
    st_.sats_jumped += static_cast<uint32_t>(jumped);
    // the pad survey, against the launch site the seed carries (a fresh one)
    if (!in_flight && have_seed_ && seed_.on_pad && seed_.pad_valid)
    {
        const double ts = static_cast<double>(unwrap32(seed_.time_us, static_cast<int64_t>(t * 1.0e6))) * 1.0e-6;
        if (std::fabs(t - ts) <= cfg_.seed_max_age_s)
        {
            double pad[3];
            geo_to_ecef(seed_.pad_lla[0], seed_.pad_lla[1], seed_.pad_lla[2], pad);
            for (int i = 0; i < n; ++i)
                lock_age_[i] = builder_.relockAge(meas_[i].sys, meas_[i].prn, e.epoch.tow_s);
            survey_.observe(meas_, n, lock_age_, e.epoch.tow_s, pad);
            st_.survey_epochs++;
        }
    }
    st_.last_pad_corrected = use_broadcast_ ? 0 : survey_.correct(meas_, n, r);
    const int64_t c1 = now();
    const RawGnssStats rs = ekf_.updateGnssRaw(meas_, n, age);
    double r_post[3], v_post[3];
    ekf_.receiverEcefAt(age, r_post, v_post);
    satcheck_.commit(meas_, n, e.epoch.tow_s, r_post);
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
    level_gate_.onEpoch(t, ekf_.velNed());
    if (rs.used_pr > 0)
    {
        own_fix_ = true;
        last_aided_t_ = t;
    }
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
                                   | (levelled_ ? RGE_LEVEL : 0)
                                   | ((imu_flags & kShadowInFlight) ? RGE_IN_FLIGHT : 0) | pend_flags_);
    pend_flags_ = 0;
    d.build_us = sat16(st_.build_us);
    d.update_us = sat16(st_.update_us);
    d.prop_us = sat16(prop_n_ ? static_cast<double>(prop_sum_) / prop_n_ : 0.0);
    prop_sum_ = 0;
    prop_n_ = 0;
    log_ready_ = true;

    // its companion: the attitude's 1-sigma in NED and the IMU's biases
    RawGnssAttData& a = att_log_;
    std::memset(&a, 0, sizeof(a));
    a.time_us = time_us;
    float att_sig[3];
    ekf_.attSigmaNed(att_sig);
    const float* ab = ekf_.accelBias();
    const float* wb = ekf_.gyroBias();
    for (int i = 0; i < 3; ++i)
    {
        a.att_sigma_cdeg[i] = sat16(static_cast<double>(att_sig[i]) * kRad2Deg * 100.0);
        a.acc_bias_mps2[i] = ab[i];
        a.gyro_bias_dps[i] = static_cast<float>(static_cast<double>(wb[i]) * kRad2Deg);
        a.acc_bias_sigma_mmps2[i] =
            sat16(static_cast<double>(ekf_.sigma(RawGnssEKF<float>::kAccBias + i)) * 1000.0);
        a.gyro_bias_sigma_mdps[i] =
            sat16(static_cast<double>(ekf_.sigma(RawGnssEKF<float>::kGyroBias + i)) * kRad2Deg * 1000.0);
    }
    att_log_ready_ = true;
}

bool RawGnssShadow::takeLog(RawGnssEkfData& out)
{
    if (!log_ready_)
        return false;
    out = log_;
    log_ready_ = false;
    return true;
}

bool RawGnssShadow::takeAttLog(RawGnssAttData& out)
{
    if (!att_log_ready_)
        return false;
    out = att_log_;
    att_log_ready_ = false;
    return true;
}

static_assert(RawGnssEKF<float>::N == PCX_COV_N, "PCX_COV carries the filter's whole covariance");
static_assert(EphemerisFrames::kMaxPayload + EphemerisFrames::kFrameOverhead <= PCX_EPH_MAX_FRAME,
              "a held ephemeris, framed, must fit one PCX_EPH record");

void RawGnssShadow::capturePadContext(uint32_t tick_us)
{
    PadContextNav& n = pcx_nav_;
    std::memset(&n, 0, sizeof(n));
    n.filter_time_us = tick_us;
    const double* lla = ekf_.lla();
    const float* v = ekf_.velNed();
    const float* q = ekf_.quat();
    const float* ab = ekf_.accelBias();
    const float* wb = ekf_.gyroBias();
    for (int i = 0; i < 3; ++i)
    {
        n.lla[i] = lla[i];
        n.vel_ned[i] = v[i];
        n.acc_bias_mps2[i] = ab[i];
        n.gyro_bias_rps[i] = wb[i];
    }
    for (int i = 0; i < 4; ++i)
        n.quat[i] = q[i];
    n.clk_bias_m = ekf_.clockBias();
    n.clk_drift_mps = ekf_.clockDrift();
    n.isb_gal_m = ekf_.isb(GNSS_SYS_GAL);
    n.isb_bds_m = ekf_.isb(GNSS_SYS_BDS);
    n.baro_offset_m = static_cast<float>(baro_offset_);
    const bool decl = have_seed_ && seed_.have_declination;
    if (decl)
    {
        n.declination_rad = seed_.declination_rad;
        n.field_ut = seed_.field_ut;
    }
    if (have_iono_)
    {
        std::memcpy(n.iono, iono_in_.raw, sizeof(n.iono));
        n.iono_gps_day = iono_in_.gps_day;
    }
    n.iono_source = ionoSource();
    n.flags = static_cast<uint8_t>((running_ ? PCN_RUNNING : 0) | (ekf_.clockReady() ? PCN_CLOCK : 0)
                                   | (baro_seen_ ? PCN_BARO : 0) | (have_iono_ ? PCN_IONO : 0)
                                   | (decl ? PCN_DECLINATION : 0) | (cfg_.primary ? PCN_PRIMARY : 0));
    // the covariance's upper triangle, row by row
    int k = 0;
    for (int i = 0; i < PCX_COV_N; ++i)
        for (int j = i; j < PCX_COV_N; ++j)
            pcx_cov_[k++] = ekf_.P(i, j);
    // the pad survey's corrections, GPS, Galileo, BeiDou
    pcx_nsurvey_ = 0;
    for (int sys = GNSS_SYS_GPS; sys <= GNSS_SYS_BDS; ++sys)
    {
        for (int prn = 1; prn < 64 && pcx_nsurvey_ < kPcxSurveyMax; ++prn)
        {
            double corr = 0.0;
            if (survey_.correction(sys, prn, corr))
                pcx_survey_[pcx_nsurvey_++] = PadSurveyEntry{static_cast<uint8_t>(sys), static_cast<uint8_t>(prn),
                                                             static_cast<float>(corr)};
        }
    }
    pcx_sum_ = survey_.summary();
    pcx_due_us_ = tick_us + static_cast<uint32_t>(cfg_.pad_context_delay_s * 1.0e6);
    pcx_stage_ = 0;
    pcx_eph_next_ = 0;
    pcx_neph_ = eph_frames_.count();
    pcx_pending_ = true;
    st_.pad_contexts++;
}

bool RawGnssShadow::takePadContext(uint8_t* out, size_t cap, size_t& len, uint32_t now_us)
{
    if (!pcx_pending_ || static_cast<int32_t>(now_us - pcx_due_us_) < 0 || out == nullptr)
        return false;
    constexpr int kCovParts = (PCX_COV_VALUES + PCX_COV_PER_PART - 1) / PCX_COV_PER_PART;
    const int survey_parts = pcx_nsurvey_ == 0 ? 1 : (pcx_nsurvey_ + PCX_SURVEY_PER_PART - 1) / PCX_SURVEY_PER_PART;
    PadContextHeader h = {now_us, 0, 0, 1};
    uint8_t* body = out + sizeof(h);
    size_t body_len = 0;
    if (pcx_stage_ == 0)
    {
        h.kind = PCX_NAV;
        body_len = sizeof(PadContextNav);
        if (sizeof(h) + body_len > cap)
            return false;
        std::memcpy(body, &pcx_nav_, body_len);
    }
    else if (pcx_stage_ <= kCovParts)
    {
        const int part = pcx_stage_ - 1;
        PadContextCov c = {};
        c.first = static_cast<uint16_t>(part * PCX_COV_PER_PART);
        const int left = PCX_COV_VALUES - c.first;
        c.n = static_cast<uint8_t>(left < PCX_COV_PER_PART ? left : PCX_COV_PER_PART);
        for (int i = 0; i < c.n; ++i)
            c.p[i] = pcx_cov_[c.first + i];
        h.kind = PCX_COV;
        h.part = static_cast<uint8_t>(part);
        h.parts = static_cast<uint8_t>(kCovParts);
        body_len = padContextCovWireSize(c.n);
        if (sizeof(h) + body_len > cap)
            return false;
        std::memcpy(body, &c, body_len);
    }
    else if (pcx_stage_ <= kCovParts + survey_parts)
    {
        const int part = pcx_stage_ - 1 - kCovParts;
        PadContextSurvey sv = {};
        sv.tracked = pcx_sum_.tracked;
        sv.settling = pcx_sum_.settling;
        sv.corrected = pcx_sum_.corrected;
        sv.pad_ok = pcx_sum_.pad_ok ? 1 : 0;
        sv.rms_m = pcx_sum_.rms_m;
        sv.pad_offset_m = pcx_sum_.pad_offset_m;
        const int first = part * PCX_SURVEY_PER_PART;
        const int left = pcx_nsurvey_ - first;
        sv.n = static_cast<uint8_t>(left < 0 ? 0 : (left < PCX_SURVEY_PER_PART ? left : PCX_SURVEY_PER_PART));
        for (int i = 0; i < sv.n; ++i)
            sv.e[i] = pcx_survey_[first + i];
        h.kind = PCX_SURVEY;
        h.part = static_cast<uint8_t>(part);
        h.parts = static_cast<uint8_t>(survey_parts);
        body_len = padContextSurveyWireSize(sv.n);
        if (sizeof(h) + body_len > cap)
            return false;
        std::memcpy(body, &sv, body_len);
    }
    else
    {
        // then each ephemeris held at the launch tick, framed as the receiver
        // framed it (a satellite's newer one, since, in its place)
        if (pcx_eph_next_ >= pcx_neph_ || cap < sizeof(h))
        {
            pcx_pending_ = false;
            return false;
        }
        h.kind = PCX_EPH;
        h.part = static_cast<uint8_t>(pcx_eph_next_);
        h.parts = static_cast<uint8_t>(pcx_neph_);
        body_len = eph_frames_.frame(pcx_eph_next_, body, cap - sizeof(h));
        if (body_len == 0)
            return false;
        pcx_eph_next_++;
        std::memcpy(out, &h, sizeof(h));
        len = sizeof(h) + body_len;
        return true;
    }
    pcx_stage_++;
    std::memcpy(out, &h, sizeof(h));
    len = sizeof(h) + body_len;
    return true;
}

}  // namespace gnss_raw
