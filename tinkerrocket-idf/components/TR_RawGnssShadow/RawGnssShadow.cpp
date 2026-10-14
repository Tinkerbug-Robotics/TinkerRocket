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

int16_t sats16(double v)
{
    if (!std::isfinite(v))
        return 0;
    return v >= 32767.0 ? 32767 : (v <= -32767.0 ? -32767 : static_cast<int16_t>(std::lround(v)));
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

RawGnssMeasStage::Config RawGnssShadow::stageConfig(const Config& c)
{
    RawGnssMeasStage::Config s;
    s.meas = c.meas;
    s.default_week = c.default_week;
    s.iono_max_age_days = c.iono_max_age_days;
    return s;
}

RawGnssShadow::RawGnssShadow() : RawGnssShadow(Config()) {}

RawGnssShadow::RawGnssShadow(const Config& cfg)
    : cfg_(cfg), ekf_(cfg.ekf), stage_(stageConfig(cfg)), survey_(cfg.survey), satcheck_(cfg.sat_check),
      fixcheck_(cfg.fix_check), level_gate_(cfg.level), fit_(cfg.heading_fit)
{
}

void RawGnssShadow::reset()
{
    if (running_)
        stop(kStopNone);
    job_.active = false;
    npending_ = 0;
    npend_built_ = 0;
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
    out.in_flight = (imu_flags_ & kShadowInFlight) ? 1 : 0;
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

bool RawGnssShadow::takeHeadingLog(HeadingFitData& out)
{
    if (nhdg_ == 0)
        return false;
    out = hdg_[0];
    for (int i = 1; i < nhdg_; ++i)
        hdg_[i - 1] = hdg_[i];
    nhdg_--;
    return true;
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
    if (cfg_.external_build)
        return;                              // the GNSS task's stage decodes it
    (void)stage_.onEphemeris(payload, len);
    st_.eph_added = stage_.stats().eph_added;
    st_.eph_bad = stage_.stats().eph_bad;
}

void RawGnssShadow::setIono(const ShadowIono& io)
{
    iono_in_ = io;
    have_iono_ = true;
    st_.iono_sets++;
    if (!cfg_.external_build)
        stage_.setIono(io);
}

bool RawGnssShadow::ionoAgeDays(int& days) const
{
    if (!cfg_.external_build)
        return stage_.ionoAgeDays(days);
    days = meas_st_.iono_age_days;
    return meas_st_.iono_age_known != 0;
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

void RawGnssShadow::onEpoch(const TimedRawEpoch& e)
{
    if (cfg_.external_build)
        return;                           // the GNSS task's stage sees it (onBuilt)
    st_.epochs++;
    stage_.see(e);
    st_.tracked = stage_.stats().tracked;
    st_.tracked_no_eph = stage_.stats().tracked_no_eph;
    last_timed_ = e.mcu_us != 0;
    // an epoch the filter will not take still tells which satellites are tracked
    if (e.mcu_us == 0)
    {
        st_.epochs_untimed++;
        stage_.noteEpoch(e.epoch);
        return;
    }
    if (!running_)
    {
        st_.epochs_idle++;
        stage_.noteEpoch(e.epoch);
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

void RawGnssShadow::onBuilt(const BuiltEpoch& b)
{
    if (!cfg_.external_build)
        return;
    if (!running_)
    {
        st_.epochs_stale++;               // built for a filter that has since stopped
        return;
    }
    if (npend_built_ == 2)
    {
        dropFront();
        st_.epochs_stale++;
        pend_flags_ |= RGE_STALE;
    }
    pend_built_[(pb_head_ + npend_built_) & 1] = b;
    npend_built_++;
}

void RawGnssShadow::onMeasStats(const ShadowMeasStats& m)
{
    meas_st_ = m;
    st_.epochs = m.epochs;
    st_.epochs_untimed = m.untimed;
    st_.epochs_idle = m.idle;
    st_.tracked = m.tracked;
    st_.tracked_no_eph = m.tracked_no_eph;
    st_.eph_added = m.eph_added;
    st_.eph_bad = m.eph_bad;
    last_timed_ = m.last_timed != 0;
}

bool RawGnssShadow::takeRelock(ShadowRelock& out)
{
    if (nrelock_ == 0)
        return false;
    out = relock_[0];
    for (int i = 1; i < nrelock_; ++i)
        relock_[i - 1] = relock_[i];
    nrelock_--;
    return true;
}

void RawGnssShadow::relock(int sys, int prn, double tow)
{
    if (!cfg_.external_build)
    {
        stage_.markRelock(sys, prn, tow);
        return;
    }
    if (nrelock_ == kRelockQ)
    {
        for (int i = 1; i < kRelockQ; ++i)
            relock_[i - 1] = relock_[i];
        nrelock_--;
    }
    ShadowRelock& r = relock_[nrelock_++];
    r.sys = static_cast<uint8_t>(sys);
    r.prn = static_cast<uint8_t>(prn);
    r.tow_s = tow;
}

void RawGnssShadow::dropFront()
{
    if (cfg_.external_build)
    {
        pb_head_ ^= 1;                    // its satellites' lock ages were taken when it was built
        npend_built_--;
        return;
    }
    stage_.noteEpoch(pending_[0].epoch);
    pending_[0] = pending_[1];
    npending_--;
}

void RawGnssShadow::onImu(const ShadowImu& s, int64_t time64_us)
{
    st_.imu++;
    imu_flags_ = s.flags;
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

    stepHeading(t, acc, gyro, static_cast<float>(span), s.flags);

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
    fit_.reset();
    fit_ground_ = !restored_;
    fit_flight_ = false;
    handed_ = false;
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
    job_.active = false;
    npending_ = 0;
    npend_built_ = 0;
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
    job_.active = false;
    for (int i = 0; i < npending_; ++i)
        stage_.noteEpoch(pending_[i].epoch);
    npending_ = 0;
    npend_built_ = 0;
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
    for (;;)
    {
        if (job_.active)
        {
            workJob(t);
            if (job_.active)
                return;                   // the rest of it at the next step
        }
        int64_t mcu_us = 0;
        if (cfg_.external_build)
        {
            if (npend_built_ == 0)
                return;
            mcu_us = pend_built_[pb_head_].mcu_us;
        }
        else
        {
            if (npending_ == 0)
                return;
            mcu_us = pending_[0].mcu_us;
        }
        const double te = static_cast<double>(mcu_us) * 1.0e-6 - cfg_.epoch_latency_s;
        if (te > t)
            return;                       // the IMU has not reached it yet
        if (t - te > cfg_.max_epoch_age_s)
        {
            st_.epochs_stale++;
            pend_flags_ |= RGE_STALE;
            dropFront();
            continue;
        }
        startJob(te, imu_flags);
    }
}

// An epoch's work is a job of units: each observation built (here, unless a
// GNSS task built it: external_build), the checks on the built sky (each
// satellite against the IMU, the pad survey), the receiver clock, each
// satellite fused, and the close. The navigation filter does it a slice a
// step (epoch_slice_us), so the state it publishes keeps up with the IMU
// however many satellites there are: on the rig (r7-r9) one pass over 14
// satellites held the state 40 ms, a sky of 30 would be 90 ms. Every unit
// takes the epoch back by the age it has when it runs, which the IMU history
// covers (1.28 s at 100 steps a second); a job epoch_job_max_s old is
// finished at once.
void RawGnssShadow::startJob(double te, uint8_t imu_flags)
{
    job_.te = te;
    job_.in_flight = (imu_flags & kShadowInFlight) != 0;
    job_.g_peak = static_cast<double>(ekf_.takeGPeak());
    if (cfg_.external_build)
    {
        built_ = pend_built_[pb_head_];
        pb_head_ ^= 1;
        npend_built_--;
        job_.phase = kJobChecks;
        job_.build_us = built_.build_us;  // the GNSS task's (wall), then the checks here
    }
    else
    {
        job_.e = pending_[0];
        pending_[0] = pending_[1];
        npending_--;
        double r[3], v[3];
        ekf_.receiverEcef(r, v);
        stage_.begin(job_.e, r, job_.in_flight, built_);
        job_.phase = kJobBuild;
        job_.build_us = 0;
    }
    job_.fused = 0;
    job_.rs = RawGnssStats();
    job_.update_us = 0;
    job_.slices = 0;
    job_.t0_us = now();
    job_.active = true;
}

void RawGnssShadow::workJob(double t)
{
    const bool sliced = cfg_.epoch_slice_us > 0.0 && clock_ != nullptr && t - job_.te <= cfg_.epoch_job_max_s;
    const int64_t s0 = now();
    int64_t u0 = s0;
    job_.slices++;
    while (job_.active)
    {
        const int phase = job_.phase;
        jobUnit(t);
        const int64_t u1 = now();
        if (phase <= kJobChecks)
            job_.build_us += static_cast<uint32_t>(u1 - u0);
        else if (phase != kJobHeading && phase != kJobHeadingFit)   // the fit times itself
            job_.update_us += static_cast<uint32_t>(u1 - u0);
        u0 = u1;
        if (sliced && static_cast<double>(u1 - s0) >= cfg_.epoch_slice_us)
            break;
    }
    const uint32_t slice = static_cast<uint32_t>(u0 - s0);
    if (slice > st_.slice_us_max)
        st_.slice_us_max = slice;
    if (!job_.active)
    {
        st_.build_us = job_.build_us;
        st_.update_us = job_.update_us;
        if (st_.build_us > st_.build_us_max)
            st_.build_us_max = st_.build_us;
        if (st_.update_us > st_.update_us_max)
            st_.update_us_max = st_.update_us;
        st_.epoch_slices = job_.slices;
        st_.epoch_wall_us = static_cast<uint32_t>(u0 - job_.t0_us);
    }
}

void RawGnssShadow::jobUnit(double t)
{
    const double age = t - job_.te;
    switch (job_.phase)
    {
    case kJobBuild:
        if (stage_.next(built_, 1))
            job_.phase = kJobChecks;
        break;
    case kJobChecks:
        jobChecks(age, t);
        job_.phase = kJobHeading;
        break;
    case kJobHeading:
        job_.phase = jobHeading(age) ? kJobHeadingFit : kJobClock;
        break;
    case kJobHeadingFit:
        jobHeadingFit();
        job_.phase = kJobClock;
        break;
    case kJobClock:
        job_.phase = ekf_.beginGnssRaw(built_.meas, built_.n, age, job_.rs) ? kJobFuse : kJobClose;
        break;
    case kJobFuse:
        if (job_.fused < built_.n)
        {
            ekf_.updateGnssRawPart(built_.meas, job_.fused, job_.fused + 1, age, job_.rs);
            job_.fused++;
        }
        if (job_.fused >= built_.n)
            job_.phase = kJobClose;
        break;
    default:
        closeJob(age, t);
        break;
    }
}

void RawGnssShadow::jobChecks(double age, double t)
{
    const int n = built_.n;
    RawMeas* m = built_.meas;
    const double tow = built_.tow_s;
    // The Doppler of a loop under stress: the IMU's largest specific force
    // since the last epoch (RawMeasBuilder's rr_dyn_sigma), added here, where
    // the IMU is, whoever built the epoch
    const double s_dyn = cfg_.meas.rr_dyn_sigma * (job_.g_peak > 0.0 ? job_.g_peak : 0.0);
    if (s_dyn > 0.0)
        for (int i = 0; i < n; ++i)
        {
            const double s = static_cast<double>(m[i].sigma_rr);
            m[i].sigma_rr = static_cast<float>(std::sqrt(s * s + s_dyn * s_dyn));
        }
    // the ionosphere chosen anew on the pad: the survey's residuals were
    // against the other model
    if (built_.iono_changed)
        survey_.reset();
    use_broadcast_ = built_.use_broadcast != 0;
    // each satellite against the IMU since the last epoch (#1508): one that
    // jumped is taken as relocked, its sigma the builder's relock sigma
    double r_prior[3], v_prior[3];
    ekf_.receiverEcefAt(age, r_prior, v_prior);
    const int jumped = satcheck_.check(m, n, tow, r_prior, jumped_);
    for (int i = 0; i < n && jumped > 0; ++i)
    {
        if (!jumped_[i])
            continue;
        relock(m[i].sys, m[i].prn, tow);
        built_.lock_age_s[i] = 0.0;
        m[i].sigma_pr_corr = static_cast<float>(
            std::hypot(static_cast<double>(m[i].sigma_pr_corr), cfg_.meas.relock_sigma_m));
    }
    st_.last_sats_jumped = jumped;
    st_.sats_jumped += static_cast<uint32_t>(jumped);
    // the pad survey, against the launch site the seed carries (a fresh one)
    if (!job_.in_flight && have_seed_ && seed_.on_pad && seed_.pad_valid)
    {
        const double ts = static_cast<double>(unwrap32(seed_.time_us, static_cast<int64_t>(t * 1.0e6))) * 1.0e-6;
        if (std::fabs(t - ts) <= cfg_.seed_max_age_s)
        {
            double pad[3];
            geo_to_ecef(seed_.pad_lla[0], seed_.pad_lla[1], seed_.pad_lla[2], pad);
            survey_.observe(m, n, built_.lock_age_s, tow, pad);
            st_.survey_epochs++;
        }
    }
    st_.last_pad_corrected = use_broadcast_ ? 0 : survey_.correct(m, n, built_.r_approx);
}

void RawGnssShadow::closeJob(double age, double t)
{
    const int n = built_.n;
    const RawGnssStats& rs = job_.rs;
    double r_post[3], v_post[3];
    ekf_.receiverEcefAt(age, r_post, v_post);
    satcheck_.commit(built_.meas, n, built_.tow_s, r_post);
    st_.last = rs;
    st_.last_age_s = age;
    st_.last_meas = n;
    st_.last_tow_ms = static_cast<uint32_t>(std::llround(std::fmod(built_.tow_s, 604800.0) * 1000.0));
    st_.epochs_applied++;
    level_gate_.onEpoch(t, ekf_.velNed());
    if (rs.used_pr > 0)
    {
        own_fix_ = true;
        last_aided_t_ = t;
    }
    if (rs.clock_jump_ms != 0 || rs.clock_step_m != 0.0 || rs.drift_step_mps != 0.0)
        pend_flags_ |= RGE_CLOCK_STEP;
    job_.active = false;
}

// The heading from thrust, each step: seeded again at every still step on the
// pad (the filter's attitude and gyro bias, and the IMU biases the hand-off
// puts back), on the gyro alone from the last one; at launch it takes epochs
// if that seed is fresh, and in act mode the filter's heading is let go.
void RawGnssShadow::stepHeading(double t, const float acc[3], const float gyro[3], float span, uint8_t imu_flags)
{
    if (cfg_.heading_mode == kHeadingOff)
        return;
    const int64_t c0 = now();
    const bool ground = !(imu_flags & kShadowInFlight);
    const bool landed = (imu_flags & kShadowLanded) != 0;
    const bool still = ground && !landed && level_gate_.gSized(acc)
                       && gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]
                              < static_cast<float>(cfg_.still_rate_rps * cfg_.still_rate_rps);
    if (still)
    {
        fit_.start(ekf_.quat(), ekf_.gyroBias(), t);
        for (int i = 0; i < 3; ++i)
        {
            pad_ab_[i] = ekf_.accelBias()[i];
            pad_wb_[i] = ekf_.gyroBias()[i];
        }
        pad_ll_[0] = ekf_.lla()[0];
        pad_ll_[1] = ekf_.lla()[1];
        ekf_.biasCovariance(pad_Pb_);
        st_.heading_seeds++;
    }
    else if (fit_.started())
    {
        fit_.imu(acc, gyro, span);
    }
    if (!ground && fit_ground_)
    {
        // the launch edge
        fit_launch_t_ = t;
        fit_flight_ = fit_.started() && t - fit_.t0() <= cfg_.heading_seed_max_s;
        handed_ = false;
        if (fit_flight_)
            st_.heading_flights++;
        if (cfg_.heading_mode == kHeadingAct)
        {
            // from here GNSS turns the heading, and only the heading
            ekf_.setAttGateVertical(true);
            ekf_.setHeadingSigma(static_cast<float>(cfg_.heading_pad_sigma_deg / kRad2Deg));
        }
    }
    else if (ground && !fit_ground_)
    {
        fit_flight_ = false;              // back on the ground
        if (cfg_.heading_mode == kHeadingAct)
            ekf_.setAttGateVertical(cfg_.ekf.att_gate_vertical);
    }
    fit_ground_ = ground;
    const uint32_t us = static_cast<uint32_t>(now() - c0);
    if (us > st_.heading_step_us_max)
        st_.heading_step_us_max = us;
}

// An epoch's part in the fit, in two units: its Doppler velocity, at the
// instant the Doppler describes, into the fit (true when it was taken); then
// the fit again, and in act mode the hand-off once it is sure -- before this
// epoch is fused.
bool RawGnssShadow::jobHeading(double age)
{
    if (cfg_.heading_mode == kHeadingOff || !fit_flight_ || !job_.in_flight)
        return false;
    if (job_.te - fit_launch_t_ > cfg_.heading_fit_max_s)
        return false;
    const int64_t c0 = now();
    double r[3], v[3];
    ekf_.receiverEcefAt(age, r, v);
    double v_ned[3] = {}, var_h = 0.0;
    int used = 0;
    // in the pad's NED, where the fit integrates the IMU (the receiver's own
    // is turned from it by the Earth's curve, 1.6e-4 rad a kilometre)
    const bool vel = dopplerVelocityNed(built_.meas, built_.n, r, pad_ll_[0], pad_ll_[1], v_ned, var_h, used);
    const int64_t c1 = now();
    st_.heading_vel_us = static_cast<uint32_t>(c1 - c0);
    if (st_.heading_vel_us > st_.heading_vel_us_max)
        st_.heading_vel_us_max = st_.heading_vel_us;
    if (!vel)
    {
        st_.heading_vel_fails++;
        return false;
    }
    const double td = job_.te - cfg_.ekf.rr_lag_s;
    if (!fit_.gnss(td, v_ned, var_h))
        return false;                     // too slow sideways yet, or past the fit's history
    st_.heading_epochs++;
    for (int i = 0; i < 3; ++i)
        job_.hdg_v[i] = v_ned[i];
    job_.hdg_var = var_h;
    job_.hdg_t = td;
    job_.hdg_sats = used;
    return true;
}

void RawGnssShadow::jobHeadingFit()
{
    const int64_t c1 = now();
    const HeadingFit::Result res = fit_.solve();
    const int64_t c2 = now();
    const double* v_ned = job_.hdg_v;
    const double var_h = job_.hdg_var, td = job_.hdg_t;
    const int used = job_.hdg_sats;
    st_.heading_solve_us = static_cast<uint32_t>(c2 - c1);
    if (st_.heading_solve_us > st_.heading_solve_us_max)
        st_.heading_solve_us_max = st_.heading_solve_us;
    st_.heading_last = res;

    double filt = 0.0;                    // the filter's heading less the fit's
    if (res.ok)
    {
        float qf[4];
        fit_.attitude(res.psi, qf);
        filt = headingBetween(ekf_.quat(), qf);
    }
    const bool act = cfg_.heading_mode == kHeadingAct;
    HeadingFitData d = {};
    d.time_us = static_cast<uint32_t>(static_cast<int64_t>(std::llround(td * 1.0e6)));
    d.psi_cdeg = sats16(res.psi * kRad2Deg * 100.0);
    d.sigma_cdeg = res.ok && res.sigma > 0.0 ? sat16(res.sigma * kRad2Deg * 100.0) : HFT_NO_SIGMA;
    d.filt_cdeg = sats16(filt * kRad2Deg * 100.0);
    d.bias_mmps2[0] = sats16(res.bias[0] * 1000.0);
    d.bias_mmps2[1] = sats16(res.bias[1] * 1000.0);
    d.speed_cmps = sat16(std::hypot(v_ned[0], v_ned[1]) * 100.0);
    d.vel_sigma_mmps = sat16(std::sqrt(var_h > 0.0 ? var_h : 0.0) * 1000.0);
    d.rms_c = sat16(res.rms * 100.0);
    d.n = static_cast<uint8_t>(res.n > 255 ? 255 : res.n);
    d.n_cut = static_cast<uint8_t>(res.n_cut > 255 ? 255 : res.n_cut);
    d.sats = static_cast<uint8_t>(used > 255 ? 255 : used);
    const bool rival = res.ok && res.rival >= 0.0 && res.rival < cfg_.heading_handoff_rival;
    if (rival)
        st_.heading_rivals++;
    d.flags = static_cast<uint8_t>((act ? HFT_ACT : 0) | (res.ok ? HFT_SOLVED : 0) | (handed_ ? HFT_HANDED : 0) |
                                   (rival ? HFT_RIVAL : 0));
    d.vel_us = sat16(static_cast<double>(st_.heading_vel_us));
    d.solve_us = sat16(static_cast<double>(st_.heading_solve_us));
    d.seed_ms = sat16((fit_launch_t_ - fit_.t0()) * 1000.0);
    if (act && !handed_ && res.ok && res.sigma > 0.0 && res.sigma * kRad2Deg <= cfg_.heading_handoff_deg && !rival &&
        res.n >= cfg_.heading_handoff_min_epochs &&
        (res.sigma * kRad2Deg <= cfg_.heading_handoff_sure_deg || headingsApart(res, filt)))
    {
        handOverHeading(res, filt);
        d.flags |= HFT_HANDED_NOW;
        d.turn_cdeg = sats16(-filt * kRad2Deg * 100.0);
    }
    if (nhdg_ < kHdgQ)
        hdg_[nhdg_++] = d;
}

// Whether the filter's heading is further from the fit's than
// heading_handoff_apart of their combined 1-sigmas: lost, not just unsure.
// Always, with it 0.
bool RawGnssShadow::headingsApart(const HeadingFit::Result& r, double filt_minus_fit) const
{
    if (cfg_.heading_handoff_apart <= 0.0)
        return true;
    float s[3];
    ekf_.attSigmaNed(s);
    const double sd = static_cast<double>(s[2]);
    return std::fabs(filt_minus_fit) > cfg_.heading_handoff_apart * std::sqrt(r.sigma * r.sigma + sd * sd);
}

// The fit's heading to the filter: its attitude turned about the vertical
// onto the fit's, its heading's sigma the fit's, and what the old heading
// left behind undone -- the IMU biases as the pad left them, the velocity
// let go (its value kept), the satellites refused under it forgotten.
void RawGnssShadow::handOverHeading(const HeadingFit::Result& r, double filt_minus_fit)
{
    ekf_.turnHeading(static_cast<float>(-filt_minus_fit));
    ekf_.setHeadingSigma(static_cast<float>(r.sigma));
    ekf_.setBiases(pad_ab_, pad_wb_, pad_Pb_);
    ekf_.releaseVelocity(static_cast<float>(cfg_.heading_handoff_vel_mps));
    ekf_.clearRejects();
    handed_ = true;
    st_.heading_handoffs++;
    st_.heading_turn_deg = static_cast<float>(-filt_minus_fit * kRad2Deg);
    st_.heading_handoff_s = static_cast<float>(t_prev_ - fit_launch_t_);
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
    d.eph_sats = u8(ephemerisSatellites());
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
