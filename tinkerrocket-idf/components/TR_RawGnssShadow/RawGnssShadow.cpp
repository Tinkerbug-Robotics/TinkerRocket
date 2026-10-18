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

// The period of GPS time of week a time falls in (pr_period_s: each whole
// second of a 10 Hz stream), on the millisecond the receivers tag their
// epochs on
int64_t periodIndex(double tow_s, double period_s)
{
    const long long p = std::llround(period_s * 1000.0);
    const long long ms = std::llround(tow_s * 1000.0);
    if (p <= 0)
        return ms;
    return ms >= 0 ? ms / p : -((p - 1 - ms) / p);
}

// v (body) into NED by q (scalar first, body -> NED): q v q*
void bodyToNed(const float q[4], const double v[3], double out[3])
{
    const double qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    const double t[3] = {2.0 * (qy * v[2] - qz * v[1]), 2.0 * (qz * v[0] - qx * v[2]), 2.0 * (qx * v[1] - qy * v[0])};
    out[0] = v[0] + qw * t[0] + (qy * t[2] - qz * t[1]);
    out[1] = v[1] + qw * t[1] + (qz * t[0] - qx * t[2]);
    out[2] = v[2] + qw * t[2] + (qx * t[1] - qy * t[0]);
}

// heading_fit_period_s: an epoch this much early still counts as the period's
// (half a 20 Hz epoch: the epochs' own times jitter)
constexpr double kFitPeriodSlackS = 0.025;

}  // namespace

RawGnssMeasStage::Config RawGnssShadow::stageConfig(const Config& c)
{
    RawGnssMeasStage::Config s;
    s.meas = c.meas;
    s.default_week = c.default_week;
    s.iono_max_age_days = c.iono_max_age_days;
    s.sat_cap = c.sat_cap;  // the profile's cap (the mosaic's 16; the LC86G's 0, none)
    return s;
}

RawGnssShadow::RawGnssShadow() : RawGnssShadow(Config()) {}

RawGnssShadow::RawGnssShadow(const Config& cfg)
    : cfg_(cfg), ekf_(cfg.ekf), stage_(stageConfig(cfg)), survey_(cfg.survey), satcheck_(cfg.sat_check),
      fixcheck_(cfg.fix_check), level_gate_(cfg.level), fit_(cfg.heading_fit)
{
    // the turning body's allowances read the filter's rate; without them it
    // keeps none, and steps as the Beetle's always has
    ekf_.setTrackRate(rotOn());
}

double RawGnssShadow::rotLeverSigma(double axial_m, double radial_m, double rate_rps, double across_rps)
{
    const double a = axial_m > 0.0 && across_rps > 0.0 ? axial_m * across_rps : 0.0;
    const double r = radial_m > 0.0 && rate_rps > 0.0 ? radial_m * rate_rps : 0.0;
    return a + r;
}

void RawGnssShadow::setAntenna(bool known, const double b_m[3])
{
    ant_known_ = known;
    for (int i = 0; i < 3; ++i)
        ant_b_m_[i] = known ? b_m[i] : 0.0;
    ant_pending_ = true;
    st_.antenna_pending = 1;
    if (!(imu_flags_ & kShadowInFlight))
        applyAntenna();
}

// The offset on the filter (and in the config: its allowances read it, and a
// scratch copy of the filter takes the filter's own)
void RawGnssShadow::applyAntenna()
{
    cfg_.ekf.lever_on = ant_known_;
    for (int i = 0; i < 3; ++i)
        cfg_.ekf.lever_b_m[i] = ant_b_m_[i];
    cfg_.ekf.lever_att_mode = ant_known_ ? cfg_.antenna_att_mode : static_cast<uint8_t>(kLeverAttOff);
    ekf_.setLever(ant_known_, ant_b_m_, cfg_.antenna_att_mode);
    ekf_.setTrackRate(rotOn());
    ant_pending_ = false;
    st_.antenna_pending = 0;
    st_.antenna_sets++;
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

// Each refused sample is counted once, by the first gate it fails: the
// calibration, the size, then the dip. onImu calls this only on the ground at
// g's size, where the specific force is gravity: the dip takes it as down.
void RawGnssShadow::applyMag(double t, const float acc[3])
{
    if (!have_seed_ || !seed_.have_declination)
        return;                           // no north to set it against
    // A hard iron the calibration has not taken out is no heading, whatever
    // the field's size says (#1590). An uncalibrated Beetle carries ~40 uT
    // of it and reads 10-90 uT by attitude: at about a fifth of attitudes |B|
    // lands within mag_field_tol of the model's, and there it turns the
    // heading by tens of degrees, past 90 at some. A sim's or the rig
    // replay's field is the truth's: nothing to take out.
    if ((mag_.flags & (kShadowMagCalibrated | kShadowMagSim)) == 0)
    {
        st_.mag_uncal++;
        return;
    }
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
    // The dip: the field's part along the accelerometer's down, as an angle
    // below the horizontal, against the model's (#1590). A hard iron the size
    // test passes has to lie across the field, and that tips the field's
    // direction as well as its heading (mag_dip_tol_deg).
    if (seed_.have_dip && cfg_.mag_dip_tol_deg > 0.0)
    {
        const double an = std::sqrt(static_cast<double>(acc[0]) * acc[0] + static_cast<double>(acc[1]) * acc[1]
                                    + static_cast<double>(acc[2]) * acc[2]);
        if (an > 0.1 && mm > 0.1)
        {
            // down is -acc: the specific force of a body at rest points up
            double s = -(static_cast<double>(m[0]) * acc[0] + static_cast<double>(m[1]) * acc[1]
                         + static_cast<double>(m[2]) * acc[2]) / (an * mm);
            s = s > 1.0 ? 1.0 : (s < -1.0 ? -1.0 : s);
            const double d_err = std::asin(s) - static_cast<double>(seed_.dip_rad);
            st_.mag_dip_err_deg = static_cast<float>(d_err * kRad2Deg);
            if (std::fabs(d_err) * kRad2Deg > cfg_.mag_dip_tol_deg)
            {
                st_.mag_dip++;
                return;
            }
        }
    }
    if (ekf_.updateMagHeading(acc, m, true, seed_.declination_rad, err))
    {
        st_.mag_fused++;
        mag_fused_run_++;
        last_mag_fused_t_ = t;
    }
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
    out.clock = ekf_.clockReady() ? 1 : 0;
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
        if (ant_pending_)
            applyAntenna();               // an offset set in flight: back on the ground
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

    if (cfg_.dv_check != kDvOff)
    {
        // the Doppler change check's lever-arm allowance: the body's rate now, and its largest since the check
        const float wn = std::sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
        for (int i = 0; i < 3; ++i)
            dv_w_now_[i] = gyro[i];
        if (wn > dv_w_peak_)
            dv_w_peak_ = wn;
    }
    const int64_t c0 = now();
    ekf_.propagate(acc, gyro, static_cast<float>(span));
    if (static_cast<double>(ekf_.gExcess()) + 1.0 > cfg_.boost_screen_g)
        boost_seen_t_ = ekf_.time();      // the boost screen's window (startJob)
    t_prev_ = t;
    if (windupRingOn() || (cfg_.dv_check != kDvOff && cfg_.dv_lever_mode == 1))
    {
        // the body's rate over the step (the gyro less its bias): the carrier wind-up's history
        const float* wb = ekf_.gyroBias();
        const float wr[3] = {gyro[0] - wb[0], gyro[1] - wb[1], gyro[2] - wb[2]};
        if (windupRingOn())
            wu_ring_.push(t, wr);   // the MCU's clock, as each epoch's instant (te)
        if (cfg_.dv_check != kDvOff && cfg_.dv_lever_mode == 1)
        {
            // the change check's lever allowance (dv_lever_mode 1): the nose's own rate in NED, R (w x x), and
            // how fast it changes -- a roll about the nose changes neither (in float: a bound's input)
            dv_nose_.step(ekf_.quat(), wr, static_cast<float>(span));
            const float wp = std::sqrt(wr[1] * wr[1] + wr[2] * wr[2]);
            if (wp > dv_wp_peak_)
                dv_wp_peak_ = wp;
        }
    }
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
            applyMag(t, acc);
    }
    // The Doppler change check's heading allowance (dv_hdg_unset_deg): on the
    // pad, whether the heading is the magnetometer's as the launch edge judges
    // it (stepHeading); in flight, and once landed, as the launch left it
    if (cfg_.dv_check != kDvOff && cfg_.dv_hdg_unset_deg > 0.0)
    {
        if (ground && !(s.flags & kShadowLanded))
            dv_hdg_known_ = cfg_.primary && mag_fused_run_ > 0 && t - last_mag_fused_t_ <= cfg_.heading_mag_recent_s;
        st_.dv_hdg_unset = dv_hdg_known_ ? 0 : 1;
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
    // The EKF's hdg_obs_mode where its scope says, before this step's epochs
    if (cfg_.ekf.hdg_obs_mode != 0 && cfg_.heading_obs_scope != kObsEverywhere)
    {
        if (ground)
            obs_apogee_ = false;
        else if (s.flags & kShadowApogee)
            obs_apogee_ = true;
        ekf_.setHdgObsOn(!ground && (cfg_.heading_obs_scope == kObsFlight || !obs_apogee_));
    }

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
    // heading: its covariance stays the start's (p0_att about north and east,
    // p0_hdg about the seed's vertical, #1626 N10) for the levelling and the
    // magnetometer to settle.
    restored_ = cfg_.primary && !seed_.on_pad && seed_.restore;
    start_gen_ = seed_.start_gen;
    fit_.reset();
    fit_ground_ = !restored_;
    fit_flight_ = false;
    handed_ = false;
    launch_seen_ = false;
    // Act's attitude gate: on the pad the ground's (whatever a run that
    // stopped in flight left); a restored flight, with no launch edge to set
    // it, the flight's -- whole from launch, else the heading's, and the
    // apogee call (which the snapshot restores) opens it whole at the first
    // step after.
    gate_full_ = false;
    obs_apogee_ = false;
    boost_seen_t_ = -1.0e9;
    vert_latch_ = false;
    dvReset();
    wu_ring_.reset();
    wu_mon_.reset();
    wu_mon_.setTau(cfg_.windup_monitor_tau_s);
    n_g_held_ = 0;
    att_hold_until_ = -1.0e9;
    rot_skip_until_ = -1.0e9;
    pr_period_k_ = -1;
    fit_last_te_ = -1.0;
    if (cfg_.heading_mode == kHeadingAct)
    {
        if (restored_)
            setFlightGate(cfg_.heading_gate == kGateFullLaunch);
        else
            setGroundGate();
    }
    if (!cfg_.primary || restored_)
        ekf_.setQuaternion(seed_.quat);
    ekf_.seedGyroBias(seed_.gyro_bias);
    if (restored_)
    {
        st_.restores++;
        // A restored flight is the snapshot's state seconds later: the reboot's
        // gap was flown on nothing, so where it is, how fast and its attitude
        // are only roughly the snapshot's (restore_*_sigma), not known to the
        // start's p0 or setQuaternion's 1e-6. Its accelerometer bias is the
        // snapshot's, not zero (restore_seed_bias).
        if (cfg_.restore_pos_sigma_m > 0.0 || cfg_.restore_vel_sigma_mps > 0.0 || cfg_.restore_att_sigma_deg > 0.0)
            ekf_.inflateRestoreSigmas(static_cast<float>(cfg_.restore_pos_sigma_m),
                                      static_cast<float>(cfg_.restore_vel_sigma_mps),
                                      static_cast<float>(cfg_.restore_att_sigma_deg / kRad2Deg));
        if (cfg_.restore_seed_bias && seed_.has_accel_bias)
            ekf_.setAccelBias(seed_.accel_bias);
    }
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
    mag_fused_run_ = 0;
    last_mag_fused_t_ = -1.0e9;
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
    // the turning body (RawGnssShadow.h): the rate since the epoch before
    // sets this epoch's allowance on its Dopplers (jobChecks), whether GNSS
    // is held off the attitude for it -- on the filter itself, so the boost
    // screen's scratch copy takes the hold along -- and whether it is fused at
    // all (kJobClock). With rot_after_apogee, none of them before the apogee
    // call (the rate is still taken, so the first epoch after it reads only
    // its own interval's)
    job_.rot_sigma = 0.0;
    job_.rot_skip = false;
    job_.rot_dropped = false;
    if (rotOn())
    {
        float all = 0.0f, across = 0.0f;
        ekf_.takeRatePeak(all, across);
        const bool acts = !cfg_.rot_after_apogee || (imu_flags & kShadowApogee) != 0;
        if (acts && static_cast<double>(across) >= leverMinRps())
            job_.rot_sigma = rotLeverSigma(leverAxialM(), leverRadialM(), static_cast<double>(all),
                                           static_cast<double>(across));
        // the offset known and the heading unset: what any heading does to the predicted lever's velocity,
        // from the rate the bounds start at (RawGnssShadow.h)
        if (acts && cfg_.ekf.lever_on && !dv_hdg_known_ && static_cast<double>(across) >= cfg_.rot_lever_min_rps)
            job_.rot_sigma += rotLeverSigma(leverUnsetM(true), leverUnsetM(false), static_cast<double>(all),
                                            static_cast<double>(across));
        st_.rot_last_sigma_mps = static_cast<float>(job_.rot_sigma);
        if (st_.rot_last_sigma_mps > st_.rot_max_sigma_mps)
            st_.rot_max_sigma_mps = st_.rot_last_sigma_mps;
        st_.rot_last_across_rps = across;
        bool hold = false;
        if (cfg_.att_hold_rate_rps > 0.0 && acts)
        {
            if (static_cast<double>(across) >= cfg_.att_hold_rate_rps)
                att_hold_until_ = te + cfg_.att_hold_s;
            hold = te <= att_hold_until_;
        }
        ekf_.setAttHold(hold);
        st_.att_last_held = hold;
        if (hold)
            st_.att_held++;
        if (cfg_.rot_skip_rate_rps > 0.0 && acts)
        {
            if (static_cast<double>(across) >= cfg_.rot_skip_rate_rps)
                rot_skip_until_ = te + cfg_.rot_skip_s;
            job_.rot_skip = te <= rot_skip_until_;
        }
    }
    // the boost screen judges this epoch: no apogee call yet (a drogue's
    // snatch is no boost), and the IMU past boost_screen_g within the last
    // boost_screen_window_s (0 g: every epoch)
    const bool screening = cfg_.boost_screen == kScreenOn
                           || (cfg_.boost_screen == kScreenAct && cfg_.heading_mode == kHeadingAct);
    job_.screen = screening && !(imu_flags & kShadowApogee)
                  && (cfg_.boost_screen_g <= 0.0 || ekf_.time() - boost_seen_t_ <= cfg_.boost_screen_window_s);
    job_.vcheck = cfg_.vert_check_mps > 0.0;   // every epoch, every phase
    if (cfg_.external_build)
    {
        built_ = pend_built_[pb_head_];
        pb_head_ ^= 1;
        npend_built_--;
        if (windupRingOn())
            applyWindup(t_prev_ - te);
        job_.phase = dvJobOn() ? kJobDvCheck : kJobChecks;
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
    // the pseudoranges once in each pr_period_s (RawGnssShadow.h): on this
    // epoch if none in its period was fused yet (closeJob); the boost
    // screen's scratch copy takes the switch along
    if (cfg_.pr_period_s > 0.0)
    {
        job_.pr_k = periodIndex(built_.tow_s, cfg_.pr_period_s);
        ekf_.setFusePr(job_.pr_k != pr_period_k_);
    }
    job_.fused = 0;
    job_.cr_screened = false;
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
        {
            if (windupRingOn())
                applyWindup(age);
            job_.phase = dvJobOn() ? kJobDvCheck : kJobChecks;
        }
        break;
    case kJobDvCheck:
        jobDvCheck(age);
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
        // a skipped epoch (startJob: the body turning) takes its clock's step,
        // and nothing else: closed as a dropped one
        if (!ekf_.beginGnssRaw(built_.meas, built_.n, age, job_.rs))
            job_.phase = kJobClose;
        else if (job_.rot_skip)
        {
            job_.rot_dropped = true;
            job_.phase = kJobClose;
        }
        else
            job_.phase = job_.screen || job_.vcheck ? kJobScreen : kJobFuse;
        break;
    case kJobScreen:
        jobScreen(age);
        break;
    case kJobScreenDry:
        jobScreenDry();
        break;
    case kJobFuse:
        if (job_.fused < built_.n)
        {
            ekf_.updateGnssRawPart(built_.meas, job_.fused, job_.fused + 1, age, job_.rs);
            job_.fused++;
        }
        if (job_.fused >= built_.n)
            job_.phase = ekf_.params().cr_mode == kCarrierFuse ? kJobCarrier : kJobClose;
        break;
    case kJobCarrier:
        // fuse mode: the screen, then every screened rate at once in the next unit
        job_.cr_screened = true;
        job_.phase = ekf_.beginCarrier(built_.meas, built_.n, age, job_.rs) > 0 ? kJobCarrierFuse : kJobClose;
        break;
    case kJobCarrierFuse:
        ekf_.fuseCarrier(built_.meas, job_.rs);
        job_.phase = kJobClose;
        break;
    default:
        closeJob(age, t);
        break;
    }
}

// rr_dyn_window_s: the largest g excess an epoch carried within the window,
// this one's included -- the window's peak, however the thrust fell inside it.
// Kept are the epochs that could still be it: those the window has not left,
// each larger than every one after it. With more of them in the window than
// there is room for (kGHeld), the newest takes the last one's larger excess:
// held a little longer, never let go early.
double RawGnssShadow::holdGExcess(double te, double g)
{
    int n = 0;
    for (int i = 0; i < n_g_held_; ++i)
        if (te - g_held_[i].te <= cfg_.rr_dyn_window_s)
            g_held_[n++] = g_held_[i];
    while (n > 0 && g_held_[n - 1].g <= g)
        n--;
    if (n < kGHeld)
        g_held_[n++] = {te, g};
    else
        g_held_[n - 1].te = te;
    n_g_held_ = n;
    return g_held_[0].g;
}

// The carrier wind-up (windup_k; RawGnssShadow.h, RawGnssWindup.h): the
// gyro's rate at the Doppler's own instant, and with windup_k each Doppler
// less what the roll about the nose (or, with windup_geom, the whole rate
// toward that satellite) put on it at its system's wavelength, each carrier
// rate less what the roll angle over its interval put on its phase. Before
// anything judges the epoch. An instant the history no longer holds leaves the
// epoch as it came (counted); a carrier interval it does not hold drops that
// rate.
void RawGnssShadow::applyWindup(double age)
{
    job_.wu_ok = false;
    job_.wu_p = 0.0;
    const double td = job_.te - (cfg_.ekf.rr_lag_s > 0.0 ? cfg_.ekf.rr_lag_s : 0.0);
    float w[3];
    if (!wu_ring_.rateAt(td, w))
    {
        if (cfg_.windup_k > 0.0)
            st_.windup_misses++;
        return;
    }
    job_.wu_ok = true;
    job_.wu_p = static_cast<double>(w[0]);
    const double k = cfg_.windup_k;
    if (!(k > 0.0))
        return;
    const double wd[3] = {static_cast<double>(w[0]), static_cast<double>(w[1]), static_cast<double>(w[2])};
    // windup_geom: each line of sight in the body, at the receiver the epoch is taken back to
    double r_e[3] = {}, ne[3][3] = {};
    if (cfg_.windup_geom)
    {
        double v_e[3];
        ekf_.receiverEcefAt(age, r_e, v_e);
        const double* lla = ekf_.lla();
        const double sl = std::sin(lla[0]), cl = std::cos(lla[0]), so = std::sin(lla[1]), co = std::cos(lla[1]);
        const double rows[3][3] = {{-sl * co, -sl * so, cl}, {-so, co, 0.0}, {-cl * co, -cl * so, -sl}};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                ne[i][j] = rows[i][j];
    }
    const float* q = ekf_.quat();
    const float qc[4] = {q[0], -q[1], -q[2], -q[3]};   // NED -> body
    for (int i = 0; i < built_.n; ++i)
    {
        RawMeas& m = built_.meas[i];
        const int sys = m.sys < kProfileSys ? m.sys : 0;
        const double lam = GNSS_C / cfg_.ekf.cr_carrier_hz[sys];
        if (m.has_rr)
        {
            double c = -windup::rollRate(lam, k, wd[0]);
            if (cfg_.windup_geom)
            {
                double d[3], dn[3], eb[3];
                double n2 = 0.0;
                for (int a = 0; a < 3; ++a)
                {
                    d[a] = m.sat_pos[a] - r_e[a];
                    n2 += d[a] * d[a];
                }
                const double inv = n2 > 0.0 ? 1.0 / std::sqrt(n2) : 0.0;
                for (int a = 0; a < 3; ++a)
                    dn[a] = (ne[a][0] * d[0] + ne[a][1] * d[1] + ne[a][2] * d[2]) * inv;
                bodyToNed(qc, dn, eb);
                c = -windup::wuRate(lam, k, wd, eb);
            }
            m.rr += c;
        }
        if (m.has_cr)
        {
            double dphi = 0.0;
            if (wu_ring_.rollOver(td - static_cast<double>(m.cr_dt), td, dphi))
                m.cr_dphi_m += k * lam * dphi / windup::kTwoPi;
            else
            {
                m.has_cr = false;
                st_.windup_cr_misses++;
            }
        }
    }
    st_.windup_epochs++;
    const float l1 = static_cast<float>(-windup::rollRate(GNSS_C / GNSS_FREQ_L1_HZ, k, wd[0]));
    st_.windup_last_p_rps = w[0];
    st_.windup_last_l1_mps = l1;
    if (std::fabs(l1) > st_.windup_max_l1_mps)
        st_.windup_max_l1_mps = std::fabs(l1);
}

// windup_monitor (observe): this epoch's Dopplers at the prior (the change
// check's dv_next_: each less its prediction at the filter's velocity, the
// clock's drift in it) fitted for a velocity, a clock rate and the roll rate
// whose wind-up differs between wavelengths (RawGnssWindup.h's InterbandFit,
// L1/E1 the reference): what is left after windup_k's correction, plus what
// it took off, against the gyro's at the same instant. Nothing changes.
void RawGnssShadow::windupMonitor()
{
    if (!job_.wu_ok)
        return;
    const double km = cfg_.windup_monitor_k > 0.0 ? cfg_.windup_monitor_k : 1.0;
    const double lam_ref = GNSS_C / GNSS_FREQ_L1_HZ;
    // each Doppler's sigma as built, and the loop's stress the update will add (jobChecks: the builder's
    // rr_dyn_sigma per g of the IMU's peak since the epoch before)
    const double g_ex = job_.g_peak > 0.0 ? job_.g_peak : 0.0;
    const double s_dyn = cfg_.meas.rr_dyn_sigma * g_ex;
    windup::InterbandFit fit;
    for (int i = 0; i < dv_next_.n; ++i)
    {
        const int sys = dv_next_.sys[i] < kProfileSys ? dv_next_.sys[i] : 0;
        const double lam = GNSS_C / cfg_.ekf.cr_carrier_hz[sys];
        const double u[3] = {dv_next_.u[i][0], dv_next_.u[i][1], dv_next_.u[i][2]};
        const double si = static_cast<double>(dv_next_.s[i]);
        fit.add(u, dv_next_.y[i], std::sqrt(si * si + s_dyn * s_dyn), -km * (lam - lam_ref) / windup::kTwoPi);
    }
    double p_res = 0.0, sig = 0.0;
    if (!fit.solve(p_res, sig))
        return;
    const double p_wu = p_res + (cfg_.windup_k > 0.0 ? cfg_.windup_k / km : 0.0) * job_.wu_p;
    wu_mon_.add(job_.te, p_wu, sig, job_.wu_p);
    st_.wumon_epochs++;
    st_.wumon_last_p_rps = static_cast<float>(p_wu);
    st_.wumon_last_sig_rps = static_cast<float>(sig);
    st_.wumon_last_gyro_rps = static_cast<float>(job_.wu_p);
    st_.wumon_dp_rps = static_cast<float>(wu_mon_.diff());
    st_.wumon_dp_sig_rps = static_cast<float>(wu_mon_.diffSigma());
    if (wu_mon_.count() >= 10 && wu_mon_.diffSigma() > 0.0)
    {
        const float z = static_cast<float>(std::fabs(wu_mon_.diff()) / wu_mon_.diffSigma());
        if (z > st_.wumon_max_z)
            st_.wumon_max_z = z;
    }
}

void RawGnssShadow::jobChecks(double age, double t)
{
    const int n = built_.n;
    RawMeas* m = built_.meas;
    const double tow = built_.tow_s;
    // The Doppler of a loop under stress: the IMU's largest specific force
    // since the last epoch (RawMeasBuilder's rr_dyn_sigma), added here, where
    // the IMU is, whoever built the epoch. With rr_dyn_window_s, the largest
    // any epoch carried within that long (holdGExcess), so at 10 Hz the
    // epochs after burnout keep the boost's weighting for the second the
    // LC86G's one epoch kept it.
    double g_ex = job_.g_peak > 0.0 ? job_.g_peak : 0.0;
    if (cfg_.rr_dyn_window_s > 0.0)
        g_ex = holdGExcess(job_.te, g_ex);
    const double s_dyn = cfg_.meas.rr_dyn_sigma * g_ex;
    if (s_dyn > 0.0)
        for (int i = 0; i < n; ++i)
        {
            const double s = static_cast<double>(m[i].sigma_rr);
            m[i].sigma_rr = static_cast<float>(std::sqrt(s * s + s_dyn * s_dyn));
        }
    // the turning body's allowance (startJob): what an antenna off the IMU
    // can add to every Doppler while the body turns
    if (job_.rot_sigma > 0.0)
        for (int i = 0; i < n; ++i)
        {
            const double s = static_cast<double>(m[i].sigma_rr);
            m[i].sigma_rr = static_cast<float>(std::sqrt(s * s + job_.rot_sigma * job_.rot_sigma));
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

void RawGnssShadow::dvReset()
{
    dv_ref_.valid = false;
    dv_ref_.n = 0;
    n_dv_hold_ = 0;
    dv_epoch_until_ = -1.0e9;
    dv_epoch_excess_ = 0.0f;
    for (int i = 0; i < 3; ++i)
        dv_w_now_[i] = dv_w_ref_[i] = 0.0f;
    dv_w_peak_ = 0.0f;
    for (int i = 0; i < RawEpoch::kMaxObs; ++i)
        dv_S_[i] = dv_V_[i] = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        dv_SW_[i] = dv_SF_[i] = 0.0;
        for (int j = 0; j < 3; ++j)
            dv_CW_[i][j] = 0.0;
    }
    dv_SB_ = dv_SL_ = 0.0;
    dv_wwin_ = 0.0;
    dv_noise_var_ = 0.0;
    dv_noise_te_ = 0.0;
    dv_latch_ = false;
    dv_hdg_known_ = false;    // a flight restored in the air never knows
    lev_ref_ok_ = false;
    dv_nose_ = dvc::NoseTurn<float>();
    dv_wp_peak_ = 0.0f;
    dv_wpwin_ = 0.0;
    wu_ref_ok_ = false;
    wu_ref_p_ = 0.0;
}

// The lever roll: this end's lever velocity and nose (the Doppler's instant),
// the other end's kept by the epoch before; g = n2 x v2 - n1 x v1 + n x dv_sf
// (n the two ends' mean nose); then g' C^-1 w and g' C^-1 g by C's Cholesky.
void RawGnssShadow::leverRoll(double age, const double w[3], const double C[3][3], const DopplerChangeImu& imu)
{
    float p2[3], v2[3], n2[3];
    ekf_.leverAt(age + cfg_.ekf.rr_lag_s, p2, v2, n2);
    double a1[3], a2[3], nm[3];
    for (int i = 0; i < 3; ++i)
    {
        a1[i] = lev_ref_v_[i];
        a2[i] = v2[i];
        nm[i] = 0.5 * (static_cast<double>(lev_ref_n_[i]) + n2[i]);
    }
    auto cross = [](const double a[3], const double b[3], double o[3]) {
        o[0] = a[1] * b[2] - a[2] * b[1];
        o[1] = a[2] * b[0] - a[0] * b[2];
        o[2] = a[0] * b[1] - a[1] * b[0];
    };
    const double n1d[3] = {lev_ref_n_[0], lev_ref_n_[1], lev_ref_n_[2]};
    const double n2d[3] = {n2[0], n2[1], n2[2]};
    const double sf[3] = {imu.dv_sf[0], imu.dv_sf[1], imu.dv_sf[2]};
    double c1[3], c2[3], c3[3], g[3];
    cross(n2d, a2, c2);
    cross(n1d, a1, c1);
    cross(nm, sf, c3);
    for (int i = 0; i < 3; ++i)
        g[i] = c2[i] - c1[i] + c3[i];
    // C = L L' (C is the fit's covariance: symmetric, positive definite)
    double L[3][3] = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j <= i; ++j)
        {
            double sum = C[i][j];
            for (int k = 0; k < j; ++k)
                sum -= L[i][k] * L[j][k];
            if (i == j)
            {
                if (!(sum > 0.0))
                    return;
                L[i][i] = std::sqrt(sum);
            }
            else
            {
                L[i][j] = sum / L[j][j];
            }
        }
    // L^-1 g and L^-1 w: g' C^-1 g = |L^-1 g|^2, g' C^-1 w = (L^-1 g) . (L^-1 w)
    double lg[3], lw[3];
    for (int i = 0; i < 3; ++i)
    {
        double sg = g[i], sw = w[i];
        for (int k = 0; k < i; ++k)
        {
            sg -= L[i][k] * lg[k];
            sw -= L[i][k] * lw[k];
        }
        lg[i] = sg / L[i][i];
        lw[i] = sw / L[i][i];
    }
    const double info = lg[0] * lg[0] + lg[1] * lg[1] + lg[2] * lg[2];
    const double hy = lg[0] * lw[0] + lg[1] * lw[1] + lg[2] * lw[2];
    st_.lever_roll_epochs++;
    st_.lever_roll_info += info;
    st_.lever_roll_hy += hy;
    st_.lever_roll_last_deg = info > 0.0 ? static_cast<float>(hy / info * kRad2Deg) : 0.0f;
    st_.lever_roll_last_sigma_deg = info > 0.0 ? static_cast<float>(kRad2Deg / std::sqrt(info)) : 0.0f;
}

// dv_latch_k's level test (DopplerChange.h levelChi2): this epoch's Dopplers
// as they stand (dv_next_: each less its prediction at the filter's velocity,
// the clock's drift in it), the satellites on hold kept out, fitted for one
// velocity offset and one clock rate -- the offset's chi-square against the
// fit's covariance, the filter's velocity covariance and the floor; -1 with
// no fit. Each row's 1-sigma is its own and row_var in quadrature. The rows
// go in dv_rows_: the change check is done with them by then. With the offset
// known each prediction took the lever's velocity at the filter's attitude
// (v_lev: this end's, as dopplerChange took it); where the heading is unset
// that attitude's heading can be any, so v_lev goes back on the fitted
// offset, as on the epoch test's change (knownLeverArm's wu): the antenna's
// velocity less the filter's, unknown's level. The Dopplers it lets go to
// the update allow for that heading too (leverUnsetM).
double RawGnssShadow::dvLevelChi2(double row_var, const float v_lev[3])
{
    const double te = job_.te;
    int n = 0;
    for (int i = 0; i < dv_next_.n && n < RawEpoch::kMaxObs; ++i)
    {
        dvc::Row& r = dv_rows_[n++];
        for (int a = 0; a < 3; ++a)
            r.u[a] = static_cast<double>(dv_next_.u[i][a]);
        r.z = dv_next_.y[i];
        const double s = static_cast<double>(dv_next_.s[i]);
        r.s = std::sqrt(s * s + row_var);
        r.im = r.iref = r.iout = -1;
        r.e = 0.0;
        bool held = false;
        for (int h = 0; h < n_dv_hold_ && !held; ++h)
            held = dv_hold_[h].sys == dv_next_.sys[i] && dv_hold_[h].prn == dv_next_.prn[i] && dv_hold_[h].until >= te;
        r.in = !held;
    }
    if (n < 5)
        return -1.0;
    const dvc::Fit f = dvc::fit(dv_rows_, n, cfg_.dv_sat_k, 6, true);
    if (!f.ok)
        return -1.0;
    const bool unlever = cfg_.ekf.lever_on && !dv_hdg_known_;
    double off[3], Cw[3][3], Pv[3][3];
    for (int i = 0; i < 3; ++i)
    {
        off[i] = unlever ? f.x[i] + static_cast<double>(v_lev[i]) : f.x[i];
        for (int j = 0; j < 3; ++j)
        {
            Cw[i][j] = f.C[i][j];
            Pv[i][j] = static_cast<double>(ekf_.P(RawGnssEKF<float>::kVel + i, RawGnssEKF<float>::kVel + j));
        }
    }
    return dvc::levelChi2(off, Cw, Pv, cfg_.dv_floor_mps * cfg_.dv_floor_mps);
}

// The Doppler change check (RawGnssShadow.h, DopplerChange.h), the epoch's
// first unit: before the stress term goes on the Dopplers (jobChecks), whose
// own allowance here is the IMU's peak since the epoch before (dv_sigma_per_g)
// -- a loop's error under stress changes with the stress, not the window's
// hold. Then, acting, the Dopplers held out are no Dopplers for the rest of
// the epoch: the heading fit, the vertical check, the boost screen and the
// update all see the rest.
void RawGnssShadow::jobDvCheck(double age)
{
    const double te = job_.te;
    DopplerChangeImu imu;
    if (cfg_.dv_check == kDvOff)
    {
        // the wind-up monitor alone (windup_monitor): this epoch's Dopplers at the prior, no change taken
        // (dv_ref_ stays as dvReset left it, invalid: the check is off)
        (void)ekf_.dopplerChange(built_.meas, built_.n, age, dv_ref_, dv_next_, dv_rows_, imu);
        windupMonitor();
        return;
    }
    const int nr = ekf_.dopplerChange(built_.meas, built_.n, age, dv_ref_, dv_next_, dv_rows_, imu);
    if (cfg_.windup_monitor)
        windupMonitor();
    const double g_ex = job_.g_peak > 0.0 ? job_.g_peak : 0.0;
    const double dyn = cfg_.dv_sigma_per_g * g_ex;
    // the measured noise, as the epochs before left it (each change has two Dopplers' worth)
    const bool noise = cfg_.dv_noise_tau_s > 0.0;
    double nvar = 0.0;
    if (noise)
    {
        const double dt_n = te - dv_noise_te_;
        nvar = dv_noise_var_ * (dt_n > 0.0 ? std::exp(-dt_n / cfg_.dv_noise_tau_s) : 1.0);
    }
    double model2[RawEpoch::kMaxObs];     // each change's variance as the model gives it (scratch)
    for (int k = 0; k < nr; ++k)
    {
        const double s = dv_rows_[k].s;
        model2[k] = s * s + cfg_.dv_floor_mps * cfg_.dv_floor_mps + dyn * dyn;
        dv_rows_[k].s = std::sqrt(model2[k] + 2.0 * nvar);
    }
    // a satellite on hold stays out of the fit (its slow bias would pass into the receiver's change, and
    // the epoch's test would hold the whole sky for it): judged against the rest
    for (int k = 0; k < nr; ++k)
    {
        const RawMeas& mk = built_.meas[dv_rows_[k].im];
        bool held = false;
        for (int h = 0; h < n_dv_hold_ && !held; ++h)
            held = dv_hold_[h].sys == mk.sys && dv_hold_[h].prn == mk.prn && dv_hold_[h].until >= te;
        dv_rows_[k].in = !held;
    }
    // four unknowns and one to spare
    dvc::Fit f;
    if (nr >= 5)
        f = dvc::fit(dv_rows_, nr, cfg_.dv_sat_k, 6, true);
    if (noise)
    {
        // this epoch's excess, the median's (half the rows can be off on their own): a change's residual has
        // its two Dopplers' variance, and chi-square(1)'s median is 0.455
        if (f.ok)
        {
            double r2[RawEpoch::kMaxObs], m2[RawEpoch::kMaxObs];
            for (int k = 0; k < nr; ++k)
            {
                const double r = dv_rows_[k].e * dv_rows_[k].s;
                r2[k] = r * r;
                m2[k] = model2[k];
            }
            const double ex = 0.5 * (ekf_detail::medianOf(r2, nr) / 0.455 - ekf_detail::medianOf(m2, nr));
            if (ex > nvar)
                nvar = ex;
        }
        dv_noise_var_ = nvar;
        dv_noise_te_ = te;
        st_.dv_noise_mps = static_cast<float>(std::sqrt(nvar));
        if (st_.dv_noise_mps > st_.dv_noise_max_mps)
            st_.dv_noise_max_mps = st_.dv_noise_mps;
    }
    const bool win = cfg_.dv_window_s > 0.0;
    const double lam = win && imu.ok ? std::exp(-imu.dt / cfg_.dv_window_s) : 0.0;
    for (int i = 0; i < dv_next_.n; ++i)
        dv_S2_[i] = dv_V2_[i] = 0.0;
    bool epoch_fail = false;
    int sat_fails = 0;
    if (f.ok)
    {
        st_.dv_checks++;
        // the clock rate's change it fitted, and (the gyro's history kept) the L1 wind-up's change over the
        // same interval: with windup_k off, the one should carry the other
        st_.dv_last_c_mps = static_cast<float>(f.x[3]);
        st_.dv_last_wu_dc_mps = 0.0f;
        if (windupRingOn() && job_.wu_ok && wu_ref_ok_)
        {
            const double k = cfg_.windup_k > 0.0 ? cfg_.windup_k : cfg_.windup_monitor_k;
            const double lam = GNSS_C / GNSS_FREQ_L1_HZ;
            st_.dv_last_wu_dc_mps =
                static_cast<float>(windup::rollRate(lam, k, job_.wu_p) - windup::rollRate(lam, k, wu_ref_p_));
        }
        const double k2 = cfg_.dv_sat_k * cfg_.dv_sat_k;
        for (int k = 0; k < nr; ++k)
        {
            dvc::Row& r = dv_rows_[k];
            bool fail = r.e * r.e > k2;
            const double ae = std::fabs(r.e);
            if (static_cast<float>(ae) > st_.dv_max_e)
                st_.dv_max_e = static_cast<float>(ae);
            double excess = ae * r.s;
            if (win && r.iout >= 0)
            {
                // The satellite's residual summed, leaky: a drift adds up, an epoch's noise does not -- each
                // change is this epoch's Doppler less the one before, so the sum is this epoch's less a
                // weighted mean of the earlier ones (the weights summing to 1), whatever the noise's own
                // correlation. Its variance at most this end's plus that mean's, each half a change's
                // (dv_V_: the changes' variance, its leaky mean -- the first change's alone).
                const double s2 = r.s * r.s;
                const double Vm = dv_V_[r.iref] > 0.0 ? lam * dv_V_[r.iref] + (1.0 - lam) * s2 : s2;
                const double S = lam * dv_S_[r.iref] + r.e * r.s;
                const double V = 0.5 * (s2 + Vm);
                dv_S2_[r.iout] = S;
                dv_V2_[r.iout] = Vm;
                if (!fail && S * S > k2 * V)
                {
                    fail = true;
                    excess = std::fabs(S);
                    st_.dv_sat_win_fails++;
                }
            }
            if (!fail)
                continue;
            sat_fails++;
            st_.dv_sat_fails++;
            // held out from this epoch to dv_hold_s after it
            const uint8_t sys = built_.meas[r.im].sys, prn = built_.meas[r.im].prn;
            int h = 0;
            while (h < n_dv_hold_ && !(dv_hold_[h].sys == sys && dv_hold_[h].prn == prn))
                h++;
            if (h == n_dv_hold_)
            {
                if (n_dv_hold_ == kDvHold)
                {
                    // full: the one whose hold ends first gives way
                    h = 0;
                    for (int j = 1; j < n_dv_hold_; ++j)
                        if (dv_hold_[j].until < dv_hold_[h].until)
                            h = j;
                }
                else
                {
                    n_dv_hold_++;
                }
                dv_hold_[h].sys = sys;
                dv_hold_[h].prn = prn;
            }
            dv_hold_[h].until = te + cfg_.dv_hold_s;
            dv_hold_[h].excess = static_cast<float>(excess);
        }
        // the epoch: its change against the IMU's, the allowances in
        const double w[3] = {f.x[0], f.x[1], f.x[2]};
        double Cw[3][3];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                Cw[i][j] = f.C[i][j];
        const double sf[3] = {imu.dv_sf[0], imu.dv_sf[1], imu.dv_sf[2]};
        double dw2 = 0.0;
        for (int i = 0; i < 3; ++i)
        {
            const double d = static_cast<double>(dv_w_now_[i]) - static_cast<double>(dv_w_ref_[i]);
            dw2 += d * d;
        }
        const double wp = static_cast<double>(dv_w_peak_);
        dvc::Allow a;
        a.att_rad = cfg_.dv_att_allow_deg / kRad2Deg;
        // the lever arm the allowance takes: the bound, or with the offset known what its prediction can be
        // off by (DopplerChange.h knownLeverArm) -- where the heading is unset, the prediction taken back out
        // of the change (wu) and the whole offset allowed. The lever roll reads the change as fitted (w).
        double arm = dvLeverM();
        double wu[3] = {w[0], w[1], w[2]};
        if (cfg_.ekf.lever_on)
        {
            const double* r = cfg_.ekf.lever_b_m;
            double dv_lev[3];
            for (int i = 0; i < 3; ++i)
            {
                const float ref = lev_ref_ok_ ? lev_ref_v_[i] : 0.0f;
                dv_lev[i] = static_cast<double>(imu.v_lev[i]) - static_cast<double>(ref);
            }
            arm = dvc::knownLeverArm(w, dv_lev, std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]), dvLeverM(),
                                     dv_hdg_known_, a.att_rad, wu);
        }
        // the arm times the whole rate's change, and the turn the peak rate can make in a step; or (dv_lever_mode
        // 1) an antenna on the nose's axis moves only as the nose turns: its velocity change between the two
        // checks' steps, and what the nose's turn can add between each step and its epoch's instant; one across
        // the axis (dv_lever_radial_m) turns with the whole rate, its velocity's change at most twice its speed
        double lever = arm * (std::sqrt(dw2) + wp * wp * imu.dt);
        if (cfg_.dv_lever_mode == 1)
        {
            const double ax =
                arm * (static_cast<double>(dv_nose_.change()) + 2.0 * age * static_cast<double>(dv_nose_.ndd_peak));
            const double turn = std::fmin(2.0 * wp, wp * wp * imu.dt);
            lever = ax + cfg_.dv_lever_radial_m * (std::sqrt(dw2) + turn);
        }
        st_.dv_last_lever_mps = static_cast<float>(lever);
        a.scale = cfg_.dv_scale_allow;
        a.bias_mps = cfg_.dv_bias_allow_mps2 * imu.dt;
        a.lever_mps = lever;
        // no magnetometer heading at launch: the heading is the start's guess,
        // any distance off, and the horizontal change only as sure as that
        if (cfg_.dv_hdg_unset_deg > 0.0 && !dv_hdg_known_)
            a.hdg_rad = cfg_.dv_hdg_unset_deg / kRad2Deg;
        const double chi2 = dvc::epochChi2(wu, Cw, sf, a);
        // the lever roll (RawGnssShadow.h): the fitted change against the roll a turning antenna points to
        if (cfg_.ekf.lever_on && cfg_.ekf.lever_att_mode == kLeverAttObserve && lev_ref_ok_ && imu.ok)
            leverRoll(age, w, Cw, imu);
        const double ke2 = cfg_.dv_epoch_k * cfg_.dv_epoch_k;
        st_.dv_last_chi2 = static_cast<float>(chi2);
        st_.dv_last_along = static_cast<float>(dvc::alongSf(wu, sf));
        if (static_cast<float>(chi2) > st_.dv_max_chi2)
            st_.dv_max_chi2 = static_cast<float>(chi2);
        double excess = std::sqrt(wu[0] * wu[0] + wu[1] * wu[1] + wu[2] * wu[2]);
        epoch_fail = chi2 < 0.0 || chi2 > ke2 || 2 * sat_fails > nr;
        if (win)
        {
            // the change summed as each satellite's (above): its noise this epoch's covariance and the leaky
            // mean's, each half; what the IMU can be off by adds up as it goes (the specific force's change
            // summed, the bias's and the lever arm's allowances summed)
            const bool first = !(dv_CW_[0][0] > 0.0);
            double Cs[3][3];
            for (int i = 0; i < 3; ++i)
            {
                dv_SW_[i] = lam * dv_SW_[i] + wu[i];
                dv_SF_[i] = lam * dv_SF_[i] + sf[i];
                for (int j = 0; j < 3; ++j)
                {
                    dv_CW_[i][j] = first ? Cw[i][j] : lam * dv_CW_[i][j] + (1.0 - lam) * Cw[i][j];
                    Cs[i][j] = 0.5 * (Cw[i][j] + dv_CW_[i][j]);
                }
            }
            dv_SB_ = lam * dv_SB_ + a.bias_mps;
            dv_SL_ = lam * dv_SL_ + a.lever_mps;
            // an antenna off the IMU: its velocity's changes telescope like the Dopplers' -- the sum is its
            // lever velocity now less a weighted mean of the earlier ones, at most the arm times the rate now
            // and the largest in the window -- so the summed allowance is capped there (summed alone, a
            // weathercocking coast's turns add up to a window that misses a slow ramp)
            dv_wwin_ = std::fmax(lam * dv_wwin_, wp);
            const double wn = std::sqrt(static_cast<double>(dv_w_now_[0]) * dv_w_now_[0]
                                        + static_cast<double>(dv_w_now_[1]) * dv_w_now_[1]
                                        + static_cast<double>(dv_w_now_[2]) * dv_w_now_[2]);
            dvc::Allow aw = a;
            aw.bias_mps = dv_SB_;
            aw.lever_mps = std::fmin(dv_SL_, arm * (wn + dv_wwin_));
            if (cfg_.dv_lever_mode == 1)
            {
                // the axial antenna's speed is the arm times the rate across the nose; the radial one's, the
                // whole rate's
                dv_wpwin_ = std::fmax(lam * dv_wpwin_, static_cast<double>(dv_wp_peak_));
                const double wpn = std::sqrt(static_cast<double>(dv_w_now_[1]) * dv_w_now_[1]
                                             + static_cast<double>(dv_w_now_[2]) * dv_w_now_[2]);
                aw.lever_mps =
                    std::fmin(dv_SL_, arm * (wpn + dv_wpwin_) + cfg_.dv_lever_radial_m * (wn + dv_wwin_));
            }
            st_.dv_last_lever_win_mps = static_cast<float>(aw.lever_mps);
            const double chi2w = dvc::epochChi2(dv_SW_, Cs, dv_SF_, aw);
            st_.dv_last_chi2_win = static_cast<float>(chi2w);
            if (!epoch_fail && chi2w > ke2)
            {
                epoch_fail = true;
                excess = std::sqrt(dv_SW_[0] * dv_SW_[0] + dv_SW_[1] * dv_SW_[1] + dv_SW_[2] * dv_SW_[2]);
                st_.dv_epoch_win_fails++;
            }
        }
        if (epoch_fail)
        {
            st_.dv_epoch_fails++;
            dv_epoch_until_ = te + cfg_.dv_epoch_hold_s;
            dv_epoch_excess_ = static_cast<float>(excess);
        }
    }
    else
    {
        // nothing to sum across: the window starts over
        for (int i = 0; i < 3; ++i)
        {
            dv_SW_[i] = dv_SF_[i] = 0.0;
            for (int j = 0; j < 3; ++j)
                dv_CW_[i][j] = 0.0;
        }
        dv_SB_ = dv_SL_ = 0.0;
        dv_wwin_ = 0.0;
        dv_wpwin_ = 0.0;
    }
    // this epoch is the next one's start: its Dopplers, and (lever_on) the lever's velocity they were predicted
    // with, as dopplerChange took it, and the nose then (the lever roll's); a reference taken with the offset
    // unknown has none
    dv_ref_ = dv_next_;
    lev_ref_ok_ = cfg_.ekf.lever_on;
    if (lev_ref_ok_)
    {
        float lp[3], lv[3];
        ekf_.leverAt(age + cfg_.ekf.rr_lag_s, lp, lv, lev_ref_n_);
        for (int i = 0; i < 3; ++i)
            lev_ref_v_[i] = imu.v_lev[i];
    }
    for (int i = 0; i < dv_ref_.n; ++i)
    {
        dv_S_[i] = dv_S2_[i];
        dv_V_[i] = dv_V2_[i];
    }
    for (int i = 0; i < 3; ++i)
        dv_w_ref_[i] = dv_w_now_[i];
    dv_w_peak_ = std::sqrt(dv_w_now_[0] * dv_w_now_[0] + dv_w_now_[1] * dv_w_now_[1] + dv_w_now_[2] * dv_w_now_[2]);
    if (cfg_.dv_lever_mode == 1)
    {
        dv_nose_.rebase();
        dv_wp_peak_ = std::sqrt(dv_w_now_[1] * dv_w_now_[1] + dv_w_now_[2] * dv_w_now_[2]);
    }
    wu_ref_p_ = job_.wu_p;
    wu_ref_ok_ = job_.wu_ok;
    // the holds that have run out
    int nh = 0;
    for (int h = 0; h < n_dv_hold_; ++h)
        if (dv_hold_[h].until >= te)
            dv_hold_[nh++] = dv_hold_[h];
    n_dv_hold_ = nh;
    // dv_latch_k: a failed epoch latches, and the latch holds every Doppler until the epoch's Dopplers as they
    // stand agree with the filter's velocity, which the IMU, the barometer and the pseudoranges carried
    // meanwhile (in flight, or on the ground with dv_ground)
    if (cfg_.dv_latch_k > 0.0)
    {
        if (!(job_.in_flight || cfg_.dv_ground))
        {
            dv_latch_ = false;
        }
        else if (epoch_fail && !dv_latch_)
        {
            dv_latch_ = true;
            st_.dv_latches++;
        }
        else if (dv_latch_ && !epoch_fail)
        {
            // each Doppler's noise as the change's model gives it (one Doppler's, not two)
            const double c2 = dvLevelChi2(cfg_.dv_floor_mps * cfg_.dv_floor_mps + dyn * dyn + nvar, imu.v_lev);
            st_.dv_last_level_chi2 = static_cast<float>(c2);
            if (c2 >= 0.0 && c2 <= cfg_.dv_latch_k * cfg_.dv_latch_k)
            {
                dv_latch_ = false;
                st_.dv_latch_releases++;
            }
        }
        if (dv_latch_)
            st_.dv_latch_epochs++;
    }
    if (cfg_.dv_check != kDvAct || !(job_.in_flight || cfg_.dv_ground))
        return;
    // acting: held out (or de-weighted) for the rest of the epoch; the measured noise on every Doppler
    bool all = cfg_.dv_latch_k > 0.0 ? dv_latch_ : te <= dv_epoch_until_;
    if (!all && cfg_.dv_hold_all_frac < 1.0)
    {
        int nrr = 0, nheld = 0;
        for (int i = 0; i < built_.n; ++i)
        {
            const RawMeas& mi = built_.meas[i];
            if (!mi.has_rr)
                continue;
            nrr++;
            for (int h = 0; h < n_dv_hold_; ++h)
                if (dv_hold_[h].sys == mi.sys && dv_hold_[h].prn == mi.prn)
                {
                    nheld++;
                    break;
                }
        }
        if (nheld > 0 && static_cast<double>(nheld) > cfg_.dv_hold_all_frac * static_cast<double>(nrr))
        {
            all = true;
            st_.dv_all_held++;
        }
    }
    const float nsig2 = static_cast<float>(nvar);
    int held = 0;
    for (int i = 0; i < built_.n; ++i)
    {
        RawMeas& mi = built_.meas[i];
        if (!mi.has_rr)
            continue;
        if (nsig2 > 0.0f)
            mi.sigma_rr = std::sqrt(mi.sigma_rr * mi.sigma_rr + nsig2);
        float excess = 0.0f;
        bool hold = false;
        if (all)
        {
            hold = true;
            excess = dv_epoch_excess_;
        }
        for (int h = 0; h < n_dv_hold_; ++h)
            if (dv_hold_[h].sys == mi.sys && dv_hold_[h].prn == mi.prn)
            {
                hold = true;
                if (dv_hold_[h].excess > excess)
                    excess = dv_hold_[h].excess;
                break;
            }
        if (!hold)
            continue;
        held++;
        if (cfg_.dv_deweight)
            mi.sigma_rr = std::sqrt(mi.sigma_rr * mi.sigma_rr + excess * excess);
        else
            mi.has_rr = false;
    }
    st_.dv_held_rr += static_cast<uint32_t>(held);
    if (held > 0)
        st_.dv_held_epochs++;
}

// The vertical check and the boost screen (RawGnssShadow.h), their first
// unit. The Dopplers at the state as the clock's step left it: with the
// vertical check on, an epoch it can fit (four Dopplers or more) whose fitted
// vertical is too far off is dropped (closeJob). Then a boost epoch, fitted or
// not: half or more of its Dopplers refused, it is dropped; else the filter is
// copied, and the epoch worked through on the copy a satellite a unit
// (jobScreenDry).
void RawGnssShadow::jobScreen(double age)
{
    RawGnssStats& rs = job_.rs;
    ekf_.screenRangeRates(built_.meas, built_.n, age, rs);
    // after a vertical drop, an epoch is dropped too unless its fit is back
    // within vert_release_mps: too thin to fit, it is not
    if (vert_latch_)
    {
        const bool back = rs.screen_fit_n >= 4
                          && std::fabs(static_cast<double>(rs.screen_dv_down_mps)) <= cfg_.vert_release_mps;
        if (!back)
        {
            st_.vert_latched++;
            rs.screen_dropped = true;
            job_.phase = kJobClose;
            return;
        }
        vert_latch_ = false;
    }
    if (job_.vcheck && rs.screen_fit_n >= 4)
    {
        const float dv = rs.screen_dv_down_mps;
        const bool drop = std::fabs(static_cast<double>(dv)) > cfg_.vert_check_mps;
        st_.vert_checked++;
        st_.vert_last_dv_down = dv;
        if (drop)
        {
            st_.vert_dropped++;
            vert_latch_ = cfg_.vert_release_mps > 0.0;
        }
        else if (std::fabs(dv) > std::fabs(st_.vert_worst_dv_down))
            st_.vert_worst_dv_down = dv;
        if (drop || !job_.screen)
        {
            rs.screen_dropped = drop;
            job_.phase = drop ? kJobClose : kJobFuse;
            return;
        }
    }
    if (!job_.screen)
    {
        job_.phase = kJobFuse;            // no boost epoch, and the vertical check passed it or could not fit it
        return;
    }
    st_.boost_screened++;
    st_.boost_last_rr = rs.screen_rr;
    st_.boost_last_refused = rs.screen_refused;
    st_.boost_last_dry_used = -1;
    st_.boost_last_dry_refused = 0;
    st_.boost_last_g = static_cast<float>(job_.g_peak + 1.0);
    if (rs.screen_refused > 0 && 2 * rs.screen_refused >= rs.screen_rr)
    {
        screenVerdict(true, false);
        return;
    }
    dry_ = ekf_;
    job_.dry_k = 0;
    job_.dry_age = age;
    job_.dry_rs = RawGnssStats();
    job_.phase = kJobScreenDry;
}

// The epoch's next measurement on the scratch copy, as the update fuses it --
// at the age the epoch had when copied: the copy does not step -- and, the
// last done, the verdict on its Dopplers' refusals.
void RawGnssShadow::jobScreenDry()
{
    if (job_.dry_k < built_.n)
    {
        dry_.updateGnssRawPart(built_.meas, job_.dry_k, job_.dry_k + 1, job_.dry_age, job_.dry_rs);
        job_.dry_k++;
    }
    if (job_.dry_k < built_.n)
        return;
    const RawGnssStats& d = job_.dry_rs;
    job_.rs.screen_seq_used = d.used_rr;
    job_.rs.screen_seq_refused = d.rejected_rr;
    st_.boost_last_dry_used = d.used_rr;
    st_.boost_last_dry_refused = d.rejected_rr;
    screenVerdict(d.rejected_rr > 0 && d.rejected_rr >= d.used_rr, true);
}

void RawGnssShadow::screenVerdict(bool drop, bool dry)
{
    job_.rs.screen_dropped = drop;
    if (drop)
    {
        st_.boost_dropped++;
        if (dry)
            st_.boost_dropped_dry++;
    }
    job_.phase = drop ? kJobClose : kJobFuse;
}

void RawGnssShadow::closeJob(double age, double t)
{
    const int n = built_.n;
    RawGnssStats& rs = job_.rs;
    if (rotOn())
    {
        st_.rot_last_skipped = job_.rot_dropped;
        if (job_.rot_dropped)
            st_.rot_skipped++;
    }
    if (rs.screen_dropped || job_.rot_dropped)
    {
        // Dropped by the vertical check or the boost screen, or skipped while
        // the body turned (rot_skip_rate_rps): none of it fused, and nothing
        // of it kept -- its receiver is no carrier rate's earlier end (the next
        // epoch's rates across it go unmatched), its pseudoranges no satellite
        // check's reference, its velocity no levelling epoch. The clock's step
        // (beginGnssRaw) stands: every satellite agreed on it.
        ekf_.dropEpoch();
        if (ekf_.params().cr_mode != kCarrierOff)
            ekf_.endCarrier(rs);
        st_.last = rs;
        st_.last_age_s = age;
        st_.last_meas = n;
        st_.last_tow_ms = static_cast<uint32_t>(std::llround(std::fmod(built_.tow_s, 604800.0) * 1000.0));
        if (rs.clock_jump_ms != 0 || rs.clock_step_m != 0.0 || rs.drift_step_mps != 0.0)
            pend_flags_ |= RGE_CLOCK_STEP;
        job_.active = false;
        return;
    }
    // the carrier: observe screens here, in the unit that closes the epoch
    // (the state untouched, no step falls differently); the epoch closed
    if (ekf_.params().cr_mode != kCarrierOff)
    {
        if (!job_.cr_screened)
            (void)ekf_.beginCarrier(built_.meas, n, age, rs);
        ekf_.endCarrier(rs);
        if (rs.cr_built > 0)
        {
            st_.cr_epochs++;
            if (rs.cr_too_few)
                st_.cr_too_few++;
        }
        st_.cr_built += static_cast<uint32_t>(rs.cr_built);
        st_.cr_unmatched += static_cast<uint32_t>(rs.cr_unmatched);
        st_.cr_in += static_cast<uint32_t>(rs.cr_in);
        st_.cr_half += static_cast<uint32_t>(rs.cr_half);
        st_.cr_repaired += static_cast<uint32_t>(rs.cr_repaired);
        st_.cr_other += static_cast<uint32_t>(rs.cr_other);
        st_.cr_used += static_cast<uint32_t>(rs.used_cr);
        if (rs.cr_in > 0)
        {
            st_.cr_last_rms_mps = rs.cr_rms_mps;
            st_.cr_last_fit_rms_mps = rs.cr_fit_rms_mps;
            st_.cr_last_dv_mps = rs.cr_dv_mps;
            st_.cr_last_common_mps = rs.cr_common_mps;
            st_.cr_last_roll_rev_s = rs.cr_roll_rev_s;
        }
    }
    double r_post[3], v_post[3];
    ekf_.receiverEcefAt(age, r_post, v_post);
    satcheck_.commit(built_.meas, n, built_.tow_s, r_post);
    st_.last = rs;
    st_.last_age_s = age;
    st_.last_meas = n;
    st_.last_tow_ms = static_cast<uint32_t>(std::llround(std::fmod(built_.tow_s, 604800.0) * 1000.0));
    st_.epochs_applied++;
    // level_on_fused_epochs: an epoch that fused nothing says nothing of the
    // velocity's steadiness, nor that there are satellites (the outage)
    if (!cfg_.level_on_fused_epochs || rs.used_pr + rs.used_rr > 0)
        level_gate_.onEpoch(t, ekf_.velNed());
    if (rs.used_pr > 0)
    {
        own_fix_ = true;
        last_aided_t_ = t;
    }
    // pr_period_s: its pseudoranges went through the update (fused or
    // refused), the period's code -- an epoch whose clock could not be taken
    // fused none, and one dropped never comes here
    if (cfg_.pr_period_s > 0.0 && job_.fused > 0)
        pr_period_k_ = job_.pr_k;
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
        fit_last_te_ = -1.0;
        launch_seen_ = true;
        fit_flight_ = fit_.started() && t - fit_.t0() <= cfg_.heading_seed_max_s;
        handed_ = false;
        if (fit_flight_)
            st_.heading_flights++;
        // the heading the pad leaves (#1590): its own 1-sigma, and whether a
        // magnetometer heading was ever fused -- if not, the heading is the
        // start's guess, and act's prior below is claimed about it. (A
        // shadow's pad heading is the flight filter's, magnetometer and all.)
        float sig[3];
        ekf_.attSigmaNed(sig);
        st_.heading_launch_sigma_deg = static_cast<float>(static_cast<double>(sig[2]) * kRad2Deg);
        if (cfg_.primary && mag_fused_run_ == 0)
            st_.heading_unset_launches++;
        // the pad's heading is the magnetometer's: a sample through its gates
        // fused within heading_mag_recent_s of this edge (RawGnssShadow.h)
        const bool primary_mag = cfg_.primary && mag_fused_run_ > 0;
        st_.heading_launch_mag_age_s = primary_mag ? static_cast<float>(t - last_mag_fused_t_) : -1.0f;
        const bool mag_heading = primary_mag && t - last_mag_fused_t_ <= cfg_.heading_mag_recent_s;
        if (cfg_.heading_mode == kHeadingAct)
        {
            // from here GNSS turns the heading, and only the heading -- or,
            // with the full gate from launch, the whole attitude (from the
            // apogee call: below) -- from the launch prior
            setFlightGate(cfg_.heading_gate == kGateFullLaunch);
            const double prior = mag_heading ? cfg_.heading_pad_sigma_deg : cfg_.heading_pad_sigma_fallback_deg;
            ekf_.setHeadingSigma(static_cast<float>(prior / kRad2Deg));
            st_.heading_act_launches++;
            if (mag_heading)
                st_.heading_mag_launches++;
            st_.heading_launch_prior_deg = static_cast<float>(prior);
            st_.heading_launch_mag = mag_heading ? 1 : 0;
            // and, if asked, the tilt let go to at least its floor
            if (cfg_.heading_launch_tilt_sigma_deg > 0.0)
                ekf_.setTiltSigmaFloor(static_cast<float>(cfg_.heading_launch_tilt_sigma_deg / kRad2Deg));
        }
    }
    else if (ground && !fit_ground_)
    {
        fit_flight_ = false;              // back on the ground
        if (cfg_.heading_mode == kHeadingAct)
            setGroundGate();
    }
    // Act, kGateFullApogee: the flight computer's apogee call opens the whole
    // attitude to GNSS, latched to the ground. Through the ascent the gate is
    // the heading's, so a wrong heading cannot reach the tilt (on the rig the
    // full gate there pulled gross magnetometer errors into 1-2 deg of pitch);
    // under the drogue GNSS corrects the tilt as well. Only the gate changes:
    // the prior, the tilt floor and the hand-off are the launch edge's.
    if (cfg_.heading_mode == kHeadingAct && cfg_.heading_gate == kGateFullApogee && !ground && !gate_full_
        && (imu_flags & kShadowApogee))
    {
        setFlightGate(true);
        st_.heading_gate_apogee++;
        st_.heading_gate_apogee_s = launch_seen_ ? static_cast<float>(t - fit_launch_t_) : -1.0f;
    }
    fit_ground_ = ground;
    const uint32_t us = static_cast<uint32_t>(now() - c0);
    if (us > st_.heading_step_us_max)
        st_.heading_step_us_max = us;
}

// Act's gate in flight: the whole attitude (neither the vertical gate nor
// cos^4), or the heading only (the vertical gate wins over cos^4, which is
// left the config's).
void RawGnssShadow::setFlightGate(bool full)
{
    ekf_.setAttGateVertical(!full);
    ekf_.setAttGateCos4(full ? false : cfg_.ekf.att_gate_cos4);
    gate_full_ = full;
}

// The ground's: the EKF config's.
void RawGnssShadow::setGroundGate()
{
    ekf_.setAttGateVertical(cfg_.ekf.att_gate_vertical);
    ekf_.setAttGateCos4(cfg_.ekf.att_gate_cos4);
    gate_full_ = false;
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
    // at most one epoch a heading_fit_period_s: the fit holds 64, a minute of
    // the LC86G's, 6.4 s at 10 Hz
    if (cfg_.heading_fit_period_s > 0.0 && fit_last_te_ >= 0.0
        && job_.te - fit_last_te_ < cfg_.heading_fit_period_s - kFitPeriodSlackS)
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
    fit_last_te_ = job_.te;
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
