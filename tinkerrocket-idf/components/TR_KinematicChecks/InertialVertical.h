// InertialVertical: the IMU's own vertical velocity, height and pitch, for the
// apogee vote's accelerometer and gyro voters (owner's design, 2026-11-19).
//
// WHY IT EXISTS.  The vote counts sensors, not filters.  The navigation
// filter's velocity fuses the accelerometer with GNSS and the barometer, so
// beside the GPS and barometer voters it is the same receiver or the same
// port voting twice.  This is the accelerometer alone, rotated by the gyro
// alone: no GNSS, no barometer, no filter.  Its pitch is the gyro alone.
//
// HOW.  On the pad it levels itself on still stretches of 250 ms (a "rest
// block": the specific force's size and the gyro barely move), taking the
// attitude from the mean specific force, the gyro bias from the mean rate and
// g from the mean's size, and zeroing velocity and height.  Between rest
// blocks it integrates, so when the motor lights it is already integrating
// from the last still moment, whatever the launch detector's latency.  At
// launch (launch()) it stops levelling and integrates for the rest of the
// flight:
//
//     q  <- q * rot((w - bias) dt)           body-to-level attitude
//     vz <- vz + ((R(q) f).up - g(h)) dt     g falls off as (R/(R+h))^2
//     h  <- h + vz dt
//     pitch = asin((R(q) x_body).up)         + nose up, as the filters'
//
// Drag needs no model: the accelerometer measures it.
//
// WHAT IT NEVER DOES.
//   * Never claims health it cannot have.  Healthy only in flight, after a
//     launch() that followed a rest block within kMaxRestAgeUs; a restored
//     flight (an FC reboot in the air) never saw the pad and stays failed.
//   * Never bridges a gap.  A sample more than kMaxGapUs after the last ends
//     it for the flight; so does a non-finite state.
//   * Never integrates a railed gyro.  The attitude is held over a railed
//     sample; more than kMaxRailUs of them in a flight ends it.
//
// Its error grows with the boost's velocity (the high-g channel's scale) and
// the coast's length (bias): about 10 m/s at apogee on a 12.8 km, Mach 2
// flight, about 3 m/s on a 1 km one.  The voters allow for that.
//
// Header-only, host-tested (tests_cpp/test_inertial_vertical.cpp).
#pragma once

#include <cmath>
#include <cstdint>

class InertialVertical
{
public:
    enum class State : uint8_t
    {
        NoRest = 0,   // no rest block seen yet: not levelled
        Pad    = 1,   // levelling on rest blocks, integrating between them
        Flying = 2,   // launched from a recent rest block: healthy
        Failed = 3,   // not usable for the rest of this flight (failReason)
    };

    static constexpr uint32_t kBlockUs       = 250000;    // a rest block
    static constexpr float    kRestRangeMps2 = 3.0f;      // |f| spread in a rest block (rail sway passes, thrust fails)
    static constexpr float    kRestGTolMps2  = 0.6f;      // mean |f| this close to standard g
    static constexpr float    kRestRateDps   = 30.0f;     // gyro spread per axis in a rest block (rail sway)
    static constexpr float    kRestMeanDps   = 5.0f;      // mean |rate| per axis in a rest block (a bias, not a turn)
    static constexpr uint32_t kMaxRestAgeUs  = 5000000;   // launch must follow a rest block within this
    static constexpr uint32_t kMaxGapUs      = 100000;    // in flight, a longer gap between samples ends it
    static constexpr uint32_t kMaxRailUs     = 500000;    // in flight, railed-gyro time that ends it
    static constexpr float    kStdG          = 9.80665f;
    static constexpr float    kEarthR        = 6371000.0f;

    InertialVertical() { reset(); }

    // Forget everything: boot, and a sim's start or stop.
    void reset()
    {
        state_ = State::NoRest;
        fail_ = "";
        have_t_ = false;
        last_t_ = 0;
        q_[0] = 1.0f; q_[1] = q_[2] = q_[3] = 0.0f;
        bias_[0] = bias_[1] = bias_[2] = 0.0f;
        have_bias_ = false;
        g0_ = kStdG;
        vz_ = h_ = 0.0f;
        rest_end_t_ = 0;
        rail_us_ = 0;
        clearBlock();
    }

    // One IMU sample in the body frame (FLU: x out the nose): specific force
    // in m/s^2, rate in deg/s, and whether the gyro railed behind it.
    void update(uint32_t t_us, const float f[3], const float w_dps[3], bool gyro_railed)
    {
        if (state_ == State::Failed)
            return;
        for (int i = 0; i < 3; ++i)
            if (!std::isfinite(f[i]) || !std::isfinite(w_dps[i]))
            {
                if (state_ == State::Flying)
                    fail("a non-finite IMU sample in flight");
                return;   // on the pad: skip it
            }

        const bool flying = (state_ == State::Flying);
        if (!have_t_)
        {
            have_t_ = true;
            last_t_ = t_us;
            if (!flying)
                startBlock(t_us);
            return;
        }
        const uint32_t dt_us = t_us - last_t_;
        last_t_ = t_us;
        if (dt_us == 0)
            return;
        if (flying && dt_us > kMaxGapUs)
        {
            fail("an IMU gap in flight");
            return;
        }

        // On the pad a gap only means the integration since the last rest
        // block is suspect: the next rest block levels it again.
        if (state_ != State::NoRest && dt_us <= kMaxGapUs)
            integrate(dt_us * 1e-6f, f, w_dps, gyro_railed);
        if (flying && rail_us_ > kMaxRailUs)
        {
            fail("the gyro railed too long in flight");
            return;
        }
        if (state_ == State::Flying && !finiteState())
        {
            fail("a non-finite state in flight");
            return;
        }

        if (!flying)
            padBlock(t_us, f, w_dps);
    }

    // Launch: stop levelling and fly.  It must follow a rest block within
    // kMaxRestAgeUs, or the integration since then is not trusted.
    void launch(uint32_t t_us)
    {
        if (state_ == State::Flying || state_ == State::Failed)
            return;
        if (state_ == State::NoRest)
        {
            fail("no still pad moment before launch");
            return;
        }
        if ((uint32_t)(t_us - rest_end_t_) > kMaxRestAgeUs)
        {
            fail("no still pad moment in the 5 s before launch");
            return;
        }
        state_ = State::Flying;
        rail_us_ = 0;
    }

    bool  healthy() const { return state_ == State::Flying; }
    State state() const { return state_; }
    const char* failReason() const { return fail_; }

    float vzUp() const { return vz_; }       // m/s, + up
    float heightM() const { return h_; }     // m above the last rest block (the pad)
    float pitchRad() const
    {
        const float s = 2.0f * (q_[1] * q_[3] - q_[0] * q_[2]);   // (R x_body).up
        return std::asin(s > 1.0f ? 1.0f : (s < -1.0f ? -1.0f : s));
    }
    // The up component of a body-frame vector, (R v).up: pitchRad() is the
    // asin of it for body +X.  #1755: whether the measured thrust axis points
    // up (thrust) or down (coast drag), in whatever frame the board flies.
    float upOf(const float v[3]) const
    {
        const float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
        return 2.0f * (x * z - w * y) * v[0] + 2.0f * (y * z + w * x) * v[1] +
               (1.0f - 2.0f * (x * x + y * y)) * v[2];
    }
    float gravity() const { return g0_; }    // the pad's measured g

private:
    void fail(const char* why)
    {
        state_ = State::Failed;
        fail_ = why;
    }

    bool finiteState() const
    {
        return std::isfinite(vz_) && std::isfinite(h_) && std::isfinite(q_[0]) &&
               std::isfinite(q_[1]) && std::isfinite(q_[2]) && std::isfinite(q_[3]);
    }

    void integrate(float dt, const float f[3], const float w_dps[3], bool gyro_railed)
    {
        static constexpr float kDeg2Rad = 0.01745329252f;
        if (gyro_railed)
            rail_us_ += (uint32_t)(dt * 1e6f);
        else
        {
            const float rx = (w_dps[0] * kDeg2Rad - bias_[0]) * dt;
            const float ry = (w_dps[1] * kDeg2Rad - bias_[1]) * dt;
            const float rz = (w_dps[2] * kDeg2Rad - bias_[2]) * dt;
            const float ang = std::sqrt(rx * rx + ry * ry + rz * rz);
            if (ang > 0.0f)
            {
                const float s = std::sin(0.5f * ang) / ang;
                const float d[4] = {std::cos(0.5f * ang), rx * s, ry * s, rz * s};
                const float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
                q_[0] = w * d[0] - x * d[1] - y * d[2] - z * d[3];
                q_[1] = w * d[1] + x * d[0] + y * d[3] - z * d[2];
                q_[2] = w * d[2] - x * d[3] + y * d[0] + z * d[1];
                q_[3] = w * d[3] + x * d[2] - y * d[1] + z * d[0];
                const float n = std::sqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
                if (n > 0.0f)
                    for (int i = 0; i < 4; ++i)
                        q_[i] /= n;
            }
        }
        // (R(q) f).up: the third row of R(q)
        const float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
        const float up = 2.0f * (x * z - w * y) * f[0] +
                         2.0f * (y * z + w * x) * f[1] +
                         (1.0f - 2.0f * (x * x + y * y)) * f[2];
        const float r = kEarthR / (kEarthR + (h_ > 0.0f ? h_ : 0.0f));
        const float g = g0_ * r * r;
        vz_ += (up - g) * dt;
        h_ += vz_ * dt;
    }

    // ------------------------------------------------------- pad levelling

    void clearBlock()
    {
        blk_n_ = 0;
        blk_t0_ = 0;
        for (int i = 0; i < 3; ++i)
        {
            blk_f_[i] = blk_w_[i] = 0.0f;
            blk_wmin_[i] = 1e9f;
            blk_wmax_[i] = -1e9f;
        }
        blk_fmin_ = 1e9f;
        blk_fmax_ = -1e9f;
    }

    void startBlock(uint32_t t_us)
    {
        clearBlock();
        blk_t0_ = t_us;
    }

    void padBlock(uint32_t t_us, const float f[3], const float w_dps[3])
    {
        const float fm = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
        for (int i = 0; i < 3; ++i)
        {
            blk_f_[i] += f[i];
            blk_w_[i] += w_dps[i];
            if (w_dps[i] < blk_wmin_[i]) blk_wmin_[i] = w_dps[i];
            if (w_dps[i] > blk_wmax_[i]) blk_wmax_[i] = w_dps[i];
        }
        if (fm < blk_fmin_) blk_fmin_ = fm;
        if (fm > blk_fmax_) blk_fmax_ = fm;
        ++blk_n_;
        if ((uint32_t)(t_us - blk_t0_) < kBlockUs)
            return;

        // The block is over: was it still?
        const float inv = 1.0f / (float)blk_n_;
        float mf[3], mw[3];
        for (int i = 0; i < 3; ++i)
        {
            mf[i] = blk_f_[i] * inv;
            mw[i] = blk_w_[i] * inv;
        }
        const float mfn = std::sqrt(mf[0] * mf[0] + mf[1] * mf[1] + mf[2] * mf[2]);
        bool still = blk_n_ >= 8 &&
                     (blk_fmax_ - blk_fmin_) < kRestRangeMps2 &&
                     std::fabs(mfn - kStdG) < kRestGTolMps2;
        for (int i = 0; i < 3 && still; ++i)
            still = (blk_wmax_[i] - blk_wmin_[i]) < kRestRateDps && std::fabs(mw[i]) < kRestMeanDps;
        if (still)
            level(t_us, mf, mfn, mw);
        startBlock(t_us);
    }

    // Level on a rest block: attitude from the mean specific force (it points
    // up), bias from the mean rate, g from the mean's size; still, at the pad.
    void level(uint32_t t_us, const float mf[3], float mfn, const float mw_dps[3])
    {
        static constexpr float kDeg2Rad = 0.01745329252f;
        const float a[3] = {mf[0] / mfn, mf[1] / mfn, mf[2] / mfn};
        // The rotation taking a onto up = (0, 0, 1): axis a x up, cos = a.z
        const float c = a[2];
        if (c < -0.9999f)
        {
            q_[0] = 0.0f; q_[1] = 1.0f; q_[2] = 0.0f; q_[3] = 0.0f;   // upside down: half a turn about x
        }
        else
        {
            const float qw = 1.0f + c, qx = a[1], qy = -a[0], qz = 0.0f;   // (1 + a.up, a x up)
            const float n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
            q_[0] = qw / n; q_[1] = qx / n; q_[2] = qy / n; q_[3] = qz / n;
        }
        for (int i = 0; i < 3; ++i)
            bias_[i] = have_bias_ ? 0.75f * bias_[i] + 0.25f * mw_dps[i] * kDeg2Rad : mw_dps[i] * kDeg2Rad;
        have_bias_ = true;
        g0_ = mfn;
        vz_ = h_ = 0.0f;
        rest_end_t_ = t_us;
        state_ = State::Pad;
    }

    State       state_;
    const char* fail_;
    bool        have_t_;
    uint32_t    last_t_;
    float       q_[4];
    float       bias_[3];
    bool        have_bias_;
    float       g0_;
    float       vz_, h_;
    uint32_t    rest_end_t_;
    uint32_t    rail_us_;

    uint32_t blk_n_, blk_t0_;
    float    blk_f_[3], blk_w_[3], blk_wmin_[3], blk_wmax_[3];
    float    blk_fmin_, blk_fmax_;
};
