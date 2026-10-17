// raw_nav_adapter.h — GpsInsEKF's face on the raw-GNSS filter's state: the
// Beetle's navigation filter (#1600, TR_RAW_GNSS_PRIMARY).
//
// On the Beetle the raw-GNSS filter (RawGnssShadow on core 0, Config::primary)
// is the only navigation filter; GpsInsEKF does not run. main.cpp's flight
// logic reads `ekf` everywhere -- the apogee and landing voters, the GNSS
// ascent gate, the transonic barometer lockout, NonSensorData, the sensor
// health, the snapshot a reboot restores -- so on the Beetle `ekf` is this:
// GpsInsEKF's methods, answered from the raw filter's state, which it
// publishes every step (ShadowLink::nav; refresh() takes the latest once a
// loop pass). The calls that would configure or update GpsInsEKF become what
// the raw filter needs to start: init() and setQuaternion() are where and how
// it starts on the pad, with the still gyro's mean; setDeclination() and
// setMagReference() its north and the field's size; update() hands it the
// magnetometer (ShadowLink::mag), each sample marked with whether its hard
// iron is out (setMagCalibration, #1590); setState() a restored flight it
// starts from in flight. main.cpp's seed publisher carries them (startSeed).
// The IMU, the barometer and the landing flag reach the raw filter from
// main.cpp as before.
//
// Heights: GpsInsEKF's, and so what main.cpp reads (ref_alt_m, imu_pos), are
// the receiver's -- above the geoid. The raw filter's are above the
// ellipsoid; the receiver's geoid separation (setGeoidSeparation) turns one
// into the other. The snapshot a reboot restores keeps the raw filter's own
// (ellipsoidal) height: it only ever comes back to the raw filter, and a
// receiver cold-started by the reboot has no separation to give for a while.
//
// Brought forward to now (#1626 K2). The raw filter publishes the state at
// the IMU tick of its last step, and the flight loop read it 22-48 ms behind
// the IMU on the rig (the task's period, the 100 Hz steps, the IMU's bursts,
// the epochs' slices). GpsInsEKF, in the loop, was current every tick. So each
// IMU tick main.cpp hands the raw filter comes here too (noteImu), and the
// state the loop reads is the published one carried forward through the ticks
// since: each tick a strapdown step as the raw filter takes its own
// (TR_RawGnssEKF::propagate) -- the attitude turned by the body rate less the
// gyro bias, the specific force less the accelerometer bias turned to NED with
// gravity added, the velocity, then the position from it. Its biases and
// covariance are the filter's, as published. A new state from the filter
// starts it over (refresh); a tick on its own carries it one more. A state
// older than the ticks kept here is read as published (forwardShort).
//
// Healthy (isHealthy) while the raw filter runs, its state is finite, it has
// published within kMaxStaleUs, and the state it published is no older than
// that either (#1626 N4): a filter that has stopped stepping -- republishing
// the same state -- is not healthy, however often it says so, and the
// forward never carries more than that on the IMU alone.
//
// The mosaic-only image (TR_RAW_NAV_MOSAIC: the raw filter is the Goblin's
// one navigation filter, with no GpsInsEKF to fall back on) adds, under its
// own guards so the Beetle's adapter is unchanged: the geoid separation for
// the snapshot, whether the filter has run (the transonic lockout), a
// restore's start height rebased to the barometer and its accelerometer bias,
// and a stop in flight held over as the last good state, carried forward,
// to restart from (setHoldLastGood, restartFromLastGood).
//
// Header-only, host-tested (tests_cpp/test_raw_nav_adapter.cpp; the mosaic
// additions in tests_cpp/test_raw_nav_adapter_mosaic.cpp).
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

#include <TR_GpsInsEKF.h>   // the data types main.cpp hands `ekf` and takes back

#include "ShadowLink.h"

class RawNavAdapter
{
public:
    static constexpr uint32_t kMaxStaleUs = 200000;   // no new state, or none this young: not healthy
    static constexpr float kRad2Deg = 57.29577951f;
    static constexpr int kTicks = 128;                // ticks kept: 0.29 s at the ~440 a second the filter is fed

    void attach(gnss_raw::ShadowLink* link) { link_ = link; }

    // Once a loop pass, before the state is read: the raw filter's latest.
    // now_us is the loop's clock (micros()), for its freshness.
    void refresh(uint32_t now_us)
    {
#if TR_RAW_NAV_MOSAIC
        if (hold_last_good_)
        {
            refreshHolding(now_us);   // a stop in flight keeps the last good state
            return;
        }
#endif
        now_us_ = now_us;
        if (link_ == nullptr)
            return;
        gnss_raw::ShadowNav n;
        uint32_t seq = 0;
        if (link_->nav.read(n, seq) && seq != nav_seq_)
        {
            nav_seq_ = seq;
            nav_ = n;
            nav_rx_us_ = now_us;
            if (n.running)
                restore_pending_ = false;   // it runs: from the pad or the restore
            forwardFromPublished();
#if TR_RAW_NAV_MOSAIC
            if (n.running)
            {
                has_run_ = true;
                held_ = false;              // running again: its own state from here
            }
#endif
        }
        // how old the state the loop now reads is, and how far it was brought
        // forward from the filter's (#1600: an epoch fused in one pass held
        // the filter's 40 ms on the rig)
        if (nav_.running)
        {
            const uint32_t behind = now_us - nav_.time_us;
            if (behind < 0x80000000u && behind > max_fwd_us_)
                max_fwd_us_ = behind;
            const uint32_t age = now_us - (fwd_ok_ ? fwd_t_us_ : nav_.time_us);
            if (age < 0x80000000u && age > max_age_us_)
                max_age_us_ = age;
        }
    }
    // The oldest state the loop was handed since the last call, and the
    // furthest behind the filter's own state was (diagnostics; read and
    // cleared from the raw filter's task, a benign race).
    uint32_t takeMaxAgeUs()
    {
        const uint32_t a = max_age_us_;
        max_age_us_ = 0;
        return a;
    }
    uint32_t takeMaxForwardUs()
    {
        const uint32_t a = max_fwd_us_;
        max_fwd_us_ = 0;
        return a;
    }
    // States read as published: older than the ticks kept.
    uint32_t forwardShort() const { return fwd_short_; }
    void setGeoidSeparation(float sep_m) { sep_m_ = sep_m; }

    // One IMU tick main.cpp hands the raw filter (rawShadowPushImu): kept,
    // and the state carried forward through it. A tick no later than the
    // last kept is a repeat (the raw filter skips those too) or time run back:
    // not kept, not stepped.
    void noteImu(const gnss_raw::ShadowImu& s)
    {
        if (imu_n_ > 0 && (int32_t)(s.time_us - imu_[(imu_head_ + kTicks - 1) % kTicks].t_us) <= 0)
            return;
        Tick& k = imu_[imu_head_];
        k.t_us = s.time_us;
        for (int i = 0; i < 3; ++i)
        {
            k.acc[i] = s.acc[i];
            k.gyro[i] = s.gyro[i];
        }
        imu_head_ = (imu_head_ + 1) % kTicks;
        if (imu_n_ < kTicks)
            imu_n_++;
        if (fwd_ok_ && (int32_t)(k.t_us - fwd_t_us_) > 0)
            forwardTick(k);
    }

    // The raw filter runs, has said so lately, and what it said is recent.
    bool running() const
    {
#if TR_RAW_NAV_MOSAIC
        if (held_)
            return false;   // the last good state, held over a stop: not the filter's
#endif
        if (nav_.running == 0 || (uint32_t)(now_us_ - nav_rx_us_) > kMaxStaleUs)
            return false;
        const int32_t age = (int32_t)(now_us_ - nav_.time_us);   // a state stamped after the loop's clock: fresh
        return age <= (int32_t)kMaxStaleUs;
    }

    // ---------------------------------------------------- configuration

    void setShockGateSettle(uint32_t) {}
    void setAttitudeCovariance(float, float) {}   // the raw filter's own start covariance
    void setGpsNoiseScale(float) {}               // it takes no receiver fix
    void setNoseFirstFlight(bool) {}
    void setGnssVerticalHeldOut(bool) {}

    // Where the raw filter starts on the pad: the receiver's position (MSL)
    // and velocity, and the gyro's mean (the init tick's gyro, deg/s). A new
    // start generation: the raw filter starts over from it.
    void init(EkfIMUData imu, EkfGNSSDataLLA gnss, EkfMagData)
    {
        start_lla_[0] = gnss.lat_rad;
        start_lla_[1] = gnss.lon_rad;
        start_lla_[2] = gnss.alt_m + sep_m_;   // the receiver's, above the geoid: to the ellipsoid
        start_vel_[0] = gnss.vel_n_mps;
        start_vel_[1] = gnss.vel_e_mps;
        start_vel_[2] = gnss.vel_d_mps;
        static constexpr float kDeg2Rad = 0.01745329252f;
        start_wb_[0] = (float)imu.gyro_x * kDeg2Rad;
        start_wb_[1] = (float)imu.gyro_y * kDeg2Rad;
        start_wb_[2] = (float)imu.gyro_z * kDeg2Rad;
        start_q_[0] = 1.0f;
        start_q_[1] = start_q_[2] = start_q_[3] = 0.0f;
        restore_pending_ = false;
#if TR_RAW_NAV_MOSAIC
        has_start_ab_ = false;
        if (held_)
        {
            // back on the pad (a sim's reset): the start's, not a flight's held state
            held_ = false;
            nav_ = gnss_raw::ShadowNav{};
            fwd_ok_ = false;
        }
#endif
        start_gen_++;
    }
    // The attitude it starts from (the accelerometer's tilt, a heading guess).
    void setQuaternion(float q0, float q1, float q2, float q3)
    {
        start_q_[0] = q0;
        start_q_[1] = q1;
        start_q_[2] = q2;
        start_q_[3] = q3;
    }
    void setDeclination(float dec_rad)
    {
        decl_rad_ = dec_rad;
        have_decl_ = true;
    }
    float getDeclination() const { return decl_rad_; }
    // The model's field at the pad (NED, uT): its size, and its dip below the
    // horizontal (#1590), for the raw filter's magnetometer gates.
    void setMagReference(const float ned_uT[3])
    {
        mag_ref_ut_ = std::sqrt(ned_uT[0] * ned_uT[0] + ned_uT[1] * ned_uT[1] + ned_uT[2] * ned_uT[2]);
        mag_ref_valid_ = std::isfinite(mag_ref_ut_) && mag_ref_ut_ > 20.0f && mag_ref_ut_ < 70.0f;
        mag_dip_rad_ = std::atan2(ned_uT[2], std::sqrt(ned_uT[0] * ned_uT[0] + ned_uT[1] * ned_uT[1]));
    }
    bool  getMagReferenceValid() const { return mag_ref_valid_; }
    float getMagReferenceTotal_uT() const { return mag_ref_ut_; }
    // the last magnetometer sample's size against the model's (#1304)
    bool  getMagCalSuspect() const { return mag_suspect_; }

    // Whether the magnetometer's hard iron is out, for the samples update()
    // hands on from here (#1590): a stored calibration is applied, or the
    // field is a sensor sim's or the rig replay's -- the truth's. main.cpp
    // says so each pass; the raw filter takes no heading from a sample with
    // neither.
    void setMagCalibration(bool hard_iron_applied, bool sim_field)
    {
        mag_flags_ = (uint8_t)((hard_iron_applied ? gnss_raw::kShadowMagCalibrated : 0)
                               | (sim_field ? gnss_raw::kShadowMagSim : 0));
    }

    // The magnetometer, each new sample, to the raw filter.
    void update(bool, EkfIMUData, EkfGNSSDataLLA, EkfMagData mag)
    {
        ticks_++;
        if (mag.time_us == 0 || mag.time_us == mag_time_us_)
            return;
        mag_time_us_ = mag.time_us;
        const float m[3] = {(float)mag.mag_x, (float)mag.mag_y, (float)mag.mag_z};
        if (mag_ref_valid_)
        {
            const float mm = std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
            mag_suspect_ = std::fabs(mm - mag_ref_ut_) > 0.15f * mag_ref_ut_;
        }
        if (link_ != nullptr)
        {
            gnss_raw::ShadowMag s;
            s.time_us = mag.time_us;
            for (int i = 0; i < 3; ++i)
                s.mag[i] = m[i];
            s.flags = mag_flags_;
            link_->mag.push(s);
        }
    }
    void baroMeasUpdate(EkfBaroData) {}                 // main.cpp hands the raw filter each sample
    void landedZeroVelocityUpdate(bool, uint32_t) {}    // the landing flag rides each IMU tick

    // A restored flight (the snapshot's state, its height the raw filter's:
    // getState): the raw filter starts from it in flight.
    void setState(const EkfStateSnapshot& s)
    {
        for (int i = 0; i < 3; ++i)
        {
            start_lla_[i] = s.pos_rrm[i];
            start_vel_[i] = s.vel_ned_mps[i];
            start_wb_[i] = s.gyro_bias[i];
        }
        for (int i = 0; i < 4; ++i)
            start_q_[i] = s.quat[i];
#if TR_RAW_NAV_MOSAIC
        // and its accelerometer bias, for the restore's seed (startSeed)
        for (int i = 0; i < 3; ++i)
            start_ab_[i] = s.accel_bias[i];
        has_start_ab_ = true;
#endif
        restore_pending_ = true;
    }
    void setCovFromDiag(const float (&)[15]) {}
    void inflateCovDiag(int, float) {}

    // The seed main.cpp publishes (ShadowLink::seed) before the raw filter runs: where and how to start, or the
    // flight to restore. Height above the ellipsoid. Once it runs it ignores the attitude and position here.
    void startSeed(gnss_raw::ShadowSeed& s) const
    {
        for (int i = 0; i < 3; ++i)
        {
            s.lla[i] = start_lla_[i];
            s.vel_ned[i] = start_vel_[i];
            s.gyro_bias[i] = start_wb_[i];
        }
        for (int i = 0; i < 4; ++i)
            s.quat[i] = start_q_[i];
        s.declination_rad = decl_rad_;
        s.have_declination = have_decl_ ? 1 : 0;
        s.field_ut = mag_ref_valid_ ? mag_ref_ut_ : 0.0f;
        s.dip_rad = mag_ref_valid_ ? mag_dip_rad_ : 0.0f;
        s.have_dip = mag_ref_valid_ ? 1 : 0;
        s.restore = restore_pending_ ? 1 : 0;
        s.start_gen = start_gen_;
#if TR_RAW_NAV_MOSAIC
        // a restored flight's accelerometer bias: the snapshot's, or the last
        // good state's (restartFromLastGood) -- RawGnssShadow::Config::
        // restore_seed_bias takes it in place of zero
        s.has_accel_bias = (restore_pending_ && has_start_ab_) ? 1 : 0;
        for (int i = 0; i < 3; ++i)
            s.accel_bias[i] = s.has_accel_bias ? start_ab_[i] : 0.0f;
#endif
    }
    bool restorePending() const { return restore_pending_; }

#if TR_RAW_NAV_MOSAIC
    // ------------------------------- the mosaic-only image's one filter (D2)

    // The receiver's geoid separation the heights are turned by. The snapshot
    // carries it (FlightSnapshotData::reserved_v5), so a reboot whose receiver
    // has none yet -- or is left out after a crash -- turns them as before.
    float geoidSeparation() const { return sep_m_; }

    // Whether the raw filter has published a running state this boot. Until
    // it has, getVelEst is where it will start -- after a restore the
    // snapshot's velocity, seconds old -- not a speed anything measured.
    bool hasRun() const { return has_run_; }

    // A restored flight starts at the height the barometer gives now (h_msl,
    // above the geoid, as GpsInsEKF's), not the snapshot's: while the restore
    // waits for its start, and only then.
    void rebaseRestoreHeight(double h_msl)
    {
        if (restore_pending_)
            start_lla_[2] = h_msl + sep_m_;
    }

    // In flight, a stop publishes "not running", and on its own the adapter
    // would then answer the start's state -- the pad's, or a restore's. With
    // this on, a stop after a running state in flight keeps that last good
    // state and carries it forward on each IMU tick (noteImu) as a free INS:
    // the voters see it unhealthy (running() and isHealthy() false), the
    // snapshot and the landing checks see the flight's state, not the pad's.
    // It ends when the filter runs again, or init() starts over on the pad.
    // Only a finite state is held: a last state with a non-finite position,
    // velocity, attitude or bias -- or one the forward has since made so --
    // is not a good one, and the stop then reads as on the Beetle (the
    // start's state), with nothing held to restart from.
    void setHoldLastGood(bool on) { hold_last_good_ = on; }
    bool stoppedInFlight() const { return held_; }

    // Starts the raw filter again from the state held: the restore path, as
    // a reboot's snapshot would (setState), with its accelerometer bias.
    // False, and nothing changed, when that state is not finite: a seed the
    // raw filter would start from as given. Field by field, not through an
    // EkfStateSnapshot (its 15 x 15 covariance is ~1 KB of the flight
    // task's stack, and a restart needs none of it).
    bool restartFromLastGood()
    {
        double p[3];
        float v[3], q[4];
        getPosEst(p);
        p[2] += sep_m_;   // the raw filter's own height, as getState gives a snapshot
        getVelEst(v);
        getQuaternion(q);
        float wb[3], ab[3];
        getRotRateBias(wb);
        getAccelBias(ab);
        for (int i = 0; i < 3; ++i)
            if (!std::isfinite(p[i]) || !std::isfinite(v[i]) || !std::isfinite(wb[i]) || !std::isfinite(ab[i]))
                return false;
        for (int i = 0; i < 4; ++i)
            if (!std::isfinite(q[i]))
                return false;
        for (int i = 0; i < 3; ++i)
        {
            start_lla_[i] = p[i];
            start_vel_[i] = v[i];
            start_wb_[i] = wb[i];
            start_ab_[i] = ab[i];
        }
        for (int i = 0; i < 4; ++i)
            start_q_[i] = q[i];
        has_start_ab_ = true;
        restore_pending_ = true;
        return true;
    }
#endif

    // ---------------------------------------------------- the state

    bool isHealthy() const
    {
        if (!running())
            return false;
        const double* lla = fwd_ok_ ? fwd_lla_ : nav_.lla;
        const float* v = fwd_ok_ ? fwd_v_ : nav_.vel_ned;
        const float* q = fwd_ok_ ? fwd_q_ : nav_.quat;
        for (int i = 0; i < 3; ++i)
            if (!std::isfinite(lla[i]) || !std::isfinite(v[i]))
                return false;
        for (int i = 0; i < 4; ++i)
            if (!std::isfinite(q[i]))
                return false;
        return true;
    }
    // lat, lon (rad) and height above the geoid (m), as GpsInsEKF's, brought
    // forward to the latest tick; before it runs, where it will start
    void getPosEst(double (&r)[3]) const
    {
        if (nav_.running)
        {
            const double* lla = fwd_ok_ ? fwd_lla_ : nav_.lla;
            r[0] = lla[0];
            r[1] = lla[1];
            r[2] = lla[2] - sep_m_;
        }
        else
        {
            r[0] = start_lla_[0];
            r[1] = start_lla_[1];
            r[2] = start_lla_[2] - sep_m_;
        }
    }
    void getVelEst(float (&r)[3]) const
    {
        for (int i = 0; i < 3; ++i)
            r[i] = nav_.running ? (fwd_ok_ ? fwd_v_[i] : nav_.vel_ned[i]) : start_vel_[i];
    }
    void getQuaternion(float (&r)[4]) const
    {
        for (int i = 0; i < 4; ++i)
            r[i] = nav_.running ? (fwd_ok_ ? fwd_q_[i] : nav_.quat[i]) : start_q_[i];
    }
    // roll, pitch, yaw (rad): GpsInsEKF::Quat2Euler
    void getOrientEst(float (&r)[3]) const
    {
        float q[4];
        getQuaternion(q);
        r[0] = std::atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
        const float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
        r[1] = std::fabs(sinp) >= 1.0f ? std::copysign(1.5707963f, sinp) : std::asin(sinp);
        r[2] = std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
    }
    void getRotRateBias(float (&r)[3]) const
    {
        for (int i = 0; i < 3; ++i)
            r[i] = nav_.running ? nav_.gyro_bias[i] : start_wb_[i];
    }
    void getAccelBias(float (&r)[3]) const
    {
        for (int i = 0; i < 3; ++i)
            r[i] = nav_.accel_bias[i];
    }
    void getCovVel(float (&r)[3]) const { block(3, r); }
    void getCovOrient(float (&r)[3]) const { block(6, r); }
    void getCovDiag(float (&out)[15]) const
    {
        for (int i = 0; i < 15; ++i)
            out[i] = nav_.var[i];
    }
    // For the snapshot: the height the raw filter's own, above the ellipsoid (above).
    void getState(EkfStateSnapshot& s) const
    {
        std::memset(&s, 0, sizeof(s));
        double p[3];
        float v[3], q[4], e[3];
        getPosEst(p);
        p[2] += sep_m_;
        getVelEst(v);
        getQuaternion(q);
        getOrientEst(e);
        for (int i = 0; i < 3; ++i)
        {
            s.pos_rrm[i] = p[i];
            s.vel_ned_mps[i] = v[i];
            s.accel_bias[i] = nav_.accel_bias[i];
            s.gyro_bias[i] = nav_.gyro_bias[i];
            s.euler[i] = e[i];
        }
        for (int i = 0; i < 4; ++i)
            s.quat[i] = q[i];
        for (int i = 0; i < 15; ++i)
            s.P[i][i] = nav_.var[i];
        s.t_prev_us = fwd_ok_ ? fwd_t_us_ : nav_.time_us;
#if TR_RAW_NAV_MOSAIC
        // Before the raw filter runs (a restore's wait for the barometer, a
        // boot with no filter) its position, velocity and attitude above are
        // where it will start; so are its biases, not the zeros a filter that
        // is not running publishes. A snapshot taken then -- the OC holds the
        // latest through the next reboot -- keeps the restored biases.
        if (!nav_.running)
        {
            for (int i = 0; i < 3; ++i)
            {
                s.gyro_bias[i] = start_wb_[i];
                if (has_start_ab_)
                    s.accel_bias[i] = start_ab_[i];
            }
        }
#endif
    }

    // The raw filter has no shock gate, canopy mode or gyro-bias gate: none trips.
    bool     underCanopy() const { return false; }
    uint32_t shockGateTrips() const { return 0; }
    uint32_t shockGateHoldTicks() const { return 0; }
    bool     shockGateHeld() const { return false; }
    uint32_t gyroBiasGateTrips() const { return 0; }
    uint32_t gyroBiasClipCount() const { return 0; }
    float gyroBiasMaxDps() const
    {
        float m = 0.0f;
        for (int i = 0; i < 3; ++i)
            m = std::fmax(m, std::fabs(nav_.gyro_bias[i]));
        return m * kRad2Deg;
    }
    float gyroBiasSigmaMaxDps() const
    {
        float m = 0.0f;
        for (int i = 12; i < 15; ++i)
            m = std::fmax(m, nav_.var[i]);
        return std::sqrt(m) * kRad2Deg;
    }
    bool gyroBiasHealthy(float max_dps = 3.0f, float max_sigma_dps = 2.0f) const
    {
        return gyroBiasMaxDps() <= max_dps && gyroBiasSigmaMaxDps() <= max_sigma_dps;
    }

    // the raw filter's own
    bool gnssAided() const { return running() && nav_.gnss_aided != 0; }
    bool baroAided() const { return running() && nav_.baro_aided != 0; }
    uint32_t epochs() const { return nav_.epochs; }
    uint32_t ticks() const { return ticks_; }

private:
#if TR_RAW_NAV_MOSAIC
    // refresh() with the hold on: one read of the link, as refresh() makes,
    // with a stop published after a running state in flight taken (its
    // sequence moves on) but not read in -- the state the loop reads stays
    // the last good one, carried forward through each tick since. A state
    // the ticks kept no longer reach is carried on from the newest tick. One
    // read, so no stop can slip in between a check and the read it guards.
    // A last state that is not finite is not held, and a held one that the
    // forward has made so is dropped: the stop is then read in as refresh()
    // reads it (the Beetle's: the start's state, nothing to restart from).
    void refreshHolding(uint32_t now_us)
    {
        now_us_ = now_us;
        if (link_ == nullptr)
            return;
        gnss_raw::ShadowNav n;
        uint32_t seq = 0;
        if (link_->nav.read(n, seq) && seq != nav_seq_)
        {
            nav_seq_ = seq;
            if (!n.running && (held_ || (nav_.running && nav_.in_flight)) && lastStateFinite())
            {
                if (!held_)
                {
                    held_ = true;
                    if (!fwd_ok_ && imu_n_ > 0)
                    {
                        fwd_t_us_ = imu_[(imu_head_ + kTicks - 1) % kTicks].t_us;
                        fwd_ok_ = true;
                    }
                }
            }
            else
            {
                // as refresh(): on the ground, the filter running again, or a
                // last state not fit to hold
                held_ = false;
                nav_ = n;
                nav_rx_us_ = now_us;
                if (n.running)
                {
                    restore_pending_ = false;
                    has_run_ = true;
                }
                forwardFromPublished();
            }
        }
        if (nav_.running)
        {
            const uint32_t behind = now_us - nav_.time_us;
            if (behind < 0x80000000u && behind > max_fwd_us_)
                max_fwd_us_ = behind;
            const uint32_t age = now_us - (fwd_ok_ ? fwd_t_us_ : nav_.time_us);
            if (age < 0x80000000u && age > max_age_us_)
                max_age_us_ = age;
        }
    }

    // The state the loop reads now (the published one, or carried forward)
    // and its biases: all finite.
    bool lastStateFinite() const
    {
        const double* lla = fwd_ok_ ? fwd_lla_ : nav_.lla;
        const float* v = fwd_ok_ ? fwd_v_ : nav_.vel_ned;
        const float* q = fwd_ok_ ? fwd_q_ : nav_.quat;
        for (int i = 0; i < 3; ++i)
            if (!std::isfinite(lla[i]) || !std::isfinite(v[i]) || !std::isfinite(nav_.accel_bias[i])
                || !std::isfinite(nav_.gyro_bias[i]))
                return false;
        for (int i = 0; i < 4; ++i)
            if (!std::isfinite(q[i]))
                return false;
        return true;
    }
#endif

    struct Tick
    {
        uint32_t t_us;
        float    acc[3];    // specific force, body FRD, m/s^2
        float    gyro[3];   // body FRD, rad/s
    };

    void block(int i0, float (&r)[3]) const
    {
        for (int i = 0; i < 3; ++i)
            r[i] = nav_.var[i0 + i];
    }

    // The filter's new state, carried through the ticks kept since its time.
    void forwardFromPublished()
    {
        fwd_ok_ = false;
        if (!nav_.running)
            return;
        for (int i = 0; i < 3; ++i)
        {
            fwd_lla_[i] = nav_.lla[i];
            fwd_v_[i] = nav_.vel_ned[i];
        }
        for (int i = 0; i < 4; ++i)
            fwd_q_[i] = nav_.quat[i];
        fwd_t_us_ = nav_.time_us;
        // the Earth's radii at the state (TR_RawGnssEKF's earthRad, WGS84):
        // over the forward's fraction of a second they do not change
        const double s = std::sin(nav_.lla[0]);
        const double d = std::fabs(1.0 - 0.0066943799901 * s * s);
        const double rew = 6378137.0 / std::sqrt(d);
        const double rns = 6378137.0 * (1.0 - 0.0066943799901) / (d * std::sqrt(d));
        inv_rn_ = 1.0 / (rns + nav_.lla[2]);
        inv_re_cos_ = 1.0 / ((rew + nav_.lla[2]) * std::cos(nav_.lla[0]));
        // the ticks after its time, oldest first: none kept that far back
        // means a gap the forward would skip over -- read it as published
        int k = 0;
        while (k < imu_n_ && (int32_t)(imu_[(imu_head_ + kTicks - 1 - k) % kTicks].t_us - nav_.time_us) > 0)
            k++;
        if (k == kTicks)
        {
            fwd_short_++;
            return;
        }
        fwd_ok_ = true;
        for (int j = k - 1; j >= 0; --j)
            forwardTick(imu_[(imu_head_ + kTicks - 1 - j) % kTicks]);
    }

    // One tick, as the raw filter steps (TR_RawGnssEKF::propagate): the
    // attitude at its start turns the specific force; the attitude is then
    // turned by the rate over the tick (a rotation vector, exact for a constant
    // rate); the velocity, then the position from it.
    void forwardTick(const Tick& k)
    {
        const float dt = (float)(int32_t)(k.t_us - fwd_t_us_) * 1.0e-6f;
        const float* q = fwd_q_;
        const float w = q[0], x = q[1], y = q[2], z = q[3];
        // NED -> body (TR_RawGnssEKF's quat2dcm); its transpose turns body to NED
        const float M[3][3] = {{1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y + w * z), 2.0f * (x * z - w * y)},
                               {2.0f * (x * y - w * z), 1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z + w * x)},
                               {2.0f * (x * z + w * y), 2.0f * (y * z - w * x), 1.0f - 2.0f * (x * x + y * y)}};
        float f[3], a[3];
        for (int i = 0; i < 3; ++i)
            f[i] = k.acc[i] - nav_.accel_bias[i];
        for (int i = 0; i < 3; ++i)
            a[i] = M[0][i] * f[0] + M[1][i] * f[1] + M[2][i] * f[2];
        a[2] += kGravity;
        const float th[3] = {(k.gyro[0] - nav_.gyro_bias[0]) * dt, (k.gyro[1] - nav_.gyro_bias[1]) * dt,
                             (k.gyro[2] - nav_.gyro_bias[2]) * dt};
        const float ang = std::sqrt(th[0] * th[0] + th[1] * th[1] + th[2] * th[2]);
        float dq[4];
        if (ang > 1e-8f)
        {
            const float sh = std::sin(0.5f * ang) / ang;
            dq[0] = std::cos(0.5f * ang);
            dq[1] = sh * th[0];
            dq[2] = sh * th[1];
            dq[3] = sh * th[2];
        }
        else
        {
            dq[0] = 1.0f;
            dq[1] = 0.5f * th[0];
            dq[2] = 0.5f * th[1];
            dq[3] = 0.5f * th[2];
        }
        float qn[4] = {w * dq[0] - x * dq[1] - y * dq[2] - z * dq[3], w * dq[1] + x * dq[0] + y * dq[3] - z * dq[2],
                       w * dq[2] - x * dq[3] + y * dq[0] + z * dq[1], w * dq[3] + x * dq[2] - y * dq[1] + z * dq[0]};
        const float n = std::sqrt(qn[0] * qn[0] + qn[1] * qn[1] + qn[2] * qn[2] + qn[3] * qn[3]);
        for (int i = 0; i < 4; ++i)
            fwd_q_[i] = qn[i] / n;
        for (int i = 0; i < 3; ++i)
            fwd_v_[i] += a[i] * dt;
        const double dts = (double)dt;
        fwd_lla_[0] += dts * (double)fwd_v_[0] * inv_rn_;
        fwd_lla_[1] += dts * (double)fwd_v_[1] * inv_re_cos_;
        fwd_lla_[2] -= dts * (double)fwd_v_[2];
        fwd_t_us_ = k.t_us;
    }

    static constexpr float kGravity = 9.807f;   // the raw filter's (gravity_wgs84 off: the flight filter's G_NOMINAL)

    gnss_raw::ShadowLink* link_ = nullptr;
    gnss_raw::ShadowNav nav_ = {};
    uint32_t nav_seq_ = 0;
    uint32_t nav_rx_us_ = 0;
    volatile uint32_t max_age_us_ = 0;
    volatile uint32_t max_fwd_us_ = 0;
    // the ticks since, and the state carried forward through them
    Tick     imu_[kTicks] = {};
    int      imu_head_ = 0, imu_n_ = 0;
    bool     fwd_ok_ = false;
    uint32_t fwd_t_us_ = 0;          // the last tick in it
    double   fwd_lla_[3] = {};
    float    fwd_v_[3] = {};
    float    fwd_q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    double   inv_rn_ = 0.0, inv_re_cos_ = 0.0;
    uint32_t fwd_short_ = 0;
    uint32_t now_us_ = 0;
    float    sep_m_ = 0.0f;
    // the start
    double   start_lla_[3] = {};
    float    start_vel_[3] = {};
    float    start_q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float    start_wb_[3] = {};
    uint8_t  start_gen_ = 0;
    bool     restore_pending_ = false;
    // the magnetometer
    float    decl_rad_ = 0.0f;
    bool     have_decl_ = false;
    float    mag_ref_ut_ = 0.0f;
    float    mag_dip_rad_ = 0.0f;
    bool     mag_ref_valid_ = false;
    bool     mag_suspect_ = false;
    uint8_t  mag_flags_ = 0;         // gnss_raw::kShadowMag*: none until main.cpp says
    uint32_t mag_time_us_ = 0;
    uint32_t ticks_ = 0;
#if TR_RAW_NAV_MOSAIC
    // the mosaic-only image's one filter
    bool     has_run_ = false;        // a running state seen this boot
    bool     hold_last_good_ = false;
    bool     held_ = false;           // a stop in flight: the last good state, carried forward
    bool     has_start_ab_ = false;   // a restore's accelerometer bias (setState)
    float    start_ab_[3] = {};
#endif
};
