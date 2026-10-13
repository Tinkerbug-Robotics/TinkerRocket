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
// magnetometer (ShadowLink::mag); setState() a restored flight it starts from
// in flight. main.cpp's seed publisher carries them (startSeed). The IMU, the
// barometer and the landing flag reach the raw filter from main.cpp as before.
//
// Heights: GpsInsEKF's, and so what main.cpp reads (ref_alt_m, imu_pos), are
// the receiver's -- above the geoid. The raw filter's are above the
// ellipsoid; the receiver's geoid separation (setGeoidSeparation) turns one
// into the other. The snapshot a reboot restores keeps the raw filter's own
// (ellipsoidal) height: it only ever comes back to the raw filter, and a
// receiver cold-started by the reboot has no separation to give for a while.
//
// Healthy (isHealthy) while the raw filter runs, its state is finite and it
// has published within max_stale_us: the voters and the gate stop trusting a
// filter whose core has stalled.
//
// Header-only, host-tested (tests_cpp/test_raw_nav_adapter.cpp).
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

#include <TR_GpsInsEKF.h>   // the data types main.cpp hands `ekf` and takes back

#include "ShadowLink.h"

class RawNavAdapter
{
public:
    static constexpr uint32_t kMaxStaleUs = 200000;   // no new state for this long: not healthy
    static constexpr float kRad2Deg = 57.29577951f;

    void attach(gnss_raw::ShadowLink* link) { link_ = link; }

    // Once a loop pass, before the state is read: the raw filter's latest.
    // now_us is the loop's clock (micros()), for its freshness.
    void refresh(uint32_t now_us)
    {
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
        }
    }
    void setGeoidSeparation(float sep_m) { sep_m_ = sep_m; }

    // The raw filter runs and has said so lately.
    bool running() const { return nav_.running != 0 && (uint32_t)(now_us_ - nav_rx_us_) <= kMaxStaleUs; }

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
    void setMagReference(const float ned_uT[3])
    {
        mag_ref_ut_ = std::sqrt(ned_uT[0] * ned_uT[0] + ned_uT[1] * ned_uT[1] + ned_uT[2] * ned_uT[2]);
        mag_ref_valid_ = std::isfinite(mag_ref_ut_) && mag_ref_ut_ > 20.0f && mag_ref_ut_ < 70.0f;
    }
    bool  getMagReferenceValid() const { return mag_ref_valid_; }
    float getMagReferenceTotal_uT() const { return mag_ref_ut_; }
    // the last magnetometer sample's size against the model's (#1304)
    bool  getMagCalSuspect() const { return mag_suspect_; }

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
        s.restore = restore_pending_ ? 1 : 0;
        s.start_gen = start_gen_;
    }
    bool restorePending() const { return restore_pending_; }

    // ---------------------------------------------------- the state

    bool isHealthy() const
    {
        if (!running())
            return false;
        for (int i = 0; i < 3; ++i)
            if (!std::isfinite(nav_.lla[i]) || !std::isfinite(nav_.vel_ned[i]))
                return false;
        for (int i = 0; i < 4; ++i)
            if (!std::isfinite(nav_.quat[i]))
                return false;
        return true;
    }
    // lat, lon (rad) and height above the geoid (m), as GpsInsEKF's; before
    // it runs, where it will start
    void getPosEst(double (&r)[3]) const
    {
        if (nav_.running)
        {
            r[0] = nav_.lla[0];
            r[1] = nav_.lla[1];
            r[2] = nav_.lla[2] - sep_m_;
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
            r[i] = nav_.running ? nav_.vel_ned[i] : start_vel_[i];
    }
    void getQuaternion(float (&r)[4]) const
    {
        for (int i = 0; i < 4; ++i)
            r[i] = nav_.running ? nav_.quat[i] : start_q_[i];
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
        s.t_prev_us = nav_.time_us;
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
    void block(int i0, float (&r)[3]) const
    {
        for (int i = 0; i < 3; ++i)
            r[i] = nav_.var[i0 + i];
    }

    gnss_raw::ShadowLink* link_ = nullptr;
    gnss_raw::ShadowNav nav_ = {};
    uint32_t nav_seq_ = 0;
    uint32_t nav_rx_us_ = 0;
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
    bool     mag_ref_valid_ = false;
    bool     mag_suspect_ = false;
    uint32_t mag_time_us_ = 0;
    uint32_t ticks_ = 0;
};
