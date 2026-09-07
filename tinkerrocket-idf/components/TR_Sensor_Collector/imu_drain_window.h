// imu_drain_window.h — statistics over the ISM6HG256 samples drained between
// two estimator passes (#1191).
//
// The flight loop pops every queued IMU sample each pass and logs each one, so
// the recorded stream runs at the chip ODR (3840 Hz to deployment under
// IMU_RATE_DYNAMIC, 960 Hz after).  The EKF, the roll controller and the
// kinematics run once per pass, ~500 Hz.  Handing them the LAST drained sample
// is a 1-in-N decimation with no anti-alias filter: the 2026-08-29 J570W boost
// carried an ~800 Hz longitudinal tone at 36-51 m/s^2 RMS (±300 m/s^2 peaks),
// and a ~496 Hz pick folds 800 Hz to ~192 Hz at full amplitude — straight into
// the EKF's near-saturation channel switching and the roll P-term, which has no
// filter of its own.  Averaging the N drained samples is an N-tap boxcar at the
// ODR: -15 dB at 800 Hz for N = 8 (-24 dB for N = 5, close to a null), under
// 1 % change at 30 Hz where the airframe bends, under a millisecond of group
// delay.  N is whatever was drained; a variable N only moves the nulls around.
//
// The mean is a RAW-count mean.  SensorConverter::convertISM6HG256Data is
// affine (per-LSB scale, Z rotation, high-g bias, board->rocket rotation), so
// converting the mean equals averaging the converted samples, and the drain
// loop stays integer adds with no float work per sample.
//
// Per-axis max |raw| rides along so the low-g near-rail test can look at the
// window's WORST sample rather than its mean: an average of clipped samples is
// still biased toward zero, and the switch to the high-g channel must fire if
// any sample in the window touched the rail.  The max is kept per SENSOR axis
// on purpose.  The ±16 g rail is a per-axis property of the chip, and the
// -45 deg mount rotation turns a single-axis rail into 11.3 g on two body axes,
// which a body-frame test at 15.5 g never sees.
//
// No ESP-IDF dependencies: tests_cpp compiles this on the host, which is the
// only place the arithmetic is exercised without a rocket on the bench.
#pragma once

#include <cstdint>
#include "RocketComputerTypes.h"

namespace imu_drain
{
    // Raw-count threshold for "this axis is near its rail": the configured
    // full scale less a margin, in LSB.  The per-LSB scale is FS / 32768 on
    // every ISM6HG256 channel (mirrors SensorConverter), so
    //   (FS - margin) g  ==  (FS - margin) / FS * 32768 LSB.
    // 16 g with a 0.5 g margin gives 31744, the same 15.5 g bar the old
    // body-frame test used.
    constexpr int32_t nearRailLsb(float fs_g, float margin_g)
    {
        return (int32_t)(((fs_g - margin_g) / fs_g) * 32768.0f);
    }

    // #1190: the two saturation bars behind the EKF's shock gate, in raw LSB
    // and per SENSOR axis — saturation is a per-axis property of the chip, and
    // the mount puts the chip at 45 deg to the thrust axis, so a body-frame
    // magnitude sits well above any axis (the low-g reads 15.5 g of magnitude
    // with 11 g on each of two axes).  Nothing below the rail is gated: a
    // value inside the sensor's range is a measurement, on any rocket.
    //
    // The ST gyro does NOT map its full scale onto the int16 span: it has a
    // fixed 0.035 mdps/LSB per dps of full scale (SensorConverter, #369), so
    // the NOMINAL full scale is the same count at every FS setting,
    // 1 / 0.035e-3 = 28571 LSB, and the int16 rail (32767) sits at 114.7 %
    // of it.  `fraction` of nominal FS, e.g. 0.95 -> 27142 LSB = 3800 dps at
    // +-4000.  The 2026-08-29 nose burst peaked at 32086 LSB (4492 dps).
    constexpr int32_t gyroFsFractionLsb(float fraction)
    {
        return (int32_t)(fraction * (1.0f / 0.035e-3f));
    }

    // The accelerometers map their full scale onto the int16 span (FS / 32768
    // per LSB), so a fraction of FS is the same count at every FS setting:
    // 0.95 -> 31129 LSB, 243 g on the +-256 g high-g channel.
    constexpr int32_t accelFsFractionLsb(float fraction)
    {
        return (int32_t)(fraction * 32768.0f);
    }

    // Integer mean rounded half away from zero.  The mean of int16 samples is
    // itself within int16 range, so the result needs no clamp.
    inline int16_t roundedMean(int32_t sum, uint32_t n)
    {
        const int32_t half = (int32_t)(n / 2u);
        const int32_t q = (sum >= 0) ? ((sum + half) / (int32_t)n)
                                     : -(((-sum) + half) / (int32_t)n);
        return (int16_t)q;
    }
}

struct ImuDrainWindow
{
    // Sums and maxima are raw LSB in SENSOR axes, one triple per channel.
    // 32-bit sums hold 65,536 full-scale samples.  The collector's queue is
    // 256 deep and drops the oldest when full, so a single drain pass can
    // never hand over more than ~256 samples plus the few the poll task adds
    // while the drain runs.
    int32_t  sum_lg[3] = {0, 0, 0};   // low-g accel
    int32_t  sum_hg[3] = {0, 0, 0};   // high-g accel
    int32_t  sum_gy[3] = {0, 0, 0};   // gyro
    int32_t  max_lg[3] = {0, 0, 0};   // max |raw| per axis (32768 for a -32768 sample)
    int32_t  max_hg[3] = {0, 0, 0};
    int32_t  max_gy[3] = {0, 0, 0};
    uint32_t n          = 0;          // samples added
    uint32_t t_first_us = 0;          // stamp of the first sample added
    uint32_t t_last_us  = 0;          // stamp of the last sample added

    void reset() { *this = ImuDrainWindow(); }

    void add(const ISM6HG256Data& s)
    {
        if (n == 0) t_first_us = s.time_us;
        t_last_us = s.time_us;
        n++;
        accumulate(s.acc_low_raw,  sum_lg, max_lg);
        accumulate(s.acc_high_raw, sum_hg, max_hg);
        accumulate(s.gyro_raw,     sum_gy, max_gy);
    }

    // The mean raw sample, stamped at the centre of the window.  A boxcar's
    // output belongs to the middle of the samples it averaged.  The EKF's dt
    // telescopes either way (consecutive windows are disjoint and ordered),
    // and the 100 ms staleness gates cannot tell a 1 ms lag.  Returns false
    // when nothing was added, and leaves `out` untouched.
    bool mean(ISM6HG256Data& out) const
    {
        if (n == 0) return false;
        out.time_us      = t_first_us + ((t_last_us - t_first_us) / 2u);  // wrap-safe
        out.acc_low_raw  = meanOf(sum_lg);
        out.acc_high_raw = meanOf(sum_hg);
        out.gyro_raw     = meanOf(sum_gy);
        return true;
    }

    // Did any sample in the window put any low-g SENSOR axis above the bar?
    bool lowGNearRail(int32_t thresh_lsb) const
    {
        return anyAbove(max_lg, thresh_lsb);
    }

    // #1190: the same question for the gyro and the high-g accelerometer —
    // the shock gate's two criteria, and the only place they are judged.
    // On the window's worst sample per sensor axis for the reason above: the
    // EKF is handed the mean, and the mean of a burst that touched the rail on
    // a few samples sits far below it (the 2026-08-29 burst had 21 samples
    // over 3800 dps in ~940).
    bool gyroAbove(int32_t thresh_lsb) const
    {
        return anyAbove(max_gy, thresh_lsb);
    }
    bool highGAbove(int32_t thresh_lsb) const
    {
        return anyAbove(max_hg, thresh_lsb);
    }

    static bool anyAbove(const int32_t (&mx)[3], int32_t thresh_lsb)
    {
        return (mx[0] > thresh_lsb) || (mx[1] > thresh_lsb) || (mx[2] > thresh_lsb);
    }

private:
    static void accumulate(const Vec3i16& v, int32_t (&sum)[3], int32_t (&mx)[3])
    {
        const int32_t x = v.x, y = v.y, z = v.z;
        sum[0] += x; sum[1] += y; sum[2] += z;
        const int32_t ax = (x < 0) ? -x : x;
        const int32_t ay = (y < 0) ? -y : y;
        const int32_t az = (z < 0) ? -z : z;
        if (ax > mx[0]) mx[0] = ax;
        if (ay > mx[1]) mx[1] = ay;
        if (az > mx[2]) mx[2] = az;
    }

    Vec3i16 meanOf(const int32_t (&sum)[3]) const
    {
        Vec3i16 v;
        v.x = imu_drain::roundedMean(sum[0], n);
        v.y = imu_drain::roundedMean(sum[1], n);
        v.z = imu_drain::roundedMean(sum[2], n);
        return v;
    }
};
