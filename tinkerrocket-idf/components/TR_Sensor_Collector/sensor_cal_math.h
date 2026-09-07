// sensor_cal_math.h — pure arithmetic for the pad calibration (#1110).
//
// The IMU poll task sums every ISM6HG256 sample it reads during the
// calibration window into a SensorCalSums; the flight task turns those sums
// into offsets here.  No ESP-IDF dependencies on purpose: tests_cpp compiles
// this header on the host, which is the only place the math is exercised
// without a rocket on the bench.
#pragma once

#include <cstdint>
#include <cmath>

struct SensorCalSums
{
    int64_t g[3]  = {0, 0, 0};   // gyro, raw LSB
    int64_t lg[3] = {0, 0, 0};   // low-g accel, raw LSB
    int64_t hg[3] = {0, 0, 0};   // high-g accel, raw LSB
    uint32_t count = 0;          // samples summed into every axis above

    // 64-bit sums: a 10 s window at the 7680 Hz ODR ceiling is 76,800
    // samples, and 76,800 x 32,767 overflows 32 bits.
};

struct SensorCalResult
{
    int16_t gyro_offset[3] = {0, 0, 0};       // zero-rate offset, raw LSB (subtracted by the poll task)
    float   hg_bias[3]     = {0.0f, 0.0f, 0.0f};  // high-g minus low-g, m/s², body frame
    float   lg_body[3]     = {0.0f, 0.0f, 0.0f};  // low-g average, m/s², body frame (for the log)
    float   hg_body[3]     = {0.0f, 0.0f, 0.0f};  // high-g average, m/s², body frame (for the log)
    float   gravity_mag    = 0.0f;                // |low-g average|, m/s²
};

namespace sensor_cal
{
    static constexpr float kG           = 9.80665f;
    static constexpr float kPi          = 3.14159265358979f;
    // A stationary rocket reads 1 g on the low-g channel to within a percent
    // or two (offset and sensitivity tolerances).  The band is wide enough
    // that no sane pad is refused and narrow enough to reject the failure
    // modes this exists for: a second SPI slave fighting for MISO (values
    // wildly off), a full-scale mismatch (a factor of two), or a calibration
    // taken while the rocket was moving.
    static constexpr float kGravityMinMs2 = 0.80f * kG;
    static constexpr float kGravityMaxMs2 = 1.20f * kG;

    inline bool gravityPlausible(float gravity_mag_ms2)
    {
        return std::isfinite(gravity_mag_ms2) &&
               gravity_mag_ms2 >= kGravityMinMs2 &&
               gravity_mag_ms2 <= kGravityMaxMs2;
    }

    // Offsets and cross-calibration from the window sums.
    //   rotation_z_deg     sensor-to-body rotation about Z (config::ISM6HG256_ROT_Z_DEG)
    //   low_g_fs_g /       the CONFIGURED full scales (#572): the per-LSB scale is
    //   high_g_fs_g        FS * 1000 / 32768 mg, mirroring SensorConverter.
    // Returns false when there is nothing to average (count == 0); the caller
    // decides separately whether the gravity magnitude is plausible.
    inline bool compute(const SensorCalSums& s, float rotation_z_deg,
                        uint16_t low_g_fs_g, uint16_t high_g_fs_g,
                        SensorCalResult& out)
    {
        if (s.count == 0) return false;
        const int64_t n = (int64_t)s.count;

        // Gyro zero-rate offset: integer average, truncated toward zero
        // exactly as the original 1000-sample loop did.  The mean of int16
        // samples is itself within int16 range.
        for (int i = 0; i < 3; i++)
        {
            out.gyro_offset[i] = (int16_t)(s.g[i] / n);
        }

        const float lg_ms2_per_lsb = ((float)low_g_fs_g  * 1000.0f / 32768.0f) * 1e-3f * kG;
        const float hg_ms2_per_lsb = ((float)high_g_fs_g * 1000.0f / 32768.0f) * 1e-3f * kG;

        // Average raw -> SI, sensor frame
        float lg[3], hg[3];
        for (int i = 0; i < 3; i++)
        {
            lg[i] = ((float)((double)s.lg[i] / (double)n)) * lg_ms2_per_lsb;
            hg[i] = ((float)((double)s.hg[i] / (double)n)) * hg_ms2_per_lsb;
        }

        // Rotate both to the body frame (same rotation for every ISM6 channel)
        const float rot = rotation_z_deg * (kPi / 180.0f);
        const float c = cosf(rot);
        const float sn = sinf(rot);

        out.lg_body[0] = lg[0] * c - lg[1] * sn;
        out.lg_body[1] = lg[0] * sn + lg[1] * c;
        out.lg_body[2] = lg[2];

        out.hg_body[0] = hg[0] * c - hg[1] * sn;
        out.hg_body[1] = hg[0] * sn + hg[1] * c;
        out.hg_body[2] = hg[2];

        for (int i = 0; i < 3; i++)
        {
            out.hg_bias[i] = out.hg_body[i] - out.lg_body[i];
        }

        // Gravity magnitude is rotation-invariant, so the sensor frame is fine.
        out.gravity_mag = sqrtf(lg[0] * lg[0] + lg[1] * lg[1] + lg[2] * lg[2]);
        return true;
    }
}
