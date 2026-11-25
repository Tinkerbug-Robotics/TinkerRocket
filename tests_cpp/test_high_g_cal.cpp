// The high-g accelerometer's six-face alignment to the low-g
// (RocketComputerTypes.h, HG_CAL_*: high = M * h - b - o, board frame).
//
//   - what the flight computer refuses to store (hg_cal_math.h);
//   - the converter: with no alignment its arithmetic is the original's, bit
//     for bit; with one, M before the bias and the offset, all before the
//     board->rocket mounting, and off in sim like the low-g offset;
//   - the pad calibration with an alignment measures the bias after it
//     (sensor_cal_math.h), so it re-measures b and keeps M;
//   - the model end to end: a board with known high-g and low-g offsets,
//     scales and misalignment, read through counts, corrects to the
//     calibrated low-g on the pad and at boost accelerations, and again after
//     the high-g offset drifts and a pad calibration re-measures it.
// The fit itself is the apps' and tools/accel_cal_six_face.py's (tested there).
#include <gtest/gtest.h>
#include "TR_Sensor_Data_Converter.h"
#include "TR_Orientation.h"
#include "hg_cal_math.h"
#include "sensor_cal_math.h"
#include <cmath>
#include <cstring>
#include <limits>
#include <random>

namespace
{
constexpr double kG = 9.80665;
constexpr double kLgPerLsb = (16.0 * 1000.0 / 32768.0) * 1e-3 * kG;    // m/s^2
constexpr double kHgPerLsb = (256.0 * 1000.0 / 32768.0) * 1e-3 * kG;

HighGCalApplyData store(const float m[9], float bx, float by, float bz)
{
    HighGCalApplyData a{};
    a.op = HG_CAL_OP_STORE;
    memcpy(a.matrix, m, sizeof(a.matrix));
    a.bias_mps2[0] = bx;
    a.bias_mps2[1] = by;
    a.bias_mps2[2] = bz;
    return a;
}

const float kIdentity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
// a plausible alignment: scales 0.8 % apart, a few tenths of a degree of cross-axis
const float kAlign[9] = {1.0081f, 0.0042f, -0.0031f,
                         -0.0027f, 0.9934f, 0.0056f,
                         0.0018f, -0.0049f, 1.0122f};

void configure(SensorConverter& c, float rot_z_deg)
{
    c.configureISM6HG256FullScale(ISM6LowGFullScale::FS_16G, ISM6HighGFullScale::FS_256G,
                                  ISM6GyroFullScale::DPS_4000);
    c.configureISM6HG256RotationZ(rot_z_deg);
}

bool sameBits(double a, double b) { return memcmp(&a, &b, sizeof(double)) == 0; }

void expectSameBits(const ISM6HG256DataSI& a, const ISM6HG256DataSI& b)
{
    EXPECT_TRUE(sameBits(a.low_g_acc_x, b.low_g_acc_x));
    EXPECT_TRUE(sameBits(a.low_g_acc_y, b.low_g_acc_y));
    EXPECT_TRUE(sameBits(a.low_g_acc_z, b.low_g_acc_z));
    EXPECT_TRUE(sameBits(a.high_g_acc_x, b.high_g_acc_x));
    EXPECT_TRUE(sameBits(a.high_g_acc_y, b.high_g_acc_y));
    EXPECT_TRUE(sameBits(a.high_g_acc_z, b.high_g_acc_z));
    EXPECT_TRUE(sameBits(a.gyro_x, b.gyro_x));
    EXPECT_TRUE(sameBits(a.gyro_y, b.gyro_y));
    EXPECT_TRUE(sameBits(a.gyro_z, b.gyro_z));
}

int16_t clampCounts(double v)
{
    const double r = std::round(v);
    return (int16_t)(r > 32767.0 ? 32767.0 : (r < -32768.0 ? -32768.0 : r));
}

// Board frame -> the chip's frame through the ISM6's Z rotation (the
// converter's inverse), as counts.
void boardToCounts(const double v[3], double lsb, double rot_z_deg, int16_t out[3])
{
    const double r = rot_z_deg * M_PI / 180.0;
    const double c = cos(r), s = sin(r);
    out[0] = clampCounts((v[0] * c + v[1] * s) / lsb);
    out[1] = clampCounts((-v[0] * s + v[1] * c) / lsb);
    out[2] = clampCounts(v[2] / lsb);
}

void mul(const double m[9], const double v[3], double out[3])
{
    for (int i = 0; i < 3; i++) out[i] = m[3 * i] * v[0] + m[3 * i + 1] * v[1] + m[3 * i + 2] * v[2];
}

// 3x3 inverse (row-major)
void inv3(const double a[9], double out[9])
{
    const double det = a[0] * (a[4] * a[8] - a[5] * a[7]) - a[1] * (a[3] * a[8] - a[5] * a[6]) +
                       a[2] * (a[3] * a[7] - a[4] * a[6]);
    out[0] = (a[4] * a[8] - a[5] * a[7]) / det;
    out[1] = (a[2] * a[7] - a[1] * a[8]) / det;
    out[2] = (a[1] * a[5] - a[2] * a[4]) / det;
    out[3] = (a[5] * a[6] - a[3] * a[8]) / det;
    out[4] = (a[0] * a[8] - a[2] * a[6]) / det;
    out[5] = (a[2] * a[3] - a[0] * a[5]) / det;
    out[6] = (a[3] * a[7] - a[4] * a[6]) / det;
    out[7] = (a[1] * a[6] - a[0] * a[7]) / det;
    out[8] = (a[0] * a[4] - a[1] * a[3]) / det;
}
} // namespace

// ---------- what the flight computer stores ----------

TEST(HighGCalRefusal, AcceptsAPlausibleAlignmentAndAClear)
{
    EXPECT_EQ(hg_cal::refusal(store(kAlign, -0.65f, 1.48f, -0.31f)), nullptr);
    EXPECT_EQ(hg_cal::refusal(store(kIdentity, 0, 0, 0)), nullptr);
    HighGCalApplyData clear{};
    clear.op = HG_CAL_OP_CLEAR;
    clear.matrix[0] = std::numeric_limits<float>::quiet_NaN();   // a clear's numbers are ignored
    EXPECT_EQ(hg_cal::refusal(clear), nullptr);
}

TEST(HighGCalRefusal, RefusesScalesCrossAxisBiasesAndNonFiniteNumbers)
{
    float m[9];
    memcpy(m, kAlign, sizeof(m));
    m[4] = 1.031f;
    EXPECT_NE(hg_cal::refusal(store(m, 0, 0, 0)), nullptr);
    m[4] = 0.969f;
    EXPECT_NE(hg_cal::refusal(store(m, 0, 0, 0)), nullptr);
    m[4] = 1.03f;   // the limits themselves are allowed
    EXPECT_EQ(hg_cal::refusal(store(m, 0, 0, 0)), nullptr);
    memcpy(m, kAlign, sizeof(m));
    m[5] = -0.031f;
    EXPECT_NE(hg_cal::refusal(store(m, 0, 0, 0)), nullptr);
    m[5] = 0.03f;
    EXPECT_EQ(hg_cal::refusal(store(m, 0, 0, 0)), nullptr);
    EXPECT_NE(hg_cal::refusal(store(kAlign, 11.01f, 0, 0)), nullptr);
    EXPECT_EQ(hg_cal::refusal(store(kAlign, -11.0f, 0, 0)), nullptr);
    memcpy(m, kAlign, sizeof(m));
    m[7] = std::numeric_limits<float>::infinity();
    EXPECT_NE(hg_cal::refusal(store(m, 0, 0, 0)), nullptr);
    EXPECT_NE(hg_cal::refusal(store(kAlign, 0, std::numeric_limits<float>::quiet_NaN(), 0)), nullptr);
    HighGCalApplyData bad = store(kAlign, 0, 0, 0);
    bad.op = 7;
    EXPECT_NE(hg_cal::refusal(bad), nullptr);
}

TEST(HighGCalRefusal, IdentityIsExact)
{
    EXPECT_TRUE(hg_cal::isIdentity(kIdentity));
    float m[9];
    hg_cal::setIdentity(m);
    EXPECT_TRUE(hg_cal::isIdentity(m));
    m[3] = 1e-9f;
    EXPECT_FALSE(hg_cal::isIdentity(m));
}

// ---------- the converter ----------

// With no alignment -- never set, set to the identity, or cleared -- every
// output is bit for bit the converter's original arithmetic (its high-g path
// is untouched; this pins that the identity takes it).
TEST(HighGCalConverter, NoAlignmentKeepsTheOriginalArithmeticBitForBit)
{
    std::mt19937 rng(1629);
    std::uniform_int_distribution<int> counts(-32768, 32767);
    for (float rot : {0.0f, -45.0f, 90.0f})
    {
        SensorConverter never, identity, cleared;
        for (SensorConverter* c : {&never, &identity, &cleared})
        {
            configure(*c, rot);
            float R[9];
            orientCodeToMatrix(16, R);   // +Z to the nose: a mounting that mixes every axis
            c->configureBoardToRocket(R);
            c->setHighGBias(-0.65f, -0.57f, 0.91f);
            c->setLowGOffset(-0.0155f, -0.1324f, -0.0306f);
        }
        identity.setHighGAlignment(kIdentity);
        cleared.setHighGAlignment(kAlign);
        cleared.clearHighGAlignment();
        EXPECT_FALSE(never.highGAlignmentSet());
        EXPECT_FALSE(identity.highGAlignmentSet());
        EXPECT_FALSE(cleared.highGAlignmentSet());
        for (int k = 0; k < 2000; k++)
        {
            ISM6HG256Data raw{};
            raw.time_us = (uint32_t)k;
            raw.acc_low_raw.x = (int16_t)counts(rng);
            raw.acc_low_raw.y = (int16_t)counts(rng);
            raw.acc_low_raw.z = (int16_t)counts(rng);
            raw.acc_high_raw.x = (int16_t)counts(rng);
            raw.acc_high_raw.y = (int16_t)counts(rng);
            raw.acc_high_raw.z = (int16_t)counts(rng);
            raw.gyro_raw.x = (int16_t)counts(rng);
            raw.gyro_raw.y = (int16_t)counts(rng);
            raw.gyro_raw.z = (int16_t)counts(rng);
            ISM6HG256DataSI a{}, b{}, c{};
            never.convertISM6HG256Data(raw, a);
            identity.convertISM6HG256Data(raw, b);
            cleared.convertISM6HG256Data(raw, c);
            expectSameBits(a, b);
            expectSameBits(a, c);
        }
    }
}

// M first, then the bias and the low-g offset, all in the board frame, then
// the mounting; the low-g is untouched by it.
TEST(HighGCalConverter, AlignmentComesBeforeTheBiasAndOffsetInTheBoardFrame)
{
    SensorConverter c;
    configure(c, -45.0f);
    float R[9];
    orientCodeToMatrix(16, R);
    c.configureBoardToRocket(R);
    c.setHighGBias(0.3f, -0.2f, 0.1f);
    c.setLowGOffset(0.01f, 0.02f, -0.03f);
    SensorConverter plain = c;
    c.setHighGAlignment(kAlign);
    EXPECT_TRUE(c.highGAlignmentSet());
    EXPECT_TRUE(c.highGAlignmentApplied());
    float back[9];
    c.highGAlignment(back);
    EXPECT_EQ(memcmp(back, kAlign, sizeof(back)), 0);

    ISM6HG256Data raw{};
    raw.acc_low_raw.x = 1500;
    raw.acc_low_raw.y = -700;
    raw.acc_low_raw.z = 1900;
    raw.acc_high_raw.x = 2100;
    raw.acc_high_raw.y = -900;
    raw.acc_high_raw.z = 3300;
    ISM6HG256DataSI si{}, ref{};
    c.convertISM6HG256Data(raw, si);
    plain.convertISM6HG256Data(raw, ref);
    EXPECT_TRUE(sameBits(si.low_g_acc_x, ref.low_g_acc_x));
    EXPECT_TRUE(sameBits(si.low_g_acc_y, ref.low_g_acc_y));
    EXPECT_TRUE(sameBits(si.low_g_acc_z, ref.low_g_acc_z));

    double h[3];
    c.highGBoardRaw(raw, h);
    double md[9];
    for (int i = 0; i < 9; i++) md[i] = kAlign[i];
    double board[3];
    mul(md, h, board);
    board[0] -= 0.3 + 0.01;
    board[1] -= -0.2 + 0.02;
    board[2] -= 0.1 - 0.03;
    double Rd[9];
    for (int i = 0; i < 9; i++) Rd[i] = R[i];
    double rocket[3];
    mul(Rd, board, rocket);
    EXPECT_NEAR(si.high_g_acc_x, rocket[0], 1e-6);
    EXPECT_NEAR(si.high_g_acc_y, rocket[1], 1e-6);
    EXPECT_NEAR(si.high_g_acc_z, rocket[2], 1e-6);
}

// The six-face calibration is the real IMU's: in sim (the counts are the
// truth's) the alignment is off with the low-g offset, and the high-g is
// the bias alone, as before.
TEST(HighGCalConverter, AlignmentIsOffInSimWithTheLowGOffset)
{
    SensorConverter c;
    configure(c, -45.0f);
    c.setHighGBias(0.3f, -0.2f, 0.1f);
    SensorConverter plain = c;
    c.setHighGAlignment(kAlign);
    c.setLowGOffsetEnabled(false);
    plain.setLowGOffsetEnabled(false);
    EXPECT_TRUE(c.highGAlignmentSet());
    EXPECT_FALSE(c.highGAlignmentApplied());
    ISM6HG256Data raw{};
    raw.acc_high_raw.x = 2100;
    raw.acc_high_raw.y = -900;
    raw.acc_high_raw.z = 3300;
    ISM6HG256DataSI a{}, b{};
    c.convertISM6HG256Data(raw, a);
    plain.convertISM6HG256Data(raw, b);
    expectSameBits(a, b);
}

TEST(HighGCalConverter, HighGBoardRawIsTheRotatedReadingWithNothingTakenOff)
{
    SensorConverter c;
    configure(c, -45.0f);
    c.setHighGBias(1.0f, 2.0f, 3.0f);
    c.setLowGOffset(0.1f, 0.2f, 0.3f);
    c.setHighGAlignment(kAlign);
    ISM6HG256Data raw{};
    raw.acc_high_raw.x = 128;
    raw.acc_high_raw.y = 0;
    raw.acc_high_raw.z = -128;
    double h[3];
    c.highGBoardRaw(raw, h);
    const double one_g = 128 * kHgPerLsb;
    EXPECT_NEAR(h[0], one_g * cos(-M_PI / 4), 1e-5);
    EXPECT_NEAR(h[1], one_g * sin(-M_PI / 4), 1e-5);
    EXPECT_NEAR(h[2], -one_g, 1e-6);
}

// ---------- the pad calibration with an alignment ----------

TEST(HighGCalPadCal, WithAnAlignmentTheBiasIsMeasuredAfterIt)
{
    // a still window: low-g (0, 0, 1 g), high-g a little off, sensor frame = body (rot 0)
    SensorCalSums s;
    s.count = 1000;
    const int16_t lg[3] = {12, -30, 2048};
    const int16_t hg[3] = {14, -9, 131};
    for (int i = 0; i < 3; i++)
    {
        s.lg[i] = (int64_t)lg[i] * s.count;
        s.hg[i] = (int64_t)hg[i] * s.count;
    }
    SensorCalResult plain, aligned;
    ASSERT_TRUE(sensor_cal::compute(s, 0.0f, 16, 256, plain));
    ASSERT_TRUE(sensor_cal::compute(s, 0.0f, 16, 256, aligned, kAlign));
    for (int i = 0; i < 3; i++)
    {
        // without one: the original, high-g minus low-g
        EXPECT_FLOAT_EQ(plain.hg_bias[i], plain.hg_body[i] - plain.lg_body[i]);
        EXPECT_FLOAT_EQ(plain.hg_aligned[i], plain.hg_body[i]);
        const float mh = kAlign[3 * i] * aligned.hg_body[0] + kAlign[3 * i + 1] * aligned.hg_body[1] +
                         kAlign[3 * i + 2] * aligned.hg_body[2];
        EXPECT_FLOAT_EQ(aligned.hg_aligned[i], mh);
        EXPECT_FLOAT_EQ(aligned.hg_bias[i], mh - aligned.lg_body[i]);
        // the rest of the window's result is the alignment's business not at all
        EXPECT_EQ(plain.lg_body[i], aligned.lg_body[i]);
        EXPECT_EQ(plain.hg_body[i], aligned.hg_body[i]);
        EXPECT_EQ(plain.gyro_offset[i], aligned.gyro_offset[i]);
    }
    EXPECT_EQ(plain.gravity_mag, aligned.gravity_mag);
}

// ---------- the model, end to end ----------

// A board whose two accelerometers differ the way real ones do. With M and b
// from its truth (what the six-face fit converges to), the corrected high-g
// reads as the calibrated low-g -- on the pad and at boost accelerations,
// where the original bias alone leaves the scale and cross-axis error.
// Then the high-g's offset drifts (a cold launch day), a pad calibration with
// the alignment re-measures b, and the two match again.
TEST(HighGCalModel, CorrectedHighGMatchesTheCalibratedLowGFromThePadToBoost)
{
    const double rot = -45.0;
    // truth: reading = S * true + offset, board frame
    const double S_lg[9] = {1.0006, 0.0, 0.0, 0.0, 0.9991, 0.0, 0.0, 0.0, 1.0004};
    const double o_lg[3] = {-0.0155, -0.1324, -0.0306};
    const double S_hg[9] = {0.9921, -0.0047, 0.0030,
                            0.0029, 1.0068, -0.0054,
                            -0.0016, 0.0051, 0.9880};
    double c_hg[3] = {0.98, -1.47, 0.62};   // the high-g's own zero-g, m/s^2 (~100-150 mg)
    // what the six faces converge to: l = M h - b  =>  M = S_lg S_hg^-1, b = M c_hg - o_lg
    double Sinv[9], M[9];
    inv3(S_hg, Sinv);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            M[3 * i + j] = S_lg[3 * i + 0] * Sinv[0 + j] + S_lg[3 * i + 1] * Sinv[3 + j] + S_lg[3 * i + 2] * Sinv[6 + j];
    double b[3];
    mul(M, c_hg, b);
    for (int i = 0; i < 3; i++) b[i] -= o_lg[i];

    float Mf[9];
    for (int i = 0; i < 9; i++) Mf[i] = (float)M[i];
    ASSERT_EQ(hg_cal::refusal(store(Mf, (float)b[0], (float)b[1], (float)b[2])), nullptr);
    SensorConverter c;
    configure(c, (float)rot);
    c.setLowGOffset((float)o_lg[0], (float)o_lg[1], (float)o_lg[2]);
    c.setHighGAlignment(Mf);
    c.setHighGBias((float)b[0], (float)b[1], (float)b[2]);
    SensorConverter bias_only;   // the original model: the pad calibration's bias, no M
    configure(bias_only, (float)rot);
    bias_only.setLowGOffset((float)o_lg[0], (float)o_lg[1], (float)o_lg[2]);

    auto counts = [&](const double a[3], ISM6HG256Data& raw) {
        double l[3], h[3];
        mul(S_lg, a, l);
        mul(S_hg, a, h);
        for (int i = 0; i < 3; i++)
        {
            l[i] += o_lg[i];
            h[i] += c_hg[i];
        }
        int16_t lc[3], hc[3];
        boardToCounts(l, kLgPerLsb, rot, lc);
        boardToCounts(h, kHgPerLsb, rot, hc);
        raw = ISM6HG256Data{};
        raw.acc_low_raw.x = lc[0]; raw.acc_low_raw.y = lc[1]; raw.acc_low_raw.z = lc[2];
        raw.acc_high_raw.x = hc[0]; raw.acc_high_raw.y = hc[1]; raw.acc_high_raw.z = hc[2];
    };
    // the low-g's calibrated reading of `a`: S_lg a, which the high-g matches
    auto lowCal = [&](const double a[3], double out[3]) { mul(S_lg, a, out); };

    // the original bias, from a pad window (rocket nose-up: board +X up)
    const double pad[3] = {kG, 0.0, 0.0};
    {
        ISM6HG256Data raw;
        counts(pad, raw);
        SensorCalSums s;
        s.count = 1;
        s.lg[0] = raw.acc_low_raw.x; s.lg[1] = raw.acc_low_raw.y; s.lg[2] = raw.acc_low_raw.z;
        s.hg[0] = raw.acc_high_raw.x; s.hg[1] = raw.acc_high_raw.y; s.hg[2] = raw.acc_high_raw.z;
        SensorCalResult r;
        ASSERT_TRUE(sensor_cal::compute(s, (float)rot, 16, 256, r));
        bias_only.setHighGBias(r.hg_bias[0], r.hg_bias[1], r.hg_bias[2]);
    }

    // one count of the high-g is 7.8 mg: the corrected reading is good to a
    // few counts of quantization, whatever the acceleration
    const double tol = 3.0 * kHgPerLsb;
    const double accels[][3] = {{kG, 0, 0}, {-kG, 0, 0}, {0, kG, 0}, {0, 0, -kG},
                                {15 * kG, 0.4 * kG, -0.3 * kG}, {40 * kG, 1.5 * kG, 0.8 * kG},
                                {120 * kG, -3 * kG, 2 * kG}};
    double worst_bias_only = 0.0;
    for (const auto& a : accels)
    {
        ISM6HG256Data raw;
        counts(a, raw);
        ISM6HG256DataSI si{}, old{};
        c.convertISM6HG256Data(raw, si);
        bias_only.convertISM6HG256Data(raw, old);
        double want[3];
        lowCal(a, want);
        EXPECT_NEAR(si.high_g_acc_x, want[0], tol);
        EXPECT_NEAR(si.high_g_acc_y, want[1], tol);
        EXPECT_NEAR(si.high_g_acc_z, want[2], tol);
        worst_bias_only = fmax(worst_bias_only, fabs(old.high_g_acc_x - want[0]));
        worst_bias_only = fmax(worst_bias_only, fabs(old.high_g_acc_y - want[1]));
    }
    // the bias alone is right on the pad only: at 120 g this board's is about
    // 1 g off (0.8 % scale)
    EXPECT_GT(worst_bias_only, 0.5 * kG);

    // The high-g's offset drifts (+2 mg/degC, a launch day 20 degC colder);
    // the pad calibration, given M, re-measures b and the match is back.
    for (int i = 0; i < 3; i++) c_hg[i] += -0.04 * kG * (i == 1 ? -1.0 : 1.0);
    {
        ISM6HG256Data raw;
        counts(pad, raw);
        ISM6HG256DataSI si{};
        c.convertISM6HG256Data(raw, si);
        double want[3];
        lowCal(pad, want);
        EXPECT_GT(fabs(si.high_g_acc_y - want[1]), 0.3);   // stale b: ~40 mg off
        SensorCalSums s;
        s.count = 1;
        s.lg[0] = raw.acc_low_raw.x; s.lg[1] = raw.acc_low_raw.y; s.lg[2] = raw.acc_low_raw.z;
        s.hg[0] = raw.acc_high_raw.x; s.hg[1] = raw.acc_high_raw.y; s.hg[2] = raw.acc_high_raw.z;
        SensorCalResult r;
        ASSERT_TRUE(sensor_cal::compute(s, (float)rot, 16, 256, r, Mf));
        c.setHighGBias(r.hg_bias[0], r.hg_bias[1], r.hg_bias[2]);
    }
    for (const auto& a : accels)
    {
        ISM6HG256Data raw;
        counts(a, raw);
        ISM6HG256DataSI si{};
        c.convertISM6HG256Data(raw, si);
        double want[3];
        lowCal(a, want);
        EXPECT_NEAR(si.high_g_acc_x, want[0], tol);
        EXPECT_NEAR(si.high_g_acc_y, want[1], tol);
        EXPECT_NEAR(si.high_g_acc_z, want[2], tol);
    }
}
