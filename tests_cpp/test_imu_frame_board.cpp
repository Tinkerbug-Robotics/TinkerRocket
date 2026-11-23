// Every board's IMU, chip counts to board frame: the board header's
// ISM6HG256_AXIS_SIGN applied the way the collector applies it as it reads
// (axis_sign.h), then the real converter at the board's ISM6HG256_ROT_Z_DEG,
// against the chip-to-board matrix taken from the layout. Built once per board
// (CMakeLists: test_imu_frame_<BOARD>), because each board's config.h is its
// own; a wrong angle or sign fails here, not on the pad.
//
// The matrices (columns = where each chip axis points, in the board frame:
// +X forward along the long axis, +Y left, +Z out of the front):
//   Front-side U2 at -45 (V5, V7, V8, V9, V10, the Beetle V1): Rz(-45).
//     The V1's is the mapping the M1 flew; the others share the constant.
//   Tinker-Beetle V2 (M2), U2 on the BACK at -45 in the layout:
//     [[-s, -s, 0], [-s, s, 0], [0, 0, -1]], s = 1/sqrt 2, carried over pad by
//     pad from the V1's U2 to the V2's (board_m2.h): chip Z out of the back.
#include <gtest/gtest.h>
#include "TR_Sensor_Data_Converter.h"
#include "axis_sign.h"
#include "config.h"
#include <cmath>

namespace {

struct Mat { double m[3][3]; };

enum class Stream { LowG, HighG, Gyro };

// The counts the chip reports, as the collector hands them on (signed), then
// converted. Bias and offsets are zero on a fresh converter.
ISM6HG256DataSI convert(const int16_t chip[3])
{
    ISM6HG256Data raw{};
    const int8_t* s = config::ISM6HG256_AXIS_SIGN;
    struct { int16_t x, y, z; } a;
    a.x = axis_sign::i16(chip[0], s[0]);
    a.y = axis_sign::i16(chip[1], s[1]);
    a.z = axis_sign::i16(chip[2], s[2]);
    raw.acc_low_raw.x = a.x;  raw.acc_low_raw.y = a.y;  raw.acc_low_raw.z = a.z;
    raw.acc_high_raw.x = a.x; raw.acc_high_raw.y = a.y; raw.acc_high_raw.z = a.z;
    raw.gyro_raw.x = a.x;     raw.gyro_raw.y = a.y;     raw.gyro_raw.z = a.z;
    SensorConverter conv;
    conv.configureISM6HG256RotationZ(config::ISM6HG256_ROT_Z_DEG);
    ISM6HG256DataSI si{};
    conv.convertISM6HG256Data(raw, si);
    return si;
}

void pick(const ISM6HG256DataSI& si, Stream st, double v[3])
{
    switch (st)
    {
    case Stream::LowG:  v[0] = si.low_g_acc_x;  v[1] = si.low_g_acc_y;  v[2] = si.low_g_acc_z;  break;
    case Stream::HighG: v[0] = si.high_g_acc_x; v[1] = si.high_g_acc_y; v[2] = si.high_g_acc_z; break;
    case Stream::Gyro:  v[0] = si.gyro_x;       v[1] = si.gyro_y;       v[2] = si.gyro_z;       break;
    }
}

Mat chipToBoard(Stream st)
{
    Mat out{};
    for (int c = 0; c < 3; c++)
    {
        int16_t chip[3] = {0, 0, 0};
        chip[c] = 10000;
        double v[3];
        pick(convert(chip), st, v);
        const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        for (int r = 0; r < 3; r++) out.m[r][c] = v[r] / n;
    }
    return out;
}

void expectMatrix(const Mat& got, const double (&want)[3][3], const char* what)
{
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            EXPECT_NEAR(got.m[r][c], want[r][c], 1e-5) << what << " row " << r << " col " << c;
}

constexpr double kS = 0.70710678118654752;
#if TR_BOARD_M2
constexpr double kWant[3][3] = {{-kS, -kS, 0}, {-kS, kS, 0}, {0, 0, -1}};
#else
constexpr double kWant[3][3] = {{kS, kS, 0}, {-kS, kS, 0}, {0, 0, 1}};   // Rz(-45)
#endif

}  // namespace

TEST(ImuFrameBoard, EveryStreamMapsAsTheLayout) {
    expectMatrix(chipToBoard(Stream::LowG), kWant, "low-g");
    expectMatrix(chipToBoard(Stream::HighG), kWant, "high-g");
    expectMatrix(chipToBoard(Stream::Gyro), kWant, "gyro");
}

// No placement mirrors a part: the mapping is a rotation (det +1), or the
// gyro and the accelerometers would fly a mirror image of the motion.
TEST(ImuFrameBoard, TheMappingIsARotation) {
    const Mat m = chipToBoard(Stream::LowG);
    const double det =
        m.m[0][0] * (m.m[1][1] * m.m[2][2] - m.m[1][2] * m.m[2][1]) -
        m.m[0][1] * (m.m[1][0] * m.m[2][2] - m.m[1][2] * m.m[2][0]) +
        m.m[0][2] * (m.m[1][0] * m.m[2][1] - m.m[1][1] * m.m[2][0]);
    EXPECT_NEAR(det, 1.0, 1e-5);
}

// Flat on a bench, front face up: the accelerometers read +1 g up the board's
// +Z whichever side the part is on. The chip's own counts follow from the
// layout matrix (chip = M^T b); a part on the back reads its own Z negative.
TEST(ImuFrameBoard, FlatOnTheBenchGravityReadsUp) {
    const double lsb_per_g = 32768.0 / config::ISM6_LOW_G_FS_G;
    const double b[3] = {0.0, 0.0, 1.0};
    int16_t chip[3];
    for (int i = 0; i < 3; i++)
        chip[i] = (int16_t)std::lround((kWant[0][i] * b[0] + kWant[1][i] * b[1] + kWant[2][i] * b[2]) * lsb_per_g);
#if TR_BOARD_M2
    EXPECT_LT(chip[2], 0) << "a part on the back reads gravity along its own -Z";
#else
    EXPECT_GT(chip[2], 0);
#endif
    const ISM6HG256DataSI si = convert(chip);
    const double g = 9.80665;
    EXPECT_NEAR(si.low_g_acc_x, 0.0, 0.01 * g);
    EXPECT_NEAR(si.low_g_acc_y, 0.0, 0.01 * g);
    EXPECT_NEAR(si.low_g_acc_z, g, 0.01 * g) << "board +Z must read +1 g flat, front face up";
}

// A railed axis stays railed through a sign: -32768 reads +32767, never wraps.
TEST(ImuFrameBoard, AReversedAxisSaturates) {
    EXPECT_EQ(axis_sign::i16(INT16_MIN, -1), INT16_MAX);
    EXPECT_EQ(axis_sign::i16(INT16_MAX, -1), -INT16_MAX);
    EXPECT_EQ(axis_sign::i16(1234, -1), -1234);
    EXPECT_EQ(axis_sign::i16(-1234, 1), -1234);
    EXPECT_EQ(axis_sign::i16(INT16_MIN, 1), INT16_MIN);
}

// The status query reports the angle the apps and tools turn the logged
// (already signed) counts by: the board header's, in centidegrees.
TEST(ImuFrameBoard, TheReportedAngleIsTheBoards) {
#if TR_BOARD_M2
    EXPECT_FLOAT_EQ(config::ISM6HG256_ROT_Z_DEG, 45.0f);
    EXPECT_FLOAT_EQ(config::MMC5983MA_ROT_Z_DEG, 90.0f);
#else
    // every other board kept the one shared pair it always had
    EXPECT_FLOAT_EQ(config::ISM6HG256_ROT_Z_DEG, -45.0f);
    EXPECT_FLOAT_EQ(config::MMC5983MA_ROT_Z_DEG, 180.0f);
    for (int i = 0; i < 3; i++)
    {
        EXPECT_EQ(config::ISM6HG256_AXIS_SIGN[i], 1);
        EXPECT_EQ(config::MMC5983MA_AXIS_SIGN[i], 1);
    }
#endif
}
