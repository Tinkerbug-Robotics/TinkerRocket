// #1589 / #1590: every board's magnetometer, chip counts to board frame,
// through the real converter at the board's own rotation constant, against
// the physically measured chip-to-board matrix.  Built once per board
// (CMakeLists: test_mag_frame_<BOARD>), because each board's config.h is its
// own; a wrong MAG_ROT_Z_DEG_* or chip sign fails here, not on the pad.
//
// The matrices (columns = where each chip axis points, in the board frame:
// +X forward along the long axis, +Y left, +Z out of the top):
//   IIS2MDC, V8/V9 (U23/U3, F.Cu, rotation 0): [[0,-1,0],[-1,0,0],[0,0,1]]
//     — the #204 bench (board +X north reads chip -Y, east reads chip -X)
//     plus Z out of the top; det -1, ST's left-handed axes (#1589).
//     V5 (U23, F.Cu, rotation 0, with its ISM6HG256 at 45 as on V9): the
//     same matrix, inferred from the placement; no V5 has been benched.
//   QMC5883P, Beetle/M1 (U3, F.Cu, rotation 90): [[0,-1,0],[-1,0,0],[0,0,-1]]
//     — a 2026-10-05 bench log: gyro-vs-field kinematics and the dip at
//     rest; det +1 with Z into the board as configured (#1590).
//   QMC5883P, V10 (U3, F.Cu, rotation 0): diag(-1, 1, -1)
//     — the Beetle's chip axes turned back by the 90 deg its footprint is
//     turned; inferred, no V10 has been built (#1590).
//   MMC5983MA, Tinker-Beetle V2 (U3, B.Cu, rotation 0): [[0,-1,0],[-1,0,0],[0,0,1]]
//     — its datasheet axes (X+ pin 1 -> 4, Y+ pin 16 -> 13, Z+ into the
//     package) on the V2 layout; det -1, the chip's left-handed axes. The
//     collector's signs first (axis_sign.h, the part is on the back), then
//     the converter's MMC rotation (board_m2.h). Inferred; no V2 is built.
#include <gtest/gtest.h>
#include "TR_Sensor_Data_Converter.h"
#include "axis_sign.h"
#include "config.h"
#include <cmath>

namespace {

struct Mat { double m[3][3]; };

Mat chipToBoard(uint8_t mag_type, float rot_z_deg)
{
    SensorConverter conv;
    conv.configureMagType(mag_type);
    conv.configureIIS2MDCRotationZ(rot_z_deg);
    Mat out{};
    for (int c = 0; c < 3; c++)
    {
        IIS2MDCData raw{};
        raw.mag_x = (c == 0) ? 1000 : 0;
        raw.mag_y = (c == 1) ? 1000 : 0;
        raw.mag_z = (c == 2) ? 1000 : 0;
        IIS2MDCDataSI si{};
        conv.convertIIS2MDCData(raw, si);
        const double k = 1000.0 * magTypeUtPerLsb(mag_type);
        out.m[0][c] = si.mag_x_uT / k;
        out.m[1][c] = si.mag_y_uT / k;
        out.m[2][c] = si.mag_z_uT / k;
    }
    return out;
}

void expectMatrix(const Mat& got, const double (&want)[3][3], const char* what)
{
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            EXPECT_NEAR(got.m[r][c], want[r][c], 1e-6) << what << " row " << r << " col " << c;
}

// Flat on a bench at ~39 N, nose (+X) north: the field points north and
// DOWN, so with board +Z up its board z is negative.  The counts the chip
// reports follow from the measured matrix (counts = M^T b), and the
// converter has to hand back b — the dip test the Beetle failed (#1590).
void expectFlatFieldPointsDown(uint8_t mag_type, float rot_z_deg, const double (&M)[3][3],
                               const char* what)
{
    const double b[3] = {21.0, 0.0, -46.0};   // µT: north, level, down
    double counts[3];
    for (int i = 0; i < 3; i++)
        counts[i] = (M[0][i] * b[0] + M[1][i] * b[1] + M[2][i] * b[2]) / magTypeUtPerLsb(mag_type);
    IIS2MDCData raw{};
    raw.mag_x = (int16_t)lround(counts[0]);
    raw.mag_y = (int16_t)lround(counts[1]);
    raw.mag_z = (int16_t)lround(counts[2]);
    SensorConverter conv;
    conv.configureMagType(mag_type);
    conv.configureIIS2MDCRotationZ(rot_z_deg);
    IIS2MDCDataSI si{};
    conv.convertIIS2MDCData(raw, si);
    EXPECT_NEAR(si.mag_x_uT, b[0], 0.2) << what;
    EXPECT_NEAR(si.mag_y_uT, b[1], 0.2) << what;
    EXPECT_NEAR(si.mag_z_uT, b[2], 0.2) << what << ": the field must point down";
}

constexpr double kIis2mdcV8V9[3][3]  = {{0, -1, 0}, {-1, 0, 0}, {0, 0, 1}};
constexpr double kQmc5883pBeetle[3][3] = {{0, -1, 0}, {-1, 0, 0}, {0, 0, -1}};
constexpr double kQmc5883pV10[3][3]  = {{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}};

}  // namespace

#if TR_BOARD_V5 || TR_BOARD_V8 || TR_BOARD_V9
TEST(MagFrameBoard, TheIIS2MDCMapsAsMeasured) {
    const float rot = config::magRotZDeg(false);
    EXPECT_FLOAT_EQ(rot, config::MAG_ROT_Z_DEG_IIS2MDC);
    expectMatrix(chipToBoard(MAG_TYPE_IIS2MDC, rot), kIis2mdcV8V9, "IIS2MDC");
    expectFlatFieldPointsDown(MAG_TYPE_IIS2MDC, rot, kIis2mdcV8V9, "IIS2MDC");
}
#endif

#if TR_BOARD_V9 || TR_BOARD_V10
// The V9 image finds the chip at boot (TR_MAG_DRIVER_AUTO), so its header
// carries both and magRotZDeg picks by what was found; a V9 image on a V10
// still converts the QMC5883P right. The V10 image drives the QMC5883P only
// (TR_MAG_DRIVER_QMC5883P, #1593), from board_v10.h's own constant.
TEST(MagFrameBoard, TheV10sQMC5883PMapsAsInferred) {
    const float rot = config::magRotZDeg(true);
    EXPECT_FLOAT_EQ(rot, config::MAG_ROT_Z_DEG_QMC5883P);
    expectMatrix(chipToBoard(MAG_TYPE_QMC5883P, rot), kQmc5883pV10, "V10 QMC5883P");
    expectFlatFieldPointsDown(MAG_TYPE_QMC5883P, rot, kQmc5883pV10, "V10 QMC5883P");
}
#endif

#if TR_BOARD_M1
TEST(MagFrameBoard, TheBeetlesQMC5883PMapsAsMeasured) {
    const float rot = config::magRotZDeg(true);
    EXPECT_FLOAT_EQ(rot, config::MAG_ROT_Z_DEG_QMC5883P);
    expectMatrix(chipToBoard(MAG_TYPE_QMC5883P, rot), kQmc5883pBeetle, "Beetle QMC5883P");
    expectFlatFieldPointsDown(MAG_TYPE_QMC5883P, rot, kQmc5883pBeetle, "Beetle QMC5883P");
}
#endif

#if TR_BOARD_M2
namespace {

constexpr double kMmcBeetleV2[3][3] = {{0, -1, 0}, {-1, 0, 0}, {0, 0, 1}};
constexpr double kMmcUtPerCount = 800.0 / 131072.0;   // the converter's: ±8 G over 2^17

// The chip's centered counts as the collector hands them on (each axis's sign,
// as it reads), then the converter at the board's MMC rotation.
MMC5983MADataSI convertMmc(const long centered[3])
{
    const int8_t* s = config::MMC5983MA_AXIS_SIGN;
    MMC5983MAData raw{};
    raw.mag_x = axis_sign::mmc18((uint32_t)(centered[0] + 131072), s[0]);
    raw.mag_y = axis_sign::mmc18((uint32_t)(centered[1] + 131072), s[1]);
    raw.mag_z = axis_sign::mmc18((uint32_t)(centered[2] + 131072), s[2]);
    SensorConverter conv;
    conv.configureMMC5983MARotationZ(config::MMC5983MA_ROT_Z_DEG);
    MMC5983MADataSI si{};
    conv.convertMMC5983MAData(raw, si);
    return si;
}

}  // namespace

TEST(MagFrameBoard, TheV2sMMC5983MAMapsAsItsDatasheet) {
    Mat got{};
    for (int c = 0; c < 3; c++)
    {
        long centered[3] = {0, 0, 0};
        centered[c] = 10000;
        const MMC5983MADataSI si = convertMmc(centered);
        const double k = 10000.0 * kMmcUtPerCount;
        got.m[0][c] = si.mag_x_uT / k;
        got.m[1][c] = si.mag_y_uT / k;
        got.m[2][c] = si.mag_z_uT / k;
    }
    expectMatrix(got, kMmcBeetleV2, "V2 MMC5983MA");
}

// Flat on a bench at ~39 N, nose north, front face up: the field reads north
// and DOWN (board z negative), with the chip on the back of the board.
TEST(MagFrameBoard, TheV2sMMC5983MAFieldPointsDownFlat) {
    const double b[3] = {21.0, 0.0, -46.0};
    long centered[3];
    for (int i = 0; i < 3; i++)
        centered[i] = lround((kMmcBeetleV2[0][i] * b[0] + kMmcBeetleV2[1][i] * b[1] +
                              kMmcBeetleV2[2][i] * b[2]) / kMmcUtPerCount);
    const MMC5983MADataSI si = convertMmc(centered);
    EXPECT_NEAR(si.mag_x_uT, b[0], 0.05);
    EXPECT_NEAR(si.mag_y_uT, b[1], 0.05);
    EXPECT_NEAR(si.mag_z_uT, b[2], 0.05) << "the field must point down";
}

// An 18-bit count reversed about the chip's zero (2^17); the negative rail
// reads the positive one.
TEST(MagFrameBoard, AReversedMmcCountStaysInRange) {
    EXPECT_EQ(axis_sign::mmc18(0x20000u, -1), 0x20000u);
    EXPECT_EQ(axis_sign::mmc18(0x20000u + 5u, -1), 0x20000u - 5u);
    EXPECT_EQ(axis_sign::mmc18(0x3FFFFu, -1), 1u);
    EXPECT_EQ(axis_sign::mmc18(0u, -1), 0x3FFFFu);
    EXPECT_EQ(axis_sign::mmc18(0x12345u, 1), 0x12345u);
}
#endif
