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
//   QMC5883P, Beetle/M1 (U3, F.Cu, rotation 90): [[0,-1,0],[-1,0,0],[0,0,-1]]
//     — a 2026-10-05 bench log: gyro-vs-field kinematics and the dip at
//     rest; det +1 with Z into the board as configured (#1590).
//   QMC5883P, V10 (U3, F.Cu, rotation 0): diag(-1, 1, -1)
//     — the Beetle's chip axes turned back by the 90 deg its footprint is
//     turned; inferred, no V10 has been built (#1590).
#include <gtest/gtest.h>
#include "TR_Sensor_Data_Converter.h"
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

#if TR_BOARD_V8 || TR_BOARD_V9
TEST(MagFrameBoard, TheIIS2MDCMapsAsMeasured) {
    const float rot = config::magRotZDeg(false);
    EXPECT_FLOAT_EQ(rot, config::MAG_ROT_Z_DEG_IIS2MDC);
    expectMatrix(chipToBoard(MAG_TYPE_IIS2MDC, rot), kIis2mdcV8V9, "IIS2MDC");
    expectFlatFieldPointsDown(MAG_TYPE_IIS2MDC, rot, kIis2mdcV8V9, "IIS2MDC");
}
#endif

#if TR_BOARD_V9
// The V9/V10 image finds the chip at boot (TR_MAG_DRIVER_AUTO), so the board
// header carries both, and magRotZDeg picks by what was found.
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
