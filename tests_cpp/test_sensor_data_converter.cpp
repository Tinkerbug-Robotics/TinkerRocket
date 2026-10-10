#include <gtest/gtest.h>
#include "TR_Sensor_Data_Converter.h"
#include "SimSensorModel.h"   // the firmware sim's gyro LSB (#369)
#include "config.h"         // flight_computer/main/config.h: the chip rotations
#include <cmath>
#include <limits>   // #850: quiet_NaN in the rail-current tests
#include <cstring>

class SensorConverterTest : public ::testing::Test {
protected:
    SensorConverter conv;

    void SetUp() override {
        // Default config: 16g low, 256g high, 4000 dps gyro, no rotation
        conv.configureISM6HG256FullScale(ISM6LowGFullScale::FS_16G,
                                         ISM6HighGFullScale::FS_256G,
                                         ISM6GyroFullScale::DPS_4000);
        conv.configureISM6HG256RotationZ(0.0f);
        conv.configureMMC5983MARotationZ(0.0f);
    }
};

// ---------- IMU Conversion ----------

TEST_F(SensorConverterTest, IMU_ZeroRaw_ZeroSI) {
    ISM6HG256Data raw{};
    raw.time_us = 1000;
    ISM6HG256DataSI si{};
    conv.convertISM6HG256Data(raw, si);

    EXPECT_EQ(si.time_us, 1000u);
    EXPECT_NEAR(si.low_g_acc_x, 0.0, 1e-6);
    EXPECT_NEAR(si.low_g_acc_y, 0.0, 1e-6);
    EXPECT_NEAR(si.low_g_acc_z, 0.0, 1e-6);
    EXPECT_NEAR(si.high_g_acc_x, 0.0, 1e-6);
    EXPECT_NEAR(si.high_g_acc_y, 0.0, 1e-6);
    EXPECT_NEAR(si.high_g_acc_z, 0.0, 1e-6);
    EXPECT_NEAR(si.gyro_x, 0.0, 1e-6);
    EXPECT_NEAR(si.gyro_y, 0.0, 1e-6);
    EXPECT_NEAR(si.gyro_z, 0.0, 1e-6);
}

TEST_F(SensorConverterTest, IMU_FullScale_CorrectConversion) {
    ISM6HG256Data raw{};
    raw.acc_low_raw.x = 32767;  // max positive
    raw.acc_low_raw.y = 0;
    raw.acc_low_raw.z = 0;
    ISM6HG256DataSI si{};
    conv.convertISM6HG256Data(raw, si);

    // At 16g FS: 32767 * (16/32768) * 9.80665 ≈ 16g * 9.80665 ≈ 156.9 m/s^2
    float expected = 16.0f * 9.80665f * (32767.0f / 32768.0f);
    EXPECT_NEAR(si.low_g_acc_x, expected, 0.1);
}

TEST_F(SensorConverterTest, IMU_NegativeRaw) {
    ISM6HG256Data raw{};
    raw.acc_low_raw.x = -32768; // max negative
    ISM6HG256DataSI si{};
    conv.convertISM6HG256Data(raw, si);

    EXPECT_LT(si.low_g_acc_x, 0.0);
}

TEST_F(SensorConverterTest, IMU_Rotation_Applied) {
    conv.configureISM6HG256RotationZ(90.0f); // 90 deg rotation

    ISM6HG256Data raw{};
    raw.acc_low_raw.x = 16384; // +8g at 16g FS
    raw.acc_low_raw.y = 0;
    ISM6HG256DataSI si{};
    conv.convertISM6HG256Data(raw, si);

    // With 90deg rotation: original x maps to y, and y maps to -x
    // x_out = x*cos(90) - y*sin(90) = 0
    // y_out = x*sin(90) + y*cos(90) = x_orig
    EXPECT_NEAR(si.low_g_acc_x, 0.0, 0.5);
    EXPECT_GT(si.low_g_acc_y, 0.0);
}

TEST_F(SensorConverterTest, IMU_HighGBias_Subtracted) {
    conv.setHighGBias(1.0f, 2.0f, 3.0f);

    ISM6HG256Data raw{};
    raw.acc_high_raw.x = 0;
    raw.acc_high_raw.y = 0;
    raw.acc_high_raw.z = 0;
    ISM6HG256DataSI si{};
    conv.convertISM6HG256Data(raw, si);

    // Zero raw with bias should give negative values
    EXPECT_NEAR(si.high_g_acc_x, -1.0, 0.01);
    EXPECT_NEAR(si.high_g_acc_y, -2.0, 0.01);
    EXPECT_NEAR(si.high_g_acc_z, -3.0, 0.01);
}

// The firmware sim encodes gyro counts for this converter to decode.  After
// #369 corrected the decode to 0.140 dps/LSB the sim kept 4000/32768, so every
// simulated rate reached the FC 14.7% high.  Encode as the sim does (lroundf
// into an int16) and decode here: the rate must come back.
TEST_F(SensorConverterTest, IMU_FirmwareSimGyroEncodingRoundTrips) {
    const float lsb = sim_sensor_model::kIsm6GyroDpsPerLsb;
    for (float rate : {1.0f, -250.0f, 1000.0f, 3999.0f}) {
        ISM6HG256Data raw{};
        raw.gyro_raw.z = (int16_t)lroundf(rate / lsb);
        ISM6HG256DataSI si{};
        conv.convertISM6HG256Data(raw, si);
        EXPECT_NEAR(si.gyro_z, rate, 0.5 * lsb) << "rate=" << rate;
    }
    // So the sim's per-axis rail sits where the chip's word does, 32767 x 0.140.
    EXPECT_NEAR(32767.0 * lsb, 4587.38, 0.01);
}

// ---------- Baro Conversion ----------

TEST_F(SensorConverterTest, Baro_Conversion) {
    BMP585Data raw{};
    raw.time_us = 5000;
    raw.temp_q16 = (int32_t)(25.0 * 65536);   // 25 deg C
    raw.press_q6 = (uint32_t)(101325.0 * 64);  // standard atmosphere
    BMP585DataSI si{};
    conv.convertBMP585Data(raw, si);

    EXPECT_NEAR(si.temperature, 25.0f, 0.01f);
    EXPECT_NEAR(si.pressure, 101325.0f, 1.0f);
}

// ---------- GNSS Conversion ----------

TEST_F(SensorConverterTest, GNSS_LatLonAlt) {
    GNSSData raw{};
    raw.lat_e7 = 337000000;   // 33.7 deg
    raw.lon_e7 = -1184000000; // -118.4 deg
    raw.alt_mm = 150000;       // 150 m
    raw.vel_n_mmps = 1000;     // 1 m/s north
    raw.pdop_x10 = 15;         // PDOP 1.5
    raw.num_sats = 12;
    GNSSDataSI si{};
    conv.convertGNSSData(raw, si);

    EXPECT_NEAR(si.lat, 33.7, 1e-6);
    EXPECT_NEAR(si.lon, -118.4, 1e-6);
    EXPECT_NEAR(si.alt, 150.0, 1e-3);
    EXPECT_NEAR(si.vel_n, 1.0, 1e-3);
    EXPECT_NEAR(si.pdop, 1.5f, 0.01f);
    EXPECT_EQ(si.num_sats, 12);
}

// ---------- Power Conversion ----------

TEST_F(SensorConverterTest, Power_VoltageCurrentSOC) {
    POWERData raw{};
    raw.time_us = 8000;
    // Voltage: 3.7V -> raw = (3.7/10) * 65535 = ~24248
    raw.voltage_raw = (uint16_t)(3.7 / 10.0 * 65535.0);
    // Current: 500 mA -> raw = (500/10000) * 32767 = ~1638
    raw.current_raw = (int16_t)(500.0 / 10000.0 * 32767.0);
    // SOC: 85% -> raw = (85+25) * (32767/150) = ~24024
    raw.soc_raw = (int16_t)((85.0 + 25.0) * (32767.0 / 150.0));

    POWERDataSI si{};
    conv.convertPowerData(raw, si);

    EXPECT_NEAR(si.voltage, 3.7f, 0.01f);
    EXPECT_NEAR(si.current, 500.0f, 5.0f); // some quantization error
    EXPECT_NEAR(si.soc, 85.0f, 0.5f);
}

// ---------- #850: high-side-switch rail currents ----------

TEST_F(SensorConverterTest, RailCurrents_RoundTripAtDesignPoints) {
    POWERDataSI si{};
    si.voltage = 7.4f; si.current = -500.0f; si.soc = 50.0f;
    si.cam_current   = 1.5f;   // camera design current
    si.servo_current = 3.0f;   // servo design current

    POWERData raw{};
    conv.packPowerData(si, raw);
    EXPECT_EQ(raw.cam_ma,   1500u);
    EXPECT_EQ(raw.servo_ma, 3000u);

    POWERDataSI back{};
    conv.convertPowerData(raw, back);
    EXPECT_NEAR(back.cam_current,   1.5f, 0.001f);
    EXPECT_NEAR(back.servo_current, 3.0f, 0.001f);
}

TEST_F(SensorConverterTest, RailCurrents_NaNMeansNoMonitorAndEncodesAsZero) {
    // V7/V8 and the mini have no IMON, so readRailAmps() returns NaN. That must
    // land as a clean 0 on the wire — never as a wrapped or garbage reading.
    POWERDataSI si{};
    si.cam_current   = std::numeric_limits<float>::quiet_NaN();
    si.servo_current = std::numeric_limits<float>::quiet_NaN();

    POWERData raw{};
    conv.packPowerData(si, raw);
    EXPECT_EQ(raw.cam_ma,   0u);
    EXPECT_EQ(raw.servo_ma, 0u);
}

TEST_F(SensorConverterTest, RailCurrents_NegativeClampsRatherThanWrapping) {
    // The IMON is a current SOURCE, so a negative reading is nonphysical and
    // means ADC noise around zero. Casting it straight to uint16 would wrap to
    // ~65 A and render as a catastrophic overcurrent.
    POWERDataSI si{};
    si.cam_current   = -0.004f;
    si.servo_current = -1.0f;

    POWERData raw{};
    conv.packPowerData(si, raw);
    EXPECT_EQ(raw.cam_ma,   0u);
    EXPECT_EQ(raw.servo_ma, 0u);
}

TEST_F(SensorConverterTest, RailCurrents_OverRangeSaturatesHigh) {
    // A fault must read HIGH, never low. Wrapping would turn a 70 A event into
    // a comfortable-looking 4.5 A.
    POWERDataSI si{};
    si.cam_current   = 70.0f;
    si.servo_current = 1000.0f;

    POWERData raw{};
    conv.packPowerData(si, raw);
    EXPECT_EQ(raw.cam_ma,   65535u);
    EXPECT_EQ(raw.servo_ma, 65535u);
}

// ---------- Magnetometer Conversion ----------

TEST_F(SensorConverterTest, Mag_Conversion) {
    MMC5983MAData raw{};
    raw.time_us = 2000;
    // Center value is 131072 (2^17). Raw 18-bit values.
    raw.mag_x = 131072;  // centered = 0
    raw.mag_y = 131072;
    raw.mag_z = 131072;
    MMC5983MADataSI si{};
    conv.convertMMC5983MAData(raw, si);

    EXPECT_NEAR(si.mag_x_uT, 0.0, 0.01);
    EXPECT_NEAR(si.mag_y_uT, 0.0, 0.01);
    EXPECT_NEAR(si.mag_z_uT, 0.0, 0.01);
}

// ---------- NonSensor Conversion ----------

TEST_F(SensorConverterTest, NonSensor_QuatAndPosition) {
    NonSensorData raw{};
    raw.time_us = 10000;
    // Identity quaternion * 10000
    raw.q0 = 10000;
    raw.q1 = 0;
    raw.q2 = 0;
    raw.q3 = 0;
    raw.e_pos = 5000;   // 50.0 m east (cm)
    raw.n_pos = -10000; // -100.0 m north
    raw.u_pos = 30000;  // 300.0 m up
    raw.e_vel = 200;    // 2.0 m/s
    raw.flags = NSF_LAUNCH | NSF_BURNOUT;
    raw.rocket_state = (uint8_t)INFLIGHT;
    raw.baro_alt_rate_dmps = 100; // 10.0 m/s

    NonSensorDataSI si{};
    conv.convertNonSensorData(raw, si);

    EXPECT_NEAR(si.q0, 1.0f, 1e-4f);
    EXPECT_NEAR(si.e_pos, 50.0, 0.01);
    EXPECT_NEAR(si.n_pos, -100.0, 0.01);
    EXPECT_NEAR(si.u_pos, 300.0, 0.01);
    EXPECT_NEAR(si.e_vel, 2.0, 0.01);
    EXPECT_TRUE(si.launch_flag);
    EXPECT_FALSE(si.alt_landed_flag);
    EXPECT_EQ(si.rocket_state, INFLIGHT);
    EXPECT_NEAR(si.altitude_rate, 10.0f, 0.1f);
}

// ---------- Board→Rocket Mounting Orientation ----------
// The converter applies an optional board→rocket rotation LAST (after the
// per-chip Z rotation and bias subtraction) so PCB-fact calibrations stay
// valid for any mounting.  Codes/matrices come from TR_Orientation.

#include "TR_Orientation.h"

class SensorConverterB2RTest : public SensorConverterTest {
protected:
    void configureNose(uint8_t code) {
        float R[9];
        orientCodeToMatrix(code, R);
        conv.configureBoardToRocket(R);
    }
};

TEST_F(SensorConverterB2RTest, ZNose_MovesBoardZToRocketX_AllChannels) {
    configureNose(16);  // +Z toward the nose

    ISM6HG256Data raw{};
    raw.acc_low_raw.z  = 16384;  // +8g
    raw.acc_high_raw.z = 1024;   // +8g at 256g FS
    raw.gyro_raw.z     = 8192;   // 8192 LSB × 0.140 mdps/LSB = 1146.88 dps (±4000 FS, #369)
    ISM6HG256DataSI si{};
    conv.convertISM6HG256Data(raw, si);

    const double exp_lg = 8.0 * 9.80665;
    EXPECT_NEAR(si.low_g_acc_x, exp_lg, 0.1);
    EXPECT_NEAR(si.low_g_acc_y, 0.0, 1e-6);
    EXPECT_NEAR(si.low_g_acc_z, 0.0, 1e-6);
    EXPECT_NEAR(si.high_g_acc_x, exp_lg, 0.2);
    EXPECT_NEAR(si.gyro_x, 1146.88, 0.5);  // #369: 8192 × 0.140 mdps/LSB
    EXPECT_NEAR(si.gyro_z, 0.0, 1e-6);
}

TEST_F(SensorConverterB2RTest, IdentityCode_MatchesUnconfigured) {
    ISM6HG256Data raw{};
    raw.acc_low_raw.x = 16384;
    raw.acc_low_raw.y = -8192;
    raw.acc_low_raw.z = 4096;
    ISM6HG256DataSI base{};
    conv.convertISM6HG256Data(raw, base);

    configureNose(ORIENT_CODE_IDENTITY);
    ISM6HG256DataSI si{};
    conv.convertISM6HG256Data(raw, si);

    EXPECT_DOUBLE_EQ(si.low_g_acc_x, base.low_g_acc_x);
    EXPECT_DOUBLE_EQ(si.low_g_acc_y, base.low_g_acc_y);
    EXPECT_DOUBLE_EQ(si.low_g_acc_z, base.low_g_acc_z);
}

TEST_F(SensorConverterB2RTest, HighGBias_SubtractedInBoardFrame_BeforeB2R) {
    // Bias is a board-frame (PCB) fact: with zero raw input the board-frame
    // value is (-1,-2,-3); a +Z-nose mounting must then rotate that vector,
    // giving rocket (-3,-2,+1).  If bias were wrongly subtracted after the
    // rotation the result would stay (-1,-2,-3).
    conv.setHighGBias(1.0f, 2.0f, 3.0f);
    configureNose(16);  // +Z nose: rocket x=+z_b, y=+y_b, z=-x_b

    ISM6HG256Data raw{};
    ISM6HG256DataSI si{};
    conv.convertISM6HG256Data(raw, si);

    EXPECT_NEAR(si.high_g_acc_x, -3.0, 0.01);
    EXPECT_NEAR(si.high_g_acc_y, -2.0, 0.01);
    EXPECT_NEAR(si.high_g_acc_z,  1.0, 0.01);
}

TEST_F(SensorConverterB2RTest, ComposesWithChipRotZ_ChipFirst) {
    // Chip rotZ 90° maps sensor +X → board +Y; a -Y-nose mounting (code 12)
    // then maps board +Y → rocket -X.  Sensor +X must come out at rocket -X.
    conv.configureISM6HG256RotationZ(90.0f);
    configureNose(12);  // -Y toward the nose

    ISM6HG256Data raw{};
    raw.acc_low_raw.x = 16384;  // +8g on sensor X
    ISM6HG256DataSI si{};
    conv.convertISM6HG256Data(raw, si);

    EXPECT_NEAR(si.low_g_acc_x, -8.0 * 9.80665, 0.1);
    EXPECT_NEAR(si.low_g_acc_y, 0.0, 0.5);
    EXPECT_NEAR(si.low_g_acc_z, 0.0, 1e-6);
}

TEST_F(SensorConverterB2RTest, Magnetometers_GetSameRotation) {
    configureNose(16);  // +Z nose

    MMC5983MAData mmc{};
    mmc.mag_x = 131072;
    mmc.mag_y = 131072;
    mmc.mag_z = 131072 + 16384;  // +100 µT on board Z
    MMC5983MADataSI mmc_si{};
    conv.convertMMC5983MAData(mmc, mmc_si);
    EXPECT_NEAR(mmc_si.mag_x_uT, 100.0, 0.1);   // board z → rocket x
    EXPECT_NEAR(mmc_si.mag_z_uT, 0.0, 0.1);

    IIS2MDCData iis{};
    iis.mag_x = 0;
    iis.mag_y = 0;
    iis.mag_z = 400;  // 400 * 0.15 = 60 µT on board Z
    IIS2MDCDataSI iis_si{};
    conv.convertIIS2MDCData(iis, iis_si);
    EXPECT_NEAR(iis_si.mag_x_uT, 60.0, 0.1);
    EXPECT_NEAR(iis_si.mag_z_uT, 0.0, 0.1);
}

// #1312: the IIS2MDC-named stream is scaled per the chip behind it.
TEST(SensorConverterMagType, TheQmcScaleIsSelectedByMagType) {
    SensorConverter conv;
    IIS2MDCData iis{};
    iis.mag_x = 3750;                   // one gauss of QMC5883P counts
    IIS2MDCDataSI si{};

    // Default: the big board's IIS2MDC, 0.15 µT/LSB.  Its left-handed chip
    // X comes out reversed (magTypeChipSign); the QMC5883P's does not.
    EXPECT_EQ(conv.magType(), MAG_TYPE_IIS2MDC);
    conv.convertIIS2MDCData(iis, si);
    EXPECT_NEAR(si.mag_x_uT, -562.5, 1e-6);

    // The mini: 3750 LSB is 1 G is 100 µT.
    conv.configureMagType(MAG_TYPE_QMC5883P);
    EXPECT_EQ(conv.magType(), MAG_TYPE_QMC5883P);
    conv.convertIIS2MDCData(iis, si);
    EXPECT_NEAR(si.mag_x_uT, 100.0, 1e-6);

    // A value no firmware stamps falls back to the IIS2MDC, as a pre-v6 log
    // reader would, and magType() reports what the conversion is doing.
    conv.configureMagType(0x7F);
    EXPECT_EQ(conv.magType(), MAG_TYPE_IIS2MDC);
    conv.convertIIS2MDCData(iis, si);
    EXPECT_NEAR(si.mag_x_uT, -562.5, 1e-6);
}

TEST(SensorConverterMagType, TheScaleIsAppliedBeforeRotationAndB2R) {
    // A 90° sensor→board rotation and a +Z-nose mounting must compose the same
    // way on QMC counts as on IIS2MDC counts — only the scale differs.
    SensorConverter conv;
    conv.configureMagType(MAG_TYPE_QMC5883P);
    conv.configureIIS2MDCRotationZ(90.0f);
    IIS2MDCData iis{};
    iis.mag_x = 1875;                   // 50 µT on sensor X
    IIS2MDCDataSI si{};
    conv.convertIIS2MDCData(iis, si);
    EXPECT_NEAR(si.mag_x_uT, 0.0, 1e-3);
    EXPECT_NEAR(si.mag_y_uT, 50.0, 1e-3);   // sensor +X → board +Y
    EXPECT_NEAR(si.mag_z_uT, 0.0, 1e-3);
}

// ---------- IIS2MDC chip frame: left-handed ----------
// The IIS2MDC's X/Y/Z are a left-handed set (ST: "frame is left-handed"), so
// no sensor->board rotation can map them.  With the +90 deg Rz alone, every
// V8/V9 board vector had board y reversed: the field turned the wrong way
// under roll and the fused heading was the mirror of the true one, which a
// |B| gate and a pad dip check cannot see.  These tests pin the conversion to
// physical facts that do not come from the converter:
//   - the #204 bench (config.h): board +X north reads the field on chip -Y,
//     board +X east reads it on chip -X;
//   - the chip is on the top side, its Z out of the top face (board +Z).
// So chip X lies along board -Y, chip Y along board -X, chip Z along board +Z.
namespace {

// Physical direction of each IIS2MDC chip axis, in board coordinates.
constexpr double kIisAxisInBoard[3][3] = {
    { 0.0, -1.0, 0.0},   // chip X: board -Y (east bench reading)
    {-1.0,  0.0, 0.0},   // chip Y: board -X (north bench reading)
    { 0.0,  0.0, 1.0},   // chip Z: board +Z (top side)
};

// What the chip reports for a board-frame field (µT): each axis reads the
// field's component along its physical direction, at 0.15 µT/LSB.
IIS2MDCData iisCountsFor(const double b[3])
{
    int16_t counts[3];
    for (int i = 0; i < 3; i++)
    {
        const double along = kIisAxisInBoard[i][0] * b[0] +
                             kIisAxisInBoard[i][1] * b[1] +
                             kIisAxisInBoard[i][2] * b[2];
        counts[i] = (int16_t)lround(along / MAG_UT_PER_LSB_IIS2MDC);
    }
    IIS2MDCData raw{};
    raw.mag_x = counts[0];
    raw.mag_y = counts[1];
    raw.mag_z = counts[2];
    return raw;
}

double det3(const double m[3][3])
{
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
         - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
         + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

// The converter's chip->board matrix, column by column from unit counts.
void chipToBoard(SensorConverter& conv, double m[3][3])
{
    for (int c = 0; c < 3; c++)
    {
        IIS2MDCData raw{};
        raw.mag_x = (c == 0) ? 1000 : 0;
        raw.mag_y = (c == 1) ? 1000 : 0;
        raw.mag_z = (c == 2) ? 1000 : 0;
        IIS2MDCDataSI si{};
        conv.convertIIS2MDCData(raw, si);
        const double k = 1000.0 * magTypeUtPerLsb(conv.magType());
        m[0][c] = si.mag_x_uT / k;
        m[1][c] = si.mag_y_uT / k;
        m[2][c] = si.mag_z_uT / k;
    }
}

}  // namespace

TEST(SensorConverterIIS2MDCFrame, TheBenchReadingsLandOnTheBoardAxes) {
    SensorConverter conv;
    conv.configureIIS2MDCRotationZ(config::MAG_ROT_Z_DEG_IIS2MDC);
    IIS2MDCDataSI si{};

    // Board +X north: the horizontal field reads on chip -Y and must come
    // out along board +X.
    IIS2MDCData north{};
    north.mag_y = -150;                 // 22.5 µT
    conv.convertIIS2MDCData(north, si);
    EXPECT_NEAR(si.mag_x_uT, 22.5, 1e-3);
    EXPECT_NEAR(si.mag_y_uT, 0.0, 1e-3);

    // Board +X east, +Z up: north is board +Y (left).  The field reads on
    // chip -X and must come out along board +Y.  Rz(+90) alone put it on
    // board -Y — the mirror.
    IIS2MDCData east{};
    east.mag_x = -150;
    conv.convertIIS2MDCData(east, si);
    EXPECT_NEAR(si.mag_x_uT, 0.0, 1e-3);
    EXPECT_NEAR(si.mag_y_uT, 22.5, 1e-3);

    // Chip Z is board Z.
    IIS2MDCData up{};
    up.mag_z = 300;
    conv.convertIIS2MDCData(up, si);
    EXPECT_NEAR(si.mag_z_uT, 45.0, 1e-3);
}

TEST(SensorConverterIIS2MDCFrame, AnyBoardFieldRoundTripsThroughThePhysicalChip) {
    SensorConverter conv;
    conv.configureIIS2MDCRotationZ(config::MAG_ROT_Z_DEG_IIS2MDC);
    const double fields[][3] = {{21.0, 0.0, -45.0}, {-12.0, 33.0, 18.0},
                                {5.0, -40.0, 27.0}, {0.0, 0.0, 50.0}};
    for (const auto& b : fields)
    {
        IIS2MDCDataSI si{};
        conv.convertIIS2MDCData(iisCountsFor(b), si);
        EXPECT_NEAR(si.mag_x_uT, b[0], 0.1);
        EXPECT_NEAR(si.mag_y_uT, b[1], 0.1);
        EXPECT_NEAR(si.mag_z_uT, b[2], 0.1);
    }
}

TEST(SensorConverterIIS2MDCFrame, TheFieldTurnsWithTheGyroUnderRoll) {
    // A world-fixed field seen from a body rolling at +w about board X turns
    // by -w in the body: m(t) = Rx(-w t) m(0).  Measure the roll with the
    // ISM6 through its own conversion (config.h rotation) and require the
    // converted field to turn the same way.  The mirror turned it the
    // opposite way, so here it would miss by ~2 x 29 µT x sin(10 deg).
    SensorConverter conv;
    conv.configureISM6HG256RotationZ(config::ISM6HG256_ROT_Z_DEG);
    conv.configureIIS2MDCRotationZ(config::MAG_ROT_Z_DEG_IIS2MDC);

    // Gyro: +200 dps about board X, encoded back into ISM6 chip counts
    // (board = Rz(rot) * chip, so chip = Rz(-rot) * board).
    const double rot = config::ISM6HG256_ROT_Z_DEG * M_PI / 180.0;
    const double w_board_dps = 200.0;
    const double dps_per_lsb = 4000.0 * 0.035e-3;
    ISM6HG256Data imu{};
    imu.gyro_raw.x = (int16_t)lround( w_board_dps * cos(rot) / dps_per_lsb);
    imu.gyro_raw.y = (int16_t)lround(-w_board_dps * sin(rot) / dps_per_lsb);
    ISM6HG256DataSI imu_si{};
    conv.convertISM6HG256Data(imu, imu_si);
    ASSERT_NEAR(imu_si.gyro_x, w_board_dps, 0.5);
    ASSERT_NEAR(imu_si.gyro_y, 0.0, 0.5);

    const double dt = 0.05;            // 10 deg of roll
    const double th = -imu_si.gyro_x * dt * M_PI / 180.0;
    const double m0[3] = {40.0, 15.0, -25.0};
    const double m1[3] = {m0[0],
                          m0[1] * cos(th) - m0[2] * sin(th),
                          m0[1] * sin(th) + m0[2] * cos(th)};

    IIS2MDCDataSI s0{}, s1{};
    conv.convertIIS2MDCData(iisCountsFor(m0), s0);
    conv.convertIIS2MDCData(iisCountsFor(m1), s1);
    // Predict the second sample from the first, by the gyro.
    const double py = s0.mag_y_uT * cos(th) - s0.mag_z_uT * sin(th);
    const double pz = s0.mag_y_uT * sin(th) + s0.mag_z_uT * cos(th);
    EXPECT_NEAR(s1.mag_x_uT, s0.mag_x_uT, 0.2);
    EXPECT_NEAR(s1.mag_y_uT, py, 0.2);
    EXPECT_NEAR(s1.mag_z_uT, pz, 0.2);
}

TEST(SensorConverterIIS2MDCFrame, TheReflectionIsTheIIS2MDCsAlone) {
    // The physical chip triad is left-handed, as ST says ...
    EXPECT_NEAR(det3(kIisAxisInBoard), -1.0, 1e-12);

    // ... so the IIS2MDC's conversion has to be a reflection, det -1: chip X
    // to board -Y, chip Y to board -X, chip Z to board +Z.
    SensorConverter conv;
    conv.configureIIS2MDCRotationZ(config::MAG_ROT_Z_DEG_IIS2MDC);
    double m[3][3];
    chipToBoard(conv, m);
    EXPECT_NEAR(det3(m), -1.0, 1e-9);
    const double want[3][3] = {{0, -1, 0}, {-1, 0, 0}, {0, 0, 1}};
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            EXPECT_NEAR(m[r][c], want[r][c], 1e-6) << "row " << r << " col " << c;

    // The QMC5883P's axes are right-handed, so its conversion stays a
    // rotation, det +1.
    conv.configureMagType(MAG_TYPE_QMC5883P);
    chipToBoard(conv, m);
    EXPECT_NEAR(det3(m), 1.0, 1e-9);
}
