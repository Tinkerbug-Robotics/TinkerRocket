// test_lora_roundtrip.cpp
// Tests the LoRa pack/unpack pipeline used by:
//   - OutComputer: packs telemetry into 57-byte LoRaData for TX
//   - BaseStation: receives LoRaData, unpacks to LoRaDataSI for BLE relay
//   - iOS App: receives LoRaDataSI fields via BLE JSON
//
// Verifying this roundtrip is critical because silent quantization errors
// in the pack step would propagate through the entire ground station chain.

#include <gtest/gtest.h>
#include "TR_Sensor_Data_Converter.h"
#include <cmath>
#include <cstring>

class LoRaRoundtripTest : public ::testing::Test {
protected:
    SensorConverter conv;

    // Fill with realistic mid-flight values
    LoRaDataSI makeNominal() {
        LoRaDataSI si{};
        si.network_id = 1;
        si.rocket_id = 1;
        si.num_sats = 12;
        si.pdop = 1.5f;
        si.ecef_x = -2430601.0;
        si.ecef_y = -4702443.0;
        si.ecef_z = 3546588.0;
        si.horizontal_accuracy = 3.0f;
        si.launch_flag = true;
        si.vel_u_apogee_flag = false;
        si.alt_apogee_flag = false;
        si.alt_landed_flag = false;
        si.camera_recording = true;
        si.logging_active = true;
        si.rocket_state = 3; // INFLIGHT
        si.acc_x = 0.5f;
        si.acc_y = -0.3f;
        si.acc_z = 9.8f;
        si.gyro_x = 15.0f;
        si.gyro_y = -3.0f;
        si.gyro_z = 180.0f;
        si.temp = 25.5f;
        si.voltage = 3.85f;
        si.current = 450.0f;
        si.soc = 72.0f;
        si.pressure_alt = 350.0f;
        si.altitude_rate = 45.0f;
        si.max_alt = 420.0f;
        si.max_speed = 95.0f;
        si.roll = -12.5f;
        si.pitch = 85.3f;
        si.yaw = 45.0f;
        si.q0 = 0.707f;
        si.q1 = 0.0f;
        si.q2 = 0.0f;
        si.q3 = 0.707f;
        si.speed = 95.0f;
        return si;
    }
};

TEST_F(LoRaRoundtripTest, NominalFlight_Roundtrip) {
    LoRaDataSI in = makeNominal();
    LoRaData packed{};
    conv.packLoRa(in, packed);

    LoRaDataSI out{};
    conv.unpackLoRa(packed, out);

    // Verify all fields survive roundtrip within quantization
    EXPECT_EQ(out.num_sats, 12);
    EXPECT_NEAR(out.pdop, 2.0f, 0.6f); // u8 0..100, integer only
    EXPECT_NEAR(out.ecef_x, in.ecef_x, 1.0);
    EXPECT_NEAR(out.ecef_y, in.ecef_y, 1.0);
    EXPECT_NEAR(out.ecef_z, in.ecef_z, 1.0);
    EXPECT_NEAR(out.horizontal_accuracy, 3.0f, 1.0f);

    // Flags
    EXPECT_TRUE(out.launch_flag);
    EXPECT_FALSE(out.vel_u_apogee_flag);
    EXPECT_FALSE(out.alt_apogee_flag);
    EXPECT_FALSE(out.alt_landed_flag);
    EXPECT_TRUE(out.camera_recording);
    EXPECT_TRUE(out.logging_active);
    EXPECT_EQ(out.rocket_state, 3);

    // Accel/gyro (×10 quantization -> 0.1 resolution)
    EXPECT_NEAR(out.acc_x, 0.5f, 0.1f);
    EXPECT_NEAR(out.acc_y, -0.3f, 0.1f);
    EXPECT_NEAR(out.acc_z, 9.8f, 0.1f);
    EXPECT_NEAR(out.gyro_x, 15.0f, 0.1f);
    EXPECT_NEAR(out.gyro_z, 180.0f, 0.1f);

    EXPECT_NEAR(out.temp, 25.5f, 0.1f);
    EXPECT_NEAR(out.voltage, 3.85f, 0.05f); // u8 2-10V, ~0.03 resolution
    EXPECT_NEAR(out.current, 450.0f, 1.0f);
    EXPECT_NEAR(out.soc, 72.0f, 1.0f);
    EXPECT_NEAR(out.pressure_alt, 350.0f, 1.0f);
    EXPECT_NEAR(out.altitude_rate, 45.0f, 1.0f);
    EXPECT_NEAR(out.max_alt, 420.0f, 1.0f);
    EXPECT_NEAR(out.max_speed, 95.0f, 1.0f);

    // Angles (centidegree -> 0.01 deg resolution)
    EXPECT_NEAR(out.roll, -12.5f, 0.01f);
    EXPECT_NEAR(out.pitch, 85.3f, 0.01f);
    EXPECT_NEAR(out.yaw, 45.0f, 0.01f);

    // Quaternion (×10000 -> 0.0001 resolution)
    EXPECT_NEAR(out.q0, 0.707f, 0.0001f);
    EXPECT_NEAR(out.q3, 0.707f, 0.0001f);

    EXPECT_NEAR(out.speed, 95.0f, 1.0f);
}

TEST_F(LoRaRoundtripTest, ExtremeValues_NoOverflow) {
    LoRaDataSI extreme{};
    extreme.num_sats = 127; // max for 7-bit
    extreme.pdop = 100.0f;
    extreme.ecef_x = 7000000.0;
    extreme.ecef_y = -7000000.0;
    extreme.ecef_z = 7000000.0;
    extreme.horizontal_accuracy = 100.0f;
    extreme.acc_x = 400.0f;
    extreme.acc_y = -400.0f;
    extreme.gyro_x = 4500.0f;
    extreme.temp = 200.0f;
    extreme.voltage = 10.0f;
    extreme.current = 10000.0f;
    extreme.soc = 125.0f;
    extreme.pressure_alt = 100000.0f;
    extreme.altitude_rate = 2000.0f;
    extreme.max_alt = 400000.0f; // will clamp to i24 max
    extreme.max_speed = 4000.0f;
    extreme.roll = 180.0f;
    extreme.pitch = 90.0f;
    extreme.yaw = 180.0f;
    extreme.q0 = 1.0f;
    extreme.speed = 4000.0f;
    extreme.launch_flag = true;
    extreme.alt_apogee_flag = true;
    extreme.vel_u_apogee_flag = true;
    extreme.alt_landed_flag = true;
    extreme.camera_recording = true;
    extreme.logging_active = true;
    extreme.rocket_state = 4; // LANDED

    LoRaData packed{};
    conv.packLoRa(extreme, packed);

    LoRaDataSI out{};
    conv.unpackLoRa(packed, out);

    EXPECT_EQ(out.num_sats, 127);
    EXPECT_TRUE(out.launch_flag);
    EXPECT_TRUE(out.alt_apogee_flag);
    EXPECT_TRUE(out.vel_u_apogee_flag);
    EXPECT_TRUE(out.alt_landed_flag);
    EXPECT_TRUE(out.camera_recording);
    EXPECT_TRUE(out.logging_active);
    EXPECT_EQ(out.rocket_state, 4);
    EXPECT_NEAR(out.acc_x, 400.0f, 0.1f);
    EXPECT_NEAR(out.voltage, 10.0f, 0.05f);
    EXPECT_NEAR(out.soc, 125.0f, 1.0f);
    EXPECT_LE(out.speed, 4000.0f);
}

TEST_F(LoRaRoundtripTest, i24_SignExtension) {
    // Test negative i24 values survive roundtrip (ECEF can be negative)
    LoRaDataSI in{};
    in.ecef_x = -5000000.0; // large negative
    in.ecef_y = -100.0;     // small negative
    in.ecef_z = 0.0;
    in.pressure_alt = -500.0f; // below sea level
    in.max_alt = -100.0f;

    LoRaData packed{};
    conv.packLoRa(in, packed);

    LoRaDataSI out{};
    conv.unpackLoRa(packed, out);

    EXPECT_NEAR(out.ecef_x, -5000000.0, 1.0);
    EXPECT_NEAR(out.ecef_y, -100.0, 1.0);
    EXPECT_NEAR(out.ecef_z, 0.0, 1.0);
    EXPECT_NEAR(out.pressure_alt, -500.0f, 1.0f);
    EXPECT_NEAR(out.max_alt, -100.0f, 1.0f);
}

TEST_F(LoRaRoundtripTest, RocketState_AllValues) {
    // All 5 RocketState enum values must survive the 3-bit encoding
    for (uint8_t state = 0; state <= 4; state++) {
        LoRaDataSI in{};
        in.rocket_state = state;

        LoRaData packed{};
        conv.packLoRa(in, packed);

        LoRaDataSI out{};
        conv.unpackLoRa(packed, out);

        EXPECT_EQ(out.rocket_state, state)
            << "RocketState " << (int)state << " failed roundtrip";
    }
}

TEST_F(LoRaRoundtripTest, FlagEncoding_AllCombinations) {
    // Test all flag combinations
    for (int flags = 0; flags < 32; flags++) {
        LoRaDataSI in{};
        in.launch_flag       = (flags & 1) != 0;
        in.vel_u_apogee_flag = (flags & 2) != 0;
        in.alt_apogee_flag   = (flags & 4) != 0;
        in.alt_landed_flag   = (flags & 8) != 0;
        in.camera_recording  = (flags & 16) != 0;

        LoRaData packed{};
        conv.packLoRa(in, packed);

        LoRaDataSI out{};
        conv.unpackLoRa(packed, out);

        EXPECT_EQ(out.launch_flag,       in.launch_flag)       << "flags=" << flags;
        EXPECT_EQ(out.vel_u_apogee_flag, in.vel_u_apogee_flag) << "flags=" << flags;
        EXPECT_EQ(out.alt_apogee_flag,   in.alt_apogee_flag)   << "flags=" << flags;
        EXPECT_EQ(out.alt_landed_flag,   in.alt_landed_flag)   << "flags=" << flags;
        EXPECT_EQ(out.camera_recording,  in.camera_recording)  << "flags=" << flags;
    }
}

TEST_F(LoRaRoundtripTest, ByteLevel_PackUnpack) {
    // Test the byte-level pack/unpack API used for actual LoRa TX/RX
    LoRaDataSI in = makeNominal();
    in.network_id = 42;
    in.rocket_id = 7;
    uint8_t bytes[SIZE_OF_LORA_DATA];
    conv.packLoRaData(in, bytes);

    LoRaDataSI out{};
    conv.unpackLoRa(bytes, out);

    EXPECT_EQ(out.network_id, 42);
    EXPECT_EQ(out.rocket_id, 7);
    EXPECT_EQ(out.num_sats, 12);
    EXPECT_TRUE(out.launch_flag);
    EXPECT_NEAR(out.acc_z, 9.8f, 0.1f);
}
