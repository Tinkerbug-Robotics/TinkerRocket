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
        si.next_channel_idx = 47;  // realistic mid-table hop target
        si.seq = 123;              // free-running TX seq (#105)
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
        // #191: EKF ENU velocity + burnout
        si.vel_e = 4.2f;
        si.vel_n = -7.8f;
        si.vel_u = 93.4f;
        si.burnout_detected = true;
        // #390: board→rocket orientation in flags2 bits 1-7
        si.imu_orient_code = 21;               // -Z r90
        si.imu_orient_mode = LORA2_OMODE_AUTO; // auto-snap
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
    EXPECT_EQ(out.network_id, 1);
    EXPECT_EQ(out.rocket_id, 1);
    EXPECT_EQ(out.next_channel_idx, 47);  // hop byte must roundtrip exactly
    EXPECT_EQ(out.seq, 123);              // seq byte must roundtrip exactly (#105)
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
    EXPECT_FALSE(out.sim_active);   // #835 item 9: a real flight must stay false
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

    // #191: Euler left the wire — unpack derives it from the packed
    // quaternion, so the in.roll/pitch/yaw set above are deliberately
    // ignored.  q=(0.707, 0, 0, 0.707) is a pure ~90° yaw.
    EXPECT_NEAR(out.roll, 0.0f, 0.1f);
    EXPECT_NEAR(out.pitch, 0.0f, 0.1f);
    EXPECT_NEAR(out.yaw, 90.0f, 0.1f);

    // Quaternion (×10000 -> 0.0001 resolution)
    EXPECT_NEAR(out.q0, 0.707f, 0.0001f);
    EXPECT_NEAR(out.q3, 0.707f, 0.0001f);

    // #191: speed = |v| of the transmitted components (in.speed ignored):
    // sqrt(4.2² + 7.8² + 93.4²) ≈ 93.82 — sharper than the old 1 m/s wire.
    EXPECT_NEAR(out.speed, 93.82f, 0.1f);

    // #191: EKF ENU velocity (dm/s -> 0.1 resolution) + burnout flag
    EXPECT_NEAR(out.vel_e, 4.2f, 0.05f);
    EXPECT_NEAR(out.vel_n, -7.8f, 0.05f);
    EXPECT_NEAR(out.vel_u, 93.4f, 0.05f);
    EXPECT_TRUE(out.burnout_detected);

    // #390: orientation must roundtrip exactly (flags2 bits 1-7)
    EXPECT_EQ(out.imu_orient_code, 21);
    EXPECT_EQ(out.imu_orient_mode, LORA2_OMODE_AUTO);
}

// #191: burnout=false must not bleed in from other flags (it lives in the
// new flags2 byte, not flags_state), and velocity must sign-roundtrip.
TEST_F(LoRaRoundtripTest, VelocityAndBurnout_IndependentOfFlagsState) {
    LoRaDataSI in = makeNominal();
    in.burnout_detected = false;   // every flags_state bit stays set-ish
    in.vel_e = -321.7f;
    in.vel_n = 0.0f;
    in.vel_u = -45.6f;             // descending

    LoRaData packed{};
    conv.packLoRa(in, packed);
    // burnout bit clear; the #390 orientation bits (code 21, auto) remain.
    EXPECT_EQ(packed.flags2,
              (uint8_t)((21u << LORA2_ORIENT_CODE_SHIFT)
                        | (LORA2_OMODE_AUTO << LORA2_ORIENT_MODE_SHIFT)));

    LoRaDataSI out{};
    conv.unpackLoRa(packed, out);
    EXPECT_FALSE(out.burnout_detected);
    EXPECT_TRUE(out.launch_flag);  // flags_state untouched by flags2
    EXPECT_NEAR(out.vel_e, -321.7f, 0.05f);
    EXPECT_NEAR(out.vel_n, 0.0f, 0.05f);
    EXPECT_NEAR(out.vel_u, -45.6f, 0.05f);
}

// #557: the GNSS-absent degraded-flight verdict rides the raw sensor_health
// bitfield over LoRa to the base station (packLoRa copies it verbatim), so the
// BS-relayed app session shows the degraded banner too.  Pin that the new slot
// (bits 22-23) survives the round-trip and does not bleed into a neighbour.
TEST_F(LoRaRoundtripTest, GnssAbsentHealthSlot_Roundtrip) {
    LoRaDataSI in = makeNominal();
    in.sensor_health = shSet(0u, SH_GNSS_ABSENT_SHIFT, SH_BAD);   // degraded active
    in.sensor_health = shSet(in.sensor_health, SH_STORAGE_SHIFT, SH_OK);  // neighbour
    in.sensor_health = shSet(in.sensor_health, SH_GNSS_SHIFT, SH_DEGRADED);

    LoRaData packed{};
    conv.packLoRa(in, packed);
    LoRaDataSI out{};
    conv.unpackLoRa(packed, out);

    EXPECT_EQ(shGet(out.sensor_health, SH_GNSS_ABSENT_SHIFT), SH_BAD);
    EXPECT_EQ(shGet(out.sensor_health, SH_STORAGE_SHIFT),     SH_OK);
    EXPECT_EQ(shGet(out.sensor_health, SH_GNSS_SHIFT),        SH_DEGRADED);
}

// #390: orientation field semantics — the wire sentinels are load-bearing.
// A pre-#390 frame (flags2 orientation bits zero-filled) must unpack as
// "not reported", never as a confident "+X default"; auto-exact's no-code
// sentinel must survive; burnout must not bleed into the code field.
TEST_F(LoRaRoundtripTest, Orientation_SentinelsAndLegacyFrames) {
    // Legacy / not-reported: everything zero except burnout.
    LoRaDataSI in = makeNominal();
    in.imu_orient_code = 0;
    in.imu_orient_mode = LORA2_OMODE_NONE;

    LoRaData packed{};
    conv.packLoRa(in, packed);
    EXPECT_EQ(packed.flags2, LORA2_BURNOUT);   // orientation bits all zero

    LoRaDataSI out{};
    conv.unpackLoRa(packed, out);
    EXPECT_EQ(out.imu_orient_mode, LORA2_OMODE_NONE);
    EXPECT_TRUE(out.burnout_detected);

    // Auto-exact: code sentinel 31 roundtrips.
    in.imu_orient_code = LORA2_ORIENT_CODE_NONE;
    in.imu_orient_mode = LORA2_OMODE_AUTO;
    conv.packLoRa(in, packed);
    conv.unpackLoRa(packed, out);
    EXPECT_EQ(out.imu_orient_code, LORA2_ORIENT_CODE_NONE);
    EXPECT_EQ(out.imu_orient_mode, LORA2_OMODE_AUTO);

    // Manual code 0 (+X nose) is distinguishable from "not reported"
    // purely by the mode bits.
    in.imu_orient_code = 0;
    in.imu_orient_mode = LORA2_OMODE_MANUAL;
    conv.packLoRa(in, packed);
    conv.unpackLoRa(packed, out);
    EXPECT_EQ(out.imu_orient_code, 0);
    EXPECT_EQ(out.imu_orient_mode, LORA2_OMODE_MANUAL);
}

TEST_F(LoRaRoundtripTest, ExtremeValues_NoOverflow) {
    LoRaDataSI extreme{};
    // #835 item 9 narrowed this field from 7 bits to 6: bit 6 became
    // LORA_SIM_BIT, next to LORA_LOGGING_BIT at bit 7.  63 is the new
    // ceiling.  Deliberate trade — a u-blox numSV across GPS+Galileo+
    // GLONASS+BeiDou realistically tops out near 40, so 63 keeps ~50%
    // headroom, and knowing a trace is synthetic is worth more than a
    // sat count nothing can produce.
    extreme.num_sats = 63; // max for the 6-bit field
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

    EXPECT_EQ(out.num_sats, 63);
    EXPECT_TRUE(out.launch_flag);
    EXPECT_TRUE(out.alt_apogee_flag);
    EXPECT_TRUE(out.vel_u_apogee_flag);
    EXPECT_TRUE(out.alt_landed_flag);
    EXPECT_TRUE(out.camera_recording);
    EXPECT_TRUE(out.logging_active);
    EXPECT_EQ(out.rocket_state, 4);
    EXPECT_NEAR(out.acc_x, 400.0f, 0.1f);
    // #563: gyro_x=4500 dps exceeds the ×10 int16 wire range — it must SATURATE
    // at 3276.7, not wrap to a sign-flipped value. (This assertion was missing,
    // so the overflow went unnoticed despite the test packing an extreme rate.)
    EXPECT_NEAR(out.gyro_x, 3276.7f, 0.1f);
    EXPECT_NEAR(out.voltage, 10.0f, 0.05f);
    EXPECT_NEAR(out.soc, 125.0f, 1.0f);
    EXPECT_LE(out.speed, 4000.0f);
}

// #563: gyro is encoded ×10 into an int16, so the wire range is ±3276.7 dps.
// The pack clamp used to be the sensor's ±4500 dps full-scale, so any
// |rate| > 3276.7 dps (~546 rpm) overflowed the int16 and wrapped to a
// SIGN-FLIPPED value — a spin-up read as a spin-down on the ground station.
// Guard the saturation: values in range round-trip accurately; values above it
// saturate at ±3276.7 with the correct sign, never wrapping.
TEST_F(LoRaRoundtripTest, GyroOverflow_SaturatesInsteadOfWrapping) {
    // Just inside the wire range still round-trips accurately.
    {
        LoRaDataSI in = makeNominal();
        in.gyro_x = 3200.0f;
        in.gyro_y = -3200.0f;
        LoRaData packed{};
        conv.packLoRa(in, packed);
        LoRaDataSI out{};
        conv.unpackLoRa(packed, out);
        EXPECT_NEAR(out.gyro_x, 3200.0f, 0.1f);
        EXPECT_NEAR(out.gyro_y, -3200.0f, 0.1f);
    }
    // Above the range — the default ±4000 dps FS and beyond: saturate at
    // ±3276.7, same sign, no wrap.
    for (float rate : {3277.0f, 4000.0f, 4500.0f}) {
        LoRaDataSI in = makeNominal();
        in.gyro_x = rate;    // fast spin one way
        in.gyro_y = -rate;   // fast spin the other way
        in.gyro_z = rate;
        LoRaData packed{};
        conv.packLoRa(in, packed);
        LoRaDataSI out{};
        conv.unpackLoRa(packed, out);

        // The bug's signature was a positive rate returning negative.
        EXPECT_GT(out.gyro_x, 0.0f) << "positive rate " << rate << " wrapped negative";
        EXPECT_LT(out.gyro_y, 0.0f) << "negative rate " << -rate << " wrapped positive";
        // Saturated at the wire max, not wrapped.
        EXPECT_NEAR(out.gyro_x, 3276.7f, 0.1f) << "rate=" << rate;
        EXPECT_NEAR(out.gyro_y, -3276.7f, 0.1f) << "rate=" << rate;
        EXPECT_NEAR(out.gyro_z, 3276.7f, 0.1f) << "rate=" << rate;
    }
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
    // All RocketState enum values incl. MAG_CALIBRATION (5) must survive the
    // 3-bit encoding — #386: the old pack clamp mangled 5 into LANDED (4), so
    // a bench mag-cal with the BS listening displayed a fake LANDED and
    // closed the BS log. The field holds 0..7.
    for (uint8_t state = 0; state <= 5; state++) {
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

TEST_F(LoRaRoundtripTest, Seq_FullU16Range) {
    // Proto v4 widened seq to 16 bits so the slow-hop seq-anchored
    // schedule can cover all channel counts (BW=125 → 139 channels at
    // dwell=4 needs 556 distinct seq positions, way past u8).  Verify
    // the round-trip works at byte boundaries, mid-range, and the wrap.
    const uint16_t test_seqs[] = {
        0, 1, 127, 128, 255, 256, 257,                 // u8 boundary
        1000, 16384, 32768, 49152,                     // mid-range
        65532, 65533, 65534, 65535                     // wrap edge
    };
    for (uint16_t s : test_seqs) {
        LoRaDataSI in = makeNominal();
        in.seq = s;
        // Pin neighbours so we'd notice if seq writes overflowed into them.
        in.network_id       = 42;
        in.rocket_id        = 7;
        in.next_channel_idx = 99;
        in.num_sats         = 12;
        // Payload canary: vel_e is a transmitted field (#191 — speed is
        // now derived on unpack, so it can no longer serve here).
        in.vel_e            = 1234.0f;

        LoRaData packed{};
        conv.packLoRa(in, packed);
        LoRaDataSI out{};
        conv.unpackLoRa(packed, out);

        EXPECT_EQ(out.seq, s)                  << "seq=" << s;
        EXPECT_EQ(out.network_id, 42)          << "seq=" << s;
        EXPECT_EQ(out.rocket_id,   7)          << "seq=" << s;
        EXPECT_EQ(out.next_channel_idx, 99)    << "seq=" << s;
        EXPECT_EQ(out.num_sats, 12)            << "seq=" << s;
        EXPECT_NEAR(out.vel_e, 1234.0f, 0.05f) << "seq=" << s;
    }
}


// ---------------------------------------------------------------------------
// #835 item 9 — sim_active on the LoRa wire.
//
// The FC raises NSF_SIM_ACTIVE while TR_Sensor_Collector_Sim substitutes
// IMU/baro/GNSS.  A DIRECT BLE link relayed it, but the LoRa frame had no room
// (flags_state and flags2 are both fully allocated), so the base station
// memset() left sim_active FALSE on every relayed frame — an affirmative
// "this is a real flight" rather than an absence.  A pad sim watched through
// the BS was indistinguishable from a real flight, including in the CSV the
// flight-report tooling consumes afterwards.
//
// The bit lives at num_sats bit 6, next to LORA_LOGGING_BIT at bit 7.
// ---------------------------------------------------------------------------

TEST_F(LoRaRoundtripTest, SimActiveSurvivesTheWire) {
    LoRaDataSI in = makeNominal();
    in.sim_active = true;
    LoRaData packed{};
    conv.packLoRa(in, packed);
    LoRaDataSI out{};
    conv.unpackLoRa(packed, out);
    EXPECT_TRUE(out.sim_active);
}

TEST_F(LoRaRoundtripTest, SimAndLoggingBitsAreIndependent) {
    // They share the num_sats byte, so a packing slip would couple them.
    for (bool sim : {false, true}) {
        for (bool logging : {false, true}) {
            LoRaDataSI in = makeNominal();
            in.sim_active = sim;
            in.logging_active = logging;
            LoRaData packed{};
            conv.packLoRa(in, packed);
            LoRaDataSI out{};
            conv.unpackLoRa(packed, out);
            EXPECT_EQ(out.sim_active, sim) << "sim=" << sim << " logging=" << logging;
            EXPECT_EQ(out.logging_active, logging) << "sim=" << sim << " logging=" << logging;
        }
    }
}

TEST_F(LoRaRoundtripTest, SatCountIsUnharmedByTheFlagBits) {
    // num_sats now has 6 bits (0-63) rather than 7. Real constellations top
    // out near 40, so this must be lossless across the plausible range even
    // with both flag bits set.
    for (uint8_t sats : {0, 1, 12, 31, 40, 63}) {
        LoRaDataSI in = makeNominal();
        in.num_sats = sats;
        in.sim_active = true;
        in.logging_active = true;
        LoRaData packed{};
        conv.packLoRa(in, packed);
        LoRaDataSI out{};
        conv.unpackLoRa(packed, out);
        EXPECT_EQ(out.num_sats, sats) << "sats=" << (int)sats;
        EXPECT_TRUE(out.sim_active);
        EXPECT_TRUE(out.logging_active);
    }
}

TEST_F(LoRaRoundtripTest, SatCountClampsRatherThanCorruptingTheFlags) {
    // A sender reporting an impossible sat count must not bleed into the flag
    // bits — that would turn a real flight into a reported sim.
    LoRaDataSI in = makeNominal();
    in.num_sats = 200;
    in.sim_active = false;
    in.logging_active = false;
    LoRaData packed{};
    conv.packLoRa(in, packed);
    LoRaDataSI out{};
    conv.unpackLoRa(packed, out);
    EXPECT_EQ(out.num_sats, 63);      // clamped
    EXPECT_FALSE(out.sim_active);     // NOT corrupted by the clamp
    EXPECT_FALSE(out.logging_active);
}

TEST_F(LoRaRoundtripTest, LegacySenderReadsAsNotSimulated) {
    // A pre-#835 sender leaves bit 6 clear. That must read as "not a sim",
    // matching the old behaviour rather than inventing a sim flight.
    LoRaDataSI in = makeNominal();
    in.sim_active = false;
    LoRaData packed{};
    conv.packLoRa(in, packed);
    packed.num_sats &= ~LORA_SIM_BIT;   // as an older firmware would send it
    LoRaDataSI out{};
    conv.unpackLoRa(packed, out);
    EXPECT_FALSE(out.sim_active);
    EXPECT_EQ(out.num_sats, 12);
}
