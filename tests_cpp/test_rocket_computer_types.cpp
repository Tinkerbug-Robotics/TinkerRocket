#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <map>
#include "RocketComputerTypes.h"
#include "GuidancePointGate.h"

// Verify packed struct sizes match the SIZE_OF_* constants.
// These must stay in sync because the I2S framing, binary log parser,
// and iOS app all depend on exact byte counts.

TEST(RocketComputerTypes, StructSizes_MatchConstants) {
    EXPECT_EQ(SIZE_OF_GNSS_DATA,       sizeof(GNSSData));
    EXPECT_EQ(SIZE_OF_BMP585_DATA,     sizeof(BMP585Data));
    EXPECT_EQ(SIZE_OF_ISM6HG256_DATA,  sizeof(ISM6HG256Data));
    EXPECT_EQ(SIZE_OF_MMC5983MA_DATA,  sizeof(MMC5983MAData));
    EXPECT_EQ(SIZE_OF_POWER_DATA,      sizeof(POWERData));
    EXPECT_EQ(SIZE_OF_NON_SENSOR_DATA, sizeof(NonSensorData));
    EXPECT_EQ(SIZE_OF_LORA_FAST,       sizeof(LoRaFastData));
    EXPECT_EQ(SIZE_OF_LORA_SLOW,       sizeof(LoRaSlowData));
    // The airtime budget is the LARGER frame, always (#850).
    EXPECT_EQ(SIZE_OF_LORA_BUDGET,     SIZE_OF_LORA_FAST);
}

TEST(RocketComputerTypes, KnownSizes) {
    // Hard-coded expected sizes from the wire protocol spec.
    // If any of these change, the iOS app and analysis scripts must update too.
    EXPECT_EQ(sizeof(GNSSData),       42u);
    EXPECT_EQ(sizeof(BMP585Data),     12u);
    EXPECT_EQ(sizeof(ISM6HG256Data),  22u);
    EXPECT_EQ(sizeof(MMC5983MAData),  16u);
    EXPECT_EQ(sizeof(POWERData),      14u);  // #850: v2, +cam_ma +servo_ma
    // v1 stays pinned: it is the length decoders must still accept for logs
    // written before #850, and Data_Analysis dispatches POWER on exactly these
    // two sizes.
    EXPECT_EQ(SIZE_OF_POWER_DATA_V1,  10u);
    EXPECT_EQ(sizeof(NonSensorData),  50u);  // #529: +uint16 ekf_ticks (2 B)
    EXPECT_EQ(sizeof(LoRaFrameHeader), 7u);   // #850: shared prefix, both frames
    EXPECT_EQ(sizeof(LoRaFastData),   55u);  // #850: 5-of-6 slots
    EXPECT_EQ(sizeof(LoRaSlowData),   22u);  // #850: 1-of-6 slots
    EXPECT_EQ(sizeof(LoRaUplinkData), 13u);  // uplink RSSI/SNR log record (0xF9)
    EXPECT_EQ(sizeof(FcBootStatusData), 4u); // FC->OC boot progress (0xFA)
    EXPECT_EQ(sizeof(i24le_t),         3u);
    EXPECT_EQ(sizeof(Vec3i16),         6u);
}

// FC boot progress: the app renders a step name and a stall from these, so the
// enum ordering and the degraded bits are wire contract, not implementation
// detail.  Append-only — the apps map an unknown step to a generic message.
TEST(RocketComputerTypes, FcBootStepContract) {
    EXPECT_EQ(FCB_LINKS,      0u);
    EXPECT_EQ(FCB_NVS,        1u);
    EXPECT_EQ(FCB_SENSORS,    2u);
    EXPECT_EQ(FCB_GNSS,       3u);
    EXPECT_EQ(FCB_SERVOS,     4u);
    EXPECT_EQ(FCB_COMPLETE,   5u);
    EXPECT_EQ(FCB_STEP_COUNT, 6u);
    // Degraded bits are independent — a boot can finish with several set.
    EXPECT_EQ(FCB_DEG_SENSORS, 0x01u);
    EXPECT_EQ(FCB_DEG_GNSS,    0x02u);
    EXPECT_EQ(FCB_DEG_SERVOS,  0x04u);
    EXPECT_EQ(FCB_DEG_NVS,     0x08u);
    const uint8_t all = FCB_DEG_SENSORS | FCB_DEG_GNSS | FCB_DEG_SERVOS | FCB_DEG_NVS;
    EXPECT_EQ(all, 0x0Fu) << "degraded bits must not overlap";
}

// #281/#278: the flight-log storage verdict the OC folds into sensor_health.  A
// full/failing NAND silently dropped the 2026-06-25 guided flight; these pin the
// thresholds and the bit slot so the pre-launch go/no-go can trust them.
TEST(RocketComputerTypes, StorageHealthVerdict) {
    constexpr uint32_t PA = 256;  // prealloc_blocks — one flight's reservation
    EXPECT_EQ(shStorageState(/*free=*/988, PA, /*write_fail=*/0), SH_OK);
    EXPECT_EQ(shStorageState(2 * PA,       PA, 0), SH_OK);
    EXPECT_EQ(shStorageState(2 * PA - 1,   PA, 0), SH_DEGRADED);  // room for <2 flights
    EXPECT_EQ(shStorageState(PA,           PA, 0), SH_DEGRADED);
    EXPECT_EQ(shStorageState(PA - 1,       PA, 0), SH_BAD);       // no room for next flight
    EXPECT_EQ(shStorageState(0,            PA, 0), SH_BAD);
    // A NAND write failure is BAD regardless of free space (full mid-flight /
    // bad-block run) — even with the whole chip nominally free.
    EXPECT_EQ(shStorageState(988,          PA, /*write_fail=*/1), SH_BAD);
    EXPECT_EQ(shStorageState(0, /*prealloc=*/0, 0), SH_NA);       // logger not configured
}

TEST(RocketComputerTypes, StorageHealth_BitPosition) {
    EXPECT_EQ(SH_STORAGE_SHIFT, 20u);
    for (uint8_t other : {SH_BARO_SHIFT, SH_IMU_SHIFT, SH_EKF_SHIFT, SH_MAG_SHIFT,
                          SH_GNSS_SHIFT, SH_BATT_SHIFT, SH_PYRO_SHIFT[0],
                          SH_PYRO_SHIFT[1], SH_PYRO_SHIFT[2], SH_PYRO_SHIFT[3]}) {
        EXPECT_NE(SH_STORAGE_SHIFT, other);   // clear of every other field
    }
    uint32_t f = shSet(0u, SH_STORAGE_SHIFT, SH_BAD);
    EXPECT_EQ(shGet(f, SH_STORAGE_SHIFT), SH_BAD);
    f = shSet(f, SH_PYRO_SHIFT[3], SH_OK);    // neighbouring field (shift 18) — no bleed
    EXPECT_EQ(shGet(f, SH_STORAGE_SHIFT), SH_BAD);
    EXPECT_EQ(shGet(f, SH_PYRO_SHIFT[3]), SH_OK);
}

// #557: the GNSS-absent degraded-flight slot rides sensor_health to the app on
// both the direct-BLE and LoRa/BS-relay paths.  Pin its slot clear of every
// existing item (esp. the neighbouring SH_STORAGE at 20 and the real SH_GNSS
// fix-health at 8) so the degraded-mode banner can never alias another verdict.
TEST(RocketComputerTypes, GnssAbsent_BitPosition) {
    EXPECT_EQ(SH_GNSS_ABSENT_SHIFT, 22u);
    for (uint8_t other : {SH_BARO_SHIFT, SH_IMU_SHIFT, SH_EKF_SHIFT, SH_MAG_SHIFT,
                          SH_GNSS_SHIFT, SH_BATT_SHIFT, SH_STORAGE_SHIFT,
                          SH_PYRO_SHIFT[0], SH_PYRO_SHIFT[1], SH_PYRO_SHIFT[2],
                          SH_PYRO_SHIFT[3]}) {
        EXPECT_NE(SH_GNSS_ABSENT_SHIFT, other);   // clear of every other field
    }
    // Slot fits inside the 32-bit field.
    EXPECT_LT(SH_GNSS_ABSENT_SHIFT + 2u, 32u);
    uint32_t f = shSet(0u, SH_GNSS_ABSENT_SHIFT, SH_BAD);  // degraded mode active
    EXPECT_EQ(shGet(f, SH_GNSS_ABSENT_SHIFT), SH_BAD);
    // Neighbouring SH_STORAGE (shift 20) and the real SH_GNSS (shift 8) don't bleed.
    f = shSet(f, SH_STORAGE_SHIFT, SH_OK);
    f = shSet(f, SH_GNSS_SHIFT,    SH_DEGRADED);
    EXPECT_EQ(shGet(f, SH_GNSS_ABSENT_SHIFT), SH_BAD);
    EXPECT_EQ(shGet(f, SH_STORAGE_SHIFT),     SH_OK);
    EXPECT_EQ(shGet(f, SH_GNSS_SHIFT),        SH_DEGRADED);
}

TEST(RocketComputerTypes, NSF_FlagBits_NoOverlap) {
    // bits 0-6 used; bit 7 reserved post per-fire-arming refactor.
    uint8_t all = NSF_ALT_LANDED | NSF_ALT_APOGEE | NSF_VEL_APOGEE |
                  NSF_LAUNCH | NSF_BURNOUT | NSF_GUIDANCE |
                  NSF_PYRO_ARMED;
    EXPECT_EQ(all, 0x7F);
    EXPECT_EQ(__builtin_popcount(NSF_PYRO_ARMED), 1);
    EXPECT_EQ(NSF_PYRO_ARMED, 1u << 6);
}

TEST(RocketComputerTypes, NSF2_ApogeeFlagBits_NoOverlap) {
    // apogee_flags byte: 3 apogee detector bits + 2 relocated signals
    // (reboot_recovery, guidance_enabled) that used to live in pyro_status,
    // plus the orientation-mismatch flag (bit 5), the #474 FC IMU-drop
    // witness (bit 6), and the deployment latch (bit 7).  The byte is now
    // FULL — a new signal needs a new field, not a bit.
    uint8_t all = NSF2_GPS_APOGEE | NSF2_PITCH_APOGEE | NSF2_MASTER_APOGEE |
                  NSF2_REBOOT_RECOVERY | NSF2_GUIDANCE_ENABLED |
                  NSF2_ORIENT_THRUST_MISMATCH | NSF2_FC_IMU_DROP |
                  NSF2_DEPLOYED;
    EXPECT_EQ(all, 0xFF);  // bits 0-7 used, none overlapping
    EXPECT_EQ(__builtin_popcount(NSF2_GPS_APOGEE),            1);
    EXPECT_EQ(__builtin_popcount(NSF2_PITCH_APOGEE),          1);
    EXPECT_EQ(__builtin_popcount(NSF2_MASTER_APOGEE),         1);
    EXPECT_EQ(__builtin_popcount(NSF2_REBOOT_RECOVERY),       1);
    EXPECT_EQ(__builtin_popcount(NSF2_GUIDANCE_ENABLED),      1);
    EXPECT_EQ(__builtin_popcount(NSF2_ORIENT_THRUST_MISMATCH), 1);
    EXPECT_EQ(__builtin_popcount(NSF2_FC_IMU_DROP),           1);
    EXPECT_EQ(__builtin_popcount(NSF2_DEPLOYED),              1);
    EXPECT_EQ(NSF2_DEPLOYED, 1u << 7);
}

TEST(RocketComputerTypes, PSF_FlagBits_NoOverlap) {
    // pyro_status is now exclusively pyro: 4 channels × (cont, fired).
    uint8_t all = PSF_CH1_CONT | PSF_CH1_FIRED |
                  PSF_CH2_CONT | PSF_CH2_FIRED |
                  PSF_CH3_CONT | PSF_CH3_FIRED |
                  PSF_CH4_CONT | PSF_CH4_FIRED;
    EXPECT_EQ(all, 0xFF);  // all 8 bits used, none overlapping
    EXPECT_EQ(__builtin_popcount(PSF_CH1_CONT),  1);
    EXPECT_EQ(__builtin_popcount(PSF_CH1_FIRED), 1);
    EXPECT_EQ(__builtin_popcount(PSF_CH2_CONT),  1);
    EXPECT_EQ(__builtin_popcount(PSF_CH2_FIRED), 1);
    EXPECT_EQ(__builtin_popcount(PSF_CH3_CONT),  1);
    EXPECT_EQ(__builtin_popcount(PSF_CH3_FIRED), 1);
    EXPECT_EQ(__builtin_popcount(PSF_CH4_CONT),  1);
    EXPECT_EQ(__builtin_popcount(PSF_CH4_FIRED), 1);
}

TEST(RocketComputerTypes, PyroConfigData_FourChannelLayout) {
    // 4 channels × (1 enabled + 1 mode + 4 value) = 24 bytes.
    EXPECT_EQ(sizeof(PyroConfigData), 24u);
    EXPECT_EQ(offsetof(PyroConfigData, ch1_enabled),       0u);
    EXPECT_EQ(offsetof(PyroConfigData, ch1_trigger_mode),  1u);
    EXPECT_EQ(offsetof(PyroConfigData, ch1_trigger_value), 2u);
    EXPECT_EQ(offsetof(PyroConfigData, ch2_enabled),       6u);
    EXPECT_EQ(offsetof(PyroConfigData, ch3_enabled),       12u);
    EXPECT_EQ(offsetof(PyroConfigData, ch4_enabled),       18u);
    EXPECT_EQ(offsetof(PyroConfigData, ch4_trigger_value), 20u);
}

TEST(RocketComputerTypes, GuidanceConfigData_Layout) {
    // v1 prefix: 8 floats (32) + u16 coast_delay (2) + 2 u8 (2) = 36.
    // #534 appended: 2 floats (8) + u8 guidance_law (1) = 45.
    // The append deliberately breaks the "floats first" ordering so that the
    // offsets 0..35 below stay FROZEN — that is what lets a 36-byte-era peer
    // parse a 45-byte frame's prefix instead of misparsing it.  Do not
    // "tidy" this by moving kp/kd up with the other floats.
    EXPECT_EQ(sizeof(GuidanceConfigData), 45u);
    EXPECT_EQ(offsetof(GuidanceConfigData, nav_gain),         0u);
    EXPECT_EQ(offsetof(GuidanceConfigData, max_accel_mps2),   4u);
    EXPECT_EQ(offsetof(GuidanceConfigData, accel_to_fin_deg), 8u);
    EXPECT_EQ(offsetof(GuidanceConfigData, max_fin_deg),      12u);
    EXPECT_EQ(offsetof(GuidanceConfigData, min_speed_mps),    16u);
    EXPECT_EQ(offsetof(GuidanceConfigData, target_e_m),       20u);
    EXPECT_EQ(offsetof(GuidanceConfigData, target_n_m),       24u);
    EXPECT_EQ(offsetof(GuidanceConfigData, target_alt_m),     28u);
    EXPECT_EQ(offsetof(GuidanceConfigData, coast_delay_ms),   32u);
    EXPECT_EQ(offsetof(GuidanceConfigData, enable),           34u);
    EXPECT_EQ(offsetof(GuidanceConfigData, target_mode),      35u);
    EXPECT_EQ(offsetof(GuidanceConfigData, kp_pos_per_s2),    36u);
    EXPECT_EQ(offsetof(GuidanceConfigData, kd_vel_per_s),     40u);
    EXPECT_EQ(offsetof(GuidanceConfigData, guidance_law),     44u);
    // The 45-byte frame must still fit the OC->FC combined-read window:
    // framed = 4 (sync/hdr) + 1 (type) + 1 (len) + 45 + 2 (crc) = 53, staged
    // after a 10-byte status response inside FC_COMBINED_READ_SIZE (96).
    EXPECT_LE(10u + 4u + 1u + 1u + sizeof(GuidanceConfigData) + 2u, 96u);
}

TEST(RocketComputerTypes, GuidanceLawCodes_MatchGuidanceLibrary) {
    // Wire codes mirror TR_GuidancePN::Mode by value.  The library is not
    // linked into the host test build, so this pins the wire side only; the
    // FC pins the pair with a static_assert at each configure site.
    EXPECT_EQ(GUIDE_LAW_PN,           0u);
    EXPECT_EQ(GUIDE_LAW_STATION_KEEP, 1u);
    EXPECT_EQ(GUIDE_LAW_MAX,          GUIDE_LAW_STATION_KEEP);
}

TEST(RocketComputerTypes, GuidanceTelemData_Layout) {
    // Logged as GUIDANCE_TELEM_MSG (0xCA): u32 + 7 i16 + u8 = 19, packed.
    EXPECT_EQ(sizeof(GuidanceTelemData), 19u);
    EXPECT_EQ(offsetof(GuidanceTelemData, time_us),            0u);
    EXPECT_EQ(offsetof(GuidanceTelemData, accel_cmd_n_cmps2),  4u);
    EXPECT_EQ(offsetof(GuidanceTelemData, accel_cmd_e_cmps2),  6u);
    EXPECT_EQ(offsetof(GuidanceTelemData, lateral_offset_cm),  8u);
    EXPECT_EQ(offsetof(GuidanceTelemData, los_angle_cdeg),     10u);
    EXPECT_EQ(offsetof(GuidanceTelemData, closing_vel_cmps),   12u);
    EXPECT_EQ(offsetof(GuidanceTelemData, pitch_fin_cmd_cdeg), 14u);
    EXPECT_EQ(offsetof(GuidanceTelemData, yaw_fin_cmd_cdeg),   16u);
    EXPECT_EQ(offsetof(GuidanceTelemData, guid_flags),         18u);
}

TEST(RocketComputerTypes, FinConfigData_Layout) {
    // Wire struct relayed app→OC→BS→FC; size is hardcoded in the relay paths.
    EXPECT_EQ(sizeof(FinConfigData), 18u);
    EXPECT_EQ(offsetof(FinConfigData, azimuth_deg),       0u);
    EXPECT_EQ(offsetof(FinConfigData, reverse_mask),      16u);
    EXPECT_EQ(offsetof(FinConfigData, roll_reverse_mask), 17u);
}

TEST(RocketComputerTypes, MaxPayload_CoversAllTypes) {
    // MAX_PAYLOAD must be >= every packed struct that can be a frame payload
    EXPECT_GE(MAX_PAYLOAD, sizeof(GNSSData));
    EXPECT_GE(MAX_PAYLOAD, sizeof(BMP585Data));
    EXPECT_GE(MAX_PAYLOAD, sizeof(ISM6HG256Data));
    EXPECT_GE(MAX_PAYLOAD, sizeof(MMC5983MAData));
    EXPECT_GE(MAX_PAYLOAD, sizeof(POWERData));
    EXPECT_GE(MAX_PAYLOAD, sizeof(NonSensorData));
    EXPECT_GE(MAX_PAYLOAD, sizeof(LoRaFastData));
    EXPECT_GE(MAX_PAYLOAD, sizeof(LoRaSlowData));
    EXPECT_GE(MAX_PAYLOAD, sizeof(RollProfileData));

    // MAX_FRAME = 4 (preamble) + 1 (type) + 1 (len) + MAX_PAYLOAD + 2 (CRC16)
    EXPECT_EQ(MAX_FRAME, 4u + 1u + 1u + MAX_PAYLOAD + 2u);
}

TEST(RocketComputerTypes, LoRaFlagEncoding) {
    // Verify LORA_* constants are consistent
    EXPECT_EQ(LORA_LAUNCH,     1u << 0);
    EXPECT_EQ(LORA_VEL_APOGEE, 1u << 1);
    EXPECT_EQ(LORA_ALT_APOGEE, 1u << 2);
    EXPECT_EQ(LORA_ALT_LANDED, 1u << 3);
    EXPECT_EQ(LORA_STATE_SHIFT, 4u);
    EXPECT_EQ(LORA_CAMERA_REC, 1u << 7);

    // Rocket state fits in 3 bits (values 0-4)
    for (uint8_t s = 0; s <= 4; s++) {
        uint8_t encoded = (s << LORA_STATE_SHIFT);
        uint8_t decoded = (encoded >> LORA_STATE_SHIFT) & 0x07;
        EXPECT_EQ(decoded, s);
    }
}

// ============================================================================
// Issues #40 / #41 phase 3: channel-set selection from scan
// ============================================================================

TEST(LoraNextActiveChannel, BasicWraparoundNoMask) {
    // Empty mask → just (idx + 1) mod n.
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    EXPECT_EQ(loraNextActiveChannelIdx(0,  mask, 10), 1);
    EXPECT_EQ(loraNextActiveChannelIdx(9,  mask, 10), 0);  // wrap
    EXPECT_EQ(loraNextActiveChannelIdx(5,  mask, 10), 6);
}

TEST(LoraNextActiveChannel, SkipsMaskedChannels) {
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    // Skip channels 1, 2, 3 — from idx 0 we should jump to idx 4.
    loraSkipMaskSet(mask, 1);
    loraSkipMaskSet(mask, 2);
    loraSkipMaskSet(mask, 3);
    EXPECT_EQ(loraNextActiveChannelIdx(0, mask, 10), 4);
    // From the last unmasked we should wrap past masked back to 0.
    EXPECT_EQ(loraNextActiveChannelIdx(9, mask, 10), 0);
}

TEST(LoraNextActiveChannel, SkipsAcrossWrap) {
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    // Skip 9 (last) — from 8, wrap should go to 0, not 9.
    loraSkipMaskSet(mask, 9);
    EXPECT_EQ(loraNextActiveChannelIdx(8, mask, 10), 0);
}

TEST(LoraNextActiveChannel, AllMaskedDegenerateSafe) {
    // Defensive — should never happen in practice (FCC floor prevents
    // it), but loraNextActiveChannelIdx must still make progress.
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES];
    for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) mask[i] = 0xFF;
    const uint8_t next = loraNextActiveChannelIdx(0, mask, 10);
    EXPECT_LT(next, 10);  // Some valid index in range; just don't crash.
}

TEST(LoraSelectChannelSet, NoScanResultsLeavesEmptyMask) {
    // No scan input: skip-mask all-zero, n_channels reflects the BW table.
    // Rendezvous freq is no longer computed by this function (it's
    // compile-time hardcoded — see LORA_FACTORY_RENDEZVOUS_MHZ).
    LoRaChannelSetSelection out{};
    loraSelectChannelSet(nullptr, nullptr, 0, /*bw_khz=*/250.0f, &out);
    EXPECT_EQ(out.n_channels, loraChannelCount(250.0f));
    for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++)
        EXPECT_EQ(out.skip_mask[i], 0u) << "byte " << i;
}

TEST(LoraSelectChannelSet, NoiseAboveThresholdGetsSkipped) {
    // Build a synthetic scan that covers the BW=125 hop table (139
    // channels, FCC floor 50).  Every grid point quiet at -110 dBm
    // except a couple loud spikes well above median + 15.  Loud
    // channels should be skipped while the floor stays satisfied.
    const uint8_t n_chan = loraChannelCount(125.0f);
    const size_t  N      = n_chan;
    float  freqs[160];   // > LORA_SKIP_MASK_MAX_BYTES * 8 for safety
    int8_t rssi[160];
    for (size_t i = 0; i < N; i++) {
        freqs[i] = loraChannelMHz(125.0f, (uint8_t)i);
        rssi[i]  = -110;
    }
    rssi[10] = -60;   // very loud
    rssi[55] = -50;   // very loud
    LoRaChannelSetSelection out{};
    loraSelectChannelSet(freqs, rssi, N, 125.0f, &out);

    EXPECT_EQ(out.n_channels, n_chan);
    EXPECT_TRUE (loraSkipMaskTest(out.skip_mask, 10));
    EXPECT_TRUE (loraSkipMaskTest(out.skip_mask, 55));
    EXPECT_FALSE(loraSkipMaskTest(out.skip_mask, 11));
    EXPECT_FALSE(loraSkipMaskTest(out.skip_mask, 0));
}

TEST(LoraSelectChannelSet, FccFloorEnforced_BW250) {
    // BW=250 → fhss_min=50.  If the relative threshold would skip more
    // than (n - 50), enforce the floor by keeping the K quietest active.
    const uint8_t n = loraChannelCount(250.0f);
    ASSERT_GE(n, 50u);

    constexpr size_t M = 70;  // assume n ≥ 50; build at-channel-rate scan
    ASSERT_GE(n, M / 2);
    float  freqs[M];
    int8_t rssi[M];
    for (size_t i = 0; i < M; i++) {
        freqs[i] = 902.125f + (float)i * 0.375f;
        // Make almost every channel "loud" — this would naively skip
        // them all.  A handful are quiet.
        rssi[i] = (i % 5 == 0) ? -100 : -50;
    }
    LoRaChannelSetSelection out{};
    loraSelectChannelSet(freqs, rssi, M, 250.0f, &out);

    // Count active (non-skipped).  Must be ≥ fhss_min = 50.
    uint8_t active = 0;
    for (uint8_t i = 0; i < out.n_channels; i++)
        if (!loraSkipMaskTest(out.skip_mask, i)) active++;
    EXPECT_GE(active, loraFhssMinChannels(250.0f));
}

TEST(LoraSelectChannelSet, AllQuietSkipsNothing) {
    // If every scan point is roughly the same RSSI, no channel exceeds
    // median+15 and the mask stays empty.
    constexpr size_t N = 53;
    float  freqs[N];
    int8_t rssi[N];
    for (size_t i = 0; i < N; i++) {
        freqs[i] = 902.0f + (float)i * 0.5f;
        rssi[i]  = -105 + (int8_t)(i % 3);  // tiny variation
    }
    LoRaChannelSetSelection out{};
    loraSelectChannelSet(freqs, rssi, N, 250.0f, &out);

    for (uint8_t i = 0; i < out.n_channels; i++)
        EXPECT_FALSE(loraSkipMaskTest(out.skip_mask, i)) << "channel " << (int)i;
}

TEST(LoraFhssMinChannels, BoundaryValues) {
    EXPECT_EQ(loraFhssMinChannels(125.0f), 50u);  // narrow → strict
    EXPECT_EQ(loraFhssMinChannels(250.0f), 50u);  // exactly 250 — at boundary
    EXPECT_EQ(loraFhssMinChannels(500.0f), 25u);  // wide → relaxed
}

// ============================================================================
// Issue #136: BS auto-acquire single-channel picker
// ============================================================================

TEST(LoraPickQuietestChannel, EmptyScanFallsBackToRendezvous) {
    EXPECT_FLOAT_EQ(loraPickQuietestChannelMHz(nullptr, nullptr, 0, 250.0f),
                    LORA_FACTORY_RENDEZVOUS_MHZ);
}

TEST(LoraPickQuietestChannel, PicksLowestRssiChannel) {
    // Use the channel table itself as the scan grid so every channel
    // maps to exactly one scan bin (no aliasing).  Then drop one bin's
    // RSSI far below the rest and verify the picker snaps to it.
    constexpr float BW = 250.0f;
    const uint8_t n = loraChannelCount(BW);
    ASSERT_GT(n, 5u);

    const uint8_t target_idx = (uint8_t)(n / 3);
    const float target_mhz = loraChannelMHz(BW, target_idx);

    float  freqs[256];
    int8_t rssi[256];
    for (uint8_t i = 0; i < n; i++) {
        freqs[i] = loraChannelMHz(BW, i);
        rssi[i]  = -80;
    }
    rssi[target_idx] = -120;  // clear winner

    const float picked = loraPickQuietestChannelMHz(freqs, rssi, n, BW);
    EXPECT_NEAR(picked, target_mhz, 0.01f);
}

TEST(LoraPickQuietestChannel, TiesFavorLowerIndex) {
    // When all bins are equal, the picker walks the channel table in
    // order and keeps the first one (lowest index = lowest freq, since
    // loraChannelMHz is monotonic).
    constexpr float BW = 250.0f;
    constexpr size_t N = 53;
    float  freqs[N];
    int8_t rssi[N];
    for (size_t i = 0; i < N; i++) {
        freqs[i] = 902.0f + (float)i * 0.5f;
        rssi[i]  = -95;
    }
    const float picked = loraPickQuietestChannelMHz(freqs, rssi, N, BW);
    EXPECT_FLOAT_EQ(picked, loraChannelMHz(BW, 0));
}

// ============================================================================
// Issues #40 / #41: per-packet channel hopping — state gate
// ============================================================================

TEST(ShouldHopInState, TelemetryStatesHopGroundSetupStatesDont) {
    // #150: every state that streams full-rate (2 Hz) telemetry hops —
    // that's what keeps per-frequency airtime under the FCC 15.247 FHSS
    // occupancy bound.  LANDED transmits just as hard as INFLIGHT, so it
    // hops too (parking 2 Hz on one channel is ~2.1 s per 10 s, 5x the
    // 0.4 s bound).  Ground setup states (INIT/READY/MAG_CAL) stay on
    // the static channel so config and the initial handshake stay simple.
    EXPECT_FALSE(shouldHopInState(INITIALIZATION));
    EXPECT_FALSE(shouldHopInState(READY));
    EXPECT_TRUE (shouldHopInState(PRELAUNCH));
    EXPECT_TRUE (shouldHopInState(INFLIGHT));
    EXPECT_TRUE (shouldHopInState(LANDED));
    EXPECT_FALSE(shouldHopInState(MAG_CALIBRATION));
}

TEST(ShouldHopInState, Uint8OverloadMatchesEnum) {
    // The BS receives state numerically from the LoRa downlink, so the
    // uint8_t overload must agree bit-for-bit with the enum overload.
    for (uint8_t s = 0; s <= 5; s++)
    {
        EXPECT_EQ(shouldHopInState(s),
                  shouldHopInState((RocketState)s)) << "state=" << (int)s;
    }
}

// ============================================================================
// Issues #40 / #41: channel-set helpers
// ============================================================================

TEST(LoRaChannelSet, FastPreset_BW500) {
    // BW=500 kHz at 1.5× spacing covers 902-928 MHz with ~35 channels.
    // Channel 0 sits half a BW above 902.0 (=902.25); channels never
    // straddle either band edge.
    const float bw_khz = 500.0f;
    const uint8_t n = loraChannelCount(bw_khz);
    EXPECT_GE(n, 25);  // FCC FHSS minimum for BW > 250 kHz
    EXPECT_LE(n, 60);  // sanity ceiling

    EXPECT_NEAR(loraChannelMHz(bw_khz, 0), 902.25f, 1e-3f);
    // First channel's lower edge must not cross the band low edge.
    EXPECT_GE(loraChannelMHz(bw_khz, 0) - bw_khz / 2000.0f, LORA_BAND_LO_MHZ);
    // Last channel's upper edge must not cross the band high edge.
    EXPECT_LE(loraChannelMHz(bw_khz, n - 1) + bw_khz / 2000.0f, LORA_BAND_HI_MHZ);
    // Out-of-range index returns 0.0 sentinel.
    EXPECT_FLOAT_EQ(loraChannelMHz(bw_khz, n), 0.0f);
}

TEST(LoRaChannelSet, StandardPreset_BW250) {
    const uint8_t n = loraChannelCount(250.0f);
    EXPECT_GE(n, 50);  // FCC FHSS minimum for BW ≤ 250 kHz
    EXPECT_NEAR(loraChannelMHz(250.0f, 0), 902.125f, 1e-3f);
    EXPECT_LE(loraChannelMHz(250.0f, n - 1) + 0.125f, LORA_BAND_HI_MHZ);
}

TEST(LoRaChannelSet, MaxRangePreset_BW125) {
    const uint8_t n = loraChannelCount(125.0f);
    EXPECT_GE(n, 100);  // narrow BW packs many channels
    EXPECT_LE(loraChannelMHz(125.0f, n - 1) + 0.0625f, LORA_BAND_HI_MHZ);
}

TEST(LoRaChannelSet, ChannelsAreEvenlySpaced) {
    // Adjacent centres differ by exactly 1.5 × BW.
    const float bw_khz = 250.0f;
    const float expected_step_mhz = (bw_khz / 1000.0f) * LORA_CHANNEL_SPACING_X;
    EXPECT_NEAR(loraChannelMHz(bw_khz, 1) - loraChannelMHz(bw_khz, 0),
                expected_step_mhz, 1e-4f);
    EXPECT_NEAR(loraChannelMHz(bw_khz, 10) - loraChannelMHz(bw_khz, 9),
                expected_step_mhz, 1e-4f);
}

TEST(LoRaChannelSet, ZeroOrNegativeBwReturnsZero) {
    // Defensive: callers shouldn't hit this, but we don't want UB.
    EXPECT_EQ(loraChannelCount(0.0f),  0);
    EXPECT_EQ(loraChannelCount(-1.0f), 0);
}

// ============================================================================
// Issue #71: rendezvous protocol & lock-for-flight helpers
// ============================================================================

TEST(RocketComputerTypes, RocketState_NumericValues) {
    // The wire protocol depends on these exact values — both ends must
    // agree, and the host-side BS firmware decodes uint8_t state directly.
    // If the enum order changes, the BS's freq-lock logic + everything
    // downstream breaks silently.
    EXPECT_EQ(static_cast<uint8_t>(INITIALIZATION), 0u);
    EXPECT_EQ(static_cast<uint8_t>(READY),          1u);
    EXPECT_EQ(static_cast<uint8_t>(PRELAUNCH),      2u);
    EXPECT_EQ(static_cast<uint8_t>(INFLIGHT),       3u);
    EXPECT_EQ(static_cast<uint8_t>(LANDED),         4u);
}

TEST(RocketComputerTypes, LoRaCmdHeartbeat_DoesNotCollide) {
    // The heartbeat cmd must not collide with any of the existing command
    // bytes the rocket / base station already use.  Existing commands
    // span 1..60 today; 0xFE was chosen as deliberately out-of-band so
    // we have headroom for new low-numbered commands.
    EXPECT_EQ(LORA_CMD_HEARTBEAT, 0xFE);
    EXPECT_NE(LORA_CMD_HEARTBEAT, LORA_BEACON_SYNC);
    // Above the highest currently-used command byte (60 = freq scan)
    EXPECT_GT(LORA_CMD_HEARTBEAT, 60);
}

// ============================================================================
// "LoRa off" — the transmit mute (BLE cmd 68 / uplink cmd 68)
// ============================================================================

TEST(LoraCmdSetTxDisabled, IdIsStableAndCollidesWithNothing) {
    // ONE number for two transports: the app's BLE command to a rocket and
    // the base station's relayed uplink command are the same byte, so a
    // renumber here is a coordinated firmware + both-apps change.
    EXPECT_EQ(LORA_CMD_SET_TX_DISABLED, 68u);
    // Must not land on any other LoRa uplink command.
    EXPECT_NE(LORA_CMD_SET_TX_DISABLED, LORA_CMD_CHANNEL_SET);
    EXPECT_NE(LORA_CMD_SET_TX_DISABLED, LORA_CMD_HOP_PAUSE);
    EXPECT_NE(LORA_CMD_SET_TX_DISABLED, LORA_CMD_SET_HOP_DISABLED);
    EXPECT_NE(LORA_CMD_SET_TX_DISABLED, LORA_CMD_HEARTBEAT);
    // Nor on the beacon discriminator, which shares the same air.
    EXPECT_NE(LORA_CMD_SET_TX_DISABLED, LORA_BEACON_SYNC);
}

TEST(LoraTxMuteChangeAllowed, MuteRefusedOnlyInFlight) {
    // Muting an airborne rocket throws away the only link that says where it
    // is, and nothing on the ground would notice until it landed somewhere
    // unknown.  Every other state may be muted — the pad states are the whole
    // point of the feature, and LANDED is where a recovered rocket gets shut
    // up before the drive home.
    EXPECT_FALSE(loraTxMuteChangeAllowed(true, INFLIGHT));
    EXPECT_TRUE(loraTxMuteChangeAllowed(true, INITIALIZATION));
    EXPECT_TRUE(loraTxMuteChangeAllowed(true, READY));
    EXPECT_TRUE(loraTxMuteChangeAllowed(true, PRELAUNCH));
    EXPECT_TRUE(loraTxMuteChangeAllowed(true, LANDED));
}

TEST(LoraTxMuteChangeAllowed, UnmuteAllowedEverywhere) {
    // The asymmetry is the point.  Un-muting can only ADD telemetry, and a
    // rocket that took off muted must stay recoverable — refusing this in
    // flight would close the one door left open by keeping the receiver up.
    for (uint8_t st = 0; st <= (uint8_t)LANDED; ++st)
    {
        EXPECT_TRUE(loraTxMuteChangeAllowed(false, (RocketState)st))
            << "un-mute refused in state " << (unsigned)st;
    }
}

TEST(LoraTxMuteChangeAllowed, MatchesTheUint8Overload) {
    // The uplink handler has the state as a raw wire byte; the BLE handler has
    // the enum.  Both must decide identically or the same command would mean
    // different things over the two transports.
    for (uint8_t st = 0; st <= (uint8_t)LANDED; ++st)
    {
        EXPECT_EQ(loraTxMuteChangeAllowed(true,  st),
                  loraTxMuteChangeAllowed(true,  (RocketState)st));
        EXPECT_EQ(loraTxMuteChangeAllowed(false, st),
                  loraTxMuteChangeAllowed(false, (RocketState)st));
    }
}

TEST(FreqLockForFlight, InflightLatchesOn) {
    // Any state transition into INFLIGHT must set the lock, regardless
    // of the previous value.
    EXPECT_TRUE(computeFreqLockForFlight(false, INFLIGHT));
    EXPECT_TRUE(computeFreqLockForFlight(true,  INFLIGHT));
}

TEST(FreqLockForFlight, ReadyClearsLock) {
    // Returning to READY (e.g. user reset between flights) clears the
    // lock so recovery / config changes are allowed again.
    EXPECT_FALSE(computeFreqLockForFlight(true,  READY));
    EXPECT_FALSE(computeFreqLockForFlight(false, READY));
}

TEST(FreqLockForFlight, LandedClearsLock) {
    // Critical post-flight transition: rocket lands, state goes
    // INFLIGHT → LANDED.  The lock must clear so silence recovery is
    // available again to relocate a rocket that drifted to a field 800m
    // away.  Symmetrical with READY for clearing.
    EXPECT_FALSE(computeFreqLockForFlight(true,  LANDED));
    EXPECT_FALSE(computeFreqLockForFlight(false, LANDED));
}

TEST(FreqLockForFlight, PrelaunchPreservesLock) {
    // PRELAUNCH must NOT clear the lock.  The "rocket regains GPS lock
    // on the ground after a flight" path goes LANDED → PRELAUNCH on
    // the FlightComputer; if PRELAUNCH cleared the lock we'd have
    // already cleared it on LANDED, but this guarantees the same
    // input-output if e.g. the FC briefly oscillates LANDED↔PRELAUNCH
    // around the boundary.
    EXPECT_TRUE(computeFreqLockForFlight(true,  PRELAUNCH));
    EXPECT_FALSE(computeFreqLockForFlight(false, PRELAUNCH));
}

TEST(FreqLockForFlight, InitializationPreservesLock) {
    // INITIALIZATION shouldn't toggle the lock either way — the rocket
    // is just booting and we don't have enough info to make a call.
    EXPECT_TRUE(computeFreqLockForFlight(true,  INITIALIZATION));
    EXPECT_FALSE(computeFreqLockForFlight(false, INITIALIZATION));
}

TEST(FreqLockForFlight, FullFlightSequence) {
    // Walk a typical flight start-to-finish and verify the lock is on
    // exactly during INFLIGHT (and stays on through any LANDED→PRELAUNCH
    // glitch — though here we go straight LANDED → READY for the next
    // flight prep, which clears it).
    bool locked = false;

    locked = computeFreqLockForFlight(locked, INITIALIZATION);
    EXPECT_FALSE(locked);
    locked = computeFreqLockForFlight(locked, READY);
    EXPECT_FALSE(locked);
    locked = computeFreqLockForFlight(locked, PRELAUNCH);
    EXPECT_FALSE(locked);  // unchanged from previous unlocked state
    locked = computeFreqLockForFlight(locked, INFLIGHT);
    EXPECT_TRUE(locked);   // latch on
    locked = computeFreqLockForFlight(locked, LANDED);
    EXPECT_FALSE(locked);  // post-flight clear
    locked = computeFreqLockForFlight(locked, PRELAUNCH);
    EXPECT_FALSE(locked);  // next flight: prelaunch keeps unlocked
    locked = computeFreqLockForFlight(locked, INFLIGHT);
    EXPECT_TRUE(locked);   // and re-locks for the next flight
}

TEST(FreqLockForFlight, Uint8Overload_MatchesEnumOverload) {
    // The base station receives state numerically over LoRa.  The two
    // overloads must produce identical output for every valid state.
    for (uint8_t s = 0; s <= 4; ++s) {
        for (bool prev : {false, true}) {
            EXPECT_EQ(computeFreqLockForFlight(prev, s),
                      computeFreqLockForFlight(prev, static_cast<RocketState>(s)))
                << "state=" << (int)s << " prev=" << prev;
        }
    }
}

TEST(ShouldBeaconInState, AllowsAllExceptInflight) {
    // Beaconing during INITIALIZATION is the key fix that lets the BS
    // find a rocket whose FC hasn't booted yet.  Suppressed only in
    // INFLIGHT to give telemetry every available slot.
    EXPECT_TRUE(shouldBeaconInState(INITIALIZATION));
    EXPECT_TRUE(shouldBeaconInState(READY));
    EXPECT_TRUE(shouldBeaconInState(PRELAUNCH));
    EXPECT_FALSE(shouldBeaconInState(INFLIGHT));
    EXPECT_TRUE(shouldBeaconInState(LANDED));
}

// ============================================================================
// Issue #90: coordinated hop pause (cmd 16)
// ============================================================================

TEST(LoraCmdHopPause, IdAndCapAreStable) {
    // Wire constants pinned by the BS-rocket protocol.  Bumping either
    // requires a coordinated firmware update on both sides + iOS app.
    EXPECT_EQ(LORA_CMD_HOP_PAUSE, 16u);
    EXPECT_GT(LORA_HOP_PAUSE_MAX_MS, 0u);
    EXPECT_LE(LORA_HOP_PAUSE_MAX_MS, 65535u);  // must fit u16 wire field
    // Cmd-15 and cmd-16 must not collide.
    EXPECT_NE(LORA_CMD_CHANNEL_SET, LORA_CMD_HOP_PAUSE);
}

TEST(LoraCmdHopPause, WireFormatRoundtrip) {
    // BS-side encode (mirrors startCoordinatedScan in base_station/main.cpp):
    //   payload[0..1] = duration_ms little-endian.
    // Rocket-side decode (mirrors processUplinkCommand cmd 16 arm):
    //   memcpy(&dur, payload, 2).
    for (uint16_t dur : {(uint16_t)1, (uint16_t)100, (uint16_t)12000,
                         (uint16_t)LORA_HOP_PAUSE_MAX_MS, (uint16_t)65535})
    {
        uint8_t payload[2];
        std::memcpy(payload, &dur, 2);
        // Independently verify little-endian byte order so a host with
        // a hypothetical big-endian compiler would catch a regression.
        EXPECT_EQ(payload[0], (uint8_t)(dur & 0xFF));
        EXPECT_EQ(payload[1], (uint8_t)((dur >> 8) & 0xFF));

        uint16_t decoded;
        std::memcpy(&decoded, payload, 2);
        EXPECT_EQ(decoded, dur);
    }
}

TEST(RocketLikelyHopping, HopActiveAlwaysWins) {
    // hop_active=true short-circuits — recency / state irrelevant.
    EXPECT_TRUE(rocketLikelyHopping(/*hop_active=*/true,
                                     /*last_packet_ms=*/0,
                                     /*now_ms=*/100000,
                                     /*last_rocket_state=*/READY,
                                     /*recent_threshold_ms=*/10000));
}

TEST(RocketLikelyHopping, NeverRxFalse) {
    // Fresh boot, never received a packet → not presumed hopping.
    EXPECT_FALSE(rocketLikelyHopping(false, /*last_packet_ms=*/0,
                                      100000, PRELAUNCH, 10000));
}

TEST(RocketLikelyHopping, RecentHopStateTrue) {
    // The #90 case: hop_active false (e.g., last packet had NO_HOP), but
    // we caught it 5 s ago in PRELAUNCH.  Treat as hopping for cmd 60.
    EXPECT_TRUE (rocketLikelyHopping(false, /*last_packet_ms=*/95000,
                                      100000, PRELAUNCH, 10000));
    EXPECT_TRUE (rocketLikelyHopping(false,  95000, 100000, INFLIGHT, 10000));
}

TEST(RocketLikelyHopping, RecentNonHopStateFalse) {
    // Recent RX but in READY/INIT — direct scan path is correct.
    EXPECT_FALSE(rocketLikelyHopping(false, 95000, 100000, READY,         10000));
    EXPECT_FALSE(rocketLikelyHopping(false, 95000, 100000, INITIALIZATION,10000));
    // #150: LANDED hops now (2 Hz telemetry continues after landing), so a
    // recent LANDED packet means a scan must take the coordinated-pause
    // path — a direct scan would drop the link mid-schedule.
    EXPECT_TRUE (rocketLikelyHopping(false, 95000, 100000, LANDED,        10000));
}

// ============================================================================
// Issue #105: observed-loss attribution from the per-TX seq counter
// ============================================================================

TEST(LoraComputeObservedLoss, FirstContactReturnsUnknown) {
    // No prior packet → BS can't compute loss; -1 means "unknown".
    EXPECT_EQ(loraComputeObservedLoss(/*prev_seq=*/-1, /*curr=*/0),    -1);
    EXPECT_EQ(loraComputeObservedLoss(/*prev_seq=*/-1, /*curr=*/200),  -1);
}

TEST(LoraComputeObservedLoss, ConsecutivePacketsHaveZeroLoss) {
    // Healthy link: every TX gets through, gap=0.
    EXPECT_EQ(loraComputeObservedLoss(0, 1),         0);
    EXPECT_EQ(loraComputeObservedLoss(50, 51),       0);
    EXPECT_EQ(loraComputeObservedLoss(65534, 65535), 0);
}

TEST(LoraComputeObservedLoss, SmallForwardJumpsReportRealLoss) {
    // delta=N → (N-1) packets lost between RXes.
    EXPECT_EQ(loraComputeObservedLoss(0, 2),     1);   // missed seq 1
    EXPECT_EQ(loraComputeObservedLoss(0, 11),   10);   // missed 1..10
    EXPECT_EQ(loraComputeObservedLoss(95, 100),  4);   // missed 96..99
    EXPECT_EQ(loraComputeObservedLoss(0, 500), 499);   // far inside default 1000 cap
}

TEST(LoraComputeObservedLoss, WrapAroundIsSeenAsForward) {
    // 16-bit wrap (proto v4): prev near top of u16, curr just past wrap.
    EXPECT_EQ(loraComputeObservedLoss(65530, 5),  10);  // delta=11, loss=10
    EXPECT_EQ(loraComputeObservedLoss(65535, 0),   0);  // exact wrap, no loss
    EXPECT_EQ(loraComputeObservedLoss(65000, 99), 634); // delta=635, loss=634 (still inside cap)
}

TEST(LoraComputeObservedLoss, ImplausibleDeltaReturnsUnknown) {
    // 16-bit math: a "backward" jump only looks implausible once the
    // wrap-forward distance exceeds the threshold.  A reboot near a
    // multiple of 65536 of the prior seq looks like a small forward run
    // — known limitation; cross-check rocket_state to disambiguate.
    EXPECT_EQ(loraComputeObservedLoss(/*prev=*/100, /*curr=*/99),  -1);  // delta=65535
    EXPECT_EQ(loraComputeObservedLoss(/*prev=*/500, /*curr=*/0),   -1);  // delta=65036
    // Threshold is 1000 by default; delta=1001 must trip it.
    EXPECT_EQ(loraComputeObservedLoss(0, 1001),                    -1);
    // delta=1000 is right at the threshold — should still report.
    EXPECT_EQ(loraComputeObservedLoss(0, 1000),                   999);
}

TEST(LoraComputeObservedLoss, AmbiguousRebootIsCountedAsLoss) {
    // Reboot blind spot from proto v3 is GONE for the common case: with
    // 16-bit seq, a rocket that rebooted from seq=200 to seq=0 now has
    // modular delta=65336 (way past the 1000 threshold) and is correctly
    // flagged unknown.  Documenting here so the regression doesn't
    // sneak back in if someone narrows seq again.
    EXPECT_EQ(loraComputeObservedLoss(200, 0),    -1);
    EXPECT_EQ(loraComputeObservedLoss(50000, 0),  -1);
    // The remaining blind spot: a reboot that lands within the
    // plausibility window still looks like real loss.  Operator should
    // disambiguate via rocket_state (INITIALIZATION on fresh boot).
    EXPECT_EQ(loraComputeObservedLoss(/*prev=*/0, /*curr=*/100), 99);
}

TEST(LoraComputeObservedLoss, DuplicateSeqIsAnomaly) {
    // Same seq twice in a row: protocol violation (duplicate or replay).
    // We don't try to be clever — flag as unknown so the operator sees it.
    EXPECT_EQ(loraComputeObservedLoss(7,   7),     -1);
    EXPECT_EQ(loraComputeObservedLoss(65535, 65535), -1);
}

TEST(LoraComputeObservedLoss, CustomThresholdHonoured) {
    // A test or operator can dial the plausibility cap.  At threshold=10
    // a delta of 50 is rejected even though it'd be accepted at default.
    EXPECT_EQ(loraComputeObservedLoss(0, 50, /*plausibility_max=*/10), -1);
    EXPECT_EQ(loraComputeObservedLoss(0,  9, /*plausibility_max=*/10),  8);
    EXPECT_EQ(loraComputeObservedLoss(0, 10, /*plausibility_max=*/10),  9);
}

// ============================================================================
// Issue #105 follow-up: seq-anchored slow-hop channel schedule
// ============================================================================

TEST(LoraHopChannelForSeq, DwellOneMatchesNaiveModulo) {
    // dwell=1 is the "advance every TX" baseline.  With an empty mask,
    // channel = seq % n_channels.  Both rocket and BS compute identically.
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    for (uint16_t s = 0; s < 200; s++) {
        EXPECT_EQ(loraHopChannelForSeq(s, /*dwell=*/1, mask, /*n=*/35),
                  (uint8_t)(s % 35)) << "seq=" << s;
    }
}

TEST(LoraHopChannelForSeq, DwellHoldsForNPackets) {
    // dwell=4 holds the same channel for 4 consecutive seq values then
    // advances exactly once.  This is the slow-hop property the BS
    // depends on for tolerating single missed packets.
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    constexpr uint8_t n     = 35;
    constexpr uint8_t dwell = 4;
    for (uint16_t s = 0; s < 4 * n; s++) {
        EXPECT_EQ(loraHopChannelForSeq(s, dwell, mask, n),
                  (uint8_t)((s / dwell) % n)) << "seq=" << s;
    }
}

TEST(LoraHopChannelForSeq, U8WouldStarveChannels_U16CoversAll) {
    // The motivation for proto v4 widening seq to 16 bits: with 8-bit
    // seq + dwell=4, only 256/4 = 64 distinct positions exist, so any
    // channel count > 64 starves the high-numbered channels.  Verify
    // that with u16 we actually cover all 69 channels at BW=250.
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    constexpr uint8_t n     = 69;
    constexpr uint8_t dwell = 4;
    bool seen[256] = {false};
    // (seq / dwell) needs to reach n-1 for full coverage; with dwell=4
    // and n=69 that requires seq >= 4*68 = 272 — well past u8 wrap.
    for (uint16_t s = 0; s < dwell * n; s++) {
        seen[loraHopChannelForSeq(s, dwell, mask, n)] = true;
    }
    for (uint8_t i = 0; i < n; i++) {
        EXPECT_TRUE(seen[i]) << "channel " << (int)i << " never visited";
    }
}

TEST(LoraHopChannelForSeq, SkipMaskExcludesMaskedChannels) {
    // Channels marked in the skip mask are never returned.  Active
    // channels are visited in their natural order.
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    loraSkipMaskSet(mask, 3);
    loraSkipMaskSet(mask, 7);
    loraSkipMaskSet(mask, 8);
    constexpr uint8_t n     = 10;
    constexpr uint8_t dwell = 1;
    for (uint16_t s = 0; s < 50; s++) {
        const uint8_t ch = loraHopChannelForSeq(s, dwell, mask, n);
        EXPECT_FALSE(loraSkipMaskTest(mask, ch))
            << "seq=" << s << " landed on masked channel " << (int)ch;
    }
}

TEST(LoraHopChannelForSeq, SkipMaskWalksActiveChannelsInOrder) {
    // Active channels: 0, 1, 2, 4, 5, 6, 9 (mask 3,7,8 + 7 active).
    // dwell=1: schedule cycles 0→1→2→4→5→6→9→0→...
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    loraSkipMaskSet(mask, 3);
    loraSkipMaskSet(mask, 7);
    loraSkipMaskSet(mask, 8);
    const uint8_t expected[] = { 0, 1, 2, 4, 5, 6, 9, 0, 1, 2, 4, 5, 6, 9 };
    for (size_t i = 0; i < sizeof(expected); i++) {
        EXPECT_EQ(loraHopChannelForSeq((uint16_t)i, 1, mask, 10), expected[i])
            << "seq=" << i;
    }
}

TEST(LoraHopChannelForSeq, BothSidesAgreeAcrossWrapBoundary) {
    // The seq-anchored property means BS and rocket compute identically
    // from the same seq.  Hammer the wrap point to verify behaviour is
    // continuous across u16 rollover.
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    constexpr uint8_t n     = 69;
    constexpr uint8_t dwell = 4;
    // Both sides (rocket-perspective vs BS-perspective) compute the
    // same value for any seq — just confirm the call is pure and
    // deterministic at the wrap.
    for (uint32_t s = 65530; s < 65540; s++) {
        const uint16_t s16 = (uint16_t)s;
        const uint8_t ch_a = loraHopChannelForSeq(s16, dwell, mask, n);
        const uint8_t ch_b = loraHopChannelForSeq(s16, dwell, mask, n);
        EXPECT_EQ(ch_a, ch_b);
    }
    // After u16 wrap, schedule continues from seq=0,1,2,3 → channel 0.
    EXPECT_EQ(loraHopChannelForSeq(0, dwell, mask, n), 0);
    EXPECT_EQ(loraHopChannelForSeq(3, dwell, mask, n), 0);
    EXPECT_EQ(loraHopChannelForSeq(4, dwell, mask, n), 1);
}

TEST(LoraHopChannelForSeq, ZeroOrInvalidArgsAreSafe) {
    // Defensive: callers shouldn't hit these but we don't want UB.
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    EXPECT_EQ(loraHopChannelForSeq(0, 4, mask, 0),  0);  // n_channels=0
    EXPECT_EQ(loraHopChannelForSeq(7, 0, mask, 35), 0);  // dwell=0
}

TEST(LoraHopChannelForSeq, AllMaskedFallsBackToRawMod) {
    // FCC floor prevents this in production, but the helper must not
    // crash.  Falls back to raw modulo so it still makes progress.
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES];
    for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) mask[i] = 0xFF;
    constexpr uint8_t n     = 16;
    constexpr uint8_t dwell = 4;
    EXPECT_EQ(loraHopChannelForSeq(0,  dwell, mask, n), 0);
    EXPECT_EQ(loraHopChannelForSeq(4,  dwell, mask, n), 1);
    EXPECT_EQ(loraHopChannelForSeq(60, dwell, mask, n), 15);
}

TEST(RocketLikelyHopping, StaleRxFalseEvenInHopState) {
    // Same recency window: ON the boundary should still pass; one ms
    // past it should fail.  Keeps the threshold intent unambiguous.
    EXPECT_TRUE (rocketLikelyHopping(false, /*last_packet_ms=*/90000,
                                      100000, PRELAUNCH, 10000));   // exactly 10 s — inclusive
    EXPECT_FALSE(rocketLikelyHopping(false, /*last_packet_ms=*/89999,
                                      100000, PRELAUNCH, 10000));   // 10001 ms ago — too stale
}

// ============================================================================
// LoRa RX SNR floor (#90 follow-up)
// ============================================================================

TEST(LoraMinValidSnrDb, MatchesPerSfTheoreticalFloor) {
    // Per-SF demod-floor − 2 dB margin.  Pinned so a typo in the formula
    // is caught immediately.
    EXPECT_FLOAT_EQ(loraMinValidSnrDb(6),   -7.0f);
    EXPECT_FLOAT_EQ(loraMinValidSnrDb(7),   -9.5f);
    EXPECT_FLOAT_EQ(loraMinValidSnrDb(8),  -12.0f);
    EXPECT_FLOAT_EQ(loraMinValidSnrDb(9),  -14.5f);
    EXPECT_FLOAT_EQ(loraMinValidSnrDb(10), -17.0f);
    EXPECT_FLOAT_EQ(loraMinValidSnrDb(11), -19.5f);
    EXPECT_FLOAT_EQ(loraMinValidSnrDb(12), -22.0f);
}

TEST(LoraMinValidSnrDb, ClampsOutOfRange) {
    // Defensive: a stale SF reading (e.g., uninitialised radio) shouldn't
    // produce a NaN threshold or wrap into a positive number.
    EXPECT_FLOAT_EQ(loraMinValidSnrDb(0),   loraMinValidSnrDb(6));
    EXPECT_FLOAT_EQ(loraMinValidSnrDb(5),   loraMinValidSnrDb(6));
    EXPECT_FLOAT_EQ(loraMinValidSnrDb(13),  loraMinValidSnrDb(12));
    EXPECT_FLOAT_EQ(loraMinValidSnrDb(255), loraMinValidSnrDb(12));
}

TEST(LoraMinValidSnrDb, RejectsFieldLogFalsePositive) {
    // The motivating case: 2026-04-29 field log saw a CRC-passing
    // decode at SNR=-12.8 dB on SF8 that was clearly garbage (zeroed
    // payload, RSSI=-110 dBm).  Threshold for SF8 must reject it.
    constexpr float field_log_snr = -12.8f;
    EXPECT_LT(field_log_snr, loraMinValidSnrDb(8));
}

TEST(LoraMinValidSnrDb, AcceptsGenuineBorderlinePackets) {
    // Conversely, a genuine packet at the SF8 sensitivity floor (-10 dB)
    // must pass, plus a small margin for noise on the SNR estimate.
    EXPECT_GE(-10.0f, loraMinValidSnrDb(8));   // sensitivity floor passes
    EXPECT_GE(-11.0f, loraMinValidSnrDb(8));   // 1 dB below sensitivity still passes
    // SF12 at sensitivity (-20 dB) must also pass.
    EXPECT_GE(-20.0f, loraMinValidSnrDb(12));
}

// --- Full config report (#915) ---
// The OC parses this by memcpy of the whole struct and turns it into readback
// JSON, so a member reordered on the FC side silently reinterprets every
// field after it — and the app would then display the result as VERIFIED,
// which is worse than the "cannot verify" state this frame exists to remove.
TEST(RocketComputerTypes, ConfigReportData_Layout) {
    EXPECT_EQ(sizeof(ConfigReportData), 169u);
    // Rides the same I2S frame path as everything else FC→OC.
    EXPECT_LE(sizeof(ConfigReportData), MAX_PAYLOAD);

    // time_us first, so the generic frame parser's "timestamp = first 4
    // payload bytes" convention still holds for this type.
    EXPECT_EQ(offsetof(ConfigReportData, time_us),            0u);
    EXPECT_EQ(offsetof(ConfigReportData, version),            4u);
    EXPECT_EQ(offsetof(ConfigReportData, imu_orient_setting), 5u);
    EXPECT_EQ(offsetof(ConfigReportData, flags),              6u);

    // Composed from the structs the app writes; their own offsetof pins guard
    // the internals, these pin where each one starts.
    EXPECT_EQ(offsetof(ConfigReportData, servo),              8u);
    EXPECT_EQ(offsetof(ConfigReportData, fin),               30u);
    EXPECT_EQ(offsetof(ConfigReportData, guidance),          48u);
    EXPECT_EQ(offsetof(ConfigReportData, roll),              93u);

    // The starts are the running sum of the nested sizes — spelled out so a
    // nested struct that grows fails HERE, naming itself, instead of only
    // tripping the total-size check above.
    EXPECT_EQ(sizeof(ServoConfigData),    22u);
    EXPECT_EQ(sizeof(FinConfigData),      18u);
    EXPECT_EQ(sizeof(GuidanceConfigData), 45u);
    EXPECT_EQ(sizeof(RollProfileData),    76u);

    // Flag bits are wire ABI: the OC reads F_ORIENT_FROM_NVS to decide
    // whether to leave the FC's orientation alone or re-push its own.
    EXPECT_EQ(ConfigReportData::F_SOUNDS,           0u);
    EXPECT_EQ(ConfigReportData::F_ORIENT_FROM_NVS,  1u);
    EXPECT_EQ(ConfigReportData::VERSION,            1u);
}

// --- Flight settings snapshot (#165) ---
// The settings frame is decoded by the iOS app (SensorTypes.swift) and built
// by the FC (flight_computer/main.cpp) at byte-exact offsets. Lock the layout
// here so a struct edit can't silently desync the cross-language decode.
TEST(RocketComputerTypes, FlightSettingsData_Layout) {
    // v2: +12 b2r orientation; v3: +8 fin cal; v5: +2 imu rate;
    // v6: +9 flown guidance target (#435).
    EXPECT_EQ(sizeof(FlightSettingsData), 222u);
    // 219 <= MAX_PAYLOAD (224, FlightSnapshotData-bound) — the v6 tail was
    // deliberately E/N+src, not lat/lon+E/N, to stay under this ceiling
    // WITHOUT growing the I2S frame bound.
    EXPECT_LE(sizeof(FlightSettingsData), MAX_PAYLOAD);

    // v7 (#837 item 6): GNSS OTP state.
    EXPECT_EQ(offsetof(FlightSettingsData, gnss_otp_state),     219u);
    // v8: roll-control speed gate is the tail, 0.1 m/s units.
    EXPECT_EQ(offsetof(FlightSettingsData, roll_min_speed_dmps), 220u);
    EXPECT_EQ(FlightSettingsData::VERSION,                      8u);
    EXPECT_EQ(offsetof(FlightSettingsData, time_us),            0u);
    EXPECT_EQ(offsetof(FlightSettingsData, version),            4u);
    EXPECT_EQ(offsetof(FlightSettingsData, flags),              5u);
    EXPECT_EQ(offsetof(FlightSettingsData, roll_delay_ms),      6u);
    EXPECT_EQ(offsetof(FlightSettingsData, kp),                 8u);
    EXPECT_EQ(offsetof(FlightSettingsData, min_cmd_deg),        24u);
    EXPECT_EQ(offsetof(FlightSettingsData, kp_angle),           32u);
    EXPECT_EQ(offsetof(FlightSettingsData, gs_v_ref),           40u);
    EXPECT_EQ(offsetof(FlightSettingsData, roll_rate_set_point), 52u);
    EXPECT_EQ(offsetof(FlightSettingsData, ism6_low_g_fs_g),    56u);
    EXPECT_EQ(offsetof(FlightSettingsData, ism6_high_g_fs_g),   57u);
    EXPECT_EQ(offsetof(FlightSettingsData, ism6_gyro_fs_dps),   59u);
    EXPECT_EQ(offsetof(FlightSettingsData, servo_bias_us),      61u);
    EXPECT_EQ(offsetof(FlightSettingsData, servo_hz),           69u);
    EXPECT_EQ(offsetof(FlightSettingsData, servo_min_us),       71u);
    EXPECT_EQ(offsetof(FlightSettingsData, servo_max_us),       73u);
    EXPECT_EQ(offsetof(FlightSettingsData, camera_type),        75u);
    EXPECT_EQ(offsetof(FlightSettingsData, pyro),               76u);
    EXPECT_EQ(offsetof(FlightSettingsData, fw_git_sha),         100u);  // 76 + 24
    EXPECT_EQ(offsetof(FlightSettingsData, roll_profile),       112u);

    // Embedded PyroConfigData reaches the right absolute offsets.
    EXPECT_EQ(offsetof(FlightSettingsData, pyro) + offsetof(PyroConfigData, ch1_trigger_value), 78u);
    EXPECT_EQ(offsetof(FlightSettingsData, pyro) + offsetof(PyroConfigData, ch2_enabled),       82u);
    EXPECT_EQ(offsetof(FlightSettingsData, pyro) + offsetof(PyroConfigData, ch2_trigger_value), 84u);
    EXPECT_EQ(offsetof(FlightSettingsData, pyro) + offsetof(PyroConfigData, ch3_enabled),       88u);
    EXPECT_EQ(offsetof(FlightSettingsData, pyro) + offsetof(PyroConfigData, ch4_enabled),       94u);
    EXPECT_EQ(offsetof(FlightSettingsData, pyro) + offsetof(PyroConfigData, ch4_trigger_value), 96u);

    // Roll profile waypoints start (RollProfileData @ 112, after num+pad).
    EXPECT_EQ(offsetof(FlightSettingsData, roll_profile) + offsetof(RollProfileData, waypoints), 116u);

    // v2 board→rocket orientation tail — appended after the full v1 layout
    // (188 bytes) so v1 parsers still decode their prefix unchanged.
    EXPECT_EQ(offsetof(FlightSettingsData, b2r_code),          188u);
    EXPECT_EQ(offsetof(FlightSettingsData, b2r_mode),          189u);
    EXPECT_EQ(offsetof(FlightSettingsData, b2r_residual_cdeg), 190u);
    EXPECT_EQ(offsetof(FlightSettingsData, b2r_q),             192u);

    // v3 fin-angle calibration tail (#267) — appended after b2r_q (192 + 8 = 200)
    // so v1/v2 parsers decode their prefix unchanged.
    EXPECT_EQ(offsetof(FlightSettingsData, fin_min_deg),       200u);
    EXPECT_EQ(offsetof(FlightSettingsData, fin_max_deg),       204u);
    EXPECT_EQ(offsetof(FlightSettingsData, ism6_update_rate_hz), 208u);

    // v6 flown-guidance-target tail (#435) — appended after
    // ism6_update_rate_hz (208 + 2 = 210) so v1-v5 parsers decode their
    // prefix unchanged.
    EXPECT_EQ(offsetof(FlightSettingsData, guid_tgt_e_m),      210u);
    EXPECT_EQ(offsetof(FlightSettingsData, guid_tgt_n_m),      214u);
    EXPECT_EQ(offsetof(FlightSettingsData, guid_tgt_src),      218u);
}

// --- Flight snapshot (crash recovery) ---
// The snapshot is built and re-parsed at byte-exact offsets by the FC
// (I2S→OC MRAM→I2C round trip) and the mini (NAND log stream tail-scan).
// Lock the layout so a struct edit can't silently desync the two ends —
// especially sim_flight (v4), the byte that keeps a mid-sim reboot from
// restoring to LIVE INFLIGHT with bench igniters connected.
TEST(RocketComputerTypes, FlightSnapshotData_Layout) {
    // 224 = one I2S frame, one I2C TX response, and the OC MRAM slot bound.
    EXPECT_EQ(sizeof(FlightSnapshotData), 224u);
    EXPECT_EQ(sizeof(FlightSnapshotData), (size_t)MAX_PAYLOAD);
    // v4: sim_flight reclaimed from the header pad.  Restore paths refuse
    // any other version — a v3 frame can't prove it wasn't a sim flight.
    EXPECT_EQ(FlightSnapshotData::VERSION, 4u);

    EXPECT_EQ(offsetof(FlightSnapshotData, magic),               0u);
    EXPECT_EQ(offsetof(FlightSnapshotData, version),             4u);
    EXPECT_EQ(offsetof(FlightSnapshotData, rocket_state),        5u);
    EXPECT_EQ(offsetof(FlightSnapshotData, sim_flight),          6u);
    EXPECT_EQ(offsetof(FlightSnapshotData, flight_elapsed_ms),   8u);
    EXPECT_EQ(offsetof(FlightSnapshotData, apogee_elapsed_ms),  12u);
    EXPECT_EQ(offsetof(FlightSnapshotData, burnout_elapsed_ms), 16u);
    EXPECT_EQ(offsetof(FlightSnapshotData, pyro_apogee_detected), 20u);
    EXPECT_EQ(offsetof(FlightSnapshotData, pyro1_fired),        21u);
    EXPECT_EQ(offsetof(FlightSnapshotData, pyro4_fired),        24u);
    EXPECT_EQ(offsetof(FlightSnapshotData, b2r_code),           25u);
    EXPECT_EQ(offsetof(FlightSnapshotData, b2r_mode),           26u);
    // #834 item 4: reclaimed from pad2[1] — same offset, same struct size, so
    // no VERSION bump. 0 (every older writer's value) means "pad datum not
    // known to be converged", which disables the GNSS main-deploy backstop.
    EXPECT_EQ(offsetof(FlightSnapshotData, ref_datum_converged), 27u);
    EXPECT_EQ(sizeof(FlightSnapshotData::ref_datum_converged),    1u);
    EXPECT_EQ(offsetof(FlightSnapshotData, ground_pressure_pa), 28u);
    EXPECT_EQ(offsetof(FlightSnapshotData, ref_lat_rad),        32u);
    EXPECT_EQ(offsetof(FlightSnapshotData, ref_alt_m),          48u);
    EXPECT_EQ(offsetof(FlightSnapshotData, ekf_initialized),    56u);
    EXPECT_EQ(offsetof(FlightSnapshotData, ekf_pos_rrm),        60u);
    EXPECT_EQ(offsetof(FlightSnapshotData, ekf_vel_ned_mps),    84u);
    EXPECT_EQ(offsetof(FlightSnapshotData, ekf_quat),           96u);
    EXPECT_EQ(offsetof(FlightSnapshotData, ekf_P_diag),        136u);
    EXPECT_EQ(offsetof(FlightSnapshotData, ekf_t_prev_us),     196u);
    EXPECT_EQ(offsetof(FlightSnapshotData, ekf_euler),         200u);
    EXPECT_EQ(offsetof(FlightSnapshotData, b2r_q),             212u);
    // CRC32 covers everything before it — both computeSnapshotCRC()
    // implementations hash [0, offsetof(crc32)).
    EXPECT_EQ(offsetof(FlightSnapshotData, crc32),             220u);
}

TEST(RocketComputerTypes, FlightSettings_FlagBits_NoOverlap) {
    uint8_t all = (uint8_t)((1u << FlightSettingsData::F_USE_ANGLE_CONTROL) |
                            (1u << FlightSettingsData::F_GAIN_SCHEDULE) |
                            (1u << FlightSettingsData::F_GUIDANCE) |
                            (1u << FlightSettingsData::F_SERVO_ENABLED) |
                            (1u << FlightSettingsData::F_FW_DIRTY) |
                            (1u << FlightSettingsData::F_SOUNDS) |
                            (1u << FlightSettingsData::F_GUIDANCE_STATION_KEEP) |
                            (1u << FlightSettingsData::F_IMU_RATE_DYNAMIC));
    EXPECT_EQ(all, 0xFFu);  // bits 0-7, no overlap — the flags byte is now FULL
}

// ============================================================================
// Dynamic IMU logging rate (BLE cmd 67, IMU_RATE_DYNAMIC)
// ============================================================================
// The dynamic sentinel rides in the existing 2-byte rate_hz field. Two
// validators exist on purpose and must NOT be conflated: imuRateValid() gates
// what may be programmed as a chip ODR, imuRateSettingValid() gates what a
// user may select.
TEST(RocketComputerTypes, ImuRate_DynamicSentinel_IsNotAProgrammableOdr) {
    EXPECT_TRUE(imuRateIsDynamic(IMU_RATE_DYNAMIC));
    EXPECT_TRUE(imuRateSettingValid(IMU_RATE_DYNAMIC));
    // Must stay false: SensorCollector::setIsm6Rate() is guarded by this, and
    // handing it the mode sentinel would try to program a 0 Hz ODR.
    EXPECT_FALSE(imuRateValid(IMU_RATE_DYNAMIC));

    for (uint16_t hz : IMU_RATE_OPTIONS_HZ) {
        EXPECT_TRUE(imuRateValid(hz)) << hz;
        EXPECT_TRUE(imuRateSettingValid(hz)) << hz;
        EXPECT_FALSE(imuRateIsDynamic(hz)) << hz;
    }

    for (uint16_t hz : {1u, 500u, 1000u, 4000u, 7680u, 65535u}) {
        EXPECT_FALSE(imuRateValid((uint16_t)hz)) << hz;
        EXPECT_FALSE(imuRateSettingValid((uint16_t)hz)) << hz;
    }
}

// The dynamic endpoints must be real whitelist steps: the FC programs them
// straight into the ODR ladder, so a value the chip cannot reach would leave
// the collector at whatever it was and the mode would silently do nothing.
TEST(RocketComputerTypes, ImuRate_DynamicEndpoints_AreProgrammable) {
    EXPECT_TRUE(imuRateValid(IMU_RATE_DYNAMIC_BOOST_HZ));
    EXPECT_TRUE(imuRateValid(IMU_RATE_DYNAMIC_POST_HZ));
    EXPECT_GT(IMU_RATE_DYNAMIC_BOOST_HZ, IMU_RATE_DYNAMIC_POST_HZ);
}

TEST(RocketComputerTypes, ImuRate_Resolve) {
    // Dynamic: boost rate until deployment latches, post rate after.
    EXPECT_EQ(imuRateResolve(IMU_RATE_DYNAMIC, false), IMU_RATE_DYNAMIC_BOOST_HZ);
    EXPECT_EQ(imuRateResolve(IMU_RATE_DYNAMIC, true),  IMU_RATE_DYNAMIC_POST_HZ);
    // Fixed: the deployment latch must not move the rate.
    for (uint16_t hz : IMU_RATE_OPTIONS_HZ) {
        EXPECT_EQ(imuRateResolve(hz, false), hz);
        EXPECT_EQ(imuRateResolve(hz, true),  hz);
    }
}

// ============================================================================
// #435: Drift-Cast guidance point (BLE cmd 28)
// ============================================================================
// The 20-byte GuidancePointData must be byte-for-byte the payload the iOS app
// builds in BLEDevice.sendGuidancePoint ({lat f64, lon f64, alt f32} LE) —
// there is no version field, so the layout IS the contract.
TEST(RocketComputerTypes, GuidancePointData_Layout) {
    EXPECT_EQ(sizeof(GuidancePointData), 20u);
    EXPECT_EQ(offsetof(GuidancePointData, lat_deg), 0u);
    EXPECT_EQ(offsetof(GuidancePointData, lon_deg), 8u);
    EXPECT_EQ(offsetof(GuidancePointData, alt_m),   16u);
}

// OutStatusQueryData current tail: the authoritative sizeof/offsetof
// static_asserts live in RocketComputerTypes.h itself (this test target
// recompiles the header, so they fire in CI); pin the version semantics here
// so a format bump can't ship without a conscious edit.
TEST(RocketComputerTypes, OutStatusQuery_MagType_V6) {
    EXPECT_EQ(sizeof(OutStatusQueryData), 42u);  // v6: +mag_type (v5 was 41: #435)
    // The zeroed default must decode as the big board's chip — every pre-v6
    // reader assumption and every zero-initialized builder then agree.
    EXPECT_EQ(MAG_TYPE_IIS2MDC, 0);
    EXPECT_EQ(MAG_TYPE_QMC5883P, 1);
}

// The FC's cmd-28 acceptance gate (GuidancePointGate.h) as a pure function —
// each reject path, the gate ORDER, and the radius boundary are pinned here
// (MramDirtyPolicy precedent: policy-as-pure-function, host-tested).
namespace {
constexpr float kMaxR = 100.0f;  // config::GUIDANCE_MAX_AIM_RADIUS_M

// All-pass baseline; individual tests break one input at a time.
uint8_t gateWith(bool ready_or_prelaunch = true, bool lockout = false,
                 uint8_t law = GUIDE_LAW_STATION_KEEP, bool have_ref = true,
                 double lat = 38.0, double lon = -122.0, float alt = 250.0f,
                 float r = 15.0f, float max_r = kMaxR) {
    return guidancePointRc(ready_or_prelaunch, lockout, law, have_ref,
                           lat, lon, alt, r, max_r);
}
}  // namespace

TEST(GuidancePointGate, AcceptsBaseline) {
    EXPECT_EQ(gateWith(), GUID_RC_ACCEPTED);
}

TEST(GuidancePointGate, RejectsWrongStateAndLockout) {
    EXPECT_EQ(gateWith(/*ready_or_prelaunch=*/false), GUID_RC_REJ_STATE);
    EXPECT_EQ(gateWith(true, /*lockout=*/true),       GUID_RC_REJ_STATE);
    // Both wrong: still STATE (single code for the whole class).
    EXPECT_EQ(gateWith(false, true),                  GUID_RC_REJ_STATE);
}

TEST(GuidancePointGate, RejectsBadValues) {
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       NAN, -122.0),                     GUID_RC_REJ_BADVAL);
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       38.0, INFINITY),                  GUID_RC_REJ_BADVAL);
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       38.0, -122.0, NAN),               GUID_RC_REJ_BADVAL);
    // alt has NO range bound, so isfinite is the ONLY check standing between
    // an infinite altitude and (int16_t)lround(INFINITY) in the echo —
    // infinities pass isnan-only mutants, hence pinned explicitly.
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       38.0, -122.0, INFINITY),          GUID_RC_REJ_BADVAL);
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       38.0, -122.0, -INFINITY),         GUID_RC_REJ_BADVAL);
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       90.001, -122.0),                  GUID_RC_REJ_BADVAL);
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       -90.001, -122.0),                 GUID_RC_REJ_BADVAL);
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       38.0, 180.001),                   GUID_RC_REJ_BADVAL);
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       38.0, -180.001),                  GUID_RC_REJ_BADVAL);
    // Boundary coordinates are valid.
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       90.0, 180.0),                     GUID_RC_ACCEPTED);
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       -90.0, -180.0),                   GUID_RC_ACCEPTED);
}

TEST(GuidancePointGate, RejectsPnLaw) {
    // Accepting a point PN silently ignores would recreate the #376 lie.
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_PN), GUID_RC_REJ_LAW);
}

TEST(GuidancePointGate, RejectsNoReference) {
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP,
                       /*have_ref=*/false),        GUID_RC_REJ_NOREF);
}

TEST(GuidancePointGate, RadiusBoundary_RejectNeverClamp) {
    // <= max accepts: the 100.0 m boundary point is valid.
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       38.0, -122.0, 250.0f, 100.0f),  GUID_RC_ACCEPTED);
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       38.0, -122.0, 250.0f, 100.01f), GUID_RC_REJ_RADIUS);
    // A non-finite radius (garbage conversion) must reject, not accept.
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       38.0, -122.0, 250.0f, NAN),     GUID_RC_REJ_RADIUS);
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       38.0, -122.0, 250.0f, INFINITY), GUID_RC_REJ_RADIUS);
}

TEST(GuidancePointGate, GateOrder_StateBeatsBadvalBeatsLawBeatsNorefBeatsRadius) {
    // Everything wrong at once: STATE wins.
    EXPECT_EQ(gateWith(false, true, GUIDE_LAW_PN, false,
                       NAN, 999.0, NAN, 1e9f),            GUID_RC_REJ_STATE);
    // State ok, rest wrong: BADVAL wins.
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_PN, false,
                       NAN, 999.0, NAN, 1e9f),            GUID_RC_REJ_BADVAL);
    // Values ok too: LAW wins over NOREF and RADIUS.
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_PN, false,
                       38.0, -122.0, 250.0f, 1e9f),       GUID_RC_REJ_LAW);
    // Law ok: NOREF wins over RADIUS (r is meaningless without a reference).
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, false,
                       38.0, -122.0, 250.0f, 1e9f),       GUID_RC_REJ_NOREF);
    // Only the radius left: RADIUS.
    EXPECT_EQ(gateWith(true, false, GUIDE_LAW_STATION_KEEP, true,
                       38.0, -122.0, 250.0f, 1e9f),       GUID_RC_REJ_RADIUS);
}

// ============================================================================
// guidanceLlaToEnu (GuidancePointGate.h) — the ONLY conversion on the cmd-28
// flight path (receipt + launch re-convert).  An E/N transpose is exactly
// radius-invariant (r is symmetric in e/n) so no runtime gate can see it, and
// a degrees-as-radians slip only fails at runtime via the 100 m gate — both
// mutations must die HERE, pinned against hand-computed flat-earth geodesy.
namespace {
constexpr double kRefLatDeg = 38.0;
constexpr double kRefLonDeg = -122.0;
constexpr double kDeg2Rad   = 0.017453292519943295;
constexpr double kRefLatRad = kRefLatDeg * kDeg2Rad;
constexpr double kRefLonRad = kRefLonDeg * kDeg2Rad;
// 1e-4 deg of latitude on the R=6378137 sphere:
//   N = 1e-4 * pi/180 * 6378137 = 11.131949 m
// and the same arc of longitude scales by cos(38 deg) = 0.788011:
//   E = 11.131949 * cos(38 deg) = 8.772097 m
constexpr double kArcPerLatTick = 11.131949;   // m per 1e-4 deg, ref_alt = 0
constexpr double kArcPerLonTick = 8.772097;    // m per 1e-4 deg at 38N
}  // namespace

TEST(GuidanceLlaToEnu, NorthOffsetMapsToNOnly) {
    float e = -1.0f, n = -1.0f;
    guidanceLlaToEnu(kRefLatDeg + 1e-4, kRefLonDeg,
                     kRefLatRad, kRefLonRad, 0.0, e, n);
    EXPECT_NEAR(n, kArcPerLatTick, 2e-3);
    EXPECT_NEAR(e, 0.0, 1e-6);
}

TEST(GuidanceLlaToEnu, EastOffsetMapsToEOnly_CosLatScaled) {
    float e = -1.0f, n = -1.0f;
    guidanceLlaToEnu(kRefLatDeg, kRefLonDeg + 1e-4,
                     kRefLatRad, kRefLonRad, 0.0, e, n);
    EXPECT_NEAR(e, kArcPerLonTick, 2e-3);   // cos(lat)-scaled — kills E/N swap
    EXPECT_NEAR(n, 0.0, 1e-6);
}

TEST(GuidanceLlaToEnu, SignsFollowSouthAndWest) {
    float e = 0.0f, n = 0.0f;
    guidanceLlaToEnu(kRefLatDeg - 1e-4, kRefLonDeg - 1e-4,
                     kRefLatRad, kRefLonRad, 0.0, e, n);
    EXPECT_NEAR(n, -kArcPerLatTick, 2e-3);
    EXPECT_NEAR(e, -kArcPerLonTick, 2e-3);
}

TEST(GuidanceLlaToEnu, RefPointIsOrigin) {
    float e = 99.0f, n = 99.0f;
    guidanceLlaToEnu(kRefLatDeg, kRefLonDeg, kRefLatRad, kRefLonRad, 250.0, e, n);
    EXPECT_EQ(e, 0.0f);
    EXPECT_EQ(n, 0.0f);
}

TEST(GuidanceLlaToEnu, RefAltitudeGrowsTheSphere) {
    // The EKF formula scales by (R_EARTH + ref_alt): 1000 m of pad altitude
    // stretches the arc by 1 + 1000/6378137 = 1.5678e-4.
    float e = 0.0f, n = 0.0f;
    guidanceLlaToEnu(kRefLatDeg + 1e-4, kRefLonDeg,
                     kRefLatRad, kRefLonRad, 1000.0, e, n);
    EXPECT_NEAR(n, kArcPerLatTick * (1.0 + 1000.0 / 6378137.0), 2e-3);
}

TEST(GuidanceLlaToEnu, RadiansContract_DegreesInputWouldExplode) {
    // Passing DEGREES where the formula multiplies by the earth radius gives
    // km-scale garbage; the pinned metre-scale values above already kill that
    // mutant, but pin the magnitude bound explicitly too.
    float e = 0.0f, n = 0.0f;
    guidanceLlaToEnu(kRefLatDeg + 1e-4, kRefLonDeg + 1e-4,
                     kRefLatRad, kRefLonRad, 0.0, e, n);
    EXPECT_LT(std::hypot(e, n), 20.0f);
}

// ============================================================================
// guidanceLaunchReconvert (GuidancePointGate.h) — the launch-time keep-vs-wipe
// policy.  A comparison flip here silently flies overhead on every Drift-Cast
// flight while the pre-flight echo stays green, so keep/wipe/boundary are
// host-pinned like the receipt gate.
TEST(GuidanceLaunchReconvert, KeepsInRangePoint) {
    const auto lt = guidanceLaunchReconvert(kRefLatDeg + 1e-4, kRefLonDeg,
                                            kRefLatRad, kRefLonRad, 0.0, 100.0f);
    EXPECT_TRUE(lt.point_kept);
    EXPECT_NEAR(lt.n_m, kArcPerLatTick, 2e-3);
    EXPECT_NEAR(lt.e_m, 0.0, 1e-6);
}

TEST(GuidanceLaunchReconvert, WipesOutOfRangePointToOverhead) {
    // ~1113 m north — e.g. a point accepted against a REAL pad reference,
    // re-converted after a sim reset rebuilt a synthetic one.
    const auto lt = guidanceLaunchReconvert(kRefLatDeg + 1e-2, kRefLonDeg,
                                            kRefLatRad, kRefLonRad, 0.0, 100.0f);
    EXPECT_FALSE(lt.point_kept);
    EXPECT_EQ(lt.e_m, 0.0f);
    EXPECT_EQ(lt.n_m, 0.0f);
}

TEST(GuidanceLaunchReconvert, BoundaryRadiusKeeps_RejectNeverClamp) {
    // <= accepts: compute the exact converted radius, then use it as max_r.
    float e = 0.0f, n = 0.0f;
    guidanceLlaToEnu(kRefLatDeg + 1e-4, kRefLonDeg + 1e-4,
                     kRefLatRad, kRefLonRad, 0.0, e, n);
    const float r = std::sqrt(e * e + n * n);
    EXPECT_TRUE(guidanceLaunchReconvert(kRefLatDeg + 1e-4, kRefLonDeg + 1e-4,
                                        kRefLatRad, kRefLonRad, 0.0, r)
                    .point_kept);
    EXPECT_FALSE(guidanceLaunchReconvert(kRefLatDeg + 1e-4, kRefLonDeg + 1e-4,
                                         kRefLatRad, kRefLonRad, 0.0,
                                         r * 0.999f)
                     .point_kept);
}

// ============================================================================
// Guidance-target echo transitions (GuidancePointGate.h) — the cmd-28 handler
// wiring rules: honor the gate verdict (a reject NEVER applies the point),
// bump seq on rejects too, leave the still-active display unchanged on
// reject, and supersede-bumps only on real content changes.
namespace {
OutStatusQueryData echoWithAcceptedPoint() {
    OutStatusQueryData q{};
    EXPECT_TRUE(guidancePointEchoApply(q, GUID_RC_ACCEPTED, 38.1, -122.2, 249.6f));
    return q;
}
}  // namespace

TEST(GuidTargetEchoApply, AcceptUpdatesDisplayAndSeqAndRc) {
    const OutStatusQueryData q = echoWithAcceptedPoint();
    EXPECT_EQ(q.tgt_status, GUID_TGT_GEO_ACTIVE);
    EXPECT_FLOAT_EQ(q.tgt_lat_deg, 38.1f);
    EXPECT_FLOAT_EQ(q.tgt_lon_deg, -122.2f);
    EXPECT_EQ(q.tgt_alt_m, 250);   // rounded, not truncated
    EXPECT_EQ(q.tgt_seq, 1);
    EXPECT_EQ(q.tgt_last_rc, GUID_RC_ACCEPTED);
}

TEST(GuidTargetEchoApply, RejectBumpsSeqButNeverAppliesOrTouchesDisplay) {
    for (uint8_t rc : {GUID_RC_REJ_RADIUS, GUID_RC_REJ_STATE, GUID_RC_REJ_LAW,
                       GUID_RC_REJ_NOREF, GUID_RC_REJ_BADVAL}) {
        OutStatusQueryData q = echoWithAcceptedPoint();
        // Returns false -> the caller must NOT move the flight target (the
        // inverted-#376 lie: app shows "rejected" while the target moved).
        EXPECT_FALSE(guidancePointEchoApply(q, rc, 40.0, -100.0, 10.0f))
            << "rc=" << (int)rc;
        // Display = the still-active PREVIOUS point, seq/rc = the verdict.
        EXPECT_EQ(q.tgt_status, GUID_TGT_GEO_ACTIVE);
        EXPECT_FLOAT_EQ(q.tgt_lat_deg, 38.1f);
        EXPECT_FLOAT_EQ(q.tgt_lon_deg, -122.2f);
        EXPECT_EQ(q.tgt_alt_m, 250);
        EXPECT_EQ(q.tgt_seq, 2);
        EXPECT_EQ(q.tgt_last_rc, rc);
    }
}

TEST(GuidTargetEchoSupersede, WipesDisplayBumpsSeqKeepsRc) {
    OutStatusQueryData q = echoWithAcceptedPoint();
    EXPECT_TRUE(guidanceTargetEchoSupersede(q, GUID_TGT_EN_ACTIVE));
    EXPECT_EQ(q.tgt_status, GUID_TGT_EN_ACTIVE);
    EXPECT_EQ(q.tgt_lat_deg, 0.0f);
    EXPECT_EQ(q.tgt_lon_deg, 0.0f);
    EXPECT_EQ(q.tgt_alt_m, 0);
    EXPECT_EQ(q.tgt_seq, 2);
    // rc is "result of the LAST processed cmd 28" — a supersede isn't one.
    EXPECT_EQ(q.tgt_last_rc, GUID_RC_ACCEPTED);
}

TEST(GuidTargetEchoSupersede, NoOpDoesNotChurnSeq) {
    // Connect-time profile push with no cmd-28 point in play: seq must NOT
    // move, or every reconnect would shift the app's send baseline.
    OutStatusQueryData q{};
    q.tgt_status = GUID_TGT_NONE;
    EXPECT_FALSE(guidanceTargetEchoSupersede(q, GUID_TGT_NONE));
    EXPECT_EQ(q.tgt_seq, 0);
    // Same status twice in a row after a real wipe: second one is a no-op.
    OutStatusQueryData g = echoWithAcceptedPoint();
    EXPECT_TRUE(guidanceTargetEchoSupersede(g, GUID_TGT_NONE));
    EXPECT_FALSE(guidanceTargetEchoSupersede(g, GUID_TGT_NONE));
    EXPECT_EQ(g.tgt_seq, 2);
}

TEST(GuidTargetEchoSupersede, StatusChangeAloneBumps) {
    // Sim-reset / cmd-65 path where only the status differs (display already
    // clear): the app still needs to SEE the transition.
    OutStatusQueryData q{};
    q.tgt_status = GUID_TGT_EN_ACTIVE;
    EXPECT_TRUE(guidanceTargetEchoSupersede(q, GUID_TGT_NONE));
    EXPECT_EQ(q.tgt_seq, 1);
}

TEST(GuidFlownTargetSrc, GeoWinsThenModeDecides) {
    EXPECT_EQ(guidanceFlownTargetSrc(true,  GUIDE_TARGET_OVERHEAD), GUID_TGT_GEO_ACTIVE);
    EXPECT_EQ(guidanceFlownTargetSrc(true,  GUIDE_TARGET_POINT),    GUID_TGT_GEO_ACTIVE);
    EXPECT_EQ(guidanceFlownTargetSrc(false, GUIDE_TARGET_POINT),    GUID_TGT_EN_ACTIVE);
    EXPECT_EQ(guidanceFlownTargetSrc(false, GUIDE_TARGET_OVERHEAD), GUID_TGT_NONE);
}

// ============================================================================
// Wire-code uniqueness guard: OC<->FC I2C message types (#132 / #148)
// ============================================================================
// The "### Message Types from In ESP32 ###" block in RocketComputerTypes.h is
// a hand-assigned 8-bit code space (~0xA0-0xF1).  Both the OC and the FC
// dispatch on these with flat if/else-if chains that take the FIRST matching
// branch, so two constants sharing a value silently turn the later handler
// into dead code -- no compiler error, no link error, no test failure.
//
// That is exactly how #132 (rocket-profile sensor-cal sync) and #148 (mag-cal
// user-verify) collided: SENSOR_CAL_APPLY_PENDING/MSG/READ were assigned
// 0xDD/0xDE/0xDF, aliasing MAG_CAL_VERIFY_DONE/RESET/FORCE_APPLY, and a
// connect-time SENSOR_CAL_READ was silently handled as MAG_CAL_FORCE_APPLY
// ("force_apply refused: not in MAG_CALIBRATION session" -- bench 2026-05-29).
//
// This test makes that class of bug a hard failure: every message-type
// constant is listed once below and the values must be pairwise distinct.
// The list IS the registry -- when you add a message type to that header
// block, add it here too (and bump the count tripwire at the end).
//
// NOTE on scope: the REG_* read addresses (0x00-0x08) and the LoRa-namespace
// constants (LORA_BEACON_SYNC=0xBE, LORA_CMD_*, ...) are SEPARATE code spaces
// and are intentionally NOT included -- e.g. LORA_BEACON_SYNC=0xBE deliberately
// coincides with SERVO_TEST_MSG=0xBE and that overlap is harmless.  Only the
// OC<->FC message-type block must be internally unique.
TEST(RocketComputerTypes, MessageTypeCodes_AllUnique) {
    struct MsgType { uint8_t value; const char* name; };
#define MT(c) MsgType{ (c), #c }
    const MsgType codes[] = {
        MT(OUT_STATUS_QUERY),         MT(GNSS_MSG),
        MT(ISM6HG256_MSG),            MT(BMP585_MSG),
        MT(MMC5983MA_MSG),            MT(NON_SENSOR_MSG),
        MT(POWER_MSG),                MT(START_LOGGING),
        MT(END_FLIGHT),               MT(OUT_STATUS_RESPONSE),
        MT(CAMERA_START),             MT(CAMERA_STOP),
        MT(SOUNDS_ENABLE),            MT(SOUNDS_DISABLE),
        MT(SERVO_CONFIG_PENDING),     MT(PID_CONFIG_PENDING),
        MT(SERVO_CONFIG_MSG),         MT(PID_CONFIG_MSG),
        MT(SERVO_CTRL_ENABLE),        MT(SERVO_CTRL_DISABLE),
        MT(SIM_CONFIG_PENDING),       MT(SIM_CONFIG_MSG),
        MT(SIM_START_CMD),            MT(SIM_STOP_CMD),
        MT(GROUND_TEST_START),        MT(GROUND_TEST_STOP),
        MT(GYRO_CAL_CMD),             MT(GAIN_SCHED_ENABLE),
        MT(GAIN_SCHED_DISABLE),       MT(SERVO_TEST_PENDING),
        MT(SERVO_TEST_MSG),           MT(SERVO_TEST_STOP),
        MT(ROLL_PROFILE_PENDING),     MT(ROLL_PROFILE_MSG),
        MT(ROLL_PROFILE_CLEAR),       MT(SERVO_REPLAY_PENDING),
        MT(SERVO_REPLAY_MSG),         MT(SERVO_REPLAY_STOP),
        MT(ROLL_CTRL_CONFIG_PENDING), MT(ROLL_CTRL_CONFIG_MSG),
        MT(GUIDANCE_ENABLE),          MT(GUIDANCE_DISABLE),
        MT(GUIDANCE_TELEM_MSG),       MT(CAMERA_CONFIG_PENDING),
        MT(CAMERA_CONFIG_MSG),        MT(PYRO_CONFIG_PENDING),
        MT(PYRO_CONFIG_MSG),          MT(PYRO_CONT_TEST),
        MT(PYRO_FIRE_TEST),           MT(IIS2MDC_MSG),
        MT(SNAPSHOT_MSG),             MT(GET_FLIGHT_SNAPSHOT),
        MT(MAG_CAL_START),            MT(MAG_CAL_ABORT),
        MT(MAG_CAL_ACCEPT),           MT(MAG_CAL_RETRY),
        MT(MAG_CAL_STATUS_MSG),       MT(MAG_CAL_COMPUTE_FIT),
        MT(MAG_CAL_VERIFY_DONE),      MT(MAG_CAL_VERIFY_RESET),
        MT(MAG_CAL_FORCE_APPLY),      MT(MAG_CAL_APPLY_PENDING),
        MT(MAG_CAL_APPLY_MSG),        MT(MAG_CAL_READ),
        MT(SENSOR_CAL_APPLY_PENDING), MT(SENSOR_CAL_APPLY_MSG),
        MT(SENSOR_CAL_READ),          MT(SENSOR_CAL_STATUS_MSG),
        MT(FLIGHT_SETTINGS_MSG),      MT(LOG_BUFFER_STATS_MSG),
        // OTA relay control block (#8 Phase 4): OC->FC control over I2C
        // (0xE3-0xE6), FC->OC status over I2S (OTA_STATUS_MSG=0xE7).  The
        // sensor-cal block was deliberately moved to 0xE8+ to keep 0xE3-0xE7
        // free for these (see RocketComputerTypes.h).
        MT(OTA_BEGIN_PENDING),        MT(OTA_BEGIN_MSG),
        MT(OTA_FINISH_CMD),           MT(OTA_ABORT_CMD),
        MT(OTA_STATUS_MSG),           MT(OTA_DATA_CHUNK),
        // FC->OC firmware-version push, relayed to the app for OTA verify (#8 P4).
        MT(FC_IDENTITY),
        // Board->rocket mounting orientation setting (app->OC->FC).
        MT(ORIENT_CONFIG_PENDING),    MT(ORIENT_CONFIG_MSG),
        MT(LORA_MSG),
        // Guidance + fin-layout config (were missing from this registry —
        // the pre-flight review's #386 gap; a colliding new code would have
        // passed the guard silently).
        MT(GUIDANCE_CONFIG_PENDING),  MT(GUIDANCE_CONFIG_MSG),
        MT(FIN_CONFIG_PENDING),       MT(FIN_CONFIG_MSG),
        // IMU logging rate config (BLE cmd 67).
        MT(IMU_RATE_CONFIG_PENDING),  MT(IMU_RATE_CONFIG_MSG),
        // #435 Drift-Cast guidance point (BLE cmd 28).
        MT(GUIDANCE_POINT_PENDING),   MT(GUIDANCE_POINT_MSG),
        // #402: FC->OC slave TX-ring desync recovery trigger.
        MT(I2C_TX_RESYNC),
        // OC-self-emitted uplink RX record — the rocket's only RF measurement.
        MT(LORA_UPLINK_MSG),
        // FC->OC boot progress during setup_fc, before the I2S stream exists.
        MT(FC_BOOT_STATUS_MSG),
        // FC->OC full config report (#915) — what the app readback can't see.
        MT(CONFIG_REPORT_MSG),
    };
#undef MT

    std::map<uint8_t, const char*> seen;
    for (const auto& c : codes) {
        auto result = seen.emplace(c.value, c.name);
        const bool inserted = result.second;
        const char* first   = result.first->second;
        EXPECT_TRUE(inserted)
            << "Duplicate OC<->FC message-type code 0x" << std::hex
            << std::uppercase << (int)c.value << std::dec << ": " << c.name
            << " collides with " << first << ". Flat if/else-if dispatch on "
               "both ends takes the first match, so one of these handlers is "
               "silently dead. Reassign one to a free value.";
    }

    // Tripwire: keep the registry above exhaustive.  If you add or remove a
    // message type in RocketComputerTypes.h, update this list AND this count
    // -- the uniqueness check is only as strong as the list it walks.
    // 87 = 80 prior + guidance/fin config pair codes (were missing, #386 gap)
    //    + I2C_TX_RESYNC (#402) + IMU rate config pair (BLE cmd 67).
    // 89 = 87 + Drift-Cast guidance point pair (#435, BLE cmd 28).
    // 90 = 89 + LORA_UPLINK_MSG (OC-self-emitted uplink RSSI/SNR record).
    // 91 = 90 + FC_BOOT_STATUS_MSG (FC->OC boot progress during setup_fc).
    // 92 = 91 + CONFIG_REPORT_MSG (#915 full config report).  This leaves
    //      exactly TWO free codes in the space (0xFC, 0xFD) — the next
    //      message after those needs an escape/extended encoding, not a
    //      thirteenth constant.
    EXPECT_EQ(sizeof(codes) / sizeof(codes[0]), 92u)
        << "Message-type count changed: update the registry in this test to "
           "match the '### Message Types from In ESP32 ###' header block.";
}

// ============================================================================
// Issue #150: adaptive FCC hop dwell — airtime-derived, compliance-bounded
// ============================================================================
// FCC 15.247 FHSS binds accumulated TRANSMIT AIRTIME on any single
// frequency: < 0.4 s per 10 s window at channel BW >= 250 kHz, per 20 s
// window below.  The schedule satisfies it per-visit (`dwell` packets on
// one channel) as long as one full cycle outlasts the window, so a channel
// is never revisited inside it.  Both halves are pinned here against the
// shared airtime formula that the OC scheduler and BS follower use.

namespace fhss150 {
// Production telemetry frame: sizeof(LoRaData), pinned at 65 by the
// header's static_asserts (#191 repack: 65 B is airtime-identical to the
// 66 B frame these numbers were derived at — LoRa symbol-block
// quantization).  Growing the frame has a real regulatory cost — at 73 B
// the SF10/BW250 long-range rung crosses the FCC 400 ms occupancy line —
// so the table below fails deliberately if it changes.
// #850: budget against the LARGER of the two frames. Sizing the schedule
// on the slow one would under-count every fast frame and walk the link
// straight into the occupancy limit.
constexpr size_t  kTelemFrameLen = SIZE_OF_LORA_BUDGET;
constexpr uint8_t kCr            = LORA_FACTORY_RENDEZVOUS_CR;  // 4/5 on both ends
// = out_computer config.h LORA_TX_RATE_HZ.  Telemetry is an unconditional
// 2 Hz in every transmitting state.  If the rate ever rises, re-derive the
// revisit bound below — BW125 hopping stops being legal near 4 Hz.
constexpr float   kTxRateHz      = 2.0f;
// Current name-beacon frame (sync + id + name).  Update if it grows.
constexpr size_t  kBeaconLen     = 23;

inline uint32_t telemToaMs(uint8_t sf, float bw_khz) {
    return loraTimeOnAirMs(kTelemFrameLen, sf, bw_khz, kCr,
                           LORA_TELEM_PREAMBLE_SYMS);
}
inline uint32_t fccWindowSec(float bw_khz) {
    return (bw_khz >= 250.0f) ? 10u : 20u;
}
}  // namespace fhss150

TEST(LoraTimeOnAir, PinsTheNumbersTheDwellTableRestsOn) {
    using namespace fhss150;
    // Semtech AN1200.13 at the production FAST frame (55 B since #850 split
    // the downlink; was 65 B, and the pinned numbers below dropped with it).
    EXPECT_NEAR(telemToaMs(8,  250.0f), 102.0, 2.0);   // was 112
    EXPECT_NEAR(telemToaMs(9,  250.0f), 183.0, 2.0);   // was 203
    // The long-range rung. This is the number the whole frame budget exists to
    // protect: at 65 B it was 386 ms against a 390 ms dwell budget — 4 ms of
    // margin, which is why the #150 doc says frame growth "breaks this first".
    // The two-frame split bought back 41 ms.
    EXPECT_NEAR(telemToaMs(10, 250.0f), 345.0, 3.0);   // was 386
    // SF11 @ BW250 still cannot fit even one packet in the budget.
    EXPECT_GT(telemToaMs(11, 250.0f), LORA_HOP_DWELL_BUDGET_MS);
}

TEST(LoraHopDwell, AdaptiveTable) {
    using namespace fhss150;
    // The ladder the iOS picker exposes (0 = hopping unavailable, GUI
    // greys the option).  Derived, not designed — if any row changes,
    // either the frame grew or someone touched the budget: both are
    // regulatory decisions that must be made consciously.
    struct Row { uint8_t sf; float bw; uint8_t dwell; };
    const Row rows[] = {
        // #850 moved four of these rows, all in the permissive direction,
        // because the FAST frame is 55 B where the single frame was 65:
        //   SF9/BW250  1 -> 2      SF9/BW500  3 -> 4
        //   SF9/BW125  0 -> 1      (hopping newly PERMITTED at that rung)
        // SF10/BW250 keeps dwell 1 but its per-visit occupancy falls 386 ->
        // 345 ms, which is the margin the split was really for.
        {7,  250.0f, 4}, {8,  250.0f, 3}, {9,  250.0f, 2},
        {10, 250.0f, 1},                       // the +5 dB long-range rung
        {11, 250.0f, 0}, {12, 250.0f, 0},
        {7,  125.0f, 3}, {8,  125.0f, 1}, {9,  125.0f, 1}, {10, 125.0f, 0},
        {7,  500.0f, 4}, {8,  500.0f, 4}, {9,  500.0f, 4},
        {10, 500.0f, 2}, {11, 500.0f, 1}, {12, 500.0f, 0},
    };
    for (const auto& r : rows) {
        EXPECT_EQ(loraHopDwellForConfig(r.sf, r.bw, kTelemFrameLen, kCr),
                  r.dwell)
            << "SF" << (int)r.sf << "/BW" << r.bw
            << " toa=" << telemToaMs(r.sf, r.bw) << " ms";
    }
}

TEST(LoraHopDwell, ComplianceInvariant) {
    using namespace fhss150;
    // The binding check: for EVERY modulation the firmware can be
    // configured into, either hopping is refused (dwell 0) or a full
    // dwell visit stays under the FCC 400 ms occupancy line.
    const float bws[] = {125.0f, 250.0f, 500.0f};
    for (uint8_t sf = 6; sf <= 12; sf++) {
        for (float bw : bws) {
            const uint8_t d = loraHopDwellForConfig(sf, bw, kTelemFrameLen, kCr);
            if (d == 0) continue;
            EXPECT_LE(d * telemToaMs(sf, bw), 400u)
                << "SF" << (int)sf << "/BW" << bw << " dwell=" << (int)d;
        }
    }
}

TEST(LoraHopDwell, FullCycleRevisitOutlastsTheFccWindow) {
    using namespace fhss150;
    // Per-visit occupancy is only sufficient if a channel is never
    // revisited inside the window.  Worst case is the FCC floor channel
    // count (skip-mask at its most aggressive) at the production TX rate.
    const float bws[] = {125.0f, 250.0f, 500.0f};
    for (uint8_t sf = 6; sf <= 12; sf++) {
        for (float bw : bws) {
            const uint8_t d = loraHopDwellForConfig(sf, bw, kTelemFrameLen, kCr);
            if (d == 0) continue;
            const float cycle_s =
                (float)loraFhssMinChannels(bw) * (float)d / kTxRateHz;
            EXPECT_GE(cycle_s, (float)fccWindowSec(bw))
                << "SF" << (int)sf << "/BW" << bw << " dwell=" << (int)d;
        }
    }
}

TEST(LoraHopDwell, BeaconSuppressionDuringHopIsLoadBearing) {
    using namespace fhss150;
    // Name beacons transmit every 2 s in non-INFLIGHT states.  If one
    // rode a hop-dwell channel visit at the long-range rung, the visit
    // would blow the occupancy line — which is exactly why the OC
    // suppresses beacons while hop_active_ (they'd ride hop channels the
    // BS already follows, so they add nothing either).
    const uint8_t d10 = loraHopDwellForConfig(10, 250.0f, kTelemFrameLen, kCr);
    ASSERT_GT(d10, 0);
    const uint32_t beacon_toa =
        loraTimeOnAirMs(kBeaconLen, 10, 250.0f, kCr, LORA_TELEM_PREAMBLE_SYMS);
    EXPECT_GT(d10 * telemToaMs(10, 250.0f) + beacon_toa, 400u)
        << "if this passes, beacon suppression stopped being load-bearing";
    // Even at SF8 (dwell 3) a beacon would leave ~zero margin: 336+61=397.
    const uint32_t beacon8 =
        loraTimeOnAirMs(kBeaconLen, 8, 250.0f, kCr, LORA_TELEM_PREAMBLE_SYMS);
    EXPECT_LE(3 * telemToaMs(8, 250.0f) + beacon8, 400u);
}

TEST(LoraHopChannelForSeq, ExactUniformityOverFullCycles) {
    // FCC: "each frequency used equally on the average".  The
    // deterministic round-robin is EXACTLY uniform over whole cycles —
    // every active channel gets k*dwell packets, masked channels get 0 —
    // from ANY starting seq.  start=0 doubles as the rocket-reboot
    // (seq reset) case: a reboot restarts the schedule but cannot bias
    // long-run channel use.
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    constexpr uint8_t n = 69;             // BW250 table
    for (uint8_t i = 0; i < 19; i++) {    // mask 19 → n_active = 50 (floor)
        loraSkipMaskSet(mask, (uint8_t)(i * 3 + 1));
    }
    constexpr uint8_t  dwell    = 3;      // = loraHopDwellForConfig @ SF8/BW250
    constexpr uint16_t n_active = 50;
    constexpr uint16_t cycle    = dwell * n_active;
    constexpr uint16_t k        = 3;
    const uint16_t starts[] = {0, 12345};
    for (uint16_t start : starts) {
        uint32_t counts[256] = {0};
        for (uint32_t s = start; s < (uint32_t)start + k * cycle; s++) {
            counts[loraHopChannelForSeq((uint16_t)s, dwell, mask, n)]++;
        }
        for (uint8_t c = 0; c < n; c++) {
            if (loraSkipMaskTest(mask, c)) {
                EXPECT_EQ(counts[c], 0u) << "masked ch " << (int)c
                                         << " start=" << start;
            } else {
                EXPECT_EQ(counts[c], (uint32_t)(k * dwell))
                    << "active ch " << (int)c << " start=" << start;
            }
        }
    }
}

TEST(LoraHopChannelForSeq, U16WrapKeepsLongRunUseNearUniform) {
    // 65536 is not a whole number of cycles, so each u16 epoch hands at
    // most one extra dwell-window to some channels.  Bound the spread —
    // this is the long-run "equal use on average" property across seq
    // wraps.
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    constexpr uint8_t n     = 69;
    constexpr uint8_t dwell = 3;
    uint32_t counts[256] = {0};
    for (uint32_t s = 0; s < 65536; s++) {
        counts[loraHopChannelForSeq((uint16_t)s, dwell, mask, n)]++;
    }
    uint32_t mn = UINT32_MAX, mx = 0;
    for (uint8_t c = 0; c < n; c++) {
        if (counts[c] < mn) mn = counts[c];
        if (counts[c] > mx) mx = counts[c];
    }
    EXPECT_LE(mx - mn, (uint32_t)dwell);
}

TEST(LoraHopSchedule, ManualScanHandoffKeepsFloorAndCoverage) {
    using namespace fhss150;
    // The pure-function path a user's manual Frequency Scan takes while
    // hopping (#150, settled decision 3): hostile scan → channel-set
    // selection enforces the FCC floor → the seq schedule then cycles
    // EXACTLY the surviving active set.
    const uint8_t n = loraChannelCount(250.0f);
    ASSERT_GE(n, 50u);
    float  freqs[139];
    int8_t rssi[139];
    for (uint8_t i = 0; i < n; i++) {
        freqs[i] = loraChannelMHz(250.0f, i);
        rssi[i]  = (i % 5 == 0) ? -100 : -50;   // most channels loud
    }
    LoRaChannelSetSelection out{};
    loraSelectChannelSet(freqs, rssi, n, 250.0f, &out);

    uint16_t n_active = 0;
    for (uint8_t i = 0; i < out.n_channels; i++) {
        if (!loraSkipMaskTest(out.skip_mask, i)) n_active++;
    }
    ASSERT_GE(n_active, loraFhssMinChannels(250.0f));

    const uint8_t dwell = loraHopDwellForConfig(8, 250.0f, kTelemFrameLen, kCr);
    ASSERT_GT(dwell, 0);
    uint32_t counts[256] = {0};
    const uint32_t cycle = (uint32_t)dwell * n_active;
    for (uint32_t s = 0; s < cycle; s++) {
        counts[loraHopChannelForSeq((uint16_t)s, dwell, out.skip_mask,
                                    out.n_channels)]++;
    }
    for (uint8_t c = 0; c < out.n_channels; c++) {
        if (loraSkipMaskTest(out.skip_mask, c)) {
            EXPECT_EQ(counts[c], 0u) << "masked ch " << (int)c;
        } else {
            EXPECT_EQ(counts[c], (uint32_t)dwell) << "active ch " << (int)c;
        }
    }
}

// ============================================================================
// Issue #150 review fixes: CR link gate, home-channel skip, 0xFE sentinel
// ============================================================================

TEST(LoraHopDwell, LinkGateRefusesNonFactoryCr) {
    using namespace fhss150;
    // CR-only cmd-10 changes are unverifiable over the air (explicit-header
    // RX decodes any payload CR, so the transaction's commit criterion is
    // CR-blind) — a one-sided CR commit would give the two ends different
    // dwells with no heal short of reboot.  The link gate refuses hopping
    // at any CR but the factory one, so no packet-loss pattern can split
    // the schedule.
    EXPECT_EQ(loraHopDwellForLink(8, 250.0f, kTelemFrameLen,
                                  LORA_FACTORY_RENDEZVOUS_CR),
              loraHopDwellForConfig(8, 250.0f, kTelemFrameLen,
                                    LORA_FACTORY_RENDEZVOUS_CR));
    for (uint8_t cr = 6; cr <= 8; cr++) {
        EXPECT_EQ(loraHopDwellForLink(8, 250.0f, kTelemFrameLen, cr), 0)
            << "cr=" << (int)cr;
    }
}

TEST(LoraHomeChannelSkip, OffGridFactoryRendezvousIsNoOp) {
    // 915.0 sits between BW250 grid channels (902.125 + 0.375k), so the
    // factory default never coincides and the skip is a no-op there.
    EXPECT_EQ(loraGridIndexOfFreq(250.0f, LORA_FACTORY_RENDEZVOUS_MHZ), -1);
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    EXPECT_FALSE(loraApplyHomeChannelSkip(mask, 250.0f,
                                          LORA_FACTORY_RENDEZVOUS_MHZ));
    for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) {
        EXPECT_EQ(mask[i], 0u);
    }
}

TEST(LoraHomeChannelSkip, OnGridHomeIsExcludedFromTheSchedule) {
    // A scan-applied operating frequency CAN land exactly on a grid
    // channel; hop-session transition packets transmit on it outside the
    // schedule, so the coincident slot is implicitly skipped on both ends
    // — otherwise a transition packet plus a scheduled dwell visit could
    // stack past the FCC 400 ms occupancy line in one window.
    const float home = loraChannelMHz(250.0f, 34);   // 914.875 MHz, on-grid
    EXPECT_EQ(loraGridIndexOfFreq(250.0f, home), 34);
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    ASSERT_TRUE(loraApplyHomeChannelSkip(mask, 250.0f, home));
    EXPECT_TRUE(loraSkipMaskTest(mask, 34));
    const uint8_t n = loraChannelCount(250.0f);
    for (uint16_t s = 0; s < (uint16_t)(4 * n); s++) {
        EXPECT_NE(loraHopChannelForSeq(s, 3, mask, n), 34) << "seq=" << s;
    }
}

TEST(LoraHomeChannelSkip, WithheldAtTheFccFloor) {
    // The implicit skip must never take the active count below the FCC
    // floor — in that corner the coincidence risk is accepted and the
    // floor wins.
    const uint8_t n     = loraChannelCount(250.0f);
    const uint8_t floor_ = loraFhssMinChannels(250.0f);
    ASSERT_GT(n, floor_);
    uint8_t mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
    uint8_t masked = 0;
    for (uint8_t i = 0; i < n && masked < (uint8_t)(n - floor_); i++) {
        if (i == 34) continue;              // keep the home channel active
        loraSkipMaskSet(mask, i);
        masked++;
    }
    // Exactly `floor_` channels remain active; the skip must be withheld.
    EXPECT_FALSE(loraApplyHomeChannelSkip(mask, 250.0f,
                                          loraChannelMHz(250.0f, 34)));
    EXPECT_FALSE(loraSkipMaskTest(mask, 34));
}

TEST(LoraHopSentinels, OffScheduleMarkerCannotCollideWithChannelIndices) {
    // 0xFE ("hopping, momentarily off-schedule") and 0xFF ("not hopping")
    // must stay clear of every real channel index at every bandwidth —
    // loraChannelCount caps at 253 to guarantee it.
    EXPECT_NE(LORA_NEXT_CH_HOP_OFFSCHEDULE, LORA_NEXT_CH_NO_HOP);
    EXPECT_LE(loraChannelCount(125.0f), 253);
    EXPECT_LE(loraChannelCount(250.0f), 253);
    EXPECT_LE(loraChannelCount(500.0f), 253);
    EXPECT_GT((int)LORA_NEXT_CH_HOP_OFFSCHEDULE,
              (int)loraChannelCount(125.0f));
}
