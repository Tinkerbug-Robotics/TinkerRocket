#include <gtest/gtest.h>
#include "RocketComputerTypes.h"

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
    EXPECT_EQ(SIZE_OF_LORA_DATA,       sizeof(LoRaData));
}

TEST(RocketComputerTypes, KnownSizes) {
    // Hard-coded expected sizes from the wire protocol spec.
    // If any of these change, the iOS app and analysis scripts must update too.
    EXPECT_EQ(sizeof(GNSSData),       42u);
    EXPECT_EQ(sizeof(BMP585Data),     12u);
    EXPECT_EQ(sizeof(ISM6HG256Data),  22u);
    EXPECT_EQ(sizeof(MMC5983MAData),  16u);
    EXPECT_EQ(sizeof(POWERData),      10u);
    EXPECT_EQ(sizeof(NonSensorData),  43u);
    EXPECT_EQ(sizeof(LoRaData),       57u);
    EXPECT_EQ(sizeof(i24le_t),         3u);
    EXPECT_EQ(sizeof(Vec3i16),         6u);
}

TEST(RocketComputerTypes, NSF_FlagBits_NoOverlap) {
    // All NonSensorData flag bits must be distinct
    uint8_t all = NSF_ALT_LANDED | NSF_ALT_APOGEE | NSF_VEL_APOGEE |
                  NSF_LAUNCH | NSF_BURNOUT | NSF_GUIDANCE |
                  NSF_PYRO1_ARMED | NSF_PYRO2_ARMED;
    EXPECT_EQ(all, 0xFF); // all 8 bits used, none overlapping
}

TEST(RocketComputerTypes, PSF_FlagBits_NoOverlap) {
    uint8_t all = PSF_CH1_CONT | PSF_CH2_CONT | PSF_CH1_FIRED | PSF_CH2_FIRED;
    // Bits 0-3 used, no overlap
    EXPECT_EQ(all, 0x0F);
    // Each is a single bit
    EXPECT_EQ(__builtin_popcount(PSF_CH1_CONT),  1);
    EXPECT_EQ(__builtin_popcount(PSF_CH2_CONT),  1);
    EXPECT_EQ(__builtin_popcount(PSF_CH1_FIRED), 1);
    EXPECT_EQ(__builtin_popcount(PSF_CH2_FIRED), 1);
}

TEST(RocketComputerTypes, MaxPayload_CoversAllTypes) {
    // MAX_PAYLOAD must be >= every packed struct that can be a frame payload
    EXPECT_GE(MAX_PAYLOAD, sizeof(GNSSData));
    EXPECT_GE(MAX_PAYLOAD, sizeof(BMP585Data));
    EXPECT_GE(MAX_PAYLOAD, sizeof(ISM6HG256Data));
    EXPECT_GE(MAX_PAYLOAD, sizeof(MMC5983MAData));
    EXPECT_GE(MAX_PAYLOAD, sizeof(POWERData));
    EXPECT_GE(MAX_PAYLOAD, sizeof(NonSensorData));
    EXPECT_GE(MAX_PAYLOAD, sizeof(LoRaData));
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
