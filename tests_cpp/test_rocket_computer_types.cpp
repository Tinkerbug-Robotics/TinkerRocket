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
    EXPECT_EQ(sizeof(LoRaData),       59u);  // 2-byte routing header + 57-byte payload
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
    uint8_t all = PSF_CH1_CONT | PSF_CH2_CONT | PSF_CH1_FIRED | PSF_CH2_FIRED |
                  PSF_REBOOT_RECOVERY | PSF_GUIDANCE_ENABLED;
    // Bits 0-5 used, no overlap
    EXPECT_EQ(all, 0x3F);
    // Each is a single bit
    EXPECT_EQ(__builtin_popcount(PSF_CH1_CONT),         1);
    EXPECT_EQ(__builtin_popcount(PSF_CH2_CONT),         1);
    EXPECT_EQ(__builtin_popcount(PSF_CH1_FIRED),        1);
    EXPECT_EQ(__builtin_popcount(PSF_CH2_FIRED),        1);
    EXPECT_EQ(__builtin_popcount(PSF_REBOOT_RECOVERY),  1);
    EXPECT_EQ(__builtin_popcount(PSF_GUIDANCE_ENABLED), 1);
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
