#include <gtest/gtest.h>
#include <cstring>
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
    EXPECT_EQ(sizeof(LoRaData),       62u);  // 5-byte routing header (incl. 16-bit seq) + 57-byte payload
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
// Issues #40 / #41: per-packet channel hopping — state gate
// ============================================================================

TEST(ShouldHopInState, OnlyPrelaunchAndInflight) {
    // Hopping is intentionally gated to the two states where the rocket
    // is committed to its modulation config and isn't in a recovery /
    // pre-handshake situation.  Ground states (READY/LANDED/INIT) stay
    // on the static channel so config and recovery work as before.
    EXPECT_FALSE(shouldHopInState(INITIALIZATION));
    EXPECT_FALSE(shouldHopInState(READY));
    EXPECT_TRUE (shouldHopInState(PRELAUNCH));
    EXPECT_TRUE (shouldHopInState(INFLIGHT));
    EXPECT_FALSE(shouldHopInState(LANDED));
}

TEST(ShouldHopInState, Uint8OverloadMatchesEnum) {
    // The BS receives state numerically from the LoRa downlink, so the
    // uint8_t overload must agree bit-for-bit with the enum overload.
    for (uint8_t s = 0; s <= 4; s++)
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
    // Recent RX but in READY/INIT/LANDED — direct scan path is correct.
    EXPECT_FALSE(rocketLikelyHopping(false, 95000, 100000, READY,         10000));
    EXPECT_FALSE(rocketLikelyHopping(false, 95000, 100000, INITIALIZATION,10000));
    EXPECT_FALSE(rocketLikelyHopping(false, 95000, 100000, LANDED,        10000));
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
