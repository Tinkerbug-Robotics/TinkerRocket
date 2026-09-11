// #1128: the rollback in TR_LoRa_Comms::reconfigure() restored the spreading
// factor before the bandwidth, so the old SF was validated against the NEW
// bandwidth and could be rejected — silently, and leaving the chip at a
// modulation neither the driver nor the ground station knew about.
//
// The order is now bandwidth-first. These pin the rule that makes that
// necessary, and the specific pair from the issue.

#include <gtest/gtest.h>

#include "lora_modulation_policy.h"

namespace P = LoraModulationPolicy;

TEST(LoraModulation, TheSpreadingFactorWindowFollowsTheBandwidth) {
    // Straight from LLCC68::setSpreadingFactor.
    EXPECT_EQ(P::maxSpreadingFactorFor(125.0f), 9);
    EXPECT_EQ(P::maxSpreadingFactorFor(250.0f), 10);
    EXPECT_EQ(P::maxSpreadingFactorFor(500.0f), 11);
}

TEST(LoraModulation, ABandwidthThePartDoesNotHaveMakesEverySfIllegal) {
    // LLCC68's switch has no default case that accepts anything — an
    // unsupported bandwidth rejects every SF, which is why a rollback that
    // reaches the chip in that state cannot restore anything.
    EXPECT_EQ(P::maxSpreadingFactorFor(62.5f), 0);
    EXPECT_EQ(P::maxSpreadingFactorFor(0.0f), 0);
    for (uint8_t sf = 5; sf <= 12; ++sf)
        EXPECT_FALSE(P::isLegalPair(62.5f, sf)) << "SF" << (int)sf;
}

TEST(LoraModulation, TheFleetDefaultIsALegalPairAndTheDocumentedTrapIsNot) {
    // TR_LoRa_Comms.h says SF8/BW250 is the operating point and calls out
    // SF10/BW125 as the illegal one. Both halves of that comment, checked.
    EXPECT_TRUE(P::isLegalPair(250.0f, 8));
    EXPECT_FALSE(P::isLegalPair(125.0f, 10));
}

TEST(LoraModulation, SfBelowFiveIsRejectedAtEveryBandwidth) {
    for (float bw : {125.0f, 250.0f, 500.0f}) {
        EXPECT_FALSE(P::isLegalPair(bw, 4)) << "BW" << bw;
        EXPECT_TRUE(P::isLegalPair(bw, 5)) << "BW" << bw;
    }
}

TEST(LoraModulation, TheIssuesOwnScenarioIsExactlyTheUnsafeRollback) {
    // #1128's worked example: on the pad at BW250/SF10, the operator sends
    // cmd 10 with bw=125, sf=9 and a typo'd 30 dBm. Bandwidth and SF both
    // take, setOutputPower is rejected, and the rollback runs with BW125 live.
    //
    // Restoring SF first there means SF10 at BW125 — rejected. The old code
    // discarded that and then restored BW250, leaving the chip at BW250/SF9
    // while every report said BW250/SF10, and the downlink was dead.
    EXPECT_FALSE(P::sfRestoreIsSafeBeforeBw(/*new_bw=*/125.0f, /*old_sf=*/10));

    // Bandwidth-first is safe for this pair, and the intermediate the chip
    // passes through is one it accepts: setBandwidth does not validate SF, and
    // the old pair is legal by construction because it was live on entry.
    EXPECT_TRUE(P::isLegalPair(250.0f, 10));
}

TEST(LoraModulation, BandwidthFirstIsSafeForEveryPairReverseOrderIsNot) {
    // The general statement, over every legal (old) -> (new) modulation change
    // the part allows. Bandwidth-first always lands on a legal pair; SF-first
    // does not, and the cases where it fails are the ones #1128 is about.
    const float bws[] = {125.0f, 250.0f, 500.0f};
    int sf_first_unsafe = 0, total = 0;
    for (float old_bw : bws) {
        for (uint8_t old_sf = 5; old_sf <= P::maxSpreadingFactorFor(old_bw); ++old_sf) {
            for (float new_bw : bws) {
                for (uint8_t new_sf = 5; new_sf <= P::maxSpreadingFactorFor(new_bw); ++new_sf) {
                    ++total;
                    // Bandwidth-first: the final state is the old pair, which
                    // was live on entry and so is legal. Always.
                    EXPECT_TRUE(P::isLegalPair(old_bw, old_sf));
                    if (!P::sfRestoreIsSafeBeforeBw(new_bw, old_sf)) ++sf_first_unsafe;
                }
            }
        }
    }
    EXPECT_GT(total, 0);
    // Not a corner case: a real share of the transition space is affected, and
    // every one of them silently corrupted the modulation before this fix.
    EXPECT_GT(sf_first_unsafe, 0)
        << "the reverse-order rollback is now safe everywhere, which would mean "
           "the LLCC68 SF window rule changed — recheck LLCC68::setSpreadingFactor";
}
