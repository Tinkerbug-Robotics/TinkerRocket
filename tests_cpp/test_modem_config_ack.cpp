#include <gtest/gtest.h>
#include "modem_config_ack.h"

using modem_config_ack::Ack;
using modem_config_ack::Want;
using modem_config_ack::accepted;

// #835 item 7.  A failed TR_LoRa_Comms::reconfigure() rolls the radio back to
// its PREVIOUS modulation and leaves it running, so the modem still acks with
// radio_enabled=1.  The host used to read that as success, cache a modulation
// that was never on the air, and let the OC write it to NVS — after which the
// next boot begin()s the illegal pair and disables the radio outright.

namespace {
constexpr float kF = 915.0f;
constexpr uint8_t kSF = 9;
}

// ---- the bug ----------------------------------------------------------------

TEST(ModemConfigAck, RollbackIsRejectedEvenThoughTheRadioIsAlive) {
    // The exact failure: operator pushes SF10 @ BW125, which the LLCC68 cannot
    // do (it caps SF at 9 for BW125). setSpreadingFactor errors, reconfigure()
    // rolls back to SF9, radio stays up and acks.
    Ack rolled_back{radio_modem::CFG_ACK_REJECTED, kF, kSF};
    EXPECT_FALSE(accepted(rolled_back, Want{kF, 10}));
}

TEST(ModemConfigAck, AnAppliedConfigIsAccepted) {
    EXPECT_TRUE(accepted(Ack{radio_modem::CFG_ACK_APPLIED, kF, kSF}, Want{kF, kSF}));
}

// ---- the compatibility rule I nearly got wrong -------------------------------

TEST(ModemConfigAck, UnknownIsNotTreatedAsRejected) {
    // config_ok was `reserved[0]` before this change, so an un-reflashed
    // daughterboard zero-fills it. Treating 0 as a rejection would make every
    // config from an older modem image fail — a regression, not a fix.
    // Directly relevant: of the two base stations in service, only the
    // UART-radio one has a daughterboard that needs reflashing at all.
    EXPECT_TRUE(accepted(Ack{radio_modem::CFG_ACK_UNKNOWN, kF, kSF}, Want{kF, kSF}));
}

TEST(ModemConfigAck, LegacyImageStillCaughtByTheOnAirCompare) {
    // ...and the legacy path is still protected: a rollback reports the
    // PREVIOUS modulation, which will not match what we asked for.
    EXPECT_FALSE(accepted(Ack{radio_modem::CFG_ACK_UNKNOWN, kF, kSF}, Want{kF, 10}));
}

// ---- the on-air comparison --------------------------------------------------

TEST(ModemConfigAck, MismatchedFrequencyIsRejected) {
    EXPECT_FALSE(accepted(Ack{radio_modem::CFG_ACK_APPLIED, 868.0f, kSF}, Want{kF, kSF}));
}

TEST(ModemConfigAck, AppliedIsStillVerifiedAgainstWhatIsOnTheAir) {
    // A modem claiming success while reporting a different SF is not believed.
    EXPECT_FALSE(accepted(Ack{radio_modem::CFG_ACK_APPLIED, kF, 7}, Want{kF, kSF}));
}

TEST(ModemConfigAck, FloatRoundTripDoesNotSpuriouslyReject) {
    // current_freq_mhz round-trips through the radio's register math, so an
    // exact compare would reject configs that really did apply.
    EXPECT_TRUE(accepted(Ack{radio_modem::CFG_ACK_APPLIED, 915.0004f, kSF}, Want{kF, kSF}));
    EXPECT_FALSE(accepted(Ack{radio_modem::CFG_ACK_APPLIED, 915.05f, kSF}, Want{kF, kSF}));
}
