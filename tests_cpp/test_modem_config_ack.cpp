#include <gtest/gtest.h>
#include <string.h>
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

// --- #1173: which half was rejected ----------------------------------------
//
// The reason byte is diagnostics, so the first thing to pin is that it changes
// NO verdict: pushConfig must still fail on CFG_ACK_REJECTED whatever the
// reason says, and a reason must never rescue a rejection or cause one.

TEST(ModemConfigAck, TheReasonByteNeverChangesTheVerdict) {
    // Every reason, and a nonsense one, against an otherwise perfect ack.
    for (uint8_t r : {uint8_t{0},
                      uint8_t{radio_modem::CFG_FAIL_MODULATION},
                      uint8_t{radio_modem::CFG_FAIL_FRAME_PARAMS},
                      uint8_t{radio_modem::CFG_FAIL_RADIO_DOWN},
                      uint8_t{0x80}}) {
        EXPECT_FALSE(accepted(Ack{radio_modem::CFG_ACK_REJECTED, kF, kSF, r},
                              Want{kF, kSF})) << "reason " << unsigned(r);
        EXPECT_TRUE(accepted(Ack{radio_modem::CFG_ACK_APPLIED, kF, kSF, r},
                             Want{kF, kSF})) << "reason " << unsigned(r);
    }
}

TEST(ModemConfigAck, FrameParamFailureIsTheOneTheOnAirCompareCannotSee) {
    // The whole reason this field is worth a byte. The requested modulation IS
    // live, so freq and SF read back exactly right and the on-air comparison
    // is happy — only config_ok says no, and only the reason says why.
    const Ack ack{radio_modem::CFG_ACK_REJECTED, kF, kSF,
                  radio_modem::CFG_FAIL_FRAME_PARAMS};
    EXPECT_FALSE(accepted(ack, Want{kF, kSF}));
    // Strip the rejection and the same numbers would have passed — which is
    // exactly why the operator's log used to read as a contradiction.
    EXPECT_TRUE(accepted(Ack{radio_modem::CFG_ACK_APPLIED, kF, kSF, 0},
                         Want{kF, kSF}));
    EXPECT_NE(strstr(reason_text(ack), "frame format"), nullptr) << reason_text(ack);
    EXPECT_NE(strstr(reason_text(ack), "IS live"), nullptr) << reason_text(ack);
}

TEST(ModemConfigAck, EachFailurePathReadsAsItsOwnEvent) {
    const Ack down{radio_modem::CFG_ACK_REJECTED, kF, kSF,
                   radio_modem::CFG_FAIL_RADIO_DOWN};
    const Ack rolled{radio_modem::CFG_ACK_REJECTED, kF, kSF,
                     radio_modem::CFG_FAIL_MODULATION};
    EXPECT_NE(strstr(reason_text(down), "nothing is on the air"), nullptr) << reason_text(down);
    EXPECT_NE(strstr(reason_text(rolled), "rolled back"), nullptr) << reason_text(rolled);
    // Three distinct paths must not collapse into one string — that collapse
    // is the defect this issue is about.
    EXPECT_STRNE(reason_text(down), reason_text(rolled));
}

TEST(ModemConfigAck, RadioDownWinsWhenMoreThanOneBitIsSet) {
    // A modem that sets several has one thing worth saying first: there is
    // nothing on the air at all, which subsumes any complaint about WHICH
    // modulation or frame format failed to take.
    const Ack many{radio_modem::CFG_ACK_REJECTED, kF, kSF,
                   uint8_t(radio_modem::CFG_FAIL_RADIO_DOWN |
                           radio_modem::CFG_FAIL_MODULATION |
                           radio_modem::CFG_FAIL_FRAME_PARAMS)};
    EXPECT_NE(strstr(reason_text(many), "nothing is on the air"), nullptr) << reason_text(many);
}

TEST(ModemConfigAck, NoReasonIsReportedWhereThereIsNothingToExplain) {
    // A legacy modem zero-fills the byte and can never send REJECTED, so the
    // field is simply never read on such a link — and an applied config has
    // nothing to explain even if a stale reason were somehow present.
    EXPECT_STREQ(reason_text(Ack{radio_modem::CFG_ACK_UNKNOWN, kF, kSF, 0}), "");
    EXPECT_STREQ(reason_text(Ack{radio_modem::CFG_ACK_APPLIED, kF, kSF,
                                 radio_modem::CFG_FAIL_MODULATION}), "");
    // Rejected with no reason is the honest "it said no and did not say why".
    EXPECT_STREQ(reason_text(Ack{radio_modem::CFG_ACK_REJECTED, kF, kSF, 0}), "");
    // An unrecognised bit is reported as unrecognised, not silently dropped.
    EXPECT_NE(strstr(reason_text(Ack{radio_modem::CFG_ACK_REJECTED, kF, kSF, 0x80}),
                     "does not recognise"), nullptr);
}

TEST(ModemStatusWire, TheReasonByteCameOutOfTheSpareBytes) {
    // #1173's constraint: take a spare byte, do not grow the struct. If this
    // ever fails, every deployed modem and host disagree about the layout of
    // everything after it.
    static_assert(sizeof(radio_modem::ModemStatusData) == 52,
                  "ModemStatusData must stay 52 bytes");
    radio_modem::ModemStatusData st = {};
    st.config_fail_reason = radio_modem::CFG_FAIL_FRAME_PARAMS;
    EXPECT_EQ(st.config_fail_reason, 0x02);
    EXPECT_EQ(sizeof(st.reserved), 1u);
}
