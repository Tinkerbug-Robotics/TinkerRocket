#include <gtest/gtest.h>

#include "tx_frame_policy.h"

using tx_frame_policy::Verdict;
using tx_frame_policy::admit;
using tx_frame_policy::owesResult;

namespace {
constexpr uint8_t kCap = radio_modem::TX_QUEUE_CAPACITY;
// len is the WHOLE payload: one seq byte plus the air bytes.
constexpr size_t kSeqOnly = 1;
}

// #1152. RadioModemProtocol.h states the invariant absolutely: "A TX frame is
// therefore never silently dropped: every accepted seq is answered." The
// host's credit window is built on it — a seq that is neither answered nor
// rejected is a credit it never gets back, and eight of those stop it
// transmitting with no diagnostic at all.

TEST(TxFramePolicy, TheOnlySilentDropIsAFrameWithNoSeqOnTheWire) {
    EXPECT_EQ(admit(0, true, 0, kCap), Verdict::DropNoSeq);
    EXPECT_FALSE(owesResult(Verdict::DropNoSeq));
    // Every other length carries a seq, so every other verdict owes a result.
    for (size_t len : {size_t{1}, size_t{2}, size_t{100},
                       radio_modem::MAX_AIR_FRAME + 2}) {
        const Verdict v = admit(len, true, 0, kCap);
        EXPECT_TRUE(v == Verdict::Accept || owesResult(v)) << len;
    }
}

TEST(TxFramePolicy, ASeqWithNoAirBytesIsAnsweredNotDropped) {
    // THE BUG. The old guard was `len < sizeof(TxFrameHeader) + 1`, i.e.
    // len < 2, justified as "no seq to answer" — but at len == 1 the seq is on
    // the wire at payload[0]. The frame was answerable and was dropped anyway,
    // leaking one credit per occurrence.
    const Verdict v = admit(kSeqOnly, true, 0, kCap);
    EXPECT_EQ(v, Verdict::RejectEmptyAir);
    EXPECT_TRUE(owesResult(v)) << "a 1-byte TX_FRAME must be answered, not dropped";
    EXPECT_NE(v, Verdict::DropNoSeq);
    EXPECT_NE(v, Verdict::Accept) << "and it must not be transmitted either";
}

TEST(TxFramePolicy, AnEmptyAirFrameIsMalformedWhateverTheRadioIsDoing) {
    // Checked before the radio state deliberately: blaming a down radio for a
    // malformed frame sends the reader after the wrong fault.
    EXPECT_EQ(admit(kSeqOnly, false, 0, kCap), Verdict::RejectEmptyAir);
    EXPECT_EQ(admit(kSeqOnly, true, kCap, kCap), Verdict::RejectEmptyAir);
}

TEST(TxFramePolicy, TheRejectionLadderIsOrderedBySpecificity) {
    const size_t ok_len = 10;
    EXPECT_EQ(admit(radio_modem::MAX_AIR_FRAME + 2, true, 0, kCap),
              Verdict::RejectTooLong);
    EXPECT_EQ(admit(ok_len, false, 0, kCap), Verdict::RejectRadioDown);
    EXPECT_EQ(admit(ok_len, true, kCap, kCap), Verdict::RejectQueueFull);
    EXPECT_EQ(admit(ok_len, true, kCap - 1, kCap), Verdict::Accept);
    // Too long outranks a down radio: the frame would be refused either way,
    // and the length is the actionable half.
    EXPECT_EQ(admit(radio_modem::MAX_AIR_FRAME + 2, false, 0, kCap),
              Verdict::RejectTooLong);
}

TEST(TxFramePolicy, TheLargestLegalAirFrameIsAccepted) {
    // Off-by-one in the other direction: MAX_AIR_FRAME air bytes is legal, so
    // the whole payload is one longer than that.
    EXPECT_EQ(admit(radio_modem::MAX_AIR_FRAME + 1, true, 0, kCap),
              Verdict::Accept);
    EXPECT_EQ(admit(radio_modem::MAX_AIR_FRAME + 2, true, 0, kCap),
              Verdict::RejectTooLong);
}

TEST(TxFramePolicy, EveryLengthEitherFliesOrIsAnswered) {
    // The invariant, swept rather than spot-checked: across every length a host
    // can send and every modem state, the only unanswered outcome is the frame
    // that carries no seq.
    for (size_t len = 0; len <= radio_modem::MAX_AIR_FRAME + 4; ++len) {
        for (bool up : {false, true}) {
            for (uint8_t used : {uint8_t{0}, uint8_t(kCap - 1), kCap}) {
                const Verdict v = admit(len, up, used, kCap);
                if (v == Verdict::DropNoSeq) {
                    EXPECT_EQ(len, 0u) << "silent drop at len " << len;
                } else {
                    EXPECT_TRUE(v == Verdict::Accept || owesResult(v)) << len;
                }
            }
        }
    }
}
