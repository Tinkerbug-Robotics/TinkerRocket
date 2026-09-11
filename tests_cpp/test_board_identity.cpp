// #773 step 2: board identity that survives a reflash.
//
// The point of the feature is that "what this board IS" and "what this image
// was BUILT FOR" are different questions, and a disagreement between them is
// the signal. These pin the normalising, the comparison and the wire encoding
// that carries the first answer across the FC->OC link.

#include <gtest/gtest.h>
#include <BoardIdentity.h>
#include <string>

using namespace board_identity;

static std::string norm(const char* in) {
    char out[kMaxRev + 1];
    normalizeRev(in, out, sizeof(out));
    return std::string(out);
}

TEST(BoardIdentity, NormalisesTheCompileTimeConstantsThisRepoActuallyUses) {
    // These are the literal TR_BOARD_REV_STR values in flight_computer/config.h.
    EXPECT_EQ(norm("V9/V10"), "V9");
    EXPECT_EQ(norm("M1 (rocket-computer-mini)"), "M1");
    EXPECT_EQ(norm("V8"), "V8");
    EXPECT_EQ(norm("V7"), "V7");
}

TEST(BoardIdentity, V9SlashV10ReducingToV9IsDeliberate) {
    // Those two revisions share one image, so the constant cannot distinguish
    // them. A board provisioned as V10 therefore DISAGREES with a V9/V10 image
    // — and that is the honest answer, not a bug to paper over.
    EXPECT_FALSE(revsMatch("V9/V10", "V10"));
    EXPECT_TRUE(revsMatch("V9/V10", "v9"));
}

TEST(BoardIdentity, CaseAndPunctuationDoNotMatter) {
    EXPECT_TRUE(revsMatch("v9", "V9"));
    EXPECT_TRUE(revsMatch(" V9 ", "v9"));
    EXPECT_TRUE(revsMatch("V9-rev-b", "v9"));
}

TEST(BoardIdentity, UnprovisionedNeverReadsAsAgreement) {
    // The whole feature fails safe on this: an empty side must never match.
    EXPECT_FALSE(revsMatch("", "V9"));
    EXPECT_FALSE(revsMatch("V9", ""));
    EXPECT_FALSE(revsMatch("", ""));
    EXPECT_FALSE(revsMatch(nullptr, "V9"));
    EXPECT_FALSE(revsMatch("V9", nullptr));
    EXPECT_FALSE(revsMatch("---", "V9"));   // nothing usable in it
}

TEST(BoardIdentity, DifferentBoardsDoNotMatch) {
    EXPECT_FALSE(revsMatch("V8", "V9"));
    EXPECT_FALSE(revsMatch("M1", "V9"));
    EXPECT_FALSE(revsMatch("V1", "V10"));   // not a prefix comparison
}

TEST(BoardIdentity, PayloadRoundTrips) {
    char buf[64], ver[40], rev[kMaxRev + 1];
    const size_t n = encodePayload("537dc3ff-dirty-v9+20260909-1835", "v9", buf, sizeof(buf));
    decodePayload(buf, n, ver, sizeof(ver), rev, sizeof(rev));
    EXPECT_STREQ(ver, "537dc3ff-dirty-v9+20260909-1835");
    EXPECT_STREQ(rev, "V9");
}

TEST(BoardIdentity, AnOldFcSendsNoRevisionAndThatReadsAsUnprovisioned) {
    // Pre-#773 the payload was a bare version string. A new OC must read that
    // as "not provisioned", never as a match against anything.
    const char* legacy = "537dc3ff-dirty-v9+20260909-1835";
    char ver[40], rev[kMaxRev + 1];
    decodePayload(legacy, strlen(legacy), ver, sizeof(ver), rev, sizeof(rev));
    EXPECT_STREQ(ver, "537dc3ff-dirty-v9+20260909-1835");
    EXPECT_STREQ(rev, "");
    EXPECT_FALSE(revsMatch(rev, "V9"));
}

TEST(BoardIdentity, AnOldOcReadingANewPayloadStillGetsTheVersion) {
    // The old OC does a bounded memcpy into a zeroed buffer and uses it as a
    // C string, so it stops at the NUL and never sees the revision. Simulate
    // exactly that.
    char buf[64];
    const size_t n = encodePayload("abc123-v9+1", "V9", buf, sizeof(buf));
    ASSERT_GT(n, strlen("abc123-v9+1"));
    char old_side[40] = {0};
    memcpy(old_side, buf, (n < sizeof(old_side) - 1) ? n : sizeof(old_side) - 1);
    EXPECT_STREQ(old_side, "abc123-v9+1");
}

TEST(BoardIdentity, ATrailingNulPaddedRevisionIsTrimmed) {
    // A fixed-size sender may NUL-pad the tail; the decode must not carry that
    // padding into the string.
    char buf[64] = {0};
    memcpy(buf, "abc-v9+1", 8);
    buf[8] = '\0';
    memcpy(buf + 9, "V9", 2);
    char ver[40], rev[kMaxRev + 1];
    decodePayload(buf, 32, ver, sizeof(ver), rev, sizeof(rev));
    EXPECT_STREQ(ver, "abc-v9+1");
    EXPECT_STREQ(rev, "V9");
}

TEST(BoardIdentity, EncodeTruncatesRatherThanOverruns) {
    char tiny[8];
    const size_t n = encodePayload("a-very-long-version-string", "V9", tiny, sizeof(tiny));
    EXPECT_LE(n, sizeof(tiny));
    EXPECT_EQ(tiny[sizeof(tiny) - 1], '\0');
}

TEST(BoardIdentity, ACorruptNvsValueCannotBecomeAnUnboundedString) {
    // The cap is why kMaxRev exists: garbage in NVS must not reach the wire.
    std::string junk(200, 'X');
    char out[kMaxRev + 1];
    const size_t n = normalizeRev(junk.c_str(), out, sizeof(out));
    EXPECT_EQ(n, kMaxRev);
    EXPECT_EQ(strlen(out), kMaxRev);
}

// --- #413: the one-byte revision code for the flight log --------------------
//
// The log had no way to say which board produced it — fw_git_sha is seven
// characters of SHA and nothing else — so Data_Analysis could not tell a V7
// log from a V9 one. One byte was all the headroom FlightSettingsData had
// against MAX_PAYLOAD, hence a code rather than a string.

TEST(BoardIdentity, RevCodeRoundTripsEveryShippedBoard) {
    char out[24];
    struct { const char* rev; uint8_t want; } cases[] = {
        {"V7", 0x17}, {"V8", 0x18}, {"V9", 0x19}, {"V10", 0x1A},
        {"M1", 0x21}, {"B1", 0x31},
    };
    for (const auto& c : cases) {
        const uint8_t code = board_identity::encodeRevCode(c.rev, false);
        EXPECT_EQ(code, c.want) << c.rev;
        board_identity::revCodeToString(code, out, sizeof(out));
        EXPECT_STREQ(out, c.rev);
    }
}

TEST(BoardIdentity, TheAssertedBitSurvivesAndIsVisible) {
    // #773's whole point: the image's idea of its own board is circular, so a
    // log that mixed the two silently would be worse than one carrying
    // neither. The byte has to say which question it answered.
    char out[24];
    const uint8_t provisioned = board_identity::encodeRevCode("V9", false);
    const uint8_t asserted = board_identity::encodeRevCode("V9", true);
    EXPECT_NE(provisioned, asserted);
    EXPECT_EQ(asserted & 0x7F, provisioned);
    EXPECT_TRUE(asserted & board_identity::kRevAsserted);
    board_identity::revCodeToString(asserted, out, sizeof(out));
    EXPECT_STREQ(out, "V9 (asserted)");
}

TEST(BoardIdentity, TheBuildFlagStringsUsedByTheFirmwareEncode) {
    // The FC passes TR_BOARD_REV_STR straight in, and those are prose:
    // "V9/V10" and "M1 (rocket-computer-mini)". normalizeRev cuts at the first
    // non-alphanumeric, so both reduce to a token. "V9/V10" reducing to V9 is
    // deliberate — the two share an image, so a V10 board provisioned as "V10"
    // is a real mismatch and must not be hidden here.
    char out[24];
    EXPECT_EQ(board_identity::encodeRevCode("V9/V10", true), 0x99);
    board_identity::revCodeToString(
        board_identity::encodeRevCode("M1 (rocket-computer-mini)", true),
        out, sizeof(out));
    EXPECT_STREQ(out, "M1 (asserted)");
}

TEST(BoardIdentity, UnrepresentableRevisionsBecomeUnknownNotAWrongGuess) {
    // A code that cannot be trusted must read as absent. Every one of these
    // would be worse as a plausible-looking wrong board than as "unknown".
    for (const char* bad : {"", "V", "9", "X9", "V0", "V16", "V1A",
                            (const char*)nullptr}) {
        EXPECT_EQ(board_identity::encodeRevCode(bad, false),
                  board_identity::kRevUnknown) << (bad ? bad : "(null)");
    }
    char out[24];
    board_identity::revCodeToString(board_identity::kRevUnknown, out, sizeof(out));
    EXPECT_STREQ(out, "unknown");
    // A pre-v9 log decodes as a zeroed byte, and that is exactly right: the
    // field was never recorded, which is not the same as a board with no name.
    board_identity::revCodeToString(0x00, out, sizeof(out));
    EXPECT_STREQ(out, "unknown");
}

TEST(BoardIdentity, ANibbleOverflowIsRefusedRatherThanWrapped) {
    // V16 does not fit four bits. Wrapping it would report V0, or worse, V1.
    EXPECT_EQ(board_identity::encodeRevCode("V16", false),
              board_identity::kRevUnknown);
    // V15 is the last one that fits, and must still work.
    char out[24];
    const uint8_t code = board_identity::encodeRevCode("V15", false);
    EXPECT_NE(code, board_identity::kRevUnknown);
    board_identity::revCodeToString(code, out, sizeof(out));
    EXPECT_STREQ(out, "V15");
}
