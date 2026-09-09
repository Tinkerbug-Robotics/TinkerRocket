// #1155 item 4: a unit name with a quote or backslash must not break the
// config_identity readback frame.
#include <gtest/gtest.h>
#include <cstring>
#include <JsonEscape.h>

using tr_json::escapeInto;
using tr_json::isPlainString;

TEST(JsonEscape, PlainNamePassesThrough)
{
    char out[64];
    ASSERT_TRUE(escapeInto(out, sizeof(out), "Rolly Polly V"));
    EXPECT_STREQ(out, "Rolly Polly V");
    EXPECT_TRUE(isPlainString("Rolly Polly V", 13));
}

TEST(JsonEscape, QuoteAndBackslashAreEscaped)
{
    char out[64];
    ASSERT_TRUE(escapeInto(out, sizeof(out), "Say \"hi\" \\ bye"));
    EXPECT_STREQ(out, "Say \\\"hi\\\" \\\\ bye");
    EXPECT_FALSE(isPlainString("Say \"hi\"", 8));
    EXPECT_FALSE(isPlainString("a\\b", 3));
}

TEST(JsonEscape, ControlBytesAreDroppedNotEmitted)
{
    char out[64];
    ASSERT_TRUE(escapeInto(out, sizeof(out), "tab\there\nnew\x7f"));
    EXPECT_STREQ(out, "tabherenew");
    EXPECT_FALSE(isPlainString("\t", 1));
    EXPECT_FALSE(isPlainString("\x7f", 1));
}

TEST(JsonEscape, Utf8IsPlain)
{
    const char* name = "Fus\xc3\xa9\x65 \xe2\x9c\x88";   // "Fusée ✈"
    char out[64];
    ASSERT_TRUE(escapeInto(out, sizeof(out), name));
    EXPECT_STREQ(out, name);
    EXPECT_TRUE(isPlainString(name, std::strlen(name)));
}

TEST(JsonEscape, RefusesRatherThanTruncates)
{
    char out[8];
    // "abcdefg" fits (7 + NUL); "abcdefgh" does not.
    ASSERT_TRUE(escapeInto(out, sizeof(out), "abcdefg"));
    EXPECT_STREQ(out, "abcdefg");
    EXPECT_FALSE(escapeInto(out, sizeof(out), "abcdefgh"));
    EXPECT_STREQ(out, "");
    // Escapes count double: four quotes need 8 + NUL.
    EXPECT_FALSE(escapeInto(out, sizeof(out), "\"\"\"\""));
    EXPECT_STREQ(out, "");
    char big[64];
    ASSERT_TRUE(escapeInto(big, sizeof(big), "\"\"\"\""));
    EXPECT_STREQ(big, "\\\"\\\"\\\"\\\"");
}

TEST(JsonEscape, WorstCaseUnitNameFitsTheReadbackBuffer)
{
    // unit_name is char[24] on every board (23 bytes + NUL); the readback
    // sizes its escape buffer at 2 * sizeof(unit_name).
    char name[24];
    std::memset(name, '"', 23); name[23] = '\0';
    char out[48];
    ASSERT_TRUE(escapeInto(out, sizeof(out), name));
    EXPECT_EQ(std::strlen(out), 46u);
}

TEST(JsonEscape, EmptyAndNull)
{
    char out[8];
    ASSERT_TRUE(escapeInto(out, sizeof(out), ""));
    EXPECT_STREQ(out, "");
    ASSERT_TRUE(escapeInto(out, sizeof(out), nullptr));
    EXPECT_STREQ(out, "");
    EXPECT_FALSE(escapeInto(nullptr, 8, "x"));
    EXPECT_FALSE(escapeInto(out, 0, "x"));
    EXPECT_TRUE(isPlainString(nullptr, 0));
}
