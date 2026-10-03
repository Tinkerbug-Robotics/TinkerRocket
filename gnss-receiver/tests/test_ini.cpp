#include "ini.h"

#include <gtest/gtest.h>

TEST(Ini, SectionsKeysAndComments)
{
    ini_t ini;
    ASSERT_EQ(ini_load_string(&ini,
                              "; header comment\n"
                              "[a.C8]\n"
                              "fs = 8184000   ; trailing comment\n"
                              "  name=x;y  \n"
                              "# hash comment\n"
                              "[b.C8]\n"
                              "fs = 2600000\r\n"
                              "dc = auto\n"),
              0);
    char v[64];
    ASSERT_EQ(ini_get(&ini, "a.C8", "fs", v, sizeof(v)), 0);
    EXPECT_STREQ(v, "8184000");
    ASSERT_EQ(ini_get(&ini, "a.C8", "name", v, sizeof(v)), 0);
    EXPECT_STREQ(v, "x;y");  // a ';' only starts a comment after whitespace
    ASSERT_EQ(ini_get(&ini, "b.C8", "fs", v, sizeof(v)), 0);
    EXPECT_STREQ(v, "2600000");
    ASSERT_EQ(ini_get(&ini, "b.C8", "dc", v, sizeof(v)), 0);
    EXPECT_STREQ(v, "auto");
    EXPECT_NE(ini_get(&ini, "a.C8", "dc", v, sizeof(v)), 0);
    EXPECT_NE(ini_get(&ini, "c.C8", "fs", v, sizeof(v)), 0);
    ini_free(&ini);
}
