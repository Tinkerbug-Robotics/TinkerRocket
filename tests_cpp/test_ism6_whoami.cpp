#include <gtest/gtest.h>
#include <string>
#include "ism6_whoami.h"

// The WHO_AM_I values the IMU driver runs: the ISM6HG256X answers 0x73 and
// the ISM6HGK256X, which shares its pinout, register map and scaling, answers
// 0x75. Anything else is a missing or wrong part, and the collector halts on
// it rather than configuring registers it does not know.

TEST(Ism6WhoAmI, AcceptsBothParts)
{
    EXPECT_TRUE(ism6_whoami::supported(0x73));
    EXPECT_TRUE(ism6_whoami::supported(0x75));
}

TEST(Ism6WhoAmI, RejectsEveryOtherValue)
{
    for (int id = 0; id <= 0xFF; ++id)
    {
        if (id == 0x73 || id == 0x75) continue;
        EXPECT_FALSE(ism6_whoami::supported(static_cast<uint8_t>(id))) << "id 0x" << std::hex << id;
    }
}

TEST(Ism6WhoAmI, NamesThePartForTheBootLog)
{
    EXPECT_EQ(std::string(ism6_whoami::part_name(0x73)), "ISM6HG256X");
    EXPECT_EQ(std::string(ism6_whoami::part_name(0x75)), "ISM6HGK256X");
    EXPECT_EQ(std::string(ism6_whoami::part_name(0x00)), "unknown");
}
