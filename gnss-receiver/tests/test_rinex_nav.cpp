// The RINEX navigation loader on a RINEX 2.11 GPS file, as the IGS broadcast files of the PSAS
// flight's day come (milestone 7): one record of brdc2000.15n, PRN 30.

extern "C" {
#include "rinex_nav.h"
}

#include <gtest/gtest.h>

#include <cstdio>
#include <string>

TEST(RinexNav, ReadsRinex2Gps)
{
    const char *text =
        "     2.11           N: GPS NAV DATA                         RINEX VERSION / TYPE\n"
        "    7.4506D-09  2.2352D-08 -5.9605D-08 -1.1921D-07          ION ALPHA\n"
        "    9.0112D+04  1.1469D+05 -6.5536D+04 -5.2429D+05          ION BETA\n"
        "    17                                                      LEAP SECONDS\n"
        "                                                            END OF HEADER\n"
        "30 15  7 12  0  0  0.0-1.111067831516D-05 7.617018127348D-12 0.000000000000D+00\n"
        "    3.500000000000D+01 3.181250000000D+01 4.253748614275D-09 2.771537949177D+00\n"
        "    1.654028892517D-06 1.622568350285D-03 1.091323792934D-05 5.153725257874D+03\n"
        "    0.000000000000D+00 1.490116119385D-08 2.809793356429D-01 3.725290298462D-09\n"
        "    9.558577817590D-01 1.681562500000D+02 2.986111601820D+00-7.834254899591D-09\n"
        "   -2.650110387735D-10 1.000000000000D+00 1.853000000000D+03 0.000000000000D+00\n"
        "    2.000000000000D+00 0.000000000000D+00 3.259629011154D-09 3.500000000000D+01\n"
        "   -3.150000000000D+03\n";
    const std::string path = ::testing::TempDir() + "brdc_v2_test.15n";
    FILE *fp = std::fopen(path.c_str(), "w");
    ASSERT_NE(fp, nullptr);
    std::fputs(text, fp);
    std::fclose(fp);

    static gps_eph_t eph[GNSS_SYS_COUNT][GNSS_MAX_PRN + 1];
    gps_iono_t iono;
    ASSERT_EQ(rinex_nav_load(path.c_str(), 1u << GNSS_SYS_GPS, 1853, 0.0, eph, &iono), 1);
    const gps_eph_t &e = eph[GNSS_SYS_GPS][30];
    ASSERT_TRUE(e.valid);
    EXPECT_DOUBLE_EQ(e.af0, -1.111067831516e-05);
    EXPECT_DOUBLE_EQ(e.af1, 7.617018127348e-12);
    EXPECT_EQ(e.iode, 35);
    EXPECT_DOUBLE_EQ(e.crs, 31.8125);
    EXPECT_DOUBLE_EQ(e.m0, 2.771537949177);
    EXPECT_DOUBLE_EQ(e.sqrt_a, 5153.725257874);
    EXPECT_DOUBLE_EQ(e.toe, 0.0);
    EXPECT_DOUBLE_EQ(e.omega, 2.986111601820);
    EXPECT_DOUBLE_EQ(e.idot, -2.650110387735e-10);
    EXPECT_EQ(e.week, 1853);
    EXPECT_EQ(e.health, 0);
    EXPECT_DOUBLE_EQ(e.tgd, 3.259629011154e-09);
    EXPECT_EQ(e.iodc, 35);
    EXPECT_DOUBLE_EQ(e.toc, 0.0);  // 2015-07-12 00:00, a Sunday
    EXPECT_TRUE(iono.valid);
    EXPECT_DOUBLE_EQ(iono.alpha[1], 2.2352e-08);
    EXPECT_DOUBLE_EQ(iono.beta[3], -5.2429e+05);
    std::remove(path.c_str());
}
