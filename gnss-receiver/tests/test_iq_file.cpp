// Reading recorded IQ: the PSAS jGPS format (MAX2769, 2-bit sign/magnitude I and Q packed two
// samples a byte), against PSAS's own description of it (milestone 7).

extern "C" {
#include "iq_file.h"
}

#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

TEST(IqFile, ReadsPsasMax2769TwoBit)
{
    // Nibbles, MSB first: I-mag, I-sign, Q-mag, Q-sign; the older sample in the high nibble.
    // 0x0 = (+1, +1); 0xF = (-3, -3); 0x8 = (+3, +1); 0x5 = (-1, -1); 0xA = (+3, +3); 0x6 = (-1, +3).
    const unsigned char bytes[] = {0x0F, 0x85, 0xA6};
    const std::string path = ::testing::TempDir() + "psas_iq_test.bin";
    FILE *fp = std::fopen(path.c_str(), "wb");
    ASSERT_NE(fp, nullptr);
    std::fwrite(bytes, 1, sizeof(bytes), fp);
    std::fclose(fp);

    iqf_t f;
    ASSERT_EQ(iqf_open(&f, path.c_str(), IQF_MAX2769_2B, 4.092e6, 1575.42e6), 0);
    EXPECT_EQ(f.nsamp, 6);
    std::vector<float> iq(12);
    ASSERT_EQ(iqf_read(&f, iq.data(), 6), 6u);
    const float want[12] = {1, 1, -3, -3, 3, 1, -1, -1, 3, 3, -1, 3};
    for (int k = 0; k < 12; k++) {
        EXPECT_EQ(iq[k], want[k]) << k;
    }
    // From an odd sample, and across a byte: samples 3..5.
    ASSERT_EQ(iqf_seek(&f, 3), 0);
    ASSERT_EQ(iqf_read(&f, iq.data(), 5), 3u);  // only three remain
    for (int k = 0; k < 6; k++) {
        EXPECT_EQ(iq[k], want[6 + k]) << k;
    }
    iqf_close(&f);
    std::remove(path.c_str());
}
