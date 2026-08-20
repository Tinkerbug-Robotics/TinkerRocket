#include <gtest/gtest.h>
#include "MramProbe.h"

// #826: TR_LogToFlash::begin() used to select the MRAM ring from the CS pin
// number alone. On a board where the part is absent, unfitted or dead, every
// frame was clocked onto an unconnected pad and noise was read back — then
// programmed into the NAND inside a valid PageHeader with a good CRC32, so
// nothing downstream could catch it while the storage scorecard stayed green.
//
// The probe now toggles WEL and requires the part to follow both commands.
// These tests pin the property that argument rests on: no CONSTANT bus value
// can pass, which is what an absent part produces.

using tr::mramProbeVerdict;

TEST(MramProbe, HealthyPartSetsThenClearsWel) {
    // WEL set after WREN, clear after WRDI.
    EXPECT_TRUE(mramProbeVerdict(0x02, 0x00));
}

TEST(MramProbe, HealthyPartWithBlockProtectBitsSet) {
    // Only WEL is examined — block-protect and reserved bits vary with how the
    // part was last configured and must not affect the verdict.
    EXPECT_TRUE(mramProbeVerdict(0x8E, 0x8C));
}

TEST(MramProbe, FloatingLowBusFails) {
    // The blind spot a bare RDSR would have: 0x00 is a plausible status byte
    // (unprotected, WEL clear), so a bus floating low reads as a healthy part
    // on a single sample. Requiring a change catches it.
    EXPECT_FALSE(mramProbeVerdict(0x00, 0x00));
}

TEST(MramProbe, FloatingHighBusFails) {
    EXPECT_FALSE(mramProbeVerdict(0xFF, 0xFF));
}

TEST(MramProbe, NoConstantBusValueCanPass) {
    // The whole argument for the two-sample form: an absent part leaves MISO
    // floating, and a floating bus returns the SAME value to both reads.
    for (int v = 0; v <= 0xFF; ++v) {
        const uint8_t b = static_cast<uint8_t>(v);
        EXPECT_FALSE(mramProbeVerdict(b, b))
            << "constant bus value 0x" << std::hex << v << " passed the probe";
    }
}

TEST(MramProbe, StuckWelFails) {
    // Part answers but never clears WEL — not a healthy device.
    EXPECT_FALSE(mramProbeVerdict(0x02, 0x02));
}

TEST(MramProbe, WelNeverSetFails) {
    // Answers, but WREN had no effect: write-protected or not really there.
    EXPECT_FALSE(mramProbeVerdict(0x00, 0x02));
}

TEST(MramProbe, InvertedOrderFails) {
    // Guards against the two samples being passed the wrong way round.
    EXPECT_FALSE(mramProbeVerdict(0x00, 0x02));
    EXPECT_TRUE(mramProbeVerdict(0x02, 0x00));
}
