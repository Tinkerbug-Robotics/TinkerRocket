// #671: the RDID -> geometry mapping. This table is what stands between a
// 2 KB-page GD5F part and the legacy 4 KB-page write path that corrupts it —
// so every entry, the fallback, and the dead-bus behaviour are pinned here.
// The IDs are cross-validated against two silicon-tested drivers (the
// vendored spi_nand_flash devices table and the Linux kernel's spi-nand
// drivers); if an entry here ever has to change, the FIRST question is which
// physical chip was misidentified, not which test is inconvenient.

#include <gtest/gtest.h>

#include "nand_geometry.h"

#include <ios>

TEST(NandGeometry, ForeseeF35SQB004G_IsTheLegacyBenchPart) {
    NandGeometry g{};
    EXPECT_TRUE(nandGeometryForId(0xCD53, &g));
    EXPECT_EQ(g.page_size, 4096u);
    EXPECT_EQ(g.pages_per_blk, 64u);
    EXPECT_EQ(g.block_count, 2048u);
    EXPECT_EQ(g.blockSize(), 256u * 1024u);
    EXPECT_EQ(g.bitmapBytes(), 256u);
}

TEST(NandGeometry, GD5F2GQ5UE_V9Part) {
    NandGeometry g{};
    EXPECT_TRUE(nandGeometryForId(0xC852, &g));
    EXPECT_EQ(g.page_size, 2048u);
    EXPECT_EQ(g.pages_per_blk, 64u);
    EXPECT_EQ(g.block_count, 2048u);
    EXPECT_EQ(g.blockSize(), 128u * 1024u);
    EXPECT_EQ(g.bitmapBytes(), 256u);
}

TEST(NandGeometry, GD5F1GQ5UE_MiniPart) {
    NandGeometry g{};
    EXPECT_TRUE(nandGeometryForId(0xC851, &g));
    EXPECT_EQ(g.page_size, 2048u);
    EXPECT_EQ(g.pages_per_blk, 64u);
    EXPECT_EQ(g.block_count, 1024u);
    EXPECT_EQ(g.blockSize(), 128u * 1024u);
    EXPECT_EQ(g.bitmapBytes(), 128u);
}

TEST(NandGeometry, MX35UF4G24AD_Z4I8_V7Part) {
    // Byte-identical to the legacy fallback on purpose — the entry exists to
    // silence the unknown-ID ERROR tripwire on the V7 board, not to change
    // behaviour.
    NandGeometry g{};
    EXPECT_TRUE(nandGeometryForId(0xC2F5, &g));
    EXPECT_EQ(g.page_size, 4096u);
    EXPECT_EQ(g.pages_per_blk, 64u);
    EXPECT_EQ(g.block_count, 2048u);
    EXPECT_EQ(g.blockSize(), 256u * 1024u);
    EXPECT_EQ(g.bitmapBytes(), 256u);
}

TEST(NandGeometry, TwoPlaneMacronixStaysUnlisted) {
    // The plain MX35UF4G24AD (DID 0xB5) is a 2-plane part this single-plane
    // driver cannot address. It must NOT get a table entry: falling back keeps
    // the ERROR tripwire loud instead of silently claiming support.
    NandGeometry g{};
    EXPECT_FALSE(nandGeometryForId(0xC2B5, &g));
}

TEST(NandGeometry, UnknownIdFallsBackToLegacy) {
    // Fail-open on purpose: an unlisted part gets the pre-#671 behaviour
    // (right for any 4 Gbit/4 KB part, loudly logged by the driver), and the
    // V8 bench boards are covered twice — table hit AND identical fallback.
    NandGeometry g{};
    EXPECT_FALSE(nandGeometryForId(0xAAAA, &g));
    EXPECT_EQ(g.page_size, NAND_GEOMETRY_LEGACY.page_size);
    EXPECT_EQ(g.block_count, NAND_GEOMETRY_LEGACY.block_count);
}

TEST(NandGeometry, DeadBusIdsFallBackToLegacy) {
    // 0x0000 / 0xFFFF are the dead-bus sentinels nandInit() detects; they must
    // never match a table entry, so a dead bus keeps today's exact behaviour.
    NandGeometry g{};
    EXPECT_FALSE(nandGeometryForId(0x0000, &g));
    EXPECT_EQ(g.page_size, 4096u);
    EXPECT_FALSE(nandGeometryForId(0xFFFF, &g));
    EXPECT_EQ(g.page_size, 4096u);
}

TEST(NandGeometry, LegacyFallbackEqualsPre671Constants) {
    // The V8 byte-identity requirement, stated as a test: the fallback IS the
    // old compile-time geometry.
    EXPECT_EQ(NAND_GEOMETRY_LEGACY.page_size, 4096u);
    EXPECT_EQ(NAND_GEOMETRY_LEGACY.pages_per_blk, 64u);
    EXPECT_EQ(NAND_GEOMETRY_LEGACY.block_count, 2048u);
}

TEST(NandGeometry, EveryTablePartFitsTheStaticMaxima) {
    for (uint16_t id : {0xCD53, 0xC852, 0xC851, 0xC2F5}) {
        NandGeometry g{};
        ASSERT_TRUE(nandGeometryForId(id, &g)) << std::hex << id;
        EXPECT_LE(g.page_size, NAND_PAGE_SIZE_MAX) << std::hex << id;
        EXPECT_LE(g.block_count, NAND_BLOCK_COUNT_MAX) << std::hex << id;
        // All supported parts are 64 pages/block — the row-address packing
        // (page in RA[5:0]) and every cursor div/mod rely on it being
        // uniform; a future part that differs needs the SPI layer re-audited,
        // not just a table entry.
        EXPECT_EQ(g.pages_per_blk, 64u) << std::hex << id;
        // The sink payload must stay positive and the dual-copy index
        // snapshot (6160 B) must fit one block at every geometry.
        EXPECT_GT(g.page_size, 16u);
        EXPECT_GE(g.blockSize(), 6160u);
    }
}

// ---------------------------------------------------------------------------
// #1148 item 2 — on-die ECC status decoding.
//
// nandReadPage()/nandReadBytesAt() waited for OIP to clear and discarded the
// status byte, so an UNCORRECTABLE ECC error returned garbage as success. That
// also made every read-side bad-block path in the component dead code:
// lfsBlockRead's markBlockBad and scanBadBlocksAtBoot's "any read error on page
// 0 is a dead-block signal" could never fire, because a read could only fail on
// a 2-second OIP timeout.
//
// Semantics mirror the vendored, silicon-validated spi_nand_flash driver
// (src/nand_impl.c is_ecc_error): a 2-bit packed field for GigaDevice and
// Macronix, and a 3-bit LINEAR bit-count for the FORESEE/Longsys parts, which
// the generic 3-bit packing would misread.
// ---------------------------------------------------------------------------

TEST(NandEcc, TwoBitPackedOnlyFailsOnNotCorrected) {
    // STAT[5:4]: 0 = ok, 1 = corrected, 2 = UNCORRECTABLE, 3 = corrected(max)
    EXPECT_FALSE(nandEccUncorrectable(NandEccKind::TwoBitPacked, 0x00));
    EXPECT_FALSE(nandEccUncorrectable(NandEccKind::TwoBitPacked, 0x10));  // 1
    EXPECT_TRUE (nandEccUncorrectable(NandEccKind::TwoBitPacked, 0x20));  // 2
    EXPECT_FALSE(nandEccUncorrectable(NandEccKind::TwoBitPacked, 0x30));  // 3
}

TEST(NandEcc, ForeseeLinearOnlyFailsAtSeven) {
    // STAT[6:4] is a linear bit-count: 0 ok, 1..6 corrected, 7 uncorrectable.
    for (uint8_t v = 0; v <= 6; ++v) {
        EXPECT_FALSE(nandEccUncorrectable(NandEccKind::ForeseeLinear3,
                                          (uint8_t)(v << 4)))
            << "linear value " << (int)v << " is a CORRECTED read, not a failure";
    }
    EXPECT_TRUE(nandEccUncorrectable(NandEccKind::ForeseeLinear3, 0x70));
}

TEST(NandEcc, ForeseeValueTwoIsNotMisreadAsUncorrectable) {
    // The whole reason the FORESEE part needs its own branch: under the 2-bit
    // packing, ECCS=2 means "not corrected". Under FORESEE's linear count it
    // means 4 bits were corrected successfully. Reading it the generic way
    // would fail good pages and, through markBlockBad, retire good blocks.
    EXPECT_TRUE (nandEccUncorrectable(NandEccKind::TwoBitPacked,   0x20));
    EXPECT_FALSE(nandEccUncorrectable(NandEccKind::ForeseeLinear3, 0x20));
}

TEST(NandEcc, OtherStatusBitsDoNotAffectTheVerdict) {
    // OIP/EFAIL/PFAIL live in the low nibble and must not leak into the ECC
    // field's interpretation.
    EXPECT_TRUE (nandEccUncorrectable(NandEccKind::TwoBitPacked, 0x2F));
    EXPECT_FALSE(nandEccUncorrectable(NandEccKind::TwoBitPacked, 0x0F));
    EXPECT_TRUE (nandEccUncorrectable(NandEccKind::ForeseeLinear3, 0x7F));
}

TEST(NandEcc, EveryTablePartCarriesAnEccKind) {
    // A part added to the table without an ecc_kind would silently get
    // TwoBitPacked and, on a FORESEE-style part, fail good reads.
    struct { uint16_t id; NandEccKind kind; } expect[] = {
        {0xCD53, NandEccKind::ForeseeLinear3},   // FORESEE F35SQB004G
        {0xC852, NandEccKind::TwoBitPacked},     // GigaDevice GD5F2GQ5UE
        {0xC851, NandEccKind::TwoBitPacked},     // GigaDevice GD5F1GQ5UE
        {0xC2F5, NandEccKind::TwoBitPacked},     // Macronix MX35UF4G24AD-Z4I8
    };
    for (const auto& e : expect) {
        NandGeometry g{};
        ASSERT_TRUE(nandGeometryForId(e.id, &g)) << std::hex << e.id;
        EXPECT_EQ(g.ecc_kind, e.kind) << std::hex << e.id;
    }
    // The unknown-id fallback is the FORESEE-compatible legacy part.
    NandGeometry g{};
    EXPECT_FALSE(nandGeometryForId(0x0000, &g));
    EXPECT_EQ(g.ecc_kind, NandEccKind::ForeseeLinear3);
}
