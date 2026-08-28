#pragma once

#include <stdint.h>

// Runtime NAND geometry, derived from the RDID that nandInit() already reads
// (#671). Pure and host-compilable (like bad_block_scan_policy.h) so the
// ID->geometry mapping is unit-testable — nothing else in tests_cpp can reach
// nandInit().
//
// Why runtime: the fleet now spans four parts that the SAME firmware image
// must drive correctly —
//
//   part                board       MID DID   page  pages/blk blocks  total
//   F35SQB004G          V8 bench   0xCD 0x53  4096      64     2048   512 MB
//   GD5F2GQ5UE          V9/V10     0xC8 0x52  2048      64     2048   256 MB
//   GD5F1GQ5UE          mini       0xC8 0x51  2048      64     1024   128 MB
//   MX35UF4G24AD-Z4I8   V7         0xC2 0xF5  4096      64     2048   512 MB
//
// Every ID is cross-validated in independent, silicon-tested drivers:
// the vendored spi_nand_flash devices table (nand_foresee.c — validated on
// the physical bench part in #492 — and nand_gigadevice.c) and the Linux
// kernel's drivers/mtd/nand/spi/{foresee,gigadevice,macronix}.c. All four
// parts are single-plane (no plane-select bit in the column address) and 64
// pages per block, so the SPI command sequences are identical across them —
// only the page size and block count differ.
//
// UNKNOWN IDs FALL BACK TO THE LEGACY GEOMETRY (the F35SQB004G numbers,
// i.e. exactly the behaviour every board had before #671). That is a
// deliberate fail-open: an unlisted 4 Gbit part keeps working, and the V8
// bench boards are byte-identical twice over — their ID is in the table AND
// the fallback equals their geometry. The cost is that an unlisted small-page
// part would still be driven with 4096-byte programs; the ERROR log the
// driver emits on an unknown ID is the tripwire for adding it here.
//
// Geometry is a pure function of the chip ID, so it is deterministic per
// physical chip across boots — which the persisted formats REQUIRE: page
// CRCs span the whole page, the flight index stores block numbers, and the
// bitmap blobs are length-checked. A geometry flap on the same die would
// CRC-fail every page and discard the bitmaps.

struct NandGeometry
{
    uint32_t page_size;       // main-array bytes per page (spare excluded)
    uint32_t pages_per_blk;
    uint32_t block_count;
    const char* name;         // for the boot log

    constexpr uint32_t blockSize() const { return page_size * pages_per_blk; }
    // 1-bit-per-block bad-block bitmap length (TR_LogToFlash's NVS blob).
    constexpr uint32_t bitmapBytes() const { return block_count / 8; }
};

// Compile-time maxima across supported parts — the ONLY legitimate uses are
// sizing static buffers and bitmap backing arrays. All arithmetic must use
// the runtime NandGeometry. (The old NAND_PAGE_SIZE / NAND_BLOCK_COUNT names
// were deliberately retired so every pre-#671 use site fails to compile and
// has to be consciously converted.)
constexpr uint32_t NAND_PAGE_SIZE_MAX   = 4096;
constexpr uint32_t NAND_BLOCK_COUNT_MAX = 2048;

// The legacy / unknown-ID fallback: the F35SQB004G numbers that were the
// compile-time geometry before #671.
constexpr NandGeometry NAND_GEOMETRY_LEGACY = {4096, 64, 2048,
                                               "legacy/unknown (F35SQB004G-compatible)"};

// chip_id is (MID << 8) | DID as nandInit() builds it — one DID byte only
// (that is all the driver captures; parts with 2-byte DIDs truncate — the
// Macronix full ID is 0xF5 0x03 — so entries here must be unambiguous in
// 16 bits, verified for these four).
// Returns true on a table hit; false leaves *out at the legacy fallback.
inline bool nandGeometryForId(uint16_t chip_id, NandGeometry* out)
{
    switch (chip_id)
    {
        case 0xCD53:  // FORESEE F35SQB004G — the V8 bench part (#492)
            *out = {4096, 64, 2048, "F35SQB004G (4Gbit)"};
            return true;
        case 0xC852:  // GigaDevice GD5F2GQ5UE — V9/V10 rocket computer
            *out = {2048, 64, 2048, "GD5F2GQ5UE (2Gbit)"};
            return true;
        case 0xC851:  // GigaDevice GD5F1GQ5UE — rocket computer mini
            *out = {2048, 64, 1024, "GD5F1GQ5UE (1Gbit)"};
            return true;
        case 0xC2F5:  // Macronix MX35UF4G24AD-Z4I8 — V7 rocket computer.
            // Geometry-identical to the legacy fallback; listed only to keep
            // the unknown-ID ERROR tripwire quiet on that board. Only the
            // -Z4I8 suffix (DID 0xF5) is single-plane; the plain
            // MX35UF4G24AD (DID 0xB5) is a 2-plane part this driver cannot
            // address — never add a 0xC2B5 entry.
            *out = {4096, 64, 2048, "MX35UF4G24AD-Z4I8 (4Gbit)"};
            return true;
        default:
            *out = NAND_GEOMETRY_LEGACY;
            return false;
    }
}

static_assert(NAND_GEOMETRY_LEGACY.page_size <= NAND_PAGE_SIZE_MAX, "max covers legacy");
static_assert(NAND_GEOMETRY_LEGACY.block_count <= NAND_BLOCK_COUNT_MAX, "max covers legacy");
