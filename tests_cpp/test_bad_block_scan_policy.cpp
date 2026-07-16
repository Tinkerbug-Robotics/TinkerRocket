// #511: BadBlockScanPolicy — the boot-time full bad-block scan (2048 blocks,
// ~2.7 s of the cmd-8 Power-On stall) must run only until one scan has
// completed for the current chip. The gate keys on the NVS "scanned" marker,
// written strictly AFTER scanBadBlocksAtBoot() — NOT on the "chip" key, which
// nandInit() writes BEFORE the scan (it exists for chip-swap map wipes), so
// "chip matches" never proves a scan completed. A dead RDID bus (0x0000 /
// 0xFFFF) must skip the scan outright: 2048 failing reads would mark every
// block bad and poison the persisted map. These tests pin the decision table.

#include <gtest/gtest.h>

#include "bad_block_scan_policy.h"

using Verdict = BadBlockScanPolicy::Verdict;

// Factory-fresh device: no NVS namespace, so no map blob and no marker.
TEST(BadBlockScanPolicy, FirstBoot_NoNamespace_Scans)
{
    EXPECT_EQ(BadBlockScanPolicy::bootScanVerdict(
                  /*dead_bus=*/false, /*map_blob_valid=*/false,
                  /*scanned_chip_id=*/0, /*current_chip_id=*/0xCD53),
              Verdict::Scan);
}

// First boot after the #511 firmware update: the map blob and "chip" key
// exist from earlier firmware, but the "scanned" marker was never written.
// One more full scan runs, then the marker gates it off forever.
TEST(BadBlockScanPolicy, PostOta_MapPresentNoMarker_ScansOnce)
{
    EXPECT_EQ(BadBlockScanPolicy::bootScanVerdict(false, true, 0, 0xCD53),
              Verdict::Scan);
}

// Steady state: marker matches the live chip — skip the 2.7 s walk.
TEST(BadBlockScanPolicy, NormalBoot_TrustedMap_Skips)
{
    EXPECT_EQ(BadBlockScanPolicy::bootScanVerdict(false, true, 0xCD53, 0xCD53),
              Verdict::SkipTrustedMap);
}

// NAND replaced: nandInit() wipes the map, but the stale marker still names
// the old chip, so the new chip gets its mandatory factory-marker scan.
TEST(BadBlockScanPolicy, ChipSwapped_Rescans)
{
    EXPECT_EQ(BadBlockScanPolicy::bootScanVerdict(false, true, 0xAAAA, 0xCD53),
              Verdict::Scan);
}

// Truncated/corrupt NVS blob: the in-RAM map was zeroed, so the marker alone
// (even matching) must not be trusted — rescan to rebuild factory knowledge.
TEST(BadBlockScanPolicy, CorruptBlob_MarkerMatches_StillScans)
{
    EXPECT_EQ(BadBlockScanPolicy::bootScanVerdict(false, false, 0xCD53, 0xCD53),
              Verdict::Scan);
}

// Power cut mid-scan: the marker is written only after a completed scan, so
// the next boot sees it absent (0) and rescans. Same inputs as PostOta.
TEST(BadBlockScanPolicy, MidScanPowerLoss_MarkerAbsent_Rescans)
{
    EXPECT_EQ(BadBlockScanPolicy::bootScanVerdict(false, true, 0, 0xCD53),
              Verdict::Scan);
}

// Dead bus, RDID all-zero. Trap pinned here: scanned==0 (never written) would
// arithmetically "match" chip==0x0000 — dead_bus must win before that compare.
TEST(BadBlockScanPolicy, DeadBus0000_SkipsEvenWithZeroMarker)
{
    EXPECT_EQ(BadBlockScanPolicy::bootScanVerdict(true, true, 0, 0x0000),
              Verdict::SkipDeadBus);
    EXPECT_EQ(BadBlockScanPolicy::bootScanVerdict(true, false, 0, 0x0000),
              Verdict::SkipDeadBus);
}

// Dead bus, RDID all-ones — including a previously-healthy map for a real
// chip. Never scan (would mark all 2048 blocks bad and poison that map).
TEST(BadBlockScanPolicy, DeadBusFFFF_SkipsRegardlessOfMap)
{
    EXPECT_EQ(BadBlockScanPolicy::bootScanVerdict(true, true, 0xFFFF, 0xFFFF),
              Verdict::SkipDeadBus);
    EXPECT_EQ(BadBlockScanPolicy::bootScanVerdict(true, true, 0xCD53, 0xFFFF),
              Verdict::SkipDeadBus);
}
