#ifndef BAD_BLOCK_SCAN_POLICY_H
#define BAD_BLOCK_SCAN_POLICY_H

#include <stdint.h>

// #511: gate for the boot-time full bad-block scan.
//
// scanBadBlocksAtBoot() probes all 2048 NAND blocks (3 reads each) and costs
// ~2.7 s — over half of the 5.05 s cmd-8 Power-On stall that leaves the OC
// deaf to BLE/LoRa. Factory bad-block markers only need to be harvested ONCE
// per physical chip: after that the persisted map plus runtime discovery
// (markBlockBad on read/program/erase failures, persisted at session close)
// keep the map current without ever re-walking the chip.
//
// Two traps this policy encodes:
//
// 1. The NVS "chip" key is written by nandInit() BEFORE the scan runs (it
//    exists to wipe a stale map on chip replacement), so "chip matches" does
//    NOT mean "a scan ever completed". The gate therefore keys on a separate
//    "scanned" marker (scanned_chip_id) written strictly AFTER
//    scanBadBlocksAtBoot() returns — a power cut mid-scan leaves it stale and
//    the next boot rescans.
//
// 2. A dead SPI bus (RDID reads 0x0000/0xFFFF) must skip the scan outright:
//    probing every block over a dead bus fails every read, marks EVERY block
//    bad, and poisons the persisted map for the real chip. (Before #511 this
//    was a live failure mode.) dead_bus also swallows the 0x0000 chip id, so
//    the scanned_chip_id==0 "no marker yet" sentinel can never falsely match.
//
// Pure decision logic (no NVS/SPI), host-tested in
// test_bad_block_scan_policy.cpp — same pattern as MramDirtyPolicy (#417).
class BadBlockScanPolicy
{
public:
    enum class Verdict : uint8_t
    {
        Scan,            // walk the chip (first boot, chip swap, or untrusted map)
        SkipTrustedMap,  // persisted map already reflects a completed scan of this chip
        SkipDeadBus,     // RDID dead bus — scanning would poison the map
    };

    // dead_bus:         RDID returned 0x0000 or 0xFFFF (see nandInit()).
    // map_blob_valid:   NVS "map" blob was present with the exact bitmap size FOR THE DETECTED CHIP (#671: finalized in nandInit() after RDID).
    // scanned_chip_id:  NVS "scanned" marker (0 = never written).
    // current_chip_id:  RDID (MID << 8) | DID read this boot.
    static Verdict bootScanVerdict(bool dead_bus,
                                   bool map_blob_valid,
                                   uint16_t scanned_chip_id,
                                   uint16_t current_chip_id)
    {
        if (dead_bus)                            return Verdict::SkipDeadBus;
        if (!map_blob_valid)                     return Verdict::Scan;
        if (scanned_chip_id != current_chip_id)  return Verdict::Scan;
        return Verdict::SkipTrustedMap;
    }
};

#endif  // BAD_BLOCK_SCAN_POLICY_H
