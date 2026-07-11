//
//  StorageStats.swift
//  TinkerRocketApp
//
//  Decoded flash-space stats for the storage bar. Two frames arrive on the
//  file_ops characteristic, each behind a discriminator byte:
//    0xCC  RocketStorageStats        (rocket NAND flight log — direct rocket link)
//    0xCD  BaseStationStorageStats   (base station's own filesystem — BS link)
//  Wire layouts mirror RocketStorageStatsData / BaseStationStorageStatsData in
//  RocketComputerTypes.h — keep both in sync.
//

import Foundation

/// Rocket NAND flight-log usage, in 256 KB blocks (wire = 15 bytes, LE packed):
///   [0..1] u16 flight_region_blocks  [2..3] u16 used  [4..5] u16 free
///   [6..7] u16 bad  [8..9] u16 system  [10..11] u16 flight_count
///   [12..13] u16 block_size_kb  [14] u8 flags(bit0=initialized)
struct RocketStorageStats: Equatable {
    let flightRegionBlocks: Int
    let usedBlocks: Int
    let freeBlocks: Int
    let badBlocks: Int
    let systemBlocks: Int
    let flightCount: Int
    let blockSizeKB: Int
    let initialized: Bool

    private var blockBytes: Int { blockSizeKB * 1024 }
    /// Full managed chip = flight region + fixed system overhead.
    var totalBytes: Int { (flightRegionBlocks + systemBlocks) * blockBytes }
    var usedBytes: Int { usedBlocks * blockBytes }
    /// Reserved = bad blocks + fixed system overhead (LFS + metadata).
    var reservedBytes: Int { (badBlocks + systemBlocks) * blockBytes }
    var freeBytes: Int { freeBlocks * blockBytes }

    /// Decode the payload *after* the 0xCC discriminator byte.
    static func decode(_ bytes: [UInt8]) -> RocketStorageStats? {
        guard bytes.count >= 15 else { return nil }
        func u16(_ o: Int) -> Int {
            Int(bytes.withUnsafeBufferPointer {
                UnsafeRawBufferPointer($0).loadUnaligned(fromByteOffset: o, as: UInt16.self)
            })
        }
        return RocketStorageStats(
            flightRegionBlocks: u16(0), usedBlocks: u16(2), freeBlocks: u16(4),
            badBlocks: u16(6), systemBlocks: u16(8), flightCount: u16(10),
            blockSizeKB: u16(12), initialized: (bytes[14] & 0x01) != 0)
    }
}

/// Base-station filesystem usage, in bytes (wire = 26 bytes, LE packed):
///   [0..7] u64 total  [8..15] u64 used  [16..23] u64 free
///   [24] u8 backend(0=SPIFFS,1=SD,2=ext-NAND)  [25] u8 flags(bit0=mounted)
struct BaseStationStorageStats: Equatable {
    let totalBytes: Int
    let usedBytes: Int
    let freeBytes: Int
    let backend: Int
    let mounted: Bool

    /// FS overhead the data accounting doesn't attribute to used or free.
    var reservedBytes: Int { max(0, totalBytes - usedBytes - freeBytes) }
    var backendName: String {
        switch backend {
        case 0:  return "Internal flash"
        case 2:  return "External NAND"
        default: return "SD card"
        }
    }

    /// Decode the payload *after* the 0xCD discriminator byte.
    static func decode(_ bytes: [UInt8]) -> BaseStationStorageStats? {
        guard bytes.count >= 26 else { return nil }
        func u64(_ o: Int) -> Int {
            Int(truncatingIfNeeded: bytes.withUnsafeBufferPointer {
                UnsafeRawBufferPointer($0).loadUnaligned(fromByteOffset: o, as: UInt64.self)
            })
        }
        return BaseStationStorageStats(
            totalBytes: u64(0), usedBytes: u64(8), freeBytes: u64(16),
            backend: Int(bytes[24]), mounted: (bytes[25] & 0x01) != 0)
    }
}
