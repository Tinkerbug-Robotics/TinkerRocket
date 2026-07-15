import Foundation

// #526: CRC-32 (ISO 3309 / zlib / PNG) — the byte-identical twin of the firmware's
// tr_crc32 (components/TR_BLE_To_APP/Crc32.h). A file downloaded over the L2CAP
// channel completes only if the CRC the app computes over the received DATA bytes
// matches the CRC the firmware sends in the END record. Both are pinned to the
// canonical check value crc32("123456789") == 0xCBF43926 in their test suites, so
// the two implementations can never silently drift apart.
// nonisolated: called from L2CAPFileReceiver's background stream thread, not the
// main actor (which this project isolates to by default).
nonisolated enum Crc32 {
    static let initial: UInt32 = 0xFFFF_FFFF

    /// Fold `data` into the running CRC. Chain across arbitrarily-split buffers —
    /// the result depends only on the concatenated byte sequence.
    static func update(_ crc: UInt32, _ data: Data) -> UInt32 {
        var c = crc
        for byte in data {
            c ^= UInt32(byte)
            for _ in 0..<8 {
                let mask: UInt32 = (c & 1) != 0 ? 0xEDB8_8320 : 0
                c = (c >> 1) ^ mask
            }
        }
        return c
    }

    static func finalize(_ crc: UInt32) -> UInt32 { crc ^ 0xFFFF_FFFF }

    static func compute(_ data: Data) -> UInt32 { finalize(update(initial, data)) }
}
