//
//  Gzip.swift
//  TinkerRocketApp
//
//  Just enough gzip (RFC 1952) to read NOAA's broadcast navigation files
//  (#1600 phase 5). Foundation has no gzip reader, but the Compression
//  framework's COMPRESSION_ZLIB is raw DEFLATE — no zlib wrapper — which is
//  exactly a gzip member's body once its header is stepped over. NOAA's
//  header carries FNAME (the file's name, NUL-terminated), so the optional
//  fields are skipped by their flags rather than assumed absent.
//
//  The 8-byte trailer (CRC-32, length) is not checked: what is read out of
//  these files is a text header the RINEX parser validates field by field.
//  Android reads the same files through GZIPInputStream, with the same cap
//  and the same leniency about a body cut short.
//

import Compression
import Foundation

nonisolated enum Gzip {
    /// A wrong URL must not be able to fill memory; a day's broadcast file
    /// inflates to a few hundred KB.
    static let defaultMaxBytes = 4 << 20

    /// The first member's contents: at most `maxBytes` of them, and for a body
    /// cut short, what inflated before the cut (the header the parser wants
    /// comes first). Nil when `data` is not gzip, or nothing inflates.
    static func decompress(_ data: Data, maxBytes: Int = defaultMaxBytes) -> Data? {
        let b = [UInt8](data)
        // ID1 ID2, CM 8 (deflate), FLG, MTIME x4, XFL, OS.
        guard b.count > 10, b[0] == 0x1F, b[1] == 0x8B, b[2] == 8 else { return nil }
        let flg = b[3]
        guard flg & 0xE0 == 0 else { return nil }   // reserved bits: not a gzip we know
        var i = 10
        if flg & 0x04 != 0 {                           // FEXTRA: XLEN, then XLEN bytes
            guard i + 2 <= b.count else { return nil }
            let xlen = Int(b[i]) | (Int(b[i + 1]) << 8)
            i += 2 + xlen
        }
        for flag: UInt8 in [0x08, 0x10] where flg & flag != 0 {   // FNAME, FCOMMENT: NUL-terminated
            guard i < b.count, let nul = b[i...].firstIndex(of: 0) else { return nil }
            i = nul + 1
        }
        if flg & 0x02 != 0 { i += 2 }                  // FHCRC
        guard i < b.count else { return nil }
        // The trailer rides along: the decoder stops at the last block.
        return inflate(b[i...], maxBytes: maxBytes)
    }

    /// Raw DEFLATE through a compression_stream rather than
    /// compression_decode_buffer, which returns nothing at all for a stream
    /// cut short.
    private static func inflate(_ body: ArraySlice<UInt8>, maxBytes: Int) -> Data? {
        let stream = UnsafeMutablePointer<compression_stream>.allocate(capacity: 1)
        defer { stream.deallocate() }
        guard compression_stream_init(stream, COMPRESSION_STREAM_DECODE, COMPRESSION_ZLIB)
                == COMPRESSION_STATUS_OK else { return nil }
        defer { compression_stream_destroy(stream) }

        let chunk = 64 * 1024
        let buffer = UnsafeMutablePointer<UInt8>.allocate(capacity: chunk)
        defer { buffer.deallocate() }
        var out = Data()
        body.withUnsafeBufferPointer { src in
            guard let base = src.baseAddress else { return }
            stream.pointee.src_ptr = base
            stream.pointee.src_size = src.count
            while out.count < maxBytes {
                stream.pointee.dst_ptr = buffer
                stream.pointee.dst_size = chunk
                let status = compression_stream_process(
                    stream, Int32(COMPRESSION_STREAM_FINALIZE.rawValue))
                let produced = chunk - stream.pointee.dst_size
                out.append(buffer, count: min(produced, maxBytes - out.count))
                // END: done. ERROR: corrupt or cut short, keep what came out.
                // OK with room left over: the input ran out mid-stream.
                if status != COMPRESSION_STATUS_OK || stream.pointee.dst_size > 0 { break }
            }
        }
        return out.isEmpty ? nil : out
    }
}
