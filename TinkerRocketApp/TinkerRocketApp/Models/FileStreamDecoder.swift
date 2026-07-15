import Foundation

// #526: decoder for the in-band record framing carried on the L2CAP CoC stream.
// The byte-identical twin of the firmware encoder + reference decoder
// (components/TR_BLE_To_APP/FileStreamFraming.h). iOS delivers the channel as a
// pure byte stream with NO record boundaries, so this reassembles records split
// at arbitrary offsets — exactly what the C++ reference test proves by feeding one
// byte at a time.
//
//   [type u8][len u32 LE][payload]
//   0x01 BEGIN : [ver u8][status u8][size_hint u32 LE][name_len u8][name]
//   0x02 DATA  : the file bytes (streamed out via onData, never buffered whole)
//   0x03 END   : [bytes u32 LE][crc32 u32 LE][status u8]
//
// Completion is the END callback; nothing else. A malformed stream latches
// failure (a byte stream cannot be resynchronised after a desync).
//
// nonisolated: the project defaults to MainActor isolation, but this is pure
// logic driven from L2CAPFileReceiver's background stream thread. Without this it
// would be MainActor-isolated and its deinit would hop to the main actor (which
// crashes in the back-deploy shim, and would be wrong regardless — it dies on a
// background thread).
nonisolated final class FileStreamDecoder {
    struct Begin { let ver: UInt8; let status: UInt8; let sizeHint: UInt32; let name: String }
    struct End { let bytes: UInt32; let crc: UInt32; let status: UInt8 }

    // Must match FileStreamFraming.h.
    static let typeBegin: UInt8 = 0x01
    static let typeData: UInt8  = 0x02
    static let typeEnd: UInt8   = 0x03
    static let endPayloadSize = 9
    static let maxBufferedPayload = 7 + 255
    static let maxDataPayload: UInt32 = 8192

    var onBegin: ((Begin) -> Void)?
    var onData: ((Data) -> Void)?     // may fire several times per DATA record
    var onEnd: ((End) -> Void)?
    var onError: ((String) -> Void)?

    private(set) var failed = false

    private enum State { case type, len, bufferedPayload, dataPayload, failed }
    private var state: State = .type
    private var curType: UInt8 = 0
    private var curLen: UInt32 = 0
    private var lenBuf = [UInt8]()          // accumulates the 4 length bytes
    private var buf = [UInt8]()             // BEGIN/END payload (bounded)
    private var dataSeen: UInt32 = 0

    func feed(_ data: Data) {
        // Memory-safe indexing (no unsafe pointers): headers byte-by-byte, DATA in
        // bulk via subdata so an 8.5 MB file is not millions of one-byte callbacks.
        // `base` handles a Data whose startIndex is non-zero (a slice).
        let base = data.startIndex
        let n = data.count
        var i = 0
        while i < n && state != .failed {
            switch state {
            case .type:
                curType = data[base + i]; i += 1
                if curType != Self.typeBegin && curType != Self.typeData && curType != Self.typeEnd {
                    fail("unknown record type \(curType)"); return
                }
                lenBuf.removeAll(keepingCapacity: true)
                state = .len

            case .len:
                lenBuf.append(data[base + i]); i += 1
                if lenBuf.count == 4 {
                    curLen = UInt32(lenBuf[0]) | (UInt32(lenBuf[1]) << 8)
                          | (UInt32(lenBuf[2]) << 16) | (UInt32(lenBuf[3]) << 24)
                    beginPayload()
                }

            case .bufferedPayload:
                let take = min(Int(curLen) - buf.count, n - i)
                buf.append(contentsOf: data.subdata(in: (base + i)..<(base + i + take)))
                i += take
                if UInt32(buf.count) == curLen { emitBuffered() }

            case .dataPayload:
                let take = min(Int(curLen - dataSeen), n - i)
                if take > 0 { onData?(data.subdata(in: (base + i)..<(base + i + take))) }
                dataSeen += UInt32(take)
                i += take
                if dataSeen == curLen { state = .type }

            case .failed:
                return
            }
        }
    }

    private func beginPayload() {
        if curType == Self.typeData {
            if curLen > Self.maxDataPayload { fail("DATA length too large"); return }
            dataSeen = 0
            state = (curLen == 0) ? .type : .dataPayload
            return
        }
        if curType == Self.typeEnd && curLen != UInt32(Self.endPayloadSize) {
            fail("END length wrong"); return
        }
        if curLen > UInt32(Self.maxBufferedPayload) { fail("buffered record too large"); return }
        if curType == Self.typeBegin && curLen < 7 { fail("BEGIN too short"); return }
        buf.removeAll(keepingCapacity: true)
        state = .bufferedPayload
    }

    private func emitBuffered() {
        if curType == Self.typeBegin {
            let ver = buf[0]
            let status = buf[1]
            let sizeHint = UInt32(buf[2]) | (UInt32(buf[3]) << 8)
                         | (UInt32(buf[4]) << 16) | (UInt32(buf[5]) << 24)
            let nameLen = Int(buf[6])
            if 7 + nameLen != Int(curLen) { fail("BEGIN name_len mismatch"); return }
            let name = String(bytes: buf[7..<(7 + nameLen)], encoding: .utf8) ?? ""
            onBegin?(Begin(ver: ver, status: status, sizeHint: sizeHint, name: name))
        } else {  // END
            let bytes = UInt32(buf[0]) | (UInt32(buf[1]) << 8)
                      | (UInt32(buf[2]) << 16) | (UInt32(buf[3]) << 24)
            let crc = UInt32(buf[4]) | (UInt32(buf[5]) << 8)
                    | (UInt32(buf[6]) << 16) | (UInt32(buf[7]) << 24)
            onEnd?(End(bytes: bytes, crc: crc, status: buf[8]))
        }
        state = .type
    }

    private func fail(_ reason: String) {
        state = .failed
        failed = true
        onError?(reason)
    }
}
