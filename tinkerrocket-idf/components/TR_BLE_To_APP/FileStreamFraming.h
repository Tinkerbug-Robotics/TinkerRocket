#pragma once

#include <stddef.h>
#include <stdint.h>

// ============================================================================
// #526: in-band framing for a file download over an L2CAP CoC channel.
//
// iOS hands the app a CBL2CAPChannel as an InputStream/OutputStream BYTE STREAM
// — SDU boundaries are invisible above the transport — so the payload must frame
// itself. Three record types, each a length-prefixed TLV:
//
//     [type u8][len u32 LE][payload len bytes]
//
//   0x01 BEGIN : [ver u8][status u8][size_hint u32 LE][name_len u8][name...]
//                status: 0 OK, 1 not found, 2 refused (INFLIGHT), 3 read error
//                size_hint is the flight index's final_bytes — PROGRESS ONLY.
//                The frame packer legitimately emits fewer bytes than final_bytes
//                for crash-recovered flights (page-rounded + 0xFF pad), so it is
//                NEVER a completion signal.
//   0x02 DATA  : the packed AA55AA55 frame bytes, exactly what sendFileChunk()
//                would have carried over GATT. Concatenating every DATA payload
//                reproduces the file BYTE-FOR-BYTE — that is what makes the
//                cmd-4 vs cmd-44 sha256 A/B comparison a real oracle.
//   0x03 END   : [bytes u32 LE][crc32 u32 LE][status u8]
//                Completion is END with status 0, a byte count matching what was
//                received, AND a matching CRC32. NOTHING ELSE completes a
//                download: no EOF flag, no >0.98 heuristic, no "stall means done".
//                A stream that ends WITHOUT a good END is a FAILURE.
//
// The firmware is the ENCODER (it streams). iOS is the DECODER. The C++ decoder
// below is the reference implementation: it pins the wire format, proves the
// encoder round-trips, and — critically — proves records survive being split at
// arbitrary byte boundaries (test_file_stream_framing.cpp feeds it one byte at a
// time). The Swift decoder in the app is a line-by-line twin of it.
// ============================================================================

namespace tr_filestream
{

static constexpr uint8_t kTypeBegin = 0x01;
static constexpr uint8_t kTypeData  = 0x02;
static constexpr uint8_t kTypeEnd   = 0x03;

static constexpr uint8_t kProtoVersion = 1;

// BEGIN status codes.
static constexpr uint8_t kBeginOk        = 0;
static constexpr uint8_t kBeginNotFound  = 1;
static constexpr uint8_t kBeginRefused   = 2;  // #383 INFLIGHT
static constexpr uint8_t kBeginReadError = 3;

// END status codes.
static constexpr uint8_t kEndOk    = 0;
static constexpr uint8_t kEndError = 1;

static constexpr size_t kRecordHeaderSize = 5;   // type(1) + len(4)
static constexpr size_t kEndPayloadSize   = 9;   // bytes(4) + crc(4) + status(1)

// A BEGIN payload is at most 7 fixed bytes + a 255-byte name. END is 9. This
// bounds the decoder's buffered records; DATA is streamed, never buffered.
static constexpr size_t kMaxBufferedPayload = 7 + 255;

// Reject an absurd DATA length as a desync rather than waiting forever for bytes
// that will never come. SDUs are ~1 KB; this is generous headroom.
static constexpr uint32_t kMaxDataPayload = 8192;

// ---------------------------------------------------------------------------
// Little-endian helpers.
// ---------------------------------------------------------------------------
inline void putU32(uint8_t* p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}
inline uint32_t getU32(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// ---------------------------------------------------------------------------
// Encoder (firmware side). Each returns bytes written, or 0 if out_cap is too
// small (never a partial write).
// ---------------------------------------------------------------------------
inline size_t encodeBegin(uint8_t* out, size_t out_cap, uint8_t ver, uint8_t status,
                          uint32_t size_hint, const char* name, uint8_t name_len)
{
    const size_t payload = 1 /*ver*/ + 1 /*status*/ + 4 /*size_hint*/ + 1 /*name_len*/ + name_len;
    const size_t total = kRecordHeaderSize + payload;
    if (out_cap < total) return 0;

    out[0] = kTypeBegin;
    putU32(out + 1, (uint32_t)payload);
    size_t o = kRecordHeaderSize;
    out[o++] = ver;
    out[o++] = status;
    putU32(out + o, size_hint); o += 4;
    out[o++] = name_len;
    for (uint8_t i = 0; i < name_len; ++i) out[o++] = (uint8_t)name[i];
    return o;
}

// Writes ONLY the DATA record header. The caller appends `payload_len` bytes
// after it (kept separate so the firmware never copies the frame bytes twice —
// it builds the SDU as header + a view onto the packed frame buffer).
inline size_t encodeDataHeader(uint8_t* out, size_t out_cap, uint32_t payload_len)
{
    if (out_cap < kRecordHeaderSize) return 0;
    out[0] = kTypeData;
    putU32(out + 1, payload_len);
    return kRecordHeaderSize;
}

inline size_t encodeEnd(uint8_t* out, size_t out_cap, uint32_t bytes, uint32_t crc32,
                        uint8_t status)
{
    const size_t total = kRecordHeaderSize + kEndPayloadSize;
    if (out_cap < total) return 0;
    out[0] = kTypeEnd;
    putU32(out + 1, (uint32_t)kEndPayloadSize);
    putU32(out + 5, bytes);
    putU32(out + 9, crc32);
    out[13] = status;
    return total;
}

// ---------------------------------------------------------------------------
// Decoder (reference for the Swift twin; the host tests drive this).
//
// Push bytes in with feed(); the sink is called as records complete. DATA is
// delivered incrementally (onData may fire several times for one record) so no
// large buffer is ever held. Any malformed input latches an error: once failed,
// the decoder stays failed (a byte stream cannot be resynchronised after a
// desync — the only sound recovery is to tear the channel down and reopen).
// ---------------------------------------------------------------------------
struct Sink
{
    virtual ~Sink() {}
    virtual void onBegin(uint8_t ver, uint8_t status, uint32_t size_hint,
                         const char* name, uint8_t name_len) = 0;
    virtual void onData(const uint8_t* data, size_t len) = 0;
    virtual void onEnd(uint32_t bytes, uint32_t crc32, uint8_t status) = 0;
    virtual void onError(const char* reason) = 0;
};

class Decoder
{
public:
    explicit Decoder(Sink* sink) : sink_(sink) {}

    bool failed() const { return state_ == State::Failed; }

    void feed(const uint8_t* data, size_t len)
    {
        size_t i = 0;
        while (i < len && state_ != State::Failed)
        {
            switch (state_)
            {
            case State::Type:
                cur_type_ = data[i++];
                if (cur_type_ != kTypeBegin && cur_type_ != kTypeData &&
                    cur_type_ != kTypeEnd)
                {
                    fail("unknown record type");
                    break;
                }
                hdr_have_ = 0;
                state_ = State::Len;
                break;

            case State::Len:
                hdr_buf_[hdr_have_++] = data[i++];
                if (hdr_have_ == 4)
                {
                    cur_len_ = getU32(hdr_buf_);
                    if (!beginPayload()) { /* fail() already latched */ }
                }
                break;

            case State::BufferedPayload:
            {
                const size_t want = cur_len_ - buf_have_;
                const size_t avail = len - i;
                const size_t take = (avail < want) ? avail : want;
                for (size_t k = 0; k < take; ++k) buf_[buf_have_ + k] = data[i + k];
                buf_have_ += take;
                i += take;
                if (buf_have_ == cur_len_) emitBuffered();
                break;
            }

            case State::DataPayload:
            {
                const size_t want = cur_len_ - data_seen_;
                const size_t avail = len - i;
                const size_t take = (avail < want) ? avail : want;
                if (take > 0) sink_->onData(data + i, take);
                data_seen_ += take;
                i += take;
                if (data_seen_ == cur_len_) state_ = State::Type;
                break;
            }

            case State::Failed:
                return;
            }
        }
    }

private:
    enum class State { Type, Len, BufferedPayload, DataPayload, Failed };

    // Called once the 5-byte record header is complete; sets up payload reading.
    // Returns false (and latches failure) on a malformed length.
    bool beginPayload()
    {
        if (cur_type_ == kTypeData)
        {
            if (cur_len_ > kMaxDataPayload) { fail("DATA length too large"); return false; }
            data_seen_ = 0;
            // A zero-length DATA record is legal (no-op); complete it immediately.
            state_ = (cur_len_ == 0) ? State::Type : State::DataPayload;
            return true;
        }

        // BEGIN / END are buffered whole.
        if (cur_type_ == kTypeEnd && cur_len_ != kEndPayloadSize)
        {
            fail("END length wrong"); return false;
        }
        if (cur_len_ > kMaxBufferedPayload) { fail("buffered record too large"); return false; }
        // A BEGIN cannot be shorter than its fixed fields; END is exactly 9 (checked
        // above). So a buffered record always has payload to read — no zero-len case.
        if (cur_type_ == kTypeBegin && cur_len_ < 7) { fail("BEGIN too short"); return false; }
        buf_have_ = 0;
        state_ = State::BufferedPayload;
        return true;
    }

    void emitBuffered()
    {
        if (cur_type_ == kTypeBegin)
        {
            const uint8_t ver = buf_[0];
            const uint8_t status = buf_[1];
            const uint32_t size_hint = getU32(buf_ + 2);
            const uint8_t name_len = buf_[6];
            if ((size_t)7 + name_len != cur_len_) { fail("BEGIN name_len mismatch"); return; }
            sink_->onBegin(ver, status, size_hint, (const char*)(buf_ + 7), name_len);
        }
        else  // kTypeEnd
        {
            const uint32_t bytes = getU32(buf_);
            const uint32_t crc = getU32(buf_ + 4);
            const uint8_t status = buf_[8];
            sink_->onEnd(bytes, crc, status);
        }
        state_ = State::Type;
    }

    void fail(const char* reason)
    {
        state_ = State::Failed;
        sink_->onError(reason);
    }

    Sink* sink_;
    State state_ = State::Type;

    uint8_t cur_type_ = 0;
    uint32_t cur_len_ = 0;

    uint8_t hdr_buf_[4] = {0};
    size_t hdr_have_ = 0;

    uint8_t buf_[kMaxBufferedPayload] = {0};
    size_t buf_have_ = 0;

    uint32_t data_seen_ = 0;
};

}  // namespace tr_filestream
