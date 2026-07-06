#pragma once

#include <stddef.h>
#include <stdint.h>

// Shared SOF + type + len + CRC16 message codec for framed UART links
// (radio daughterboard <-> host, #409).
//
// Wire format — byte-identical to TR_I2C_Interface::packMessage / the
// TR_I2S_Stream framing, so all three transports speak one codec family:
//
//   [0..3]  SOF        AA 55 AA 55
//   [4]     type       message type (link-local namespace)
//   [5]     len        payload byte count (0..MAX_PAYLOAD)
//   [6..]   payload
//   [+0,+1] CRC16      big-endian, over type + len + payload
//                      (Rob Tillaart CRC16 with library defaults —
//                       identical call to TR_I2C_Interface)
//
// Unlike I2C/I2S — where the driver hands us whole transactions — UART is
// a raw byte stream, so this header also provides MsgDeframer: a per-byte
// state machine that hunts for SOF, validates length + CRC, and
// resynchronizes on garbage. A corrupted frame is dropped (counted in
// stats) and the parser recovers at the next SOF; at LoRa frame rates a
// rare drop is recovered by the next telemetry/command frame.
//
// This header is intentionally free of ESP-IDF includes: tests_cpp
// compiles TR_MsgCodec.cpp directly on the host
// (tests_cpp/test_uart_link_codec.cpp).

namespace tr_msg
{

static constexpr uint8_t SOF0 = 0xAA;
static constexpr uint8_t SOF1 = 0x55;
static constexpr uint8_t SOF2 = 0xAA;
static constexpr uint8_t SOF3 = 0x55;

// The len field is one byte, so the codec's capacity is a superset of the
// I2C link's MAX_PAYLOAD (RocketComputerTypes.h) — same format, no shared
// upper bound needed.
static constexpr size_t MAX_PAYLOAD = 255;
static constexpr size_t FRAME_OVERHEAD = 4 + 1 + 1 + 2;  // SOF + type + len + CRC
static constexpr size_t MAX_FRAME = FRAME_OVERHEAD + MAX_PAYLOAD;

// Pack one message into frame_out. Returns false on bad args / capacity.
// Mirrors TR_I2C_Interface::packMessage byte-for-byte.
bool pack(uint8_t type,
          const uint8_t* payload,
          size_t len,
          uint8_t* frame_out,
          size_t frame_out_capacity,
          size_t& frame_len_out);

// Streaming deframer: feed() one byte at a time; when it returns true a
// complete CRC-valid frame is available via type()/payload()/payloadLen()
// until the next feed() call.
class MsgDeframer
{
public:
    struct Stats
    {
        uint32_t frames = 0;        // complete valid frames delivered
        uint32_t crc_fails = 0;     // frames dropped for CRC mismatch
        uint32_t resync_bytes = 0;  // bytes discarded hunting for SOF
    };

    bool feed(uint8_t b);

    uint8_t type() const { return buf_[0]; }
    const uint8_t* payload() const { return &buf_[2]; }
    size_t payloadLen() const { return payload_len_; }
    const Stats& stats() const { return stats_; }

    // Abandon any partial frame and return to SOF hunting (stats survive).
    void reset() { state_ = St::Hunt0; }

private:
    enum class St : uint8_t
    {
        Hunt0,    // waiting for SOF0
        Hunt1,    // got SOF0, waiting for SOF1
        Hunt2,
        Hunt3,
        Type,
        Len,
        Payload,
        Crc0,
        Crc1,
    };

    // A mismatched hunt byte may itself begin a new SOF sequence — route it
    // back into the hunt instead of blindly restarting at Hunt0.
    void rehunt(uint8_t b);

    St state_ = St::Hunt0;
    // type + len + payload, contiguous so the CRC check runs straight over it
    // (CRC covers exactly these bytes).
    uint8_t buf_[2 + MAX_PAYLOAD] = {};
    size_t payload_len_ = 0;
    size_t payload_got_ = 0;
    uint8_t crc_hi_ = 0;
    Stats stats_;
};

}  // namespace tr_msg
