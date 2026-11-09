#pragma once

#include <stddef.h>
#include <stdint.h>

// The RunCam readiness probe (GET_DEVICE_INFO) without waiting on the camera.
//
// The probe used to be one blocking exchange on the flight loop: write the
// request, then uart_read_bytes() for up to RUNCAM_PROBE_READ_MS (60 ms). A
// camera that never answers ran every probe the full 60 ms, so the loop stopped
// for 60 ms every 250 ms on the pad. CENJARS 2026-11-01: neither GTV RunCam ever
// answered, and GTV Blue's log lost baro, mag and NonSensor frames in six
// 59-62 ms holes in the last 1.5 s before liftoff (the IMU FIFO kept its
// samples). #568 stops NEW probes once INFLIGHT is set, but a probe already
// waiting when the motor lit held the loop, launch detection included, for the
// rest of its 60 ms.
//
// Now the request goes out on one pass and every later pass reads whatever bytes
// have arrived, with a zero timeout, into a ReplyParser. The reply window and the
// probe cadence are unchanged; only the waiting moved off the loop.
//
// Pure so the parser and the per-pass decision are host-testable, in the house
// style of pwr_hold_policy.h.

namespace RunCamProbePolicy {

// RunCam Device Protocol CRC8: poly 0xD5, init 0x00, no reflection.
// (crc8({0xCC,0x01,0x01}) == 0xE7, matching RUNCAM_PWR_CMD.)
inline uint8_t crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5)
                               : (uint8_t)(crc << 1);
    }
    return crc;
}

constexpr uint8_t kHeader   = 0xCC;
constexpr size_t  kReplyLen = 5;  // header, protocol version, features lo, features hi, CRC8

// One GET_DEVICE_INFO reply, assembled from the pieces non-blocking reads
// return. Bytes before the 0xCC header are boot chatter and are skipped (the
// blocking read failed a reply that had any). The first five bytes from the
// header on decide it: Valid on a good CRC, Invalid otherwise, and nothing fed
// after that changes the verdict.
class ReplyParser
{
public:
    enum class State : uint8_t { Pending, Valid, Invalid };

    void reset()
    {
        n_ = 0;
        state_ = State::Pending;
    }

    State feed(const uint8_t* data, size_t len)
    {
        for (size_t i = 0; i < len && state_ == State::Pending; ++i)
        {
            if (n_ == 0 && data[i] != kHeader) continue;
            buf_[n_++] = data[i];
            if (n_ == kReplyLen)
                state_ = (crc8(buf_, kReplyLen - 1) == buf_[kReplyLen - 1])
                             ? State::Valid : State::Invalid;
        }
        return state_;
    }

    State state() const { return state_; }
    // Meaningful once Valid.
    uint8_t protocolVersion() const { return buf_[1]; }
    // Little-endian (bench reply cc 01 77 00 -> 0x0077).
    uint16_t features() const { return (uint16_t)buf_[2] | ((uint16_t)buf_[3] << 8); }
    const uint8_t* bytes() const { return buf_; }

private:
    uint8_t buf_[kReplyLen] = {0};
    size_t  n_ = 0;
    State   state_ = State::Pending;
};

// What one Probing pass does, on the pad. (In flight the caller starts the
// camera blind before asking: no probe, no read.)
enum class Step : uint8_t {
    Send,       // no probe out: write GET_DEVICE_INFO, read from the next pass on
    Wait,       // the reply window is open and nothing has decided it: read again next pass
    Alive,      // a valid reply: command START_RECORDING
    NextProbe,  // the window closed empty, or on a bad reply, before the boot
                // deadline: the next probe goes out at sent + the probe interval
    Silent,     // the same, at or after the boot deadline: the whole boot window was silent
};

// probe_out   — a request is out and its window has not been closed
// reply       — the parser's state for that request
// sent_ms     — when the request went out
// deadline_ms — the boot window's end (camera power-on + RUNCAM_BOOT_MAX_MS)
// window_ms   — the reply window (RUNCAM_PROBE_READ_MS)
//
// The deadline is judged only when a window closes, as the blocking probe did
// after its read: a request is always sent when one is due, and a reply inside
// its window counts even past the deadline. All times wrap with millis().
inline Step step(bool probe_out, ReplyParser::State reply, uint32_t sent_ms,
                 uint32_t now_ms, uint32_t deadline_ms, uint32_t window_ms)
{
    if (!probe_out) return Step::Send;
    if (reply == ReplyParser::State::Valid) return Step::Alive;
    if (reply == ReplyParser::State::Pending &&
        (uint32_t)(now_ms - sent_ms) < window_ms)
        return Step::Wait;
    return ((int32_t)(now_ms - deadline_ms) >= 0) ? Step::Silent : Step::NextProbe;
}

}  // namespace RunCamProbePolicy
