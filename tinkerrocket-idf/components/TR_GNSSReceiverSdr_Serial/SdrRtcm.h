// SdrRtcm.h — the gnss-sdr-dev board's RTCM3 stream (pure, host-compilable)
//
// TinkerRocket's own receiver board (gnss repo, firmware/gnss-sdr-dev) sends a
// flight computer nothing but RTCM 3 frames, back to back, on its J401 pin 2
// at 2 Mbaud 8N1, and ignores what it is sent:
//   - MSM7 1077 (GPS), 1097 (Galileo E1-C) and 1127 (BeiDou B1C pilot, signal
//     ID 31), one set an epoch with the multiple-message bit on all but the
//     last: 100 epochs a second in its measurement mode, 10 in its fix mode;
//   - TinkerRocket's message 4001 (in the range RTCM keeps for manufacturers,
//     not registered): sub-type 1 after each epoch's MSMs (the GPS week, the
//     epoch's time of week, the clock's steps, the observables' count, the
//     correlator's sample count, then the ranges its checks left out and the
//     satellites it marked: up to 78 bytes), sub-type 2 after each fix (ECEF
//     position and velocity, clock bias and drift, PDOP, residual rms);
//   - its ephemerides, 1019, 1046 and 1042, each new one and again every 30 s.
// It sends nothing before its first fix (no time, no epochs).
//
// Rtcm3Framer pulls the frames whose CRC-24Q checks out of the byte stream,
// in any slicing. A frame that fails is given up one byte at a time, so a
// frame that began inside it (after bytes lost on the line) is still found.
// The rest reads the few fields of 4001 the flight computer looks at. Shared
// by the boot probe (gnss_detect.cpp) and the driver
// (TR_GNSSReceiverSdr_Serial.cpp); host-tested in tests_cpp/test_sdr_rtcm.cpp.
#ifndef SDRRTCM_H
#define SDRRTCM_H

#include <cstddef>
#include <cstdint>

namespace sdr_rtcm
{

static constexpr uint32_t kBaud        = 2000000;   // the board's `rx out uart 2000000`
static constexpr uint16_t kTrMessage   = 4001;      // TinkerRocket's message
static constexpr uint8_t  kTrEpoch     = 1;         // its sub-types
static constexpr uint8_t  kTrFix       = 2;
static constexpr uint8_t  kTrStatus    = 3;         // before the board has GPS time: once a second
static constexpr size_t   kMaxFrame    = 3 + 1023 + 3;   // preamble and length, the payload, the CRC

// The message number of a whole frame (the 12 bits after its 3-byte header);
// 0 when it is too short to have one.
uint16_t messageNumber(const uint8_t* frame, size_t len);

// 4001's sub-type (the 4 bits after the number); 0 for any other message.
uint8_t trSubType(const uint8_t* frame, size_t len);

// What the flight computer reads of a 4001 sub-type 1 (its layout:
// receiver/core/nav/rtcm_msm.c rtcm_tr_epoch_encode in the gnss repo). The
// fields come first; what follows the 48-bit sample (since 2026-11-02 the
// counts of ranges left out and of marked satellites, and each mark) is not
// read here, and a frame with or without it decodes the same.
struct TrEpoch
{
    uint16_t week = 0;     // GPS week
    uint32_t tow_ms = 0;   // the epoch's MSMs' time tag, GPS time of week
    uint8_t  mode = 0;     // 0: the fix's epochs; 1: the measurement mode's
    uint8_t  nobs = 0;     // observables in its MSMs
};
bool decodeTrEpoch(const uint8_t* frame, size_t len, TrEpoch& out);

// And of a sub-type 2.
struct TrFix
{
    uint16_t week = 0;
    uint32_t tow_ms = 0;
    bool     valid = false;
    uint8_t  nsat = 0;
};
bool decodeTrFix(const uint8_t* frame, size_t len, TrFix& out);

// And of a sub-type 3: the board's state while it has no GPS time yet (its
// first fix), when it sends nothing else, so a listener knows it is there.
struct TrStatus
{
    uint32_t fw = 0;          // its firmware's commit (8 hex digits)
    bool     fw_dirty = false;
    uint32_t uptime_s = 0;
    uint8_t  ntrack = 0;      // channels tracking, and with their PLL locked
    uint8_t  nlock = 0;
    bool     time_valid = false;
    bool     fix_valid = false;
};
bool decodeTrStatus(const uint8_t* frame, size_t len, TrStatus& out);

class Rtcm3Framer
{
public:
    struct Sink
    {
        virtual ~Sink() = default;
        // A frame from its 0xD3 through its CRC, whose CRC checks. `after`:
        // the bytes fed since its last byte (0 when it ended on the last one).
        virtual void onFrame(const uint8_t* frame, size_t len, size_t after) = 0;
    };

    void reset() { n_ = 0; }
    void feed(const uint8_t* data, size_t n, Sink& sink);

    uint32_t frames() const   { return frames_; }
    uint32_t crcFails() const { return crc_fails_; }   // whole frames whose CRC did not check
    uint32_t skipped() const  { return skipped_; }     // bytes in no frame (none in a clean stream)

private:
    void parse(Sink& sink);
    void drop(size_t k);

    // A frame at most plus room to take more in before the next parse.
    static constexpr size_t kBuf = 2 * kMaxFrame;
    uint8_t  buf_[kBuf] = {};
    size_t   n_ = 0;
    size_t   fed_after_ = 0;   // bytes of this feed() still to come into buf_
    uint32_t frames_ = 0;
    uint32_t crc_fails_ = 0;
    uint32_t skipped_ = 0;
};

}  // namespace sdr_rtcm

#endif
