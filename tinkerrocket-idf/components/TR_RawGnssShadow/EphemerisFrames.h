// EphemerisFrames.h — the broadcast ephemerides the raw-GNSS filter holds, as
// the receiver sent them, to write to the log again after launch
//
// The LC86G sends one broadcast ephemeris a second, round-robin over its
// satellites ($PAIR436,1), and the log keeps only about a second before
// ignition: a flight log of a short flight holds a handful, and a replay of
// its raw measurements has to fetch the rest. So the latest payload of each
// message and satellite the filter is handed (RTCM 1019 GPS, 1042 BeiDou,
// 1046 Galileo; any other kept the same way) is kept here, and the pad
// context (RawGnssShadow::takePadContext) writes each back as a whole RTCM3
// frame -- the same bytes the receiver framed -- in PAD_CONTEXT_MSG records
// (PCX_EPH).
// Pure C++ (tests_cpp/test_raw_gnss_shadow.cpp).
#ifndef EPHEMERIS_FRAMES_H
#define EPHEMERIS_FRAMES_H

#include <cstddef>
#include <cstdint>

namespace gnss_raw
{

class EphemerisFrames
{
public:
    static constexpr int    kMax = 64;            // satellites kept; the oldest goes when full
    static constexpr size_t kMaxPayload = 72;     // the driver's kEphMaxPayload (1042, the largest, is 64)
    static constexpr size_t kFrameOverhead = 6;   // 0xD3, 6 reserved bits + 10 of length, CRC-24Q

    // A broadcast ephemeris message's payload (no framing): kept as its
    // message's latest for its satellite. False if too short or too long.
    bool add(const uint8_t* payload, size_t len);
    int count() const { return n_; }
    // The i-th (0..count()-1) as a whole RTCM3 frame: its length, or 0 if
    // there is no i-th or it does not fit cap.
    size_t frame(int i, uint8_t* out, size_t cap) const;
    // RTCM3's CRC-24Q over len bytes (the frame from its 0xD3 to its payload's end).
    static uint32_t crc24q(const uint8_t* data, size_t len);

private:
    struct Item
    {
        uint16_t msg;      // the RTCM message number
        uint8_t  sat;      // its satellite ID field
        uint8_t  len;
        uint32_t seq;      // when it was added (the oldest goes first)
        uint8_t  bytes[kMaxPayload];
    };
    Item     items_[kMax] = {};
    int      n_ = 0;
    uint32_t seq_ = 0;
};

}  // namespace gnss_raw

#endif  // EPHEMERIS_FRAMES_H
