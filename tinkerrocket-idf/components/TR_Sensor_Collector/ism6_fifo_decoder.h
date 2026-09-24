#pragma once

// #1485: turns ISM6HG256X FIFO words back into IMU samples.
//
// Reading one sample per DRDY edge loses every sample the poll task is late
// for: DRDY is latched, so the next sample overwrites the unread one and
// leaves no trace. On the mini's ESP32-S3 that cost ~25 % of the 3,840 Hz
// stream. In FIFO mode the chip keeps every sample until it is read, and the
// task drains a burst at a time.
//
// The FIFO hands out 7-byte words: a tag byte, then three little-endian int16
// axes. The tag byte holds the sensor code in bits 7-3 and a 2-bit time-slot
// counter in bits 2-1. With the gyro, the low-g and the high-g accelerometer
// batched at one rate, every slot carries one word of each, and all three
// share the slot counter. This groups them back into samples.
//
// Header-only and IDF-free, so the host tests pin it.

#include <stddef.h>
#include <stdint.h>

struct Ism6FifoSample
{
    int16_t g[3];   // gyro
    int16_t lg[3];  // low-g accelerometer
    int16_t hg[3];  // high-g accelerometer
};

class Ism6FifoDecoder
{
public:
    static constexpr size_t WORD_BYTES = 7;

    // Raw TAG_SENSOR codes, as the silicon writes them. ST's driver enum
    // renumbers some of these (its TIMESTAMP_TAG is 4, the hardware code is
    // 3), so the decoder works on the raw values.
    static constexpr uint8_t TAG_GY = 0x01;
    static constexpr uint8_t TAG_XL = 0x02;
    static constexpr uint8_t TAG_TIMESTAMP = 0x03;
    static constexpr uint8_t TAG_TEMPERATURE = 0x04;
    static constexpr uint8_t TAG_CFG_CHANGE = 0x05;
    static constexpr uint8_t TAG_XL_HG = 0x1D;

    static uint8_t tagSensor(uint8_t tag_byte) { return (uint8_t)(tag_byte >> 3); }
    static uint8_t tagCount(uint8_t tag_byte) { return (uint8_t)((tag_byte >> 1) & 0x3u); }

    // Counters, cumulative since reset().
    uint32_t samples = 0;           // complete slots handed to the sink
    uint32_t incomplete_slots = 0;  // slots that ended without all three words
    uint32_t counter_gaps = 0;      // slot-counter jumps: slots lost (mod 4)
    uint32_t cfg_changes = 0;       // CFG_CHANGE words (a rate change landed)
    uint32_t other_words = 0;       // timestamp, temperature, anything else

    void reset() { *this = Ism6FifoDecoder{}; }

    // A slot has some of its words but not all three: the rest are still in
    // the FIFO (or were lost), so the newest sample read is not the newest the
    // chip produced.
    bool hasPartialSlot() const { return have_ != 0; }

    // Decode n_words whole words. sink(const Ism6FifoSample&) runs once per
    // complete slot, in FIFO order. A slot still missing a word when the burst
    // ends is kept and finished by the next burst.
    template <typename Sink>
    void feed(const uint8_t* words, size_t n_words, Sink&& sink)
    {
        for (size_t i = 0; i < n_words; ++i)
        {
            const uint8_t* w = words + i * WORD_BYTES;
            const uint8_t sensor = tagSensor(w[0]);

            int16_t* dst = nullptr;
            uint8_t bit = 0;
            switch (sensor)
            {
                case TAG_GY:    dst = cur_.g;  bit = HAVE_G;  break;
                case TAG_XL:    dst = cur_.lg; bit = HAVE_LG; break;
                case TAG_XL_HG: dst = cur_.hg; bit = HAVE_HG; break;
                case TAG_CFG_CHANGE: cfg_changes++; continue;
                default: other_words++; continue;
            }

            const uint8_t cnt = tagCount(w[0]);
            if (slot_cnt_ < 0 || cnt != (uint8_t)slot_cnt_ || (have_ & bit) != 0)
            {
                // A new slot. The one before it never got all three words.
                if (have_ != 0) incomplete_slots++;
                if (slot_cnt_ >= 0)
                {
                    const uint8_t step = (uint8_t)((cnt - (uint8_t)slot_cnt_) & 0x3u);
                    // step 1 is the next slot. 2 or 3 means slots went missing;
                    // 0 with a repeated sensor means four did (or a multiple).
                    if (step != 1) counter_gaps++;
                }
                have_ = 0;
                slot_cnt_ = cnt;
            }

            dst[0] = (int16_t)((uint16_t)w[1] | ((uint16_t)w[2] << 8));
            dst[1] = (int16_t)((uint16_t)w[3] | ((uint16_t)w[4] << 8));
            dst[2] = (int16_t)((uint16_t)w[5] | ((uint16_t)w[6] << 8));
            have_ |= bit;

            if (have_ == HAVE_ALL)
            {
                sink(static_cast<const Ism6FifoSample&>(cur_));
                samples++;
                have_ = 0;  // slot_cnt_ stays: the next slot must advance it
            }
        }
    }

private:
    static constexpr uint8_t HAVE_G = 1u << 0;
    static constexpr uint8_t HAVE_LG = 1u << 1;
    static constexpr uint8_t HAVE_HG = 1u << 2;
    static constexpr uint8_t HAVE_ALL = HAVE_G | HAVE_LG | HAVE_HG;

    Ism6FifoSample cur_ = {};
    uint8_t have_ = 0;
    int8_t slot_cnt_ = -1;  // counter of the slot being filled; -1 before the first word
};
