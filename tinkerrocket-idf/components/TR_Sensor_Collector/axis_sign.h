#pragma once

#include <stdint.h>

// A chip axis reversed as it is read, for a part on the BACK of the board
// (SensorCollector::configureAxisSigns; the Tinker-Beetle V2's IMU and
// magnetometer, board_m2.h). IDF-free, so the host tests turn a reading over
// with the same code the collector does (tests_cpp/test_imu_frame_board.cpp,
// test_mag_frame_board.cpp).
namespace axis_sign {

// An ISM6HG256X count. The negation cannot wrap: -32768, the negative rail,
// reads +32767, so a railed axis stays railed (as satSubI16 keeps it, #1154
// item 10). A sign of +1 (or 0) leaves the count alone.
inline int16_t i16(int16_t v, int8_t s)
{
    if (s >= 0) return v;
    return v == INT16_MIN ? INT16_MAX : (int16_t)(-v);
}

// An MMC5983MA count: 18 bits, offset binary about the chip's zero, 2^17
// (the converter's mmc5983ma_centered_counts). Reversed about that zero; 0,
// the negative rail, reads 0x3FFFF, the positive one.
inline uint32_t mmc18(uint32_t raw18, int8_t s)
{
    raw18 &= 0x3FFFFu;
    if (s >= 0) return raw18;
    return raw18 == 0u ? 0x3FFFFu : (0x40000u - raw18);
}

}  // namespace axis_sign
