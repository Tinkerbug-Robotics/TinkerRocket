#pragma once
//
//  SimHighGBias.h — the rig replay's high-g counts, carrying the flight
//  computer's high-g bias (#1600 on the V9 mosaic image).
//
//  Where the low-g accelerometer nears its rail the flight computer takes the
//  high-g channel instead, less its high-g bias (SensorConverter::
//  setHighGBias: board frame, after the sensor's Z rotation, about 1.3 m/s^2
//  on the Goblin's V9). The replay writes the flight's own specific force into
//  those counts, so without the bias in them every railed sample came out the
//  bias short -- 48 ms at 18.8 g in rp5_0829, 22 ms in eagleclaw_0517: 0.03 to
//  0.06 m/s. With it the converter returns the flight's force, and the log's
//  own conversion and the offline harness, which take the logged bias off,
//  agree with the flight computer.
//
//  Pure, host-tested in tests_cpp/test_sim_high_g_bias.cpp against the
//  converter itself.
//

#include <math.h>
#include <stdint.h>

namespace sim_high_g {

// A board-frame specific force (m/s^2) plus the bias the converter takes off
// (m/s^2, board frame), in the sensor's axes (the inverse of the converter's
// Z rotation: c and s its cosine and sine) and counts of lsb m/s^2, clamped
// to the 16-bit range.
inline void countsCarryingBias(const float f_board[3], const float bias[3], float c, float s, float lsb,
                               int16_t out[3])
{
    const float hx = f_board[0] + bias[0];
    const float hy = f_board[1] + bias[1];
    const float hz = f_board[2] + bias[2];
    const float v[3] = {hx * c + hy * s, -hx * s + hy * c, hz};
    for (int i = 0; i < 3; ++i)
    {
        const long n = lroundf(v[i] / lsb);
        out[i] = static_cast<int16_t>(n < -32768 ? -32768 : (n > 32767 ? 32767 : n));
    }
}

}  // namespace sim_high_g
