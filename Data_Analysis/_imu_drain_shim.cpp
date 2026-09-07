// C shim so replay_imu_drain.py can run the REAL drain-window accumulator
// (imu_drain_window.h, #1191) over a recorded IMU stream, instead of a Python
// port of it.  Same pattern as _launch_detect_shim.cpp: the driver rebuilds
// this whenever the firmware header changes, so the analysis can never
// quietly disagree with the vehicle.
//
// Built on demand by the Python driver -- see _build_shim() there.  Not a
// standalone program.

#include <cstdint>
#include "imu_drain_window.h"

extern "C" {

// Replay one IMU stream through the flight loop's drain, one window per tick.
//
//   n_samples, t_us   the logged samples in stream order; t_us is the raw
//                     uint32 micros() stamp widened to 64 bits and unwrapped
//   lg, hg, gy        raw int16 counts, 3 * n_samples each, x y z interleaved
//   n_ticks, tick_us  the estimator passes: tick k drains every not-yet-drained
//                     sample whose stamp is <= tick_us[k], in stream order —
//                     exactly the while(getISM6HG256Data()) loop
//   near_rail_lsb     the low-g bar in raw LSB (imu_drain::nearRailLsb)
//
// Outputs, one per tick:
//   out_n             samples drained by that tick (0 = nothing new; the mean
//                     and freshest outputs then repeat the previous tick's,
//                     exactly as ism6_latest_si retains its value)
//   out_stamp_us      the mean sample's stamp (window centre), uint32 as the
//                     firmware carries it; 0 until the first drained window
//   out_mean          9 * n_ticks: the window mean the new path converts
//   out_last          9 * n_ticks: the freshest sample the old path converted
//   out_last_idx      index of that freshest sample in the input (-1 if none yet)
//   out_near_rail     the new verdict (window max |raw| per sensor axis > bar)
int tr_imu_drain_replay(int n_samples, const unsigned long long* t_us,
                        const short* lg, const short* hg, const short* gy,
                        int n_ticks, const unsigned long long* tick_us,
                        int near_rail_lsb,
                        unsigned* out_n, unsigned* out_stamp_us,
                        short* out_mean, short* out_last, int* out_last_idx,
                        unsigned char* out_near_rail)
{
    ISM6HG256Data mean_raw = {};
    ISM6HG256Data last_raw = {};
    int last_idx = -1;
    bool near_rail = false;
    int i = 0;

    for (int k = 0; k < n_ticks; ++k)
    {
        ImuDrainWindow win;
        while (i < n_samples && t_us[i] <= tick_us[k])
        {
            ISM6HG256Data s;
            s.time_us = (uint32_t)t_us[i];
            s.acc_low_raw  = {lg[3 * i], lg[3 * i + 1], lg[3 * i + 2]};
            s.acc_high_raw = {hg[3 * i], hg[3 * i + 1], hg[3 * i + 2]};
            s.gyro_raw     = {gy[3 * i], gy[3 * i + 1], gy[3 * i + 2]};
            win.add(s);
            last_raw = s;
            last_idx = i;
            ++i;
        }

        out_n[k] = win.n;
        if (win.mean(mean_raw))
        {
            near_rail = win.lowGNearRail(near_rail_lsb);
        }

        out_stamp_us[k]  = mean_raw.time_us;
        out_last_idx[k]  = last_idx;
        out_near_rail[k] = near_rail ? 1 : 0;

        short* m = out_mean + 9 * k;
        m[0] = mean_raw.acc_low_raw.x;  m[1] = mean_raw.acc_low_raw.y;  m[2] = mean_raw.acc_low_raw.z;
        m[3] = mean_raw.acc_high_raw.x; m[4] = mean_raw.acc_high_raw.y; m[5] = mean_raw.acc_high_raw.z;
        m[6] = mean_raw.gyro_raw.x;     m[7] = mean_raw.gyro_raw.y;     m[8] = mean_raw.gyro_raw.z;

        short* l = out_last + 9 * k;
        l[0] = last_raw.acc_low_raw.x;  l[1] = last_raw.acc_low_raw.y;  l[2] = last_raw.acc_low_raw.z;
        l[3] = last_raw.acc_high_raw.x; l[4] = last_raw.acc_high_raw.y; l[5] = last_raw.acc_high_raw.z;
        l[6] = last_raw.gyro_raw.x;     l[7] = last_raw.gyro_raw.y;     l[8] = last_raw.gyro_raw.z;
    }
    return i;   // samples consumed
}

}  // extern "C"
