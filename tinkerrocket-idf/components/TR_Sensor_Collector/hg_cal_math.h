// hg_cal_math.h — the high-g accelerometer's six-face alignment to the low-g
// (RocketComputerTypes.h, HG_CAL_*: high = M * h - b - o).  What the flight
// computer checks before it stores an alignment, and the identity test the
// converter and NVS loading share.  The fit itself is the apps' (and
// tools/accel_cal_six_face.py's); the flight computer only judges what it is
// sent.  Pure: tests_cpp compiles it on the host.
#pragma once

#include <cmath>
#include <cstdint>
#include <RocketComputerTypes.h>

namespace hg_cal
{
    inline void setIdentity(float m[9])
    {
        for (int i = 0; i < 9; i++) m[i] = (i % 4 == 0) ? 1.0f : 0.0f;
    }

    // Exactly the identity: no alignment, so the converter keeps its original
    // arithmetic.
    inline bool isIdentity(const float m[9])
    {
        for (int i = 0; i < 9; i++)
        {
            if (m[i] != ((i % 4 == 0) ? 1.0f : 0.0f)) return false;
        }
        return true;
    }

    // Why the flight computer will not store `a`, or nullptr when it will.
    // A clear always may; a store needs every number finite, the alignment's
    // diagonal within HG_CAL_SCALE_MIN..MAX, the rest within
    // +-HG_CAL_CROSS_MAX, and each bias within +-HG_CAL_BIAS_MAX_MPS2.
    inline const char* refusal(const HighGCalApplyData& a)
    {
        if (a.op == HG_CAL_OP_CLEAR) return nullptr;
        if (a.op != HG_CAL_OP_STORE) return "unknown op";
        for (int i = 0; i < 9; i++)
        {
            const float v = a.matrix[i];
            if (!std::isfinite(v)) return "a matrix entry is not finite";
            if (i % 4 == 0)
            {
                if (v < HG_CAL_SCALE_MIN || v > HG_CAL_SCALE_MAX) return "a scale is outside 0.97..1.03";
            }
            else if (std::fabs(v) > HG_CAL_CROSS_MAX)
            {
                return "a cross-axis term is past 0.03";
            }
        }
        for (int i = 0; i < 3; i++)
        {
            const float v = a.bias_mps2[i];
            if (!std::isfinite(v)) return "a bias is not finite";
            if (std::fabs(v) > HG_CAL_BIAS_MAX_MPS2) return "a bias is past 11 m/s^2";
        }
        return nullptr;
    }
}
