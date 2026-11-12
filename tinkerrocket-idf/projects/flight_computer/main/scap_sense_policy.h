#pragma once

#include <stdint.h>
#include <RocketComputerTypes.h>   // fcStatusScapMv, FCS_SCAP_NO_READING

// #1409: the hold-up supercap voltage on the Tinker-Mantis V10/V11, where the
// sense divider (R125/R126, net V_SCAP_ADC) lands on this processor's GPIO17
// rather than on the out computer as on the mini (#1166). The flight computer
// reads it once a second and sends it in FcStatusData.scap_mv; the out
// computer runs the mini's verdict on it (holdup_policy.h).
//
// Pure: no IDF, so tests_cpp drives it. The ADC calls stay in main.cpp.
namespace scap_sense
{
    // The FcStatusData.scap_mv field from one read's calibrated samples of the
    // pad: `pin_mv_sum` over `taken` samples, times the divider. No sample
    // taken (every ADC call failed) is FCS_SCAP_NO_READING, which the out
    // computer reports as NO READING rather than as a voltage.
    inline uint16_t fieldFromPinSamples(int32_t pin_mv_sum, int taken, float divider_ratio)
    {
        if (taken <= 0) return FCS_SCAP_NO_READING;
        const float pin_mv = (float)pin_mv_sum / (float)taken;
        return fcStatusScapMv(pin_mv * 0.001f * divider_ratio);
    }
}
