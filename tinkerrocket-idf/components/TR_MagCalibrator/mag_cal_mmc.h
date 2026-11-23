#pragma once

// The MMC5983MA in the hard-iron calibration (#96).  On a board whose
// magnetometer is the MMC5983MA (the Tinker-Beetle V2; a V7 whose I2C slot
// found nothing), the FC feeds the calibrator the MMC's counts restated in
// the IIS2MDC's 0.15 µT counts, and turns the fit back into MMC counts for
// the converter's software offset (SensorConverter::setMMCOffset).
//
// Why the IIS2MDC's counts and not the MMC's own: the calibrator holds
// int16 samples, and the MMC's 18-bit counts (163.84 per µT) put a 200 µT
// hard iron past int16.  At 0.15 µT the whole MMC range, offset included,
// sits inside int16 (static_assert below).  And everything downstream of the
// calibrator already speaks those counts: the status frame the apps decode at
// the board's mag_type scale (an MMC board reports MAG_TYPE_IIS2MDC), the
// NVS cx/cy/cz, and the MAG_CAL_APPLY profile push.  A calibration saved on an
// MMC board round-trips through the app unchanged.
//
// The ratio is exact: 0.15 µT / (800/131072 µT) = 24.576 = 3072/125, so the
// conversions are integer, rounded half away from zero.  IDF-free, so the host
// tests run the same code (tests_cpp/test_mag_calibrator.cpp).

#include <stdint.h>
#include <RocketComputerTypes.h>

namespace mag_cal_mmc {

// MMC counts per calibrator count, as a fraction.
constexpr int32_t kNum = 3072;
constexpr int32_t kDen = 125;
static_assert(MAG_UT_PER_LSB_IIS2MDC / MAG_UT_PER_COUNT_MMC5983MA - (double)kNum / kDen < 1e-9 &&
              (double)kNum / kDen - MAG_UT_PER_LSB_IIS2MDC / MAG_UT_PER_COUNT_MMC5983MA < 1e-9,
              "3072/125 is not the IIS2MDC/MMC5983MA count ratio");

// An 18-bit MMC count less a centered-count offset spans at most 2^18 either
// way; restated at 0.15 µT that is 10667, well inside int16.
static_assert((int64_t)(1 << 18) * kDen / kNum < 32767,
              "an MMC count, offset included, overflows the calibrator's int16");

inline int32_t divRound(int64_t n, int64_t d)   // d > 0
{
    return (int32_t)(n >= 0 ? (n + d / 2) / d : -((-n + d / 2) / d));
}

// The MMC's raw 18-bit count, centered (mmc5983ma_centered_counts in the
// converter): offset binary about 2^17.
inline int32_t centered(uint32_t raw18)
{
    return (int32_t)(raw18 & 0x3FFFFu) - 131072;
}

// A centered MMC count, less the offset the converter subtracts now, in the
// calibrator's counts.  The calibrator sees what the converter would turn
// into µT, the way it sees an IIS2MDC sample after the chip's OFFSET
// registers: raw while sampling (the session zeroes the offset), corrected
// while verifying (the candidate is on).
inline int16_t toCalCounts(uint32_t raw18, int32_t offset_counts)
{
    return (int16_t)divRound((int64_t)(centered(raw18) - offset_counts) * kDen, kNum);
}

// A calibrator offset (0.15 µT counts) in MMC centered counts, for
// SensorConverter::setMMCOffset and the NVS mmc_cx/cy/cz.
inline int32_t toMmcCounts(int16_t cal_counts)
{
    return divRound((int64_t)cal_counts * kNum, kDen);
}

}  // namespace mag_cal_mmc
