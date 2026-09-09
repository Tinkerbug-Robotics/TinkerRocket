#pragma once
#include <cstdint>

// #1155 item 17: the INA230 CALIBRATION arithmetic on its own, so it can be
// host-tested without the I2C driver (tests_cpp/test_ina230_cal.cpp).
//
// CAL = 0.00512 / (Current_LSB * R_SHUNT)  (datasheet equation 1). The old
// code narrowed the float to uint16_t and only THEN clamped to 0x7FFF, so any
// CAL >= 65536 wrapped modulo 65536 first — 2,560,000 became 4096, which
// passed the clamp and was programmed while calibrate() returned OK, leaving
// CURRENT and POWER wrong by a large integer factor. The clamp now happens in
// float, before narrowing, the result is rounded to nearest instead of
// truncated, and a CAL that would round to 0 (a flat 0 A that still reports
// OK) is refused instead of saturated.
namespace tr_ina230 {

inline bool computeCalibration(float r_shunt_ohm, float current_lsb_A, uint16_t& cal)
{
    // Written as negations so NaN fails too (#297 guarded only the divide).
    if (!(current_lsb_A > 0.0f) || !(r_shunt_ohm > 0.0f)) return false;
    float cal_f = 0.00512f / (current_lsb_A * r_shunt_ohm);
    if (!(cal_f >= 1.0f)) return false;          // would program CAL = 0 (or NaN/inf)
    if (cal_f > 32767.0f) cal_f = 32767.0f;      // clamp BEFORE narrowing
    // Nearest, not truncation: the float quotient for the nominal 2 mOhm /
    // 1 mA pair is 2559.9998, which the old cast programmed as 2559.
    cal = static_cast<uint16_t>(cal_f + 0.5f);
    return true;
}

}  // namespace tr_ina230
