#pragma once

// The Tinker-Base V2's INA230 (U24, board_v5.h) -> the base station's battery
// fields. Pure, so the host gtest pins the sign and the voltage sum
// (tests_cpp/test_bs_cell_current.cpp). Same pattern as bs_battery_soc.h.
//
// How the part is wired (hardware/tinker-base, battery.kicad_sch):
//
//     BT2 + ---- B+ ---[ R62 20 mOhm ]-- VCC ---+--- U5 BQ21040 OUT (charge in)
//                |                        |      +--- U22 load switch VIN (load out)
//               IN+                    IN-, BUS
//
// The shunt carries the cell's NET current. Discharge flows B+ -> VCC, IN+ to
// IN-, and the INA230 reads it POSITIVE. Charge from U5 flows VCC -> B+ and
// reads NEGATIVE.
//
// The base station's convention is the gauges' (bs_current, BLE "bcur", the
// app's "Base Stn" current, bs_battery_soc::ocvFromTerminal): milliamps,
// POSITIVE WHEN CHARGING. So the INA230's sign is flipped here, once.
//
// The voltage: BUS sits on VCC, the shunt's load end, so it reads the cell
// minus the shunt drop. The shunt register is V(IN+) - V(IN-) = V(B+) - V(VCC),
// so BUS + SHUNT is the cell's own terminal voltage, B+, exactly, in either
// direction. At 20 mOhm the correction is 4 mV per 200 mA: small, but it is
// free, and it keeps the reported voltage from moving with the load.

namespace bs_cell_current {

struct Reading
{
    float cell_v;      // B+, the cell terminal (V)
    float current_ma;  // cell current (mA), POSITIVE = CHARGING
};

// bus_v:     the BUS_VOLTAGE register in volts (VCC)
// shunt_mv:  the SHUNT_VOLTAGE register in millivolts, V(B+) - V(VCC)
// current_a: the CURRENT register in amps, positive = discharge (INA230 sign)
inline Reading fromIna230(float bus_v, float shunt_mv, float current_a)
{
    return Reading{bus_v + shunt_mv * 0.001f, -current_a * 1000.0f};
}

}  // namespace bs_cell_current
