#pragma once
// #714 — the external 2S flight pack on the base station's charger jack (J4),
// read through two dividers: PosADC sees the whole pack, MidADC the top of
// cell 1. This turns the two pin voltages into what the operator wants to
// know — is a pack there, what is it at, and are its cells together. Nothing
// here touches hardware; tests_cpp/test_bs_pack_sense_policy.cpp pins it.

#include <cmath>

namespace bs_pack_sense {

// A 2S pack below this is not a pack: the jack is open (the divider reads the
// charger's leakage, tens of millivolts) or the cells are so far gone that no
// number about them is worth acting on. A live 2S pack cannot sit below ~5 V
// without being destroyed.
constexpr float kPresentMinV = 5.0f;

// A mid tap closer than this to either rail is not a cell: the tap is open, or
// shorted to one end of the pack.
constexpr float kCellMinV = 0.5f;

struct Reading {
    float pack_v  = NAN;   // whole pack; NaN = no pack
    float cell1_v = NAN;   // the mid tap; NaN when it cannot be read as a cell
    float cell2_v = NAN;   // pack minus mid
};

// pos_v / mid_v: the two divider readings scaled back to the top of each
// divider, NaN when the channel could not be read.  charger_says_absent: the
// MP2672's BATTFLOAT_STAT while the charger is powered — with charge input
// present and no pack, the charger regulates its own output onto the pack
// terminal, and PosADC reads a convincing 8.x V of no battery at all.
inline Reading derive(float pos_v, float mid_v, bool charger_says_absent)
{
    Reading r;
    if (std::isnan(pos_v) || pos_v < kPresentMinV || charger_says_absent) return r;
    r.pack_v = pos_v;
    if (std::isnan(mid_v) || mid_v < kCellMinV || mid_v > pos_v - kCellMinV) return r;
    r.cell1_v = mid_v;
    r.cell2_v = pos_v - mid_v;
    return r;
}

}  // namespace bs_pack_sense
