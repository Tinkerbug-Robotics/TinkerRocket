#pragma once

#include <stdint.h>

// #1128: the LLCC68's spreading-factor window depends on the CURRENT
// bandwidth, which makes the order of a multi-step modulation change part of
// its correctness rather than a detail.
//
// LLCC68::setSpreadingFactor switches on the bandwidth already programmed into
// the chip and range-checks SF against it (RadioLib
// modules/LLCC68/LLCC68.cpp): 5..9 at BW125, 5..10 at BW250, 5..11 at BW500,
// and EVERY SF is rejected if the bandwidth is not one of those three. So
// "set bandwidth first" is not a style choice in reconfigure() — it is the
// only order in which an increase in SF can be accepted.
//
// The rollback did not inherit that. It undid the four steps in strict reverse
// order, which puts setSpreadingFactor(old_sf) BEFORE setBandwidth(old_bw) and
// therefore evaluates the old SF against the NEW bandwidth. Going from
// BW250/SF10 to BW125, the restore of SF10 is rejected at BW125, the error was
// discarded by a (void) cast, and the chip was left at the NEW spreading
// factor with the OLD bandwidth — while cfg_sf_/cfg_bw_khz_, and so
// currentSpreadingFactor(), the OC's lora_sf and every LORA_UPLINK_MSG record,
// still reported the old pair. The downlink is dead from that point and only a
// reboot restores it.
//
// This header is the rule on its own so it can be tested off-target, where the
// chip is not available. The driver applies it; nothing here talks to RadioLib.

namespace LoraModulationPolicy {

// Largest spreading factor the LLCC68 accepts at this bandwidth. 0 means the
// bandwidth itself is not one the part supports, at which point no SF is legal.
inline uint8_t maxSpreadingFactorFor(float bw_khz)
{
    // Compared with a tolerance because these arrive as floats from the wire.
    auto near = [](float a, float b) { return (a - b) < 0.5f && (b - a) < 0.5f; };
    if (near(bw_khz, 125.0f)) return 9;
    if (near(bw_khz, 250.0f)) return 10;
    if (near(bw_khz, 500.0f)) return 11;
    return 0;
}

// Is this a pair the chip will actually accept?
inline bool isLegalPair(float bw_khz, uint8_t sf)
{
    const uint8_t hi = maxSpreadingFactorFor(bw_khz);
    if (hi == 0) return false;
    return sf >= 5 && sf <= hi;
}

// Given a modulation change that failed partway and must be undone, may the
// spreading factor be restored BEFORE the bandwidth?
//
// Only when old_sf is legal against the bandwidth that is live at that moment,
// which during a reverse-order rollback is the NEW one. reconfigure() does not
// rely on this being true — it restores bandwidth first unconditionally, which
// is safe for every pair — but stating it is what makes the regression
// testable.
inline bool sfRestoreIsSafeBeforeBw(float new_bw_khz, uint8_t old_sf)
{
    return isLegalPair(new_bw_khz, old_sf);
}

}  // namespace LoraModulationPolicy
