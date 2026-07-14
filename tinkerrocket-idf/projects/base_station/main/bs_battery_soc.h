#pragma once

// Voltage-based state-of-charge estimator (#501).
//
// The BQ27Z746's own SoC is unusable on this board: it reported 0% with the
// pack at 4.17 V and accepting 449 mA, with RemainingCapacity=0 and
// FullChargeCapacity=27932 mAh against a 2800 mAh design (~10x).
//
// Root cause is the CURRENT path, not the driver — TR_BQ27Z746.cpp reads RSOC /
// RemainingCapacity / FullChargeCapacity straight through with no scaling. The
// gauge's CC Gain (data flash 0x4006) still sits at its factory default and has
// never been calibrated against a known current (the driver header says so), and
// ManufacturingStatus[GAUGE_EN] ships at 0. An Impedance-Track gauge derives
// RM/FCC/SoC by integrating current, so a wrong (or ungauged) current makes all
// three garbage — while Voltage(), a direct ADC read, stays correct.
//
// So: estimate SoC from voltage. It is less accurate than a *working* coulomb
// counter but it is honest, and it degrades gracefully — which beats reporting a
// confident 0% into the app on a full pack, with no low-battery warning.
//
// Pure — no ESP-IDF / hardware dependencies — so the host gtest drives it
// directly. Same pattern as bs_uplink_queue.h / bs_uplink_policy.h.

#include <cstddef>

namespace bs_battery_soc {

// Rest (open-circuit) voltage -> SoC for one Li-ion cell at room temperature.
// Generic LiCoO2/graphite curve, which is what the 18650 packs here are. The
// knee below ~3.5 V is steep and the 3.6-4.0 V plateau is flat — that flatness
// is the fundamental accuracy limit of any voltage-only method, and the reason
// this is a fallback rather than the preferred approach.
struct OcvPoint
{
    float volts;
    float soc;  // percent
};

inline constexpr OcvPoint kOcvCurve[] = {
    {3.00f,   0.0f},
    {3.35f,   5.0f},
    {3.45f,  10.0f},
    {3.51f,  15.0f},
    {3.55f,  20.0f},
    {3.58f,  25.0f},
    {3.60f,  30.0f},
    {3.63f,  35.0f},
    {3.65f,  40.0f},
    {3.68f,  45.0f},
    {3.71f,  50.0f},
    {3.75f,  55.0f},
    {3.79f,  60.0f},
    {3.83f,  65.0f},
    {3.87f,  70.0f},
    {3.92f,  75.0f},
    {3.97f,  80.0f},
    {4.02f,  85.0f},
    {4.08f,  90.0f},
    {4.13f,  95.0f},
    {4.20f, 100.0f},
};

inline constexpr size_t kOcvCurveN = sizeof(kOcvCurve) / sizeof(kOcvCurve[0]);

inline constexpr float kCellEmptyV = 3.00f;
inline constexpr float kCellFullV  = 4.20f;

// Piecewise-linear lookup, clamped at both ends. Input is PER-CELL open-circuit
// volts.
inline float socFromCellOcv(float ocv_v)
{
    if (ocv_v <= kOcvCurve[0].volts)              return kOcvCurve[0].soc;
    if (ocv_v >= kOcvCurve[kOcvCurveN - 1].volts) return kOcvCurve[kOcvCurveN - 1].soc;

    for (size_t i = 1; i < kOcvCurveN; i++)
    {
        if (ocv_v <= kOcvCurve[i].volts)
        {
            const OcvPoint& lo = kOcvCurve[i - 1];
            const OcvPoint& hi = kOcvCurve[i];
            const float span = hi.volts - lo.volts;
            if (span <= 0.0f) return lo.soc;
            const float t = (ocv_v - lo.volts) / span;
            return lo.soc + t * (hi.soc - lo.soc);
        }
    }
    return kOcvCurve[kOcvCurveN - 1].soc;
}

// Terminal volts -> open-circuit volts, removing the IR offset.
//
// current_ma is POSITIVE when charging (the gauge's convention). Charging lifts
// the terminal above the true OCV, discharging drags it below, so OCV = V - I*R.
//
// Pass r_int_ohm = 0 to DISABLE this, which is what the base station does today:
// compensating with the BQ27Z746's uncalibrated current would inject a scaling
// error rather than remove one. Wired up and tested so it can be switched on the
// moment CC Gain is calibrated (#501 follow-up).
inline float ocvFromTerminal(float terminal_v, float current_ma, float r_int_ohm)
{
    if (r_int_ohm <= 0.0f) return terminal_v;
    return terminal_v - (current_ma / 1000.0f) * r_int_ohm;
}

// Is the gauge's OWN SoC believable? If yes, prefer it — a learned coulomb
// counter beats a voltage guess. Every clause below is something the 2026-07-14
// bench log actually violated.
inline bool gaugeSocPlausible(bool gauge_en,
                              float soc_pct,
                              float fcc_mah,
                              float design_mah,
                              float pack_v,
                              int   cells)
{
    // Impedance Track disabled -> RM/FCC/SoC were never gauged at all.
    if (!gauge_en) return false;

    if (soc_pct < 0.0f || soc_pct > 100.0f) return false;

    // A learned FullChargeCapacity sits near design. 27932 vs 2800 does not.
    if (design_mah > 0.0f)
    {
        if (fcc_mah < 0.5f * design_mah) return false;
        if (fcc_mah > 1.5f * design_mah) return false;
    }

    // 0% while the cells are plainly not empty — the headline symptom.
    if (cells > 0 && soc_pct <= 0.0f && (pack_v / (float)cells) > 3.5f) return false;

    return true;
}

// Exponentially-smoothed voltage -> SoC. Smoothing is on the VOLTAGE, not the
// SoC, so the curve's steep knee isn't flattened by the filter.
class Estimator
{
public:
    // alpha: EMA weight for each new sample (0..1]. 0.1 at the base station's
    // ~1 Hz battery poll gives a ~10 s time constant — long enough to ride out
    // TX current spikes, short enough to track a real discharge.
    explicit Estimator(float alpha = 0.1f) : alpha_(alpha) {}

    // pack_v: measured terminal volts across the whole pack.
    // cells:  cells in series (BQ27Z746 board is 1S).
    // Returns the SoC estimate in percent.
    float update(float pack_v, int cells, float current_ma = 0.0f, float r_int_ohm = 0.0f)
    {
        if (cells <= 0) cells = 1;
        const float cell_v = ocvFromTerminal(pack_v, current_ma, r_int_ohm) / (float)cells;

        if (!primed_)
        {
            filtered_cell_v_ = cell_v;   // snap on the first sample, don't ramp from 0
            primed_ = true;
        }
        else
        {
            filtered_cell_v_ += alpha_ * (cell_v - filtered_cell_v_);
        }
        return socFromCellOcv(filtered_cell_v_);
    }

    bool  primed() const          { return primed_; }
    float filteredCellV() const   { return filtered_cell_v_; }
    float soc() const             { return primed_ ? socFromCellOcv(filtered_cell_v_) : 0.0f; }
    void  reset()                 { primed_ = false; filtered_cell_v_ = 0.0f; }

private:
    float alpha_;
    float filtered_cell_v_ = 0.0f;
    bool  primed_          = false;
};

}  // namespace bs_battery_soc
