#pragma once

// #1108: keep the barometric ground reference from following a launch.
//
// THE DEFECT.  While the FC is in INITIALIZATION / READY (and MAG_CALIBRATION)
// the ground-pressure reference is re-seeded from the CURRENT sample on every
// pass, so pressure_altitude_m is identically 0 and the altitude filter's rate
// is 0 no matter how fast the vehicle climbs.  That is what makes the pad
// datum the last steady reading before PRELAUNCH freezes it (#297) -- and it
// is also why a launch from READY (#382: OC dead, no fix, boost before lock)
// is invisible to any barometric launch detector.  With a working IMU that
// never matters: accel latches launch, and PRELAUNCH is not needed for it.
// With the IMU stale or absent, the baro-only launch fallback in
// TR_KinematicChecks is the only path to INFLIGHT, and it needs the reference
// to STOP FOLLOWING the climb.
//
// DESIGN.  Weather moves the pad pressure by about a pascal a minute.
// Handling moves it by tens of pascals over seconds (carrying the airframe up
// a ladder at 1 m/s is ~12 Pa/s).  A launch moves it by hundreds of pascals a
// second within the first few hundred milliseconds.  So: keep a short history
// of (time, pressure); while the rate over that window is under FREEZE_PA_S
// the reference may track, and once it exceeds it the reference HOLDS --
// rolled back to the value from before the window, so the datum is the
// pre-motion pressure and the detection latency does not turn into an AGL
// error for the rest of the flight.  A hold is STICKY: tracking resumes only
// after the rate has stayed under the bar for RESUME_STEADY_US, because in a
// real boost the window rate can dip under it for a moment (the boost-suction
// transient snapping back at burnout cancels the climb for one window), and a
// single Track verdict there would re-seed the datum at altitude -- the corpus
// replay showed 76 m on one flight before this rule.  A lifted airframe still
// re-seeds at its new height a second after the lift ends, so a false hold
// costs a second of tracking and nothing else.
//
// This is not a launch detector.  It answers "may the datum move right now",
// nothing more; the launch decision stays in TR_KinematicChecks.  The rate is
// taken between the mean of the three oldest and the three newest entries in
// the window, not between two single samples: BMP585 samples are ~2 Pa noisy,
// which is ~11 Pa/s one-sigma across a 250 ms window sample-to-sample and
// ~6.5 Pa/s with the averaging, so the 60 Pa/s bar is ~9 sigma from a still
// pad (a single-sample 40 Pa/s bar false-held every few seconds on the host
// test).  60 Pa/s is ~5 m/s of climb: a 2 g-net launch crosses it about
// 350 ms after ignition, a hand-carried airframe never does.
//
// Pure and host-tested (tests_cpp/test_ground_ref_freeze.cpp); the FC and the
// mini share it.

#include <stdint.h>
#include <math.h>

namespace GroundRefFreeze {

constexpr float    FREEZE_PA_S = 60.0f;    // ~5 m/s of climb near sea level
constexpr uint32_t WINDOW_US   = 250000u;  // rate is measured over this span
constexpr uint32_t SPACING_US  = 25000u;   // history entries at least this far apart
constexpr int      HISTORY     = 12;       // 12 x 25 ms > WINDOW_US
constexpr int      END_SAMPLES = 3;        // entries averaged at each end of the window
constexpr uint32_t RESUME_STEADY_US = 1000000u;  // under the bar this long before tracking resumes

enum class Verdict : uint8_t {
    Track      = 0,   // rate under the bar: the reference may follow this sample
    FreezeEdge = 1,   // first sample over the bar: hold, and roll back to rollback_pa
    Frozen     = 2,   // still over the bar: hold
};

struct State {
    uint32_t t_us[HISTORY] = {};
    float    p_pa[HISTORY] = {};
    int      count       = 0;     // entries filled (<= HISTORY)
    int      head        = 0;     // next write slot
    bool     frozen      = false;
    uint32_t last_over_us = 0;    // last sample whose window rate was over the bar
    float    rollback_pa = 0.0f;  // the pre-motion pressure; valid from FreezeEdge on
    float    rate_pa_s   = 0.0f;  // last computed rate, for logging
};

inline void reset(State& st) { st = State{}; }

// Feed the latest barometer sample on every pass.  A sample presented again
// with the same timestamp is not re-recorded, so the FC's 1 kHz loop can call
// this with a ~240 Hz sensor and the history still spans WINDOW_US.
inline Verdict step(State& st, uint32_t t_us, float p_pa)
{
    if (st.count == 0 ||
        (uint32_t)(t_us - st.t_us[(st.head + HISTORY - 1) % HISTORY]) >= SPACING_US)
    {
        st.t_us[st.head] = t_us;
        st.p_pa[st.head] = p_pa;
        st.head = (st.head + 1) % HISTORY;
        if (st.count < HISTORY) st.count++;
    }

    // Mean of the END_SAMPLES oldest entries inside the window against the
    // mean of the END_SAMPLES newest (the current sample included), each with
    // its mean age, so the rate is a difference of averages rather than of two
    // noisy samples.  Needs at least half a window of span between the two.
    float old_p = 0.0f, old_age = 0.0f; int old_n = 0;
    float new_p = p_pa, new_age = 0.0f; int new_n = 1;
    for (int k = 0; k < st.count; ++k)
    {
        const int idx = (st.head + HISTORY - st.count + k) % HISTORY;   // oldest first
        const uint32_t age = (uint32_t)(t_us - st.t_us[idx]);
        if (age > WINDOW_US) continue;
        if (old_n < END_SAMPLES) { old_p += st.p_pa[idx]; old_age += (float)age; old_n++; }
    }
    for (int k = 1; k < END_SAMPLES && k <= st.count; ++k)
    {
        const int idx = (st.head + HISTORY - k) % HISTORY;              // newest first
        const uint32_t age = (uint32_t)(t_us - st.t_us[idx]);
        if (age > WINDOW_US) break;
        new_p += st.p_pa[idx]; new_age += (float)age; new_n++;
    }
    if (old_n >= END_SAMPLES)
    {
        old_p /= old_n; old_age /= old_n;
        new_p /= new_n; new_age /= new_n;
        const float span_s = (old_age - new_age) * 1e-6f;
        if (span_s >= (float)(WINDOW_US / 2) * 1e-6f)
        {
            st.rate_pa_s = (new_p - old_p) / span_s;
            if (fabsf(st.rate_pa_s) < FREEZE_PA_S)
            {
                if (st.frozen &&
                    (uint32_t)(t_us - st.last_over_us) < RESUME_STEADY_US)
                    return Verdict::Frozen;        // sticky: not steady long enough yet
                st.frozen = false;
                return Verdict::Track;
            }
            st.last_over_us = t_us;
            if (!st.frozen)
            {
                st.frozen      = true;
                st.rollback_pa = old_p;            // the pre-motion pressure
                return Verdict::FreezeEdge;
            }
            return Verdict::Frozen;
        }
    }
    // Not enough history yet (the first quarter second after boot or reset).
    // A hold in force stays in force through this branch: with the sticky
    // rule it can only be released by a steady second, never by a gap.
    return st.frozen ? Verdict::Frozen : Verdict::Track;
}

}  // namespace GroundRefFreeze
