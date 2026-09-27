#pragma once

// GNSS ascent admission: when may the receiver's VERTICAL solution feed the
// EKF, and GNSS cast an apogee vote, again after launch?  Owner's rule,
// 2026-09-26.
//
// THE DEFECT.  The receiver runs its own navigation filter, and under boost
// that filter is outside the dynamics it was configured for.  Measured on the
// 26 historical flights with IMU, baro and GNSS in the log (all u-blox
// SAM-M10Q), every fix below flagged valid — FixMode 3, >= 4 satellites:
//
//   * every flight: vertical velocity 5-25 m/s behind the IMU for the first
//     ~0.5 s of the burn, back within 1-2 s on a good sky;
//   * most flights: altitude 10-50 m low by burnout, and it STAYS low through
//     the coast, converging over 5-10 s;
//   * Rolly Polly V nosecone 08-29, Rolly Polly III 06-14, Rolly Polly 54 mm
//     08-29 (8-13 satellites on the pad, 28-94 g): vertical velocity frozen or
//     reversed (-38 m/s while climbing at 60), altitude 70-380 m off, with the
//     receiver reporting 1-2 m vertical accuracy and PDOP 1.5-3.
//
// No instantaneous test separates those fixes: the receiver's own accuracy,
// satellites and PDOP, a recent-high-g window and a 1 s GNSS-vs-IMU velocity
// check each catch 23-57 % of them.  The error is a smooth, self-consistent
// lag.  Rolly Polly III's GNSS apogee voter fired 7.5 s before apogee (#242),
// and the flown filters were dragged by up to 23 m/s.  In the descent the same
// receivers agree with the barometer to 0.3-2 m/s.
//
// THE RULE.  From launch, the receiver's altitude and vertical velocity are
// held out of the EKF, and GNSS out of the apogee vote.  They are admitted —
// once, for the rest of the flight — after burnout, when its altitude has
// agreed with the barometer for a sustained second:
//
//     | (gnss_agl - pad_offset) - baro_agl |  <=  max(15 m, 10 % of |baro_agl|)
//
//   * pad_offset is GNSS AGL minus baro AGL averaged on the pad.  The GNSS
//     datum freezes 2 min after the first fix and the pad altitude wanders
//     after it: the V9 nosecone sat 23 m off the barometer at launch with
//     both sensors healthy, which says nothing about the ascent.
//   * The 10 % is the barometer's own error, not the receiver's.  ISA assumes
//     15 C; fitted over the descents, where both sensors are good, GNSS sits
//     +2..+11 % above the barometer on summer days and 3-17 % below it in
//     March and May.  A fixed band holds a healthy receiver out for most of
//     the descent on a 600 m flight (RIM-66 05-17: 27 s past apogee).
//   * Held for a second of fixes with no gap over 0.5 s: Rolly Polly III's
//     altitude swept THROUGH the barometer on its way from 300 m low to 40 m
//     high, and a single fix inside the band on the way through must not
//     admit it.
//
// Replayed on the historical flights: the healthy ones are admitted 1-2 s
// after burnout, 2-6 s before apogee, and within 2 m/s and 10 m of the IMU
// afterwards; Rolly Polly III is admitted 1.1 s after apogee, once its
// receiver had recovered; the Rolly Polly V nosecone 2.4 s before apogee,
// after it re-acquired.
//
// WHEN THE BAROMETER CANNOT VOUCH.
//   * Stale, outside 25-125 kPa, or in the transonic lockout: the EKF does
//     not fuse it and the GNSS vertical is held out, so the EKF's own
//     altitude is the IMU dead-reckoned from the last trusted anchor.  The
//     same band is applied against that altitude instead.
//   * Stuck — a sealed or taped static port reads fresh, in range and flat.
//     The ascent mirror of MainDeployGate's Layer 1 catches it: after burnout
//     and before apogee, a barometer flat to 1 m/s while GNSS says the
//     vehicle is climbing faster than 1 m/s, accumulated for 2 s (leaky, like
//     Layer 1).  Here the EKF's own altitude is no reference — it has been
//     fusing the stuck barometer all along — so the verdict itself admits
//     GNSS, and the EKF stops fusing the barometer while the verdict stands.
//     Without this, a sealed port would leave only the pitch voter standing:
//     no apogee, no drogue, no main.  A receiver lag reads LOW on a climb, so
//     it cannot fake the barometer being flat while GNSS climbs.
//
// NOT DONE, deliberately:
//   * The horizontal is NOT held out.  The corruption measured is vertical,
//     and with the IMU alone the horizontal velocity drifted a median 14 m/s
//     (up to 37) by the time the vertical was admitted — against 0.6-6.4 m/s
//     for the GNSS-aided filters that flew on 2026-08-29.  The owner chose
//     the vertical-only hold, 2026-09-26.
//   * The vertical velocity is admitted with the altitude, not before it.
//     On Rolly Polly III the receiver's velocity had recovered ~4 s before
//     its altitude converged, and a replay held out that long ran the
//     filter's climb rate 3-4 m/s low late in the coast (vs 1-2 m/s high
//     with GNSS fused).  Admitting the velocity on its own agreement first
//     is the refinement if that matters; on healthy flights, admitted 1-2 s
//     after burnout, it does not.
//   * No re-qualification after an in-flight outage once admitted, and no
//     re-arming for a second burn.  The rule is for the ascent.
//   * MainDeployGate's backstop keeps reading GNSS directly.  A barometer
//     dead from the pad is the case that backstop exists for, and GNSS never
//     being admitted there must not take the backstop with it.
//
// Pure and host-testable (tests_cpp/test_gnss_ascent_gate.cpp).  Split like
// MainDeployGate: one mutating step() per flight-loop tick, const readers for
// the EKF (GpsInsEKF::setGnssVerticalHeldOut), the barometer fusion and the
// apogee vote.

#include <math.h>
#include <stdint.h>

namespace GnssAscentGate {

enum class Phase : uint8_t {
    Pad      = 1,   // not flying: GNSS feeds the EKF exactly as before
    HeldOut  = 2,   // flying, vertical not yet admitted
    Admitted = 3,   // flying, admitted — latched until the next launch/reset
};

enum class Reason : uint8_t {
    None         = 0,
    BaroAgreed   = 1,   // altitude agreed with the barometer
    FilterAgreed = 2,   // barometer could not vouch; agreed with the EKF
    BaroStuck    = 3,   // barometer judged stuck; GNSS is the only reference
};

struct Config {
    float    agree_floor_m    = 15.0f;   // band floor
    float    agree_frac       = 0.10f;   // band grows with |reference AGL|
    uint32_t agree_dwell_ms   = 1000;    // agreement held this long
    uint32_t max_fix_gap_ms   = 500;     // a longer gap between fixes restarts it
    float    pad_offset_tau_s = 5.0f;    // pad offset EMA time constant
    // Stuck-barometer mirror of MainDeployGate Layer 1 (same bars).
    float    stuck_baro_mps   = 1.0f;
    float    stuck_gnss_mps   = 1.0f;
    uint32_t stuck_dwell_ms   = 2000;
};

struct Inputs {
    uint32_t now_ms        = 0;
    bool     in_flight     = false;  // rocket_state == INFLIGHT
    bool     burnout       = false;  // burnout_detected
    bool     apogee        = false;  // an apogee has been declared
    // GNSS: the receiver's latest record, whatever the EKF does with it.
    bool     gnss_fix_ok   = false;  // 3-D, enough satellites, fresh
    uint32_t gnss_fix_id   = 0;      // changes once per fix; 0 = none
    float    gnss_agl_m    = 0.0f;   // alt - ref_alt_m (pad datum)
    float    gnss_vel_u_mps = 0.0f;  // up-positive
    // Barometer.
    bool     baro_healthy  = false;  // fresh and 25-125 kPa (#257 predicate)
    bool     baro_locked   = false;  // transonic lockout
    float    baro_agl_m    = 0.0f;   // pressure altitude above the pad
    float    baro_rate_mps = 0.0f;   // baro KF rate, up-positive
    // The EKF's own altitude above the same datum as gnss_agl_m.
    bool     ekf_valid     = false;
    float    ekf_agl_m     = 0.0f;
};

struct State {
    Phase    phase            = Phase::Pad;
    Reason   reason           = Reason::None;
    // GNSS-minus-baro on the pad (EMA), frozen at launch.
    bool     pad_offset_valid = false;
    float    pad_offset_m     = 0.0f;
    uint32_t pad_last_ms      = 0;
    // Agreement dwell.
    bool     agreeing         = false;
    bool     agree_on_baro    = false;   // which reference the run is against
    uint32_t agree_since_ms   = 0;
    bool     have_fix         = false;
    uint32_t last_fix_id      = 0;
    uint32_t last_fix_ms      = 0;
    // Stuck-barometer accumulator (leaky).
    int32_t  stuck_ms         = 0;
    bool     baro_stuck       = false;
    // Timing.
    bool     seeded           = false;
    uint32_t last_step_ms     = 0;
    // For the log line.
    uint32_t admitted_ms      = 0;
    float    admitted_diff_m  = 0.0f;
    float    admitted_band_m  = 0.0f;
};

// Boot, sim reset, mag-cal: back on the pad.
inline void reset(State& st)
{
    st = State{};
}

// Launch: freeze the pad offset and hold GNSS out.
inline void onLaunch(State& st, uint32_t now_ms)
{
    st.phase          = Phase::HeldOut;
    st.reason         = Reason::None;
    st.agreeing       = false;
    st.stuck_ms       = 0;
    st.baro_stuck     = false;
    st.admitted_ms    = 0;
    st.last_step_ms   = now_ms;
    st.seeded         = true;
}

// May GNSS altitude and vertical velocity feed the EKF this tick?  (The
// horizontal always may.)
inline bool gnssVerticalToEkf(const State& st) { return st.phase != Phase::HeldOut; }

// May GNSS cast an apogee vote?  The vote only runs after burnout anyway.
inline bool gnssVotes(const State& st) { return st.phase != Phase::HeldOut; }

// May the EKF fuse the barometer?  The caller keeps its transonic lockout.
inline bool baroFusable(const State& st, bool baro_healthy)
{
    return baro_healthy && !st.baro_stuck;
}

inline float band(float ref_agl_m, const Config& cfg)
{
    const float scaled = cfg.agree_frac * fabsf(ref_agl_m);
    return scaled > cfg.agree_floor_m ? scaled : cfg.agree_floor_m;
}

// Once per flight-loop tick.
inline void step(State& st, const Inputs& in, const Config& cfg = Config{})
{
    if (!st.seeded)
    {
        st.seeded       = true;
        st.last_step_ms = in.now_ms;
    }
    const uint32_t dt_ms = (uint32_t)(in.now_ms - st.last_step_ms);
    st.last_step_ms = in.now_ms;

    const bool new_fix = in.gnss_fix_ok && in.gnss_fix_id != 0 &&
                         (!st.have_fix || in.gnss_fix_id != st.last_fix_id);

    if (st.phase == Phase::Pad)
    {
        // Learn GNSS-minus-baro while both are good and the vehicle is still.
        if (!in.in_flight && new_fix && in.baro_healthy)
        {
            const float d = in.gnss_agl_m - in.baro_agl_m;
            if (!st.pad_offset_valid)
            {
                st.pad_offset_m     = d;
                st.pad_offset_valid = true;
            }
            else
            {
                float a = (float)(in.now_ms - st.pad_last_ms) * 1e-3f /
                          cfg.pad_offset_tau_s;
                if (a > 1.0f) a = 1.0f;
                st.pad_offset_m += a * (d - st.pad_offset_m);
            }
            st.pad_last_ms = in.now_ms;
        }
        if (new_fix)
        {
            st.have_fix    = true;
            st.last_fix_id = in.gnss_fix_id;
            st.last_fix_ms = in.now_ms;
        }
        st.stuck_ms   = 0;
        st.baro_stuck = false;
        return;
    }

    // --- Flying: the stuck-barometer mirror runs whether or not admitted, so
    // the EKF stops fusing a stuck barometer for the rest of the climb.
    const bool disagree = in.burnout && !in.apogee && in.baro_healthy &&
                          in.gnss_fix_ok &&
                          fabsf(in.baro_rate_mps) < cfg.stuck_baro_mps &&
                          in.gnss_vel_u_mps > cfg.stuck_gnss_mps;
    if (disagree)
    {
        st.stuck_ms += (int32_t)dt_ms;
        if (st.stuck_ms >= (int32_t)cfg.stuck_dwell_ms)
        {
            st.stuck_ms   = (int32_t)cfg.stuck_dwell_ms;
            st.baro_stuck = true;
        }
    }
    else
    {
        st.stuck_ms -= (int32_t)dt_ms;
        if (st.stuck_ms <= 0)
        {
            st.stuck_ms   = 0;
            st.baro_stuck = false;   // leaky, not latched
        }
    }

    if (st.phase == Phase::Admitted) return;

    // --- Held out.
    auto admit = [&](Reason why, float diff_m, float band_m) {
        st.phase           = Phase::Admitted;
        st.reason          = why;
        st.admitted_ms     = in.now_ms;
        st.admitted_diff_m = diff_m;
        st.admitted_band_m = band_m;
        st.agreeing        = false;
    };

    if (!(in.burnout || in.apogee))
    {
        st.agreeing = false;
        if (new_fix)
        {
            st.have_fix    = true;
            st.last_fix_id = in.gnss_fix_id;
            st.last_fix_ms = in.now_ms;
        }
        return;
    }

    if (st.baro_stuck && in.gnss_fix_ok)
    {
        admit(Reason::BaroStuck, 0.0f, 0.0f);
        return;
    }

    if (!new_fix)
    {
        if (st.agreeing && (uint32_t)(in.now_ms - st.last_fix_ms) > cfg.max_fix_gap_ms)
            st.agreeing = false;
        return;
    }

    const bool contiguous = st.have_fix &&
        (uint32_t)(in.now_ms - st.last_fix_ms) <= cfg.max_fix_gap_ms;
    st.have_fix    = true;
    st.last_fix_id = in.gnss_fix_id;
    st.last_fix_ms = in.now_ms;

    // The reference: the barometer when it can vouch, else the EKF's own
    // altitude (IMU-carried: GNSS is held out and the barometer not fused).
    const bool on_baro = in.baro_healthy && !in.baro_locked && !st.baro_stuck;
    if (!on_baro && !in.ekf_valid)
    {
        st.agreeing = false;
        return;
    }
    const float ref    = on_baro ? in.baro_agl_m : in.ekf_agl_m;
    const float gnss   = in.gnss_agl_m - (st.pad_offset_valid ? st.pad_offset_m : 0.0f);
    const float diff   = gnss - ref;
    const float band_m = band(ref, cfg);

    if (!(fabsf(diff) <= band_m))           // negated: NaN disagrees
    {
        st.agreeing = false;
        return;
    }
    if (!st.agreeing || !contiguous || st.agree_on_baro != on_baro)
    {
        st.agreeing       = true;
        st.agree_on_baro  = on_baro;
        st.agree_since_ms = in.now_ms;
        return;
    }
    if ((uint32_t)(in.now_ms - st.agree_since_ms) >= cfg.agree_dwell_ms)
        admit(on_baro ? Reason::BaroAgreed : Reason::FilterAgreed, diff, band_m);
}

// ---------------------------------------------------------------------------
// Flight-snapshot byte (FlightSnapshotData::gnss_admission): whether the GNSS
// vertical is in the EKF yet.  0 is what every older writer left in that pad
// byte and means "not recorded".
//   low nibble: 1 pad, 2 held out, 3 admitted (baro agreed),
//               4 admitted (filter agreed), 5 admitted (barometer stuck)
//   bit 7:      the barometer is judged stuck now (EKF not fusing it)
// ---------------------------------------------------------------------------
static constexpr uint8_t CODE_PAD          = 1;
static constexpr uint8_t CODE_HELD_OUT     = 2;
static constexpr uint8_t CODE_BARO_AGREED  = 3;
static constexpr uint8_t CODE_EKF_AGREED   = 4;
static constexpr uint8_t CODE_BARO_STUCK   = 5;
static constexpr uint8_t CODE_STUCK_BIT    = 0x80;

inline uint8_t encode(const State& st)
{
    uint8_t c = CODE_PAD;
    if (st.phase == Phase::HeldOut) c = CODE_HELD_OUT;
    else if (st.phase == Phase::Admitted)
    {
        c = (st.reason == Reason::FilterAgreed) ? CODE_EKF_AGREED
          : (st.reason == Reason::BaroStuck)    ? CODE_BARO_STUCK
          :                                       CODE_BARO_AGREED;
    }
    if (st.baro_stuck) c |= CODE_STUCK_BIT;
    return c;
}

// Mid-flight reboot: an admission survives the reboot; anything else —
// including 0 from an older writer — has to qualify again.  The pad offset
// is not in the snapshot, so a re-qualification runs without it.
inline void restore(State& st, uint8_t code, uint32_t now_ms)
{
    reset(st);
    onLaunch(st, now_ms);
    switch (code & 0x0F)
    {
        case CODE_BARO_AGREED: st.phase = Phase::Admitted; st.reason = Reason::BaroAgreed;   break;
        case CODE_EKF_AGREED:  st.phase = Phase::Admitted; st.reason = Reason::FilterAgreed; break;
        case CODE_BARO_STUCK:  st.phase = Phase::Admitted; st.reason = Reason::BaroStuck;    break;
        default: break;   // held out
    }
    if (st.phase == Phase::Admitted) st.admitted_ms = now_ms;
}

}  // namespace GnssAscentGate
