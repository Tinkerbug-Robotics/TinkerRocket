#pragma once

#include <stdint.h>

// #1553: drop arm consent when the pack current says an e-match or its harness
// has shorted mid-pulse.
//
// Both the Tinker-Mantis V10 and the Tinker-Beetle fire straight from the
// pack, and nothing in the fire loop limits current: J8 -> Q11 -> R72 (the
// 2 mOhm INA230 shunt) -> VBAT_CON -> channel FET -> J2 -> e-match -> PYRO_GND
// -> U9 -> GND. A short during the 200 ms pulse is roughly 73-130 mOhm of loop,
// 60-115 A on a 2S pack, against a 6.5-9.6 A normal fire. That is past the
// single-pulse IDM of Q11 (-66 A) and the channel FET (-70 A), and Q11 and R72
// carry the whole board's supply: if either fails open, every later channel,
// the main included, is lost (layout review 2026-09-29, item D1). A shorted
// harness reads like a good e-match on the continuity sense, so nothing warns
// beforehand. The fuse is hardware (#1554); this is the firmware half, and it
// works on the boards as drawn.
//
// The OC already owns both ends. It reads the INA230 on R72, and U9 conducts
// only while OC_ARM_EN is high (arm_consent_policy.h), so dropping consent
// opens the fire loop whatever the FC is doing. While consent is up the INA230
// runs its fastest configuration with the shunt over-limit (SOL) alert armed:
//
//   - Mantis V10: INA_ALERT drives an S3 pin, and its falling edge drops
//     OC_ARM_EN from the ISR. Latency is one or two 140 us conversions plus
//     the ISR, then U9's gate discharge through R22 (100 k, 0.2-0.5 ms in its
//     linear region; a smaller R22 is part of #1554).
//   - Beetle: U23's ALERT pin is not connected, so a task polls the shunt
//     register every tick (1 ms) while consent is up. Slower; the next Beetle
//     revision routes ALERT (#1554).
//
// After a trip consent stays low for kHoldOffMs, which outlasts the FC's
// settle + pulse on the shorted channel. The FC then marks that channel Done
// and never retries it, so re-arming cannot fire into the same short; it can
// only let a later channel (the main) fire. The trip never re-arms while the
// pack still reads over the limit.
//
// ### The threshold, and what it costs (owner decision, 2026-10-04) ###
// 30 A, below the INA230's 40.96 A clip at R72 = 2 mOhm, so a hard short
// (which clips) always trips. The FC CAN pulse several channels at once: every
// channel whose trigger is met on the same pass moves to Firing together
// (servicePyroChannels), so two channels with the same apogee delay fire as a
// pair. Worst-case legitimate draw is N x 9.6 A + ~5 A of servo, camera and
// logic: 2 channels ~24 A clears 30 A, 3 channels ~34 A does NOT. A trip during
// a legitimate fire drops consent mid-pulse and loses those channels, so the
// OC warns when a pyro config can put 3 or more channels in Firing together
// (maxConcurrentChannels() below, "pfo" in telemetry). The warning is advisory:
// it refuses nothing.
//
// ### A missing measurement never holds consent low ###
// The trip is protection layered on the arm, not part of it. An INA230 that is
// absent, or stops answering, leaves consent exactly where
// arm_consent_policy.h puts it: losing a deployment to a dead current monitor
// is worse than the fault this guards against. For the same reason a trip
// re-arms on time when the reading after it is unavailable.
//
// ### The alert pin is shared with #1409 ###
// The INA230 runs one alert function at a time. #1409 plans a brownout warning
// on the same INA_ALERT pin; while consent is up SOL owns it, and #1409's
// function may run only while consent is down (alertOwner()).
//
// Pure so the decision table is host-testable (test_pyro_fault_trip_policy).
// main.cpp owns the pin, the INA230 and the task that polls it.

namespace PyroFaultTrip {

// ---- Scale ------------------------------------------------------------------

// INA230 shunt-voltage register: 2.5 uV per count, +/-32767 counts
// (+/-81.92 mV). R72 = 2 mOhm, so 800 counts per amp and +/-40.96 A full scale.
inline constexpr uint32_t kShuntMicroOhm     = 2000;
inline constexpr uint32_t kShuntNanoVPerLsb  = 2500;
inline constexpr int32_t  kCountsPerAmp      =
    (int32_t)(kShuntMicroOhm * 1000U / kShuntNanoVPerLsb);
static_assert(kCountsPerAmp == 800, "R72 = 2 mOhm at 2.5 uV/count is 800 counts/A");

inline constexpr int32_t  kTripThresholdA    = 30;
inline constexpr int16_t  kTripLimitCounts   = (int16_t)(kTripThresholdA * kCountsPerAmp);
static_assert(kTripLimitCounts == 24000, "30 A at 800 counts/A");
static_assert(kTripLimitCounts < 32767,
              "the limit must sit below the clip, or a hard short (which clips) never trips");

// What a legitimate pulse can draw, for the concurrency warning: the bench's
// worst e-match (6.5-9.6 A at 8.4 V) and the rest of the board with consent up.
inline constexpr int32_t  kChannelDrawMilliA = 9600;
inline constexpr int32_t  kBoardDrawMilliA   = 5000;
// The most channels the threshold covers firing at once.
inline constexpr uint8_t  kMaxConcurrentChannels = 2;
static_assert(kMaxConcurrentChannels * kChannelDrawMilliA + kBoardDrawMilliA
                  < kTripThresholdA * 1000,
              "the threshold must clear kMaxConcurrentChannels firing together");
static_assert((kMaxConcurrentChannels + 1) * kChannelDrawMilliA + kBoardDrawMilliA
                  >= kTripThresholdA * 1000,
              "kMaxConcurrentChannels is understated: the threshold clears one more");

// ---- Timing -----------------------------------------------------------------

// The FC's fire timing (flight_computer/main/config.h). test_pyro_fault_trip_
// policy pins these against that header, so a change there fails the build of
// the tests rather than shortening the hold-off silently.
inline constexpr uint32_t kFcArmSettleMs = 10;    // PYRO_ARM_SETTLE_MS
inline constexpr uint32_t kFcFireMs      = 200;   // PYRO_FIRE_DURATION_MS
// Consent stays low this long after a trip: the shorted channel's whole
// settle + pulse, wherever in it the trip landed, plus margin for the FC's
// loop and the OC's.
inline constexpr uint32_t kHoldOffMs     = kFcArmSettleMs + kFcFireMs + 40;
static_assert(kHoldOffMs == 250, "the issue's ~250 ms");

// ---- INA230 configuration -----------------------------------------------------

// Configuration register words (datasheet Table 7-4): AVG [11:9],
// VBUSCT [8:6], VSHCT [5:3], MODE [2:0]. Written whole with writeRegister.
//
//   Guard    — consent up: 1 sample, 140 us shunt + 140 us bus, continuous.
//              A fresh shunt sample every 280 us; the bus keeps converting so
//              telemetry keeps its pack voltage through a flight.
//   RailOn   — the #1485 rail-on config: 1 sample, 332 us + 332 us.
//   LowPower — rail off: 1024-sample averaging at 332 us + 332 us.
enum class InaConfig : uint8_t
{
    LowPower = 0,
    RailOn   = 1,
    Guard    = 2,
};

inline constexpr uint16_t kCfgGuard    = (0u << 9) | (0u << 6) | (0u << 3) | 0x7u;
inline constexpr uint16_t kCfgRailOn   = (0u << 9) | (2u << 6) | (2u << 3) | 0x7u;
inline constexpr uint16_t kCfgLowPower = (7u << 9) | (2u << 6) | (2u << 3) | 0x7u;

inline InaConfig inaConfig(bool consent_up, bool rail_on)
{
    if (consent_up) return InaConfig::Guard;   // whatever the rail is doing
    return rail_on ? InaConfig::RailOn : InaConfig::LowPower;
}

inline uint16_t configWord(InaConfig c)
{
    switch (c)
    {
    case InaConfig::Guard:  return kCfgGuard;
    case InaConfig::RailOn: return kCfgRailOn;
    default:                return kCfgLowPower;
    }
}

// ---- Alert pin ownership (shared with #1409) ----------------------------------

enum class AlertOwner : uint8_t
{
    None     = 0,
    Sol      = 1,   // this file: shunt over-limit, while consent is up
    Brownout = 2,   // #1409: its function, only while consent is down
};

inline AlertOwner alertOwner(bool consent_up, bool brownout_wanted)
{
    if (consent_up)      return AlertOwner::Sol;
    if (brownout_wanted) return AlertOwner::Brownout;
    return AlertOwner::None;
}

// Mask/Enable bits (datasheet Table 7-9).
inline constexpr uint16_t kMeSol  = 1u << 15;   // shunt over-limit
inline constexpr uint16_t kMeCnvr = 1u << 10;   // conversion ready on the ALERT pin

// The Mask/Enable word for SOL ownership, and the Alert Limit word.
//
// CNVR also drives the ALERT pin, once per conversion. Where the pin reaches
// an S3 (alert_pin_wired) it must be off, or every conversion reads as a trip.
// Where it does not (the Beetle) keep it on: it is what the rail-on read path
// has always run with. The CVRF flag (bit 3) that path polls does not depend
// on it (datasheet: CVRF is set after every conversion; CNVR only routes it to
// the pin) — the V10 bench confirms that before its first flight.
//
// Transparent alert (LEN = 0): the pin follows each conversion, so it releases
// on its own once consent is down and the current has gone, and the 100 Hz
// Mask/Enable read for CVRF cannot clear a trip by reading it.
inline uint16_t solMaskEnable(bool alert_pin_wired)
{
    return alert_pin_wired ? kMeSol : (uint16_t)(kMeSol | kMeCnvr);
}
inline constexpr uint16_t kSolAlertLimit = (uint16_t)kTripLimitCounts;

// ---- The trip -----------------------------------------------------------------

// Where the over-limit was seen.
enum class Source : uint8_t
{
    Poll  = 1,   // the shunt-register poll (every board with consent)
    Alert = 2,   // the INA_ALERT edge (boards with the pin wired)
};

inline bool overLimit(int16_t shunt_counts)
{
    return shunt_counts >= kTripLimitCounts;
}

// One trip, as it is logged and reported.
struct Event
{
    uint32_t at_ms       = 0;
    int16_t  peak_counts = 0;   // highest shunt reading seen during the hold-off
    uint8_t  reason      = 0;   // ArmConsentPolicy::Reason that was up
    uint8_t  source      = 0;   // Source
};

struct State
{
    bool     holding = false;   // consent forced low
    Event    current;           // the trip being held, valid while holding
    Event    last;              // the most recent trip, valid once trips > 0
    uint16_t trips   = 0;       // this boot
};

// An over-limit was seen at now_ms with consent up for `reason`. Returns true
// when it starts a trip; an over-limit seen while already holding only raises
// the recorded peak.
inline bool onOverLimit(State& s, uint32_t now_ms, int16_t counts,
                        uint8_t reason, Source src)
{
    if (s.holding)
    {
        if (counts > s.current.peak_counts) s.current.peak_counts = counts;
        return false;
    }
    s.holding             = true;
    s.current.at_ms       = now_ms;
    s.current.peak_counts = counts;
    s.current.reason      = reason;
    s.current.source      = (uint8_t)src;
    if (s.trips < 0xFFFF) s.trips++;
    s.last = s.current;
    return true;
}

// A shunt reading taken while holding: keeps the logged peak honest (an
// alert-pin trip starts from the limit, not from the reading).
inline void notePeak(State& s, int16_t counts)
{
    if (s.holding && counts > s.current.peak_counts)
    {
        s.current.peak_counts = counts;
        s.last.peak_counts    = counts;
    }
}

// The latest shunt reading, as the re-arm sees it.
struct Reading
{
    bool    valid  = false;   // false = the INA230 did not answer
    int16_t counts = 0;
};

// Does the trip still hold consent low at now_ms? Releases once kHoldOffMs has
// passed AND the latest reading is not over the limit; an unavailable reading
// does not hold it (see the file header). Releasing clears `holding`, which is
// also what keeps the deadline wrap-safe.
inline bool stillHolding(State& s, uint32_t now_ms, Reading latest)
{
    if (!s.holding) return false;
    if ((now_ms - s.current.at_ms) < kHoldOffMs) return true;
    if (latest.valid && overLimit(latest.counts)) return true;
    s.holding = false;
    return false;
}

// The consent pin: arm_consent_policy.h's verdict, vetoed while a trip holds.
inline bool pinHigh(bool consent_wanted, bool holding)
{
    return consent_wanted && !holding;
}

// Shunt counts as amps x10, for the log record and telemetry.
inline int16_t countsToDeciAmps(int16_t counts)
{
    return (int16_t)(((int32_t)counts * 10) / kCountsPerAmp);
}

// ---- The concurrency warning --------------------------------------------------

struct ChannelCfg
{
    bool    enabled = false;
    uint8_t mode    = 0;      // PyroTriggerMode
    float   value   = 0.0f;   // s after apogee, or m AGL on descent
};

// How far apart two triggers of the same mode can be and still put both
// channels in Firing together. Time after apogee: the pulses overlap when the
// starts are under settle + pulse apart. Altitude on descent: settle + pulse
// at up to ~100 m/s of ballistic descent is ~21 m; 25 m covers it. Mixed
// modes can coincide too, but only by the flight's luck, not by the config,
// so they are not counted.
inline constexpr float kTimeOverlapS   = (float)(kFcArmSettleMs + kFcFireMs) / 1000.0f;
inline constexpr float kAltOverlapM    = 25.0f;
inline constexpr uint8_t kModeTime     = 0;   // PYRO_TRIGGER_TIME_AFTER_APOGEE
inline constexpr uint8_t kModeAltitude = 1;   // PYRO_TRIGGER_ALTITUDE_ON_DESCENT

inline float overlapWindow(uint8_t mode)
{
    return mode == kModeAltitude ? kAltOverlapM : kTimeOverlapS;
}

// The largest number of enabled channels this config can have in Firing at
// once: for each channel, the same-mode channels whose trigger falls in
// [value, value + window). 0 when none is enabled.
inline uint8_t maxConcurrentChannels(const ChannelCfg ch[4])
{
    uint8_t best = 0;
    for (int i = 0; i < 4; ++i)
    {
        if (!ch[i].enabled) continue;
        const float w = overlapWindow(ch[i].mode);
        uint8_t n = 0;
        for (int j = 0; j < 4; ++j)
        {
            if (!ch[j].enabled || ch[j].mode != ch[i].mode) continue;
            const float d = ch[j].value - ch[i].value;
            if (d >= 0.0f && d < w) n++;
        }
        if (n > best) best = n;
    }
    return best;
}

inline bool concurrencyWarning(uint8_t max_concurrent)
{
    return max_concurrent > kMaxConcurrentChannels;
}

}  // namespace PyroFaultTrip
