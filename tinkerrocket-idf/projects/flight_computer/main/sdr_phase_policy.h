#pragma once

#include <stddef.h>
#include <stdint.h>

#include <RocketComputerTypes.h>   // RocketState, GnssClockData
#include <GnssClockRecord.h>       // TR_GNSSReceiverLC86_Serial: gnssClockGpsAt, GPS time off the board's line
#include <SdrAid.h>                // TR_GNSSReceiverSdr_Serial: TinkerRocket's message 4002

// The flight phase a flight image sends the gnss-sdr-dev board on J1
// (TinkerRocket's message 4002, sub-type 3: SdrAid.h), from the state
// machine's own flags and the IMU's own cue of an ignition.
//
// Why the board needs it: it boots with `rx aid fc` and `rx fc phase on`, so
// only this side's phase picks its boost tracking loops. That is the owner's
// decision ("the IMU is better at acceleration than GNSS"), after the rig's
// false boosts on a static pad when the board's GPS channels chose for
// themselves. With no request in force the board runs its quiet loops, and
// the rig lost Galileo and BeiDou in a boost on those. So a flight image
// sends the real phase: the boost loops from the ignition to burnout, the
// quiet ones after it, none (the board's quiet loops) on the pad and after
// apogee.
//
// The map, the P1 spec's (section 2.5), first match wins, with one line of
// its own (the ignition cue). The flight computer has no burnout or apogee
// state of its own: inside INFLIGHT two latches mark them (burnout_detected,
// pyro_apogee_detected).
//   LANDED                  -> landed, no request
//   INFLIGHT, apogee        -> descent, no request; canopy from the deployment
//                              latch or the filter's canopy mode
//   INFLIGHT, burnout       -> coast, the quiet loops
//   INFLIGHT                -> boost, the boost loops
//   a pad state, the cue    -> boost, the boost loops (INITIALIZATION, READY,
//                              PRELAUNCH: the states a launch leaves)
//   anything else           -> pad, no request (INITIALIZATION, READY,
//                              PRELAUNCH, MAG_CALIBRATION)
// The unknown phase (0) is never sent. Apogee wins over burnout: a burnout
// never seen (a stale IMU) keeps the boost until apogee, and nothing after an
// apogee asks for a boost. The board refuses a boost after a descent or a
// landing until the next pad (its guard); a coast leaves one open, for a
// second stage, but this flight computer latches one burnout a flight, so a
// second burn is reported as the coast it follows (the quiet loops).
//
// Why the cue: INFLIGHT comes only with the launch flag, which latches a
// median 60 ms and at most 265 ms after the ignition on its barometer-
// confirmed path, and 262-956 ms after it on the accel-only one (a blocked
// static port: TR_KinematicChecks.cpp). Spec 2.5 sent none on the pad because
// the board's own GPS decision would take the boost within 16 ms; rev 2.2
// turned that decision off, so without the cue the board's quiet loops would
// meet the whole ignition step, which they cannot follow (rx.h on
// rx_gps_accel). On the rig's hotshot scene the quiet loops through the
// motor's ramp (0.15-0.35 s after the ignition) lost three BeiDou satellites
// for good (p1_link_fc), and the boost loops from 77 ms after it, one
// (p1_phase_fc); a launch flag at its slow end leaves the ramp to the quiet
// loops, the accel-only one most of a short burn. The board's choice of loops
// arms nothing, so its cue needs no launch-grade confirmation: IgnitionCue
// below, the launch detector's own first bar (2 g of specific force, 1 g at
// rest) held for 15 ticks, ~15 ms. A false cue costs only the boost loops for
// as long as it holds; a static pad can never give one (the owner's
// objection to the GPS rule). Not yet measured on the rig against a launch
// flag at its real latency (the P1 script's lag at 0/60/265/600 ms). Once the
// launch
// flag latches INFLIGHT takes the boost over with the same key, so no frame
// goes for the hand-over; a cue that lapses with no launch is a change back
// to the pad, sent at once.
//
// since_ms counts from the launch flag (not the ignition), the burnout latch
// and the apogee latch, saturated at 65535, the field's "that long or
// unknown"; the pad, the cue's boost (no latch yet) and the landing have no
// start this side keeps, so 65535. Every phase holds 2.5 s (kPhaseHoldMs): a
// frame a second survives one lost frame, and a flight computer gone silent
// lets its request lapse within 2.5 s. The values are the vehicle's own: src
// kSrcOwn (the state machine runs on its IMU and baro; the board gives it no
// fix), test only while a sim or a rig replay flies (the latch the pyro
// dry-fire keys on), which the board takes only under its `rx fc test on`;
// tq 5, stamps off the board's own latch line with its output delay left in
// (2-6 ms early), 6 once that line is more than 100 s old (kLineTqFreshUs).
//
// When a frame goes (Sender): on each change of the phase, the loops, the
// canopy or the test flag at once, then three times 100 ms apart, then once a
// second (sdr_aid::AidSchedule with the phase alone), looked at every pass.
// Each is stamped in GPS time off the board's clock line at the pass that read
// the flags. Nothing goes until that line is valid (10 s of the board's
// epochs after its first fix), and the last valid one is kept (noteLine) for
// up to kLineHoldUs after it last moved, through a restart of the line and
// through a stop of the board's epochs: the board sends an epoch only with an
// observable, which a boost can take away, and its boost request would then
// lapse mid-burn and the burnout's change never go. At the 100 ppm a crystal
// can be off at most (GnssTimeSync::kMaxDrift) 600 s puts a stamp 60 ms out,
// far inside the 2.5 s hold (17 ppm measured: 10 ms; fitted: under 1 ms).
//
// Pure: the caller (main.cpp) checks that the receiver on J1 is the board,
// hands Sender each clock record the driver gives, steps IgnitionCue on each
// flight-logic tick, reads the flags each pass, writes the frame Sender gives
// it (never waiting: a frame with no room in the UART's TX ring goes on a
// later pass) and logs it. Host-tested in tests_cpp/test_sdr_aid.cpp.

namespace sdr_phase
{

// What the state machine holds at one pass.
struct Flight
{
    RocketState state = INITIALIZATION;
    bool        burnout = false;   // burnout_detected
    bool        apogee = false;    // pyro_apogee_detected
    bool        canopy = false;    // deployment_detected, or the filter's canopy mode
    bool        sim = false;       // a sim or a rig replay flies (sim_flight::simulated)
    bool        ignition = false;  // the IMU's ignition cue (IgnitionCue::on)
    uint32_t    now_ms = 0;        // this pass (time_ms())
    uint32_t    launch_ms = 0;     // launch_time_millis
    uint32_t    burnout_ms = 0;    // burnout_time_ms
    uint32_t    apogee_ms = 0;     // pyro_apogee_time_ms
};

// The IMU's own cue of an ignition, for the board's loops alone (the state
// machine, the pyro and the launch detector never see it): kCount ticks in a
// row with the specific force's magnitude over kMs2, the launch detector's
// own first bar and count unit (TR_KinematicChecks: launch_count, one a
// flight-logic tick at 1 kHz), then on for kHoldMs after the last tick of
// such a run. The hold bridges a boost's brief dips under the bar until the
// launch flag latches (at most 956 ms after the ignition, and that path
// needs 250 ticks in a row over 3 g, which refresh it throughout); a run
// that never becomes a launch lets it go kHoldMs after.
class IgnitionCue
{
public:
    static constexpr float    kMs2 = 20.0f;      // m/s^2, ~2 g; 1 g at rest
    static constexpr uint16_t kCount = 15;       // ticks in a row over it, ~15 ms
    static constexpr uint32_t kHoldMs = 1000;    // on this long after the run's last such tick

    // One flight-logic tick at now_ms (time_ms()): acc_ms2 the specific
    // force's magnitude, 0 from a stale IMU (the caller's freshness gate, as
    // the launch detector's), which breaks a run as a low one does; so does a
    // NaN (it compares false).
    void step(uint32_t now_ms, float acc_ms2)
    {
        if (acc_ms2 > kMs2)
        {
            if (run_ < kCount) run_++;
            if (run_ >= kCount)
            {
                on_ = true;
                last_ms_ = now_ms;
                return;
            }
        }
        else
        {
            run_ = 0;
        }
        if (on_ && now_ms - last_ms_ >= kHoldMs) on_ = false;   // (cleared, so 32-bit ms never wrap it back on)
    }

    // Whether the cue holds at now_ms. Not stepped (MAG_CALIBRATION skips the
    // flight-logic's checks), it lapses all the same.
    bool on(uint32_t now_ms) const { return on_ && now_ms - last_ms_ < kHoldMs; }

private:
    uint16_t run_ = 0;       // ticks in a row over kMs2, up to kCount
    bool     on_ = false;
    uint32_t last_ms_ = 0;   // the last tick a run of kCount was seen
};

// ms from from_ms to now_ms, saturated at 65535; 0 for a start not yet
// passed (never the case for a latch this side set, but never a wrap
// either).
inline uint32_t sinceMs(uint32_t now_ms, uint32_t from_ms)
{
    const int32_t d = (int32_t)(now_ms - from_ms);
    if (d <= 0) return 0;
    return d > 65535 ? 65535u : (uint32_t)d;
}

// The phase message for these flags; its stamp's time fields and seq are
// left to stamped(), and its tq is the fresh line's (Sender lowers it for a
// held one).
inline sdr_aid::PhaseMsg phaseOf(const Flight& f)
{
    sdr_aid::PhaseMsg p;
    p.h.src = sdr_aid::kSrcOwn;
    p.h.test = f.sim;
    p.h.tq = sdr_aid::kLineTq;
    p.hold_ms = sdr_aid::kPhaseHoldMs;
    p.canopy = false;
    p.since_ms = 65535;
    if (f.state == LANDED)
    {
        p.phase = sdr_aid::kPhLanded;
        p.loops = sdr_aid::kLoopsNone;
    }
    else if (f.state == INFLIGHT && f.apogee)
    {
        p.phase = sdr_aid::kPhDescent;
        p.loops = sdr_aid::kLoopsNone;
        p.canopy = f.canopy;
        p.since_ms = sinceMs(f.now_ms, f.apogee_ms);
    }
    else if (f.state == INFLIGHT && f.burnout)
    {
        p.phase = sdr_aid::kPhCoast;
        p.loops = sdr_aid::kLoopsQuiet;
        p.since_ms = sinceMs(f.now_ms, f.burnout_ms);
    }
    else if (f.state == INFLIGHT)
    {
        p.phase = sdr_aid::kPhBoost;
        p.loops = sdr_aid::kLoopsBoost;
        p.since_ms = sinceMs(f.now_ms, f.launch_ms);
    }
    else if (f.ignition && (f.state == INITIALIZATION || f.state == READY || f.state == PRELAUNCH))
    {
        // The cue, before the launch flag: no latch to count from (65535)
        p.phase = sdr_aid::kPhBoost;
        p.loops = sdr_aid::kLoopsBoost;
    }
    else
    {
        p.phase = sdr_aid::kPhPad;
        p.loops = sdr_aid::kLoopsNone;
    }
    return p;
}

// The message stamped for the wire: the GPS instant its flags were read
// (continuous seconds, week * 604800 + time of week, off the board's clock
// line) to the nearest ms with the rest in ns, and the phase frames' count.
inline sdr_aid::PhaseMsg stamped(sdr_aid::PhaseMsg p, double gps_s, uint8_t seq)
{
    const sdr_aid::Header t = sdr_aid::gpsStamp(gps_s);
    p.h.week = t.week;
    p.h.tow_ms = t.tow_ms;
    p.h.sub_ns = t.sub_ns;
    p.h.seq = seq;
    return p;
}

// How long the last valid line stays good after it last moved, us: the
// flight computer's longest flight (main.cpp MAX_FLIGHT_TIME_MS, 600 s, after
// which it calls the flight landed).
static constexpr int64_t kLineHoldUs = 600000000;
// How long a held line's stamps keep tq 5 (under 16.4 ms), us: the line's own
// 2-6 ms plus a crystal at GnssTimeSync::kMaxDrift (100 ppm) for 100 s is
// 16 ms. After that tq 6 (under 65.5 ms), the 600 s hold's worst case. The
// board reads no tq off a phase today (rx_in.c takes its phase, loops and
// hold); P2/P3 will weigh stamps by it.
static constexpr int64_t kLineTqFreshUs = 100000000;
static constexpr uint8_t kHeldLineTq = 6;

// The flight image's phase sender, one call a pass: noteLine() is handed each
// clock record the driver gives, frame() gives the frame due now, if any, and
// sent() is told when it went. A frame that did not go (no room in the TX
// ring) is given again on the next pass, freshly stamped.
class Sender
{
public:
    // A clock record as the driver handed it, at now_us (esp_timer, us; 64
    // bits, so an old line never looks fresh again when 32-bit micros wrap).
    // Only a valid one (GCL_VALID) is kept, with when it came: a restart of
    // the line (five epochs off it) sends records that are not, for the 10 s
    // it takes to be valid again, and the phase goes on off the last valid
    // line through them (a bench image takes every record: not this).
    void noteLine(const GnssClockData& rec, int64_t now_us)
    {
        if ((rec.flags & GCL_VALID) == 0)
            return;
        line_ = rec;
        line_at_us_ = now_us;
    }

    // The phase frame to send now into out (cap at least
    // sdr_aid::kPhaseFrame): its length, 0 for none. Nothing without a valid
    // line noted within kLineHoldUs.
    size_t frame(int64_t now_us, const Flight& f, uint8_t* out, size_t cap)
    {
        const int64_t age_us = now_us - line_at_us_;
        if ((line_.flags & GCL_VALID) == 0 || age_us >= kLineHoldUs)
            return 0;
        sdr_aid::PhaseMsg p = phaseOf(f);
        const uint32_t key = sdr_aid::phaseKey(p);
        if (sched_.due(now_us, key) != sdr_aid::kPhase)
            return 0;
        double gps_s = 0.0;
        if (!lc86::gnssClockGpsAt(line_, now_us, gps_s))
            return 0;
        if (age_us > kLineTqFreshUs)
            p.h.tq = kHeldLineTq;
        key_ = key;
        return sdr_aid::encode(stamped(p, gps_s, seq_), out, cap);
    }

    // The frame frame() last gave went out at now_us.
    void sent(int64_t now_us)
    {
        seq_++;
        sched_.sent(sdr_aid::kPhase, now_us, key_);
    }

    uint8_t seq() const { return seq_; }   // the next frame's

private:
    sdr_aid::AidSchedule sched_{1u << sdr_aid::kPhase};
    GnssClockData line_ = {};   // the last valid line noted (flags 0: none yet)
    int64_t  line_at_us_ = 0;   // when it was noted
    uint8_t  seq_ = 0;          // phase frames sent, mod 256
    uint32_t key_ = 0;          // the key of the frame frame() last gave
};

}  // namespace sdr_phase
