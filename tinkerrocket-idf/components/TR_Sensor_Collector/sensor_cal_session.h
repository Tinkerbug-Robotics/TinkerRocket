// sensor_cal_session.h — the request/answer handshake around the pad
// calibration window (#1114).
//
// The IMU poll task measures the calibration (#1110).  Since #1114 the flight
// task no longer waits for it: it posts a request, keeps flying, and polls for
// the answer once per loop pass — so launch detection, the EKF, the state
// machine and the deployment channels all keep running through the ~10 s
// window, and a launch inside the window simply cancels the calibration.
//
// One sequence number ties a request to its answer:
//   SensorCalSession  — flight task.  Issues a seq per request, remembers the
//                       one outstanding, accepts only that answer.
//   SensorCalWindow   — poll task.  Opens a window when it sees a seq it has
//                       not acted on, restarts on a newer one, closes at the
//                       deadline and reports which seq it answered.
// A cancel just forgets the outstanding seq; the poll task finishes its window
// anyway and the answer is dropped because nobody is waiting for it.
//
// No ESP-IDF dependencies on purpose: tests_cpp drives both halves on the
// host.  The semaphore that carries the answer between the two tasks is the
// one part that needs a rocket.
#pragma once

#include <cstdint>
#include <RocketComputerTypes.h>   // RocketState

struct SensorCalSession
{
    uint32_t last_seq = 0;   // last seq issued
    uint32_t wait_seq = 0;   // seq outstanding; 0 = none

    bool waiting() const { return wait_seq != 0; }

    // Issue a request.  Returns its seq (never 0), or 0 when one is already
    // outstanding — the caller keeps the running window rather than restart.
    uint32_t start()
    {
        if (wait_seq != 0) return 0;
        if (++last_seq == 0) ++last_seq;   // 0 is reserved for "none"
        wait_seq = last_seq;
        return wait_seq;
    }

    // Forget the outstanding request.  Its answer, if it ever lands, is
    // refused by accept().
    void cancel() { wait_seq = 0; }

    // True exactly once, for the answer to the outstanding request.  Any
    // other answer — a window cancelled earlier, or nothing outstanding —
    // is refused and leaves the session as it was.
    bool accept(uint32_t done_seq)
    {
        if (wait_seq == 0 || done_seq != wait_seq) return false;
        wait_seq = 0;
        return true;
    }
};

struct SensorCalWindow
{
    uint32_t seen_seq    = 0;      // latest request acted on
    uint32_t seq         = 0;      // request this window answers
    bool     active      = false;
    uint32_t deadline_us = 0;

    // Once per poll wake.  Opens the window for a request newer than the
    // last one acted on — restarting an open window if the request changed
    // under it.  Returns true when a window opened, so the caller zeroes
    // its sums.
    bool open(uint32_t request_seq, uint32_t now_us, uint32_t window_us)
    {
        if (request_seq == seen_seq) return false;
        seen_seq    = request_seq;
        seq         = request_seq;
        active      = true;
        deadline_us = now_us + window_us;
        return true;
    }

    // Once per poll wake, after open().  Closes the window once its deadline
    // has passed; true exactly once per window, with the seq to report.
    // Unsigned wrap-safe like every other deadline in the poll task.
    bool close(uint32_t now_us, uint32_t& done_seq)
    {
        if (!active || (int32_t)(now_us - deadline_us) < 0) return false;
        active   = false;
        done_seq = seq;
        return true;
    }
};

// What one pollCalibration() call reports.
enum class SensorCalPoll : uint8_t
{
    Idle,        // nothing outstanding
    Running,     // window open — keep flying
    Committed,   // the window closed and its result passed the gate: new offsets live
    Rejected     // the window closed with nothing usable, or the poll task never
                 // answered: previous calibration kept
};

namespace sensor_cal
{
    // Why a pad calibration must not run now, or nullptr when the rocket is
    // on the pad.  Asked at the request and again once per pass while a
    // window is open (a non-null answer then cancels it).  launch_flag is the
    // detector's own verdict, ahead of the state machine by a pass; INFLIGHT
    // and LANDED cover a launch it did not call — a sim, a restored flight.
    // Every other state (INITIALIZATION, READY, PRELAUNCH, MAG_CALIBRATION)
    // is a rocket on the pad or the bench: outdoors the FC auto-promotes to
    // PRELAUNCH within seconds of GNSS lock, so a READY-only gate would
    // refuse the on-pad calibration the app exists to run.
    inline const char* refusal(RocketState state, bool launch_flag)
    {
        if (launch_flag)       return "launch detected";
        if (state == INFLIGHT) return "INFLIGHT";
        if (state == LANDED)   return "LANDED";
        return nullptr;
    }
}
