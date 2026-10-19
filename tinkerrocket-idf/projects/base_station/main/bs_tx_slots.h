#pragma once

// Base-station transmit-slot controller (three-rocket support).
//
// Every rocket transmits a frame every 500 ms on its own free-running clock. One
// FAST frame is 102 ms on air (SF8/BW250/CR4-5, preamble 12), so three rockets
// on one channel collide whenever two of them drift within 102 ms of each other
// — in streaks, as their clocks slide past one another. The fix is a TDMA grid:
// slot k of 3 starts k * 500/3 ms after slot 0, and a FAST frame plus a 30 ms
// guard fits in each 166.7 ms slot.
//
// The BASE STATION runs the grid, not the rockets (a design review rejected
// rockets aligning to each other: two that cannot hear each other deaf-lock).
// It times each rocket's frames on its own clock and tells each rocket where to
// put its grid with a targeted uplink LORA_CMD_TX_SLOT_ADJUST
// [ref_seq u16][delta_ms i16]: "put your grid at (when you sent frame ref_seq)
// + delta, then every 500 ms". The BS measured that frame's on-air start, so a
// rocket's own send latency cancels; the adjust is idempotent, so a blind
// retry or a re-send is harmless.
//
// The rules (each one pinned by test_bs_tx_slots):
//   * timing: a frame's on-air start = its RX time - its time on air. Delays
//     only add (ISR latency, the rocket's loop), so the estimate is the
//     EARLIEST of the last kWindow FAST frames, compared modulo 500 ms. That
//     frame is also the ref_seq, so the rocket refers to the very frame that was
//     measured best.
//   * slot = (rid - 1) % 3. Two rockets that want one slot: the lower rid keeps
//     it, the higher one takes the lowest free slot. A fourth rocket gets none.
//     Assignments are frozen while any rocket is in flight.
//   * base phase: fixed when the first rocket is adjusted, from that rocket's
//     own timing, so it gets delta ~0 — and still gets the adjust, because a
//     free-running rocket drifts by its loop latency every period and only a
//     rocket on a grid stays where it is put.
//   * a rocket more than kTolMs (10 ms) off its target is adjusted; the next
//     check waits for kWindow FAST frames received after the adjust's retries
//     finished, and re-sends if it is still off.
//   * while any rocket is INFLIGHT only a rocket more than kFlightTolMs (50 ms)
//     off is adjusted — a reboot, not drift; the uplink costs the flight frames.
//   * never a rocket that is hopping or on another modulation (the caller's
//     eligibility), and never when a FAST frame + kGuardMs does not fit a slot
//     (SF9 and up, or CR4/8, at BW250).
//
// Pure — no ESP-IDF / radio dependencies — so the host gtest drives it directly.

#include <cstddef>
#include <cstdint>

#include "RocketComputerTypes.h"  // LORA_TX_SLOT_COUNT, loraTimeOnAirMs, SIZE_OF_LORA_FAST

namespace bs_tx_slots {

inline constexpr uint32_t kPeriodMs     = 500;
inline constexpr uint8_t  kSlots        = LORA_TX_SLOT_COUNT;   // 3
inline constexpr uint32_t kTolMs        = 10;    // ground: adjust beyond this
inline constexpr uint32_t kFlightTolMs  = 50;    // any rocket INFLIGHT: only beyond this
inline constexpr uint32_t kGuardMs      = 30;    // FAST frame + this must fit a slot
inline constexpr uint8_t  kWindow       = 4;     // FAST frames per estimate
inline constexpr uint32_t kWindowAgeMs  = 4000;  // older window frames are dropped
inline constexpr uint32_t kAwaitMaxMs   = 10000; // an adjust "in the queue" longer than this is lost

// Start of slot k relative to slot 0: 0, 167, 333 ms.
inline uint32_t slotOffsetMs(uint8_t slot)
{
    return ((uint32_t)slot * kPeriodMs + kSlots / 2) / kSlots;
}

// x wrapped into (-P/2, P/2] — (-250, 250] for the 500 ms period.
inline int32_t wrapHalf(int32_t x, uint32_t period_ms = kPeriodMs)
{
    const int32_t p = (int32_t)period_ms;
    int32_t m = x % p;
    if (m < 0) m += p;            // [0, P)
    if (m > p / 2) m -= p;        // (-P/2, P/2]
    return m;
}

// (a - b) modulo the period, into (-P/2, P/2], for two millis() stamps less
// than ~24 days apart (the difference is taken as a signed 32-bit value).
inline int32_t phaseDiff(uint32_t a, uint32_t b)
{
    return wrapHalf((int32_t)(a - b));
}

// The slot a rocket asks for. rid 0 and 0xFF are not rockets (0 = unset,
// 0xFF = broadcast) and get none.
inline int8_t preferredSlot(uint8_t rid)
{
    if (rid == 0 || rid == 0xFF) return -1;
    return (int8_t)((rid - 1) % kSlots);
}

// Does a FAST frame plus the guard fit one slot at this modulation?
inline uint32_t fastAirtimeMs(uint8_t sf, float bw_khz, uint8_t cr)
{
    return loraTimeOnAirMs(SIZE_OF_LORA_FAST, sf, bw_khz, cr, LORA_TELEM_PREAMBLE_SYMS);
}
inline bool airtimeFits(uint8_t sf, float bw_khz, uint8_t cr)
{
    // (toa + guard) <= 500 / 3, kept in integers.
    return (fastAirtimeMs(sf, bw_khz, cr) + kGuardMs) * kSlots <= kPeriodMs;
}

// ---------------------------------------------------------------------------
// Slot assignment over the tracked rockets. rids[i] == 0 is an empty entry.
// keep[i] >= 0 is a slot to hold (assignment frozen in flight): those are
// placed first, as they are; everyone else then claims a slot by the rule.
// ---------------------------------------------------------------------------
inline constexpr size_t kMaxRockets = 8;   // assignSlots capacity (the BS tracks 4)

inline void assignSlots(const uint8_t* rids, const int8_t* keep, size_t n, int8_t* out)
{
    if (n > kMaxRockets) n = kMaxRockets;
    bool taken[kSlots] = {};

    // Entries in ascending rid order (insertion sort; n is tiny).
    size_t order[kMaxRockets];
    for (size_t i = 0; i < n; i++)
    {
        size_t j = i;
        while (j > 0 && rids[order[j - 1]] > rids[i]) { order[j] = order[j - 1]; j--; }
        order[j] = i;
    }

    for (size_t i = 0; i < n; i++)
    {
        out[i] = -1;
        if (keep != nullptr && keep[i] >= 0 && keep[i] < (int8_t)kSlots &&
            preferredSlot(rids[i]) >= 0 && !taken[keep[i]])
        {
            out[i] = keep[i];
            taken[keep[i]] = true;
        }
    }

    // Pass 1: the preferred slot, lowest rid first, so the lower rid wins a clash.
    bool lost[kMaxRockets] = {};
    for (size_t k = 0; k < n; k++)
    {
        const size_t i = order[k];
        const int8_t want = preferredSlot(rids[i]);
        if (out[i] >= 0 || want < 0) continue;
        if (taken[want]) { lost[i] = true; continue; }
        out[i] = want;
        taken[want] = true;
    }
    // Pass 2: the clash losers, lowest rid first, take the lowest free slot.
    for (size_t k = 0; k < n; k++)
    {
        const size_t i = order[k];
        if (!lost[i]) continue;
        for (uint8_t s = 0; s < kSlots; s++)
        {
            if (!taken[s]) { out[i] = (int8_t)s; taken[s] = true; break; }
        }
    }
}

// ---------------------------------------------------------------------------
// Per-rocket timing + control state. Lives in the BS tracker slot and is reset
// with it (eviction, slots off).
// ---------------------------------------------------------------------------
struct FrameTime
{
    uint16_t seq      = 0;
    uint32_t start_ms = 0;   // measured on-air start (RX time - time on air)
};

struct RocketSlot
{
    FrameTime win[kWindow];
    uint8_t   n    = 0;          // frames in win since the last window reset
    uint8_t   head = 0;          // next write position

    int8_t    slot            = -1;
    bool      adjusted        = false;  // an adjust was sent since the last reset
    bool      locked          = false;  // the last check after an adjust was on target
    bool      awaiting_uplink = false;  // an adjust is queued / still retrying
    uint32_t  awaiting_since_ms = 0;
    bool      have_err        = false;
    int16_t   last_err_ms     = 0;      // signed: + = late
    uint16_t  adjusts_sent    = 0;
    // Consecutive adjusts after which the rocket was still off target, and the
    // FAST frames to skip before the next check. A rocket that ignores the
    // command (firmware without the follower) is re-sent ever more rarely
    // rather than every few seconds for ever: 0, 4, 12, 28 frames of holdoff.
    uint8_t   miss_streak     = 0;
    uint8_t   holdoff         = 0;

    void resetWindow() { n = 0; head = 0; }

    // Back to "free-running, never adjusted" (reboot, modulation change) —
    // keeps the slot and the adjust count for the stats.
    void resetTiming()
    {
        resetWindow();
        adjusted = false;
        locked = false;
        have_err = false;
        miss_streak = 0;
        holdoff = 0;
    }

    void reset() { *this = RocketSlot{}; }

    void addFast(uint16_t seq, uint32_t start_ms)
    {
        if (holdoff > 0) { holdoff--; return; }
        win[head].seq = seq;
        win[head].start_ms = start_ms;
        head = (uint8_t)((head + 1) % kWindow);
        if (n < kWindow) n++;
    }

    // A FAST frame from this rocket. Frames heard while an adjust is still in
    // the uplink queue are skipped: the rocket may or may not have moved yet.
    void onFast(uint16_t seq, uint32_t start_ms, uint32_t now_ms)
    {
        if (awaiting_uplink) return;
        pruneOld(now_ms);
        addFast(seq, start_ms);
    }

    // Drop window frames older than kWindowAgeMs (a long dropout: the phase
    // they carry may be stale by the rocket's drift).
    void pruneOld(uint32_t now_ms)
    {
        bool any_old = false;
        for (uint8_t i = 0; i < n; i++)
        {
            const uint8_t idx = (uint8_t)((head + kWindow - 1 - i) % kWindow);
            if ((int32_t)(now_ms - win[idx].start_ms) > (int32_t)kWindowAgeMs) { any_old = true; break; }
        }
        if (any_old) resetWindow();
    }

    // Min-delay estimate: the window frame that started EARLIEST, comparing all
    // of them modulo the period against the newest. Returns false until the
    // window is full.
    bool estimate(uint16_t& ref_seq, uint32_t& ref_start_ms) const
    {
        if (n < kWindow) return false;
        const uint8_t newest = (uint8_t)((head + kWindow - 1) % kWindow);
        int best = newest;
        int32_t best_d = 0;
        for (uint8_t i = 0; i < kWindow; i++)
        {
            const int32_t d = phaseDiff(win[i].start_ms, win[newest].start_ms);
            if (d < best_d) { best_d = d; best = i; }
        }
        ref_seq = win[best].seq;
        ref_start_ms = win[best].start_ms;
        return true;
    }
};

// ---------------------------------------------------------------------------
// The controller: base phase + the per-frame decision.
// ---------------------------------------------------------------------------
struct Decision
{
    bool     adjust    = false;  // queue LORA_CMD_TX_SLOT_ADJUST
    bool     base_set  = false;  // this decision fixed the base phase
    bool     lock      = false;  // rocket newly on target
    bool     unlock    = false;  // a locked rocket found off target
    uint16_t ref_seq   = 0;
    int16_t  delta_ms  = 0;
    int16_t  err_ms    = 0;
};

struct Controller
{
    bool     enabled   = true;
    bool     have_base = false;
    uint32_t base_ms   = 0;     // on-air start of slot 0, modulo the period

    void resetBase() { have_base = false; base_ms = 0; }

    // Target on-air start of `slot`, as a BS time modulo the period.
    uint32_t targetMs(uint8_t slot) const { return base_ms + slotOffsetMs(slot); }

    // Evaluate one rocket after a FAST frame went into its window. eligible:
    // the caller's checks (not hopping, current modulation, not a duplicate rid,
    // no scan / transaction / recovery). airtime_ok: airtimeFits() now.
    Decision evaluate(RocketSlot& r, bool eligible, bool airtime_ok, bool any_inflight)
    {
        Decision d{};
        if (!enabled || !eligible || !airtime_ok) return d;
        if (r.slot < 0 || r.awaiting_uplink) return d;

        uint16_t ref_seq = 0;
        uint32_t ref_start = 0;
        if (!r.estimate(ref_seq, ref_start)) return d;

        if (!have_base)
        {
            // In flight nothing new is started: a 0 ms error is not worth an
            // uplink then, so the base waits for the ground.
            if (any_inflight) return d;
            base_ms = ref_start - slotOffsetMs((uint8_t)r.slot);
            have_base = true;
            d.base_set = true;
        }
        // Keep the base within a period of the newest frame (same phase), so
        // the signed differences below never span more than a few periods.
        if ((int32_t)(ref_start - base_ms) >= (int32_t)kPeriodMs)
            base_ms = ref_start - ((ref_start - base_ms) % kPeriodMs);

        const int32_t err = phaseDiff(ref_start, targetMs((uint8_t)r.slot));
        const uint32_t mag = (uint32_t)(err < 0 ? -err : err);
        r.last_err_ms = (int16_t)err;
        r.have_err = true;
        d.err_ms = (int16_t)err;

        const uint32_t tol = any_inflight ? kFlightTolMs : kTolMs;
        bool send = false;
        if (!r.adjusted)
        {
            // Free-running: put it on a grid even when it happens to be on target
            // (it will not stay there) — except in flight, unless it is far off.
            send = !any_inflight || mag > kFlightTolMs;
        }
        else if (mag > tol)
        {
            send = true;
            if (r.locked)
            {
                // Was on target: it drifted, or rebooted onto a new phase.
                d.unlock = true;
                r.locked = false;
                r.miss_streak = 0;
            }
            else if (r.miss_streak < 255)
            {
                r.miss_streak++;   // the last adjust did not land
            }
        }
        else if (!r.locked)
        {
            r.locked = true;
            r.miss_streak = 0;
            d.lock = true;
        }

        if (send)
        {
            d.adjust   = true;
            d.ref_seq  = ref_seq;
            d.delta_ms = (int16_t)wrapHalf(-err);
        }
        return d;
    }

    // The caller queued the adjust from `d` for this rocket.
    static void onAdjustQueued(RocketSlot& r, uint32_t now_ms)
    {
        r.adjusted = true;
        r.awaiting_uplink = true;
        r.awaiting_since_ms = now_ms;
        if (r.adjusts_sent < 0xFFFF) r.adjusts_sent++;
        r.resetWindow();
    }

    // The adjust's retries are over (sent or dropped): frames from now on show
    // whether it landed — after the holdoff, when earlier ones did not.
    static void onAdjustDone(RocketSlot& r)
    {
        if (!r.awaiting_uplink) return;
        r.awaiting_uplink = false;
        r.resetWindow();
        const uint8_t s = r.miss_streak > 3 ? 3 : r.miss_streak;
        r.holdoff = (uint8_t)(kWindow * ((1u << s) - 1u));
    }
};

// An adjust the caller never reported done (lost from the queue): give up on it.
inline bool awaitingExpired(const RocketSlot& r, uint32_t now_ms)
{
    return r.awaiting_uplink && (now_ms - r.awaiting_since_ms) > kAwaitMaxMs;
}

// Uplink payload: [ref_seq u16 LE][delta_ms i16 LE].
inline void packAdjust(uint8_t out[4], uint16_t ref_seq, int16_t delta_ms)
{
    const uint16_t d = (uint16_t)delta_ms;
    out[0] = (uint8_t)(ref_seq & 0xFF);
    out[1] = (uint8_t)(ref_seq >> 8);
    out[2] = (uint8_t)(d & 0xFF);
    out[3] = (uint8_t)(d >> 8);
}

}  // namespace bs_tx_slots
