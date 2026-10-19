#pragma once

// Uplink transmit-window policy (#506) — stop the blind-TX burst deafening the
// receiver.
//
// The radio is half-duplex: while the base station transmits an uplink it cannot
// hear the rocket. The 2026-07-14 bench run measured the cost exactly — the ONLY
// downlink loss in a clean 92 s flight (3 packets, and zero CRC/length/net-id
// drops) fell inside the cmd5+cmd6 blind-TX burst.
//
// It is not bad luck, it is arithmetic. At SF8/BW250/CR4-5:
//
//     downlink telemetry (46 B)  ~82 ms on air
//     uplink sim-config  (22 B)  ~51 ms on air
//     retry interval                100 ms
//
// so the RX gaps between retries are only 100 - 51 = ~49 ms, and an 82 ms packet
// CANNOT fit in a 49 ms gap. During a burst the BS doesn't lose some packets, it
// loses essentially all of them. Simply spacing the retries wider doesn't fix
// that — it just stretches the deaf window.
//
// What does fix it: the rocket transmits on a fixed ~500 ms cadence, so once we
// have received a packet we know exactly when the next one is due, and the air is
// quiet in between. Transmit inside that quiet stretch and the downlink is never
// stepped on. This is the same insight the hop safe-window already encodes
// (HOP_BS_TX_SAFE_WINDOW_MS in main.cpp) — but that one is gated on hop_active_,
// so it does nothing when hopping is off, which is exactly when the bench run lost
// its packets.
//
// It should also improve UPLINK delivery: right after the rocket's own TX is
// precisely when the rocket is listening.
//
// Pure — no ESP-IDF / radio dependencies — so the host gtest drives it directly.
// Same pattern as bs_uplink_queue.h / bs_uplink_policy.h.

#include <cstddef>
#include <cstdint>

#include "RocketComputerTypes.h"  // loraTimeOnAirMs — shared with the #150 hop-dwell math

namespace bs_uplink_txwin {

// ---------------------------------------------------------------------------
// LoRa time-on-air (Semtech AN1200.13). Needed because the usable window shrinks
// by however long OUR packet takes — a 33-byte channel-set push occupies the air
// far longer than a 0-byte sim-start.
//
// #150: delegates to the shared implementation in RocketComputerTypes.h so the
// TX-window math and the hop-dwell math cannot drift apart. The default
// preamble stays 8 (pre-#150 behaviour of this call site); the hop-dwell path
// pins the true telemetry preamble via LORA_TELEM_PREAMBLE_SYMS.
// ---------------------------------------------------------------------------
inline uint32_t timeOnAirMs(size_t payload_len, uint8_t sf, float bw_khz,
                            uint8_t cr /* 5..8 => 4/5..4/8 */,
                            uint8_t preamble_syms = 8)
{
    return loraTimeOnAirMs(payload_len, sf, bw_khz, cr, preamble_syms);
}

// ---------------------------------------------------------------------------
// Rocket downlink cadence, learned from RX timestamps. Learned rather than
// hardcoded so a telemetry-rate change can't silently invalidate the window.
// ---------------------------------------------------------------------------
inline constexpr uint32_t kDefaultPeriodMs = 500;   // 2 Hz, the configured rate
inline constexpr uint32_t kMinPlausibleMs  = 100;
inline constexpr uint32_t kMaxPlausibleMs  = 2000;

class RxCadence
{
public:
    // Call on every accepted downlink packet.
    void onPacket(uint32_t now_ms)
    {
        if (last_ms_ != 0)
        {
            const uint32_t delta = now_ms - last_ms_;
            // Ignore duplicates/bursts and long dropouts — neither is the cadence.
            if (delta >= kMinPlausibleMs && delta <= kMaxPlausibleMs)
            {
                if (samples_ == 0) period_ms_ = (float)delta;
                else               period_ms_ += 0.25f * ((float)delta - period_ms_);
                if (samples_ < 255) samples_++;
            }
        }
        last_ms_ = now_ms;
    }

    // Two clean intervals before we trust it enough to gate transmissions on it.
    // Until then the caller should not gate at all — there is no established
    // telemetry stream to protect, and holding commands back would be all cost
    // and no benefit.
    bool valid() const { return samples_ >= 2; }

    // Timestamp of the last packet in the cadence stream — the phase anchor.
    uint32_t lastMs() const { return last_ms_; }

    uint32_t periodMs() const
    {
        return valid() ? (uint32_t)(period_ms_ + 0.5f) : kDefaultPeriodMs;
    }

    void reset() { last_ms_ = 0; period_ms_ = 0.0f; samples_ = 0; }

private:
    uint32_t last_ms_   = 0;
    float    period_ms_ = 0.0f;
    uint8_t  samples_   = 0;
};

// ---------------------------------------------------------------------------
// The gate.
// ---------------------------------------------------------------------------
struct Params
{
    uint32_t period_ms      = kDefaultPeriodMs;  // measured downlink cadence
    uint32_t tx_airtime_ms  = 52;                // OUR packet's time on air
    uint32_t rx_reserve_ms  = 140;               // keep clear for the inbound packet
                                                 //   (~82 ms on air + margin)
    uint32_t link_stale_ms  = 2000;              // no packet this long => no link to protect
    uint32_t max_defer_ms   = 1500;              // liveness backstop, see below
};

// May we START an uplink transmission right now?
//
// last_rx_ms == 0 means we have never heard the rocket.
// deferred_for_ms is how long THIS command has already been held back by this gate.
//
// Liveness matters more than tidiness here: the uplink is blind and carries
// safety-relevant commands (pyro arm, camera, logging). A command that never goes
// out is far worse than a dropped telemetry row, so every path that cannot find a
// safe window falls through to "transmit anyway" rather than blocking.
inline bool mayStartTx(uint32_t now_ms,
                       uint32_t last_rx_ms,
                       uint32_t deferred_for_ms,
                       const Params& p)
{
    // No link yet — rendezvous / recovery / silence-recovery must be able to TX,
    // and there is no downlink to protect anyway.
    if (last_rx_ms == 0) return true;

    const uint32_t age = now_ms - last_rx_ms;

    // Rocket has gone quiet. Nothing to step on, and this is exactly when we most
    // need to shout (recovery pushes).
    if (age >= p.link_stale_ms) return true;

    // Backstop: never let the gate starve a command indefinitely (e.g. a cadence
    // estimate that leaves no usable window, or a pathological loop timing).
    if (deferred_for_ms >= p.max_defer_ms) return true;

    // The next downlink packet is due at last_rx_ms + period, and needs the air
    // clear for rx_reserve_ms before that. Our transmission occupies
    // [now, now + tx_airtime]. Require it to finish first.
    const uint32_t needed = p.rx_reserve_ms + p.tx_airtime_ms;
    if (needed >= p.period_ms) return true;   // no window can exist — don't deadlock

    const uint32_t window_end = p.period_ms - needed;
    return age <= window_end;
}

// ---------------------------------------------------------------------------
// Multi-rocket gap gate (three-rocket support).
//
// mayStartTx protects ONE rocket: the focused one, whose cadence rx_cadence
// learns (#570). With three rockets on the channel the air is busy three times
// per 500 ms, and an uplink fired into the focused rocket's quiet stretch lands
// on top of somebody else's frame. Slotted, the three FAST frames sit 500/3 ms
// apart and leave a ~54 ms hole after each one (166.7 - 102 - 10 at SF8/BW250/
// CR4-5) — enough for every short command (a 6 B heartbeat is 35 ms, the 10 B
// slot adjust 40 ms, the 17 B cmd-10 relay 50 ms).
//
// So predict every fresh rocket's next frames and start an uplink only when
// [now, now + airtime] hits none of them:
//   * frame k of a rocket starts at (its last measured on-air start) + k * 500.
//     The FIXED period, not the learned RxCadence: a rocket's frames are 500 ms
//     apart by construction, and the learned value is the focused rocket's only.
//   * every frame is assumed FAST (the longer one) — a SLOW frame is followed by
//     a beacon now and then, and guessing which is which buys nothing.
//   * kGapMarginMs is kept clear before each predicted start for jitter.
//   * only rockets heard within kGapFreshMs on the CURRENT channel and SF count;
//     a rocket on another channel or modulation is not on our air.
//
// A command that does not fit the holes (the 33 B channel-set push is 71 ms)
// would wait forever, so each command carries a GapBudget: the time the gap gate
// held it while today's focused gate would have let it go. When that reaches
// kGapBudgetMs the command finishes its retries under the focused gate. That
// bounds what the gate can ever add to a command at ~400 ms over today's
// behaviour — which keeps the cmd-10 relay inside TXN_MAX_RELAY_MS and the
// coordinated-scan pause arithmetic valid — while a short command in a slotted
// air goes one retry per hole and is never charged at all for the time today's
// gate would have held it too.
// ---------------------------------------------------------------------------
inline constexpr uint32_t kFramePeriodMs = 500;   // telemetry period, fixed
inline constexpr uint32_t kGapMarginMs   = 10;    // clear air before each predicted start
inline constexpr uint32_t kGapFreshMs    = 2000;  // a rocket heard this recently is on the air
inline constexpr uint32_t kGapBudgetMs   = 400;   // per command, then the focused gate

// What the gate needs to know about one tracked rocket.
struct RocketPhase
{
    bool     valid          = false;  // a frame start has been measured
    uint32_t start_ms       = 0;      // on-air start of the last frame heard
    uint32_t heard_ms       = 0;      // when that frame was received
    uint32_t chan_key       = 0;      // channel/modulation it was heard on (caller's key)
};

// True when a TX of tx_airtime_ms starting at now overlaps no predicted frame of
// this one rocket. frame_airtime_ms is the FAST frame's time on air.
inline bool clearOfRocket(uint32_t now_ms, uint32_t tx_airtime_ms,
                          uint32_t frame_airtime_ms, const RocketPhase& r,
                          uint32_t period_ms = kFramePeriodMs,
                          uint32_t margin_ms = kGapMarginMs)
{
    // Elapsed since the measured start; wrap-safe. A start in the future (a
    // clock oddity) is treated as "frame on air now" — never the permissive case.
    const uint32_t e = now_ms - r.start_ms;
    if ((int32_t)e < 0) return false;
    // First predicted frame that has not finished by now: k * P + F > e.
    const uint32_t k = (e >= frame_airtime_ms) ? ((e - frame_airtime_ms) / period_ms + 1u) : 0u;
    const uint32_t next_start = k * period_ms;   // relative to r.start_ms
    // Blocked when that frame's guarded start comes before our TX ends. Later
    // frames start later still, earlier ones have already ended.
    return next_start >= e + tx_airtime_ms + margin_ms;
}

// The gap gate over every tracked rocket. A rocket counts only when it was
// heard within fresh_ms on chan_key (the radio's channel/modulation now).
inline bool gapClear(uint32_t now_ms, uint32_t tx_airtime_ms, uint32_t frame_airtime_ms,
                     const RocketPhase* rockets, size_t n, uint32_t chan_key,
                     uint32_t fresh_ms = kGapFreshMs,
                     uint32_t period_ms = kFramePeriodMs,
                     uint32_t margin_ms = kGapMarginMs)
{
    for (size_t i = 0; i < n; i++)
    {
        const RocketPhase& r = rockets[i];
        if (!r.valid || r.chan_key != chan_key) continue;
        if ((now_ms - r.heard_ms) >= fresh_ms) continue;   // stale: not on the air
        if (!clearOfRocket(now_ms, tx_airtime_ms, frame_airtime_ms, r, period_ms, margin_ms))
            return false;
    }
    return true;
}

// Per-command budget: how long the gap gate has held this command while the
// focused gate would have sent it. Reset when a command reaches the queue head.
struct GapBudget
{
    uint32_t charged_ms = 0;
    uint32_t prev_ms    = 0;      // previous held pass; 0 = not holding
    bool     exhausted  = false;  // the rest of this command's retries use the focused gate

    void reset() { *this = GapBudget{}; }
};

enum class GateMode : uint8_t { Gap, Focused };

// Decide one pass for the head command, which is otherwise ready to transmit
// (retry interval elapsed, radio free, no scan). gap_applicable is false where
// the gap gate does not apply at all (the BS is following a hop: one rocket, one
// schedule, today's gate). Returns true to transmit now; mode says which gate
// decided it (for the stats).
inline bool gateDecide(GapBudget& b, uint32_t now_ms, bool gap_applicable,
                       bool gap_clear, bool focused_allows, GateMode& mode,
                       uint32_t budget_ms = kGapBudgetMs)
{
    if (!gap_applicable || b.exhausted)
    {
        mode = GateMode::Focused;
        b.prev_ms = 0;
        return focused_allows;
    }
    mode = GateMode::Gap;
    if (gap_clear)
    {
        b.prev_ms = 0;
        return true;
    }
    // Held by the gap gate. Charge only what today's gate would not also have
    // held: that is the delay this gate adds.
    if (b.prev_ms != 0 && focused_allows) b.charged_ms += now_ms - b.prev_ms;
    b.prev_ms = now_ms;
    if (b.charged_ms >= budget_ms)
    {
        b.exhausted = true;
        b.prev_ms   = 0;
        mode        = GateMode::Focused;
        return focused_allows;
    }
    return false;
}

}  // namespace bs_uplink_txwin
