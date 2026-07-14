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

namespace bs_uplink_txwin {

// ---------------------------------------------------------------------------
// LoRa time-on-air (Semtech AN1200.13). Needed because the usable window shrinks
// by however long OUR packet takes — a 33-byte channel-set push occupies the air
// far longer than a 0-byte sim-start.
// ---------------------------------------------------------------------------
inline uint32_t timeOnAirMs(size_t payload_len, uint8_t sf, float bw_khz,
                            uint8_t cr /* 5..8 => 4/5..4/8 */,
                            uint8_t preamble_syms = 8)
{
    if (sf < 6)   sf = 6;
    if (sf > 12)  sf = 12;
    if (bw_khz <= 0.0f) bw_khz = 125.0f;

    // cr is the denominator form used in config (5 == 4/5). The formula wants 1..4.
    int cr_idx = (int)cr - 4;
    if (cr_idx < 1) cr_idx = 1;
    if (cr_idx > 4) cr_idx = 4;

    const float ts_ms = (float)(1u << sf) / bw_khz;      // symbol time, ms (bw in kHz)
    const float preamble_ms = ((float)preamble_syms + 4.25f) * ts_ms;

    // Low-data-rate optimise kicks in when a symbol is long (SF11/12 @125k).
    const int de = (ts_ms >= 16.0f) ? 1 : 0;

    const int num = 8 * (int)payload_len - 4 * (int)sf + 28 + 16;   // explicit header, CRC on
    const int den = 4 * ((int)sf - 2 * de);
    int n_payload = 0;
    if (num > 0 && den > 0)
    {
        n_payload = ((num + den - 1) / den) * (cr_idx + 4);   // ceil(num/den) * (CR+4)
    }
    const float payload_ms = (float)(8 + n_payload) * ts_ms;

    return (uint32_t)(preamble_ms + payload_ms + 0.5f);
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

}  // namespace bs_uplink_txwin
