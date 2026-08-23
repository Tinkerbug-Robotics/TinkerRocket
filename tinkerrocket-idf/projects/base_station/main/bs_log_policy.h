#pragma once

// Base-station LoRa CSV logging policy helpers (#137, #381).
//
// Pure functions extracted from main.cpp so the host-side gtest can drive
// them without dragging in the LoRa radio, SD card, or BLE stack.  Anything
// in here must remain free of ESP-IDF / FreeRTOS / hardware dependencies.
// (RocketComputerTypes.h is pure types/helpers and host-compiles — the
// rocket-state enum and computeFreqLockForFlight come from there so the
// transition rules can't drift from the rocket side.)

#include <cstdint>
#include <cstdio>
#include <cstring>

#include <RocketComputerTypes.h>  // RocketState, computeFreqLockForFlight()

namespace bs_log_policy {

// ---------------------------------------------------------------------------
// Per-rocket log-transition state (#381).
//
// The CSV open/close/freq-lock state machine used to key off ONE global
// last_rocket_state while tracked_rockets[] accepts up to 4 rockets. With two
// powered rockets in range (A landed + still transmitting, B powering up
// READY), every interleaved A-then-B packet pair read as a LANDED transition
// followed by a fresh non-LANDED packet: close, reopen, close, reopen — one
// CSV per interleave cycle, rows scattered, and freq_locked_for_flight
// flapping per packet. Each tracked slot now carries its own transition
// state; the log open/close decision aggregates over the fresh slots.
// ---------------------------------------------------------------------------
struct RocketLogState {
    uint8_t  last_state        = 0;      // last seen RocketState (wire value)
    bool     have_seen_first   = false;  // boot-edge guard (per rocket)
    uint32_t inflight_entry_ms = 0;      // INFLIGHT safety arm time; 0 = off
    bool     freq_lock         = false;  // per-rocket computeFreqLockForFlight
};

// Edges detected while folding one received packet into a rocket's state.
struct PacketEdges {
    bool state_changed = false;  // any state transition (post boot-edge)
    bool landed_edge   = false;  // non-LANDED -> LANDED transition
};

// Fold one packet's rocket_state into the per-rocket state: detect edges,
// arm/disarm the INFLIGHT safety timer, latch the per-rocket freq lock.
// Mirrors the old global logic exactly, including the boot-edge special
// cases (first packet never edges; first packet already INFLIGHT arms the
// safety timer so a BS that boots mid-flight still bounds the log).
static inline PacketEdges updateRocketLogState(RocketLogState& s,
                                               uint8_t in_state,
                                               uint32_t now_ms)
{
    PacketEdges e{};
    if (s.have_seen_first) {
        e.state_changed = (in_state != s.last_state);
        e.landed_edge   = (in_state == LANDED && s.last_state != LANDED);
        if (in_state == INFLIGHT && s.last_state != INFLIGHT) {
            s.inflight_entry_ms = now_ms;
        } else if (s.last_state == INFLIGHT && in_state != INFLIGHT) {
            s.inflight_entry_ms = 0;
        }
    } else if (in_state == INFLIGHT) {
        s.inflight_entry_ms = now_ms;
    }
    s.have_seen_first = true;
    s.last_state      = in_state;
    s.freq_lock       = computeFreqLockForFlight(s.freq_lock, in_state);
    return e;
}

// Snapshot of one tracked slot for the aggregate decisions below.
struct RocketView {
    bool     active            = false;
    uint32_t last_seen_ms      = 0;
    uint8_t  state             = 0;
    uint32_t inflight_entry_ms = 0;
    bool     freq_lock         = false;
};

// A rocket silent longer than fresh_window_ms no longer vetoes a log close
// and no longer holds the freq lock — it powered off or left range. The
// caller passes LOG_SILENCE_TIMEOUT_MS: coherent with the file-level rule
// ("that silent" would have closed the log anyway). Unsigned subtraction is
// millis()-wraparound-safe.
static inline bool rocketFresh(const RocketView& r, uint32_t now_ms,
                               uint32_t fresh_window_ms)
{
    return r.active && (now_ms - r.last_seen_ms) <= fresh_window_ms;
}

// True when NO fresh rocket is still "in play" — i.e. every fresh rocket is
// either LANDED or presumed down (INFLIGHT safety timer expired: armed and
// past safety_ms with no state change — telemetry lost mid-flight). This is
// the multi-rocket close condition: rocket A's LANDED closes the log only
// when it is the last fresh rocket flying.
static inline bool noFreshRocketFlying(const RocketView* rockets, int n,
                                       uint32_t now_ms,
                                       uint32_t fresh_window_ms,
                                       uint32_t safety_ms)
{
    for (int i = 0; i < n; ++i) {
        const RocketView& r = rockets[i];
        if (!rocketFresh(r, now_ms, fresh_window_ms)) continue;
        if (r.state == LANDED) continue;
        if (r.inflight_entry_ms != 0 &&
            (now_ms - r.inflight_entry_ms) >= safety_ms) continue;  // presumed down
        return false;
    }
    return true;
}

// True when any active rocket's INFLIGHT safety timer has expired — the
// periodic close trigger (paired with noFreshRocketFlying so a second
// still-flying rocket keeps the log open). Deliberately ignores freshness:
// an expired timer means telemetry stopped, which is exactly the case the
// safety net exists for.
static inline bool anySafetyExpired(const RocketView* rockets, int n,
                                    uint32_t now_ms, uint32_t safety_ms)
{
    for (int i = 0; i < n; ++i) {
        const RocketView& r = rockets[i];
        if (r.active && r.inflight_entry_ms != 0 &&
            (now_ms - r.inflight_entry_ms) >= safety_ms) return true;
    }
    return false;
}

// Aggregate frequency lock: locked while ANY fresh rocket's per-rocket lock
// is latched. Replaces the old single global that the last-arrived packet
// overwrote (rocket B's READY unlocked mid-flight-of-A every other packet).
static inline bool aggregateFreqLock(const RocketView* rockets, int n,
                                     uint32_t now_ms, uint32_t fresh_window_ms)
{
    for (int i = 0; i < n; ++i) {
        if (rocketFresh(rockets[i], now_ms, fresh_window_ms) &&
            rockets[i].freq_lock) return true;
    }
    return false;
}

// Cached aggregate frequency lock (#835 item 6).
//
// aggregateFreqLock() above is freshness-gated, but a CACHED result only
// decays if something recomputes it.  main.cpp assigned its global from
// exactly one place -- the accepted-telemetry RX path -- so when the packets
// stopped, the last `true` froze for the rest of the power cycle: the window
// implemented right above was unreachable and only a power cycle cleared the
// lock.  The cache lives HERE rather than in main.cpp so the host test can
// drive the real object; the pre-#835 harness modelled a recompute-on-read
// semantics the firmware never had, which is exactly why the existing
// FreqLockStableAcrossInterleave passed against the broken code.
//
// TWO windows, because the consumers are not equally dangerous:
//
//   flight - the short (silence) window.  Gates the passive consumers:
//            silence recovery and fixed-mode heartbeats.  These want to come
//            back promptly once a rocket is gone, which is the whole point of
//            the fix -- a lost rocket must not wedge them until reboot.
//
//   retune - the long (INFLIGHT-safety) window.  Gates anything that
//            physically moves the radio off the flight channel.  Expiring
//            that at the short window would hand the operator a live cmd-10
//            while the rocket is still descending, and the cmd-10 relay is a
//            BROADCAST whose transaction commits -- and persists to NVS -- on
//            any netid-matching packet, so a second rocket on the pad
//            answering on the new channel strands the airborne one for the
//            rest of its descent.  Pre-#835 the frozen lock happened to
//            prevent that; the long window keeps it shut for the whole
//            plausible flight and still clears itself, so the operator is
//            never wedged until a power cycle.
struct FreqLockLatch {
    bool     flight       = false;  // aggregate over the short window
    bool     retune       = false;  // aggregate over the long window
    uint32_t last_eval_ms = 0;      // when both were computed

    // Recompute from a fresh view snapshot.  MUST be called unconditionally,
    // never behind an edge guard: last_eval_ms is what the read-side backstop
    // below measures against, so a stamp that only advances on an edge turns
    // the backstop into a mid-flight fuse that expires the lock a window
    // after the FIRST inflight packet no matter how much telemetry follows.
    void update(const RocketView* rockets, int n, uint32_t now_ms,
                uint32_t flight_window_ms, uint32_t retune_window_ms)
    {
        flight       = aggregateFreqLock(rockets, n, now_ms, flight_window_ms);
        retune       = aggregateFreqLock(rockets, n, now_ms, retune_window_ms);
        last_eval_ms = now_ms;
    }

    // Read-side backstop.  last_eval_ms is always >= the last_seen_ms of
    // whichever rocket made the flag true, so this can only ever hold the
    // lock LONGER than the true freshness rule, never release it earlier --
    // the safe direction for both consumers.  It is inert while update()
    // runs every loop pass (last_eval_ms == now), and it exists so that a
    // future regression that deletes or re-gates that call degrades to a
    // bounded window instead of back to "frozen until reboot", which was the
    // actual root cause here.  Unsigned subtraction: millis()-wrap safe.
    bool flightLockedAt(uint32_t now_ms, uint32_t window_ms) const
    {
        return flight && (now_ms - last_eval_ms) <= window_ms;
    }

    bool retuneLockedAt(uint32_t now_ms, uint32_t window_ms) const
    {
        return retune && (now_ms - last_eval_ms) <= window_ms;
    }
};

// Strict parser for the sequential lora_NNN.csv filenames used when the BS
// has no phone time-sync yet.  Returns true and sets `out_num` only when
// `name` exactly matches that pattern with the entire string consumed.
//
// Pre-fix, findNextFileNumber() used a bare `sscanf("lora_%hu.csv", &num)`,
// which sscanf will happily satisfy with partial matches:
//
//   sscanf("lora_20260509_164143.csv", "lora_%hu.csv", &num) -> 1, num=9885
//
// because `%hu` consumes the leading digits "20260509" (truncating to the
// low 16 bits = 9885), and the trailing format mismatch on '.' is not
// reflected in the return value.  The result was that any timestamped file
// on the SD inflated max_num past every real sequential, so the next
// no-time-sync boot produced silly names like `lora_9886.csv` even when the
// card had only ever held `lora_001.csv` next to timestamped flights (#137).
//
// Walks digits explicitly (rather than letting sscanf %hu accept signs /
// whitespace and wrap), so any non-digit between "lora_" and ".csv"
// disqualifies the match.
static inline bool parseSequentialFilename(const char* name, uint16_t& out_num)
{
    if (name == nullptr) return false;
    const size_t len = strlen(name);
    constexpr const char PREFIX[] = "lora_";
    constexpr const char SUFFIX[] = ".csv";
    constexpr size_t PREFIX_LEN = sizeof(PREFIX) - 1;
    constexpr size_t SUFFIX_LEN = sizeof(SUFFIX) - 1;
    if (len <= PREFIX_LEN + SUFFIX_LEN) return false;
    if (memcmp(name, PREFIX, PREFIX_LEN) != 0) return false;
    if (memcmp(name + len - SUFFIX_LEN, SUFFIX, SUFFIX_LEN) != 0) return false;
    const size_t digits = len - PREFIX_LEN - SUFFIX_LEN;
    if (digits == 0 || digits > 5) return false;
    uint32_t value = 0;
    for (size_t i = 0; i < digits; i++) {
        const char c = name[PREFIX_LEN + i];
        if (c < '0' || c > '9') return false;
        value = value * 10 + (uint32_t)(c - '0');
    }
    if (value > 0xFFFFu) return false;  // doesn't fit uint16
    out_num = (uint16_t)value;
    return true;
}

} // namespace bs_log_policy
