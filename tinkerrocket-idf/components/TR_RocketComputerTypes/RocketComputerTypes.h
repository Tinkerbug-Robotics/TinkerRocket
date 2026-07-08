#ifndef ROCKETCOMPUTERTYPES_H
#define ROCKETCOMPUTERTYPES_H

#include <stdint.h>
#include <stddef.h>

// Logging state flag structure
union LogStateFlags 
{
    uint8_t all;
    struct 
    {
        uint8_t log        : 1;
        uint8_t end_flight : 1;
        uint8_t _unused    : 6;
    } bits;
};
static_assert(sizeof(LogStateFlags) == 1, 
              "LogStateFlags must be 1 byte");

// Data ready flag structure
union DataReadyFlags
{
    uint8_t all;
    struct
    {
        uint8_t gnss      : 1;
        uint8_t power     : 1;
        uint8_t bmp585    : 1;
        uint8_t ism6hg256 : 1;
        uint8_t mmc5983ma : 1;
        uint8_t nonsensor : 1;
        uint8_t logstate  : 1;
        uint8_t _unused   : 1;
    } bits;
};
static_assert(sizeof(DataReadyFlags) == 1, 
              "DataReadyFlags must be 1 byte");

// Rocket‐state enum
//
// MAG_CALIBRATION is an optional ground-only excursion from READY for the
// hard-iron magnetometer cal flow (issue #96).  Entered by user request via
// BLE cmd MAG_CAL_START, exits back to READY on accept/abort/retry.
// Refused if the rocket is in PRELAUNCH/INFLIGHT/LANDED — only safe on the
// pad / bench.  Sticky-flight, beacon, and hop predicates above all leave
// MAG_CALIBRATION at their ground-state defaults (no lock, beacons on, no
// hopping).
enum RocketState : uint8_t
{
    INITIALIZATION,
    READY,
    PRELAUNCH,
    INFLIGHT,
    LANDED,
    MAG_CALIBRATION
};

// ============================================================================
// Shared LoRa-recovery helpers (issue #71)
// ============================================================================
// Pure functions used by both the rocket and base station firmware so the
// behaviour stays bit-for-bit identical and is unit-testable from the host
// side without dragging in the radio/state machinery.

// Sticky frequency-lock-for-flight transition.  Given the previous lock
// state and the latest rocket state, returns the new lock state:
//   • INFLIGHT       → true  (latch on)
//   • READY/LANDED   → false (clear: back on the ground)
//   • everything else (INITIALIZATION/PRELAUNCH) → unchanged
// PRELAUNCH explicitly does NOT clear, so a post-flight LANDED→PRELAUNCH
// transition (rocket regains GPS while still on the ground) doesn't drop
// the lock prematurely.  Overload taking uint8_t exists because the base
// station receives state numerically from the LoRa downlink.
static inline bool computeFreqLockForFlight(bool prev_locked, RocketState s)
{
    if (s == INFLIGHT)                  return true;
    if (s == LANDED || s == READY)      return false;
    return prev_locked;
}

static inline bool computeFreqLockForFlight(bool prev_locked, uint8_t s)
{
    return computeFreqLockForFlight(prev_locked, (RocketState)s);
}

// Whether the rocket should be transmitting LoRa name beacons in this
// state.  Suppress only in INFLIGHT — telemetry needs that airtime, and
// the BS already knows where we are.  All other states (including
// INITIALIZATION) beacon: this lets the BS find the rocket even before
// the FlightComputer has finished booting and reported READY.
static inline bool shouldBeaconInState(RocketState s)
{
    return s != INFLIGHT;
}

// Whether per-packet channel hopping should be active in this state.
// PRELAUNCH and INFLIGHT only — ground states (READY/LANDED/INITIALIZATION)
// stay on the static configured channel so configuration, recovery, and
// initial handshake stay simple and predictable.
//
// Hopping starts at PRELAUNCH (rather than INFLIGHT) so the behaviour gets
// real ground-test airtime before the rocket leaves the pad — that's the
// window where defects are cheap to find.
//
// Edge case: a post-flight LANDED → PRELAUNCH transition (rocket regains
// GPS without a power cycle) would also activate hopping, which is
// suboptimal for recovery.  The existing rendezvous fallback recovers
// from this gracefully if it ever happens; revisit with a sticky
// "post-flight" gate if it bites.
static inline bool shouldHopInState(RocketState s)
{
    return s == PRELAUNCH || s == INFLIGHT;
}

static inline bool shouldHopInState(uint8_t s)
{
    return shouldHopInState((RocketState)s);
}

// ============================================================================
// Channel-set generator for per-packet frequency hopping (issues #40 / #41)
// ============================================================================
// The rocket and base station derive an identical channel table from the
// active LoRa bandwidth.  Both sides hold the same NVS preset (set via cmd
// 10), so neither side has to push the table over the wire — the agreement
// falls out of the existing modulation-config sync.
//
// Layout: channel centers are spaced at 1.5 × BW so each channel keeps a
// half-BW guard from its neighbours.  Channel 0 sits half a BW above the
// low band edge; the last channel sits half a BW below the high edge.  The
// US 902-928 MHz ISM band fits 35/69/130 channels at the Fast/250kHz/Max
// Range presets respectively.  All three counts comfortably exceed the
// FCC Part 15.247 FHSS thresholds (25 channels for BW > 250 kHz, 50 for
// BW ≤ 250 kHz) — though we operate as digital modulation (DTS), not
// FHSS, so those thresholds are headroom rather than a binding constraint.
//
// next_channel_idx in the LoRa header is 1 byte (0..N-1).  The sentinel
// LORA_NEXT_CH_NO_HOP means "stay on the current channel for one more
// window".  Phase 1 wires the byte through but always sends the sentinel;
// Phase 2 will start emitting real indices.
static constexpr float   LORA_BAND_LO_MHZ        = 902.0f;  // US ISM band low edge
static constexpr float   LORA_BAND_HI_MHZ        = 928.0f;  // US ISM band high edge
static constexpr float   LORA_CHANNEL_SPACING_X  = 1.5f;    // spacing as multiple of BW
static constexpr uint8_t LORA_NEXT_CH_NO_HOP     = 0xFF;    // sentinel: don't hop

// ============================================================================
// LoRa factory rendezvous (#105 follow-up): a single shared known-good config
// ============================================================================
// Hardcoded ONCE in this shared header so the BS and OC firmware are
// guaranteed to compile against the same fallback.  Issue #105 hit the
// chicken-and-egg case: both sides had matching factory defaults in their
// per-project config.h files, but NVS-persisted operating freq drifted
// (OC was on 926.5 MHz from a prior test, BS on 915), and neither side
// knew where the other was so the BS uplink couldn't push corrections.
//
// Both projects' config.h re-export these as `LORA_RENDEZVOUS_*` so all
// existing call sites keep working unchanged — the move just removes the
// possibility of the two config.h files drifting apart in source.
//
// Frequency, BW, SF, CR, and TX power must ALL match between BS and OC for
// the radios to decode each other (LLCC68 demodulator parameters), so the
// rendezvous config carries the full modulation tuple, not just the freq.
//
// Moved up here (was further down with the LoRa structs) so
// loraPickQuietestChannelMHz() below can fall back to it without a
// forward declaration.
static constexpr float   LORA_FACTORY_RENDEZVOUS_MHZ    = 915.0f;
static constexpr uint8_t LORA_FACTORY_RENDEZVOUS_SF     = 8;
static constexpr float   LORA_FACTORY_RENDEZVOUS_BW_KHZ = 250.0f;
static constexpr uint8_t LORA_FACTORY_RENDEZVOUS_CR     = 5;
static constexpr int8_t  LORA_FACTORY_RENDEZVOUS_TX_DBM = 12;

// Number of channels available for a given LoRa bandwidth.  Returns 0 if
// the BW would not fit a single channel inside the band.
static inline uint8_t loraChannelCount(float bw_khz)
{
    if (bw_khz <= 0.0f) return 0;
    const float bw_mhz   = bw_khz / 1000.0f;
    const float spacing  = bw_mhz * LORA_CHANNEL_SPACING_X;
    const float min_f    = LORA_BAND_LO_MHZ + bw_mhz * 0.5f;
    const float max_f    = LORA_BAND_HI_MHZ - bw_mhz * 0.5f;
    if (max_f <= min_f) return 0;
    const float span     = max_f - min_f;
    const int   n        = (int)(span / spacing) + 1;
    if (n < 1)   return 0;
    if (n > 255) return 255;  // cap so idx fits in the 1-byte header field
    return (uint8_t)n;
}

// Center frequency of channel `idx` for the given bandwidth.  Returns 0.0
// if idx is out of range — callers should treat that as "fall back to
// rendezvous".
static inline float loraChannelMHz(float bw_khz, uint8_t idx)
{
    const uint8_t n = loraChannelCount(bw_khz);
    if (idx >= n) return 0.0f;
    const float bw_mhz  = bw_khz / 1000.0f;
    const float spacing = bw_mhz * LORA_CHANNEL_SPACING_X;
    const float min_f   = LORA_BAND_LO_MHZ + bw_mhz * 0.5f;
    return min_f + spacing * (float)idx;
}

// ============================================================================
// Channel-set selection from pre-launch scan (#40 / #41 phase 3)
// ============================================================================
// After the pre-launch frequency scan (#6), the BS analyzes the result to
// pick (a) a rendezvous channel — the quietest frequency in the scanned
// range — and (b) a per-preset skip-mask of channels too noisy to use for
// hopping.  Both are then pushed to the rocket via LORA_CMD_CHANNEL_SET
// so the per-packet hop sequence skips noisy channels and the
// slow-rendezvous fallback uses the quietest meeting point.
//
// Skip-mask packing: bit i of byte (i / 8) represents channel i, LSB
// first.  A 1 means "skip this channel."  The mask is sized for the
// largest hop table we generate (139 channels at Max Range / BW=125,
// rounded up to 18 bytes = 144 bits).
//
// Threshold rule: a channel is marked as skip if its peak RSSI exceeds
// median(all channel peaks) + LORA_NOISE_THRESHOLD_DB.  An FCC floor
// (loraFhssMinChannels) is applied last — if the relative rule would
// take the active count below the floor, we instead keep the K quietest
// channels and skip the rest, regardless of absolute level.

static constexpr size_t  LORA_SKIP_MASK_MAX_BYTES = 18;   // 18*8 = 144 bits ≥ 139 channels
static constexpr uint8_t LORA_NOISE_THRESHOLD_DB  = 15;   // skip if peak > median + this
static constexpr uint8_t LORA_CMD_CHANNEL_SET     = 15;   // uplink cmd: rendezvous + mask
static constexpr uint8_t LORA_CMD_HOP_PAUSE       = 16;   // uplink cmd: park on lora_freq_mhz for N ms (#90)
static constexpr uint16_t LORA_HOP_PAUSE_MAX_MS   = 60000; // server-side cap on cmd 16 duration
static constexpr uint8_t LORA_CMD_SET_HOP_DISABLED = 17;  // uplink cmd: 1 byte payload, 0=hopping enabled (default), 1=disabled (fixed-frequency mode for diagnostics, #106)

// ── App ↔ Base Station BLE command IDs (#287) ─────────────────────────────
// The command byte the iOS app sends to the *base station* on the BLE command
// characteristic.  Promoted from bare literals in the BS dispatch so a renumber
// is greppable and tools/check_ble_command_ids.py resolves them to numbers.
// This is the BS command space; the app↔Out-Computer space is INDEPENDENT and
// may reuse the same number for a different meaning (e.g. 50 = relay-to-rocket
// here, but mag-cal-start on the OC — that overlap is correct).  Some are
// relayed verbatim to the rocket (camera/logging/sim/servo-test); their value
// happens to equal the OC's inner command number.  LORA_CMD_SET_HOP_DISABLED
// (17, above) doubles as the BS hop-disable BLE command and is reused as-is.
static constexpr uint8_t BLE_BS_CMD_CAMERA_TOGGLE     = 1;
static constexpr uint8_t BLE_BS_CMD_FILE_LIST         = 2;
static constexpr uint8_t BLE_BS_CMD_FILE_DELETE       = 3;
static constexpr uint8_t BLE_BS_CMD_FILE_DOWNLOAD     = 4;
static constexpr uint8_t BLE_BS_CMD_SIM_CONFIG        = 5;
static constexpr uint8_t BLE_BS_CMD_SIM_START         = 6;
static constexpr uint8_t BLE_BS_CMD_SIM_STOP          = 7;
static constexpr uint8_t BLE_BS_CMD_TIME_SYNC         = 9;
static constexpr uint8_t BLE_BS_CMD_LORA_RECONFIG     = 10;
static constexpr uint8_t BLE_BS_CMD_CONFIG_READBACK   = 20;
static constexpr uint8_t BLE_BS_CMD_LOGGING_TOGGLE    = 23;
static constexpr uint8_t BLE_BS_CMD_SERVO_TEST_ANGLES = 24;
static constexpr uint8_t BLE_BS_CMD_SERVO_TEST_STOP   = 25;
static constexpr uint8_t BLE_BS_CMD_SET_UNIT_NAME     = 40;
static constexpr uint8_t BLE_BS_CMD_SET_NETWORK_ID    = 41;
static constexpr uint8_t BLE_BS_CMD_RELAY_TO_ROCKET   = 50;
static constexpr uint8_t BLE_BS_CMD_FREQ_SCAN         = 60;

// Minimum SNR (dB) for an RX packet to be considered trustworthy at the
// given spreading factor.  Bench-confirmed in #90 follow-up: a CRC-
// passing decode at -12.8 dB SNR on SF8 (sensitivity -10 dB) was a
// noise-floor false positive that confused recovery + hop tracking for
// 50+ s.  Any decode below this threshold is treated as garbage and
// dropped before it touches application state.
//
// LoRa modulation theory gives demod sensitivity per SF (SNR floor at
// PER ~10%):
//   SF6:  -5    SF7:  -7.5   SF8:  -10    SF9:  -12.5
//   SF10: -15   SF11: -17.5  SF12: -20    (in dB)
//
// We keep a 2 dB margin BELOW that floor so the cutoff doesn't reject
// borderline-but-real packets that the radio happens to demod a touch
// below the theoretical limit.  Floor − 2 dB lands at:
//   SF6:  -7    SF7:  -9.5   SF8:  -12    SF9:  -14.5
//   SF10: -17   SF11: -19.5  SF12: -22
// — enough to drop the field-log -12.8 dB SF8 false positive while
// preserving every genuine decode within the demod's working range.
static inline float loraMinValidSnrDb(uint8_t sf)
{
    if (sf < 6)  sf = 6;
    if (sf > 12) sf = 12;
    const float sensitivity = -2.5f * (float)(sf - 6) - 5.0f;
    return sensitivity - 2.0f;
}

// Compute observed packet-loss between two consecutive RXes on the BS,
// from the rocket's free-running TX sequence counter (#105).  Widened to
// 16-bit seq in proto v4 (slow-hop seq-anchored hop schedule needs the
// extra range — see LORA_HOP_DWELL_PACKETS).
//
//   prev_seq_signed  : last seq seen (cast from uint16_t), or -1 if no
//                      prior packet (first contact).  Caller stores as
//                      int32_t to hold the full uint16 range plus the -1
//                      sentinel.
//   curr_seq         : seq from the just-received packet.
//   plausibility_max : maximum forward delta to treat as a real run.  A
//                      delta beyond this is almost certainly a rocket
//                      reboot (seq reset to 0) or an extended outage we
//                      can't attribute, so we report -1 ("unknown") rather
//                      than a misleading "you lost N packets" reading.
//                      Default raised to 1000 to track the wider seq
//                      range; a 1000-packet outage at 2 Hz is 8 minutes,
//                      well past the rocket's slow-rendezvous timer.
//
// Returns the number of packets lost between the two RXes, or -1 when
// not computable (first contact, duplicate seq, or implausible delta).
// Pure / branch-only so it's safely host-testable without millis().
static inline int32_t loraComputeObservedLoss(int32_t  prev_seq_signed,
                                                uint16_t curr_seq,
                                                uint32_t plausibility_max = 1000)
{
    if (prev_seq_signed < 0)            return -1;            // first contact
    const uint16_t prev  = (uint16_t)prev_seq_signed;
    const uint32_t delta = (uint32_t)((uint16_t)(curr_seq - prev));
    if (delta == 0)                     return -1;            // duplicate / replay
    if (delta > plausibility_max)       return -1;            // reboot or long outage
    return (int32_t)(delta - 1);
}

// "Is the rocket presumed to be hopping right now, from the BS's
// perspective?" — used to decide whether a BLE cmd 60 should take the
// direct-scan or coordinated-pause path (#90).
//
// Returns true when:
//   • we're actively following the hop sequence (hop_active = true), OR
//   • we caught a recent packet (within recent_threshold_ms) showing the
//     rocket in PRELAUNCH/INFLIGHT, even if its next_channel_idx was
//     LORA_NEXT_CH_NO_HOP.  The latter case covers a rocket that is
//     bootstrapping (hop_first_pkt_ true but the BS missed it), visiting
//     rendezvous as a hop-silence fallback (#41 phase 2b), or already
//     paused for an earlier coordinated scan.  In all of those, a direct
//     scan would still drop the link — the rocket is conceptually
//     hopping, just momentarily off the regular schedule.
//
// Pure helper so we can unit-test the predicate without spinning up the
// BS module-statics that real code reads from.
static inline bool rocketLikelyHopping(bool hop_active,
                                        uint32_t last_packet_ms,
                                        uint32_t now_ms,
                                        uint8_t last_rocket_state,
                                        uint32_t recent_threshold_ms)
{
    if (hop_active) return true;
    if (last_packet_ms == 0) return false;
    if ((now_ms - last_packet_ms) > recent_threshold_ms) return false;
    return shouldHopInState(last_rocket_state);
}

// FCC Part 15.247 minimum channel count for FHSS classification.  We
// operate as digital modulation (DTS) so this isn't strictly binding,
// but keeping ≥ this many channels active also preserves the diversity
// that motivates hopping in the first place.
static inline uint8_t loraFhssMinChannels(float bw_khz)
{
    return (bw_khz <= 250.0f) ? 50 : 25;
}

// Skip-mask bit accessors (LSB-first within each byte).
static inline bool loraSkipMaskTest(const uint8_t* mask, uint8_t idx)
{
    return (mask[idx >> 3] >> (idx & 7)) & 1u;
}

static inline void loraSkipMaskSet(uint8_t* mask, uint8_t idx)
{
    mask[idx >> 3] |= (uint8_t)(1u << (idx & 7));
}

// Given the current channel index and a skip-mask, returns the next
// active (non-skipped) channel index, wrapping at n_channels.  Used by
// both sides when computing next_channel_idx for the hop header.  If
// every channel were masked (defensive — the FCC floor prevents this),
// returns (current + 1) % n_channels so we still make progress.
static inline uint8_t loraNextActiveChannelIdx(
    uint8_t current_idx, const uint8_t* mask, uint8_t n_channels)
{
    if (n_channels == 0) return 0;
    for (uint8_t step = 1; step <= n_channels; step++)
    {
        const uint8_t cand = (uint8_t)((current_idx + step) % n_channels);
        if (!loraSkipMaskTest(mask, cand)) return cand;
    }
    return (uint8_t)((current_idx + 1) % n_channels);
}

// Seq-anchored hop schedule (#105 follow-up).  Both sides compute the
// physical channel for a given TX seq deterministically from this
// function — no chained handoff via next_channel_idx.  If the BS misses
// a packet within a dwell window, the *next* received packet's seq tells
// it exactly where the rocket is, allowing single-packet self-correction.
//
// Schedule:
//   active_idx = (seq / dwell) % n_active   -- which active-channel slot
//   physical_idx = the active_idx-th non-skipped channel in mask
//
// dwell == 1 reduces to the original "advance every TX" behaviour;
// dwell >= 2 keeps the rocket on a channel for `dwell` consecutive
// packets before advancing.  See LORA_HOP_DWELL_PACKETS for the
// production value.
//
// Pure helper, host-testable.  O(n_channels) lookup — fine for n ≤ 144.
static inline uint8_t loraHopChannelForSeq(
    uint16_t seq, uint8_t dwell,
    const uint8_t* mask, uint8_t n_channels)
{
    if (n_channels == 0 || dwell == 0) return 0;

    // Count active (non-skipped) channels.  Empty mask → n_active == n_channels.
    uint8_t n_active = 0;
    for (uint8_t i = 0; i < n_channels; i++)
    {
        if (!loraSkipMaskTest(mask, i)) n_active++;
    }
    // Defensive: every channel masked (FCC floor prevents this).  Fall
    // back to raw mod-n_channels so we still make forward progress.
    if (n_active == 0)
    {
        return (uint8_t)((seq / dwell) % n_channels);
    }

    const uint16_t active_idx = (uint16_t)((seq / dwell) % n_active);

    // Walk the channel space, counting non-skipped channels until we
    // hit the active_idx-th one.
    uint16_t found = 0;
    for (uint8_t i = 0; i < n_channels; i++)
    {
        if (!loraSkipMaskTest(mask, i))
        {
            if (found == active_idx) return i;
            found++;
        }
    }
    return 0;  // unreachable given the n_active check above
}

// Skip-mask + channel count produced by the pre-launch noise scan.  The
// rendezvous frequency was previously part of this struct (and thus part
// of the cmd 15 payload) so the scan could pick the quietest channel as
// the rendezvous, but that introduced a divergence path: BS and OC could
// end up with different stored rdv_mhz values and never meet again.
// Issue #105 follow-up: rendezvous is now hardcoded at compile time
// (LORA_FACTORY_RENDEZVOUS_MHZ above) so both sides cannot disagree.
// The skip-mask still drives hop-channel selection.
typedef struct
{
    uint8_t n_channels;                         // hop table size for the given BW
    uint8_t skip_mask[LORA_SKIP_MASK_MAX_BYTES]; // 1 = skip, LSB-first
} LoRaChannelSetSelection;

// Find the scan grid point with frequency closest to center_mhz; return
// its RSSI.  Returns INT8_MIN if scan_count is zero.
static inline int8_t loraScanRssiAtMHz(
    const float* scan_freqs, const int8_t* scan_rssi,
    size_t scan_count, float center_mhz)
{
    if (scan_count == 0) return INT8_MIN;
    size_t best = 0;
    float best_dist = scan_freqs[0] - center_mhz;
    if (best_dist < 0) best_dist = -best_dist;
    for (size_t i = 1; i < scan_count; i++)
    {
        float d = scan_freqs[i] - center_mhz;
        if (d < 0) d = -d;
        if (d < best_dist) { best = i; best_dist = d; }
    }
    return scan_rssi[best];
}

// Insertion-sort + median.  In-place.  n ≤ 130 in practice, so O(n²)
// is fine (~17k ops, microseconds on ESP32).
static inline int8_t loraI8MedianInPlace(int8_t* arr, size_t n)
{
    if (n == 0) return INT8_MIN;
    for (size_t i = 1; i < n; i++)
    {
        const int8_t key = arr[i];
        size_t j = i;
        while (j > 0 && arr[j - 1] > key) { arr[j] = arr[j - 1]; j--; }
        arr[j] = key;
    }
    return arr[n / 2];
}

// Pick the single quietest channel from a (freq, rssi) scan grid, snapped
// to the channel-table grid for the operating BW.  Used by the BS
// auto-acquire flow (#136) when we want one fixed frequency for the whole
// flight rather than a hop table.  Hopping logic is still present in the
// codebase but the default user build keeps `lora_hop_disabled = true`, so
// the BS picks one channel at boot and both ends stay on it.
//
// Returns the chosen frequency in MHz.  Falls back to
// LORA_FACTORY_RENDEZVOUS_MHZ if the scan is empty or the BW yields zero
// channels — those cases shouldn't happen in practice, but failing back
// to the rendezvous keeps the link from disappearing if they do.
static inline float loraPickQuietestChannelMHz(
    const float* scan_freqs, const int8_t* scan_rssi, size_t scan_count,
    float bw_khz)
{
    const uint8_t n = loraChannelCount(bw_khz);
    if (scan_count == 0 || n == 0) return LORA_FACTORY_RENDEZVOUS_MHZ;
    uint8_t best_idx  = 0;
    int8_t  best_rssi = INT8_MAX;
    for (uint8_t i = 0; i < n; i++)
    {
        const float c = loraChannelMHz(bw_khz, i);
        const int8_t r = loraScanRssiAtMHz(scan_freqs, scan_rssi,
                                            scan_count, c);
        if (r < best_rssi) { best_rssi = r; best_idx = i; }
    }
    return loraChannelMHz(bw_khz, best_idx);
}

// Pure orchestrator.  Computes the skip-mask for the current operating
// BW from a (freq, rssi) scan grid.  See the comment block above for the
// threshold rule and FCC-floor handling.  Rendezvous freq is no longer
// computed here — it is hardcoded to LORA_FACTORY_RENDEZVOUS_MHZ on both
// sides of the link so they always agree on a meeting place (#105).
//
// scan_count == 0 (e.g., scan never run yet) leaves skip_mask all-zero
// (no skips) and n_channels reflecting the BW table.
static inline void loraSelectChannelSet(
    const float* scan_freqs, const int8_t* scan_rssi, size_t scan_count,
    float bw_khz,
    LoRaChannelSetSelection* out)
{
    out->n_channels = loraChannelCount(bw_khz);
    for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) out->skip_mask[i] = 0;

    if (scan_count == 0 || out->n_channels == 0)
    {
        return;
    }

    // (2) Per-channel peak RSSI from the nearest scan grid point.
    int8_t peak[LORA_SKIP_MASK_MAX_BYTES * 8];
    for (uint8_t i = 0; i < out->n_channels; i++)
    {
        const float c = loraChannelMHz(bw_khz, i);
        peak[i] = loraScanRssiAtMHz(scan_freqs, scan_rssi, scan_count, c);
    }

    // (3) Median + threshold.
    int8_t sorted[LORA_SKIP_MASK_MAX_BYTES * 8];
    for (uint8_t i = 0; i < out->n_channels; i++) sorted[i] = peak[i];
    const int8_t median    = loraI8MedianInPlace(sorted, out->n_channels);
    const int    threshold = (int)median + (int)LORA_NOISE_THRESHOLD_DB;

    // (4) Initial mask: channels above threshold.
    uint8_t skip_count = 0;
    for (uint8_t i = 0; i < out->n_channels; i++)
    {
        if ((int)peak[i] > threshold)
        {
            loraSkipMaskSet(out->skip_mask, i);
            skip_count++;
        }
    }

    // (5) FCC floor: keep at least fhss_min channels active.  If the
    // relative rule wants more than that skipped, fall back to "keep
    // the quietest fhss_min" by re-deriving from the sorted peaks.
    const uint8_t fhss_min = loraFhssMinChannels(bw_khz);
    const uint8_t active   = (uint8_t)(out->n_channels - skip_count);
    if (active < fhss_min && fhss_min <= out->n_channels)
    {
        const int8_t cutoff = sorted[fhss_min - 1];
        for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) out->skip_mask[i] = 0;
        for (uint8_t i = 0; i < out->n_channels; i++)
        {
            if (peak[i] > cutoff) loraSkipMaskSet(out->skip_mask, i);
        }
    }
}

// Payload sent with OUT_STATUS_QUERY so the OUT processor can configure
// its SensorConverter consistently with the FlightComputer.
typedef struct __attribute__((packed))
{
    uint8_t  ism6_low_g_fs_g;     // e.g. 16
    uint16_t ism6_high_g_fs_g;    // e.g. 256
    uint16_t ism6_gyro_fs_dps;    // e.g. 4000
    int16_t  ism6_rot_z_cdeg;     // centi-deg
    int16_t  mmc_rot_z_cdeg;      // centi-deg
    uint8_t  format_version;      // payload format version
                                  //   2 = has HG bias
                                  //   3 = has board→rocket orientation
                                  //   4 = has IIS2MDC rotation
    int16_t  hg_bias_x_cmss;     // high-g bias X, centi-m/s² (0.01 m/s² units)
    int16_t  hg_bias_y_cmss;     // high-g bias Y, centi-m/s²
    int16_t  hg_bias_z_cmss;     // high-g bias Z, centi-m/s²

    // Board→rocket mounting orientation (format_version >= 3).  The
    // quaternion (scalar-first, ×10000 like NonSensorData) is what the
    // OC actually applies to its SensorConverter — it covers both the
    // 24 discrete codes and a future exact (non-snapped) auto rotation.
    // code/mode (TR_Orientation ORIENT_*) describe it for display/log.
    uint8_t  b2r_code;           // discrete orientation code (0 = +X nose)
    uint8_t  b2r_mode;           // ORIENT_MODE_* (how it was determined)
    int16_t  b2r_q[4];           // board→rocket quaternion ×10000

    // Per-chip IIS2MDC magnetometer rotation (format_version >= 4, #204).
    // New-PCB boards carry the IIS2MDC, whose sensor→board rotation differs
    // from the MMC5983MA's (mmc_rot_z_cdeg).  Consumers must apply THIS to
    // IIS2MDC data — using mmc_rot_z_cdeg gives a 180° (or worse) heading error.
    int16_t  iis2mdc_rot_z_cdeg; // centi-deg
} OutStatusQueryData;
static_assert(sizeof(OutStatusQueryData) == 28,
              "OutStatusQueryData must be 28 bytes");

// ### Data Structures ###
// Packed and unpacked data structures for each type ---

typedef struct __attribute__((packed))
{
    uint32_t time_us;

    uint16_t year;
    uint8_t  month;
    uint8_t  day;

    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint16_t milli_second;

    uint8_t  fix_mode;     // 0..5 (no fix → time only)
    uint8_t  num_sats;     // 0..255
    uint8_t  pdop_x10;     // PDOP * 10

    int32_t  lat_e7;       // deg * 1e7
    int32_t  lon_e7;       // deg * 1e7
    int32_t  alt_mm;       // mm

    int32_t  vel_e_mmps;   // mm/s
    int32_t  vel_n_mmps;
    int32_t  vel_u_mmps;

    uint8_t  h_acc_m;      // horizontal accuracy (m)
    uint8_t  v_acc_m;      // vertical accuracy (m)

} GNSSData;

static_assert(sizeof(GNSSData) == 42, 
              "GNSSData must be 42 bytes");

// --- GNSS ---
typedef struct 
{
    uint32_t time_us;
    uint16_t year;
    uint8_t month;
    uint8_t day; 
    uint8_t hour;
    uint8_t minute; 
    uint8_t second;
    uint16_t milli_second;
    uint8_t fix_mode; // 0: No Fix, 1: Dead Reckoning, 2: 2D Fix, 3: 3D Fix, 4:GNSS + Dead Reckoning, 5: Time Only
    uint8_t num_sats;
    float pdop;
    double lat; // deg
    double lon; // deg
    double alt; // m
    double vel_e; // m/s
    double vel_n; // m/s
    double vel_u; // m/s
    float horizontal_accuracy; // m
    float vertical_accuracy; // m
} GNSSDataSI;

// --- Power Data ---
typedef struct __attribute__((packed))
{
    uint32_t time_us;
    uint16_t voltage_raw; // (V / 10.0) * 65535        → 0..10 V
    int16_t  current_raw; // (mA / 10000.0) * 32767   → -10000..+10000 mA
    int16_t  soc_raw;     // (soc + 25) * (32767/150) → -25..+125 %

} POWERData;

static_assert(sizeof(POWERData) == 10, "POWERData must be 10 bytes");

typedef struct 
{
    uint32_t time_us;
    float voltage;
    float current;
    float soc;
} POWERDataSI;

// --- BMP585 Pressure and Temperature Data ---
typedef struct __attribute__((packed))
{
    uint32_t time_us;   // micros()
    int32_t  temp_q16;  // degC * 65536
    uint32_t press_q6;  // Pa * 64
} BMP585Data;
static_assert(sizeof(BMP585Data) == 12, 
              "BMP585Data must be 12 bytes");

typedef struct 
{
    uint32_t time_us;
    float pressure;
    float temperature;
} BMP585DataSI;

// --- ISM6HG256 IMU Data ---
typedef struct __attribute__((packed))
{
    int16_t x;
    int16_t y;
    int16_t z;
} Vec3i16;
static_assert(sizeof(Vec3i16) == 6, 
              "Vec3i16 must be 6 bytes");

typedef struct __attribute__((packed))
{
    uint32_t time_us;     // micros()

    Vec3i16 acc_low_raw;  // low-G accel raw counts
    Vec3i16 acc_high_raw; // high-G accel raw counts
    Vec3i16 gyro_raw;     // gyro raw counts

} ISM6HG256Data;

static_assert(sizeof(ISM6HG256Data) == 22,
              "ISM6HG256Data must be 22 bytes");

typedef struct 
{
    uint32_t time_us;
    double low_g_acc_x;
    double low_g_acc_y;
    double low_g_acc_z;
    double high_g_acc_x;
    double high_g_acc_y;
    double high_g_acc_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
} ISM6HG256DataSI;

// --- Magnetometer Data ---
typedef struct __attribute__((packed))
{
    uint32_t time_us;// micros()
    uint32_t mag_x;  // Raw counts
    uint32_t mag_y;
    uint32_t mag_z;

} MMC5983MAData;

static_assert(sizeof(MMC5983MAData) == 16, 
              "MMC5983MAData must be 16 bytes");

typedef struct
{
    uint32_t time_us;
    double mag_x_uT; // Micro Tesla
    double mag_y_uT;
    double mag_z_uT;
} MMC5983MADataSI;

// --- IIS2MDC Magnetometer Data (new PCB rev) ---
// Raw int16 per axis at 0.15 µT/LSB (datasheet 9.13). Scaling and frame
// rotation handled by TR_Sensor_Data_Converter (Stage 2 follow-up).
typedef struct __attribute__((packed))
{
    uint32_t time_us;   // micros()
    int16_t  mag_x;     // Raw counts (signed 16-bit)
    int16_t  mag_y;
    int16_t  mag_z;
} IIS2MDCData;

static_assert(sizeof(IIS2MDCData) == 10,
              "IIS2MDCData must be 10 bytes");

typedef struct
{
    uint32_t time_us;
    double mag_x_uT;
    double mag_y_uT;
    double mag_z_uT;
} IIS2MDCDataSI;

// --- Magnetometer hard-iron calibration status (#96) ---
// Single fixed-size frame carrying either live progress, final fit (review),
// applied (post-NVS-save), aborted, or idle.  FC→OC over I2S, and OC
// forwards verbatim to the iOS app over BLE on the file_ops characteristic
// with a 0xCA discriminator byte prepended.  Sized small so the OC→BLE
// notification stays well under MTU regardless of negotiated value.
//
// Field encoding:
//   offset_*       — raw IIS2MDC LSB units, signed 16-bit, 0.15 µT/LSB.
//                    On the MMC5983MA path these are the centered-counts
//                    offset (caller multiplies by UT_PER_COUNT for µT).
//   *_uT_x10       — magnitude in tenths of µT (range 0..6553.5 µT, ~130×
//                    Earth's field — plenty of headroom over the 50 µT
//                    nominal and the 1640 µT residual seen on the new PCB).
//   reject_code    — only meaningful when sub_type == REVIEW. 0 = fit
//                    passes the R-band gate AND has sufficient coverage —
//                    iOS shows "Accept".  Non-zero codes describe why iOS
//                    should show "Retry".
enum MagCalSubType : uint8_t {
    MAG_CAL_SUB_IDLE      = 0,
    MAG_CAL_SUB_SAMPLING  = 1,
    MAG_CAL_SUB_REVIEW    = 2,
    MAG_CAL_SUB_APPLIED   = 3,
    MAG_CAL_SUB_ABORTED   = 4,
    // #206 post-accept verification pass — the chip's OFFSET regs are
    // programmed with the newly-accepted cal but NVS is not yet written.
    // The FC samples mag for ~5 s while the user rotates the rocket, and
    // confirms the corrected |B| stays inside a tight band before
    // persisting.  On verify-fail the OFFSET regs are restored and the
    // calibrator falls back to REVIEW with MAG_CAL_REJECT_VERIFY_FAILED
    // so the user can retry.
    MAG_CAL_SUB_VERIFYING = 5
};

enum MagCalRejectCode : uint8_t {
    MAG_CAL_OK                    = 0,
    MAG_CAL_REJECT_R_TOO_LOW      = 1,  // fitted R < 20 µT
    MAG_CAL_REJECT_R_TOO_HIGH     = 2,  // fitted R > 80 µT
    MAG_CAL_REJECT_HIGH_RESIDUAL  = 3,  // RMS residual > threshold (poor sphere fit)
    MAG_CAL_REJECT_LOW_COVERAGE   = 4,  // < min populated wedges
    // #206: post-accept verification pass failed.  Originally a single
    // code; split into sub-codes so the iOS UI can show the actual gate
    // that failed (the previous "Verify failed — corrected |B| reached
    // X µT" message was misleading when the gate that failed was
    // coverage or sample-count rather than |B| magnitude — observed by
    // user 2026-05-26).  In all cases inst_field_uT_x10 in the same
    // frame carries the worst observed |B| (most-extreme for the
    // _TOO_HIGH/LOW/RANGE cases, last-observed for the COUNT cases).
    MAG_CAL_REJECT_VERIFY_FAILED          = 5,  // legacy / kept for back-compat
    MAG_CAL_REJECT_VERIFY_TOO_HIGH        = 6,  // verify |B| max > VERIFY_MAX_UT
    MAG_CAL_REJECT_VERIFY_TOO_LOW         = 7,  // verify |B| min < VERIFY_MIN_UT
    MAG_CAL_REJECT_VERIFY_RANGE_WIDE      = 8,  // max - min > VERIFY_RANGE_UT
    MAG_CAL_REJECT_VERIFY_LOW_COVERAGE    = 9,  // < VERIFY_MIN_COVERAGE_BINS wedges during verify
    MAG_CAL_REJECT_VERIFY_FEW_SAMPLES     = 10  // < VERIFY_MIN_SAMPLES samples accumulated
};

typedef struct __attribute__((packed))
{
    uint32_t time_us;
    uint8_t  sub_type;            // MagCalSubType
    uint8_t  coverage_bins;       // 0..32 populated wedges (truncated-icosahedron tessellation, see #148; == popcount(coverage_mask))
    uint16_t sample_count;        // total samples accumulated this run
    uint16_t inst_field_uT_x10;   // |B| of the most recent sample × 10
    int16_t  offset_x;            // fitted hard-iron offset (raw LSB units), or 0 if no fit
    int16_t  offset_y;
    int16_t  offset_z;
    uint16_t field_R_uT_x10;      // fitted Earth-field magnitude × 10, or 0 if no fit
    uint16_t residual_uT_x10;     // fit RMS residual × 10
    uint8_t  reject_code;         // MagCalRejectCode (only valid in REVIEW)
    uint8_t  _pad;
    // Bitmap of populated wedges (bit i = wedge i, see directionWedge()
    // in TR_MagCalibrator).  Drives the iOS UI's 3D sphere visualization
    // — cells fill with semi-transparent green as the rocket rotates
    // through each orientation.  Bits 0–31 map 1:1 to the 32 cells of
    // the truncated-icosahedron tessellation (12 pentagonal cells at
    // icosahedron vertices, indices 0–11; 20 hexagonal cells at face
    // centroids, indices 12–31).  See #148 for the switch from the
    // older 3³-1 = 26 cube-aligned scheme.
    uint32_t coverage_mask;
    // Live raw mag vector (last sample), in IIS2MDC LSB units
    // (0.15 µT/LSB).  Lets iOS render real-time direction feedback so
    // the user can see which body axis is currently dominant and
    // adjust the orientation — without this they're guessing whether
    // the rocket is "really" pointing up.
    int16_t  inst_x_lsb;
    int16_t  inst_y_lsb;
    int16_t  inst_z_lsb;
    // #148 — partial-coverage mask.  Bit i is set when wedge i has
    // accumulated 1..(MAG_CAL_MIN_SAMPLES_PER_WEDGE - 1) samples and
    // NOT yet been promoted into coverage_mask.  Lets iOS render a
    // 3-state cell: untouched / in-progress (this mask) / captured
    // (coverage_mask).  coverage_mask and partial_mask are disjoint
    // — a wedge graduates from partial → captured once enough samples
    // accumulate.  Wire-format extension (frame goes 32 → 36 bytes);
    // old iOS builds that decode the 32-byte tail still work — they
    // just don't get the new field and stay on 2-state visualisation.
    uint32_t partial_mask;
} MagCalStatusData;
static_assert(sizeof(MagCalStatusData) == 36,
              "MagCalStatusData must be 36 bytes");

// 14-byte payload for MAG_CAL_APPLY_MSG (issue #132).  Carries the hard-iron
// offsets in IIS2MDC LSB units plus the R / residual diagnostics so NVS state
// is bit-identical to what cmd 52 writes after a fresh cal — re-boot path is
// the same regardless of whether cal came from the sphere fit or an app push.
// No soft-iron / no MMC offsets: v1 behaviour matches MAG_CAL_ACCEPT.
typedef struct __attribute__((packed))
{
    int16_t cx;        // raw IIS2MDC LSB
    int16_t cy;
    int16_t cz;
    float   R_uT;      // fitted field magnitude when the cal was originally accepted
    float   res_uT;    // fit RMS residual when the cal was originally accepted
} MagCalApplyData;
static_assert(sizeof(MagCalApplyData) == 14,
              "MagCalApplyData must be 14 bytes");

// Sensor cal payload (issue #132): gyro zero-rate bias in raw LSB (subtracted
// from raw gyro) + high-g accel bias in m/s².  app→FC via SENSOR_CAL_APPLY_MSG.
typedef struct __attribute__((packed))
{
    int16_t gyro_x;    // raw gyro LSB
    int16_t gyro_y;
    int16_t gyro_z;
    float   hg_x;      // high-g accel bias, m/s²
    float   hg_y;
    float   hg_z;
} SensorCalApplyData;
static_assert(sizeof(SensorCalApplyData) == 18,
              "SensorCalApplyData must be 18 bytes");

// FC→OC/app sensor-cal readback: same fields plus a validity flag (0 when the
// rocket has no stored sensor cal).  Rides the file_ops characteristic behind
// a 0xCB discriminator, like mag cal's 0xCA frame.
typedef struct __attribute__((packed))
{
    uint8_t valid;     // 1 if a sensor cal is stored in NVS, else 0
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    float   hg_x;
    float   hg_y;
    float   hg_z;
} SensorCalStatusData;
static_assert(sizeof(SensorCalStatusData) == 19,
              "SensorCalStatusData must be 19 bytes");

// OC→app flash-space stats for the rocket NAND flight log.  Rides the file_ops
// characteristic behind a 0xCC discriminator (sibling of the 0xCB sensor-cal
// frame).  All counts are 128 KB NAND blocks; the app derives bytes and the
// used / reserved / free bar: total = flight_region + system, used = used_blocks,
// reserved = bad + system, free = free_blocks.  Accurate on a DIRECT rocket link.
typedef struct __attribute__((packed))
{
    uint16_t flight_region_blocks;  // total blocks the flight layer manages (~988)
    uint16_t used_blocks;           // ALLOCATED (finalized + active flights)
    uint16_t free_blocks;           // FREE
    uint16_t bad_blocks;            // BAD (unusable)
    uint16_t system_blocks;         // LFS + metadata blocks (fixed overhead)
    uint16_t flight_count;          // entries in the flight index
    uint8_t  block_size_kb;         // NAND block size in KB (128)
    uint8_t  flags;                 // bit0 = flight log initialized
} RocketStorageStatsData;
static_assert(sizeof(RocketStorageStatsData) == 14,
              "RocketStorageStatsData must be 14 bytes");

// BS→app flash-space stats for the base station's own log filesystem.  Rides the
// file_ops characteristic behind a 0xCD discriminator.  Bytes (not blocks) since
// the backend may be SD/FAT (GB) or SPIFFS (MB).  reserved is FS overhead =
// total - used - free (often ~0).  Accurate on a base-station link.
typedef struct __attribute__((packed))
{
    uint64_t total_bytes;
    uint64_t used_bytes;
    uint64_t free_bytes;
    uint8_t  backend;   // 0 = SPIFFS, 1 = SD/FAT, 2 = ext-NAND/FAT
    uint8_t  flags;     // bit0 = filesystem mounted
} BaseStationStorageStatsData;
static_assert(sizeof(BaseStationStorageStatsData) == 26,
              "BaseStationStorageStatsData must be 26 bytes");

// MMC5983MA centered-counts offset (legacy path).  Stored in NVS as
// int32_t in the same 18-bit signed centered-counts space as
// mmc5983ma_centered_counts() returns.  Subtracted before scaling to µT.
typedef struct __attribute__((packed))
{
    int32_t cx_counts;
    int32_t cy_counts;
    int32_t cz_counts;
} MagCalMMCOffset;
static_assert(sizeof(MagCalMMCOffset) == 12,
              "MagCalMMCOffset must be 12 bytes");

// Sphere-fit R sanity gate (issue #96).  WMM total field at Earth's
// surface ranges ~22-67 µT.  20-80 µT covers everywhere with margin and
// rejects fits dominated by a moving magnet during tumble.
static constexpr float MAG_CAL_R_MIN_UT = 20.0f;
static constexpr float MAG_CAL_R_MAX_UT = 80.0f;

// Coverage gate — minimum populated wedges (out of 32) for the fit to be
// considered well-conditioned.  A perfect tumble hits all 32; insisting on
// 32 is fragile, so we accept ≥ 22 (~70% — same proportion the old
// 18/26 ≈ 69% bar used in the cube-aligned scheme).  See #148 for the
// switch from 3³-1 = 26 cube wedges to a 32-cell truncated icosahedron.
static constexpr uint8_t MAG_CAL_MIN_COVERAGE_BINS = 22;

// Residual gate — RMS deviation from the fitted sphere, in µT.  A clean
// sphere on the bench should sit well under 5 µT; bigger means soft-iron
// or moving interferer dominated the capture.
static constexpr float MAG_CAL_MAX_RESIDUAL_UT = 8.0f;

// Sample-buffer cap.  Bucketed per accel-wedge (27 wedges × 100 slots
// each = 2700 total) so a user lingering in one orientation can't
// crowd out samples from other orientations — diversity is preserved
// even across long sessions.  Heap cost: 2700 × 3 × int16 ≈ 16 KiB.
static constexpr uint16_t MAG_CAL_MAX_SAMPLES = 2700;

// Minimum samples before we'll attempt a fit.  Matches the issue's "≥500
// samples (~5 s at 100 Hz)" guidance.
static constexpr uint16_t MAG_CAL_MIN_SAMPLES = 500;

// #148 — per-wedge sample threshold for a wedge to count as "captured"
// in coverage_mask.  Before this, coverage_mask's bit was set on the
// FIRST sample landing in a wedge, which meant the iOS sphere
// visualisation cleared a cell after a fleeting visit — misleading
// because the fit's linear-LSQ needs multiple samples per orientation
// to be well-conditioned.  With this threshold, coverage_mask bits
// only set once a wedge has accumulated enough samples to actually
// contribute to a stable fit.  Wedges with 1..(threshold-1) samples
// are reported in MagCalStatusData.partial_mask so iOS can render
// an intermediate "in-progress" colour.  20 samples ≈ 200 ms at the
// IIS2MDC's 100 Hz ODR — a brief deliberate pause, not a fleeting
// glance, and 22 cells × 20 samples = 440 ≈ MAG_CAL_MIN_SAMPLES so
// both gates align.
static constexpr uint16_t MAG_CAL_MIN_SAMPLES_PER_WEDGE = 20;

// NVS schema version for the mag_cal namespace.  Bump if the persisted
// field set changes meaningfully so old persisted state is ignored.
static constexpr uint8_t MAG_CAL_NVS_SCHEMA_VERSION = 1;

// --- #206 post-accept verification thresholds ---
//
// After accept, the chip's OFFSET regs are programmed and we measure
// |B| of the corrected stream over a short rotation.  The fit's R is
// already inside [20, 80] µT thanks to MAG_CAL_R_MIN/MAX_UT; here we
// insist the rotated |B| also lands inside a *tighter* band — if the
// fitted center was wrong (good fit, wrong center) the rotation will
// expose it by sweeping |B| outside this band even though the fit's
// own R sat in the wider band.
//
// 20-70 µT is the sustainable band: WMM tops out around 67 µT, and 70
// gives a couple µT of room.  Going below 20 means we landed in a hard
// magnetic null — should be physically impossible at Earth's surface,
// so it indicates the cal is over-subtracting.
static constexpr float MAG_CAL_VERIFY_MIN_UT = 20.0f;
static constexpr float MAG_CAL_VERIFY_MAX_UT = 70.0f;

// Max acceptable |B| spread (max - min) across the verify rotation.
// A perfectly-calibrated mag tracing Earth's field gives near-constant
// |B| as the rocket rotates — sub-µT in theory, a few µT after sensor
// noise and small soft-iron.  25 µT leaves headroom for tilt-coupling
// from imperfect axis alignment without letting a 30-µT residual hard
// iron sneak through.
static constexpr float MAG_CAL_VERIFY_RANGE_UT = 25.0f;

// Minimum accel-wedge coverage during verification.  The user should
// rotate the rocket through enough orientations that a residual
// hard-iron offset becomes visible — without rotation the |B|
// trivially stays in band even with a wrong center.  Tracked via the
// existing per-accel-wedge mechanism (see TR_MagCalibrator).  Bumped
// from 6 (out of 26 cube wedges) to 8 (out of 32 geodesic cells) to
// keep the proportion roughly constant after the #148 tessellation
// switch.
static constexpr uint8_t MAG_CAL_VERIFY_MIN_COVERAGE_BINS = 8;

// Minimum samples accumulated in VERIFYING before evaluateVerify() is
// allowed to declare pass.  At the new-PCB IIS2MDC's ~100 Hz output rate
// this hits in ~1 s — the FC main loop drives a longer duration target
// (~5 s) but we cap the floor here so a slow ODR can't trip the gate.
static constexpr uint16_t MAG_CAL_VERIFY_MIN_SAMPLES = 100;

// --- Non sensor data ---
typedef struct __attribute__((packed))
{
    uint32_t time_us;

    // Attitude quaternion (unit quaternion * 10000, scalar-first)
    // q = q0 + q1*i + q2*j + q3*k,  |q| = 1
    // Range: -10000 .. 10000  (resolution: 0.0001 ≈ 0.01° at small angles)
    int16_t q0;
    int16_t q1;
    int16_t q2;
    int16_t q3;
    int16_t roll_cmd;   // deg * 100

    // Position (cm)
    int32_t e_pos;
    int32_t n_pos;
    int32_t u_pos;

    // Velocity (cm/s)
    int32_t e_vel;
    int32_t n_vel;
    int32_t u_vel;

    // Flags (bitfield)
    uint8_t flags;
    /*
        bit 0: alt_landed_flag
        bit 1: alt_apogee_flag
        bit 2: vel_u_apogee_flag
        bit 3: launch_flag
        bit 4: burnout_detected
        bit 5: guidance_active
        bit 6–7: reserved
    */

    uint8_t rocket_state; // RocketState enum

    // KF-filtered barometric altitude rate from FlightComputer (dm/s = 0.1 m/s)
    int16_t baro_alt_rate_dmps;

    // Pyro channel status (bitfield)
    uint8_t pyro_status;
    /*
        bit 0: ch1 continuity (1 = load present)
        bit 1: ch2 continuity (1 = load present)
        bit 2: ch1 fired
        bit 3: ch2 fired
        bit 4–7: reserved
    */

    // Apogee detector outputs (bitfield, appended in #142/#143).
    // The `flags` byte was full, so the remaining per-detector flags and the
    // master voted result live here.  alt_apogee_flag and vel_u_apogee_flag
    // remain in `flags` for backwards-compatible decoding of old logs.
    uint8_t apogee_flags;
    /*
        bit 0: gps_apogee_flag
        bit 1: pitch_apogee_flag
        bit 2: apogee_flag (master voted result)
        bit 3–7: reserved
    */

    // Sensor health scorecard (#303): 2 bits per item (SensorHealthState), see
    // the SH_*_SHIFT positions below — incl. a state per pyro channel.  FC fills
    // baro/IMU/EKF/mag/GNSS/pyro×4; the OC ORs in battery + flight-log storage
    // before relay (both OC-owned — the FC reads neither).
    uint32_t sensor_health;

} NonSensorData;

static_assert(sizeof(NonSensorData) == 48,
              "NonSensorData must be 48 bytes");

// ── Sensor health scorecard (#303) ──────────────────────────────────────────
// 2 bits per item packed into NonSensorData.sensor_health (FC→OC) and
// LoRaData.sensor_health (OC→BS), a uint32.  Surfaced to the operator pre-launch
// on iOS (BLE JSON key "h").  Mirror this encoding in the iOS app's
// TelemetryData.  Per pyro channel so the operator sees each configured charge.
enum SensorHealthState : uint8_t { SH_NA = 0, SH_OK = 1, SH_DEGRADED = 2, SH_BAD = 3 };
// Bit shift (×2) per item within the 32-bit field.
static constexpr uint8_t SH_BARO_SHIFT = 0;
static constexpr uint8_t SH_IMU_SHIFT  = 2;
static constexpr uint8_t SH_EKF_SHIFT  = 4;   // EKF filter health: init + isHealthy + covariance converged
static constexpr uint8_t SH_MAG_SHIFT  = 6;
static constexpr uint8_t SH_GNSS_SHIFT = 8;
static constexpr uint8_t SH_BATT_SHIFT = 10;  // set by the OC from POWERData
// Per pyro channel.  SH_NA = channel not configured for this flight (ignored by
// the operator's go/no-go); OK = continuity present; DEGRADED = configured but
// continuity not yet tested; BAD = configured, tested, no continuity.
static constexpr uint8_t SH_PYRO_SHIFT[4] = { 12, 14, 16, 18 };
// Flight-log storage (#281 no-eviction / #278 silent drop): OC-owned, like
// battery.  BAD = a full or write-failing NAND that will NOT record the flight
// — the silent failure mode that lost the 2026-06-25 guided flight.
static constexpr uint8_t SH_STORAGE_SHIFT = 20;
// bits 22-31 reserved
static inline uint32_t shSet(uint32_t field, uint8_t shift, SensorHealthState st) {
    return (field & ~(uint32_t)(0x3u << shift)) | ((uint32_t)st << shift);
}
static inline uint8_t shGet(uint32_t field, uint8_t shift) {
    return (uint8_t)((field >> shift) & 0x3u);
}
// Battery verdict from pack voltage (2S Li-ion).  The OC owns this — the FC
// never reads the pack — so both OC downlink paths (LoRa relay + direct BLE)
// classify here to keep the thresholds in one place.  NaN/implausible -> NA.
static inline SensorHealthState shBatteryState(float voltage) {
    if (!(voltage == voltage) || voltage < 1.0f) return SH_NA;  // NaN or no reading
    if (voltage >= 7.0f) return SH_OK;
    if (voltage >= 6.6f) return SH_DEGRADED;
    return SH_BAD;
}
// Flight-log storage verdict (#281/#278).  The OC owns the NAND flight log; the
// FC has no visibility, so the OC classifies here and ORs the bits into
// sensor_health before relay (mirrors shBatteryState).  A full or write-failing
// store silently dropped the 2026-06-25 flight — this surfaces it pre-launch.
//   free_blocks     : BlockStateBitmap::countInState(BLOCK_FREE)
//   prealloc_blocks : blocks one flight reserves (TR_FlightLog::Config)
//   write_fail      : NAND program failures this session (nand_prog_fail)
// BAD = won't record (writes failing, or no room for the next flight);
// DEGRADED = room for fewer than 2 more flights; OK = ample room + healthy writes.
static inline SensorHealthState shStorageState(uint32_t free_blocks,
                                               uint32_t prealloc_blocks,
                                               uint32_t write_fail) {
    if (prealloc_blocks == 0) return SH_NA;          // logger not configured
    if (write_fail > 0) return SH_BAD;               // NAND writes failing now
    if (free_blocks < prealloc_blocks) return SH_BAD;        // no room for a flight
    if (free_blocks < 2u * prealloc_blocks) return SH_DEGRADED;  // low space
    return SH_OK;
}

static constexpr uint8_t NSF_ALT_LANDED   = (1u << 0);
static constexpr uint8_t NSF_ALT_APOGEE   = (1u << 1);
static constexpr uint8_t NSF_VEL_APOGEE   = (1u << 2);
static constexpr uint8_t NSF_LAUNCH       = (1u << 3);
static constexpr uint8_t NSF_BURNOUT      = (1u << 4);
static constexpr uint8_t NSF_GUIDANCE     = (1u << 5);
// New-PCB pyro: single shared arming FET drives all four channels, so
// only one global "armed" bit is reported. Live-mirrors the ARM pin.
static constexpr uint8_t NSF_PYRO_ARMED   = (1u << 6);
// Simulated flight in progress (#393): reported so the app's sim banner /
// Stop-sim control is driven by the rocket's actual state instead of a
// client-side latch that dies on BLE reconnect (BLEDevice is recreated).
static constexpr uint8_t NSF_SIM_ACTIVE   = (1u << 7);

// NonSensorData.apogee_flags bit masks (appended in #142/#143).
static constexpr uint8_t NSF2_GPS_APOGEE       = (1u << 0);
static constexpr uint8_t NSF2_PITCH_APOGEE     = (1u << 1);
static constexpr uint8_t NSF2_MASTER_APOGEE    = (1u << 2);
// Relocated from pyro_status when that byte was reclaimed for the
// 4-channel pyro layout.
static constexpr uint8_t NSF2_REBOOT_RECOVERY  = (1u << 3);  // mid-flight reboot recovery occurred
static constexpr uint8_t NSF2_GUIDANCE_ENABLED = (1u << 4);  // FC's live guidance_enabled config
// Thrust-axis cross-check failed: the boost-phase accel direction
// disagreed with the latched board→rocket orientation by >25°.  The
// orientation is NOT corrected mid-flight — this flags the mismatch for
// telemetry and post-flight analysis (suspect attitude-derived data).
static constexpr uint8_t NSF2_ORIENT_THRUST_MISMATCH = (1u << 5);

// Pyro status byte — 4 channels × (continuity, fired) = exactly 8 bits.
static constexpr uint8_t PSF_CH1_CONT  = (1u << 0);
static constexpr uint8_t PSF_CH1_FIRED = (1u << 1);
static constexpr uint8_t PSF_CH2_CONT  = (1u << 2);
static constexpr uint8_t PSF_CH2_FIRED = (1u << 3);
static constexpr uint8_t PSF_CH3_CONT  = (1u << 4);
static constexpr uint8_t PSF_CH3_FIRED = (1u << 5);
static constexpr uint8_t PSF_CH4_CONT  = (1u << 6);
static constexpr uint8_t PSF_CH4_FIRED = (1u << 7);

typedef struct
{
    uint32_t time_us;

    // Quaternion (scalar-first, unit quaternion)
    float q0;
    float q1;
    float q2;
    float q3;

    // Euler angles (derived from quaternion, for display)
    float roll;      // deg
    float pitch;     // deg
    float yaw;       // deg
    float roll_cmd;  // deg

    double e_pos;    // m
    double n_pos;
    double u_pos;

    double e_vel;    // m/s
    double n_vel;
    double u_vel;

    float pressure_alt;   // m
    float altitude_rate;  // m/s
    float max_alt;        // m
    float max_speed;      // m/s

    bool alt_landed_flag;
    bool alt_apogee_flag;
    bool vel_u_apogee_flag;
    bool launch_flag;

    // Per #142/#143: full apogee detector set + master voted result.
    bool gps_apogee_flag;
    bool pitch_apogee_flag;
    bool apogee_flag;        // master voted result

    RocketState rocket_state;

} NonSensorDataSI;

// --- LoRa Data ---

typedef struct __attribute__((packed)) 
{
    // little-endian signed 24-bit stored in 3 bytes
    uint8_t b0, b1, b2; 
} i24le_t;
static_assert(sizeof(i24le_t) == 3, "i24le_t must be 3 bytes");

// LoRa NVS schema version (#105 follow-up).  Stored alongside the LoRa
// settings; on boot, if the stored version doesn't match this constant,
// the LoRa-related NVS keys are wiped so the device falls back to the
// factory rendezvous above.  Bump whenever the NVS field set changes
// meaningfully (new keys, repurposed keys, byte-format changes), or when
// you ship a build that needs to force-clear stale settings.
//   v1: original LoRa NVS layout
//   v2: post-#105 — first version that gates on this field
//   v3: dropped rdv_mhz NVS key (rendezvous freq is now compile-time
//       hardcoded to LORA_FACTORY_RENDEZVOUS_MHZ; cmd 15 no longer
//       carries a rendezvous freq either).
static constexpr uint8_t LORA_NVS_SCHEMA_VERSION = 3;

// LoRa protocol version — bump on frame format changes.
//   v1: original 60-byte frame with 2-byte routing header.
//   v2: 60-byte frame with 3-byte routing header (next_channel_idx for #40/#41 hopping).
//   v3: 61-byte frame, appends a 1-byte free-running TX sequence counter (#105
//       lock-loss diagnostics).  BS uses the seq to compute observed loss
//       rates and distinguish missed packets from full hop-table desync.
//   v4: 62-byte frame, seq widened to 16 bits for slow-hop seq-anchored
//       hop schedule.  With dwell=4 and BW=250 (69 channels) an 8-bit seq
//       only spans 64 active positions, leaving 5 channels unreachable —
//       16 bits covers any (BW, dwell) combination we'd reasonably pick.
static constexpr uint8_t LORA_PROTO_VERSION = 4;

// Slow-hop dwell — how many consecutive packets the rocket transmits on a
// single channel before advancing.  At 2 Hz TX with dwell=4, channel
// changes every 2 s instead of every 500 ms.  This gives the BS up to
// (dwell-1) consecutive missed packets per channel before it falls out of
// sync, eliminates BS-side TX-vs-retune races (heartbeat retries no
// longer collide with hop boundaries), and reduces per-cycle radio churn
// 4x.  Both rocket and BS compile against the same value; bumping it
// requires a coordinated re-flash.
//
// FCC compliance: we operate as DTS, not FHSS, so there is no specific
// per-channel dwell-time limit.  At dwell=4 + 2 Hz we still hit every
// channel inside 140 s (BW=250, 69 channels), enough interference
// diversity for our use case.
static constexpr uint8_t LORA_HOP_DWELL_PACKETS = 4;

// LoRa name beacon sync byte (distinguishes from telemetry by size + prefix)
static constexpr uint8_t LORA_BEACON_SYNC = 0xBE;

// Heartbeat uplink command (issue #71).  Sent by the base station roughly
// every 30 s while it's actively hearing rocket telemetry, so the rocket
// has positive proof of comms in the absence of any user-initiated
// uplink.  Without this, the rocket's slow-rendezvous timer would expire
// during a passive monitoring session and waste airtime visiting the
// rendezvous frequency.  The handler is a no-op — last_uplink_rx_ms
// updates unconditionally on any successfully decoded uplink, which is
// all the rocket needs to keep the timer reset.
static constexpr uint8_t LORA_CMD_HEARTBEAT = 0xFE;

// LoRa data to send from rocket to ground station
typedef struct __attribute__((packed))
{
    // --- Routing header (proto v2: hop byte added for #40/#41) ---
    uint8_t  network_id;      // LoRa network namespace (0..255)
    uint8_t  rocket_id;       // Source rocket ID within network (1..254, 0=unset, 255=broadcast)
    uint8_t  next_channel_idx;// 0..N-1 = hop to that channel after this RX; 0xFF = stay
    uint16_t seq;             // free-running TX sequence (proto v4, #105) — wraps mod 65536

    // --- Telemetry payload (unchanged from proto v0) ---
    uint8_t num_sats;        // 0..255
    uint8_t pdop_u8;         // 0..100 (as you do now)

    i24le_t ecef_x_m;        // meters, signed 24-bit
    i24le_t ecef_y_m;
    i24le_t ecef_z_m;

    uint8_t hacc_u8;         // 0..100

    uint8_t flags_state;     // bits 0..3 flags, bits 4..6 rocket_state

    int16_t acc_x_x10;       // m/s^2 * 10
    int16_t acc_y_x10;
    int16_t acc_z_x10;

    int16_t gyro_x_x10;      // deg/s * 10
    int16_t gyro_y_x10;
    int16_t gyro_z_x10;

    int16_t temp_x10;        // degC * 10

    uint8_t voltage_u8;      // encodeVoltage_2_10_01()

    int16_t current_ma;      // mA

    int8_t  soc_i8;          // -128..127

    i24le_t pressure_alt_m;  // meters

    int16_t altitude_rate;   // m/s

    i24le_t max_alt_m;       // meters

    int16_t max_speed;       // m/s

    int16_t roll_cd;         // centideg
    int16_t pitch_cd;
    int16_t yaw_cd;

    int16_t q0;              // quaternion × 10000
    int16_t q1;
    int16_t q2;
    int16_t q3;

    int16_t speed;           // m/s

    uint32_t sensor_health;  // #303 scorecard bitfield (see SH_*_SHIFT)

} LoRaData;

static_assert(sizeof(LoRaData) == 66,
              "LoRaData must be 66 bytes (62 + uint32 sensor_health, #303)");

static constexpr uint8_t LORA_LAUNCH      = (1u << 0);  // bit 0
static constexpr uint8_t LORA_VEL_APOGEE  = (1u << 1);  // bit 1
static constexpr uint8_t LORA_ALT_APOGEE  = (1u << 2);  // bit 2
static constexpr uint8_t LORA_ALT_LANDED  = (1u << 3);  // bit 3
static constexpr uint8_t LORA_STATE_SHIFT = 4;           // bits 4-6: rocket state
static constexpr uint8_t LORA_CAMERA_REC  = (1u << 7);  // bit 7: camera recording

// logging_active is packed into the MSB of num_sats (real range 0-40, 7 bits plenty)
static constexpr uint8_t LORA_LOGGING_BIT = 0x80;

// Readable LoRa data structure
typedef struct
{                                   // Precision    : Range
    uint8_t  network_id;             // Routing header: network namespace
    uint8_t  rocket_id;              // Routing header: source rocket ID
    uint8_t  next_channel_idx;       // Routing header: 0..N-1 hop target, 0xFF = stay
    uint16_t seq;                    // Routing header: free-running TX seq (#105, proto v4)
    uint8_t num_sats;               // int          : 0 to 255
    float   pdop;                   // meter        : 0 to 100
    double  ecef_x, ecef_y, ecef_z; // meter        : +/- 7,000,000
    float   horizontal_accuracy;    // meter        : 0 to 100
    bool    alt_landed_flag,        // bit          : bool
            alt_apogee_flag,        // bit          : bool
            vel_u_apogee_flag,      // bit          : bool
            launch_flag,            // bit          : bool
            camera_recording,       // bit 7        : bool
            logging_active;         // MSB of num_sats byte
    uint8_t rocket_state;           // 3 bits       : states 0 through 5 (incl. MAG_CALIBRATION; field holds 0-7)
    float   acc_x;                  // 0.1 m/s2     : -400 to 400
    float   acc_y;                  // 0.1 m/s2     : -400 to 400
    float   acc_z;                  // 0.1 m/s2     : -400 to 400
    float   gyro_x;                 // 0.1 deg/s    : +/- 4500
    float   gyro_y;                 // 0.1 deg/s    : +/- 4500
    float   gyro_z;                 // 0.1 deg/s    : +/- 4500
    float   temp;                   // 0.1 deg      : -40 to 200
    float   voltage;                // 0.1          : 3 to 10
    float   current;                // 1 mA         : -10000 to 10000
    float   soc;                    // 1%           : -25 to 125
    float   base_station_voltage;   // N/A
    float   base_station_current;   // N/A
    float   base_station_soc;       // N/A
    float   rssi;                   // N/A
    float   snr;                    // N/A
    float   pressure_alt;           // 1 m          : -1000 to 100000
    float   altitude_rate;          // 1 m/s        : -2000 to 2000
    float   max_alt;                // 1 m          : -1000 to 400000
    float   max_speed;              // 1 m/s        : 0 to 4000
    float   roll;                   // int16_t      : -180 to 180
    float   pitch;                  // int16_t      : -90 to 90
    float   yaw;                    // int16_t      : -180 to 180
    float   q0, q1, q2, q3;        // quaternion   : -1 to 1
    float   speed;                  // 1 m/s        : 0 to 4000
    uint32_t sensor_health;         // #303 scorecard bitfield (see SH_*_SHIFT)
} LoRaDataSI;


// ### Register Read Addressess ###
static constexpr uint8_t REG_MMC5983MA_DATA = 0x00;
static constexpr uint8_t REG_BMP585_DATA    = 0x01;
static constexpr uint8_t REG_GNSS_DATA      = 0x02;
static constexpr uint8_t REG_ISM6HG256_DATA = 0x03;
static constexpr uint8_t REG_POWER_DATA     = 0x04;
static constexpr uint8_t REG_NONSENSOR_DATA = 0x05;
static constexpr uint8_t REG_DATA_READY     = 0x06;
static constexpr uint8_t REG_LOG_STATE      = 0x07;
static constexpr uint8_t REG_TEST           = 0x08;

// ### Message Types from In ESP32 ###
static constexpr uint8_t OUT_STATUS_QUERY    = 0xA0;
static constexpr uint8_t GNSS_MSG            = 0xA1;
static constexpr uint8_t ISM6HG256_MSG       = 0xA2;
static constexpr uint8_t BMP585_MSG          = 0xA3;
static constexpr uint8_t MMC5983MA_MSG       = 0xA4;
static constexpr uint8_t NON_SENSOR_MSG      = 0xA5;
static constexpr uint8_t POWER_MSG           = 0xA6;
static constexpr uint8_t START_LOGGING       = 0xA7;
static constexpr uint8_t END_FLIGHT          = 0xA8;
static constexpr uint8_t OUT_STATUS_RESPONSE = 0xA9;
static constexpr uint8_t CAMERA_START        = 0xAA;
static constexpr uint8_t CAMERA_STOP         = 0xAB;
static constexpr uint8_t SOUNDS_ENABLE       = 0xAC;
static constexpr uint8_t SOUNDS_DISABLE      = 0xAD;
static constexpr uint8_t SERVO_CONFIG_PENDING = 0xAE;
static constexpr uint8_t PID_CONFIG_PENDING   = 0xAF;
static constexpr uint8_t SERVO_CONFIG_MSG     = 0xB0;
static constexpr uint8_t PID_CONFIG_MSG       = 0xB1;
static constexpr uint8_t SERVO_CTRL_ENABLE    = 0xB2;
static constexpr uint8_t SERVO_CTRL_DISABLE   = 0xB3;
static constexpr uint8_t SIM_CONFIG_PENDING   = 0xB4;
static constexpr uint8_t SIM_CONFIG_MSG       = 0xB5;
static constexpr uint8_t SIM_START_CMD        = 0xB6;
static constexpr uint8_t SIM_STOP_CMD         = 0xB7;
static constexpr uint8_t GROUND_TEST_START   = 0xB8;
static constexpr uint8_t GROUND_TEST_STOP    = 0xB9;
static constexpr uint8_t GYRO_CAL_CMD        = 0xBA;
static constexpr uint8_t GAIN_SCHED_ENABLE   = 0xBB;
static constexpr uint8_t GAIN_SCHED_DISABLE  = 0xBC;
static constexpr uint8_t SERVO_TEST_PENDING  = 0xBD;
static constexpr uint8_t SERVO_TEST_MSG      = 0xBE;
static constexpr uint8_t SERVO_TEST_STOP     = 0xBF;
static constexpr uint8_t ROLL_PROFILE_PENDING = 0xC0;
static constexpr uint8_t ROLL_PROFILE_MSG     = 0xC1;
static constexpr uint8_t ROLL_PROFILE_CLEAR   = 0xC2;
static constexpr uint8_t SERVO_REPLAY_PENDING = 0xC3;
static constexpr uint8_t SERVO_REPLAY_MSG     = 0xC4;
static constexpr uint8_t SERVO_REPLAY_STOP    = 0xC5;
static constexpr uint8_t ROLL_CTRL_CONFIG_PENDING = 0xC6;
static constexpr uint8_t ROLL_CTRL_CONFIG_MSG     = 0xC7;
static constexpr uint8_t GUIDANCE_ENABLE          = 0xC8;
static constexpr uint8_t GUIDANCE_DISABLE         = 0xC9;
static constexpr uint8_t GUIDANCE_TELEM_MSG       = 0xCA;
static constexpr uint8_t CAMERA_CONFIG_PENDING    = 0xCB;
static constexpr uint8_t CAMERA_CONFIG_MSG        = 0xCC;
static constexpr uint8_t PYRO_CONFIG_PENDING      = 0xCD;
static constexpr uint8_t PYRO_CONFIG_MSG          = 0xCE;
static constexpr uint8_t PYRO_CONT_TEST           = 0xCF;  // momentary arm → read continuity → disarm
static constexpr uint8_t PYRO_FIRE_TEST            = 0xD0;  // test-fire a pyro channel from app
static constexpr uint8_t IIS2MDC_MSG          = 0xD1;  // new-PCB IIS2MDC magnetometer raw frame
static constexpr uint8_t SNAPSHOT_MSG         = 0xD2;  // FlightSnapshotData — FC→OC over I2S during INFLIGHT,
                                                       // and OC→FC over I2C as the response to GET_FLIGHT_SNAPSHOT
static constexpr uint8_t GET_FLIGHT_SNAPSHOT  = 0xD3;  // FC→OC: request the latest snapshot from MRAM at boot

// --- Magnetometer hard-iron cal (issue #96) ---
// OC→FC commands (passed via I2C as setPendingCommand byte) and a single
// FC→OC status message that carries either live progress or the final fit.
static constexpr uint8_t MAG_CAL_START        = 0xD4;  // OC→FC: enter MAG_CALIBRATION + begin sampling
static constexpr uint8_t MAG_CAL_ABORT        = 0xD5;  // OC→FC: drop sampling, return to READY
static constexpr uint8_t MAG_CAL_ACCEPT       = 0xD6;  // OC→FC: program new offsets, enter VERIFYING
static constexpr uint8_t MAG_CAL_RETRY        = 0xD7;  // OC→FC: discard fit, restart sampling
static constexpr uint8_t MAG_CAL_STATUS_MSG   = 0xD8;  // FC→OC: live progress / final result (MagCalStatusData)
static constexpr uint8_t MAG_CAL_COMPUTE_FIT  = 0xD9;  // OC→FC: run sphere fit on current buffer, transition to REVIEW
// User-driven verify (replaces the original 5 s auto-timeout) — see #148
// for the UX rationale.  After ACCEPT puts the FC into VERIFYING, the
// user rotates the rocket and watches a live min/max readout on iOS;
// they tap "Done" when satisfied (sends DONE) or "Retry verification"
// to clear the min/max accumulators and continue (sends RESET).  A 60 s
// safety timeout is still armed on the FC in case the user just walks
// away — without it the chip could sit with unverified offsets forever.
static constexpr uint8_t MAG_CAL_VERIFY_DONE  = 0xDD;  // OC→FC: evaluate verify gates now; pass→APPLIED, fail→REVIEW
static constexpr uint8_t MAG_CAL_VERIFY_RESET = 0xDE;  // OC→FC: clear verify min/max/coverage, stay in VERIFYING
// #148 — user-override save.  The iOS Verifying screen evaluates the
// gates locally (it has the same |B| stream as the FC).  When the user
// taps Save, iOS sends either VERIFY_DONE (when all gates green —
// firmware re-checks for safety) or FORCE_APPLY (when some gates red —
// user explicitly accepts a borderline cal).  FORCE_APPLY skips the
// gate evaluation entirely and writes NVS unconditionally.
static constexpr uint8_t MAG_CAL_FORCE_APPLY  = 0xDF;

// --- App-driven mag cal apply / read (issue #132 — rocket profiles) ---
// The iOS app holds source-of-truth for cal as part of a rocket profile.
// On connect, the app pushes the saved cal back into FC NVS (APPLY) or
// reads what's already there (READ).  Both bypass the cal flow entirely
// — no sphere fit, no MAG_CALIBRATION state transition.
static constexpr uint8_t MAG_CAL_APPLY_PENDING = 0xDA;  // OC→FC: payload follows as MAG_CAL_APPLY_MSG
static constexpr uint8_t MAG_CAL_APPLY_MSG     = 0xDB;  // 14-byte MagCalApplyData payload
static constexpr uint8_t MAG_CAL_READ          = 0xDC;  // OC→FC: publish current NVS cal as a status frame

// --- App-driven sensor cal apply / read (issue #132 — rocket profiles) ---
// The on-pad "Calibrate Sensors" routine produces a gyro zero-rate bias and
// a high-g accel bias.  Mirroring mag cal, the app stores both per-rocket and
// pushes them back on connect (APPLY) or reads what's stored (READ).  The
// low-g accelerometer is the cal reference, not itself corrected, so there is
// nothing to store for it.
//
// These three OC→FC codes originally sat at 0xDD/0xDE/0xDF, which ALIAS the
// mag-cal MAG_CAL_VERIFY_DONE/RESET/FORCE_APPLY codes above.  The FC's
// command dispatch is a flat if/else-if chain that matched the mag-cal
// branch first, so the SENSOR_CAL_* branches were dead code and a
// connect-time SENSOR_CAL_READ (0xDF) was refused on the FC as
// MAG_CAL_FORCE_APPLY ("force_apply refused: not in MAG_CALIBRATION
// session" — bench, 2026-05-29).  Reassigned to the first free block after
// the OTA relay codes.  NOTE the values are intentionally NOT contiguous
// with SENSOR_CAL_STATUS_MSG (0xE0): 0xE3–0xE7 are reserved for the OTA
// Phase 4 FC relay control codes (OTA_BEGIN_PENDING … OTA_STATUS_MSG, branch
// ota/phase4-fc, #8), so sensor cal jumps to 0xE8.  Keep that block free
// here to avoid re-colliding when ota/phase4-fc merges.  NVS layout is
// unaffected — this is purely the I2C wire-code routing.
static constexpr uint8_t SENSOR_CAL_APPLY_PENDING = 0xE8;  // OC→FC: payload follows as SENSOR_CAL_APPLY_MSG
static constexpr uint8_t SENSOR_CAL_APPLY_MSG     = 0xE9;  // 18-byte SensorCalApplyData payload
static constexpr uint8_t SENSOR_CAL_READ          = 0xEA;  // OC→FC: publish current NVS sensor cal
static constexpr uint8_t SENSOR_CAL_STATUS_MSG    = 0xE0;  // FC→OC: SensorCalStatusData

// FC→OC over I2S, emitted once at the PRELAUNCH→INFLIGHT transition. A
// snapshot of the active roll-control / IMU settings so the per-flight .bin
// (and the .json the app derives from it) records what actually flew (#165).
static constexpr uint8_t FLIGHT_SETTINGS_MSG = 0xE1;

// OC→self, emitted into the log once per stats interval (~1 Hz) while a
// session is open.  Carries the runtime ring-buffer capacity and peak-fill
// metrics so post-flight analysis can judge how much MRAM the flight
// actually used and whether a smaller / cheaper chip would have fit
// (the MR25H10 is 128 KB; payload here lets us compare against the real
// high-water mark per flight).  See LogBufferStatsData.
static constexpr uint8_t LOG_BUFFER_STATS_MSG = 0xE2;

// --- OTA firmware relay to the Flight Computer (#8 Phase 4) ---
// Control plane rides the OC↔FC I2C link: the OC stages OTA_BEGIN_PENDING
// (with the OTA_BEGIN_MSG payload) / OTA_FINISH_CMD / OTA_ABORT_CMD as
// pending commands, the FC pulls them on its poll. The FC reports progress
// back over I2S via OTA_STATUS_MSG (Layer 2). The image bytes themselves
// travel over the reconfigured I2S link (Phase 4 Layer 3), not here.
static constexpr uint8_t OTA_BEGIN_PENDING   = 0xE3;  // OC→FC: OTA_BEGIN_MSG payload follows
static constexpr uint8_t OTA_BEGIN_MSG       = 0xE4;  // [size:4 LE][sha256:32] = 36 bytes
static constexpr uint8_t OTA_FINISH_CMD      = 0xE5;  // OC→FC: finalize + verify + reboot
static constexpr uint8_t OTA_ABORT_CMD       = 0xE6;  // OC→FC: abort session
static constexpr uint8_t OTA_STATUS_MSG      = 0xE7;  // FC→OC: OtaRelayStatusData. Rides I2S in
                                                      // Layer 2; once the link flips for the image
                                                      // pump it rides I2C instead (FC master write).
// Image-pump frame (Layer 3): OC→FC over the *flipped* I2S link (OC master TX,
// FC slave RX). Payload = [offset:4 LE][image bytes:N]; the FC writes each by
// offset, ignoring stale-repeat offsets (< bytes_written) so an I2S underrun
// that replays a DMA buffer can't corrupt the image. 0xE8-0xEA are sensor cal.
static constexpr uint8_t OTA_DATA_CHUNK      = 0xEB;

// FC→OC: firmware version string (null-terminated, <=31 chars), pushed by the
// FC every ~2 s over I2C. The OC caches it and relays it to the app as a small
// "fc_identity" config JSON. Lets the app detect a *FC* OTA rollback by comparing
// the FC's real running version — the OC's own "fw" never changes on an FC-only
// update, which is why the connected-device version check false-positived (#8).
static constexpr uint8_t FC_IDENTITY         = 0xEC;

// --- Board→rocket mounting orientation setting (app → OC → FC) ---
// Two-phase like CAMERA_CONFIG: the OC stages ORIENT_CONFIG_PENDING with an
// ImuOrientConfigData payload readable as ORIENT_CONFIG_MSG.  The OC also
// re-stages a stored MANUAL setting whenever the FC's status query reports a
// different mapping (e.g. after an FC reboot fell back to auto), so manual
// roll clocking survives power cycles without the app connected.
static constexpr uint8_t ORIENT_CONFIG_PENDING = 0xED;  // OC→FC: payload follows as ORIENT_CONFIG_MSG
static constexpr uint8_t ORIENT_CONFIG_MSG     = 0xEE;  // 1-byte ImuOrientConfigData
static constexpr uint8_t GUIDANCE_CONFIG_PENDING = 0xEF;  // OC→FC: payload follows as GUIDANCE_CONFIG_MSG
static constexpr uint8_t GUIDANCE_CONFIG_MSG     = 0xF0;  // 36-byte GuidanceConfigData
static constexpr uint8_t FIN_CONFIG_PENDING      = 0xF2;  // OC→FC: payload follows as FIN_CONFIG_MSG
static constexpr uint8_t FIN_CONFIG_MSG          = 0xF3;  // 18-byte FinConfigData

// #402: FC→OC over I2C, sent as a master WRITE (writes still work while the
// slave TX ring is desynced by an aborted read).  On receipt the OC resets its
// V2 slave device to flush the residue; the FC guarantees a bus-idle window by
// suspending ALL polling for I2C_RESYNC_GRACE_MS after sending, so the reset
// can never race an in-flight read (the #279 constraint).  No payload.
static constexpr uint8_t I2C_TX_RESYNC           = 0xF4;

// I2S sample rate for the Layer 3 image pump.  BCLK = rate * 32 (16-bit stereo).
// Counter-intuitively this wants to be SLOW, not fast.  BLE (~6-16 KB/s) is the
// real bottleneck, and the FC writes each received frame to flash (~2-3 ms/frame)
// with its I2S RX effectively blind during that write.  At a fast BCLK the OC's
// per-chunk burst outruns the FC: the frames after the first arrive while it's
// mid-flash-write and are dropped, and the forward-only pump can't resend them
// (bench 2026-05-31: stuck at bytes_written=212, every later frame logged as a
// "gap").  ~50 KB/s spaces frames ~4 ms apart — longer than one flash write — so
// the FC catches every frame, while still far exceeding the BLE feed rate.  Both
// OC and FC read this one constant so the master/slave clock can't drift.
// Bench-tunable (#15); the doc's 2.5 MHz target assumed I2S was the bottleneck.
static constexpr uint32_t OTA_I2S_SAMPLE_RATE_HZ = 12500;  // ~50 KB/s, ~400 kHz BCLK

// OtaRelayStatusData.state values (FC→OC). The OC maps these to the iOS
// ota_status JSON strings it already sends for local (OC/BS) OTA.
static constexpr uint8_t OTA_RELAY_READY         = 1;  // begin OK, ota_1 erased, ready for image
static constexpr uint8_t OTA_RELAY_WRITING       = 2;  // receiving image (Layer 3)
static constexpr uint8_t OTA_RELAY_READY_TO_BOOT = 3;  // verified, rebooting (Layer 3)
static constexpr uint8_t OTA_RELAY_VERIFY_FAILED = 4;  // begin/verify error (see err)
static constexpr uint8_t OTA_RELAY_ABORTED       = 5;  // session aborted
static constexpr uint8_t OTA_RELAY_DATA_READY    = 6;  // FC flipped to I2S slave-RX; OC may start
                                                       // the image pump (Layer 3 flip handshake)

struct __attribute__((packed)) OtaRelayStatusData {
    uint8_t  state;          // OTA_RELAY_* above
    uint8_t  err;            // 0 = none; else TR_OTA_Receiver::Error code
    uint32_t bytes_written;  // progress / final size
};

static constexpr uint8_t LORA_MSG            = 0xF1;

// Camera types
static constexpr uint8_t CAM_TYPE_NONE   = 0;
static constexpr uint8_t CAM_TYPE_GOPRO  = 1;
static constexpr uint8_t CAM_TYPE_RUNCAM = 2;

// Camera config data (1 byte)
typedef struct __attribute__((packed))
{
    uint8_t camera_type;  // CAM_TYPE_NONE, CAM_TYPE_GOPRO, CAM_TYPE_RUNCAM
} CameraConfigData;
static_assert(sizeof(CameraConfigData) == 1, "CameraConfigData must be 1 byte");

// Board→rocket mounting orientation setting (ORIENT_CONFIG_MSG payload).
// IMU_ORIENT_AUTO lets the pad-gravity detect drive the mapping (fine for
// non-controlled flights); a manual TR_Orientation code (0..23) is
// authoritative and also fixes the roll clocking, which gravity cannot
// observe — required when roll control / guidance must know which way the
// control surfaces point.
static constexpr uint8_t IMU_ORIENT_AUTO = 0xFF;
typedef struct __attribute__((packed))
{
    uint8_t setting;  // IMU_ORIENT_AUTO or orientation code 0..23
} ImuOrientConfigData;
static_assert(sizeof(ImuOrientConfigData) == 1, "ImuOrientConfigData must be 1 byte");

// Pyro trigger modes
enum PyroTriggerMode : uint8_t {
    PYRO_TRIGGER_TIME_AFTER_APOGEE    = 0,
    PYRO_TRIGGER_ALTITUDE_ON_DESCENT  = 1,
};

// Pyro channel configuration — 4 channels × (enabled, mode, value) = 24 bytes
typedef struct __attribute__((packed))
{
    uint8_t  ch1_enabled;       // 0 = disabled, 1 = enabled
    uint8_t  ch1_trigger_mode;  // PyroTriggerMode
    float    ch1_trigger_value; // seconds (time mode) or meters (altitude mode)
    uint8_t  ch2_enabled;
    uint8_t  ch2_trigger_mode;
    float    ch2_trigger_value;
    uint8_t  ch3_enabled;
    uint8_t  ch3_trigger_mode;
    float    ch3_trigger_value;
    uint8_t  ch4_enabled;
    uint8_t  ch4_trigger_mode;
    float    ch4_trigger_value;
} PyroConfigData;
static_assert(sizeof(PyroConfigData) == 24, "PyroConfigData must be 24 bytes");

// Packed config data structures for BLE → I2C relay
typedef struct __attribute__((packed))
{
    int16_t bias_us[4];   // Per-servo bias in microseconds
    int16_t hz;           // PWM frequency
    int16_t min_us;       // Minimum pulse width
    int16_t max_us;       // Maximum pulse width
    float   fin_min_deg;  // #267: physical fin angle (deg) at min_us pulse
    float   fin_max_deg;  // #267: physical fin angle (deg) at max_us pulse
} ServoConfigData;
static_assert(sizeof(ServoConfigData) == 22, "ServoConfigData must be 22 bytes");

typedef struct __attribute__((packed))
{
    float kp;
    float ki;
    float kd;
    float min_cmd;
    float max_cmd;
} PIDConfigData;
static_assert(sizeof(PIDConfigData) == 20, "PIDConfigData must be 20 bytes");

// PN guidance target mode (which point the rocket steers toward).
static constexpr uint8_t GUIDE_TARGET_OVERHEAD = 0;  // directly over the pad: (0,0,target_alt)
static constexpr uint8_t GUIDE_TARGET_POINT    = 1;  // configured point: (target_e,target_n,target_alt)

// App-configurable PN guidance.  Floats first so every field is naturally aligned
// inside the packed struct (no internal padding) -> sizeof == 36.  ENU meters are
// relative to the launch pad, matching the FC's imu_pos frame (no conversion).
typedef struct __attribute__((packed))
{
    float    nav_gain;          // PN navigation constant N (3-5)
    float    max_accel_mps2;    // lateral accel command clamp (m/s^2)
    float    accel_to_fin_deg;  // accel (m/s^2) -> fin (deg) scale
    float    max_fin_deg;       // per-fin deflection clamp in guided mode (deg)
    float    min_speed_mps;     // airspeed gate below which guidance is inactive
    float    target_e_m;        // POINT: East rel. pad (m); OVERHEAD: ignored (0)
    float    target_n_m;        // POINT: North rel. pad (m); OVERHEAD: ignored (0)
    float    target_alt_m;      // target altitude above pad (m), both modes
    uint16_t coast_delay_ms;    // delay after burnout before guidance engages (ms)
    uint8_t  enable;            // 0/1 runtime guidance master enable
    uint8_t  target_mode;       // GUIDE_TARGET_OVERHEAD / _POINT
} GuidanceConfigData;
static_assert(sizeof(GuidanceConfigData) == 36, "GuidanceConfigData must be 36 bytes");

// App-configurable fin→servo mix.  azimuth_deg[i] is the CONTROL azimuth of the fin
// driven by servo i:
//   deflection_i = tilt_i·(pitch·cos(az_i) + yaw·sin(az_i)) + roll_i·roll
// tilt_i = -1 if bit i of reverse_mask is set (flips that fin's pitch/yaw response);
// roll_i = -1 if bit i of roll_reverse_mask is set (flips its roll response).  The two
// are INDEPENDENT: a fin's tilt and roll directions don't always share a sign in real
// hardware (e.g. a linkage that mirrors pitch/yaw but not the roll moment), so one bit
// can't express both.  Both masks 0 + azimuths {0,90,180,270} reproduce the legacy
// hardcoded "+" mix (servo 0 = top/+pitch, 1 = right/+yaw, 2 = bottom, 3 = left).  The
// app derives all of this from the ring GUI.
typedef struct __attribute__((packed))
{
    float   azimuth_deg[4];     // per-servo fin control azimuth (deg)
    uint8_t reverse_mask;       // bit i ⇒ negate servo i pitch/yaw (tilt) response
    uint8_t roll_reverse_mask;  // bit i ⇒ negate servo i roll response (independent)
} FinConfigData;
static_assert(sizeof(FinConfigData) == 18, "FinConfigData must be 18 bytes");

typedef struct __attribute__((packed))
{
    float mass_kg;
    float thrust_n;
    float burn_time_s;
    float descent_rate_mps;
} SimConfigData;
static_assert(sizeof(SimConfigData) == 16, "SimConfigData must be 16 bytes");

typedef struct __attribute__((packed))
{
    int16_t angle_cdeg[4];  // Per-servo angle in centi-degrees (-2000 to +2000)
} ServoTestAnglesData;
static_assert(sizeof(ServoTestAnglesData) == 8, "ServoTestAnglesData must be 8 bytes");

// --- Servo Replay Data ---
// Replays recorded flight data through the control loop to observe servo response
typedef struct __attribute__((packed))
{
    int16_t roll_rate_cdps;   // centi-deg/s  (roll rate * 100)
    int16_t speed_cmps;       // centi-m/s    (speed * 100)
} ServoReplayData;
static_assert(sizeof(ServoReplayData) == 4, "ServoReplayData must be 4 bytes");

// --- Roll Profile Data ---
// Max 8 waypoints: fits in single BLE MTU with header
static constexpr uint8_t MAX_ROLL_WAYPOINTS = 8;

// Controller behavior for a stretch of the profile (roll_profile_query result).
// Since FlightSettingsData v4 this is NOT a per-waypoint choice anymore: the
// profile is pure (time, angle) waypoints — the target ramps linearly
// (shortest path) between them, null-rate flies before the first waypoint,
// and the last angle holds after it. To hold an angle mid-profile, give two
// consecutive waypoints the same angle.
enum RollSegmentMode : uint8_t
{
    ROLL_SEG_ANGLE     = 0,  // track the profile's interpolated target angle (cascaded angle PID)
    ROLL_SEG_NULL_RATE = 1,  // hold roll rate = 0 (rate-only inner PID); angle field ignored
};

typedef struct __attribute__((packed))
{
    float   time_s;     // seconds after launch
    float   angle_deg;  // target roll angle (deg) at this time
    uint8_t mode;       // LEGACY (pre-v4 per-waypoint RollSegmentMode); kept for wire
                        // layout, ignored by firmware — always write ROLL_SEG_ANGLE
} RollWaypoint;
static_assert(sizeof(RollWaypoint) == 9, "RollWaypoint must be 9 bytes");

typedef struct __attribute__((packed))
{
    uint8_t      num_waypoints;             // 0 = no profile (rate-only mode)
    uint8_t      _pad[3];                   // alignment padding
    RollWaypoint waypoints[MAX_ROLL_WAYPOINTS];
} RollProfileData;
static_assert(sizeof(RollProfileData) == 76, "RollProfileData must be 76 bytes");

// --- Roll Control Config (runtime-configurable from app) ---
typedef struct __attribute__((packed))
{
    uint8_t  use_angle_control;   // 0 = rate-only (null roll), 1 = cascaded angle control w/ profile
    uint8_t  _pad;
    uint16_t roll_delay_ms;       // ms after launch before any roll control activates
    float    kp_angle_rate_cap_dps;  // outer-loop angle→rate command cap (deg/s); <=0 keeps firmware default
    float    kp_angle;               // outer-loop angle P-gain; <=0 keeps firmware default
    float    integral_sep_threshold_dps;  // roll-rate PID integral-separation anti-windup threshold (deg/s); >=0 applies (0 disables), <0 keeps firmware default
} RollControlConfigData;
static_assert(sizeof(RollControlConfigData) == 16, "RollControlConfigData must be 16 bytes");

// --- Flight settings snapshot (#165) ---
// One-shot snapshot of the rocket's runtime settings, emitted FC→OC over I2S
// at the PRELAUNCH→INFLIGHT transition and logged into the per-flight .bin.
// The iOS app decodes it into the flight summary .json so post-flight analysis
// can reconstruct the exact loop that flew, instead of guessing from config.h
// defaults (the original confusion in #165 — wrong Kp sent reconstruction off
// by 5x).  Covers every per-rocket setting editable in the app's settings UI
// (PID, gain schedule, servo trim/timing, roll control + profile, camera,
// pyro, sounds) plus the issue's IMU full-scale and outer-loop knobs.  Read at
// the snapshot point from the live servo_control values, the runtime override
// globals, config:: constants, pyro_config, and the active roll_profile.
// time_us is first so the generic frame parser's "timestamp = first 4 payload
// bytes" convention still holds.
struct __attribute__((packed)) FlightSettingsData
{
    // v2: appended board→rocket mounting orientation (b2r_* fields).
    // v3: appended fin-angle calibration (fin_min_deg/fin_max_deg, #267).
    // v4: NO layout change — marks the roll-profile semantics switch: the
    //     target ramps linearly between waypoints, null-rate before the first
    //     waypoint, hold after the last; per-waypoint mode bytes are ignored.
    //     (Pre-v4 firmware stepped to the NEXT waypoint's angle and honored
    //     per-waypoint null_rate modes — analysis must branch on this.)
    static constexpr uint8_t VERSION = 4;

    // flags bit positions
    static constexpr uint8_t F_USE_ANGLE_CONTROL = 0;  // cascaded angle vs rate-only
    static constexpr uint8_t F_GAIN_SCHEDULE     = 1;  // gain scheduling enabled
    static constexpr uint8_t F_GUIDANCE          = 2;  // PN guidance enabled
    static constexpr uint8_t F_SERVO_ENABLED     = 3;  // servo/roll control enabled at all
    static constexpr uint8_t F_FW_DIRTY          = 4;  // build had uncommitted changes
    static constexpr uint8_t F_SOUNDS            = 5;  // piezo sounds enabled

    uint32_t time_us;            // micros() at snapshot
    uint8_t  version;            // = VERSION
    uint8_t  flags;              // see F_* bit positions above
    uint16_t roll_delay_ms;      // roll-control activation delay after launch (ms)

    // Inner rate PID — gain-schedule base gains (what the loop uses at V_ref)
    float    kp;
    float    ki;
    float    kd;
    float    d_lpf_hz;           // D-term low-pass cutoff (Hz; 0 = disabled)
    float    min_cmd_deg;        // fin command lower limit (deg)
    float    max_cmd_deg;        // fin command upper limit (deg)

    // Outer (cascaded angle) loop
    float    kp_angle;
    float    kp_angle_rate_cap_dps;

    // Gain schedule (meaningful only when F_GAIN_SCHEDULE set)
    float    gs_v_ref;           // reference speed (m/s)
    float    gs_v_min;           // min speed clamp (m/s)
    float    gs_scale_cap;       // max gain scale factor

    // Rate-only setpoint (deg/s)
    float    roll_rate_set_point;

    // IMU full-scale (mirror of OutStatusQueryData)
    uint8_t  ism6_low_g_fs_g;    // e.g. 16
    uint16_t ism6_high_g_fs_g;   // e.g. 256
    uint16_t ism6_gyro_fs_dps;   // e.g. 4000

    // Servo trim + timing (app "Servo" settings)
    int16_t  servo_bias_us[4];   // per-servo µs trim offset
    int16_t  servo_hz;           // PWM frequency
    int16_t  servo_min_us;       // min pulse width (µs)
    int16_t  servo_max_us;       // max pulse width (µs)

    // Camera (0 = none, 1 = GoPro, 2 = RunCam)
    uint8_t  camera_type;

    // Pyro channel config (all four channels)
    PyroConfigData pyro;

    // Firmware identity: git short SHA, NUL-terminated ("unknown" if no git)
    char     fw_git_sha[12];

    // Active roll profile (num_waypoints == 0 → rate-only)
    RollProfileData roll_profile;

    // Board→rocket mounting orientation that actually flew (v2+).
    // Mirrors OutStatusQueryData v3: quaternion is authoritative,
    // code/mode (TR_Orientation ORIENT_*) describe it.  residual is the
    // angle between the auto-detected nose vector and the snapped axis
    // (0 for manual/default; meaningful once auto-detect lands).
    uint8_t  b2r_code;            // discrete orientation code (0 = +X nose)
    uint8_t  b2r_mode;            // ORIENT_MODE_*
    int16_t  b2r_residual_cdeg;   // auto-snap residual, centi-deg
    int16_t  b2r_q[4];            // board→rocket quaternion ×10000

    // Fin-angle calibration that actually flew (v3+, #267): physical fin
    // deflection (deg) at servo_min_us / servo_max_us.
    float    fin_min_deg;
    float    fin_max_deg;
};
static_assert(sizeof(FlightSettingsData) == 208, "FlightSettingsData layout check");

// --- Log Buffer Stats Data (OC self-emitted, ~1 Hz while logging) -----------
// Snapshot of the OC's ring-buffer health written into the flight log so the
// per-flight .bin records exactly how much MRAM the session actually used.
// This is the data point we need to decide if the current 1 Mbit MR25H10
// (128 KB) is over-provisioned and we can drop to a smaller / cheaper part.
//
// All counters are cumulative since boot (the OC doesn't reset between
// sessions in normal operation); analysis tooling pulls the *last* sample
// before close to get the per-flight peak.
typedef struct __attribute__((packed))
{
    uint32_t time_us;                  // micros() at emit
    uint32_t ring_size;                // ring capacity in bytes
                                       //   (MRAM region size when MRAM is wired,
                                       //    else RAM fallback ring size)
    uint32_t ring_fill;                // current bytes resident in ring at emit
    uint32_t ring_highwater;           // peak ring fill since boot
    uint32_t ring_overruns;            // count of overrun events since boot
    uint32_t ring_drop_oldest_bytes;   // cumulative bytes dropped from tail since boot
    uint32_t ring_bad_sof_clears;      // ring-clear events from corrupt-tail recovery
} LogBufferStatsData;
static_assert(sizeof(LogBufferStatsData) == 28,
              "LogBufferStatsData must be 28 bytes");

// --- Guidance Telemetry Data (sent at ~10 Hz during guided coast) ---
// Logged to the flight log as GUIDANCE_TELEM_MSG (0xCA). NOTE: the field
// names prior to this struct's extension were misleading — pitch_cmd/yaw_cmd
// actually held PN acceleration commands and pitch_fin/yaw_fin held the LOS
// angle and closing velocity, NOT fin deflections. Fields are now named for
// their true contents, and the genuinely-commanded pitch/yaw fin deflections
// (the guidance analogue of NonSensorData.roll_cmd) are logged explicitly.
typedef struct __attribute__((packed))
{
    uint32_t time_us;            // FC monotonic time (us)
    int16_t  accel_cmd_n_cmps2;  // PN acceleration command, North (m/s^2 x100)
    int16_t  accel_cmd_e_cmps2;  // PN acceleration command, East  (m/s^2 x100)
    int16_t  lateral_offset_cm;  // lateral distance from pad vertical (cm)
    int16_t  los_angle_cdeg;     // line-of-sight angle to target (deg x100)
    int16_t  closing_vel_cmps;   // closing velocity to target (m/s x100)
    int16_t  pitch_fin_cmd_cdeg; // commanded pitch fin deflection, pre-mix (deg x100)
    int16_t  yaw_fin_cmd_cdeg;   // commanded yaw fin deflection, pre-mix (deg x100)
    uint8_t  guid_flags;         // bit 0: guidance_active, bit 1: burnout_detected
} GuidanceTelemData;
static_assert(sizeof(GuidanceTelemData) == 19, "GuidanceTelemData must be 19 bytes");

// --- Flight snapshot (#104 follow-up) ---
// Periodic snapshot of FC flight state for crash recovery.  Sent FC→OC
// over I2S at 10 Hz during INFLIGHT; OC stores the latest in a reserved
// MRAM region.  On reboot, FC requests the snapshot back via I2C
// (GET_FLIGHT_SNAPSHOT) and restores state if the magic+CRC validate.
//
// Replaces the older NVS-based recovery — keeping NVS writes off the FC
// kept loop_fc free of the ~70-90 ms periodic stalls (#104).
//
// Sized to fit one I2S frame: payload ≤ MAX_PAYLOAD, ≤ 255 byte length.
//
// EKF covariance is stored as the diagonal only (15 floats) instead of
// the full 15×15 matrix (225 floats).  On recovery, the EKF rebuilds
// off-diagonals from zero — cross-correlations are re-discovered within
// ~0.5-1 s of running the filter, but the per-state uncertainty is
// preserved exactly.  This is the wire-format compromise that keeps the
// snapshot in a single I2S frame.
struct __attribute__((packed)) FlightSnapshotData
{
    static constexpr uint32_t MAGIC   = 0xF1A75A7E;  // distinct from old NVS magic (0xF1A7C0DE)
    static constexpr uint8_t  VERSION = 3;           // v2: 4-channel pyro layout, no per-channel ARM
                                                     // v3: board→rocket orientation (b2r_*)

    // --- Header ---
    uint32_t magic;
    uint8_t  version;
    uint8_t  rocket_state;
    uint8_t  pad[2];

    // --- Flight timestamps (relative to launch) ---
    uint32_t flight_elapsed_ms;
    uint32_t apogee_elapsed_ms;
    uint32_t burnout_elapsed_ms;

    // --- Pyro state (safety-critical) ---
    // "Armed" is no longer snapshotted: with per-fire arming, the ARM
    // pin is only HIGH momentarily, and a reboot leaves it LOW anyway.
    // On recovery we re-fire any unfired channel whose trigger condition
    // is still satisfied.
    uint8_t  pyro_apogee_detected;
    uint8_t  pyro1_fired;
    uint8_t  pyro2_fired;
    uint8_t  pyro3_fired;
    uint8_t  pyro4_fired;
    // Board→rocket orientation that was active at launch (v3, reclaimed
    // from pad2).  Restored BEFORE the EKF state below — the quaternion,
    // velocities and biases were all estimated in this rocket frame, so a
    // post-reboot boot-default orientation would silently invalidate them.
    uint8_t  b2r_code;            // discrete orientation code
    uint8_t  b2r_mode;            // ORIENT_MODE_*
    uint8_t  pad2[1];

    // --- Flight references ---
    float    ground_pressure_pa;
    double   ref_lat_rad;
    double   ref_lon_rad;
    double   ref_alt_m;

    // --- Control state flags ---
    uint8_t  ekf_initialized;
    uint8_t  guidance_enabled;
    uint8_t  burnout_detected;
    uint8_t  servo_enabled;

    // --- EKF state (covariance reduced to diagonal) ---
    double   ekf_pos_rrm[3];
    float    ekf_vel_ned_mps[3];
    float    ekf_quat[4];
    float    ekf_accel_bias[3];
    float    ekf_gyro_bias[3];
    float    ekf_P_diag[15];      // diagonal of P[15][15]
    uint32_t ekf_t_prev_us;
    float    ekf_euler[3];

    // Board→rocket quaternion ×10000 (v3) — authoritative rotation,
    // covers AUTO_EXACT mountings the discrete code can't express.
    int16_t  b2r_q[4];

    // --- Integrity (CRC32 over all preceding bytes) ---
    uint32_t crc32;
};
static_assert(sizeof(FlightSnapshotData) == 224,
              "FlightSnapshotData must be 224 bytes — fits one I2S frame and one I2C TX response");

static constexpr size_t SIZE_OF_GNSS_DATA = sizeof(GNSSData);
static constexpr size_t SIZE_OF_BMP585_DATA     = sizeof(BMP585Data);
static constexpr size_t SIZE_OF_ISM6HG256_DATA  = sizeof(ISM6HG256Data);
static constexpr size_t SIZE_OF_MMC5983MA_DATA  = sizeof(MMC5983MAData);
static constexpr size_t SIZE_OF_IIS2MDC_DATA    = sizeof(IIS2MDCData);
static constexpr size_t SIZE_OF_POWER_DATA      = sizeof(POWERData);
static constexpr size_t SIZE_OF_NON_SENSOR_DATA = sizeof(NonSensorData);
// --- #386: wire-layout offset pins ------------------------------------
// The cross-device structs are size-pinned, but a same-size field reorder
// or type swap passes a size assert silently and scrambles every consumer
// of the wire format. Pin every field offset so any layout change is a
// loud compile error on firmware and host builds alike. Values were
// derived from the declared layouts; the compiler proves each line.
static_assert(sizeof(LoRaData) == 66, "LoRaData wire size");
static_assert(offsetof(LoRaData, network_id) == 0, "LoRaData.network_id moved");
static_assert(offsetof(LoRaData, rocket_id) == 1, "LoRaData.rocket_id moved");
static_assert(offsetof(LoRaData, next_channel_idx) == 2, "LoRaData.next_channel_idx moved");
static_assert(offsetof(LoRaData, seq) == 3, "LoRaData.seq moved");
static_assert(offsetof(LoRaData, num_sats) == 5, "LoRaData.num_sats moved");
static_assert(offsetof(LoRaData, pdop_u8) == 6, "LoRaData.pdop_u8 moved");
static_assert(offsetof(LoRaData, ecef_x_m) == 7, "LoRaData.ecef_x_m moved");
static_assert(offsetof(LoRaData, ecef_y_m) == 10, "LoRaData.ecef_y_m moved");
static_assert(offsetof(LoRaData, ecef_z_m) == 13, "LoRaData.ecef_z_m moved");
static_assert(offsetof(LoRaData, hacc_u8) == 16, "LoRaData.hacc_u8 moved");
static_assert(offsetof(LoRaData, flags_state) == 17, "LoRaData.flags_state moved");
static_assert(offsetof(LoRaData, acc_x_x10) == 18, "LoRaData.acc_x_x10 moved");
static_assert(offsetof(LoRaData, acc_y_x10) == 20, "LoRaData.acc_y_x10 moved");
static_assert(offsetof(LoRaData, acc_z_x10) == 22, "LoRaData.acc_z_x10 moved");
static_assert(offsetof(LoRaData, gyro_x_x10) == 24, "LoRaData.gyro_x_x10 moved");
static_assert(offsetof(LoRaData, gyro_y_x10) == 26, "LoRaData.gyro_y_x10 moved");
static_assert(offsetof(LoRaData, gyro_z_x10) == 28, "LoRaData.gyro_z_x10 moved");
static_assert(offsetof(LoRaData, temp_x10) == 30, "LoRaData.temp_x10 moved");
static_assert(offsetof(LoRaData, voltage_u8) == 32, "LoRaData.voltage_u8 moved");
static_assert(offsetof(LoRaData, current_ma) == 33, "LoRaData.current_ma moved");
static_assert(offsetof(LoRaData, soc_i8) == 35, "LoRaData.soc_i8 moved");
static_assert(offsetof(LoRaData, pressure_alt_m) == 36, "LoRaData.pressure_alt_m moved");
static_assert(offsetof(LoRaData, altitude_rate) == 39, "LoRaData.altitude_rate moved");
static_assert(offsetof(LoRaData, max_alt_m) == 41, "LoRaData.max_alt_m moved");
static_assert(offsetof(LoRaData, max_speed) == 44, "LoRaData.max_speed moved");
static_assert(offsetof(LoRaData, roll_cd) == 46, "LoRaData.roll_cd moved");
static_assert(offsetof(LoRaData, pitch_cd) == 48, "LoRaData.pitch_cd moved");
static_assert(offsetof(LoRaData, yaw_cd) == 50, "LoRaData.yaw_cd moved");
static_assert(offsetof(LoRaData, q0) == 52, "LoRaData.q0 moved");
static_assert(offsetof(LoRaData, q1) == 54, "LoRaData.q1 moved");
static_assert(offsetof(LoRaData, q2) == 56, "LoRaData.q2 moved");
static_assert(offsetof(LoRaData, q3) == 58, "LoRaData.q3 moved");
static_assert(offsetof(LoRaData, speed) == 60, "LoRaData.speed moved");
static_assert(offsetof(LoRaData, sensor_health) == 62, "LoRaData.sensor_health moved");

static_assert(sizeof(GNSSData) == 42, "GNSSData wire size");
static_assert(offsetof(GNSSData, time_us) == 0, "GNSSData.time_us moved");
static_assert(offsetof(GNSSData, year) == 4, "GNSSData.year moved");
static_assert(offsetof(GNSSData, month) == 6, "GNSSData.month moved");
static_assert(offsetof(GNSSData, day) == 7, "GNSSData.day moved");
static_assert(offsetof(GNSSData, hour) == 8, "GNSSData.hour moved");
static_assert(offsetof(GNSSData, minute) == 9, "GNSSData.minute moved");
static_assert(offsetof(GNSSData, second) == 10, "GNSSData.second moved");
static_assert(offsetof(GNSSData, milli_second) == 11, "GNSSData.milli_second moved");
static_assert(offsetof(GNSSData, fix_mode) == 13, "GNSSData.fix_mode moved");
static_assert(offsetof(GNSSData, num_sats) == 14, "GNSSData.num_sats moved");
static_assert(offsetof(GNSSData, pdop_x10) == 15, "GNSSData.pdop_x10 moved");
static_assert(offsetof(GNSSData, lat_e7) == 16, "GNSSData.lat_e7 moved");
static_assert(offsetof(GNSSData, lon_e7) == 20, "GNSSData.lon_e7 moved");
static_assert(offsetof(GNSSData, alt_mm) == 24, "GNSSData.alt_mm moved");
static_assert(offsetof(GNSSData, vel_e_mmps) == 28, "GNSSData.vel_e_mmps moved");
static_assert(offsetof(GNSSData, vel_n_mmps) == 32, "GNSSData.vel_n_mmps moved");
static_assert(offsetof(GNSSData, vel_u_mmps) == 36, "GNSSData.vel_u_mmps moved");
static_assert(offsetof(GNSSData, h_acc_m) == 40, "GNSSData.h_acc_m moved");
static_assert(offsetof(GNSSData, v_acc_m) == 41, "GNSSData.v_acc_m moved");

static_assert(sizeof(NonSensorData) == 48, "NonSensorData wire size");
static_assert(offsetof(NonSensorData, time_us) == 0, "NonSensorData.time_us moved");
static_assert(offsetof(NonSensorData, q0) == 4, "NonSensorData.q0 moved");
static_assert(offsetof(NonSensorData, q1) == 6, "NonSensorData.q1 moved");
static_assert(offsetof(NonSensorData, q2) == 8, "NonSensorData.q2 moved");
static_assert(offsetof(NonSensorData, q3) == 10, "NonSensorData.q3 moved");
static_assert(offsetof(NonSensorData, roll_cmd) == 12, "NonSensorData.roll_cmd moved");
static_assert(offsetof(NonSensorData, e_pos) == 14, "NonSensorData.e_pos moved");
static_assert(offsetof(NonSensorData, n_pos) == 18, "NonSensorData.n_pos moved");
static_assert(offsetof(NonSensorData, u_pos) == 22, "NonSensorData.u_pos moved");
static_assert(offsetof(NonSensorData, e_vel) == 26, "NonSensorData.e_vel moved");
static_assert(offsetof(NonSensorData, n_vel) == 30, "NonSensorData.n_vel moved");
static_assert(offsetof(NonSensorData, u_vel) == 34, "NonSensorData.u_vel moved");
static_assert(offsetof(NonSensorData, flags) == 38, "NonSensorData.flags moved");
static_assert(offsetof(NonSensorData, rocket_state) == 39, "NonSensorData.rocket_state moved");
static_assert(offsetof(NonSensorData, baro_alt_rate_dmps) == 40, "NonSensorData.baro_alt_rate_dmps moved");
static_assert(offsetof(NonSensorData, pyro_status) == 42, "NonSensorData.pyro_status moved");
static_assert(offsetof(NonSensorData, apogee_flags) == 43, "NonSensorData.apogee_flags moved");
static_assert(offsetof(NonSensorData, sensor_health) == 44, "NonSensorData.sensor_health moved");

static_assert(sizeof(OutStatusQueryData) == 28, "OutStatusQueryData wire size");
static_assert(offsetof(OutStatusQueryData, ism6_low_g_fs_g) == 0, "OutStatusQueryData.ism6_low_g_fs_g moved");
static_assert(offsetof(OutStatusQueryData, ism6_high_g_fs_g) == 1, "OutStatusQueryData.ism6_high_g_fs_g moved");
static_assert(offsetof(OutStatusQueryData, ism6_gyro_fs_dps) == 3, "OutStatusQueryData.ism6_gyro_fs_dps moved");
static_assert(offsetof(OutStatusQueryData, ism6_rot_z_cdeg) == 5, "OutStatusQueryData.ism6_rot_z_cdeg moved");
static_assert(offsetof(OutStatusQueryData, mmc_rot_z_cdeg) == 7, "OutStatusQueryData.mmc_rot_z_cdeg moved");
static_assert(offsetof(OutStatusQueryData, format_version) == 9, "OutStatusQueryData.format_version moved");
static_assert(offsetof(OutStatusQueryData, hg_bias_x_cmss) == 10, "OutStatusQueryData.hg_bias_x_cmss moved");
static_assert(offsetof(OutStatusQueryData, hg_bias_y_cmss) == 12, "OutStatusQueryData.hg_bias_y_cmss moved");
static_assert(offsetof(OutStatusQueryData, hg_bias_z_cmss) == 14, "OutStatusQueryData.hg_bias_z_cmss moved");
static_assert(offsetof(OutStatusQueryData, b2r_code) == 16, "OutStatusQueryData.b2r_code moved");
static_assert(offsetof(OutStatusQueryData, b2r_mode) == 17, "OutStatusQueryData.b2r_mode moved");
static_assert(offsetof(OutStatusQueryData, b2r_q) == 18, "OutStatusQueryData.b2r_q moved");
static_assert(offsetof(OutStatusQueryData, iis2mdc_rot_z_cdeg) == 26, "OutStatusQueryData.iis2mdc_rot_z_cdeg moved");

static_assert(sizeof(ServoConfigData) == 22, "ServoConfigData wire size");
static_assert(offsetof(ServoConfigData, bias_us) == 0, "ServoConfigData.bias_us moved");
static_assert(offsetof(ServoConfigData, hz) == 8, "ServoConfigData.hz moved");
static_assert(offsetof(ServoConfigData, min_us) == 10, "ServoConfigData.min_us moved");
static_assert(offsetof(ServoConfigData, max_us) == 12, "ServoConfigData.max_us moved");
static_assert(offsetof(ServoConfigData, fin_min_deg) == 14, "ServoConfigData.fin_min_deg moved");
static_assert(offsetof(ServoConfigData, fin_max_deg) == 18, "ServoConfigData.fin_max_deg moved");

static_assert(sizeof(RollProfileData) == 76, "RollProfileData wire size");
static_assert(offsetof(RollProfileData, num_waypoints) == 0, "RollProfileData.num_waypoints moved");
static_assert(offsetof(RollProfileData, _pad) == 1, "RollProfileData._pad moved");
static_assert(offsetof(RollProfileData, waypoints) == 4, "RollProfileData.waypoints moved");

// LoRaChannelSetSelection is all-uint8_t (alignment 1, padding impossible),
// so a size pin fully constrains the layout without adding packed (which
// could have changed the in-memory layout for existing consumers).
static_assert(sizeof(LoRaChannelSetSelection) == 1 + LORA_SKIP_MASK_MAX_BYTES,
              "LoRaChannelSetSelection wire size");

static constexpr size_t SIZE_OF_LORA_DATA       = sizeof(LoRaData);
static constexpr size_t SIZE_OF_LORA_DATA_SI = sizeof(LoRaDataSI);

// Calculate max message size to get MAX_PAYLOAD and MAX_FRAME
static constexpr size_t P1 = SIZE_OF_ISM6HG256_DATA;
static constexpr size_t P2 = SIZE_OF_BMP585_DATA;
static constexpr size_t P3 = SIZE_OF_MMC5983MA_DATA;
static constexpr size_t P4 = SIZE_OF_GNSS_DATA;
static constexpr size_t P5 = SIZE_OF_POWER_DATA;
static constexpr size_t P6 = SIZE_OF_NON_SENSOR_DATA;
static constexpr size_t P7 = SIZE_OF_LORA_DATA;
static constexpr size_t P8 = sizeof(RollProfileData);
static constexpr size_t P9 = sizeof(FlightSnapshotData);  // largest payload (224 B as of snapshot v3)

static constexpr size_t M12   = (P1 > P2 ? P1 : P2);
static constexpr size_t M34   = (P3 > P4 ? P3 : P4);
static constexpr size_t M56   = (P5 > P6 ? P5 : P6);
static constexpr size_t M567  = (M56 > P7 ? M56 : P7);
static constexpr size_t M1234 = (M12 > M34 ? M12 : M34);
static constexpr size_t M_SENSOR = (M1234 > M567 ? M1234 : M567);

static constexpr size_t M_SENSOR_OR_PROFILE = (M_SENSOR > P8 ? M_SENSOR : P8);
static constexpr size_t MAX_PAYLOAD = (M_SENSOR_OR_PROFILE > P9 ? M_SENSOR_OR_PROFILE : P9);

// The flight settings snapshot (#165) rides the same I2S frame path, so it
// must also fit one payload.  Checked here since MAX_PAYLOAD is defined below
// the struct.
static_assert(sizeof(FlightSettingsData) <= MAX_PAYLOAD,
              "FlightSettingsData must fit one I2S frame");

// Frame: [0xAA][0x55][0xAA][0x55] + type + length + payload + CRC16
static constexpr size_t MAX_FRAME = 4 + 1 + 1 + MAX_PAYLOAD + 2;
#endif
