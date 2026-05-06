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
enum RocketState : uint8_t
{
    INITIALIZATION,
    READY,
    PRELAUNCH,
    INFLIGHT,
    LANDED
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

typedef struct
{
    float   rendezvous_mhz;                     // best-quietest scan freq, or fallback
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

// Pure orchestrator.  Computes rendezvous freq + skip-mask for the
// current operating BW from a (freq, rssi) scan grid.  See the comment
// block above for the threshold rule and FCC-floor handling.
//
// fallback_rendezvous_mhz is used when scan_count == 0 (e.g., scan
// never run yet, or skipped).  In that case skip_mask is left all-zero
// (no skips) and n_channels reflects the BW table.
static inline void loraSelectChannelSet(
    const float* scan_freqs, const int8_t* scan_rssi, size_t scan_count,
    float bw_khz, float fallback_rendezvous_mhz,
    LoRaChannelSetSelection* out)
{
    out->n_channels = loraChannelCount(bw_khz);
    for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) out->skip_mask[i] = 0;

    if (scan_count == 0 || out->n_channels == 0)
    {
        out->rendezvous_mhz = fallback_rendezvous_mhz;
        return;
    }

    // (1) Rendezvous = lowest-RSSI scan point.
    {
        size_t best = 0;
        for (size_t i = 1; i < scan_count; i++)
        {
            if (scan_rssi[i] < scan_rssi[best]) best = i;
        }
        out->rendezvous_mhz = scan_freqs[best];
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
    uint8_t  format_version;      // payload format version (2 = has HG bias)
    int16_t  hg_bias_x_cmss;     // high-g bias X, centi-m/s² (0.01 m/s² units)
    int16_t  hg_bias_y_cmss;     // high-g bias Y, centi-m/s²
    int16_t  hg_bias_z_cmss;     // high-g bias Z, centi-m/s²
} OutStatusQueryData;
static_assert(sizeof(OutStatusQueryData) == 16,
              "OutStatusQueryData must be 16 bytes");

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

} NonSensorData;

static_assert(sizeof(NonSensorData) == 43,
              "NonSensorData must be 43 bytes");

static constexpr uint8_t NSF_ALT_LANDED   = (1u << 0);
static constexpr uint8_t NSF_ALT_APOGEE   = (1u << 1);
static constexpr uint8_t NSF_VEL_APOGEE   = (1u << 2);
static constexpr uint8_t NSF_LAUNCH       = (1u << 3);
static constexpr uint8_t NSF_BURNOUT      = (1u << 4);
static constexpr uint8_t NSF_GUIDANCE     = (1u << 5);
static constexpr uint8_t NSF_PYRO1_ARMED  = (1u << 6);
static constexpr uint8_t NSF_PYRO2_ARMED  = (1u << 7);

// Pyro status byte bit masks
static constexpr uint8_t PSF_CH1_CONT  = (1u << 0);
static constexpr uint8_t PSF_CH2_CONT  = (1u << 1);
static constexpr uint8_t PSF_CH1_FIRED = (1u << 2);
static constexpr uint8_t PSF_CH2_FIRED = (1u << 3);
static constexpr uint8_t PSF_REBOOT_RECOVERY = (1u << 4);  // mid-flight reboot recovery occurred
// Reuse of this byte for a non-pyro signal: the FlightComputer's live
// guidance_enabled config. OutComputer uses this as the source of truth
// to avoid iOS/OUT/FC NVS caches silently diverging.
static constexpr uint8_t PSF_GUIDANCE_ENABLED = (1u << 5);

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

    RocketState rocket_state;

} NonSensorDataSI;

// --- LoRa Data ---

typedef struct __attribute__((packed)) 
{
    // little-endian signed 24-bit stored in 3 bytes
    uint8_t b0, b1, b2; 
} i24le_t;
static_assert(sizeof(i24le_t) == 3, "i24le_t must be 3 bytes");

// LoRa protocol version — bump on frame format changes
static constexpr uint8_t LORA_PROTO_VERSION = 1;

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
    uint8_t network_id;      // LoRa network namespace (0..255)
    uint8_t rocket_id;       // Source rocket ID within network (1..254, 0=unset, 255=broadcast)
    uint8_t next_channel_idx;// 0..N-1 = hop to that channel after this RX; 0xFF = stay

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

} LoRaData;

static_assert(sizeof(LoRaData) == 60,
              "LoRaData must be 60 bytes (3-byte header + 57-byte payload)");

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
    uint8_t network_id;             // Routing header: network namespace
    uint8_t rocket_id;              // Routing header: source rocket ID
    uint8_t next_channel_idx;       // Routing header: 0..N-1 hop target, 0xFF = stay
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
    uint8_t rocket_state;           // 3 bits       : states 0 through 4
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

// Pyro trigger modes
enum PyroTriggerMode : uint8_t {
    PYRO_TRIGGER_TIME_AFTER_APOGEE    = 0,
    PYRO_TRIGGER_ALTITUDE_ON_DESCENT  = 1,
};

// Pyro channel configuration (both channels, 12 bytes)
typedef struct __attribute__((packed))
{
    uint8_t  ch1_enabled;       // 0 = disabled, 1 = enabled
    uint8_t  ch1_trigger_mode;  // PyroTriggerMode
    float    ch1_trigger_value; // seconds (time mode) or meters (altitude mode)
    uint8_t  ch2_enabled;
    uint8_t  ch2_trigger_mode;
    float    ch2_trigger_value;
} PyroConfigData;
static_assert(sizeof(PyroConfigData) == 12, "PyroConfigData must be 12 bytes");

// Packed config data structures for BLE → I2C relay
typedef struct __attribute__((packed))
{
    int16_t bias_us[4];   // Per-servo bias in microseconds
    int16_t hz;           // PWM frequency
    int16_t min_us;       // Minimum pulse width
    int16_t max_us;       // Maximum pulse width
} ServoConfigData;
static_assert(sizeof(ServoConfigData) == 14, "ServoConfigData must be 14 bytes");

typedef struct __attribute__((packed))
{
    float kp;
    float ki;
    float kd;
    float min_cmd;
    float max_cmd;
} PIDConfigData;
static_assert(sizeof(PIDConfigData) == 20, "PIDConfigData must be 20 bytes");

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

typedef struct __attribute__((packed))
{
    float time_s;       // seconds after launch
    float angle_deg;    // target roll angle (deg)
} RollWaypoint;
static_assert(sizeof(RollWaypoint) == 8, "RollWaypoint must be 8 bytes");

typedef struct __attribute__((packed))
{
    uint8_t num_waypoints;      // 0 = no profile (rate-only mode)
    uint8_t _pad[3];            // alignment padding
    RollWaypoint waypoints[MAX_ROLL_WAYPOINTS];
} RollProfileData;
static_assert(sizeof(RollProfileData) == 68, "RollProfileData must be 68 bytes");

// --- Roll Control Config (runtime-configurable from app) ---
typedef struct __attribute__((packed))
{
    uint8_t  use_angle_control;   // 0 = rate-only (null roll), 1 = cascaded angle control w/ profile
    uint8_t  _pad;
    uint16_t roll_delay_ms;       // ms after launch before any roll control activates
} RollControlConfigData;
static_assert(sizeof(RollControlConfigData) == 4, "RollControlConfigData must be 4 bytes");

// --- Guidance Telemetry Data (sent at ~10 Hz during coast) ---
typedef struct __attribute__((packed))
{
    uint32_t time_us;
    int16_t  pitch_cmd_cdeg;    // guidance pitch command (centi-deg)
    int16_t  yaw_cmd_cdeg;      // guidance yaw command (centi-deg)
    int16_t  lateral_offset_cm; // lateral distance from pad vertical (cm)
    int16_t  pitch_fin_cdeg;    // pitch fin command after PID (centi-deg)
    int16_t  yaw_fin_cdeg;      // yaw fin command after PID (centi-deg)
    uint8_t  guid_flags;        // bit 0: guidance_active, bit 1: burnout_detected
} GuidanceTelemData;
static_assert(sizeof(GuidanceTelemData) == 15, "GuidanceTelemData must be 15 bytes");

static constexpr size_t SIZE_OF_GNSS_DATA = sizeof(GNSSData);
static constexpr size_t SIZE_OF_BMP585_DATA     = sizeof(BMP585Data);
static constexpr size_t SIZE_OF_ISM6HG256_DATA  = sizeof(ISM6HG256Data);
static constexpr size_t SIZE_OF_MMC5983MA_DATA  = sizeof(MMC5983MAData);
static constexpr size_t SIZE_OF_IIS2MDC_DATA    = sizeof(IIS2MDCData);
static constexpr size_t SIZE_OF_POWER_DATA      = sizeof(POWERData);
static constexpr size_t SIZE_OF_NON_SENSOR_DATA = sizeof(NonSensorData);
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

static constexpr size_t M12   = (P1 > P2 ? P1 : P2);
static constexpr size_t M34   = (P3 > P4 ? P3 : P4);
static constexpr size_t M56   = (P5 > P6 ? P5 : P6);
static constexpr size_t M567  = (M56 > P7 ? M56 : P7);
static constexpr size_t M1234 = (M12 > M34 ? M12 : M34);
static constexpr size_t M_SENSOR = (M1234 > M567 ? M1234 : M567);

static constexpr size_t MAX_PAYLOAD = (M_SENSOR > P8 ? M_SENSOR : P8);

// Frame: [0xAA][0x55][0xAA][0x55] + type + length + payload + CRC16
static constexpr size_t MAX_FRAME = 4 + 1 + 1 + MAX_PAYLOAD + 2;
#endif
