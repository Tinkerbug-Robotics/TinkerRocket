// SkyTraqBinary.h — the SkyTraq PX1105R's binary stream: frames, the commands
// this driver sends, and the messages it reads (pure, host-compilable)
//
// Bytes in, checksum-valid message bodies out, and the bodies of the four
// messages the raw-GNSS filter and the fix need decoded into plain structs:
//   0xE5 extended raw measurements (pseudorange, carrier, Doppler, C/N0 and
//        the channel's flags, per signal),
//   0xDF receiver navigation state (its own fix, ECEF, and its clock),
//   0xE7 channel status (which satellites its fix used),
//   0x83 / 0x84 ACK / NACK, and the replies to the queries the setup reads
//        back (SkyTraqLink.h).
// The navigation subframes (0xE0 GPS, 0xE2 BeiDou D1, 0xE6 Galileo I/NAV) are
// SkyTraqNav.h's. Free of ESP-IDF and FreeRTOS, so the framing, the resync
// after a corrupt frame and the decoders are pinned by host tests
// (tests_cpp/test_skytraq_binary.cpp); the UART glue is
// TR_GNSSReceiverPX1105R_Serial.cpp.
//
// Reference: SkyTraq AN0039 (binary messages, v1.4.x) and AN0037 (RTK
// receivers), as the receiver test rig's tools use them
// (gnss repo cocom-rig/skytraq_raw.py, skytraq_binary.py, sdr/px1105r_run.py,
// sim/tinkerrocket_gnss/gnss_raw.py), and checked on the PX1105R's own
// captures (bench-backups/px1105r_outdoor_20hz_1200s_2026-09-26):
//   frame      A0 A1 | length u16 BE | body | XOR of the body | 0D 0A
//   numbers    big-endian; float64 / float32 IEEE
// Text (the factory NMEA) can sit between frames; it is counted, not kept.
#ifndef SKYTRAQ_BINARY_H
#define SKYTRAQ_BINARY_H

#include <cstddef>
#include <cstdint>

namespace skytraq
{

// Message IDs (the first byte of a body).
enum MsgId : uint8_t
{
    MSG_SW_VERSION    = 0x80,   // reply to 0x02
    MSG_ACK           = 0x83,
    MSG_NACK          = 0x84,
    MSG_POS_RATE      = 0x86,   // reply to 0x10
    MSG_BIN_MEAS_CFG  = 0x89,   // reply to 0x1F
    MSG_ELEV_CNR_MASK = 0xB0,   // reply to 0x2F
    MSG_POWER_MODE    = 0xB9,   // reply to 0x15
    MSG_GPS_SUBFRAME  = 0xE0,
    MSG_BDS_D1        = 0xE2,
    MSG_RCV_STATE     = 0xDF,
    MSG_EXT_RAW       = 0xE5,
    MSG_GAL_PAGE      = 0xE6,
    MSG_SV_CH_STATUS  = 0xE7,
    MSG_SV_ELV_AZM    = 0xE8,
    MSG_SUB_64        = 0x64,   // sub-ID messages: nav mode (0x8B reply), constellations (0x8C)
    MSG_SUB_6A        = 0x6A,   // RTK (0x83 reply: mode)
};

// The longest body kept. An 0xE5 is 14 + 31 B a signal; the PX1105R lists
// 36-45 signals at 20 Hz on the open sky (L1 and L5 of ~20 satellites,
// 2026-09-26), ~1.4 kB, so 2.4 kB holds 76. A longer length field is a false
// sync: one stray A0 A1 then costs at most this much before its checksum fails.
static constexpr size_t kMaxBody = 2400;

// A whole frame around `body` (n bytes) into out (cap bytes): n + 7 bytes, or
// 0 if it does not fit.
size_t buildFrame(const uint8_t* body, size_t n, uint8_t* out, size_t cap);

// What the parser hands its sink. `index` is the position, in the slice passed
// to feed(), of the byte that completed the frame (its 0x0A), so the caller
// can tell when that byte arrived.
class FrameSink
{
public:
    virtual ~FrameSink() = default;
    // A body whose length, checksum and 0D 0A check.
    virtual void onFrame(const uint8_t* body, size_t len, size_t index) = 0;
};

class FrameParser
{
public:
    // Feed a slice of received bytes; frames complete in order, through the
    // sink, as their last byte is consumed. Any slicing.
    void feed(const uint8_t* data, size_t n, FrameSink& sink);
    // Drop a partial frame (after a baud change). Counters survive.
    void reset();

    uint32_t frames() const { return frames_; }          // checksum-valid frames
    uint32_t badChecksums() const { return bad_cs_; }     // a frame whose XOR or 0D 0A failed
    uint32_t badLengths() const { return bad_len_; }      // A0 A1 with a length of 0 or over kMaxBody
    uint32_t textLines() const { return text_lines_; }    // NMEA (or other text) lines between frames
    uint32_t nmeaLines() const { return nmea_lines_; }    //   of which started "$G" or "$B" (a talker)

private:
    void step(uint8_t b, FrameSink& sink);
    // A false sync (a bad length, checksum or end): give back the candidate's
    // bytes from its next A0 on for a second look, before any new input.
    void resync();
    void textByte(uint8_t b);

    enum class State : uint8_t { IDLE, A1, LEN_HI, LEN_LO, BODY, CS, CR, LF };
    State    state_ = State::IDLE;
    uint8_t  raw_[kMaxBody + 8] = {};   // the candidate frame from its A0
    size_t   raw_n_ = 0;
    size_t   want_ = 0;                 // its length field
    uint8_t  cs_ = 0;
    size_t   index_ = 0;
    // bytes given back by resync(), read before new input
    uint8_t  back_[kMaxBody + 8] = {};
    size_t   back_pos_ = 0;
    size_t   back_n_ = 0;
    // text between frames, for the counts (the probe tells NMEA from noise)
    char     text_[2] = {};
    uint8_t  text_n_ = 0;
    bool     text_live_ = false;

    uint32_t frames_ = 0, bad_cs_ = 0, bad_len_ = 0, text_lines_ = 0, nmea_lines_ = 0;
};

// ── 0xE5 extended raw measurements (version 1) ──────────────────────────────
// Header: [1] version [2] IOD [3..4] receiver week [5..8] receiver time of
// week, ms [9..10] measurement period, ms [11] measurement indicator
// [12] reserved [13] the number of signals; then 31 B each:
//   [0] GNSS type (bits 0-3: 0 GPS, 1 SBAS, 2 GLONASS, 3 Galileo, 4 QZSS,
//       5 BeiDou, 6 NavIC) and signal type (bits 4-7: 0 the L1 band's,
//       C/A / E1 / B1I; 4 the L5 band's, L5 / E5a / B2a on this receiver)
//   [1] SVID  [2] bits 0-3 GLONASS frequency, bits 4-7 the lock time
//       indicator (lockTimeMinMs)  [3] C/N0, dB-Hz
//   [4..11] pseudorange, m  [12..19] accumulated carrier, cycles
//   [20..23] Doppler, Hz (whole hertz, truncated toward zero: dopplerHz)
//   [24..26] the pseudorange, carrier and Doppler standard deviation indices
//   [27..28] channel indicator (kInd*)  [29..30] reserved
// The receiver time is its own clock: the GPS instant of an epoch is the time
// of week less the clock bias of 0xDF (which follows each 0xE5), as with the
// mosaic's MeasEpoch. The pseudorange and carrier carry that bias too.
static constexpr uint16_t kIndPr       = 0x0001;   // pseudorange available
static constexpr uint16_t kIndDoppler  = 0x0002;   // Doppler available
static constexpr uint16_t kIndCarrier  = 0x0004;   // carrier available
static constexpr uint16_t kIndSlip     = 0x0008;   // cycle slip possible
static constexpr uint16_t kIndCoherent = 0x0010;   // coherent integration over 10 ms
static constexpr uint16_t kIndHalfCyc  = 0x0020;   // half-cycle ambiguity not yet resolved

static constexpr uint8_t kGnssGps = 0, kGnssSbas = 1, kGnssGlo = 2, kGnssGal = 3, kGnssQzs = 4, kGnssBds = 5,
                         kGnssNavic = 6;
static constexpr uint8_t kSigL1 = 0, kSigL5 = 4;

struct RawCell
{
    uint8_t  gnss = 0;          // GNSS type (bits 0-3 of [0])
    uint8_t  sig = 0;           // signal type (bits 4-7 of [0])
    uint8_t  svid = 0;
    uint8_t  lock_ind = 0;      // bits 4-7 of [2]
    uint8_t  cn0 = 0;           // dB-Hz
    uint16_t ind = 0;           // channel indicator
    double   pr_m = 0.0;
    double   carrier_cyc = 0.0;
    float    doppler_hz = 0.0f; // as sent
    uint8_t  pr_std = 0, carrier_std = 0, doppler_std = 0;
};

struct RawE5
{
    static constexpr int kMaxCells = 72;
    uint8_t  version = 0;
    uint8_t  iod = 0;
    uint16_t week = 0;          // the receiver's GPS week (full)
    uint32_t tow_ms = 0;        // its time of week, ms
    uint16_t period_ms = 0;
    uint8_t  meas_ind = 0;
    uint8_t  n_listed = 0;      // signals the header lists
    int      n = 0;             // cells kept (at most kMaxCells)
    RawCell  cell[kMaxCells];
};

// False for another message, a version other than 1, or a body shorter than
// its signal count says.
bool decodeRawE5(const uint8_t* body, size_t len, RawE5& out);

// The lock time a lock time indicator promises, ms (a lower bound). From the
// PX1105R's captures: an indicator of k never came with a lock older than
// 100 ms * 2^k (the 20-minute static run's maxima at k = 11, 12, 13 are
// exactly 204.8, 409.6 and 819.2 s), and it climbs one a doubling, so k means
// at least 50 ms * 2^k; 0 is a fresh lock.
uint32_t lockTimeMinMs(uint8_t lock_ind);

// The Doppler as the range rate needs it, Hz. The PX1105R sends whole hertz
// truncated toward zero (half a hertz short on average, -0.095 m/s at L1:
// the carrier phase shows it, 2026-09-26), so a whole value moves to the
// middle of its step; a fractional value (a receiver that does not round) is
// left alone.
double dopplerHz(float sent);

// ── 0xDF receiver navigation state (81 B) ───────────────────────────────────
// [1] IOD [2] navigation state [3..4] GPS week [5..12] GPS time of week, s
// [13..36] ECEF X, Y, Z, m (float64) [37..48] ECEF velocity, m/s (float32)
// [49..56] clock bias, m (float64) [57..60] clock drift, m/s (float32)
// [61..80] GDOP, PDOP, HDOP, VDOP, TDOP (float32)
// Its time of week is GPS time: the 0xE5 of the same epoch's time of week
// less this bias (569303.2997 = 569303.300 - 91751 m / c on the 09-26 capture).
static constexpr uint8_t kNavNoFix = 0, kNavPrediction = 1, kNavFix2d = 2, kNavFix3d = 3, kNavDiff = 4;

struct RcvState
{
    uint8_t  iod = 0;
    uint8_t  nav_state = 0;
    uint16_t week = 0;
    double   tow_s = 0.0;
    double   ecef_m[3] = {};
    float    vel_mps[3] = {};
    double   clk_bias_m = 0.0;
    float    clk_drift_mps = 0.0f;
    float    gdop = 0, pdop = 0, hdop = 0, vdop = 0, tdop = 0;
};

bool decodeRcvState(const uint8_t* body, size_t len, RcvState& out);

// ── 0xE7 channel status ─────────────────────────────────────────────────────
// [1] version [2] IOD [3] the number of channels; 7 B each: [0] channel
// [1] GNSS (bits 0-3) and signal (4-7) [2] SVID [3] status [4] URA
// [5] C/N0 (int8) [6] channel status: bit 4 used in the fix.
// Summed here to what the fix reports: satellites used (one a satellite,
// whatever signals), and tracked.
struct ChannelSummary
{
    uint8_t iod = 0;
    uint8_t channels = 0;       // listed
    uint8_t used = 0;           // satellites with a signal used in the fix
    uint8_t tracked = 0;        // satellites with a signal at C/N0 > 0
};

bool decodeSvChStatus(const uint8_t* body, size_t len, ChannelSummary& out);

// ── ACK / NACK ──────────────────────────────────────────────────────────────
// [0] 0x83 / 0x84 [1] the message ID it answers [2] its sub-ID, for the
// sub-ID commands (0x64, 0x6A) where the receiver sends one.
// True when body is an ACK or NACK of `id` (and of `sub`, when sub >= 0 and
// the body carries one); `acked` says which.
bool isAckOf(const uint8_t* body, size_t len, uint8_t id, int sub, bool& acked);

// Big-endian readers, for the decoders here and in SkyTraqLink / SkyTraqNav.
uint16_t be16(const uint8_t* p);
uint32_t be32(const uint8_t* p);
float    beF32(const uint8_t* p);
double   beF64(const uint8_t* p);

}  // namespace skytraq

#endif  // SKYTRAQ_BINARY_H
