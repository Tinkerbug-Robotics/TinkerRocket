// SbfParser.h — Septentrio mosaic-G5 output stream: SBF blocks and command replies (pure, host-compilable)
//
// Bytes in, CRC-checked SBF blocks and ASCII reply lines out. Free of every
// ESP-IDF / FreeRTOS dependency so the framing, the CRC, the resync after a
// corrupt block and the field decoders are pinned by host tests
// (tests_cpp/test_sbf_parser.cpp). The UART glue lives in
// TR_GNSSReceiverMosaic_Serial.cpp.
//
// Reference: mosaic-G5 Reference Guide, firmware 1.0.1 (2025-11-13):
//   SBF block:  "$@", CRC u2, ID u2 (block number bits 0-12, revision 13-15),
//               Length u2 (the whole block, a multiple of 4), TOW u4 (ms),
//               WNc u2, body. The CRC is CRC-CCITT (0x1021, seed 0, no
//               reflection, no final XOR) over ID through the last byte (4.1.1).
//   Replies:    "$R: <command>" for a valid set/get/exe command, "$R? ..." for
//               an invalid one, "$R; <command>" for a lst command, then lines,
//               then the prompt "COM1>" (no line end). A reply to set/get/exe
//               is atomic: no SBF block lands inside it (3.1.3).
// Everything on this port that is not an SBF block is text, and every text
// item starts a new line with a '$' or follows a CR/LF, so the stream splits
// on "$@" and on line ends.
#ifndef SBFPARSER_H
#define SBFPARSER_H

#include <cstddef>
#include <cstdint>

#include <RocketComputerTypes.h>

namespace mosaic
{

// SBF block numbers this firmware asks for (Reference Guide, appendix B).
enum SbfBlock : uint16_t
{
    SBF_MEAS_EPOCH      = 4027,
    SBF_MEAS_EXTRA      = 4000,
    SBF_PVT_GEODETIC    = 4007,
    SBF_DOP             = 4001,
    SBF_RECEIVER_STATUS = 4014,
    SBF_RECEIVER_TIME   = 5914,
    SBF_XPPS_OFFSET     = 5911,
    SBF_DISK_STATUS     = 4059,
};

static constexpr uint32_t kTowDnu = 4294967295U;   // TOW do-not-use
static constexpr uint16_t kWncDnu = 65535U;        // WNc do-not-use

// CRC-CCITT as SBF computes it (polynomial 0x1021, seed `crc`).
uint16_t crc16Ccitt(const uint8_t* data, size_t len, uint16_t crc = 0);

// What the parser hands its sink. `index` is the position, in the slice
// passed to feed(), of the byte that completed the item, so the caller can
// tell when that byte arrived.
class SbfSink
{
public:
    virtual ~SbfSink() = default;
    // A whole block whose length and CRC check (len = its Length field).
    virtual void onBlock(const uint8_t* block, size_t len, size_t index) = 0;
    // A complete text line, without its CR/LF, NUL-terminated.
    virtual void onLine(const char* line, size_t index) = 0;
    // The command prompt: `port` is the connection it names ("COM1"), or
    // "STOP" after a command that halts the receiver.
    virtual void onPrompt(const char* port, size_t index) = 0;
};

class SbfParser
{
public:
    // The longest block kept. The mosaic's MeasExtra is 16 B a signal and
    // MeasEpoch about 14 (2026-10-15 spin-rig capture: 1,812 and 1,652 B at
    // ~100 signals), so 4 kB holds either at 250. A longer Length is taken
    // for a false sync: one stray "$@" can then cost at most this much
    // waiting before its CRC fails.
    static constexpr size_t kMaxBlock = 4096;
    static constexpr size_t kMaxLine  = 160;

    // Feed a slice of received bytes. Items complete in order, through the
    // sink, as their last byte is consumed; bytes may arrive in any slicing.
    void feed(const uint8_t* data, size_t n, SbfSink& sink);

    // Drop any partial block or line (after a baud change, so bytes sampled
    // at the old rate cannot glue onto the next real item). Counters survive.
    void reset();

    uint32_t blocks() const { return blocks_; }          // CRC-valid blocks
    uint32_t crcFails() const { return crc_fails_; }      // blocks whose CRC failed
    uint32_t badLengths() const { return bad_lengths_; }  // "$@" with an impossible Length
    uint32_t longLines() const { return long_lines_; }    // text lines cut at kMaxLine

private:
    void step(uint8_t b, SbfSink& sink);
    void textByte(uint8_t b, SbfSink& sink);
    void flushLine(SbfSink& sink);
    // A false sync: give back everything after its '$' for a second look.
    void resync();

    enum class State : uint8_t { TEXT, DOLLAR, BLOCK };
    State    state_ = State::TEXT;
    uint8_t  buf_[kMaxBlock] = {};
    size_t   n_ = 0;
    size_t   want_ = 0;           // the Length of the block being read (0 until its header is in)
    char     line_[kMaxLine + 1] = {};
    size_t   line_n_ = 0;
    bool     line_cut_ = false;
    size_t   index_ = 0;          // the slice position of the byte being consumed

    // Bytes handed back by resync(), read before any new input. A resync
    // gives back at most one block's worth, and one inside a replay only
    // what that replay has not read yet, so kMaxBlock always holds them.
    uint8_t  back_[kMaxBlock] = {};
    size_t   back_head_ = 0;
    size_t   back_n_ = 0;

    uint32_t blocks_ = 0;
    uint32_t crc_fails_ = 0;
    uint32_t bad_lengths_ = 0;
    uint32_t long_lines_ = 0;
};

// ── Field decoders ─────────────────────────────────────────────────────────
// Each takes a whole block as onBlock() delivered it and returns false if it
// is not that block or is too short for the revision it claims.

struct SbfHeader
{
    uint16_t number = 0;     // block number (ID bits 0-12)
    uint8_t  revision = 0;   // ID bits 13-15
    uint16_t length = 0;
    uint32_t tow_ms = kTowDnu;
    uint16_t wnc = kWncDnu;
};
bool sbfHeader(const uint8_t* b, size_t len, SbfHeader& out);

// PVTGeodetic (4007) rev 2: the receiver's position, velocity and time.
struct PvtGeodetic
{
    uint32_t tow_ms = kTowDnu;
    uint16_t wnc = kWncDnu;
    uint8_t  mode = 0;          // bits 0-3: 0 none, 1 stand-alone, 2 DGNSS, 4/5 RTK, 6 SBAS...; bit 7: 2D
    uint8_t  error = 0;         // 0 = a solution; otherwise why not
    double   lat_rad = 0.0;
    double   lon_rad = 0.0;
    double   height_m = 0.0;    // above the ellipsoid
    float    undulation_m = 0.0f;   // the geoid above the ellipsoid
    float    vn = 0.0f, ve = 0.0f, vu = 0.0f;   // m/s
    double   rx_clk_bias_ms = 0.0;  // receiver time minus system time
    float    rx_clk_drift_ppm = 0.0f;
    uint8_t  time_system = 255;
    uint8_t  nr_sv = 0;
    uint16_t latency_100us = 65535;   // epoch to block, receiver processing only
    uint16_t h_acc_cm = 65535;        // 2DRMS: 95 % of horizontal errors fall inside
    uint16_t v_acc_cm = 65535;        // 2-sigma vertical
    bool     has_solution = false;    // mode != 0, error == 0 and the fields are not do-not-use
};
bool decodePvtGeodetic(const uint8_t* b, size_t len, PvtGeodetic& out);

// ReceiverTime (5914): UTC and the leap seconds.
struct ReceiverTime
{
    int8_t  delta_ls = -128;    // GPS - UTC, s; -128 = not yet known
    uint8_t sync_level = 0;     // bit 0 WNSET, 1 TOWSET, 2 FINETIME
};
bool decodeReceiverTime(const uint8_t* b, size_t len, ReceiverTime& out);

// DOP (4001).
struct Dop
{
    uint8_t  nr_sv = 0;
    uint16_t pdop_x100 = 0;     // 0 = not available
};
bool decodeDop(const uint8_t* b, size_t len, Dop& out);

// ReceiverStatus (4014) rev 1, the fields the boot console reports.
struct ReceiverStatus
{
    uint8_t  cpu_load = 0;      // %
    uint8_t  ext_error = 0;
    uint32_t uptime_s = 0;
    uint32_t rx_state = 0;
    uint32_t rx_error = 0;
    int16_t  temp_c = -32768;   // -32768 = not reported
};
bool decodeReceiverStatus(const uint8_t* b, size_t len, ReceiverStatus& out);

// xPPSOffset (5911): sent just after each pulse of the first PPS output.
struct XppsOffset
{
    uint32_t tow_ms = kTowDnu;
    uint16_t wnc = kWncDnu;
    uint8_t  sync_age_s = 0;
    uint8_t  time_scale = 0;    // 1 GPS, 2 UTC, 3 receiver time, ...
    float    offset_ns = 0.0f;  // the pulse's error against its true position
};
bool decodeXppsOffset(const uint8_t* b, size_t len, XppsOffset& out);

// DiskStatus (4059): the internal microSD card (DiskID 1).
struct DiskStatus
{
    bool     present = false;   // a DiskID 1 sub-block was in the block
    uint8_t  status = 0;        // bit 0 mounted, 1 full, 2 activity, 3 logging, 4 mounting, 5 formatting
    uint8_t  error = 0;         // 0 none, 1 partition too large, 2 no partition, 3 fsck failed, 4 in use over USB
    uint64_t used_bytes = 0;
    uint32_t size_mb = 0;
};
bool decodeDiskStatus(const uint8_t* b, size_t len, DiskStatus& out);

// GPS week and time of week, less the leap seconds, as a UTC calendar date
// and time (proleptic Gregorian; GPS time began 1980-01-06).
struct UtcTime
{
    uint16_t year = 0;
    uint8_t  month = 0, day = 0, hour = 0, minute = 0, second = 0;
    uint16_t millisecond = 0;
};
void gpsToUtc(uint16_t wnc, uint32_t tow_ms, int leap_s, UtcTime& out);

// The fix as the flight computer's GNSSData carries it, in the u-blox
// driver's terms: fix_mode 3 for a 3D solution, 2 when the receiver held
// the height (Mode bit 7), 0 without one; alt above the geoid (Height less
// Undulation); ENU velocity; the accuracies HALVED to the u-blox's one-sigma
// scale (the receiver states 95 % figures, and the EKF scales its GNSS noise
// by h_acc / 3 m on the u-blox's scale) and rounded to whole meters as that
// driver rounds, 255 when unknown. time_us is left 0 for the driver to stamp.
void gnssDataFromPvt(const PvtGeodetic& pvt, int leap_s, uint16_t pdop_x100, GNSSData& out);

}  // namespace mosaic

#endif  // SBFPARSER_H
