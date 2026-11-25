// SdrRtcm.h — the gnss-sdr-dev board's RTCM3 stream (pure, host-compilable)
//
// TinkerRocket's own receiver board (gnss repo, firmware/gnss-sdr-dev) sends a
// flight computer nothing but RTCM 3 frames, back to back, on its J401 pin 2
// at 2 Mbaud 8N1, and ignores what it is sent:
//   - MSM7 1077 (GPS), 1097 (Galileo E1-C) and 1127 (BeiDou B1C pilot, signal
//     ID 31), one set an epoch with the multiple-message bit on all but the
//     last: 100 epochs a second in its measurement mode, 10 in its fix mode;
//   - TinkerRocket's message 4001 (in the range RTCM keeps for manufacturers,
//     not registered): sub-type 1 after each epoch's MSMs (the GPS week, the
//     epoch's time of week, the clock's steps, the observables' count, the
//     correlator's sample count, then the ranges its checks left out and the
//     satellites it marked: up to 78 bytes), sub-type 2 after each fix (ECEF
//     position and velocity, clock bias and drift, PDOP, residual rms),
//     sub-type 3 its state before its first fix, sub-type 4 the GPS broadcast
//     ionosphere it has (RTCM has no message for it), when it changes and
//     again every 30 s, sub-type 5 its IMU's mounting (below);
//   - its ephemerides, 1019, 1046 and 1042, each new one and again every 30 s.
// It sends nothing before its first fix (no time, no epochs).
//
// Rtcm3Framer pulls the frames whose CRC-24Q checks out of the byte stream,
// in any slicing. A frame that fails is given up one byte at a time, so a
// frame that began inside it (after bytes lost on the line) is still found.
// The rest reads the few fields of 4001 the flight computer looks at. Shared
// by the boot probe (gnss_detect.cpp) and the driver
// (TR_GNSSReceiverSdr_Serial.cpp); host-tested in tests_cpp/test_sdr_rtcm.cpp.
#ifndef SDRRTCM_H
#define SDRRTCM_H

#include <cstddef>
#include <cstdint>

namespace sdr_rtcm
{

static constexpr uint32_t kBaud        = 2000000;   // the board's `rx out uart 2000000`
static constexpr uint16_t kTrMessage   = 4001;      // TinkerRocket's message
static constexpr uint8_t  kTrEpoch     = 1;         // its sub-types
static constexpr uint8_t  kTrFix       = 2;
static constexpr uint8_t  kTrStatus    = 3;         // before the board has GPS time: once a second
static constexpr uint8_t  kTrIono      = 4;         // the GPS broadcast ionosphere it has
static constexpr uint8_t  kTrAlign     = 5;         // its IMU's mounting: once a second, and on each change
static constexpr size_t   kMaxFrame    = 3 + 1023 + 3;   // preamble and length, the payload, the CRC

// The message number of a whole frame (the 12 bits after its 3-byte header);
// 0 when it is too short to have one.
uint16_t messageNumber(const uint8_t* frame, size_t len);

// 4001's sub-type (the 4 bits after the number); 0 for any other message.
uint8_t trSubType(const uint8_t* frame, size_t len);

// A 4001 sub-type 1 (its layout: receiver/core/nav/rtcm_msm.c
// rtcm_tr_epoch_encode in the gnss repo). Its fields to the 48-bit sample
// came first; the tail after it (since 2026-11-02: the ranges the board's
// checks left out, and the satellites it marked) is read where the frame has
// it, and a frame without it decodes with none.
static constexpr int     kMaxMarks = 32;   // RTCM_TR_MAX_MARK
static constexpr uint8_t kMarkFix  = 1;    // the last fix's residual test left this pseudorange out
static constexpr uint8_t kMarkNew  = 2;    // its channel started under the board's settling time ago
struct TrMark
{
    uint8_t sys = 0;       // gnss_sys_t: 0 GPS, 1 Galileo, 2 BeiDou
    uint8_t prn = 0;
    uint8_t flags = 0;     // kMark*
};
struct TrEpoch
{
    uint16_t week = 0;          // GPS week
    uint32_t tow_ms = 0;        // the epoch's MSMs' time tag, GPS time of week
    int32_t  shift_ns = 0;      // the tag minus the receiver's time at its latch
    uint16_t clk_steps = 0;     // the receiver clock's steps since the board started (wrapping)
    double   clk_step_m = 0.0;  // their sum, m (to the mm): every pseudorange and phase carries it
    uint8_t  mode = 0;          // 0: the fix's epochs; 1: the measurement mode's
    uint8_t  nobs = 0;          // observables in its MSMs
    uint64_t sample = 0;        // the latch's correlator sample count (48 bits)
    bool     has_tail = false;  // the frame carries the counts below
    uint8_t  ndrop = 0;         // ranges the board's checks left out of the MSMs
    uint8_t  nmark = 0;         // marks that follow (at most kMaxMarks)
    TrMark   mark[kMaxMarks];
};
bool decodeTrEpoch(const uint8_t* frame, size_t len, TrEpoch& out);

// And of a sub-type 2. The position is the flight image's place for the
// attitude it sends the board (sdr_aid_policy.h): the board's own fix, so its
// NED is the board's.
struct TrFix
{
    uint16_t week = 0;
    uint32_t tow_ms = 0;
    int32_t  sub_ns = 0;        // the fix's time is tow_ms plus this
    bool     valid = false;
    bool     vel_valid = false;
    uint8_t  nsat = 0;
    double   pos[3] = {};       // the board's antenna, ECEF, m (to 0.1 mm)
    double   vel[3] = {};       // ECEF, m/s (to 0.1 mm/s)
    double   clk_bias_m = 0.0, clk_drift_mps = 0.0;
    double   pdop = 0.0, resid_rms_m = 0.0;
};
bool decodeTrFix(const uint8_t* frame, size_t len, TrFix& out);

// And of a sub-type 3: the board's state while it has no GPS time yet (its
// first fix), when it sends nothing else, so a listener knows it is there.
struct TrStatus
{
    uint32_t fw = 0;          // its firmware's commit (8 hex digits)
    bool     fw_dirty = false;
    uint32_t uptime_s = 0;
    uint8_t  ntrack = 0;      // channels tracking, and with their PLL locked
    uint8_t  nlock = 0;
    bool     time_valid = false;
    bool     fix_valid = false;
};
bool decodeTrStatus(const uint8_t* frame, size_t len, TrStatus& out);

// And of a sub-type 4: the GPS broadcast (Klobuchar) ionosphere the board
// has, as GPS broadcasts it (alpha0..3, beta0..3: RawMeasBuilder::
// ionoFromBroadcast's bytes), where it came from and when the board took it.
// After a fix when it changes, and every 30 s for a late listener.
static constexpr uint8_t kIonoLnav = 1;   // decoded from GPS LNAV (subframe 4 page 18)
static constexpr uint8_t kIonoLoad = 2;   // a navigation file's header (`rx eph load`)
struct TrIono
{
    uint16_t week = 0;        // the GPS week and second of week the board took it
    uint32_t tow_s = 0;
    uint8_t  src = 0;         // kIonoLnav, kIonoLoad
    int8_t   raw[8] = {};
    // The GPS day it was taken: days since 1980-01-06 (ShadowIono::gps_day)
    uint16_t gpsDay() const { return static_cast<uint16_t>(week * 7u + tow_s / 86400u); }
};
bool decodeTrIono(const uint8_t* frame, size_t len, TrIono& out);

// And of a sub-type 5: the board IMU's mounting in the flight computer and
// its accelerometer's calibration, which it finds from the flight computer's
// rests (4002 sub-type 5, its own IMU's mean over the same still window
// beside each: the gnss repo's gnss/imu_align.h), and what it keeps in its
// NVS. Once a second, before GPS time too, and within 100 ms of a change (a
// rest paired, a command, a save). Layout: the gnss repo's receiver/README.md
// ("The RTCM stream"); scales below. A frame shorter than its poses need is
// not decoded; bits after them (a later revision's) are left.
struct TrAlign
{
    static constexpr int kPoses = 8;
    enum : uint8_t { kHaveNone = 0, kHaveTilt = 1, kHaveWhole = 2 };
    enum : uint8_t { kFrameBody = 0, kFrameImu = 1 };   // what the mounting turns U10 into
    uint8_t seq = 0;          // this sub-type's frames sent, mod 256
    uint8_t gen = 0;          // the solution's generation: +1 on every change, mod 256
    uint8_t imu = 0;          // the IMU watch: 0 off (no pairing), 1 watching, 2 aiding
    bool    time = false;     // the board can place a rest: GPS time, or a tq-0 rest's arrival
    uint8_t have = 0;         // kHave*
    uint8_t from = 0;         // the one in use: 0 none, 1 this boot's rests, 2 the saved one,
                              // 3 the saved one with this boot's rests added
    uint8_t saved = 0;        // 0 nothing saved, 1 saved and the same as in use, 2 saved but
                              // the one in use differs
    bool    autosave = false; // the board saves by its own rule this boot (any 4002/6 turns it off)
    uint8_t frame = 0;        // kFrame*: the flight computer's body, or its IMU's own axes
    uint8_t n_pose = 0;       // poses held, 0-8
    uint8_t cal_axes = 0;     // bit j: the board's axis j calibrated
    uint8_t last_rest = 0;    // the last rest's fate: 0 none yet, 1 paired, 2 the board moving,
                              // 3 not covered by its still windows, 4 never reached, 5 not
                              // 1 g, 6 no time to place it, 7 the IMU off
    uint8_t n_rest = 0;       // rests received, mod 256
    uint8_t n_pair = 0;       // rests paired, mod 256
    uint8_t cmd_id = 0;       // the last 4002 sub-type 6 it acted on: its id, op and result
    uint8_t cmd_op = 0;
    uint8_t cmd_res = 0;      // 0 none yet, 1 done, 2 nothing to save, 3 refused (a flight
                              // phase, a test frame, or its IMU off), 4 its NVS write failed,
                              // 5 an unknown op
    double  spread_deg = 0;   // whole: the poses apart (1 deg)
    double  resid_deg = 0;    // the poses off the solution, the largest (0.02 deg; 5.1: that or more)
    double  cal_rms = 0;      // the calibration's fit, rms, m/s^2 (1e-3; 0.255: that or more)
    double  q[4] = {1, 0, 0, 0};          // U10 to `frame`, Hamilton, scalar first (2^-14)
    double  scale[3] = {1, 1, 1};         // each board axis's: f_read = s f_true + b (1e-5 on s - 1)
    double  bias[3] = {};                 // m/s^2 (1e-4)
    double  moved_deg = 0;    // from 3 only (a saved rotation with this boot's rests): the rail
                              // pose off the saved rotation (0.02 deg; 5.1: that or more); else 0
    struct Pose
    {
        double  dir[3] = {};  // the specific force's direction, `frame`'s axes (1/127)
        uint8_t pairs = 0;    // the rests paired into it (255: that or more)
    } pose[kPoses];
};
bool decodeTrAlign(const uint8_t* frame, size_t len, TrAlign& out);

class Rtcm3Framer
{
public:
    struct Sink
    {
        virtual ~Sink() = default;
        // A frame from its 0xD3 through its CRC, whose CRC checks. `after`:
        // the bytes fed since its last byte (0 when it ended on the last one).
        virtual void onFrame(const uint8_t* frame, size_t len, size_t after) = 0;
    };

    void reset() { n_ = 0; }
    void feed(const uint8_t* data, size_t n, Sink& sink);

    uint32_t frames() const   { return frames_; }
    uint32_t crcFails() const { return crc_fails_; }   // whole frames whose CRC did not check
    uint32_t skipped() const  { return skipped_; }     // bytes in no frame (none in a clean stream)

private:
    void parse(Sink& sink);
    void drop(size_t k);

    // A frame at most plus room to take more in before the next parse.
    static constexpr size_t kBuf = 2 * kMaxFrame;
    uint8_t  buf_[kBuf] = {};
    size_t   n_ = 0;
    size_t   fed_after_ = 0;   // bytes of this feed() still to come into buf_
    uint32_t frames_ = 0;
    uint32_t crc_fails_ = 0;
    uint32_t skipped_ = 0;
};

}  // namespace sdr_rtcm

#endif
