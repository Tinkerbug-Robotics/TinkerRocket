// SdrAid.h — TinkerRocket's message 4002, the flight computer to the gnss-sdr-dev board (pure, host-compilable)
//
// The board (gnss repo, firmware/gnss-sdr-dev) reads its UART RX (GPIO11: J1's
// "module RXD", which the FC drives from its GPIO4) at 2 Mbaud for RTCM 3
// frames of TinkerRocket's message 4002 (in the range RTCM keeps for
// manufacturers, not registered, as the board's own 4001 is), one sub-type
// each:
//   - 1, the attitude: body to ECEF and the body's rates, 10 a second;
//   - 2, the navigation state: ECEF position and velocity, 5 a second;
//   - 3, the flight phase and the tracking loops it asks for, on each change,
//     then three times 100 ms apart, then every second;
//   - 4, the geometry: the antenna's lever arm and the board's mounting,
//     at the start, then every 10 s.
// Each is stamped with the GPS instant its values hold, carries its own
// sequence number, and says where its values come from (src, test): the P1
// bench script or the vehicle's filter. Layouts: P1 of
// docs/plans/sdr-v10-aided-receiver-plan.md (its spec's section 2). The board
// decodes them with receiver/core/nav/rtcm_fc.c, and both repos pin the same
// golden frames (tests_cpp/test_sdr_aid.cpp and receiver/tests/test_rtcm_fc.cpp
// in the gnss repo), so the rules below keep the two sides' bytes equal:
//   - fields most significant bit first, signed ones two's complement;
//   - a scaled value is the board's clamp_i(v * scale): saturated at
//     +-(2^(n-1) - 1) past the field, else rounded to nearest, halves away
//     from zero; every scale an exact constant, multiplied, never divided;
//   - a 1-sigma rounds up to its LSB (forgiving a thousandth of one, never 0)
//     and is all ones when unknown (NaN, negative) or past the field;
//   - a field whose valid bit is 0 goes out as 0, and a NaN anywhere in a
//     group (a quaternion, the rates, a position, a velocity, the lever arm)
//     sends that group as not valid; an infinity saturates;
//   - a quaternion goes out with its scalar part >= 0, not normalized;
//   - doubles, no fused multiply-add: SdrAid.cpp is built with
//     -ffp-contract=off (CMakeLists.txt), as the board's codec is.
// The decoders read a sub-type's own length and ignore any bytes after it: a
// later revision appends to a sub-type, a changed meaning takes a new one.
//
// Also here: the P1 bench script (scripted values every implementation
// computes alike from the stamp's whole ms), the sender's schedule, and the
// aiding block the board appends to its 4001 sub-type 1 epoch, which says
// what the board does with what it was sent and how many frames it took.
#ifndef SDRAID_H
#define SDRAID_H

#include <cstddef>
#include <cstdint>
#include <limits>

#include "SdrRtcm.h"

namespace sdr_aid
{

static constexpr uint16_t kFcMessage     = 4002;   // TinkerRocket's message from the flight computer
static constexpr uint8_t  kAttitude      = 1;      // its sub-types
static constexpr uint8_t  kNavState      = 2;
static constexpr uint8_t  kPhase         = 3;
static constexpr uint8_t  kGeometry      = 4;
static constexpr size_t   kAttitudeBytes = 30;     // each sub-type's message (its frame: 6 more)
static constexpr size_t   kNavStateBytes = 46;
static constexpr size_t   kPhaseBytes    = 16;
static constexpr size_t   kGeometryBytes = 28;
static constexpr size_t   kAttitudeFrame = 3 + kAttitudeBytes + 3;
static constexpr size_t   kNavStateFrame = 3 + kNavStateBytes + 3;
static constexpr size_t   kPhaseFrame    = 3 + kPhaseBytes + 3;
static constexpr size_t   kGeometryFrame = 3 + kGeometryBytes + 3;
static constexpr size_t   kMaxFrame      = 3 + 64 + 3;   // no 4002 message is longer than 64 bytes

// Where the values come from (the header's src).
enum : uint8_t
{
    kSrcScript      = 0,   // the bench script: always a test frame
    kSrcOwn         = 1,   // the flight computer's filter on its own sensors (no GNSS)
    kSrcFusedBoard  = 2,   // its filter fused with this board's measurements
    kSrcFusedOther  = 3,   // fused with another GNSS receiver
};
// The flight phase (sub-type 3).
enum : uint8_t { kPhUnknown = 0, kPhPad = 1, kPhBoost = 2, kPhCoast = 3, kPhDescent = 4, kPhLanded = 5 };
// The loops the flight computer asks the board for: none (the board's quiet
// loops under its `rx aid fc`, the boot's; its GPS channels' own decision
// only under `rx aid gps`), its quiet profile, its boost profile. The board
// refuses a boost after this side has reported a descent or a landing, until
// it reports the pad; a coast leaves one open (a second stage).
enum : uint8_t { kLoopsNone = 0, kLoopsQuiet = 1, kLoopsBoost = 2 };

// An unknown 1-sigma.
static constexpr double kUnknown = std::numeric_limits<double>::quiet_NaN();

// Every sub-type's first 96 bits. The stamp is the instant the values hold
// (a filter's: its state's IMU sample), not when the frame goes out.
struct Header
{
    uint16_t week = 0;          // GPS week of the stamp
    uint32_t tow_ms = 0;        // its time of week, the nearest ms
    int32_t  sub_ns = 0;        // the stamp minus tow_ms, ns (+-500000)
    uint8_t  seq = 0;           // this sub-type's frames sent, mod 256
    uint8_t  src = kSrcScript;  // kSrc*
    bool     test = true;       // not the vehicle's state (the script, simulated or replayed sensors)
    uint8_t  tq = 0;            // the stamp's 1-sigma: 0 unknown; k = 1..6 under 16 * 4^k us; 7 worse
};

struct Attitude
{
    Header  h;
    bool    att_valid = false;
    bool    rate_valid = false;
    uint8_t frame = 0;                     // 0: body to ECEF (1-3 reserved: the board refuses them)
    double  q[4] = {1.0, 0.0, 0.0, 0.0};   // body (FRD) to ECEF, Hamilton, scalar first: v_ECEF = q v_body q*
    double  w[3] = {};                     // the body's rate in body axes, the gyro less its bias, rad/s
    double  sig_tilt_deg = kUnknown;       // 1-sigmas (0.1 deg and 0.5 deg on the wire)
    double  sig_hdg_deg = kUnknown;
};

struct NavState
{
    Header  h;
    bool    pos_valid = false;
    bool    vel_valid = false;
    uint8_t point = 0;          // 0: the board's antenna phase center; 1: the flight computer's IMU
    double  pos[3] = {};        // ECEF, m
    double  vel[3] = {};        // ECEF, m/s
    double  sig[4] = {kUnknown, kUnknown, kUnknown, kUnknown};   // 1-sigmas: position horizontal and
                                                                 // vertical (m), velocity likewise (m/s)
};

struct PhaseMsg
{
    Header   h;
    uint8_t  phase = kPhUnknown;
    uint8_t  loops = kLoopsNone;
    bool     canopy = false;     // under a parachute
    uint32_t since_ms = 65535;   // in this phase at the stamp; 65535 or more: that long or unknown
    uint32_t hold_ms = 0;        // the request holds until the stamp plus this (20 ms steps, to 5.1 s); 0: none
};

struct Geometry
{
    Header  h;
    bool    lever_valid = false;
    bool    rot_valid = false;
    uint8_t ant_type = 0;                    // 0 unknown, 1 patch, 2 quadrifilar helix, 3 wrap-around array
    uint8_t ant_axis = 0;                    // its boresight in the board frame: 0 unknown, 1 +x, 2 -x, 3 +y,
                                             // 4 -y, 5 +z, 6 -z
    double  lever_m[3] = {};                 // the board's IMU to the antenna's phase center, board frame, m
    double  rot[4] = {1.0, 0.0, 0.0, 0.0};   // board to body, Hamilton, scalar first: v_body = r v_board r*
};

// Each writes its whole frame to out (36, 52, 22 or 34 bytes) and returns
// that length; 0 when cap is shorter.
size_t encode(const Attitude& a, uint8_t* out, size_t cap);
size_t encode(const NavState& a, uint8_t* out, size_t cap);
size_t encode(const PhaseMsg& a, uint8_t* out, size_t cap);
size_t encode(const Geometry& a, uint8_t* out, size_t cap);

// 4002's sub-type of a whole frame (the 4 bits after the number); 0 for any
// other message.
uint8_t fcSubType(const uint8_t* frame, size_t len);

// A whole frame (0xD3 through its CRC, which its framer has checked) of the
// sub-type, at least as long as that sub-type's message: true with out
// filled. A sigma's all-ones code reads NaN.
bool decode(const uint8_t* frame, size_t len, Attitude& out);
bool decode(const uint8_t* frame, size_t len, NavState& out);
bool decode(const uint8_t* frame, size_t len, PhaseMsg& out);
bool decode(const uint8_t* frame, size_t len, Geometry& out);

// The header of a GPS instant: its time of week to the nearest ms and the
// rest in ns, a stamp that rounds to the week's end the next week's 0 (the
// board's rounding of its own fixes). seq, src, test and tq as Header's.
Header stampAt(int week, double tow_s);

// ── The P1 bench script (TR_BENCH_SDR_AID) ──────────────────────────────
// Scripted values for a header's tow_ms, the same on the board, which checks
// each frame against them: a roll back and forth at the rig scene's pad, the
// pad swinging 2 m east and west, the hotshot scene's phases near its
// ignition (a 120 s cycle through every phase elsewhere), and a fixed
// geometry. Script frames carry sub_ns 0, src kSrcScript, test and kScriptTq.
static constexpr uint8_t  kScriptTq     = 5;      // under 16.4 ms: the board's output delay is not taken out
static constexpr uint32_t kScriptHoldMs = 2500;
// A script frame's header at a GPS instant (continuous seconds, week *
// 604800 + time of week): its whole ms, the script's values being that ms's.
Header scriptStamp(double gps_s);
Attitude scriptAttitude(const Header& h);
NavState scriptNav(const Header& h);
PhaseMsg scriptPhase(const Header& h);
Geometry scriptGeometry(const Header& h);
// What a change of the phase message is: phase | loops << 3 | canopy << 5.
uint32_t scriptPhaseKey(uint32_t tow_ms);

// ── The board's aiding block ────────────────────────────────────────────
// What the board appends to each 4001 sub-type 1 (32 bits at 214 + 10 x the
// mark count), read from its live receiver as it sends the epoch. A frame
// from older firmware has none.
struct EpochAid
{
    int32_t shift_ns = 0;     // the epoch's time tag minus the board's time at its latch (every epoch has it)
    bool    present = false;  // the block is there (its version not 0); the fields below are 0 otherwise
    uint8_t ver = 0;
    uint8_t src = 0;          // what the loops are aided by: 0 nothing, 1 the GPS-derived rate, 2 the board's IMU
    uint8_t check = 0;        // the aiding check: 0 not run, 1 passing, 2 refused
    bool    boost = false;    // the boost profile is the loops' target
    bool    wide = false;     // the loops in force still more than 10 % wider than the quiet profile
    uint8_t by = 0;           // who set the target: 0 the GPS channels, 1 the flight computer (its
                              // request, or the quiet loops it leaves without one), 2 a caller
                              // (`rx aid off`), 3 the board's IMU
    bool    planned = false;  // an aided start from the flight computer's state
    uint8_t phase = 0;        // the flight computer's phase the board holds (kPh*), 0 none
    bool    att = false;      // a flight computer attitude is in the receiver
    bool    nav = false;      // a navigation state under 1 s old is held
    bool    late = false;     // read with the board's latch ring more than an epoch behind
    uint8_t fc_frames = 0;    // CRC-good 4002 frames the board has received, mod 256
};
// Any 4001 sub-type 1 SdrRtcm's decodeTrEpoch takes: true, with its shift
// and the block if it has one.
bool decodeEpochAid(const uint8_t* frame, size_t len, EpochAid& out);

// What an epoch gives a clock line fitted to its arrivals (the bench's): the
// GPS time the board latched its measurements at (continuous seconds, week *
// 604800 + time of week), its MSMs' tag less its sub-ms shift. The shift is
// fixed within a board's boot and random between boots, so a line fitted to
// the latch, not the tag, keeps a delay measured on one boot good on the
// next. False for an epoch the board marks late: it left its latch ring
// behind others (up to 0.64 s when its console starves the observables
// task), so its arrival carries that wait, not the board's delay, and
// enough of them in a row would restart the line.
bool latchGps(const sdr_rtcm::TrEpoch& e, const EpochAid& aid, double& gps_s);

// ── What the board says it took ─────────────────────────────────────────
// The frames this flight computer sent that the board took, from the 8-bit
// count in its epochs' blocks (fc_frames), followed block by block against
// what went out between them: between two blocks the board counts no more
// than went out, give or take a few frames on their way (kSlack). Where its
// count cannot be followed it is picked up again at the block, and what
// went out since the last one is set aside, not compared: before the first
// block, after a gap of more than kGapUs in them (a reboot could be in it),
// after more went out than 8 bits tell apart, and when the count went back
// (the board's `rx fc clear`, or a reboot).
class EchoCount
{
public:
    static constexpr uint32_t kSlack = 16;
    static constexpr int64_t  kGapUs = 2000000;

    // An epoch's block: its fc_frames, the frames this computer has sent so
    // far (every sub-type), and when the epoch came, us.
    void add(uint8_t fc_frames, uint32_t sent, int64_t at_us);

    uint32_t took() const { return took_; }       // the board took this many ...
    uint32_t of() const { return of_; }           // ... of these, sent where its count was followed
    uint32_t setAside() const { return aside_; }  // sent where it was not

private:
    bool     have_ = false;
    uint8_t  fc_frames_ = 0;   // the last block's
    uint32_t sent_ = 0;        // sent by then
    int64_t  at_us_ = 0;
    uint32_t took_ = 0;
    uint32_t of_ = 0;
    uint32_t aside_ = 0;
};

// ── When each frame goes ────────────────────────────────────────────────
// The sender's pacing, one frame at most per call and none within kSpacingUs
// of the last, so the board's 128-byte FIFO never holds more than one frame:
// in order, a changed phase key (at once), the attitude every 100 ms, the
// navigation state every 200 ms (5 ms after the attitude), the phase repeated
// three times 100 ms apart after a change and every second after that (3 ms
// after the change's pass), and the geometry at the start and every 10 s
// (7 ms after the attitude): about 16 frames a second. Each next due time
// steps by its period, so the mean rates are exact; one more than a period
// behind restarts from now, so a stall brings no burst after it.
class AidSchedule
{
public:
    static constexpr int64_t kSpacingUs = 1000;
    static constexpr int64_t kKeyLookUs = 10000;   // see anyDue

    // The sub-type to send now (0: none), given the phase key as the caller
    // has it now: a key that is not the last one sent goes first.
    uint8_t due(int64_t now_us, uint32_t phase_key);
    // Whether anything is due now, before the caller knows the key (the
    // bench's script has its key only from a stamp, worked out when a frame
    // is due): then due() with the key at the stamp, which gives a sub-type
    // whenever this said yes. It never sees a changed key, so such a caller
    // also looks at its key kKeyLookUs after its last look (due() then gives
    // the change, or 0): a change goes within about that of its instant, not
    // with the next frame due, which can be 100 ms away.
    bool anyDue(int64_t now_us);
    // A frame of `sub` went out at now_us; a phase frame carries phase_key.
    void sent(uint8_t sub, int64_t now_us, uint32_t phase_key);

private:
    int64_t  next_us_[kGeometry + 1] = {};
    int64_t  last_us_ = 0;
    int64_t  repeat_us_ = 0;   // where the last change's repeats step from: its pass plus the phase's offset
    uint32_t key_ = 0;         // the last phase key sent
    bool     started_ = false;
    bool     any_sent_ = false;
    bool     have_key_ = false;
};

}  // namespace sdr_aid

#endif
