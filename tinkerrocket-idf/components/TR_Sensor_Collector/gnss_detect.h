#pragma once
// gnss_detect.h — which GNSS receiver is on the V9's J1 (pure, host-compilable)
//
// The V9's J1 takes the stock u-blox SAM-M10Q carrier or the harness of a
// Septentrio mosaic-G5 (mosaic-go G5 P3): one connector, pins 1-4 the same on
// both (board_v9.h). Until this the receiver was a build-time choice, and an
// image built for the other one searched for 30-60 s and flew GNSS-absent.
// This finds out which one is there before either driver starts, by asking at
// the bauds each can be at:
//
//   mosaic-G5  2 Mbaud once a flight computer has set it up (the setup is
//              saved into the receiver, so it streams SBF from power-up);
//              115200 from the factory, silent until spoken to. The probe is
//              a CR, which brings the prompt ("COM1>") on a port that takes
//              commands. Proof: a CRC-valid SBF block, the prompt, or a "$R"
//              command reply.
//   u-blox     460800 once a flight computer has set it up, until it loses
//              power (the SAM-M10Q carrier keeps no backup power); 38400 or
//              9600 from the factory, with NMEA once a second. The probe is a
//              UBX-MON-VER poll. Proof: a checksum-valid UBX frame, or an
//              NMEA sentence whose checksum checks. NMEA counts only at the
//              end of a probe that brought no mosaic proof: a mosaic can be
//              told to send NMEA, but never UBX.
//
// Neither probe can upset the other receiver: each is a few bytes at a baud
// the other is not at, which its UART reads as noise, and neither receiver
// acts on the other's protocol. Nothing here touches J1 pin 5 (GPIO2): it is
// the receiver's PPS on two carriers and a receiver INPUT on the PX1105R's.
//
// The hint (the receiver found last boot) puts that receiver's probes first,
// so a boot with the same receiver as last time costs one probe, 0.1-0.4 s.
// Anything not proven a mosaic-G5 gets the u-blox driver, which is what every
// V9 ran before this, and which has its own, longer search for a receiver in
// an odd state (another baud, swapped wires).
//
// MosaicLink's shape: the driver feeds it the bytes and the time and carries
// out the actions it returns (set the UART's baud, send bytes). All of the
// UART is in gnss_auto_driver.cpp. Host-tested in
// tests_cpp/test_gnss_detect.cpp.

#include <cstddef>
#include <cstdint>

#include <RocketComputerTypes.h>   // GNSS_TYPE_*
#include "SbfParser.h"             // TR_GNSSReceiverMosaic_Serial: blocks, reply lines, prompts

namespace gnss_detect
{

// Where each driver looks for its receiver.
static constexpr uint32_t kMosaicTargetBaud  = 2000000;   // TR_GNSSReceiverMosaic_Serial's kTargetBaud
static constexpr uint32_t kMosaicFactoryBaud = 115200;    // and its kFactoryBaud
static constexpr uint32_t kUbloxRunBaud      = 460800;    // TR_GNSSReceiverUBlox_Serial's preferred_baud

// The bauds the u-blox driver sweeps (its probe_bauds): the only rates a
// remembered u-blox baud is trusted at.
bool isUbloxBaud(uint32_t baud);

// How long each probe listens after its bytes have gone out. The mosaic
// answers a CR within milliseconds and, set up, streams ~60 blocks a second;
// the u-blox needs 100 ms plus the ~230 bytes of its MON-VER reply on the
// wire (412 ms at 9600 baud).
static constexpr uint16_t kMosaicTargetListenMs  = 150;
static constexpr uint16_t kMosaicFactoryListenMs = 250;
uint16_t ubloxListenMs(uint32_t baud);

// How long planFor() keeps going round its probes, by hint. With the mosaic
// last time: as long as the mosaic driver's own search, for a receiver still
// booting. With the u-blox: once round, because the u-blox driver searches
// for 60 s itself. With nothing known (the first boot of this image): long
// enough for a receiver that powered up with the flight computer to boot.
static constexpr uint32_t kBudgetMosaicMs = 30000;
static constexpr uint32_t kBudgetUbloxMs  = 0;
static constexpr uint32_t kBudgetNoneMs   = 6000;

// What proved the receiver.
enum class Proof : uint8_t { NONE, SBF_BLOCK, PROMPT, REPLY, UBX_FRAME, NMEA };
const char* proofName(Proof p);

struct Probe
{
    uint32_t baud = 0;
    uint8_t  asks = GNSS_TYPE_NONE;   // GNSS_TYPE_MOSAIC: a CR; GNSS_TYPE_UBLOX: a UBX-MON-VER poll
    uint16_t listen_ms = 0;
};

struct Plan
{
    static constexpr size_t kMaxProbes = 8;
    Probe    probes[kMaxProbes] = {};
    size_t   n = 0;
    // Round and round the probes until this long has passed since start(),
    // checked at the end of each round (0: once round).
    uint32_t budget_ms = 0;
    // What to run when nothing answered: GNSS_TYPE_UBLOX (that driver's own
    // full search) or GNSS_TYPE_NONE (fly GNSS-absent).
    uint8_t  fallback = GNSS_TYPE_UBLOX;
};

// The probes for this boot.
//   hint                 the receiver found last time (GNSS_TYPE_*); anything
//                        else reads as nothing known
//   ublox_last_baud      the baud the u-blox driver last found its receiver
//                        at (its "gnsslink" record), 0 for none
//   receiver_kept_power  not a power-on, so a u-blox the flight computer set
//                        up last boot is still at 460800
Plan planFor(uint8_t hint, uint32_t ublox_last_baud, bool receiver_kept_power);

// After the u-blox driver's own search found nothing: one more look for a
// mosaic-G5, which a minute on has finished booting if it is there at all.
Plan secondLookPlan();

// UBX-MON-VER poll and the CR, as the probes send them.
extern const uint8_t kUbxMonVerPoll[8];
extern const uint8_t kCr[1];

struct Action
{
    enum Kind : uint8_t { NONE, SET_BAUD, SEND };
    Kind           kind = NONE;
    uint32_t       baud = 0;          // SET_BAUD: also drop what the UART holds, it was heard at the old baud
    const uint8_t* data = nullptr;    // SEND
    size_t         len = 0;
};

// UBX frames whose checksum checks, in a stream of any slicing.
class UbxScanner
{
public:
    static constexpr uint16_t kMaxPayload = 2048;   // a longer Length is a false sync

    void reset() { state_ = 0; }
    // True when `b` completed a frame whose checksum checks.
    bool step(uint8_t b);
    uint8_t  lastClass() const { return cls_; }
    uint8_t  lastId() const { return id_; }
    uint16_t lastLength() const { return len_; }

private:
    uint8_t  state_ = 0;
    uint8_t  cls_ = 0, id_ = 0;
    uint16_t len_ = 0, got_ = 0;
    uint8_t  ck_a_ = 0, ck_b_ = 0;
    void add(uint8_t b) { ck_a_ = static_cast<uint8_t>(ck_a_ + b); ck_b_ = static_cast<uint8_t>(ck_b_ + ck_a_); }
};

// An NMEA sentence without its line end: '$', an address of 2-6 capitals or
// digits ("GPGGA", "PUBX"), ',', printable fields, '*' and two hex digits
// that are the XOR of everything between '$' and '*'.
bool isNmeaSentence(const char* line);

class Detector : private mosaic::SbfSink
{
public:
    void start(const Plan& plan, uint32_t now_ms);

    // The bytes heard since the last call, in any slicing.
    void feed(const uint8_t* data, size_t n, uint32_t now_ms);

    // Let time pass. The next thing to do, NONE when nothing; call again
    // until NONE. Nothing at all once done().
    Action poll(uint32_t now_ms);

    bool     done() const      { return done_; }
    uint8_t  found() const     { return found_; }        // GNSS_TYPE_NONE: nothing answered
    Proof    proof() const     { return proof_; }
    uint32_t foundBaud() const { return found_baud_; }
    uint32_t elapsedMs() const { return elapsed_ms_; }   // start() to the verdict
    uint32_t probesRun() const { return probes_run_; }
    uint32_t baud() const      { return baud_; }          // what the UART was last set to

private:
    // SbfSink
    void onBlock(const uint8_t* block, size_t len, size_t index) override;
    void onLine(const char* line, size_t index) override;
    void onPrompt(const char* port, size_t index) override;

    void enterProbe(size_t i, uint32_t now_ms);
    void finish(uint8_t type, Proof proof, uint32_t now_ms);
    void push(Action::Kind kind, uint32_t baud, const uint8_t* data, size_t len);

    Plan       plan_;
    size_t     i_ = 0;
    uint32_t   t_start_ = 0;
    uint32_t   t0_ = 0;            // the current probe's bytes went out
    uint32_t   now_ms_ = 0;        // for the sink, which is called from feed()
    uint32_t   baud_ = 0;
    bool       nmea_ = false;      // an NMEA sentence this probe
    bool       done_ = true;
    uint8_t    found_ = GNSS_TYPE_NONE;
    Proof      proof_ = Proof::NONE;
    uint32_t   found_baud_ = 0;
    uint32_t   elapsed_ms_ = 0;
    uint32_t   probes_run_ = 0;

    mosaic::SbfParser parser_;
    UbxScanner        ubx_;

    // A probe queues at most a baud change and a send.
    static constexpr size_t kQueue = 2;
    Action     q_[kQueue] = {};
    size_t     q_head_ = 0;
    size_t     q_n_ = 0;
};

// The order of things at boot, pure so the tests can walk every path.
// TR_GNSSAuto::begin() carries out each step and reports back:
//
//   DETECT        run plan() through a Detector, then detected()
//   BEGIN_UBLOX   the u-blox driver's begin() as it has always run, then
//                 driverResult()
//   BEGIN_MOSAIC  the mosaic driver's begin() (mosaicTimeoutMs()), then
//                 driverResult()
//   SECOND_LOOK   run plan() again, then detected(): after the u-blox
//                 driver's own search came up empty, a last look for a
//                 mosaic-G5 that was still booting during DETECT
//   DONE          active() is the receiver in use, GNSS_TYPE_NONE for none
//
// A receiver the probes proved gets its own driver straight away. With
// nothing proven the plan's fallback runs: the u-blox driver (what every V9
// ran before this), or, for a board whose mosaic answered last time, nothing
// at all. The hint to keep for next boot is hint(); hintChanged() says
// whether it needs writing (the probes found another receiver, the u-blox
// driver found its own by its full search, or a mosaic board answered
// nothing and is forgotten so that next boot is not stuck on its plan).
//
// Worst case, a dead receiver: about 31 s with the mosaic hint; with any
// other, the round of probes (~1.2 s, 6 s on the first boot) plus the u-blox
// driver's 60 s plus the second look (~0.8 s), where the u-blox image took
// the 60 s alone.
class BringUp
{
public:
    enum class Step : uint8_t { DETECT, BEGIN_UBLOX, BEGIN_MOSAIC, SECOND_LOOK, DONE };

    // The mosaic driver's own search: its usual 30 s, and 15 s when it
    // follows the u-blox driver's 60 s, to keep the boot well inside the
    // setup watchdog (fc_rescue_policy.h kSetupStallMs).
    static constexpr uint32_t kMosaicTimeoutMs      = 30000;
    static constexpr uint32_t kMosaicLateTimeoutMs  = 15000;

    // ublox_wires_swapped: the u-blox driver last found its receiver with RX
    // and TX the other way round (its "gnsslink" record). The probes use the
    // board's own wiring, so they would hear nothing there and drive the TX
    // line against the receiver's output for a round: such a board goes
    // straight to the u-blox driver, which knows the wiring, unless a mosaic
    // was found since.
    void start(uint8_t hint, uint32_t ublox_last_baud, bool receiver_kept_power,
               bool ublox_wires_swapped = false);
    void detected(uint8_t found);
    void driverResult(bool ok);

    Step        step() const       { return step_; }
    const Plan& plan() const       { return plan_; }
    uint8_t     active() const     { return active_; }
    uint8_t     hint() const       { return hint_; }
    bool        hintChanged() const { return hint_ != hint_in_; }
    uint8_t     proven() const     { return proven_; }    // what the first round of probes proved
    bool        secondLookRan() const { return second_look_; }
    uint32_t    mosaicTimeoutMs() const { return second_look_ ? kMosaicLateTimeoutMs : kMosaicTimeoutMs; }

private:
    Step    step_ = Step::DONE;
    Plan    plan_;
    uint8_t hint_in_ = GNSS_TYPE_NONE;
    uint8_t hint_ = GNSS_TYPE_NONE;
    uint8_t proven_ = GNSS_TYPE_NONE;
    uint8_t active_ = GNSS_TYPE_NONE;
    bool    second_look_ = false;
};

}  // namespace gnss_detect
