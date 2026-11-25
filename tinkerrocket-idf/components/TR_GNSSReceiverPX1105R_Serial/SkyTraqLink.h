// SkyTraqLink.h — finding and setting up the PX1105R on its UART (pure, host-compilable)
//
// The receiver can be anywhere when the flight computer starts: still
// booting (the FC's GNSS rail comes up with it), at its factory 115200 baud
// streaming NMEA as an RTK rover, or (a reboot of the FC alone) already at the
// target baud streaming the binary set. This state machine finds it and sets
// it up whichever it is, and again whenever a running link goes silent (a
// receiver reset in flight), without blocking: the driver feeds it the
// parser's frames, the NMEA lines it counts and the time, and carries out the
// actions it returns (set the UART baud, send a frame). begin() runs it to
// RUNNING under a deadline; the poll task runs it after that.
//
// The setup, all in SRAM (a power cycle undoes it), in the order the receiver
// test rig's tools use (gnss repo cocom-rig/skytraq_raw.py, sdr/nav_mode.py,
// sdr/px1105r_run.py), each command sent until the receiver ACKs it:
//   0x05      UART to 921600 (from the baud it was found at)
//   0x6A/0x06 RTK base mode, kinematic function: the PX1105R sends raw
//             measurements only as an RTK base (AN0037; as a rover it sends
//             NMEA and no binary); the kinematic base re-solves every epoch,
//             so it may move
//   0x1E      binary measurement output at the rate: 0xE5 raw, 0xDF state,
//             0xE7/0xE8 status, the navigation subframes of GPS, Galileo and
//             BeiDou (0x0D), and GLONASS's strings where the driver asks
//             (0x0F: TR_RAW_GLONASS)
//   0x0E      position update rate, the same (with 0x1E at 20 Hz and this
//             left at 1 Hz, 0xE5 came at 0.4 Hz)
//   0x64/0x17 navigation mode airborne (5): in auto (0) the rig's flights
//             froze after apogee (2026-10-21); airborne fixed the descent
//   0x0C      power normal (it ships in power save, which throttles the search)
//   0x2B      elevation mask 5 deg (it ships at 15)
//   0x64/0x1A the constellations it tracks, read (AN0028: the 0x64/0x8C reply,
//             a mask: 1 GPS, 2 GLONASS, 4 Galileo, 8 BeiDou), and only where
//             they are not the ones asked for (all four, 0x0F),
//   0x64/0x19 those set: a receiver tracks no system its mask leaves out, and
//             a mask saved to its flash by an earlier test comes back at every
//             power-up. Sending it restarts every channel (0xE5 stopped 9-16 s
//             on the rig, 2026-11-09), so a receiver that already tracks them
//             never gets it; it goes last, to a receiver already an RTK base,
//             as on the rig. A receiver that does not answer the query keeps
//             its own.
// Then each is read back where the receiver can say (0x10, 0x1F, 0x64/0x18,
// 0x15, 0x64/0x1A), and the link runs once 0xDF arrives: the receiver's
// state, which an RTK base sends every epoch whether or not it tracks
// anything (the rig, 2026-11-01). A readback that disagrees, or no 0xDF, sets
// the whole setup up again (twice at most, then the link runs with what it
// has and says so).
//
// Once per power-up: the setup restarts the receiver (the rig's tools lose the
// 0x0E ACK while it does), so its search starts over each time. 0xE5 comes only
// while it tracks satellites -- none on a cold start until it has found them --
// so a missing 0xE5 is not a lost setup, and the link never sets the receiver
// up again for it (V10 board 1 by a window, 2026-11-13: set up every ~10 s for
// that, it tracked 0-1 satellites in 13 min). Only silence does that: a reset
// takes the receiver back to its factory baud. And a receiver found already
// set up at the target baud (the FC rebooted alone) is read back first and,
// when it agrees, left alone: no command, no restart.
//
// Commands go one at a time: the receiver answers slowly after a change (its
// ACKs come up to ~1-2 s late on the bench, 2026-10-11), and a command sent
// before the last one's ACK can be lost. Host-tested in
// tests_cpp/test_skytraq_link.cpp.
#ifndef SKYTRAQ_LINK_H
#define SKYTRAQ_LINK_H

#include <cstddef>
#include <cstdint>

namespace skytraq
{

struct LinkConfig
{
    uint32_t target_baud   = 921600;
    uint8_t  rate_hz       = 20;     // 1, 2, 4, 5, 8, 10 or 20
    uint8_t  nav_mode      = 5;      // airborne
    uint8_t  elev_mask_deg = 5;
    uint8_t  subframes     = 0x0D;   // 1 GPS, 2 GLONASS (0xE1 strings), 4 Galileo, 8 BeiDou
    uint16_t constellations = 0x0F;  // tracked (0x64/0x19): the same bits; 0 leaves the receiver's own
    bool     flash         = false;  // SRAM only: a power cycle undoes the setup
    uint32_t probe_ms      = 700;    // listen at one baud for a frame or NMEA
    uint32_t switch_ms     = 300;    // after 0x05: let the ACK and the switch finish
    uint32_t reply_ms      = 2500;   // an ACK or a query's reply
    uint8_t  retries       = 2;      // re-sends of an unanswered command
    uint32_t stream_ms     = 5000;   // after the setup: 0xDF within this
    uint32_t silence_ms    = 3000;   // running with no frame this long: find the receiver again
    uint8_t  max_setups    = 3;      // setups in a row before the link runs with what it has
};

enum class LinkState : uint8_t
{
    PROBE,         // listening for the receiver at one baud
    SWITCH_BAUD,   // 0x05 sent; waiting out the switch
    VERIFY_BAUD,   // at the target baud, waiting for a frame
    CONFIGURE,     // the setup commands
    CHECK,         // the readbacks (after the setup, or before it at the target baud)
    STREAM,        // waiting for 0xDF
    RUNNING,       // set up; watching for silence
};

struct LinkAction
{
    enum Kind : uint8_t { NONE, SET_BAUD, SEND };
    Kind           kind = NONE;
    uint32_t       baud = 0;          // SET_BAUD
    const uint8_t* bytes = nullptr;   // SEND: a whole frame; valid until the next poll()
    size_t         len = 0;
};

// The 0x05 code of a baud (AN0039), or -1.
int baudCode(uint32_t baud);
// The 0x1E code of a rate (AN0039), or -1.
int rateCode(uint8_t hz);

class SkyTraqLink
{
public:
    static constexpr int kMaxCommands = 10;
    static constexpr size_t kMaxCommand = 48;   // longest body (0x6A/0x06 is 37 B)

    void start(const LinkConfig& cfg, uint32_t now_ms);

    // The parser's items, as they arrive.
    void onFrame(const uint8_t* body, size_t len, uint32_t now_ms);
    void onNmea(uint32_t now_ms);

    // Let time pass. Returns the next thing the driver must do, NONE when
    // nothing; call again until NONE.
    LinkAction poll(uint32_t now_ms);

    LinkState   state() const { return state_; }
    bool        running() const { return state_ == LinkState::RUNNING; }
    uint32_t    baud() const { return baud_; }
    uint32_t    foundBaud() const { return found_baud_; }   // where the last probe found it
    bool        foundNmea() const { return found_nmea_; }   //   talking NMEA (a fresh power-up)
    uint32_t    setups() const { return setups_; }          // times the setup finished
    uint32_t    relinks() const { return relinks_; }        // times a running link went silent
    bool        leftAlone() const { return left_alone_; }   // the last found already set up: no command sent
    uint32_t    refused() const { return refused_; }        // commands NACKed or unanswered, last setup
    uint32_t    checksFailed() const { return checks_failed_; }   // readbacks that disagreed, last setup
    bool        degraded() const { return degraded_; }      // running without the whole setup
    const char* lastFailure() const { return last_failure_; }
    // From the 0x80 reply: kernel, ODM and revision, e.g. "3.6.0 1.7.33 230831".
    const char* version() const { return version_; }
    // The last 0xE5's measurement period (ms), as the receiver states it.
    uint16_t    rawPeriodMs() const { return raw_period_ms_; }
    // The constellations the receiver last said it tracks (0x64/0x8C), -1 when
    // it has not said since the setup began.
    int         constellations() const { return mask_read_; }
    uint32_t    constellationSets() const { return mask_sets_; }   // 0x64/0x19 sent and ACKed

private:
    struct Cmd
    {
        uint8_t     body[kMaxCommand];
        uint8_t     len;
        int16_t     ack_sub;    // the sub-ID the ACK names (-1: none)
        bool        required;
        const char* name;
        int16_t     reply_sub;  // a query: done at its 0x64 reply of this sub-ID, after the ACK (-1: a command)
        bool        if_mask_off;   // sent only when the receiver's constellations are not cfg_.constellations
    };
    void enterProbe(uint32_t baud_idx, uint32_t now_ms);
    void enterSwitch(uint32_t now_ms);
    void enterVerify(uint32_t now_ms);
    void enterConfigure(uint32_t now_ms);
    void enterCheck(uint32_t now_ms);
    void enterStream(uint32_t now_ms);
    void enterRunning(uint32_t now_ms);
    void sendCommand(uint32_t now_ms);
    void nextCommand(uint32_t now_ms);
    bool skipCommand(const Cmd& c) const;
    void sendQuery(uint32_t now_ms);
    void buildSetup();
    void noteFailure(const char* what);
    bool checkReply(const uint8_t* body, size_t len);   // a readback's reply: does it agree?

    void push(LinkAction::Kind kind, uint32_t baud, const uint8_t* body, size_t len);

    LinkConfig cfg_;
    LinkState  state_ = LinkState::PROBE;
    uint32_t   t0_ = 0;
    uint32_t   baud_ = 0;
    uint32_t   probe_idx_ = 0;
    uint32_t   found_baud_ = 0;
    bool       found_nmea_ = false;
    bool       heard_ = false;          // a frame (or NMEA) since the state began
    uint8_t    nmea_seen_ = 0;
    bool       queried_ = false;

    Cmd        cmds_[kMaxCommands] = {};
    int        n_cmds_ = 0;
    int        cmd_ = 0;
    uint8_t    attempt_ = 0;
    bool       got_ack_ = false;
    bool       got_nack_ = false;

    int        query_ = 0;              // CHECK: which readback
    bool       precheck_ = false;       // CHECK before any command: found at the target baud
    bool       left_alone_ = false;
    bool       got_reply_ = false;
    bool       reply_ok_ = false;
    uint8_t    setups_in_row_ = 0;

    uint32_t   last_frame_ms_ = 0;
    uint16_t   raw_period_ms_ = 0;
    uint8_t    states_seen_ = 0;        // STREAM: 0xDF since the setup
    int        mask_read_ = -1;         // the last 0x64/0x8C's mask; -1 none since the setup began
    uint32_t   mask_sets_ = 0;

    uint32_t   setups_ = 0, relinks_ = 0, refused_ = 0, checks_failed_ = 0;
    bool       degraded_ = false;
    char       last_failure_[48] = {};
    char       version_[40] = {};   // the longest a version can print to (three bytes a field)

    static constexpr size_t kQueue = 4;
    LinkAction q_[kQueue] = {};
    size_t     q_head_ = 0;
    size_t     q_n_ = 0;
    uint8_t    tx_[kQueue][kMaxCommand + 8] = {};
    size_t     tx_next_ = 0;
};

}  // namespace skytraq

#endif  // SKYTRAQ_LINK_H
