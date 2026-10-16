// MosaicLink.h — finding and setting up the mosaic-G5 on its COM port (pure, host-compilable)
//
// The receiver can be anywhere when the flight computer starts: still
// booting (the switched rail that feeds its buck comes up with the FC), at
// the factory 115200 baud with nothing streaming, or already at the target
// baud and streaming the setup a previous boot saved into it. This state
// machine finds it and sets it up whichever it is, and again whenever a
// running link goes silent (a receiver reset in flight), without blocking:
// the driver feeds it the parser's items and the time, and carries out the
// actions it returns (set the UART baud, send a command). begin() runs it to
// RUNNING under a deadline; the poll task runs it after that.
//
// Commands go one at a time: the receiver's reply ends with its prompt
// ("COM1>"), and a command sent before the prompt can be lost (Reference
// Guide 3.1). A reply is "$R: <command>" when the receiver took it, "$R?"
// when it refused it. Host-tested in tests_cpp/test_mosaic_link.cpp.
#ifndef MOSAICLINK_H
#define MOSAICLINK_H

#include <cstddef>
#include <cstdint>

namespace mosaic
{

// One command of the setup. "{cd}" in the text stands for the port the
// receiver's prompt names (COM1 on the V9 harness).
struct SetupCommand
{
    const char* text;
    bool        required;   // without it the flight log loses what this link is for
};

struct LinkConfig
{
    uint32_t target_baud   = 2000000;
    uint32_t factory_baud  = 115200;
    uint32_t probe_ms      = 400;    // listen at one baud for a prompt or a block
    uint32_t switch_ms     = 250;    // after "scs": let the reply and the receiver's switch finish
    uint32_t reply_ms      = 1500;   // a set/exe command's reply, through its prompt
    uint32_t list_reply_ms = 3000;   // lif's longer reply
    uint32_t save_ms       = 5000;   // eccf writes the receiver's flash
    uint8_t  retries       = 2;      // re-sends of an unanswered command
    uint32_t silence_ms    = 2000;   // running with no block this long: find the receiver again
    bool     save_boot     = true;   // eccf, Current, Boot once per start()
};

enum class LinkState : uint8_t
{
    PROBE,         // listening for the receiver at one baud
    IDENTIFY,      // lif, Identification
    SWITCH_BAUD,   // scs sent; waiting out the switch
    VERIFY_BAUD,   // at the target baud, waiting for the prompt
    CONFIGURE,     // the setup commands
    SAVE,          // eccf, Current, Boot
    RUNNING,       // set up; watching for silence
};

struct LinkAction
{
    enum Kind : uint8_t { NONE, SET_BAUD, SEND };
    Kind        kind = NONE;
    uint32_t    baud = 0;         // SET_BAUD
    const char* text = nullptr;   // SEND: the bytes, CR included; valid until the next poll()
};

class MosaicLink
{
public:
    static constexpr size_t kMaxText = 320;   // longest command after "{cd}" is expanded

    // Begin looking for the receiver. `cmds` must outlive the link.
    void start(const LinkConfig& cfg, const SetupCommand* cmds, size_t n, uint32_t now_ms);

    // The parser's items, as they arrive.
    void onBlock(uint32_t now_ms);
    void onLine(const char* line, uint32_t now_ms);
    void onPrompt(const char* port, uint32_t now_ms);

    // Let time pass. Returns the next thing the driver must do, NONE when
    // nothing; call again until NONE.
    LinkAction poll(uint32_t now_ms);

    // A one-off command while RUNNING (the navigation data re-sent after
    // launch). False if one is still waiting or the text does not fit; it is
    // sent at the next poll() that finds the link idle and running.
    bool queueCommand(const char* text);

    LinkState   state() const { return state_; }
    bool        running() const { return state_ == LinkState::RUNNING; }
    uint32_t    baud() const { return baud_; }
    const char* port() const { return port_; }
    const char* product() const { return product_; }     // from lif, e.g. "mosaic-G5 P3"
    const char* firmware() const { return firmware_; }   // e.g. "1.0.0"
    uint32_t    setups() const { return setups_; }       // times the setup finished
    uint32_t    relinks() const { return relinks_; }     // times a running link went silent
    uint32_t    refused() const { return refused_; }     // commands refused or unanswered, last setup
    uint32_t    requiredFailed() const { return required_failed_; }
    bool        saved() const { return saved_; }
    // The last refused or unanswered command, for the boot console.
    const char* lastFailure() const { return last_failure_; }
    uint32_t    oneOffsSent() const { return oneoffs_sent_; }
    uint32_t    oneOffsRefused() const { return oneoffs_refused_; }

private:
    void enterProbe(uint32_t baud, uint32_t now_ms);
    void enterIdentify(uint32_t now_ms);
    void enterSwitch(uint32_t now_ms);
    void enterVerify(uint32_t now_ms);
    void enterConfigure(uint32_t now_ms);
    void sendCommand(uint32_t now_ms);
    void enterSave(uint32_t now_ms);
    void enterRunning(uint32_t now_ms);
    void nextCommand(uint32_t now_ms);
    void noteFailure(const char* text);

    void push(LinkAction::Kind kind, uint32_t baud, const char* text);
    void setBaud(uint32_t baud);
    void send(const char* text);          // expands "{cd}", appends CR
    void sendRaw(const char* text);       // as it is

    LinkConfig          cfg_;
    const SetupCommand* cmds_ = nullptr;
    size_t              n_cmds_ = 0;
    LinkState           state_ = LinkState::PROBE;
    uint32_t            t0_ = 0;          // when the current wait began
    uint32_t            baud_ = 0;        // what the UART was last set to
    uint32_t            probe_baud_ = 0;
    bool                saw_block_ = false;
    uint8_t             escapes_ = 0;
    size_t              cmd_ = 0;
    uint8_t             attempt_ = 0;
    uint8_t             timeouts_in_row_ = 0;
    bool                got_ok_ = false;
    bool                got_refused_ = false;
    bool                saved_ = false;
    uint32_t            last_block_ms_ = 0;

    // RUNNING-time one-off command
    char                oneoff_[kMaxText] = {};
    bool                oneoff_pending_ = false;
    bool                oneoff_waiting_ = false;
    uint32_t            oneoff_t0_ = 0;
    uint32_t            oneoffs_sent_ = 0;
    uint32_t            oneoffs_refused_ = 0;

    char                port_[8] = "COM1";
    char                product_[32] = {};
    char                firmware_[16] = {};
    char                last_failure_[64] = {};
    uint32_t            setups_ = 0;
    uint32_t            relinks_ = 0;
    uint32_t            refused_ = 0;
    uint32_t            required_failed_ = 0;

    // Actions waiting to be handed out (at most a baud change and a send).
    static constexpr size_t kQueue = 4;
    LinkAction          q_[kQueue] = {};
    size_t              q_head_ = 0;
    size_t              q_n_ = 0;
    // One buffer per queued send, so a SEND queued behind another stays valid.
    char                tx_[kQueue][kMaxText + 2] = {};
    size_t              tx_next_ = 0;
};

}  // namespace mosaic

#endif  // MOSAICLINK_H
