// MosaicLink.cpp — see MosaicLink.h. Pure: no ESP-IDF, host-tested.
#include "MosaicLink.h"

#include <cstdio>
#include <cstring>

namespace mosaic
{

namespace
{

inline bool elapsed(uint32_t now_ms, uint32_t t0_ms, uint32_t ms)
{
    return static_cast<uint32_t>(now_ms - t0_ms) >= ms;
}

inline bool startsWith(const char* s, const char* prefix)
{
    return std::strncmp(s, prefix, std::strlen(prefix)) == 0;
}

// The value of `key"..."` in a lif line, e.g. key `product="`.
bool attrValue(const char* line, const char* key, char* out, size_t cap)
{
    const char* p = std::strstr(line, key);
    if (p == nullptr || cap == 0) return false;
    p += std::strlen(key);
    size_t k = 0;
    while (*p != '\0' && *p != '"' && k + 1 < cap) out[k++] = *p++;
    out[k] = '\0';
    return k > 0;
}

// Ten S's then Enter reopen a port whose input was set to something other
// than commands (Reference Guide 3.1).
constexpr const char* kEscape = "SSSSSSSSSS\r";

}  // namespace

void MosaicLink::start(const LinkConfig& cfg, const SetupCommand* cmds, size_t n, uint32_t now_ms)
{
    cfg_ = cfg;
    cmds_ = cmds;
    n_cmds_ = n;
    saved_ = false;
    setups_ = 0;
    relinks_ = 0;
    refused_ = 0;
    required_failed_ = 0;
    product_[0] = '\0';
    firmware_[0] = '\0';
    last_failure_[0] = '\0';
    oneoff_pending_ = false;
    oneoff_waiting_ = false;
    q_head_ = 0;
    q_n_ = 0;
    baud_ = 0;
    enterProbe(cfg_.target_baud, now_ms);
}

// ── Items from the parser ───────────────────────────────────────────────

void MosaicLink::onBlock(uint32_t now_ms)
{
    last_block_ms_ = now_ms;
    // Alive at this baud. The CR sent on entry should bring the prompt; if
    // it does not, poll() tries the escape sequence before moving on.
    if (state_ == LinkState::PROBE) saw_block_ = true;
}

void MosaicLink::onLine(const char* line, uint32_t now_ms)
{
    (void)now_ms;
    const bool ok = startsWith(line, "$R:");
    const bool refused = startsWith(line, "$R?");
    switch (state_)
    {
        case LinkState::IDENTIFY:
            if (product_[0] == '\0') attrValue(line, "product=\"", product_, sizeof(product_));
            if (firmware_[0] == '\0') attrValue(line, "<firmware version=\"", firmware_, sizeof(firmware_));
            break;
        case LinkState::SWITCH_BAUD:
            if (refused)
            {
                // It kept its baud. The setup goes ahead at this one, but
                // 20 Hz of measurements cannot fit it: a required failure.
                refused_++;
                required_failed_++;
                noteFailure("scs (baud)");
                state_ = LinkState::VERIFY_BAUD;   // the prompt that follows starts CONFIGURE
                t0_ = now_ms;
            }
            break;
        case LinkState::CONFIGURE:
        case LinkState::SAVE:
            if (ok) got_ok_ = true;
            if (refused) got_refused_ = true;
            break;
        case LinkState::RUNNING:
            if (oneoff_waiting_)
            {
                if (ok) got_ok_ = true;
                if (refused) got_refused_ = true;
            }
            break;
        default:
            break;
    }
}

void MosaicLink::onPrompt(const char* port, uint32_t now_ms)
{
    if (std::strcmp(port, "STOP") != 0)
    {
        std::strncpy(port_, port, sizeof(port_) - 1);
        port_[sizeof(port_) - 1] = '\0';
    }
    switch (state_)
    {
        case LinkState::PROBE:
            enterIdentify(now_ms);
            break;
        case LinkState::IDENTIFY:
            if (baud_ != cfg_.target_baud)
                enterSwitch(now_ms);
            else
                enterConfigure(now_ms);
            break;
        case LinkState::VERIFY_BAUD:
            enterConfigure(now_ms);
            break;
        case LinkState::CONFIGURE:
            if (got_ok_)
            {
                timeouts_in_row_ = 0;
                nextCommand(now_ms);
            }
            else if (got_refused_)
            {
                timeouts_in_row_ = 0;
                refused_++;
                if (cmds_[cmd_].required) required_failed_++;
                noteFailure(cmds_[cmd_].text);
                nextCommand(now_ms);
            }
            // A prompt with no reply before it is not this command's: wait on.
            break;
        case LinkState::SAVE:
            saved_ = got_ok_;
            if (!saved_) noteFailure("eccf, Current, Boot");
            enterRunning(now_ms);
            break;
        case LinkState::RUNNING:
            if (oneoff_waiting_)
            {
                if (!got_ok_) oneoffs_refused_++;
                oneoff_waiting_ = false;
            }
            break;
        default:   // SWITCH_BAUD: the old baud's prompt after the reply
            break;
    }
}

// ── Time ────────────────────────────────────────────────────────────────

LinkAction MosaicLink::poll(uint32_t now_ms)
{
    switch (state_)
    {
        case LinkState::PROBE:
            if (elapsed(now_ms, t0_, cfg_.probe_ms))
            {
                if (saw_block_ && escapes_ < 2)
                {
                    // Talking, but not to commands: reopen the port's input.
                    escapes_++;
                    saw_block_ = false;
                    sendRaw(kEscape);
                    t0_ = now_ms;
                }
                else
                {
                    enterProbe(probe_baud_ == cfg_.target_baud ? cfg_.factory_baud : cfg_.target_baud, now_ms);
                }
            }
            break;
        case LinkState::IDENTIFY:
            if (elapsed(now_ms, t0_, cfg_.list_reply_ms)) enterProbe(baud_, now_ms);
            break;
        case LinkState::SWITCH_BAUD:
            if (elapsed(now_ms, t0_, cfg_.switch_ms)) enterVerify(now_ms);
            break;
        case LinkState::VERIFY_BAUD:
            // No prompt at the new baud: start over, the old baud first.
            if (elapsed(now_ms, t0_, 2 * cfg_.probe_ms)) enterProbe(cfg_.factory_baud, now_ms);
            break;
        case LinkState::CONFIGURE:
            if (elapsed(now_ms, t0_, cfg_.reply_ms))
            {
                if (++timeouts_in_row_ >= 3)
                {
                    enterProbe(baud_, now_ms);   // it stopped answering altogether
                }
                else if (attempt_ < cfg_.retries)
                {
                    attempt_++;
                    sendCommand(now_ms);
                }
                else
                {
                    refused_++;
                    if (cmds_[cmd_].required) required_failed_++;
                    noteFailure(cmds_[cmd_].text);
                    nextCommand(now_ms);
                }
            }
            break;
        case LinkState::SAVE:
            if (elapsed(now_ms, t0_, cfg_.save_ms))
            {
                noteFailure("eccf, Current, Boot");
                enterRunning(now_ms);
            }
            break;
        case LinkState::RUNNING:
            if (elapsed(now_ms, last_block_ms_, cfg_.silence_ms))
            {
                relinks_++;
                oneoff_waiting_ = false;
                enterProbe(cfg_.target_baud, now_ms);
                break;
            }
            if (oneoff_waiting_ && elapsed(now_ms, oneoff_t0_, cfg_.reply_ms))
            {
                oneoff_waiting_ = false;
                oneoffs_refused_++;
            }
            if (oneoff_pending_ && !oneoff_waiting_)
            {
                oneoff_pending_ = false;
                oneoff_waiting_ = true;
                oneoff_t0_ = now_ms;
                got_ok_ = false;
                got_refused_ = false;
                send(oneoff_);
                oneoffs_sent_++;
            }
            break;
    }

    LinkAction a;
    if (q_n_ > 0)
    {
        a = q_[q_head_];
        q_head_ = (q_head_ + 1) % kQueue;
        q_n_--;
    }
    return a;
}

bool MosaicLink::queueCommand(const char* text)
{
    if (oneoff_pending_ || oneoff_waiting_ || std::strlen(text) >= sizeof(oneoff_)) return false;
    std::strcpy(oneoff_, text);
    oneoff_pending_ = true;
    return true;
}

// ── States ──────────────────────────────────────────────────────────────

void MosaicLink::enterProbe(uint32_t baud, uint32_t now_ms)
{
    state_ = LinkState::PROBE;
    probe_baud_ = baud;
    saw_block_ = false;
    escapes_ = 0;
    t0_ = now_ms;
    setBaud(baud);
    sendRaw("\r");   // an empty command: the reply is the prompt
}

void MosaicLink::enterIdentify(uint32_t now_ms)
{
    state_ = LinkState::IDENTIFY;
    t0_ = now_ms;
    refused_ = 0;
    required_failed_ = 0;
    last_failure_[0] = '\0';
    send("lif, Identification");
}

void MosaicLink::enterSwitch(uint32_t now_ms)
{
    state_ = LinkState::SWITCH_BAUD;
    t0_ = now_ms;
    char cmd[80];
    std::snprintf(cmd, sizeof(cmd), "scs, {cd}, baud%lu, bits8, No, bit1, none",
                  static_cast<unsigned long>(cfg_.target_baud));
    send(cmd);
}

void MosaicLink::enterVerify(uint32_t now_ms)
{
    state_ = LinkState::VERIFY_BAUD;
    t0_ = now_ms;
    setBaud(cfg_.target_baud);
    sendRaw("\r");
}

void MosaicLink::enterConfigure(uint32_t now_ms)
{
    state_ = LinkState::CONFIGURE;
    cmd_ = 0;
    attempt_ = 0;
    timeouts_in_row_ = 0;
    if (n_cmds_ == 0)
    {
        cmd_ = 0;
        nextCommand(now_ms);   // straight on to the save
        return;
    }
    sendCommand(now_ms);
}

void MosaicLink::sendCommand(uint32_t now_ms)
{
    got_ok_ = false;
    got_refused_ = false;
    t0_ = now_ms;
    send(cmds_[cmd_].text);
}

void MosaicLink::nextCommand(uint32_t now_ms)
{
    if (n_cmds_ > 0) cmd_++;
    attempt_ = 0;
    if (cmd_ < n_cmds_)
    {
        sendCommand(now_ms);
        return;
    }
    if (cfg_.save_boot && !saved_)
        enterSave(now_ms);
    else
        enterRunning(now_ms);
}

void MosaicLink::enterSave(uint32_t now_ms)
{
    state_ = LinkState::SAVE;
    t0_ = now_ms;
    got_ok_ = false;
    got_refused_ = false;
    send("eccf, Current, Boot");
}

void MosaicLink::enterRunning(uint32_t now_ms)
{
    state_ = LinkState::RUNNING;
    last_block_ms_ = now_ms;
    timeouts_in_row_ = 0;
    setups_++;
}

// ── Actions ─────────────────────────────────────────────────────────────

void MosaicLink::noteFailure(const char* text)
{
    std::strncpy(last_failure_, text, sizeof(last_failure_) - 1);
    last_failure_[sizeof(last_failure_) - 1] = '\0';
}

void MosaicLink::push(LinkAction::Kind kind, uint32_t baud, const char* text)
{
    if (q_n_ >= kQueue) return;   // cannot happen: a state queues at most a baud and a send
    LinkAction& a = q_[(q_head_ + q_n_) % kQueue];
    a.kind = kind;
    a.baud = baud;
    a.text = text;
    q_n_++;
}

void MosaicLink::setBaud(uint32_t baud)
{
    if (baud == baud_) return;
    baud_ = baud;
    push(LinkAction::SET_BAUD, baud, nullptr);
}

void MosaicLink::send(const char* text)
{
    char* out = tx_[tx_next_];
    tx_next_ = (tx_next_ + 1) % kQueue;
    const size_t port_n = std::strlen(port_);
    size_t k = 0;
    for (const char* p = text; *p != '\0' && k < kMaxText;)
    {
        if (std::strncmp(p, "{cd}", 4) == 0)
        {
            for (size_t i = 0; i < port_n && k < kMaxText; i++) out[k++] = port_[i];
            p += 4;
        }
        else
        {
            out[k++] = *p++;
        }
    }
    out[k++] = '\r';
    out[k] = '\0';
    push(LinkAction::SEND, 0, out);
}

void MosaicLink::sendRaw(const char* text)
{
    char* out = tx_[tx_next_];
    tx_next_ = (tx_next_ + 1) % kQueue;
    std::strncpy(out, text, kMaxText + 1);
    out[kMaxText + 1] = '\0';
    push(LinkAction::SEND, 0, out);
}

}  // namespace mosaic
