// gnss_detect.cpp — see gnss_detect.h. Pure: no ESP-IDF, host-tested.
#include "gnss_detect.h"

#include <cstring>

namespace gnss_detect
{

namespace
{

inline bool elapsed(uint32_t now_ms, uint32_t t0_ms, uint32_t ms)
{
    return static_cast<uint32_t>(now_ms - t0_ms) >= ms;
}

inline int hexValue(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

}  // namespace

// UBX-MON-VER, polled: class 0x0A, id 0x04, no payload, its checksum.
const uint8_t kUbxMonVerPoll[8] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
const uint8_t kCr[1] = {'\r'};
// 0x02 with software type 1 (system code); the checksum is the XOR of the two.
const uint8_t kSkyTraqVersionQuery[9] = {0xA0, 0xA1, 0x00, 0x02, 0x02, 0x01, 0x03, 0x0D, 0x0A};

bool isUbloxBaud(uint32_t baud)
{
    // TR_GNSSReceiverUBlox_Serial.cpp's probe_bauds.
    return baud == 38400U || baud == 9600U || baud == 230400U || baud == 115200U ||
           baud == 460800U || baud == 57600U;
}

uint16_t ubloxListenMs(uint32_t baud)
{
    if (baud == 0) return 100;
    // 100 ms for the receiver to answer, then ~300 bytes at ten bits each.
    return static_cast<uint16_t>(100U + 3000000U / baud);
}

const char* proofName(Proof p)
{
    switch (p)
    {
        case Proof::SBF_BLOCK: return "an SBF block";
        case Proof::PROMPT:    return "its prompt";
        case Proof::REPLY:     return "a command reply";
        case Proof::UBX_FRAME: return "a UBX frame";
        case Proof::NMEA:      return "NMEA";
        case Proof::RTCM_TR:   return "an RTCM 4001 frame";
        case Proof::SKYTRAQ_FRAME: return "a SkyTraq frame";
        default:               return "nothing";
    }
}

// The PX1105R's two probes into a plan: first of all with it the hint (then
// its own budget, and the gnss-sdr-dev board's driver when nothing answers:
// kBudgetPx1105rMs), else just before the first other probe at 115200, where
// its NMEA would otherwise read as a u-blox's (or at the end with none there).
static void addPx1105r(Plan& p, bool first)
{
    const Probe px[2] = {Probe{kPx1105rRunBaud, GNSS_TYPE_PX1105R, kPx1105rRunListenMs},
                         Probe{kPx1105rFactoryBaud, GNSS_TYPE_PX1105R, kPx1105rFactoryListenMs}};
    size_t at = p.n;
    if (first)
    {
        at = 0;
    }
    else
    {
        for (size_t i = 0; i < p.n; i++)
        {
            if (p.probes[i].baud == kPx1105rFactoryBaud)
            {
                at = i;
                break;
            }
        }
    }
    size_t room = Plan::kMaxProbes - p.n;
    const size_t add = room < 2 ? room : 2;
    for (size_t i = p.n; i-- > at;) p.probes[i + add] = p.probes[i];
    for (size_t k = 0; k < add; k++) p.probes[at + k] = px[k];
    p.n += add;
    p.px1105r = true;
    if (first)
    {
        p.budget_ms = kBudgetPx1105rMs;
        p.fallback = GNSS_TYPE_SDR;
    }
}

namespace
{
// The u-blox, where it most likely is first: still at the run baud when it
// kept its power, else where its driver found it last, then the two factory
// rates (38400 is the M10's; the bench SAM-M10Q comes up at 9600).
size_t ubloxBauds(uint32_t ublox_last_baud, bool receiver_kept_power, uint32_t (&ub)[4])
{
    size_t nub = 0;
    auto add = [&](uint32_t b) {
        if (!isUbloxBaud(b) || nub >= 4) return;
        for (size_t i = 0; i < nub; i++)
        {
            if (ub[i] == b) return;
        }
        ub[nub++] = b;
    };
    if (receiver_kept_power) add(kUbloxRunBaud);
    add(ublox_last_baud);
    add(38400U);
    add(9600U);
    add(kUbloxRunBaud);
    return nub;
}
}  // namespace

Plan restoredFlightPlan(uint32_t ublox_last_baud, bool receiver_kept_power, bool px1105r)
{
    uint32_t ub[4] = {};
    const size_t nub = ubloxBauds(ublox_last_baud, receiver_kept_power, ub);
    Plan p;
    for (size_t i = 0; i < nub; i++) p.probes[p.n++] = Probe{ub[i], GNSS_TYPE_UBLOX, ubloxListenMs(ub[i])};
    p.budget_ms = kBudgetRestoredMs; // for a u-blox booting with the FC
    p.fallback = GNSS_TYPE_NONE;     // silent: GNSS-absent, no search
    p.px1105r = px1105r;
    return p;
}

Plan planFor(uint8_t hint, uint32_t ublox_last_baud, bool receiver_kept_power, bool px1105r)
{
    uint32_t ub[4] = {};
    const size_t nub = ubloxBauds(ublox_last_baud, receiver_kept_power, ub);

    Plan p;
    auto addMosaic = [&]() {
        p.probes[p.n++] = Probe{kMosaicTargetBaud, GNSS_TYPE_MOSAIC, kMosaicTargetListenMs};
        p.probes[p.n++] = Probe{kMosaicFactoryBaud, GNSS_TYPE_MOSAIC, kMosaicFactoryListenMs};
    };
    auto addUblox = [&]() {
        for (size_t i = 0; i < nub; i++) p.probes[p.n++] = Probe{ub[i], GNSS_TYPE_UBLOX, ubloxListenMs(ub[i])};
    };
    const uint16_t sdr_listen_ms = px1105r ? kSdrListenRawMs : kSdrListenMs;
    auto addSdr = [&]() { p.probes[p.n++] = Probe{kSdrBaud, GNSS_TYPE_SDR, sdr_listen_ms}; };

    if (hint == GNSS_TYPE_SDR)
    {
        addSdr();
        addUblox();
        addMosaic();
        p.budget_ms = kBudgetSdrMs;
        // Silent is what the board is after every cold power-up (it sends from
        // its first fix): its driver listens for it, and the hint stays.
        p.fallback = GNSS_TYPE_SDR;
    }
    else if (hint == GNSS_TYPE_MOSAIC)
    {
        addMosaic();
        addUblox();
        addSdr();
        p.budget_ms = kBudgetMosaicMs;
        // A board that had a mosaic-G5 and now answers nothing flies
        // GNSS-absent after the mosaic driver's own 30 s, as the mosaic image
        // did: the u-blox driver's 60 s on top would bring a boot to the
        // setup watchdog. The driver forgets the hint then, so the next boot
        // takes the plan for a board with nothing known.
        p.fallback = GNSS_TYPE_NONE;
    }
    else
    {
        // The board's listening probe before the mosaic's 2 Mbaud CR, so a
        // board that is streaming is found without being sent anything.
        addUblox();
        addSdr();
        addMosaic();
        p.budget_ms = (hint == GNSS_TYPE_UBLOX) ? kBudgetUbloxMs : kBudgetNoneMs;
        p.fallback = GNSS_TYPE_UBLOX;
    }
    if (px1105r && p.fallback == GNSS_TYPE_UBLOX && p.budget_ms < kBudgetBoardMs) p.budget_ms = kBudgetBoardMs;
    if (px1105r) addPx1105r(p, hint == GNSS_TYPE_PX1105R);
    return p;
}

Plan secondLookPlan(bool px1105r)
{
    Plan p;
    p.probes[p.n++] = Probe{kSdrBaud, GNSS_TYPE_SDR, px1105r ? kSdrListenRawMs : kSdrListenMs};
    p.probes[p.n++] = Probe{kMosaicTargetBaud, GNSS_TYPE_MOSAIC, kMosaicTargetListenMs};
    p.probes[p.n++] = Probe{kMosaicFactoryBaud, GNSS_TYPE_MOSAIC, kMosaicFactoryListenMs};
    p.budget_ms = px1105r ? kBudgetSecondLookRawMs : 1000;   // twice round
    p.fallback = GNSS_TYPE_NONE;
    if (px1105r) addPx1105r(p, false);
    return p;
}

// ── UBX ─────────────────────────────────────────────────────────────────

bool UbxScanner::step(uint8_t b)
{
    switch (state_)
    {
        case 0:   // sync 1
            if (b == 0xB5) state_ = 1;
            return false;
        case 1:   // sync 2
            if (b == 0x62)
            {
                ck_a_ = 0;
                ck_b_ = 0;
                state_ = 2;
            }
            else if (b != 0xB5)
            {
                state_ = 0;
            }
            return false;
        case 2:
            cls_ = b;
            add(b);
            state_ = 3;
            return false;
        case 3:
            id_ = b;
            add(b);
            state_ = 4;
            return false;
        case 4:
            len_ = b;
            add(b);
            state_ = 5;
            return false;
        case 5:
            len_ = static_cast<uint16_t>(len_ | (b << 8));
            add(b);
            got_ = 0;
            if (len_ > kMaxPayload)
                state_ = (b == 0xB5) ? 1 : 0;   // a false sync
            else
                state_ = (len_ == 0) ? 7 : 6;
            return false;
        case 6:
            add(b);
            if (++got_ >= len_) state_ = 7;
            return false;
        case 7:
            if (b == ck_a_)
            {
                state_ = 8;
            }
            else
            {
                state_ = (b == 0xB5) ? 1 : 0;
            }
            return false;
        case 8:
            state_ = 0;
            if (b == ck_b_) return true;
            if (b == 0xB5) state_ = 1;
            return false;
        default:
            state_ = 0;
            return false;
    }
}

// ── NMEA ────────────────────────────────────────────────────────────────

bool isNmeaSentence(const char* line)
{
    if (line == nullptr || line[0] != '$') return false;
    const size_t n = std::strlen(line);
    // The shortest: "$GPTXT,*hh"; NMEA 0183 caps a sentence at 82 characters
    // with its line end, and some receivers run a little over.
    if (n < 10 || n > 100) return false;
    size_t i = 1;
    while (i < n && ((line[i] >= 'A' && line[i] <= 'Z') || (line[i] >= '0' && line[i] <= '9'))) i++;
    const size_t address = i - 1;
    if (address < 2 || address > 6 || i >= n || line[i] != ',') return false;
    if (line[n - 3] != '*') return false;
    uint8_t sum = 0;
    for (size_t k = 1; k < n - 3; k++)
    {
        const char c = line[k];
        if (c < 0x20 || c > 0x7E || c == '*' || c == '$') return false;
        sum = static_cast<uint8_t>(sum ^ static_cast<uint8_t>(c));
    }
    const int hi = hexValue(line[n - 2]);
    const int lo = hexValue(line[n - 1]);
    if (hi < 0 || lo < 0) return false;
    return static_cast<uint8_t>((hi << 4) | lo) == sum;
}

// ── Detector ────────────────────────────────────────────────────────────

void Detector::start(const Plan& plan, uint32_t now_ms)
{
    plan_ = plan;
    if (plan_.n > Plan::kMaxProbes) plan_.n = Plan::kMaxProbes;
    t_start_ = now_ms;
    now_ms_ = now_ms;
    baud_ = 0;
    done_ = false;
    found_ = GNSS_TYPE_NONE;
    proof_ = Proof::NONE;
    found_baud_ = 0;
    elapsed_ms_ = 0;
    probes_run_ = 0;
    q_head_ = 0;
    q_n_ = 0;
    skytraq_sink_.d = this;
    if (plan_.n == 0)
    {
        finish(GNSS_TYPE_NONE, Proof::NONE, now_ms);
        return;
    }
    enterProbe(0, now_ms);
}

void Detector::enterProbe(size_t i, uint32_t now_ms)
{
    i_ = i;
    t0_ = now_ms;
    nmea_ = false;
    ubx_.reset();
    parser_.reset();
    rtcm_.reset();
    skytraq_.reset();
    const Probe& p = plan_.probes[i];
    if (p.baud != baud_)
    {
        baud_ = p.baud;
        push(Action::SET_BAUD, p.baud, nullptr, 0);
    }
    if (p.asks == GNSS_TYPE_MOSAIC)
        push(Action::SEND, 0, kCr, sizeof(kCr));
    else if (p.asks == GNSS_TYPE_UBLOX)
        push(Action::SEND, 0, kUbxMonVerPoll, sizeof(kUbxMonVerPoll));
    else if (p.asks == GNSS_TYPE_PX1105R)
        push(Action::SEND, 0, kSkyTraqVersionQuery, sizeof(kSkyTraqVersionQuery));
    // GNSS_TYPE_SDR: nothing; it listens
    probes_run_++;
}

void Detector::feed(const uint8_t* data, size_t n, uint32_t now_ms)
{
    if (done_ || data == nullptr || n == 0) return;
    now_ms_ = now_ms;
    // The gnss-sdr-dev board's frames, at its baud: whichever probe is
    // listening there (its own, or the mosaic's 2 Mbaud one).
    if (baud_ == kSdrBaud)
    {
        rtcm_.feed(data, n, *this);   // onFrame below may finish()
        if (done_) return;
    }
    // u-blox proof counts only at a baud a u-blox can be at, which 2 Mbaud is
    // not (they stop at 921600): a set-up mosaic's SBF is binary and can hold
    // anything, so the one place it streams is kept out of the UBX scan.
    if (isUbloxBaud(baud_))
    {
        for (size_t k = 0; k < n; k++)
        {
            if (ubx_.step(data[k]))
            {
                finish(GNSS_TYPE_UBLOX, Proof::UBX_FRAME, now_ms);
                return;
            }
        }
    }
    parser_.feed(data, n, *this);   // the sink below may finish()
    // The PX1105R's frames, at its two bauds, when it is looked for.
    if (!done_ && plan_.px1105r && (baud_ == kPx1105rRunBaud || baud_ == kPx1105rFactoryBaud))
        skytraq_.feed(data, n, skytraq_sink_);   // onSkyTraqFrame below may finish()
}

Action Detector::poll(uint32_t now_ms)
{
    if (!done_ && q_n_ == 0 && elapsed(now_ms, t0_, plan_.probes[i_].listen_ms))
    {
        if (nmea_)
        {
            finish(GNSS_TYPE_UBLOX, Proof::NMEA, now_ms);
        }
        else if (i_ + 1 < plan_.n)
        {
            enterProbe(i_ + 1, now_ms);
        }
        else if (elapsed(now_ms, t_start_, plan_.budget_ms))
        {
            finish(GNSS_TYPE_NONE, Proof::NONE, now_ms);   // round the plan, and out of time
        }
        else
        {
            enterProbe(0, now_ms);
        }
    }

    Action a;
    if (q_n_ > 0)
    {
        a = q_[q_head_];
        q_head_ = (q_head_ + 1) % kQueue;
        q_n_--;
    }
    return a;
}

void Detector::finish(uint8_t type, Proof proof, uint32_t now_ms)
{
    done_ = true;
    found_ = type;
    proof_ = proof;
    found_baud_ = (type != GNSS_TYPE_NONE) ? baud_ : 0;
    elapsed_ms_ = static_cast<uint32_t>(now_ms - t_start_);
    q_n_ = 0;
}

void Detector::push(Action::Kind kind, uint32_t baud, const uint8_t* data, size_t len)
{
    if (q_n_ >= kQueue) return;   // cannot happen: a probe queues a baud change and a send
    Action& a = q_[(q_head_ + q_n_) % kQueue];
    a.kind = kind;
    a.baud = baud;
    a.data = data;
    a.len = len;
    q_n_++;
}

// ── What the mosaic sends ───────────────────────────────────────────────

void Detector::onBlock(const uint8_t* block, size_t len, size_t index)
{
    (void)block;
    (void)len;
    (void)index;
    if (!done_) finish(GNSS_TYPE_MOSAIC, Proof::SBF_BLOCK, now_ms_);
}

void Detector::onLine(const char* line, size_t index)
{
    (void)index;
    if (done_ || line == nullptr) return;
    // "$R: <command>", "$R? <why>", "$R; <command>": Reference Guide 3.1.3.
    if (line[0] == '$' && line[1] == 'R' && (line[2] == ':' || line[2] == '?' || line[2] == ';'))
    {
        finish(GNSS_TYPE_MOSAIC, Proof::REPLY, now_ms_);
        return;
    }
    // NMEA at 115200 is the PX1105R's as likely as a u-blox's when the plan
    // looks for it: there only a UBX frame proves the u-blox.
    if (isUbloxBaud(baud_) && !(plan_.px1105r && baud_ == kPx1105rFactoryBaud) && isNmeaSentence(line))
        nmea_ = true;
}

void Detector::onPrompt(const char* port, size_t index)
{
    (void)port;
    (void)index;
    if (!done_) finish(GNSS_TYPE_MOSAIC, Proof::PROMPT, now_ms_);
}

// ── What the gnss-sdr-dev board sends ───────────────────────────────────

void Detector::onFrame(const uint8_t* frame, size_t len, size_t after)
{
    (void)after;
    // Its MSM7 and ephemerides are standard RTCM3 that another receiver could
    // send; TinkerRocket's own message is the board's alone.
    if (!done_ && sdr_rtcm::messageNumber(frame, len) == sdr_rtcm::kTrMessage)
        finish(GNSS_TYPE_SDR, Proof::RTCM_TR, now_ms_);
}

// ── What the PX1105R sends ──────────────────────────────────────────────

void Detector::SkyTraqSink::onFrame(const uint8_t* body, size_t len, size_t index)
{
    (void)body;
    (void)len;
    (void)index;
    if (d != nullptr) d->onSkyTraqFrame();
}

void Detector::onSkyTraqFrame()
{
    // Any frame: its length, XOR and 0D 0A checked, nothing else on J1 sends
    // one (the query's ACK and reply, or the set-up binary stream).
    if (!done_) finish(GNSS_TYPE_PX1105R, Proof::SKYTRAQ_FRAME, now_ms_);
}

// ── BringUp ─────────────────────────────────────────────────────────────

void BringUp::start(uint8_t hint, uint32_t ublox_last_baud, bool receiver_kept_power,
                    bool ublox_wires_swapped, bool px1105r, bool restored_flight)
{
    restored_ = restored_flight;
    // Only the receivers J1 takes are hints; anything else (a blank or
    // corrupt record) is nothing known. The PX1105R only where it is looked
    // for.
    px1105r_ = px1105r;
    hint_in_ = (hint == GNSS_TYPE_UBLOX || hint == GNSS_TYPE_MOSAIC || hint == GNSS_TYPE_SDR ||
                (px1105r && hint == GNSS_TYPE_PX1105R))
                   ? hint
                   : GNSS_TYPE_NONE;
    hint_ = hint_in_;
    proven_ = GNSS_TYPE_NONE;
    active_ = GNSS_TYPE_NONE;
    second_look_ = false;
    plan_ = restored_ ? restoredFlightPlan(ublox_last_baud, receiver_kept_power, px1105r)
                      : planFor(hint_in_, ublox_last_baud, receiver_kept_power, px1105r);
    if (ublox_wires_swapped && hint_in_ != GNSS_TYPE_MOSAIC && hint_in_ != GNSS_TYPE_SDR &&
        hint_in_ != GNSS_TYPE_PX1105R)
    {
        plan_.n = 0;   // nothing to probe: detected(NONE) runs the u-blox driver
        plan_.budget_ms = 0;
        // (a restored flight's: nothing can prove it, and its driver's search
        // is what a restored flight must not wait on -- GNSS-absent)
        plan_.fallback = restored_ ? GNSS_TYPE_NONE : GNSS_TYPE_UBLOX;
    }
    step_ = Step::DETECT;
}

void BringUp::detected(uint8_t found)
{
    if (step_ == Step::DETECT)
    {
        proven_ = found;
        if (found == GNSS_TYPE_UBLOX || found == GNSS_TYPE_MOSAIC || found == GNSS_TYPE_SDR ||
            found == GNSS_TYPE_PX1105R)
            hint_ = found;
        const uint8_t driver = (found != GNSS_TYPE_NONE) ? found : plan_.fallback;
        if (driver == GNSS_TYPE_UBLOX)
        {
            step_ = Step::BEGIN_UBLOX;
        }
        else if (driver == GNSS_TYPE_MOSAIC)
        {
            step_ = Step::BEGIN_MOSAIC;
        }
        else if (driver == GNSS_TYPE_SDR)
        {
            step_ = Step::BEGIN_SDR;
        }
        else if (driver == GNSS_TYPE_PX1105R)
        {
            step_ = Step::BEGIN_PX1105R;
        }
        else
        {
            // The mosaic hint and no answer: forget it, so a board whose
            // receiver changed is not held to the mosaic plan for good. A
            // restored flight's silence changes no hint (#1765): the next
            // pad boot looks as this flight's did.
            if (!restored_) hint_ = GNSS_TYPE_NONE;
            active_ = GNSS_TYPE_NONE;
            step_ = Step::DONE;
        }
        return;
    }
    if (step_ == Step::SECOND_LOOK)
    {
        if (found == GNSS_TYPE_MOSAIC)
        {
            hint_ = GNSS_TYPE_MOSAIC;
            step_ = Step::BEGIN_MOSAIC;
        }
        else if (found == GNSS_TYPE_SDR)
        {
            hint_ = GNSS_TYPE_SDR;
            step_ = Step::BEGIN_SDR;
        }
        else if (found == GNSS_TYPE_PX1105R)
        {
            hint_ = GNSS_TYPE_PX1105R;
            step_ = Step::BEGIN_PX1105R;
        }
        else
        {
            active_ = GNSS_TYPE_NONE;
            step_ = Step::DONE;
        }
    }
}

void BringUp::flightFoundLate()
{
    restored_ = true;
    // still probing: nothing proven is GNSS-absent, not the u-blox search
    if (step_ == Step::DETECT) plan_.fallback = GNSS_TYPE_NONE;
    if ((step_ == Step::BEGIN_UBLOX && proven_ == GNSS_TYPE_NONE) || step_ == Step::SECOND_LOOK)
    {
        // the hint is still the one the boot started with: nothing proven
        active_ = GNSS_TYPE_NONE;
        step_ = Step::DONE;
    }
}

void BringUp::driverResult(bool ok)
{
    if (step_ == Step::BEGIN_UBLOX)
    {
        if (ok)
        {
            // Found by its own search when the probes proved nothing (another
            // baud, swapped wires): the u-blox all the same.
            hint_ = GNSS_TYPE_UBLOX;
            active_ = GNSS_TYPE_UBLOX;
            step_ = Step::DONE;
        }
        else if (proven_ == GNSS_TYPE_NONE)
        {
            // It ran because nothing answered the probes. A mosaic-G5 still
            // booting then has had a minute since.
            second_look_ = true;
            plan_ = secondLookPlan(px1105r_);
            step_ = Step::SECOND_LOOK;
        }
        else
        {
            // A UBX frame proved the u-blox; no mosaic is on J1. (A restored
            // flight, #1765, runs the driver only for a u-blox proven so.)
            active_ = GNSS_TYPE_NONE;
            step_ = Step::DONE;
        }
        return;
    }
    if (step_ == Step::BEGIN_MOSAIC)
    {
        active_ = ok ? GNSS_TYPE_MOSAIC : GNSS_TYPE_NONE;
        step_ = Step::DONE;
        return;
    }
    if (step_ == Step::BEGIN_SDR)
    {
        // Its driver only fails when the UART will not come up.
        active_ = ok ? GNSS_TYPE_SDR : GNSS_TYPE_NONE;
        step_ = Step::DONE;
        return;
    }
    if (step_ == Step::BEGIN_PX1105R)
    {
        active_ = ok ? GNSS_TYPE_PX1105R : GNSS_TYPE_NONE;
        step_ = Step::DONE;
    }
}

}  // namespace gnss_detect
