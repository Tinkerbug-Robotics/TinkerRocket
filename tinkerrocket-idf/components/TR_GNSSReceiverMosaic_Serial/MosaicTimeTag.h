// MosaicTimeTag.h — when a mosaic-G5 raw epoch's time tag can be trusted: the clock bias that places it, and the 1PPS that confirms it
//
// An epoch's tag (MosaicRawEpoch.h) is mcuAt(TOW - dtr): the PPS clock line
// (GnssTimeSync) turns a GPS instant into MCU time, and dtr, the receiver's
// clock bias, turns the epoch's receiver time into a GPS instant. Each is the
// receiver's statement of GPS time: the line from its 1PPS edges (the last
// minute of them), dtr from its PVTGeodetic's RxClkBias. A tag is right only
// while both state the same GPS time.
//
// What went wrong (V9 on the RF rig, 2026-10-17, image 2f724a00): a 6.854 ms
// transmitter underrun (253,312 bytes at 18.48 MS/s) delayed the whole sky.
// The receiver lost it, re-acquired 15 s later, found its bias 6.85 ms larger,
// stepped its clock 7 ms (a jump count step of 7: a restart here) and moved
// its 1PPS to the new sky. The very next PVT's bias tagged the epochs at once;
// the line, a lower envelope over a minute of edges, kept the old seconds and
// could not follow a step later. Every whole second then read -6855 us (24 of
// 24 judged). And the epochs with no PVT bias to carry (26) went out with dtr
// 0, which is within 0.5 ms only while the receiver's clock is steered to the
// sky it tracks: not after the sky stepped, nor after its clock's 7 ms step.
//
// What the PPS tag check cannot see. At every whole second it reads
// ppsResidualUs(tag, that second's edge, that second's own RxClkBias), which
// is (the line less the edge) + (the PVT's bias less the dtr carried): the
// line against its edges, and the carry against the PVT. Whether the edges
// and the PVTs state the same GPS time it cannot tell: a receiver whose 1PPS
// is still on the old sky a second or more after its PVTs moved reads 0 there
// while every tag is off by the whole step. The receiver aligns its 1PPS from
// its PVT solution [RG 1.10], so in steady state they agree; after a step in
// the sky, only once it has re-aligned the pulse. And an edge stamped late (an
// interrupt held off) looks like a 1PPS that moved later. So the edges are
// also held against the PVTs' own account of GPS time (the chain, below).
//
// So the tag is right or untimed, never a mix of the two statements:
//   - the bias is carried from the last PVT that anchors the jump count
//     (pvtAnchorsJumps), by its drift and the jumps since (dtrMs), for up to
//     Config::carry_ms. The receiver's clock is free-running and steps only
//     whole milliseconds (counted), so the carry holds across a lost sky. No
//     anchor: untimed (NO_BIAS), never dtr 0, which is up to 0.5 ms off with
//     a fix and anything at all after a restart or a step in the sky;
//   - the anchors form a chain. Each one's bias less the bias carried to it
//     from the one before, by its drift and every jump since (the count's
//     jsum_ms, across a big jump's start-over: MosaicJumpState), is how far
//     the receiver's GPS time moved against its own clock: a step in the sky,
//     which a real one never makes. The chain's displacement (chain_ms_)
//     adds those up since the reference below, a ramp as well as a step.
//     Where the receiver's clock was lost (a reset: a gap, time going back,
//     a jump the count cannot hold) nothing carries; the sky is taken not to
//     have moved, and the 1PPS has to show it;
//   - the reference: the line where the last trusted edge sat, refreshed
//     every Config::ref_refresh_us while trusted and dropped once
//     Config::ref_max_age_us old (the crystal's drift is not known well
//     enough to run it on further). Each edge, as it pairs, against the
//     reference run on to its second is how far the 1PPS moved (edgeRefUs);
//     an edge that moved by the chain's displacement (within
//     Config::on_line_us) is on the chain. Off it, no epoch is timed
//     (OFF_CHAIN), its check confirms nothing, and on the line and off the
//     chain it breaks the chain (OFF_CHAIN): the line follows edges the PVTs
//     do not account for. The line may follow a step of its edges later in
//     time only onto edges on the chain (edgeOnChain: GnssTimeSync's
//     may_step_later): a late edge with no step in the PVTs is an interrupt
//     held off, untimed and set aside. With no reference (the first anchor,
//     a reference dropped) every edge is on the chain: the 1PPS is all there
//     is, and a receiver aligns its first pulses from its first fix;
//   - a chain breaks where its bias cannot be carried: the first anchor
//     (FIRST), the receiver's clock lost (RESET), the jump count starting
//     over at a big jump (REBASE), an anchor too old to carry (EXPIRED), a
//     PVT more than Config::bias_step_ms off the bias carried to it
//     (BIAS_STEP). A failed PPS check breaks it too (CHECK_FAILED), so does
//     a restart of the line (LINE_RESTART: its new seconds may be labelled
//     whole seconds off, which the check cannot see either) and an edge on
//     the line off the chain (OFF_CHAIN). After a break the tag is held: its
//     epochs go untimed (HELD) until Config::confirm whole seconds in a row,
//     tagged in the new chain, pass the PPS check within Config::on_line_us
//     against an edge on the line and on the chain. A failed one while held,
//     or one against an edge off the chain, starts that count over; one
//     against an edge that came late (notePpsTag judges it where it was due:
//     right for a held-off interrupt, wrong for the first edge of a step)
//     counts neither way. The check runs on those held seconds' tags all the
//     same (noteWholeSecond). A release refreshes the reference;
//   - each 1PPS edge, as it pairs, against the line it moved (notePair): an
//     edge more than Config::on_line_us off it holds every epoch untimed
//     (EDGE_OFF) until one is back on it.
// The PPS check's bar for a bad tag (PpsTagStats, Config::bar_us) stays
// 200 us; the decisions use half of it, so a tag that is handed over stays
// inside the bar while the line or the bias moves away.
//
// Pure and header-only, shared by the driver (TR_GNSSReceiverMosaic_Serial)
// and the replay (tools/raw_replay/mosaic_feed.h). Host tests:
// tests_cpp/test_mosaic_time_tag.cpp.
#ifndef MOSAIC_TIME_TAG_H
#define MOSAIC_TIME_TAG_H

#include <cmath>
#include <cstdint>

#include "MosaicRawEpoch.h"
#include "SbfParser.h"

namespace mosaic
{

class MosaicTimeTag
{
public:
    // A PVT's RxClkBias is a statement of the receiver's clock up to this,
    // ms (its do-not-use is -2e10). A receiver about to step its clock a
    // second or more says so here: an anchor, whose step breaks the chain.
    static constexpr double kMaxBiasMs = 1.0e6;

    struct Config
    {
        int64_t carry_ms = 120000;              // an anchor's bias carried at most this long (receiver time)
        double  bias_step_ms = 0.1;             // a PVT this far from the bias carried to it breaks the chain
        int32_t on_line_us = 100;               // an edge this close to the line or the chain, a check this small: pass
        int32_t bar_us = 200;                   // the PPS check's bar for a bad tag (PpsTagStats)
        uint8_t confirm = 2;                    // checks in a row that confirm a chain
        int64_t pair_max_age_us = 120000000;    // no edge paired this long: the line is the crystal's alone
        int64_t ref_refresh_us = 10000000;      // the reference moves up at most this often while trusted
        int64_t ref_max_age_us = 120000000;     // a reference this old is dropped
    };

    // Why an epoch is not handed over timed (TIMED: it is). In this order:
    // the first that applies.
    enum class Why : uint8_t
    {
        TIMED,
        NO_LINE,        // the PPS line is not valid
        NOT_FINETIME,   // the receiver's time is not FINETIME (its last ReceiverTime)
        PPS_STALE,      // no edge paired within pair_max_age_us
        NO_BIAS,        // no anchor carries to it
        EDGE_OFF,       // the last edge sat off the line
        OFF_CHAIN,      // the last edge sat off the chain: the 1PPS and the PVTs state different GPS times
        HELD,           // the chain is not confirmed by the 1PPS yet
        COUNT
    };

    // What started a hold (the chain's break).
    enum class Break : uint8_t
    {
        NONE,
        FIRST,          // the first anchor
        RESET,          // the receiver's clock lost since the last (MosaicJumpState::resets)
        REBASE,         // the jump count started over at a big jump since the last
        EXPIRED,        // the last too old to carry
        BIAS_STEP,      // its bias more than bias_step_ms off the one carried to it
        CHECK_FAILED,   // a whole second's PPS check failed
        LINE_RESTART,   // the PPS line restarted
        OFF_CHAIN,      // an edge on the line off the chain, or the reference dropped while trusted
        COUNT
    };

    struct Stats
    {
        uint32_t why[static_cast<int>(Why::COUNT)] = {};       // epochs handed over, by Why (countEpoch)
        uint32_t breaks[static_cast<int>(Break::COUNT)] = {};  // holds started, by Break
        uint32_t confirmed = 0;          // holds the 1PPS released
        uint32_t pvt_passed = 0;         // PVTGeodetics that could not anchor (pvtAnchorsJumps)
        uint32_t edges_off = 0;          // edges paired more than on_line_us off the line
        uint32_t edges_off_chain = 0;    // edges paired more than on_line_us off the chain
        uint32_t refs_dropped = 0;       // references dropped as too old
        double   last_bias_step_ms = 0.0;   // the last BIAS_STEP: the PVT's bias less the carried one
        PpsTagStats pps;                 // the PPS check, judged on the seconds handed over timed
    };

    // The last whole second's PPS check, for a bench trace.
    struct LastCheck
    {
        bool     have = false;
        int64_t  rx_ms = 0;              // its receiver time (WNc * 604800000 + TOW)
        int64_t  tag_mcu_us = 0;
        int64_t  edge_us = 0;
        int32_t  residual_us = 0;        // against where the edge was due, if it came late
        bool     published = false;
        bool     on_chain = true;        // its edge on the chain then
    };

    static const char* whyName(Why w)
    {
        switch (w)
        {
            case Why::TIMED: return "timed";
            case Why::NO_LINE: return "no PPS line";
            case Why::NOT_FINETIME: return "not FINETIME";
            case Why::PPS_STALE: return "no edge in 120 s";
            case Why::NO_BIAS: return "no bias to carry";
            case Why::EDGE_OFF: return "edge off the line";
            case Why::OFF_CHAIN: return "edge off the chain";
            case Why::HELD: return "held";
            default: return "?";
        }
    }
    static const char* breakName(Break b)
    {
        switch (b)
        {
            case Break::NONE: return "none";
            case Break::FIRST: return "first anchor";
            case Break::RESET: return "receiver clock lost";
            case Break::REBASE: return "jump count restart";
            case Break::EXPIRED: return "anchor expired";
            case Break::BIAS_STEP: return "bias step";
            case Break::CHECK_FAILED: return "check failed";
            case Break::LINE_RESTART: return "line restart";
            case Break::OFF_CHAIN: return "edge off the chain";
            default: return "?";
        }
    }

    MosaicTimeTag() : MosaicTimeTag(Config{}) {}
    explicit MosaicTimeTag(const Config& c) : cfg_(c) {}

    const Config& config() const { return cfg_; }

    // A replay of a log that begins long after the receiver did (the flight
    // log starts at launch): its first anchor stands for a chain the 1PPS
    // confirmed before the log began, and the line for one its last edge sat
    // on, as the replay's clock line takes its first record for the ones
    // before it. A boot (the driver) never calls this.
    void assumeConfirmedAtStart()
    {
        assume_start_ = true;
        edge_ok_ = true;
    }

    // ReceiverTime's FINETIME, as it last said.
    void noteFineTime(bool fine) { fine_ = fine; }
    bool fineTime() const { return fine_; }

    // Whether an edge of GPS second gps_s, before the line takes it, sits on
    // the chain: moved from the reference by the chain's displacement (no
    // reference: anywhere). The line's permit to step later onto it.
    bool edgeOnChain(int64_t gps_s, int64_t edge_us) const
    {
        if (!ref_have_)
            return true;
        return within(static_cast<double>(edge_us - refAt(gps_s)) - chain_ms_ * 1000.0);
    }

    // A 1PPS edge paired with its GPS second gps_s, after the line took it:
    // line_mcu_us is the line's MCU time of that second, line_valid false if
    // it gave none, line_drift its d(MCU - GPS)/d(GPS) and line_restarts its
    // restarts so far (GnssTimeSync). An edge never takes part in its own
    // place on the line (GnssTimeSync refits on the seconds before it), so
    // this is the line's prediction against the edge.
    void notePair(int64_t gps_s, int64_t edge_us, bool line_valid, int64_t line_mcu_us, double line_drift,
                  uint32_t line_restarts)
    {
        // new seconds, which may be labelled whole seconds off (a block that
        // came a second late pairs the next edge): the 1PPS confirms them anew
        if (restarts_have_ && line_restarts != line_restarts_)
            hold(Break::LINE_RESTART);
        restarts_have_ = true;
        line_restarts_ = line_restarts;

        pair_have_ = true;
        pair_gps_s_ = gps_s;
        pair_edge_us_ = edge_us;
        pair_line_valid_ = line_valid;
        pair_line_mcu_us_ = line_mcu_us;
        pair_line_drift_ = line_drift;
        edge_line_us_ = line_valid ? edge_us - line_mcu_us : 0;
        edge_ok_ = line_valid && within(static_cast<double>(edge_line_us_));
        if (line_valid && !edge_ok_)
            st_.edges_off++;

        if (ref_have_ && edge_us - ref_mcu_us_ > cfg_.ref_max_age_us)
        {
            // too old to say where an edge belongs: the 1PPS alone confirms the next chain
            dropRef();
            st_.refs_dropped++;
            if (!held_)
                hold(Break::OFF_CHAIN);
        }
        edge_ref_us_ = ref_have_ ? edge_us - refAt(gps_s) : 0;
        const bool on_chain = onChain();
        if (!on_chain)
            st_.edges_off_chain++;
        // the line on edges the PVTs do not account for (a 1PPS still on the
        // old sky, a step of the line no PVT made): off it alone, the edge is
        // what moved (a late one) and the line kept what it had
        if (edge_ok_ && !on_chain && !held_)
            hold(Break::OFF_CHAIN);
        if (!held_ && edge_ok_ && on_chain && (!ref_have_ || edge_us - ref_mcu_us_ >= cfg_.ref_refresh_us))
            setRef();
    }

    // The receiver clock bias (ms) of an epoch whose header js has just
    // noted (info from rawEpochFromMeasEpoch or noteMeasEpochClock): the
    // anchor carried by its drift and the jumps since. False without one that
    // carries to it: the epoch is untimed (NO_BIAS).
    bool carry(const MosaicEpochInfo& info, const MosaicJumpState& js, double& dtr_ms) const
    {
        if (!anc_have_ || js.rebases != anc_rebases_)
            return false;
        const int64_t rx_ms = static_cast<int64_t>(info.wnc) * 604800000LL + info.tow_ms;
        const int64_t age = rx_ms - anc_rx_ms_;
        if (age < 0 || age > cfg_.carry_ms)
            return false;
        dtr_ms = dtrMs(anc_bias_ms_, anc_drift_ppm_, static_cast<double>(age) * 1.0e-3, info.j_ms - anc_j_ms_);
        return true;
    }

    // Whether an epoch may be handed over timed now: the line valid, the
    // receiver FINETIME, an edge paired within pair_max_age_us (now_us and
    // the edges on the same clock), carried (carry()), the last edge on the
    // line and on the chain, and the chain confirmed.
    Why judge(bool line_valid, bool carried, int64_t now_us) const
    {
        if (!line_valid)
            return Why::NO_LINE;
        if (!fine_)
            return Why::NOT_FINETIME;
        if (!pair_have_ || now_us - pair_edge_us_ >= cfg_.pair_max_age_us)
            return Why::PPS_STALE;
        if (!carried)
            return Why::NO_BIAS;
        if (!edge_ok_)
            return Why::EDGE_OFF;
        if (!onChain())
            return Why::OFF_CHAIN;
        if (held_)
            return Why::HELD;
        return Why::TIMED;
    }

    void countEpoch(Why why) { st_.why[static_cast<int>(why)]++; }

    // A whole second's epoch placed at tag_mcu_us (handed over timed or
    // not): its PPS check comes with its own PVTGeodetic (notePvt).
    void noteWholeSecond(int64_t rx_ms, int64_t tag_mcu_us, bool published)
    {
        cand_have_ = true;
        cand_rx_ms_ = rx_ms;
        cand_mcu_us_ = tag_mcu_us;
        cand_published_ = published;
        cand_seq_ = seq_;
    }

    // A PVTGeodetic, after its own MeasEpoch was noted into js. First the
    // PPS check of its second's epoch, if that epoch was placed and its edge
    // paired (second_us: the line's MCU length of the second that ended
    // there; line_settled: the line has fitted the crystal's drift), with
    // this PVT's own RxClkBias; then the anchor, if the PVT may be one.
    void notePvt(const PvtGeodetic& p, const MosaicJumpState& js, int64_t second_us, bool line_settled)
    {
        // A solution's clock: RxClkBias is not in has_solution's
        // do-not-use test, and its do-not-use is -2e10
        if (!p.has_solution || !std::isfinite(p.rx_clk_bias_ms) || std::fabs(p.rx_clk_bias_ms) > kMaxBiasMs ||
            !std::isfinite(p.rx_clk_drift_ppm) || std::fabs(p.rx_clk_drift_ppm) > 1000.0f)
            return;
        const int64_t rx_ms = static_cast<int64_t>(p.wnc) * 604800000LL + p.tow_ms;

        if (cand_have_ && cand_rx_ms_ == rx_ms)
        {
            cand_have_ = false;
            if (pair_have_ && pair_gps_s_ * 1000LL == rx_ms)
            {
                const uint32_t late_before = st_.pps.late;
                const int32_t res = notePpsTag(st_.pps, pair_gps_s_, cand_mcu_us_, pair_edge_us_, second_us,
                                               line_settled, p.rx_clk_bias_ms, cfg_.bar_us, cand_published_);
                last_ = LastCheck{true, rx_ms, cand_mcu_us_, pair_edge_us_, res, cand_published_, onChain()};
                // an edge that came late is judged where it was due, which
                // is right for a held-off interrupt and wrong for the first
                // edge of a step: it neither confirms nor breaks the chain
                if (st_.pps.late == late_before)
                    noteCheck(res);
            }
        }

        if (!pvtAnchorsJumps(js, p.wnc, p.tow_ms))
        {
            // its own MeasEpoch was not the last seen: its bias may hold a
            // jump the count has not; the anchor before stands in
            st_.pvt_passed++;
            return;
        }
        Break why = Break::NONE;
        if (!anc_have_)
        {
            why = Break::FIRST;
        }
        else
        {
            // the receiver's clock kept since the last anchor: every jump
            // counted (across a big one's start-over), so the bias carries,
            // and what it does not carry is the sky moving against the clock
            const bool kept = js.resets == anc_resets_;
            if (kept)
            {
                const double carried =
                    dtrMs(anc_bias_ms_, anc_drift_ppm_, static_cast<double>(rx_ms - anc_rx_ms_) * 1.0e-3,
                          static_cast<int32_t>(js.jsum_ms - anc_jsum_ms_));
                const double b = p.rx_clk_bias_ms - carried;
                chain_ms_ += b;
                if (std::fabs(b) > cfg_.bias_step_ms)
                {
                    why = Break::BIAS_STEP;
                    st_.last_bias_step_ms = b;
                }
            }
            // else: lost, its new bias says nothing of the sky; it is taken
            // not to have moved (chain_ms_ kept), and the 1PPS must show it
            if (!kept)
                why = Break::RESET;
            else if (js.rebases != anc_rebases_)
                why = Break::REBASE;
            else if (rx_ms < anc_rx_ms_ || rx_ms - anc_rx_ms_ > cfg_.carry_ms)
                why = Break::EXPIRED;
        }
        anc_have_ = true;
        anc_bias_ms_ = p.rx_clk_bias_ms;
        anc_drift_ppm_ = p.rx_clk_drift_ppm;
        anc_rx_ms_ = rx_ms;
        anc_j_ms_ = js.j_ms;
        anc_rebases_ = js.rebases;
        anc_resets_ = js.resets;
        anc_jsum_ms_ = js.jsum_ms;
        if (why == Break::FIRST)
        {
            // no chain before it to say where its edges belong
            dropRef();
            if (assume_start_)
                return;   // a replay's first: confirmed before the log began
        }
        if (why != Break::NONE)
            hold(why);
    }

    // The state, for the status line and a bench trace.
    const Stats& stats() const { return st_; }
    const LastCheck& lastCheck() const { return last_; }
    bool held() const { return held_; }
    Break holdBreak() const { return held_ ? hold_why_ : Break::NONE; }
    uint8_t passes() const { return passes_; }
    bool edgeOnLine() const { return edge_ok_; }
    int64_t edgeOffLineUs() const { return edge_line_us_; }
    bool anchored() const { return anc_have_; }
    double anchorBiasMs() const { return anc_bias_ms_; }
    int64_t anchorRxMs() const { return anc_rx_ms_; }
    bool pairHave() const { return pair_have_; }
    int64_t pairGpsS() const { return pair_gps_s_; }
    int64_t pairEdgeUs() const { return pair_edge_us_; }
    // the reference, the chain's displacement since it (ms) and the last
    // edge against it (us): on the chain while they agree
    bool refHave() const { return ref_have_; }
    int64_t refMcuUs() const { return ref_mcu_us_; }
    double chainMs() const { return chain_ms_; }
    int64_t edgeRefUs() const { return edge_ref_us_; }
    bool onChain() const { return !ref_have_ || within(static_cast<double>(edge_ref_us_) - chain_ms_ * 1000.0); }

private:
    bool within(double us) const { return std::fabs(us) <= static_cast<double>(cfg_.on_line_us); }

    // the reference run on to GPS second gps_s, MCU us
    int64_t refAt(int64_t gps_s) const
    {
        return ref_mcu_us_ +
               static_cast<int64_t>(std::llround(static_cast<double>(gps_s - ref_gps_s_) * 1.0e6 * (1.0 + ref_drift_)));
    }

    // the last edge's place on the line becomes the reference, the chain
    // agreeing with it from here
    void setRef()
    {
        if (!pair_have_ || !pair_line_valid_)
            return;
        ref_have_ = true;
        ref_gps_s_ = pair_gps_s_;
        ref_mcu_us_ = pair_line_mcu_us_;
        ref_drift_ = pair_line_drift_;
        chain_ms_ = 0.0;
        edge_ref_us_ = pair_edge_us_ - pair_line_mcu_us_;
    }

    void dropRef()
    {
        ref_have_ = false;
        chain_ms_ = 0.0;
        edge_ref_us_ = 0;
    }

    void hold(Break why)
    {
        st_.breaks[static_cast<int>(why)]++;
        held_ = true;
        hold_why_ = why;
        passes_ = 0;
        seq_++;   // a check of a tag placed before this says nothing of the chain after it
    }

    void noteCheck(int32_t res)
    {
        if (cand_seq_ != seq_)
            return;
        const int32_t mag = res == (-2147483647 - 1) ? 2147483647 : (res < 0 ? -res : res);
        if (mag > cfg_.on_line_us)
        {
            // held already: the confirmation starts over, under the break
            // that began the hold
            if (held_)
                passes_ = 0;
            else
                hold(Break::CHECK_FAILED);
            return;
        }
        // against an edge off the chain the check sees the line and the
        // carry, not which GPS time is right: it confirms nothing
        if (!onChain() || !edge_ok_)
        {
            passes_ = 0;
            return;
        }
        if (held_ && ++passes_ >= cfg_.confirm)
        {
            held_ = false;
            st_.confirmed++;
            setRef();
        }
    }

    Config cfg_;
    Stats st_;
    bool fine_ = false;
    bool assume_start_ = false;
    // the anchor
    bool anc_have_ = false;
    double anc_bias_ms_ = 0.0;
    double anc_drift_ppm_ = 0.0;
    int64_t anc_rx_ms_ = 0;
    int32_t anc_j_ms_ = 0;
    uint32_t anc_rebases_ = 0;
    uint32_t anc_resets_ = 0;
    int64_t anc_jsum_ms_ = 0;
    // the chain's trust
    bool held_ = false;
    Break hold_why_ = Break::NONE;
    uint8_t passes_ = 0;
    uint32_t seq_ = 0;
    // the reference and the chain's displacement since it
    bool ref_have_ = false;
    int64_t ref_gps_s_ = 0;
    int64_t ref_mcu_us_ = 0;
    double ref_drift_ = 0.0;
    double chain_ms_ = 0.0;
    int64_t edge_ref_us_ = 0;
    // the last edge paired, and the line then
    bool pair_have_ = false;
    int64_t pair_gps_s_ = 0;
    int64_t pair_edge_us_ = 0;
    bool pair_line_valid_ = false;
    int64_t pair_line_mcu_us_ = 0;
    double pair_line_drift_ = 0.0;
    int64_t edge_line_us_ = 0;
    bool edge_ok_ = false;
    bool restarts_have_ = false;
    uint32_t line_restarts_ = 0;
    // the last whole second's tag, for the check at its PVT
    bool cand_have_ = false;
    int64_t cand_rx_ms_ = 0;
    int64_t cand_mcu_us_ = 0;
    bool cand_published_ = false;
    uint32_t cand_seq_ = 0;
    LastCheck last_;
};

}  // namespace mosaic

#endif  // MOSAIC_TIME_TAG_H
