// The GNSS antenna offset's readback source on the OC (antenna_readback_policy.h):
// the FC's report ("fc") or the OC's cache ("oc").
//
// The load-bearing rows are the two that serve the cache although the FC is
// up with a v4 report. The apps adopt only "fc" into the rocket's profile, and
// a later Send all writes the profile back, so the FC's default Unknown while
// the self-heal is sending it the measured cache, or the FC's old value while
// a new cmd 81 waits in the queue, would erase the owner's setting.

#include <gtest/gtest.h>

#include "antenna_readback_policy.h"

using AntennaReadbackPolicy::Inputs;
using AntennaReadbackPolicy::reportSettles;
using AntennaReadbackPolicy::serveFc;

namespace {
// An FC that is up, with a v4 report of a setting it remembers, nothing
// staged, and a measured cache: the plain case, served from the FC.
Inputs fcUp()
{
    Inputs in;
    in.rail_on            = true;
    in.report_valid       = true;
    in.report_has_antenna = true;
    in.fc_remembers       = true;
    in.staged_unconfirmed = false;
    in.cache_known        = true;
    return in;
}
}  // namespace

TEST(AntennaReadbackPolicy, FcThatRemembersIsServed) {
    Inputs in = fcUp();
    EXPECT_TRUE(serveFc(in));
    in.cache_known = false;   // its record stands whatever the cache says
    EXPECT_TRUE(serveFc(in));
}

TEST(AntennaReadbackPolicy, FcWithNoRecordAndMeasuredCacheServesTheCache) {
    // Finding a: the FC reports its default Unknown exactly when the heal is
    // about to send it the cache. Serving it as "fc" let the apps adopt
    // Unknown and a later Send all erase the measurement.
    Inputs in = fcUp();
    in.fc_remembers = false;
    EXPECT_FALSE(serveFc(in));
}

TEST(AntennaReadbackPolicy, FcWithNoRecordAndUnknownCacheIsServed) {
    // Both Unknown: nothing will be sent, and the FC's is the same answer.
    Inputs in = fcUp();
    in.fc_remembers = false;
    in.cache_known  = false;
    EXPECT_TRUE(serveFc(in));
}

TEST(AntennaReadbackPolicy, UnconfirmedCommandServesTheCache) {
    // Finding b: a cmd 81 queued with the rail off, or before the FC's first
    // report. That report holds the FC's old value; the cache holds the new.
    for (int remembers = 0; remembers < 2; ++remembers)
        for (int known = 0; known < 2; ++known) {
            Inputs in = fcUp();
            in.fc_remembers       = remembers != 0;
            in.cache_known        = known != 0;
            in.staged_unconfirmed = true;
            EXPECT_FALSE(serveFc(in)) << "remembers " << remembers << " cache known " << known;
        }
}

TEST(AntennaReadbackPolicy, NoReportServesTheCache) {
    Inputs in = fcUp();
    in.report_valid       = false;   // none since the OC booted
    in.report_has_antenna = false;
    EXPECT_FALSE(serveFc(in));
}

TEST(AntennaReadbackPolicy, OlderReportServesTheCache) {
    // A v1-v3 report: its zeroed antenna would read as Unknown, a real setting.
    Inputs in = fcUp();
    in.report_has_antenna = false;
    EXPECT_FALSE(serveFc(in));
    in.cache_known = false;
    EXPECT_FALSE(serveFc(in));
}

TEST(AntennaReadbackPolicy, RailOffServesTheCache) {
    // The report is kept across a rail cycle; with the rail off it is not the
    // rocket's word on anything.
    Inputs in = fcUp();
    in.rail_on = false;
    EXPECT_FALSE(serveFc(in));
    in.fc_remembers = false;
    in.cache_known  = false;
    EXPECT_FALSE(serveFc(in));
}

TEST(AntennaReadbackPolicy, EveryRowServesFcOnlyWhenAllHold) {
    // The whole table: "fc" exactly when the FC is up with a v4 report,
    // nothing staged is unconfirmed, and the FC remembers or the cache is
    // Unknown.
    for (unsigned bits = 0; bits < 64U; ++bits) {
        Inputs in;
        in.rail_on            = (bits & 1U) != 0;
        in.report_valid       = (bits & 2U) != 0;
        in.report_has_antenna = (bits & 4U) != 0;
        in.fc_remembers       = (bits & 8U) != 0;
        in.staged_unconfirmed = (bits & 16U) != 0;
        in.cache_known        = (bits & 32U) != 0;
        const bool want = in.rail_on && in.report_valid && in.report_has_antenna &&
                          !in.staged_unconfirmed && (in.fc_remembers || !in.cache_known);
        EXPECT_EQ(serveFc(in), want) << "bits " << bits;
    }
}

TEST(AntennaReadbackPolicy, ReportHoldingTheStagedValueSettles) {
    // Confirmed, whether or not the command is still being served.
    EXPECT_TRUE(reportSettles(true, false));
    EXPECT_TRUE(reportSettles(true, true));
}

TEST(AntennaReadbackPolicy, ReportWithTheCommandStillQueuedDoesNotSettle) {
    // The FC's first report after power-on comes before it takes the queued
    // command: still the old value, and not the last word.
    EXPECT_FALSE(reportSettles(false, false));
}

TEST(AntennaReadbackPolicy, ReportAfterTheCommandLeftSettles) {
    // Served out (or dropped) and the FC still reports something else: it
    // refused it or never got it, and its own value is shown again.
    EXPECT_TRUE(reportSettles(false, true));
}
