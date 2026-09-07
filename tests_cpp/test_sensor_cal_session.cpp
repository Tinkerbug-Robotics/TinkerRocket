// Pad-calibration handshake (#1114): the flight task posts a request and
// keeps flying; the IMU poll task measures the window and answers with the
// request's sequence number; a launch inside the window cancels it and the
// answer is dropped.  Both halves are driven here on the host — the semaphore
// that carries the answer between the two tasks is the one part that needs a
// rocket.
#include <gtest/gtest.h>
#include "sensor_cal_session.h"

namespace
{
constexpr uint32_t kWindowUs = 10'000'000;   // SensorCollector::CAL_WINDOW_US
constexpr uint32_t kT0       = 5'000'000;
}

// ---------- SensorCalSession: the flight task's half ----------

TEST(SensorCalSession, StartsIdleAndRefusesEveryAnswer)
{
    SensorCalSession s;
    EXPECT_FALSE(s.waiting());
    EXPECT_FALSE(s.accept(0));
    EXPECT_FALSE(s.accept(1));
    EXPECT_FALSE(s.waiting());
}

TEST(SensorCalSession, StartIssuesANonZeroSeqAndWaitsForIt)
{
    SensorCalSession s;
    const uint32_t seq = s.start();
    EXPECT_NE(seq, 0u);
    EXPECT_TRUE(s.waiting());
    EXPECT_EQ(s.wait_seq, seq);
}

TEST(SensorCalSession, ASecondStartWhileWaitingIsRefusedAndChangesNothing)
{
    SensorCalSession s;
    const uint32_t seq = s.start();
    EXPECT_EQ(s.start(), 0u);
    EXPECT_TRUE(s.waiting());
    EXPECT_EQ(s.wait_seq, seq);       // still the first request
    EXPECT_TRUE(s.accept(seq));       // and its answer still lands
}

TEST(SensorCalSession, AcceptsOnlyTheOutstandingSeqAndOnlyOnce)
{
    SensorCalSession s;
    const uint32_t seq = s.start();
    EXPECT_FALSE(s.accept(seq + 1));  // not ours
    EXPECT_FALSE(s.accept(0));
    EXPECT_TRUE(s.waiting());         // a wrong answer leaves the wait alone
    EXPECT_TRUE(s.accept(seq));
    EXPECT_FALSE(s.waiting());
    EXPECT_FALSE(s.accept(seq));      // the same answer a second time is refused
}

TEST(SensorCalSession, CancelDropsTheOutstandingAnswerForGood)
{
    SensorCalSession s;
    const uint32_t seq = s.start();
    s.cancel();
    EXPECT_FALSE(s.waiting());
    EXPECT_FALSE(s.accept(seq));      // the cancelled window's answer is refused

    const uint32_t seq2 = s.start();  // a fresh request gets a fresh seq
    EXPECT_NE(seq2, 0u);
    EXPECT_NE(seq2, seq);
    EXPECT_FALSE(s.accept(seq));      // still refused, even while waiting again
    EXPECT_TRUE(s.waiting());
    EXPECT_TRUE(s.accept(seq2));
}

TEST(SensorCalSession, EachRequestGetsANewSeq)
{
    SensorCalSession s;
    const uint32_t a = s.start(); EXPECT_TRUE(s.accept(a));
    const uint32_t b = s.start(); EXPECT_TRUE(s.accept(b));
    const uint32_t c = s.start(); EXPECT_TRUE(s.accept(c));
    EXPECT_NE(a, b);
    EXPECT_NE(b, c);
    EXPECT_NE(a, c);
}

TEST(SensorCalSession, SeqSkipsZeroOnWrap)
{
    SensorCalSession s;
    s.last_seq = 0xFFFFFFFFu;
    const uint32_t seq = s.start();
    EXPECT_NE(seq, 0u);               // 0 means "nothing outstanding"
    EXPECT_TRUE(s.waiting());
    EXPECT_TRUE(s.accept(seq));
}

// ---------- SensorCalWindow: the poll task's half ----------

TEST(SensorCalWindow, NoRequestNoWindow)
{
    SensorCalWindow w;
    uint32_t done = 0;
    EXPECT_FALSE(w.open(0, kT0, kWindowUs));
    EXPECT_FALSE(w.active);
    EXPECT_FALSE(w.close(kT0 + 2 * kWindowUs, done));
}

TEST(SensorCalWindow, ARequestOpensOneWindowNotOnePerWake)
{
    SensorCalWindow w;
    EXPECT_TRUE(w.open(1, kT0, kWindowUs));
    EXPECT_TRUE(w.active);
    EXPECT_EQ(w.seq, 1u);
    EXPECT_EQ(w.deadline_us, kT0 + kWindowUs);
    // The request stays posted across every later wake: the window must not
    // reopen (and re-zero the caller's sums) on each of them.
    EXPECT_FALSE(w.open(1, kT0 + 1000, kWindowUs));
    EXPECT_FALSE(w.open(1, kT0 + 2000, kWindowUs));
    EXPECT_EQ(w.deadline_us, kT0 + kWindowUs);
}

TEST(SensorCalWindow, ClosesExactlyOnceAtTheDeadlineWithItsSeq)
{
    SensorCalWindow w;
    uint32_t done = 0;
    ASSERT_TRUE(w.open(7, kT0, kWindowUs));
    EXPECT_FALSE(w.close(kT0, done));
    EXPECT_FALSE(w.close(kT0 + kWindowUs - 1, done));
    EXPECT_TRUE(w.active);
    EXPECT_TRUE(w.close(kT0 + kWindowUs, done));
    EXPECT_EQ(done, 7u);
    EXPECT_FALSE(w.active);
    EXPECT_FALSE(w.close(kT0 + kWindowUs + 1, done));   // never twice
}

TEST(SensorCalWindow, ANewerRequestRestartsAnOpenWindow)
{
    SensorCalWindow w;
    uint32_t done = 0;
    ASSERT_TRUE(w.open(1, kT0, kWindowUs));
    // Cancelled and re-requested 5 s in: the window restarts from now.
    EXPECT_TRUE(w.open(2, kT0 + kWindowUs / 2, kWindowUs));
    EXPECT_EQ(w.seq, 2u);
    EXPECT_FALSE(w.close(kT0 + kWindowUs, done));        // the old deadline no longer counts
    EXPECT_TRUE(w.close(kT0 + kWindowUs + kWindowUs / 2, done));
    EXPECT_EQ(done, 2u);                                  // and only the new seq is ever answered
}

TEST(SensorCalWindow, ARequestAfterACloseOpensAgain)
{
    SensorCalWindow w;
    uint32_t done = 0;
    ASSERT_TRUE(w.open(1, kT0, kWindowUs));
    ASSERT_TRUE(w.close(kT0 + kWindowUs, done));
    EXPECT_FALSE(w.open(1, kT0 + kWindowUs + 1, kWindowUs));  // same request: already answered
    EXPECT_TRUE(w.open(2, kT0 + kWindowUs + 1, kWindowUs));
    EXPECT_TRUE(w.close(kT0 + 2 * kWindowUs + 1, done));
    EXPECT_EQ(done, 2u);
}

TEST(SensorCalWindow, DeadlineSurvivesTheMicrosecondTimerWrap)
{
    SensorCalWindow w;
    uint32_t done = 0;
    const uint32_t t_open = 0xFFFFFFFFu - 1'000'000u;    // 1 s before the wrap
    ASSERT_TRUE(w.open(3, t_open, kWindowUs));
    EXPECT_FALSE(w.close(0xFFFFFFFFu, done));            // just before the wrap
    EXPECT_FALSE(w.close(1'000'000u, done));             // 2 s in, after the wrap
    EXPECT_TRUE(w.close(9'000'000u, done));              // 10 s in
    EXPECT_EQ(done, 3u);
}

// ---------- both halves together ----------

TEST(SensorCalHandshake, ARequestIsAnsweredOnceAndAccepted)
{
    SensorCalSession s;
    SensorCalWindow  w;
    uint32_t done = 0;
    const uint32_t seq = s.start();
    ASSERT_TRUE(w.open(seq, kT0, kWindowUs));
    // ~10 s of poll wakes with nothing to report and the flight task polling.
    for (uint32_t t = kT0; t < kT0 + kWindowUs; t += 100'000)
    {
        EXPECT_FALSE(w.open(seq, t, kWindowUs));
        EXPECT_FALSE(w.close(t, done));
        EXPECT_TRUE(s.waiting());
    }
    ASSERT_TRUE(w.close(kT0 + kWindowUs, done));
    EXPECT_TRUE(s.accept(done));
    EXPECT_FALSE(s.waiting());
}

TEST(SensorCalHandshake, ALaunchInsideTheWindowCancelsItAndItsAnswerIsDropped)
{
    SensorCalSession s;
    SensorCalWindow  w;
    uint32_t done = 0;
    const uint32_t seq = s.start();
    ASSERT_TRUE(w.open(seq, kT0, kWindowUs));
    s.cancel();                                            // motor lit at +3 s
    // The poll task neither knows nor cares: its window runs out on its own.
    ASSERT_TRUE(w.close(kT0 + kWindowUs, done));
    EXPECT_EQ(done, seq);
    EXPECT_FALSE(s.accept(done));                          // nobody is waiting for it
    EXPECT_FALSE(s.waiting());
}

TEST(SensorCalHandshake, ARequestAfterACancelIsAnsweredByItsOwnWindowOnly)
{
    SensorCalSession s;
    SensorCalWindow  w;
    uint32_t done = 0;
    const uint32_t seq1 = s.start();
    ASSERT_TRUE(w.open(seq1, kT0, kWindowUs));
    s.cancel();
    const uint32_t seq2 = s.start();                       // re-tapped 2 s later
    ASSERT_TRUE(w.open(seq2, kT0 + 2'000'000, kWindowUs)); // the poll task restarts the window
    EXPECT_FALSE(w.close(kT0 + kWindowUs, done));          // seq1's deadline means nothing now
    ASSERT_TRUE(w.close(kT0 + 2'000'000 + kWindowUs, done));
    EXPECT_EQ(done, seq2);
    EXPECT_TRUE(s.accept(done));
}

TEST(SensorCalHandshake, ALateAnswerFromACancelledWindowDoesNotEndTheNewOne)
{
    // The cancelled window closed before the poll task saw the new request:
    // its answer is sitting there when the flight task polls again.
    SensorCalSession s;
    SensorCalWindow  w;
    uint32_t done = 0;
    const uint32_t seq1 = s.start();
    ASSERT_TRUE(w.open(seq1, kT0, kWindowUs));
    s.cancel();
    ASSERT_TRUE(w.close(kT0 + kWindowUs, done));           // seq1 answered, unwanted
    const uint32_t seq2 = s.start();
    EXPECT_FALSE(s.accept(done));                          // the late seq1 answer is refused...
    EXPECT_TRUE(s.waiting());                              // ...and the new request stays open
    ASSERT_TRUE(w.open(seq2, kT0 + kWindowUs + 1, kWindowUs));
    ASSERT_TRUE(w.close(kT0 + 2 * kWindowUs + 1, done));
    EXPECT_EQ(done, seq2);
    EXPECT_TRUE(s.accept(done));
}

// ---------- the refusal rule ----------

TEST(SensorCalRefusal, EveryPadStateIsAllowed)
{
    // Outdoors the FC auto-promotes to PRELAUNCH within seconds of GNSS lock,
    // so this is the state an on-pad calibration actually runs in.
    EXPECT_EQ(sensor_cal::refusal(PRELAUNCH,       false), nullptr);
    EXPECT_EQ(sensor_cal::refusal(READY,           false), nullptr);
    EXPECT_EQ(sensor_cal::refusal(INITIALIZATION,  false), nullptr);
    EXPECT_EQ(sensor_cal::refusal(MAG_CALIBRATION, false), nullptr);
}

TEST(SensorCalRefusal, InflightAndLandedAreRefused)
{
    EXPECT_STREQ(sensor_cal::refusal(INFLIGHT, false), "INFLIGHT");
    EXPECT_STREQ(sensor_cal::refusal(LANDED,   false), "LANDED");
}

TEST(SensorCalRefusal, TheDetectorsOwnVerdictWinsBeforeTheStateMachineMoves)
{
    // launch_flag latches a pass before the state machine enters INFLIGHT;
    // the cancel must not wait for it.
    EXPECT_STREQ(sensor_cal::refusal(PRELAUNCH, true), "launch detected");
    EXPECT_STREQ(sensor_cal::refusal(READY,     true), "launch detected");
    EXPECT_STREQ(sensor_cal::refusal(INFLIGHT,  true), "launch detected");
}
