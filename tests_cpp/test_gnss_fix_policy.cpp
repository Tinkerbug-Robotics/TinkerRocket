// #1763: a receiver that loses its fix but keeps sending records must drop out
// of the apogee vote, as a silent one does (flight_computer/main/gnss_fix_policy.h).
//
// The vote is the real TR_KinematicChecks, by sensor, above the barometer's
// range: the barometer is out, the gyro's pitch may not vote, GNSS is admitted.
// The receiver sends a record every 55 ms (the SAM-M10Q's 18 Hz); the checks
// run every 2 ms.  main.cpp's read is modelled by one line: new_gps is
// gnss_fix::usable() of the record (the fix), or true for every record (the
// wiring before it).
#include <gtest/gtest.h>

#include "TR_KinematicChecks.h"
#include "gnss_fix_policy.h"

namespace
{

constexpr uint8_t kMinSats = 4;   // config::GNSS_MIN_SATS

TEST(GnssFix, UsableIsA3DFixWithEnoughSatellites)
{
    EXPECT_TRUE(gnss_fix::usable(3, 4, kMinSats));
    EXPECT_TRUE(gnss_fix::usable(3, 25, kMinSats));
    EXPECT_TRUE(gnss_fix::usable(4, 12, kMinSats));    // u-blox GNSS + dead reckoning
    EXPECT_FALSE(gnss_fix::usable(3, 3, kMinSats));    // too few satellites
    EXPECT_FALSE(gnss_fix::usable(2, 12, kMinSats));   // a 2-D fix (mosaic-G5, PX1105R)
    EXPECT_FALSE(gnss_fix::usable(1, 12, kMinSats));   // PX1105R prediction, u-blox DR alone
    EXPECT_FALSE(gnss_fix::usable(0, 0, kMinSats));    // no fix
    EXPECT_FALSE(gnss_fix::usable(0, 7, kMinSats));    // no fix, satellites still counted
}

// The PX100's records on the 65mm Roll Control flight (BARC 2026-11-21), one
// per stretch: the pad, the burn's prediction, the no-fix coast (+1.65 to
// +8.95 s, 0 to 7 satellites), the fall.
TEST(GnssFix, ThePx100FlightsRecordsCountOnlyWithTheirFix)
{
    EXPECT_TRUE(gnss_fix::usable(3, 24, kMinSats));
    EXPECT_FALSE(gnss_fix::usable(1, 12, kMinSats));
    EXPECT_FALSE(gnss_fix::usable(0, 7, kMinSats));
    EXPECT_TRUE(gnss_fix::usable(3, 6, kMinSats));
}

enum class Receiver
{
    FixKept,        // a usable fix all the way, reading the true velocity
    NoFixRecords,   // loses the fix, keeps sending: fix_mode 0, velocity 0
    NoFixHeld,      // the same, with the last fix's velocity held (a receiver's own freeze)
    FewSatellites,  // keeps a 3-D fix with 3 satellites, reading the true velocity
    Silent,         // loses the fix and sends nothing more
    Regained,       // no-fix records, then the fix back 0.5 s after apogee, reading the descent
    Flickering,     // no-fix records, with a fix (reading the true velocity) on every 10th
};

enum class Wiring
{
    UsableFix,      // the fix: new_gps = gnss_fix::usable(record)
    EveryRecord,    // before it: new_gps on every record
};

struct Outcome
{
    bool called = false;
    bool single = false;
    int32_t after_apogee_ms = 0;   // the call's time after the true apogee
};

class GnssFixVote : public ::testing::Test
{
protected:
    TR_KinematicChecks kc;

    void SetUp() override
    {
        setMockMillis(0);
        kc.reset();
    }

    void launch()
    {
        for (int i = 0; i < 80; i++)
        {
            setMockMillis(i * 2);
            float pos[3] = {0, 0, 0.5f * i};
            float vel[3] = {0, 0, 10.0f};
            kc.kinematicChecks(0.5f * i, 25.0f, pos, vel, 0.0f, true, 0.5f * i, false, 1.57f, false, false,
                               0.0f, true, true, true, true);
        }
        ASSERT_TRUE(kc.launch_flag);
    }

    // One pass of the checks above the barometer's range: the accelerometer
    // reads vz (+ up), the gyro may not vote, the barometer is out.
    void pass(uint32_t now_ms, float vz, bool new_gps, float gps_vel_u)
    {
        setMockMillis(now_ms);
        float pos[3] = {0, 0, 12000.0f};
        float vel[3] = {0, 0, vz};
        TR_KinematicChecks::InertialVote iv;
        iv.in_use = true;
        iv.healthy = true;
        iv.vz_up_mps = vz;
        iv.height_m = 11000.0f;
        iv.pitch_rad = 0.0f;
        iv.pitch_may_vote = false;   // above the barometer's range
        kc.kinematicChecks(12000.0f, 2.0f, pos, vel, 0.0f, true, 12000.0f, new_gps, 0.0f,
                           /*burnout*/ true, /*baro_lockout*/ false, gps_vel_u, /*ekf_healthy*/ true,
                           /*baro_healthy*/ false, /*imu_healthy*/ true, /*gnss_may_vote*/ true,
                           /*baro_reading_fresh*/ true, &iv);
    }

    // A coast through apogee: up at 40 m/s from 200 ms, falling at g.  The
    // receiver loses its fix loss_before_apogee_ms before apogee (never
    // regaining it within the run).  Runs 30 s past apogee or until the call.
    Outcome coast(Receiver rx, Wiring wiring, uint32_t loss_before_apogee_ms = 2000)
    {
        launch();
        const uint32_t t0 = 200;
        const float v0 = 40.0f, g = 9.81f;
        const uint32_t apogee_ms = t0 + (uint32_t)(v0 / g * 1000.0f);
        const uint32_t loss_ms = apogee_ms - loss_before_apogee_ms;
        float held_vel_u = 0.0f;
        Outcome out;
        uint32_t next_record_ms = t0;
        uint32_t lost_records = 0;
        for (uint32_t now = t0; now < apogee_ms + 30000; now += 2)
        {
            const float vz = v0 - g * (float)(now - t0) * 1e-3f;
            bool new_gps = false;
            float gps_vel_u = 0.0f;
            if (now >= next_record_ms)
            {
                next_record_ms += 55;
                bool lost = now >= loss_ms && rx != Receiver::FixKept && rx != Receiver::FewSatellites;
                if (lost && rx == Receiver::Regained && now >= apogee_ms + 500) lost = false;
                if (lost && rx == Receiver::Flickering && ++lost_records % 10 == 0) lost = false;
                if (!(lost && rx == Receiver::Silent))
                {
                    uint8_t fix_mode = 3, sats = 12;
                    float vel_u = vz;
                    if (rx == Receiver::FewSatellites) sats = 3;
                    if (lost)
                    {
                        fix_mode = 0;
                        sats = 0;
                        vel_u = (rx == Receiver::NoFixHeld) ? held_vel_u : 0.0f;
                    }
                    else
                    {
                        held_vel_u = vel_u;
                    }
                    new_gps = (wiring == Wiring::UsableFix) ? gnss_fix::usable(fix_mode, sats, kMinSats) : true;
                    gps_vel_u = vel_u;
                }
            }
            pass(now, vz, new_gps, gps_vel_u);
            if (kc.apogee_flag)
            {
                out.called = true;
                out.single = kc.apogee_single_sensor_flag;
                out.after_apogee_ms = (int32_t)now - (int32_t)apogee_ms;
                break;
            }
        }
        return out;
    }
};

// The issue's case: the fix lost 2 s before apogee, the records keep coming.
// The accelerometer, left alone, calls apogee -- on the same pass as with a
// receiver that went silent at the same moment.
TEST_F(GnssFixVote, NoFixRecordsLeaveTheAccelerometerToCallAloneAsASilentReceiverDoes)
{
    const Outcome silent = coast(Receiver::Silent, Wiring::UsableFix);
    SetUp();
    const Outcome records = coast(Receiver::NoFixRecords, Wiring::UsableFix);
    ASSERT_TRUE(silent.called);
    ASSERT_TRUE(records.called);
    EXPECT_TRUE(records.single);
    EXPECT_EQ(records.after_apogee_ms, silent.after_apogee_ms);
    // 15 m/s down (SINGLE_SENSOR_DESCENT_MPS) plus its count: about 1.5 s
    EXPECT_GT(records.after_apogee_ms, 1400);
    EXPECT_LT(records.after_apogee_ms, 1700);
}

TEST_F(GnssFixVote, AHeldVelocityIsKeptOutToo)
{
    const Outcome o = coast(Receiver::NoFixHeld, Wiring::UsableFix);
    ASSERT_TRUE(o.called);
    EXPECT_TRUE(o.single);
    EXPECT_LT(o.after_apogee_ms, 1700);
}

// The control: the same records counted as GPS (the wiring before the fix)
// hold apogee off for the whole run.
TEST_F(GnssFixVote, CountedAsGpsTheSameRecordsHoldApogeeOff)
{
    EXPECT_FALSE(coast(Receiver::NoFixRecords, Wiring::EveryRecord).called);
    SetUp();
    EXPECT_FALSE(coast(Receiver::NoFixHeld, Wiring::EveryRecord).called);
}

// A fix lost just before apogee: the voter still drops out 500 ms after the
// last fix, so the lone call comes when the accelerometer is ready.
TEST_F(GnssFixVote, AFixLostJustBeforeApogeeStillLeavesTheLoneCall)
{
    const Outcome o = coast(Receiver::NoFixRecords, Wiring::UsableFix, 200);
    ASSERT_TRUE(o.called);
    EXPECT_TRUE(o.single);
    EXPECT_LT(o.after_apogee_ms, 1700);
}

// A fix kept: GNSS and the accelerometer agree, so the quorum calls it, well
// before the lone call would.  Nothing changes for a receiver with its fix.
TEST_F(GnssFixVote, AFixKeptCallsByTheQuorum)
{
    const Outcome kept = coast(Receiver::FixKept, Wiring::UsableFix);
    SetUp();
    const Outcome before = coast(Receiver::FixKept, Wiring::EveryRecord);
    ASSERT_TRUE(kept.called);
    EXPECT_FALSE(kept.single);
    EXPECT_LT(kept.after_apogee_ms, 1000);
    EXPECT_EQ(kept.after_apogee_ms, before.after_apogee_ms);
}

// The fix back before the accelerometer's lone call, reading the descent:
// GNSS joins the vote again and the quorum calls it, as before the fix.
TEST_F(GnssFixVote, AFixRegainedBeforeTheLoneCallJoinsTheQuorum)
{
    const Outcome now_ = coast(Receiver::Regained, Wiring::UsableFix);
    SetUp();
    const Outcome before = coast(Receiver::Regained, Wiring::EveryRecord);
    ASSERT_TRUE(now_.called);
    EXPECT_FALSE(now_.single);
    EXPECT_GT(now_.after_apogee_ms, 500);
    EXPECT_LT(now_.after_apogee_ms, 1000);
    EXPECT_EQ(now_.after_apogee_ms, before.after_apogee_ms);
}

// A fix that comes and goes (one record in ten): the no-fix records between
// no longer hold apogee off.  Counted, they did, for the whole run.
TEST_F(GnssFixVote, AFlickeringFixNoLongerHoldsApogeeOff)
{
    const Outcome o = coast(Receiver::Flickering, Wiring::UsableFix);
    ASSERT_TRUE(o.called);
    EXPECT_LT(o.after_apogee_ms, 2000);
    SetUp();
    EXPECT_FALSE(coast(Receiver::Flickering, Wiring::EveryRecord).called);
}

// A 3-D fix on 3 satellites is not usable (the ascent gate's bar): it does
// not vote even when it reads the descent, so the accelerometer calls alone.
TEST_F(GnssFixVote, ThreeSatellitesDoNotVote)
{
    const Outcome o = coast(Receiver::FewSatellites, Wiring::UsableFix);
    ASSERT_TRUE(o.called);
    EXPECT_TRUE(o.single);
}

}  // namespace
