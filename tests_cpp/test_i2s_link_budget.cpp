// #1137 item 13 — the FC->OC I2S link budget, computed rather than commented.
//
// The budget lived only in a comment in config.h, and it went stale twice
// without anyone noticing: it still budgeted NonSensorData at 24 B after #529
// and #1190 had grown it to 52, and it predated GuidanceTelemData entirely.
// By the time the roundup looked, a guided coast at the shipped IMU boost rate
// was running at 98.6% of a fixed 176,400 B/s link while the comment claimed
// 88%.  Frames past the limit are dropped silently by enqueueI2STx, so the
// symptom would have been a guided coast whose telemetry thins out with
// nothing logged anywhere to say why.
//
// A comment cannot fail. This can: it derives the same number from the actual
// sizeof()s and the actual configured rates, so growing a struct on the link
// breaks the build's tests rather than the flight's data.

#include <gtest/gtest.h>

#include "RocketComputerTypes.h"
#include "config.h"          // flight_computer/main/config.h

namespace {

// Framing overhead per message: MAX_FRAME = 4 (sync) + 1 (type) + 1 (len)
//                                          + payload + 2 (crc)
constexpr size_t kFrameOverhead = 8;

constexpr double framed(size_t payload, double rate_hz)
{
    return (double)(payload + kFrameOverhead) * rate_hz;
}

// #1485: the IMU goes as ISM6_BATCH_MSG, up to ISM6_BATCH_MAX records a frame.
// The FC flushes a partial batch at the end of every loop pass, so a batch is
// not always full; the budget allows for them being half full on average.
double imuBatched(double rate_hz)
{
    const double frames = rate_hz / ((double)ISM6_BATCH_MAX / 2.0);
    return rate_hz * (double)sizeof(ISM6HG256Data)       // the records
         + frames * (double)(1 + kFrameOverhead);          // count byte + framing, per frame
}

// GNSS_RTCM_MSG: the LC86G's raw MSM7, one epoch a second as five messages
// (GPS, GLONASS, Galileo, BeiDou, QZSS). An open sky measured 441 B/s of
// frames (Beetle, 2026-09-25, up to 29 satellites); budgeted at 40 satellites,
// 15 B each (one L1 signal: 36 bits per satellite and 80 per cell, plus a mask
// bit of each), 22 B of message header and 6 of RTCM3 framing per message, and
// the record header and I2S framing per frame. Plus the broadcast ephemeris
// ($PAIR436,1): measured at exactly one message a second (Beetle,
// 2026-10-12), budgeted at the largest, BeiDou's 1042 (64 B + 6 of framing).
// Zero on u-blox boards, which emit none.
double gnssRtcmInflow()
{
#if defined(TR_GNSS_DRIVER_LC86) && TR_GNSS_DRIVER_LC86
    const double msgs_per_sec = 5.0 + 1.0;
    const double frame_bytes  = 40.0 * 15.0 + 5.0 * (22.0 + 6.0) + 1.0 * (64.0 + 6.0);
    return frame_bytes + msgs_per_sec * (double)(sizeof(GnssRtcmHeader) + kFrameOverhead);
#else
    return 0.0;
#endif
}

// RAW_GNSS_EKF_MSG: the raw-GNSS shadow filter's state (#1600), 10 a second
// where it runs (config.h TR_RAW_GNSS_SHADOW: the LC86G boards), and its
// check of each receiver fix against the IMU (GNSS_CHECK_MSG, #1508), one per
// fix at the receiver's 10 Hz, and the attitude 1-sigma and IMU biases beside
// each state record (RAW_GNSS_ATT_MSG).
double rawGnssShadowInflow()
{
#if defined(TR_GNSS_DRIVER_LC86) && TR_GNSS_DRIVER_LC86
    return framed(sizeof(RawGnssEkfData), 10.0) + framed(sizeof(RawGnssAttData), 10.0)
         + framed(sizeof(GnssCheckData), 10.0);
#else
    return 0.0;
#endif
}

// The link: I2S_SAMPLE_RATE samples/s x 4 B (16-bit stereo).
constexpr double kLinkBytesPerSec = (double)config::I2S_SAMPLE_RATE * 4.0;

// Steady-state inflow during a GUIDED coast at the fastest IMU rate this board
// flies (IMU_RATE_MAX_HZ: a fixed rate or a dynamic mode's boost) — the worst
// case, and the phase guidance telemetry exists for. Built once per board.
double guidedCoastInflow()
{
    return imuBatched((double)config::IMU_RATE_MAX_HZ)
         + framed(sizeof(NonSensorData),   (double)config::NON_SENSOR_UPDATE_RATE)
         + framed(sizeof(BMP585Data),      (double)config::BMP585_UPDATE_RATE)
         + framed(sizeof(GuidanceTelemData), (double)config::GUIDANCE_TELEM_RATE_HZ)
         + framed(sizeof(FlightSnapshotData), 10.0)
         + framed(sizeof(IIS2MDCData),     100.0)
         + framed(sizeof(GNSSData),        (double)config::GNSS_UPDATE_RATE)
         // #1154 item 4: the FC's camera truth. Counted here because that is
         // the whole reason it is a 5 Hz message and not a byte in the 500 Hz
         // NonSensorData — the byte version measured 95.06% and tripped the
         // line below. A stream the budget does not know about is how this
         // block went stale twice before.
         + framed(sizeof(FcStatusData),    (double)config::FC_STATUS_RATE_HZ)
         + gnssRtcmInflow()
         + rawGnssShadowInflow()
         + 220.0;   // POWERData, ~15 Hz
}

}  // namespace

TEST(I2SLinkBudget, GuidedCoastLeavesRealHeadroom) {
    const double inflow = guidedCoastInflow();
    const double used   = inflow / kLinkBytesPerSec;

    // 95% is the line. Below it there is room for jitter and for the burst
    // traffic this steady-state sum ignores (config readback, OTA chunks, the
    // snapshot cadence changing); above it, enqueueI2STx starts dropping and
    // says nothing.  At GUIDANCE_TELEM_RATE_HZ 500 this was 0.986.
    EXPECT_LT(used, 0.95)
        << "guided-coast inflow " << inflow << " B/s is " << (used * 100.0)
        << "% of the " << kLinkBytesPerSec << " B/s link. A struct on the link "
        << "grew, or a rate went up — re-derive the budget block in config.h "
        << "and bring this back under the line.";
}

TEST(I2SLinkBudget, TheDominantStreamIsTheIMU) {
    // Sanity on the shape of the budget: if anything ever outgrows the IMU
    // stream, the tuning advice in config.h (and the bench gauges it names)
    // is pointing at the wrong thing.
    const double imu = imuBatched((double)config::IMU_RATE_MAX_HZ);
    EXPECT_GT(imu, guidedCoastInflow() * 0.5);
}

TEST(I2SLinkBudget, NonGuidedFlightHasMoreRoomStill) {
    // GuidanceTelem is only emitted while guidance is active, so an unguided
    // flight is strictly cheaper.  Stated so the guided case is understood as
    // the binding one rather than the only one.
    const double guided   = guidedCoastInflow();
    const double unguided = guided - framed(sizeof(GuidanceTelemData),
                                            (double)config::GUIDANCE_TELEM_RATE_HZ);
    EXPECT_LT(unguided, guided);
    EXPECT_LT(unguided / kLinkBytesPerSec, 0.95);
}

TEST(I2SLinkBudget, GuidanceRateDividesTheNonSensorRate) {
    // The emit is a modulo counter inside the NonSensor TX block, so a rate
    // that does not divide it silently emits at the wrong cadence rather than
    // failing.  config.h static_asserts this too; pinned here as well because
    // the consequence is a wrong number in the log, not a build error.
    EXPECT_EQ(config::NON_SENSOR_UPDATE_RATE % config::GUIDANCE_TELEM_RATE_HZ, 0u);
    EXPECT_LE(config::GUIDANCE_TELEM_RATE_HZ, config::NON_SENSOR_UPDATE_RATE);
}

TEST(I2SLinkBudget, TheFcRunsTheSharedLinkRate) {
    // The OC takes its slave rate from I2S_LINK_SAMPLE_RATE_HZ directly; the
    // FC spells the number out (its config.h is included without the shared
    // header). main.cpp static_asserts the pair; pinned here too, per board.
    EXPECT_EQ(config::I2S_SAMPLE_RATE, I2S_LINK_SAMPLE_RATE_HZ);
}
