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

// The link is fixed: 44100 samples/s x 4 B.
constexpr double kLinkBytesPerSec = (double)config::I2S_SAMPLE_RATE * 4.0;

// Steady-state inflow during a GUIDED coast at the shipped IMU_RATE_DYNAMIC
// boost rate — the worst case, and the phase guidance telemetry exists for.
double guidedCoastInflow()
{
    return framed(sizeof(ISM6HG256Data),   (double)IMU_RATE_DYNAMIC_BOOST_HZ)
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
    const double imu = framed(sizeof(ISM6HG256Data),
                              (double)IMU_RATE_DYNAMIC_BOOST_HZ);
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
