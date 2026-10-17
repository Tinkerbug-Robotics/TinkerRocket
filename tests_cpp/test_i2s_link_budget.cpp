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

#include <algorithm>
#include <cmath>
#include <vector>

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
// The same record on the V9 with a mosaic-G5 carries its SBF blocks (TR_GNSS_DRIVER_MOSAIC):
// MeasEpoch, MeasExtra and PVTGeodetic at 20 Hz, status at 1 Hz. The
// 2026-10-15 spin-rig capture (33 satellites, ~99 signals) measured MeasEpoch
// 1,495 B, MeasExtra 1,631 B and PVTGeodetic 96 B; budgeted here at 45
// satellites and 130 signals: MeasEpoch 20 B + 20 a satellite + 12 a further
// signal, MeasExtra 20 B + 16 a signal, each block split into 219-byte parts
// with the record header and I2S framing on each. The status blocks measured
// 1.4 kB/s; budgeted at 2.2. The navigation data is a few blocks a minute
// (and one ~7 kB burst after launch, under a second of the headroom).
double gnssSbfBlock(double bytes)
{
    const double parts = std::ceil(bytes / (double)GNSS_RTCM_MAX_CHUNK);
    return bytes + parts * (double)(sizeof(GnssRtcmHeader) + kFrameOverhead);
}

// What one flight of this image streams, by the receiver that flies it: most
// images have one; the V9 image (TR_GNSS_DRIVER_AUTO) flies whichever is on
// J1, the u-blox at the board's IMU limit or the mosaic-G5 at the limit it
// sets (imuRateMaxForGnss), so the budget has to hold for both.
struct Flight
{
    const char* receiver;
    uint8_t     gnss;        // GNSS_TYPE_*
    double      imu_hz;      // the fastest IMU rate it flies
    double      gnss_hz;     // GNSSData's rate
};

std::vector<Flight> flights()
{
    const double board = (double)config::IMU_RATE_MAX_HZ;
#if defined(TR_GNSS_DRIVER_LC86) && TR_GNSS_DRIVER_LC86
    return {{"LC86G", GNSS_TYPE_LC86G, board, (double)config::GNSS_UPDATE_RATE}};
#elif defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
    return {{"mosaic-G5", GNSS_TYPE_MOSAIC,
             (double)imuRateMaxForGnss(config::IMU_RATE_MAX_HZ, GNSS_TYPE_MOSAIC),
             (double)config::GNSS_UPDATE_RATE}};
#elif defined(TR_GNSS_DRIVER_AUTO) && TR_GNSS_DRIVER_AUTO
    return {{"u-blox", GNSS_TYPE_UBLOX, board, (double)config::GNSS_UPDATE_RATE},
            {"mosaic-G5", GNSS_TYPE_MOSAIC,
             (double)imuRateMaxForGnss(config::IMU_RATE_MAX_HZ, GNSS_TYPE_MOSAIC),
             (double)config::GNSS_MOSAIC_UPDATE_RATE}};
#else
    return {{"u-blox", GNSS_TYPE_UBLOX, board, (double)config::GNSS_UPDATE_RATE}};
#endif
}

double gnssRtcmInflow(const Flight& f)
{
    if (f.gnss == GNSS_TYPE_LC86G)
    {
        const double msgs_per_sec = 5.0 + 1.0;
        const double frame_bytes  = 40.0 * 15.0 + 5.0 * (22.0 + 6.0) + 1.0 * (64.0 + 6.0);
        return frame_bytes + msgs_per_sec * (double)(sizeof(GnssRtcmHeader) + kFrameOverhead);
    }
    if (f.gnss == GNSS_TYPE_MOSAIC)
    {
        const double sats = 45.0, signals = 130.0;
        const double meas_epoch = 20.0 + 20.0 * sats + 12.0 * (signals - sats);
        const double meas_extra = 20.0 + 16.0 * signals;
        const double per_epoch = gnssSbfBlock(meas_epoch) + gnssSbfBlock(meas_extra) + gnssSbfBlock(96.0);
        return f.gnss_hz * per_epoch + 2200.0;
    }
    return 0.0;
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
         + framed(sizeof(GnssCheckData), 10.0)
         + framed(sizeof(GnssClockData), 1.0);   // the clock line, once a second with a fix
#else
    return 0.0;
#endif
}

// PAD_CONTEXT_MSG: written once, from 6 s after launch -- inside a guided
// coast -- one record each config::PAD_CONTEXT_PERIOD_MS, at worst all the
// largest kind's. Not steady: about 1.5 s of it. Zero on u-blox boards, which
// have no raw-GNSS filter (the flight loop's one PCX_CAL record is noise).
double padContextBurst()
{
#if defined(TR_GNSS_DRIVER_LC86) && TR_GNSS_DRIVER_LC86
    const size_t largest = sizeof(PadContextHeader)
                         + std::max({sizeof(PadContextNav), padContextCovWireSize(PCX_COV_PER_PART),
                                     padContextSurveyWireSize(PCX_SURVEY_PER_PART), PCX_EPH_MAX_FRAME});
    return framed(largest, 1000.0 / (double)config::PAD_CONTEXT_PERIOD_MS);
#else
    return 0.0;
#endif
}

// The link: I2S_SAMPLE_RATE samples/s x 4 B (16-bit stereo).
constexpr double kLinkBytesPerSec = (double)config::I2S_SAMPLE_RATE * 4.0;

// Steady-state inflow during a GUIDED coast at the fastest IMU rate the flight
// flies (IMU_RATE_MAX_HZ, or the receiver's lower limit: a fixed rate or a
// dynamic mode's boost) — the worst case, and the phase guidance telemetry
// exists for. Built once per board, for each receiver its image flies.
double guidedCoastInflow(const Flight& f)
{
    return imuBatched(f.imu_hz)
         + framed(sizeof(NonSensorData),   (double)config::NON_SENSOR_UPDATE_RATE)
         + framed(sizeof(BMP585Data),      (double)config::BMP585_UPDATE_RATE)
         + framed(sizeof(GuidanceTelemData), (double)config::GUIDANCE_TELEM_RATE_HZ)
         + framed(sizeof(FlightSnapshotData), 10.0)
         + framed(sizeof(IIS2MDCData),     100.0)
         + framed(sizeof(GNSSData),        f.gnss_hz)
         // #1154 item 4: the FC's camera truth. Counted here because that is
         // the whole reason it is a 5 Hz message and not a byte in the 500 Hz
         // NonSensorData — the byte version measured 95.06% and tripped the
         // line below. A stream the budget does not know about is how this
         // block went stale twice before.
         + framed(sizeof(FcStatusData),    (double)config::FC_STATUS_RATE_HZ)
         + framed(sizeof(Ism6TempData),    1.0)   // the IMU's die temperature
         + gnssRtcmInflow(f)
         + rawGnssShadowInflow()
         + 220.0;   // POWERData, ~15 Hz
}

}  // namespace

TEST(I2SLinkBudget, GuidedCoastLeavesRealHeadroom) {
    for (const Flight& f : flights())
    {
        const double inflow = guidedCoastInflow(f);
        const double used   = inflow / kLinkBytesPerSec;

        // 95% is the line. Below it there is room for jitter and for the burst
        // traffic this steady-state sum ignores (config readback, OTA chunks, the
        // snapshot cadence changing); above it, enqueueI2STx starts dropping and
        // says nothing.  At GUIDANCE_TELEM_RATE_HZ 500 this was 0.986.
        EXPECT_LT(used, 0.95)
            << "with the " << f.receiver << " at " << f.imu_hz << " Hz IMU, "
            << "guided-coast inflow " << inflow << " B/s is " << (used * 100.0)
            << "% of the " << kLinkBytesPerSec << " B/s link. A struct on the link "
            << "grew, or a rate went up — re-derive the budget block in config.h "
            << "and bring this back under the line.";
    }
}

TEST(I2SLinkBudget, ThePadContextFitsInAGuidedCoast) {
    // Its burst on top of the steady state: it must not push the coast it
    // lands in over the line either.
    for (const Flight& f : flights())
    {
        const double inflow = guidedCoastInflow(f) + padContextBurst();
        const double used   = inflow / kLinkBytesPerSec;
        EXPECT_LT(used, 0.95)
            << "with the " << f.receiver << ", a guided coast with the pad context's burst is "
            << (used * 100.0) << "% of the link: lengthen config::PAD_CONTEXT_PERIOD_MS.";
    }
}

TEST(I2SLinkBudget, TheDominantStreamIsTheIMU) {
    // Sanity on the shape of the budget: if anything ever outgrows the IMU
    // stream, the tuning advice in config.h (and the bench gauges it names)
    // is pointing at the wrong thing. With the mosaic-G5 its SBF is the other
    // big stream, about the IMU's size at 3840 Hz, and config.h names both.
    for (const Flight& f : flights())
    {
        const double imu = imuBatched(f.imu_hz);
        const double big = imu + (f.gnss == GNSS_TYPE_MOSAIC ? gnssRtcmInflow(f) : 0.0);
        EXPECT_GT(big, guidedCoastInflow(f) * 0.5) << f.receiver;
    }
}

TEST(I2SLinkBudget, NonGuidedFlightHasMoreRoomStill) {
    // GuidanceTelem is only emitted while guidance is active, so an unguided
    // flight is strictly cheaper.  Stated so the guided case is understood as
    // the binding one rather than the only one.
    for (const Flight& f : flights())
    {
        const double guided   = guidedCoastInflow(f);
        const double unguided = guided - framed(sizeof(GuidanceTelemData),
                                                (double)config::GUIDANCE_TELEM_RATE_HZ);
        EXPECT_LT(unguided, guided);
        EXPECT_LT(unguided / kLinkBytesPerSec, 0.95) << f.receiver;
    }
}

TEST(I2SLinkBudget, TheMosaicFliesUnderItsOwnImuLimit) {
    // Where an image can fly the mosaic-G5, the IMU limit it sets is what
    // keeps its SBF from crowding the link; at the board's own limit the same
    // coast would run close to the line.
    for (const Flight& f : flights())
    {
        if (f.gnss != GNSS_TYPE_MOSAIC) continue;
        EXPECT_LE(f.imu_hz, (double)IMU_RATE_MAX_HZ_MOSAIC);
        const double used = guidedCoastInflow(f) / kLinkBytesPerSec;
        EXPECT_LT(used, 0.75) << "the mosaic-G5 at " << f.imu_hz << " Hz IMU: " << (used * 100.0) << "%";
    }
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
