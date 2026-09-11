#ifndef TR_QMC5883P_H
#define TR_QMC5883P_H

#include <compat.h>
#include <driver/i2c_master.h>
#include "qmc5883p_regs.h"

// ---------------------------------------------------------------------------
//  TR_QMC5883P — QST QMC5883P magnetometer on the ESP-IDF i2c_master API.
//
//  rocket-computer-mini's U3 (#797), driven through TR_Sensor_Collector's
//  TR_MAG_DRIVER_QMC5883P seam (#1312).  Register facts are in
//  qmc5883p_regs.h with their datasheet references.
//
//  The call surface is TR_IIS2MDC's on purpose — begin / isConnected /
//  softReset / configure / dataReady / readRawXYZ / setHardIronOffset /
//  readFieldsXYZ_uT — so the collector swaps the TYPE and nothing else, and
//  every consumer of the IIS2MDC-named count stream (OC, EKF, calibrator,
//  log, telemetry) keeps seeing the 10-byte IIS2MDCData it always did.  Two
//  things differ underneath and are absorbed here so they do not leak:
//
//  * No hard-iron OFFSET registers.  setHardIronOffset() keeps the offset in
//    the driver and readRawXYZ() subtracts it, saturating to int16 the way
//    the IIS2MDC's silicon does — so the logged counts are already corrected
//    on both parts, and the asymmetry #1303 warned a converter-side subtract
//    would create does not arise.  softReset() zeroes the stored offset for
//    the reason the IIS2MDC's reset zeroes its registers: the boot sequence
//    relies on "offset is zero after begin(); apply it afterwards".
//
//  * No block-data-update.  The output registers refresh at the ODR whether
//    or not a read is in progress (datasheet 9.2.1), so a 6-byte burst can
//    straddle a refresh and hand back two half-samples.  readRawXYZ()
//    brackets the burst with two STATUS reads: the first clears DRDY, the
//    second reports whether a refresh landed anywhere in between; if it did,
//    the burst is read again, now a full ODR period clear of the next one.
//    Cost: two 1-byte reads per sample (~0.2 ms at 400 kHz) plus a second
//    burst on the ~2% of polls that straddle.  This holds only while the
//    part paces itself at an ODR — continuous mode free-runs at up to
//    1.5 kHz and would defeat it; configure() defaults to normal mode.
//
//  Scale: the default ±8 G is 3750 LSB/G (datasheet Table 2) = 100/3750
//  µT/LSB, the value MAG_TYPE_QMC5883P promises every log reader.  Keep
//  configure()'s range at G8 unless that contract changes with it.
//
//  Axes (datasheet Figure 2, package seen from above, pin-1 dot top-left):
//  +X points from the GND/C1 edge (pins 9-12) toward the SCK/VDD edge
//  (pins 1-4); +Y points from the SDA edge (pins 13-16) toward the all-NC
//  edge (pins 5-8); +Z points up out of the package.  Right-handed, with
//  the 0x29 = 0x06 sign word configure() writes.  The chip→board rotation
//  (IIS2MDC_ROT_Z_DEG) is a placement fact of the mini layout and is still
//  marked VERIFY there; the axis silk on the board waits on the same bench.
// ---------------------------------------------------------------------------

typedef enum {
    TR_QMC5883P_OK    =  0,
    TR_QMC5883P_ERROR = -1
} TR_QMC5883PStatus;

static constexpr uint8_t QMC5883P_CHIP_ID_VALUE = qmc5883p::CHIP_ID;
static constexpr uint8_t QMC5883P_DEFAULT_ADDR  = qmc5883p::I2C_ADDR;

// The scale of the counts this driver hands out at its default ±8 G — the
// number behind MAG_TYPE_QMC5883P in RocketComputerTypes.h.
static constexpr float QMC5883P_LSB_TO_uT = qmc5883p::uTPerLsb(qmc5883p::Range::G8);

// Raw measurement (signed 16-bit per axis, chip frame, hard iron subtracted)
struct QMC5883P_RawData {
    int16_t x;
    int16_t y;
    int16_t z;
};

class TR_QMC5883P
{
public:
    TR_QMC5883P(uint8_t addr = QMC5883P_DEFAULT_ADDR);

    /// Add the device to an existing master bus, verify the chip ID, then
    /// soft-reset — a rail that stayed up across an MCU reset would otherwise
    /// carry the previous mode, range and sign word into this session (the
    /// same reason TR_IIS2MDC::begin() resets).
    TR_QMC5883PStatus begin(i2c_master_bus_handle_t bus,
                            uint32_t clock_hz = 400000);

    /// True iff register 0x00 reads the QMC5883P chip ID (0x80).
    bool isConnected();

    /// CTRL2.SOFT_RST: every register back to default (suspend mode), and the
    /// driver's hard-iron offset back to zero.  POR completes inside 250 µs
    /// (datasheet Table 7); the driver waits 5 ms.
    TR_QMC5883PStatus softReset();

    /// One-shot configuration, in the datasheet's order (section 7): the
    /// axis sign word (0x29 = 0x06), then suspend — the mode a mode change
    /// has to pass through (9.2.3), which also makes this a safe
    /// re-configure of a part that is already running (the stall-revive
    /// path) — then CTRL2 (range, set/reset), then CTRL1 (OSR, ODR, mode).
    ///
    /// Defaults: normal mode at 100 Hz (the IIS2MDC's cadence, and what the
    /// collector's poll gate assumes), ±8 G, OSR1 = 8 / OSR2 = 8 (the most
    /// filtering the part offers; ~600 µA), set/reset on (periodic degauss,
    /// the counterpart of the IIS2MDC's OFF_CANC with periodic Set pulses).
    TR_QMC5883PStatus configure(qmc5883p::Odr      odr       = qmc5883p::Odr::HZ_100,
                                qmc5883p::Mode     mode      = qmc5883p::Mode::NORMAL,
                                qmc5883p::Range    range     = qmc5883p::Range::G8,
                                qmc5883p::Osr1     osr1      = qmc5883p::Osr1::X8,
                                qmc5883p::Osr2     osr2      = qmc5883p::Osr2::X8,
                                qmc5883p::SetReset set_reset = qmc5883p::SetReset::SET_AND_RESET_ON);

    /// STATUS.DRDY — and reading it clears it (datasheet 9.2.2).  Latches
    /// OVFL for overflowed().  Not used by the collector, which paces reads
    /// by time; here for bench use and parity with the ST driver.
    bool dataReady();

    /// One sample: hard iron subtracted, tear-checked (see the header note).
    /// Up to four transfers, bailing at the first failure, so a wedged bus
    /// costs one I2C timeout, not four.
    TR_QMC5883PStatus readRawXYZ(QMC5883P_RawData *out);

    /// Set the hard-iron offset the driver subtracts from every sample, in
    /// raw LSB (100/3750 µT/LSB at ±8 G, signed 16-bit) — the units and the
    /// effect of writing the IIS2MDC's OFFSET_X/Y/Z.  No bus traffic, so it
    /// cannot fail; the status return is for signature parity.  Written by
    /// the app task while the poll task reads: a torn triple lasts one
    /// sample and the next cal apply rewrites all three (the collector makes
    /// the same call about its own copy).
    TR_QMC5883PStatus setHardIronOffset(int16_t cx, int16_t cy, int16_t cz);
    void getHardIronOffset(int16_t *cx, int16_t *cy, int16_t *cz) const;

    /// One sample in µT at the configured range.
    TR_QMC5883PStatus readFieldsXYZ_uT(float *x_uT, float *y_uT, float *z_uT);

    /// Register 0x00 (expected 0x80).
    TR_QMC5883PStatus readChipId(uint8_t *id);

    /// OVFL seen on the most recent readRawXYZ() / dataReady(): an axis went
    /// past ±30000 LSB (±8 G) — a magnet, or a pyro current loop, next to
    /// the part.
    bool overflowed() const { return _overflow; }

    /// How many readRawXYZ() calls needed the second burst.  A few per second
    /// at 100 Hz is the expected drift between the chip's clock and the poll
    /// gate; a steadily high count means the part is not pacing itself at
    /// the ODR configure() asked for.
    uint32_t tearRereads() const { return _rereads; }

    /// µT per LSB at the configured range.
    float lsbToUt() const { return qmc5883p::uTPerLsb(_range); }

    /// Low-level register access.
    TR_QMC5883PStatus readRegister(uint8_t reg, uint8_t *value);
    TR_QMC5883PStatus writeRegister(uint8_t reg, uint8_t value);

private:
    i2c_master_dev_handle_t _dev;
    uint8_t         _addr;
    qmc5883p::Range _range;
    int16_t         _off_x, _off_y, _off_z;
    bool            _overflow;
    uint32_t        _rereads;

    TR_QMC5883PStatus readRegisters(uint8_t reg, uint8_t *buf, size_t len);
    TR_QMC5883PStatus readBurst(int16_t *x, int16_t *y, int16_t *z);
};

#endif  // TR_QMC5883P_H
