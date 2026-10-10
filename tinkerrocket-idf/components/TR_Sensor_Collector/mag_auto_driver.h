#pragma once

// #1590: one image, two magnetometers.  The V9 and V10 Tinker-Mantis boards
// build from one board header and wire their magnetometer to the same I2C
// pins (MAG_SDA GPIO47, MAG_SCL GPIO48), but V9 fits an ST IIS2MDC (0x1E)
// and V10 a QST QMC5883P (0x2C).  begin() asks for the IIS2MDC first, then
// the QMC5883P, and keeps whichever answered; every later call goes to that
// one.  It has the call surface both drivers share, so it slots into
// SensorCollector's magnetometer seam (TR_MAG_DRIVER_AUTO) the same way
// either of them does.  type() reports the chip found, and is what the
// converter, the calibrator, the SIL and the OUT_STATUS_QUERY stamp read.

#include <cstdint>
#include <driver/i2c_master.h>
#include <RocketComputerTypes.h>
#include <TR_IIS2MDC.h>
#include <TR_QMC5883P.h>

typedef enum {
    TR_MAG_AUTO_OK    =  0,
    TR_MAG_AUTO_ERROR = -1
} TR_MagAutoStatus;

// Same layout as IIS2MDC_RawData and QMC5883P_RawData: chip-frame counts.
struct MagAuto_RawData {
    int16_t x;
    int16_t y;
    int16_t z;
};

// The scale each MAG_TYPE promises log readers has to be the driver's.
static_assert(IIS2MDC_LSB_TO_uT - magTypeUtPerLsb(MAG_TYPE_IIS2MDC) < 1e-6 &&
              magTypeUtPerLsb(MAG_TYPE_IIS2MDC) - IIS2MDC_LSB_TO_uT < 1e-6,
              "IIS2MDC_LSB_TO_uT disagrees with magTypeUtPerLsb(MAG_TYPE_IIS2MDC)");
static_assert(QMC5883P_LSB_TO_uT - magTypeUtPerLsb(MAG_TYPE_QMC5883P) < 1e-6 &&
              magTypeUtPerLsb(MAG_TYPE_QMC5883P) - QMC5883P_LSB_TO_uT < 1e-6,
              "QMC5883P_LSB_TO_uT disagrees with magTypeUtPerLsb(MAG_TYPE_QMC5883P)");

class TR_MagAuto
{
public:
    explicit TR_MagAuto(uint8_t iis2mdc_addr = IIS2MDC_DEFAULT_ADDR,
                        uint8_t qmc5883p_addr = QMC5883P_DEFAULT_ADDR);

    // Probe the IIS2MDC, then the QMC5883P, on this bus.  OK when either
    // answered (and soft-reset); the one that did is type() from then on.
    TR_MagAutoStatus begin(i2c_master_bus_handle_t bus, uint32_t clock_hz = 400000);

    bool             isConnected();
    TR_MagAutoStatus configure();   // each driver's defaults: 100 Hz
    TR_MagAutoStatus readRawXYZ(MagAuto_RawData *out);
    TR_MagAutoStatus setHardIronOffset(int16_t cx, int16_t cy, int16_t cz);
    TR_MagAutoStatus readFieldsXYZ_uT(float *x_uT, float *y_uT, float *z_uT);

    // MAG_TYPE_IIS2MDC until begin() has found a QMC5883P, so a board where
    // neither answered reads as the IIS2MDC, as an unstamped log does.
    uint8_t     type() const { return type_; }
    bool        found() const { return found_; }
    float       lsbToUt() const;
    const char* name() const;
    const char* configNote() const;

    // The driver begin() chose, for the chip-specific boot diagnostics.
    TR_IIS2MDC&  iis2mdc()  { return iis_; }
    TR_QMC5883P& qmc5883p() { return qmc_; }

private:
    TR_IIS2MDC  iis_;
    TR_QMC5883P qmc_;
    uint8_t     type_  = MAG_TYPE_IIS2MDC;
    bool        found_ = false;
};
