#include "mag_auto_driver.h"

TR_MagAuto::TR_MagAuto(uint8_t iis2mdc_addr, uint8_t qmc5883p_addr)
    : iis_(iis2mdc_addr), qmc_(qmc5883p_addr)
{
}

TR_MagAutoStatus TR_MagAuto::begin(i2c_master_bus_handle_t bus, uint32_t clock_hz)
{
    // The IIS2MDC first: it is the V9's, the board in the field.  A miss is
    // one NACKed WHO_AM_I read; the QMC5883P is then asked for its chip ID.
    if (iis_.begin(bus, clock_hz) == TR_IIS2MDC_OK)
    {
        type_ = MAG_TYPE_IIS2MDC;
        found_ = true;
        return TR_MAG_AUTO_OK;
    }
    if (qmc_.begin(bus, clock_hz) == TR_QMC5883P_OK)
    {
        type_ = MAG_TYPE_QMC5883P;
        found_ = true;
        return TR_MAG_AUTO_OK;
    }
    found_ = false;
    return TR_MAG_AUTO_ERROR;
}

bool TR_MagAuto::isConnected()
{
    if (!found_) return false;
    return (type_ == MAG_TYPE_QMC5883P) ? qmc_.isConnected() : iis_.isConnected();
}

TR_MagAutoStatus TR_MagAuto::configure()
{
    if (!found_) return TR_MAG_AUTO_ERROR;
    const bool ok = (type_ == MAG_TYPE_QMC5883P) ? qmc_.configure() == TR_QMC5883P_OK
                                                 : iis_.configure() == TR_IIS2MDC_OK;
    return ok ? TR_MAG_AUTO_OK : TR_MAG_AUTO_ERROR;
}

TR_MagAutoStatus TR_MagAuto::readRawXYZ(MagAuto_RawData *out)
{
    if (!found_ || out == nullptr) return TR_MAG_AUTO_ERROR;
    if (type_ == MAG_TYPE_QMC5883P)
    {
        QMC5883P_RawData r = {};
        if (qmc_.readRawXYZ(&r) != TR_QMC5883P_OK) return TR_MAG_AUTO_ERROR;
        out->x = r.x; out->y = r.y; out->z = r.z;
    }
    else
    {
        IIS2MDC_RawData r = {};
        if (iis_.readRawXYZ(&r) != TR_IIS2MDC_OK) return TR_MAG_AUTO_ERROR;
        out->x = r.x; out->y = r.y; out->z = r.z;
    }
    return TR_MAG_AUTO_OK;
}

TR_MagAutoStatus TR_MagAuto::setHardIronOffset(int16_t cx, int16_t cy, int16_t cz)
{
    if (!found_) return TR_MAG_AUTO_ERROR;
    const bool ok = (type_ == MAG_TYPE_QMC5883P)
                        ? qmc_.setHardIronOffset(cx, cy, cz) == TR_QMC5883P_OK
                        : iis_.setHardIronOffset(cx, cy, cz) == TR_IIS2MDC_OK;
    return ok ? TR_MAG_AUTO_OK : TR_MAG_AUTO_ERROR;
}

TR_MagAutoStatus TR_MagAuto::readFieldsXYZ_uT(float *x_uT, float *y_uT, float *z_uT)
{
    if (!found_) return TR_MAG_AUTO_ERROR;
    const bool ok = (type_ == MAG_TYPE_QMC5883P)
                        ? qmc_.readFieldsXYZ_uT(x_uT, y_uT, z_uT) == TR_QMC5883P_OK
                        : iis_.readFieldsXYZ_uT(x_uT, y_uT, z_uT) == TR_IIS2MDC_OK;
    return ok ? TR_MAG_AUTO_OK : TR_MAG_AUTO_ERROR;
}

float TR_MagAuto::lsbToUt() const
{
    return (type_ == MAG_TYPE_QMC5883P) ? QMC5883P_LSB_TO_uT : IIS2MDC_LSB_TO_uT;
}

const char* TR_MagAuto::name() const
{
    if (!found_) return "IIS2MDC or QMC5883P";
    return (type_ == MAG_TYPE_QMC5883P) ? "QMC5883P" : "IIS2MDC";
}

const char* TR_MagAuto::configNote() const
{
    return (type_ == MAG_TYPE_QMC5883P)
               ? "100 Hz normal mode, +/-8 G, set/reset on, no BDU: tear-checked reads"
               : "100 Hz continuous, BDU on";
}
