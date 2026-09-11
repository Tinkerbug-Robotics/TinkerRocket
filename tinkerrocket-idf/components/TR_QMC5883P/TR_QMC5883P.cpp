#include <TR_QMC5883P.h>

// Per-transfer timeout in MILLISECONDS (the IDF-v6 i2c_master_* API takes
// ms, not ticks — #297).  Same 10 ms as TR_IIS2MDC and for the same reason
// (#1111): it caps what one failed attempt costs the IMU poll loop, which
// pays for every blocking mag transfer out of its own sample rate.
static constexpr uint32_t I2C_TIMEOUT_MS = 10;

using namespace qmc5883p;

// ---------------------------------------------------------------------------
//  Constructor
// ---------------------------------------------------------------------------
TR_QMC5883P::TR_QMC5883P(uint8_t addr)
    : _dev(nullptr),
      _addr(addr),
      _range(Range::G8),
      _off_x(0), _off_y(0), _off_z(0),
      _overflow(false),
      _rereads(0)
{}

// ---------------------------------------------------------------------------
//  begin – add device to bus, probe the chip ID, reset to a known state
// ---------------------------------------------------------------------------
TR_QMC5883PStatus TR_QMC5883P::begin(i2c_master_bus_handle_t bus,
                                     uint32_t clock_hz)
{
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = _addr;
    dev_cfg.scl_speed_hz    = clock_hz;

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &_dev);
    if (err != ESP_OK) return TR_QMC5883P_ERROR;

    if (!isConnected()) return TR_QMC5883P_ERROR;

    // Clean slate.  The part comes out of POR in suspend mode with default
    // registers, but only if it saw a POR — a rail that stayed up across an
    // MCU reset leaves whatever the last session configured.
    if (softReset() != TR_QMC5883P_OK) return TR_QMC5883P_ERROR;
    return TR_QMC5883P_OK;
}

// ---------------------------------------------------------------------------
//  isConnected – true iff the chip ID reads 0x80
// ---------------------------------------------------------------------------
bool TR_QMC5883P::isConnected()
{
    uint8_t id = 0;
    if (readChipId(&id) != TR_QMC5883P_OK) return false;
    return (id == CHIP_ID);
}

// ---------------------------------------------------------------------------
//  softReset – CTRL2.SOFT_RST, and the software offset back to zero
// ---------------------------------------------------------------------------
TR_QMC5883PStatus TR_QMC5883P::softReset()
{
    // Zeroed whether or not the write lands: after a reset the part holds no
    // correction, and a driver that remembered one would subtract it from a
    // chip that is not.  A failed begin() reports "no magnetometer" anyway.
    _off_x = _off_y = _off_z = 0;
    _overflow = false;
    const TR_QMC5883PStatus st = writeRegister(REG_CTRL2, CTRL2_SOFT_RST);
    // Table 7: POR is complete within 250 µs; soft reset has no figure of
    // its own.  5 ms is paid once, at begin().
    delay(5);
    return st;
}

// ---------------------------------------------------------------------------
//  configure – sign word, suspend, CTRL2, CTRL1 (datasheet section 7 order)
// ---------------------------------------------------------------------------
TR_QMC5883PStatus TR_QMC5883P::configure(Odr odr, Mode mode, Range range,
                                         Osr1 osr1, Osr2 osr2,
                                         SetReset set_reset)
{
    // "Write Register 29H by 0x06 (Define the sign for X Y and Z axis)" —
    // first in every example the datasheet gives.  Re-written on every
    // configure() because softReset() restores every register and the
    // datasheet does not say whether 0x29 is among them.
    if (writeRegister(REG_AXIS_SIGN, AXIS_SIGN_VALUE) != TR_QMC5883P_OK)
        return TR_QMC5883P_ERROR;

    // 9.2.3: "Suspend Mode should be added in the middle of mode shifting."
    // Park the part before touching range or rate.  Harmless on a part that
    // is already suspended (fresh reset); required on one that is running
    // (reviveIIS2MDC() re-configures without a reset).
    if (writeRegister(REG_CTRL1, ctrl1(Mode::SUSPEND, odr, osr1, osr2)) != TR_QMC5883P_OK)
        return TR_QMC5883P_ERROR;

    if (writeRegister(REG_CTRL2, ctrl2(range, set_reset)) != TR_QMC5883P_OK)
        return TR_QMC5883P_ERROR;
    _range = range;

    return writeRegister(REG_CTRL1, ctrl1(mode, odr, osr1, osr2));
}

// ---------------------------------------------------------------------------
//  dataReady – STATUS.DRDY (cleared by this very read)
// ---------------------------------------------------------------------------
bool TR_QMC5883P::dataReady()
{
    uint8_t status = 0;
    if (readRegister(REG_STATUS, &status) != TR_QMC5883P_OK) return false;
    _overflow = (status & STATUS_OVFL) != 0;
    return (status & STATUS_DRDY) != 0;
}

// ---------------------------------------------------------------------------
//  readBurst – the 6 output registers from XOUT_L, little-endian per axis
// ---------------------------------------------------------------------------
TR_QMC5883PStatus TR_QMC5883P::readBurst(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6] = {0};
    if (readRegisters(REG_XOUT_L, buf, 6) != TR_QMC5883P_OK)
        return TR_QMC5883P_ERROR;
    *x = decodeAxis(&buf[0]);
    *y = decodeAxis(&buf[2]);
    *z = decodeAxis(&buf[4]);
    return TR_QMC5883P_OK;
}

// ---------------------------------------------------------------------------
//  readRawXYZ – tear-checked sample with the hard iron subtracted
// ---------------------------------------------------------------------------
TR_QMC5883PStatus TR_QMC5883P::readRawXYZ(QMC5883P_RawData *out)
{
    // 1. Read STATUS to clear DRDY, so that step 3 can only see a refresh
    //    that landed after this instant.
    uint8_t status = 0;
    if (readRegister(REG_STATUS, &status) != TR_QMC5883P_OK)
        return TR_QMC5883P_ERROR;
    bool ovfl = (status & STATUS_OVFL) != 0;

    // 2. The sample.
    int16_t x = 0, y = 0, z = 0;
    if (readBurst(&x, &y, &z) != TR_QMC5883P_OK)
        return TR_QMC5883P_ERROR;

    // 3. Did the output registers refresh between 1 and now?  Then the burst
    //    may have straddled it — read again.  The re-read cannot straddle:
    //    the next refresh is a whole ODR period away (10 ms at 100 Hz, ~50x
    //    the burst).  A refresh that landed AFTER the burst also trips this
    //    and simply hands back the newer sample.
    if (readRegister(REG_STATUS, &status) != TR_QMC5883P_OK)
        return TR_QMC5883P_ERROR;
    ovfl = ovfl || ((status & STATUS_OVFL) != 0);
    if (status & STATUS_DRDY)
    {
        _rereads++;
        if (readBurst(&x, &y, &z) != TR_QMC5883P_OK)
            return TR_QMC5883P_ERROR;
    }
    _overflow = ovfl;

    // Hard iron, the way the IIS2MDC's OFFSET registers do it in silicon.
    out->x = subtractSaturating(x, _off_x);
    out->y = subtractSaturating(y, _off_y);
    out->z = subtractSaturating(z, _off_z);
    return TR_QMC5883P_OK;
}

// ---------------------------------------------------------------------------
//  Hard-iron offset (software; the part has no OFFSET registers)
// ---------------------------------------------------------------------------
TR_QMC5883PStatus TR_QMC5883P::setHardIronOffset(int16_t cx, int16_t cy, int16_t cz)
{
    _off_x = cx;
    _off_y = cy;
    _off_z = cz;
    return TR_QMC5883P_OK;
}

void TR_QMC5883P::getHardIronOffset(int16_t *cx, int16_t *cy, int16_t *cz) const
{
    *cx = _off_x;
    *cy = _off_y;
    *cz = _off_z;
}

// ---------------------------------------------------------------------------
//  readFieldsXYZ_uT – counts × (100 / LSB-per-gauss at the configured range)
// ---------------------------------------------------------------------------
TR_QMC5883PStatus TR_QMC5883P::readFieldsXYZ_uT(float *x_uT, float *y_uT, float *z_uT)
{
    QMC5883P_RawData raw = {};
    if (readRawXYZ(&raw) != TR_QMC5883P_OK) return TR_QMC5883P_ERROR;
    const float k = lsbToUt();
    *x_uT = (float)raw.x * k;
    *y_uT = (float)raw.y * k;
    *z_uT = (float)raw.z * k;
    return TR_QMC5883P_OK;
}

// ---------------------------------------------------------------------------
//  Chip ID
// ---------------------------------------------------------------------------
TR_QMC5883PStatus TR_QMC5883P::readChipId(uint8_t *id)
{
    return readRegister(REG_CHIP_ID, id);
}

// ---------------------------------------------------------------------------
//  Low-level I2C register access (8-bit registers)
// ---------------------------------------------------------------------------
TR_QMC5883PStatus TR_QMC5883P::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    esp_err_t err = i2c_master_transmit(_dev, buf, 2, I2C_TIMEOUT_MS);
    return (err == ESP_OK) ? TR_QMC5883P_OK : TR_QMC5883P_ERROR;
}

TR_QMC5883PStatus TR_QMC5883P::readRegister(uint8_t reg, uint8_t *value)
{
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, value, 1,
                                                 I2C_TIMEOUT_MS);
    return (err == ESP_OK) ? TR_QMC5883P_OK : TR_QMC5883P_ERROR;
}

TR_QMC5883PStatus TR_QMC5883P::readRegisters(uint8_t reg, uint8_t *buf, size_t len)
{
    // The register pointer auto-increments across 01H-06H: every setup
    // example in the datasheet reads "data Register 01H ~ 06H" as one burst.
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, buf, len,
                                                 I2C_TIMEOUT_MS);
    return (err == ESP_OK) ? TR_QMC5883P_OK : TR_QMC5883P_ERROR;
}
