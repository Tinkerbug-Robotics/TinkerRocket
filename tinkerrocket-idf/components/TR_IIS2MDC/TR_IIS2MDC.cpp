#include <TR_IIS2MDC.h>

static constexpr uint32_t I2C_TIMEOUT_MS = 50;

// CFG_REG_A bit positions
static constexpr uint8_t CFG_A_COMP_TEMP_EN = (1U << 7);
static constexpr uint8_t CFG_A_REBOOT       = (1U << 6);
static constexpr uint8_t CFG_A_SOFT_RST     = (1U << 5);
static constexpr uint8_t CFG_A_LP           = (1U << 4);
static constexpr uint8_t CFG_A_ODR_SHIFT    = 2;
static constexpr uint8_t CFG_A_MD_SHIFT     = 0;

// CFG_REG_B bit positions
static constexpr uint8_t CFG_B_SET_FREQ     = (1U << 2);
static constexpr uint8_t CFG_B_OFF_CANC     = (1U << 1);
static constexpr uint8_t CFG_B_LPF          = (1U << 0);

// CFG_REG_C bit positions
static constexpr uint8_t CFG_C_BDU          = (1U << 4);
static constexpr uint8_t CFG_C_DRDY_ON_PIN  = (1U << 0);

// STATUS_REG bit positions
static constexpr uint8_t STATUS_ZYXDA       = (1U << 3);

// ---------------------------------------------------------------------------
//  Constructor
// ---------------------------------------------------------------------------
TR_IIS2MDC::TR_IIS2MDC(uint8_t addr)
    : _dev(nullptr),
      _addr(addr)
{}

// ---------------------------------------------------------------------------
//  begin – add device to bus and probe WHO_AM_I
// ---------------------------------------------------------------------------
TR_IIS2MDCStatus TR_IIS2MDC::begin(i2c_master_bus_handle_t bus,
                                   uint32_t clock_hz)
{
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = _addr;
    dev_cfg.scl_speed_hz    = clock_hz;

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &_dev);
    if (err != ESP_OK) return TR_IIS2MDC_ERROR;

    if (!isConnected()) return TR_IIS2MDC_ERROR;

    // Force a clean state. If the IIS2MDC rail stayed alive across an ESP32
    // reset, internal state (CFG bits, OFFSET regs, OFF_CANC tracking) can
    // be stale and the power-on Set pulse never re-fires. Soft reset clears
    // it. Datasheet: reset settles in ~9.7 ms.
    if (softReset() != TR_IIS2MDC_OK) return TR_IIS2MDC_ERROR;
    delay(15);
    return TR_IIS2MDC_OK;
}

// ---------------------------------------------------------------------------
//  isConnected – returns true iff WHO_AM_I reads back 0x40
// ---------------------------------------------------------------------------
bool TR_IIS2MDC::isConnected()
{
    uint8_t id = 0;
    if (readWhoAmI(&id) != TR_IIS2MDC_OK) return false;
    return (id == IIS2MDC_WHO_AM_I_VALUE);
}

// ---------------------------------------------------------------------------
//  softReset – set SOFT_RST bit in CFG_REG_A
// ---------------------------------------------------------------------------
TR_IIS2MDCStatus TR_IIS2MDC::softReset()
{
    return writeRegister(IIS2MDC_Reg::CFG_REG_A, CFG_A_SOFT_RST);
}

// ---------------------------------------------------------------------------
//  configure – set ODR, mode, and the most useful flags in one shot
// ---------------------------------------------------------------------------
TR_IIS2MDCStatus TR_IIS2MDC::configure(IIS2MDC_ODR odr,
                                       IIS2MDC_Mode mode,
                                       bool low_power,
                                       bool offset_cancellation,
                                       bool set_pulse_periodic,
                                       bool low_pass_filter,
                                       bool block_data_update,
                                       bool drdy_on_int_pin)
{
    uint8_t cfg_a = CFG_A_COMP_TEMP_EN
                  | ((uint8_t)odr  << CFG_A_ODR_SHIFT)
                  | ((uint8_t)mode << CFG_A_MD_SHIFT);
    if (low_power) cfg_a |= CFG_A_LP;
    if (writeRegister(IIS2MDC_Reg::CFG_REG_A, cfg_a) != TR_IIS2MDC_OK)
        return TR_IIS2MDC_ERROR;

    uint8_t cfg_b = 0;
    if (offset_cancellation) cfg_b |= CFG_B_OFF_CANC;
    if (set_pulse_periodic)  cfg_b |= CFG_B_SET_FREQ;
    if (low_pass_filter)     cfg_b |= CFG_B_LPF;
    if (writeRegister(IIS2MDC_Reg::CFG_REG_B, cfg_b) != TR_IIS2MDC_OK)
        return TR_IIS2MDC_ERROR;

    uint8_t cfg_c = 0;
    if (block_data_update) cfg_c |= CFG_C_BDU;
    if (drdy_on_int_pin)   cfg_c |= CFG_C_DRDY_ON_PIN;
    return writeRegister(IIS2MDC_Reg::CFG_REG_C, cfg_c);
}

// ---------------------------------------------------------------------------
//  dataReady – Zyxda bit in STATUS_REG
// ---------------------------------------------------------------------------
bool TR_IIS2MDC::dataReady()
{
    uint8_t status = 0;
    if (readRegister(IIS2MDC_Reg::STATUS_REG, &status) != TR_IIS2MDC_OK)
        return false;
    return (status & STATUS_ZYXDA) != 0;
}

// ---------------------------------------------------------------------------
//  readRawXYZ – burst read 6 bytes from OUTX_L_REG
// ---------------------------------------------------------------------------
TR_IIS2MDCStatus TR_IIS2MDC::readRawXYZ(IIS2MDC_RawData *out)
{
    uint8_t buf[6] = {0};
    if (readRegisters(IIS2MDC_Reg::OUTX_L_REG, buf, 6) != TR_IIS2MDC_OK)
        return TR_IIS2MDC_ERROR;

    // Datasheet 9.16: little-endian per axis (L then H).
    out->x = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    out->y = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    out->z = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    return TR_IIS2MDC_OK;
}

// ---------------------------------------------------------------------------
//  readFieldsXYZ_uT – raw counts * 0.15 uT/LSB
// ---------------------------------------------------------------------------
TR_IIS2MDCStatus TR_IIS2MDC::readFieldsXYZ_uT(float *x_uT, float *y_uT, float *z_uT)
{
    IIS2MDC_RawData raw = {};
    if (readRawXYZ(&raw) != TR_IIS2MDC_OK) return TR_IIS2MDC_ERROR;
    *x_uT = (float)raw.x * IIS2MDC_LSB_TO_uT;
    *y_uT = (float)raw.y * IIS2MDC_LSB_TO_uT;
    *z_uT = (float)raw.z * IIS2MDC_LSB_TO_uT;
    return TR_IIS2MDC_OK;
}

// ---------------------------------------------------------------------------
//  WHO_AM_I
// ---------------------------------------------------------------------------
TR_IIS2MDCStatus TR_IIS2MDC::readWhoAmI(uint8_t *id)
{
    return readRegister(IIS2MDC_Reg::WHO_AM_I, id);
}

// ---------------------------------------------------------------------------
//  Low-level I2C register access (8-bit registers)
// ---------------------------------------------------------------------------
TR_IIS2MDCStatus TR_IIS2MDC::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    esp_err_t err = i2c_master_transmit(_dev, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    return (err == ESP_OK) ? TR_IIS2MDC_OK : TR_IIS2MDC_ERROR;
}

TR_IIS2MDCStatus TR_IIS2MDC::readRegister(uint8_t reg, uint8_t *value)
{
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, value, 1,
                                                 pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    return (err == ESP_OK) ? TR_IIS2MDC_OK : TR_IIS2MDC_ERROR;
}

TR_IIS2MDCStatus TR_IIS2MDC::readRegisters(uint8_t reg, uint8_t *buf, size_t len)
{
    // IIS2MDC auto-increments the sub-address on multi-byte reads (datasheet 6.1.1).
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, buf, len,
                                                 pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    return (err == ESP_OK) ? TR_IIS2MDC_OK : TR_IIS2MDC_ERROR;
}
