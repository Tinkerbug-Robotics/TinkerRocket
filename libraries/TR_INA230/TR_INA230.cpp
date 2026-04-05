#include <TR_INA230.h>

static constexpr uint32_t I2C_TIMEOUT_MS = 50;

// ---------------------------------------------------------------------------
//  Constructor
// ---------------------------------------------------------------------------
TR_INA230::TR_INA230(i2c_port_t port, uint8_t addr)
    : _port(port),
      _addr(addr),
      _current_lsb_A(0.0f)
{}

// ---------------------------------------------------------------------------
//  begin – install I2C master driver and probe the device
// ---------------------------------------------------------------------------
TR_INA230Status TR_INA230::begin(int sda_pin, int scl_pin,
                                 uint32_t clock_hz,
                                 bool internal_pullups)
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = static_cast<gpio_num_t>(sda_pin);
    conf.scl_io_num = static_cast<gpio_num_t>(scl_pin);
    conf.sda_pullup_en = internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = clock_hz;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(_port, &conf);
    if (err != ESP_OK) return TR_INA230_ERROR;

    err = i2c_driver_install(_port, conf.mode, 0, 0, 0);
    if (err != ESP_OK) return TR_INA230_ERROR;

    // Probe: try reading the configuration register (should return 0x4127 POR)
    uint16_t cfg = 0;
    return readRegister(INA230_Reg::CONFIGURATION, &cfg);
}

// ---------------------------------------------------------------------------
//  reset – software reset via RST bit (D15 of config register)
// ---------------------------------------------------------------------------
TR_INA230Status TR_INA230::reset()
{
    return writeRegister(INA230_Reg::CONFIGURATION, 0x8000);
}

// ---------------------------------------------------------------------------
//  Configuration helpers
// ---------------------------------------------------------------------------
TR_INA230Status TR_INA230::setAveraging(INA230_Avg avg)
{
    uint16_t cfg = 0;
    if (readRegister(INA230_Reg::CONFIGURATION, &cfg) != TR_INA230_OK)
        return TR_INA230_ERROR;
    cfg = (cfg & ~(0x07 << 9)) | ((uint16_t)avg << 9);
    return writeRegister(INA230_Reg::CONFIGURATION, cfg);
}

TR_INA230Status TR_INA230::setBusConvTime(INA230_ConvTime ct)
{
    uint16_t cfg = 0;
    if (readRegister(INA230_Reg::CONFIGURATION, &cfg) != TR_INA230_OK)
        return TR_INA230_ERROR;
    cfg = (cfg & ~(0x07 << 6)) | ((uint16_t)ct << 6);
    return writeRegister(INA230_Reg::CONFIGURATION, cfg);
}

TR_INA230Status TR_INA230::setShuntConvTime(INA230_ConvTime ct)
{
    uint16_t cfg = 0;
    if (readRegister(INA230_Reg::CONFIGURATION, &cfg) != TR_INA230_OK)
        return TR_INA230_ERROR;
    cfg = (cfg & ~(0x07 << 3)) | ((uint16_t)ct << 3);
    return writeRegister(INA230_Reg::CONFIGURATION, cfg);
}

TR_INA230Status TR_INA230::setMode(INA230_Mode mode)
{
    uint16_t cfg = 0;
    if (readRegister(INA230_Reg::CONFIGURATION, &cfg) != TR_INA230_OK)
        return TR_INA230_ERROR;
    cfg = (cfg & ~0x07) | (uint16_t)mode;
    return writeRegister(INA230_Reg::CONFIGURATION, cfg);
}

TR_INA230Status TR_INA230::setConfiguration(INA230_Avg avg,
                                            INA230_ConvTime bus_ct,
                                            INA230_ConvTime shunt_ct,
                                            INA230_Mode mode)
{
    uint16_t cfg = ((uint16_t)avg      << 9) |
                   ((uint16_t)bus_ct   << 6) |
                   ((uint16_t)shunt_ct << 3) |
                   ((uint16_t)mode);
    return writeRegister(INA230_Reg::CONFIGURATION, cfg);
}

TR_INA230Status TR_INA230::getConfiguration(uint16_t *value)
{
    return readRegister(INA230_Reg::CONFIGURATION, value);
}

// ---------------------------------------------------------------------------
//  Calibration
// ---------------------------------------------------------------------------
TR_INA230Status TR_INA230::setCalibration(uint16_t cal)
{
    return writeRegister(INA230_Reg::CALIBRATION, cal);
}

TR_INA230Status TR_INA230::calibrate(float r_shunt_ohm, float current_lsb_A)
{
    _current_lsb_A = current_lsb_A;

    // CAL = 0.00512 / (Current_LSB * R_SHUNT)   (Equation 1)
    float cal_f = 0.00512f / (current_lsb_A * r_shunt_ohm);
    uint16_t cal = (uint16_t)cal_f;
    if (cal > 0x7FFF) cal = 0x7FFF;

    return setCalibration(cal);
}

// ---------------------------------------------------------------------------
//  Data read – Shunt Voltage  (reg 0x01, LSB = 2.5 uV, signed)
// ---------------------------------------------------------------------------
TR_INA230Status TR_INA230::readShuntVoltageRaw(int16_t *raw)
{
    return readRegister(INA230_Reg::SHUNT_VOLTAGE, (uint16_t *)raw);
}

TR_INA230Status TR_INA230::readShuntVoltage_mV(float *mV)
{
    int16_t raw = 0;
    if (readShuntVoltageRaw(&raw) != TR_INA230_OK) return TR_INA230_ERROR;
    *mV = (float)raw * 0.0025f;   // 2.5 uV/bit -> mV
    return TR_INA230_OK;
}

// ---------------------------------------------------------------------------
//  Data read – Bus Voltage  (reg 0x02, LSB = 1.25 mV, unsigned)
// ---------------------------------------------------------------------------
TR_INA230Status TR_INA230::readBusVoltageRaw(uint16_t *raw)
{
    return readRegister(INA230_Reg::BUS_VOLTAGE, raw);
}

TR_INA230Status TR_INA230::readBusVoltage_V(float *volts)
{
    uint16_t raw = 0;
    if (readBusVoltageRaw(&raw) != TR_INA230_OK) return TR_INA230_ERROR;
    *volts = (float)raw * 0.00125f;   // 1.25 mV/bit -> V
    return TR_INA230_OK;
}

// ---------------------------------------------------------------------------
//  Data read – Current  (reg 0x04, signed, requires calibration)
// ---------------------------------------------------------------------------
TR_INA230Status TR_INA230::readCurrentRaw(int16_t *raw)
{
    return readRegister(INA230_Reg::CURRENT, (uint16_t *)raw);
}

TR_INA230Status TR_INA230::readCurrent_A(float *amps)
{
    int16_t raw = 0;
    if (readCurrentRaw(&raw) != TR_INA230_OK) return TR_INA230_ERROR;
    *amps = (float)raw * _current_lsb_A;
    return TR_INA230_OK;
}

// ---------------------------------------------------------------------------
//  Data read – Power  (reg 0x03, unsigned, requires calibration)
//  Power LSB = 25 * Current_LSB
// ---------------------------------------------------------------------------
TR_INA230Status TR_INA230::readPowerRaw(uint16_t *raw)
{
    return readRegister(INA230_Reg::POWER, raw);
}

TR_INA230Status TR_INA230::readPower_W(float *watts)
{
    uint16_t raw = 0;
    if (readPowerRaw(&raw) != TR_INA230_OK) return TR_INA230_ERROR;
    *watts = (float)raw * 25.0f * _current_lsb_A;
    return TR_INA230_OK;
}

// ---------------------------------------------------------------------------
//  Read all measurements
// ---------------------------------------------------------------------------
TR_INA230Status TR_INA230::readAll(INA230_Data *data)
{
    if (readShuntVoltage_mV(&data->shunt_voltage_mV) != TR_INA230_OK)
        return TR_INA230_ERROR;
    if (readBusVoltage_V(&data->bus_voltage_V) != TR_INA230_OK)
        return TR_INA230_ERROR;
    if (readCurrent_A(&data->current_A) != TR_INA230_OK)
        return TR_INA230_ERROR;
    if (readPower_W(&data->power_W) != TR_INA230_OK)
        return TR_INA230_ERROR;
    return TR_INA230_OK;
}

// ---------------------------------------------------------------------------
//  Alert
// ---------------------------------------------------------------------------
TR_INA230Status TR_INA230::setAlert(INA230_Alert func, uint16_t limit)
{
    uint16_t me = 0;
    if (readRegister(INA230_Reg::MASK_ENABLE, &me) != TR_INA230_OK)
        return TR_INA230_ERROR;
    me = (me & 0x07FF) | (uint16_t)func;
    if (writeRegister(INA230_Reg::MASK_ENABLE, me) != TR_INA230_OK)
        return TR_INA230_ERROR;
    return writeRegister(INA230_Reg::ALERT_LIMIT, limit);
}

TR_INA230Status TR_INA230::enableConversionReadyAlert(bool enable)
{
    uint16_t me = 0;
    if (readRegister(INA230_Reg::MASK_ENABLE, &me) != TR_INA230_OK)
        return TR_INA230_ERROR;
    if (enable)
        me |=  (1 << 10);
    else
        me &= ~(1 << 10);
    return writeRegister(INA230_Reg::MASK_ENABLE, me);
}

TR_INA230Status TR_INA230::readMaskEnable(uint16_t *value)
{
    return readRegister(INA230_Reg::MASK_ENABLE, value);
}

// ---------------------------------------------------------------------------
//  Die ID
// ---------------------------------------------------------------------------
TR_INA230Status TR_INA230::readDieId(uint16_t *id)
{
    return readRegister(INA230_Reg::DIE_ID, id);
}

// ---------------------------------------------------------------------------
//  Low-level I2C register access using ESP-IDF i2c_cmd_link API
//  INA230 registers are 16-bit, MSB first.
// ---------------------------------------------------------------------------
TR_INA230Status TR_INA230::writeRegister(uint8_t reg, uint16_t value)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (uint8_t)(value >> 8);    // MSByte
    buf[2] = (uint8_t)(value & 0xFF);  // LSByte

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 3, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    return (err == ESP_OK) ? TR_INA230_OK : TR_INA230_ERROR;
}

TR_INA230Status TR_INA230::readRegister(uint8_t reg, uint16_t *value)
{
    // Phase 1: write register pointer
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    // Phase 2: repeated start + read 2 bytes
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_READ, true);
    uint8_t data[2] = {0, 0};
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) return TR_INA230_ERROR;
    *value = ((uint16_t)data[0] << 8) | data[1];
    return TR_INA230_OK;
}
