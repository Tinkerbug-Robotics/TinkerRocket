#include <TR_BQ27Z746.h>

#include <esp_log.h>
#include <math.h>

// i2c_master_*() take an int timeout in MILLISECONDS (not ticks).
static constexpr int I2C_TIMEOUT_MS = 100;

namespace Reg = BQ27Z746_Reg;
static const char* DTAG = "BQ27Z746";

TR_BQ27Z746::TR_BQ27Z746(uint8_t addr)
    : _dev(nullptr),
      _addr(addr),
      _device_type(0),
      _cfg(),
      _data()
{
    _data.voltage       = NAN;
    _data.current       = NAN;
    _data.soc           = NAN;
    _data.temperature   = NAN;
    _data.capacity      = NAN;
    _data.full_capacity = NAN;
    _data.batt_status   = 0;
    _data.chg_fet_on    = false;
    _data.dsg_fet_on    = false;
    _data.fet_en        = false;
    _data.safety_active = false;
}

// ---------------------------------------------------------------------------
// Low-level I2C
// ---------------------------------------------------------------------------
bool TR_BQ27Z746::readWord(uint8_t reg, uint16_t& value)
{
    if (_dev == nullptr) return false;
    uint8_t buf[2] = {};
    if (i2c_master_transmit_receive(_dev, &reg, 1, buf, 2, I2C_TIMEOUT_MS) != ESP_OK)
        return false;
    value = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);  // little-endian on the wire
    return true;
}

// Write a 16-bit subcommand to AltManufacturerAccess (no data payload, e.g. a toggle).
esp_err_t TR_BQ27Z746::macCmd(uint16_t sub)
{
    if (_dev == nullptr) return ESP_FAIL;
    uint8_t cmd[3] = { Reg::ALT_MFR_ACCESS, (uint8_t)(sub & 0xFF), (uint8_t)(sub >> 8) };
    return i2c_master_transmit(_dev, cmd, sizeof(cmd), I2C_TIMEOUT_MS);
}

// Write a subcommand, then read the response block back from MACData (0x40).
esp_err_t TR_BQ27Z746::macRead(uint16_t sub, uint8_t* blk, size_t n)
{
    esp_err_t e = macCmd(sub);
    if (e != ESP_OK) return e;
    vTaskDelay(pdMS_TO_TICKS(5));            // let the gauge stage the result
    uint8_t reg = Reg::MAC_DATA;
    return i2c_master_transmit_receive(_dev, &reg, 1, blk, n, I2C_TIMEOUT_MS);
}

// ---------------------------------------------------------------------------
// begin
// ---------------------------------------------------------------------------
esp_err_t TR_BQ27Z746::begin(i2c_master_bus_handle_t bus,
                             const TR_BQ27Z746_Config& cfg,
                             uint32_t clock_hz)
{
    _cfg = cfg;

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = _addr;
    dev_cfg.scl_speed_hz    = clock_hz;

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &_dev);
    if (err != ESP_OK) return err;

    // Probe: a Voltage read must succeed.
    uint16_t v = 0;
    if (!readWord(Reg::VOLTAGE, v))
    {
        i2c_master_bus_rm_device(_dev);
        _dev = nullptr;
        return ESP_FAIL;
    }

    // Positive ID (best-effort).
    uint8_t blk[4] = {};
    if (macRead(Reg::SUB_DEVICE_TYPE, blk, sizeof(blk)) == ESP_OK)
        _device_type = (uint16_t)blk[0] | ((uint16_t)blk[1] << 8);

    if (_cfg.auto_enable_fets)
        enableFetsIfNeeded();

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// FET / protection status
// ---------------------------------------------------------------------------
esp_err_t TR_BQ27Z746::readFetStatus()
{
    uint8_t op[4] = {}, mf[4] = {};
    bool ok = (macRead(Reg::SUB_OP_STATUS,  op, 4) == ESP_OK);
    ok      = (macRead(Reg::SUB_MFG_STATUS, mf, 4) == ESP_OK) && ok;
    if (!ok) return ESP_FAIL;

    uint16_t opA = (uint16_t)op[0] | ((uint16_t)op[1] << 8);
    uint16_t mfg = (uint16_t)mf[0] | ((uint16_t)mf[1] << 8);
    _data.chg_fet_on    = !(opA & Reg::OPA_XCHG);
    _data.dsg_fet_on    = !(opA & Reg::OPA_XDSG);
    _data.safety_active = (opA & Reg::OPA_SS) != 0;
    _data.fet_en        = (mfg & Reg::MFG_FET_EN) != 0;
    return ESP_OK;
}

esp_err_t TR_BQ27Z746::enableFetsIfNeeded()
{
    if (_dev == nullptr) return ESP_FAIL;
    if (readFetStatus() != ESP_OK) return ESP_FAIL;

    if (_data.safety_active)
    {
        ESP_LOGW(DTAG, "Safety protection active -- refusing to enable FETs; clear fault first");
        return ESP_FAIL;
    }
    if (_data.fet_en)
        return ESP_OK;   // already enabled (persists in data flash)

    ESP_LOGW(DTAG, "FET_EN=0 -> sending AltManufacturerAccess(0x0022) FET Enable");
    esp_err_t e = macCmd(Reg::SUB_FET_ENABLE);
    if (e != ESP_OK) return e;
    vTaskDelay(pdMS_TO_TICKS(150));
    readFetStatus();

    if (_data.fet_en && _data.chg_fet_on && _data.dsg_fet_on)
    {
        ESP_LOGI(DTAG, "FETs enabled (FET_EN=1, CHG+DSG on)");
        return ESP_OK;
    }
    ESP_LOGW(DTAG, "FET enable toggled but not fully on (FET_EN=%d CHG=%d DSG=%d) -- check gate path",
             _data.fet_en, _data.chg_fet_on, _data.dsg_fet_on);
    return ESP_FAIL;
}

// ---------------------------------------------------------------------------
// update
// ---------------------------------------------------------------------------
esp_err_t TR_BQ27Z746::update()
{
    if (_dev == nullptr) return ESP_FAIL;

    uint16_t v = 0, t = 0, st = 0, rc = 0, fc = 0, soc = 0, cur_raw = 0;
    bool ok = true;
    ok &= readWord(Reg::VOLTAGE,     v);
    ok &= readWord(Reg::TEMPERATURE, t);
    ok &= readWord(Reg::BATT_STATUS, st);
    ok &= readWord(Reg::CURRENT,     cur_raw);
    ok &= readWord(Reg::REMAIN_CAP,  rc);
    ok &= readWord(Reg::FULL_CAP,    fc);
    ok &= readWord(Reg::RSOC,        soc);
    if (!ok) return ESP_FAIL;

    int16_t cur = (int16_t)cur_raw;          // signed mA
    _data.voltage       = v / 1000.0f;
    _data.temperature   = t / 10.0f - 273.15f;
    _data.current       = _cfg.current_invert ? -(float)cur : (float)cur;
    _data.capacity      = (float)rc;
    _data.full_capacity = (float)fc;
    _data.soc           = (float)soc;
    _data.batt_status   = st;

    readFetStatus();   // best-effort refresh of FET flags
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// logDiagnostics
// ---------------------------------------------------------------------------
void TR_BQ27Z746::logDiagnostics(const char* log_tag)
{
    const char* tag = log_tag ? log_tag : DTAG;
    update();
    ESP_LOGI(tag, "BQ27Z746 devtype=0x%04X  V=%.3f I=%.0fmA SOC=%.0f%% T=%.1fC Rem=%.0f/%.0f mAh",
             _device_type, (double)_data.voltage, (double)_data.current, (double)_data.soc,
             (double)_data.temperature, (double)_data.capacity, (double)_data.full_capacity);
    ESP_LOGI(tag, "  BattStatus=0x%04X  FET_EN=%d CHG=%d DSG=%d SafetyActive=%d",
             _data.batt_status, _data.fet_en, _data.chg_fet_on, _data.dsg_fet_on, _data.safety_active);
}
