#include <TR_MAX17303.h>

#include <math.h>

static constexpr uint32_t I2C_TIMEOUT_MS = 100;
static const char* TAG = "MAX17303";

namespace Reg = MAX17303_Reg;

TR_MAX17303::TR_MAX17303(uint8_t addr)
    : _dev(nullptr),
      _addr(addr),
      _devname(0),
      _cfg(),
      _data()
{
    _data.voltage       = NAN;
    _data.current       = NAN;
    _data.soc           = NAN;
    _data.temperature   = NAN;
    _data.capacity      = NAN;
    _data.full_capacity = NAN;
    _data.prot_status   = 0;
    _data.chg_fet_on    = false;
    _data.dis_fet_on    = false;
}

// ---------------------------------------------------------------------------
// Low-level I2C
// ---------------------------------------------------------------------------
bool TR_MAX17303::readReg(uint8_t reg, uint16_t& value)
{
    if (_dev == nullptr) return false;
    uint8_t buf[2] = {};
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, buf, 2,
                                                I2C_TIMEOUT_MS);
    if (err != ESP_OK) return false;
    value = ((uint16_t)buf[1] << 8) | buf[0];  // Little-endian on the wire
    return true;
}

bool TR_MAX17303::writeReg(uint8_t reg, uint16_t value)
{
    if (_dev == nullptr) return false;
    uint8_t buf[3] = { reg,
                       (uint8_t)(value & 0xFF),
                       (uint8_t)((value >> 8) & 0xFF) };
    return i2c_master_transmit(_dev, buf, 3, I2C_TIMEOUT_MS) == ESP_OK;
}

// ---------------------------------------------------------------------------
// begin — attach + identify (DevName splits us from a MAX17205 at 0x36)
// ---------------------------------------------------------------------------
esp_err_t TR_MAX17303::begin(i2c_master_bus_handle_t bus,
                             const TR_MAX17303_Config& cfg,
                             uint32_t clock_hz)
{
    _cfg = cfg;

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = _addr;
    dev_cfg.scl_speed_hz    = clock_hz;

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &_dev);
    if (err != ESP_OK) return err;

    if (!readReg(Reg::DEV_NAME, _devname))
    {
        i2c_master_bus_rm_device(_dev);
        _dev = nullptr;
        return ESP_FAIL;
    }

    // Family field: production silicon reads 0x406, older samples 0x404/
    // 0x405 — accept the whole 0x40x die-rev range. Variant nibble 7 =
    // MAX17303. A MAX17205 reads a ~0x02xx firmware-rev value here, so a
    // mismatch normally means "not this part" — hand the address back...
    // unless the board header asserts this PCB carries a MAX17303
    // (assume_when_unidentified): then claim it anyway and log the value,
    // so an unlisted die rev can't silently demote V3 to the wrong-map
    // MAX17205 driver (bench 2026-07-10: exactly that happened).
    const uint16_t family  = _devname >> 4;
    const uint16_t variant = _devname & 0xF;
    const bool is_1730x = (family & 0xFF0) == 0x400;
    if (!is_1730x)
    {
        if (!_cfg.assume_when_unidentified)
        {
            ESP_LOGI(TAG, "DevName 0x%04X is not a MAX1730x — handing 0x36 back",
                     _devname);
            i2c_master_bus_rm_device(_dev);
            _dev = nullptr;
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGW(TAG, "DevName 0x%04X unrecognized, but the board says this is "
                      "a MAX17303 — claiming it (report this value!)",
                 _devname);
    }
    else if (variant != Reg::DEVNAME_VARIANT_303)
    {
        // Sibling part (MAX17300/01/02) — same driver works; just say so.
        ESP_LOGW(TAG, "MAX1730x variant %u (DevName 0x%04X), driving as MAX17303",
                 (unsigned)variant, _devname);
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// DesignCap scaling — 1 LSB = 5 µVh / Rsense (0.5 mAh @ 10 mΩ)
// ---------------------------------------------------------------------------
uint16_t TR_MAX17303::designCapRaw() const
{
    float raw = (float)_cfg.design_mah * 2.0f * (10.0f / _cfg.rsense_mohm);
    if (raw < 0.0f)     raw = 0.0f;
    if (raw > 65535.0f) raw = 65535.0f;
    return (uint16_t)raw;
}

// ---------------------------------------------------------------------------
// initIfNeeded
// ---------------------------------------------------------------------------
esp_err_t TR_MAX17303::initIfNeeded()
{
    if (_dev == nullptr) return ESP_FAIL;

    uint16_t status = 0;
    if (!readReg(Reg::STATUS, status))
    {
        ESP_LOGE(TAG, "Status read failed during init check");
        return ESP_FAIL;
    }
    const bool por = (status & 0x0002);

    // The chip restores DesignCap from its NV copy at POR. If we have a
    // configured pack capacity and the restored value is >20% off (fresh
    // part still on factory NV), re-seed volatile RAM.
    bool cap_stale = false;
    if (_cfg.design_mah > 0)
    {
        const uint16_t expected = designCapRaw();
        uint16_t dc = 0;
        if (readReg(Reg::DESIGN_CAP, dc))
        {
            const uint32_t diff = (dc > expected) ? (dc - expected)
                                                  : (expected - dc);
            if (diff > (uint32_t)(expected / 5)) cap_stale = true;
        }
    }

    if (!por && !cap_stale) return ESP_OK;  // Quiet no-op path

    // Data-not-ready clears 0.4-1.9 s after POR; wait it out first.
    for (int i = 0; i < 200; ++i)
    {
        uint16_t fstat = 0;
        if (readReg(Reg::FSTAT, fstat) && !(fstat & 0x0001)) break;
        delay(10);
    }

    if (cap_stale)
    {
        seedCapacityAndRestart();
    }

    // Clear Status.POR so we don't re-run while the chip stays powered
    // across MCU reboots. Write-back preserves the other bits.
    uint16_t status_after = 0;
    if (readReg(Reg::STATUS, status_after))
    {
        writeReg(Reg::STATUS, status_after & ~(uint16_t)0x0002);
    }
    return ESP_OK;
}

// Volatile-RAM capacity seed (no NV writes, no Command-register use):
// write DesignCap/FullCapRep, then Config2.POR_CMD restarts the fuel-gauge
// firmware WITHOUT an NV recall so the model picks the new values up.
esp_err_t TR_MAX17303::seedCapacityAndRestart()
{
    const uint16_t design_raw = designCapRaw();

    writeReg(Reg::DESIGN_CAP,   design_raw);
    writeReg(Reg::FULL_CAP_REP, design_raw);

    uint16_t cfg2 = 0;
    if (!readReg(Reg::CONFIG2, cfg2)) return ESP_FAIL;
    writeReg(Reg::CONFIG2, cfg2 | 0x8000);  // POR_CMD, self-clearing

    bool restarted = false;
    for (int i = 0; i < 100; ++i)
    {
        if (readReg(Reg::CONFIG2, cfg2) && !(cfg2 & 0x8000))
        {
            restarted = true;
            break;
        }
        delay(10);
    }
    // Firmware restart re-runs data collection; wait for DNR again.
    for (int i = 0; i < 200; ++i)
    {
        uint16_t fstat = 0;
        if (readReg(Reg::FSTAT, fstat) && !(fstat & 0x0001)) break;
        delay(10);
    }

    uint16_t dc_back = 0;
    readReg(Reg::DESIGN_CAP, dc_back);
    if (dc_back != design_raw || !restarted)
    {
        ESP_LOGW(TAG, "capacity seed incomplete: DesignCap readback 0x%04X vs "
                      "wrote 0x%04X, restart=%s",
                 dc_back, design_raw, restarted ? "OK" : "TIMEOUT");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Seeded %u mAh pack (1S, Rsense %.1f mΩ) in volatile RAM",
             (unsigned)_cfg.design_mah, (double)_cfg.rsense_mohm);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// update
// ---------------------------------------------------------------------------
esp_err_t TR_MAX17303::update()
{
    if (_dev == nullptr) return ESP_FAIL;

    uint16_t raw = 0;

    // 1S part: VCell IS the pack voltage (78.125 µV/LSB).
    if (readReg(Reg::VCELL, raw))
        _data.voltage = (float)raw * 0.078125e-3f;

    // RepSOC (1/256 %) — the chip's reported SOC. Unlike the V1 MAX17205
    // board there is no known CSP/CSN swap, so no VFSOC workaround here.
    if (readReg(Reg::REP_SOC, raw))
        _data.soc = (float)raw / 256.0f;

    if (readReg(Reg::CURRENT, raw))
    {
        float cur_ma = (float)(int16_t)raw * 1.5625e-3f / (_cfg.rsense_mohm * 1e-3f);
        _data.current = _cfg.current_invert ? -cur_ma : cur_ma;
    }

    if (readReg(Reg::TEMP, raw))
        _data.temperature = (float)(int16_t)raw / 256.0f;

    const float cap_lsb_mah = 0.5f * (10.0f / _cfg.rsense_mohm);
    if (readReg(Reg::REP_CAP, raw))
        _data.capacity = (float)raw * cap_lsb_mah;
    if (readReg(Reg::FULL_CAP_REP, raw))
        _data.full_capacity = (float)raw * cap_lsb_mah;

    // Protector observability (the BQ FET saga lesson: always know why the
    // battery path is open).
    if (readReg(Reg::PROT_STATUS, raw))
        _data.prot_status = raw;
    if (readReg(Reg::HCONFIG2, raw))
    {
        _data.chg_fet_on = (raw & (1u << 6)) != 0;  // CHGs
        _data.dis_fet_on = (raw & (1u << 7)) != 0;  // DISs
    }

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
// Decode ProtStatus fault bits (datasheet Table: ProtStatus 0x0D9).
static void protStatusToString(uint16_t ps, char* out, size_t out_len)
{
    static const struct { uint16_t bit; const char* name; } kBits[] = {
        {1u << 0,  "Shdn"},    {1u << 1,  "TooColdD"}, {1u << 2,  "ODCP"},
        {1u << 3,  "UVP"},     {1u << 4,  "TooHotD"},  {1u << 5,  "DieHot"},
        {1u << 6,  "PermFail"},{1u << 9,  "Qovflw"},   {1u << 10, "OCCP"},
        {1u << 11, "OVP"},     {1u << 12, "TooColdC"}, {1u << 13, "Full"},
        {1u << 14, "TooHotC"}, {1u << 15, "ChgWDT"},
    };
    out[0] = '\0';
    if (ps == 0)
    {
        snprintf(out, out_len, "none");
        return;
    }
    size_t used = 0;
    for (const auto& b : kBits)
    {
        if (!(ps & b.bit)) continue;
        used += snprintf(out + used, (used < out_len) ? out_len - used : 0,
                         "%s%s", (used > 0) ? "," : "", b.name);
    }
}

void TR_MAX17303::logDiagnostics(const char* log_tag)
{
    if (_dev == nullptr) return;
    uint16_t raw = 0;
    const float cap_lsb_mah = 0.5f * (10.0f / _cfg.rsense_mohm);

    ESP_LOGI(log_tag, "[FG-DIAG] --- MAX17303 register dump ---");
    ESP_LOGI(log_tag, "[FG-DIAG] DevName    (0x21) = 0x%04X", _devname);
    if (readReg(Reg::STATUS,       raw)) ESP_LOGI(log_tag, "[FG-DIAG] Status     (0x00) = 0x%04X", raw);
    if (readReg(Reg::REP_SOC,      raw)) ESP_LOGI(log_tag, "[FG-DIAG] RepSOC     (0x06) = %.2f %% (raw=0x%04X)", (float)raw / 256.0f, raw);
    if (readReg(Reg::REP_CAP,      raw)) ESP_LOGI(log_tag, "[FG-DIAG] RepCap     (0x05) = %.0f mAh (raw=0x%04X)", raw * cap_lsb_mah, raw);
    if (readReg(Reg::FULL_CAP_REP, raw)) ESP_LOGI(log_tag, "[FG-DIAG] FullCapRep (0x10) = %.0f mAh (raw=0x%04X)", raw * cap_lsb_mah, raw);
    if (readReg(Reg::DESIGN_CAP,   raw)) ESP_LOGI(log_tag, "[FG-DIAG] DesignCap  (0x18) = %.0f mAh (raw=0x%04X)", raw * cap_lsb_mah, raw);
    if (readReg(Reg::CYCLES,       raw)) ESP_LOGI(log_tag, "[FG-DIAG] Cycles     (0x17) = %.2f (25%%/LSB)", (float)raw * 0.25f);
    if (readReg(Reg::VCELL,        raw)) ESP_LOGI(log_tag, "[FG-DIAG] VCell      (0x1A) = %.1f mV (raw=0x%04X)", (double)((float)raw * 0.078125f), raw);
    if (readReg(Reg::CURRENT,      raw)) {
        const int16_t signed_raw = (int16_t)raw;
        const float ma = (float)signed_raw * 1.5625e-3f / (_cfg.rsense_mohm * 1e-3f);
        ESP_LOGI(log_tag, "[FG-DIAG] Current    (0x1C) = %d (raw signed) -> %.0f mA chip-frame, %.0f mA app-frame (invert=%d)",
                 signed_raw, (double)ma,
                 _cfg.current_invert ? -(double)ma : (double)ma,
                 _cfg.current_invert);
    }
    if (readReg(Reg::TEMP,         raw)) ESP_LOGI(log_tag, "[FG-DIAG] Temp       (0x1B) = %.1f C", (float)(int16_t)raw / 256.0f);
    if (readReg(Reg::PROT_STATUS,  raw)) {
        char faults[96];
        protStatusToString(raw, faults, sizeof(faults));
        ESP_LOGI(log_tag, "[FG-DIAG] ProtStatus (0xD9) = 0x%04X [%s]", raw, faults);
    }
    if (readReg(Reg::HCONFIG2,     raw)) {
        ESP_LOGI(log_tag, "[FG-DIAG] HConfig2   (0xF5) = 0x%04X  CHG FET=%s DIS FET=%s",
                 raw, (raw & (1u << 6)) ? "ON" : "OFF",
                 (raw & (1u << 7)) ? "ON" : "OFF");
    }
    ESP_LOGI(log_tag, "[FG-DIAG] Pack: 1S, design %u mAh, Rsense %.1f mΩ",
             (unsigned)_cfg.design_mah, (double)_cfg.rsense_mohm);
    ESP_LOGI(log_tag, "[FG-DIAG] --------------------------------");
}
