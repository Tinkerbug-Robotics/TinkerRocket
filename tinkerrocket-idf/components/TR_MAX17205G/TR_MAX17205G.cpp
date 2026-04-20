#include <TR_MAX17205G.h>

#include <math.h>

static constexpr uint32_t I2C_TIMEOUT_MS = 100;

namespace Reg = MAX17205_Reg;

TR_MAX17205G::TR_MAX17205G(uint8_t addr)
    : _dev(nullptr),
      _addr(addr),
      _cfg(),
      _data()
{
    _data.voltage       = NAN;
    _data.current       = NAN;
    _data.soc           = NAN;
    _data.temperature   = NAN;
    _data.capacity      = NAN;
    _data.full_capacity = NAN;
}

// ---------------------------------------------------------------------------
// Low-level I2C
// ---------------------------------------------------------------------------
bool TR_MAX17205G::readReg(uint8_t reg, uint16_t& value)
{
    if (_dev == nullptr) return false;
    uint8_t buf[2] = {};
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, buf, 2,
                                                pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return false;
    value = ((uint16_t)buf[1] << 8) | buf[0];  // Little-endian on the wire
    return true;
}

bool TR_MAX17205G::writeReg(uint8_t reg, uint16_t value)
{
    if (_dev == nullptr) return false;
    uint8_t buf[3] = { reg,
                       (uint8_t)(value & 0xFF),
                       (uint8_t)((value >> 8) & 0xFF) };
    return i2c_master_transmit(_dev, buf, 3, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == ESP_OK;
}

// ---------------------------------------------------------------------------
// begin
// ---------------------------------------------------------------------------
esp_err_t TR_MAX17205G::begin(i2c_master_bus_handle_t bus,
                              const TR_MAX17205G_Config& cfg,
                              uint32_t clock_hz)
{
    _cfg = cfg;

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = _addr;
    dev_cfg.scl_speed_hz    = clock_hz;

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &_dev);
    if (err != ESP_OK) return err;

    // Probe: most registers are always readable, pick Status (0x00).
    uint16_t probe = 0;
    if (!readReg(Reg::STATUS, probe))
    {
        i2c_master_bus_rm_device(_dev);
        _dev = nullptr;
        return ESP_FAIL;
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// DesignCap scaling
// ---------------------------------------------------------------------------
// Capacity registers use 1 LSB = 5 µVh / Rsense.
// For Rsense = 10 mΩ: 1 LSB = 0.5 mAh, so raw = mAh * 2.
// For generic Rsense: raw = mAh * 2 * (10 / rsense_mohm).
uint16_t TR_MAX17205G::designCapRaw() const
{
    float raw = (float)_cfg.design_mah * 2.0f * (10.0f / _cfg.rsense_mohm);
    if (raw < 0.0f)       raw = 0.0f;
    if (raw > 65535.0f)   raw = 65535.0f;
    return (uint16_t)raw;
}

// ---------------------------------------------------------------------------
// initIfNeeded
// ---------------------------------------------------------------------------
esp_err_t TR_MAX17205G::initIfNeeded()
{
    if (_dev == nullptr) return ESP_FAIL;

    uint16_t status = 0;
    if (!readReg(Reg::STATUS, status))
    {
        ESP_LOGE("MAX17205", "Status read failed during init check");
        return ESP_FAIL;
    }

    bool por = (status & 0x0002);

    // Sanity check: if FullCapNom is more than 20% off from our configured
    // DesignCap, assume the chip is stuck on a stale NV value and re-init.
    bool cap_stale = false;
    uint16_t expected = designCapRaw();
    uint16_t fcn = 0;
    if (readReg(Reg::FULL_CAP_NOM, fcn))
    {
        uint32_t diff = (fcn > expected) ? (fcn - expected) : (expected - fcn);
        if (diff > (uint32_t)(expected / 5)) cap_stale = true;
    }

    if (!por && !cap_stale) return ESP_OK;  // Quiet no-op path
    return runEzInit();
}

esp_err_t TR_MAX17205G::forceReinit()
{
    if (_dev == nullptr) return ESP_FAIL;
    return runEzInit();
}

// ---------------------------------------------------------------------------
// runEzInit — ModelGauge m5 "EZ" configuration + capacity override
//
// The datasheet's recommended init writes DesignCap and triggers a ModelCfg
// refresh. By itself, that isn't enough when the chip has a stale nFullCapNom
// in its nonvolatile backup (which our parts do — they restore FullCapNom to
// ~655 mAh on every POR regardless of DesignCap). After the refresh we also
// directly write FullCap / FullCapRep / FullCapNom / RepCap in volatile RAM
// so RepSOC computes against the correct pack capacity from the very first
// sample. Volatile writes repeat on each cold boot — no NV write budget spent.
// ---------------------------------------------------------------------------
esp_err_t TR_MAX17205G::runEzInit()
{
    const uint16_t design_raw = designCapRaw();

    // 1. Wait for FStat.DNR to clear — up to ~2 s after POR
    for (int i = 0; i < 200; ++i)
    {
        uint16_t fstat = 0;
        if (readReg(Reg::FSTAT, fstat) && !(fstat & 0x0001)) break;
        delay(10);
    }

    // 2. Save HibCfg, pull chip out of hibernate so writes take effect
    uint16_t orig_hibcfg = 0;
    readReg(Reg::HIB_CFG, orig_hibcfg);
    writeReg(Reg::SOFT_WAKEUP, 0x0090);
    writeReg(Reg::HIB_CFG, 0x0000);
    writeReg(Reg::SOFT_WAKEUP, 0x0000);
    delay(10);

    // 3. Seed model inputs
    writeReg(Reg::DESIGN_CAP, design_raw);
    writeReg(Reg::ICHG_TERM,  0x0640);   // ~250 mA at 10 mΩ — default is fine
    writeReg(Reg::V_EMPTY,    0x9661);   // VE=3000 mV, VR=3880 mV (standard Li-ion)

    // 4. Trigger ModelCfg.Refresh with default 4.2 V Li-ion chemistry
    writeReg(Reg::MODEL_CFG, 0x8000);
    bool refresh_ok = false;
    for (int i = 0; i < 200; ++i)
    {
        uint16_t mcfg = 0;
        if (readReg(Reg::MODEL_CFG, mcfg) && !(mcfg & 0x8000))
        {
            refresh_ok = true;
            break;
        }
        delay(10);
    }

    // 5. Override stuck capacity registers (volatile — bypasses bad NV state)
    writeReg(Reg::FULL_CAP,     design_raw);
    writeReg(Reg::FULL_CAP_REP, design_raw);
    writeReg(Reg::FULL_CAP_NOM, design_raw);

    // 6. Seed RepCap from VFSOC × FullCap so RepSOC starts sensible.
    //    VFSOC is in 1/256 %, so the full-scale raw is 25600 (not 65536).
    //    RepCap_raw = FullCap_raw × VFSOC_raw / 25600.
    uint16_t vfsoc = 0;
    if (readReg(Reg::VFSOC, vfsoc))
    {
        uint32_t rep_cap = ((uint32_t)design_raw * (uint32_t)vfsoc) / 25600U;
        if (rep_cap > design_raw) rep_cap = design_raw;
        writeReg(Reg::REP_CAP, (uint16_t)rep_cap);
    }

    // 7. Restore HibCfg
    writeReg(Reg::HIB_CFG, orig_hibcfg);

    // 8. Clear Status.POR (bit 1), Br (bit 15), Bi (bit 11) so we don't re-run
    //    init on every MCU reboot while the chip stays battery-powered
    uint16_t status_after = 0;
    if (readReg(Reg::STATUS, status_after))
    {
        writeReg(Reg::STATUS, status_after & ~((uint16_t)0x8802));
    }

    // Verify writes landed; warn loudly only if they didn't.
    uint16_t dc_back = 0;
    readReg(Reg::DESIGN_CAP, dc_back);
    if (dc_back != design_raw || !refresh_ok)
    {
        ESP_LOGW("MAX17205",
                 "EZ init incomplete: DesignCap readback 0x%04X vs wrote 0x%04X, refresh=%s",
                 dc_back, design_raw, refresh_ok ? "OK" : "TIMEOUT");
    }
    else
    {
        ESP_LOGI("MAX17205", "Configured for %u mAh pack (%u cells, Rsense %.1f mΩ)",
                 (unsigned)_cfg.design_mah,
                 (unsigned)_cfg.num_cells,
                 (double)_cfg.rsense_mohm);
    }

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// update
// ---------------------------------------------------------------------------
esp_err_t TR_MAX17205G::update()
{
    if (_dev == nullptr) return ESP_FAIL;

    uint16_t raw = 0;

    // Pack voltage: VCell is per-cell voltage (0.078125 mV/LSB).
    if (readReg(Reg::VCELL, raw))
        _data.voltage = (float)raw * 0.078125e-3f * (float)_cfg.num_cells;

    // RepSOC: 1/256 % per LSB
    if (readReg(Reg::REP_SOC, raw))
        _data.soc = (float)raw / 256.0f;

    // Current: signed, 1.5625 µV per LSB / Rsense. Result in mA.
    if (readReg(Reg::CURRENT, raw))
    {
        float cur_ma = (float)(int16_t)raw * 1.5625e-3f / (_cfg.rsense_mohm * 1e-3f);
        _data.current = _cfg.current_invert ? -cur_ma : cur_ma;
    }

    // Temperature: signed, 1/256 °C per LSB
    if (readReg(Reg::TEMP, raw))
        _data.temperature = (float)(int16_t)raw / 256.0f;

    // Remaining capacity: 0.5 mAh/LSB at 10 mΩ, scales with rsense
    if (readReg(Reg::REP_CAP, raw))
        _data.capacity = (float)raw * 0.5f * (10.0f / _cfg.rsense_mohm);

    // Full capacity (learned)
    if (readReg(Reg::FULL_CAP_REP, raw))
        _data.full_capacity = (float)raw * 0.5f * (10.0f / _cfg.rsense_mohm);

    return ESP_OK;
}

// ---------------------------------------------------------------------------
// logDiagnostics — one-shot register dump for troubleshooting
// ---------------------------------------------------------------------------
void TR_MAX17205G::logDiagnostics(const char* log_tag)
{
    if (_dev == nullptr) return;
    uint16_t raw = 0;
    auto pct = [](uint16_t r) { return (float)r / 256.0f; };
    const float cap_lsb_mah = 0.5f * (10.0f / _cfg.rsense_mohm);

    ESP_LOGI(log_tag, "[FG-DIAG] --- MAX17205 register dump ---");
    if (readReg(Reg::STATUS,       raw)) ESP_LOGI(log_tag, "[FG-DIAG] Status     (0x00) = 0x%04X", raw);
    if (readReg(Reg::REP_SOC,      raw)) ESP_LOGI(log_tag, "[FG-DIAG] RepSOC     (0x06) = %.2f %% (raw=0x%04X)", pct(raw), raw);
    if (readReg(Reg::MIX_SOC,      raw)) ESP_LOGI(log_tag, "[FG-DIAG] MixSOC     (0x0D) = %.2f %% (raw=0x%04X)", pct(raw), raw);
    if (readReg(Reg::AV_SOC,       raw)) ESP_LOGI(log_tag, "[FG-DIAG] AvSOC      (0x0E) = %.2f %% (raw=0x%04X)", pct(raw), raw);
    if (readReg(Reg::VFSOC,        raw)) ESP_LOGI(log_tag, "[FG-DIAG] VFSOC      (0xFF) = %.2f %% (raw=0x%04X)", pct(raw), raw);
    if (readReg(Reg::REP_CAP,      raw)) ESP_LOGI(log_tag, "[FG-DIAG] RepCap     (0x05) = %.0f mAh (raw=0x%04X)", raw * cap_lsb_mah, raw);
    if (readReg(Reg::FULL_CAP,     raw)) ESP_LOGI(log_tag, "[FG-DIAG] FullCap    (0x10) = %.0f mAh (raw=0x%04X)", raw * cap_lsb_mah, raw);
    if (readReg(Reg::FULL_CAP_REP, raw)) ESP_LOGI(log_tag, "[FG-DIAG] FullCapRep (0x35) = %.0f mAh (raw=0x%04X)", raw * cap_lsb_mah, raw);
    if (readReg(Reg::FULL_CAP_NOM, raw)) ESP_LOGI(log_tag, "[FG-DIAG] FullCapNom (0x23) = %.0f mAh (raw=0x%04X)", raw * cap_lsb_mah, raw);
    if (readReg(Reg::DESIGN_CAP,   raw)) ESP_LOGI(log_tag, "[FG-DIAG] DesignCap  (0x18) = %.0f mAh (raw=0x%04X)", raw * cap_lsb_mah, raw);
    if (readReg(Reg::PACK_CFG,     raw)) ESP_LOGI(log_tag, "[FG-DIAG] PackCfg    (0xBD) = 0x%04X", raw);
    ESP_LOGI(log_tag, "[FG-DIAG] Pack: %u cells series, design %u mAh, Rsense %.1f mΩ",
             (unsigned)_cfg.num_cells, (unsigned)_cfg.design_mah, (double)_cfg.rsense_mohm);
    ESP_LOGI(log_tag, "[FG-DIAG] --------------------------------");
}
