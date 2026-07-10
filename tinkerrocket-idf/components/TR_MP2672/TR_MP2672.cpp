#include <TR_MP2672.h>

static constexpr uint32_t I2C_TIMEOUT_MS = 100;
static const char* TAG = "MP2672";

namespace Reg = MP2672_Reg;

TR_MP2672::TR_MP2672(uint8_t addr)
    : _dev(nullptr),
      _addr(addr),
      _cfg(),
      _data(),
      _config_applied(false)
{
}

// ---------------------------------------------------------------------------
// Low-level I2C — single-byte transactions only (see header)
// ---------------------------------------------------------------------------
bool TR_MP2672::readReg(uint8_t reg, uint8_t& value)
{
    if (_dev == nullptr) return false;
    return i2c_master_transmit_receive(_dev, &reg, 1, &value, 1,
                                       I2C_TIMEOUT_MS) == ESP_OK;
}

bool TR_MP2672::writeReg(uint8_t reg, uint8_t value)
{
    if (_dev == nullptr) return false;
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(_dev, buf, 2, I2C_TIMEOUT_MS) == ESP_OK;
}

// ---------------------------------------------------------------------------
// Desired register images
// ---------------------------------------------------------------------------
uint8_t TR_MP2672::desiredReg00() const
{
    // VBATT_PRE fixed at the -0000 default 100b (6.4 V pack / 3.2 V per
    // cell); bit0 kept 0 (meaning differs between MP2672 and MP2672A —
    // 0 is safe on both).
    uint8_t vreg = _cfg.vbatt_reg_code;
    if (vreg > 6) vreg = 1;  // code 7 is variant-ambiguous (9.0 vs 8.2 V) — refuse
    return (uint8_t)((vreg << 5) |
                     (_cfg.charge_enable ? (1u << 4) : 0) |
                     (0x4 << 1));
}

uint8_t TR_MP2672::desiredReg01() const
{
    // NTC_TYPE = JEITA (the -0000 default; harmless with the fixed NTC
    // divider), balance thresholds at defaults (3.5 V start, 50 mV).
    return (uint8_t)(0x80 | (_cfg.icc_code & 0x0F));
}

uint8_t TR_MP2672::desiredReg02() const
{
    // FSW 1.2 MHz (default), watchdog ON at 40 s (host-crash posture: on
    // expiry the chip reverts to 8.4 V / RISET-clamped defaults and keeps
    // charging safely), safety timer per config, bit0 = 1 (default on both
    // variants). Kick bit (6) is OR'd in by service().
    return (uint8_t)((1u << 7) | (0x1 << 4) |
                     ((_cfg.chg_timer_code & 0x3) << 1) | 0x01);
}

// ---------------------------------------------------------------------------
// applyConfig — write + readback-verify (silent-write-refusal field reports)
// ---------------------------------------------------------------------------
bool TR_MP2672::applyConfig()
{
    struct { uint8_t reg; uint8_t val; } seq[] = {
        { Reg::REG01, desiredReg01() },
        { Reg::REG02, desiredReg02() },
        { Reg::REG00, desiredReg00() },  // enable last, once limits are in
    };
    bool ok = true;
    for (const auto& w : seq)
    {
        uint8_t back = 0xFF;
        if (!writeReg(w.reg, w.val) || !readReg(w.reg, back) || back != w.val)
        {
            // The kick bit self-behavior is undocumented; exclude bit6 from
            // the REG02 comparison.
            if (w.reg == Reg::REG02 && (back & ~0x40) == (w.val & ~0x40))
                continue;
            ESP_LOGW(TAG, "REG%02X write not verified (wrote 0x%02X, read 0x%02X)",
                     w.reg, w.val, back);
            _data.write_verify_fails++;
            ok = false;
        }
    }
    if (ok)
    {
        _data.config_applies++;
        ESP_LOGI(TAG, "config applied: REG00=0x%02X REG01=0x%02X REG02=0x%02X "
                      "(%s, vreg code %u, ICC code %u)",
                 desiredReg00(), desiredReg01(), desiredReg02(),
                 _cfg.charge_enable ? "charge enabled" : "charge DISABLED",
                 _cfg.vbatt_reg_code, _cfg.icc_code);
    }
    return ok;
}

// ---------------------------------------------------------------------------
// begin
// ---------------------------------------------------------------------------
esp_err_t TR_MP2672::begin(i2c_master_bus_handle_t bus,
                           const TR_MP2672_Config& cfg,
                           uint32_t clock_hz)
{
    _cfg = cfg;

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = _addr;
    dev_cfg.scl_speed_hz    = clock_hz;

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &_dev);
    if (err != ESP_OK) return err;

    // First reconcile now — detects whether a charge input is already
    // attached and configures immediately if so.
    service();
    ESP_LOGI(TAG, "pack charger driver up (0x%02X): %s", _addr,
             _data.present ? "chip answering (input present)"
                           : "no answer — charge input absent (normal)");
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// service — the reconcile pass
// ---------------------------------------------------------------------------
void TR_MP2672::service()
{
    if (_dev == nullptr) return;

    const bool was_present = _data.present;
    uint8_t r00 = 0;
    if (!readReg(Reg::REG00, r00))
    {
        // Chip unpowered (VCC LDO runs from VIN): input absent — not an
        // error. All config was lost; re-apply on next appearance.
        _data.present = false;
        _config_applied = false;
        if (was_present)
        {
            ESP_LOGI(TAG, "charge input removed (chip unpowered)");
            _data.state = MP2672ChargeState::NotCharging;
            _data.faults = 0;
            _data.status_raw = 0;
        }
        return;
    }
    _data.present = true;
    if (!was_present)
    {
        ESP_LOGI(TAG, "charge input attached (REG00=0x%02X on wake)", r00);
    }

    // (Re)apply config on first contact after plug-in, or when the live
    // REG00 drifted from desired (watchdog expiry silently reverts it).
    if (!_config_applied || r00 != desiredReg00())
    {
        _config_applied = applyConfig();
    }
    else
    {
        // Kick the 40 s watchdog by rewriting REG02 with the kick bit.
        writeReg(Reg::REG02, (uint8_t)(desiredReg02() | 0x40));
    }

    uint8_t r03 = 0, r04 = 0;
    if (readReg(Reg::REG03, r03))
    {
        const auto new_state = (MP2672ChargeState)((r03 & Reg::CHG_STAT_MASK)
                                                   >> Reg::CHG_STAT_SHIFT);
        if (new_state != _data.state)
        {
            ESP_LOGI(TAG, "charge state: %s -> %s (REG03=0x%02X)",
                     stateName(_data.state), stateName(new_state), r03);
        }
        _data.state          = new_state;
        _data.status_raw     = r03;
        _data.battery_absent = (r03 & Reg::BATTFLOAT_STAT) != 0;
        _data.in_ppm         = (r03 & Reg::PPM_STAT) != 0;
        _data.in_thermal_reg = (r03 & Reg::THERM_STAT) != 0;
        _data.in_vsys_reg    = (r03 & Reg::VSYS_STAT) != 0;
    }
    if (readReg(Reg::REG04, r04))
    {
        if (r04 != _data.faults && r04 != 0)
        {
            ESP_LOGW(TAG, "faults 0x%02X:%s%s%s%s%s ntc=%u", r04,
                     (r04 & Reg::WD_FAULT)      ? " WD"       : "",
                     (r04 & Reg::INPUT_FAULT)   ? " INPUT_OVP": "",
                     (r04 & Reg::THERMSD_FAULT) ? " THERM_SD" : "",
                     (r04 & Reg::TIMER_FAULT)   ? " SAFETY_TMR" : "",
                     (r04 & Reg::BAT_FAULT)     ? " BATT_OVP" : "",
                     (unsigned)(r04 & Reg::NTC_FAULT_MASK));
        }
        else if (r04 == 0 && _data.faults != 0)
        {
            ESP_LOGI(TAG, "faults cleared");
        }
        if (r04 & Reg::WD_FAULT)
        {
            // Watchdog expired since last pass — registers reverted; force a
            // config re-apply next block above on the following service().
            _config_applied = false;
        }
        _data.faults = r04;
    }
}

const char* TR_MP2672::stateName(MP2672ChargeState s)
{
    switch (s)
    {
        case MP2672ChargeState::NotCharging: return "not-charging";
        case MP2672ChargeState::PreCharge:   return "pre-charge";
        case MP2672ChargeState::FastCharge:  return "fast-charge (CC/CV)";
        case MP2672ChargeState::Done:        return "done";
    }
    return "?";
}

void TR_MP2672::logDiagnostics(const char* log_tag)
{
    if (_dev == nullptr) return;
    uint8_t v = 0;
    ESP_LOGI(log_tag, "[CHG-DIAG] --- MP2672 register dump ---");
    if (!_data.present)
    {
        ESP_LOGI(log_tag, "[CHG-DIAG] chip unpowered (no charge input)");
        return;
    }
    for (uint8_t r = Reg::REG00; r <= Reg::REG04; ++r)
    {
        if (readReg(r, v))
            ESP_LOGI(log_tag, "[CHG-DIAG] REG%02X = 0x%02X", r, v);
    }
    ESP_LOGI(log_tag, "[CHG-DIAG] state=%s battery_absent=%d ppm=%d therm=%d "
                      "vsys=%d applies=%lu verify_fails=%lu",
             stateName(_data.state), (int)_data.battery_absent,
             (int)_data.in_ppm, (int)_data.in_thermal_reg,
             (int)_data.in_vsys_reg,
             (unsigned long)_data.config_applies,
             (unsigned long)_data.write_verify_fails);
    ESP_LOGI(log_tag, "[CHG-DIAG] --------------------------------");
}
