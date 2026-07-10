#ifndef TR_MP2672_H
#define TR_MP2672_H

#include <compat.h>
#include <driver/i2c_master.h>

// MPS MP2672 (MP2672GD-0000-Z) — 2-cell-series Li-ion boost charger with
// cell balancing, used on the V3 base station to charge external 2S rocket
// flight packs from the 5 V input. I2C 7-bit address 0x4B, five host
// registers 0x00-0x04 (REG05/06 are factory OTP, not accessible).
//
// Driver shape is dictated by three datasheet realities (MP2672 Rev 0.2
// preliminary + MP2672A Rev 1.0 — "mostly interchangeable" per MPS):
//  1. VOLATILE CONFIG: the VCC LDO is powered from VIN, so with no charge
//     input the chip is dead — I2C NACKs mean "input absent", not a bus
//     fault, and every register resets to -0000 defaults on each plug-in.
//  2. I2C WATCHDOG: 40 s by default; expiry reverts the WD=Y register bits
//     to defaults (charging continues at RISET-clamped full scale) and sets
//     WD_FAULT. We keep the watchdog ON as the host-crash posture and kick
//     it every service() pass.
//  3. WRITE VERIFICATION: field reports of parts silently refusing writes —
//     every write is read back; persistent mismatch is surfaced.
// => service() is a reconcile-to-desired-state loop: (re)detect the chip,
//    (re)apply config whenever it POR'd or drifted, kick the watchdog, poll
//    status/faults, and log transitions under its own tag.
//
// Only single-byte transactions are used (the production MP2672A datasheet
// documents single-byte R/W only; no auto-increment).
//
// Current scaling: fast-charge current = full_scale * (5 + code) / 20 where
// full_scale = 12 kΩ / RISET amps (RISET is a board resistor). Code 0 is
// therefore always the minimum (25% of full scale) — the safe default until
// the board's RISET value is confirmed on the bench. The -0000 default is
// code 15 = full scale, which is also the hardware clamp the pack falls
// back to if the watchdog fires with a dead host: RISET, not software, is
// the real current backstop. Choose RISET accordingly.

namespace MP2672_Reg {
    static constexpr uint8_t REG00 = 0x00;  // VBATT_REG[7:5] CHG_CONFIG[4] VBATT_PRE[3:1]
    static constexpr uint8_t REG01 = 0x01;  // NTC_TYPE[7] balance[6:4] ICC[3:0]
    static constexpr uint8_t REG02 = 0x02;  // FSW[7] WD_KICK[6] WD_TIMER[5:4] RST[3] CHG_TMR[2:1]
    static constexpr uint8_t REG03 = 0x03;  // status (RO)
    static constexpr uint8_t REG04 = 0x04;  // faults (RO)

    // REG03 status bits
    static constexpr uint8_t CHG_STAT_SHIFT = 4;   // [5:4]
    static constexpr uint8_t CHG_STAT_MASK  = 0x30;
    static constexpr uint8_t PPM_STAT       = 1u << 3;
    static constexpr uint8_t BATTFLOAT_STAT = 1u << 2;
    static constexpr uint8_t THERM_STAT     = 1u << 1;
    static constexpr uint8_t VSYS_STAT      = 1u << 0;

    // REG04 fault bits
    static constexpr uint8_t WD_FAULT       = 1u << 7;
    static constexpr uint8_t INPUT_FAULT    = 1u << 6;   // input OVP (>6.0 V)
    static constexpr uint8_t THERMSD_FAULT  = 1u << 5;
    static constexpr uint8_t TIMER_FAULT    = 1u << 4;
    static constexpr uint8_t BAT_FAULT      = 1u << 3;   // battery OVP
    static constexpr uint8_t NTC_FAULT_MASK = 0x07;
}

enum class MP2672ChargeState : uint8_t {
    NotCharging = 0,   // REG03 CHG_STAT 00
    PreCharge   = 1,   // 01
    FastCharge  = 2,   // 10 (CC or CV — no separate CV code)
    Done        = 3,   // 11
};

struct TR_MP2672_Config
{
    // REG00 VBATT_REG[2:0]: pack regulation = 8.3 V + code * 100 mV.
    // 1 = 8.4 V (4.2 V/cell). Never exceed 1 for LiPo; code 7 is ambiguous
    // across MP2672/MP2672A silicon (9.0 vs 8.2 V) — the driver refuses it.
    uint8_t vbatt_reg_code = 1;

    // REG01 ICC[3:0]: fast-charge current = full_scale*(5+code)/20.
    // Default 0 = minimum (25% of RISET full scale) until RISET is known.
    uint8_t icc_code = 0;

    // REG02 CHG_TMR[1:0]: fast-charge safety timer. 01 = 8 h (the one
    // encoding both datasheet revisions agree on).
    uint8_t chg_timer_code = 1;

    bool charge_enable = true;   // REG00 CHG_CONFIG
};

struct TR_MP2672_Data
{
    bool present = false;            // chip answered on the last service()
    MP2672ChargeState state = MP2672ChargeState::NotCharging;
    bool battery_absent = false;     // BATTFLOAT_STAT
    bool in_ppm = false;             // input voltage-limit regulation
    bool in_thermal_reg = false;
    bool in_vsys_reg = false;        // pack below VBATT_PRE or absent
    uint8_t faults = 0;              // raw REG04
    uint8_t status_raw = 0;          // raw REG03
    uint32_t config_applies = 0;     // times config was (re)applied
    uint32_t write_verify_fails = 0;
};

class TR_MP2672
{
public:
    explicit TR_MP2672(uint8_t addr = 0x4B);

    // Adds the device to the bus. Succeeds even when the charger is
    // unpowered (no charge input) — presence is (re)detected per service().
    esp_err_t begin(i2c_master_bus_handle_t bus,
                    const TR_MP2672_Config& cfg,
                    uint32_t clock_hz = 400000);

    // Reconcile pass — call on the battery-poll cadence (a few seconds;
    // must be well inside the 40 s watchdog). Cheap when unpowered (one
    // failed read). Logs state/fault/presence transitions.
    void service();

    const TR_MP2672_Data& data() const { return _data; }
    bool present() const { return _data.present; }
    MP2672ChargeState state() const { return _data.state; }
    static const char* stateName(MP2672ChargeState s);

    // One-shot register dump.
    void logDiagnostics(const char* log_tag);

    bool readReg(uint8_t reg, uint8_t& value);
    bool writeReg(uint8_t reg, uint8_t value);

private:
    uint8_t desiredReg00() const;
    uint8_t desiredReg01() const;
    uint8_t desiredReg02() const;   // without the kick bit
    bool applyConfig();             // write + readback-verify REG01/02/00

    i2c_master_dev_handle_t _dev;
    uint8_t                 _addr;
    TR_MP2672_Config        _cfg;
    TR_MP2672_Data          _data;
    bool                    _config_applied;
};

#endif
