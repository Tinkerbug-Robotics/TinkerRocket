#ifndef TR_MAX17303_H
#define TR_MAX17303_H

#include <compat.h>
#include <driver/i2c_master.h>

// MAX17303 ModelGauge m5 EZ fuel gauge with integrated 1-level protector
// (MAX17300-MAX17303 family, datasheet 19-100463 Rev 6). 1S only — the
// protector drives two external high-side N-FETs (CHG/DIS).
//
// This is NOT a MAX17055/MAX1720x-style part despite sharing the m5 core:
//  - It self-restores its full configuration + battery model from internal
//    nonvolatile memory at POR, with no host involvement. There is no
//    documented ModelCfg.Refresh EZ flow and no HibCfg/soft-wakeup dance.
//  - Command (0x060) is the NV/SHA command register: a stray write can burn
//    one of the SEVEN lifetime NV config copies (0xE904) or PERMANENTLY lock
//    pages (0x6AXX). This driver never touches it.
//  - RAM config registers (DesignCap etc.) restore from NV each POR; the
//    driver may re-seed them in volatile RAM (zero NV writes) and apply via
//    Config2.POR_CMD, which restarts the fuel-gauge firmware WITHOUT an NV
//    recall.
//  - Unlike the BQ27Z746 there is no FET_EN ship-state trap: with a battery
//    attached and no faults the protection FETs conduct out of the box.
//    FET state is observable in HConfig2 (CHGs/DISs) and fault causes in
//    ProtStatus.
//
// Only the primary I2C address (0x36, registers 0x000-0x0FF) is used. The
// nonvolatile shadow page behind secondary address 0x0B is intentionally
// not touched (limited write budget; see above).

namespace MAX17303_Reg {
    // Note: measurement addresses DIFFER from the MAX1720x map (e.g. VCell
    // is 0x1A here vs 0x09 there) — do not copy constants across drivers.
    static constexpr uint8_t STATUS       = 0x00;  // POR bit 1, PA bit 15
    static constexpr uint8_t REP_CAP      = 0x05;
    static constexpr uint8_t REP_SOC      = 0x06;  // 1/256 %
    static constexpr uint8_t FULL_CAP_REP = 0x10;
    static constexpr uint8_t TTE          = 0x11;  // 5.625 s/LSB
    static constexpr uint8_t CYCLES       = 0x17;  // 25% of a cycle per LSB
    static constexpr uint8_t DESIGN_CAP   = 0x18;
    static constexpr uint8_t AVG_VCELL    = 0x19;
    static constexpr uint8_t VCELL        = 0x1A;  // 78.125 µV/LSB
    static constexpr uint8_t TEMP         = 0x1B;  // 1/256 °C signed
    static constexpr uint8_t CURRENT      = 0x1C;  // 1.5625 µV/Rsense signed
    static constexpr uint8_t AVG_CURRENT  = 0x1D;
    static constexpr uint8_t ICHG_TERM    = 0x1E;
    static constexpr uint8_t DEV_NAME     = 0x21;  // 0x4067 = MAX17303
    static constexpr uint8_t DIE_TEMP     = 0x34;
    static constexpr uint8_t V_EMPTY      = 0x3A;
    static constexpr uint8_t FSTAT        = 0x3D;  // DNR bit 0
    static constexpr uint8_t COMM_STAT    = 0x61;  // NVBusy/NVError (read-only use)
    static constexpr uint8_t CONFIG2      = 0xAB;  // POR_CMD bit 15 (self-clearing)
    static constexpr uint8_t PROT_ALRT    = 0xAF;  // latched fault history
    static constexpr uint8_t PROT_STATUS  = 0xD9;  // live protector faults
    static constexpr uint8_t HCONFIG2     = 0xF5;  // CHGs bit 6, DISs bit 7

    // DevName check: bits[15:4] identify the family/silicon rev (0x406 on
    // production parts, older samples read 0x404/0x405); low nibble is the
    // variant (7 = MAX17303). MAX1720x parts at the same I2C address read a
    // firmware-revision-based value (~0x02xx) with device nibble 1/5, so the
    // family field cleanly separates the two.
    static constexpr uint16_t DEVNAME_FAMILY      = 0x406;
    static constexpr uint16_t DEVNAME_VARIANT_303 = 0x7;
}

struct TR_MAX17303_Data
{
    float voltage;        // Cell/pack voltage (1S), V
    float current;        // mA (positive = charging after optional invert)
    float soc;            // State-of-charge, % (RepSOC)
    float temperature;    // °C
    float capacity;       // Remaining capacity, mAh
    float full_capacity;  // Full capacity (learned), mAh
    uint16_t prot_status; // Raw ProtStatus (0 = no faults)
    bool chg_fet_on;      // HConfig2.CHGs
    bool dis_fet_on;      // HConfig2.DISs
};

struct TR_MAX17303_Config
{
    uint16_t design_mah     = 2800;   // Pack design capacity; 0 = leave the
                                      //   chip's NV-restored value untouched
    float    rsense_mohm    = 10.0f;  // Sense resistor value (host-side
                                      //   current/capacity scaling only)
    bool     current_invert = false;  // Flip the *displayed* current sign
                                      //   (CSP/CSN swap on the PCB)
};

class TR_MAX17303
{
public:
    explicit TR_MAX17303(uint8_t addr = 0x36);

    // Add device to an existing I2C master bus, then verify DevName. Returns
    // ESP_ERR_NOT_FOUND (device removed from the bus again) when the part at
    // this address is not a MAX1730x — the caller can then fall through to
    // the MAX17205 driver, which shares the address.
    esp_err_t begin(i2c_master_bus_handle_t bus,
                    const TR_MAX17303_Config& cfg,
                    uint32_t clock_hz = 400000);

    // POR handling: wait FStat.DNR, optionally re-seed DesignCap-derived
    // capacity registers in volatile RAM (applied via Config2.POR_CMD — zero
    // NV writes), clear Status.POR. Self-gated; safe to call every boot.
    esp_err_t initIfNeeded();

    // Re-read voltage/current/soc/temperature/capacity/protector state.
    esp_err_t update();

    const TR_MAX17303_Data& data() const { return _data; }
    float voltage() const     { return _data.voltage; }
    float current() const     { return _data.current; }
    float soc() const         { return _data.soc; }
    float temperature() const { return _data.temperature; }
    uint16_t protStatus() const { return _data.prot_status; }
    // Both protection FETs conducting (battery actually connected to the
    // system) — the analog of the BQ27Z746 FET check, but observation-only.
    bool fetsOn() const { return _data.chg_fet_on && _data.dis_fet_on; }

    uint16_t devName() const { return _devname; }

    // Low-level helpers (exposed for diagnostics).
    bool readReg(uint8_t reg, uint16_t& value);
    bool writeReg(uint8_t reg, uint16_t value);

    // One-shot diagnostic dump incl. decoded ProtStatus + FET states.
    void logDiagnostics(const char* log_tag);

    uint16_t designCapRaw() const;

private:
    esp_err_t seedCapacityAndRestart();

    i2c_master_dev_handle_t _dev;
    uint8_t                 _addr;
    uint16_t                _devname;
    TR_MAX17303_Config      _cfg;
    TR_MAX17303_Data        _data;
};

#endif
