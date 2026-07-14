#ifndef TR_BQ27Z746_H
#define TR_BQ27Z746_H

#include <compat.h>
#include <driver/i2c_master.h>

// TI BQ27Z746 single-cell Impedance Track fuel gauge + protection driver.
// Register map per BQ27Z746 TRM (SLUUCA6). Standard commands are 16-bit
// little-endian words; extended data uses AltManufacturerAccess() (write the
// 16-bit subcommand to 0x3E/0x3F, read the response back from MACData 0x40).
//
// New-board replacement for the MAX17205 (which sat at 0x36); the BQ27Z746
// lives at 0x55. The public accessor surface (voltage/current/soc/temperature/
// update) matches TR_MAX17205G so the two are interchangeable in the base
// station firmware via runtime detection.

namespace BQ27Z746_Reg {
    // Standard commands (16-bit, little-endian)
    static constexpr uint8_t TEMPERATURE = 0x06;  // 0.1 K
    static constexpr uint8_t VOLTAGE     = 0x08;  // mV
    static constexpr uint8_t BATT_STATUS = 0x0A;  // flags
    static constexpr uint8_t CURRENT     = 0x0C;  // mA, signed (+ = charge)
    static constexpr uint8_t REMAIN_CAP  = 0x10;  // mAh
    static constexpr uint8_t FULL_CAP    = 0x12;  // mAh
    static constexpr uint8_t RSOC        = 0x2C;  // %

    // AltManufacturerAccess command / data block
    static constexpr uint8_t ALT_MFR_ACCESS = 0x3E;  // write 16-bit subcommand here
    static constexpr uint8_t MAC_DATA       = 0x40;  // read response block here

    // AltManufacturerAccess subcommands
    static constexpr uint16_t SUB_DEVICE_TYPE   = 0x0001;  // -> 0x1746 for BQ27Z746
    static constexpr uint16_t SUB_FET_ENABLE    = 0x0022;  // toggles ManufacturingStatus[FET_EN]
    static constexpr uint16_t SUB_SAFETY_STATUS = 0x0051;
    static constexpr uint16_t SUB_OP_STATUS     = 0x0054;  // OperationStatus A:B
    static constexpr uint16_t SUB_MFG_STATUS    = 0x0057;

    // OperationStatus A bits
    static constexpr uint16_t OPA_XCHG = (1u << 14);  // 1 = charging disabled (CHG FET off)
    static constexpr uint16_t OPA_XDSG = (1u << 13);  // 1 = discharging disabled (DSG FET off)
    static constexpr uint16_t OPA_SS   = (1u << 11);  // 1 = a Safety protection is active
    // ManufacturingStatus bits
    static constexpr uint16_t MFG_CAL_EN   = (1u << 15);  // CALIBRATION mode
    static constexpr uint16_t MFG_FET_EN   = (1u << 4);
    static constexpr uint16_t MFG_GAUGE_EN = (1u << 3);

    // Data-flash + calibration access (TRM SLUUCA6 §15.1.28/29, §16, §17)
    static constexpr uint8_t  MAC_DATA_CHECKSUM = 0x60;  // ~(addr+data) sum
    static constexpr uint8_t  MAC_DATA_LENGTH   = 0x61;  // = data bytes + 4
    static constexpr uint16_t SUB_CAL_TOGGLE     = 0x002D;  // toggles ManufacturingStatus[CAL_EN]
    static constexpr uint16_t SUB_CAL_OUTPUT     = 0xF081;  // raw CC/ADC -> MACData (needs CAL_EN)
    static constexpr uint16_t SUB_CAL_OUTPUT_OFF = 0xF080;  // exit calibration output mode
    // Data-flash addresses (little-endian values per §16.2.2)
    static constexpr uint16_t DF_DESIGN_CAP_MAH = 0x4600;  // I2, mAh (factory default 5300)
    static constexpr uint16_t DF_CC_GAIN        = 0x4006;  // F4, default 3.68 (needs known-current cal)
    static constexpr uint16_t DF_CAPACITY_GAIN  = 0x400A;  // F4
}

struct TR_BQ27Z746_Data
{
    float voltage;        // Pack/cell voltage, V
    float current;        // mA (+ = charging per gauge, optionally inverted)
    float soc;            // State-of-charge, %
    float temperature;    // °C
    float capacity;       // Remaining capacity, mAh
    float full_capacity;  // Full (learned) capacity, mAh
    uint16_t batt_status; // BatteryStatus() (0x0A)
    bool chg_fet_on;      // !XCHG  (CHG FET conducting)
    bool dsg_fet_on;      // !XDSG  (DSG FET conducting)
    bool fet_en;          // ManufacturingStatus FET_EN (autonomous FET control)
    bool gauge_en;        // ManufacturingStatus GAUGE_EN — Impedance Track gauging.
                          //   Ships at 0. While it is 0 the gauge never integrates
                          //   current, so soc/capacity/full_capacity are meaningless
                          //   even though voltage/temperature are fine (#501).
    bool safety_active;   // OperationStatus SS (a protection is active)
};

struct TR_BQ27Z746_Config
{
    bool current_invert   = false; // Flip the *displayed* current sign if SRP/SRN are
                                   //   swapped vs. the datasheet typical app circuit.
    bool auto_enable_fets = true;  // On begin(), enable the CHG/DSG FETs if FET_EN=0 and
                                   //   no safety fault. See enableFetsIfNeeded().
};

class TR_BQ27Z746
{
public:
    explicit TR_BQ27Z746(uint8_t addr = 0x55);

    // Add device to an existing I2C master bus and probe it (Voltage read must
    // succeed). Reads the device type and, if cfg.auto_enable_fets, enables the
    // FETs when needed.
    esp_err_t begin(i2c_master_bus_handle_t bus,
                    const TR_BQ27Z746_Config& cfg,
                    uint32_t clock_hz = 400000);

    // Re-read voltage/current/soc/temperature/capacity + FET status into data().
    esp_err_t update();

    // If FET_EN=0 and no safety fault, toggle AltManufacturerAccess(0x0022) to
    // enable the CHG/DSG FETs (persists to data flash). Idempotent — only acts
    // when FET_EN=0 — so it is safe to call on every boot.
    esp_err_t enableFetsIfNeeded();

    // Device type read at begin() (0x1746 for a BQ27Z746); 0 if not read.
    uint16_t deviceType() const { return _device_type; }

    const TR_BQ27Z746_Data& data() const { return _data; }
    float voltage() const     { return _data.voltage; }
    float current() const     { return _data.current; }
    float soc() const         { return _data.soc; }
    float temperature() const { return _data.temperature; }
    bool  fetsEnabled() const { return _data.fet_en; }
    // False => Impedance Track gauging is off, so soc()/capacity()/full_capacity
    // are not gauged values at all. Callers must not report them as SoC (#501).
    bool  gaugingEnabled() const { return _data.gauge_en; }
    float fullCapacity() const   { return _data.full_capacity; }
    bool  chgFetOn() const    { return _data.chg_fet_on; }
    bool  dsgFetOn() const    { return _data.dsg_fet_on; }

    // Low-level helpers (exposed for diagnostics).
    bool      readWord(uint8_t reg, uint16_t& value);
    esp_err_t macRead(uint16_t sub, uint8_t* blk, size_t n);
    esp_err_t macCmd(uint16_t sub);

    // --- Data-flash provisioning + diagnostics (new-PCB bring-up) ---
    // Read/write a 16-bit little-endian data-flash word (TRM §16). The write
    // verifies by read-back; the gauge validates checksum+length and rejects a
    // malformed write (so a wrong protocol is non-destructive, just returns false).
    bool readDataFlashI16(uint16_t addr, int16_t& value);
    bool writeDataFlashI16(uint16_t addr, int16_t value);

    // One-shot: ensure the gauge's DesignCapacity matches design_mah. The part
    // ships at 5300 mAh; we never configured it, so SOC/capacity are meaningless
    // until this runs. No-op (no flash write) once it already matches.
    esp_err_t provisionDesignCapacity(int16_t design_mah);

    // Diagnostic (read-only): raw coulomb-counter ADC current via 0xF081. Tells
    // whether the current channel actually sees the sense resistor (varies with
    // real battery current) vs. a stuck/garbage reading. Logs the raw block.
    bool readRawCcCurrent(int16_t& raw_cc);

    // One-shot diagnostic dump under the given log tag.
    void logDiagnostics(const char* log_tag);

private:
    esp_err_t readFetStatus();   // refresh chg/dsg/fet_en/safety flags in _data
    esp_err_t macWrite(uint16_t addr, const uint8_t* data, size_t n);  // MAC/DF block write (checksum+length)

    i2c_master_dev_handle_t _dev;
    uint8_t                 _addr;
    uint16_t                _device_type;
    TR_BQ27Z746_Config      _cfg;
    TR_BQ27Z746_Data        _data;
};

#endif
