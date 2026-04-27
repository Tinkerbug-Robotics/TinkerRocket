#ifndef TR_MAX17205G_H
#define TR_MAX17205G_H

#include <compat.h>
#include <driver/i2c_master.h>

// MAX17201/MAX17205 ModelGauge m5 fuel gauge driver.
// Supports 1S/2S/3S Li-ion packs. Uses the primary I2C address (0x36) for
// volatile registers 0x00-0xFF. Nonvolatile registers at 0x100-0x1FF (accessed
// via secondary address 0x0B) are intentionally NOT written — the chip has a
// limited NV write budget (~7) and all needed config can be re-seeded on each
// cold boot into volatile shadow RAM.

namespace MAX17205_Reg {
    // Output / status
    static constexpr uint8_t STATUS     = 0x00;  // POR bit 1, Br bit 15, Bi bit 11
    static constexpr uint8_t REP_CAP    = 0x05;  // Reported remaining capacity
    static constexpr uint8_t REP_SOC    = 0x06;  // Reported SOC (1/256 %)
    static constexpr uint8_t TEMP       = 0x08;  // Die temp (signed, 1/256 °C)
    static constexpr uint8_t VCELL      = 0x09;  // Per-cell voltage (0.078125 mV/LSB)
    static constexpr uint8_t CURRENT    = 0x0A;  // Signed (1.5625 µV / Rsense)
    static constexpr uint8_t AVG_CUR    = 0x0B;
    static constexpr uint8_t MIX_SOC    = 0x0D;
    static constexpr uint8_t AV_SOC     = 0x0E;
    static constexpr uint8_t FULL_CAP   = 0x10;
    static constexpr uint8_t AVG_VCELL  = 0x19;
    static constexpr uint8_t FULL_CAP_NOM = 0x23;
    static constexpr uint8_t FULL_CAP_REP = 0x35;
    static constexpr uint8_t VFSOC      = 0xFF;
    static constexpr uint8_t FSTAT      = 0x3D;  // DNR bit 0

    // Config / command
    static constexpr uint8_t DESIGN_CAP = 0x18;
    static constexpr uint8_t ICHG_TERM  = 0x1E;
    static constexpr uint8_t V_EMPTY    = 0x3A;
    static constexpr uint8_t SOFT_WAKEUP = 0x60;
    static constexpr uint8_t HIB_CFG    = 0xBA;
    static constexpr uint8_t PACK_CFG   = 0xBD;
    static constexpr uint8_t MODEL_CFG  = 0xDB;  // Bit 15 = Refresh (self-clearing)
}

struct TR_MAX17205G_Data
{
    float voltage;        // Pack voltage, V
    float current;        // mA (positive = charging after optional invert)
    float soc;            // State-of-charge, %
    float temperature;    // °C
    float capacity;       // Remaining capacity, mAh
    float full_capacity;  // Full capacity (learned), mAh
};

struct TR_MAX17205G_Config
{
    uint16_t design_mah     = 2800;   // Pack design capacity in mAh (series = per-cell)
    float    rsense_mohm    = 10.0f;  // Sense resistor value
    bool     current_invert = true;   // Flip the *displayed* current sign. Set true on
                                      //   boards where CSP/CSN are swapped vs. the
                                      //   MAX17205 typical app circuit (which expects
                                      //   CSP→BATT-, CSN→GND for charge to read positive).
                                      //   NOTE: this only affects the value returned by
                                      //   current(); the chip's internal m5 algorithm uses
                                      //   the raw register and cannot be told about the
                                      //   swap, so RepSOC integrates current the wrong way.
                                      //   Driver reads VFSOC (voltage-based) for soc() to
                                      //   sidestep this.
    uint8_t  num_cells      = 2;      // Series cell count (for pack-voltage scaling
                                      //   off the VCell register)
};

class TR_MAX17205G
{
public:
    explicit TR_MAX17205G(uint8_t addr = 0x36);

    // Add device to an existing I2C master bus and probe it.
    esp_err_t begin(i2c_master_bus_handle_t bus,
                    const TR_MAX17205G_Config& cfg,
                    uint32_t clock_hz = 400000);

    // Runs the ModelGauge m5 EZ init (DesignCap + ModelCfg.Refresh + override of
    // FullCap/FullCapRep/FullCapNom/RepCap). Decides whether to run based on
    // Status.POR and a sanity check that FullCapNom roughly matches DesignCap.
    // Always safe to call at boot.
    esp_err_t initIfNeeded();

    // Force the full init regardless of chip state. Use from diagnostics.
    esp_err_t forceReinit();

    // Re-read voltage/current/soc/temperature/capacity. Populates data().
    esp_err_t update();

    const TR_MAX17205G_Data& data() const { return _data; }
    float voltage() const     { return _data.voltage; }
    float current() const     { return _data.current; }
    float soc() const         { return _data.soc; }
    float temperature() const { return _data.temperature; }

    // Low-level helpers (exposed for diagnostics).
    bool readReg(uint8_t reg, uint16_t& value);
    bool writeReg(uint8_t reg, uint16_t value);

    // One-shot diagnostic dump: Status, candidate SoC registers, capacity
    // registers, and pack voltage. Uses ESP_LOGI under the given log tag.
    void logDiagnostics(const char* log_tag);

    // Compute the DesignCap raw value for the configured pack. Exposed for
    // tests and the staleness check.
    uint16_t designCapRaw() const;

private:
    esp_err_t runEzInit();

    i2c_master_dev_handle_t _dev;
    uint8_t                 _addr;
    TR_MAX17205G_Config     _cfg;
    TR_MAX17205G_Data       _data;
};

#endif
