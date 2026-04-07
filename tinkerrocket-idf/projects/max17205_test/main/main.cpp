/**
 * MAX17205G I2C Test — ESP32-S3 (pure ESP-IDF, new i2c_master API)
 *
 * 1. Scans the I2C bus and prints all responding addresses.
 * 2. Reads key registers from the MAX17205G fuel gauge.
 *
 * MAX17205G has TWO I2C addresses:
 *   0x36 — primary (ModelGauge m5 registers: SOC, voltage, current, etc.)
 *   0x0B — secondary (non-volatile memory / shadow RAM, registers 0x180–0x1FF)
 *
 * Wiring:
 *   SCL = GPIO 37
 *   SDA = GPIO 38
 */

#include <cstdio>
#include <cstring>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "MAX17205";

// ── Pin configuration ──
static constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_37;
static constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_38;
static constexpr uint32_t   I2C_FREQ_HZ = 400'000;  // 400 kHz

// ── MAX17205 I2C addresses (7-bit) ──
static constexpr uint16_t MAX17205_ADDR_PRIMARY   = 0x36;  // Registers 0x000–0x0FF
static constexpr uint16_t MAX17205_ADDR_SECONDARY = 0x0B;  // Registers 0x180–0x1FF

// ── MAX17205 register addresses (primary 0x36) ──
static constexpr uint8_t REG_STATUS      = 0x00;
static constexpr uint8_t REG_REP_CAP     = 0x05;
static constexpr uint8_t REG_REP_SOC     = 0x06;
static constexpr uint8_t REG_TEMP        = 0x08;
static constexpr uint8_t REG_VCELL       = 0x09;
static constexpr uint8_t REG_CURRENT     = 0x0A;
static constexpr uint8_t REG_AVG_CURRENT = 0x0B;
static constexpr uint8_t REG_FULL_CAP_REP = 0x10;
static constexpr uint8_t REG_TTE         = 0x11;
static constexpr uint8_t REG_AVG_VCELL   = 0x19;
static constexpr uint8_t REG_TTF         = 0x20;
static constexpr uint8_t REG_DEV_NAME    = 0x21;
static constexpr uint8_t REG_PACK_CFG    = 0xBD;
static constexpr uint8_t REG_CELL1       = 0xD8;  // Cell 1 voltage (top cell)
static constexpr uint8_t REG_CELL2       = 0xD7;  // Cell 2 voltage
static constexpr uint8_t REG_BATT        = 0xDA;  // Pack voltage (BATT to GND)

// Secondary address (0x0B) register offsets — subtract 0x180 to get wire byte
// e.g. nRSense at 0x1CF → send 0xCF to address 0x0B
static constexpr uint8_t REG_NRSENSE     = 0xCF;  // Sense resistor value (addr 0x1CF)
static constexpr uint8_t REG_NPACKCFG    = 0xB5;  // nPackCfg (addr 0x1B5)
static constexpr uint8_t REG_NDESIGNCAP  = 0xB3;  // nDesignCap (addr 0x1B3)

// ── Conversion factors (from MAX17205 datasheet + linux driver) ──
// VCell/Cell1/Cell2:  0.078125 mV/LSB  (1.25 mV / 16)
// Batt (pack):        1.25 mV/LSB
// Current:            102.4 mV full-scale / 65536 over Rsense
//                     → 1.5625 uV/LSB, divide by Rsense(Ohm) for amps
// Capacity:           0.5 mAh/LSB (with 10 mOhm Rsense)
//                     scales as 5 uVh/LSB ÷ Rsense
// Temperature:        1/256 deg C/LSB
// SOC:                1/256 %/LSB
// Time:               5.625 s/LSB
static constexpr float VCELL_LSB_MV     = 0.078125f;  // VCell, Cell1, Cell2
static constexpr float BATT_LSB_MV      = 1.25f;      // Batt (pack voltage)
static constexpr float CAPACITY_LSB_MAH = 0.5f;       // with 10 mOhm Rsense
static constexpr float CURRENT_LSB_UV   = 1.5625f;    // voltage across Rsense in uV
static constexpr float TEMP_LSB_C       = 1.0f / 256.0f;
static constexpr float SOC_LSB_PCT      = 1.0f / 256.0f;
static constexpr float TIME_LSB_S       = 5.625f;

// Default Rsense (adjust to match your board)
static constexpr float RSENSE_MOHM      = 10.0f;      // 10 mOhm

// ────────────────────────────────────────────────────────────────────
// I2C handles (new master API)
// ────────────────────────────────────────────────────────────────────

static i2c_master_bus_handle_t bus_handle = nullptr;
static i2c_master_dev_handle_t dev_primary = nullptr;    // 0x36
static i2c_master_dev_handle_t dev_secondary = nullptr;  // 0x0B

static esp_err_t i2c_init()
{
    // Configure the I2C bus
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port     = I2C_NUM_0;
    bus_cfg.sda_io_num   = I2C_SDA_PIN;
    bus_cfg.scl_io_num   = I2C_SCL_PIN;
    bus_cfg.clk_source   = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = false;

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus_handle);
    if (err != ESP_OK) return err;

    // Add the primary device (0x36)
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = MAX17205_ADDR_PRIMARY;
    dev_cfg.scl_speed_hz    = I2C_FREQ_HZ;

    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_primary);
    if (err != ESP_OK) return err;

    // Add the secondary device (0x0B)
    dev_cfg.device_address = MAX17205_ADDR_SECONDARY;
    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_secondary);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

/** Read a 16-bit register (little-endian) from a device handle. */
static bool readReg16(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t& value)
{
    uint8_t buf[2] = {};
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, buf, 2, pdMS_TO_TICKS(100));
    if (err != ESP_OK)
        return false;

    value = ((uint16_t)buf[1] << 8) | buf[0];  // Little-endian
    return true;
}

// ────────────────────────────────────────────────────────────────────
// I2C bus scan
// ────────────────────────────────────────────────────────────────────

static void i2cScan()
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "I2C Bus Scan (SCL=%d, SDA=%d, %lu kHz)",
             (int)I2C_SCL_PIN, (int)I2C_SDA_PIN, (unsigned long)(I2C_FREQ_HZ / 1000));
    ESP_LOGI(TAG, "========================================");

    int found = 0;
    for (uint16_t addr = 1; addr < 127; addr++)
    {
        esp_err_t err = i2c_master_probe(bus_handle, addr, pdMS_TO_TICKS(50));
        if (err == ESP_OK)
        {
            const char* label = "";
            if (addr == MAX17205_ADDR_PRIMARY)   label = " <-- MAX17205 primary";
            if (addr == MAX17205_ADDR_SECONDARY) label = " <-- MAX17205 secondary (nV mem)";
            ESP_LOGI(TAG, "  0x%02X  ACK%s", addr, label);
            found++;
        }
    }

    if (found == 0)
        ESP_LOGW(TAG, "  No devices found! Check wiring.");
    else
        ESP_LOGI(TAG, "  %d device(s) found.", found);

    ESP_LOGI(TAG, "");
}

// ────────────────────────────────────────────────────────────────────
// MAX17205 register dump
// ────────────────────────────────────────────────────────────────────

static void readMAX17205()
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "MAX17205G Register Read");
    ESP_LOGI(TAG, "========================================");

    if (i2c_master_probe(bus_handle, MAX17205_ADDR_PRIMARY, pdMS_TO_TICKS(50)) != ESP_OK)
    {
        ESP_LOGE(TAG, "MAX17205 not responding at 0x%02X", MAX17205_ADDR_PRIMARY);
        return;
    }

    // Current LSB in mA depends on Rsense:
    //   register LSB = 1.5625 uV across Rsense
    //   current_mA = raw * 1.5625e-3 mV / Rsense_mOhm
    const float current_lsb_ma = CURRENT_LSB_UV * 1e-3f / RSENSE_MOHM;

    uint16_t raw;

    // ── Identification ──
    if (readReg16(dev_primary, REG_DEV_NAME, raw))
        ESP_LOGI(TAG, "  DevName:       0x%04X", raw);
    else
        ESP_LOGW(TAG, "  DevName:       READ FAILED");

    if (readReg16(dev_primary, REG_STATUS, raw))
        ESP_LOGI(TAG, "  Status:        0x%04X", raw);

    if (readReg16(dev_primary, REG_PACK_CFG, raw))
    {
        uint8_t nCells = raw & 0x0F;
        ESP_LOGI(TAG, "  PackCfg:       0x%04X  (nCells=%d)", raw, nCells);
    }

    // ── nRSense from secondary address (0x0B, reg 0x1CF) ──
    if (readReg16(dev_secondary, REG_NRSENSE, raw))
    {
        // nRSense is in units of 10^-5 Ohm (i.e. raw=1000 → 10 mOhm)
        float rsense_mohm = (float)raw / 100.0f;
        ESP_LOGI(TAG, "  nRSense:       %.2f mOhm  (raw=0x%04X = %u)", (double)rsense_mohm, raw, raw);
    }

    // ── Voltages ──
    if (readReg16(dev_primary, REG_VCELL, raw))
    {
        float mv = (float)raw * VCELL_LSB_MV;
        ESP_LOGI(TAG, "  VCell (low):   %.1f mV  (raw=0x%04X)", (double)mv, raw);
    }

    if (readReg16(dev_primary, REG_AVG_VCELL, raw))
    {
        float mv = (float)raw * VCELL_LSB_MV;
        ESP_LOGI(TAG, "  AvgVCell:      %.1f mV  (raw=0x%04X)", (double)mv, raw);
    }

    if (readReg16(dev_primary, REG_CELL1, raw))
    {
        float mv = (float)raw * VCELL_LSB_MV;
        ESP_LOGI(TAG, "  Cell1:         %.1f mV  (raw=0x%04X)", (double)mv, raw);
    }

    if (readReg16(dev_primary, REG_CELL2, raw))
    {
        float mv = (float)raw * VCELL_LSB_MV;
        ESP_LOGI(TAG, "  Cell2:         %.1f mV  (raw=0x%04X)", (double)mv, raw);
    }

    if (readReg16(dev_primary, REG_BATT, raw))
    {
        float mv = (float)raw * BATT_LSB_MV;
        ESP_LOGI(TAG, "  Batt (pack):   %.0f mV  (%.2f V)  (raw=0x%04X)",
                 (double)mv, (double)(mv / 1000.0f), raw);
    }

    // ── Current (signed) ──
    if (readReg16(dev_primary, REG_CURRENT, raw))
    {
        float ma = (float)(int16_t)raw * current_lsb_ma;
        ESP_LOGI(TAG, "  Current:       %.3f mA  (raw=0x%04X)", (double)ma, raw);
    }

    if (readReg16(dev_primary, REG_AVG_CURRENT, raw))
    {
        float ma = (float)(int16_t)raw * current_lsb_ma;
        ESP_LOGI(TAG, "  AvgCurrent:    %.3f mA  (raw=0x%04X)", (double)ma, raw);
    }

    // ── Temperature (signed) ──
    if (readReg16(dev_primary, REG_TEMP, raw))
    {
        float degC = (float)(int16_t)raw * TEMP_LSB_C;
        ESP_LOGI(TAG, "  Temperature:   %.1f C  (raw=0x%04X)", (double)degC, raw);
    }

    // ── State of charge & capacity ──
    if (readReg16(dev_primary, REG_REP_SOC, raw))
    {
        float pct = (float)raw * SOC_LSB_PCT;
        ESP_LOGI(TAG, "  RepSOC:        %.1f %%  (raw=0x%04X)", (double)pct, raw);
    }

    if (readReg16(dev_primary, REG_REP_CAP, raw))
    {
        float mah = (float)raw * CAPACITY_LSB_MAH;
        ESP_LOGI(TAG, "  RepCap:        %.1f mAh  (raw=0x%04X)", (double)mah, raw);
    }

    if (readReg16(dev_primary, REG_FULL_CAP_REP, raw))
    {
        float mah = (float)raw * CAPACITY_LSB_MAH;
        ESP_LOGI(TAG, "  FullCapRep:    %.1f mAh  (raw=0x%04X)", (double)mah, raw);
    }

    // ── Time estimates ──
    if (readReg16(dev_primary, REG_TTE, raw))
    {
        if (raw == 0xFFFF)
            ESP_LOGI(TAG, "  TimeToEmpty:   N/A (not discharging)");
        else
        {
            float mins = (float)raw * TIME_LSB_S / 60.0f;
            ESP_LOGI(TAG, "  TimeToEmpty:   %.1f min  (raw=0x%04X)", (double)mins, raw);
        }
    }

    if (readReg16(dev_primary, REG_TTF, raw))
    {
        if (raw == 0xFFFF)
            ESP_LOGI(TAG, "  TimeToFull:    N/A (not charging)");
        else
        {
            float mins = (float)raw * TIME_LSB_S / 60.0f;
            ESP_LOGI(TAG, "  TimeToFull:    %.1f min  (raw=0x%04X)", (double)mins, raw);
        }
    }

    ESP_LOGI(TAG, "");
}

// ────────────────────────────────────────────────────────────────────
// app_main
// ────────────────────────────────────────────────────────────────────

extern "C" void app_main()
{
    vTaskDelay(pdMS_TO_TICKS(1000));  // Let USB serial attach

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "MAX17205G I2C Test");
    ESP_LOGI(TAG, "");

    esp_err_t err = i2c_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return;
    }

    i2cScan();
    readMAX17205();

    // Re-read every 5 seconds
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        readMAX17205();
    }
}
