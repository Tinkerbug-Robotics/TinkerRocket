// ==========================================================================
// Power consumption test — minimal firmware
//
// Incrementally enable features to find the 10mA → 20mA regression.
// Uncomment one #define at a time, rebuild, and check current.
//
// TEST 1: CPU 40MHz + INA230 only           → expect ~11 mA
// TEST 2: Enable BLE                        → expect ?
// TEST 3: Enable light sleep                → expect ?
// TEST 4: Raise CPU to 80 MHz               → expect ?
// ==========================================================================

// --- Feature toggles — uncomment one at a time ---
#define ENABLE_BLE
#define ENABLE_LIGHT_SLEEP
#define CPU_MAX_MHZ  80    // TEST 4: 80 MHz

#include <compat.h>
#include <cstdio>
#include <cmath>
#include <esp_pm.h>
#include <esp_bt.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <rom/ets_sys.h>

#include "config.h"
#include <TR_INA230.h>
#include <TR_Sensor_Data_Converter.h>

#ifdef ENABLE_BLE
#include <TR_BLE_To_APP.h>
#include "host/ble_gap.h"
static TR_BLE_To_APP ble_app("TinkerRocket");
#endif

// INA230
static i2c_master_bus_handle_t ina230_bus = nullptr;
static TR_INA230 ina230(0x40);
static bool ina230_ok = false;
static constexpr float INA230_R_SHUNT_OHM   = 0.002f;
static constexpr float INA230_CURRENT_LSB_A  = 0.001f;
static SensorConverter sensor_converter;

static void init_ina230()
{
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_1;
    bus_cfg.sda_io_num = static_cast<gpio_num_t>(config::PWR_SDA);
    bus_cfg.scl_io_num = static_cast<gpio_num_t>(config::PWR_SCL);
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = false;

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &ina230_bus);
    if (err != ESP_OK) {
        ESP_LOGW("PWR", "I2C bus init failed: %s", esp_err_to_name(err));
        return;
    }

    if (ina230.begin(ina230_bus, 400000) == TR_INA230_OK) {
        ina230.setConfiguration(INA230_Avg::AVG_1,
                                INA230_ConvTime::CT_332us,
                                INA230_ConvTime::CT_332us,
                                INA230_Mode::POWER_DOWN);
        ina230.calibrate(INA230_R_SHUNT_OHM, INA230_CURRENT_LSB_A);
        ina230.enableConversionReadyAlert(true);
        ina230_ok = true;
        ESP_LOGI("PWR", "INA230 OK");
    } else {
        ESP_LOGW("PWR", "INA230 not found");
    }
}

static void read_and_print_power()
{
    if (!ina230_ok) {
        ESP_LOGW("PWR", "INA230 not available");
        return;
    }

    ina230.setMode(INA230_Mode::SHUNT_BUS_TRIG);
    for (int i = 0; i < 20; i++) {
        delayMicroseconds(100);
        uint16_t me = 0;
        if (ina230.readMaskEnable(&me) == TR_INA230_OK && (me & (1 << 3)))
            break;
    }

    float bus_v = 0.0f, current_a = 0.0f;
    if (ina230.readBusVoltage_V(&bus_v) != TR_INA230_OK) return;
    if (ina230.readCurrent_A(&current_a) != TR_INA230_OK) return;

    float current_ma = current_a * 1000.0f;
    ESP_LOGW("POWER", "%.1f V  %.1f mA  (CPU %lu MHz)", bus_v, current_ma,
             (unsigned long)ets_get_cpu_frequency());
}

extern "C" void app_main(void)
{
    // NVS init
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Power rail OFF
    pinMode(config::PWR_PIN, OUTPUT);
    digitalWrite(config::PWR_PIN, LOW);

    ESP_LOGW("TEST", "========================================");
    ESP_LOGW("TEST", "  POWER CONSUMPTION TEST");
    ESP_LOGW("TEST", "  CPU max: %d MHz", CPU_MAX_MHZ);
#ifdef ENABLE_BLE
    ESP_LOGW("TEST", "  BLE: ON");
#else
    ESP_LOGW("TEST", "  BLE: OFF");
#endif
#ifdef ENABLE_LIGHT_SLEEP
    ESP_LOGW("TEST", "  Light sleep: ON");
#else
    ESP_LOGW("TEST", "  Light sleep: OFF");
#endif
    ESP_LOGW("TEST", "========================================");

    // INA230 power monitor
    init_ina230();

#ifdef ENABLE_BLE
    if (!ble_app.begin()) {
        ESP_LOGE("BLE", "BLE failed to start");
    }
    esp_bt_sleep_enable();
    ESP_LOGI("BLE", "BLE advertising started");
#endif

    // Power management
#if defined(CONFIG_PM_ENABLE)
    esp_pm_config_t pm_cfg = {};
    pm_cfg.max_freq_mhz = CPU_MAX_MHZ;
    pm_cfg.min_freq_mhz = 40;
#ifdef ENABLE_LIGHT_SLEEP
    pm_cfg.light_sleep_enable = true;
#else
    pm_cfg.light_sleep_enable = false;
#endif
    esp_err_t pm_err = esp_pm_configure(&pm_cfg);
    if (pm_err == ESP_OK)
        ESP_LOGI("PWR", "PM configured");
    else
        ESP_LOGE("PWR", "PM failed: %s", esp_err_to_name(pm_err));
#endif

    ESP_LOGW("TEST", "Entering idle loop — readings every 5s");

    uint32_t iteration = 0;
    while (true) {
        read_and_print_power();

        // PM lock dump every 30s
#if defined(CONFIG_PM_ENABLE)
        if (++iteration % 6 == 0) {
            esp_pm_dump_locks(stdout);
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
