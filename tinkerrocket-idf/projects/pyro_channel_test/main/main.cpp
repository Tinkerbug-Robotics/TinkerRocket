/**
 * VN5E160STR-E Pyro Channel Test — ESP32-P4 (ESP-IDF)
 *
 * Circuit: ARM drives Q1 (DTC123J) → SI7615 P-FET → powers VN5E160 VCC.
 *          R36 (50k) between VCC and OUTPUT provides sense current.
 *          STATUS pin (pin 3) → R31 (1k) → PYRO_CONT GPIO.
 *          R34 (10k) pullup to 3V3 on STATUS line.
 *
 * Continuity logic (only valid when armed / VN5E160 powered):
 *   1 = load connected  (verified with Arduino test — LED connected reads HIGH)
 *   0 = open circuit    (no load)
 *
 * When disarmed the VN5E160 is unpowered — continuity reading is meaningless.
 *
 * Test sequence:
 *   1. Disarmed: read continuity (undefined — chip unpowered)
 *   2. Fire without arming (should do nothing — proves ARM safety)
 *   3. ARM: power on VN5E160, read continuity (expect 0 — load detected)
 *   4. FIRE for 2 s, then safe
 *   5. Re-arm, post-fire continuity (expect 1 if e-match burned, 0 if LED)
 *
 * Pin mapping:
 *   ARM        = GPIO 14   (output — enables P-FET to power VN5E160)
 *   FIRE       = GPIO 15   (output — VN5E160 INPUT pin, triggers output)
 *   CONTINUITY = GPIO 16   (input  — VN5E160 STATUS via R31)
 */

#include <cstdio>
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "PYRO_TEST";

// ── Pin configuration ──
static constexpr gpio_num_t PIN_ARM  = GPIO_NUM_14;
static constexpr gpio_num_t PIN_FIRE = GPIO_NUM_15;
static constexpr gpio_num_t PIN_CONT = GPIO_NUM_16;

// ── Timing ──
static constexpr uint32_t FIRE_DURATION_MS   = 2000;
static constexpr uint32_t STEP_DELAY_MS      = 2000;
static constexpr uint32_t SETTLE_DELAY_MS    = 100;

static void init_pins()
{
    // ARM — output, start LOW (safe)
    gpio_config_t arm_cfg = {};
    arm_cfg.pin_bit_mask = 1ULL << PIN_ARM;
    arm_cfg.mode         = GPIO_MODE_OUTPUT;
    arm_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    arm_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&arm_cfg);
    gpio_set_level(PIN_ARM, 0);

    // FIRE — output, start LOW (safe)
    gpio_config_t fire_cfg = {};
    fire_cfg.pin_bit_mask = 1ULL << PIN_FIRE;
    fire_cfg.mode         = GPIO_MODE_OUTPUT;
    fire_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    fire_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&fire_cfg);
    gpio_set_level(PIN_FIRE, 0);

    // CONTINUITY — input (external 10k pullup to 3V3 on PCB)
    gpio_config_t cont_cfg = {};
    cont_cfg.pin_bit_mask = 1ULL << PIN_CONT;
    cont_cfg.mode         = GPIO_MODE_INPUT;
    cont_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cont_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&cont_cfg);
}

static int read_continuity()
{
    return gpio_get_level(PIN_CONT);
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "=== VN5E160STR-E Pyro Channel Test ===");
    vTaskDelay(pdMS_TO_TICKS(1000));

    init_pins();

    // ── Step 1: Disarmed — VN5E160 unpowered, STATUS undefined ──
    ESP_LOGI(TAG, "--- Step 1: Disarmed (VN5E160 unpowered) ---");
    gpio_set_level(PIN_ARM, 0);
    gpio_set_level(PIN_FIRE, 0);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_DELAY_MS));

    int cont = read_continuity();
    ESP_LOGI(TAG, "Continuity = %d  (undefined — chip not powered)", cont);

    vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));

    // ── Step 2: FIRE without ARM — should do nothing ──
    ESP_LOGI(TAG, "--- Step 2: Fire without arming (VN5E160 unpowered) ---");
    gpio_set_level(PIN_FIRE, 1);
    vTaskDelay(pdMS_TO_TICKS(FIRE_DURATION_MS));
    gpio_set_level(PIN_FIRE, 0);
    ESP_LOGI(TAG, "Fire pin toggled — load should NOT have activated");

    vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));

    // ── Step 3: ARM — power VN5E160, 50k sense current flows through load ──
    ESP_LOGI(TAG, "--- Step 3: ARM (powering VN5E160) ---");
    gpio_set_level(PIN_ARM, 1);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_DELAY_MS));

    cont = read_continuity();
    ESP_LOGI(TAG, "Continuity = %d  (expected 1 — load connected)", cont);
    if (cont == 1) {
        ESP_LOGI(TAG, "PASS: Good continuity — load detected");
    } else {
        ESP_LOGW(TAG, "FAIL: No continuity — check e-match / load connection!");
    }

    vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));

    // ── Step 4: FIRE ──
    ESP_LOGI(TAG, "--- Step 4: FIRE ---");
    ESP_LOGW(TAG, "FIRING in 3...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGW(TAG, "2...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGW(TAG, "1...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGW(TAG, "FIRE!");
    gpio_set_level(PIN_FIRE, 1);
    vTaskDelay(pdMS_TO_TICKS(FIRE_DURATION_MS));

    // ── Safe ──
    gpio_set_level(PIN_FIRE, 0);
    gpio_set_level(PIN_ARM, 0);
    ESP_LOGI(TAG, "Fire complete — channel safed");

    // ── Step 5: Re-arm to check post-fire continuity ──
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "--- Step 5: Post-fire continuity (re-arming to read) ---");
    gpio_set_level(PIN_ARM, 1);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_DELAY_MS));

    cont = read_continuity();
    ESP_LOGI(TAG, "Post-fire continuity = %d  (expect 0 if e-match burned, 1 if LED still intact)", cont);

    gpio_set_level(PIN_ARM, 0);
    ESP_LOGI(TAG, "Channel safed");

    ESP_LOGI(TAG, "=== Test complete ===");

    // Idle forever
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
