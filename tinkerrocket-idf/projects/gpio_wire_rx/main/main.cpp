/**
 * GPIO_Wire_Test_RX — OutComputer (ESP32-S3)
 *
 * Reads the 4 GPIO pins connected to the FlightComputer and reports
 * their state every 100ms.  Use alongside GPIO_Wire_Test_TX to verify
 * physical connections.
 *
 * Expected mapping:
 *   FC pin 27 (BCLK)  → OC pin 1
 *   FC pin 28 (WS)    → OC pin 2
 *   FC pin 23 (DOUT)  → OC pin 21
 *   FC pin 17 (FSYNC) → OC pin 45
 */

#include <Arduino.h>
#include <esp_log.h>

static const char* TAG = "GPIO_RX";

static constexpr int PINS[] = { 1, 2, 21, 45 };
static constexpr const char* NAMES[] = { "BCLK", "WS", "DIN", "FSYNC" };
static constexpr int NUM_PINS = 4;

// Power rail control
static constexpr int PWR_PIN = 6;

// Edge counters
static uint32_t rise_count[4] = {};
static int prev_state[4] = {};

void setup()
{
    delay(500);
    ESP_LOGI(TAG, "=== GPIO Wire Test RX (OutComputer) ===");

    // Power on FC — same sequence as OutComputer to avoid brownout
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(100);
    digitalWrite(PWR_PIN, HIGH);
    ESP_LOGI(TAG, "PWR_PIN %d -> HIGH", PWR_PIN);
    vTaskDelay(pdMS_TO_TICKS(500));  // power rail stabilize
    vTaskDelay(1);                    // feed watchdog
    delay(2000);                      // wait for FC to boot

    for (int i = 0; i < NUM_PINS; i++)
    {
        pinMode(PINS[i], INPUT_PULLDOWN);
        prev_state[i] = digitalRead(PINS[i]);
        ESP_LOGI(TAG, "  Pin %d (%s) -> INPUT_PULLDOWN, initial=%d",
                 PINS[i], NAMES[i], prev_state[i]);
    }

    ESP_LOGI(TAG, "Monitoring pins... (expecting toggle pattern from TX)");
}

void loop()
{
    // Sample pins rapidly and count rising edges
    for (int sample = 0; sample < 100; sample++)
    {
        for (int i = 0; i < NUM_PINS; i++)
        {
            int s = digitalRead(PINS[i]);
            if (s == 1 && prev_state[i] == 0)
                rise_count[i]++;
            prev_state[i] = s;
        }
        delayMicroseconds(500);  // 2 kHz sample rate per pin
    }

    // Print state every ~50ms
    static uint32_t last_print = 0;
    uint32_t now = millis();
    if (now - last_print >= 200)
    {
        int states[4];
        for (int i = 0; i < NUM_PINS; i++)
            states[i] = digitalRead(PINS[i]);

        ESP_LOGI(TAG, "BCLK(1)=%d[%lu]  WS(2)=%d[%lu]  DIN(21)=%d[%lu]  FSYNC(45)=%d[%lu]",
                 states[0], (unsigned long)rise_count[0],
                 states[1], (unsigned long)rise_count[1],
                 states[2], (unsigned long)rise_count[2],
                 states[3], (unsigned long)rise_count[3]);

        last_print = now;
    }
}
