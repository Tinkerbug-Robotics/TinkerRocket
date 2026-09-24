/**
 * GPIO_Wire_Test_TX — FlightComputer (ESP32-P4)
 *
 * Toggles each of the 4 I2S GPIO pins one at a time with a distinctive
 * pattern so the RX side can confirm which wires are connected.
 *
 * Pin 27 (BCLK):  toggles 1x per cycle
 * Pin 28 (WS):    toggles 2x per cycle
 * Pin 23 (DOUT):  toggles 3x per cycle
 * Pin 17 (FSYNC): toggles 4x per cycle
 */

#include <Arduino.h>
#include <esp_log.h>

static const char* TAG = "GPIO_TX";

static constexpr int PINS[] = { 27, 28, 23, 17 };
static constexpr const char* NAMES[] = { "BCLK", "WS", "DOUT", "FSYNC" };
static constexpr int NUM_PINS = 4;

void setup()
{
    delay(500);
    ESP_LOGI(TAG, "=== GPIO Wire Test TX (FlightComputer) ===");

    for (int i = 0; i < NUM_PINS; i++)
    {
        pinMode(PINS[i], OUTPUT);
        digitalWrite(PINS[i], LOW);
        ESP_LOGI(TAG, "  Pin %d (%s) -> OUTPUT LOW", PINS[i], NAMES[i]);
    }

    ESP_LOGI(TAG, "Starting toggle pattern...");
}

void loop()
{
    // Test each pin individually: set all LOW, then toggle one pin
    for (int p = 0; p < NUM_PINS; p++)
    {
        // All LOW
        for (int i = 0; i < NUM_PINS; i++)
            digitalWrite(PINS[i], LOW);

        delay(200);

        // Toggle pin p with its unique count
        int toggles = p + 1;
        for (int t = 0; t < toggles; t++)
        {
            digitalWrite(PINS[p], HIGH);
            delay(150);
            digitalWrite(PINS[p], LOW);
            delay(150);
        }

        ESP_LOGI(TAG, "Toggled %s (pin %d) %dx", NAMES[p], PINS[p], toggles);
    }

    ESP_LOGI(TAG, "--- cycle complete ---");
    delay(500);
}
