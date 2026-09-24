/**
 * I2S_Stream_TX_Test — 960 Hz with fixed 8-byte payload
 */
#include <Arduino.h>
#include <TR_I2S_Stream.h>
#include <RocketComputerTypes.h>
#include <esp_log.h>

static const char* TAG = "I2S_TX";

static constexpr int I2S_BCLK_PIN  = 27;
static constexpr int I2S_WS_PIN    = 28;
static constexpr int I2S_DOUT_PIN  = 23;
static constexpr int I2S_FSYNC_PIN = 17;

static constexpr uint32_t TX_INTERVAL_US = 1000000 / 960;  // ~1041 us

static TR_I2S_Stream i2s_stream;
static uint32_t tx_count = 0;
static uint32_t tx_errors = 0;
static uint32_t last_stats_ms = 0;

void setup()
{
    delay(500);
    ESP_LOGI(TAG, "=== I2S 960Hz TX Test (fixed 8-byte payload) ===");

    esp_err_t err = i2s_stream.beginMasterTx(
        I2S_BCLK_PIN, I2S_WS_PIN, I2S_DOUT_PIN, I2S_FSYNC_PIN);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2S init FAILED: %s", esp_err_to_name(err));
        while (true) delay(1000);
    }
    ESP_LOGI(TAG, "I2S TX ready");
    last_stats_ms = millis();
}

void loop()
{
    static uint32_t next_tx_us = 0;
    const uint32_t now_us = micros();

    if (now_us >= next_tx_us)
    {
        next_tx_us = now_us + TX_INTERVAL_US;

        // Same fixed payload as the 1 Hz test
        uint8_t payload[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

        esp_err_t err = i2s_stream.writeFrame(ISM6HG256_MSG, payload, sizeof(payload));
        if (err == ESP_OK)
            tx_count++;
        else
            tx_errors++;
    }
    else
    {
        // Yield periodically to feed watchdog
        static uint32_t yield_count = 0;
        yield_count++;
        if ((yield_count & 0x3F) == 0)
            vTaskDelay(1);
    }

    const uint32_t now_ms = millis();
    if ((now_ms - last_stats_ms) >= 1000)
    {
        ESP_LOGI(TAG, "[TX] frames=%lu  errors=%lu",
                 (unsigned long)tx_count, (unsigned long)tx_errors);
        tx_count = 0;
        tx_errors = 0;
        last_stats_ms = now_ms;
    }
}
