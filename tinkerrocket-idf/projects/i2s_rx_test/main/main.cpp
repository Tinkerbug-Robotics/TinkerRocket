/**
 * I2S_Stream_RX_Test — 960 Hz with fixed 8-byte payload
 * Parses frames using SOF scan, counts good/bad/resync.
 */
#include <Arduino.h>
#include <TR_I2S_Stream.h>
#include <TR_I2C_Interface.h>
#include <RocketComputerTypes.h>
#include <esp_log.h>
#include <soc/rtc_cntl_reg.h>

static const char* TAG = "I2S_RX";

// Actual wiring: FC 27→OC 21, FC 28→OC 45, FC 23→OC 2, FC 17→OC 1
static constexpr int I2S_BCLK_PIN  = 21;
static constexpr int I2S_WS_PIN    = 45;
static constexpr int I2S_DIN_PIN   = 2;
static constexpr int I2S_FSYNC_PIN = 1;
static constexpr int PWR_PIN       = 6;

static TR_I2S_Stream i2s_stream;

// ── Ring buffer ──
static constexpr size_t RX_RING_SIZE = 16384;
static uint8_t rx_ring[RX_RING_SIZE];
static volatile size_t rx_head = 0;
static size_t rx_tail = 0;

static inline void rxPush(uint8_t b)
{
    const size_t next = (rx_head + 1) % RX_RING_SIZE;
    if (next != rx_tail) { rx_ring[rx_head] = b; rx_head = next; }
}
static inline size_t rxAvailable()
{
    const size_t h = rx_head;
    return (h >= rx_tail) ? (h - rx_tail) : (RX_RING_SIZE - rx_tail + h);
}
static inline uint8_t rxPeek(size_t offset) { return rx_ring[(rx_tail + offset) % RX_RING_SIZE]; }
static inline void rxConsume(size_t n) { rx_tail = (rx_tail + n) % RX_RING_SIZE; }

// ── Stats ──
static uint32_t rx_frames = 0;
static uint32_t rx_crc_errors = 0;
static uint32_t rx_resync_drops = 0;
static uint32_t rx_payload_bad = 0;  // payload doesn't match expected pattern
static uint32_t last_stats_ms = 0;

// Expected frame: AA 55 AA 55 A2 08 01 02 03 04 05 06 07 08 CD 48
static const uint8_t EXPECTED_PAYLOAD[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

static void parseRxStream()
{
    while (rxAvailable() >= 8)
    {
        if (rxPeek(0) != 0xAA || rxPeek(1) != 0x55 ||
            rxPeek(2) != 0xAA || rxPeek(3) != 0x55)
        {
            rxConsume(1);
            rx_resync_drops++;
            continue;
        }

        const uint8_t payload_len = rxPeek(5);
        if (payload_len > MAX_PAYLOAD) { rxConsume(1); rx_resync_drops++; continue; }

        const size_t frame_len = 4 + 1 + 1 + payload_len + 2;
        if (rxAvailable() < frame_len) break;

        uint8_t frame[MAX_FRAME];
        for (size_t i = 0; i < frame_len; i++) frame[i] = rxPeek(i);

        uint8_t type = 0;
        uint8_t payload[MAX_PAYLOAD];
        size_t payload_out_len = 0;

        if (TR_I2C_Interface::unpackMessage(frame, frame_len,
                                             type, payload, sizeof(payload),
                                             payload_out_len, true))
        {
            rx_frames++;
            // Verify payload matches expected pattern
            if (payload_out_len == 8 && memcmp(payload, EXPECTED_PAYLOAD, 8) != 0)
                rx_payload_bad++;
            rxConsume(frame_len);
        }
        else
        {
            rx_crc_errors++;
            rxConsume(1);
        }
    }
}

static void i2sReceiverTask(void*)
{
    uint8_t dma_buf[1024];
    size_t bytes_read = 0;

    for (;;)
    {
        esp_err_t err = i2s_stream.readRaw(dma_buf, sizeof(dma_buf),
                                            &bytes_read, portMAX_DELAY);
        if (err == ESP_OK && bytes_read > 0)
        {
            for (size_t i = 0; i < bytes_read; i++)
                rxPush(dma_buf[i]);
            parseRxStream();
        }
    }
}

void setup()
{
    delay(500);
    ESP_LOGI(TAG, "=== I2S 960Hz RX Test ===");

    CLEAR_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
    gpio_config_t pwr_cfg = {};
    pwr_cfg.pin_bit_mask = 1ULL << PWR_PIN;
    pwr_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&pwr_cfg);
    gpio_set_level(static_cast<gpio_num_t>(PWR_PIN), 1);
    ESP_LOGI(TAG, "PWR_PIN -> HIGH");
    for (int i = 0; i < 10; i++) vTaskDelay(pdMS_TO_TICKS(50));
    SET_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);

    esp_err_t err = i2s_stream.beginSlaveRx(
        I2S_BCLK_PIN, I2S_WS_PIN, I2S_DIN_PIN, I2S_FSYNC_PIN);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2S init FAILED: %s", esp_err_to_name(err));
        while (true) delay(1000);
    }
    ESP_LOGI(TAG, "I2S RX ready");

    xTaskCreatePinnedToCore(i2sReceiverTask, "i2s_rx", 4096,
                            nullptr, 3, nullptr, 0);
    last_stats_ms = millis();
}

void loop()
{
    const uint32_t now_ms = millis();
    if ((now_ms - last_stats_ms) >= 1000)
    {
        ESP_LOGI(TAG, "[RX] frames=%lu  crc_err=%lu  resync=%lu  payload_bad=%lu  ring=%lu",
                 (unsigned long)rx_frames,
                 (unsigned long)rx_crc_errors,
                 (unsigned long)rx_resync_drops,
                 (unsigned long)rx_payload_bad,
                 (unsigned long)rxAvailable());
        rx_frames = 0;
        rx_crc_errors = 0;
        rx_resync_drops = 0;
        rx_payload_bad = 0;
        last_stats_ms = now_ms;
    }
    delay(100);
}
