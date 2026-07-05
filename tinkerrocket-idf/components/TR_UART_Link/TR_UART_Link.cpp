#include "TR_UART_Link.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "TR_UART_Link";

esp_err_t TR_UART_Link::begin(const Config& cfg)
{
    cfg_ = cfg;

    const uart_config_t uart_cfg = {
        .baud_rate = cfg.baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        // Frame-level flow control is in-band (TX_RESULT credits,
        // RadioModemProtocol.h) — no RTS/CTS wires on the daughterboard
        // connector.
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(cfg.port,
                                        static_cast<int>(cfg.rx_ring),
                                        static_cast<int>(cfg.tx_ring),
                                        0, nullptr, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_param_config(cfg.port, &uart_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_set_pin(cfg.port, cfg.tx_pin, cfg.rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    began_ = true;
    return ESP_OK;
}

bool TR_UART_Link::sendFrame(uint8_t type, const uint8_t* payload, size_t len,
                             uint32_t timeout_ms)
{
    if (!began_)
    {
        return false;
    }

    size_t frame_len = 0;
    if (!tr_msg::pack(type, payload, len, tx_frame_, sizeof(tx_frame_), frame_len))
    {
        stats_.tx_fails++;
        return false;
    }

    // uart_write_bytes copies into the driver TX ring; it only blocks when
    // the ring is full. Guard with a bounded wait for free space so a wedged
    // link degrades to counted drops instead of stalling the caller's loop.
    size_t free_space = 0;
    if (uart_get_tx_buffer_free_size(cfg_.port, &free_space) == ESP_OK &&
        free_space < frame_len)
    {
        const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
        while (free_space < frame_len)
        {
            if (xTaskGetTickCount() >= deadline)
            {
                stats_.tx_fails++;
                return false;
            }
            vTaskDelay(1);
            if (uart_get_tx_buffer_free_size(cfg_.port, &free_space) != ESP_OK)
            {
                break;
            }
        }
    }

    const int written = uart_write_bytes(cfg_.port, tx_frame_, frame_len);
    if (written != static_cast<int>(frame_len))
    {
        stats_.tx_fails++;
        return false;
    }
    stats_.tx_frames++;
    return true;
}

void TR_UART_Link::poll(FrameHandler handler, void* ctx)
{
    if (!began_)
    {
        return;
    }

    for (;;)
    {
        const int n = uart_read_bytes(cfg_.port, rx_chunk_, sizeof(rx_chunk_), 0);
        if (n <= 0)
        {
            break;
        }
        for (int i = 0; i < n; i++)
        {
            if (deframer_.feed(rx_chunk_[i]))
            {
                handler(ctx, deframer_.type(), deframer_.payload(),
                        deframer_.payloadLen());
            }
        }
    }

    // Mirror deframer counters into link stats so one getStats() call tells
    // the whole RX health story.
    const tr_msg::MsgDeframer::Stats& ds = deframer_.stats();
    stats_.rx_frames = ds.frames;
    stats_.rx_crc_fails = ds.crc_fails;
    stats_.rx_resync_bytes = ds.resync_bytes;
}

void TR_UART_Link::getStats(Stats& out) const
{
    out = stats_;
}
