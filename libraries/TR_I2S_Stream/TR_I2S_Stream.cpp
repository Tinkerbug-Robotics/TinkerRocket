#include "TR_I2S_Stream.h"
#include <TR_I2C_Interface.h>   // for packMessage() static method
#include <cstring>
#include <esp_log.h>

static const char* TAG = "I2S_STREAM";

TR_I2S_Stream::~TR_I2S_Stream()
{
    if (chan_handle_)
    {
        i2s_channel_disable(chan_handle_);
        i2s_del_channel(chan_handle_);
        chan_handle_ = nullptr;
    }
}

// ────────────────────────────────────────────────────────────────────
// Master TX (FlightComputer side)
// ────────────────────────────────────────────────────────────────────

esp_err_t TR_I2S_Stream::beginMasterTx(int bclk_pin,
                                        int ws_pin,
                                        int dout_pin,
                                        int frame_sync_pin,
                                        uint32_t sample_rate,
                                        int dma_desc_num,
                                        int dma_frame_num)
{
    is_tx_ = true;
    frame_sync_pin_ = frame_sync_pin;

    // ── FRAME_SYNC GPIO (output) ──
    if (frame_sync_pin_ >= 0)
    {
        gpio_config_t io_cfg = {};
        io_cfg.pin_bit_mask = 1ULL << frame_sync_pin_;
        io_cfg.mode = GPIO_MODE_OUTPUT;
        io_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        io_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_cfg.intr_type = GPIO_INTR_DISABLE;
        esp_err_t err = gpio_config(&io_cfg);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "FRAME_SYNC gpio_config: %s", esp_err_to_name(err));
            return err;
        }
        gpio_set_level(static_cast<gpio_num_t>(frame_sync_pin_), 0);
    }

    // ── I2S channel ──
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = dma_desc_num;
    chan_cfg.dma_frame_num = dma_frame_num;

    esp_err_t err = i2s_new_channel(&chan_cfg, &chan_handle_, nullptr);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2s_new_channel TX: %s", esp_err_to_name(err));
        return err;
    }

    // ── Standard mode config ──
    i2s_std_config_t std_cfg = {};

    // Clock
    std_cfg.clk_cfg.sample_rate_hz = sample_rate;
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    // Slot — 16-bit stereo Philips (raw byte transport).
    // NOTE: Bandwidth = sample_rate * 4 bytes.  Must exceed total framed
    // sensor data rate (~30 KB/s currently).  If sensor rates increase,
    // raise the sample_rate proportionally.
    std_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);

    // GPIO
    std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.bclk = static_cast<gpio_num_t>(bclk_pin);
    std_cfg.gpio_cfg.ws   = static_cast<gpio_num_t>(ws_pin);
    std_cfg.gpio_cfg.dout = static_cast<gpio_num_t>(dout_pin);
    std_cfg.gpio_cfg.din  = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.invert_flags.mclk_inv = false;
    std_cfg.gpio_cfg.invert_flags.bclk_inv = false;
    std_cfg.gpio_cfg.invert_flags.ws_inv   = false;

    err = i2s_channel_init_std_mode(chan_handle_, &std_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode TX: %s", esp_err_to_name(err));
        i2s_del_channel(chan_handle_);
        chan_handle_ = nullptr;
        return err;
    }

    err = i2s_channel_enable(chan_handle_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2s_channel_enable TX: %s", esp_err_to_name(err));
        i2s_del_channel(chan_handle_);
        chan_handle_ = nullptr;
        return err;
    }

    ESP_LOGI(TAG, "Master TX ready: BCLK=%d WS=%d DOUT=%d FSYNC=%d rate=%luHz bw=%luKB/s",
             bclk_pin, ws_pin, dout_pin, frame_sync_pin_,
             (unsigned long)sample_rate,
             (unsigned long)(sample_rate * 4 / 1024));
    return ESP_OK;
}

// ────────────────────────────────────────────────────────────────────
// Slave RX (OutComputer side)
// ────────────────────────────────────────────────────────────────────

esp_err_t TR_I2S_Stream::beginSlaveRx(int bclk_pin,
                                       int ws_pin,
                                       int din_pin,
                                       int frame_sync_pin,
                                       uint32_t sample_rate,
                                       int dma_desc_num,
                                       int dma_frame_num)
{
    is_tx_ = false;
    frame_sync_pin_ = frame_sync_pin;

    // ── FRAME_SYNC GPIO (input) ──
    if (frame_sync_pin_ >= 0)
    {
        gpio_config_t io_cfg = {};
        io_cfg.pin_bit_mask = 1ULL << frame_sync_pin_;
        io_cfg.mode = GPIO_MODE_INPUT;
        io_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        io_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_cfg.intr_type = GPIO_INTR_DISABLE;
        esp_err_t err = gpio_config(&io_cfg);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "FRAME_SYNC gpio_config: %s", esp_err_to_name(err));
            return err;
        }
    }

    // ── I2S channel ──
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    chan_cfg.dma_desc_num  = dma_desc_num;
    chan_cfg.dma_frame_num = dma_frame_num;

    esp_err_t err = i2s_new_channel(&chan_cfg, nullptr, &chan_handle_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2s_new_channel RX: %s", esp_err_to_name(err));
        return err;
    }

    // ── Standard mode config ──
    i2s_std_config_t std_cfg = {};

    // Clock
    std_cfg.clk_cfg.sample_rate_hz = sample_rate;
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    // Slot — 16-bit stereo Philips (must match master)
    std_cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);

    // GPIO
    std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.bclk = static_cast<gpio_num_t>(bclk_pin);
    std_cfg.gpio_cfg.ws   = static_cast<gpio_num_t>(ws_pin);
    std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.din  = static_cast<gpio_num_t>(din_pin);
    std_cfg.gpio_cfg.invert_flags.mclk_inv = false;
    std_cfg.gpio_cfg.invert_flags.bclk_inv = false;
    std_cfg.gpio_cfg.invert_flags.ws_inv   = false;

    err = i2s_channel_init_std_mode(chan_handle_, &std_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode RX: %s", esp_err_to_name(err));
        i2s_del_channel(chan_handle_);
        chan_handle_ = nullptr;
        return err;
    }

    err = i2s_channel_enable(chan_handle_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2s_channel_enable RX: %s", esp_err_to_name(err));
        i2s_del_channel(chan_handle_);
        chan_handle_ = nullptr;
        return err;
    }

    ESP_LOGI(TAG, "Slave RX ready: BCLK=%d WS=%d DIN=%d FSYNC=%d rate=%luHz",
             bclk_pin, ws_pin, din_pin, frame_sync_pin_, (unsigned long)sample_rate);
    return ESP_OK;
}

// ────────────────────────────────────────────────────────────────────
// Zero-copy receive callback
// ────────────────────────────────────────────────────────────────────

IRAM_ATTR bool TR_I2S_Stream::onRecvISR(i2s_chan_handle_t handle,
                                          i2s_event_data_t* event,
                                          void* user_ctx)
{
    TR_I2S_Stream* self = static_cast<TR_I2S_Stream*>(user_ctx);
    if (self->recv_cb_ && event->dma_buf && event->size > 0)
    {
        return self->recv_cb_(
            static_cast<const uint8_t*>(event->dma_buf),
            event->size,
            self->recv_cb_ctx_);
    }
    return false;
}

esp_err_t TR_I2S_Stream::registerRecvCallback(i2s_recv_cb_t cb, void* user_ctx)
{
    if (!chan_handle_ || is_tx_)
        return ESP_ERR_INVALID_STATE;

    recv_cb_ = cb;
    recv_cb_ctx_ = user_ctx;

    i2s_event_callbacks_t cbs = {};
    cbs.on_recv = onRecvISR;
    cbs.on_recv_q_ovf = nullptr;  // could add overflow detection later
    cbs.on_sent = nullptr;
    cbs.on_send_q_ovf = nullptr;

    // Must disable channel to register callbacks, then re-enable
    esp_err_t err = i2s_channel_disable(chan_handle_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "disable for callback: %s", esp_err_to_name(err));
        return err;
    }

    err = i2s_channel_register_event_callback(chan_handle_, &cbs, this);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "register_event_callback: %s", esp_err_to_name(err));
        i2s_channel_enable(chan_handle_);  // re-enable even on failure
        return err;
    }

    err = i2s_channel_enable(chan_handle_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "re-enable after callback: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "RX recv callback registered (zero-copy DMA)");
    return ESP_OK;
}

// ────────────────────────────────────────────────────────────────────
// Write a framed message
// ────────────────────────────────────────────────────────────────────

esp_err_t TR_I2S_Stream::writeFrame(uint8_t type,
                                     const uint8_t* payload,
                                     size_t len,
                                     uint32_t timeout_ms)
{
    if (!chan_handle_ || !is_tx_)
        return ESP_ERR_INVALID_STATE;

    uint8_t frame[MAX_FRAME];
    size_t frame_len = 0;

    if (!TR_I2C_Interface::packMessage(type, payload, len, frame, sizeof(frame), frame_len))
        return ESP_ERR_INVALID_SIZE;

    // Assert FRAME_SYNC
    if (frame_sync_pin_ >= 0)
        gpio_set_level(static_cast<gpio_num_t>(frame_sync_pin_), 1);

    size_t bytes_written = 0;
    esp_err_t err = i2s_channel_write(chan_handle_, frame, frame_len,
                                       &bytes_written, pdMS_TO_TICKS(timeout_ms));

    // De-assert FRAME_SYNC
    if (frame_sync_pin_ >= 0)
        gpio_set_level(static_cast<gpio_num_t>(frame_sync_pin_), 0);

    if (err != ESP_OK)
        return err;

    return (bytes_written == frame_len) ? ESP_OK : ESP_ERR_TIMEOUT;
}

// ────────────────────────────────────────────────────────────────────
// Idle fill
// ────────────────────────────────────────────────────────────────────

esp_err_t TR_I2S_Stream::writeIdleFill(size_t num_bytes, uint32_t timeout_ms)
{
    if (!chan_handle_ || !is_tx_)
        return ESP_ERR_INVALID_STATE;

    size_t remaining = num_bytes;
    while (remaining > 0)
    {
        const size_t chunk = (remaining < IDLE_BUF_SIZE) ? remaining : IDLE_BUF_SIZE;
        size_t written = 0;
        esp_err_t err = i2s_channel_write(chan_handle_, idle_buf_, chunk,
                                           &written, pdMS_TO_TICKS(timeout_ms));
        if (err != ESP_OK)
            return err;
        remaining -= written;
        if (written == 0)
            break;  // timeout
    }
    return ESP_OK;
}

// ────────────────────────────────────────────────────────────────────
// Raw write / read
// ────────────────────────────────────────────────────────────────────

esp_err_t TR_I2S_Stream::writeRaw(const uint8_t* data,
                                   size_t len,
                                   size_t* bytes_written,
                                   uint32_t timeout_ms)
{
    if (!chan_handle_ || !is_tx_)
        return ESP_ERR_INVALID_STATE;

    size_t written = 0;
    esp_err_t err = i2s_channel_write(chan_handle_, data, len,
                                       &written, pdMS_TO_TICKS(timeout_ms));
    if (bytes_written)
        *bytes_written = written;
    return err;
}

esp_err_t TR_I2S_Stream::readRaw(uint8_t* buf,
                                  size_t buf_len,
                                  size_t* bytes_read,
                                  uint32_t timeout_ms)
{
    if (!chan_handle_ || is_tx_)
        return ESP_ERR_INVALID_STATE;

    return i2s_channel_read(chan_handle_, buf, buf_len,
                             bytes_read, pdMS_TO_TICKS(timeout_ms));
}

// ────────────────────────────────────────────────────────────────────
// FRAME_SYNC read (slave side)
// ────────────────────────────────────────────────────────────────────

int TR_I2S_Stream::readFrameSync() const
{
    if (frame_sync_pin_ < 0)
        return -1;
    return gpio_get_level(static_cast<gpio_num_t>(frame_sync_pin_));
}
