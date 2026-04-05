#ifndef TR_I2S_STREAM_H
#define TR_I2S_STREAM_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2s_std.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <cstdint>
#include <cstddef>
#include <RocketComputerTypes.h>

/**
 * TR_I2S_Stream — raw byte-stream transport over I2S with DMA.
 *
 * Used as a unidirectional link from FlightComputer (master TX) to
 * OutComputer (slave RX) for high-frequency sensor telemetry.
 *
 * Framing reuses TR_I2C_Interface::packMessage() so both ends share
 * the same SOF + type + len + payload + CRC16 protocol.
 *
 * IMPORTANT: The I2S sample rate must provide enough bandwidth for the
 * total framed data throughput.  At 16-bit stereo, bandwidth = rate * 4.
 * Current sensor load is ~30 KB/s framed.  If sensor rates increase,
 * the I2S sample rate must be raised proportionally.
 *
 * An optional FRAME_SYNC GPIO is asserted HIGH while a framed message
 * is being written, giving the receiver a hardware resync signal.
 *
 * RX supports two modes:
 *   1. Polling: call readRaw() in a loop (simple but may re-read stale DMA)
 *   2. Callback: register on_recv callback via registerRecvCallback() for
 *      zero-copy DMA access — each callback = one fresh DMA buffer.
 */

/// Callback type for zero-copy DMA receive.
/// Called from ISR context when a DMA buffer completes.
/// buf points directly into the DMA descriptor — do NOT free it.
/// Return true if a higher-priority task was woken.
typedef bool (*i2s_recv_cb_t)(const uint8_t* buf, size_t len, void* user_ctx);

class TR_I2S_Stream
{
public:
    TR_I2S_Stream() = default;
    ~TR_I2S_Stream();

    // Non-copyable
    TR_I2S_Stream(const TR_I2S_Stream&) = delete;
    TR_I2S_Stream& operator=(const TR_I2S_Stream&) = delete;

    /**
     * Initialise as I2S master transmitter (FlightComputer side).
     *
     * @param bclk_pin   Bit clock output
     * @param ws_pin     Word select output
     * @param dout_pin   Data output
     * @param frame_sync_pin  Optional GPIO output pulsed per message (-1 to disable)
     * @param sample_rate     I2S sample rate in Hz (default 22050)
     * @param dma_desc_num    Number of DMA descriptors (default 8)
     * @param dma_frame_num   Frames per DMA descriptor (default 256)
     *
     * NOTE: sample_rate * 4 bytes (16-bit stereo) must exceed the total
     * framed sensor data rate.  Current load ~30 KB/s needs >= 8 kHz.
     * Using 10 kHz (40 KB/s) provides ~33% headroom.
     */
    esp_err_t beginMasterTx(int bclk_pin,
                            int ws_pin,
                            int dout_pin,
                            int frame_sync_pin = -1,
                            uint32_t sample_rate = 22050,
                            int dma_desc_num = 8,
                            int dma_frame_num = 256);

    /**
     * Initialise as I2S slave receiver (OutComputer side).
     *
     * @param bclk_pin   Bit clock input
     * @param ws_pin     Word select input
     * @param din_pin    Data input
     * @param frame_sync_pin  Optional GPIO input for resync (-1 to disable)
     * @param sample_rate     I2S sample rate in Hz (must match master)
     * @param dma_desc_num    Number of DMA descriptors (default 8)
     * @param dma_frame_num   Frames per DMA descriptor (default 256)
     */
    esp_err_t beginSlaveRx(int bclk_pin,
                           int ws_pin,
                           int din_pin,
                           int frame_sync_pin = -1,
                           uint32_t sample_rate = 22050,
                           int dma_desc_num = 8,
                           int dma_frame_num = 256);

    /**
     * Register a zero-copy receive callback (slave RX only).
     * The callback fires from ISR context each time a DMA buffer completes.
     * This eliminates stale DMA re-reads that cause data gaps.
     *
     * When using callbacks, do NOT call readRaw() — the callback provides
     * the data directly.  Use a task notification or queue to wake a
     * processing task from the callback.
     *
     * Must be called AFTER beginSlaveRx() and BEFORE data starts flowing.
     */
    esp_err_t registerRecvCallback(i2s_recv_cb_t cb, void* user_ctx);

    /**
     * Write a framed message to the I2S DMA stream.
     * Packs SOF + type + len + payload + CRC16, then writes via i2s_channel_write().
     * FRAME_SYNC GPIO is asserted during the write if configured.
     *
     * @return ESP_OK on success, or an error code.
     */
    esp_err_t writeFrame(uint8_t type,
                         const uint8_t* payload,
                         size_t len,
                         uint32_t timeout_ms = 100);

    /**
     * Write idle fill bytes (0x00) to keep the I2S clock running.
     * Call this when no telemetry messages are queued.
     */
    esp_err_t writeIdleFill(size_t num_bytes,
                            uint32_t timeout_ms = 100);

    /**
     * Write raw bytes to the I2S DMA TX buffer.
     */
    esp_err_t writeRaw(const uint8_t* data,
                       size_t len,
                       size_t* bytes_written = nullptr,
                       uint32_t timeout_ms = 100);

    /**
     * Read raw bytes from the I2S DMA RX buffer (blocking).
     * Blocks until data is available or timeout expires.
     * NOTE: Prefer registerRecvCallback() for zero-copy reception.
     */
    esp_err_t readRaw(uint8_t* buf,
                      size_t buf_len,
                      size_t* bytes_read,
                      uint32_t timeout_ms = portMAX_DELAY);

    /**
     * Read the FRAME_SYNC GPIO input level (slave side only).
     * Returns 1 if HIGH, 0 if LOW, -1 if not configured.
     */
    int readFrameSync() const;

    /** True if beginMasterTx() or beginSlaveRx() succeeded. */
    bool isInitialized() const { return chan_handle_ != nullptr; }

    /** Get the channel handle for advanced use (e.g. registering callbacks) */
    i2s_chan_handle_t getHandle() const { return chan_handle_; }

private:
    i2s_chan_handle_t chan_handle_ = nullptr;
    int frame_sync_pin_ = -1;
    bool is_tx_ = false;

    // Callback state
    i2s_recv_cb_t recv_cb_ = nullptr;
    void* recv_cb_ctx_ = nullptr;

    // ISR trampoline for the event callback
    static IRAM_ATTR bool onRecvISR(i2s_chan_handle_t handle,
                                     i2s_event_data_t* event,
                                     void* user_ctx);

    static constexpr size_t IDLE_BUF_SIZE = 256;
    uint8_t idle_buf_[IDLE_BUF_SIZE] = {};  // pre-zeroed
};

#endif // TR_I2S_STREAM_H
