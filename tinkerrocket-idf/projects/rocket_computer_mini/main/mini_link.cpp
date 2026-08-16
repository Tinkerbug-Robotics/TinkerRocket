#include "mini_link.h"

#include <string.h>
#include <atomic>

#include "esp_log.h"
#include <TR_I2C_Interface.h>   // packMessage() static only — no bus is created
#include <TR_LogToFlash.h>

static const char* TAG = "mini_link";

// Defined in main.cpp alongside the other global objects.
extern TR_LogToFlash logger;

namespace mini_link {

QueueHandle_t cmd_queue = nullptr;
TelemState telem = {};
portMUX_TYPE telem_mux = portMUX_INITIALIZER_UNLOCKED;

static std::atomic<bool> log_sink_enabled{false};

void init()
{
    cmd_queue = xQueueCreate(CMD_QUEUE_DEPTH, sizeof(CmdFrame));
    configASSERT(cmd_queue != nullptr);
}

bool sendCommand(uint8_t type, const uint8_t* payload, uint8_t len)
{
    if (cmd_queue == nullptr || len > MAX_PAYLOAD) {
        return false;
    }
    CmdFrame frame;
    frame.type = type;
    frame.len = len;
    if (len > 0) {
        memcpy(frame.payload, payload, len);
    }
    if (xQueueSend(cmd_queue, &frame, 0) != pdTRUE) {
        ESP_LOGW(TAG, "cmd_queue full, dropped type 0x%02X", type);
        return false;
    }
    return true;
}

void enableLogSink(bool on)
{
    log_sink_enabled.store(on, std::memory_order_release);
}

bool logFrame(uint8_t type, const uint8_t* payload, uint8_t len)
{
    if (!log_sink_enabled.load(std::memory_order_acquire)) {
        return false;
    }
    uint8_t frame[MAX_FRAME];
    size_t frame_len = 0;
    if (!TR_I2C_Interface::packMessage(type, payload, len, frame, sizeof(frame), frame_len)) {
        return false;
    }
    return logger.enqueueFrame(frame, frame_len);
}

}  // namespace mini_link
