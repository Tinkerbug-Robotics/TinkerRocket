#ifndef TR_OTA_BACKEND_ESP_H
#define TR_OTA_BACKEND_ESP_H

#include "TR_OTA_Backend.h"

#include <esp_ota_ops.h>
#include <esp_partition.h>

// Real ESP-IDF backend. Wraps esp_ota_begin / esp_ota_write / esp_ota_end /
// esp_ota_set_boot_partition. Picks the next OTA partition via
// esp_ota_get_next_update_partition().
class TR_OTA_Backend_esp : public TR_OTA_Backend
{
public:
    TR_OTA_Backend_esp() = default;
    ~TR_OTA_Backend_esp() override;

    int  begin(size_t image_size) override;
    int  write(const uint8_t* data, size_t len) override;
    int  end() override;
    int  setBootPartition() override;
    void abort() override;

private:
    const esp_partition_t* partition_ = nullptr;
    esp_ota_handle_t       handle_ = 0;
    bool                   session_active_ = false;
};

#endif // TR_OTA_BACKEND_ESP_H
