#include "TR_OTA_Backend_esp.h"

#include <esp_log.h>

static const char* TAG = "TR_OTA_BE";

TR_OTA_Backend_esp::~TR_OTA_Backend_esp()
{
    abort();
}

int TR_OTA_Backend_esp::begin(size_t image_size)
{
    if (session_active_) return -1;

    partition_ = esp_ota_get_next_update_partition(nullptr);
    if (!partition_)
    {
        ESP_LOGE(TAG, "esp_ota_get_next_update_partition returned null");
        return -2;
    }

    esp_err_t err = esp_ota_begin(partition_, image_size, &handle_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin(size=%u): %s", (unsigned)image_size, esp_err_to_name(err));
        partition_ = nullptr;
        return err;
    }

    session_active_ = true;
    ESP_LOGI(TAG, "OTA begin: partition '%s' @ 0x%08x, size=%u",
             partition_->label, (unsigned)partition_->address, (unsigned)image_size);
    return 0;
}

int TR_OTA_Backend_esp::write(const uint8_t* data, size_t len)
{
    if (!session_active_) return -1;
    esp_err_t err = esp_ota_write(handle_, data, len);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_write(len=%u): %s", (unsigned)len, esp_err_to_name(err));
        return err;
    }
    return 0;
}

int TR_OTA_Backend_esp::end()
{
    if (!session_active_) return -1;
    esp_err_t err = esp_ota_end(handle_);
    session_active_ = false;
    handle_ = 0;
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end: %s", esp_err_to_name(err));
        return err;
    }
    return 0;
}

int TR_OTA_Backend_esp::setBootPartition()
{
    if (!partition_) return -1;
    esp_err_t err = esp_ota_set_boot_partition(partition_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition('%s'): %s",
                 partition_->label, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "OTA boot partition set: '%s'", partition_->label);
    return 0;
}

void TR_OTA_Backend_esp::abort()
{
    if (session_active_)
    {
        esp_ota_abort(handle_);
        session_active_ = false;
        handle_ = 0;
    }
    partition_ = nullptr;
}
