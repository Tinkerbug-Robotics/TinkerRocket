#ifndef TR_OTA_BACKEND_H
#define TR_OTA_BACKEND_H

#include <cstdint>
#include <cstddef>

// Pure-virtual flash-side of an OTA session. The real ESP-IDF
// implementation wraps esp_ota_begin / esp_ota_write / esp_ota_end /
// esp_ota_set_boot_partition. Tests inject a fake that just tracks bytes.
//
// Owning, single-session: at most one begin() in flight at a time. abort()
// is always safe to call. Return values are 0 on success, non-zero on
// failure (errors are surfaced as TR_OTA_Receiver::Error::WriteFailed etc
// rather than propagated verbatim).
class TR_OTA_Backend
{
public:
    virtual ~TR_OTA_Backend() = default;

    virtual int begin(size_t image_size) = 0;
    virtual int write(const uint8_t* data, size_t len) = 0;
    virtual int end() = 0;
    virtual int setBootPartition() = 0;
    virtual void abort() = 0;
};

#endif // TR_OTA_BACKEND_H
