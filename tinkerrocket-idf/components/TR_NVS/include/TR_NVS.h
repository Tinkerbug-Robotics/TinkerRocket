/**
 * TR_NVS.h — Drop-in replacement for Arduino Preferences class
 *
 * Wraps ESP-IDF nvs_flash API with an interface matching
 * the Arduino Preferences class used throughout the codebase.
 *
 * Failures are reported, not swallowed (#500).  This class used to ignore
 * every nvs_* return code, so a base station whose NVS had never been
 * initialised read back the caller's defaults and dropped every write on
 * the floor without a single log line.  Reads that miss return the default
 * (a key that was never written is normal); anything else is logged.
 */
#pragma once

#include <nvs_flash.h>
#include <nvs.h>
#include <string.h>
#include <esp_log.h>

class Preferences {
public:
    Preferences() : handle_(0), opened_(false), read_only_(false) {}

    ~Preferences() { end(); }

    bool begin(const char* ns, bool readOnly = false)
    {
        if (opened_) end();
        esp_err_t err = nvs_open(ns, readOnly ? NVS_READONLY : NVS_READWRITE, &handle_);
        opened_    = (err == ESP_OK);
        read_only_ = readOnly;
        if (!opened_)
        {
            handle_ = 0;
            // A read-only open of a namespace nobody has written yet is an
            // ordinary "nothing saved" — callers seed defaults and move on.
            // Every other failure means NVS itself is unusable.
            if (readOnly && err == ESP_ERR_NVS_NOT_FOUND)
                ESP_LOGD(TAG_, "namespace '%s' not present yet", ns);
            else
                ESP_LOGE(TAG_, "open('%s') failed: %s — settings will NOT persist",
                         ns, esp_err_to_name(err));
        }
        return opened_;
    }

    void end()
    {
        if (opened_) {
            if (!read_only_) {
                esp_err_t err = nvs_commit(handle_);
                if (err != ESP_OK)
                    ESP_LOGE(TAG_, "commit failed: %s — changes lost", esp_err_to_name(err));
            }
            nvs_close(handle_);
            opened_ = false;
            handle_ = 0;
        }
    }

    /* ── Getters ──────────────────────────────────────── */

    bool getBool(const char* key, bool def = false)
    {
        return getUChar(key, def ? 1 : 0) != 0;
    }

    int8_t getChar(const char* key, int8_t def = 0)
    {
        return (int8_t)getUChar(key, (uint8_t)def);
    }

    uint8_t getUChar(const char* key, uint8_t def = 0)
    {
        if (!opened_) return def;
        uint8_t val;
        return readFailed_(key, nvs_get_u8(handle_, key, &val)) ? def : val;
    }

    int16_t getShort(const char* key, int16_t def = 0)
    {
        return (int16_t)getUShort(key, (uint16_t)def);
    }

    uint16_t getUShort(const char* key, uint16_t def = 0)
    {
        if (!opened_) return def;
        uint16_t val;
        return readFailed_(key, nvs_get_u16(handle_, key, &val)) ? def : val;
    }

    int32_t getInt(const char* key, int32_t def = 0)
    {
        return (int32_t)getUInt(key, (uint32_t)def);
    }

    uint32_t getUInt(const char* key, uint32_t def = 0)
    {
        if (!opened_) return def;
        uint32_t val;
        return readFailed_(key, nvs_get_u32(handle_, key, &val)) ? def : val;
    }

    float getFloat(const char* key, float def = 0.0f)
    {
        if (!opened_) return def;
        uint32_t raw;
        if (readFailed_(key, nvs_get_u32(handle_, key, &raw))) return def;
        float result;
        memcpy(&result, &raw, sizeof(result));
        return result;
    }

    size_t getBytesLength(const char* key)
    {
        if (!opened_) return 0;
        size_t len = 0;
        return readFailed_(key, nvs_get_blob(handle_, key, NULL, &len)) ? 0 : len;
    }

    size_t getBytes(const char* key, void* buf, size_t maxLen)
    {
        if (!opened_) return 0;
        size_t len = maxLen;
        return readFailed_(key, nvs_get_blob(handle_, key, buf, &len)) ? 0 : len;
    }

    bool isKey(const char* key)
    {
        if (!opened_) return false;
        // Try to get as blob first (covers most types)
        size_t len = 0;
        esp_err_t err = nvs_get_blob(handle_, key, NULL, &len);
        if (err == ESP_OK || err == ESP_ERR_NVS_INVALID_LENGTH) return true;
        // Try u8
        uint8_t u8;
        if (nvs_get_u8(handle_, key, &u8) == ESP_OK) return true;
        // Try u16
        uint16_t u16;
        if (nvs_get_u16(handle_, key, &u16) == ESP_OK) return true;
        // Try u32
        uint32_t u32;
        if (nvs_get_u32(handle_, key, &u32) == ESP_OK) return true;
        return false;
    }

    /* ── Setters ──────────────────────────────────────── */

    void putBool(const char* key, bool val)
    {
        putUChar(key, val ? 1 : 0);
    }

    void putChar(const char* key, int8_t val)
    {
        putUChar(key, (uint8_t)val);
    }

    void putUChar(const char* key, uint8_t val)
    {
        if (!opened_) return;
        checkWrite_(key, nvs_set_u8(handle_, key, val));
    }

    void putShort(const char* key, int16_t val)
    {
        putUShort(key, (uint16_t)val);
    }

    void putUShort(const char* key, uint16_t val)
    {
        if (!opened_) return;
        checkWrite_(key, nvs_set_u16(handle_, key, val));
    }

    void putInt(const char* key, int32_t val)
    {
        putUInt(key, (uint32_t)val);
    }

    void putUInt(const char* key, uint32_t val)
    {
        if (!opened_) return;
        checkWrite_(key, nvs_set_u32(handle_, key, val));
    }

    void putFloat(const char* key, float val)
    {
        if (!opened_) return;
        uint32_t raw;
        memcpy(&raw, &val, sizeof(raw));
        checkWrite_(key, nvs_set_u32(handle_, key, raw));
    }

    size_t putBytes(const char* key, const void* buf, size_t len)
    {
        if (!opened_) return 0;
        return checkWrite_(key, nvs_set_blob(handle_, key, buf, len)) ? len : 0;
    }

    bool remove(const char* key)
    {
        if (!opened_) return false;
        return nvs_erase_key(handle_, key) == ESP_OK;
    }

    bool clear()
    {
        if (!opened_) return false;
        return nvs_erase_all(handle_) == ESP_OK;
    }

private:
    static constexpr const char* TAG_ = "NVS";

    // true when the read did not produce a value, so the caller substitutes
    // its default.  A missing key is the normal "never written" case and stays
    // quiet; a corrupt/unopened/wrong-type store is not, and says so.
    static bool readFailed_(const char* key, esp_err_t err)
    {
        if (err == ESP_OK) return false;
        if (err != ESP_ERR_NVS_NOT_FOUND)
            ESP_LOGW(TAG_, "read '%s' failed: %s — using default", key, esp_err_to_name(err));
        return true;
    }

    static bool checkWrite_(const char* key, esp_err_t err)
    {
        if (err == ESP_OK) return true;
        ESP_LOGE(TAG_, "write '%s' failed: %s", key, esp_err_to_name(err));
        return false;
    }

    nvs_handle_t handle_;
    bool opened_;
    bool read_only_;
};
