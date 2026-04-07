/**
 * TR_NVS.h — Drop-in replacement for Arduino Preferences class
 *
 * Wraps ESP-IDF nvs_flash API with an interface matching
 * the Arduino Preferences class used throughout the codebase.
 */
#pragma once

#include <nvs_flash.h>
#include <nvs.h>
#include <string.h>
#include <esp_log.h>

class Preferences {
public:
    Preferences() : handle_(0), opened_(false) {}

    ~Preferences() { end(); }

    bool begin(const char* ns, bool readOnly = false)
    {
        if (opened_) end();
        esp_err_t err = nvs_open(ns, readOnly ? NVS_READONLY : NVS_READWRITE, &handle_);
        opened_ = (err == ESP_OK);
        read_only_ = readOnly;
        return opened_;
    }

    void end()
    {
        if (opened_) {
            if (!read_only_) nvs_commit(handle_);
            nvs_close(handle_);
            opened_ = false;
        }
    }

    /* ── Getters ──────────────────────────────────────── */

    bool getBool(const char* key, bool def = false)
    {
        uint8_t val = def ? 1 : 0;
        nvs_get_u8(handle_, key, &val);
        return val != 0;
    }

    int8_t getChar(const char* key, int8_t def = 0)
    {
        return (int8_t)getUChar(key, (uint8_t)def);
    }

    uint8_t getUChar(const char* key, uint8_t def = 0)
    {
        uint8_t val = def;
        nvs_get_u8(handle_, key, &val);
        return val;
    }

    int16_t getShort(const char* key, int16_t def = 0)
    {
        return (int16_t)getUShort(key, (uint16_t)def);
    }

    uint16_t getUShort(const char* key, uint16_t def = 0)
    {
        uint16_t val = def;
        nvs_get_u16(handle_, key, &val);
        return val;
    }

    int32_t getInt(const char* key, int32_t def = 0)
    {
        return (int32_t)getUInt(key, (uint32_t)def);
    }

    uint32_t getUInt(const char* key, uint32_t def = 0)
    {
        uint32_t val = def;
        nvs_get_u32(handle_, key, &val);
        return val;
    }

    float getFloat(const char* key, float def = 0.0f)
    {
        uint32_t raw;
        memcpy(&raw, &def, sizeof(raw));
        nvs_get_u32(handle_, key, &raw);
        float result;
        memcpy(&result, &raw, sizeof(result));
        return result;
    }

    size_t getBytesLength(const char* key)
    {
        size_t len = 0;
        esp_err_t err = nvs_get_blob(handle_, key, NULL, &len);
        return (err == ESP_OK) ? len : 0;
    }

    size_t getBytes(const char* key, void* buf, size_t maxLen)
    {
        size_t len = maxLen;
        esp_err_t err = nvs_get_blob(handle_, key, buf, &len);
        return (err == ESP_OK) ? len : 0;
    }

    bool isKey(const char* key)
    {
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
        nvs_set_u8(handle_, key, val ? 1 : 0);
    }

    void putChar(const char* key, int8_t val)
    {
        putUChar(key, (uint8_t)val);
    }

    void putUChar(const char* key, uint8_t val)
    {
        nvs_set_u8(handle_, key, val);
    }

    void putShort(const char* key, int16_t val)
    {
        putUShort(key, (uint16_t)val);
    }

    void putUShort(const char* key, uint16_t val)
    {
        nvs_set_u16(handle_, key, val);
    }

    void putInt(const char* key, int32_t val)
    {
        putUInt(key, (uint32_t)val);
    }

    void putUInt(const char* key, uint32_t val)
    {
        nvs_set_u32(handle_, key, val);
    }

    void putFloat(const char* key, float val)
    {
        uint32_t raw;
        memcpy(&raw, &val, sizeof(raw));
        nvs_set_u32(handle_, key, raw);
    }

    size_t putBytes(const char* key, const void* buf, size_t len)
    {
        esp_err_t err = nvs_set_blob(handle_, key, buf, len);
        return (err == ESP_OK) ? len : 0;
    }

    bool remove(const char* key)
    {
        return nvs_erase_key(handle_, key) == ESP_OK;
    }

    bool clear()
    {
        return nvs_erase_all(handle_) == ESP_OK;
    }

private:
    nvs_handle_t handle_;
    bool opened_;
    bool read_only_;
};
