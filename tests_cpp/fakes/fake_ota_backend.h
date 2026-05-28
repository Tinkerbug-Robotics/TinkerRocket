// Fake TR_OTA_Backend for host tests. Tracks bytes written + lets tests
// inject failures into individual backend methods.
#ifndef FAKE_OTA_BACKEND_H
#define FAKE_OTA_BACKEND_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "TR_OTA_Backend.h"

class FakeOTABackend : public TR_OTA_Backend
{
public:
    // ---- failure injection (set non-zero to make the corresponding call fail) ----
    int begin_rc           = 0;
    int write_rc           = 0;
    int end_rc             = 0;
    int set_boot_rc        = 0;
    bool fail_write_after_n = false;   // if true, write returns -100 after `write_fail_threshold` bytes
    size_t write_fail_threshold = 0;

    // ---- observable state ----
    bool   active = false;
    size_t total_size = 0;
    size_t bytes_written = 0;
    bool   boot_set = false;
    bool   aborted_once = false;
    bool   ended_once = false;
    int    begin_calls = 0;
    int    abort_calls = 0;
    std::vector<uint8_t> bytes;   // exactly the data passed through write()

    int begin(size_t image_size) override
    {
        ++begin_calls;
        if (begin_rc != 0) return begin_rc;
        active = true;
        total_size = image_size;
        bytes_written = 0;
        bytes.clear();
        bytes.reserve(image_size);
        return 0;
    }

    int write(const uint8_t* data, size_t len) override
    {
        if (write_rc != 0) return write_rc;
        if (fail_write_after_n && bytes_written + len > write_fail_threshold) return -100;
        if (!active) return -1;
        bytes.insert(bytes.end(), data, data + len);
        bytes_written += len;
        return 0;
    }

    int end() override
    {
        ended_once = true;
        if (end_rc != 0) { active = false; return end_rc; }
        active = false;
        return 0;
    }

    int setBootPartition() override
    {
        if (set_boot_rc != 0) return set_boot_rc;
        boot_set = true;
        return 0;
    }

    void abort() override
    {
        ++abort_calls;
        aborted_once = true;
        active = false;
    }
};

#endif  // FAKE_OTA_BACKEND_H
