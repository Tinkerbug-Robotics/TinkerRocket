#pragma once

#include "TR_BitmapStore.h"

#include <cstdint>
#include <cstring>
#include <vector>

namespace tr_flightlog_test {

// In-memory TR_BitmapStore for host tests. Simulates persistence across
// begin() calls within a test by holding the last-saved blob.
class MemoryBitmapStore : public tr_flightlog::TR_BitmapStore {
public:
    bool load(uint8_t* buf, size_t len) override {
        if (!has_saved_ || saved_.size() != len) return false;
        std::memcpy(buf, saved_.data(), len);
        ++load_count_;
        return true;
    }

    bool save(const uint8_t* buf, size_t len) override {
        saved_.assign(buf, buf + len);
        has_saved_ = true;
        ++save_count_;
        return true;
    }

    // ---- Test helpers ----
    void reset()                       { saved_.clear(); has_saved_ = false;
                                         load_count_ = save_count_ = 0; }
    size_t saveCount() const           { return save_count_; }
    size_t loadCount() const           { return load_count_; }
    const std::vector<uint8_t>& raw() const { return saved_; }

    // Simulate a wiped NVS without clearing counters.
    void forgetSaved()                 { saved_.clear(); has_saved_ = false; }

private:
    std::vector<uint8_t> saved_;
    bool                 has_saved_   = false;
    size_t               save_count_  = 0;
    size_t               load_count_  = 0;
};

}  // namespace tr_flightlog_test
