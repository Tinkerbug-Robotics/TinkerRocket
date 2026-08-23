#pragma once

// Base-station BLE file-list selection (#835 item 5).
//
// Pure helper extracted from main.cpp so the host-side gtest can drive it
// without a filesystem.  Anything in here must remain free of ESP-IDF /
// FreeRTOS / hardware dependencies.
//
// Why a streaming window instead of a collection array: handleFileListCommand
// used to read the directory into `FileEntry entries[64]`, stop at 64, and
// only THEN sort descending.  readdir walks the directory in on-disk slot
// order, which on a card where nothing has been deleted is creation order, so
// the entries past index 63 are the NEWEST logs — exactly the ones the sort
// was supposed to surface first.  Once the directory held 64+ files the flight
// the operator just recorded could not be listed or downloaded at all.
//
// TopNames fixes that by letting the caller stream the whole directory past a
// window that only ever retains the N greatest names.  Page 0 is therefore
// correct no matter how many files exist; the capacity bound limits how DEEP
// the operator can page, not whether the newest logs are reachable.

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace bs_file_list {

// Matches the name field the JSON list emits (and the old FileEntry::name).
static constexpr size_t NAME_CAP = 32;

struct Entry { char name[NAME_CAP]; };

// Fixed-capacity "keep the N greatest names" window, ordered by strcmp
// descending — the same order the old qsort produced, so a directory that
// fits in the window lists exactly as it always did.
class TopNames
{
public:
    // `storage` must hold at least `capacity` entries and outlive the window.
    TopNames(Entry* storage, size_t capacity)
        : buf_(storage), cap_(storage ? capacity : 0) {}

    void offer(const char* name)
    {
        ++total_;
        if (cap_ == 0 || name == nullptr) return;

        // Truncate up front so every comparison below is against the value we
        // would actually store — a name longer than NAME_CAP-1 must not sort
        // by characters that never make it into the window.
        Entry cand{};
        std::strncpy(cand.name, name, NAME_CAP - 1);
        cand.name[NAME_CAP - 1] = '\0';

        // Full and no better than the weakest name we hold: nothing to do.
        if (count_ == cap_ && std::strcmp(cand.name, buf_[cap_ - 1].name) <= 0)
            return;

        // Slot we may overwrite: one past the end while filling, else the
        // weakest entry (which this candidate displaces).
        const size_t last = (count_ < cap_) ? count_ : cap_ - 1;

        size_t pos = last;
        while (pos > 0 && std::strcmp(buf_[pos - 1].name, cand.name) < 0)
            --pos;

        if (last > pos)
            std::memmove(&buf_[pos + 1], &buf_[pos], (last - pos) * sizeof(Entry));
        buf_[pos] = cand;
        if (count_ < cap_) ++count_;
    }

    // Names retained, i.e. min(capacity, total()).
    size_t count() const { return count_; }

    // Names offered, uncapped — the true directory count, for logging and for
    // telling "the window filled up" apart from "that is the whole directory".
    size_t total() const { return total_; }

    size_t capacity() const { return cap_; }

    // Rank `i` in descending order; 0 is the greatest name. nullptr if out of
    // range.
    const char* at(size_t i) const { return (i < count_) ? buf_[i].name : nullptr; }

private:
    Entry* buf_ = nullptr;
    size_t cap_ = 0;
    size_t count_ = 0;
    size_t total_ = 0;
};

// Entries a window must hold to answer `page` (0-based) at `per_page` rows.
// Returns 0 when the request is past `max_window`, which the caller answers
// with an empty page — the same "list ended" signal the apps already act on
// when they hit a short page.
inline size_t windowFor(uint32_t page, size_t per_page, size_t max_window)
{
    const uint64_t want = (uint64_t)(page + 1u) * (uint64_t)per_page;
    return (want > (uint64_t)max_window) ? 0u : (size_t)want;
}

}  // namespace bs_file_list
