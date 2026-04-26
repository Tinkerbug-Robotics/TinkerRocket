#pragma once

#include "BlockStateBitmap.h"
#include "FlightIndex.h"
#include "TR_BitmapStore.h"
#include "TR_FlightLog_types.h"
#include "TR_NandBackend.h"

namespace tr_flightlog {

// Custom append-only NAND flight-log layer. Replaces LittleFS on the write
// hot path; preserves BLE cmd 2 / 3 / 4 wire formats on the read side.
//
// Owns blocks [flight_region_start, flight_region_end) of the NAND as raw
// storage, plus four metadata blocks for the flight index + bitmap shadow.
//
// Stage 1 scaffold: only types and the interface are defined. Method bodies
// land in subsequent steps (see docs/plans/50-custom-nand-flight-log.md).
class TR_FlightLog {
public:
    struct Config {
        uint16_t prealloc_blocks     = 256;   // 32 MB
        uint16_t extend_blocks       = 64;    // overflow step (~200 ms stall)
        uint16_t flight_region_start = 32;    // first block owned by this layer
        uint16_t flight_region_end   = 1020;  // one past last flight block
        uint16_t metadata_blocks[4]  = {1020, 1021, 1022, 1023};
    };

    TR_FlightLog() = default;

    // `bitmap_store` is optional — pass nullptr for volatile (test / first-boot)
    // operation. On hardware, supply an NVS-backed store so the bitmap survives
    // reboots.
    Status begin(TR_NandBackend& nand, const Config& cfg,
                 TR_BitmapStore* bitmap_store = nullptr);

    // Mark a block as bad (sticky; persists immediately if a store is attached).
    Status markBlockBad(uint32_t block);

    // Inspect the bitmap — exposed for tests and for diagnostic reporting.
    const BlockStateBitmap& bitmap() const { return bitmap_; }
    const FlightIndex&      index()  const { return index_; }

    // Active-flight accessors (stable after prepareFlight, before finalizeFlight).
    bool      isFlightActive()     const { return flight_active_; }
    uint32_t  activeFlightId()     const { return active_flight_id_; }
    uint32_t  activeStartBlock()   const { return active_start_block_; }
    uint32_t  activeBlockCount()   const { return active_n_blocks_; }
    uint32_t  activePagesWritten() const { return active_next_page_; }

    // Number of times the pre-allocated range has been extended mid-flight.
    // Useful for post-flight log analysis to spot overflow events.
    uint32_t  overflowExtensionCount() const { return extension_count_; }

    // Pre-launch: pick + erase a free contiguous range. May stall ~770 ms.
    // Writes the assigned flight_id to `flight_id_out` on success.
    //
    // This is the synchronous form. On hardware, prefer
    // requestPrepareFlight() + servicePendingPrepareFlight() so the 256-block
    // erase loop runs on the flush task's core (Core 0) instead of blocking
    // sensor ingest on the calling core (issue #77).
    Status prepareFlight(uint32_t& flight_id_out);

    // Defer a prepareFlight call. Sets a flag and returns immediately; the
    // ~770 ms erase loop is performed by a later servicePendingPrepareFlight()
    // call (typically from the flush task on Core 0). Idempotent — calling
    // multiple times before service queues at most one prepare. No-op if a
    // flight is already active.
    void requestPrepareFlight();

    // True iff requestPrepareFlight() has been called and the request has not
    // yet been picked up by servicePendingPrepareFlight().
    bool isPrepareFlightPending() const { return prepare_request_pending_; }

    // If a request is pending and no flight is currently active, runs
    // prepareFlight() inline and reports its outcome via id_out / status_out.
    // Returns true when work was done; false when there was nothing to do
    // (no request pending, or a flight is already active). The pending flag
    // is cleared in either case so a stale request can't fire after the
    // flight has been started by some other path.
    //
    // Intended call site: once per flush-task iteration on the same core that
    // owns expensive NAND I/O.
    bool servicePendingPrepareFlight(uint32_t& id_out, Status& status_out);

    // Hot path: deterministic page write. Called by flush task.
    // `page` must be exactly NAND_PAGE_SIZE bytes. No allocation, no metadata.
    // The caller owns the page layout — recovery requires a PageHeader at the
    // start; consider `writeFrame` instead for that case.
    Status writePage(const uint8_t* page);

    // Hot-path helper: wrap `payload` (up to NAND_PAGE_SIZE - sizeof(PageHeader)
    // = 2032 bytes) in a PageHeader (active flight_id + monotonic seq + CRC32)
    // and program it. This is the recommended write API — pages written via
    // writeFrame are recoverable by the brownout scanner if finalizeFlight is
    // never called.
    Status writeFrame(const uint8_t* payload, size_t payload_len);

    // Post-flight: append index entry. May stall.
    Status finalizeFlight(const char* filename, uint32_t final_bytes);

    // BLE-facing read surface.
    size_t listFlights(FlightIndexEntry* out, size_t max, size_t page, size_t per_page);
    Status readFlightPage(const char* filename, uint32_t offset, uint8_t* buf,
                          size_t buf_len, size_t& out_len);
    Status deleteFlight(const char* filename);    // releases blocks for reuse
    Status renameFlight(const char* old_name, const char* new_name);

    // --- Introspection / test surface ---
    bool          isInitialized() const { return initialized_; }
    const Config& config() const        { return cfg_; }

private:
    TR_NandBackend*    nand_          = nullptr;
    TR_BitmapStore*    bitmap_store_  = nullptr;
    BlockStateBitmap   bitmap_        = {};
    FlightIndex        index_         = {};
    Config             cfg_           = {};
    bool               initialized_   = false;

    uint32_t active_flight_id_    = 0;
    uint32_t active_start_block_  = 0;
    uint32_t active_n_blocks_     = 0;
    uint32_t active_next_page_    = 0;  // absolute page index within the flight range
    uint32_t extension_count_     = 0;
    bool     flight_active_       = false;

    // Single-producer (any task) / single-consumer (flush task) request flag
    // for the deferred prepareFlight path. `volatile` is sufficient because
    // the consumer is idempotent: a missed clear at worst causes one extra
    // service call that no-ops via the flight_active_ check.
    volatile bool prepare_request_pending_ = false;

    void persistBitmap();
    void seedBitmapFromBackend();  // initial bad-block scan into the fresh bitmap

    // Extend the active flight by cfg_.extend_blocks contiguous blocks after
    // the current range. Returns true on success; false if the adjacent slot
    // isn't free or erase fails.
    bool extendActiveRange();

    // After loading bitmap + index, scan for allocated ranges not covered by
    // any index entry — those are flights interrupted by a brownout before
    // finalizeFlight ran. For each, walk the PageHeader magic/CRC32 to find
    // the last valid page, synthesize a partial FlightIndexEntry, and persist.
    // Orphaned ranges with no valid pages are released back to FREE.
    Status scanForBrownoutRecovery();
};

}  // namespace tr_flightlog
