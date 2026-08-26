#include <TR_LogToFlash.h>
#include <MramProbe.h>
#include <CRC.h>
#include <TR_NVS.h>
#include <cstring>
#include <cstdio>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_timer.h>

// Helpers for the stall-instrumentation below.  esp_timer_get_time() is
// a ~sub-microsecond monotonic clock, cheaper than millis() for short ops.
#define LFS_TIMING_START()   const int64_t _t0 = esp_timer_get_time()
#define LFS_TIMING_END(peak_field, op_name)                                     \
    do {                                                                        \
        uint32_t _dt = (uint32_t)(esp_timer_get_time() - _t0);                  \
        if (_dt > (peak_field)) (peak_field) = _dt;                             \
        if (_dt > (uint32_t)STALL_THRESHOLD_US) {                               \
            ESP_LOGW(TAG, "STALL: %s took %lu us", (op_name), (unsigned long)_dt); \
        }                                                                       \
    } while (0)

static const char* TAG = "LOG";

// Timestamp storage is now persisted in NAND flash as part of the catalog
// (removed static in-memory timestamp array)

TR_LogToFlash::TR_LogToFlash()
    : spi(nullptr),
      spi_nand(40'000'000, MSBFIRST, SPI_MODE0)
{
    memset(page_buf, 0xFF, sizeof(page_buf));
}

bool TR_LogToFlash::begin(SPIClass& spi_in, const TR_LogToFlashConfig& cfg_in)
{
    spi = &spi_in;
    cfg = cfg_in;
    spi_nand = SPISettings(cfg.spi_hz_nand, MSBFIRST, cfg.spi_mode_nand);

    pinMode(cfg.nand_cs, OUTPUT);
    csHigh(cfg.nand_cs);

    // Initialize ring buffer — MRAM (SPI) if one actually answers, else heap RAM
    use_mram_ = false;
    mram_probe_failed_ = false;
    if (cfg.mram_cs >= 0)
    {
        spi_mram = SPISettings(cfg.spi_hz_mram, MSBFIRST, cfg.spi_mode_mram);

        pinMode(cfg.mram_cs, OUTPUT);
        digitalWrite(cfg.mram_cs, HIGH);

        // #826: this used to be `use_mram_ = true` on the strength of the pin
        // number alone. A configured-but-absent MRAM then clocked every frame
        // onto an unconnected pad and read noise back, and that noise was
        // programmed into the NAND inside a valid PageHeader with a good
        // CRC32 — so nothing downstream could catch it, and ocStorageHealth()
        // stayed green while every flight log was lost. The board split
        // (#822) closed the reachable path; this closes the class, including
        // a fitted part that is dead or badly soldered.
        if (mramProbe())
        {
            use_mram_ = true;
            ring_size_ = cfg.mram_size;
            if (cfg.debug) ESP_LOGI(TAG, "MRAM ring: %lu bytes on CS pin %d",
                                          (unsigned long)ring_size_, cfg.mram_cs);
        }
        else
        {
            // Loud and unconditional (not gated on cfg.debug): silence here is
            // exactly the #826 failure. Logging continues on the RAM ring, so
            // this degrades capacity and costs brownout recovery — it does not
            // stop the vehicle logging.
            mram_probe_failed_ = true;
            ESP_LOGE(TAG, "MRAM configured on CS pin %d but NO DEVICE ANSWERED "
                          "— falling back to a RAM ring. Check the board "
                          "revision flag and the MRAM part/solder. In-flight "
                          "reboot recovery is unavailable this session.",
                     cfg.mram_cs);
        }
    }

    if (!use_mram_)
    {
        ring_in_psram_ = false;

        // #822: on a board with no MRAM but with in-package PSRAM (V9/V10's
        // ESP32-S3RH2), the ring belongs in PSRAM — that part is the MRAM's
        // designated replacement and exists to buffer NAND writes. Try it
        // first, at its own (much larger) size.
        //
        // Safe to attempt unconditionally: with CONFIG_SPIRAM off, or the part
        // absent under SPIRAM_IGNORE_NOTFOUND, this allocation simply returns
        // null and we take the internal path below. The ring is only ever
        // memcpy'd (never a DMA target), so PSRAM's DMA restrictions don't
        // apply to it.
        if (cfg.psram_ring_size > 0)
        {
            ring_size_ = cfg.psram_ring_size;
            ring_buf_ = (uint8_t*)heap_caps_malloc(ring_size_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (ring_buf_)
            {
                ring_in_psram_ = true;
                if (cfg.debug) ESP_LOGI(TAG, "Allocated %lu byte PSRAM ring (free PSRAM: %lu)",
                                        (unsigned long)ring_size_,
                                        (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            }
            else
            {
                // Loud, not silent: this is the difference between seconds of
                // NAND-stall headroom and a fraction of one, and the whole
                // point of the board's PSRAM. Falling back keeps the vehicle
                // logging, but somebody needs to know why the ring shrank.
                ESP_LOGW(TAG, "PSRAM ring (%lu B) requested but unavailable — "
                              "falling back to %lu B of internal RAM. Check "
                              "CONFIG_SPIRAM and the boot-time PSRAM init log.",
                         (unsigned long)cfg.psram_ring_size,
                         (unsigned long)cfg.ring_buffer_size);
            }
        }

        if (!ring_buf_)
        {
            ring_size_ = cfg.ring_buffer_size;
            ring_buf_ = (uint8_t*)heap_caps_malloc(ring_size_, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!ring_buf_)
            {
                if (cfg.debug) ESP_LOGE(TAG, "Failed to allocate %lu byte ring buffer",
                                              (unsigned long)ring_size_);
                return false;
            }
            if (cfg.debug) ESP_LOGI(TAG, "Allocated %lu byte RAM ring (free heap: %lu)",
                                          (unsigned long)ring_size_, (unsigned long)esp_get_free_heap_size());
        }
    }
    ring_prelaunch_cap_ = prelaunchCap();

    // SPI bus mutex — created unconditionally.  It used to live inside the
    // MRAM branch above, which left every NAND transaction unserialized
    // whenever MRAM was absent: the flush task (Core 0) programs pages while
    // the BLE download path reads them, and spiAcquire/spiRelease silently
    // no-op on a null handle.  Nothing caught it because the RAM-ring branch
    // has never been flown — but it becomes live the moment a board ships
    // without the MRAM part.  Keeping it unconditional also keeps the #398
    // spi_wait/spi_hold instrumentation meaningful on both paths.
    spi_mutex_ = xSemaphoreCreateMutex();
    if (!spi_mutex_)
    {
        if (cfg.debug) ESP_LOGE(TAG, "Failed to create SPI mutex");
        return false;
    }

    // Mutex to serialize ringPush callers and to block clearRing from
    // running concurrently with an in-flight push (#74). Unconditionally
    // created — parser + oc_loop can race on Core 1 regardless of ring
    // backing, so RAM and MRAM paths both need it.
    push_mutex_ = xSemaphoreCreateMutex();
    if (!push_mutex_)
    {
        if (cfg.debug) ESP_LOGE(TAG, "Failed to create push mutex");
        return false;
    }

    rb_head = rb_tail = rb_count = 0;
    rb_overruns = rb_highwater = 0;
    rb_drop_oldest_bytes = 0;
    rb_bad_sof_clears = 0;
    nand_page = nand_block = 0;
    nand_bytes_written = 0;
    nand_prog_fail = nand_erase_fail = 0;
    nand_prog_ops = nand_erase_ops = 0;
    logging_active = false;
    start_logging_requested = false;
    end_flight_requested = false;
    page_buf_idx = 0;
    bytes_received = 0;
    frames_received = 0;
    frames_dropped = 0;
    drop_warned_ = false;
    recovery_performed = false;
    recovery_bytes = 0;
    file_open = false;
    current_file_bytes = 0;
    memset(recovery_filename, 0, sizeof(recovery_filename));
    memset(current_filename, 0, sizeof(current_filename));

    // Load persistent bad-block bitmap (#47) before any NAND I/O so the LFS
    // callbacks can short-circuit known-bad blocks from the very first mount.
    loadBadBlocksFromNVS();

    if (!nandInit())
    {
        return false;
    }
    // #671: geometry is resolved now — fix the byte-denominated sync cadence
    // (64 pages on the legacy 4 KB part, identical to the old constant).
    sync_interval_pages_ = SYNC_INTERVAL_BYTES / geom_.page_size;

    // Boot-time non-destructive bad-block scan (#47): walks every not-yet-
    // known-bad block and probes for read errors + factory bad markers.
    // #511: the full block walk (2048 blocks ~2.7 s on the legacy part;
    // half that on the mini's 1024) costs cmd-8 Power-On
    // stall, so it now runs only until one scan has completed for this chip
    // (NVS "scanned" marker, written strictly after the scan). Runtime
    // discovery (markBlockBad on read/prog/erase failures) keeps the map
    // current afterwards. A dead RDID bus skips the scan entirely — probing
    // a chip's worth of failing reads would mark every block bad and poison
    // the map.
    switch (BadBlockScanPolicy::bootScanVerdict(nand_dead_bus_,
                                                bad_block_map_blob_ok_,
                                                bad_block_scanned_chip_,
                                                current_chip_id_))
    {
        case BadBlockScanPolicy::Verdict::Scan:
            scanBadBlocksAtBoot();
            // Persist right away so a reboot during the rest of begin()
            // doesn't lose the discoveries. The marker goes last: a power
            // cut between the two just rescans next boot.
            persistBadBlocksIfDirty();
            markBootScanComplete();
            break;
        case BadBlockScanPolicy::Verdict::SkipTrustedMap:
            if (cfg.debug) ESP_LOGI(TAG, "Bad-block boot scan skipped: map trusted (chip 0x%04X, %lu known bad)",
                                          (unsigned)current_chip_id_,
                                          (unsigned long)countBadBlocks());
            break;
        case BadBlockScanPolicy::Verdict::SkipDeadBus:
            // Not debug-gated — a dead NAND bus is serious, and the mount
            // below will fail with less obvious symptoms.
            ESP_LOGE(TAG, "NAND RDID dead bus (0x%04X) — skipping bad-block scan",
                          (unsigned)current_chip_id_);
            break;
    }

    // Allocate LittleFS buffers on heap to avoid stack overflow
    lfs_read_buffer = (uint8_t*)malloc(geom_.page_size);
    lfs_prog_buffer = (uint8_t*)malloc(geom_.page_size);
    lfs_lookahead_buffer = (uint8_t*)malloc(128);

    if (!lfs_read_buffer || !lfs_prog_buffer || !lfs_lookahead_buffer)
    {
        if (cfg.debug) ESP_LOGE(TAG, "Failed to allocate LittleFS buffers");
        free(lfs_read_buffer);
        free(lfs_prog_buffer);
        free(lfs_lookahead_buffer);
        return false;
    }

    // Configure LittleFS
    lfs_cfg = (lfs_config*)malloc(sizeof(lfs_config));
    if (!lfs_cfg)
    {
        if (cfg.debug) ESP_LOGE(TAG, "Failed to allocate LittleFS config");
        free(lfs_read_buffer);
        free(lfs_prog_buffer);
        free(lfs_lookahead_buffer);
        return false;
    }

    memset(lfs_cfg, 0, sizeof(lfs_config));

    lfs_cfg->context = this;  // For callbacks to access instance
    lfs_cfg->read = lfsBlockRead;
    lfs_cfg->prog = lfsBlockProg;
    lfs_cfg->erase = lfsBlockErase;
    lfs_cfg->sync = lfsBlockSync;

    lfs_cfg->read_size = geom_.page_size;
    lfs_cfg->prog_size = geom_.page_size;
    lfs_cfg->block_size = geom_.blockSize();
    // Default to the full chip; caller may shrink (issue #50 Stage 2c) so the
    // trailing blocks can be owned by TR_FlightLog.
    lfs_cfg->block_count = (cfg.lfs_block_count > 0) ? cfg.lfs_block_count
                                                     : geom_.block_count;
    lfs_cfg->cache_size = geom_.page_size;
    lfs_cfg->lookahead_size = 128;            // 128 bytes = 1024 bits
    lfs_cfg->block_cycles = 500;              // Wear leveling

    lfs_cfg->read_buffer = lfs_read_buffer;
    lfs_cfg->prog_buffer = lfs_prog_buffer;
    lfs_cfg->lookahead_buffer = lfs_lookahead_buffer;

    // Try to mount existing filesystem
    if (cfg.debug) ESP_LOGI(TAG, "Mounting LittleFS...");
    int err = lfs_mount(&lfs, lfs_cfg);

    if (err)
    {
        // No filesystem found, format fresh
        if (cfg.debug) ESP_LOGW(TAG, "Mount failed (%d), formatting NAND...", err);
        err = lfs_format(&lfs, lfs_cfg);
        if (err)
        {
            if (cfg.debug) ESP_LOGE(TAG, "Format failed: %d", err);
            free(lfs_read_buffer);
            free(lfs_prog_buffer);
            free(lfs_lookahead_buffer);
            free(lfs_cfg);
            return false;
        }
        err = lfs_mount(&lfs, lfs_cfg);
        if (err)
        {
            if (cfg.debug) ESP_LOGE(TAG, "Mount after format failed: %d", err);
            free(lfs_read_buffer);
            free(lfs_prog_buffer);
            free(lfs_lookahead_buffer);
            free(lfs_cfg);
            return false;
        }
    }

    // Verify filesystem is healthy by creating and removing a test file.
    // If this fails with LFS_ERR_CORRUPT, reformat the entire filesystem.
    {
        int test_err = lfs_file_open(&lfs, &file, "/.health_check",
                                      LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
        if (test_err == LFS_ERR_CORRUPT)
        {
            ESP_LOGW(TAG, "Filesystem corrupt — reformatting...");
            lfs_unmount(&lfs);
            err = lfs_format(&lfs, lfs_cfg);
            if (err)
            {
                ESP_LOGE(TAG, "Reformat failed: %d", err);
                free(lfs_read_buffer);
                free(lfs_prog_buffer);
                free(lfs_lookahead_buffer);
                free(lfs_cfg);
                return false;
            }
            err = lfs_mount(&lfs, lfs_cfg);
            if (err)
            {
                ESP_LOGE(TAG, "Mount after reformat failed: %d", err);
                free(lfs_read_buffer);
                free(lfs_prog_buffer);
                free(lfs_lookahead_buffer);
                free(lfs_cfg);
                return false;
            }
            ESP_LOGI(TAG, "Filesystem reformatted successfully");
        }
        else if (test_err == 0)
        {
            lfs_file_close(&lfs, &file);
            lfs_remove(&lfs, "/.health_check");
        }
    }

    if (cfg.debug) ESP_LOGI(TAG, "LittleFS mounted successfully");

    runStartupRecovery();
    return true;
}

bool TR_LogToFlash::enqueueFrame(const uint8_t* frame, size_t len)
{
    if (frame == nullptr || len == 0 || len > MAX_FRAME)
    {
        return false;
    }
    if (end_flight_requested)
    {
        return false;
    }
    // Accept frames when logging is active OR when the log file has been
    // pre-created (PRELAUNCH).  Pre-launch frames buffer in the ring
    // (capped at 75% by prelaunchCap(), which keeps ~1 s of pre-ignition
    // history while leaving the launch transient headroom — this said 50%,
    // the cap's value before that tuning, #837 item 9) and are flushed once
    // activateLogging() fires.
    // Cross-session stale MRAM is handled by runStartupRecovery() at boot;
    // within a session, processFrame() has a timestamp monotonicity filter
    // that catches anything that would look like a replay from the ring.
    if (!logging_active && !file_open)
    {
        return false;
    }
    frames_received++;
    bytes_received += len;

    // MRAM path: stage in RAM and write MRAM in STAGING_SIZE batches — a
    // per-frame MRAM push costs two SPI transactions under the bus mutex
    // (~325 us/frame with flush contention), capping ingest at ~2.1k
    // frames/s.  RAM-ring path keeps the direct per-frame push (memcpy is
    // already cheap).
    if (use_mram_ && len <= STAGING_SIZE)
    {
        if (push_mutex_) xSemaphoreTake(push_mutex_, portMAX_DELAY);
        bool ok = true;
        if (staged_len_ + len > STAGING_SIZE)
        {
            ok = flushStagingLocked();
        }
        if (staged_len_ == 0)
        {
            staged_first_us_ = esp_timer_get_time();
        }
        memcpy(staging_buf_ + staged_len_, frame, len);
        staged_len_ += static_cast<uint32_t>(len);
        staged_frames_++;
        if (push_mutex_) xSemaphoreGive(push_mutex_);
        if (!ok)
        {
            return false;  // this frame is staged; the DROPPED batch was counted
        }
        return true;
    }

    if (!ringPush(frame, static_cast<uint32_t>(len)))
    {
        frames_dropped++;
        // #278: the ring rejected the NEWEST frame (in-flight drop-newest). It's
        // counted in frames_dropped / telemetry frames_drop, but warn loudly the
        // first time so a truncated flight can't look healthy in the moment.
        if (!drop_warned_)
        {
            drop_warned_ = true;
            ESP_LOGW(TAG, "RING FULL in flight: dropped NEWEST frame (%lu B); log "
                          "truncates at the tail (received=%lu, highwater=%lu). "
                          "Further drops counted in frames_dropped.",
                     static_cast<unsigned long>(len),
                     static_cast<unsigned long>(frames_received),
                     static_cast<unsigned long>(rb_highwater));
        }
        return false;
    }
    return true;
}

// Write the staged frames to the MRAM ring as one batched push and reset
// staging.  Caller holds push_mutex_.  On ring rejection (in-flight, ring
// full behind a NAND stall) the whole batch is dropped-newest — the same
// bytes a per-frame push would have shed under the same stall — and counted.
bool TR_LogToFlash::flushStagingLocked()
{
    if (staged_len_ == 0)
    {
        return true;
    }
    const bool ok = ringPushLocked(staging_buf_, staged_len_);
    if (!ok)
    {
        frames_dropped += staged_frames_;
        if (!drop_warned_)
        {
            drop_warned_ = true;
            ESP_LOGW(TAG, "RING FULL in flight: dropped staged batch (%lu B, %lu "
                          "frames); log truncates at the tail (received=%lu, "
                          "highwater=%lu).",
                     static_cast<unsigned long>(staged_len_),
                     static_cast<unsigned long>(staged_frames_),
                     static_cast<unsigned long>(frames_received),
                     static_cast<unsigned long>(rb_highwater));
        }
    }
    else
    {
        staging_flushes_++;
    }
    staged_len_ = 0;
    staged_frames_ = 0;
    return ok;
}

// Push staged frames that have been waiting longer than max_age_us, so the
// staleness of ring contents (and the brownout-durability gap) stays bounded
// even when inflow is a trickle.  Called from the flush task loop (Core 0)
// and the pre-task service() fallback; safe cross-core via push_mutex_.
void TR_LogToFlash::flushStagingIfStale(int64_t max_age_us)
{
    if (!use_mram_ || staged_len_ == 0)
    {
        return;
    }
    if ((esp_timer_get_time() - staged_first_us_) < max_age_us)
    {
        return;
    }
    if (push_mutex_) xSemaphoreTake(push_mutex_, portMAX_DELAY);
    flushStagingLocked();
    if (push_mutex_) xSemaphoreGive(push_mutex_);
}

void TR_LogToFlash::startLogging()
{
    start_logging_requested = true;
}

void TR_LogToFlash::prepareLogFile()
{
    prepare_file_requested_ = true;
}

void TR_LogToFlash::endLogging()
{
    // Idempotent when nothing is logging. The flush-task drain block clears
    // end_flight_requested only when (end_flight_requested && logging_active
    // && file_open) all hold, so latching it here while logging_active is
    // already false leaves it stuck true forever — and isLoggingActive()
    // (returns logging_active || end_flight_requested) then never goes
    // false. That made the BLE cmd=23 toggle handler stuck on "stop" after
    // a normal LANDED-triggered drain.
    if (!logging_active && !file_open) return;
    end_flight_requested = true;
}

void TR_LogToFlash::service()
{
    // When the flush task is running (Core 0), service() is a no-op on Core 1.
    // The flush task loop handles openLogSession, flushRingToNand,
    // closeLogSession, and the staged-write staleness flush.
    if (flush_task_running_)
    {
        return;
    }

    flushStagingIfStale(STAGING_MAX_AGE_US);

    // Single-threaded fallback (flush task not started yet — startup/recovery).
    // #365: same consume-on-observe shape as flushTaskLoop — the launch edge
    // is raised from another task, so even this path must not blind-clear.
    if (start_logging_requested)
    {
        if (!logging_active)
        {
            if (!file_open)
            {
                openLogSession();
            }
            activateLogging();   // #417: markDirty now lives here
        }
        start_logging_requested = false;
    }
    flushRingToNand();
}

void TR_LogToFlash::getStats(TR_LogToFlashStats& out) const
{
    out.ring_size = ring_size_;
    out.ring_fill = rb_count;
    out.ring_highwater = rb_highwater;
    out.ring_overruns = rb_overruns;
    out.ring_drop_oldest_bytes = rb_drop_oldest_bytes;
    out.ring_bad_sof_clears = rb_bad_sof_clears;
    out.bytes_received = bytes_received;
    out.frames_received = frames_received;
    out.frames_dropped = frames_dropped;
    out.bytes_written_nand = nand_bytes_written;
    out.nand_prog_fail = nand_prog_fail;
    out.nand_erase_fail = nand_erase_fail;
    out.nand_prog_ops = nand_prog_ops;
    out.nand_erase_ops = nand_erase_ops;
    out.logging_active = logging_active;
    out.nand_page = nand_page;
    out.nand_block = log_curr_block;

    out.write_max_us = write_max_us_;
    out.sync_max_us = sync_max_us_;
    out.erase_max_us = erase_max_us_;
    out.open_max_us = open_max_us_;
    out.close_max_us = close_max_us_;
    out.activate_max_us = activate_max_us_;
    out.clear_ring_max_us = clear_ring_max_us_;
    out.flush_iter_max_us = flush_iter_max_us_;
    out.spi_wait_max_us = spi_wait_max_us_;   // #398
    out.spi_hold_max_us = spi_hold_max_us_;   // #398
    out.drain_pages = drain_pages_win_;       // #510
    out.drain_bytes = drain_bytes_win_;       // #510
    out.pop_sum_us = pop_us_win_;             // #510
    out.write_sum_us = write_us_win_;         // #510
    out.syncs_performed = syncs_performed_;

    out.known_bad_blocks = countBadBlocks();
    out.bad_block_skips = bad_block_skips_;
}

void TR_LogToFlash::resetIntervalTimings()
{
    write_max_us_ = 0;
    sync_max_us_ = 0;
    erase_max_us_ = 0;
    open_max_us_ = 0;
    close_max_us_ = 0;
    activate_max_us_ = 0;
    clear_ring_max_us_ = 0;
    flush_iter_max_us_ = 0;
    spi_wait_max_us_ = 0;   // #398
    spi_hold_max_us_ = 0;   // #398
    drain_pages_win_ = 0;   // #510
    drain_bytes_win_ = 0;   // #510
    pop_us_win_ = 0;        // #510
    write_us_win_ = 0;      // #510
}

void TR_LogToFlash::getRecoveryInfo(TR_LogToFlashRecoveryInfo& out) const
{
    out.recovered = recovery_performed;
    out.recovered_bytes = recovery_bytes;
    strncpy(out.filename, recovery_filename, sizeof(out.filename) - 1);
    out.filename[sizeof(out.filename) - 1] = '\0';
}

size_t TR_LogToFlash::listFiles(TR_LogFileInfo* out, size_t max_files) const
{
    lfs_dir_t dir;
    struct lfs_info info;
    size_t count = 0;

    // Open root directory
    if (lfs_dir_open((lfs_t*)&lfs, &dir, "/") < 0)
    {
        return 0;
    }

    // Iterate through directory entries
    while (lfs_dir_read((lfs_t*)&lfs, &dir, &info) > 0 && count < max_files)
    {
        // Skip "." and ".." entries
        if (strcmp(info.name, ".") == 0 || strcmp(info.name, "..") == 0)
        {
            continue;
        }

        // Skip directories, only include regular files
        if (info.type != LFS_TYPE_REG)
        {
            continue;
        }

        // Skip active file (OutComputer expects this)
        if (file_open && strcmp(info.name, current_filename + 1) == 0)  // +1 to skip leading '/'
        {
            continue;
        }

        // Populate file info
        strncpy(out[count].filename, info.name, sizeof(out[count].filename) - 1);
        out[count].filename[sizeof(out[count].filename) - 1] = '\0';
        out[count].size_bytes = info.size;

        // Read timestamp from custom attribute (if exists)
        struct __attribute__((packed))
        {
            uint16_t year;
            uint8_t month, day, hour, minute, second;
        } ts;

        char path[64];
        snprintf(path, sizeof(path), "/%s", info.name);
        int attr_len = lfs_getattr((lfs_t*)&lfs, path, 'T', &ts, sizeof(ts));

        if (attr_len == sizeof(ts))
        {
            out[count].year = ts.year;
            out[count].month = ts.month;
            out[count].day = ts.day;
            out[count].hour = ts.hour;
            out[count].minute = ts.minute;
            out[count].second = ts.second;
            out[count].has_timestamp = true;
        }
        else
        {
            out[count].has_timestamp = false;
        }

        count++;
    }

    lfs_dir_close((lfs_t*)&lfs, &dir);
    return count;
}

bool TR_LogToFlash::readFileChunk(const char* fname,
                                  uint32_t offset,
                                  uint8_t* out,
                                  size_t max_len,
                                  size_t& out_len,
                                  bool& eof)
{
    if (fname == nullptr || out == nullptr || max_len == 0)
    {
        out_len = 0;
        eof = true;
        return false;
    }

    // Build full path
    char path[64];
    snprintf(path, sizeof(path), "/%s", fname);

    // Open file for reading
    lfs_file_t f;
    int err = lfs_file_open((lfs_t*)&lfs, &f, path, LFS_O_RDONLY);
    if (err < 0)
    {
        out_len = 0;
        eof = true;
        return false;
    }

    // Get file size
    lfs_soff_t size = lfs_file_size((lfs_t*)&lfs, &f);
    if (size < 0 || offset >= (uint32_t)size)
    {
        lfs_file_close((lfs_t*)&lfs, &f);
        out_len = 0;
        eof = true;
        return true;  // Not an error, just past end of file
    }

    // Seek to offset
    lfs_file_seek((lfs_t*)&lfs, &f, offset, LFS_SEEK_SET);

    // Read chunk
    lfs_ssize_t read = lfs_file_read((lfs_t*)&lfs, &f, out, max_len);
    if (read < 0)
    {
        lfs_file_close((lfs_t*)&lfs, &f);
        out_len = 0;
        eof = true;
        return false;
    }

    out_len = (size_t)read;
    eof = (offset + read >= (uint32_t)size);

    lfs_file_close((lfs_t*)&lfs, &f);
    return true;
}

bool TR_LogToFlash::deleteFile(const char* fname)
{
    if (fname == nullptr)
    {
        return false;
    }

    // Can't delete active file
    if (file_open && strcmp(fname, current_filename + 1) == 0)  // +1 to skip leading '/'
    {
        return false;
    }

    // Build full path
    char path[64];
    snprintf(path, sizeof(path), "/%s", fname);

    // Remove file from filesystem
    int err = lfs_remove((lfs_t*)&lfs, path);
    return (err == 0);
}


bool TR_LogToFlash::formatFilesystem()
{
    // Cannot format while logging is active
    if (logging_active || file_open)
    {
        if (cfg.debug)
        {
            ESP_LOGE(TAG, "Cannot format: logging is active");
        }
        return false;
    }

    if (cfg.debug)
    {
        ESP_LOGW(TAG, "Formatting filesystem - all data will be lost!");
    }

    // Unmount filesystem
    int err = lfs_unmount((lfs_t*)&lfs);
    if (err && cfg.debug)
    {
        ESP_LOGW(TAG, "Unmount before format returned: %d", err);
    }

    // Format the filesystem
    err = lfs_format((lfs_t*)&lfs, (lfs_config*)lfs_cfg);
    if (err)
    {
        if (cfg.debug)
        {
            ESP_LOGE(TAG, "Format failed: %d", err);
        }
        // Try to remount even if format failed
        lfs_mount((lfs_t*)&lfs, (lfs_config*)lfs_cfg);
        return false;
    }

    // Remount the formatted filesystem
    err = lfs_mount((lfs_t*)&lfs, (lfs_config*)lfs_cfg);
    if (err)
    {
        if (cfg.debug)
        {
            ESP_LOGE(TAG, "Mount after format failed: %d", err);
        }
        return false;
    }

    if (cfg.debug)
    {
        ESP_LOGI(TAG, "Filesystem formatted successfully");
    }

    return true;
}

const char* TR_LogToFlash::currentFilename() const
{
    // Return without leading '/' for compatibility
    return current_filename[0] == '/' ? current_filename + 1 : current_filename;
}

bool TR_LogToFlash::isLoggingActive() const
{
    // Report active while stop is pending (drain in progress)
    // so toggle commands don't re-start logging
    return logging_active || end_flight_requested;
}

// ============================================================================
// SPI bus mutex (always active — NAND alone needs it; see begin())
// ============================================================================

void TR_LogToFlash::spiAcquire()
{
    if (!spi_mutex_) return;
    // #834 item 2: the parking task already owns this bus exclusively and
    // the mutex is not recursive — pass through instead of deadlocking it.
    if (spi_park_owner_ == xTaskGetCurrentTaskHandle()) return;
    // #398: measure time blocked acquiring + hold duration, so the stats
    // window shows exactly which flush-side work starves the parser's MRAM
    // pushes (the multi-second parser_max mystery).
    const int64_t t0 = esp_timer_get_time();
    xSemaphoreTake(spi_mutex_, portMAX_DELAY);
    const int64_t now = esp_timer_get_time();
    const uint32_t waited = (uint32_t)(now - t0);
    if (waited > spi_wait_max_us_) spi_wait_max_us_ = waited;
    spi_hold_start_us_ = now;
}

void TR_LogToFlash::spiRelease()
{
    if (!spi_mutex_) return;
    // #834 item 2: never unpark. Without this half a single re-entrant
    // call would hand the bus back moments before the reset.
    if (spi_park_owner_ == xTaskGetCurrentTaskHandle()) return;
    const uint32_t held = (uint32_t)(esp_timer_get_time() - spi_hold_start_us_);
    if (held > spi_hold_max_us_) spi_hold_max_us_ = held;
    xSemaphoreGive(spi_mutex_);
}

// #834 item 2: one-way park of the shared SPI bus ahead of a reset. See the
// header for the contract. Deliberately does NOT go through spiAcquire() —
// this wait is not a contention sample and must not pollute spi_wait_max_us,
// and it needs a bounded take where spiAcquire() uses portMAX_DELAY.
bool TR_LogToFlash::parkSpiBusForReset(uint32_t timeout_ms)
{
    if (!spi_mutex_)
    {
        // begin() never got as far as creating it, which means begin()
        // returned false and the flush task was never started. Nothing drives
        // this bus, so there is nothing to park and the reset is already safe.
        return true;
    }
    if (spi_park_owner_ != nullptr) return true;          // idempotent

    if (xSemaphoreTake(spi_mutex_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return false;
    }
    spi_park_owner_ = xTaskGetCurrentTaskHandle();
    // No spiRelease() — intentional. The bus stays ours until the reset.
    return true;
}

// ============================================================================
// MRAM SPI helpers (MR25H10 — 128 KB, byte-addressable, no erase needed)
// ============================================================================

void TR_LogToFlash::mramWriteBytes(uint32_t addr, const uint8_t* data, uint32_t len)
{
    spiAcquire();

    // WREN
    spi->beginTransaction(spi_mram);
    csLow(cfg.mram_cs);
    spi->transfer(MRAM_WREN);
    csHigh(cfg.mram_cs);
    spi->endTransaction();

    // WRITE
    uint32_t a = addr % ring_size_;
    spi->beginTransaction(spi_mram);
    csLow(cfg.mram_cs);
    spi->transfer(MRAM_WRITE);
    spi->transfer((a >> 16) & 0xFF);
    spi->transfer((a >> 8) & 0xFF);
    spi->transfer(a & 0xFF);
    spi->writeBytes(data, len);
    csHigh(cfg.mram_cs);
    spi->endTransaction();

    spiRelease();
}

void TR_LogToFlash::mramReadBytes(uint32_t addr, uint8_t* out, uint32_t len)
{
    spiAcquire();

    uint32_t a = addr % ring_size_;
    spi->beginTransaction(spi_mram);
    csLow(cfg.mram_cs);
    spi->transfer(MRAM_READ);
    spi->transfer((a >> 16) & 0xFF);
    spi->transfer((a >> 8) & 0xFF);
    spi->transfer(a & 0xFF);

    // ESP32 SPI: send dummy bytes while reading
    uint8_t dummy[64];
    memset(dummy, 0x00, sizeof(dummy));
    uint32_t remaining = len;
    uint8_t* dst = out;
    while (remaining > 0)
    {
        uint32_t chunk = (remaining > sizeof(dummy)) ? sizeof(dummy) : remaining;
        spi->transferBytes(dummy, dst, chunk);
        dst += chunk;
        remaining -= chunk;
    }

    csHigh(cfg.mram_cs);
    spi->endTransaction();

    spiRelease();
}

// Raw MRAM access for clients (e.g. FlightSnapshot region) that have
// reserved space above ring_size_.  Same SPI primitives as
// mramWriteBytes/mramReadBytes but no ring modulo — caller controls the
// absolute address.  Returns false if MRAM isn't enabled.
// #826: does a real MRAM answer on cfg.mram_cs?
//
// Deliberately NON-DESTRUCTIVE. Every byte of this part is live at boot: after
// an unclean shutdown drainMramToSink() replays the whole ring [0, mram_size)
// through the write sink (#274), and the caller's reserved region above it
// holds the FlightSnapshot (#104) and the dirty marker. So this cannot be the
// obvious scratch write/read-back — there is no scratch. Toggling WEL changes
// only the status register and leaves memory untouched.
//
// It also cannot be fooled by an absent part, which a bare RDSR could be: with
// nothing driving MISO the bus floats to a CONSTANT (0x00, 0xFF or noise), and
// 0x00 is a perfectly plausible status byte — an unprotected part with WEL
// clear. Demanding two DIFFERENT values in response to two commands we issue
// means any constant fails one of the two checks, and noise only passes if it
// happens to land correctly twice in a row.
//
// Runs before spi_mutex_ exists, which is safe: begin() is single-threaded
// (the flush task is not started and nandInit() has not run), and the NAND CS
// was driven high above so it cannot answer these opcodes.
bool TR_LogToFlash::mramProbe()
{
    auto sendCmd = [this](uint8_t op) {
        spi->beginTransaction(spi_mram);
        csLow(cfg.mram_cs);
        spi->transfer(op);
        csHigh(cfg.mram_cs);
        spi->endTransaction();
    };
    auto readStatus = [this]() -> uint8_t {
        spi->beginTransaction(spi_mram);
        csLow(cfg.mram_cs);
        spi->transfer(MRAM_RDSR);
        const uint8_t sr = spi->transfer(0x00);
        csHigh(cfg.mram_cs);
        spi->endTransaction();
        return sr;
    };

    sendCmd(MRAM_WREN);
    const uint8_t sr_set = readStatus();
    sendCmd(MRAM_WRDI);            // leave WEL clear — the safe resting state
    const uint8_t sr_clr = readStatus();

    const bool ok = tr::mramProbeVerdict(sr_set, sr_clr);
    if (!ok)
    {
        ESP_LOGE(TAG, "MRAM probe FAILED on CS %d: status after WREN=0x%02X, "
                      "after WRDI=0x%02X (expected WEL set then clear)",
                 cfg.mram_cs, sr_set, sr_clr);
    }
    return ok;
}

bool TR_LogToFlash::mramRawWrite(uint32_t addr, const uint8_t* data, uint32_t len)
{
    if (!use_mram_ || data == nullptr || len == 0) return false;

    spiAcquire();

    // WREN
    spi->beginTransaction(spi_mram);
    csLow(cfg.mram_cs);
    spi->transfer(MRAM_WREN);
    csHigh(cfg.mram_cs);
    spi->endTransaction();

    // WRITE — absolute address, no wrap
    spi->beginTransaction(spi_mram);
    csLow(cfg.mram_cs);
    spi->transfer(MRAM_WRITE);
    spi->transfer((addr >> 16) & 0xFF);
    spi->transfer((addr >> 8) & 0xFF);
    spi->transfer(addr & 0xFF);
    spi->writeBytes(data, len);
    csHigh(cfg.mram_cs);
    spi->endTransaction();

    spiRelease();
    return true;
}

bool TR_LogToFlash::mramRawRead(uint32_t addr, uint8_t* out, uint32_t len)
{
    if (!use_mram_ || out == nullptr || len == 0) return false;

    spiAcquire();

    spi->beginTransaction(spi_mram);
    csLow(cfg.mram_cs);
    spi->transfer(MRAM_READ);
    spi->transfer((addr >> 16) & 0xFF);
    spi->transfer((addr >> 8) & 0xFF);
    spi->transfer(addr & 0xFF);

    uint8_t dummy[64];
    memset(dummy, 0x00, sizeof(dummy));
    uint32_t remaining = len;
    uint8_t* dst = out;
    while (remaining > 0)
    {
        uint32_t chunk = (remaining > sizeof(dummy)) ? sizeof(dummy) : remaining;
        spi->transferBytes(dummy, dst, chunk);
        dst += chunk;
        remaining -= chunk;
    }

    csHigh(cfg.mram_cs);
    spi->endTransaction();

    spiRelease();
    return true;
}

// ============================================================================
// Ring buffer helpers (MRAM or RAM, depending on use_mram_)
// ============================================================================

bool TR_LogToFlash::ringPush(const uint8_t* data, uint32_t len)
{
    // Serialize against concurrent pushes (parser and oc_loop can preempt
    // each other on Core 1) and against clearRing (Core 0 flush task). This
    // closes the #74 race where a push snapshotted rb_head before clearRing
    // ran and then wrote a stale rb_head value, clobbering the reset.
    if (push_mutex_) xSemaphoreTake(push_mutex_, portMAX_DELAY);
    const bool ok = ringPushLocked(data, len);
    if (push_mutex_) xSemaphoreGive(push_mutex_);
    return ok;
}

bool TR_LogToFlash::ringPushLocked(const uint8_t* data, uint32_t len)
{
    if (len == 0 || len > ring_size_)
    {
        return false;
    }

    // Read current count under spinlock
    portENTER_CRITICAL(&ring_mux_);
    const uint32_t count_now = rb_count;
    portEXIT_CRITICAL(&ring_mux_);

    if (logging_active)
    {
        // During flight: reject if no room — flush task is draining the tail,
        // so we must NOT touch rb_tail from this core.
        if (ring_prelaunch_cap_ - count_now < len)
        {
            rb_overruns++;
            return false;
        }
    }
    else
    {
        // Pre-launch: drop oldest frames from tail to make room (single-threaded,
        // flush task not touching tail yet).  Headers are walked in chunked
        // window reads — the old per-frame 6-byte ringPeekAt cost one full
        // MRAM SPI transaction per dropped frame, which at the prelaunch cap
        // roughly doubled the per-frame push cost (#74 noted it; the 1920 Hz
        // stream made it a throughput ceiling).
        bool did_overrun = false;
        uint32_t local_count = count_now;
        while (ring_prelaunch_cap_ - local_count < len)
        {
            if (local_count < 6)
            {
                rb_drop_oldest_bytes += local_count;
                clearRingLocked();
                local_count = 0;
                break;
            }

            uint8_t win[512];
            const uint32_t take = (local_count < sizeof(win))
                                      ? local_count : (uint32_t)sizeof(win);
            ringPeekAt(rb_tail, win, take);

            bool cleared = false;
            uint32_t off = 0;
            uint32_t dropped_here = 0;
            while (ring_prelaunch_cap_ - local_count < len && off + 6 <= take)
            {
                if (win[off] != 0xAA || win[off + 1] != 0x55 ||
                    win[off + 2] != 0xAA || win[off + 3] != 0x55)
                {
                    if (cfg.debug)
                    {
                        ESP_LOGW(TAG, "ringPush: bad SOF at tail, clearing ring");
                    }
                    rb_drop_oldest_bytes += local_count + dropped_here;
                    rb_bad_sof_clears++;
                    clearRingLocked();
                    local_count = 0;
                    cleared = true;
                    break;
                }

                const uint32_t payload_len = win[off + 5];
                const uint32_t frame_size = 4 + 1 + 1 + payload_len + 2;

                if (frame_size > local_count)
                {
                    if (cfg.debug)
                    {
                        ESP_LOGW(TAG, "ringPush: partial frame at tail (%lu > %lu), clearing ring",
                                      (unsigned long)frame_size, (unsigned long)local_count);
                    }
                    rb_drop_oldest_bytes += local_count + dropped_here;
                    rb_bad_sof_clears++;
                    clearRingLocked();
                    local_count = 0;
                    cleared = true;
                    break;
                }

                // Frame extends past this window: re-read from the new tail.
                // frame_size <= MAX_FRAME < sizeof(win), so the next window
                // always makes progress.
                if (off + frame_size > take)
                {
                    break;
                }

                off += frame_size;
                local_count -= frame_size;
                dropped_here += frame_size;
                did_overrun = true;
            }

            if (cleared)
            {
                break;
            }
            if (dropped_here > 0)
            {
                rb_tail = (rb_tail + dropped_here) % ring_size_;
                rb_drop_oldest_bytes += dropped_here;
                portENTER_CRITICAL(&ring_mux_);
                rb_count = local_count;
                portEXIT_CRITICAL(&ring_mux_);
            }
        }
        if (did_overrun)
        {
            rb_overruns++;
        }
    }

    // Write data to ring at rb_head (only this core writes head)
    const uint32_t head = rb_head;
    const uint32_t to_end = ring_size_ - head;
    if (use_mram_)
    {
        if (len <= to_end)
        {
            mramWriteBytes(head, data, len);
        }
        else
        {
            mramWriteBytes(head, data, to_end);
            mramWriteBytes(0, data + to_end, len - to_end);
        }
    }
    else
    {
        if (len <= to_end)
        {
            memcpy(ring_buf_ + head, data, len);
        }
        else
        {
            memcpy(ring_buf_ + head, data, to_end);
            memcpy(ring_buf_, data + to_end, len - to_end);
        }
    }

    rb_head = (head + len) % ring_size_;

    portENTER_CRITICAL(&ring_mux_);
    rb_count += len;
    const uint32_t new_count = rb_count;
    portEXIT_CRITICAL(&ring_mux_);

    ringpush_bytes_ += len;

    if (new_count > rb_highwater)
    {
        rb_highwater = new_count;
    }
    return true;
}

uint32_t TR_LogToFlash::ringPop(uint8_t* out, uint32_t len)
{
    // #370: serialize against ringPush's prelaunch drop-oldest path, which
    // advances rb_tail and stores rb_count ABSOLUTELY under push_mutex_.
    // At the logging-activation edge a Core-1 push that sampled
    // logging_active==false microseconds before the flush task flipped it
    // can still be inside that loop while this pop (Core 0) advances the
    // same rb_tail — rewinding the tail into consumed data and clobbering
    // the count, which garbles the prelaunch + early-boost drain into the
    // bad-SOF -> clearRing path.  #74 covered push-vs-push and
    // push-vs-clearRing; this closes push-vs-pop.  Lock order matches the
    // push side (push_mutex_ -> spi_mutex_ inside the MRAM access), so no
    // inversion.  Cost: a push can block for one pop's MRAM read (<= one
    // page) — the two already serialize on spi_mutex_ per transaction, so
    // the added wait is the same order of magnitude.
    if (push_mutex_) xSemaphoreTake(push_mutex_, portMAX_DELAY);

    // Read count under spinlock
    portENTER_CRITICAL(&ring_mux_);
    const uint32_t count_now = rb_count;
    portEXIT_CRITICAL(&ring_mux_);

    if (len == 0 || len > count_now)
    {
        if (push_mutex_) xSemaphoreGive(push_mutex_);
        return 0;
    }

    const uint32_t tail = rb_tail;
    const uint32_t to_end = ring_size_ - tail;
    if (use_mram_)
    {
        if (len <= to_end)
        {
            mramReadBytes(tail, out, len);
        }
        else
        {
            mramReadBytes(tail, out, to_end);
            mramReadBytes(0, out + to_end, len - to_end);
        }
    }
    else
    {
        if (len <= to_end)
        {
            memcpy(out, ring_buf_ + tail, len);
        }
        else
        {
            memcpy(out, ring_buf_ + tail, to_end);
            memcpy(out + to_end, ring_buf_, len - to_end);
        }
    }
    rb_tail = (tail + len) % ring_size_;

    portENTER_CRITICAL(&ring_mux_);
    rb_count -= len;
    portEXIT_CRITICAL(&ring_mux_);

    ringpop_bytes_ += len;

    if (push_mutex_) xSemaphoreGive(push_mutex_);
    return len;
}

void TR_LogToFlash::ringPeekAt(uint32_t offset, uint8_t* out, uint32_t len)
{
    // Read len bytes starting at ring buffer position 'offset' without consuming.
    // Handles wrap-around at the ring boundary.
    const uint32_t to_end = ring_size_ - offset;
    if (use_mram_)
    {
        if (len <= to_end)
        {
            mramReadBytes(offset, out, len);
        }
        else
        {
            mramReadBytes(offset, out, to_end);
            mramReadBytes(0, out + to_end, len - to_end);
        }
    }
    else
    {
        if (len <= to_end)
        {
            memcpy(out, ring_buf_ + offset, len);
        }
        else
        {
            memcpy(out, ring_buf_ + offset, to_end);
            memcpy(out + to_end, ring_buf_, len - to_end);
        }
    }
}

void TR_LogToFlash::nandWREN()
{
    spi->beginTransaction(spi_nand);
    csLow(cfg.nand_cs);
    spi->transfer(NAND_WREN);
    csHigh(cfg.nand_cs);
    spi->endTransaction();
}

uint8_t TR_LogToFlash::nandGetFeature(uint8_t addr)
{
    spi->beginTransaction(spi_nand);
    csLow(cfg.nand_cs);
    spi->transfer(NAND_GETFEAT);
    spi->transfer(addr);
    const uint8_t v = spi->transfer(0x00);
    csHigh(cfg.nand_cs);
    spi->endTransaction();
    return v;
}

void TR_LogToFlash::nandSetFeature(uint8_t addr, uint8_t val)
{
    nandWREN();
    spi->beginTransaction(spi_nand);
    csLow(cfg.nand_cs);
    spi->transfer(NAND_SETFEAT);
    spi->transfer(addr);
    spi->transfer(val);
    csHigh(cfg.nand_cs);
    spi->endTransaction();
}

bool TR_LogToFlash::nandWaitReady(uint32_t timeout_us)
{
    const uint32_t t0 = micros();
    while (true)
    {
        // Acquire SPI mutex for each poll so Core 1 can do MRAM writes between polls
        spiAcquire();
        const uint8_t st = nandGetFeature(FEAT_STAT);
        spiRelease();

        if ((st & STAT_OIP) == 0)
        {
            return true;
        }
        if ((micros() - t0) > timeout_us)
        {
            return false;
        }
        // Yield to other tasks while NAND is busy
        if (flush_task_running_)
        {
            vTaskDelay(1);  // ~1ms yield — NAND erase takes 2-5ms
        }
        else
        {
            delayMicroseconds(30);  // Single-threaded path (startup/recovery)
        }
    }
}

bool TR_LogToFlash::nandEraseBlock(uint32_t blockIndex)
{
    const uint32_t row = blockIndex * geom_.pages_per_blk;

    spiAcquire();
    nandWREN();
    spi->beginTransaction(spi_nand);
    csLow(cfg.nand_cs);
    spi->transfer(NAND_BLKERASE);
    spi->transfer((row >> 16) & 0xFF);
    spi->transfer((row >> 8) & 0xFF);
    spi->transfer(row & 0xFF);
    csHigh(cfg.nand_cs);
    spi->endTransaction();
    spiRelease();

    if (!nandWaitReady())  // acquires/releases per poll internally
    {
        nand_erase_fail++;
        return false;
    }

    spiAcquire();
    const uint8_t st = nandGetFeature(FEAT_STAT);
    spiRelease();

    if (st & STAT_EFAIL)
    {
        nand_erase_fail++;
        return false;
    }
    nand_erase_ops++;
    return true;
}

bool TR_LogToFlash::nandProgramPage(uint32_t rowPageAddr, const uint8_t* data, uint32_t len)
{
    if (len != geom_.page_size)
    {
        return false;
    }

    spiAcquire();
    nandWREN();
    spi->beginTransaction(spi_nand);
    csLow(cfg.nand_cs);
    spi->transfer(NAND_PROGLOAD);
    spi->transfer(0x00);
    spi->transfer(0x00);
    spi->writeBytes(data, len);
    csHigh(cfg.nand_cs);
    spi->endTransaction();

    spi->beginTransaction(spi_nand);
    csLow(cfg.nand_cs);
    spi->transfer(NAND_PROGEXEC);
    spi->transfer((rowPageAddr >> 16) & 0xFF);
    spi->transfer((rowPageAddr >> 8) & 0xFF);
    spi->transfer(rowPageAddr & 0xFF);
    csHigh(cfg.nand_cs);
    spi->endTransaction();
    spiRelease();

    if (!nandWaitReady())  // acquires/releases per poll internally
    {
        nand_prog_fail++;
        return false;
    }

    spiAcquire();
    const uint8_t st = nandGetFeature(FEAT_STAT);
    spiRelease();

    if (st & STAT_PFAIL)
    {
        nand_prog_fail++;
        return false;
    }
    nand_prog_ops++;
    nand_bytes_written += len;
    return true;
}

bool TR_LogToFlash::nandReadPage(uint32_t rowPageAddr, uint8_t* out, uint32_t len)
{
    if (len > geom_.page_size)
    {
        return false;
    }

    spiAcquire();
    spi->beginTransaction(spi_nand);
    csLow(cfg.nand_cs);
    spi->transfer(NAND_PAGEREAD);
    spi->transfer((rowPageAddr >> 16) & 0xFF);
    spi->transfer((rowPageAddr >> 8) & 0xFF);
    spi->transfer(rowPageAddr & 0xFF);
    csHigh(cfg.nand_cs);
    spi->endTransaction();
    spiRelease();

    if (!nandWaitReady())  // acquires/releases per poll internally
    {
        return false;
    }

    spiAcquire();
    spi->beginTransaction(spi_nand);
    csLow(cfg.nand_cs);
    spi->transfer(NAND_READCACHE);
    spi->transfer(0x00);
    spi->transfer(0x00);
    spi->transfer(0x00);

    uint8_t dummy[64];
    memset(dummy, 0x00, sizeof(dummy));
    uint32_t remaining = len;
    uint8_t* dst = out;
    while (remaining > 0)
    {
        const uint32_t chunk = (remaining > sizeof(dummy)) ? sizeof(dummy) : remaining;
        spi->transferBytes(dummy, dst, chunk);
        dst += chunk;
        remaining -= chunk;
    }
    csHigh(cfg.nand_cs);
    spi->endTransaction();
    spiRelease();
    return true;
}

bool TR_LogToFlash::nandReadBytesAt(uint32_t rowPageAddr, uint32_t column,
                                     uint8_t* out, uint32_t len)
{
    if (len == 0 || out == nullptr) return false;

    // Stage 1: load the target page into the chip's cache.
    spiAcquire();
    spi->beginTransaction(spi_nand);
    csLow(cfg.nand_cs);
    spi->transfer(NAND_PAGEREAD);
    spi->transfer((rowPageAddr >> 16) & 0xFF);
    spi->transfer((rowPageAddr >> 8) & 0xFF);
    spi->transfer(rowPageAddr & 0xFF);
    csHigh(cfg.nand_cs);
    spi->endTransaction();
    spiRelease();

    if (!nandWaitReady())
    {
        return false;  // treat a stuck page-read as a suspect block
    }

    // Stage 2: stream `len` bytes out of the cache starting at `column`.
    // MT29F READCACHE (0x03) takes a 2-byte column address then 1 dummy byte.
    spiAcquire();
    spi->beginTransaction(spi_nand);
    csLow(cfg.nand_cs);
    spi->transfer(NAND_READCACHE);
    spi->transfer((column >> 8) & 0xFF);
    spi->transfer(column & 0xFF);
    spi->transfer(0x00);  // dummy

    uint8_t dummy[64];
    memset(dummy, 0x00, sizeof(dummy));
    uint32_t remaining = len;
    uint8_t* dst = out;
    while (remaining > 0)
    {
        const uint32_t chunk = (remaining > sizeof(dummy)) ? sizeof(dummy) : remaining;
        spi->transferBytes(dummy, dst, chunk);
        dst += chunk;
        remaining -= chunk;
    }
    csHigh(cfg.nand_cs);
    spi->endTransaction();
    spiRelease();
    return true;
}

bool TR_LogToFlash::nandInit()
{
    nandSetFeature(FEAT_PROT, 0x00);

    spi->beginTransaction(spi_nand);
    csLow(cfg.nand_cs);
    spi->transfer(NAND_RDID);
    // SPI NAND parts (Macronix MX35LF, Winbond W25N, Micron MT29F) require a
    // dummy / address byte after 0x9F before the MID/DID stream begins.
    (void)spi->transfer(0x00);
    const uint8_t mid = spi->transfer(0x00);
    const uint8_t did = spi->transfer(0x00);
    csHigh(cfg.nand_cs);
    spi->endTransaction();

    if (cfg.debug)
    {
        ESP_LOGI(TAG, "RDID MID=0x%02X DID=0x%02X", mid, did);
    }

    // Detect chip replacement (or first boot under firmware that records the
    // chip ID) and wipe the persisted bad-block bitmap so a fresh chip isn't
    // tarnished by the previous chip's bad-block history. Skip when RDID looks
    // like a dead bus (0x0000 / 0xFFFF) — leave the bitmap alone and let the
    // rest of begin() surface the real error.
    const uint16_t current_chip_id = (uint16_t)(((uint16_t)mid << 8) | (uint16_t)did);
    const bool dead_bus = (current_chip_id == 0x0000) || (current_chip_id == 0xFFFF);
    // #511: stash both for the boot-scan gate in begin() — the gate must not
    // read bad_block_chip_id_, which the wipe below rewrites pre-scan.
    current_chip_id_ = current_chip_id;
    nand_dead_bus_ = dead_bus;

    // #671: resolve the RUNTIME geometry from the chip ID. Table hit -> that
    // part's numbers; unknown or dead-bus ID -> the legacy fallback, i.e.
    // exactly the pre-#671 behaviour (keeps the V8 bench byte-identical and
    // fails open for unlisted 4 Gbit parts). Unknown is an ERROR, not a
    // WARN: on a 2 KB-page part the fallback corrupts data, so an unlisted
    // ID must get added to nand_geometry.h, and this line is the tripwire.
    const bool geom_known = nandGeometryForId(current_chip_id, &geom_);
    if (dead_bus)
    {
        // geom_ is the legacy fallback; the dead bus is reported by the
        // boot-scan gate in begin() and the mount failure that follows.
    }
    else if (geom_known)
    {
        ESP_LOGI(TAG, "NAND %s: %lu B pages x %lu/blk x %lu blocks (%lu MB)",
                 geom_.name,
                 (unsigned long)geom_.page_size,
                 (unsigned long)geom_.pages_per_blk,
                 (unsigned long)geom_.block_count,
                 (unsigned long)(geom_.blockSize() / 1024 * geom_.block_count / 1024));
    }
    else
    {
        ESP_LOGE(TAG, "UNKNOWN NAND ID 0x%04X — assuming legacy geometry "
                      "(%lu B pages, %lu blocks). If this part has 2 KB pages "
                      "this WILL corrupt data: add its ID to nand_geometry.h.",
                 (unsigned)current_chip_id,
                 (unsigned long)geom_.page_size,
                 (unsigned long)geom_.block_count);
    }

    // #671: the NVS bitmap blob was loaded BEFORE this RDID read (begin()
    // ordering), so its length check had to wait until the expected size was
    // known. Exact-size-for-this-chip keeps the #511 trusted-map gate
    // meaningful: a blob persisted under a different geometry (chip swapped
    // on a bench board) fails here and forces a rescan, which is the safe
    // outcome.
    bad_block_map_blob_ok_ = (bad_block_blob_len_ == geom_.bitmapBytes()) &&
                             (bad_block_scan_page_size_ == geom_.page_size);
    if (!bad_block_map_blob_ok_)
    {
        // Pre-#671 semantic, restored now that the expected size is known: a
        // wrong-length blob is untrusted IN FULL. Keeping its partial bits
        // would poison the forced rescan — the scan SKIPS already-known-bad
        // blocks, so spurious bad bits from a truncated/foreign-geometry blob
        // would never be re-probed and would then be persisted as truth.
        if (bad_block_blob_len_ != 0)
        {
            memset(bad_block_bitmap_, 0, sizeof(bad_block_bitmap_));
        }
        // Dirty EVEN when the blob was absent (len 0, already clean): the
        // forced rescan's persist must write the blob, or a chip whose NVS
        // holds a matching "chip"/"scanned" but no "map" — e.g. a scan that
        // found zero bad blocks and so never dirtied anything — re-runs the
        // full boot scan on every boot forever. (Review finding; the old code
        // converged by accident via load-side dirtying.)
        bad_block_bitmap_dirty_ = true;
    }

    if (!dead_bus && current_chip_id != bad_block_chip_id_)
    {
        if (cfg.debug)
        {
            ESP_LOGW(TAG, "NAND chip changed (saved=0x%04X, current=0x%04X) — clearing bad-block map",
                     (unsigned)bad_block_chip_id_, (unsigned)current_chip_id);
        }
        memset(bad_block_bitmap_, 0, sizeof(bad_block_bitmap_));
        bad_block_chip_id_ = current_chip_id;
        bad_block_bitmap_dirty_ = true;
        // Persist immediately so a reboot during the rest of begin() doesn't
        // re-load the stale bitmap on next start.
        persistBadBlocksIfDirty();
    }

    return true;
}

// ============================================================================
// LittleFS Block Device Adapter
// ============================================================================

int TR_LogToFlash::lfsBlockRead(const struct lfs_config *c, lfs_block_t block,
                                 lfs_off_t off, void *buffer, lfs_size_t size)
{
    TR_LogToFlash* self = (TR_LogToFlash*)c->context;

    self->lfs_cb_reads_++;

    // Short-circuit known-bad blocks without touching NAND (#47).
    if (self->isBlockBad(block))
    {
        self->bad_block_skips_++;
        return LFS_ERR_CORRUPT;
    }

    uint8_t* buf = (uint8_t*)buffer;
    lfs_size_t bytes_read = 0;

    // Yield 1 tick to let IDLE task reset the watchdog.
    // vTaskDelay(0) only yields to equal-or-higher priority,
    // which doesn't help IDLE (priority 0).
    vTaskDelay(1);

    while (bytes_read < size)
    {
        uint32_t block_offset = off + bytes_read;
        uint32_t page_in_block = block_offset / self->geom_.page_size;
        uint32_t offset_in_page = block_offset % self->geom_.page_size;

        uint32_t row_page = (block * self->geom_.pages_per_blk) + page_in_block;

        // Read full page into temp buffer
        uint8_t page_buf[NAND_PAGE_SIZE_MAX];
        if (!self->nandReadPage(row_page, page_buf, self->geom_.page_size))
        {
            // Read failure — the block is suspect.  Tell LFS it's bad so it
            // relocates, and remember it so we skip on future mounts.
            self->markBlockBad(block);
            return LFS_ERR_CORRUPT;
        }

        // Copy requested portion
        uint32_t bytes_to_copy = (size - bytes_read < self->geom_.page_size - offset_in_page)
                                 ? (size - bytes_read)
                                 : (self->geom_.page_size - offset_in_page);
        memcpy(buf + bytes_read, page_buf + offset_in_page, bytes_to_copy);
        bytes_read += bytes_to_copy;
    }

    return LFS_ERR_OK;
}

int TR_LogToFlash::lfsBlockProg(const struct lfs_config *c, lfs_block_t block,
                                 lfs_off_t off, const void *buffer, lfs_size_t size)
{
    TR_LogToFlash* self = (TR_LogToFlash*)c->context;

    self->lfs_cb_progs_++;

    // Short-circuit known-bad blocks (#47) — returns ~µs instead of paying
    // the full LFS remap cost that would otherwise fire on every encounter.
    if (self->isBlockBad(block))
    {
        self->bad_block_skips_++;
        return LFS_ERR_CORRUPT;
    }

    // IMPORTANT: NAND pages can only be programmed ONCE after erase
    // We cannot do read-modify-write on NAND
    // LittleFS should only call this with page-aligned offsets and sizes

    // Verify alignment (LittleFS should respect prog_size)
    if (off % self->geom_.page_size != 0 || size % self->geom_.page_size != 0)
    {
        return LFS_ERR_INVAL;
    }

    const uint8_t* buf = (const uint8_t*)buffer;
    lfs_size_t bytes_written = 0;

    while (bytes_written < size)
    {
        uint32_t block_offset = off + bytes_written;
        uint32_t page_in_block = block_offset / self->geom_.page_size;
        uint32_t row_page = (block * self->geom_.pages_per_blk) + page_in_block;

        // Write full page directly (no read-modify-write)
        if (!self->nandProgramPage(row_page, buf + bytes_written, self->geom_.page_size))
        {
            // Program failed (STAT_PFAIL or timeout).  Mark the block bad
            // and tell LFS to relocate — CORRUPT (not IO) triggers the
            // remap path rather than propagating a hard FS error.
            self->markBlockBad(block);
            return LFS_ERR_CORRUPT;
        }

        bytes_written += self->geom_.page_size;
    }

    return LFS_ERR_OK;
}

int TR_LogToFlash::lfsBlockErase(const struct lfs_config *c, lfs_block_t block)
{
    TR_LogToFlash* self = (TR_LogToFlash*)c->context;

    self->lfs_cb_erases_++;

    // Short-circuit known-bad blocks (#47).
    if (self->isBlockBad(block))
    {
        self->bad_block_skips_++;
        return LFS_ERR_CORRUPT;
    }

    // Yield before erase — this is the slowest NAND operation (~2ms per block)
    vTaskDelay(1);

    const int64_t _t0 = esp_timer_get_time();
    bool ok = self->nandEraseBlock(block);
    uint32_t _dt = (uint32_t)(esp_timer_get_time() - _t0);
    if (_dt > self->erase_max_us_) self->erase_max_us_ = _dt;
    if (_dt > (uint32_t)STALL_THRESHOLD_US) {
        ESP_LOGW(TAG, "STALL: nandEraseBlock(%lu) took %lu us",
                 (unsigned long)block, (unsigned long)_dt);
    }

    if (!ok)
    {
        // Erase failed (STAT_EFAIL or timeout) — mark bad and let LFS
        // relocate rather than bubble an I/O error up.
        self->markBlockBad(block);
        return LFS_ERR_CORRUPT;
    }
    return LFS_ERR_OK;
}

int TR_LogToFlash::lfsBlockSync(const struct lfs_config *c)
{
    // No-op for NAND (no caching to flush)
    return LFS_ERR_OK;
}

// ============================================================================
// CORE FUNCTIONS (LittleFS-based logging)
// ============================================================================

void TR_LogToFlash::openLogSession()
{
    if (file_open) return;

    // Write-sink mode (issue #50 Stage 2c-3c): TR_FlightLog owns the hot path
    // and LFS has no role in flight logging. Skip all LFS operations — the
    // filename is synthesized for log messages only; the flush task will drain
    // into the sink, not into an lfs_file_t.
    if (cfg.write_sink != nullptr)
    {
        snprintf(current_filename, sizeof(current_filename), "/flight.bin");
        file_open = true;
        current_file_bytes = 0;
        current_file_has_timestamp = false;
        page_buf_idx = 0;
        pages_since_sync_ = 0;
        end_flight_requested = false;
        if (cfg.debug) ESP_LOGI(TAG, "openLogSession (sink mode, LFS skipped)");
        return;
    }

    // Generate filename - find next unused number
    struct lfs_info info;

    // Start with numbered filename (will rename when timestamp arrives)
    int idx = 1;
    do {
        snprintf(current_filename, sizeof(current_filename),
                 "/flight_%03d.bin", idx++);
    } while (lfs_stat(&lfs, current_filename, &info) >= 0 && idx < 1000);

    if (cfg.debug)
    {
        ESP_LOGI(TAG, "Opening log file: %s", current_filename);
    }

    // Open file for writing — this is the expensive NAND operation
    int err;
    {
        LFS_TIMING_START();
        err = lfs_file_open(&lfs, &file, current_filename,
                            LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL);
        LFS_TIMING_END(open_max_us_, "lfs_file_open");
    }
    if (err)
    {
        if (cfg.debug) ESP_LOGE(TAG, "File open failed: %d", err);
        return;
    }

    file_open = true;
    current_file_bytes = 0;
    current_file_has_timestamp = false;
    page_buf_idx = 0;
    pages_since_sync_ = 0;
    end_flight_requested = false;

    // Pre-warm LFS's free-block allocator (#47 follow-up).  During flight
    // the first lfs_file_write after activate used to stall for 1.76 s
    // scanning metadata (reads=718) to rebuild the lookahead buffer.
    // Running a full traverse here, on the pad, moves that scan to
    // PRELAUNCH where timing doesn't matter — the lookahead ends up
    // populated with this FS's free-block bitmap, so the first few
    // in-flight allocations come from the buffer instead of triggering
    // another scan.
    {
        const uint32_t t0 = millis();
        auto noop_cb = [](void*, lfs_block_t) -> int { return 0; };
        int tr_err = lfs_fs_traverse(&lfs, noop_cb, nullptr);
        const uint32_t dt = millis() - t0;
        if (cfg.debug)
        {
            ESP_LOGI(TAG, "lfs_fs_traverse (allocator pre-warm): %lu ms, err=%d",
                          (unsigned long)dt, tr_err);
        }
    }

    // NOTE: logging_active and ring_prelaunch_cap_ are NOT set here.
    // They are set by activateLogging() when launch is actually detected.
    // This allows pre-creating the file during PRELAUNCH without switching
    // the ring buffer out of drop-oldest mode.
}

/// Activate logging after file is already open (fast — no NAND I/O).
/// Called when launch is detected. If the file was pre-created during
/// PRELAUNCH, this is the only thing that needs to happen at launch time.
void TR_LogToFlash::activateLogging()
{
    LFS_TIMING_START();

    // Issue #74 diagnostic: log pointer state at entry (end of prelaunch).
    ESP_LOGW(TAG, "ACT0 entry     h=%lu t=%lu c=%lu push=%llu pop=%llu",
             (unsigned long)rb_head, (unsigned long)rb_tail, (unsigned long)rb_count,
             (unsigned long long)ringpush_bytes_, (unsigned long long)ringpop_bytes_);

    // Preserve the prelaunch ring contents. The next flushRingToNand drains
    // from rb_tail forward, so the flight file starts with ~500 ms of
    // pre-command sensor data (the drop-oldest cap bounds the window).
    // clearRing is intentionally NOT called here — it was the source of the
    // clobber race in #74, and the monotonic dedup filter in processFrame
    // already rejects any stale MRAM frames that cross session boundaries.
    // Cross-boot stale MRAM is handled by runStartupRecovery at begin().
    logging_active = true;
    ring_prelaunch_cap_ = ring_size_;
    end_flight_requested = false;

    // #417: set the dirty marker HERE, not at openLogSession()/pre-create.
    // A pre-created-but-never-launched session (bench: PRELAUNCH → power cut,
    // no launch, no clean close) must NOT look "dirty" on the next boot, or it
    // spawns a bogus flight_mram_recovered_*.bin on nearly every power cycle.
    // The marker now means "logging was activated" (launch-detect start, or a
    // deliberate manual cmd-23 start — both route through activateLogging), so
    // a surviving marker flags a genuine in-flight brownout with real ring data
    // to replay (the #274 case). Cleared symmetrically by closeLogSession().
    dirty_policy_.onActivate();
    syncDirtyMarker();
    // Arm per-drain diagnostic logs for the first few flushRingToNand
    // drains. Kept small on purpose: these are blocking UART writes (~9 ms
    // each at 115200) inside the launch-critical prelaunch-drain window —
    // 20 of them measured ~180 ms of the 438 ms activation drain on the
    // 2026-07-09 bench. Four lines still show the handoff shape.
    flush_log_remaining_ = 4;

    ESP_LOGW(TAG, "ACT2 exit      h=%lu t=%lu c=%lu (logging_active=1)",
             (unsigned long)rb_head, (unsigned long)rb_tail, (unsigned long)rb_count);

    if (cfg.debug) ESP_LOGI(TAG, "Logging activated (prelaunch buffer preserved)");

    LFS_TIMING_END(activate_max_us_, "activateLogging");
}

void TR_LogToFlash::closeLogSession()
{
    if (!file_open) return;

    // Write-sink mode: the flush task wrote via the sink, so there's no open
    // lfs_file_t to flush/close. Any partial page in page_buf gets handed to
    // the sink too — matches what the LFS path does for partial final pages.
    if (cfg.write_sink != nullptr)
    {
        if (page_buf_idx > 0)
        {
            // Pad the tail chunk with the existing 0xFF fill from NAND prior
            // state is not possible here (page_buf is just RAM), so we pass
            // whatever bytes we have — the sink (writeFrame) wraps payload
            // in a PageHeader and programs a full 4096-byte page, zero-
            // padding the unused payload tail. Accepted small loss.
            //
            // #837 item 8: the result USED to be cast away. current_file_bytes
            // counted these bytes when they were popped off the ring, and it is
            // snapshotted into last_closed_session_bytes_ a few lines below and
            // handed to TR_FlightLog::finalizeFlight as final_bytes. So a failed
            // tail write — flight region full, or the tail landing on a run of
            // bad blocks — left finalize keeping a page that was never
            // programmed. readFlightPage then memcpy'd erased 0xFF into the
            // download: a file of the advertised length whose tail is garbage,
            // with nand_prog_fail unchanged so the storage scorecard stayed
            // green. Mirror the flushRingToNand !ok path instead.
            const bool tail_ok =
                cfg.write_sink(cfg.write_sink_ctx, page_buf, page_buf_idx);
            if (!tail_ok)
            {
                nand_prog_fail++;                     // -> shStorageState -> DEGRADED
                current_file_bytes -= page_buf_idx;   // never reached NAND
                ESP_LOGE(TAG, "closeLogSession: final %lu-byte page FAILED to "
                              "write — session reported as %lu bytes, not %lu",
                         (unsigned long)page_buf_idx,
                         (unsigned long)current_file_bytes,
                         (unsigned long)(current_file_bytes + page_buf_idx));
            }
            else
            {
                // The tail page is a NAND program op like any other; counting
                // it keeps nand_prog_ops / nand_bytes_written honest, which
                // they were not (one page short per session).
                nand_bytes_written += page_buf_idx;
                nand_prog_ops++;
            }
            page_buf_idx = 0;
        }
        file_open = false;
        logging_active = false;
        dirty_policy_.onCloseClean();   // #417
        syncDirtyMarker();
        persistBadBlocksIfDirty();
        ring_prelaunch_cap_ = prelaunchCap();
        if (cfg.debug)
        {
            ESP_LOGI(TAG, "closeLogSession (sink mode): %lu bytes handed off",
                     (unsigned long)current_file_bytes);
        }
        last_closed_session_bytes_ = current_file_bytes;
        current_file_bytes = 0;
        return;
    }

    // Write any remaining data in page staging buffer
    if (page_buf_idx > 0)
    {
        lfs_ssize_t written = lfs_file_write(&lfs, &file, page_buf, page_buf_idx);
        if (written > 0)
        {
            nand_bytes_written += (uint64_t)written;
        }
        page_buf_idx = 0;
    }

    // Sync and close file
    {
        LFS_TIMING_START();
        lfs_file_sync(&lfs, &file);
        LFS_TIMING_END(sync_max_us_, "lfs_file_sync(close)");
    }
    {
        LFS_TIMING_START();
        lfs_file_close(&lfs, &file);
        LFS_TIMING_END(close_max_us_, "lfs_file_close");
    }

    file_open = false;
    logging_active = false;
    dirty_policy_.onCloseClean();   // #417
    syncDirtyMarker();

    // Flush any new bad-block discoveries to NVS now — no longer in the
    // hot path, and the next session should see the same list.
    persistBadBlocksIfDirty();

    // Restore pre-launch ring cap so the buffer doesn't fill completely
    // before the next launch — leaves headroom for the initial flush.
    ring_prelaunch_cap_ = prelaunchCap();

    if (cfg.debug)
    {
        ESP_LOGI(TAG, "Closed %s (%lu bytes)",
                      current_filename, current_file_bytes);
    }

    last_closed_session_bytes_ = current_file_bytes;
    current_file_bytes = 0;
}

// ============================================================================
// Dirty flag persistence (LittleFS marker file, replaces MRAM persist state)
// ============================================================================

// #274: sentinel written to the MRAM dirty marker (cfg.dirty_marker_addr) while a
// sink-mode log session is open and cleared on a clean close. It lives in the
// non-volatile MRAM, so a surviving copy on the next boot flags unflushed ring data.
static constexpr uint32_t kMramDirtyMagic = 0x52443274u;

void TR_LogToFlash::markDirty()
{
    // Sink mode (issue #50 Stage 2c-3c): the "dirty" marker file exists so
    // checkDirtyOnStartup() can detect an unclean shutdown and replay the MRAM
    // ring into LFS. With the write_sink in place, the ring drains into
    // TR_FlightLog, not LFS — and TR_FlightLog has its own brownout recovery
    // (PageHeader + scanForBrownoutRecovery). Touching LFS here just hammers
    // block 0/1 superblock metadata for no gain ("Bad block at 0x1 /
    // Superblock 0x1 has become unwritable" warnings on chips that have been
    // reformatted many times during development).
    if (cfg.write_sink != nullptr)
    {
        // #274: sink mode skips the LittleFS marker (superblock churn) but sets a
        // non-volatile MRAM marker, so a dirty boot is still detected and the
        // surviving ring can be replayed through the sink.
        if (use_mram_ && cfg.dirty_marker_addr != 0)
        {
            const uint32_t magic = kMramDirtyMagic;
            mramRawWrite(cfg.dirty_marker_addr,
                         reinterpret_cast<const uint8_t*>(&magic), sizeof(magic));
        }
        return;
    }

    lfs_file_t f;
    int err = lfs_file_open(&lfs, &f, "/.dirty",
                            LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (err == 0)
    {
        uint8_t marker = 1;
        lfs_file_write(&lfs, &f, &marker, 1);
        lfs_file_close(&lfs, &f);
    }
}

void TR_LogToFlash::clearDirty()
{
    // See markDirty() — symmetric skip in sink mode.
    if (cfg.write_sink != nullptr)
    {
        // #274: clear the non-volatile MRAM dirty marker on a clean close.
        if (use_mram_ && cfg.dirty_marker_addr != 0)
        {
            const uint32_t zero = 0;
            mramRawWrite(cfg.dirty_marker_addr,
                         reinterpret_cast<const uint8_t*>(&zero), sizeof(zero));
        }
        return;
    }

    lfs_remove(&lfs, "/.dirty");
}

// #417: mirror the MramDirtyPolicy intent to the physical marker. This is the
// single write path for the live session — lifecycle transitions update the
// policy, then call here. markDirty()/clearDirty() remain the hardware writers
// (LittleFS marker file in non-sink mode, MRAM word in sink mode).
void TR_LogToFlash::syncDirtyMarker()
{
    if (dirty_policy_.markerShouldBeSet())
        markDirty();
    else
        clearDirty();
}

bool TR_LogToFlash::checkDirtyOnStartup()
{
    struct lfs_info info;
    return (lfs_stat(&lfs, "/.dirty", &info) >= 0);
}

bool TR_LogToFlash::checkMramDirty()
{
    if (!use_mram_ || cfg.dirty_marker_addr == 0) return false;
    uint32_t magic = 0;
    if (!mramRawRead(cfg.dirty_marker_addr,
                     reinterpret_cast<uint8_t*>(&magic), sizeof(magic)))
        return false;
    return magic == kMramDirtyMagic;
}

// #274: replay the entire surviving MRAM ring through the write_sink. The sink
// consumes (runtime page size - 16)-byte chunks (it prepends a 16-byte PageHeader),
// exactly like flushRingToNand; the ring is dumped raw and the downstream parser
// handles SOF framing / CRC / stale + partial frames (parity with the legacy LFS
// dump). The caller must have opened a destination flight on the sink first.
// Returns the number of bytes handed to the sink.
uint32_t TR_LogToFlash::drainMramToSink()
{
    if (!use_mram_ || cfg.write_sink == nullptr) return 0;
    // #671: the sink quantum is the RUNTIME payload-per-page — the same
    // sinkPayloadSize() flushRingToNand derives its chunk_target from (4080
    // legacy, 2032 on the GD5F parts). Chunk via page_buf instead of the old
    // 4080-byte constexpr STACK array: runtime sizing can't be constexpr,
    // and a page-sized stack array is exactly the pattern the
    // runStartupRecovery comment records overflowing an 8 KB task stack.
    // page_buf is idle here by construction — recovery replay runs from
    // begin()'s caller before startFlushTask() and before any log session
    // opens, and page_buf_idx is 0 until logging activates.
    const uint32_t kChunk = sinkPayloadSize();
    uint8_t* const buf = page_buf;
    uint32_t total = 0;
    for (uint32_t off = 0; off < ring_size_; off += kChunk)
    {
        const uint32_t n = (ring_size_ - off < kChunk) ? (ring_size_ - off) : kChunk;
        if (!mramRawRead(off, buf, n)) break;
        if (!cfg.write_sink(cfg.write_sink_ctx, buf, n)) break;  // sink full / failed
        total += n;
    }
    return total;
}

void TR_LogToFlash::finishMramRecovery()
{
    clearRing();
    dirty_policy_.onRecoveryFinished();   // #417
    syncDirtyMarker();                    // clears the MRAM marker (sink mode)
    pending_mram_recovery_ = false;
}

void TR_LogToFlash::clearRing()
{
    // Public entry point: acquire push_mutex_ so any in-flight ringPush on
    // Core 1 finishes before we start, and no new push can start until we
    // return. Without this guard, a push that already snapshotted rb_head
    // would clobber our reset back to a prelaunch value on its trailing
    // rb_head assignment (the #74 race).
    if (push_mutex_) xSemaphoreTake(push_mutex_, portMAX_DELAY);
    // Discard staged-but-unwritten frames too — the caller is wiping the ring,
    // and a later staging flush must not resurrect pre-wipe data.  Only here,
    // NOT in clearRingLocked(): the drop-oldest path inside ringPushLocked
    // calls clearRingLocked() while flushing staging_buf_ itself.
    staged_len_ = 0;
    staged_frames_ = 0;
    clearRingLocked();
    if (push_mutex_) xSemaphoreGive(push_mutex_);
}

void TR_LogToFlash::clearRingLocked()
{
    LFS_TIMING_START();

    // Issue #74 diagnostic: log pointer state going in so we can see what
    // the prelaunch ring looked like before the reset.
    ESP_LOGW(TAG, "CR0 pre-reset  h=%lu t=%lu c=%lu push=%llu pop=%llu",
             (unsigned long)rb_head, (unsigned long)rb_tail, (unsigned long)rb_count,
             (unsigned long long)ringpush_bytes_, (unsigned long long)ringpop_bytes_);

    portENTER_CRITICAL(&ring_mux_);
    rb_head = 0;
    rb_tail = 0;
    rb_count = 0;
    portEXIT_CRITICAL(&ring_mux_);

    ESP_LOGW(TAG, "CR1 post-reset h=%lu t=%lu c=%lu (MRAM zero-sweep next)",
             (unsigned long)rb_head, (unsigned long)rb_tail, (unsigned long)rb_count);

    // When using MRAM, zero-fill the entire ring to prevent stale data from
    // a previous session from leaking into the log file.  MRAM is non-volatile,
    // so clearRing() resetting pointers alone is not sufficient — the flush task
    // could read stale bytes if rb_count becomes inconsistent due to any race.
    //
    // Throughput is transaction-overhead-limited: at 40 MHz SPI the 128 KB of
    // data is only 26 ms, but each SPI call adds ~200 µs of CS/mode/command
    // overhead.  At 4 KB chunks that's 32 × 200 µs = 6 ms overhead vs. 512 ×
    // 200 µs = 100 ms with 256 B chunks — see #48.
    //
    // `zeros` MUST be static: the flush task has only 8 KB of stack, and a
    // stack-allocated 4 KB array left just enough for a deep LFS call chain
    // to overflow into the return address — which crashed the board with a
    // double-exception on 4/22.  BSS placement costs 4 KB of RAM but it's a
    // once-ever cost.
    if (use_mram_)
    {
        static constexpr size_t ZERO_CHUNK = 4096;
        static uint8_t zeros[ZERO_CHUNK] = {};
        for (uint32_t addr = 0; addr < ring_size_; addr += ZERO_CHUNK)
        {
            uint32_t len = (ring_size_ - addr < ZERO_CHUNK)
                         ? (ring_size_ - addr) : ZERO_CHUNK;
            mramWriteBytes(addr, zeros, len);
        }
    }

    // Issue #74 diagnostic: log pointer state after the 33 ms zero-sweep
    // to detect if ringPush races clobbered the reset.
    ESP_LOGW(TAG, "CR2 post-sweep h=%lu t=%lu c=%lu push=%llu pop=%llu",
             (unsigned long)rb_head, (unsigned long)rb_tail, (unsigned long)rb_count,
             (unsigned long long)ringpush_bytes_, (unsigned long long)ringpop_bytes_);

    LFS_TIMING_END(clear_ring_max_us_, "clearRing");
}

// ============================================================================
// Persistent bad-block bitmap (#47)
// ============================================================================
//
// NAND blocks wear out.  When LittleFS hits a failing block it does a
// synchronous remap (write-fail / erase-to-test / mark-bad / realloc) that
// can take several hundred ms on the hot log-write path — we measured up to
// 928 ms in bench testing.  Persisting the bad-block list across boots means
// the cost is paid once per block per chip lifetime; every subsequent
// encounter short-circuits in the LFS callback with LFS_ERR_CORRUPT before
// any SPI traffic.

bool TR_LogToFlash::isBlockBad(uint32_t block) const
{
    if (block >= geom_.block_count) return true;  // treat OOB as bad
    return (bad_block_bitmap_[block / 8] & (uint8_t)(1u << (block % 8))) != 0;
}

void TR_LogToFlash::markBlockBad(uint32_t block)
{
    if (block >= geom_.block_count) return;
    uint8_t& byte = bad_block_bitmap_[block / 8];
    const uint8_t mask = (uint8_t)(1u << (block % 8));
    if (!(byte & mask))
    {
        byte |= mask;
        bad_block_bitmap_dirty_ = true;
        ESP_LOGW(TAG, "Marking NAND block %lu as bad (total bad: %lu)",
                 (unsigned long)block, (unsigned long)countBadBlocks());
    }
}

uint32_t TR_LogToFlash::countBadBlocks() const
{
    uint32_t n = 0;
    for (uint32_t i = 0; i < geom_.bitmapBytes(); ++i)
    {
        n += (uint32_t)__builtin_popcount(bad_block_bitmap_[i]);
    }
    return n;
}

void TR_LogToFlash::loadBadBlocksFromNVS()
{
    Preferences prefs;
    if (!prefs.begin("bblk", true))  // read-only
    {
        // First boot, namespace doesn't exist yet — nothing to load.
        memset(bad_block_bitmap_, 0, sizeof(bad_block_bitmap_));
        bad_block_blob_len_ = 0;
        bad_block_chip_id_ = 0;
        bad_block_scanned_chip_ = 0;
        bad_block_scan_page_size_ = 0;
        bad_block_map_blob_ok_ = false;
        bad_block_bitmap_dirty_ = false;
        if (cfg.debug) ESP_LOGI(TAG, "Bad-block NVS namespace not found, starting clean");
        return;
    }
    // #671: this runs BEFORE nandInit()'s RDID read, so the expected blob
    // length (geom_.bitmapBytes()) is not known yet. Load up to the max and
    // record what was actually stored; nandInit() finishes the validation
    // once geometry is resolved. (Preferences::getBytes returns 0 when the
    // stored blob is LARGER than the buffer, and the stored length when
    // smaller — either way a mismatch fails the deferred check safely.)
    bad_block_blob_len_ = prefs.getBytes("map", bad_block_bitmap_, sizeof(bad_block_bitmap_));
    bad_block_chip_id_ = prefs.getUShort("chip", 0);
    bad_block_scanned_chip_ = prefs.getUShort("scanned", 0);  // #511: written only post-scan
    // #671: the page size the scan RAN UNDER. A scan's factory-marker probes
    // are only valid for the geometry they used (the marker column is the
    // page size), so the trusted-map gate must match it against the detected
    // chip. Missing key = 4096: every pre-#671 firmware scanned with
    // 4096-page arithmetic — correct on the legacy part (its maps stay
    // trusted, no spurious V8 rescan), wrong on a 2 KB part (forces the one
    // rescan that heals a map poisoned by old firmware on a GD5F chip).
    bad_block_scan_page_size_ = prefs.getUShort("gpage", 4096);
    prefs.end();
    bad_block_map_blob_ok_ = false;  // provisional; finalized in nandInit()
    bad_block_bitmap_dirty_ = false;
    if (bad_block_blob_len_ != sizeof(bad_block_bitmap_))
    {
        // Not necessarily an error (a 1024-block part stores 128 B), but the
        // tail must be clean either way; nandInit() decides validity.
        memset(bad_block_bitmap_ + bad_block_blob_len_, 0,
               sizeof(bad_block_bitmap_) - bad_block_blob_len_);
        // NOT marked dirty here: a length mismatch against the STATIC max is
        // normal on a 1024-block chip (its valid blob is 128 B). nandInit()
        // owns the dirty/wipe decision once the per-chip expected length is
        // known — dirtying here would rewrite a perfectly valid mini blob on
        // every session close.
    }
    const uint32_t n_bad = countBadBlocks();
    if (cfg.debug) ESP_LOGI(TAG, "Loaded bad-block map: %lu known-bad blocks (saved chip=0x%04X)",
                                  (unsigned long)n_bad, (unsigned)bad_block_chip_id_);
}

void TR_LogToFlash::persistBadBlocksIfDirty()
{
    if (!bad_block_bitmap_dirty_) return;
    Preferences prefs;
    if (!prefs.begin("bblk", false))  // read-write
    {
        if (cfg.debug) ESP_LOGW(TAG, "Bad-block NVS open failed, will retry next close");
        return;
    }
    // #671: persist exactly the detected chip's bitmap length (256 B on the
    // 2048-block parts, 128 B on the mini's 1024-block part) so the deferred
    // load-side check sees a size match on the next boot.
    prefs.putBytes("map", bad_block_bitmap_, geom_.bitmapBytes());
    prefs.putUShort("chip", bad_block_chip_id_);
    prefs.end();
    bad_block_bitmap_dirty_ = false;
    if (cfg.debug) ESP_LOGI(TAG, "Persisted bad-block map (%lu bad, chip=0x%04X)",
                                  (unsigned long)countBadBlocks(),
                                  (unsigned)bad_block_chip_id_);
}

void TR_LogToFlash::markBootScanComplete()
{
    Preferences prefs;
    if (!prefs.begin("bblk", false))  // read-write
    {
        // Not fatal: the marker stays stale and the next boot rescans.
        if (cfg.debug) ESP_LOGW(TAG, "Bad-block NVS open failed; boot scan will re-run next boot");
        return;
    }
    prefs.putUShort("scanned", current_chip_id_);
    prefs.putUShort("gpage", (uint16_t)geom_.page_size);  // #671: scan-time geometry
    prefs.end();
    bad_block_scanned_chip_ = current_chip_id_;
    bad_block_scan_page_size_ = (uint16_t)geom_.page_size;
    if (cfg.debug) ESP_LOGI(TAG, "Bad-block boot scan complete for chip 0x%04X — future boots skip it",
                                  (unsigned)current_chip_id_);
}

uint32_t TR_LogToFlash::scanBadBlocksAtBoot()
{
    const uint32_t t0 = millis();
    uint32_t n_new = 0;
    uint32_t n_scanned = 0;
    uint8_t main_byte = 0xFF;
    uint8_t spare_byte[2] = { 0xFF, 0xFF };

    for (uint32_t b = 0; b < geom_.block_count; ++b)
    {
        // Already-known bad blocks from NVS or this run — skip, no NAND work.
        if (isBlockBad(b)) continue;
        ++n_scanned;

        const uint32_t page_0_row = b * geom_.pages_per_blk;
        const uint32_t page_1_row = page_0_row + 1;

        // Option A — any read error on page 0 is a dead-block signal.
        // A 1-byte read at column 0 is essentially the same cost as reading
        // the whole page, because PAGEREAD + status-poll dominates.
        if (!nandReadBytesAt(page_0_row, 0, &main_byte, 1))
        {
            markBlockBad(b);
            ++n_new;
            continue;
        }

        // Option B — factory bad-block markers live at the first spare column (= runtime page size)
        // (first byte of the spare/OOB area) on pages 0 and 1 per the MT29F
        // datasheet.  A fresh good block reads 0xFF in both; any other value
        // means the manufacturer flagged it.  A read failure here also
        // counts as a suspect block.
        if (!nandReadBytesAt(page_0_row, geom_.page_size, &spare_byte[0], 1) ||
            !nandReadBytesAt(page_1_row, geom_.page_size, &spare_byte[1], 1))
        {
            markBlockBad(b);
            ++n_new;
            continue;
        }
        if (spare_byte[0] != 0xFF || spare_byte[1] != 0xFF)
        {
            markBlockBad(b);
            ++n_new;
        }
    }

    const uint32_t dt_ms = millis() - t0;
    if (cfg.debug)
    {
        ESP_LOGI(TAG, "Bad-block boot scan: %lu blocks scanned, %lu new bad, "
                      "%lu total bad, took %lu ms",
                      (unsigned long)n_scanned,
                      (unsigned long)n_new,
                      (unsigned long)countBadBlocks(),
                      (unsigned long)dt_ms);
    }
    return n_new;
}

void TR_LogToFlash::runStartupRecovery()
{
    const bool lfs_dirty  = checkDirtyOnStartup();   // LittleFS marker (non-sink)
    const bool mram_dirty = checkMramDirty();        // #274: MRAM marker (sink mode)

    if (!lfs_dirty && !mram_dirty)
    {
        clearRing();
        if (cfg.debug) ESP_LOGI(TAG, "Clean startup, no recovery needed.");
        return;
    }

    // Previous session was interrupted (a dirty marker is present).
    if (!use_mram_)
    {
        // RAM ring is volatile — nothing to recover.
        clearRing();
        clearDirty();
        if (cfg.debug) ESP_LOGW(TAG, "Dirty startup — RAM ring volatile, data lost.");
        return;
    }

    // #274: in sink mode the unflushed MRAM ring must be replayed THROUGH the
    // sink into a NAND flight — but the sink (TR_FlightLog) isn't initialized yet
    // at begin() time. Defer: preserve the ring + marker and let the OC drive
    // drainMramToSink() once flightlog.begin() has run.
    // #417: the surviving MRAM marker (set only once logging was activated) is
    // the boot-time recovery gate — one documented home shared with the tests.
    if (cfg.write_sink != nullptr &&
        MramDirtyPolicy::shouldRecoverOnBoot(mram_dirty))
    {
        pending_mram_recovery_ = true;
        if (cfg.debug)
            ESP_LOGW(TAG, "Dirty startup (sink) — MRAM ring preserved for deferred recovery.");
        return;   // do NOT clearRing / clearDirty here
    }

    // MRAM ring survived the reset — drain it into a recovery file.
    // rb_head/rb_tail/rb_count are in volatile RAM and lost, but the MRAM
    // contents are intact.  Read the entire MRAM into a recovery file.
    // The data may contain partial frames at boundaries, but the downstream
    // parser already handles that (length-prefixed frames with CRC).
    if (cfg.debug) ESP_LOGI(TAG, "Dirty startup — recovering MRAM ring (%lu bytes)...",
                                  (unsigned long)ring_size_);

    // Generate recovery filename
    struct lfs_info info;
    int idx = 1;
    do {
        snprintf(recovery_filename, sizeof(recovery_filename),
                 "/recovery_%03d.bin", idx++);
    } while (lfs_stat(&lfs, recovery_filename, &info) >= 0 && idx < 1000);

    lfs_file_t rf;
    int err = lfs_file_open(&lfs, &rf, recovery_filename,
                            LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL);
    if (err)
    {
        if (cfg.debug) ESP_LOGE(TAG, "Recovery file open failed: %d", err);
        clearRing();
        clearDirty();
        return;
    }

    // Read MRAM in page-sized chunks and write to LittleFS. Reuse the idle
    // page staging member rather than a 4 KB (F35SQB004G page) stack array so
    // this buffer plus the nested lfsBlockRead page buffer don't both land on
    // one task stack. Boot-only, single-threaded, staging is idle here.
    uint8_t* const chunk = page_buf;
    uint32_t offset = 0;
    uint32_t total_written = 0;
    while (offset < ring_size_)
    {
        uint32_t len = ring_size_ - offset;
        if (len > geom_.page_size) len = geom_.page_size;

        mramReadBytes(offset, chunk, len);
        lfs_ssize_t written = lfs_file_write(&lfs, &rf, chunk, len);
        if (written > 0) total_written += (uint32_t)written;
        offset += len;
    }

    lfs_file_sync(&lfs, &rf);
    lfs_file_close(&lfs, &rf);

    recovery_performed = true;
    recovery_bytes = total_written;

    if (cfg.debug) ESP_LOGI(TAG, "Recovered %lu bytes to %s",
                                  (unsigned long)total_written, recovery_filename);

    clearRing();
    clearDirty();
}

void TR_LogToFlash::flushRingToNand()
{
    if (!logging_active || !file_open)
    {
        return;
    }

    // When running on the flush task (Core 0), no rate limit needed —
    // this is a dedicated task that won't stall sensor reads on Core 1.
    // When running single-threaded (startup recovery), we drain everything too.

    // Hot-path target size: full NAND page for LFS, or page - 16 B for the
    // TR_FlightLog sink (leaves room for a PageHeader it will prepend).
    // The 16-byte figure must match sizeof(tr_flightlog::PageHeader); a
    // constant literal is used here to avoid pulling TR_FlightLog headers
    // into TR_LogToFlash and creating a dependency cycle.
    const uint32_t chunk_target = (cfg.write_sink != nullptr)
        ? sinkPayloadSize()
        : geom_.page_size;

    // Read current count
    portENTER_CRITICAL(&ring_mux_);
    uint32_t avail = rb_count;
    portEXIT_CRITICAL(&ring_mux_);

    // Drain RAM ring buffer to page staging buffer
    while (avail > 0)
    {
        const uint32_t need = chunk_target - page_buf_idx;
        const uint32_t chunk = (avail < need) ? avail : need;
        if (chunk == 0)
        {
            break;
        }

        // Issue #74 diagnostic: for the first 4 drains after activateLogging
        // (flush_log_remaining_ is armed to 4; the FL numbering starts at 16),
        // peek the first 8 bytes at rb_tail and log with pointer state.
        // `AA 55 AA 55 <type> <len>` = real frame; all-zero = post-clearRing
        // zeroed MRAM; anything else = stale prelaunch data being re-exposed.
        if (flush_log_remaining_ > 0)
        {
            uint8_t peek[8] = {0};
            if (chunk >= 8) ringPeekAt(rb_tail, peek, 8);
            ESP_LOGW(TAG, "FL%02lu h=%lu t=%lu c=%lu len=%lu peek=%02X%02X%02X%02X%02X%02X%02X%02X push=%llu pop=%llu",
                     (unsigned long)(20 - flush_log_remaining_),
                     (unsigned long)rb_head, (unsigned long)rb_tail,
                     (unsigned long)rb_count, (unsigned long)chunk,
                     peek[0], peek[1], peek[2], peek[3],
                     peek[4], peek[5], peek[6], peek[7],
                     (unsigned long long)ringpush_bytes_,
                     (unsigned long long)ringpop_bytes_);
            flush_log_remaining_--;
        }

        // #510: MRAM-pop wall time. Deliberately excludes the FL print above —
        // console cost must land in the ledger's drain-other bucket, not here.
        const int64_t _p0 = esp_timer_get_time();
        const uint32_t popped = ringPop(page_buf + page_buf_idx, chunk);
        {
            const uint32_t _pdt = (uint32_t)(esp_timer_get_time() - _p0);
            iter_ledger_.pop_us += _pdt;
            pop_us_win_ += _pdt;
        }
        if (popped == 0)
        {
            break;
        }

        page_buf_idx += popped;
        current_file_bytes += popped;

        // Full chunk ready — ship it either to the TR_FlightLog sink or to LFS.
        if (page_buf_idx == chunk_target)
        {
            bool ok = false;
            const int64_t _t0 = esp_timer_get_time();

            if (cfg.write_sink != nullptr)
            {
                ok = cfg.write_sink(cfg.write_sink_ctx, page_buf, chunk_target);
                const uint32_t _dt = (uint32_t)(esp_timer_get_time() - _t0);
                if (_dt > write_max_us_) write_max_us_ = _dt;
                iter_ledger_.write_us += _dt;   // #510
                write_us_win_ += _dt;
                if (_dt > (uint32_t)STALL_THRESHOLD_US)
                {
                    ESP_LOGW(TAG, "STALL: write_sink took %lu us", (unsigned long)_dt);
                }
            }
            else
            {
                // Legacy LFS path — unchanged from before Stage 2c-3c.
                const uint32_t cb_reads_before  = lfs_cb_reads_;
                const uint32_t cb_progs_before  = lfs_cb_progs_;
                const uint32_t cb_erases_before = lfs_cb_erases_;

                lfs_ssize_t written = lfs_file_write(&lfs, &file, page_buf, geom_.page_size);
                const uint32_t _dt = (uint32_t)(esp_timer_get_time() - _t0);
                if (_dt > write_max_us_) write_max_us_ = _dt;
                iter_ledger_.write_us += _dt;   // #510
                write_us_win_ += _dt;
                if (_dt > (uint32_t)STALL_THRESHOLD_US)
                {
                    ESP_LOGW(TAG, "STALL: lfs_file_write took %lu us "
                                  "(reads=%lu progs=%lu erases=%lu)",
                                  (unsigned long)_dt,
                                  (unsigned long)(lfs_cb_reads_  - cb_reads_before),
                                  (unsigned long)(lfs_cb_progs_  - cb_progs_before),
                                  (unsigned long)(lfs_cb_erases_ - cb_erases_before));
                }
                ok = (written == (lfs_ssize_t)geom_.page_size);
                if (!ok && cfg.debug) ESP_LOGE(TAG, "Write failed: %d", written);
            }

            if (!ok)
            {
                // #271: a persistent write failure (flight region full or a
                // bad-block run) must NOT wedge the drain.  The staged page was
                // already popped from the ring (:ringPop above) and can't be
                // recovered, so drop it and keep going: rewind current_file_bytes
                // (this page never reached NAND) and reset page_buf_idx so the
                // next flushRingToNand computes need>0 and keeps draining —
                // instead of need==0 -> chunk==0 -> break, forever, which dropped
                // the entire rest of the flight as ring overruns.
                nand_prog_fail++;
                current_file_bytes -= chunk_target;
                page_buf_idx = 0;
                return;
            }
            nand_bytes_written += chunk_target;
            nand_prog_ops++;
            page_buf_idx = 0;
            iter_ledger_.drain_pages++;              // #510
            iter_ledger_.drain_bytes += chunk_target;
            drain_pages_win_++;
            drain_bytes_win_ += chunk_target;

            // Periodic LFS sync — only meaningful when LFS is the destination.
            // TR_FlightLog pages are self-describing (PageHeader + CRC32), so
            // brownout recovery rebuilds the index scan-side; no sync needed.
            if (cfg.write_sink == nullptr &&
                ++pages_since_sync_ >= sync_interval_pages_)
            {
                {
                    LFS_TIMING_START();
                    lfs_file_sync(&lfs, &file);
                    LFS_TIMING_END(sync_max_us_, "lfs_file_sync");
                }
                syncs_performed_++;
                pages_since_sync_ = 0;
            }
        }

        // Re-read count (may have increased from Core 1 pushes)
        portENTER_CRITICAL(&ring_mux_);
        avail = rb_count;
        portEXIT_CRITICAL(&ring_mux_);
    }
}

// ============================================================================
// FreeRTOS Flush Task (runs on Core 0)
// ============================================================================

void TR_LogToFlash::flushTaskEntry(void* param)
{
    TR_LogToFlash* self = static_cast<TR_LogToFlash*>(param);
    self->flushTaskLoop();
    self->flush_task_running_ = false;
    vTaskDelete(nullptr);
}

void TR_LogToFlash::flushTaskLoop()
{
    while (!flush_task_stop_)
    {
        const int64_t iter_t0 = esp_timer_get_time();
        // #510: per-iteration section accounting — see flush_iter_ledger.h.
        iter_ledger_.reset();

        // Push any staged frames that have waited past the age bound, so
        // trickle-rate traffic still reaches the (brownout-durable) MRAM
        // ring promptly even when staging never fills.
        {
            const int64_t _s0 = esp_timer_get_time();
            flushStagingIfStale(STAGING_MAX_AGE_US);
            iter_ledger_.staging_us = (uint32_t)(esp_timer_get_time() - _s0);
        }

        // Handle deferred pre-create request (from PRELAUNCH state).
        // #365: consume the request ONLY after observing it set.  The old
        // unconditional else-clear could destroy a request that landed
        // between the condition load and the store — the pre-create would
        // silently vanish and openLogSession would instead run at launch
        // detect, right in the busiest window.
        if (prepare_file_requested_)
        {
            if (!file_open && !logging_active)
            {
                const int64_t _o0 = esp_timer_get_time();
                openLogSession();   // Creates file on NAND (slow, but we're not in a hurry yet)
                iter_ledger_.open_us += (uint32_t)(esp_timer_get_time() - _o0);
                // #417: pre-create must NOT dirty the marker — the session isn't
                // active yet. activateLogging() sets it at launch/manual-start.
                dirty_policy_.onPreCreate();
                if (cfg.debug) ESP_LOGI(TAG, "Log file pre-created for launch");
            }
            // else: file already open / logging — request is moot; consume it.
            prepare_file_requested_ = false;
        }

        // Periodic hook for deferred Core-0 work. Used by main.cpp to run
        // TR_FlightLog::servicePendingPrepareFlight on Core 0 (issue #77),
        // moving the prealloc-sized erase loop off the requesting task
        // (~120 ms at the current 40 blocks; this said "~770 ms 256-block",
        // the retired preallocation — #837 item 10).
        // file_open is already set above, so frames keep flowing into the
        // ring on Core 1 while this hook runs.
        if (cfg.flush_task_hook != nullptr)
        {
            const int64_t _h0 = esp_timer_get_time();
            cfg.flush_task_hook(cfg.flush_task_hook_ctx);
            iter_ledger_.hook_us = (uint32_t)(esp_timer_get_time() - _h0);
        }

        // Handle deferred start-logging request (launch detected).
        // #365: consume ONLY after observing the request.  The old blind
        // else-clear raced the Core-1 setter: a launch edge landing between
        // the condition load and the else-store was destroyed with no retry
        // — logging_active never set, and the whole flight was reduced to
        // the last ~63 KB MRAM remnant.  A duplicate request set while we
        // service this one is absorbed harmlessly (logging_active is already
        // true by the time we clear).
        if (start_logging_requested)
        {
            if (!logging_active)
            {
                const int64_t _a0 = esp_timer_get_time();
                if (!file_open)
                {
                    // File not pre-created — create now (legacy path)
                    openLogSession();
                }
                activateLogging();  // Fast — flips flags + markDirty (#417), no NAND I/O
                iter_ledger_.activate_us = (uint32_t)(esp_timer_get_time() - _a0);
            }
            // else: already logging — duplicate request; consume it.
            start_logging_requested = false;
        }

        // Defer timestamp rename to end-of-logging to avoid NAND stalls
        // during active recording.  The rename involves LittleFS directory
        // operations that can stall for 100-500ms, causing ring overflow.

        // Handle end-of-logging FIRST — check before flushing more data.
        // enqueueFrame() rejects new data when end_flight_requested is set,
        // so the drain loop will terminate quickly.
        if (end_flight_requested && logging_active && file_open)
        {
            const int64_t _e0 = esp_timer_get_time();
            uint32_t t0 = millis();

            // The last <=STAGING_SIZE of accepted frames may still sit in RAM
            // staging (enqueueFrame rejects new data once end_flight_requested
            // is set, so no more can arrive) — push them into the ring so the
            // drain below captures the true tail of the flight.
            if (push_mutex_) xSemaphoreTake(push_mutex_, portMAX_DELAY);
            flushStagingLocked();
            if (push_mutex_) xSemaphoreGive(push_mutex_);

            // Drain remaining data
            portENTER_CRITICAL(&ring_mux_);
            uint32_t remaining = rb_count;
            portEXIT_CRITICAL(&ring_mux_);

            ESP_LOGI("LOG", "Draining %lu bytes...", (unsigned long)remaining);

            while (remaining > 0)
            {
                flushRingToNand();
                vTaskDelay(1);  // feed WDT during drain
                portENTER_CRITICAL(&ring_mux_);
                remaining = rb_count;
                portEXIT_CRITICAL(&ring_mux_);
            }

            uint32_t t1 = millis();
            // Apply deferred timestamp rename now (after drain, before close).
            // Doing it here instead of during active logging avoids NAND stalls
            // that cause ring buffer overflow and frame drops.
            applyPendingTimestamp();
            closeLogSession();
            uint32_t t2 = millis();
            end_flight_requested = false;

            ESP_LOGI("LOG", "Stop: drain=%lums close=%lums total=%lums",
                     (unsigned long)(t1 - t0),
                     (unsigned long)(t2 - t1),
                     (unsigned long)(t2 - t0));
            iter_ledger_.endflight_us = (uint32_t)(esp_timer_get_time() - _e0);
        }

        // Normal flush: write ring buffer data to NAND.
        // With RAM ring buffer (no MRAM), there's no SPI contention on the
        // push path, so we can flush aggressively.
        if (logging_active && file_open)
        {
            portENTER_CRITICAL(&ring_mux_);
            const uint32_t avail = rb_count;
            portEXIT_CRITICAL(&ring_mux_);

            if (avail > 0)
            {
                const int64_t _d0 = esp_timer_get_time();
                flushRingToNand();
                iter_ledger_.drain_us = (uint32_t)(esp_timer_get_time() - _d0);
                vTaskDelay(1);  // 1ms yield — enough for WDT, no BLE contention
            }
        }

        // Iteration wall time — peaks indicate the flush loop itself
        // blocked for a long time (not just a single LFS op).
        // #510: classify against the section ledger. A long iteration the
        // sections explain (arm-time erase in hook, activation ring drain)
        // is designed work → INFO; only a genuine blind spot stays WARN,
        // and both name where the time went.
        {
            uint32_t iter_dt = (uint32_t)(esp_timer_get_time() - iter_t0);
            if (iter_dt > flush_iter_max_us_) flush_iter_max_us_ = iter_dt;
            const FlushIterLedger::Verdict v = FlushIterLedger::classify(
                iter_dt, iter_ledger_.accountedUs(), (uint32_t)STALL_THRESHOLD_US);
            if (v != FlushIterLedger::Verdict::Quiet)
            {
                const FlushIterLedger& L = iter_ledger_;
                if (v == FlushIterLedger::Verdict::LongUnaccounted)
                {
                    ESP_LOGW(TAG, "STALL: flush iter %lu us, unaccounted %lu us "
                                  "(staging=%lu open=%lu hook=%lu act=%lu drain=%lu "
                                  "[pg=%lu pop=%lu wr=%lu oth=%lu] end=%lu)",
                             (unsigned long)iter_dt,
                             (unsigned long)L.unaccountedUs(iter_dt),
                             (unsigned long)L.staging_us, (unsigned long)L.open_us,
                             (unsigned long)L.hook_us, (unsigned long)L.activate_us,
                             (unsigned long)L.drain_us, (unsigned long)L.drain_pages,
                             (unsigned long)L.pop_us, (unsigned long)L.write_us,
                             (unsigned long)L.drainOtherUs(), (unsigned long)L.endflight_us);
                }
                else
                {
                    ESP_LOGI(TAG, "Long flush iteration (accounted): %lu us "
                                  "(staging=%lu open=%lu hook=%lu act=%lu drain=%lu "
                                  "[pg=%lu pop=%lu wr=%lu oth=%lu] end=%lu unacc=%lu)",
                             (unsigned long)iter_dt,
                             (unsigned long)L.staging_us, (unsigned long)L.open_us,
                             (unsigned long)L.hook_us, (unsigned long)L.activate_us,
                             (unsigned long)L.drain_us, (unsigned long)L.drain_pages,
                             (unsigned long)L.pop_us, (unsigned long)L.write_us,
                             (unsigned long)L.drainOtherUs(), (unsigned long)L.endflight_us,
                             (unsigned long)L.unaccountedUs(iter_dt));
                }
            }
        }

        // Always yield at least 1 tick so the IDLE0 task can run and reset
        // the task watchdog.  1ms between flush iterations is still plenty —
        // at ~78 KB/s input rate the ring buffer only gains ~78 bytes per ms.
        vTaskDelay(1);
    }
}

void TR_LogToFlash::startFlushTask(uint8_t core, uint32_t stackSize, uint8_t priority)
{
    if (flush_task_ != nullptr)
    {
        return;  // Already started
    }

    flush_task_stop_ = false;
    flush_task_running_ = true;

    BaseType_t ret = xTaskCreatePinnedToCore(
        flushTaskEntry,
        "log_flush",
        stackSize,
        this,
        priority,
        &flush_task_,
        core
    );

    if (ret != pdPASS)
    {
        if (cfg.debug) ESP_LOGE(TAG, "Failed to create flush task");
        flush_task_running_ = false;
        flush_task_ = nullptr;
        return;
    }

    if (cfg.debug) ESP_LOGI(TAG, "Flush task started on core %d", core);
}

// END CORE FUNCTIONS

void TR_LogToFlash::setFileTimestamp(const char* filename, uint16_t year, uint8_t month, uint8_t day,
                                      uint8_t hour, uint8_t minute, uint8_t second)
{
    if (filename == nullptr || year == 0)
    {
        return;  // Invalid timestamp
    }

    // When flush task is running, defer LittleFS operations to Core 0
    // to avoid concurrent access to non-thread-safe LittleFS structures.
    if (flush_task_running_)
    {
        strncpy(pending_ts_filename_, filename, sizeof(pending_ts_filename_) - 1);
        pending_ts_filename_[sizeof(pending_ts_filename_) - 1] = '\0';
        pending_ts_year_ = year;
        pending_ts_month_ = month;
        pending_ts_day_ = day;
        pending_ts_hour_ = hour;
        pending_ts_minute_ = minute;
        pending_ts_second_ = second;
        __sync_synchronize();        // Ensure all fields are visible before flag
        pending_timestamp_ = true;   // Signal flush task (volatile write last)
        return;
    }

    // Single-threaded path (flush task not running)
    applyPendingTimestamp_impl(filename, year, month, day, hour, minute, second);
}

void TR_LogToFlash::applyPendingTimestamp()
{
    if (!pending_timestamp_) return;
    __sync_synchronize();  // Ensure timestamp fields are read after flag
    pending_timestamp_ = false;

    // Sink mode (issue #50): no LFS file exists for this flight, so
    // setattr/rename would hit a phantom filename. The sink owner
    // (TR_FlightLog in out_computer) synthesizes its own timestamped name on
    // finalize.
    if (cfg.write_sink != nullptr) return;

    applyPendingTimestamp_impl(pending_ts_filename_,
                               pending_ts_year_, pending_ts_month_, pending_ts_day_,
                               pending_ts_hour_, pending_ts_minute_, pending_ts_second_);
}

void TR_LogToFlash::applyPendingTimestamp_impl(const char* filename, uint16_t year, uint8_t month, uint8_t day,
                                                uint8_t hour, uint8_t minute, uint8_t second)
{
    // Build full path
    char path[64];
    snprintf(path, sizeof(path), "/%s", filename);

    // Store timestamp as custom attribute
    struct __attribute__((packed))
    {
        uint16_t year;
        uint8_t month, day, hour, minute, second;
    } ts = {year, month, day, hour, minute, second};

    int err = lfs_setattr((lfs_t*)&lfs, path, 'T', &ts, sizeof(ts));
    if (err < 0)
    {
        if (cfg.debug)
        {
            ESP_LOGE(TAG, "Failed to set timestamp: %d", err);
        }
        return;
    }

    // If this is the active file and hasn't been renamed yet, rename it with timestamp-based name
    if (file_open && !current_file_has_timestamp && strcmp(filename, current_filename + 1) == 0)  // +1 to skip leading '/'
    {
        char newpath[64];
        snprintf(newpath, sizeof(newpath),
                 "/flight_%04u%02u%02u_%02u%02u%02u.bin",
                 year, month, day, hour, minute, second);

        // Close file before renaming
        lfs_file_close((lfs_t*)&lfs, &file);

        // Rename
        err = lfs_rename((lfs_t*)&lfs, path, newpath);
        if (err == 0)
        {
            // Update current filename
            strncpy(current_filename, newpath, sizeof(current_filename) - 1);
            current_filename[sizeof(current_filename) - 1] = '\0';

            // Mark that this file has been timestamp-renamed
            current_file_has_timestamp = true;

            // Reopen with new name
            lfs_file_open((lfs_t*)&lfs, &file, newpath, LFS_O_WRONLY | LFS_O_APPEND);

            if (cfg.debug)
            {
                ESP_LOGI(TAG, "Renamed to %s", newpath + 1);  // +1 to skip leading '/'
            }
        }
        else
        {
            // Rename failed, reopen with old name
            lfs_file_open((lfs_t*)&lfs, &file, path, LFS_O_WRONLY | LFS_O_APPEND);

            if (cfg.debug)
            {
                ESP_LOGE(TAG, "Rename failed: %d", err);
            }
        }
    }
}

// ─── Raw NAND bridge (TR_FlightLog, issue #50 Stage 2) ───────────────────
// Forwarders that convert (block, page_in_block) -> rowPageAddr and always
// operate on a full runtime-page-size page. See TR_LogToFlash.h for rationale.

bool TR_LogToFlash::readNandPage(uint32_t block, uint32_t page_in_block, uint8_t* out)
{
    if (block >= geom_.block_count || page_in_block >= geom_.pages_per_blk) return false;
    uint32_t rowPageAddr = block * geom_.pages_per_blk + page_in_block;
    return nandReadPage(rowPageAddr, out, geom_.page_size);
}

bool TR_LogToFlash::programNandPage(uint32_t block, uint32_t page_in_block, const uint8_t* data)
{
    if (block >= geom_.block_count || page_in_block >= geom_.pages_per_blk) return false;
    uint32_t rowPageAddr = block * geom_.pages_per_blk + page_in_block;
    return nandProgramPage(rowPageAddr, data, geom_.page_size);
}

bool TR_LogToFlash::eraseNandBlock(uint32_t block)
{
    if (block >= geom_.block_count) return false;
    return nandEraseBlock(block);
}

bool TR_LogToFlash::isNandBlockBad(uint32_t block) const
{
    return isBlockBad(block);
}

bool TR_LogToFlash::markNandBlockBad(uint32_t block)
{
    if (block >= geom_.block_count) return false;
    markBlockBad(block);
    return true;
}
