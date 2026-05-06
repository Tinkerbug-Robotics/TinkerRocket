#include <TR_LogToFlash.h>
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

    // Initialize ring buffer — MRAM (SPI) if configured, else heap RAM
    if (cfg.mram_cs >= 0)
    {
        use_mram_ = true;
        ring_size_ = cfg.mram_size;
        spi_mram = SPISettings(cfg.spi_hz_mram, MSBFIRST, cfg.spi_mode_mram);

        pinMode(cfg.mram_cs, OUTPUT);
        digitalWrite(cfg.mram_cs, HIGH);

        // Create SPI bus mutex (both cores share the NAND/MRAM SPI bus)
        spi_mutex_ = xSemaphoreCreateMutex();
        if (!spi_mutex_)
        {
            if (cfg.debug) ESP_LOGE(TAG, "Failed to create SPI mutex");
            return false;
        }

        if (cfg.debug) ESP_LOGI(TAG, "MRAM ring: %lu bytes on CS pin %d",
                                      (unsigned long)ring_size_, cfg.mram_cs);
    }
    else
    {
        use_mram_ = false;
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
    ring_prelaunch_cap_ = ring_size_ / 2;

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

    // Boot-time non-destructive bad-block scan (#47): walks every not-yet-
    // known-bad block and probes for read errors + factory bad markers.
    // Findings get markBlockBad'd; we persist right away so a reboot during
    // the scan doesn't lose the discoveries.
    scanBadBlocksAtBoot();
    persistBadBlocksIfDirty();

    // Allocate LittleFS buffers on heap to avoid stack overflow
    lfs_read_buffer = (uint8_t*)malloc(NAND_PAGE_SIZE);
    lfs_prog_buffer = (uint8_t*)malloc(NAND_PAGE_SIZE);
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

    lfs_cfg->read_size = NAND_PAGE_SIZE;      // 2048
    lfs_cfg->prog_size = NAND_PAGE_SIZE;      // 2048
    lfs_cfg->block_size = NAND_BLOCK_SIZE;    // 128KB
    // Default to the full chip; caller may shrink (issue #50 Stage 2c) so the
    // trailing blocks can be owned by TR_FlightLog.
    lfs_cfg->block_count = (cfg.lfs_block_count > 0) ? cfg.lfs_block_count
                                                     : NAND_BLOCK_COUNT;
    lfs_cfg->cache_size = NAND_PAGE_SIZE;     // 2048
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
    // (capped at 50%) and are flushed once activateLogging() fires.
    // Cross-session stale MRAM is handled by runStartupRecovery() at boot;
    // within a session, processFrame() has a timestamp monotonicity filter
    // that catches anything that would look like a replay from the ring.
    if (!logging_active && !file_open)
    {
        return false;
    }
    frames_received++;
    bytes_received += len;
    if (!ringPush(frame, static_cast<uint32_t>(len)))
    {
        frames_dropped++;
        return false;
    }
    return true;
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
    // The flush task loop handles openLogSession, flushRingToNand, closeLogSession.
    if (flush_task_running_)
    {
        return;
    }

    // Single-threaded fallback (flush task not started yet — startup/recovery)
    if (start_logging_requested && !logging_active)
    {
        if (!file_open)
        {
            openLogSession();
            markDirty();
        }
        activateLogging();
    }
    start_logging_requested = false;
    flushRingToNand();
}

void TR_LogToFlash::getStats(TR_LogToFlashStats& out) const
{
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
// SPI bus mutex (active when MRAM is enabled)
// ============================================================================

void TR_LogToFlash::spiAcquire()
{
    if (spi_mutex_) xSemaphoreTake(spi_mutex_, portMAX_DELAY);
}

void TR_LogToFlash::spiRelease()
{
    if (spi_mutex_) xSemaphoreGive(spi_mutex_);
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
    if (len == 0 || len > ring_size_)
    {
        return false;
    }

    // Serialize against concurrent pushes (parser and oc_loop can preempt
    // each other on Core 1) and against clearRing (Core 0 flush task). This
    // closes the #74 race where a push snapshotted rb_head before clearRing
    // ran and then wrote a stale rb_head value, clobbering the reset.
    if (push_mutex_) xSemaphoreTake(push_mutex_, portMAX_DELAY);

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
            if (push_mutex_) xSemaphoreGive(push_mutex_);
            return false;
        }
    }
    else
    {
        // Pre-launch: drop oldest frames from tail to make room (single-threaded,
        // flush task not touching tail yet).
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

            uint8_t hdr[6];
            ringPeekAt(rb_tail, hdr, 6);

            if (hdr[0] != 0xAA || hdr[1] != 0x55 || hdr[2] != 0xAA || hdr[3] != 0x55)
            {
                if (cfg.debug)
                {
                    ESP_LOGW(TAG, "ringPush: bad SOF at tail, clearing ring");
                }
                rb_drop_oldest_bytes += local_count;
                rb_bad_sof_clears++;
                clearRingLocked();
                local_count = 0;
                break;
            }

            const uint32_t payload_len = hdr[5];
            const uint32_t frame_size = 4 + 1 + 1 + payload_len + 2;

            if (frame_size > local_count)
            {
                if (cfg.debug)
                {
                    ESP_LOGW(TAG, "ringPush: partial frame at tail (%lu > %lu), clearing ring",
                                  (unsigned long)frame_size, (unsigned long)local_count);
                }
                rb_drop_oldest_bytes += local_count;
                rb_bad_sof_clears++;
                clearRingLocked();
                local_count = 0;
                break;
            }

            rb_tail = (rb_tail + frame_size) % ring_size_;
            local_count -= frame_size;

            portENTER_CRITICAL(&ring_mux_);
            rb_count = local_count;
            portEXIT_CRITICAL(&ring_mux_);

            rb_drop_oldest_bytes += frame_size;
            did_overrun = true;
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
    if (push_mutex_) xSemaphoreGive(push_mutex_);
    return true;
}

uint32_t TR_LogToFlash::ringPop(uint8_t* out, uint32_t len)
{
    // Read count under spinlock
    portENTER_CRITICAL(&ring_mux_);
    const uint32_t count_now = rb_count;
    portEXIT_CRITICAL(&ring_mux_);

    if (len == 0 || len > count_now)
    {
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
    const uint32_t row = blockIndex * NAND_PAGES_PER_BLK;

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
    if (len != NAND_PAGE_SIZE)
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
    if (len > NAND_PAGE_SIZE)
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
        uint32_t page_in_block = block_offset / NAND_PAGE_SIZE;
        uint32_t offset_in_page = block_offset % NAND_PAGE_SIZE;

        uint32_t row_page = (block * NAND_PAGES_PER_BLK) + page_in_block;

        // Read full page into temp buffer
        uint8_t page_buf[NAND_PAGE_SIZE];
        if (!self->nandReadPage(row_page, page_buf, NAND_PAGE_SIZE))
        {
            // Read failure — the block is suspect.  Tell LFS it's bad so it
            // relocates, and remember it so we skip on future mounts.
            self->markBlockBad(block);
            return LFS_ERR_CORRUPT;
        }

        // Copy requested portion
        uint32_t bytes_to_copy = (size - bytes_read < NAND_PAGE_SIZE - offset_in_page)
                                 ? (size - bytes_read)
                                 : (NAND_PAGE_SIZE - offset_in_page);
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
    if (off % NAND_PAGE_SIZE != 0 || size % NAND_PAGE_SIZE != 0)
    {
        return LFS_ERR_INVAL;
    }

    const uint8_t* buf = (const uint8_t*)buffer;
    lfs_size_t bytes_written = 0;

    while (bytes_written < size)
    {
        uint32_t block_offset = off + bytes_written;
        uint32_t page_in_block = block_offset / NAND_PAGE_SIZE;
        uint32_t row_page = (block * NAND_PAGES_PER_BLK) + page_in_block;

        // Write full page directly (no read-modify-write)
        if (!self->nandProgramPage(row_page, buf + bytes_written, NAND_PAGE_SIZE))
        {
            // Program failed (STAT_PFAIL or timeout).  Mark the block bad
            // and tell LFS to relocate — CORRUPT (not IO) triggers the
            // remap path rather than propagating a hard FS error.
            self->markBlockBad(block);
            return LFS_ERR_CORRUPT;
        }

        bytes_written += NAND_PAGE_SIZE;
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
    // Arm per-drain diagnostic logs for the first 20 flushRingToNand drains.
    flush_log_remaining_ = 20;

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
            // in a PageHeader and programs a full 2048-byte page, zero-
            // padding the unused payload tail. Accepted small loss.
            (void)cfg.write_sink(cfg.write_sink_ctx, page_buf, page_buf_idx);
            page_buf_idx = 0;
        }
        file_open = false;
        logging_active = false;
        clearDirty();
        persistBadBlocksIfDirty();
        ring_prelaunch_cap_ = ring_size_ / 2;
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
    clearDirty();

    // Flush any new bad-block discoveries to NVS now — no longer in the
    // hot path, and the next session should see the same list.
    persistBadBlocksIfDirty();

    // Restore pre-launch ring cap so the buffer doesn't fill completely
    // before the next launch — leaves headroom for the initial flush.
    ring_prelaunch_cap_ = ring_size_ / 2;

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
    if (cfg.write_sink != nullptr) return;

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
    if (cfg.write_sink != nullptr) return;

    lfs_remove(&lfs, "/.dirty");
}

bool TR_LogToFlash::checkDirtyOnStartup()
{
    struct lfs_info info;
    return (lfs_stat(&lfs, "/.dirty", &info) >= 0);
}

void TR_LogToFlash::clearRing()
{
    // Public entry point: acquire push_mutex_ so any in-flight ringPush on
    // Core 1 finishes before we start, and no new push can start until we
    // return. Without this guard, a push that already snapshotted rb_head
    // would clobber our reset back to a prelaunch value on its trailing
    // rb_head assignment (the #74 race).
    if (push_mutex_) xSemaphoreTake(push_mutex_, portMAX_DELAY);
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
    if (block >= NAND_BLOCK_COUNT) return true;  // treat OOB as bad
    return (bad_block_bitmap_[block / 8] & (uint8_t)(1u << (block % 8))) != 0;
}

void TR_LogToFlash::markBlockBad(uint32_t block)
{
    if (block >= NAND_BLOCK_COUNT) return;
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
    for (uint32_t i = 0; i < BAD_BLOCK_BITMAP_BYTES; ++i)
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
        bad_block_chip_id_ = 0;
        bad_block_bitmap_dirty_ = false;
        if (cfg.debug) ESP_LOGI(TAG, "Bad-block NVS namespace not found, starting clean");
        return;
    }
    const size_t got = prefs.getBytes("map", bad_block_bitmap_, sizeof(bad_block_bitmap_));
    bad_block_chip_id_ = prefs.getUShort("chip", 0);
    prefs.end();
    if (got != sizeof(bad_block_bitmap_))
    {
        memset(bad_block_bitmap_, 0, sizeof(bad_block_bitmap_));
    }
    bad_block_bitmap_dirty_ = false;
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
    prefs.putBytes("map", bad_block_bitmap_, sizeof(bad_block_bitmap_));
    prefs.putUShort("chip", bad_block_chip_id_);
    prefs.end();
    bad_block_bitmap_dirty_ = false;
    if (cfg.debug) ESP_LOGI(TAG, "Persisted bad-block map (%lu bad, chip=0x%04X)",
                                  (unsigned long)countBadBlocks(),
                                  (unsigned)bad_block_chip_id_);
}

uint32_t TR_LogToFlash::scanBadBlocksAtBoot()
{
    const uint32_t t0 = millis();
    uint32_t n_new = 0;
    uint32_t n_scanned = 0;
    uint8_t main_byte = 0xFF;
    uint8_t spare_byte[2] = { 0xFF, 0xFF };

    for (uint32_t b = 0; b < NAND_BLOCK_COUNT; ++b)
    {
        // Already-known bad blocks from NVS or this run — skip, no NAND work.
        if (isBlockBad(b)) continue;
        ++n_scanned;

        const uint32_t page_0_row = b * NAND_PAGES_PER_BLK;
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

        // Option B — factory bad-block markers live at column NAND_PAGE_SIZE
        // (first byte of the spare/OOB area) on pages 0 and 1 per the MT29F
        // datasheet.  A fresh good block reads 0xFF in both; any other value
        // means the manufacturer flagged it.  A read failure here also
        // counts as a suspect block.
        if (!nandReadBytesAt(page_0_row, NAND_PAGE_SIZE, &spare_byte[0], 1) ||
            !nandReadBytesAt(page_1_row, NAND_PAGE_SIZE, &spare_byte[1], 1))
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
    if (!checkDirtyOnStartup())
    {
        clearRing();
        if (cfg.debug) ESP_LOGI(TAG, "Clean startup, no recovery needed.");
        return;
    }

    // Previous session was interrupted (dirty flag present).
    if (!use_mram_)
    {
        // RAM ring is volatile — nothing to recover.
        clearRing();
        clearDirty();
        if (cfg.debug) ESP_LOGW(TAG, "Dirty startup — RAM ring volatile, data lost.");
        return;
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

    // Read MRAM in page-sized chunks and write to LittleFS
    uint8_t chunk[NAND_PAGE_SIZE];
    uint32_t offset = 0;
    uint32_t total_written = 0;
    while (offset < ring_size_)
    {
        uint32_t len = ring_size_ - offset;
        if (len > NAND_PAGE_SIZE) len = NAND_PAGE_SIZE;

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
        ? (NAND_PAGE_SIZE - 16u)
        : NAND_PAGE_SIZE;

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

        // Issue #74 diagnostic: for the first 20 drains after activateLogging,
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

        const uint32_t popped = ringPop(page_buf + page_buf_idx, chunk);
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

                lfs_ssize_t written = lfs_file_write(&lfs, &file, page_buf, NAND_PAGE_SIZE);
                const uint32_t _dt = (uint32_t)(esp_timer_get_time() - _t0);
                if (_dt > write_max_us_) write_max_us_ = _dt;
                if (_dt > (uint32_t)STALL_THRESHOLD_US)
                {
                    ESP_LOGW(TAG, "STALL: lfs_file_write took %lu us "
                                  "(reads=%lu progs=%lu erases=%lu)",
                                  (unsigned long)_dt,
                                  (unsigned long)(lfs_cb_reads_  - cb_reads_before),
                                  (unsigned long)(lfs_cb_progs_  - cb_progs_before),
                                  (unsigned long)(lfs_cb_erases_ - cb_erases_before));
                }
                ok = (written == NAND_PAGE_SIZE);
                if (!ok && cfg.debug) ESP_LOGE(TAG, "Write failed: %d", written);
            }

            if (!ok)
            {
                nand_prog_fail++;
                return;
            }
            nand_bytes_written += chunk_target;
            nand_prog_ops++;
            page_buf_idx = 0;

            // Periodic LFS sync — only meaningful when LFS is the destination.
            // TR_FlightLog pages are self-describing (PageHeader + CRC32), so
            // brownout recovery rebuilds the index scan-side; no sync needed.
            if (cfg.write_sink == nullptr &&
                ++pages_since_sync_ >= SYNC_INTERVAL_PAGES)
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

        // Handle deferred pre-create request (from PRELAUNCH state)
        if (prepare_file_requested_ && !file_open && !logging_active)
        {
            openLogSession();   // Creates file on NAND (slow, but we're not in a hurry yet)
            markDirty();
            prepare_file_requested_ = false;
            if (cfg.debug) ESP_LOGI(TAG, "Log file pre-created for launch");
        }
        else
        {
            prepare_file_requested_ = false;
        }

        // Periodic hook for deferred Core-0 work. Used by main.cpp to run
        // TR_FlightLog::servicePendingPrepareFlight on Core 0 (issue #77),
        // moving the ~770 ms 256-block erase loop off the requesting task.
        // file_open is already set above, so frames keep flowing into the
        // ring on Core 1 while this hook runs.
        if (cfg.flush_task_hook != nullptr)
        {
            cfg.flush_task_hook(cfg.flush_task_hook_ctx);
        }

        // Handle deferred start-logging request (launch detected)
        if (start_logging_requested && !logging_active)
        {
            if (!file_open)
            {
                // File not pre-created — create now (legacy path)
                openLogSession();
                markDirty();
            }
            activateLogging();  // Fast — just flips flags, no NAND I/O
            start_logging_requested = false;
        }
        else
        {
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
            uint32_t t0 = millis();

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
                flushRingToNand();
                vTaskDelay(1);  // 1ms yield — enough for WDT, no BLE contention
            }
        }

        // Iteration wall time — peaks indicate the flush loop itself
        // blocked for a long time (not just a single LFS op).
        {
            uint32_t iter_dt = (uint32_t)(esp_timer_get_time() - iter_t0);
            if (iter_dt > flush_iter_max_us_) flush_iter_max_us_ = iter_dt;
            if (iter_dt > (uint32_t)STALL_THRESHOLD_US) {
                ESP_LOGW(TAG, "STALL: flushTaskLoop iteration took %lu us",
                         (unsigned long)iter_dt);
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
// operate on a full NAND_PAGE_SIZE page. See TR_LogToFlash.h for rationale.

bool TR_LogToFlash::readNandPage(uint32_t block, uint32_t page_in_block, uint8_t* out)
{
    if (block >= NAND_BLOCK_COUNT || page_in_block >= NAND_PAGES_PER_BLK) return false;
    uint32_t rowPageAddr = block * NAND_PAGES_PER_BLK + page_in_block;
    return nandReadPage(rowPageAddr, out, NAND_PAGE_SIZE);
}

bool TR_LogToFlash::programNandPage(uint32_t block, uint32_t page_in_block, const uint8_t* data)
{
    if (block >= NAND_BLOCK_COUNT || page_in_block >= NAND_PAGES_PER_BLK) return false;
    uint32_t rowPageAddr = block * NAND_PAGES_PER_BLK + page_in_block;
    return nandProgramPage(rowPageAddr, data, NAND_PAGE_SIZE);
}

bool TR_LogToFlash::eraseNandBlock(uint32_t block)
{
    if (block >= NAND_BLOCK_COUNT) return false;
    return nandEraseBlock(block);
}

bool TR_LogToFlash::isNandBlockBad(uint32_t block) const
{
    return isBlockBad(block);
}

bool TR_LogToFlash::markNandBlockBad(uint32_t block)
{
    if (block >= NAND_BLOCK_COUNT) return false;
    markBlockBad(block);
    return true;
}
