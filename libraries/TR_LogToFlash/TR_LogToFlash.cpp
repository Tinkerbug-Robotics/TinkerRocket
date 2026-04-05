#include <TR_LogToFlash.h>
#include <CRC.h>
#include <cstring>
#include <cstdio>
#include <esp_heap_caps.h>
#include <esp_log.h>

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
                                      (unsigned long)ring_size_, (unsigned long)ESP.getFreeHeap());
    }
    ring_prelaunch_cap_ = ring_size_ / 2;

    rb_head = rb_tail = rb_count = 0;
    rb_overruns = rb_highwater = 0;
    rb_drop_oldest_bytes = 0;
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

    if (!nandInit())
    {
        return false;
    }

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
    lfs_cfg->block_count = NAND_BLOCK_COUNT;  // 1024
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
    // Stale MRAM data is no longer a concern: clearRing() now zeros
    // the MRAM, and processFrame() has a timestamp monotonicity filter.
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

// ============================================================================
// Ring buffer helpers (MRAM or RAM, depending on use_mram_)
// ============================================================================

bool TR_LogToFlash::ringPush(const uint8_t* data, uint32_t len)
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
        // flush task not touching tail yet).
        bool did_overrun = false;
        uint32_t local_count = count_now;
        while (ring_prelaunch_cap_ - local_count < len)
        {
            if (local_count < 6)
            {
                rb_drop_oldest_bytes += local_count;
                clearRing();
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
                clearRing();
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
                clearRing();
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

    if (new_count > rb_highwater)
    {
        rb_highwater = new_count;
    }
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

bool TR_LogToFlash::nandInit()
{
    nandSetFeature(FEAT_PROT, 0x00);

    spi->beginTransaction(spi_nand);
    csLow(cfg.nand_cs);
    spi->transfer(NAND_RDID);
    const uint8_t mid = spi->transfer(0x00);
    const uint8_t did = spi->transfer(0x00);
    csHigh(cfg.nand_cs);
    spi->endTransaction();

    if (cfg.debug)
    {
        ESP_LOGI(TAG, "RDID MID=0x%02X DID=0x%02X", mid, did);
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
            return LFS_ERR_IO;
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
            return LFS_ERR_IO;
        }

        bytes_written += NAND_PAGE_SIZE;
    }

    return LFS_ERR_OK;
}

int TR_LogToFlash::lfsBlockErase(const struct lfs_config *c, lfs_block_t block)
{
    TR_LogToFlash* self = (TR_LogToFlash*)c->context;

    // Yield before erase — this is the slowest NAND operation (~2ms per block)
    vTaskDelay(1);

    if (!self->nandEraseBlock(block))
    {
        return LFS_ERR_IO;
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
    int err = lfs_file_open(&lfs, &file, current_filename,
                            LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL);
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
    // Clear stale pre-launch data from the ring buffer so only
    // fresh data from this moment forward gets logged.
    clearRing();

    logging_active = true;
    ring_prelaunch_cap_ = ring_size_;
    end_flight_requested = false;
    if (cfg.debug) ESP_LOGI(TAG, "Logging activated (ring cleared + cap raised)");
}

void TR_LogToFlash::closeLogSession()
{
    if (!file_open) return;

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
    lfs_file_sync(&lfs, &file);
    lfs_file_close(&lfs, &file);

    file_open = false;
    logging_active = false;
    clearDirty();

    // Restore pre-launch ring cap so the buffer doesn't fill completely
    // before the next launch — leaves headroom for the initial flush.
    ring_prelaunch_cap_ = ring_size_ / 2;

    if (cfg.debug)
    {
        ESP_LOGI(TAG, "Closed %s (%lu bytes)",
                      current_filename, current_file_bytes);
    }

    current_file_bytes = 0;
}

// ============================================================================
// Dirty flag persistence (LittleFS marker file, replaces MRAM persist state)
// ============================================================================

void TR_LogToFlash::markDirty()
{
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
    lfs_remove(&lfs, "/.dirty");
}

bool TR_LogToFlash::checkDirtyOnStartup()
{
    struct lfs_info info;
    return (lfs_stat(&lfs, "/.dirty", &info) >= 0);
}

void TR_LogToFlash::clearRing()
{
    portENTER_CRITICAL(&ring_mux_);
    rb_head = 0;
    rb_tail = 0;
    rb_count = 0;
    portEXIT_CRITICAL(&ring_mux_);

    // When using MRAM, zero-fill the entire ring to prevent stale data from
    // a previous session from leaking into the log file.  MRAM is non-volatile,
    // so clearRing() resetting pointers alone is not sufficient — the flush task
    // could read stale bytes if rb_count becomes inconsistent due to any race.
    // At 40 MHz SPI, zeroing 128 KB takes ~26 ms — acceptable at launch time.
    if (use_mram_)
    {
        static constexpr size_t ZERO_CHUNK = 256;
        uint8_t zeros[ZERO_CHUNK] = {};
        for (uint32_t addr = 0; addr < ring_size_; addr += ZERO_CHUNK)
        {
            uint32_t len = (ring_size_ - addr < ZERO_CHUNK)
                         ? (ring_size_ - addr) : ZERO_CHUNK;
            mramWriteBytes(addr, zeros, len);
        }
    }
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

    // Read current count
    portENTER_CRITICAL(&ring_mux_);
    uint32_t avail = rb_count;
    portEXIT_CRITICAL(&ring_mux_);

    // Drain RAM ring buffer to page staging buffer
    while (avail > 0)
    {
        const uint32_t need = NAND_PAGE_SIZE - page_buf_idx;
        const uint32_t chunk = (avail < need) ? avail : need;
        if (chunk == 0)
        {
            break;
        }

        const uint32_t popped = ringPop(page_buf + page_buf_idx, chunk);
        if (popped == 0)
        {
            break;
        }

        page_buf_idx += popped;
        current_file_bytes += popped;

        // Write full page to LittleFS
        if (page_buf_idx == NAND_PAGE_SIZE)
        {
            lfs_ssize_t written = lfs_file_write(&lfs, &file, page_buf, NAND_PAGE_SIZE);
            if (written != NAND_PAGE_SIZE)
            {
                if (cfg.debug) ESP_LOGE(TAG, "Write failed: %d", written);
                nand_prog_fail++;
                return;
            }
            nand_bytes_written += NAND_PAGE_SIZE;
            nand_prog_ops++;
            page_buf_idx = 0;

            // Periodic sync: commit LittleFS metadata to NAND so that a hard
            // reset loses at most SYNC_INTERVAL_PAGES worth of data (~256 KB).
            // The ring buffer absorbs incoming frames during the sync stall.
            if (++pages_since_sync_ >= SYNC_INTERVAL_PAGES)
            {
                lfs_file_sync(&lfs, &file);
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
