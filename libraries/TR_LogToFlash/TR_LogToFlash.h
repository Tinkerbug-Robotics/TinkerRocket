#ifndef TR_LOG_TO_FLASH_H
#define TR_LOG_TO_FLASH_H

#include <Arduino.h>
#include <SPI.h>
#include <RocketComputerTypes.h>
#include "lfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

struct TR_LogToFlashConfig
{
    int nand_cs = -1;
    uint32_t spi_hz_nand = 40'000'000;
    uint8_t spi_mode_nand = SPI_MODE0;
    uint32_t ring_buffer_size = 65536;  // RAM ring buffer size (bytes)
    bool debug = false;

    // MRAM ring buffer (optional — set mram_cs >= 0 to enable).
    // When enabled the ring buffer lives in SPI MRAM instead of ESP32 RAM,
    // providing larger capacity (128 KB) and power-loss survivability.
    int mram_cs = -1;                   // chip-select pin (-1 = use RAM ring)
    uint32_t spi_hz_mram = 40'000'000;
    uint8_t spi_mode_mram = SPI_MODE0;
    uint32_t mram_size = 131072;        // 128 KB (MR25H10)
};

struct TR_LogToFlashStats
{
    uint32_t ring_fill = 0;
    uint32_t ring_highwater = 0;
    uint32_t ring_overruns = 0;
    uint32_t ring_drop_oldest_bytes = 0;
    uint64_t bytes_received = 0;
    uint32_t frames_received = 0;
    uint32_t frames_dropped = 0;
    uint64_t bytes_written_nand = 0;
    uint32_t nand_prog_fail = 0;
    uint32_t nand_erase_fail = 0;
    uint32_t nand_prog_ops = 0;
    uint32_t nand_erase_ops = 0;
    bool logging_active = false;
    uint32_t nand_page = 0;
    uint32_t nand_block = 0;
};

struct TR_LogToFlashRecoveryInfo
{
    bool recovered = false;
    uint32_t recovered_bytes = 0;
    char filename[64] = {};
};

struct TR_LogFileInfo
{
    char filename[64] = {};
    uint32_t size_bytes = 0;
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    bool has_timestamp = false;  // Flag to indicate if timestamp is valid
};

class TR_LogToFlash
{
public:
    TR_LogToFlash();

    bool begin(SPIClass& spi, const TR_LogToFlashConfig& cfg);
    bool enqueueFrame(const uint8_t* frame, size_t len);
    void startLogging();
    void endLogging(); // request close after buffered data is flushed
    void prepareLogFile();  // Pre-create log file (call during PRELAUNCH to avoid NAND stall at launch)
    void service();
    void startFlushTask(uint8_t core = 0, uint32_t stackSize = 8192, uint8_t priority = 1);

    void getStats(TR_LogToFlashStats& out) const;
    void getRecoveryInfo(TR_LogToFlashRecoveryInfo& out) const;
    size_t listFiles(TR_LogFileInfo* out, size_t max_files) const;
    bool readFileChunk(const char* filename,
                       uint32_t offset,
                       uint8_t* out,
                       size_t max_len,
                       size_t& out_len,
                       bool& eof);
    bool deleteFile(const char* filename);
    bool formatFilesystem();  // WARNING: Erases all files!
    const char* currentFilename() const;
    bool isLoggingActive() const;
    void setFileTimestamp(const char* filename, uint16_t year, uint8_t month, uint8_t day,
                          uint8_t hour, uint8_t minute, uint8_t second);

private:
    // --- NAND ---
    static constexpr uint8_t NAND_RDID = 0x9F;
    static constexpr uint8_t NAND_WREN = 0x06;
    static constexpr uint8_t NAND_GETFEAT = 0x0F;
    static constexpr uint8_t NAND_SETFEAT = 0x1F;
    static constexpr uint8_t NAND_BLKERASE = 0xD8;
    static constexpr uint8_t NAND_PROGLOAD = 0x02;
    static constexpr uint8_t NAND_PROGEXEC = 0x10;
    static constexpr uint8_t NAND_PAGEREAD = 0x13;
    static constexpr uint8_t NAND_READCACHE = 0x03;

    static constexpr uint8_t FEAT_STAT = 0xC0;
    static constexpr uint8_t FEAT_PROT = 0xA0;
    static constexpr uint8_t STAT_OIP = 0x01;
    static constexpr uint8_t STAT_PFAIL = 0x08;
    static constexpr uint8_t STAT_EFAIL = 0x04;

    static constexpr uint32_t NAND_PAGE_SIZE = 2048;
    static constexpr uint32_t NAND_PAGES_PER_BLK = 64;
    static constexpr uint32_t NAND_BLOCK_SIZE = NAND_PAGE_SIZE * NAND_PAGES_PER_BLK;  // 128KB
    static constexpr uint32_t NAND_BLOCK_COUNT = 1024;

    struct __attribute__((packed)) LogMeta
    {
        uint32_t magic;
        uint16_t next_file_index;
        uint16_t reserved;
        uint16_t crc16;
    };
    static constexpr uint32_t LOG_META_MAGIC = 0x4C4F4746; // "F G O L"
    static constexpr uint16_t LOG_META_VERSION = 2;  // Version 2 adds timestamps
    static constexpr uint32_t LOG_META_PAGE = 0;
    static constexpr uint32_t LOG_META_BLOCK = 0;

    // SPI/context
    SPIClass* spi = nullptr;
    TR_LogToFlashConfig cfg = {};
    SPISettings spi_nand;

    // Ring buffer storage (RAM fallback or MRAM via SPI)
    bool use_mram_ = false;
    uint8_t* ring_buf_ = nullptr;       // Used only when use_mram_ == false
    uint32_t ring_size_ = 0;
    uint32_t rb_head = 0;
    uint32_t rb_tail = 0;
    uint32_t rb_count = 0;
    uint32_t rb_overruns = 0;
    uint32_t rb_drop_oldest_bytes = 0;
    uint32_t rb_highwater = 0;
    // Before logging starts, cap the ring at 50% so the initial flush at
    // launch detection doesn't stall the main loop.  Raised to full size
    // once logging is active (see openLogSession / closeLogSession).
    uint32_t ring_prelaunch_cap_ = 0;  // Set in begin() from ring_size_

    // NAND/log state
    uint32_t nand_page = 0;
    uint32_t nand_block = 0;
    uint64_t nand_bytes_written = 0;
    uint32_t nand_prog_fail = 0;
    uint32_t nand_erase_fail = 0;
    uint32_t nand_prog_ops = 0;
    uint32_t nand_erase_ops = 0;
    uint16_t file_index = 0;
    char filename[64] = {};
    bool logging_active = false;
    bool start_logging_requested = false;
    bool end_flight_requested = false;
    uint32_t log_start_block = 1;
    uint32_t log_curr_block = 1;
    bool log_block_erased = false;

    // page staging
    uint8_t page_buf[NAND_PAGE_SIZE];
    uint32_t page_buf_idx = 0;

    // counters
    uint64_t bytes_received = 0;
    uint32_t frames_received = 0;
    uint32_t frames_dropped = 0;

    // startup recovery
    bool recovery_performed = false;
    uint32_t recovery_bytes = 0;
    char recovery_filename[64] = {};

    // LittleFS filesystem
    lfs_t lfs;
    lfs_file_t file;
    bool file_open = false;

    // LittleFS config and buffers (heap allocated to avoid stack overflow)
    lfs_config* lfs_cfg = nullptr;
    uint8_t* lfs_read_buffer = nullptr;
    uint8_t* lfs_prog_buffer = nullptr;
    uint8_t* lfs_lookahead_buffer = nullptr;

    // Current file tracking
    char current_filename[64] = {};
    uint32_t current_file_bytes = 0;
    bool current_file_has_timestamp = false;

    // Deferred timestamp (set from Core 1, applied by flush task on Core 0)
    volatile bool pending_timestamp_ = false;
    uint16_t pending_ts_year_ = 0;
    uint8_t pending_ts_month_ = 0;
    uint8_t pending_ts_day_ = 0;
    uint8_t pending_ts_hour_ = 0;
    uint8_t pending_ts_minute_ = 0;
    uint8_t pending_ts_second_ = 0;
    char pending_ts_filename_[64] = {};
    void applyPendingTimestamp();  // Called by flush task
    void applyPendingTimestamp_impl(const char* filename, uint16_t year, uint8_t month, uint8_t day,
                                     uint8_t hour, uint8_t minute, uint8_t second);

    // Pre-create log file support
    volatile bool prepare_file_requested_ = false;

    // SPI bus mutex (active when MRAM is enabled — both cores share SPI bus)
    SemaphoreHandle_t spi_mutex_ = nullptr;
    void spiAcquire();
    void spiRelease();

    // MRAM SPI interface (MR25H10)
    SPISettings spi_mram;
    static constexpr uint8_t MRAM_WREN  = 0x06;
    static constexpr uint8_t MRAM_READ  = 0x03;
    static constexpr uint8_t MRAM_WRITE = 0x02;
    void mramWriteBytes(uint32_t addr, const uint8_t* data, uint32_t len);
    void mramReadBytes(uint32_t addr, uint8_t* out, uint32_t len);

    // FreeRTOS flush task (Core 0)
    portMUX_TYPE ring_mux_ = portMUX_INITIALIZER_UNLOCKED;  // Spinlock for rb_head/rb_tail/rb_count
    TaskHandle_t flush_task_ = nullptr;
    volatile bool flush_task_running_ = false;
    volatile bool flush_task_stop_ = false;     // Signal flush task to exit

    static void flushTaskEntry(void* param);    // FreeRTOS task entry (static)
    void flushTaskLoop();                        // Instance method called by entry

    // helpers
    inline void csLow(int pin) { digitalWrite(pin, LOW); }
    inline void csHigh(int pin) { digitalWrite(pin, HIGH); }

    bool ringPush(const uint8_t* data, uint32_t len);
    uint32_t ringPop(uint8_t* out, uint32_t len);
    void ringPeekAt(uint32_t offset, uint8_t* out, uint32_t len);

    void nandWREN();
    uint8_t nandGetFeature(uint8_t addr);
    void nandSetFeature(uint8_t addr, uint8_t val);
    bool nandWaitReady(uint32_t timeout_us = 2'000'000);
    bool nandEraseBlock(uint32_t blockIndex);
    bool nandProgramPage(uint32_t rowPageAddr, const uint8_t* data, uint32_t len);
    bool nandReadPage(uint32_t rowPageAddr, uint8_t* out, uint32_t len);
    bool nandInit();

    // LittleFS block device adapter callbacks (static to access from C API)
    static int lfsBlockRead(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, void *buffer, lfs_size_t size);
    static int lfsBlockProg(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, const void *buffer, lfs_size_t size);
    static int lfsBlockErase(const struct lfs_config *c, lfs_block_t block);
    static int lfsBlockSync(const struct lfs_config *c);

    void openLogSession();
    void activateLogging();
    void closeLogSession();
    void flushRingToNand();

    void markDirty();           // Write LittleFS marker file on log start
    void clearDirty();          // Remove LittleFS marker file on log close
    bool checkDirtyOnStartup(); // Check if previous session was dirty
    void clearRing();
    void runStartupRecovery();
};

#endif
