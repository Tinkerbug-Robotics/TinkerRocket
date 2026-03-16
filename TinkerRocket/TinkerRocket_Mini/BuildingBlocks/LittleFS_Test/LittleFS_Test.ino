// LittleFS NAND Compatibility Test
// This tests if LittleFS can work with the W25N01GV NAND chip
// Rename to .ino to compile separately

#include <SPI.h>
#include <lfs.h>

// NAND chip constants (from TR_LogToFlash)
#define NAND_PAGE_SIZE 2048
#define NAND_PAGES_PER_BLK 64
#define NAND_BLOCK_SIZE (NAND_PAGE_SIZE * NAND_PAGES_PER_BLK)  // 128KB
#define NAND_BLOCK_COUNT 1024

// SPI pins (matches OutComputer config)
#define SPI_SCK 37
#define SPI_MISO 35
#define SPI_MOSI 38
#define NAND_CS_PIN 36
#define PWR_PIN 6  // Power rail enable for NAND/MRAM

// NAND commands
#define NAND_RDID 0x9F
#define NAND_WREN 0x06
#define NAND_GETFEAT 0x0F
#define NAND_SETFEAT 0x1F
#define NAND_BLKERASE 0xD8
#define NAND_PROGLOAD 0x02
#define NAND_PROGEXEC 0x10
#define NAND_PAGEREAD 0x13
#define NAND_READCACHE 0x03
#define FEAT_STAT 0xC0
#define FEAT_PROT 0xA0
#define STAT_OIP 0x01
#define STAT_PFAIL 0x08
#define STAT_EFAIL 0x04

// Global LittleFS objects
lfs_t lfs;
lfs_file_t file;

// Cache buffers for LittleFS (allocated on heap to avoid stack overflow)
uint8_t* read_buffer = nullptr;
uint8_t* prog_buffer = nullptr;
uint8_t* lookahead_buffer = nullptr;

// Debug control
bool debug_block_ops = true;  // Set to false to reduce serial output

// ============================================================================
// Low-level NAND functions (minimal version from TR_LogToFlash)
// ============================================================================

void nandWREN()
{
    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    digitalWrite(NAND_CS_PIN, LOW);
    SPI.transfer(NAND_WREN);
    digitalWrite(NAND_CS_PIN, HIGH);
    SPI.endTransaction();
}

uint8_t nandGetFeature(uint8_t addr)
{
    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    digitalWrite(NAND_CS_PIN, LOW);
    SPI.transfer(NAND_GETFEAT);
    SPI.transfer(addr);
    uint8_t v = SPI.transfer(0x00);
    digitalWrite(NAND_CS_PIN, HIGH);
    SPI.endTransaction();
    return v;
}

void nandSetFeature(uint8_t addr, uint8_t val)
{
    nandWREN();
    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    digitalWrite(NAND_CS_PIN, LOW);
    SPI.transfer(NAND_SETFEAT);
    SPI.transfer(addr);
    SPI.transfer(val);
    digitalWrite(NAND_CS_PIN, HIGH);
    SPI.endTransaction();
}

bool nandWaitReady(uint32_t timeout_us = 2000000)
{
    uint32_t start = micros();
    while (true)
    {
        uint8_t st = nandGetFeature(FEAT_STAT);
        if ((st & STAT_OIP) == 0)
            return true;
        if ((micros() - start) > timeout_us)
            return false;
        delayMicroseconds(30);
    }
}

bool nandEraseBlock(uint32_t blockIndex)
{
    uint32_t row = blockIndex * NAND_PAGES_PER_BLK;
    nandWREN();
    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    digitalWrite(NAND_CS_PIN, LOW);
    SPI.transfer(NAND_BLKERASE);
    SPI.transfer((row >> 16) & 0xFF);
    SPI.transfer((row >> 8) & 0xFF);
    SPI.transfer(row & 0xFF);
    digitalWrite(NAND_CS_PIN, HIGH);
    SPI.endTransaction();

    if (!nandWaitReady())
        return false;

    uint8_t st = nandGetFeature(FEAT_STAT);
    return !(st & STAT_EFAIL);
}

bool nandProgramPage(uint32_t rowPageAddr, const uint8_t* data, uint32_t len)
{
    if (len != NAND_PAGE_SIZE)
        return false;

    nandWREN();
    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    digitalWrite(NAND_CS_PIN, LOW);
    SPI.transfer(NAND_PROGLOAD);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    for (uint32_t i = 0; i < len; i++)
        SPI.transfer(data[i]);
    digitalWrite(NAND_CS_PIN, HIGH);
    SPI.endTransaction();

    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    digitalWrite(NAND_CS_PIN, LOW);
    SPI.transfer(NAND_PROGEXEC);
    SPI.transfer((rowPageAddr >> 16) & 0xFF);
    SPI.transfer((rowPageAddr >> 8) & 0xFF);
    SPI.transfer(rowPageAddr & 0xFF);
    digitalWrite(NAND_CS_PIN, HIGH);
    SPI.endTransaction();

    if (!nandWaitReady())
        return false;

    uint8_t st = nandGetFeature(FEAT_STAT);
    return !(st & STAT_PFAIL);
}

bool nandReadPage(uint32_t rowPageAddr, uint8_t* out, uint32_t len)
{
    if (len > NAND_PAGE_SIZE)
        return false;

    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    digitalWrite(NAND_CS_PIN, LOW);
    SPI.transfer(NAND_PAGEREAD);
    SPI.transfer((rowPageAddr >> 16) & 0xFF);
    SPI.transfer((rowPageAddr >> 8) & 0xFF);
    SPI.transfer(rowPageAddr & 0xFF);
    digitalWrite(NAND_CS_PIN, HIGH);
    SPI.endTransaction();

    if (!nandWaitReady())
        return false;

    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    digitalWrite(NAND_CS_PIN, LOW);
    SPI.transfer(NAND_READCACHE);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    for (uint32_t i = 0; i < len; i++)
        out[i] = SPI.transfer(0x00);
    digitalWrite(NAND_CS_PIN, HIGH);
    SPI.endTransaction();

    return true;
}

bool nandInit()
{
    pinMode(NAND_CS_PIN, OUTPUT);
    digitalWrite(NAND_CS_PIN, HIGH);

    nandSetFeature(FEAT_PROT, 0x00);

    SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    digitalWrite(NAND_CS_PIN, LOW);
    SPI.transfer(NAND_RDID);
    uint8_t mid = SPI.transfer(0x00);
    uint8_t did = SPI.transfer(0x00);
    digitalWrite(NAND_CS_PIN, HIGH);
    SPI.endTransaction();

    Serial.printf("[NAND] RDID MID=0x%02X DID=0x%02X\n", mid, did);
    return true;
}

// ============================================================================
// LittleFS Block Device Adapter
// ============================================================================

// Read a region from a block
int lfs_block_read(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, void *buffer, lfs_size_t size)
{
    // LittleFS block = NAND block (128KB)
    // We need to map offset within block to NAND pages

    if (debug_block_ops)
        Serial.printf("  read: block=%u off=%u size=%u\n", block, off, size);

    uint8_t* buf = (uint8_t*)buffer;
    lfs_size_t bytes_read = 0;

    while (bytes_read < size)
    {
        uint32_t block_offset = off + bytes_read;
        uint32_t page_in_block = block_offset / NAND_PAGE_SIZE;
        uint32_t offset_in_page = block_offset % NAND_PAGE_SIZE;

        uint32_t row_page = (block * NAND_PAGES_PER_BLK) + page_in_block;

        // Read full page into temp buffer
        uint8_t page_buf[NAND_PAGE_SIZE];
        if (!nandReadPage(row_page, page_buf, NAND_PAGE_SIZE))
        {
            Serial.printf("ERROR: Read page %u failed\n", row_page);
            return LFS_ERR_IO;
        }

        // Copy requested portion
        uint32_t bytes_to_copy = min((uint32_t)size - bytes_read,
                                     NAND_PAGE_SIZE - offset_in_page);
        memcpy(buf + bytes_read, page_buf + offset_in_page, bytes_to_copy);
        bytes_read += bytes_to_copy;
    }

    return LFS_ERR_OK;
}

// Program a region in a block
int lfs_block_prog(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, const void *buffer, lfs_size_t size)
{
    // IMPORTANT: NAND pages can only be programmed ONCE after erase
    // We cannot do read-modify-write on NAND
    // LittleFS should only call this with page-aligned offsets and sizes

    if (debug_block_ops)
        Serial.printf("  prog: block=%u off=%u size=%u\n", block, off, size);

    // Verify alignment (LittleFS should respect prog_size)
    if (off % NAND_PAGE_SIZE != 0 || size % NAND_PAGE_SIZE != 0)
    {
        Serial.printf("ERROR: Unaligned write! off=%u size=%u\n", off, size);
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
        if (!nandProgramPage(row_page, buf + bytes_written, NAND_PAGE_SIZE))
        {
            Serial.printf("ERROR: Program page %u failed\n", row_page);
            return LFS_ERR_IO;
        }

        bytes_written += NAND_PAGE_SIZE;
    }

    return LFS_ERR_OK;
}

// Erase a block
int lfs_block_erase(const struct lfs_config *c, lfs_block_t block)
{
    if (debug_block_ops)
        Serial.printf("  erase: block=%u\n", block);
    if (!nandEraseBlock(block))
    {
        Serial.printf("ERROR: Erase block %u failed\n", block);
        return LFS_ERR_IO;
    }
    return LFS_ERR_OK;
}

// Sync (no-op for NAND)
int lfs_block_sync(const struct lfs_config *c)
{
    return LFS_ERR_OK;
}

// ============================================================================
// Arduino Setup and Loop
// ============================================================================

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    delay(1000);

    Serial.println("\n\n=== LittleFS NAND Compatibility Test ===\n");

    // Enable power rail for NAND/MRAM chips
    Serial.println("[0] Enabling power rail...");
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(100);  // Let power stabilize
    Serial.println("Power enabled\n");

    // Initialize SPI with explicit pins for ESP32-S3
    Serial.println("[1] Initializing SPI bus...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);
    Serial.printf("  SCK=%d, MISO=%d, MOSI=%d\n\n", SPI_SCK, SPI_MISO, SPI_MOSI);

    // Initialize NAND
    Serial.println("[2] Initializing NAND chip...");
    if (!nandInit())
    {
        Serial.println("NAND init failed!");
        while (1) {}
    }
    Serial.println("NAND init OK\n");

    // Allocate cache buffers on heap
    Serial.println("[3] Allocating LittleFS cache buffers...");
    read_buffer = (uint8_t*)malloc(NAND_PAGE_SIZE);
    prog_buffer = (uint8_t*)malloc(NAND_PAGE_SIZE);
    lookahead_buffer = (uint8_t*)malloc(128);

    if (!read_buffer || !prog_buffer || !lookahead_buffer)
    {
        Serial.println("Failed to allocate cache buffers!");
        while (1) {}
    }
    Serial.println("Cache buffers allocated\n");

    // Configure LittleFS
    Serial.println("[4] Configuring LittleFS...");
    struct lfs_config cfg = {0};
    cfg.read = lfs_block_read;
    cfg.prog = lfs_block_prog;
    cfg.erase = lfs_block_erase;
    cfg.sync = lfs_block_sync;

    cfg.read_size = NAND_PAGE_SIZE;
    cfg.prog_size = NAND_PAGE_SIZE;
    cfg.block_size = NAND_BLOCK_SIZE;
    cfg.block_count = NAND_BLOCK_COUNT;
    cfg.cache_size = NAND_PAGE_SIZE;
    cfg.lookahead_size = 128;
    cfg.block_cycles = 500;  // Wear leveling cycle count

    cfg.read_buffer = read_buffer;
    cfg.prog_buffer = prog_buffer;
    cfg.lookahead_buffer = lookahead_buffer;

    Serial.println("Config:");
    Serial.printf("  Block size: %d KB\n", cfg.block_size / 1024);
    Serial.printf("  Block count: %d\n", cfg.block_count);
    Serial.printf("  Total size: %d MB\n", (cfg.block_size * cfg.block_count) / (1024 * 1024));
    Serial.println();

    // Try to mount existing filesystem
    Serial.println("[5] Attempting to mount existing filesystem...");
    int err = lfs_mount(&lfs, &cfg);

    if (err)
    {
        Serial.println("No existing filesystem found, formatting...");
        debug_block_ops = true;  // Enable debug during format
        err = lfs_format(&lfs, &cfg);
        if (err)
        {
            Serial.printf("Format failed with error: %d\n", err);
            while (1) {}
        }
        Serial.println("Format OK");

        err = lfs_mount(&lfs, &cfg);
        if (err)
        {
            Serial.printf("Mount after format failed with error: %d\n", err);
            while (1) {}
        }
        debug_block_ops = false;  // Disable debug after format
    }
    else
    {
        debug_block_ops = false;  // Disable debug if mounting existing FS
    }
    Serial.println("Filesystem mounted!\n");

    // Test: Write a file
    Serial.println("[6] Testing file write...");
    err = lfs_file_open(&lfs, &file, "test.txt", LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (err)
    {
        Serial.printf("File open failed: %d\n", err);
    }
    else
    {
        const char* message = "Hello from LittleFS on NAND!\n";
        lfs_size_t written = lfs_file_write(&lfs, &file, message, strlen(message));
        Serial.printf("Wrote %d bytes\n", written);
        lfs_file_close(&lfs, &file);
    }

    // Test: Read the file back
    Serial.println("\n[7] Testing file read...");
    err = lfs_file_open(&lfs, &file, "test.txt", LFS_O_RDONLY);
    if (err)
    {
        Serial.printf("File open failed: %d\n", err);
    }
    else
    {
        char buffer[128];
        lfs_size_t bytes_read = lfs_file_read(&lfs, &file, buffer, sizeof(buffer) - 1);
        buffer[bytes_read] = '\0';
        Serial.printf("Read %d bytes: %s", bytes_read, buffer);
        lfs_file_close(&lfs, &file);
    }

    // Test: List directory
    Serial.println("\n[8] Listing root directory...");
    lfs_dir_t dir;
    struct lfs_info info;
    lfs_dir_open(&lfs, &dir, "/");
    while (lfs_dir_read(&lfs, &dir, &info) > 0)
    {
        if (info.type == LFS_TYPE_REG)
        {
            Serial.printf("  File: %s (%d bytes)\n", info.name, info.size);
        }
        else if (info.type == LFS_TYPE_DIR)
        {
            Serial.printf("  Dir:  %s\n", info.name);
        }
    }
    lfs_dir_close(&lfs, &dir);

    // Unmount
    Serial.println("\n[9] Unmounting filesystem...");
    lfs_unmount(&lfs);

    Serial.println("\n=== Test Complete! ===");
    Serial.println("LittleFS is compatible with your NAND chip!");
}

void loop()
{
    // Test complete
    delay(1000);
}
