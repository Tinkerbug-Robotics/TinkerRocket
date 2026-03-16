#include <Arduino.h>
#include <SPI.h>

const int PWR_PIN = 6;


// ===================== Pins / SPI =====================
static constexpr int PIN_SCK      = 37;
static constexpr int PIN_MISO     = 35;
static constexpr int PIN_MOSI     = 38;
static constexpr int PIN_CS_MRAM  = 34;
static constexpr int PIN_CS_NAND  = 36;

// Tune these for your hardware
static constexpr uint32_t SPI_HZ_MRAM = 40'000'000;
static constexpr uint32_t SPI_HZ_NAND = 40'000'000;
static constexpr uint8_t  SPI_MODE_MRAM = SPI_MODE0;
static constexpr uint8_t  SPI_MODE_NAND = SPI_MODE0;

// ===================== Message Types =====================
static constexpr uint8_t H3LIS331_MSG   = 0xA1;
static constexpr uint8_t ICM45686_MSG   = 0xA2;
static constexpr uint8_t GNSS_MSG       = 0xA3;
static constexpr uint8_t MS5611_MSG     = 0xA4;
static constexpr uint8_t POWER_MSG      = 0xA5;
static constexpr uint8_t NON_SENSOR_MSG = 0xA8;
static constexpr uint8_t LIS3MDL_MSG    = 0xF2;

// ===================== Payload sizes =====================
static constexpr uint16_t SZ_GNSS_PAYLOAD = 31; // log gnss_payload[31]
static constexpr uint16_t SZ_ICM45686     = 36;
static constexpr uint16_t SZ_H3LIS331     = 10;
static constexpr uint16_t SZ_MS5611       = 10;
static constexpr uint16_t SZ_LIS3MDL      = 10;
static constexpr uint16_t SZ_POWER        = 10;

// Pick a fixed payload size for NonSensorData log payload (adjust if yours differs)
static constexpr uint16_t SZ_NONSENSOR    = 65;

// ===================== Rates =====================
static constexpr uint32_t HZ_ICM45686   = 1000;
static constexpr uint32_t HZ_H3LIS331   = 1000;
static constexpr uint32_t HZ_MS5611     = 100;
static constexpr uint32_t HZ_LIS3MDL    = 1000;
static constexpr uint32_t HZ_GNSS       = 25;
static constexpr uint32_t HZ_POWER      = 50;
static constexpr uint32_t HZ_NONSENSOR  = 1000;

// ===================== Frame format =====================
static constexpr uint8_t  SOF0 = 0xAA;
static constexpr uint8_t  SOF1 = 0x55;

// Must be >= max payload you use
static constexpr uint16_t MAX_PAYLOAD = 200;
static constexpr uint16_t MAX_FRAME   = 4 + 1 + 1 + MAX_PAYLOAD + 2;

// ===================== CRC16 (CCITT-FALSE) =====================
static uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// ===================== SPI Settings =====================
static SPISettings spiMRAM(SPI_HZ_MRAM, MSBFIRST, SPI_MODE_MRAM);
static SPISettings spiNAND(SPI_HZ_NAND, MSBFIRST, SPI_MODE_NAND);

static inline void csLow(int pin)  { digitalWrite(pin, LOW); }
static inline void csHigh(int pin) { digitalWrite(pin, HIGH); }

// ===================== MR25H10 MRAM (SPI) =====================
// READ  0x03 + 24-bit addr
// WRITE 0x02 + 24-bit addr (requires WREN 0x06)
static constexpr uint8_t MRAM_WREN  = 0x06;
static constexpr uint8_t MRAM_READ  = 0x03;
static constexpr uint8_t MRAM_WRITE = 0x02;

// 1 Mbit => 128 KiB
static constexpr uint32_t MRAM_SIZE_BYTES = 128UL * 1024UL;

// Entire MRAM as ring (byte FIFO)
static constexpr uint32_t MRAM_RING_BASE = 0;
static constexpr uint32_t MRAM_RING_SIZE = MRAM_SIZE_BYTES;

// Ring state in RAM
static volatile uint32_t rb_head = 0;
static volatile uint32_t rb_tail = 0;
static volatile uint32_t rb_count = 0;
static volatile uint32_t rb_overruns = 0;
static volatile uint32_t rb_highwater = 0;

static void mramWREN() {
  SPI.beginTransaction(spiMRAM);
  csLow(PIN_CS_MRAM);
  SPI.transfer(MRAM_WREN);
  csHigh(PIN_CS_MRAM);
  SPI.endTransaction();
}

static void mramWriteBytes(uint32_t addr, const uint8_t* data, uint32_t len) {
  uint32_t a = MRAM_RING_BASE + (addr % MRAM_RING_SIZE);

  mramWREN();
  SPI.beginTransaction(spiMRAM);
  csLow(PIN_CS_MRAM);
  SPI.transfer(MRAM_WRITE);
  SPI.transfer((a >> 16) & 0xFF);
  SPI.transfer((a >> 8) & 0xFF);
  SPI.transfer(a & 0xFF);

  // ESP32 Arduino supports writeBytes
  SPI.writeBytes(data, len);

  csHigh(PIN_CS_MRAM);
  SPI.endTransaction();
}

static void mramReadBytes(uint32_t addr, uint8_t* out, uint32_t len) {
  uint32_t a = MRAM_RING_BASE + (addr % MRAM_RING_SIZE);

  SPI.beginTransaction(spiMRAM);
  csLow(PIN_CS_MRAM);
  SPI.transfer(MRAM_READ);
  SPI.transfer((a >> 16) & 0xFF);
  SPI.transfer((a >> 8) & 0xFF);
  SPI.transfer(a & 0xFF);

  // No readBytes() on ESP32 SPIClass; use transferBytes().
  // Send dummy 0x00 while reading.
  static uint8_t dummy[64];
  memset(dummy, 0x00, sizeof(dummy));

  uint32_t remaining = len;
  uint8_t* dst = out;
  while (remaining > 0) {
    uint32_t chunk = (remaining > sizeof(dummy)) ? sizeof(dummy) : remaining;
    SPI.transferBytes(dummy, dst, chunk);
    dst += chunk;
    remaining -= chunk;
  }

  csHigh(PIN_CS_MRAM);
  SPI.endTransaction();
}

// Ring enqueue to MRAM
static bool ringPush(const uint8_t* data, uint32_t len) {
  if (len == 0 || len > MRAM_RING_SIZE) return false;

  uint32_t freeSpace = MRAM_RING_SIZE - rb_count;
  if (len > freeSpace) {
    rb_overruns++;
    return false;
  }

  uint32_t head = rb_head;
  uint32_t toEnd = MRAM_RING_SIZE - head;

  if (len <= toEnd) {
    mramWriteBytes(head, data, len);
  } else {
    mramWriteBytes(head, data, toEnd);
    mramWriteBytes(0, data + toEnd, len - toEnd);
  }

  rb_head = (head + len) % MRAM_RING_SIZE;
  rb_count += len;
  if (rb_count > rb_highwater) rb_highwater = rb_count;
  return true;
}

static bool ringPop(uint8_t* out, uint32_t len) {
  if (len == 0) return true;
  if (len > rb_count) return false;

  uint32_t tail = rb_tail;
  uint32_t toEnd = MRAM_RING_SIZE - tail;

  if (len <= toEnd) {
    mramReadBytes(tail, out, len);
  } else {
    mramReadBytes(tail, out, toEnd);
    mramReadBytes(0, out + toEnd, len - toEnd);
  }

  rb_tail = (tail + len) % MRAM_RING_SIZE;
  rb_count -= len;
  return true;
}

// ===================== W25N01 SPI NAND (minimal subset) =====================
static constexpr uint8_t NAND_RDID      = 0x9F;
static constexpr uint8_t NAND_WREN      = 0x06;
static constexpr uint8_t NAND_GETFEAT   = 0x0F;
static constexpr uint8_t NAND_SETFEAT   = 0x1F;
static constexpr uint8_t NAND_BLKERASE  = 0xD8;
static constexpr uint8_t NAND_PROGLOAD  = 0x02;
static constexpr uint8_t NAND_PROGEXEC  = 0x10;

static constexpr uint8_t FEAT_STAT = 0xC0;
static constexpr uint8_t FEAT_PROT = 0xA0;

static constexpr uint8_t STAT_OIP   = 0x01;
static constexpr uint8_t STAT_PFAIL = 0x08;
static constexpr uint8_t STAT_EFAIL = 0x04;

static constexpr uint32_t NAND_PAGE_SIZE     = 2048;
static constexpr uint32_t NAND_PAGES_PER_BLK = 64;
static constexpr uint32_t NAND_BLOCK_SIZE    = NAND_PAGE_SIZE * NAND_PAGES_PER_BLK;

static uint32_t nand_page  = 0;
static uint32_t nand_block = 0;

static uint64_t nand_bytes_written = 0;
static uint32_t nand_prog_fail = 0;
static uint32_t nand_erase_fail = 0;
static uint32_t nand_prog_ops = 0;
static uint32_t nand_erase_ops = 0;

static uint32_t max_prog_busy_us  = 0;
static uint32_t max_erase_busy_us = 0;

static void nandWREN() {
  SPI.beginTransaction(spiNAND);
  csLow(PIN_CS_NAND);
  SPI.transfer(NAND_WREN);
  csHigh(PIN_CS_NAND);
  SPI.endTransaction();
}

static uint8_t nandGetFeature(uint8_t addr) {
  SPI.beginTransaction(spiNAND);
  csLow(PIN_CS_NAND);
  SPI.transfer(NAND_GETFEAT);
  SPI.transfer(addr);
  uint8_t v = SPI.transfer(0x00);
  csHigh(PIN_CS_NAND);
  SPI.endTransaction();
  return v;
}

static void nandSetFeature(uint8_t addr, uint8_t val) {
  nandWREN();
  SPI.beginTransaction(spiNAND);
  csLow(PIN_CS_NAND);
  SPI.transfer(NAND_SETFEAT);
  SPI.transfer(addr);
  SPI.transfer(val);
  csHigh(PIN_CS_NAND);
  SPI.endTransaction();
}

static void nandWaitReady(uint32_t &busy_us_out) {
  uint32_t t0 = micros();
  while (true) {
    uint8_t st = nandGetFeature(FEAT_STAT);
    if ((st & STAT_OIP) == 0) {
      busy_us_out = micros() - t0;
      return;
    }
    delayMicroseconds(50);
  }
}

static bool nandEraseBlock(uint32_t blockIndex) {
  uint32_t row = blockIndex * NAND_PAGES_PER_BLK;

  nandWREN();
  SPI.beginTransaction(spiNAND);
  csLow(PIN_CS_NAND);
  SPI.transfer(NAND_BLKERASE);
  SPI.transfer((row >> 16) & 0xFF);
  SPI.transfer((row >> 8) & 0xFF);
  SPI.transfer(row & 0xFF);
  csHigh(PIN_CS_NAND);
  SPI.endTransaction();

  uint32_t busy_us = 0;
  nandWaitReady(busy_us);
  if (busy_us > max_erase_busy_us) max_erase_busy_us = busy_us;

  uint8_t st = nandGetFeature(FEAT_STAT);
  if (st & STAT_EFAIL) {
    nand_erase_fail++;
    return false;
  }
  nand_erase_ops++;
  return true;
}

static bool nandProgramPage(uint32_t rowPageAddr, const uint8_t* data, uint32_t len) {
  if (len != NAND_PAGE_SIZE) return false;

  nandWREN();
  SPI.beginTransaction(spiNAND);
  csLow(PIN_CS_NAND);
  SPI.transfer(NAND_PROGLOAD);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.writeBytes(data, len);
  csHigh(PIN_CS_NAND);
  SPI.endTransaction();

  SPI.beginTransaction(spiNAND);
  csLow(PIN_CS_NAND);
  SPI.transfer(NAND_PROGEXEC);
  SPI.transfer((rowPageAddr >> 16) & 0xFF);
  SPI.transfer((rowPageAddr >> 8) & 0xFF);
  SPI.transfer(rowPageAddr & 0xFF);
  csHigh(PIN_CS_NAND);
  SPI.endTransaction();

  uint32_t busy_us = 0;
  nandWaitReady(busy_us);
  if (busy_us > max_prog_busy_us) max_prog_busy_us = busy_us;

  uint8_t st = nandGetFeature(FEAT_STAT);
  if (st & STAT_PFAIL) {
    nand_prog_fail++;
    return false;
  }
  nand_prog_ops++;
  nand_bytes_written += len;
  return true;
}

static void nandInitOrDie() {
  // Clear protection
  nandSetFeature(FEAT_PROT, 0x00);

  // Read ID
  SPI.beginTransaction(spiNAND);
  csLow(PIN_CS_NAND);
  SPI.transfer(NAND_RDID);
  uint8_t mid = SPI.transfer(0x00);
  uint8_t did = SPI.transfer(0x00);
  csHigh(PIN_CS_NAND);
  SPI.endTransaction();

  Serial.printf("[NAND] RDID: MID=0x%02X DID=0x%02X\n", mid, did);

  Serial.println("[NAND] Erasing block 0...");
  if (!nandEraseBlock(0)) {
    Serial.println("[NAND] ERROR: erase block 0 failed");
    while (1) delay(1000);
  }

  nand_page = 0;
  nand_block = 0;
}

// ===================== Frame builder =====================
static uint8_t frameBuf[MAX_FRAME];

static uint16_t buildFrame(uint8_t type, const uint8_t* payload, uint8_t len) {
  uint16_t idx = 0;
  frameBuf[idx++] = SOF0;
  frameBuf[idx++] = SOF1;
  frameBuf[idx++] = SOF0;
  frameBuf[idx++] = SOF1;

  frameBuf[idx++] = type;
  frameBuf[idx++] = len;

  if (len && payload) {
    memcpy(&frameBuf[idx], payload, len);
    idx += len;
  }

  uint16_t crc = crc16_ccitt_false(&frameBuf[4], 1 + 1 + len);
  frameBuf[idx++] = uint8_t((crc >> 8) & 0xFF);
  frameBuf[idx++] = uint8_t(crc & 0xFF);

  return idx;
}

// ===================== Synthetic payload generators =====================
static uint32_t g_counter = 0;

static void fillPayload(uint8_t* p, uint16_t n, uint8_t tag) {
  uint32_t t = micros();
  for (uint16_t i = 0; i < n; i++) {
    p[i] = uint8_t((i * 31u) ^ (t >> (i & 7)) ^ (g_counter & 0xFF) ^ tag);
  }
}

static uint8_t payICM[SZ_ICM45686];
static uint8_t payH3[SZ_H3LIS331];
static uint8_t payMS[SZ_MS5611];
static uint8_t payMAG[SZ_LIS3MDL];
static uint8_t payGNSS[SZ_GNSS_PAYLOAD];
static uint8_t payPWR[SZ_POWER];
static uint8_t payNS[SZ_NONSENSOR];

// ===================== Scheduling =====================
struct RateStream {
  uint8_t  type;
  uint16_t payloadLen;
  uint32_t period_us;
  uint32_t next_us;
  uint8_t* payload;
  uint64_t produced_frames = 0;
  uint64_t produced_bytes  = 0;
  uint32_t drops = 0;
};

static RateStream streams[] = {
  { ICM45686_MSG,   SZ_ICM45686,     1000000UL / HZ_ICM45686,  0, payICM },
  { H3LIS331_MSG,   SZ_H3LIS331,     1000000UL / HZ_H3LIS331,  0, payH3  },
  { MS5611_MSG,     SZ_MS5611,       1000000UL / HZ_MS5611,    0, payMS  },
  { LIS3MDL_MSG,    SZ_LIS3MDL,      1000000UL / HZ_LIS3MDL,   0, payMAG },
  { GNSS_MSG,       SZ_GNSS_PAYLOAD, 1000000UL / HZ_GNSS,      0, payGNSS},
  { POWER_MSG,      SZ_POWER,        1000000UL / HZ_POWER,     0, payPWR },
  { NON_SENSOR_MSG, SZ_NONSENSOR,    1000000UL / HZ_NONSENSOR, 0, payNS  },
};
static constexpr size_t N_STREAMS = sizeof(streams)/sizeof(streams[0]);

// ===================== Flush control / Stats =====================
static uint8_t pageBuf[NAND_PAGE_SIZE];

static uint64_t total_pushed_bytes = 0;
static uint64_t total_pushed_frames = 0;
static uint64_t total_dropped_frames = 0;

static uint32_t lastStatsMs = 0;
static uint64_t lastPushedBytes = 0;
static uint64_t lastNandBytes = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(500);

  pinMode(PIN_CS_MRAM, OUTPUT);
  pinMode(PIN_CS_NAND, OUTPUT);
  csHigh(PIN_CS_MRAM);
  csHigh(PIN_CS_NAND);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  Serial.println("\n=== ESP32-P4 MRAM->SPI NAND Logging Benchmark ===");
  Serial.printf("SPI: SCK=%d MISO=%d MOSI=%d | MRAM_CS=%d NAND_CS=%d\n",
                PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS_MRAM, PIN_CS_NAND);
  Serial.printf("MRAM ring size: %lu bytes\n", (unsigned long)MRAM_RING_SIZE);

  nandInitOrDie();

  uint32_t now = micros();
  for (size_t i = 0; i < N_STREAMS; i++) {
    streams[i].next_us = now + streams[i].period_us;
  }

  lastStatsMs = millis();
}

static void maybeProduce() {
  uint32_t now = micros();

  for (size_t i = 0; i < N_STREAMS; i++) {
    while ((int32_t)(now - streams[i].next_us) >= 0) {
      fillPayload(streams[i].payload, streams[i].payloadLen, streams[i].type);

      uint16_t frameLen = buildFrame(streams[i].type,
                                     streams[i].payload,
                                     (uint8_t)streams[i].payloadLen);

      if (ringPush(frameBuf, frameLen)) {
        streams[i].produced_frames++;
        streams[i].produced_bytes += frameLen;
        total_pushed_frames++;
        total_pushed_bytes += frameLen;
      } else {
        streams[i].drops++;
        total_dropped_frames++;
      }

      g_counter++;
      streams[i].next_us += streams[i].period_us;
      now = micros();
    }
  }
}

static void maybeFlushToNand() {
  while (rb_count >= NAND_PAGE_SIZE) {
    if (!ringPop(pageBuf, NAND_PAGE_SIZE)) return;

    uint32_t newBlock = nand_page / NAND_PAGES_PER_BLK;
    if (newBlock != nand_block) {
      nand_block = newBlock;
      Serial.printf("[NAND] Erasing block %lu...\n", (unsigned long)nand_block);
      (void)nandEraseBlock(nand_block);
    }

    (void)nandProgramPage(nand_page, pageBuf, NAND_PAGE_SIZE);
    nand_page++;
  }
}

static void printStats() {
  uint32_t nowMs = millis();
  if (nowMs - lastStatsMs < 1000) return;
  uint32_t dtMs = nowMs - lastStatsMs;
  lastStatsMs = nowMs;

  uint64_t pushedDelta = total_pushed_bytes - lastPushedBytes;
  lastPushedBytes = total_pushed_bytes;

  uint64_t nandDelta = nand_bytes_written - lastNandBytes;
  lastNandBytes = nand_bytes_written;

  float prodKBs  = (dtMs > 0) ? (float)pushedDelta / (float)dtMs : 0.0f;
  float flushKBs = (dtMs > 0) ? (float)nandDelta   / (float)dtMs : 0.0f;

  Serial.printf("\n[t=%lus] Produced: %.1f KB/s, Flushed: %.1f KB/s | MRAM fill=%lu / %lu (hi=%lu) | drops=%llu overruns=%lu\n",
                (unsigned long)(nowMs / 1000),
                (double)prodKBs,
                (double)flushKBs,
                (unsigned long)rb_count,
                (unsigned long)MRAM_RING_SIZE,
                (unsigned long)rb_highwater,
                (unsigned long long)total_dropped_frames,
                (unsigned long)rb_overruns);

  Serial.printf("NAND: pages=%lu blocks=%lu written=%llu KB | prog_ops=%lu erase_ops=%lu | prog_fail=%lu erase_fail=%lu | max_prog=%lu us max_erase=%lu us\n",
                (unsigned long)nand_page,
                (unsigned long)nand_block,
                (unsigned long long)(nand_bytes_written / 1024),
                (unsigned long)nand_prog_ops,
                (unsigned long)nand_erase_ops,
                (unsigned long)nand_prog_fail,
                (unsigned long)nand_erase_fail,
                (unsigned long)max_prog_busy_us,
                (unsigned long)max_erase_busy_us);
}

void loop() {
  maybeProduce();
  maybeFlushToNand();
  printStats();
  yield();
}