#include <SPI.h>

// IMU (STM lib you pasted)
#include <ISM6HG256XSensor.h>

// ---- Forward declare for Arduino auto-prototype insertion ----
struct ImuRawFrame;
void printImuRaw(const ImuRawFrame *f);

// SPI pins
const uint8_t SPI_SCK = 9;
const uint8_t SPI_SDO = 10;
const uint8_t SPI_SDI = 8;

// Chip select pins
const uint8_t MMC5983MA_CS = 13;
const uint8_t BMP585_CS    = 6;
const uint8_t ISM6HG256_CS = 7;

// ---------------- IMU ----------------
ISM6HG256XSensor imu(&SPI, ISM6HG256_CS);

// We count “frames” when all 3 are ready (LG+HG+G).
volatile uint32_t g_imuCount = 0;

struct __attribute__((packed)) ImuRawFrame
{
  uint32_t t_us;              // micros() timestamp (wraps ~71.6 minutes)
  int16_t  lg_x, lg_y, lg_z;  // low-g accel raw counts
  int16_t  hg_x, hg_y, hg_z;  // high-g accel raw counts
  int16_t  g_x,  g_y,  g_z;   // gyro raw counts
};

#if __cplusplus >= 201103L
static_assert(sizeof(ImuRawFrame) == 22, "ImuRawFrame size expected to be 22 bytes");
#endif

ImuRawFrame g_lastImu = {};

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Booting...");

  // Disable all CS initially
  pinMode(MMC5983MA_CS, OUTPUT); digitalWrite(MMC5983MA_CS, HIGH);
  pinMode(BMP585_CS,    OUTPUT); digitalWrite(BMP585_CS, HIGH);
  pinMode(ISM6HG256_CS, OUTPUT); digitalWrite(ISM6HG256_CS, HIGH);
  delay(10);

  // SPI bus
  SPI.begin(SPI_SCK, SPI_SDO, SPI_SDI);
  delay(10);

  // ---------------- IMU init/config ----------------
  imu.begin();

  uint8_t status = 0;
  status |= imu.Enable_X();     // Low-g accel
  status |= imu.Enable_HG_X();  // High-g accel
  status |= imu.Enable_G();     // Gyro

  status |= imu.Set_X_OutputDataRate(960);
  status |= imu.Set_X_FullScale(16);

  status |= imu.Set_HG_X_OutputDataRate(960);
  status |= imu.Set_HG_X_FullScale(256);

  status |= imu.Set_G_OutputDataRate(960);
  status |= imu.Set_G_FullScale(4000);

  if (status != ISM6HG256X_OK)
  {
    Serial.println("ISM6HG256XSensor failed to init/configure");
    while (1) {}
  }

  Serial.println("Sensor ready.");
}

void loop()
{
  
  // ----------- IMU: poll DRDY and read RAW -----------
  uint8_t lg_drdy = 0, hg_drdy = 0, g_drdy = 0;
  (void)imu.Get_X_DRDY_Status(&lg_drdy);
  (void)imu.HG_X_Get_DRDY_Status(&hg_drdy);
  (void)imu.Get_G_DRDY_Status(&g_drdy);

  uint8_t drdyBits = (lg_drdy ? 0x01 : 0) | (hg_drdy ? 0x02 : 0) | (g_drdy ? 0x04 : 0);

  // Only count/log when ALL THREE are ready (coherent triplet)
  if (drdyBits == 0x07)
  {
    ISM6HG256X_AxesRaw_t lg_raw = {};
    ISM6HG256X_AxesRaw_t hg_raw = {};
    ISM6HG256X_AxesRaw_t g_raw  = {};

    (void)imu.Get_X_AxesRaw(&lg_raw);
    (void)imu.Get_HG_X_AxesRaw(&hg_raw);
    (void)imu.Get_G_AxesRaw(&g_raw);

    ImuRawFrame f = {};
    f.t_us = (uint32_t)micros();
    f.lg_x = lg_raw.x; f.lg_y = lg_raw.y; f.lg_z = lg_raw.z;
    f.hg_x = hg_raw.x; f.hg_y = hg_raw.y; f.hg_z = hg_raw.z;
    f.g_x  = g_raw.x;  f.g_y  = g_raw.y;  f.g_z  = g_raw.z;

    g_lastImu = f;
    g_imuCount++;
  }

  // ----------- Print Hz once per second -----------
  static uint32_t lastPrintMs = 0;
  static uint32_t lastImuCount = 0;
  static uint32_t lastBmpCount = 0;

  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= 1000)
  {
    lastPrintMs = nowMs;

    uint32_t imuCount = g_imuCount;

    uint32_t imuDelta = imuCount - lastImuCount; lastImuCount = imuCount;

    Serial.print("IMU Hz = ");
    Serial.print((float)imuDelta, 1);

    Serial.print("IMU t_us="); Serial.println(g_lastImu.t_us);
    printImuRaw(&g_lastImu);
    Serial.println("----");
  }
}

// Pointer form avoids Arduino auto-prototype issues with references
void printImuRaw(const ImuRawFrame *f)
{
  Serial.print("LG: ");
  Serial.print(f->lg_x); Serial.print(", ");
  Serial.print(f->lg_y); Serial.print(", ");
  Serial.println(f->lg_z);

  Serial.print("HG: ");
  Serial.print(f->hg_x); Serial.print(", ");
  Serial.print(f->hg_y); Serial.print(", ");
  Serial.println(f->hg_z);

  Serial.print("G : ");
  Serial.print(f->g_x); Serial.print(", ");
  Serial.print(f->g_y); Serial.print(", ");
  Serial.println(f->g_z);
}