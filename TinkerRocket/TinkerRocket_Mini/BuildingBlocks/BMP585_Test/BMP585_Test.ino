#include <SPI.h>

// Pressure sensor (your custom lib)
#include <TR_BMP585.h>

// SPI pins
const uint8_t SPI_SCK = 9;
const uint8_t SPI_SDO = 10;
const uint8_t SPI_SDI = 8;

// Chip select pins
const uint8_t MMC5983MA_CS = 13;
const uint8_t BMP585_CS    = 6;
const uint8_t ISM6HG256_CS = 7;

// Interrupt pins
const uint8_t BMP585_INT_PIN = 11;

// ---------------- BMP585 ----------------
static const SPISettings bmpSPISettings(1000000, MSBFIRST, SPI_MODE0);
TR_BMP585 bmp(SPI, BMP585_CS, bmpSPISettings);

// Data ready flag
volatile bool bmp_data_ready = false;

// Counter to measure update rate
volatile uint32_t bmp_count = 0;

// Interrupt service routine
void bmpInterruptHandler()
{
  bmp_data_ready = true;
  bmp_count++;
}


void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("BMP585 Test ...");

    // Disable all CS initially
    pinMode(MMC5983MA_CS, OUTPUT); digitalWrite(MMC5983MA_CS, HIGH);
    pinMode(BMP585_CS,    OUTPUT); digitalWrite(BMP585_CS, HIGH);
    pinMode(ISM6HG256_CS, OUTPUT); digitalWrite(ISM6HG256_CS, HIGH);
    delay(10);

    // SPI bus
    SPI.begin(SPI_SCK, SPI_SDO, SPI_SDI);
    delay(10);



    // ---------------- BMP585 init (retry loop) ----------------
    while (!bmp.begin())
    {
        Serial.print("BMP585 initialization failed, ID 0x ");
        Serial.println(bmp.readChipIdCached());
        delay(1000);
    }

    Serial.print("BMP585 OK. chip_id = 0x");
    Serial.println(bmp.readChipId(), HEX);

    bmp.setPowerMode(TR_BMP585::PowerMode::Continuous);
    bmp.setTemperatureOversampling(TR_BMP585::Oversampling::x1);
    bmp.setPressureOversampling(TR_BMP585::Oversampling::x1);
    bmp.setIirFilter(TR_BMP585::IirCoeff::Bypass, TR_BMP585::IirCoeff::Bypass);
    bmp.setOutputDataRate(TR_BMP585::OutputDataRate::ODR_240Hz);
    bmp.enableDataReadyInterrupt(true, 
                                /*latched=*/ false, 
                                /*activeHigh=*/ true, 
                                /*openDrain=*/ false);

    // BMP 585 interrupt
    pinMode(BMP585_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(BMP585_INT_PIN),
                    bmpInterruptHandler,
                    RISING );
      
    Serial.println("Sensors ready.");
}

void loop()
{
  // ----------- BMP: handle DRDY events -----------
  if (bmp_data_ready)
  {

    noInterrupts();
    bmp_data_ready = false;
    interrupts();
    
    TR_BMP585::BmpCompFrame f;
    bmp.readCompFrame(f);

  }



  // ----------- Print Hz once per second -----------
  static uint32_t lastPrintMs = 0;
  static uint32_t lastBmpCount = 0;

  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= 1000)
  {
    lastPrintMs = nowMs;

    noInterrupts();
    uint32_t local_bmp_count = bmp_count;
    bmp_count = 0;
    interrupts();

    Serial.print(" BMP Hz = ");
    Serial.println((float)local_bmp_count, 1);
      
    // Example usage in sketch:
    TR_BMP585::BmpCompFrame f;
    if (bmp.readCompFrame(f))
    {
      Serial.print("t_us = "); Serial.print(f.t_us);
      Serial.print("  T_q16 = "); Serial.print(f.temp_q16);
      Serial.print("  P_q6 = "); Serial.println(f.press_q6);

      Serial.print("Temp C = "); Serial.print(tempC_from_q16(f.temp_q16), 4);
      Serial.print("  Press Pa = "); Serial.println(pressPa_from_q6(f.press_q6), 2);
    }

    Serial.println("----");
  }
}

static inline float tempC_from_q16(int32_t t_q16) { return ((float)t_q16) / 65536.0f; }
static inline float pressPa_from_q6(uint32_t p_q6){ return ((float)p_q6) / 64.0f; }