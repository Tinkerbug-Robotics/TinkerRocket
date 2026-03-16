
#include <SPI.h>

#include <SparkFun_MMC5983MA_Arduino_Library.h>


static constexpr uint8_t SPI_SCK  = 9;
static constexpr uint8_t SPI_MISO = 10;
static constexpr uint8_t SPI_MOSI = 8;

static constexpr uint8_t MMC5983_CS   = 13;
static constexpr uint8_t BMP585_CS    = 6;
static constexpr uint8_t ISM6HG256_CS = 7;

static constexpr uint8_t MMC5983_INT   = 12;


static const SPISettings magSPISettings(1000000, MSBFIRST, SPI_MODE0);

// Magnetometer object
SFE_MMC5983MA mag;

volatile bool new_mag_data_available = true;

volatile uint32_t mag_count = 0;

uint32_t mag_raw_X = 0;
uint32_t mag_raw_Y = 0;
uint32_t mag_raw_Z = 0;

uint32_t currentX = 0;
uint32_t currentY = 0;
uint32_t currentZ = 0;
double scaledX = 0;
double scaledY = 0;
double scaledZ = 0;

void magInterruptRoutine()
{
    new_mag_data_available = true;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("MMC5983MA Test");

    pinMode(MMC5983_CS,   OUTPUT); digitalWrite(MMC5983_CS, HIGH);
    pinMode(BMP585_CS,    OUTPUT); digitalWrite(BMP585_CS, HIGH);
    pinMode(ISM6HG256_CS, OUTPUT); digitalWrite(ISM6HG256_CS, HIGH);
    delay(100);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    pinMode(MMC5983_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(MMC5983_INT), magInterruptRoutine, RISING);

    while (mag.begin(MMC5983_CS) == false)
    {
        Serial.println("MMC5983MA did not respond. Retrying...");
        delay(500);
        mag.softReset();
        delay(500);
    }

    Serial.println("MMC5983MA connected");

    mag.softReset();

    mag.performSetOperation();
    delay(10);
    mag.performResetOperation();
    delay(10);

    int celsius = mag.getTemperature();
    float fahrenheit = (celsius * 9.0f / 5.0f) + 32.0f;
    Serial.print("Die temperature: ");
    Serial.print(celsius);
    Serial.print("°C or ");
    Serial.print((int)fahrenheit);
    Serial.println("°F");

    Serial.println("Setting filter bandwith to 800 Hz for continuous operation...");
    mag.setFilterBandwidth(800);
    Serial.print("Reading back filter bandwith: ");
    Serial.println(mag.getFilterBandwith());

    Serial.println("Setting continuous measurement frequency to 100 Hz...");
    mag.setContinuousModeFrequency(1000);
    Serial.print("Reading back continuous measurement frequency: ");
    Serial.println(mag.getContinuousModeFrequency());

    // Serial.println("Enabling auto set/reset...");
    // mag.enableAutomaticSetReset();
    Serial.print("Reading back automatic set/reset: ");
    Serial.println(mag.isAutomaticSetResetEnabled() ? "enabled" : "disabled");

    Serial.println("Enabling continuous mode...");
    mag.enableContinuousMode();
    Serial.print("Reading back continuous mode: ");
    Serial.println(mag.isContinuousModeEnabled() ? "enabled" : "disabled");

    Serial.println("Enabling interrupt...");
    mag.enableInterrupt();
    Serial.print("Reading back interrupt status: ");
    Serial.println(mag.isInterruptEnabled() ? "enabled" : "disabled");

    // Set our interrupt flag, just in case we missed the rising edge
    new_mag_data_available = true;

    mag.getMeasurementXYZ(&currentX, &currentY, &currentZ);

    Serial.print("Raw X,Y,Z:\t");Serial.print(currentX);
    Serial.print("\t");Serial.print(currentY);
    Serial.print("\t");Serial.println(currentZ);
    scaledX = (double)currentX - 131072.0;scaledX /= 131072.0;
    scaledY = (double)currentY - 131072.0;scaledY /= 131072.0;
    scaledZ = (double)currentZ - 131072.0;scaledZ /= 131072.0;
    Serial.print("Gauss X,YZ:\t");Serial.print(scaledX * 8, 5);
    Serial.print("\t");Serial.print(scaledY * 8, 5);
    Serial.print("\t");Serial.println(scaledZ * 8, 5);


}

void loop()
{

    if (new_mag_data_available)
    {
        new_mag_data_available = false;
        mag_count++;

        mag.clearMeasDoneInterrupt();

        mag.readFieldsXYZ(&mag_raw_X, &mag_raw_Y, &mag_raw_Z);

    }

     // ----------- Print Hz once per second -----------
    static uint32_t lastPrintMs = 0;
    static uint32_t lastBmpCount = 0;

    uint32_t nowMs = millis();
    if (nowMs - lastPrintMs >= 1000)
    {
        lastPrintMs = nowMs;

        noInterrupts();
        uint32_t local_mag_count = mag_count;
        mag_count = 0;
        interrupts();

        Serial.print("Mag Update Rate = ");Serial.print(local_mag_count);Serial.println(" Hz");

        mag.readFieldsXYZ(&mag_raw_X, &mag_raw_Y, &mag_raw_Z);
        Serial.print("Raw X,Y,Z:\t");Serial.print(mag_raw_X);
        Serial.print("\t");Serial.print(mag_raw_Y);
        Serial.print("\t");Serial.println(mag_raw_Z);
        scaledX = (double)mag_raw_X - 131072.0;scaledX /= 131072.0;
        scaledY = (double)mag_raw_Y - 131072.0;scaledY /= 131072.0;
        scaledZ = (double)mag_raw_Z - 131072.0;scaledZ /= 131072.0;
        Serial.print("Gauss X,YZ:\t");Serial.print(scaledX * 8, 5);
        Serial.print("\t");Serial.print(scaledY * 8, 5);
        Serial.print("\t");Serial.println(scaledZ * 8, 5);
    }
}

