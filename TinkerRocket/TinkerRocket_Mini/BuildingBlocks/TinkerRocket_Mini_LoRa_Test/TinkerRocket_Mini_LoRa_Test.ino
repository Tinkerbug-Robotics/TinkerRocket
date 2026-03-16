#include <RadioLib.h>

const int PWR_PIN = 6;


// LoRa Radio (LLC68)
static constexpr uint8_t  L_DIO1 = 9;
static constexpr uint8_t  L_CS   = 18;
static constexpr uint8_t  L_BUSY = 13;
static constexpr uint8_t  L_RST  = 17;
static constexpr uint8_t  L_MISO = 11;
static constexpr uint8_t  L_MOSI = 12;
static constexpr uint8_t  L_SCK  = 14;

// LoRa Radio Configuration
static constexpr float   FREQUENCY        = 905.0;
static constexpr float   BANDWIDTH        = 250.0;
static constexpr uint8_t SPREADING_FACTOR = 8;
static constexpr uint8_t CODING_RATE      = 5;
static constexpr uint8_t OUTPUT_POWER     = 12;
static constexpr uint8_t PREAMBLE_LENGTH  = 12;

volatile bool opDone = false;   // DIO1 ISR flag

// ---------- LoRa radio ----------
LLCC68 radio = LLCC68(new Module(L_CS, 
                                 L_DIO1,
                                 L_RST,
                                 L_BUSY,
                                 SPI));

// ---------- LoRa ISR ----------
void IRAM_ATTR dio1ISR()
{
    opDone = true;
}

// Simple payload: 20 bytes
uint8_t pkt[20];

// Optional: simple sequence counter
uint32_t seq = 0;

void setup()
{

    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting...");

    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(100);

    // Start SPI for LoRa radio
    SPI.begin(L_SCK, L_MISO, L_MOSI, L_CS);

    // Initialize GPIO pins for the radio
    pinMode(L_BUSY, INPUT);
    pinMode(L_DIO1, INPUT);
    pinMode(L_RST, OUTPUT);

       // LoRa reset sequence
    digitalWrite(L_RST, LOW);
    delay(100);
    digitalWrite(L_RST, HIGH);
    delay(100);

    // RF switch via DIO2 (switch is internal to radio module)
    radio.setDio2AsRfSwitch(true);

    Serial.print("LoRa radio initializing ... ");

    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print(F("Radio init failed, code: "));
        Serial.println(state);
        while(1) delay(500);
    }

    if ((state = radio.setFrequency(FREQUENCY)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setFrequency failed");
        while(1) delay(100);
    }
    if ((state = radio.setBandwidth(BANDWIDTH)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setBandwidth failed");
        while(1) delay(100);
    }
    if ((state = radio.setSpreadingFactor(SPREADING_FACTOR)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setSpreadingFactor failed");
        while(1) delay(100);
    }
    if ((state = radio.setCodingRate(CODING_RATE)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setCodingRate failed");
        while(1) delay(100);
    }
    if ((state = radio.setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setSyncWord failed");
        while(1) delay(100);
    }
    if ((state = radio.setOutputPower(OUTPUT_POWER)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setOutputPower failed");
        while(1) delay(100);
    }
    if ((state = radio.setPreambleLength(PREAMBLE_LENGTH)) != RADIOLIB_ERR_NONE)
    {
        Serial.println("setPreambleLength failed");
        while(1) delay(100);
    }
    Serial.println(" radio initialized successfully");

    // Non-blocking TX done callback
    radio.setDio1Action(dio1ISR);

    // Build an initial packet pattern
    for (int i = 0; i < 20; i++) pkt[i] = (uint8_t)i;
}

void loop() {
    // Fill the 20-byte payload with arbitrary-but-changing data:
    // bytes 0..3 = seq (little endian), bytes 4..19 = pattern
    pkt[0] = (uint8_t)(seq >> 0);
    pkt[1] = (uint8_t)(seq >> 8);
    pkt[2] = (uint8_t)(seq >> 16);
    pkt[3] = (uint8_t)(seq >> 24);
    for (int i = 4; i < 20; i++) {
        pkt[i] = (uint8_t)(0xA0 + ((seq + i) & 0x3F)); // arbitrary changing bytes
    }

    opDone = false;

    // Start async TX (DIO1 will fire when done)
    int state = radio.startTransmit(pkt, sizeof(pkt));
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("startTransmit failed, code: ");
        Serial.println(state);
        delay(200);
        return;
    }

    // Wait for DIO1 TX done (with a timeout)
    uint32_t t0 = millis();
    while (!opDone && (millis() - t0 < 2000)) {
        delay(1);
    }

    // Always stop TX (cleans up RadioLib state)
    radio.finishTransmit();

    if (!opDone) {
        Serial.println("TX timeout (no DIO1?)");
    } else {
        Serial.print("TX ok, seq=");
        Serial.print(seq);
        Serial.print("  len=");
        Serial.print(sizeof(pkt));
        Serial.print("  RSSI=");
        Serial.print(radio.getRSSI());
        Serial.print("  SNR=");
        Serial.println(radio.getSNR());
    }

    seq++;

    // Small gap between packets (adjust as needed)
    delay(2000);
}
