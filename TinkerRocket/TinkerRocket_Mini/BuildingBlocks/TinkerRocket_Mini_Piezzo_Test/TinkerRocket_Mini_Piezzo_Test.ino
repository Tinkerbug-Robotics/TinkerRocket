const uint8_t PIEZO_PIN = 53;


void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting...");

    pinMode(PIEZO_PIN, OUTPUT);
    digitalWrite(PIEZO_PIN, LOW);
}

void loop()
{
  // 2 kHz tone for 200 ms
  tone(PIEZO_PIN, 2000);
  delay(200);

  // silence
  noTone(PIEZO_PIN);
  delay(800);
}
