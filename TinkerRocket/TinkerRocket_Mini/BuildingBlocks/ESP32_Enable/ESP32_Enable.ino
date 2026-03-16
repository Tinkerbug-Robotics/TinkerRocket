#include <Arduino.h>

const int PWR_PIN = 6;

void setup() 
{
    Serial.begin(115200);

    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);

}

void loop()
{
    Serial.println("ESP32-S3");
    delay(1000);

}
