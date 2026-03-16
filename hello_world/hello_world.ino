void setup() {
  delay(500);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 3000) { delay(10); } // timeout
  Serial.println("Booting...");
}

void loop() {
  Serial.println("Hello World ...");
  delay(1000);
}
