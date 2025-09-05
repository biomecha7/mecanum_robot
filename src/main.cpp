#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1000); // give USB time to settle
  Serial.println("Hello World from Heltec WiFi LoRa 32 V3!");
}

void loop() {
  Serial.println("Looping...");
  delay(1000); // print every second
}