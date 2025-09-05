#include <Arduino.h>
#include <PSX.h>   // using your vendored lib in lib/ArduinoPSX/src

// Heltec WiFi LoRa 32 V3 safe pins (avoid LoRa/OLED/USB)
#define PIN_PS2_ATT  5   // ATT / Select (yellow)
#define PIN_PS2_CLK  7   // CLK (blue)
#define PIN_PS2_CMD  6   // CMD (orange)  ESP -> Pad
#define PIN_PS2_DAT  4   // DAT (brown)   Pad -> ESP

PSX psx;                 // default constructor
PSX::PSXDATA state;      // holds buttons + sticks

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[PS2] init (ArduinoPSX v1.0.1)…");

  // NOTE: setupPins order in your lib is (data, cmd, att, clock, delay_us)
  psx.setupPins(PIN_PS2_DAT, PIN_PS2_CMD, PIN_PS2_ATT, PIN_PS2_CLK, 10);

  // Put controller into analog mode (also disables vibration per your lib)
  psx.config(PSXMODE_ANALOG);

  Serial.println("[PS2] Config requested: ANALOG mode");
}

static void printButtons(uint16_t b) {
  // In this library, bits are set when a button is PRESSED (see PSX.cpp)
  if (b & PSXBTN_START)    Serial.println("START");
  if (b & PSXBTN_SELECT)   Serial.println("SELECT");

  if (b & PSXBTN_UP)       Serial.println("DPAD_UP");
  if (b & PSXBTN_RIGHT)    Serial.println("DPAD_RIGHT");
  if (b & PSXBTN_DOWN)     Serial.println("DPAD_DOWN");
  if (b & PSXBTN_LEFT)     Serial.println("DPAD_LEFT");

  if (b & PSXBTN_TRIANGLE) Serial.println("TRIANGLE");
  if (b & PSXBTN_CIRCLE)   Serial.println("CIRCLE");
  if (b & PSXBTN_CROSS)    Serial.println("CROSS");
  if (b & PSXBTN_SQUARE)   Serial.println("SQUARE");

  if (b & PSXBTN_L1) Serial.println("L1");
  if (b & PSXBTN_R1) Serial.println("R1");
  if (b & PSXBTN_L2) Serial.println("L2");
  if (b & PSXBTN_R2) Serial.println("R2");
}

void loop() {
  // Read the controller into `state`
  int err = psx.read(state);
  if (err == PSXERROR_SUCCESS) {
    printButtons(state.buttons);

    static uint32_t tLog = 0;
    if (millis() - tLog > 200) {
      // Sticks are 0..255, ~128 centered
      Serial.printf("LX=%3u  LY=%3u  RX=%3u  RY=%3u\n",
        state.JoyLeftX, state.JoyLeftY, state.JoyRightX, state.JoyRightY);
      tLog = millis();
    }
  } else {
    // No data (PSXERROR_NODATA): brief pause to avoid spam
    static uint32_t tErr = 0;
    if (millis() - tErr > 1000) {
      Serial.println("[PS2] No data (check wiring/power).");
      tErr = millis();
    }
  }

  delay(10);
}