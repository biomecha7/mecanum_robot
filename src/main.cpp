#include <Arduino.h>
#include "MotionController.h"
#include "PS2Controller.h"
#include "ControlTask.h"
#include "Supervisor.h"
#include "OledDisplay.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

MotionController motionController;
PS2Controller ps2Controller;
Supervisor supervisor(ps2Controller);
ControlTask controlTask(motionController, supervisor);

static void oledKeepAliveTask(void*) {
  for (;;) {
    Oled::service();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("Mecanum Robot — open-loop");
  Serial.println("  START: Arm / Disarm (boots DISARMED)");
  Serial.println("  Sticks / D-pad: drive (buzzes if disarmed)");
  Serial.println("  START+TRIANGLE: ESTOP");
  Serial.println("  SELECT+START: Clear ESTOP");
  Serial.println("========================================");

  // OLED before motors so Vext is held before LEDC attaches
  Oled::begin();
  xTaskCreatePinnedToCore(oledKeepAliveTask, "OledKeep", 3072, nullptr, 1, nullptr, 0);

  motionController.initialize();
  Oled::service();

  ps2Controller.initialize();
  ps2Controller.setDeadband(motionController.getDeadband());

  if (!supervisor.initialize() || !supervisor.start()) {
    Serial.println("❌ Supervisor failed");
    return;
  }

  if (!controlTask.initialize() || !controlTask.start()) {
    Serial.println("❌ ControlTask failed");
    return;
  }

  Serial.println("✅ Ready — press START to arm");
  Serial.println("ℹ️  Keep GPIO 36 free — it is OLED Vext_Ctrl");
}

void loop() {
  delay(1000);
}
