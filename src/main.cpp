#include <Arduino.h>
#include <Wire.h>
#include "EncoderTask.h"
#include "MotionController.h"
#include "PS2Controller.h"
#include "IMUTask.h"
#include "ControlTask.h"
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// FOR DEBUGGING and STATUS
static const gpio_num_t LED_PIN_RED = GPIO_NUM_42; // On-board LED for ESP32
static const gpio_num_t LED_PIN_GRN = GPIO_NUM_41; // On-board LED for ESP32
static const gpio_num_t LED_PIN_YLW = GPIO_NUM_40; // On-board LED for ESP32


// ---- Setup ----
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("🤖 Mecanum Robot Controller - ENHANCED");
  
  // Create objects locally
  EncoderTask encoderTask;
  MotionController motionController(encoderTask);
  PS2Controller ps2Controller;
  IMUTask imuTask(IMU_SDA, IMU_SCL);
  ControlTask controlTask(motionController, ps2Controller, imuTask);
  
  Serial.println("Wheelbase: " + String(motionController.getWheelbaseInches()) + "\" square");
  Serial.println("Wheel Diameter: " + String(motionController.getWheelDiameterMm()) + "mm");
  Serial.println("========================================");
  Serial.println("🎮 CONTROLS:");
  Serial.println("  L1: Slow mode (35%)");
  Serial.println("  R1: Fast mode (100%)");
  Serial.println("  L2: Medium mode (50%)");
  Serial.println("  SELECT: Emergency Stop");
  Serial.println("  START: Toggle Closed-Loop Control");
  Serial.println("  R2: Toggle Debug Mode");
  Serial.println("========================================");

  // Initialize encoder task FIRST (MotionController depends on it)
  Serial.println("Initializing encoder task...");
  if (!encoderTask.initialize()) {
    Serial.println("❌ Encoder task initialization failed");
    return; // Cannot continue without encoders
  } else if (!encoderTask.start()) {
    Serial.println("❌ Encoder task start failed");
    return; // Cannot continue without encoders
  } else {
    Serial.println("✅ Encoder task started successfully");
  }

  // Initialize motion controller (handles PWM setup internally)
  Serial.println("Initializing motion controller...");
  motionController.initialize();
  
  // Initialize PS2 controller
  ps2Controller.initialize();
  
  // Initialize IMU task
  Serial.println("Initializing IMU task...");
  if (!imuTask.initialize()) {
    Serial.println("❌ IMU task initialization failed");
  } else if (!imuTask.start()) {
    Serial.println("❌ IMU task start failed");
  } else {
    Serial.println("✅ IMU task started successfully");
  }
  
  // Initialize control task
  Serial.println("Initializing control task...");
  if (!controlTask.initialize()) {
    Serial.println("❌ Control task initialization failed");
    return;
  } else if (!controlTask.start()) {
    Serial.println("❌ Control task start failed");
    return;
  } else {
    Serial.println("✅ Control task started successfully");
  }
  
  // Set deadband for joystick mapping
  ps2Controller.setDeadband(motionController.getDeadband());
  
  // Start with open-loop control
  motionController.setControlMode(ControlMode::OPEN_LOOP);
  motionController.enablePIDControl(false);
  
  Serial.println("Setup complete. Ready to drive!");
  Serial.println("Press START to enable closed-loop control");
}

void loop() {
  // empty - all work is done in FreeRTOS tasks
}