#include <Arduino.h>
#include <Wire.h>
#include "MotionController.h"
#include "PS2Controller.h"
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---- Globals ----
MotionController motionController;
PS2Controller ps2Controller;

static const gpio_num_t LED_PIN = GPIO_NUM_35; // On-board LED for ESP32

// 1 Hz heartbeat task
void HeartbeatTask(void*) {
  pinMode(LED_PIN, OUTPUT);
  const TickType_t period = pdMS_TO_TICKS(500); // 500ms toggle
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    // toggle LED
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    // JSON heartbeat
    StaticJsonDocument<64> doc;
    doc["heartbeat"] = millis();  // Send current uptime in ms
    serializeJson(doc, Serial);
    Serial.println(); // newline delimiter

    vTaskDelayUntil(&last, period);
  }
}

// ---- Control State ----
bool closedLoopEnabled = false;
bool debugMode = false;
uint32_t lastDebugPrint = 0;
uint32_t lastSensorUpdate = 0;

// ---- Emergency Stop ----
void emergencyStop() {
  motionController.stop();
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("🤖 Mecanum Robot Controller - ENHANCED");
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

  // Initialize motion controller (handles PWM setup internally)
  Serial.println("Initializing motion controller...");
  motionController.initialize();
  
  // Initialize PS2 controller
  ps2Controller.initialize();
  
  // Set deadband for joystick mapping
  ps2Controller.setDeadband(motionController.getDeadband());
  
  // Start with open-loop control
  motionController.setControlMode(ControlMode::OPEN_LOOP);
  motionController.enablePIDControl(false);
  
  Serial.println("Setup complete. Ready to drive!");
  Serial.println("Press START to enable closed-loop control");
  
  // Start heartbeat task
  xTaskCreate(HeartbeatTask, "HeartbeatTask", 4096, nullptr, 1, nullptr);
}

// ---- Main Control Loop ----
void loop() {
  // Update controller and check if we have valid data
  if (!ps2Controller.update()) {
    // Controller not connected or emergency stop
    if (ps2Controller.isEmergencyStop()) {
      emergencyStop();
      delay(100);
      return;
    }
    delay(10);
    return;
  }
  
  // Check for emergency stop
  if (ps2Controller.isEmergencyStop()) {
    emergencyStop();
    delay(100);
    return;
  }
  
  // Handle control mode toggles
  static bool lastStartButton = false;
  static bool lastR2Button = false;
  
  bool startButton = ps2Controller.getButton(PSXBTN_START);
  bool r2Button = ps2Controller.getButton(PSXBTN_R2);
  
  // Toggle closed-loop control
  if (startButton && !lastStartButton) {
    closedLoopEnabled = !closedLoopEnabled;
    motionController.enablePIDControl(closedLoopEnabled);
    
    if (closedLoopEnabled) {
      motionController.setControlMode(ControlMode::VELOCITY_PID);
      Serial.println("✅ Closed-loop control ENABLED");
    } else {
      motionController.setControlMode(ControlMode::OPEN_LOOP);
      Serial.println("❌ Closed-loop control DISABLED");
    }
  }
  
  // Toggle debug mode
  if (r2Button && !lastR2Button) {
    debugMode = !debugMode;
    Serial.println(debugMode ? "🐛 Debug mode ENABLED" : "🐛 Debug mode DISABLED");
  }
  
  lastStartButton = startButton;
  lastR2Button = r2Button;
  
  // Update sensors at high rate
  uint32_t currentTime = millis();
  if (currentTime - lastSensorUpdate >= 10) { // 100Hz sensor update
    motionController.updateSensors();
    lastSensorUpdate = currentTime;
  }
  
  // Get control values from controller
  float vx = ps2Controller.getVx();
  float vy = ps2Controller.getVy();
  float wz = ps2Controller.getWz();
  float speed_scale = ps2Controller.getSpeedScale();

  // Drive using MotionController
  if (closedLoopEnabled) {
    // Use closed-loop control
    motionController.drive(vx * speed_scale, vy * speed_scale, wz * speed_scale);
  } else {
    // Use open-loop control (original behavior)
    motionController.drive(vx * speed_scale, vy * speed_scale, wz * speed_scale);
  }

  // ---- Status Reporting ----
  if (currentTime - lastDebugPrint >= 500) {  // Every 500ms
    if (debugMode) {
      motionController.printDebugInfo();
      if (closedLoopEnabled) {
        motionController.printPIDStatus();
      }
    } else {
      // Basic status
      Serial.printf("Mode: %s | vx=%.2f vy=%.2f wz=%.2f | speed=%.2f\n",
                    closedLoopEnabled ? "CLOSED" : "OPEN",
                    vx, vy, wz, speed_scale);
    }
    lastDebugPrint = currentTime;
  }

  delay(10);  // 100Hz control loop
}