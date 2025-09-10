#include <Arduino.h>
#include <Wire.h>
#include "MotionController.h"
#include "PS2Controller.h"

// ---- Globals ----
MotionController motionController;
PS2Controller ps2Controller;

// ---- Emergency Stop ----
void emergencyStop() {
  motionController.stop();
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("🤖 Mecanum Robot Controller - DRIVE MODE");
  Serial.println("Wheelbase: " + String(motionController.getWheelbaseInches()) + "\" square");
  Serial.println("Wheel Diameter: " + String(motionController.getWheelDiameterMm()) + "mm");
  Serial.println("========================================");
  Serial.println("🎮 CONTROLS:");
  Serial.println("  L1: Slow mode (35%)");
  Serial.println("  R1: Fast mode (100%)");
  Serial.println("  L2: Medium mode (50%)");
  Serial.println("  SELECT: Emergency Stop");
  Serial.println("========================================");

  // Initialize motion controller (handles PWM setup internally)
  Serial.println("Initializing motion controller...");
  motionController.initialize();
  
  // Initialize PS2 controller
  ps2Controller.initialize();
  
  // Set deadband for joystick mapping
  ps2Controller.setDeadband(motionController.getDeadband());
  
  Serial.println("Setup complete. Ready to drive!");
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
  
  // Get control values from controller
  float vx = ps2Controller.getVx();
  float vy = ps2Controller.getVy();
  float wz = ps2Controller.getWz();
  float speed_scale = ps2Controller.getSpeedScale();

  // Drive using MotionController
  motionController.drive(vx * speed_scale, vy * speed_scale, wz * speed_scale);

  // ---- Status Reporting ----
  static uint32_t last_status = 0;
  uint32_t now = millis();
  if (now - last_status > 500) {  // Every 500ms
    Serial.printf("vx=%.2f vy=%.2f wz=%.2f | speed=%.2f\n",
                  vx, vy, wz, speed_scale);
    last_status = now;
  }

  delay(10);  // 100Hz control loop
}
