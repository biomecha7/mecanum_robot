#include <Arduino.h>
#include <Wire.h>
#include "EncoderTask.h"
#include "MotionController.h"
#include "PS2Controller.h"
#include "IMUTask.h"
#include "ControlTask.h"
#include "CommsTask.h"
#include "Supervisor.h"
#include "RobotPins.h"
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---- Global Objects (needed for task lifetime) ----
EncoderTask encoderTask;
MotionController motionController(encoderTask);
PS2Controller ps2Controller;
IMUTask imuTask(IMU_SDA, IMU_SCL);
Supervisor supervisor(ps2Controller);
ControlTask controlTask(motionController, ps2Controller, imuTask, supervisor);
CommsTask commsTask;

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

  // Initialize CommsTask FIRST (needed for other tasks)
  Serial.println("Initializing CommsTask...");
  if (!commsTask.initialize()) {
    Serial.println("❌ CommsTask initialization failed");
    return; // Cannot continue without communication
  } else if (!commsTask.start()) {
    Serial.println("❌ CommsTask start failed");
    return; // Cannot continue without communication
  } else {
    Serial.println("✅ CommsTask started successfully");
  }

  // Initialize encoder task (MotionController depends on it)
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
  
  // Subscribe CommsTask to EncoderTask
  commsTask.subscribeToEncoderTask(encoderTask);

  // Initialize motion controller (handles PWM setup internally)
  Serial.println("Initializing motion controller...");
  motionController.initialize();
  
  // Initialize PS2 controller
  ps2Controller.initialize();
  
  // Initialize Supervisor (state machine)
  Serial.println("Initializing Supervisor...");
  if (!supervisor.initialize()) {
    Serial.println("❌ Supervisor initialization failed");
    return;
  } else if (!supervisor.start()) {
    Serial.println("❌ Supervisor start failed");
    return;
  } else {
    Serial.println("✅ Supervisor started successfully");
  }
  
  // Subscribe CommsTask to Supervisor
  commsTask.subscribeToSupervisor(supervisor);
  
  // Initialize IMU task
  Serial.println("Initializing IMU task...");
  if (!imuTask.initialize()) {
    Serial.println("❌ IMU task initialization failed");
  } else if (!imuTask.start()) {
    Serial.println("❌ IMU task start failed");
  } else {
    Serial.println("✅ IMU task started successfully");
  }
  
  // Subscribe CommsTask to IMUTask
  commsTask.subscribeToIMUTask(imuTask);
  
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
  
  Serial.println("✅ Setup complete. Robot ready for operation!");
  Serial.println("🎮 Use joysticks to control movement");
  Serial.println("🎯 Press START to enable closed-loop control");
  Serial.println("🛑 Press SELECT for emergency stop");
}

void loop() {
  // empty - all work is done in FreeRTOS tasks
}