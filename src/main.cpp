#include <Arduino.h>
#include <Wire.h>
#include "EncoderTask.h"
#include "MotionController.h"
#include "PS2Controller.h"
#include "IMUTask.h"
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---- Globals ----
EncoderTask encoderTask;
MotionController motionController(encoderTask);
PS2Controller ps2Controller;
IMUTask imuTask(IMU_SDA, IMU_SCL);

// FOR DEBUGGING and STATUS
static const gpio_num_t LED_PIN_RED = GPIO_NUM_42; // On-board LED for ESP32
static const gpio_num_t LED_PIN_GRN = GPIO_NUM_41; // On-board LED for ESP32
static const gpio_num_t LED_PIN_YLW = GPIO_NUM_40; // On-board LED for ESP32

// ---- Control State ----
bool closedLoopEnabled = false;
bool debugMode = false;
uint32_t lastDebugPrint = 0;
uint32_t lastSensorUpdate = 0;

// ---- Emergency Stop ----
void emergencyStop() {
  motionController.stop();
}

static const int CONTROL_HZ = 100; // Control loop frequency

void ControlTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_HZ);
  TickType_t last = xTaskGetTickCount();

  pinMode(LED_PIN_YLW, OUTPUT);

  for (;;) {
    // Update controller and check if we have valid data
    if (!ps2Controller.update()) {
      if (ps2Controller.isEmergencyStop()) {
        emergencyStop();
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelayUntil(&last, period);
        continue;
      }
      vTaskDelay(pdMS_TO_TICKS(10));   // short yield if no controller
      vTaskDelayUntil(&last, period);
      continue;
    }

    // Emergency stop
    if (ps2Controller.isEmergencyStop()) {
      emergencyStop();
      vTaskDelay(pdMS_TO_TICKS(100));
      vTaskDelayUntil(&last, period);
      continue;
    }

    // Handle control mode toggles
    static bool lastStartButton = false;
    static bool lastR2Button = false;

    bool startButton = ps2Controller.getButton(PSXBTN_START);
    bool r2Button    = ps2Controller.getButton(PSXBTN_R2);

    if (startButton && !lastStartButton) {
      closedLoopEnabled = !closedLoopEnabled;
      motionController.enablePIDControl(closedLoopEnabled);
      if (closedLoopEnabled) {
        motionController.setControlMode(ControlMode::VELOCITY_PID);
        Serial.println("✅ Closed-loop control ENABLED");
        // turn on LED to indicate closed-loop mode
        digitalWrite(LED_PIN_YLW, true);
      } else {
        motionController.setControlMode(ControlMode::OPEN_LOOP);
        Serial.println("❌ Closed-loop control DISABLED");
        // turn off LED to indicate open-loop mode
        digitalWrite(LED_PIN_YLW, false);
      }
    }

    if (r2Button && !lastR2Button) {
      debugMode = !debugMode;
      Serial.println(debugMode ? "🐛 Debug mode ENABLED" : "🐛 Debug mode DISABLED");
    }

    lastStartButton = startButton;
    lastR2Button = r2Button;

    // Update sensors at ~100 Hz
    uint32_t currentTime = millis();
    if (currentTime - lastSensorUpdate >= 10) {
      motionController.updateSensors();
      lastSensorUpdate = currentTime;
    }

    // Drive
    float vx = ps2Controller.getVx();
    float vy = ps2Controller.getVy();
    float wz = ps2Controller.getWz();
    float speed_scale = ps2Controller.getSpeedScale();

    motionController.drive(vx * speed_scale, vy * speed_scale, wz * speed_scale);

    // Status / debug (unchanged timing)
    if (currentTime - lastDebugPrint >= 500) {
      if (debugMode) {
        motionController.printDebugInfo();
        if (closedLoopEnabled) motionController.printPIDStatus();
        
        // Print IMU data if available
        IMUData imu_data;
        if (imuTask.getLatestData(imu_data, 0)) {  // Non-blocking
          Serial.printf("IMU: accel(%.2f,%.2f,%.2f) gyro(%.2f,%.2f,%.2f) temp=%.1f°C\n",
                        imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                        imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                        imu_data.temperature);
        }
      } else {
        Serial.printf("Mode: %s | vx=%.2f vy=%.2f wz=%.2f | speed=%.2f\n",
                      closedLoopEnabled ? "CLOSED" : "OPEN",
                      vx, vy, wz, speed_scale);
      }
      lastDebugPrint = currentTime;
    }
    vTaskDelayUntil(&last, period);
  }
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
  
  // Set deadband for joystick mapping
  ps2Controller.setDeadband(motionController.getDeadband());
  
  // Start with open-loop control
  motionController.setControlMode(ControlMode::OPEN_LOOP);
  motionController.enablePIDControl(false);
  
  Serial.println("Setup complete. Ready to drive!");
  Serial.println("Press START to enable closed-loop control");
  
  // Start control task
  xTaskCreatePinnedToCore(ControlTask, "ControlTask", 8192, nullptr, 4, nullptr, 1); // priority 4 on core 1
}

void loop() {
  // empty - all work is done in FreeRTOS tasks
}