#include "ControlTask.h"
#include <Arduino.h>

ControlTask::ControlTask(MotionController& mc, PS2Controller& ps2, IMUTask& imu)
  : _mc(mc), _ps2(ps2), _imu(imu) {
}

bool ControlTask::initialize() {
  // Initialize LED pin
  pinMode(LED_PIN_YLW, OUTPUT);
  digitalWrite(LED_PIN_YLW, false); // Start with LED off (open-loop mode)
  
  return true;
}

bool ControlTask::start() {
  if (_task != nullptr) {
    return false; // Already started
  }
  
  BaseType_t result = xTaskCreatePinnedToCore(
    taskTrampoline,
    "ControlTask",
    8192,        // Stack size
    this,        // Parameter (this pointer)
    4,           // Priority
    &_task,      // Task handle
    1            // Core 1
  );
  
  return (result == pdPASS);
}

void ControlTask::stop() {
  if (_task != nullptr) {
    vTaskDelete(_task);
    _task = nullptr;
  }
}

void ControlTask::taskTrampoline(void* arg) {
  ControlTask* instance = static_cast<ControlTask*>(arg);
  instance->taskLoop();
}

void ControlTask::taskLoop() {
  const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_HZ);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    // Update controller and check if we have valid data
    if (!_ps2.update()) {
      if (_ps2.isEmergencyStop()) {
        _mc.stop();
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelayUntil(&last, period);
        continue;
      }
      vTaskDelay(pdMS_TO_TICKS(10));   // short yield if no controller
      vTaskDelayUntil(&last, period);
      continue;
    }

    // Emergency stop
    if (_ps2.isEmergencyStop()) {
      _mc.stop();
      vTaskDelay(pdMS_TO_TICKS(100));
      vTaskDelayUntil(&last, period);
      continue;
    }

    // Handle control mode toggles
    static bool lastStartButton = false;
    static bool lastR2Button = false;

    bool startButton = _ps2.getButton(PSXBTN_START);
    bool r2Button    = _ps2.getButton(PSXBTN_R2);

    if (startButton && !lastStartButton) {
      _closedLoopEnabled = !_closedLoopEnabled;
      _mc.enablePIDControl(_closedLoopEnabled);
      if (_closedLoopEnabled) {
        _mc.setControlMode(ControlMode::VELOCITY_PID);
        Serial.println("✅ Closed-loop control ENABLED");
        // turn on LED to indicate closed-loop mode
        digitalWrite(LED_PIN_YLW, true);
      } else {
        _mc.setControlMode(ControlMode::OPEN_LOOP);
        Serial.println("❌ Closed-loop control DISABLED");
        // turn off LED to indicate open-loop mode
        digitalWrite(LED_PIN_YLW, false);
      }
    }

    if (r2Button && !lastR2Button) {
      _debugMode = !_debugMode;
      Serial.println(_debugMode ? "🐛 Debug mode ENABLED" : "🐛 Debug mode DISABLED");
    }

    lastStartButton = startButton;
    lastR2Button = r2Button;

    // Update sensors at ~100 Hz
    uint32_t currentTime = millis();
    if (currentTime - _lastSensorUpdate >= 10) {
      _mc.updateSensors();
      _lastSensorUpdate = currentTime;
    }

    // Drive
    float vx = _ps2.getVx();
    float vy = _ps2.getVy();
    float wz = _ps2.getWz();
    float speed_scale = _ps2.getSpeedScale();

    _mc.drive(vx * speed_scale, vy * speed_scale, wz * speed_scale);

    // Status / debug (unchanged timing)
    if (currentTime - _lastDebugPrint >= 500) {
      if (_debugMode) {
        _mc.printDebugInfo();
        if (_closedLoopEnabled) _mc.printPIDStatus();
        
        // Print IMU data if available
        IMUData imu_data;
        if (_imu.getLatestData(imu_data, 0)) {  // Non-blocking
          Serial.printf("IMU: accel(%.2f,%.2f,%.2f) gyro(%.2f,%.2f,%.2f) temp=%.1f°C\n",
                        imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                        imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                        imu_data.temperature);
        }
      } else {
        Serial.printf("Mode: %s | vx=%.2f vy=%.2f wz=%.2f | speed=%.2f\n",
                      _closedLoopEnabled ? "CLOSED" : "OPEN",
                      vx, vy, wz, speed_scale);
      }
      _lastDebugPrint = currentTime;
    }
    vTaskDelayUntil(&last, period);
  }
}