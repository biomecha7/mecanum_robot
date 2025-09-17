#include "ControlTask.h"
#include <Arduino.h>

ControlTask::ControlTask(MotionController& mc, PS2Controller& ps2, IMUTask& imu, ISetpointProvider& provider)
  : _mc(mc), _ps2(ps2), _imu(imu), _provider(provider) {
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
    // Update PS2 controller for button handling only
    bool ps2_healthy = _ps2.update();
    
    // Debug PS2 status every 2 seconds
    static uint32_t last_ps2_debug = 0;
    uint32_t now = millis();
    if (now - last_ps2_debug >= 2000) {
      Serial.printf("🔧 PS2 Debug: Connected=%s, VX=%.3f, VY=%.3f, WZ=%.3f, Estop=%s\n",
                    ps2_healthy ? "YES" : "NO",
                    _ps2.getVx(), _ps2.getVy(), _ps2.getWz(),
                    _ps2.isEmergencyStop() ? "YES" : "NO");
      last_ps2_debug = now;
    }
    
    // Handle control mode toggles (only if PS2 is healthy)
    if (ps2_healthy) {
      static bool lastStartButton = false;
      static bool lastR2Button = false;

      bool startButton = _ps2.getButton(PSXBTN_START);
      bool r2Button    = _ps2.getButton(PSXBTN_R2);

      if (startButton && !lastStartButton) {
        _closedLoopEnabled = !_closedLoopEnabled;
        _mc.enablePIDControl(_closedLoopEnabled);
        if (_closedLoopEnabled) {
          _mc.setControlMode(ControlMode::VELOCITY_PID);
          // turn on LED to indicate closed-loop mode
          digitalWrite(LED_PIN_YLW, true);
        } else {
          _mc.setControlMode(ControlMode::OPEN_LOOP);
          // turn off LED to indicate open-loop mode
          digitalWrite(LED_PIN_YLW, false);
        }
      }

      if (r2Button && !lastR2Button) {
        _debugMode = !_debugMode;
      }

      lastStartButton = startButton;
      lastR2Button = r2Button;
    }

    // Update sensors at ~100 Hz
    uint32_t currentTime = millis();
    if (currentTime - _lastSensorUpdate >= 10) {
      _mc.updateSensors();
      _lastSensorUpdate = currentTime;
    }

    // Get setpoint from provider (authoritative source)
    BodyCmd sp = _provider.latest();
    
    // Drive with setpoint from provider
    _mc.drive(sp.vx, sp.vy, sp.wz);

    // Status / debug (unchanged timing)
    if (currentTime - _lastDebugPrint >= 500) {
      if (_debugMode) {
        _mc.printDebugInfo();
        if (_closedLoopEnabled) _mc.printPIDStatus();
      }
      
      // Status information available for subscribers
      // Note: Provider state is published separately by Supervisor
      
      _lastDebugPrint = currentTime;
    }
    vTaskDelayUntil(&last, period);
  }
}