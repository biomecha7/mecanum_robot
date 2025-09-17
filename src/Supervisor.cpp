#include "Supervisor.h"
#include <Arduino.h>

Supervisor::Supervisor(PS2Controller& ps2) : _ps2(ps2) {
}

bool Supervisor::initialize() {
  // Initialize command to zero
  _cmd.vx = 0.0f;
  _cmd.vy = 0.0f;
  _cmd.wz = 0.0f;
  _cmd.t_ms = 0;
  
  return true;
}

bool Supervisor::start() {
  if (_task != nullptr) {
    return false; // Already started
  }
  
  BaseType_t result = xTaskCreatePinnedToCore(
    taskTrampoline,
    "Supervisor",
    4096,        // Stack size
    this,        // Parameter (this pointer)
    3,           // Priority (between control and comms)
    &_task,      // Task handle
    0            // Core 0
  );
  
  return (result == pdPASS);
}

void Supervisor::stop() {
  if (_task != nullptr) {
    vTaskDelete(_task);
    _task = nullptr;
  }
}

BodyCmd Supervisor::latest() const {
  // Return a copy of the current command (atomic read)
  BodyCmd cmd;
  cmd.vx = _cmd.vx;
  cmd.vy = _cmd.vy;
  cmd.wz = _cmd.wz;
  cmd.t_ms = _cmd.t_ms;
  return cmd;
}

const char* Supervisor::stateName() const {
  switch (_state) {
    case SupervisorState::IDLE:
      return "IDLE";
    case SupervisorState::MANUAL_PS2:
      return "MANUAL_PS2";
    case SupervisorState::ESTOP:
      return "ESTOP";
    default:
      return "UNKNOWN";
  }
}

void Supervisor::taskTrampoline(void* arg) {
  Supervisor* instance = static_cast<Supervisor*>(arg);
  instance->taskLoop();
}

void Supervisor::taskLoop() {
  const TickType_t period = pdMS_TO_TICKS(1000 / SUPERVISOR_HZ);
  TickType_t last = xTaskGetTickCount();
  
  uint32_t current_time = millis();
  
  for (;;) {
    current_time = millis();
    
    // Update PS2 controller
    bool ps2_healthy = _ps2.update();
    
    // State machine logic
    switch (_state) {
      case SupervisorState::IDLE:
        // Check for PS2 connection
        if (ps2_healthy) {
          _state = SupervisorState::MANUAL_PS2;
        }
        // Set zero command
        _cmd.vx = 0.0f;
        _cmd.vy = 0.0f;
        _cmd.wz = 0.0f;
        _cmd.t_ms = current_time;
        break;
        
      case SupervisorState::MANUAL_PS2:
        // Check for emergency stop
        if (_ps2.isEmergencyStop()) {
          _state = SupervisorState::ESTOP;
          break;
        }
        
        // Check for PS2 disconnection
        if (!ps2_healthy) {
          _state = SupervisorState::IDLE;
          break;
        }
        
        // Get PS2 commands (same math as before)
        _cmd.vx = _ps2.getVx() * _ps2.getSpeedScale();
        _cmd.vy = _ps2.getVy() * _ps2.getSpeedScale();
        _cmd.wz = _ps2.getWz() * _ps2.getSpeedScale();
        _cmd.t_ms = current_time;
        break;
        
      case SupervisorState::ESTOP:
        // Emergency stop - zero command
        _cmd.vx = 0.0f;
        _cmd.vy = 0.0f;
        _cmd.wz = 0.0f;
        _cmd.t_ms = current_time;
        
        // Check for PS2 disconnection to clear ESTOP
        if (!ps2_healthy) {
          _state = SupervisorState::IDLE;
        }
        // Note: ESTOP remains latched until PS2 is disconnected
        break;
    }
    
    vTaskDelayUntil(&last, period);
  }
}