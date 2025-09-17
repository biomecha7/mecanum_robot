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
    case SupervisorState::TELEOP_PI:
      return "TELEOP_PI";
    case SupervisorState::ESTOP:
      return "ESTOP";
    default:
      return "UNKNOWN";
  }
}

void Supervisor::feedPiCmd(float vx, float vy, float wz, uint32_t t_ms) {
  _pi_cmd.vx = vx;
  _pi_cmd.vy = vy;
  _pi_cmd.wz = wz;
  _pi_cmd.t_ms = t_ms;
  _pi_cmd.last_received_ms = millis();
}

void Supervisor::requestMode(const char* mode) {
  if (strcmp(mode, "TELEOP_PI") == 0) {
    // Request teleop mode - will be granted if Pi command is fresh
    // State machine will handle the transition
  } else if (strcmp(mode, "MANUAL_PS2") == 0) {
    // Request manual mode - will be granted if PS2 is healthy
    // State machine will handle the transition
  }
}

uint32_t Supervisor::getCmdAgeMs() const {
  uint32_t current_time = millis();
  if (_pi_cmd.last_received_ms == 0) {
    return UINT32_MAX; // No command received yet
  }
  return current_time - _pi_cmd.last_received_ms;
}

void Supervisor::taskTrampoline(void* arg) {
  Supervisor* instance = static_cast<Supervisor*>(arg);
  instance->taskLoop();
}

const char* Supervisor::stateNameFromEnum(SupervisorState state) {
  switch (state) {
    case SupervisorState::IDLE: return "IDLE";
    case SupervisorState::MANUAL_PS2: return "MANUAL_PS2";
    case SupervisorState::TELEOP_PI: return "TELEOP_PI";
    case SupervisorState::ESTOP: return "ESTOP";
    default: return "UNKNOWN";
  }
}

void Supervisor::taskLoop() {
  const TickType_t period = pdMS_TO_TICKS(1000 / SUPERVISOR_HZ);
  TickType_t last = xTaskGetTickCount();
  
  uint32_t current_time = millis();
  
  for (;;) {
    current_time = millis();
    
    // Update PS2 controller
    bool ps2_healthy = _ps2.update();
    
    // Debug state transitions
    static SupervisorState last_state = SupervisorState::IDLE;
    if (_state != last_state) {
      Serial.printf("🔄 Supervisor: %s -> %s\n", 
                    stateNameFromEnum(last_state), stateNameFromEnum(_state));
      last_state = _state;
    }
    
    // State machine logic
    switch (_state) {
      case SupervisorState::IDLE:
        // Check for PS2 connection
        if (ps2_healthy) {
          _state = SupervisorState::MANUAL_PS2;
        }
        // Check for fresh Pi command
        else if (getCmdAgeMs() <= PI_CMD_TIMEOUT_MS) {
          _state = SupervisorState::TELEOP_PI;
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
        
      case SupervisorState::TELEOP_PI:
        // Check for emergency stop
        if (_ps2.isEmergencyStop()) {
          _state = SupervisorState::ESTOP;
          break;
        }
        
        // Check for PS2 connection (higher priority)
        if (ps2_healthy) {
          _state = SupervisorState::MANUAL_PS2;
          break;
        }
        
        // Check for Pi command freshness
        if (getCmdAgeMs() > PI_CMD_TIMEOUT_MS) {
          _state = SupervisorState::IDLE;
          break;
        }
        
        // Use Pi command
        _cmd.vx = _pi_cmd.vx;
        _cmd.vy = _pi_cmd.vy;
        _cmd.wz = _pi_cmd.wz;
        _cmd.t_ms = current_time;
        break;
        
      case SupervisorState::ESTOP:
        // Emergency stop - zero command
        _cmd.vx = 0.0f;
        _cmd.vy = 0.0f;
        _cmd.wz = 0.0f;
        _cmd.t_ms = current_time;
        
        // Check if ESTOP has been cleared
        if (!_ps2.isEmergencyStop()) {
          _state = SupervisorState::IDLE;
          break;
        }
        
        // Check for PS2 disconnection to clear ESTOP (fallback)
        if (!ps2_healthy) {
          _state = SupervisorState::IDLE;
        }
        // Note: ESTOP can now be cleared by SELECT+START button combination
        break;
    }
    
    vTaskDelayUntil(&last, period);
  }
}