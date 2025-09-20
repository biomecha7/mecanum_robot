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
    case SupervisorState::PS2_OPEN_LOOP:
      return "PS2_OPEN_LOOP";
    case SupervisorState::PS2_CLOSED_LOOP:
      return "PS2_CLOSED_LOOP";
    case SupervisorState::PI_TELEOP:
      return "PI_TELEOP";
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
  if (strcmp(mode, "PI_TELEOP") == 0) {
    // Request Pi teleop mode - always granted
    _mode_request = ModeRequest::PI_TELEOP;
    _mode_request_time_ms = millis();
    Serial.printf("🔄 Supervisor: Mode request for PI_TELEOP\n");
  } else if (strcmp(mode, "PS2_OPEN_LOOP") == 0) {
    // Request PS2 open loop mode - granted if PS2 is healthy
    _mode_request = ModeRequest::PS2_OPEN_LOOP;
    _mode_request_time_ms = millis();
    Serial.printf("🔄 Supervisor: Mode request for PS2_OPEN_LOOP\n");
  } else if (strcmp(mode, "PS2_CLOSED_LOOP") == 0) {
    // Request PS2 closed loop mode - granted if PS2 is healthy
    _mode_request = ModeRequest::PS2_CLOSED_LOOP;
    _mode_request_time_ms = millis();
    Serial.printf("🔄 Supervisor: Mode request for PS2_CLOSED_LOOP\n");
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
    case SupervisorState::PS2_OPEN_LOOP: return "PS2_OPEN_LOOP";
    case SupervisorState::PS2_CLOSED_LOOP: return "PS2_CLOSED_LOOP";
    case SupervisorState::PI_TELEOP: return "PI_TELEOP";
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
    
    // Debug PS2 status every 2 seconds
    static uint32_t last_ps2_debug = 0;
    if (current_time - last_ps2_debug >= 2000) {
      Serial.printf("🔧 Supervisor Debug: PS2_healthy=%s, Cmd_age=%ums, State=%s\n",
                    ps2_healthy ? "YES" : "NO", 
                    getCmdAgeMs(),
                    stateName());
      last_ps2_debug = current_time;
    }
    
    // Debug state transitions
    static SupervisorState last_state = SupervisorState::IDLE;
    if (_state != last_state) {
      Serial.printf("🔄 Supervisor: %s -> %s\n", 
                    stateNameFromEnum(last_state), stateNameFromEnum(_state));
      last_state = _state;
    }
    
    // Handle mode requests (highest priority after ESTOP)
    if (_mode_request != ModeRequest::NONE) {
      uint32_t request_age = current_time - _mode_request_time_ms;
      
      if (_mode_request == ModeRequest::PI_TELEOP) {
        // Grant PI_TELEOP request immediately (Pi has override authority)
        if (_state != SupervisorState::PI_TELEOP) {
          _state = SupervisorState::PI_TELEOP;
          Serial.printf("✅ Supervisor: Granted PI_TELEOP mode request\n");
        }
        _mode_request = ModeRequest::NONE; // Clear request
      } else if (_mode_request == ModeRequest::PS2_OPEN_LOOP) {
        // Grant PS2_OPEN_LOOP request if PS2 is healthy
        if (ps2_healthy) {
          if (_state != SupervisorState::PS2_OPEN_LOOP) {
            _state = SupervisorState::PS2_OPEN_LOOP;
            Serial.printf("✅ Supervisor: Granted PS2_OPEN_LOOP mode request\n");
          }
          _mode_request = ModeRequest::NONE; // Clear request
        } else if (request_age > 1000) {
          // Request expired
          Serial.printf("⏰ Supervisor: PS2_OPEN_LOOP mode request expired (PS2 not connected)\n");
          _mode_request = ModeRequest::NONE;
        }
      } else if (_mode_request == ModeRequest::PS2_CLOSED_LOOP) {
        // Grant PS2_CLOSED_LOOP request if PS2 is healthy
        if (ps2_healthy) {
          if (_state != SupervisorState::PS2_CLOSED_LOOP) {
            _state = SupervisorState::PS2_CLOSED_LOOP;
            Serial.printf("✅ Supervisor: Granted PS2_CLOSED_LOOP mode request\n");
          }
          _mode_request = ModeRequest::NONE; // Clear request
        } else if (request_age > 1000) {
          // Request expired
          Serial.printf("⏰ Supervisor: PS2_CLOSED_LOOP mode request expired (PS2 not connected)\n");
          _mode_request = ModeRequest::NONE;
        }
      }
    }
    
    // Handle PS2 UP/DOWN buttons (cycle through states)
    static bool last_up_button = false;
    static bool last_down_button = false;
    
    bool up_button = _ps2.getButton(PSXBTN_UP);
    bool down_button = _ps2.getButton(PSXBTN_DOWN);
    
    if (up_button && !last_up_button && ps2_healthy) {
      // UP button: cycle forward through states
      if (_state == SupervisorState::IDLE) {
        _state = SupervisorState::PS2_OPEN_LOOP;
        Serial.printf("🔄 Supervisor: UP button -> PS2_OPEN_LOOP\n");
      } else if (_state == SupervisorState::PS2_OPEN_LOOP) {
        _state = SupervisorState::PS2_CLOSED_LOOP;
        Serial.printf("🔄 Supervisor: UP button -> PS2_CLOSED_LOOP\n");
      } else if (_state == SupervisorState::PS2_CLOSED_LOOP) {
        _state = SupervisorState::IDLE;
        Serial.printf("🔄 Supervisor: UP button -> IDLE\n");
      }
    }
    
    if (down_button && !last_down_button && ps2_healthy) {
      // DOWN button: cycle backward through states
      if (_state == SupervisorState::IDLE) {
        _state = SupervisorState::PS2_CLOSED_LOOP;
        Serial.printf("🔄 Supervisor: DOWN button -> PS2_CLOSED_LOOP\n");
      } else if (_state == SupervisorState::PS2_CLOSED_LOOP) {
        _state = SupervisorState::PS2_OPEN_LOOP;
        Serial.printf("🔄 Supervisor: DOWN button -> PS2_OPEN_LOOP\n");
      } else if (_state == SupervisorState::PS2_OPEN_LOOP) {
        _state = SupervisorState::IDLE;
        Serial.printf("🔄 Supervisor: DOWN button -> IDLE\n");
      }
    }
    
    last_up_button = up_button;
    last_down_button = down_button;
    
    // State machine logic
    switch (_state) {
      case SupervisorState::IDLE:
        // Check for fresh Pi command (auto-transition to PI_TELEOP)
        if (getCmdAgeMs() <= PI_CMD_TIMEOUT_MS) {
          _state = SupervisorState::PI_TELEOP;
        }
        // Set zero command
        _cmd.vx = 0.0f;
        _cmd.vy = 0.0f;
        _cmd.wz = 0.0f;
        _cmd.t_ms = current_time;
        break;
        
      case SupervisorState::PS2_OPEN_LOOP:
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
        
        // Get PS2 commands
        _cmd.vx = _ps2.getVx() * _ps2.getSpeedScale();
        _cmd.vy = _ps2.getVy() * _ps2.getSpeedScale();
        _cmd.wz = _ps2.getWz() * _ps2.getSpeedScale();
        _cmd.t_ms = current_time;
        break;
        
      case SupervisorState::PS2_CLOSED_LOOP:
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
        
        // Get PS2 commands
        _cmd.vx = _ps2.getVx() * _ps2.getSpeedScale();
        _cmd.vy = _ps2.getVy() * _ps2.getSpeedScale();
        _cmd.wz = _ps2.getWz() * _ps2.getSpeedScale();
        _cmd.t_ms = current_time;
        break;
        
      case SupervisorState::PI_TELEOP:
        // Check for emergency stop
        if (_ps2.isEmergencyStop()) {
          _state = SupervisorState::ESTOP;
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
        break;
    }
    
    vTaskDelayUntil(&last, period);
  }
}