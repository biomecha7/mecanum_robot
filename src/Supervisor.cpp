#include "Supervisor.h"
#include <Arduino.h>

Supervisor::Supervisor(PS2Controller& ps2) : _ps2(ps2) {}

bool Supervisor::initialize() {
  _cmd = {};
  pinMode(LED_PIN_GRN, OUTPUT);
  setArmedLed(false);
  _state = SupervisorState::DISARMED;
  return true;
}

bool Supervisor::start() {
  if (_task != nullptr) return false;

  return xTaskCreatePinnedToCore(
           taskTrampoline, "Supervisor", 4096, this, 3, &_task, 0) == pdPASS;
}

void Supervisor::stop() {
  if (_task != nullptr) {
    vTaskDelete(_task);
    _task = nullptr;
  }
}

BodyCmd Supervisor::latest() const {
  return _cmd;
}

const char* Supervisor::stateName() const {
  switch (_state) {
    case SupervisorState::DISARMED: return "DISARMED";
    case SupervisorState::ARMED:    return "ARMED";
    case SupervisorState::ESTOP:    return "ESTOP";
    default:                        return "UNKNOWN";
  }
}

void Supervisor::setArmedLed(bool on) {
  digitalWrite(LED_PIN_GRN, on ? HIGH : LOW);
}

bool Supervisor::startPressedForArm() const {
  if (!_ps2.getButtonPressed(PSXBTN_START)) return false;
  if (_ps2.getButton(PSXBTN_SELECT) || _ps2.getButton(PSXBTN_TRIANGLE)) return false;
  return true;
}

void Supervisor::taskTrampoline(void* arg) {
  static_cast<Supervisor*>(arg)->taskLoop();
}

void Supervisor::taskLoop() {
  const TickType_t period = pdMS_TO_TICKS(1000 / SUPERVISOR_HZ);
  TickType_t last = xTaskGetTickCount();
  SupervisorState last_state = SupervisorState::DISARMED;

  for (;;) {
    bool ps2_healthy = _ps2.update();

    if (_state != last_state) {
      Serial.printf("🔄 Supervisor: %s -> %s\n",
                    last_state == SupervisorState::DISARMED ? "DISARMED" :
                    last_state == SupervisorState::ARMED ? "ARMED" : "ESTOP",
                    stateName());
      last_state = _state;
      setArmedLed(isArmed());
    }

    // Always zero command unless armed
    _cmd = {};

    switch (_state) {
      case SupervisorState::DISARMED:
        if (_ps2.isEmergencyStop()) {
          _state = SupervisorState::ESTOP;
          _ps2.setRumble(false);
          break;
        }

        if (ps2_healthy && startPressedForArm()) {
          _state = SupervisorState::ARMED;
          _ps2.setRumble(false);
          Serial.println("🟢 ARMED");
          break;
        }

        // Buzz controller if user tries to drive while disarmed
        if (ps2_healthy && _ps2.hasMotorInput()) {
          _ps2.setRumble(true);
        } else {
          _ps2.setRumble(false);
        }
        break;

      case SupervisorState::ARMED:
        if (_ps2.isEmergencyStop()) {
          _state = SupervisorState::ESTOP;
          _ps2.setRumble(false);
          break;
        }

        if (!ps2_healthy) {
          _state = SupervisorState::DISARMED;
          _ps2.setRumble(false);
          Serial.println("🔴 DISARMED — controller lost");
          break;
        }

        if (startPressedForArm()) {
          _state = SupervisorState::DISARMED;
          _ps2.setRumble(false);
          Serial.println("🔴 DISARMED");
          break;
        }

        _ps2.setRumble(false);
        _cmd.vx = _ps2.getVx() * _ps2.getSpeedScale();
        _cmd.vy = _ps2.getVy() * _ps2.getSpeedScale();
        _cmd.wz = _ps2.getWz() * _ps2.getSpeedScale();
        break;

      case SupervisorState::ESTOP:
        _ps2.setRumble(false);
        if (!_ps2.isEmergencyStop() || !ps2_healthy) {
          _state = SupervisorState::DISARMED;
          Serial.println("🔴 DISARMED — press START to arm");
        }
        break;
    }

    vTaskDelayUntil(&last, period);
  }
}
