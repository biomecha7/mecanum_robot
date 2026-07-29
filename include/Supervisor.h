#pragma once
#include "PS2Controller.h"
#include "RobotPins.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

struct BodyCmd {
  float vx;
  float vy;
  float wz;
};

enum class SupervisorState {
  DISARMED,
  ARMED,
  ESTOP
};

class Supervisor {
public:
  explicit Supervisor(PS2Controller& ps2);

  bool initialize();
  bool start();
  void stop();

  BodyCmd latest() const;
  const char* stateName() const;
  bool isArmed() const { return _state == SupervisorState::ARMED; }

private:
  static void taskTrampoline(void* arg);
  void taskLoop();
  void setArmedLed(bool on);
  bool startPressedForArm() const;

  PS2Controller& _ps2;
  TaskHandle_t _task{nullptr};
  volatile SupervisorState _state{SupervisorState::DISARMED};
  BodyCmd _cmd{};

  static const int SUPERVISOR_HZ = 50;
};
