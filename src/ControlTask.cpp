#include "ControlTask.h"
#include "OledDisplay.h"
#include <Arduino.h>
#include <math.h>

ControlTask::ControlTask(MotionController& mc, Supervisor& supervisor)
  : _mc(mc), _supervisor(supervisor) {}

bool ControlTask::initialize() {
  return true;
}

bool ControlTask::start() {
  if (_task != nullptr) return false;

  return xTaskCreatePinnedToCore(
           taskTrampoline, "ControlTask", 4096, this, 4, &_task, 1) == pdPASS;
}

void ControlTask::stop() {
  if (_task != nullptr) {
    vTaskDelete(_task);
    _task = nullptr;
  }
}

void ControlTask::taskTrampoline(void* arg) {
  static_cast<ControlTask*>(arg)->taskLoop();
}

void ControlTask::taskLoop() {
  const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_HZ);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    BodyCmd sp = _supervisor.latest();
    _mc.drive(sp.vx, sp.vy, sp.wz);

    const bool motorsActive =
        fabsf(sp.vx) > 0.02f || fabsf(sp.vy) > 0.02f || fabsf(sp.wz) > 0.02f;

    Oled::setStatus(_supervisor.stateName());
    Oled::setMotorsActive(motorsActive);
    Oled::service();

    vTaskDelayUntil(&last, period);
  }
}
