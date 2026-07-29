#pragma once
#include "MotionController.h"
#include "Supervisor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class ControlTask {
public:
  ControlTask(MotionController& mc, Supervisor& supervisor);
  bool initialize();
  bool start();
  void stop();

private:
  static void taskTrampoline(void* arg);
  void taskLoop();

  MotionController& _mc;
  Supervisor& _supervisor;
  TaskHandle_t _task{nullptr};

  static const int CONTROL_HZ = 100;
};
