#pragma once
#include "MotionController.h"
#include "PS2Controller.h"
#include "IMUTask.h"
#include "ISetpointProvider.h"
#include "RobotPins.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class ControlTask {
public:
  ControlTask(MotionController& mc, PS2Controller& ps2, IMUTask& imu, ISetpointProvider& provider);
  bool initialize();   // any LEDs, preflight checks
  bool start();        // xTaskCreatePinnedToCore(...) + trampoline
  void stop();

private:
  static void taskTrampoline(void* arg);
  void taskLoop();     // moved from main.cpp (same timing & behavior)

  MotionController& _mc;
  PS2Controller&    _ps2;
  IMUTask&          _imu;
  ISetpointProvider& _provider;
  TaskHandle_t      _task{nullptr};
  
  // Control state (moved from globals)
  bool _closedLoopEnabled = false;
  bool _debugMode = false;
  uint32_t _lastDebugPrint = 0;
  uint32_t _lastSensorUpdate = 0;
  
  // LED pins for status indication (using definitions from RobotPins.h)
  // LED_PIN_YLW is now available from RobotPins.h include
  
  // Control loop frequency
  static const int CONTROL_HZ = 100;
};