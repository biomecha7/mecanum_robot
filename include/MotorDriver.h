#pragma once
#include <Arduino.h>

class MotorDriver {
public:
  MotorDriver(int chR, int chL, int pinR, int pinL, int freq = 16000, int res = 10);
  void setSpeed(float speed);  // [-1, 1]
  void stop();

private:
  int _chR, _chL, _maxDuty;
  float _last = 0.0f;
};
