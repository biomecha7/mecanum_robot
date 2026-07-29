#include "MotorDriver.h"
#include <math.h>

MotorDriver::MotorDriver(int chR, int chL, int pinR, int pinL, int freq, int res)
  : _chR(chR), _chL(chL) {
  _maxDuty = (1 << res) - 1;
  ledcSetup(_chR, freq, res);
  ledcAttachPin(pinR, _chR);
  ledcSetup(_chL, freq, res);
  ledcAttachPin(pinL, _chL);
  stop();
}

void MotorDriver::stop() {
  ledcWrite(_chR, 0);
  ledcWrite(_chL, 0);
  _last = 0.0f;
}

void MotorDriver::setSpeed(float speed) {
  if (speed > 1.0f) speed = 1.0f;
  if (speed < -1.0f) speed = -1.0f;

  const float maxStep = 0.10f;
  float d = speed - _last;
  if (d >  maxStep) speed = _last + maxStep;
  if (d < -maxStep) speed = _last - maxStep;
  _last = speed;

  int duty = (int)(_maxDuty * fabsf(speed));
  if (speed > 0.0f) {
    ledcWrite(_chR, duty);
    ledcWrite(_chL, 0);
  } else if (speed < 0.0f) {
    ledcWrite(_chR, 0);
    ledcWrite(_chL, duty);
  } else {
    ledcWrite(_chR, 0);
    ledcWrite(_chL, 0);
  }
}
