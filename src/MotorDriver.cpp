#include "MotorDriver.h"


MotorDriver::MotorDriver(int chR, int chL, int pwmPinR, int pwmPinL, int freq, int res)
    : _chR(chR), _chL(chL), _pwmPinR(pwmPinR), _pwmPinL(pwmPinL), _freq(freq), _res(res) {
    _maxDuty = (1 << _res) - 1;
    ledcSetup(_chR, _freq, _res);
    ledcAttachPin(_pwmPinR, _chR);
    ledcSetup(_chL, _freq, _res);
    ledcAttachPin(_pwmPinL, _chL);
}

void MotorDriver::drive(int duty, int dir) {
    duty = constrain(duty, 0, _maxDuty);
    if (dir > 0) {
        ledcWrite(_chR, duty);
        ledcWrite(_chL, 0);
    } else if (dir < 0) {
        ledcWrite(_chR, 0);
        ledcWrite(_chL, duty);
    } else {
        ledcWrite(_chR, 0);
        ledcWrite(_chL, 0);
    }
}

void MotorDriver::stop() {
    ledcWrite(_chR, 0);
    ledcWrite(_chL, 0);
}

/**
 * @brief Set the motor speed in range [-1, 1]. Positive is forward, negative is reverse.
 * @param speed Motor speed, normalized [-1, 1]
 */
void MotorDriver::setSpeed(float speed) {
    set(speed);
}

/**
 * @brief Internal function to set motor speed and direction.
 * @param speed Motor speed, normalized [-1, 1]
 */
void MotorDriver::set(float speed) {
    // Clamp to [-1, 1]
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    // Per-motor slew limit
    const float maxStep = 0.10f;  // ≈5% per call at ~100 Hz
    float delta = speed - _lastSpeed;
    if (delta >  maxStep) speed = _lastSpeed + maxStep;
    if (delta < -maxStep) speed = _lastSpeed - maxStep;
    _lastSpeed = speed;

    int duty = (int)(_maxDuty * fabs(speed));
    int dir  = (speed >= 0) ? 1 : -1;
    drive(duty, dir);
}


void MotorDriver::setMaxDuty(int maxDuty) {
    _maxDuty = maxDuty;
}
