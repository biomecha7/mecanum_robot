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

void MotorDriver::setMaxDuty(int maxDuty) {
    _maxDuty = maxDuty;
}
