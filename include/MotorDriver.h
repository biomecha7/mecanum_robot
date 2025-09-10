#pragma once
#include <Arduino.h>


class MotorDriver {
public:
    MotorDriver(int chR, int chL, int pwmPinR, int pwmPinL, int freq = 16000, int res = 10);
    void drive(int duty, int dir);
    void stop();
    void setMaxDuty(int maxDuty);
private:
    int _chR, _chL;
    int _pwmPinR, _pwmPinL;
    int _freq, _res, _maxDuty;
};
