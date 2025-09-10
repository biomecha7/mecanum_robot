#pragma once
#include <Arduino.h>


class MotorDriver {
public:
    MotorDriver(int chR, int chL, int pwmPinR, int pwmPinL, int freq = 16000, int res = 10);
    void drive(int duty, int dir);
    void stop();
    void setMaxDuty(int maxDuty);
    /**
     * @brief Set the motor speed in range [-1, 1]. Positive is forward, negative is reverse.
     * @param speed Motor speed, normalized [-1, 1]
     */
    void setSpeed(float speed); // alias for set, for compatibility
    private:
        int _chR, _chL;
        int _pwmPinR, _pwmPinL;
        int _freq, _res, _maxDuty;
    /**
     * @brief Internal function to set motor speed and direction.
     * @param speed Motor speed, normalized [-1, 1]
     */
    void set(float speed); // speed in range [-1, 1]
};
