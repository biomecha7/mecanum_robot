#pragma once
#include <Arduino.h>

class MadgwickAHRS {
public:
    MadgwickAHRS(float sampleFreq = 100.0f, float beta = 0.1f);
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz);
    void getQuaternion(float& q0, float& q1, float& q2, float& q3) const;
    void getEuler(float& roll, float& pitch, float& yaw) const;
private:
    float _sampleFreq;
    float _beta;
    float _q0, _q1, _q2, _q3;
};
