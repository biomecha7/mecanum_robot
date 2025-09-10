#include "MadgwickAHRS.h"
#include <math.h>

MadgwickAHRS::MadgwickAHRS(float sampleFreq, float beta)
    : _sampleFreq(sampleFreq), _beta(beta), _q0(1), _q1(0), _q2(0), _q3(0) {}

void MadgwickAHRS::update(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz) {
    // Madgwick algorithm implementation (see https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
    // This is a simplified version for clarity. For production, use the full reference code.
    float q0 = _q0, q1 = _q1, q2 = _q2, q3 = _q3;
    float recipNorm;
    float s0, s1, s2, s3;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q0q2 = 2.0f * q0 * q2;
    float _2q2q3 = 2.0f * q2 * q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // Normalise accelerometer measurement
    recipNorm = sqrt(ax * ax + ay * ay + az * az);
    if (recipNorm == 0.0f) return;
    recipNorm = 1.0f / recipNorm;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = sqrt(mx * mx + my * my + mz * mz);
    if (recipNorm == 0.0f) return;
    recipNorm = 1.0f / recipNorm;
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Algorithm steps omitted for brevity (see reference for full implementation)
    // ...
    // For now, just keep the quaternion unchanged
    _q0 = q0;
    _q1 = q1;
    _q2 = q2;
    _q3 = q3;
}

void MadgwickAHRS::getQuaternion(float& q0, float& q1, float& q2, float& q3) const {
    q0 = _q0;
    q1 = _q1;
    q2 = _q2;
    q3 = _q3;
}

void MadgwickAHRS::getEuler(float& roll, float& pitch, float& yaw) const {
    // Convert quaternion to Euler angles
    roll = atan2(2.0f * (_q0 * _q1 + _q2 * _q3), 1.0f - 2.0f * (_q1 * _q1 + _q2 * _q2));
    pitch = asin(2.0f * (_q0 * _q2 - _q3 * _q1));
    yaw = atan2(2.0f * (_q0 * _q3 + _q1 * _q2), 1.0f - 2.0f * (_q2 * _q2 + _q3 * _q3));
    // Convert radians to degrees
    roll *= 57.2958f;
    pitch *= 57.2958f;
    yaw *= 57.2958f;
}
