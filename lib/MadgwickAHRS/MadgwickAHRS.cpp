#include "MadgwickAHRS.h"
#include <math.h>

MadgwickAHRS::MadgwickAHRS(float sampleFreq, float beta)
    : _sampleFreq(sampleFreq), _beta(beta), _q0(1), _q1(0), _q2(0), _q3(0) {}

void MadgwickAHRS::update(float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz) {
    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    float q0 = _q0, q1 = _q1, q2 = _q2, q3 = _q3;
    float recipNorm;
    float s0, s1, s2, s3;
    float hx, hy, _2bx, _2bz;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q0q2 = 2.0f * q0 * q2;
    float _2q2q3 = 2.0f * q2 * q3;

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

    // Reference direction of Earth's magnetic field
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    hx = mx * q0 * q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1 * q1 + _2q1mx * q2 + mx * q2 * q2 + mx * q3 * q3;
    hy = _2q0mx * q3 + my * q0 * q0 - _2q0mz * q1 + _2q1mx * q3 + my * q1 * q1 + my * q2 * q2 + my * q3 * q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0 * q0 + _2q1mx * q3 + mz * q1 * q1 + mz * q2 * q2 + mz * q3 * q3;

    // Gradient descent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1 * q3 - _2q0 * q2 - ax) + _2q1 * (2.0f * q0 * q1 + _2q2 * q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + _2bx * q2 * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
    s1 = _2q3 * (2.0f * q1 * q3 - _2q0 * q2 - ax) + _2q0 * (2.0f * q0 * q1 + _2q2 * q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1 * q1 - 2.0f * q2 * q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + (_2bx * q3 - 4.0f * q1) * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
    s2 = -_2q0 * (2.0f * q1 * q3 - _2q0 * q2 - ax) + _2q3 * (2.0f * q0 * q1 + _2q2 * q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1 * q1 - 2.0f * q2 * q2 - az) + (-_2bx * q2 - _2bz * q0) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + (_2bx * q1 + _2bz * q3) * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
    s3 = _2q1 * (2.0f * q1 * q3 - _2q0 * q2 - ax) + _2q2 * (2.0f * q0 * q1 + _2q2 * q3 - ay) + (-_2bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2 * q2 - q3 * q3) + _2bz * (q1 * q3 - q0 * q2) - mx) + (-_2bx * q1 + _2bz * q0) * (_2bx * (q1 * q2 - q0 * q3) + _2bz * (q0 * q1 + q2 * q3) - my) + _2bx * q0 * (_2bx * (q0 * q2 + q1 * q3) + _2bz * (0.5f - q1 * q1 - q2 * q2) - mz);
    recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Integrate rate of change of quaternion
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * (0.5f / _sampleFreq) - _beta * s0 / _sampleFreq;
    q1 += (q0 * gx + q2 * gz - q3 * gy) * (0.5f / _sampleFreq) - _beta * s1 / _sampleFreq;
    q2 += (q0 * gy - q1 * gz + q3 * gx) * (0.5f / _sampleFreq) - _beta * s2 / _sampleFreq;
    q3 += (q0 * gz + q1 * gy - q2 * gx) * (0.5f / _sampleFreq) - _beta * s3 / _sampleFreq;

    // Normalise quaternion
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    _q0 = q0 * recipNorm;
    _q1 = q1 * recipNorm;
    _q2 = q2 * recipNorm;
    _q3 = q3 * recipNorm;
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
